#include <Arduino.h>
#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>
#include <ACANFD_STM32.h>

#include "config.h"
#include "pins.h"
#include "can_protocol.h"

#include <ACANFD_STM32.h>
#include <ACANFD_STM32_CANMessage.h>

#define FOC_LOOP_HZ            5000u
#define TX_RATE_HZ               50u
#define CAN_BITRATE          (500u * 1000u)
#define CAN_COMMAND_TIMEOUT_MS  250u
#define ENABLE_COMMAND_TIMEOUT    0    // 1 pour stop quand timeout
#define CAN_FRAME_TYPE CANFDMessage::CANFD_NO_BIT_RATE_SWITCH  // or CAN_DATA for vcan

StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(A_IN1_PIN, A_IN2_PIN, B_IN1_PIN, B_IN2_PIN, A_SLEEP_PIN, B_SLEEP_PIN);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, SPI_CS_PIN);
foc_node_cfg node_cfg;
CalibratedSensor* sensor_calibrated = nullptr;
SPIClass SPI_3(SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN);

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

static volatile uint8_t g_mode = FOC_MODE_IDLE;
static volatile float   g_target = 0.0f;
static volatile bool    g_enabled = false;

static float g_zero_offset = 0.0f;

static volatile uint32_t g_loop_us_max = 0;
static uint32_t g_last_command_ms = 0;

HardwareTimer* focTimer = nullptr;

static void focLoopISR() {
  const uint32_t t0 = micros();
  motor.target = g_enabled ? g_target : 0.0f;
  motor.loopFOC();
  motor.move();
  const uint32_t dt = micros() - t0;
  if (dt > g_loop_us_max) g_loop_us_max = dt;
}

static bool sendFrame(uint8_t cmd, const void* payload, uint8_t len) {
  CANFDMessage msg;
  msg.ext  = false;
  msg.type = CAN_FRAME_TYPE;
  msg.id   = ((uint32_t)node_cfg.motor_id << 5) | (cmd & 0x1F);
  msg.len  = len;
  memcpy(msg.data, payload, len);
  return fdcan2.tryToSendReturnStatusFD(msg) == 0;
}

static void applySetTarget(const foc_set_target_t& t) {
  if (t.flags & FOC_FLAG_SET_ZERO) g_zero_offset = motor.shaft_angle;

  if (t.flags & FOC_FLAG_CLEAR_FAULTS) {
    g_enabled = false;
    g_mode = FOC_MODE_IDLE;
    motor.disable();
  }

  if (t.mode == FOC_MODE_VELOCITY || t.mode == FOC_MODE_POSITION) {
    if (t.mode != g_mode) {
      motor.controller = (t.mode == FOC_MODE_POSITION) ? MotionControlType::angle
                                                       : MotionControlType::velocity;
      g_mode = t.mode;
    }
    if (t.limit > 0) {
      if (t.mode == FOC_MODE_POSITION) motor.updateVelocityLimit(t.limit * 0.01f);
      else                             motor.updateCurrentLimit(t.limit * 0.01f);
    }
    g_target = (t.mode == FOC_MODE_POSITION) ? (t.target + g_zero_offset) : t.target;

    if (t.flags & FOC_FLAG_ENABLE) { motor.enable();  g_enabled = true; }
    else                           { motor.disable(); g_enabled = false; }
  } else {
    g_mode = FOC_MODE_IDLE;
    g_target = 0.0f;
    g_enabled = false;
    motor.disable();
  }
  g_last_command_ms = millis();
}

static void handleCan() {
  CANFDMessage msg;
  while (fdcan2.receiveFD0(msg)) {
    if (msg.ext) continue;
    if ((uint8_t)((msg.id >> 5) & 0x3F) != node_cfg.motor_id) continue;
    if ((uint8_t)(msg.id & 0x1F) == FOC_CMD_SET_TARGET &&
        msg.len >= sizeof(foc_set_target_t)) {
      foc_set_target_t t;
      memcpy(&t, msg.data, sizeof(t));
      applySetTarget(t);
    }
  }

#if ENABLE_COMMAND_TIMEOUT
  if (g_enabled && (millis() - g_last_command_ms) > CAN_COMMAND_TIMEOUT_MS) {
    g_enabled = false;
    g_mode = FOC_MODE_IDLE;
    motor.disable();
  }
#endif
}

static void sendFeedback() {
  static uint32_t next_ms = 0;
  const uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return;
  next_ms = now + (1000u / TX_RATE_HZ);

  foc_state_t st;
  st.pos = motor.shaft_angle - g_zero_offset;
  st.vel = motor.shaft_velocity;

  foc_status_t status;
  status.mode = g_mode;
  status.flags = (g_enabled ? FOC_STATUS_ENABLED : 0)
               | (digitalRead(A_FAULT_PIN) ? 0 : FOC_STATUS_FAULT_A)
               | (digitalRead(B_FAULT_PIN) ? 0 : FOC_STATUS_FAULT_B);
  status.loop_us = min<uint32_t>((uint32_t)g_loop_us_max, 65535u);
  status.voltage_q = motor.voltage.q;
  g_loop_us_max = 0;

  sendFrame(FOC_CMD_STATE, &st, sizeof(st));
  sendFrame(FOC_CMD_STATUS, &status, sizeof(status));
}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable();
  motor.useMonitoring(Serial);

  sensor.min_elapsed_time = 0.0003;
  sensor.init(&SPI_3);

  driver.voltage_power_supply = 24;
  driver.voltage_limit = 10;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.target = 0;

  motor.phase_resistance = 2.3;
  motor.axis_inductance.d = 3.3 / 1000;
  motor.axis_inductance.q = 3.1 / 1000;
  motor.KV_rating = 38;

  motor.LPF_velocity = 0.08;
  motor.PID_velocity.P = 2.2;
  motor.PID_velocity.I = 90;
  motor.PID_velocity.D = 0.;

  motor.updateVelocityLimit(30);
  motor.updateCurrentLimit(1.5);

  motor.sensor_direction   = node_cfg.sensor_direction;
  motor.zero_electric_angle = node_cfg.zero_electric_angle;

  get_config(&node_cfg);
  Serial.print("node id: ");
  Serial.println(node_cfg.motor_id);
  sensor_calibrated = new CalibratedSensor(sensor, CAL_LUT_LEN, node_cfg.calibrationLut);
  delay(1000);
  sensor_calibrated->calibrate(motor, 60);
  motor.linkSensor(sensor_calibrated);

  motor.init();
  motor.initFOC();

  pinMode(A_FAULT_PIN, INPUT_PULLUP);
  pinMode(B_FAULT_PIN, INPUT_PULLUP);

  ACANFD_STM32_Settings settings(CAN_BITRATE, DataBitRateFactor::x1);
  const uint32_t err = fdcan2.beginFD(settings);   // PB6 / PB5
  if (err != 0) { Serial.print("FDCAN init failed: 0x"); Serial.println(err, HEX); }

  focTimer = new HardwareTimer(TIM5);
  focTimer->setOverflow(FOC_LOOP_HZ, HERTZ_FORMAT);
  focTimer->attachInterrupt(focLoopISR);
  focTimer->resume();

  command.add('M', doMotor, "Motor");
  motor.monitor_start_char = 'M';
  motor.monitor_end_char = 'M';
  command.verbose = VerboseMode::machine_readable;
  motor.monitor_downsample = 1500;

  Serial.print("Motor ready. Node id ");
  Serial.println(node_cfg.motor_id);
}

static uint32_t next_dbg = 0;
void loop() {
  command.run();
  handleCan();
  sendFeedback();
  if ((int32_t)(millis() - next_dbg) >= 0) {
    next_dbg += 500;
    // Serial.print(motor.shaft_angle, 4); Serial.print('\t');
    // Serial.println(motor.shaft_velocity, 4);
  }
}