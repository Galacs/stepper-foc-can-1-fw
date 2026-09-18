#include <Arduino.h>
#include <SimpleFOC.h>
#include <encoders/calibrated/CalibratedSensor.h>

#include "config.h"
#include "pins.h"

StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(A_IN1_PIN, A_IN2_PIN, B_IN1_PIN, B_IN2_PIN, A_SLEEP_PIN, B_SLEEP_PIN);

MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, PA15);
foc_node_cfg node_cfg;
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, 200, node_cfg.calibrationLut);
SPIClass SPI_3(PC12, PC11, PC10);

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable();
  motor.useMonitoring(Serial);

  get_config(&node_cfg);

  sensor.min_elapsed_time = 0.0003;
  sensor.init(&SPI_3);

  driver.voltage_power_supply = 24;
  driver.voltage_limit = 10;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::estimated_current;

  motor.target = 5;

  motor.phase_resistance = 2.3;
  motor.axis_inductance.d = 3.3/1000;
  motor.axis_inductance.q = 3.1/1000;
  motor.KV_rating = 38;

  motor.LPF_velocity = 0.08;
  motor.PID_velocity.P = 2.2;
  motor.PID_velocity.I = 90;
  motor.PID_velocity.D = 0.;

  motor.updateVelocityLimit(30);
  motor.updateCurrentLimit(1.5);

  motor.sensor_direction = node_cfg.sensor_direction;
  motor.zero_electric_angle = node_cfg.zero_electric_angle;

  motor.init();
  // sensor_calibrated.calibrate(motor);
  motor.linkSensor(&sensor_calibrated);

  motor.initFOC();

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");

  HardwareTimer* timer = new HardwareTimer(TIM5);
  timer->setOverflow(11000, HERTZ_FORMAT); 
  timer->attachInterrupt([](){
    motor.loopFOC();
    motor.move();
  });
  timer->resume();

  command.add('M', doMotor, "Motor");
  motor.monitor_start_char = 'M';
  motor.monitor_end_char = 'M';
  command.verbose = VerboseMode::machine_readable;
  motor.monitor_downsample = 1500;
  _delay(1000);
}

unsigned long last_print = 0;
void loop() {
  command.run();
  motor.monitor();
}