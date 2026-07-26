#include <Arduino.h>
#include <SimpleFOC.h>

StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(PC7, PC6, PC9, PC8, PB15, PA9);

InlineCurrentSense current_sense  = InlineCurrentSense(0.01f, 50.0f, PB11, PB12);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimitCurrent(char* cmd) { command.scalar(&motor.current_limit, cmd); }

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable();

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 4;
  driver.init();
  current_sense.linkDriver(&driver);
  // link the motor and the driver
  motor.linkDriver(&driver);
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;
  // torque control mode 
  motor.torque_controller = TorqueControlType::foc_current;

  // setting target velocity
  motor.target = 3.14*6;  // [rad/s]
  // limiting motor current (provided resistance)
  motor.updateCurrentLimit(1.5);   // [Amps]
 
  // init motor hardware
  motor.phase_resistance = 2.3;
  motor.axis_inductance.d = 3.3/1000;
  motor.axis_inductance.q = 3.1/1000;
  motor.init();
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('C', doLimitCurrent, "current limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");

  int res = motor.tuneCurrentController(300.0);

  if (res != 0) {
    Serial.printf("res: %d\n", res);
    // 1: bandwidth <= 0
    // 2: bandwidth too high for loop frequency
    // 3: motor characterisation failed
  }

  HardwareTimer* timer = new HardwareTimer(TIM5);
  timer->setOverflow(10000, HERTZ_FORMAT); 
  timer->attachInterrupt([](){
    motor.loopFOC();
    motor.move();
  });
  timer->resume();

  _delay(1000);
}

void loop() {
  // user communication
  command.run();

}