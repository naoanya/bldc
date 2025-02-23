/** 
 * B-G431B-ESC1 position motion control example with encoder
 *
 */
#include <Arduino.h>
#include "SimpleFOC.h"

/*********************************************************************/

#include "config.hpp"

//#define ENABLE_FIND_POLE_PAIR   (1)
//#define ENABLE_SENSOR_TEST_1		(1)
//#define ENABLE_SENSOR_TEST_2		(1)
//#define ENABLE_SENSOR_TEST_3		(1)

/*********************************************************************/

#define ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM   (0)
#define ROT_SENSE_TYPE_MAGNETIC_SENSE_I2C   (1)
#define ROT_SENSE_TYPE_ENCODER              (2)

/*********************************************************************/

void init_sensor(void);

void find_pole_pair(void);
void sensor_test1(void);
void sensor_test2(void);
void sensor_test3(void);

void demo_move(void);

/*********************************************************************/

BLDCMotor motor = BLDCMotor(
  POLE_PAIR
  #if !ENABLE_FIND_POLE_PAIR
  , PHASE_RESISTANCE
  #endif
);

BLDCDriver6PWM driver = BLDCDriver6PWM(
  A_PHASE_UH, A_PHASE_UL,
  A_PHASE_VH, A_PHASE_VL,
  A_PHASE_WH, A_PHASE_WL);

LowsideCurrentSense currentSense = LowsideCurrentSense(
  0.003,
  -64.0/7.0,
  A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

#if ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM

  MagneticSensorPWM rotSense = MagneticSensorPWM(
		  A_PWM, MAGSENSE_PWM_MIN_CLOCK, MAGSENSE_PWM_MAX_CLOCK);
  void doPWM(){rotSense.handlePWM();}

#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_I2C

  MagneticSensorI2C rotSense = MagneticSensorI2C(AS5600_I2C);

#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_ENCODER

  Encoder rotSense = Encoder(A_HALL1, A_HALL2, 2048, A_HALL3);
  void doA(){rotSense.handleA();}
  void doB(){rotSense.handleB();}
  void doZ(){rotSense.handleIndex();}

#else
  #error "No ROT_SENSE_TYPE defined."
#endif

/*********************************************************************/

// Encoder monitor inverval
// < 0 : disable
// 0 < : monitor interval ms
int encoderMonitorIntervalMs_ = -1;

int preEncoderMonitorTimeMs_ = 0;

int enableDemo_ = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.motion(&motor, cmd); }
void doCommand(char* cmd) { command.motor(&motor, cmd); }
void doEncoder(char* cmd) { encoderMonitorIntervalMs_ = atoi(cmd); }
void doDemo(char* cmd) { enableDemo_ = atoi(cmd); }
void doReset(char* cmd) { HAL_NVIC_SystemReset(); }

/*********************************************************************/

void init_sensor(void)
{
#if ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM
  rotSense.init();
  rotSense.enableInterrupt(doPWM);
#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_I2C
  rotSense.init(&Wire);
  Wire.setClock(400000);
#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_ENCODER
  rotSense.init();
  rotSense.enableInterrupts(doA, doB, doZ);
#else
  #error "No ROT_SENSE_TYPE defined."
#endif
}

void setup()
{
#if ENABLE_SENSOR_TEST_1
  sensor_test1();
  while (1) {;}
#endif

#if ENABLE_SENSOR_TEST_2
  sensor_test2();
  while (1) {;}
#endif

#if ENABLE_SENSOR_TEST_3
  sensor_test3();
  while (1) {;}
#endif

#if ENABLE_FIND_POLE_PAIR
  find_pole_pair();
  while (1) {;}
#endif

  init_sensor();
  motor.linkSensor(&rotSense);

  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.init();

  motor.linkDriver(&driver);
  currentSense.linkDriver(&driver);
  
  currentSense.init();
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // aligning voltage [V]
  motor.voltage_sensor_align = 1;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  // MotionControlType::torque
  // MotionControlType::velocity
  // MotionControlType::angle
  // MotionControlType::velocity_openloop
  // MotionControlType::angle_openloop
  // TorqueControlType::voltage
  // TorqueControlType::dc_current
  // TorqueControlType::foc_current
  // control loop type and torque mode
  #ifdef TORQUE_CTRL
  motor.torque_controller = TORQUE_CTRL;
  #endif
  #ifdef TORQUE_CTRL
  motor.controller = MOTION_CTRL;
  #endif
  motor.motion_downsample = DEF_MOTION_DOWNSMAPLE;

  // velocity loop PID
  #ifdef VELOCITY_PID_P
  motor.PID_velocity.P = VELOCITY_PID_P;
  #endif
  #ifdef VELOCITY_PID_I
  motor.PID_velocity.I = VELOCITY_PID_I;
  #endif
  #ifdef VELOCITY_PID_D
  motor.PID_velocity.D = VELOCITY_PID_D;
  #endif
  #ifdef VELOCITY_OUTPUT_RAMP
  motor.PID_velocity.output_ramp = VELOCITY_OUTPUT_RAMP;
  #endif
  #ifdef VELOCITY_LIMIT
  motor.PID_velocity.limit = VELOCITY_LIMIT;
  #endif
  #ifdef VELOCITY_FILTER
  motor.LPF_velocity.Tf = VELOCITY_FILTER;
  #endif
  // angle loop PID
  #ifdef ANGLE_PID_P
  motor.P_angle.P = ANGLE_PID_P;
  #endif
  #ifdef ANGLE_PID_I
  motor.P_angle.I = ANGLE_PID_I;
  #endif
  #ifdef ANGLE_PID_D
  motor.P_angle.D = ANGLE_PID_D;
  #endif
  #ifdef ANGLE_OUTPUT_RAMP
  motor.P_angle.output_ramp = ANGLE_OUTPUT_RAMP;
  #endif
  #ifdef ANGLE_LIMIT
  motor.P_angle.limit = ANGLE_LIMIT;
  #endif
  #ifdef ANGLE_FILTER
  motor.LPF_angle.Tf = ANGLE_FILTER;
  #endif
  // current q loop PID 
  #ifdef Q_CUR_PID_P
  motor.PID_current_q.P = Q_CUR_PID_P;
  #endif
  #ifdef Q_CUR_PID_I
  motor.PID_current_q.I = Q_CUR_PID_I;
  #endif
  #ifdef Q_CUR_PID_D
  motor.PID_current_q.D = Q_CUR_PID_D;
  #endif
  #ifdef Q_CUR_OUTPUT_RAMP
  motor.PID_current_q.output_ramp = Q_CUR_OUTPUT_RAMP;
  #endif
  #ifdef Q_CUR_LIMIT
  motor.PID_current_q.limit = Q_CUR_LIMIT;
  #endif
  #ifdef Q_CUR_FILTER
  motor.LPF_current_q.Tf = Q_CUR_FILTER;
  #endif
  // current d loop PID
  #ifdef D_CUR_PID_P
  motor.PID_current_d.P = D_CUR_PID_P;
  #endif
  #ifdef D_CUR_PID_I
  motor.PID_current_d.I = D_CUR_PID_I;
  #endif
  #ifdef D_CUR_PID_D
  motor.PID_current_d.D = D_CUR_PID_D;
  #endif
  #ifdef D_CUR_OUTPUT_RAMP
  motor.PID_current_d.output_ramp = D_CUR_OUTPUT_RAMP;
  #endif
  #ifdef D_CUR_LIMIT
  motor.PID_current_d.limit = D_CUR_LIMIT;
  #endif
  #ifdef D_CUR_FILTER
  motor.LPF_current_d.Tf = D_CUR_FILTER;
  #endif
  // Limits 
  #ifdef CUR_LIMIT
  motor.current_limit = CUR_LIMIT;
  #endif
  #ifdef VOL_LIMIT
  motor.voltage_limit = VOL_LIMIT;
  #endif
  #ifdef VEL_LIMIT
  motor.velocity_limit = VEL_LIMIT;
  #endif

  // general settings
  // motor phase resistance
  motor.phase_resistance = PHASE_RESISTANCE;
  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;

  //SimpleFOCDebug::enable();
  //SimpleFOCDebug::enable(&Serial);
  command.verbose = VerboseMode::user_friendly;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, (char*)"target angle");
  command.add('M', doCommand, (char*)"motor command");
  command.add('E', doEncoder, (char*)"encoder monitor");
  command.add('D', doDemo, (char*)"demo");
  command.add('R', doReset, (char*)"system reset");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);

  preEncoderMonitorTimeMs_ = millis();
}

void loop()
{
  if (enableDemo_) {
    // Demo move
    demo_move();
  } else {
    // iterative setting FOC phase voltage
    motor.loopFOC();

    // Motion control function
    motor.move();
  }

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();

  // user communication
  command.run();

  // Encoder monitor
  if (encoderMonitorIntervalMs_ >= 0) {
    int curEncoderMonitorTimeMs = millis();
    if (curEncoderMonitorTimeMs - preEncoderMonitorTimeMs_ >= encoderMonitorIntervalMs_) {
      preEncoderMonitorTimeMs_ = curEncoderMonitorTimeMs;
      //Serial.printf("E %f %f\n", rotSense.getAngle(), rotSense.getVelocity());
      Serial.print("E ");
      Serial.print(rotSense.getAngle());
      Serial.print(" ");
      Serial.print(rotSense.getVelocity());
      #if 0
      Serial.print(" ");
      Serial.print(100);
      Serial.print(" ");
      Serial.print(-100);
      #endif
      Serial.println();
    }
  }
}

/*********************************************************************/

void find_pole_pair(void)
{
  init_sensor();
  motor.linkSensor(&rotSense);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  //currentSense.init();
  //currentSense.skip_align = true;
  //motor.linkCurrentSense(&currentSense);

  motor.init();

  Serial.begin(115200);

  // pole pairs calculation routine
  Serial.println("Pole pairs (PP) estimator");
  Serial.println("-\n");
  _delay(1000);
  Serial.println("-5\n");
  _delay(1000);
  Serial.println("-4\n");
  _delay(1000);
  Serial.println("-3\n");
  _delay(1000);
  Serial.println("-2\n");
  _delay(1000);
  Serial.println("-1\n");
  _delay(1000);

  float pp_search_voltage = 4; // maximum power_supply_voltage/2
  float pp_search_angle = 1*M_PI; // search electrical angle to turn

  // move motor to the electrical angle 0
  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit=pp_search_voltage;
  motor.move(0);
  _delay(1000);
  // read the encoder angle
  rotSense.update(); 
  float angle_begin = rotSense.getAngle();
  _delay(50);

  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while(motor_angle <= pp_search_angle){
    motor_angle += 0.001f;
    motor.move(motor_angle);
  }
  _delay(1000);
  // read the encoder value for 180
  rotSense.update(); 
  float angle_end = rotSense.getAngle();
  _delay(50);
  // turn off the motor
  motor.move(0);
  _delay(1000);

  // calculate the pole pair number
  int pp = round((pp_search_angle)/(angle_end-angle_begin));

  Serial.print(F("Estimated PP : "));
  Serial.println(pp);
  Serial.println(F("PP = Electrical angle / Encoder angle "));
  Serial.print(pp_search_angle*180/M_PI);
  Serial.print("/");
  Serial.print((angle_end-angle_begin)*180/M_PI);
  Serial.print(" = ");
  Serial.println((pp_search_angle)/(angle_end-angle_begin));
  Serial.println();

  // a bit of monitoring the result
  if(pp <= 0 ){
    Serial.println(F("PP number cannot be negative"));
    Serial.println(F(" - Try changing the search_voltage value or motor/encoder configuration."));
    return;
  }else if(pp > 30){
    Serial.println(F("PP number very high, possible error."));
  }else{
    Serial.println(F("If PP is estimated well your motor should turn now!"));
    Serial.println(F(" - If it is not moving try to relaunch the program!"));
    Serial.println(F(" - You can also try to adjust the target voltage using serial terminal!"));
  }

  // set FOC loop to be used
  motor.controller = MotionControlType::torque;
  // set the pole pair number to the motor
  motor.pole_pairs = pp;
  //align encoder and start FOC
  motor.initFOC();
  _delay(1000);

  Serial.println(F("\n Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
}

void sensor_test1(void)
{
  init_sensor();
  motor.linkSensor(&rotSense);
}

void sensor_test2(void)
{
#if ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM
  Serial.begin(115200);

  init_sensor();

  while(1) {
    static unsigned long max_pulse = 0;
    static unsigned long min_pulse = 10000;

    rotSense.update();

    if (rotSense.pulse_length_us > max_pulse) max_pulse = rotSense.pulse_length_us;
    else if (rotSense.pulse_length_us < min_pulse) min_pulse = rotSense.pulse_length_us;

    Serial.print("angle:");
    Serial.print(rotSense.getAngle());
    Serial.print("\t, raw:");
    Serial.print(rotSense.pulse_length_us);
    Serial.print("\t, min:");
    Serial.print(min_pulse);
    Serial.print("\t, max:");
    Serial.println(max_pulse);
  }
#else
  while (1) {
    Serial.println("Not supported configuration");
    delay(1000);
  }
#endif
}

void sensor_test3(void)
{
  Serial.begin(115200);

  init_sensor();

  while(1) {
    rotSense.update();

    Serial.print("angle:");
    Serial.print(rotSense.getAngle());
    Serial.print("\t,");
    Serial.print("sensor angle:");
    Serial.print(rotSense.getSensorAngle());
    Serial.print("\t,");
    Serial.print("mech angle:");
    Serial.print(rotSense.getMechanicalAngle());
    Serial.print("\t,");
    Serial.print("prec angle:");
    Serial.print(rotSense.getPreciseAngle());
    Serial.print("\t,");
    Serial.print("full rot:");
    Serial.print(rotSense.getFullRotations());
    Serial.print("\t,");
    Serial.print("vel:");
    Serial.println(rotSense.getVelocity());
  }
}

void demo_move(void)
{
  static float offset = 2.22;
  static float target = 0.0;
  static int state = 0;
  static int state2 = 0;
  static int pretime = 0;

  int curtime = millis();

  switch(state) {
  case 0:
    pretime = millis();
    state++;
    break;
  case 1:
    motor.controller = MotionControlType::velocity;
    state2 = 0;
    state++;
    break;
  case 2:
    switch (state2)
    {
    case 0:
      target = 5.0;
      if (curtime - pretime >= 3000) { pretime = curtime; state2++; }
      break;
    case 1:
      target = -5.0;
      if (curtime - pretime >= 3000) { pretime = curtime; state2++; }
      break;
    case 2:
      target = 10.0;
      if (curtime - pretime >= 3000) { pretime = curtime; state2++; }
      break;
    case 3:
      target = -10.0;
      if (curtime - pretime >= 3000) { pretime = curtime; state2++; }
      break;
    case 4:
      target = 20.0;
      if (curtime - pretime >= 3000) { pretime = curtime; state2++; }
      break;
    case 5:
      target = -20.0;
      if (curtime - pretime >= 3000) { pretime = curtime; state2++; }
      break;
    case 6:
      state2 = 0;
      state++;
      break;
    default:
      state++;
    }
    break;
  case 3:
    motor.controller = MotionControlType::angle;
    state2 = 0;
    state++;
    break;
  case 4:
    target = 0;
    if (curtime - pretime >= 1000) { pretime = curtime; state++; }
    break;
  case 5:
    switch (state2)
    {
    case 0:
      target = ((_PI / 2.0) * 1);
      if (curtime - pretime >= 300) { pretime = curtime; state2++; }
      break;
    case 1:
      target = ((_PI / 2.0) * 2);
      if (curtime - pretime >= 300) { pretime = curtime; state2++; }
      break;
    case 2:
      target = ((_PI / 2.0) * 3);
      if (curtime - pretime >= 300) { pretime = curtime; state2++; }
      break;
    case 3:
      target = ((_PI / 2.0) * 4);
      if (curtime - pretime >= 300) { pretime = curtime; state2++; }
      break;
    case 4:
      target = ((_PI / 2.0) * 3);
      if (curtime - pretime >= 300) { pretime = curtime; state2++; }
      break;
    case 5:
      target = ((_PI / 2.0) * 2);
      if (curtime - pretime >= 300) { pretime = curtime; state2++; }
      break;
    case 6:
      target = ((_PI / 2.0) * 1);
      if (curtime - pretime >= 300) { pretime = curtime; state2++; }
      break;
    case 7:
      target = ((_PI / 2.0) * 0);
      if (curtime - pretime >= 300) { pretime = curtime; state2++; }
      break;
    case 8:
      state2 = 0;
      state++;
      break;
    default:
      state++;
    }
    break;
  case 6:
    switch (state2)
    {
    case 0:
      target = ((_PI / 2.0) * 2);
      if (curtime - pretime >= 400) { pretime = curtime; state2++; }
      break;
    case 1:
      target = ((_PI / 2.0) * 4);
      if (curtime - pretime >= 400) { pretime = curtime; state2++; }
      break;
    case 2:
      target = ((_PI / 2.0) * 6);
      if (curtime - pretime >= 400) { pretime = curtime; state2++; }
      break;
    case 3:
      target = ((_PI / 2.0) * 8);
      if (curtime - pretime >= 400) { pretime = curtime; state2++; }
      break;
    case 4:
      target = ((_PI / 2.0) * 6);
      if (curtime - pretime >= 400) { pretime = curtime; state2++; }
      break;
    case 5:
      target = ((_PI / 2.0) * 4);
      if (curtime - pretime >= 400) { pretime = curtime; state2++; }
      break;
    case 6:
      target = ((_PI / 2.0) * 2);
      if (curtime - pretime >= 400) { pretime = curtime; state2++; }
      break;
    case 7:
      target = ((_PI / 2.0) * 0);
      if (curtime - pretime >= 400) { pretime = curtime; state2++; }
      break;
    case 8:
      state2 = 0;
      state++;
      break;
    default:
      state++;
    }
    break;
  default:
    state = 0;
    break;
  }
  
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // Motion control function
  if (motor.controller == MotionControlType::angle)
    motor.move(target + offset);
  else
    motor.move(target);
}
