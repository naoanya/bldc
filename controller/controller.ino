/** 
 * BLDC Motor Controller using SimpleFOC
 */
#include <Arduino.h>
#include "SimpleFOC.h"

/*********************************************************************/

#include "config.hpp"

/*********************************************************************/

#define DRIVER_TYPE_BLDC_6PWM               (0)
#define DRIVER_TYPE_BLDC_3PWM               (1)

#define ROT_SENSE_TYPE_NONE                 (0)
#define ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM   (1)
#define ROT_SENSE_TYPE_MAGNETIC_SENSE_I2C   (2)
#define ROT_SENSE_TYPE_ENCODER              (3)

#define CURRENT_SENSE_TYPE_NONE             (0)
#define CURRENT_SENSE_TYPE_LOWSIZE          (1)

/*********************************************************************/

#ifndef COMMANDER_SERIAL
#define COMMANDER_SERIAL      Serial
#endif

#ifndef MONITOR_SERIAL
#define MONITOR_SERIAL        Serial
#endif

#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL          Serial
#endif

/*********************************************************************/

static void init_rot_sensor(void);

static void find_pole_pair(void);
static void rot_sensor_pulse_len_checker(void);
static void rot_sensor_monitor(void);

static void encoder_monitor(void);
static void demo_move(void);

/*********************************************************************/

BLDCMotor motor = BLDCMotor(
  #if !ENABLE_FIND_POLE_PAIR
    POLE_PAIR
    #ifdef PHASE_RESISTANCE
    , PHASE_RESISTANCE
    #endif
  #else
    1
  #endif
);

#if DRIVER_TYPE == DRIVER_TYPE_BLDC_6PWM
  BLDCDriver6PWM driver = BLDCDriver6PWM(
    PIN_PHASE_UH,
    PIN_PHASE_UL,
    PIN_PHASE_VH,
    PIN_PHASE_VL,
    PIN_PHASE_WH,
    PIN_PHASE_WL
    );
#elif DRIVER_TYPE == DRIVER_TYPE_BLDC_3PWM
  BLDCDriver3PWM driver = BLDCDriver3PWM(
    PIN_PHASE_A,
    PIN_PHASE_B,
    PIN_PHASE_C,
    PIN_ENA_1,
    PIN_ENA_2,
    PIN_ENA_3
    );
#else
  #error "No DRIVER_TYPE defined."
#endif

#if CURRENT_SENSE_TYPE == CURRENT_SENSE_TYPE_LOWSIZE
  LowsideCurrentSense currentSense = LowsideCurrentSense(
    LOWSIDE_CURSENSE_SHUNT,
    LOWSIDE_CURSENSE_GAIN,
    LOWSIDE_CURSENSE_PIN_A_PHASE_ADC,
    LOWSIDE_CURSENSE_PIN_B_PHASE_ADC,
    LOWSIDE_CURSENSE_PIN_C_PHASE_ADC
    );
#elif CURRENT_SENSE_TYPE == CURRENT_SENSE_TYPE_NONE
  // Not use
#else
  #error "No CURRENT_SENSE_TYPE defined."
#endif

#if ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM
  MagneticSensorPWM rotSense = MagneticSensorPWM(
		MAGSENSE_PWM_PIN,
    MAGSENSE_PWM_MIN_CLOCK,
    MAGSENSE_PWM_MAX_CLOCK
    );
  void doPWM(){rotSense.handlePWM();}
#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_I2C
  MagneticSensorI2C rotSense = MagneticSensorI2C(
    AS5600_I2C
    );
#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_ENCODER
  Encoder rotSense = Encoder(
    A_HALL1, 
    A_HALL2, 
    2048, 
    A_HALL3
    );
  void doA(){rotSense.handleA();}
  void doB(){rotSense.handleB();}
  void doZ(){rotSense.handleIndex();}
#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_NONE
  // Not use
#else
  #error "No ROT_SENSE_TYPE defined."
#endif

/*********************************************************************/

// Encoder monitor inverval
// < 0 : disable
// 0 < : monitor interval ms
int encoderMonitorIntervalMs_ = -1;
int preEncoderMonitorTimeMs_ = 0;

// SimpleFOC monitor enable
// == 0 : disable
// != 0 : enable
int enableSimpleFOCMonitor_ = 0;

// Demo mode enable flag
// == 0 : disable
// != 0 : enable
int enableDemo_ = 0;

// instantiate the commander
Commander command = Commander(COMMANDER_SERIAL);
void doTarget(char* cmd) { command.motion(&motor, cmd); }
void doCommand(char* cmd) { command.motor(&motor, cmd); }
void doEncoder(char* cmd) { encoderMonitorIntervalMs_ = atoi(cmd); }
void doMonitor(char* cmd) { enableSimpleFOCMonitor_ = atoi(cmd); }
void doDemo(char* cmd) { enableDemo_ = atoi(cmd); }
void doReset(char* cmd) { HAL_NVIC_SystemReset(); }

/*********************************************************************/

void setup()
{
  init_serial();

  // wait for host ready
  //delay(3000);
  //Serial.printf("Start Program\n");

  init_wire();

  #if ENABLE_ROT_SENSOR_PULSE_LEN_CHECKER
  rot_sensor_pulse_len_checker();
  while (1) {;}
  #endif

  #if ENABLE_ROT_SENSOR_MONITOR
  rot_sensor_monitor();
  while (1) {;}
  #endif

  #if ENABLE_FIND_POLE_PAIR
  find_pole_pair();
  while (1) {;}
  #endif

  init_rot_sensor();

  driver.voltage_power_supply = VOLTAGE_POWER_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  init_current_sensor();

  // general settings
  // motor phase resistance
  motor.phase_resistance = PHASE_RESISTANCE;

  init_motor_config();

  //SimpleFOCDebug::enable();
  //SimpleFOCDebug::enable(&DEBUG_SERIAL);
  command.verbose = VerboseMode::user_friendly;

  // use monitoring with serial
  motor.useMonitoring(MONITOR_SERIAL);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, (char*)"target angle");
  command.add('M', doCommand, (char*)"motor command");
  command.add('E', doEncoder, (char*)"encoder monitor");
  command.add('S', doEncoder, (char*)"simplefoc monitor");
  command.add('D', doDemo, (char*)"demo");
  command.add('R', doReset, (char*)"system reset");

  DEBUG_SERIAL.println(F("Motor ready."));
  DEBUG_SERIAL.println(F("Set the target angle using serial terminal:"));
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

  if (enableSimpleFOCMonitor_) {
    // function intended to be used with serial plotter to monitor motor variables
    // significantly slowing the execution down!!!!
    motor.monitor();
  }

  // user communication
  command.run();

  // Encoder monitor
  if (encoderMonitorIntervalMs_ >= 0) {
    encoder_monitor();
  }
}

/*********************************************************************/

void init_rot_sensor(void)
{
#if ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM
  rotSense.init();
  rotSense.enableInterrupt(doPWM);
  motor.linkSensor(&rotSense);
#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_I2C
  rotSense.init(&Wire);
  // @note If run setClock() before sensor.init(), the system will hang.
  // STM32-DUINO only problem?
  Wire.setClock(400000);
  motor.linkSensor(&rotSense);
#elif ROT_SENSE_TYPE == ROT_SENSE_TYPE_ENCODER
  rotSense.init();
  rotSense.enableInterrupts(doA, doB, doZ);
  motor.linkSensor(&rotSense);
#endif
}

void init_current_sensor(void)
{
#if CURRENT_SENSE_TYPE != CURRENT_SENSE_TYPE_NONE
  currentSense.linkDriver(&driver);
  currentSense.init();
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);
#endif
}

/*********************************************************************/

#if ENABLE_FIND_POLE_PAIR
void find_pole_pair(void)
{
  init_rot_sensor();

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.init();

  DEBUG_SERIAL.begin(115200);

  // pole pairs calculation routine
  DEBUG_SERIAL.println("Pole pairs (PP) estimator");
  DEBUG_SERIAL.println("-\n");
  _delay(1000);
  DEBUG_SERIAL.println("-5\n");
  _delay(1000);
  DEBUG_SERIAL.println("-4\n");
  _delay(1000);
  DEBUG_SERIAL.println("-3\n");
  _delay(1000);
  DEBUG_SERIAL.println("-2\n");
  _delay(1000);
  DEBUG_SERIAL.println("-1\n");
  _delay(1000);

  float pp_search_voltage = POLE_PAIRE_SEARCH_VOLTAGE; // maximum power_supply_voltage/2
  float pp_search_angle = 1*M_PI; // search electrical angle to turn

  // move motor to the electrical angle 0
  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit = pp_search_voltage;
  
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
    _delay(1);
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

  DEBUG_SERIAL.print(F("Estimated PP : "));
  DEBUG_SERIAL.println(pp);
  DEBUG_SERIAL.println(F("PP = Electrical angle / Encoder angle "));
  DEBUG_SERIAL.print(pp_search_angle*180/M_PI);
  DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print((angle_end-angle_begin)*180/M_PI);
  DEBUG_SERIAL.print(" = ");
  DEBUG_SERIAL.println((pp_search_angle)/(angle_end-angle_begin));
  DEBUG_SERIAL.println();

  // a bit of monitoring the result
  if(pp <= 0 ){
    DEBUG_SERIAL.println(F("PP number cannot be negative"));
    DEBUG_SERIAL.println(F(" - Try changing the search_voltage value or motor/encoder configuration."));
    return;
  }else if(pp > 30){
    DEBUG_SERIAL.println(F("PP number very high, possible error."));
  }else{
    DEBUG_SERIAL.println(F("If PP is estimated well your motor should turn now!"));
    DEBUG_SERIAL.println(F(" - If it is not moving try to relaunch the program!"));
    DEBUG_SERIAL.println(F(" - You can also try to adjust the target voltage using serial terminal!"));
  }
}
#endif

#if ENABLE_ROT_SENSOR_PULSE_LEN_CHECKER
void rot_sensor_pulse_len_checker(void)
{
#if ROT_SENSE_TYPE == ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM
  init_rot_sensor();

  while(1) {
    static unsigned long max_pulse = 0;
    static unsigned long min_pulse = 10000;

    rotSense.update();

    if (rotSense.pulse_length_us > max_pulse) max_pulse = rotSense.pulse_length_us;
    else if (rotSense.pulse_length_us < min_pulse) min_pulse = rotSense.pulse_length_us;

    DEBUG_SERIAL.print("angle:");
    DEBUG_SERIAL.print(rotSense.getAngle());
    DEBUG_SERIAL.print("\t, raw:");
    DEBUG_SERIAL.print(rotSense.pulse_length_us);
    DEBUG_SERIAL.print("\t, min:");
    DEBUG_SERIAL.print(min_pulse);
    DEBUG_SERIAL.print("\t, max:");
    DEBUG_SERIAL.println(max_pulse);
  }
#else
  while (1) {
    DEBUG_SERIAL.println("Not supported configuration");
    delay(1000);
  }
#endif
}
#endif

#if ENABLE_ROT_SENSOR_MONITOR
void rot_sensor_monitor(void)
{
  init_rot_sensor();

  while(1) {
    rotSense.update();

    DEBUG_SERIAL.print("angle:");
    DEBUG_SERIAL.print(rotSense.getAngle());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("sensor angle:");
    DEBUG_SERIAL.print(rotSense.getSensorAngle());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("mech angle:");
    DEBUG_SERIAL.print(rotSense.getMechanicalAngle());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("prec angle:");
    DEBUG_SERIAL.print(rotSense.getPreciseAngle());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("full rot:");
    DEBUG_SERIAL.print(rotSense.getFullRotations());
    DEBUG_SERIAL.print("\t,");
    DEBUG_SERIAL.print("vel:");
    DEBUG_SERIAL.println(rotSense.getVelocity());
  }
}
#endif

/*********************************************************************/

void encoder_monitor(void)
{
  int curEncoderMonitorTimeMs = millis();
  if (curEncoderMonitorTimeMs - preEncoderMonitorTimeMs_ >= encoderMonitorIntervalMs_) {
    preEncoderMonitorTimeMs_ = curEncoderMonitorTimeMs;
    COMMANDER_SERIAL.print("E ");
    COMMANDER_SERIAL.print(rotSense.getAngle());
    COMMANDER_SERIAL.print(" ");
    COMMANDER_SERIAL.print(rotSense.getVelocity());
    #if 0
    COMMANDER_SERIAL.print(" ");
    COMMANDER_SERIAL.print(100);
    COMMANDER_SERIAL.print(" ");
    COMMANDER_SERIAL.print(-100);
    #endif
    COMMANDER_SERIAL.println();
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
    case 0: target =   5.0; if (curtime - pretime >= 3000) { pretime = curtime; state2++; } break;
    case 1: target =  -5.0; if (curtime - pretime >= 3000) { pretime = curtime; state2++; } break;
    case 2: target =  10.0; if (curtime - pretime >= 3000) { pretime = curtime; state2++; } break;
    case 3: target = -10.0; if (curtime - pretime >= 3000) { pretime = curtime; state2++; } break;
    case 4: target =  20.0; if (curtime - pretime >= 3000) { pretime = curtime; state2++; } break;
    case 5: target = -20.0; if (curtime - pretime >= 3000) { pretime = curtime; state2++; } break;
    case 6: state2 = 0; state++; break;
    default: state++;
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
    case 0: target = ((_PI / 2.0) * 1); if (curtime - pretime >= 300) { pretime = curtime; state2++; } break;
    case 1: target = ((_PI / 2.0) * 2); if (curtime - pretime >= 300) { pretime = curtime; state2++; } break;
    case 2: target = ((_PI / 2.0) * 3); if (curtime - pretime >= 300) { pretime = curtime; state2++; } break;
    case 3: target = ((_PI / 2.0) * 4); if (curtime - pretime >= 300) { pretime = curtime; state2++; } break;
    case 4: target = ((_PI / 2.0) * 3); if (curtime - pretime >= 300) { pretime = curtime; state2++; } break;
    case 5: target = ((_PI / 2.0) * 2); if (curtime - pretime >= 300) { pretime = curtime; state2++; } break;
    case 6: target = ((_PI / 2.0) * 1); if (curtime - pretime >= 300) { pretime = curtime; state2++; } break;
    case 7: target = ((_PI / 2.0) * 0); if (curtime - pretime >= 300) { pretime = curtime; state2++; } break;
    case 8: state2 = 0; state++; break;
    default: state++;
    }
    break;
  case 6:
    switch (state2)
    {
    case 0: target = ((_PI / 2.0) * 2); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 1: target = ((_PI / 2.0) * 4); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 2: target = ((_PI / 2.0) * 6); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 3: target = ((_PI / 2.0) * 8); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 4: target = ((_PI / 2.0) * 6); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 5: target = ((_PI / 2.0) * 4); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 6: target = ((_PI / 2.0) * 2); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 7: target = ((_PI / 2.0) * 0); if (curtime - pretime >= 400) { pretime = curtime; state2++; } break;
    case 8: state2 = 0; state++; break;
    default: state++;
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
