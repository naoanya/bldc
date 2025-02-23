/** 
 * B-G431B-ESC1 position motion control example with encoder
 *
 */
#include <Arduino.h>
#include "SimpleFOC.h"

BLDCMotor motor = BLDCMotor(14, 0.27);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
//MagneticSensorPWM sensor = MagneticSensorPWM(A_HALL1, 2, 925);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
//void doPWM(){sensor.handlePWM();}

// angle set point variable
float target_angle = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
void doCommand(char* cmd) { command.motor(&motor, cmd); }

int max_pulse= 0;
int min_pulse = 10000;
void sensor_test(void)
{
	// iterative function updating the sensor internal variables
	// it is usually called in motor.loopFOC()
	// this function reads the sensor hardware and
	// has to be called before getAngle nad getVelocity
	sensor.update();

#if 0
	// keep track of min and max
	if(sensor.pulse_length_us > max_pulse) max_pulse = sensor.pulse_length_us;
	else if(sensor.pulse_length_us < min_pulse) min_pulse = sensor.pulse_length_us;

	// display the raw count, and max and min raw count
	Serial.print("angle:");
	Serial.print(sensor.getAngle());
	Serial.print("\t, raw:");
	Serial.print(sensor.pulse_length_us);
	Serial.print("\t, min:");
	Serial.print(min_pulse);
	Serial.print("\t, max:");
	Serial.println(max_pulse);
#else
	Serial.print("angle:");
	Serial.print(sensor.getAngle());
	Serial.print("\t,");
	Serial.print("vel:");
	Serial.println(sensor.getVelocity());
#endif
}

void setup() {
  
  // initialise magnetic sensor hardware
  sensor.init(&Wire);
  Wire.setClock(400000);

#if 0
  // comment out to use sensor in blocking (non-interrupt) way
  sensor.enableInterrupt(doPWM);
#endif

  motor.linkSensor(&sensor);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // aligning voltage [V]
  motor.voltage_sensor_align = 1;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;


#if 0
  // set motion control loop to be used
  // MotionControlType::torque
  // MotionControlType::velocity
  // MotionControlType::angle
  // MotionControlType::velocity_openloop
  // MotionControlType::angle_openloop
  // TorqueControlType::voltage
  // TorqueControlType::dc_current
  // TorqueControlType::foc_current
#if 0
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
#elif 0
  motor.controller = MotionControlType::velocity;
#elif 1
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;
#endif

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;
  // angle P controller
  motor.P_angle.P = 20;
  
  //  maximal velocity of the position control
  motor.velocity_limit = 3;
  // default voltage_power_supply
  motor.voltage_limit = 3;
  // since the phase resistance is provided we set the current limit not voltage
  // default 0.2
  motor.current_limit = 1; // Amps
#else

  // control loop type and torque mode
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity;
  motor.motion_downsample = 0.0;

  // velocity loop PID
  motor.PID_velocity.P = 1.0;
  motor.PID_velocity.I = 10.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 100.0;
  motor.PID_velocity.limit = 0.3;
  // Low pass filtering time constant
  motor.LPF_velocity.Tf = 0.1;
  // angle loop PID
  motor.P_angle.P = 20.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  motor.P_angle.output_ramp = 0.0;
  motor.P_angle.limit = 1000.0;
  // Low pass filtering time constant
  motor.LPF_angle.Tf = 0.0;
  // current q loop PID
  motor.PID_current_q.P = 3.0;
  motor.PID_current_q.I = 100.0;
  motor.PID_current_q.D = 0.0;
  motor.PID_current_q.output_ramp = 0.0;
  motor.PID_current_q.limit = 7.0;
  // Low pass filtering time constant
  motor.LPF_current_q.Tf = 0.02;
  // current d loop PID
  motor.PID_current_d.P = 3.0;
  motor.PID_current_d.I = 100.0;
  motor.PID_current_d.D = 0.0;
  motor.PID_current_d.output_ramp = 0.0;
  motor.PID_current_d.limit = 7.0;
  // Low pass filtering time constant
  motor.LPF_current_d.Tf = 0.02;
  // Limits
  motor.velocity_limit = 1000.0;
  motor.voltage_limit = 7.0;
  motor.current_limit = 1.0;
  // sensor zero offset - home position
  motor.sensor_offset = 0.0;
  // general settings
  // motor phase resistance
  motor.phase_resistance = 3.477;
  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;


#endif

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");
  command.add('M', doCommand, "motor command");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}

//// angle set point variable
//float target_angle = 0;

void loop() {
#if 1
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // Motion control function
  motor.move(target_angle);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();
  
  // user communication
  command.run();
#else
  sensor_test();
#endif
}
