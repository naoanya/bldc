//
// * Controller : B-G431B-ESC1
// * Motor : T-Motor MN1005
// * MCU : STM32G431CB
// * Encoder : AS5048
// * Description :
//    Setup to TorqueControlType::voltage
//

#define VOLTAGE_POWER_SUPPLY        (12)

#define POLE_PAIR                   (21)
#define PHASE_RESISTANCE            (0.168)

#define DRIVER_TYPE                 (DRIVER_TYPE_BLDC_6PWM)
#define CURRENT_SENSE_TYPE          (CURRENT_SENSE_TYPE_LOWSIZE)
#define ROT_SENSE_TYPE              (ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM)

#define PIN_PHASE_UH                (A_PHASE_UH)
#define PIN_PHASE_UL                (A_PHASE_UL)
#define PIN_PHASE_VH                (A_PHASE_VH)
#define PIN_PHASE_VL                (A_PHASE_VL)
#define PIN_PHASE_WH                (A_PHASE_WH)
#define PIN_PHASE_WL                (A_PHASE_WL)

#define LOWSIDE_CURSENSE_SHUNT                  (0.003)
#define LOWSIDE_CURSENSE_GAIN                   (-64.0/7.0)
#define LOWSIDE_CURSENSE_PIN_A_PHASE_ADC        (A_OP1_OUT)
#define LOWSIDE_CURSENSE_PIN_B_PHASE_ADC        (A_OP2_OUT)
#define LOWSIDE_CURSENSE_PIN_C_PHASE_ADC        (A_OP3_OUT)

#define MAGSENSE_PWM_PIN            (A_PWM)
#define MAGSENSE_PWM_MIN_CLOCK      (3)
#define MAGSENSE_PWM_MAX_CLOCK      (917)

// Debug options
#define ENABLE_FIND_POLE_PAIR                   (0)
#define  POLE_PAIRE_SEARCH_VOLTAGE              (0.5)
#define ENABLE_ROT_SENSOR_PULSE_LEN_CHECKER     (0)
#define ENABLE_ROT_SENSOR_MONITOR               (0)

/*********************************************************************/

void init_serial(void)
{
  Serial.begin(115200);
}

void init_wire(void)
{
  // Not use
}

void init_motor_config(void)
{
  extern BLDCMotor motor;
  
  // aligning voltage [V]
  motor.voltage_sensor_align = 1;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // TorqueControlType::voltage
  // TorqueControlType::dc_current
  // TorqueControlType::foc_current
  motor.torque_controller = TorqueControlType::voltage;

  // MotionControlType::torque
  // MotionControlType::velocity
  // MotionControlType::angle
  // MotionControlType::velocity_openloop
  // MotionControlType::angle_openloop
  motor.controller = MotionControlType::velocity;

  motor.motion_downsample = DEF_MOTION_DOWNSMAPLE;

  // velocity loop PID
  motor.PID_velocity.P = 0.03;
  motor.PID_velocity.I = 10.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000.0;
  //motor.PID_velocity.limit = 10.0;
  motor.LPF_velocity.Tf = 0.001;

  // angle loop PID
  motor.P_angle.P = 10.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  //motor.P_angle.output_ramp = 0.0;
  //motor.P_angle.limit = 1000.0;
  motor.LPF_angle.Tf = 0.001;
  
  // current q loop PID
  //motor.PID_current_q.P = 0.0;
  //motor.PID_current_q.I = 0.0;
  //motor.PID_current_q.D = 0.0;
  //motor.PID_current_q.output_ramp = 0.0;
  //motor.PID_current_q.limit = 0.0;
  //motor.LPF_current_q.Tf = 0.0;
  
  // current d loop PID
  //motor.PID_current_d.P = 0.0;
  //motor.PID_current_d.I = 0.0;
  //motor.PID_current_d.D = 0.0;
  //motor.PID_current_d.output_ramp = 0.0;
  //motor.PID_current_d.limit = 0.0;
  //motor.LPF_current_d.Tf = 0.0;
  
  // Limits 
  motor.current_limit = 6.0;
  motor.voltage_limit = 2.0;
  motor.velocity_limit = 50.0;
  
  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;
}
