//
// MN1005
//

#define POLE_PAIR                   (21)
#define PHASE_RESISTANCE            (0.168)

#define ROT_SENSE_TYPE              ROT_SENSE_TYPE_MAGNETIC_SENSE_PWM
#define MAGSENSE_PWM_MIN_CLOCK      (3)
#define MAGSENSE_PWM_MAX_CLOCK      (917)

#define VOLTAGE_POWER_SUPPLY        (12)

#define TORQUE_CTRL                 (TorqueControlType::voltage)
#define MOTION_CTRL                 (MotionControlType::velocity)

#define VELOCITY_PID_P              (0.03)
#define VELOCITY_PID_I              (10.0)
#define VELOCITY_PID_D              (0.0)
#define VELOCITY_OUTPUT_RAMP        (1000.0)
//#define VELOCITY_LIMIT              (10.0)
#define VELOCITY_FILTER             (0.001)

#define ANGLE_PID_P                 (10.00)
#define ANGLE_PID_I                 (0.0)
#define ANGLE_PID_D                 (0.0)
//#define ANGLE_OUTPUT_RAMP           (0.0)
//#define ANGLE_LIMIT                 (1000.0)
#define ANGLE_FILTER                (0.001)

//#define Q_CUR_PID_P                 (0.00)
//#define Q_CUR_PID_I                 (0.0)
//#define Q_CUR_PID_D                 (0.0)
//#define Q_CUR_OUTPUT_RAMP           (0.0)
//#define Q_CUR_LIMIT                 (0.0)
//#define Q_CUR_FILTER                (0.001)

//#define D_CUR_PID_P                 (0.00)
//#define D_CUR_PID_I                 (0.0)
//#define D_CUR_PID_D                 (0.0)
//#define D_CUR_OUTPUT_RAMP           (0.0)
//#define D_CUR_LIMIT                 (0.0)
//#define D_CUR_FILTER                (0.0)

#define CUR_LIMIT                   (6.0)
#define VOL_LIMIT                   (2.0)
#define VEL_LIMIT                   (50.0)

// Debug options
#define ENABLE_FIND_POLE_PAIR       (0)
#define ENABLE_SENSOR_TEST          (0)
#define ENABLE_SENSOR_TEST_2        (0)
#define ENABLE_SENSOR_TEST_3        (0)
