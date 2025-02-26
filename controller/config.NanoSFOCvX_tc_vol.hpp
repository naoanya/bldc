//
// * Module : NANO SFOC BLDC Motor version ?
// * Motor : ?
// * MCU : STM32F103CBT6
// * Encoder : AS5600
// * Description :
//    Setup to TorqueControlType::voltage
//

#define VOLTAGE_POWER_SUPPLY        (12)

#define POLE_PAIR                   (7)
#define PHASE_RESISTANCE            (8.5)

#define DRIVER_TYPE                 (DRIVER_TYPE_BLDC_3PWM)
#define CURRENT_SENSE_TYPE          (CURRENT_SENSE_TYPE_NONE)
#define ROT_SENSE_TYPE              (ROT_SENSE_TYPE_MAGNETIC_SENSE_I2C)

#define PIN_PHASE_A                 (PinName::PA_6)
#define PIN_PHASE_B                 (PinName::PA_7)
#define PIN_PHASE_C                 (PinName::PB_0)
#define PIN_ENA_1                   (PinName::PB_12)
#define PIN_ENA_2                   (NOT_SET)
#define PIN_ENA_3                   (NOT_SET)

// Debug options
#define ENABLE_FIND_POLE_PAIR                   (0)
#define  POLE_PAIRE_SEARCH_VOLTAGE              (0.5)
#define ENABLE_ROT_SENSOR_PULSE_LEN_CHECKER     (0)
#define ENABLE_ROT_SENSOR_MONITOR               (0)

/*********************************************************************/

void init_serial(void)
{
  Serial.setRx(PinName::PA_10);
  Serial.setTx(PinName::PA_9);
  Serial.begin(115200);
}

void init_wire(void)
{
  Wire.setSCL(PinName::PB_6);
  Wire.setSDA(PinName::PB_7);
  // @note Wire.begin() will be called in sensor.init()
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
  motor.PID_velocity.P = 0.005;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 1000.0;
  //motor.PID_velocity.limit = 10.0;
  motor.LPF_velocity.Tf = 0.001;

  // angle loop PID
  motor.P_angle.P = 15.0;
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
  motor.voltage_limit = 4.0;
  motor.velocity_limit = 50.0;
  
  // pwm modulation settings
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;
}

/*********************************************************************/

#include <Arduino.h>

// Use HSE (Extern Clock 8 MHz)
// SYSCLOCK set to 72 MHz
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
