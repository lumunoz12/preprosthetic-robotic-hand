/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Control de prototipo de mano robótica pre-prótesica
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "cyl_targets.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HOMING_STALL_MS_I_MCP   400
#define HOMING_STALL_MS_I_PIP   400
#define HOMING_STALL_MS_P_ABD   800
#define HOMING_STALL_MS_P_MCP   800
#define HOMING_STALL_MS_M_MCP   400
#define HOMING_STALL_MS_M_PIP   400
#define HOMING_STALL_MS_A_MCP   400
#define HOMING_STALL_MS_A_PIP   400
#define HOMING_STALL_MS_Q_MCP   400
#define HOMING_STALL_MS_Q_PIP   400
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE BEGIN PM */

// ===== INDICE MCP (PC8/PC9, TIM14) =====
#define INDICE_MCP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   \
} while(0)
#define INDICE_MCP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); \
} while(0)
#define INDICE_MCP_STOP() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);     \
} while(0)

// ===== INDICE PIP (PC10/PC11, TIM13) =====
#define INDICE_PIP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   \
} while(0)
#define INDICE_PIP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); \
} while(0)
#define INDICE_PIP_STOP() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);      \
} while(0)

// ===== PULGAR ABD (PC12/PD2, TIM12 CH1) =====
#define PULGAR_ABD_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,  GPIO_PIN_SET);   \
} while(0)
#define PULGAR_ABD_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,  GPIO_PIN_RESET); \
} while(0)
#define PULGAR_ABD_STOP() do { \
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,  GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);      \
} while(0)

// ===== PULGAR MCP (PG2/PG3, TIM12 CH2) =====
#define PULGAR_MCP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);   \
} while(0)
#define PULGAR_MCP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); \
} while(0)
#define PULGAR_MCP_STOP() do { \
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);     \
} while(0)

// ===== MEDIO MCP (PD7/PD6, TIM11) =====
#define MEDIO_MCP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);   \
} while(0)
#define MEDIO_MCP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); \
} while(0)
#define MEDIO_MCP_STOP() do { \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);     \
} while(0)

// ===== MEDIO PIP (PD5/PD4, TIM10) =====
#define MEDIO_PIP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);   \
} while(0)
#define MEDIO_PIP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); \
} while(0)
#define MEDIO_PIP_STOP() do { \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);     \
} while(0)

// ===== ANULAR MCP (PF3/PF5, TIM9 CH1) =====
#define ANULAR_MCP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);   \
} while(0)
#define ANULAR_MCP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET); \
} while(0)
#define ANULAR_MCP_STOP() do { \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);      \
} while(0)

// ===== ANULAR PIP (PD1/PG0, TIM9 CH2) =====
#define ANULAR_PIP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_SET);   \
} while(0)
#define ANULAR_PIP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET); \
} while(0)
#define ANULAR_PIP_STOP() do { \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);      \
} while(0)

// ===== MENIQUE MCP (PE2/PE4, TIM5 CH1) =====
#define MENIQUE_MCP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);   \
} while(0)
#define MENIQUE_MCP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); \
} while(0)
#define MENIQUE_MCP_STOP() do { \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);      \
} while(0)

// ===== MENIQUE PIP (PE7/PE8, TIM5 CH2) =====
#define MENIQUE_PIP_FORWARD() do { \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);   \
} while(0)
#define MENIQUE_PIP_BACKWARD() do { \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);   \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); \
} while(0)
#define MENIQUE_PIP_STOP() do { \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET); \
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET); \
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);      \
} while(0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */

#define PPR_MOTOR  7
#define REDUCCION  298
#define PPR_SALIDA (PPR_MOTOR * 4 * REDUCCION * 2)

// ===== Ganancias LQR (iguales para todos los motores) =====
static const float K1 = 2192.4f;
static const float K2 =  757.4f;

// ===== PWM =====
static const float deadzone_pwm        = 1400.0f;
static const float deadzone_pwm_homing        = 1400.0f;  // indice
static const float deadzone_pwm_homing_pulgar = 1400.0f;  // pulgar ABD y MCP
static const float deadzone_pwm_anular        = 1400.0f;  // <-- NUEVO: normal anular
static const float deadzone_pwm_homing_anular_mcp = 1400.0f;  // MCP necesita mas fuerza
static const float deadzone_pwm_homing_anular_pip = 1400.0f;  // PIP ya andaba con esto
static const float deadzone_pwm_menique        = 1400.0f;
static const float deadzone_pwm_homing_menique = 1400.0f;
static const float max_pwm             = 4095.0f;

// ===== FSR umbral =====
#define FSR_UMBRAL  1500
static uint32_t fsr_indice_val = 0;
static uint32_t fsr_pulgar_val = 0;

// ===== UART =====
static uint8_t rx_byte       = 0;
static uint8_t rx_buffer[128] = {0};
static uint8_t rx_index      = 0;

// ===== Tiempo =====
static uint32_t lastTime  = 0;
static uint32_t lastLog   = 0;
static uint32_t startTime = 0;

// ============================================================
//  DEDO INDICE
// ============================================================
// MCP (TIM1 encoder, TIM14 PWM)
static int32_t encCount_i_mcp  = 0;
static int32_t lastCount_i_mcp = 0;
static float   curDeg_i_mcp    = 0.0f;
static float   lastDeg_i_mcp   = 0.0f;
static float   vel_i_mcp       = 0.0f;
static float   tgtDeg_i_mcp    = 0.0f;
static float   tgtRad_i_mcp    = 0.0f;
static uint8_t running_i_mcp   = 0;
static uint8_t stopped_i_mcp   = 0;

// PIP (TIM2 encoder, TIM13 PWM)
static int32_t encCount_i_pip  = 0;
static int32_t lastCount_i_pip = 0;
static float   curDeg_i_pip    = 0.0f;
static float   lastDeg_i_pip   = 0.0f;
static float   vel_i_pip       = 0.0f;
static float   tgtDeg_i_pip    = 0.0f;
static float   tgtRad_i_pip    = 0.0f;
static uint8_t running_i_pip   = 0;
static uint8_t stopped_i_pip   = 0;

// FSR indice
static uint8_t fsr_indice_blocked = 0;

// Homing indice
static uint8_t  homing_i          = 0;
static uint32_t homingStallTime_i = 0;
static int32_t  homingLastCount_i = 0;
static uint8_t  homingStallInit_i = 0;
static uint8_t  homing_i_stage    = 0; // 0=MCP, 1=PIP

// ============================================================
//  PULGAR
// ============================================================
// ABD (TIM3 encoder, TIM12 CH1)
static int32_t encCount_p_abd  = 0;
static int32_t lastCount_p_abd = 0;
static float   curDeg_p_abd    = 0.0f;
static float   lastDeg_p_abd   = 0.0f;
static float   vel_p_abd       = 0.0f;
static float   tgtDeg_p_abd    = 0.0f;
static float   tgtRad_p_abd    = 0.0f;
static uint8_t running_p_abd   = 0;
static uint8_t stopped_p_abd   = 0;

// MCP (TIM4 encoder, TIM12 CH2)
static int32_t encCount_p_mcp  = 0;
static int32_t lastCount_p_mcp = 0;
static float   curDeg_p_mcp    = 0.0f;
static float   lastDeg_p_mcp   = 0.0f;
static float   vel_p_mcp       = 0.0f;
static float   tgtDeg_p_mcp    = 0.0f;
static float   tgtRad_p_mcp    = 0.0f;
static uint8_t running_p_mcp   = 0;
static uint8_t stopped_p_mcp   = 0;

// FSR pulgar
static uint8_t fsr_pulgar_blocked = 0;

// Homing pulgar
static uint8_t  homing_p          = 0;
static uint32_t homingStallTime_p = 0;
static int32_t  homingLastCount_p = 0;
static uint8_t  homingStallInit_p = 0;
static uint8_t  homing_p_stage    = 0; // 0=ABD, 1=MCP

// ============================================================
//  DEDO MEDIO
// ============================================================
static volatile int32_t encCount_m_mcp = 0;   //
static float   curDeg_m_mcp    = 0.0f;
static float   lastDeg_m_mcp   = 0.0f;
static float   vel_m_mcp       = 0.0f;
static float   tgtDeg_m_mcp    = 0.0f;
static float   tgtRad_m_mcp    = 0.0f;
static uint8_t running_m_mcp   = 0;
static uint8_t stopped_m_mcp   = 0;

// PIP (TIM8 encoder, TIM10 PWM)
static int32_t encCount_m_pip  = 0;
static int32_t lastCount_m_pip = 0;
static float   curDeg_m_pip    = 0.0f;
static float   lastDeg_m_pip   = 0.0f;
static float   vel_m_pip       = 0.0f;
static float   tgtDeg_m_pip    = 0.0f;
static float   tgtRad_m_pip    = 0.0f;
static uint8_t running_m_pip   = 0;
static uint8_t stopped_m_pip   = 0;

// Homing medio
static uint8_t  homing_m          = 0;
static uint32_t homingStallTime_m = 0;
static int32_t  homingLastCount_m = 0;
static uint8_t  homingStallInit_m = 0;
static uint8_t  homing_m_stage    = 0; // 0=MCP, 1=PIP

// ============================================================
//  DEDO ANULAR  (encoder por IRQ, TIM9 PWM)
// ============================================================
static volatile int32_t encCount_a_mcp = 0;  //
static float   curDeg_a_mcp  = 0.0f;
static float   lastDeg_a_mcp = 0.0f;
static float   vel_a_mcp     = 0.0f;
static float   tgtDeg_a_mcp  = 0.0f;
static float   tgtRad_a_mcp  = 0.0f;
static uint8_t running_a_mcp = 0;
static uint8_t stopped_a_mcp = 0;


static volatile int32_t encCount_a_pip = 0;  //
static float   curDeg_a_pip  = 0.0f;
static float   lastDeg_a_pip = 0.0f;
static float   vel_a_pip     = 0.0f;
static float   tgtDeg_a_pip  = 0.0f;
static float   tgtRad_a_pip  = 0.0f;
static uint8_t running_a_pip = 0;
static uint8_t stopped_a_pip = 0;

// Homing anular
static uint8_t  homing_a          = 0;
static uint32_t homingStallTime_a = 0;
static int32_t  homingLastCount_a = 0;
static uint8_t  homingStallInit_a = 0;
static uint8_t  homing_a_stage    = 0; // 0=MCP, 1=PIP

static volatile uint32_t irq_mcp = 0;
static volatile uint32_t irq_pip = 0;
static volatile uint32_t cnt_mcp = 0;
static volatile uint32_t cnt_pip = 0;
static volatile uint32_t min_dt_mcp = 0xFFFFFFFF;
static volatile uint32_t min_dt_pip = 0xFFFFFFFF;

// ============================================================
//  DEDO MENIQUE  (encoder por IRQ, TIM5 PWM)
// ============================================================
static volatile int32_t encCount_q_mcp = 0;
static float   curDeg_q_mcp  = 0.0f;
static float   lastDeg_q_mcp = 0.0f;
static float   vel_q_mcp     = 0.0f;
static float   tgtDeg_q_mcp  = 0.0f;
static float   tgtRad_q_mcp  = 0.0f;
static uint8_t running_q_mcp = 0;
static uint8_t stopped_q_mcp = 0;

static volatile int32_t encCount_q_pip = 0;
static float   curDeg_q_pip  = 0.0f;
static float   lastDeg_q_pip = 0.0f;
static float   vel_q_pip     = 0.0f;
static float   tgtDeg_q_pip  = 0.0f;
static float   tgtRad_q_pip  = 0.0f;
static uint8_t running_q_pip = 0;
static uint8_t stopped_q_pip = 0;

// Homing meñique
static uint8_t  homing_q          = 0;
static uint32_t homingStallTime_q = 0;
static int32_t  homingLastCount_q = 0;
static uint8_t  homingStallInit_q = 0;
static uint8_t  homing_q_stage    = 0; // 0=MCP, 1=PIP

// ============================================================
//  CINEMATICA INVERSA — parametros geometricos
// ============================================================
// Indice: L0=27, L1=55, L2=38.16, L3=34
// Medio:  L0=27, L1=69, L2=45.5,  L3=34.8
// Pulgar: L0=45.51, L1=33.21, L2=22
//   pos_base=[26.127, -73.939, 0], tilt=50 deg


static uint8_t  ikp_seq_active = 0;
static uint8_t  ikp_seq_stage  = 0;
static float    ikp_target_mcp = 0;
// ===== Secuencia AGARRE LATERAL (indice -> ABD -> MCP) =====
static uint8_t grasp_lat_active = 0;
static uint8_t grasp_lat_stage  = 0;   // 0=espera indice, 1=espera ABD, 2=espera MCP
static float   grasp_lat_abd    = 0;
static float   grasp_lat_mcp    = 0;
// ===== Secuencia AGARRE PINZA (ABD -> MCP pulgar -> INDICE) =====
static uint8_t grasp_pin_active = 0;
static uint8_t grasp_pin_stage  = 0;   // 0=ABD, 1=MCP pulgar, 2=indice MCP+PIP
// ===== Secuencia AGARRE CILINDRICO (ABD -> MCP indice+pulgar+seguidores -> PIP indice+seguidores) =====
static uint8_t grasp_cil_active = 0;
static uint8_t grasp_cil_stage  = 0;   // 0=ABD pulgar, 1=MCP indice+pulgar+seguidores, 2=PIP indice+seguidores
static CylTargets cil_tg = {0};        // <-- objetivos por dedo, calculados al recibir el comando
static uint8_t grasp_cil2_active = 0;
static uint8_t grasp_cil2_stage  = 0;

static const float GRASP_I_MCP   = 75.0f;
static const float GRASP_I_PIP   = 45.0f;
static const float GRASP_P_MCP   = 80.0f;  // mcp_pulgar_fijo

// ===== Agarre de PINZA (yema) — angulos de la simulacion =====
static const float GRASP_PINZA_I_MCP = 85.0f;
static const float GRASP_PINZA_I_PIP = 70.0f;
static const float GRASP_PINZA_P_ABD = 70.0f;
static const float GRASP_PINZA_P_MCP = 55.0f;

// ===== Agarre CILINDRICO — angulos de la simulacion =====
static const float GRASP_CIL_MCP_P = 50.0f;   // mcp_pulgar_obj   (MCP pulgar)

// ===== Agarre CILINDRICO: PIP manuales (ajustables por dedo) =====
static float cil_pip_indice  = 60.0f;   // PIP indice   [deg]
static float cil_pip_medio   = 60.0f;   // PIP medio    [deg]
static float cil_pip_anular  = 60.0f;   // PIP anular   [deg]
static float cil_pip_menique = 60.0f;   // PIP menique  [deg]
// ===== Agarre CILINDRICO: ABD pulgar manual =====
static float cil_abd_pulgar = 80.0f;   // ABD pulgar [deg]


// ===== Trayectoria (perfil trapezoidal) =====
static const float TRAJ_VMAX = 40.0f;    // deg/s   velocidad de crucero
static const float TRAJ_AMAX = 200.0f;   // deg/s^2 aceleracion
static const float deadzone_pwm_i = 1400.0f;  // friccion+gravedad i_mcp
static const float deadzone_pwm_i_pip = 1400.0f;  // ajusta por experimento
static const float deadzone_pwm_p_abd = 1400.0f;  // ajusta por experimento
static const float deadzone_pwm_p_mcp = 1400.0f;  // ajusta por experimento

// ===== Agarre CILINDRICO 2 (inclinado) =====
// MCP calculados por dedo (R=36 y posicion fijos) -> NO tunear
static float cil2_mcp_indice  = 30.7f;
static float cil2_mcp_medio   = 47.6f;
static float cil2_mcp_anular  = 65.9f;
static float cil2_mcp_menique = 83.1f;
// PIP manuales (tunear a gusto)
static float cil2_pip_indice  = 45.0f;
static float cil2_pip_medio   = 82.0f;
static float cil2_pip_anular  = 60.0f;
static float cil2_pip_menique = 45.0f;
// Pulgar manual
static float cil2_abd_pulgar  = 60.0f;
static float cil2_mcp_pulgar  = 50.0f;

typedef struct {
    float    startDeg, endDeg, dir;
    float    vmax, amax, ta, tc, ttotal;
    uint32_t t0;
    uint8_t  active;
} Traj;

static Traj traj_i_mcp = {0};
static Traj traj_i_pip = {0};
static Traj traj_p_abd = {0};
static Traj traj_p_mcp = {0};
static Traj traj_m_mcp = {0};
static Traj traj_m_pip = {0};
static Traj traj_a_mcp = {0};
static Traj traj_a_pip = {0};
static Traj traj_q_mcp = {0};
static Traj traj_q_pip = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
static float calcular_abd_pulgar_lateral(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float calcular_abd_pulgar_lateral(void)
{
    // ----- Postura del indice (solo el MCP define la falange proximal) -----
    const float th2_i = GRASP_I_MCP;
    const float L0_i  = 27.0f;
    const float L1_i  = 55.0f;
    const float off_i = 33.3f;

    float phi2   = th2_i * (float)M_PI / 180.0f;
    float PMCP_r = L0_i,             PMCP_z = 0.0f;
    float PPIP_r = PMCP_r + cosf(phi2)*L1_i;
    float PPIP_z = PMCP_z + sinf(phi2)*L1_i;

    // Punto medio de la falange proximal del indice, en 3D
    float prox_x = off_i;
    float prox_y = PMCP_r + 0.5f*(PPIP_r - PMCP_r);
    float prox_z = PMCP_z + 0.5f*(PPIP_z - PMCP_z);

    // ----- Frame base del pulgar (identico a resolverPulgar) -----
    //  Xl=[sin t; -cos t; 0]  Yl=[cos t; sin t; 0]  Zl=[0;0;1]
    const float tilt = 50.0f;
    const float pbx = 26.127f, pby = -73.939f, pbz = 0.0f;
    float ct = cosf(tilt*(float)M_PI/180.0f);
    float st = sinf(tilt*(float)M_PI/180.0f);

    float dx = prox_x - pbx;
    float dy = prox_y - pby;
    float dz = prox_z - pbz;

    // Componentes en el marco local del pulgar:
    //   Vloc_y = Yl . V = ct*dx + st*dy
    //   Vloc_z = Zl . V = dz
    float Vloc_y = ct*dx + st*dy;
    float Vloc_z = dz;

    // Abduccion exacta (rotacion pura alrededor de Xl): atan2(z_loc, y_loc)
    float abd = atan2f(Vloc_z, Vloc_y) * 180.0f / (float)M_PI;

    if (abd < 0.0f)  abd = 0.0f;
    if (abd > 90.0f) abd = 90.0f;

    return abd;
}

static void traj_plan(Traj *t, float startDeg, float endDeg)
{
    float D = endDeg - startDeg;
    t->startDeg = startDeg;
    t->endDeg   = endDeg;
    t->dir      = (D >= 0.0f) ? 1.0f : -1.0f;
    t->amax     = TRAJ_AMAX;
    D = fabsf(D);

    if (D < 1e-3f) { t->ta=0; t->tc=0; t->ttotal=0; t->vmax=0; t->active=0; return; }

    float ta = TRAJ_VMAX / TRAJ_AMAX;
    float da = 0.5f * TRAJ_AMAX * ta * ta;

    if (2.0f*da > D) {                 // triangular (no alcanza VMAX)
        t->ta   = sqrtf(D / TRAJ_AMAX);
        t->vmax = TRAJ_AMAX * t->ta;
        t->tc   = 0.0f;
    } else {                           // trapezoidal
        t->ta   = ta;
        t->vmax = TRAJ_VMAX;
        t->tc   = (D - 2.0f*da) / TRAJ_VMAX;
    }
    t->ttotal = 2.0f*t->ta + t->tc;
    t->t0     = HAL_GetTick();
    t->active = 1;
}

static void traj_eval(Traj *t, float *refDeg, float *refVel)
{
    if (!t->active) { *refDeg = t->endDeg; *refVel = 0.0f; return; }

    float tt = (float)(HAL_GetTick() - t->t0) * 0.001f;   // s
    float pos, vel;

    if (tt >= t->ttotal) { *refDeg = t->endDeg; *refVel = 0.0f; t->active = 0; return; }

    if (tt < t->ta) {                          // acelerando
        vel = t->amax * tt;
        pos = 0.5f * t->amax * tt * tt;
    } else if (tt < t->ta + t->tc) {           // crucero
        float tcr = tt - t->ta;
        vel = t->vmax;
        pos = 0.5f*t->amax*t->ta*t->ta + t->vmax*tcr;
    } else {                                   // desacelerando
        float td = tt - t->ta - t->tc;
        vel = t->vmax - t->amax*td;
        pos = 0.5f*t->amax*t->ta*t->ta + t->vmax*t->tc
            + t->vmax*td - 0.5f*t->amax*td*td;
    }
    *refDeg = t->startDeg + t->dir * pos;
    *refVel = t->dir * vel;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
      // ===== Encoders =====
      HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // Indice MCP
      HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // Indice PIP
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // Pulgar ABD
      HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Pulgar MCP
      HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL); // Medio PIP

      // ===== PWM =====
      HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); // Indice MCP
      HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1); // Indice PIP
      HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // Pulgar ABD
      HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // Pulgar MCP
      HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1); // Medio MCP
      HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); // Medio PIP
      HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // Anular MCP
      HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); // Anular PIP
      HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); // Menique MCP
      HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2); // Menique PIP

      // ===== Stop todos =====
      INDICE_MCP_STOP(); INDICE_PIP_STOP();
      PULGAR_ABD_STOP(); PULGAR_MCP_STOP();
      MEDIO_MCP_STOP();  MEDIO_PIP_STOP();
      ANULAR_MCP_STOP(); ANULAR_PIP_STOP();
      MENIQUE_MCP_STOP(); MENIQUE_PIP_STOP();

      // ===== Leer contadores iniciales =====
      lastCount_i_mcp = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
      lastCount_i_pip = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
      lastCount_p_abd = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
      lastCount_p_mcp = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
      lastCount_m_pip = (int32_t)__HAL_TIM_GET_COUNTER(&htim8);

      lastTime = HAL_GetTick();

      const char *banner =
                "=== Control Mano 5 Dedos ===\r\n"
                "Comandos: home | i_mcp,<a> | i_pip,<a> | p_abd,<a> | p_mcp,<a> | m_mcp,<a> | m_pip,<a> | a_mcp,<a> | a_pip,<a> | q_mcp,<a> | q_pip,<a> | ik_i,<x>,<y>,<z> | ik_p,<x>,<y>,<z> | agarre_lateral | reset\r\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)banner, strlen(banner), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
                  while (1)
                            {
                                // ================= UART =================
                                if (HAL_UART_Receive(&huart3, &rx_byte, 1, 0) == HAL_OK)
                                {
                                    if (rx_byte == '\n')
                                    {
                                        rx_buffer[rx_index] = '\0';
                                        rx_index = 0;
                                        char *buf = (char*)rx_buffer;

                                        // ---- home ----
                                        if (strncmp(buf, "home", 4) == 0)
                                        {
                                            rx_index = 0;
                                            INDICE_MCP_STOP(); INDICE_PIP_STOP();
                                            PULGAR_ABD_STOP(); PULGAR_MCP_STOP();
                                            MEDIO_MCP_STOP();  MEDIO_PIP_STOP();
                                            ANULAR_MCP_STOP(); ANULAR_PIP_STOP();
                                            MENIQUE_MCP_STOP(); MENIQUE_PIP_STOP();

                                            running_i_mcp = 0; stopped_i_mcp = 0;
                                            running_i_pip = 0; stopped_i_pip = 0;
                                            running_p_abd = 0; stopped_p_abd = 0;
                                            running_p_mcp = 0; stopped_p_mcp = 0;
                                            running_m_mcp = 0; stopped_m_mcp = 0;
                                            running_m_pip = 0; stopped_m_pip = 0;
                                            running_a_mcp = 0; stopped_a_mcp = 0;
                                            running_a_pip = 0; stopped_a_pip = 0;
                                            running_q_mcp = 0; stopped_q_mcp = 0;
                                            running_q_pip = 0; stopped_q_pip = 0;


                                            ikp_seq_active = 0;
                                            ikp_seq_stage  = 0;
                                            grasp_lat_active = 0;   // <-- cancelar secuencia agarre lateral
                                            grasp_lat_stage  = 0;
                                            grasp_pin_active = 0;   // <-- cancelar secuencia agarre pinza
                                            grasp_pin_stage  = 0;
                                            grasp_cil_active = 0;   // <-- cancelar secuencia agarre cilindrico
                                            grasp_cil_stage  = 0;
                                            grasp_cil2_active = 0;
                                            grasp_cil2_stage  = 0;

                                            fsr_indice_blocked = 0;
                                            fsr_pulgar_blocked = 0;

                                            homing_i          = 1;
                                            homing_i_stage    = 0;
                                            homingStallInit_i = 0;
                                            homing_p          = 0;
                                            homing_m          = 0;
                                            homing_a          = 0;
                                            homing_q          = 0;

                                            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, (uint32_t)deadzone_pwm_homing);
                                            INDICE_MCP_BACKWARD();

                                            char out[] = "HOMING iniciado...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                        // ---- i_mcp,<ang> ----
                                        else if (strncmp(buf, "i_mcp,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            tgtDeg_i_mcp = v;
                                            tgtRad_i_mcp = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_i_mcp, curDeg_i_mcp, v);
                                            stopped_i_mcp = 0; running_i_mcp = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "I_MCP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- i_pip,<ang> ----
                                        else if (strncmp(buf, "i_pip,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            tgtDeg_i_pip = v;
                                            tgtRad_i_pip = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_i_pip, curDeg_i_pip, v);
                                            stopped_i_pip = 0; running_i_pip = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "I_PIP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- p_abd,<ang> ----
                                        else if (strncmp(buf, "p_abd,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            ikp_seq_active = 0;
                                            tgtDeg_p_abd = v;
                                            tgtRad_p_abd = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_p_abd, curDeg_p_abd, v);
                                            stopped_p_abd = 0; running_p_abd = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "P_ABD -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- p_mcp,<ang> ----
                                        else if (strncmp(buf, "p_mcp,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            ikp_seq_active = 0;
                                            tgtDeg_p_mcp = v;
                                            tgtRad_p_mcp = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_p_mcp, curDeg_p_mcp, v);
                                            stopped_p_mcp = 0; running_p_mcp = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "P_MCP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- m_mcp,<ang> ----  (Dedo medio MCP)
                                        else if (strncmp(buf, "m_mcp,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            tgtDeg_m_mcp = v;
                                            tgtRad_m_mcp = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_m_mcp, curDeg_m_mcp, v);
                                            stopped_m_mcp = 0; running_m_mcp = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "M_MCP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- m_pip,<ang> ----  (Dedo medio PIP)
                                        else if (strncmp(buf, "m_pip,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            tgtDeg_m_pip = v;
                                            tgtRad_m_pip = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_m_pip, curDeg_m_pip, v);
                                            stopped_m_pip = 0; running_m_pip = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "M_PIP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- a_mcp,<ang> ----  (Dedo anular MCP)
                                        else if (strncmp(buf, "a_mcp,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            tgtDeg_a_mcp = v;
                                            tgtRad_a_mcp = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_a_mcp, curDeg_a_mcp, v);
                                            stopped_a_mcp = 0; running_a_mcp = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "A_MCP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- a_pip,<ang> ----  (Dedo anular PIP)
                                        else if (strncmp(buf, "a_pip,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            tgtDeg_a_pip = v;
                                            tgtRad_a_pip = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_a_pip, curDeg_a_pip, v);
                                            stopped_a_pip = 0; running_a_pip = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "A_PIP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- q_mcp,<ang> ----  (Dedo menique MCP)
                                        else if (strncmp(buf, "q_mcp,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            tgtDeg_q_mcp = v;
                                            tgtRad_q_mcp = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_q_mcp, curDeg_q_mcp, v);
                                            stopped_q_mcp = 0; running_q_mcp = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "Q_MCP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- q_pip,<ang> ----  (Dedo menique PIP)
                                        else if (strncmp(buf, "q_pip,", 6) == 0)
                                        {
                                            float v = atof(buf + 6);
                                            if (v < 0.0f)  v = 0.0f;
                                            if (v > 90.0f) v = 90.0f;
                                            tgtDeg_q_pip = v;
                                            tgtRad_q_pip = v * (float)M_PI / 180.0f;
                                            traj_plan(&traj_q_pip, curDeg_q_pip, v);
                                            stopped_q_pip = 0; running_q_pip = 1;
                                            startTime = HAL_GetTick();
                                            char out[40];
                                            int len = snprintf(out, sizeof(out), "Q_PIP -> %.1f deg\r\n", v);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- ik_i,<x>,<y>,<z> -> Indice ----
                                        else if (strncmp(buf, "ik_i,", 5) == 0)
                                        {
                                            float px = 0, py = 0, pz = 0;
                                            char *p = buf + 5;
                                            px = atof(p); p = strchr(p,','); if(p) p++;
                                            py = atof(p); p = strchr(p,','); if(p) p++;
                                            pz = atof(p);

                                            const float L0=27.0f, L1=55.0f, L2=38.16f, L3=34.0f;
                                            const float offset=33.3f;
                                            float dx = px - offset;
                                            float dy = py;
                                            float r  = sqrtf(dx*dx + dy*dy);
                                            float z  = pz;
                                            float th2 = 20.0f, th3 = 30.0f;
                                            float alpha = 1.0f, lambda = 0.01f;
                                            int   iter;
                                            for (iter = 0; iter < 200; iter++)
                                            {
                                                float th4 = (-0.00000016f)*th3*th3*th3*th3
                                                          + (0.00006721f)*th3*th3*th3
                                                          + (-0.01062799f)*th3*th3
                                                          + (1.68304757f)*th3
                                                          + (0.04984945f);
                                                if (th4 < 0.0f) th4 = 0.0f;
                                                if (th4 > 90.0f) th4 = 90.0f;
                                                float phi2 = th2*(float)M_PI/180.0f;
                                                float phi3 = (th2+th3)*(float)M_PI/180.0f;
                                                float phi4 = (th2+th3+th4)*(float)M_PI/180.0f;
                                                float pr = L0 + L1*cosf(phi2) + L2*cosf(phi3) + L3*cosf(phi4);
                                                float pz2=      L1*sinf(phi2) + L2*sinf(phi3) + L3*sinf(phi4);
                                                float er = r - pr;
                                                float ez = z - pz2;
                                                float err = sqrtf(er*er + ez*ez);
                                                if (err < 0.1f) break;
                                                float dth4_dth3 = 4*(-0.00000016f)*th3*th3*th3
                                                                + 3*(0.00006721f)*th3*th3
                                                                + 2*(-0.01062799f)*th3
                                                                + 1.68304757f;
                                                float k = (float)M_PI/180.0f;
                                                float dphi4_dth2 = 1.0f;
                                                float dphi4_dth3 = 1.0f + dth4_dth3;
                                                float J11 = k*(-sinf(phi2)*L1 - sinf(phi3)*L2 - sinf(phi4)*dphi4_dth2*L3);
                                                float J21 = k*( cosf(phi2)*L1 + cosf(phi3)*L2 + cosf(phi4)*dphi4_dth2*L3);
                                                float J12 = k*(-sinf(phi3)*L2 - sinf(phi4)*dphi4_dth3*L3);
                                                float J22 = k*( cosf(phi3)*L2 + cosf(phi4)*dphi4_dth3*L3);
                                                float JtJ11 = J11*J11 + J21*J21 + lambda*lambda;
                                                float JtJ12 = J11*J12 + J21*J22;
                                                float JtJ22 = J12*J12 + J22*J22 + lambda*lambda;
                                                float Jte1  = J11*er + J21*ez;
                                                float Jte2  = J12*er + J22*ez;
                                                float det   = JtJ11*JtJ22 - JtJ12*JtJ12;
                                                if (fabsf(det) < 1e-9f) break;
                                                float dth2 = alpha*(JtJ22*Jte1 - JtJ12*Jte2)/det;
                                                float dth3 = alpha*(-JtJ12*Jte1 + JtJ11*Jte2)/det;
                                                th2 += dth2; th3 += dth3;
                                                if (th2 < 0.0f) th2=0.0f; if (th2>90.0f) th2=90.0f;
                                                if (th3 < 0.0f) th3=0.0f; if (th3>90.0f) th3=90.0f;
                                            }

                                            tgtDeg_i_mcp = th2;
                                            tgtRad_i_mcp = th2 * (float)M_PI / 180.0f;

                                            tgtDeg_i_pip = th3;
                                            tgtRad_i_pip = th3 * (float)M_PI / 180.0f;

                                            stopped_i_mcp = 0;
                                            running_i_mcp = 1;

                                            stopped_i_pip = 0;
                                            running_i_pip = 1;

                                            startTime = HAL_GetTick();

                                            char out[60];
                                            int len = snprintf(out, sizeof(out), "IK_INDICE -> MCP:%.1f PIP:%.1f\r\n", th2, th3);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- ik_p,<x>,<y>,<z> -> Pulgar (modelo workspace corregido) ----
                                        else if (strncmp(buf, "ik_p,", 5) == 0)
                                        {
                                            float px = 0, py = 0, pz = 0;
                                            char *p = buf + 5;
                                            px = atof(p); p = strchr(p,','); if(p) p++;
                                            py = atof(p); p = strchr(p,','); if(p) p++;
                                            pz = atof(p);

                                            const float L0=45.51f, L1=33.21f, L2=22.0f;
                                            const float Ltot = L0 + L1 + L2;
                                            const float pb_x=26.127f, pb_y=-73.939f;
                                            const float tilt=50.0f;

                                            float ct = cosf(tilt*(float)M_PI/180.0f);
                                            float st = sinf(tilt*(float)M_PI/180.0f);
                                            float dx = px - pb_x;
                                            float dy = py - pb_y;
                                            float dz = pz - 0.0f;

                                            float dl_x = st*dx - ct*dy;
                                            float dl_y = ct*dx + st*dy;
                                            float dl_z = dz;

                                            float s1 = dl_z / Ltot;
                                            if (s1 >  1.0f) s1 =  1.0f;
                                            if (s1 < -1.0f) s1 = -1.0f;
                                            float th1_p = asinf(s1) * 180.0f/(float)M_PI;
                                            if (th1_p < 0.0f)  th1_p = 0.0f;
                                            if (th1_p > 90.0f) th1_p = 90.0f;

                                            float c1 = cosf(th1_p*(float)M_PI/180.0f);
                                            if (c1 < 1e-6f) c1 = 1e-6f;

                                            float r_t =  dl_y / c1;
                                            float h_t = -dl_x / c1;

                                            float th2 = 20.0f;
                                            float alpha = 1.0f, lambda = 0.01f;
                                            int   iter;
                                            for (iter = 0; iter < 200; iter++)
                                            {
                                                float th3 = (0.00000075f)*th2*th2*th2*th2
                                                          + (-0.00008762f)*th2*th2*th2
                                                          + (0.00106596f)*th2*th2
                                                          + (1.27148703f)*th2
                                                          + (0.14418172f);
                                                if (th3 < 0.0f)  th3 = 0.0f;
                                                if (th3 > 90.0f) th3 = 90.0f;
                                                float phi2 = th2*(float)M_PI/180.0f;
                                                float phi3 = (th2+th3)*(float)M_PI/180.0f;
                                                float pr = L0 + L1*cosf(phi2) + L2*cosf(phi3);
                                                float ph =      L1*sinf(phi2) + L2*sinf(phi3);
                                                float er = r_t - pr;
                                                float ez = h_t - ph;
                                                float err = sqrtf(er*er + ez*ez);
                                                if (err < 0.1f) break;
                                                float dth3_dth2 = 4*(0.00000075f)*th2*th2*th2
                                                                + 3*(-0.00008762f)*th2*th2
                                                                + 2*(0.00106596f)*th2
                                                                + 1.27148703f;
                                                float k = (float)M_PI/180.0f;
                                                float dphi3_dth2 = 1.0f + dth3_dth2;
                                                float Jr = k*(-sinf(phi2)*L1 - sinf(phi3)*dphi3_dth2*L2);
                                                float Jz = k*( cosf(phi2)*L1 + cosf(phi3)*dphi3_dth2*L2);
                                                float JtJ = Jr*Jr + Jz*Jz + lambda*lambda;
                                                float Jte = Jr*er + Jz*ez;
                                                float dth2s = alpha*Jte/JtJ;
                                                th2 += dth2s;
                                                if (th2 < 0.0f)  th2 = 0.0f;
                                                if (th2 > 90.0f) th2 = 90.0f;
                                            }

                                            tgtDeg_p_abd = th1_p;
                                            tgtRad_p_abd = th1_p * (float)M_PI / 180.0f;
                                            stopped_p_abd = 0;
                                            running_p_abd = 1;

                                            ikp_target_mcp = th2;
                                            ikp_seq_active = 1;
                                            ikp_seq_stage  = 0;

                                            running_p_mcp = 0;
                                            stopped_p_mcp = 1;
                                            PULGAR_MCP_STOP();

                                            startTime = HAL_GetTick();

                                            char out[70];
                                            int len = snprintf(out, sizeof(out), "IK_PULGAR -> ABD:%.1f (MCP:%.1f pendiente)\r\n", th1_p, th2);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- agarre_lateral ----  (INDICE -> ABD -> MCP, escalonado)
                                        else if (strncmp(buf, "agarre_lateral", 14) == 0)
                                        {
                                            float abd_p = calcular_abd_pulgar_lateral();

                                            // Arranca limpio (descarta bloqueos FSR de un agarre anterior)
                                            fsr_indice_blocked = 0;
                                            fsr_pulgar_blocked = 0;

                                            // 1) Primero SOLO el indice (MCP + PIP)
                                            tgtDeg_i_mcp = GRASP_I_MCP;
                                            tgtRad_i_mcp = GRASP_I_MCP * (float)M_PI / 180.0f;
                                            stopped_i_mcp = 0;
                                            running_i_mcp = 1;

                                            tgtDeg_i_pip = GRASP_I_PIP;
                                            tgtRad_i_pip = GRASP_I_PIP * (float)M_PI / 180.0f;
                                            stopped_i_pip = 0;
                                            running_i_pip = 1;

                                            // El pulgar queda QUIETO hasta que el indice llegue
                                            running_p_abd = 0; stopped_p_abd = 1; PULGAR_ABD_STOP();
                                            running_p_mcp = 0; stopped_p_mcp = 1; PULGAR_MCP_STOP();

                                            // Desactiva la secuencia antigua de IK pulgar
                                            ikp_seq_active = 0;
                                            ikp_seq_stage  = 0;

                                            // Activa la secuencia escalonada del agarre lateral
                                            grasp_lat_abd    = abd_p;
                                            grasp_lat_mcp    = GRASP_P_MCP;
                                            grasp_lat_active = 1;
                                            grasp_lat_stage  = 0;   // esperando que termine el indice

                                            startTime = HAL_GetTick();

                                            char out[90];
                                            int len = snprintf(out, sizeof(out),
                                                "AGARRE_LATERAL -> INDICE primero (ABD:%.1f MCP:%.1f pendientes)\r\n",
                                                abd_p, GRASP_P_MCP);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- agarre_pinza ----  (ABD -> MCP pulgar -> INDICE, escalonado)
                                        else if (strncmp(buf, "agarre_pinza", 12) == 0)
                                        {
                                            // Arranca limpio (descarta bloqueos FSR de un agarre anterior)
                                            fsr_indice_blocked = 0;
                                            fsr_pulgar_blocked = 0;

                                            // 1) Primero SOLO el ABD del pulgar
                                            tgtDeg_p_abd = GRASP_PINZA_P_ABD;
                                            tgtRad_p_abd = GRASP_PINZA_P_ABD * (float)M_PI / 180.0f;
                                            stopped_p_abd = 0;
                                            running_p_abd = 1;

                                            // MCP pulgar e indice quietos hasta su turno
                                            running_p_mcp = 0; stopped_p_mcp = 1; PULGAR_MCP_STOP();
                                            running_i_mcp = 0; stopped_i_mcp = 1; INDICE_MCP_STOP();
                                            running_i_pip = 0; stopped_i_pip = 1; INDICE_PIP_STOP();

                                            // Desactiva otras secuencias
                                            ikp_seq_active   = 0; ikp_seq_stage   = 0;
                                            grasp_lat_active = 0; grasp_lat_stage = 0;

                                            // Activa la secuencia de pinza
                                            grasp_pin_active = 1;
                                            grasp_pin_stage  = 0;   // esperando que termine el ABD

                                            startTime = HAL_GetTick();

                                            char out[100];
                                            int len = snprintf(out, sizeof(out),
                                                "AGARRE_PINZA -> ABD pulgar primero (MCP_p:%.1f I_MCP:%.1f I_PIP:%.1f pendientes)\r\n",
                                                GRASP_PINZA_P_MCP, GRASP_PINZA_I_MCP, GRASP_PINZA_I_PIP);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        else if (strncmp(buf, "agarre_cilindrico_2", 19) == 0)
                                        {
                                            fsr_indice_blocked = 0;
                                            fsr_pulgar_blocked = 0;

                                            // 1) SOLO el ABD del pulgar (manual)
                                            tgtDeg_p_abd = cil2_abd_pulgar;
                                            tgtRad_p_abd = cil2_abd_pulgar * (float)M_PI / 180.0f;
                                            traj_plan(&traj_p_abd, curDeg_p_abd, cil2_abd_pulgar);
                                            stopped_p_abd = 0; running_p_abd = 1;

                                            // Todo lo demas quieto (copia el mismo bloque de STOP del agarre 1)
                                            running_p_mcp = 0; stopped_p_mcp = 1; PULGAR_MCP_STOP();
                                            running_i_mcp = 0; stopped_i_mcp = 1; INDICE_MCP_STOP();
                                            running_i_pip = 0; stopped_i_pip = 1; INDICE_PIP_STOP();
                                            running_m_mcp = 0; stopped_m_mcp = 1; MEDIO_MCP_STOP();
                                            running_m_pip = 0; stopped_m_pip = 1; MEDIO_PIP_STOP();
                                            running_a_mcp = 0; stopped_a_mcp = 1; ANULAR_MCP_STOP();
                                            running_a_pip = 0; stopped_a_pip = 1; ANULAR_PIP_STOP();
                                            running_q_mcp = 0; stopped_q_mcp = 1; MENIQUE_MCP_STOP();
                                            running_q_pip = 0; stopped_q_pip = 1; MENIQUE_PIP_STOP();

                                            // Cancela las otras secuencias (incluida grasp_cil)
                                            ikp_seq_active   = 0; ikp_seq_stage   = 0;
                                            grasp_lat_active = 0; grasp_lat_stage = 0;
                                            grasp_pin_active = 0; grasp_pin_stage = 0;
                                            grasp_cil_active = 0; grasp_cil_stage = 0;

                                            grasp_cil2_active = 1;
                                            grasp_cil2_stage  = 0;
                                            startTime = HAL_GetTick();

                                            char out[] = "AGARRE_CILINDRICO_2 -> ABD pulgar...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                        // ---- agarre_cilindrico,<R> ----  (ABD -> MCP indice+pulgar+seguidores -> PIP; angulos POR DEDO)
                                        else if (strncmp(buf, "agarre_cilindrico", 17) == 0)
                                        {
                                            float Rc = 25.0f;                       // radio por defecto si no mandan argumento
                                            char *cq = strchr(buf, ',');
                                            if (cq) Rc = atof(cq + 1);
                                            cyl_compute_targets(Rc, &cil_tg);

                                            fsr_indice_blocked = 0;
                                            fsr_pulgar_blocked = 0;

                                            // 1) Primero SOLO el ABD del pulgar (manual)
                                            tgtDeg_p_abd = cil_abd_pulgar;
                                            tgtRad_p_abd = cil_abd_pulgar * (float)M_PI / 180.0f;
                                            traj_plan(&traj_p_abd, curDeg_p_abd, cil_abd_pulgar);
                                            stopped_p_abd = 0; running_p_abd = 1;

                                            // Todo lo demas quieto hasta su turno
                                            running_p_mcp = 0; stopped_p_mcp = 1; PULGAR_MCP_STOP();
                                            running_i_mcp = 0; stopped_i_mcp = 1; INDICE_MCP_STOP();
                                            running_i_pip = 0; stopped_i_pip = 1; INDICE_PIP_STOP();
                                            running_m_mcp = 0; stopped_m_mcp = 1; MEDIO_MCP_STOP();
                                            running_m_pip = 0; stopped_m_pip = 1; MEDIO_PIP_STOP();
                                            running_a_mcp = 0; stopped_a_mcp = 1; ANULAR_MCP_STOP();
                                            running_a_pip = 0; stopped_a_pip = 1; ANULAR_PIP_STOP();
                                            running_q_mcp = 0; stopped_q_mcp = 1; MENIQUE_MCP_STOP();
                                            running_q_pip = 0; stopped_q_pip = 1; MENIQUE_PIP_STOP();

                                            // Desactiva otras secuencias
                                            ikp_seq_active   = 0; ikp_seq_stage   = 0;
                                            grasp_lat_active = 0; grasp_lat_stage = 0;
                                            grasp_pin_active = 0; grasp_pin_stage = 0;

                                            // Activa la secuencia cilindrica
                                            grasp_cil_active = 1;
                                            grasp_cil_stage  = 0;   // esperando que termine el ABD

                                            startTime = HAL_GetTick();

                                            char out[120];
                                            int len = snprintf(out, sizeof(out),
                                                "AGARRE_CILINDRICO R=%.1f -> ABD:%.1f MCP_p:%.1f (I_MCP:%.1f I_PIP:%.1f)\r\n",
                                                Rc, cil_abd_pulgar, GRASP_CIL_MCP_P, cil_tg.i_mcp, cil_pip_indice);
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, len, HAL_MAX_DELAY);
                                        }
                                        // ---- reset ----
                                        else if (strncmp(buf, "reset", 5) == 0)
                                        {
                                            INDICE_MCP_STOP(); INDICE_PIP_STOP();
                                            PULGAR_ABD_STOP(); PULGAR_MCP_STOP();
                                            MEDIO_MCP_STOP();  MEDIO_PIP_STOP();
                                            ANULAR_MCP_STOP(); ANULAR_PIP_STOP();
                                            MENIQUE_MCP_STOP(); MENIQUE_PIP_STOP();

                                            encCount_i_mcp=0; lastCount_i_mcp=(int32_t)__HAL_TIM_GET_COUNTER(&htim1);
                                            encCount_i_pip=0; lastCount_i_pip=(int32_t)__HAL_TIM_GET_COUNTER(&htim2);
                                            encCount_p_abd=0; lastCount_p_abd=(int32_t)__HAL_TIM_GET_COUNTER(&htim3);
                                            encCount_p_mcp=0; lastCount_p_mcp=(int32_t)__HAL_TIM_GET_COUNTER(&htim4);
                                            encCount_m_pip=0; lastCount_m_pip=(int32_t)__HAL_TIM_GET_COUNTER(&htim8);
                                            __disable_irq();
                                            encCount_m_mcp=0;
                                            encCount_a_mcp=0; encCount_a_pip=0;
                                            encCount_q_mcp=0; encCount_q_pip=0;
                                            __enable_irq();

                                            curDeg_i_mcp=0; lastDeg_i_mcp=0; vel_i_mcp=0;
                                            curDeg_i_pip=0; lastDeg_i_pip=0; vel_i_pip=0;
                                            curDeg_p_abd=0; lastDeg_p_abd=0; vel_p_abd=0;
                                            curDeg_p_mcp=0; lastDeg_p_mcp=0; vel_p_mcp=0;
                                            curDeg_m_mcp=0; lastDeg_m_mcp=0; vel_m_mcp=0;
                                            curDeg_m_pip=0; lastDeg_m_pip=0; vel_m_pip=0;
                                            curDeg_a_mcp=0; lastDeg_a_mcp=0; vel_a_mcp=0;
                                            curDeg_a_pip=0; lastDeg_a_pip=0; vel_a_pip=0;
                                            curDeg_q_mcp=0; lastDeg_q_mcp=0; vel_q_mcp=0;
                                            curDeg_q_pip=0; lastDeg_q_pip=0; vel_q_pip=0;

                                            tgtDeg_i_mcp=0; tgtRad_i_mcp=0;
                                            tgtDeg_i_pip=0; tgtRad_i_pip=0;
                                            tgtDeg_p_abd=0; tgtRad_p_abd=0;
                                            tgtDeg_p_mcp=0; tgtRad_p_mcp=0;
                                            tgtDeg_m_mcp=0; tgtRad_m_mcp=0;
                                            tgtDeg_m_pip=0; tgtRad_m_pip=0;
                                            tgtDeg_a_mcp=0; tgtRad_a_mcp=0;
                                            tgtDeg_a_pip=0; tgtRad_a_pip=0;
                                            tgtDeg_q_mcp=0; tgtRad_q_mcp=0;
                                            tgtDeg_q_pip=0; tgtRad_q_pip=0;

                                            running_i_mcp=0; stopped_i_mcp=0;
                                            running_i_pip=0; stopped_i_pip=0;
                                            running_p_abd=0; stopped_p_abd=0;
                                            running_p_mcp=0; stopped_p_mcp=0;
                                            running_m_mcp=0; stopped_m_mcp=0;
                                            running_m_pip=0; stopped_m_pip=0;
                                            running_a_mcp=0; stopped_a_mcp=0;
                                            running_a_pip=0; stopped_a_pip=0;
                                            running_q_mcp=0; stopped_q_mcp=0;
                                            running_q_pip=0; stopped_q_pip=0;

                                            ikp_seq_active = 0;
                                            ikp_seq_stage  = 0;
                                            grasp_lat_active = 0;   // <-- cancelar secuencia agarre lateral
                                            grasp_lat_stage  = 0;
                                            grasp_pin_active = 0;   // <-- cancelar secuencia agarre pinza
                                            grasp_pin_stage  = 0;
                                            grasp_cil_active = 0;   // <-- cancelar secuencia agarre cilindrico
                                            grasp_cil_stage  = 0;
                                            grasp_cil2_active = 0;
                                            grasp_cil2_stage  = 0;

                                            fsr_indice_blocked = 0;
                                            fsr_pulgar_blocked = 0;

                                            homing_i=0; homing_p=0; homing_m=0; homing_a=0; homing_q=0;

                                            char out[] = "RESET OK\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                        else
                                        {
                                            char e[] = "ERROR: comando invalido\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)e, strlen(e), HAL_MAX_DELAY);
                                        }
                                    }
                                    else if (rx_byte != '\r')
                                    {
                                        if (rx_index < sizeof(rx_buffer) - 1)
                                            rx_buffer[rx_index++] = rx_byte;
                                    }
                                }
                                else
                                {
                                    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE))
                                    {
                                        __HAL_UART_CLEAR_OREFLAG(&huart3);
                                        rx_index = 0;
                                    }
                                }
                                // ================= TEMPORIZACION 20 ms =================
                                uint32_t now = HAL_GetTick();
                                if ((now - lastTime) < 20)
                                    continue;

                                const float dt = 0.02f;
                                lastTime = now;

                                // ================= LECTURA FSR =================
                                HAL_ADC_Start(&hadc3);
                                if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)
                                    fsr_pulgar_val = HAL_ADC_GetValue(&hadc3);
                                HAL_ADC_Stop(&hadc3);

                                HAL_ADC_Start(&hadc2);
                                if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
                                    fsr_indice_val = HAL_ADC_GetValue(&hadc2);
                                HAL_ADC_Stop(&hadc2);

                                if (fsr_indice_val > FSR_UMBRAL) fsr_indice_blocked = 1;
                                if (fsr_pulgar_val > FSR_UMBRAL) fsr_pulgar_blocked = 1;

                                // ================= ENCODERS =================
                                const float dps = 360.0f / (float)PPR_SALIDA;

                                #define READ_ENC(htim, encCount, lastCount, curDeg, lastDeg, vel) do { \
                                    int32_t _raw = (int32_t)__HAL_TIM_GET_COUNTER(&(htim));            \
                                    int32_t _d   = _raw - (lastCount);                                  \
                                    if (_d >  32767) _d -= 65536;                                       \
                                    if (_d < -32768) _d += 65536;                                       \
                                    (lastCount)  = _raw;                                                \
                                    (encCount)  += _d;                                                  \
                                    (curDeg)     = (encCount) * dps;                                    \
                                    (vel) = 0.7f*(vel) + 0.3f*((curDeg)-(lastDeg))/dt;                 \
                                    (lastDeg) = (curDeg);                                               \
                                } while(0)

                                READ_ENC(htim1, encCount_i_mcp, lastCount_i_mcp, curDeg_i_mcp, lastDeg_i_mcp, vel_i_mcp);
                                READ_ENC(htim2, encCount_i_pip, lastCount_i_pip, curDeg_i_pip, lastDeg_i_pip, vel_i_pip);
                                READ_ENC(htim3, encCount_p_abd, lastCount_p_abd, curDeg_p_abd, lastDeg_p_abd, vel_p_abd);
                                READ_ENC(htim4, encCount_p_mcp, lastCount_p_mcp, curDeg_p_mcp, lastDeg_p_mcp, vel_p_mcp);
                                READ_ENC(htim8, encCount_m_pip, lastCount_m_pip, curDeg_m_pip, lastDeg_m_pip, vel_m_pip);

                                // ===== Encoders por IRQ (sin wrap de 16 bits) =====
                                __disable_irq();
                                int32_t snap_a_mcp = encCount_a_mcp;   // anular MCP
                                int32_t snap_a_pip = encCount_a_pip;   // anular PIP
                                int32_t snap_m_mcp = encCount_m_mcp;   // medio MCP
                                int32_t snap_q_mcp = encCount_q_mcp;   // menique MCP
                                int32_t snap_q_pip = encCount_q_pip;   // menique PIP
                                __enable_irq();

                                curDeg_a_mcp = (float)snap_a_mcp * dps;
                                vel_a_mcp = 0.7f*vel_a_mcp + 0.3f*((curDeg_a_mcp - lastDeg_a_mcp)/dt);
                                lastDeg_a_mcp = curDeg_a_mcp;

                                curDeg_a_pip = (float)snap_a_pip * dps;
                                vel_a_pip = 0.7f*vel_a_pip + 0.3f*((curDeg_a_pip - lastDeg_a_pip)/dt);
                                lastDeg_a_pip = curDeg_a_pip;

                                curDeg_m_mcp = (float)snap_m_mcp * dps;
                                vel_m_mcp = 0.7f*vel_m_mcp + 0.3f*((curDeg_m_mcp - lastDeg_m_mcp)/dt);
                                lastDeg_m_mcp = curDeg_m_mcp;

                                curDeg_q_mcp = (float)snap_q_mcp * dps;
                                vel_q_mcp = 0.7f*vel_q_mcp + 0.3f*((curDeg_q_mcp - lastDeg_q_mcp)/dt);
                                lastDeg_q_mcp = curDeg_q_mcp;

                                curDeg_q_pip = (float)snap_q_pip * dps;
                                vel_q_pip = 0.7f*vel_q_pip + 0.3f*((curDeg_q_pip - lastDeg_q_pip)/dt);
                                lastDeg_q_pip = curDeg_q_pip;

                                // ================= HOMING INDICE =================
                                if (homing_i)
                                {
                                    if (homing_i_stage == 0) // MCP
                                    {
                                        if (!homingStallInit_i)
                                        {
                                            homingLastCount_i = encCount_i_mcp;
                                            homingStallTime_i = now;
                                            homingStallInit_i = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = encCount_i_mcp - homingLastCount_i;
                                            homingLastCount_i = encCount_i_mcp;
                                            if (abs(moved) > 2) homingStallTime_i = now;
                                            if ((now - homingStallTime_i) >= HOMING_STALL_MS_I_MCP)
                                            {
                                                INDICE_MCP_STOP();
                                                encCount_i_mcp=0; lastCount_i_mcp=(int32_t)__HAL_TIM_GET_COUNTER(&htim1);
                                                curDeg_i_mcp=0.0f; lastDeg_i_mcp=0.0f; vel_i_mcp=0;
                                                homing_i_stage    = 1;
                                                homingStallInit_i = 0;

                                                __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, (uint32_t)deadzone_pwm_homing);
                                                INDICE_PIP_BACKWARD();
                                            }
                                        }
                                    }
                                    else // PIP
                                    {
                                        if (!homingStallInit_i)
                                        {
                                            homingLastCount_i = encCount_i_pip;
                                            homingStallTime_i = now;
                                            homingStallInit_i = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = encCount_i_pip - homingLastCount_i;
                                            homingLastCount_i = encCount_i_pip;
                                            if (abs(moved) > 2) homingStallTime_i = now;
                                            if ((now - homingStallTime_i) >= HOMING_STALL_MS_I_PIP)
                                            {
                                                INDICE_PIP_STOP();
                                                encCount_i_pip=0; lastCount_i_pip=(int32_t)__HAL_TIM_GET_COUNTER(&htim2);
                                                curDeg_i_pip=0.0f; lastDeg_i_pip=0.0f; vel_i_pip=0;
                                                homing_i       = 0;
                                                homing_p       = 1;
                                                homing_p_stage = 0;
                                                homingStallInit_p = 0;

                                                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (uint32_t)deadzone_pwm_homing_pulgar);
                                                PULGAR_MCP_BACKWARD();

                                                char out[] = "HOME INDICE OK - iniciando PULGAR...\r\n";
                                                HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                            }
                                        }
                                    }
                                    goto log_section;
                                }

                                // ================= HOMING PULGAR =================
                                if (homing_p)
                                {
                                    if (homing_p_stage == 0) // MCP
                                    {
                                        if (!homingStallInit_p)
                                        {
                                            homingLastCount_p = encCount_p_mcp;
                                            homingStallTime_p = now;
                                            homingStallInit_p = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = encCount_p_mcp - homingLastCount_p;
                                            homingLastCount_p = encCount_p_mcp;
                                            if (abs(moved) > 2) homingStallTime_p = now;
                                            if ((now - homingStallTime_p) >= HOMING_STALL_MS_P_MCP)
                                            {
                                                PULGAR_MCP_STOP();
                                                encCount_p_mcp=0; lastCount_p_mcp=(int32_t)__HAL_TIM_GET_COUNTER(&htim4);
                                                curDeg_p_mcp=0.0f; lastDeg_p_mcp=0.0f; vel_p_mcp=0;
                                                homing_p_stage    = 1;
                                                homingStallInit_p = 0;

                                                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, (uint32_t)deadzone_pwm_homing_pulgar);
                                                PULGAR_ABD_BACKWARD();
                                            }
                                        }
                                    }
                                    else // ABD
                                    {
                                        if (!homingStallInit_p)
                                        {
                                            homingLastCount_p = encCount_p_abd;
                                            homingStallTime_p = now;
                                            homingStallInit_p = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = encCount_p_abd - homingLastCount_p;
                                            homingLastCount_p = encCount_p_abd;
                                            if (abs(moved) > 2) homingStallTime_p = now;
                                            if ((now - homingStallTime_p) >= HOMING_STALL_MS_P_ABD)
                                            {
                                                PULGAR_ABD_STOP();
                                                encCount_p_abd=0; lastCount_p_abd=(int32_t)__HAL_TIM_GET_COUNTER(&htim3);
                                                curDeg_p_abd=0.0f; lastDeg_p_abd=0.0f; vel_p_abd=0;
                                                homing_p          = 0;
                                                homing_m          = 1;
                                                homing_m_stage    = 0;
                                                homingStallInit_m = 0;

                                                __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (uint32_t)deadzone_pwm_homing);
                                                MEDIO_MCP_BACKWARD();

                                                char out[] = "HOME PULGAR OK - iniciando MEDIO...\r\n";
                                                HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                            }
                                        }
                                    }
                                    goto log_section;
                                }

                                // ================= HOMING MEDIO =================
                                if (homing_m)
                                {
                                    if (homing_m_stage == 0) // MCP (encoder IRQ)
                                    {
                                        if (!homingStallInit_m)
                                        {
                                            homingLastCount_m = snap_m_mcp;
                                            homingStallTime_m = now;
                                            homingStallInit_m = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = snap_m_mcp - homingLastCount_m;
                                            homingLastCount_m = snap_m_mcp;
                                            if (abs(moved) > 2) homingStallTime_m = now;
                                            if ((now - homingStallTime_m) >= HOMING_STALL_MS_M_MCP)
                                            {
                                                MEDIO_MCP_STOP();
                                                __disable_irq(); encCount_m_mcp = 0; __enable_irq();
                                                curDeg_m_mcp=0.0f; lastDeg_m_mcp=0.0f; vel_m_mcp=0;
                                                homing_m_stage    = 1;
                                                homingStallInit_m = 0;

                                                __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, (uint32_t)deadzone_pwm_homing);
                                                MEDIO_PIP_BACKWARD();
                                            }
                                        }
                                    }
                                    else // PIP (TIM8 encoder)
                                    {
                                        if (!homingStallInit_m)
                                        {
                                            homingLastCount_m = encCount_m_pip;
                                            homingStallTime_m = now;
                                            homingStallInit_m = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = encCount_m_pip - homingLastCount_m;
                                            homingLastCount_m = encCount_m_pip;
                                            if (abs(moved) > 2) homingStallTime_m = now;
                                            if ((now - homingStallTime_m) >= HOMING_STALL_MS_M_PIP)
                                            {
                                                MEDIO_PIP_STOP();
                                                encCount_m_pip=0; lastCount_m_pip=(int32_t)__HAL_TIM_GET_COUNTER(&htim8);
                                                curDeg_m_pip=0.0f; lastDeg_m_pip=0.0f; vel_m_pip=0;
                                                homing_m = 0;

                                                // ===== continuar con ANULAR =====
                                                homing_a          = 1;
                                                homing_a_stage    = 0;
                                                homingStallInit_a = 0;
                                                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (uint32_t)deadzone_pwm_homing_anular_mcp);
                                                ANULAR_MCP_BACKWARD();

                                                char out[] = "HOME MEDIO OK - iniciando ANULAR...\r\n";
                                                HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                            }
                                        }
                                    }
                                    goto log_section;
                                }

                                // ================= HOMING ANULAR =================
                                if (homing_a)
                                {
                                    if (homing_a_stage == 0) // MCP
                                    {
                                        if (!homingStallInit_a)
                                        {
                                            homingLastCount_a = snap_a_mcp;
                                            homingStallTime_a = now;
                                            homingStallInit_a = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = snap_a_mcp - homingLastCount_a;
                                            homingLastCount_a = snap_a_mcp;
                                            if (abs(moved) > 2) homingStallTime_a = now;
                                            if ((now - homingStallTime_a) >= HOMING_STALL_MS_A_MCP)
                                            {
                                                ANULAR_MCP_STOP();
                                                __disable_irq(); encCount_a_mcp = 0; __enable_irq();
                                                curDeg_a_mcp=0.0f; lastDeg_a_mcp=0.0f; vel_a_mcp=0;
                                                homing_a_stage    = 1;
                                                homingStallInit_a = 0;

                                                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, (uint32_t)deadzone_pwm_homing_anular_pip);
                                                ANULAR_PIP_BACKWARD();
                                            }
                                        }
                                    }
                                    else // PIP
                                    {
                                        if (!homingStallInit_a)
                                        {
                                            homingLastCount_a = snap_a_pip;
                                            homingStallTime_a = now;
                                            homingStallInit_a = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = snap_a_pip - homingLastCount_a;
                                            homingLastCount_a = snap_a_pip;
                                            if (abs(moved) > 2) homingStallTime_a = now;
                                            if ((now - homingStallTime_a) >= HOMING_STALL_MS_A_PIP)
                                            {
                                                ANULAR_PIP_STOP();
                                                __disable_irq(); encCount_a_pip = 0; __enable_irq();
                                                curDeg_a_pip=0.0f; lastDeg_a_pip=0.0f; vel_a_pip=0;
                                                homing_a = 0;

                                                // ===== continuar con MENIQUE =====
                                                homing_q          = 1;
                                                homing_q_stage    = 0;
                                                homingStallInit_q = 0;
                                                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint32_t)deadzone_pwm_homing_menique);
                                                MENIQUE_MCP_BACKWARD();

                                                char out[] = "HOME ANULAR OK - iniciando MENIQUE...\r\n";
                                                HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                            }
                                        }
                                    }
                                    goto log_section;
                                }

                                // ================= HOMING MENIQUE =================
                                if (homing_q)
                                {
                                    if (homing_q_stage == 0) // MCP (encoder IRQ)
                                    {
                                        if (!homingStallInit_q)
                                        {
                                            homingLastCount_q = snap_q_mcp;
                                            homingStallTime_q = now;
                                            homingStallInit_q = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = snap_q_mcp - homingLastCount_q;
                                            homingLastCount_q = snap_q_mcp;
                                            if (abs(moved) > 2) homingStallTime_q = now;
                                            if ((now - homingStallTime_q) >= HOMING_STALL_MS_Q_MCP)
                                            {
                                                MENIQUE_MCP_STOP();
                                                __disable_irq(); encCount_q_mcp = 0; __enable_irq();
                                                curDeg_q_mcp=0.0f; lastDeg_q_mcp=0.0f; vel_q_mcp=0;
                                                homing_q_stage    = 1;
                                                homingStallInit_q = 0;

                                                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint32_t)deadzone_pwm_homing_menique);
                                                MENIQUE_PIP_BACKWARD();
                                            }
                                        }
                                    }
                                    else // PIP (encoder IRQ)
                                    {
                                        if (!homingStallInit_q)
                                        {
                                            homingLastCount_q = snap_q_pip;
                                            homingStallTime_q = now;
                                            homingStallInit_q = 1;
                                        }
                                        else
                                        {
                                            int32_t moved = snap_q_pip - homingLastCount_q;
                                            homingLastCount_q = snap_q_pip;
                                            if (abs(moved) > 2) homingStallTime_q = now;
                                            if ((now - homingStallTime_q) >= HOMING_STALL_MS_Q_PIP)
                                            {
                                                MENIQUE_PIP_STOP();
                                                __disable_irq(); encCount_q_pip = 0; __enable_irq();
                                                curDeg_q_pip=0.0f; lastDeg_q_pip=0.0f; vel_q_pip=0;
                                                homing_q = 0;

                                                char out[] = "HOME OK - todos los dedos en 0 grados\r\n";
                                                HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                            }
                                        }
                                    }
                                    goto log_section;
                                }

                                // ================= SECUENCIA IK PULGAR (ABD -> MCP) =================
                                if (ikp_seq_active)
                                {
                                    if (ikp_seq_stage == 0)
                                    {
                                        if (stopped_p_abd)
                                        {
                                            ikp_seq_stage = 1;

                                            tgtDeg_p_mcp = ikp_target_mcp;
                                            tgtRad_p_mcp = ikp_target_mcp * (float)M_PI / 180.0f;
                                            stopped_p_mcp = 0;
                                            running_p_mcp = 1;
                                        }
                                    }
                                    else
                                    {
                                        if (stopped_p_mcp)
                                        {
                                            ikp_seq_active = 0;
                                        }
                                    }
                                }

                                // ================= SECUENCIA AGARRE LATERAL (INDICE -> ABD -> MCP) =================
                                if (grasp_lat_active)
                                {
                                    if (grasp_lat_stage == 0)            // esperando que el INDICE llegue
                                    {
                                        // Avanza cuando el indice llegó, o si quedó topado por su FSR
                                        if ((stopped_i_mcp && stopped_i_pip) || fsr_indice_blocked)
                                        {
                                            grasp_lat_stage = 1;
                                            tgtDeg_p_abd = grasp_lat_abd;
                                            tgtRad_p_abd = grasp_lat_abd * (float)M_PI / 180.0f;
                                            stopped_p_abd = 0;
                                            running_p_abd = 1;

                                            char out[] = "AGARRE: indice OK -> moviendo ABD pulgar...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else if (grasp_lat_stage == 1)       // esperando que el ABD llegue
                                    {
                                        if (stopped_p_abd || fsr_pulgar_blocked)
                                        {
                                            grasp_lat_stage = 2;
                                            tgtDeg_p_mcp = grasp_lat_mcp;
                                            tgtRad_p_mcp = grasp_lat_mcp * (float)M_PI / 180.0f;
                                            stopped_p_mcp = 0;
                                            running_p_mcp = 1;

                                            char out[] = "AGARRE: ABD OK -> moviendo MCP pulgar...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else                                  // esperando que el MCP llegue
                                    {
                                        if (stopped_p_mcp || fsr_pulgar_blocked)
                                        {
                                            grasp_lat_active = 0;
                                            char out[] = "AGARRE LATERAL COMPLETO\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                }
                                // ================= SECUENCIA AGARRE PINZA (ABD -> MCP pulgar -> INDICE) =================
                                if (grasp_pin_active)
                                {
                                    if (grasp_pin_stage == 0)            // esperando que el ABD pulgar llegue
                                    {
                                        if (stopped_p_abd || fsr_pulgar_blocked)
                                        {
                                            grasp_pin_stage = 1;
                                            tgtDeg_p_mcp = GRASP_PINZA_P_MCP;
                                            tgtRad_p_mcp = GRASP_PINZA_P_MCP * (float)M_PI / 180.0f;
                                            stopped_p_mcp = 0;
                                            running_p_mcp = 1;

                                            char out[] = "PINZA: ABD OK -> moviendo MCP pulgar...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else if (grasp_pin_stage == 1)       // esperando que el MCP pulgar llegue
                                    {
                                        if (stopped_p_mcp || fsr_pulgar_blocked)
                                        {
                                            grasp_pin_stage = 2;
                                            tgtDeg_i_mcp = GRASP_PINZA_I_MCP;
                                            tgtRad_i_mcp = GRASP_PINZA_I_MCP * (float)M_PI / 180.0f;
                                            stopped_i_mcp = 0;
                                            running_i_mcp = 1;

                                            tgtDeg_i_pip = GRASP_PINZA_I_PIP;
                                            tgtRad_i_pip = GRASP_PINZA_I_PIP * (float)M_PI / 180.0f;
                                            stopped_i_pip = 0;
                                            running_i_pip = 1;

                                            char out[] = "PINZA: MCP pulgar OK -> cerrando indice...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else                                  // esperando que el indice llegue
                                    {
                                        if ((stopped_i_mcp && stopped_i_pip) || fsr_indice_blocked)
                                        {
                                            grasp_pin_active = 0;
                                            char out[] = "AGARRE PINZA COMPLETO\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                }
                                // ================= SECUENCIA AGARRE CILINDRICO =================
                                // ABD pulgar -> MCP pulgar -> MCP indice (seguidores copian) -> PIP indice (+ PIP seguidores fijos)
                                if (grasp_cil_active)
                                {
                                    // --- Freeze de seguidores si el indice se topa (no sobrepasar) ---
                                    if (fsr_indice_blocked)
                                    {
                                        tgtDeg_m_mcp = curDeg_m_mcp; tgtDeg_m_pip = curDeg_m_pip;
                                        tgtDeg_a_mcp = curDeg_a_mcp; tgtDeg_a_pip = curDeg_a_pip;
                                        tgtDeg_q_mcp = curDeg_q_mcp; tgtDeg_q_pip = curDeg_q_pip;
                                        stopped_m_mcp=1; running_m_mcp=0; MEDIO_MCP_STOP();
                                        stopped_m_pip=1; running_m_pip=0; MEDIO_PIP_STOP();
                                        stopped_a_mcp=1; running_a_mcp=0; ANULAR_MCP_STOP();
                                        stopped_a_pip=1; running_a_pip=0; ANULAR_PIP_STOP();
                                        stopped_q_mcp=1; running_q_mcp=0; MENIQUE_MCP_STOP();
                                        stopped_q_pip=1; running_q_pip=0; MENIQUE_PIP_STOP();
                                    }

                                    if (grasp_cil_stage == 0)            // esperando que el ABD pulgar llegue
                                    {
                                        if (stopped_p_abd || fsr_pulgar_blocked)
                                        {
                                            grasp_cil_stage = 1;
                                            // MCP del PULGAR (manual)
                                            tgtDeg_p_mcp = GRASP_CIL_MCP_P; tgtRad_p_mcp = GRASP_CIL_MCP_P*(float)M_PI/180.0f;
                                            stopped_p_mcp = 0; running_p_mcp = 1;
                                            char out[] = "CILINDRO: ABD OK -> MCP pulgar...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else if (grasp_cil_stage == 1)       // esperando que el MCP pulgar llegue (o tope)
                                                                        {
                                                                            if (stopped_p_mcp || fsr_pulgar_blocked)
                                                                            {
                                                                                grasp_cil_stage = 2;
                                                                                // MCP del INDICE (calculado)
                                                                                tgtDeg_i_mcp = cil_tg.i_mcp; tgtRad_i_mcp = cil_tg.i_mcp*(float)M_PI/180.0f;
                                                                                stopped_i_mcp = 0; running_i_mcp = 1;
                                                                                // SEGUIDORES MCP -> MISMO objetivo que el indice (se mueven juntos)
                                                                                tgtDeg_m_mcp = cil_tg.i_mcp; tgtRad_m_mcp = cil_tg.i_mcp*(float)M_PI/180.0f; stopped_m_mcp=0; running_m_mcp=1;
                                                                                tgtDeg_a_mcp = cil_tg.i_mcp; tgtRad_a_mcp = cil_tg.i_mcp*(float)M_PI/180.0f; stopped_a_mcp=0; running_a_mcp=1;
                                                                                tgtDeg_q_mcp = cil_tg.i_mcp; tgtRad_q_mcp = cil_tg.i_mcp*(float)M_PI/180.0f; stopped_q_mcp=0; running_q_mcp=1;
                                                                                char out[] = "CILINDRO: MCP pulgar OK -> MCP indice + seguidores...\r\n";
                                                                                HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                                                            }
                                                                        }
                                                                        else if (grasp_cil_stage == 2)       // esperando que el MCP del INDICE llegue (o tope)
                                    {
                                        if (stopped_i_mcp || fsr_indice_blocked)
                                        {
                                            grasp_cil_stage = 3;
                                            // PIP del INDICE (manual)
                                            tgtDeg_i_pip = cil_pip_indice; tgtRad_i_pip = cil_pip_indice*(float)M_PI/180.0f;
                                            stopped_i_pip = 0; running_i_pip = 1;
                                            // PIP seguidores (manual fijo por dedo)
                                            tgtDeg_m_pip = cil_pip_medio;   tgtRad_m_pip = cil_pip_medio*(float)M_PI/180.0f;   stopped_m_pip=0; running_m_pip=1;
                                            tgtDeg_a_pip = cil_pip_anular;  tgtRad_a_pip = cil_pip_anular*(float)M_PI/180.0f;  stopped_a_pip=0; running_a_pip=1;
                                            tgtDeg_q_pip = cil_pip_menique; tgtRad_q_pip = cil_pip_menique*(float)M_PI/180.0f; stopped_q_pip=0; running_q_pip=1;
                                            char out[] = "CILINDRO: MCP indice OK -> PIP indice + seguidores...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else                                  // stage 3: esperando que el PIP del INDICE llegue (o tope)
                                    {
                                        if (stopped_i_pip || fsr_indice_blocked)
                                        {
                                            grasp_cil_active = 0;
                                            char out[] = "AGARRE CILINDRICO COMPLETO\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                }
                                // ================= SECUENCIA AGARRE CILINDRICO 2 (inclinado) =================
                                // ABD pulgar -> MCP pulgar -> MCP por dedo (calculados) -> PIP (manuales por dedo)
                                if (grasp_cil2_active)
                                {
                                    // --- Freeze de seguidores si el indice se topa (no sobrepasar) ---
                                    if (fsr_indice_blocked)
                                    {
                                        tgtDeg_m_mcp = curDeg_m_mcp; tgtDeg_m_pip = curDeg_m_pip;
                                        tgtDeg_a_mcp = curDeg_a_mcp; tgtDeg_a_pip = curDeg_a_pip;
                                        tgtDeg_q_mcp = curDeg_q_mcp; tgtDeg_q_pip = curDeg_q_pip;
                                        stopped_m_mcp=1; running_m_mcp=0; MEDIO_MCP_STOP();
                                        stopped_m_pip=1; running_m_pip=0; MEDIO_PIP_STOP();
                                        stopped_a_mcp=1; running_a_mcp=0; ANULAR_MCP_STOP();
                                        stopped_a_pip=1; running_a_pip=0; ANULAR_PIP_STOP();
                                        stopped_q_mcp=1; running_q_mcp=0; MENIQUE_MCP_STOP();
                                        stopped_q_pip=1; running_q_pip=0; MENIQUE_PIP_STOP();
                                    }

                                    if (grasp_cil2_stage == 0)            // esperando que el ABD pulgar llegue
                                    {
                                        if (stopped_p_abd || fsr_pulgar_blocked)
                                        {
                                            grasp_cil2_stage = 1;
                                            // MCP del PULGAR (manual)
                                            tgtDeg_p_mcp = cil2_mcp_pulgar; tgtRad_p_mcp = cil2_mcp_pulgar*(float)M_PI/180.0f;
                                            stopped_p_mcp = 0; running_p_mcp = 1;
                                            char out[] = "CILINDRO2: ABD OK -> MCP pulgar...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else if (grasp_cil2_stage == 1)       // esperando que el MCP pulgar llegue (o tope)
                                    {
                                        if (stopped_p_mcp || fsr_pulgar_blocked)
                                        {
                                            grasp_cil2_stage = 2;
                                            // MCP por dedo (calculados, cada uno su propio angulo)
                                            tgtDeg_i_mcp = cil2_mcp_indice;  tgtRad_i_mcp = cil2_mcp_indice *(float)M_PI/180.0f; stopped_i_mcp=0; running_i_mcp=1;
                                            tgtDeg_m_mcp = cil2_mcp_medio;   tgtRad_m_mcp = cil2_mcp_medio  *(float)M_PI/180.0f; stopped_m_mcp=0; running_m_mcp=1;
                                            tgtDeg_a_mcp = cil2_mcp_anular;  tgtRad_a_mcp = cil2_mcp_anular *(float)M_PI/180.0f; stopped_a_mcp=0; running_a_mcp=1;
                                            tgtDeg_q_mcp = cil2_mcp_menique; tgtRad_q_mcp = cil2_mcp_menique*(float)M_PI/180.0f; stopped_q_mcp=0; running_q_mcp=1;
                                            char out[] = "CILINDRO2: MCP pulgar OK -> MCP por dedo...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else if (grasp_cil2_stage == 2)       // esperando que TODOS los MCP de dedo lleguen (o tope indice)
                                    {
                                        if ((stopped_i_mcp && stopped_m_mcp && stopped_a_mcp && stopped_q_mcp) || fsr_indice_blocked)
                                        {
                                            grasp_cil2_stage = 3;
                                            // PIP del INDICE (manual)
                                            tgtDeg_i_pip = cil2_pip_indice; tgtRad_i_pip = cil2_pip_indice*(float)M_PI/180.0f;
                                            stopped_i_pip = 0; running_i_pip = 1;
                                            // PIP seguidores (manual fijo por dedo)
                                            tgtDeg_m_pip = cil2_pip_medio;   tgtRad_m_pip = cil2_pip_medio*(float)M_PI/180.0f;   stopped_m_pip=0; running_m_pip=1;
                                            tgtDeg_a_pip = cil2_pip_anular;  tgtRad_a_pip = cil2_pip_anular*(float)M_PI/180.0f;  stopped_a_pip=0; running_a_pip=1;
                                            tgtDeg_q_pip = cil2_pip_menique; tgtRad_q_pip = cil2_pip_menique*(float)M_PI/180.0f; stopped_q_pip=0; running_q_pip=1;
                                            char out[] = "CILINDRO2: MCP dedos OK -> PIP por dedo...\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                    else                                  // stage 3: esperando que el PIP del INDICE llegue (o tope)
                                    {
                                        if (stopped_i_pip || fsr_indice_blocked)
                                        {
                                            grasp_cil2_active = 0;
                                            char out[] = "AGARRE CILINDRICO 2 COMPLETO\r\n";
                                            HAL_UART_Transmit(&huart3, (uint8_t*)out, strlen(out), HAL_MAX_DELAY);
                                        }
                                    }
                                }
                                // ================= CONTROL LQR =================
                                #define LQR_MOTOR(curDeg, vel, tgtDeg, tgtRad, running, stopped,  \
                                                  FWD, BWD, STOP, htim, ch, dz) do {               \
                                    int _pwm = 0;                                                   \
                                    if (stopped) { STOP(); }                                        \
                                    else if (running)                                               \
                                    {                                                               \
                                        float _cRad = (curDeg)*(float)M_PI/180.0f;                 \
                                        float _vRad = (vel)*(float)M_PI/180.0f;                    \
                                        float _et   = (tgtRad) - _cRad;                            \
                                        float _ew   = 0.0f - _vRad;                                \
                                        float _ctrl = K1*_et + K2*_ew;                             \
                                        if (_ctrl >  4095.0f) _ctrl =  4095.0f;                    \
                                        if (_ctrl < -4095.0f) _ctrl = -4095.0f;                    \
                                        float _ac = fabsf(_ctrl);                                   \
                                        if (fabsf((tgtDeg)-(curDeg)) < 1.0f)                        \
                                        {                                                           \
                                            stopped = 1;                                            \
                                            STOP();                                                 \
                                        }                                                           \
                                        else if (_ac < 2.0f)                                      \
                                        {                                                           \
                                            STOP();                                                 \
                                        }                                                           \
                                        else {                                                      \
                                            _pwm = (int)((dz) + _ac);                              \
                                            if (_pwm > (int)max_pwm) _pwm = (int)max_pwm;          \
                                            if (_ctrl > 0) { FWD(); }                               \
                                            else           { BWD(); }                               \
                                            __HAL_TIM_SET_COMPARE(&(htim), (ch), (uint32_t)_pwm);  \
                                        }                                                           \
                                    }                                                               \
                                } while(0)

								#define TRAJ_MOTOR(traj, curDeg, vel, tgtDeg, running, stopped,        \
												   FWD, BWD, STOP, htim, ch, dz) do {                    \
									if (running)                                                        \
									{                                                                   \
										if (!(traj).active && !(stopped) &&                             \
											fabsf((traj).endDeg - (tgtDeg)) > 0.05f &&                  \
											fabsf((tgtDeg) - (curDeg))      > 1.5f)                     \
										{                                                               \
											traj_plan(&(traj), (curDeg), (tgtDeg));                     \
										}                                                               \
										float _refDeg, _refVel;                                         \
										traj_eval(&(traj), &_refDeg, &_refVel);                         \
										float _et   = (_refDeg - (curDeg)) * (float)M_PI/180.0f;       \
										float _ew   = (_refVel - (vel))    * (float)M_PI/180.0f;       \
										float _ctrl = K1*_et + K2*_ew;                                  \
										if (_ctrl >  4095.0f) _ctrl =  4095.0f;                        \
										if (_ctrl < -4095.0f) _ctrl = -4095.0f;                        \
										if (!(traj).active && fabsf((tgtDeg) - (curDeg)) < 1.5f)        \
										{                                                               \
											(stopped) = 1; (running) = 0;                               \
											STOP();                                                     \
										}                                                               \
										else                                                            \
										{                                                               \
											float _dir = (traj).active ? (traj).dir                     \
														  : (((_refDeg - (curDeg)) >= 0.0f) ? 1.0f : -1.0f); \
											float _u = (dz) * _dir + _ctrl;                             \
											if (_u >  max_pwm) _u =  max_pwm;                          \
											if (_u < -max_pwm) _u = -max_pwm;                          \
											if (_u >= 0.0f) FWD(); else BWD();                          \
											__HAL_TIM_SET_COMPARE(&(htim), (ch), (uint32_t)fabsf(_u)); \
										}                                                               \
									}                                                                   \
									else { STOP(); }                                                    \
								} while(0)

                                if (!fsr_indice_blocked)
                                {
                                    TRAJ_MOTOR(traj_i_mcp, curDeg_i_mcp, vel_i_mcp, tgtDeg_i_mcp,
                                               running_i_mcp, stopped_i_mcp,
                                               INDICE_MCP_FORWARD, INDICE_MCP_BACKWARD, INDICE_MCP_STOP,
                                               htim14, TIM_CHANNEL_1, deadzone_pwm_i);

                                    TRAJ_MOTOR(traj_i_pip, curDeg_i_pip, vel_i_pip, tgtDeg_i_pip,
                                               running_i_pip, stopped_i_pip,
                                               INDICE_PIP_FORWARD, INDICE_PIP_BACKWARD, INDICE_PIP_STOP,
                                               htim13, TIM_CHANNEL_1, deadzone_pwm_i_pip);
                                }
                                else
                                {
                                    INDICE_MCP_STOP();
                                    INDICE_PIP_STOP();
                                }
                                if (!fsr_pulgar_blocked)
                                {
                                    TRAJ_MOTOR(traj_p_abd, curDeg_p_abd, vel_p_abd, tgtDeg_p_abd,
                                               running_p_abd, stopped_p_abd,
                                               PULGAR_ABD_FORWARD, PULGAR_ABD_BACKWARD, PULGAR_ABD_STOP,
                                               htim12, TIM_CHANNEL_1, deadzone_pwm_p_abd);

                                    TRAJ_MOTOR(traj_p_mcp, curDeg_p_mcp, vel_p_mcp, tgtDeg_p_mcp,
                                               running_p_mcp, stopped_p_mcp,
                                               PULGAR_MCP_FORWARD, PULGAR_MCP_BACKWARD, PULGAR_MCP_STOP,
                                               htim12, TIM_CHANNEL_2, deadzone_pwm_p_mcp);
                                }
                                else
                                {
                                    PULGAR_ABD_STOP();
                                    PULGAR_MCP_STOP();
                                }

                                // ===== CONTROL DEDO MEDIO (sin FSR) =====
                                TRAJ_MOTOR(traj_m_mcp, curDeg_m_mcp, vel_m_mcp, tgtDeg_m_mcp,
                                           running_m_mcp, stopped_m_mcp,
                                           MEDIO_MCP_FORWARD, MEDIO_MCP_BACKWARD, MEDIO_MCP_STOP,
                                           htim11, TIM_CHANNEL_1, deadzone_pwm);
                                TRAJ_MOTOR(traj_m_pip, curDeg_m_pip, vel_m_pip, tgtDeg_m_pip,
                                           running_m_pip, stopped_m_pip,
                                           MEDIO_PIP_FORWARD, MEDIO_PIP_BACKWARD, MEDIO_PIP_STOP,
                                           htim10, TIM_CHANNEL_1, deadzone_pwm);

                                // ===== CONTROL DEDO ANULAR (sin FSR) =====
                                TRAJ_MOTOR(traj_a_mcp, curDeg_a_mcp, vel_a_mcp, tgtDeg_a_mcp,
                                           running_a_mcp, stopped_a_mcp,
                                           ANULAR_MCP_FORWARD, ANULAR_MCP_BACKWARD, ANULAR_MCP_STOP,
                                           htim9, TIM_CHANNEL_1, deadzone_pwm_anular);
                                TRAJ_MOTOR(traj_a_pip, curDeg_a_pip, vel_a_pip, tgtDeg_a_pip,
                                           running_a_pip, stopped_a_pip,
                                           ANULAR_PIP_FORWARD, ANULAR_PIP_BACKWARD, ANULAR_PIP_STOP,
                                           htim9, TIM_CHANNEL_2, deadzone_pwm_anular);

                                // ===== CONTROL DEDO MENIQUE (sin FSR) =====
                                TRAJ_MOTOR(traj_q_mcp, curDeg_q_mcp, vel_q_mcp, tgtDeg_q_mcp,
                                           running_q_mcp, stopped_q_mcp,
                                           MENIQUE_MCP_FORWARD, MENIQUE_MCP_BACKWARD, MENIQUE_MCP_STOP,
                                           htim5, TIM_CHANNEL_1, deadzone_pwm_menique);
                                TRAJ_MOTOR(traj_q_pip, curDeg_q_pip, vel_q_pip, tgtDeg_q_pip,
                                           running_q_pip, stopped_q_pip,
                                           MENIQUE_PIP_FORWARD, MENIQUE_PIP_BACKWARD, MENIQUE_PIP_STOP,
                                           htim5, TIM_CHANNEL_2, deadzone_pwm_menique);

                                log_section:
                                // ================= LOG cada 100 ms =================
                                if ((now - lastLog) >= 100)
                                {
                                    lastLog = now;

                                    if (homing_i || homing_p || homing_m || homing_a || homing_q)
                                    {
                                        char b[160];
                                        int  l = 0;

                                        if (homing_i && homing_i_stage == 0)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] >>INDICE_MCP<< enc=%.1f stall=%lums | I_PIP=%.1f P_ABD=%.1f P_MCP=%.1f\r\n",
                                                curDeg_i_mcp,
                                                (unsigned long)(homingStallInit_i ? (now - homingStallTime_i) : 0),
                                                curDeg_i_pip, curDeg_p_abd, curDeg_p_mcp);

                                        else if (homing_i && homing_i_stage == 1)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] I_MCP=DONE >>INDICE_PIP<< enc=%.1f stall=%lums | P_ABD=%.1f P_MCP=%.1f\r\n",
                                                curDeg_i_pip,
                                                (unsigned long)(homingStallInit_i ? (now - homingStallTime_i) : 0),
                                                curDeg_p_abd, curDeg_p_mcp);

                                        else if (homing_p && homing_p_stage == 0)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] INDICE=DONE >>PULGAR_MCP<< enc=%.1f stall=%lums | P_ABD=%.1f\r\n",
                                                curDeg_p_mcp,
                                                (unsigned long)(homingStallInit_p ? (now - homingStallTime_p) : 0),
                                                curDeg_p_abd);

                                        else if (homing_p && homing_p_stage == 1)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] P_MCP=DONE >>PULGAR_ABD<< enc=%.1f stall=%lums\r\n",
                                                curDeg_p_abd,
                                                (unsigned long)(homingStallInit_p ? (now - homingStallTime_p) : 0));

                                        else if (homing_m && homing_m_stage == 0)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] PULGAR=DONE >>MEDIO_MCP<< enc=%.1f stall=%lums | M_PIP=%.1f\r\n",
                                                curDeg_m_mcp,
                                                (unsigned long)(homingStallInit_m ? (now - homingStallTime_m) : 0),
                                                curDeg_m_pip);

                                        else if (homing_m && homing_m_stage == 1)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] M_MCP=DONE >>MEDIO_PIP<< enc=%.1f stall=%lums\r\n",
                                                curDeg_m_pip,
                                                (unsigned long)(homingStallInit_m ? (now - homingStallTime_m) : 0));

                                        else if (homing_a && homing_a_stage == 0)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] MEDIO=DONE >>ANULAR_MCP<< enc=%.1f stall=%lums | A_PIP=%.1f\r\n",
                                                curDeg_a_mcp,
                                                (unsigned long)(homingStallInit_a ? (now - homingStallTime_a) : 0),
                                                curDeg_a_pip);

                                        else if (homing_a && homing_a_stage == 1)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] A_MCP=DONE >>ANULAR_PIP<< enc=%.1f stall=%lums\r\n",
                                                curDeg_a_pip,
                                                (unsigned long)(homingStallInit_a ? (now - homingStallTime_a) : 0));

                                        else if (homing_q && homing_q_stage == 0)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] ANULAR=DONE >>MENIQUE_MCP<< enc=%.1f stall=%lums | Q_PIP=%.1f\r\n",
                                                curDeg_q_mcp,
                                                (unsigned long)(homingStallInit_q ? (now - homingStallTime_q) : 0),
                                                curDeg_q_pip);

                                        else if (homing_q && homing_q_stage == 1)
                                            l = snprintf(b, sizeof(b),
                                                "[HOME] Q_MCP=DONE >>MENIQUE_PIP<< enc=%.1f stall=%lums\r\n",
                                                curDeg_q_pip,
                                                (unsigned long)(homingStallInit_q ? (now - homingStallTime_q) : 0));

                                        if (l > 0)
                                            HAL_UART_Transmit(&huart3, (uint8_t*)b, l, HAL_MAX_DELAY);
                                    }
                                    else
                                    {
                                        char b[400];
                                        int  l = snprintf(b, sizeof(b),
                                            "I_MCP T:%.1f C:%.1f | I_PIP T:%.1f C:%.1f | P_ABD T:%.1f C:%.1f | P_MCP T:%.1f C:%.1f | M_MCP T:%.1f C:%.1f | M_PIP T:%.1f C:%.1f | A_MCP T:%.1f C:%.1f | A_PIP T:%.1f C:%.1f | Q_MCP T:%.1f C:%.1f | Q_PIP T:%.1f C:%.1f | FSR_I:%lu FSR_P:%lu\r\n",
                                            tgtDeg_i_mcp, curDeg_i_mcp,
                                            tgtDeg_i_pip, curDeg_i_pip,
                                            tgtDeg_p_abd, curDeg_p_abd,
                                            tgtDeg_p_mcp, curDeg_p_mcp,
                                            tgtDeg_m_mcp, curDeg_m_mcp,
                                            tgtDeg_m_pip, curDeg_m_pip,
                                            tgtDeg_a_mcp, curDeg_a_mcp,
                                            tgtDeg_a_pip, curDeg_a_pip,
                                            tgtDeg_q_mcp, curDeg_q_mcp,
                                            tgtDeg_q_pip, curDeg_q_pip,
                                            fsr_indice_val, fsr_pulgar_val);
                                        HAL_UART_Transmit(&huart3, (uint8_t*)b, l, HAL_MAX_DELAY);

                                        uint8_t any_running = running_i_mcp || running_i_pip ||
                                                              running_p_abd || running_p_mcp ||
                                                              running_m_mcp || running_m_pip ||
                                                              running_a_mcp || running_a_pip ||
                                                              running_q_mcp || running_q_pip;
                                        uint8_t any_stopped = stopped_i_mcp || stopped_i_pip ||
                                                              stopped_p_abd || stopped_p_mcp ||
                                                              stopped_m_mcp || stopped_m_pip ||
                                                              stopped_a_mcp || stopped_a_pip ||
                                                              stopped_q_mcp || stopped_q_pip;

                                        if ((any_running || any_stopped) && !grasp_lat_active && !ikp_seq_active && !grasp_pin_active && !grasp_cil_active && !grasp_cil2_active)
                                        {
                                            uint8_t all_stopped = stopped_i_mcp && stopped_i_pip &&
                                                                  stopped_p_abd && stopped_p_mcp &&
                                                                  stopped_m_mcp && stopped_m_pip &&
                                                                  stopped_a_mcp && stopped_a_pip &&
                                                                  stopped_q_mcp && stopped_q_pip;
                                            if (all_stopped && !any_running && (now - startTime) > 500)
                                            {
                                                running_i_mcp=0; running_i_pip=0;
                                                running_p_abd=0; running_p_mcp=0;
                                                running_m_mcp=0; running_m_pip=0;
                                                running_a_mcp=0; running_a_pip=0;
                                                running_q_mcp=0; running_q_pip=0;
                                                stopped_i_mcp=0; stopped_i_pip=0;
                                                stopped_p_abd=0; stopped_p_mcp=0;
                                                stopped_m_mcp=0; stopped_m_pip=0;
                                                stopped_a_mcp=0; stopped_a_pip=0;
                                                stopped_q_mcp=0; stopped_q_pip=0;
                                                char d[] = "DONE\r\n";
                                                HAL_UART_Transmit(&huart3, (uint8_t*)d, strlen(d), HAL_MAX_DELAY);
                                            }
                                        }
                                    }
                                }

                            } // fin while(1)
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4095;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4095;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4095;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 4095;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 4095;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 4095;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4095;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_11
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE4 PE7 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF3 PF5 PF7 PF11
                           PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_11
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE14 PE15 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG2 PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 PD4 PD5 PD6
                           PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
    // Anular + Medio MCP (PB10/PF13) -> ya estaba habilitado
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    // Menique PIP : A=PF4, B=PB6
    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);   // cubre PB6 (EXTI6)
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // ===== ANULAR MCP : A=PE12, B=PB11 =====
    if (GPIO_Pin == GPIO_PIN_12 || GPIO_Pin == GPIO_PIN_11)
    {
        irq_mcp++;

        uint8_t A = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
        uint8_t B = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

        if (GPIO_Pin == GPIO_PIN_12)        // flanco en A (PE12)
        {
            if (A == B) encCount_a_mcp++;
            else        encCount_a_mcp--;
        }
        else                                // flanco en B (PB11)
        {
            if (A == B) encCount_a_mcp--;
            else        encCount_a_mcp++;
        }
    }
    // ===== ANULAR PIP : A=PE14, B=PE15 =====
    else if (GPIO_Pin == GPIO_PIN_14 || GPIO_Pin == GPIO_PIN_15)
    {
        irq_pip++;

        uint8_t A = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
        uint8_t B = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);

        if (GPIO_Pin == GPIO_PIN_14)        // flanco en A (PE14)
        {
            if (A == B) encCount_a_pip++;
            else        encCount_a_pip--;
        }
        else                                // flanco en B (PE15)
        {
            if (A == B) encCount_a_pip--;
            else        encCount_a_pip++;
        }
    }
    // ===== MEDIO MCP : A=PB10, B=PF13 =====
    else if (GPIO_Pin == GPIO_PIN_10 || GPIO_Pin == GPIO_PIN_13)
    {
        uint8_t A = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
        uint8_t B = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13);

        if (GPIO_Pin == GPIO_PIN_10)        // flanco en A (PB10)
        {
            if (A == B) encCount_m_mcp--;
            else        encCount_m_mcp++;
        }
        else                                // flanco en B (PF13)
        {
            if (A == B) encCount_m_mcp++;
            else        encCount_m_mcp--;
        }
    }
    // ===== MENIQUE MCP : A=PE0, B=PG8 =====
    else if (GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_8)
    {
        uint8_t A = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
        uint8_t B = HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_8);

        if (GPIO_Pin == GPIO_PIN_0)         // flanco en A (PE0)
        {
            if (A == B) encCount_q_mcp++;
            else        encCount_q_mcp--;
        }
        else                                // flanco en B (PG8)
        {
            if (A == B) encCount_q_mcp--;
            else        encCount_q_mcp++;
        }
    }
    // ===== MENIQUE PIP : A=PF4, B=PB6 =====
    else if (GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_6)
    {
        uint8_t A = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4);
        uint8_t B = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);

        if (GPIO_Pin == GPIO_PIN_4)         // flanco en A (PF4)
        {
            if (A == B) encCount_q_pip--;
            else        encCount_q_pip++;
        }
        else                                // flanco en B (PB6)
        {
            if (A == B) encCount_q_pip++;
            else        encCount_q_pip--;
        }
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
