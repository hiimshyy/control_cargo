/*
 * modbus_rtu.h
 *
 *  Created on: Nov 25, 2025
 *      Author: tiensy
 */

#ifndef INC_MODBUS_RTU_H_
#define INC_MODBUS_RTU_H_

#include "main.h"
#include "modbus_regs.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>

/* ============================================================================
 * MODBUS RTU CONFIGURATION
 * ============================================================================ */
#define MODBUS_SLAVE_ADDRESS        0x07       // Địa chỉ slave mặc định (1-247)
#define MODBUS_BAUDRATE             115200      // Tốc độ baud (115200 bps)
#define MODBUS_TIMEOUT_MS           100         // Timeout giữa các frame (ms)
#define MODBUS_MAX_FRAME_SIZE       256         // Kích thước buffer tối đa
#define MODBUS_T35_TIMEOUT_MS       5           // 3.5 ký tự timeout (~5ms @ 115200 baud, tăng để an toàn)

/* ============================================================================
 * MODBUS FUNCTION CODES
 * ============================================================================ */
#define MODBUS_FC_READ_HOLDING_REGS     0x03    // Đọc Holding Registers
#define MODBUS_FC_READ_INPUT_REGS       0x04    // Đọc Input Registers
#define MODBUS_FC_WRITE_SINGLE_REG      0x06    // Ghi 1 Register
#define MODBUS_FC_WRITE_MULTIPLE_REGS   0x10    // Ghi nhiều Registers

/* ============================================================================
 * MODBUS EXCEPTION CODES
 * ============================================================================ */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS   0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE     0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE   0x04

/* ============================================================================
 * MODBUS REGISTER TYPES
 * ============================================================================ */
typedef enum {
    MODBUS_REG_HOLDING = 0,     // Holding Registers (Read/Write)
    MODBUS_REG_INPUT            // Input Registers (Read Only)
} ModbusRegType_t;

/* ============================================================================
 * MODBUS DATA STRUCTURES
 * ============================================================================ */

// Cấu trúc lưu trữ tất cả Modbus registers
typedef struct {
    // Door States (0x0000-0x0003)
    uint16_t doorState;                 // REG_DOOR_STATE
    uint16_t doorPositionPercent;       // REG_DOOR_POSITION_PERCENT
    uint16_t doorTargetPosition;        // REG_DOOR_TARGET_POSITION
    uint16_t sensorStates;              // REG_SENSOR_STATES
    
    // Object Detection (0x0004)
    uint16_t objectDetection;           // REG_OBJECT_DETECTION
    
    // Motor 1 Status (0x0005-0x000C)
    uint16_t motor1PositionAbs;         // REG_MOTOR1_POSITION_ABS
    uint16_t motor1PositionRel;         // REG_MOTOR1_POSITION_REL
    uint16_t motor1CurrentSpeed;        // REG_MOTOR1_CURRENT_SPEED
    uint16_t motor1TargetSpeed;         // REG_MOTOR1_TARGET_SPEED
    uint16_t motor1PwmDuty;             // REG_MOTOR1_PWM_DUTY
    uint16_t motor1MaxSpeed;            // REG_MOTOR1_MAX_SPEED
    uint16_t motor1Acceleration;        // REG_MOTOR1_ACCELERATION
    uint16_t motor1Deceleration;        // REG_MOTOR1_DECELERATION
    
    // Motor 2 Status (0x000D-0x0014)
    uint16_t motor2PositionAbs;         // REG_MOTOR2_POSITION_ABS
    uint16_t motor2PositionRel;         // REG_MOTOR2_POSITION_REL
    uint16_t motor2CurrentSpeed;        // REG_MOTOR2_CURRENT_SPEED
    uint16_t motor2TargetSpeed;         // REG_MOTOR2_TARGET_SPEED
    uint16_t motor2PwmDuty;             // REG_MOTOR2_PWM_DUTY
    uint16_t motor2MaxSpeed;            // REG_MOTOR2_MAX_SPEED
    uint16_t motor2Acceleration;        // REG_MOTOR2_ACCELERATION
    uint16_t motor2Deceleration;        // REG_MOTOR2_DECELERATION
    
    // PID Parameters (0x0015-0x0017)
    uint16_t pidKp;                     // REG_PID_KP
    uint16_t pidKi;                     // REG_PID_KI
    uint16_t pidKd;                     // REG_PID_KD
    
    // Sensor Values (0x0018-0x001B)
    uint16_t proximity1State;           // REG_PROXIMITY_1_STATE
    uint16_t proximity2State;           // REG_PROXIMITY_2_STATE
    uint16_t proximity3State;           // REG_PROXIMITY_3_STATE
    uint16_t proximity4State;           // REG_PROXIMITY_4_STATE
    
    // Encoder Status (0x001C-0x001F)
    uint16_t motor1EncoderCount;        // REG_MOTOR1_ENCODER_COUNT
    uint16_t motor2EncoderCount;        // REG_MOTOR2_ENCODER_COUNT
    uint16_t motor1EncoderErrors;       // REG_MOTOR1_ENCODER_ERRORS
    uint16_t motor2EncoderErrors;       // REG_MOTOR2_ENCODER_ERRORS
    
    // Fault Relay (0x0020)
    uint16_t faultRelayStatus;          // REG_FAULT_RL_STATUS
    
    // System Information (0x0021-0x0025)
    uint16_t systemUptimeHi;            // REG_SYSTEM_UPTIME_HI
    uint16_t systemUptimeLo;            // REG_SYSTEM_UPTIME_LO
    uint16_t cpuUsage;                  // REG_CPU_USAGE
    uint16_t freeMemory;                // REG_FREE_MEMORY
    uint16_t taskCount;                 // REG_TASK_COUNT
    
    // Communication Status (0x0026-0x0028)
    uint16_t errorCode;                 // REG_ERROR_CODE
    uint16_t faultFlags;                // REG_FAULT_FLAGS
    uint16_t warningFlags;              // REG_WARNING_FLAGS
    
    // System Configuration (0x0029-0x002C)
    uint16_t systemMode;                // REG_SYSTEM_MODE
    uint16_t autoCalibrate;             // REG_AUTO_CALIBRATE
    uint16_t safetyTimeout;             // REG_SAFETY_TIMEOUT
    uint16_t emergencyStop;             // REG_EMERGENCY_STOP
    
    // Position Limits (0x002D-0x0032)
    uint16_t doorOpenLimit;             // REG_DOOR_OPEN_LIMIT
    uint16_t doorCloseLimit;            // REG_DOOR_CLOSE_LIMIT
    uint16_t motor1SoftLimitMin;        // REG_MOTOR1_SOFT_LIMIT_MIN
    uint16_t motor1SoftLimitMax;        // REG_MOTOR1_SOFT_LIMIT_MAX
    uint16_t motor2SoftLimitMin;        // REG_MOTOR2_SOFT_LIMIT_MIN
    uint16_t motor2SoftLimitMax;        // REG_MOTOR2_SOFT_LIMIT_MAX
    
    // Calibration Data (0x0033-0x003A)
    uint16_t calibrateMotor1Home;       // REG_CALIBRATE_MOTOR1_HOME
    uint16_t calibrateMotor2Home;       // REG_CALIBRATE_MOTOR2_HOME
    uint16_t calibrateDoorOpenPos;      // REG_CALIBRATE_DOOR_OPEN_POS
    uint16_t calibrateDoorClosePos;     // REG_CALIBRATE_DOOR_CLOSE_POS
    uint16_t startCalibration;          // REG_START_CALIBRATION
    uint16_t stopCalibration;           // REG_STOP_CALIBRATION
    uint16_t calibrationStatus;         // REG_CALIBRATION_STATUS
    uint16_t calibrationError;          // REG_CALIBRATION_ERROR
    uint16_t reserved3B;                // REG_RESERVED_3B (dummy)
    
    // System Registers (0x0100-0x010A)
    uint16_t deviceId;                  // REG_DEVICE_ID
    uint16_t configBaudrate;            // REG_CONFIG_BAUDRATE
    uint16_t configParity;              // REG_CONFIG_PARITY
    uint16_t configStopBits;            // REG_CONFIG_STOP_BITS
    uint16_t moduleType;                // REG_MODULE_TYPE
    uint16_t firmwareVersion;           // REG_FIRMWARE_VERSION
    uint16_t hardwareVersion;           // REG_HARDWARE_VERSION
    uint16_t systemStatus;              // REG_SYSTEM_STATUS
    uint16_t systemError;               // REG_SYSTEM_ERROR
    uint16_t resetErrorCmd;             // REG_RESET_ERROR_CMD
    uint16_t resetSystem;               // REG_RESET_SYSTEM
} ModbusRegs_t;

// Thống kê Modbus
typedef struct {
    uint32_t rxFrames;          // Số frame nhận được
    uint32_t txFrames;          // Số frame gửi đi
    uint32_t crcErrors;         // Lỗi CRC
    uint32_t timeoutErrors;     // Lỗi timeout
    uint32_t exceptionsSent;    // Số exception đã gửi
    uint32_t lastErrorCode;     // Mã lỗi cuối cùng
} ModbusStats_t;

/* ============================================================================
 * GLOBAL VARIABLES (extern)
 * ============================================================================ */
extern ModbusRegs_t modbusRegs;         // Modbus registers
extern ModbusStats_t modbusStats;       // Thống kê
extern osMutexId_t modbusMutexHandle;   // Mutex bảo vệ registers

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

/**
 * @brief Khởi tạo Modbus RTU slave
 * @retval None
 */
void Modbus_Init();

/**
 * @brief Task xử lý Modbus (gọi từ FreeRTOS task)
 * @param argument: Tham số task (không sử dụng)
 * @retval None
 */
void Modbus_Task(void *argument);

/**
 * @brief Đọc giá trị register (thread-safe)
 * @param regAddr: Địa chỉ register
 * @param value: Con trỏ lưu giá trị đọc được
 * @retval 0: Thành công, -1: Lỗi
 */
int Modbus_ReadRegister(uint16_t regAddr, uint16_t *value);

/**
 * @brief Ghi giá trị register (thread-safe)
 * @param regAddr: Địa chỉ register
 * @param value: Giá trị cần ghi
 * @retval 0: Thành công, -1: Lỗi
 */
int Modbus_WriteRegister(uint16_t regAddr, uint16_t value);

/**
 * @brief Tính CRC16 Modbus
 * @param buffer: Dữ liệu cần tính CRC
 * @param length: Độ dài dữ liệu
 * @retval Giá trị CRC16
 */
uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length);

/**
 * @brief Xử lý frame Modbus nhận được
 * @param frame: Buffer chứa frame
 * @param length: Độ dài frame
 * @retval None
 */
void Modbus_ProcessFrame(uint8_t *frame, uint16_t length);

/**
 * @brief Gửi response Modbus
 * @param frame: Buffer chứa response
 * @param length: Độ dài response
 * @retval None
 */
void Modbus_SendResponse(uint8_t *frame, uint16_t length);

/**
 * @brief Gửi exception response
 * @param functionCode: Mã function
 * @param exceptionCode: Mã exception
 * @retval None
 */
void Modbus_SendException(uint8_t functionCode, uint8_t exceptionCode);

/**
 * @brief Callback khi có register được ghi (để task khác xử lý)
 * @param regAddr: Địa chỉ register
 * @param value: Giá trị mới
 * @retval None
 */
void Modbus_OnRegisterWrite(uint16_t regAddr, uint16_t value);

#endif /* INC_MODBUS_RTU_H_ */
