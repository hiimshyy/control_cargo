/*
 * modbus_rtu.c
 *
 *  Created on: Nov 27, 2025
 *      Author: tiensy
 *  
 *  Description: Modbus RTU Slave implementation cho STM32F103C8T6
 *               Hỗ trợ Function Codes: 0x03, 0x04, 0x06, 0x10
 *               UART2 (PA2-TX, PA3-RX) với RS485
 */

#include "modbus_rtu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ============================================================================
 * PRIVATE VARIABLES
 * ============================================================================ */
static uint8_t modbusSlaveAddr = MODBUS_SLAVE_ADDRESS;
static uint8_t rxBuffer[MODBUS_MAX_FRAME_SIZE];
static uint16_t rxIndex = 0;
static uint32_t lastRxTime = 0;

// Modbus registers và statistics (global để các task khác truy cập)
ModbusRegs_t modbusRegs = {0};
ModbusStats_t modbusStats = {0};

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================================================ */
static void Modbus_HandleReadHoldingRegisters(uint8_t *frame, uint16_t length);
static void Modbus_HandleReadInputRegisters(uint8_t *frame, uint16_t length);
static void Modbus_HandleWriteSingleRegister(uint8_t *frame, uint16_t length);
static void Modbus_HandleWriteMultipleRegisters(uint8_t *frame, uint16_t length);
static uint16_t* Modbus_GetRegisterPointer(uint16_t regAddr);
static int Modbus_IsRegisterWritable(uint16_t regAddr);

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

/**
 * @brief Khởi tạo Modbus RTU slave
 */
void Modbus_Init()
{
    modbusSlaveAddr = MODBUS_SLAVE_ADDRESS;  // Khởi tạo slave address
    rxIndex = 0;
    lastRxTime = 0;
    
    // Khởi tạo giá trị mặc định cho một số registers
    modbusRegs.deviceId = MODBUS_SLAVE_ADDRESS;
    modbusRegs.moduleType = 0x0007;         // Cargo Control Module
    modbusRegs.firmwareVersion = 0x0100;    // v1.00
    modbusRegs.hardwareVersion = 0x0100;    // v1.00
    modbusRegs.configBaudrate = 7;          // 115200 baud (7=115200)
    modbusRegs.configParity = 0;            // None
    modbusRegs.configStopBits = 1;          // 1 stop bit
    
    // Giá trị mặc định cho motor
    modbusRegs.motor1MaxSpeed = 1000;       // 1000 RPM
    modbusRegs.motor2MaxSpeed = 1000;
    modbusRegs.motor1Acceleration = 100;    // 100 RPM/s
    modbusRegs.motor2Acceleration = 100;
    modbusRegs.motor1Deceleration = 100;
    modbusRegs.motor2Deceleration = 100;
    
    // PID mặc định (x1000)
    modbusRegs.pidKp = 1000;    // Kp = 1.0
    modbusRegs.pidKi = 100;     // Ki = 0.1
    modbusRegs.pidKd = 50;      // Kd = 0.05
    
    // Safety timeout
    modbusRegs.safetyTimeout = 30;  // 30 seconds
    
    // Bật UART interrupt để nhận dữ liệu
    HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1);
}

/**
 * @brief Task xử lý Modbus (polling và xử lý frame)
 */
void Modbus_Task(void *argument)
{
    uint32_t currentTime;
    
    for(;;)
    {
        currentTime = HAL_GetTick();
        
        // Kiểm tra timeout giữa các byte (T3.5)
        if (rxIndex > 0 && (currentTime - lastRxTime) >= MODBUS_T35_TIMEOUT_MS)
        {
            // Đã nhận đủ frame, xử lý
            if (rxIndex >= 4)  // Tối thiểu: SlaveAddr + FuncCode + CRC (4 bytes)
            {
                Modbus_ProcessFrame(rxBuffer, rxIndex);
                modbusStats.rxFrames++;
                // Lưu ý: Modbus_SendResponse() sẽ tự restart RX interrupt
            }
            else
            {
                // Frame quá ngắn, có thể là noise
                modbusStats.timeoutErrors++;
                
                // Reset buffer và restart RX
                rxIndex = 0;
                HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1);
            }
        }
        
        // Cập nhật system uptime (mỗi giây)
        static uint32_t lastUptimeUpdate = 0;
        if (currentTime - lastUptimeUpdate >= 1000)
        {
            lastUptimeUpdate = currentTime;
            uint32_t uptime = currentTime / 1000;
            
            // Thread-safe update
            if (osMutexAcquire(modbusMutexHandle, osWaitForever) == osOK)
            {
                modbusRegs.systemUptimeHi = (uint16_t)(uptime >> 16);
                modbusRegs.systemUptimeLo = (uint16_t)(uptime & 0xFFFF);
                osMutexRelease(modbusMutexHandle);
            }
        }
        
        osDelay(1);  // 1ms delay
    }
}

/**
 * @brief Xử lý frame Modbus
 */
void Modbus_ProcessFrame(uint8_t *frame, uint16_t length)
{
    uint16_t receivedCRC, calculatedCRC;
    uint8_t slaveAddr, functionCode;
    
    // Kiểm tra địa chỉ slave
    slaveAddr = frame[0];
    if (slaveAddr != modbusSlaveAddr && slaveAddr != 0)  // 0 = broadcast
    {
        return;  // Không phải cho mình
    }
    
    // Kiểm tra CRC
    receivedCRC = (frame[length - 1] << 8) | frame[length - 2];
    calculatedCRC = Modbus_CRC16(frame, length - 2);
    
    if (receivedCRC != calculatedCRC)
    {
        modbusStats.crcErrors++;
        modbusStats.lastErrorCode = 0xFFFF;  // CRC error code
        return;  // CRC sai, bỏ qua
    }
    
    // Xử lý theo function code
    functionCode = frame[1];
    
    switch (functionCode)
    {
        case MODBUS_FC_READ_HOLDING_REGS:
            Modbus_HandleReadHoldingRegisters(frame, length);
            break;
            
        case MODBUS_FC_READ_INPUT_REGS:
            Modbus_HandleReadInputRegisters(frame, length);
            break;
            
        case MODBUS_FC_WRITE_SINGLE_REG:
            Modbus_HandleWriteSingleRegister(frame, length);
            break;
            
        case MODBUS_FC_WRITE_MULTIPLE_REGS:
            Modbus_HandleWriteMultipleRegisters(frame, length);
            break;
            
        default:
            // Function code không hỗ trợ
            Modbus_SendException(functionCode, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            break;
    }
}

/**
 * @brief Xử lý Read Holding Registers (0x03)
 */
static void Modbus_HandleReadHoldingRegisters(uint8_t *frame, uint16_t length)
{
    uint16_t startAddr, numRegs;
    uint8_t response[MODBUS_MAX_FRAME_SIZE];
    uint16_t responseLen = 0;
    uint16_t i, regValue;
    uint16_t crc;
    
    // Parse request
    startAddr = (frame[2] << 8) | frame[3];
    numRegs = (frame[4] << 8) | frame[5];
    
    // Kiểm tra số lượng registers
    if (numRegs == 0 || numRegs > 125)
    {
        Modbus_SendException(MODBUS_FC_READ_HOLDING_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    
    // Tạo response
    response[0] = modbusSlaveAddr;
    response[1] = MODBUS_FC_READ_HOLDING_REGS;
    response[2] = numRegs * 2;  // Byte count
    responseLen = 3;
    
    // Đọc registers (thread-safe)
    if (osMutexAcquire(modbusMutexHandle, 100) == osOK)
    {
        for (i = 0; i < numRegs; i++)
        {
            if (Modbus_ReadRegister(startAddr + i, &regValue) != 0)
            {
                osMutexRelease(modbusMutexHandle);
                Modbus_SendException(MODBUS_FC_READ_HOLDING_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
                return;
            }
            
            response[responseLen++] = (regValue >> 8) & 0xFF;  // High byte
            response[responseLen++] = regValue & 0xFF;         // Low byte
        }
        osMutexRelease(modbusMutexHandle);
    }
    else
    {
        Modbus_SendException(MODBUS_FC_READ_HOLDING_REGS, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    }
    
    // Thêm CRC
    crc = Modbus_CRC16(response, responseLen);
    response[responseLen++] = crc & 0xFF;        // CRC Low
    response[responseLen++] = (crc >> 8) & 0xFF; // CRC High
    
    // Gửi response
    Modbus_SendResponse(response, responseLen);
}

/**
 * @brief Xử lý Read Input Registers (0x04)
 */
static void Modbus_HandleReadInputRegisters(uint8_t *frame, uint16_t length)
{
    // Giống Read Holding Registers nhưng chỉ đọc (không ghi được)
    uint16_t startAddr, numRegs;
    uint8_t response[MODBUS_MAX_FRAME_SIZE];
    uint16_t responseLen = 0;
    uint16_t i, regValue;
    uint16_t crc;
    
    startAddr = (frame[2] << 8) | frame[3];
    numRegs = (frame[4] << 8) | frame[5];
    
    if (numRegs == 0 || numRegs > 125)
    {
        Modbus_SendException(MODBUS_FC_READ_INPUT_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    
    response[0] = modbusSlaveAddr;
    response[1] = MODBUS_FC_READ_INPUT_REGS;
    response[2] = numRegs * 2;
    responseLen = 3;
    
    if (osMutexAcquire(modbusMutexHandle, 100) == osOK)
    {
        for (i = 0; i < numRegs; i++)
        {
            if (Modbus_ReadRegister(startAddr + i, &regValue) != 0)
            {
                osMutexRelease(modbusMutexHandle);
                Modbus_SendException(MODBUS_FC_READ_INPUT_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
                return;
            }
            
            response[responseLen++] = (regValue >> 8) & 0xFF;
            response[responseLen++] = regValue & 0xFF;
        }
        osMutexRelease(modbusMutexHandle);
    }
    else
    {
        Modbus_SendException(MODBUS_FC_READ_INPUT_REGS, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    }
    
    crc = Modbus_CRC16(response, responseLen);
    response[responseLen++] = crc & 0xFF;
    response[responseLen++] = (crc >> 8) & 0xFF;
    
    Modbus_SendResponse(response, responseLen);
}

/**
 * @brief Xử lý Write Single Register (0x06)
 */
static void Modbus_HandleWriteSingleRegister(uint8_t *frame, uint16_t length)
{
    uint16_t regAddr, regValue;
    uint8_t response[MODBUS_MAX_FRAME_SIZE];
    
    regAddr = (frame[2] << 8) | frame[3];
    regValue = (frame[4] << 8) | frame[5];
    
    // Kiểm tra xem register có ghi được không
    if (!Modbus_IsRegisterWritable(regAddr))
    {
        Modbus_SendException(MODBUS_FC_WRITE_SINGLE_REG, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    // Ghi register (thread-safe)
    if (osMutexAcquire(modbusMutexHandle, 100) == osOK)
    {
        if (Modbus_WriteRegister(regAddr, regValue) != 0)
        {
            osMutexRelease(modbusMutexHandle);
            Modbus_SendException(MODBUS_FC_WRITE_SINGLE_REG, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
            return;
        }
        osMutexRelease(modbusMutexHandle);
    }
    else
    {
        Modbus_SendException(MODBUS_FC_WRITE_SINGLE_REG, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    }
    
    // Callback để task khác xử lý
    Modbus_OnRegisterWrite(regAddr, regValue);
    
    // Echo response (giống request)
    memcpy(response, frame, length);
    
    // Gửi response
    Modbus_SendResponse(response, length);
}

/**
 * @brief Xử lý Write Multiple Registers (0x10)
 */
static void Modbus_HandleWriteMultipleRegisters(uint8_t *frame, uint16_t length)
{
    uint16_t startAddr, numRegs, byteCount;
    uint8_t response[MODBUS_MAX_FRAME_SIZE];
    uint16_t responseLen = 0;
    uint16_t i, regValue;
    uint16_t crc;
    
    startAddr = (frame[2] << 8) | frame[3];
    numRegs = (frame[4] << 8) | frame[5];
    byteCount = frame[6];
    
    // Kiểm tra
    if (numRegs == 0 || numRegs > 123 || byteCount != numRegs * 2)
    {
        Modbus_SendException(MODBUS_FC_WRITE_MULTIPLE_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return;
    }
    
    // Ghi registers (thread-safe)
    if (osMutexAcquire(modbusMutexHandle, 100) == osOK)
    {
        for (i = 0; i < numRegs; i++)
        {
            if (!Modbus_IsRegisterWritable(startAddr + i))
            {
                osMutexRelease(modbusMutexHandle);
                Modbus_SendException(MODBUS_FC_WRITE_MULTIPLE_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
                return;
            }
            
            regValue = (frame[7 + i * 2] << 8) | frame[7 + i * 2 + 1];
            
            if (Modbus_WriteRegister(startAddr + i, regValue) != 0)
            {
                osMutexRelease(modbusMutexHandle);
                Modbus_SendException(MODBUS_FC_WRITE_MULTIPLE_REGS, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS);
                return;
            }
            
            // Callback
            Modbus_OnRegisterWrite(startAddr + i, regValue);
        }
        osMutexRelease(modbusMutexHandle);
    }
    else
    {
        Modbus_SendException(MODBUS_FC_WRITE_MULTIPLE_REGS, MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE);
        return;
    }
    
    // Tạo response
    response[0] = modbusSlaveAddr;
    response[1] = MODBUS_FC_WRITE_MULTIPLE_REGS;
    response[2] = (startAddr >> 8) & 0xFF;
    response[3] = startAddr & 0xFF;
    response[4] = (numRegs >> 8) & 0xFF;
    response[5] = numRegs & 0xFF;
    responseLen = 6;
    
    crc = Modbus_CRC16(response, responseLen);
    response[responseLen++] = crc & 0xFF;
    response[responseLen++] = (crc >> 8) & 0xFF;
    
    Modbus_SendResponse(response, responseLen);
}

/**
 * @brief Gửi exception response
 */
void Modbus_SendException(uint8_t functionCode, uint8_t exceptionCode)
{
    uint8_t response[5];
    uint16_t crc;
    
    response[0] = modbusSlaveAddr;
    response[1] = functionCode | 0x80;  // Set bit 7
    response[2] = exceptionCode;
    
    crc = Modbus_CRC16(response, 3);
    response[3] = crc & 0xFF;
    response[4] = (crc >> 8) & 0xFF;
    
    modbusStats.exceptionsSent++;
    modbusStats.lastErrorCode = exceptionCode;
    
    Modbus_SendResponse(response, 5);
}

/**
 * @brief Gửi response qua UART2
 */
void Modbus_SendResponse(uint8_t *frame, uint16_t length)
{
    HAL_StatusTypeDef status;
    
    // Abort RX interrupt trước khi TX (quan trọng!)
    HAL_UART_AbortReceive_IT(&huart2);
    
    // Gửi dữ liệu (blocking mode)
    status = HAL_UART_Transmit(&huart2, frame, length, 100);
    
    if (status == HAL_OK)
    {
        modbusStats.txFrames++;
    }
    else
    {
        // TX failed
        modbusStats.lastErrorCode = 0xEEEE;  // TX error
    }
    
    // Restart RX interrupt sau khi TX xong
    rxIndex = 0;
    HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1);
}

/**
 * @brief Đọc register (không lock mutex - gọi từ bên trong mutex)
 */
int Modbus_ReadRegister(uint16_t regAddr, uint16_t *value)
{
    uint16_t *regPtr = Modbus_GetRegisterPointer(regAddr);
    
    if (regPtr == NULL)
    {
        return -1;  // Địa chỉ không hợp lệ
    }
    
    *value = *regPtr;
    return 0;
}

/**
 * @brief Ghi register (không lock mutex - gọi từ bên trong mutex)
 */
int Modbus_WriteRegister(uint16_t regAddr, uint16_t value)
{
    uint16_t *regPtr = Modbus_GetRegisterPointer(regAddr);
    
    if (regPtr == NULL)
    {
        return -1;  // Địa chỉ không hợp lệ
    }
    
    *regPtr = value;
    return 0;
}

/**
 * @brief Lấy con trỏ đến register theo địa chỉ
 */
static uint16_t* Modbus_GetRegisterPointer(uint16_t regAddr)
{
    switch (regAddr)
    {
        // Door States (0x0000-0x0003)
        case REG_DOOR_STATE:                return &modbusRegs.doorState;
        case REG_DOOR_POSITION_PERCENT:     return &modbusRegs.doorPositionPercent;
        case REG_DOOR_TARGET_POSITION:      return &modbusRegs.doorTargetPosition;
        case REG_SENSOR_STATES:             return &modbusRegs.sensorStates;
        
        // Object Detection (0x0004)
        case REG_OBJECT_DETECTION:          return &modbusRegs.objectDetection;
        
        // Motor 1 (0x0005-0x000C)
        case REG_MOTOR1_POSITION_ABS:       return &modbusRegs.motor1PositionAbs;
        case REG_MOTOR1_POSITION_REL:       return &modbusRegs.motor1PositionRel;
        case REG_MOTOR1_CURRENT_SPEED:      return &modbusRegs.motor1CurrentSpeed;
        case REG_MOTOR1_TARGET_SPEED:       return &modbusRegs.motor1TargetSpeed;
        case REG_MOTOR1_PWM_DUTY:           return &modbusRegs.motor1PwmDuty;
        case REG_MOTOR1_MAX_SPEED:          return &modbusRegs.motor1MaxSpeed;
        case REG_MOTOR1_ACCELERATION:       return &modbusRegs.motor1Acceleration;
        case REG_MOTOR1_DECELERATION:       return &modbusRegs.motor1Deceleration;
        
        // Motor 2 (0x000D-0x0014)
        case REG_MOTOR2_POSITION_ABS:       return &modbusRegs.motor2PositionAbs;
        case REG_MOTOR2_POSITION_REL:       return &modbusRegs.motor2PositionRel;
        case REG_MOTOR2_CURRENT_SPEED:      return &modbusRegs.motor2CurrentSpeed;
        case REG_MOTOR2_TARGET_SPEED:       return &modbusRegs.motor2TargetSpeed;
        case REG_MOTOR2_PWM_DUTY:           return &modbusRegs.motor2PwmDuty;
        case REG_MOTOR2_MAX_SPEED:          return &modbusRegs.motor2MaxSpeed;
        case REG_MOTOR2_ACCELERATION:       return &modbusRegs.motor2Acceleration;
        case REG_MOTOR2_DECELERATION:       return &modbusRegs.motor2Deceleration;
        
        // PID (0x0015-0x0017)
        case REG_PID_KP:                    return &modbusRegs.pidKp;
        case REG_PID_KI:                    return &modbusRegs.pidKi;
        case REG_PID_KD:                    return &modbusRegs.pidKd;
        
        // Sensors (0x0018-0x001B)
        case REG_PROXIMITY_1_STATE:         return &modbusRegs.proximity1State;
        case REG_PROXIMITY_2_STATE:         return &modbusRegs.proximity2State;
        case REG_PROXIMITY_3_STATE:         return &modbusRegs.proximity3State;
        case REG_PROXIMITY_4_STATE:         return &modbusRegs.proximity4State;
        
        // Encoders (0x001C-0x001F)
        case REG_MOTOR1_ENCODER_COUNT:      return &modbusRegs.motor1EncoderCount;
        case REG_MOTOR2_ENCODER_COUNT:      return &modbusRegs.motor2EncoderCount;
        case REG_MOTOR1_ENCODER_ERRORS:     return &modbusRegs.motor1EncoderErrors;
        case REG_MOTOR2_ENCODER_ERRORS:     return &modbusRegs.motor2EncoderErrors;
        
        // Fault Relay (0x0020)
        case REG_FAULT_RL_STATUS:           return &modbusRegs.faultRelayStatus;
        
        // System Info (0x0021-0x0025)
        case REG_SYSTEM_UPTIME_HI:          return &modbusRegs.systemUptimeHi;
        case REG_SYSTEM_UPTIME_LO:          return &modbusRegs.systemUptimeLo;
        case REG_CPU_USAGE:                 return &modbusRegs.cpuUsage;
        case REG_FREE_MEMORY:               return &modbusRegs.freeMemory;
        case REG_TASK_COUNT:                return &modbusRegs.taskCount;
        
        // Communication Status (0x0026-0x0028)
        case REG_ERROR_CODE:                return &modbusRegs.errorCode;
        case REG_FAULT_FLAGS:               return &modbusRegs.faultFlags;
        case REG_WARNING_FLAGS:             return &modbusRegs.warningFlags;
        
        // System Config (0x0029-0x002C)
        case REG_SYSTEM_MODE:               return &modbusRegs.systemMode;
        case REG_AUTO_CALIBRATE:            return &modbusRegs.autoCalibrate;
        case REG_SAFETY_TIMEOUT:            return &modbusRegs.safetyTimeout;
        case REG_EMERGENCY_STOP:            return &modbusRegs.emergencyStop;
        
        // Position Limits (0x002D-0x0032)
        case REG_DOOR_OPEN_LIMIT:           return &modbusRegs.doorOpenLimit;
        case REG_DOOR_CLOSE_LIMIT:          return &modbusRegs.doorCloseLimit;
        case REG_MOTOR1_SOFT_LIMIT_MIN:     return &modbusRegs.motor1SoftLimitMin;
        case REG_MOTOR1_SOFT_LIMIT_MAX:     return &modbusRegs.motor1SoftLimitMax;
        case REG_MOTOR2_SOFT_LIMIT_MIN:     return &modbusRegs.motor2SoftLimitMin;
        case REG_MOTOR2_SOFT_LIMIT_MAX:     return &modbusRegs.motor2SoftLimitMax;
        
        // Calibration (0x0033-0x003B)
        case REG_CALIBRATE_MOTOR1_HOME:     return &modbusRegs.calibrateMotor1Home;
        case REG_CALIBRATE_MOTOR2_HOME:     return &modbusRegs.calibrateMotor2Home;
        case REG_CALIBRATE_DOOR_OPEN_POS:   return &modbusRegs.calibrateDoorOpenPos;
        case REG_CALIBRATE_DOOR_CLOSE_POS:  return &modbusRegs.calibrateDoorClosePos;
        case REG_START_CALIBRATION:         return &modbusRegs.startCalibration;
        case REG_STOP_CALIBRATION:          return &modbusRegs.stopCalibration;
        case REG_CALIBRATION_STATUS:        return &modbusRegs.calibrationStatus;
        case REG_CALIBRATION_ERROR:         return &modbusRegs.calibrationError;
        case REG_RESERVED_3B:               return &modbusRegs.reserved3B;
        
        // System Registers (0x0100-0x010A)
        case REG_DEVICE_ID:                 return &modbusRegs.deviceId;
        case REG_CONFIG_BAUDRATE:           return &modbusRegs.configBaudrate;
        case REG_CONFIG_PARITY:             return &modbusRegs.configParity;
        case REG_CONFIG_STOP_BITS:          return &modbusRegs.configStopBits;
        case REG_MODULE_TYPE:               return &modbusRegs.moduleType;
        case REG_FIRMWARE_VERSION:          return &modbusRegs.firmwareVersion;
        case REG_HARDWARE_VERSION:          return &modbusRegs.hardwareVersion;
        case REG_SYSTEM_STATUS:             return &modbusRegs.systemStatus;
        case REG_SYSTEM_ERROR:              return &modbusRegs.systemError;
        case REG_RESET_ERROR_CMD:           return &modbusRegs.resetErrorCmd;
        case REG_RESET_SYSTEM:              return &modbusRegs.resetSystem;
        
        default:
            return NULL;  // Địa chỉ không hợp lệ
    }
}

/**
 * @brief Kiểm tra register có ghi được không
 */
static int Modbus_IsRegisterWritable(uint16_t regAddr)
{
    // Các register READ-ONLY (status, sensors, encoders, system info)
    switch (regAddr)
    {
        // Read-only registers
        case REG_DOOR_STATE:
        case REG_DOOR_POSITION_PERCENT:
        case REG_SENSOR_STATES:
        case REG_OBJECT_DETECTION:
        case REG_MOTOR1_POSITION_ABS:
        case REG_MOTOR1_POSITION_REL:
        case REG_MOTOR1_CURRENT_SPEED:
        case REG_MOTOR1_PWM_DUTY:
        case REG_MOTOR2_POSITION_ABS:
        case REG_MOTOR2_POSITION_REL:
        case REG_MOTOR2_CURRENT_SPEED:
        case REG_MOTOR2_PWM_DUTY:
        case REG_PROXIMITY_1_STATE:
        case REG_PROXIMITY_2_STATE:
        case REG_PROXIMITY_3_STATE:
        case REG_PROXIMITY_4_STATE:
        case REG_MOTOR1_ENCODER_COUNT:
        case REG_MOTOR2_ENCODER_COUNT:
        case REG_MOTOR1_ENCODER_ERRORS:
        case REG_MOTOR2_ENCODER_ERRORS:
        case REG_FAULT_RL_STATUS:
        case REG_SYSTEM_UPTIME_HI:
        case REG_SYSTEM_UPTIME_LO:
        case REG_CPU_USAGE:
        case REG_FREE_MEMORY:
        case REG_TASK_COUNT:
        case REG_ERROR_CODE:
        case REG_FAULT_FLAGS:
        case REG_WARNING_FLAGS:
        case REG_CALIBRATION_STATUS:
        case REG_CALIBRATION_ERROR:
        case REG_MODULE_TYPE:
        case REG_FIRMWARE_VERSION:
        case REG_HARDWARE_VERSION:
        case REG_SYSTEM_STATUS:
        case REG_SYSTEM_ERROR:
            return 0;  // Read-only
        
        default:
            return 1;  // Writable
    }
}

/**
 * @brief Tính CRC16 Modbus
 */
uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    
    for (i = 0; i < length; i++)
    {
        crc ^= buffer[i];
        
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Callback khi register được ghi (weak function - có thể override)
 */
__weak void Modbus_OnRegisterWrite(uint16_t regAddr, uint16_t value)
{
    // Xử lý các command registers đặc biệt
    switch (regAddr)
    {
        case REG_EMERGENCY_STOP:
            if (value == 1)
            {
                // Trigger emergency stop
                // TODO: Gửi signal đến motor task
            }
            break;
            
        case REG_START_CALIBRATION:
            if (value == 1)
            {
                // Start calibration
                // TODO: Gửi signal đến motor task
            }
            break;
            
        case REG_STOP_CALIBRATION:
            if (value == 1)
            {
                // Stop calibration
                // TODO: Gửi signal đến motor task
            }
            break;
            
        case REG_RESET_ERROR_CMD:
            if (value == 1)
            {
                // Reset error flags
                modbusRegs.errorCode = 0;
                modbusRegs.faultFlags = 0;
                modbusRegs.warningFlags = 0;
                modbusRegs.resetErrorCmd = 0;
            }
            break;
            
        case REG_RESET_SYSTEM:
            if (value == 1)
            {
                // System reset
                // TODO: Implement system reset
            }
            break;
            
        default:
            // Các register khác - có thể xử lý bằng queue hoặc notification
            break;
    }
}

/**
 * @brief UART RX Interrupt Callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        lastRxTime = HAL_GetTick();
        rxIndex++;
        
        if (rxIndex < MODBUS_MAX_FRAME_SIZE)
        {
            // Tiếp tục nhận byte tiếp theo
            HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1);
        }
        else
        {
            // Buffer full, reset
            rxIndex = 0;
            HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1);
        }
    }
}