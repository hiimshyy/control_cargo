/*
 * modbus_regs.h
 *
 *  Created on: Nov 25, 2025
 *      Author: tiensy
 */

#ifndef INC_MODBUS_REGS_H_
#define INC_MODBUS_REGS_H_

// Door States Registers (0x0000-0x0003)
#define REG_DOOR_STATE            		0x0000  // Door state
#define REG_DOOR_POSITION_PERCENT       0x0001  // Door position %
#define REG_DOOR_TARGET_POSITION        0x0002  // Door target position
#define REG_SENSOR_STATES               0x0003  // Sensor states bitmap

// Object Detection (0x0004)
#define REG_OBJECT_DETECTION     		0x0004  // Object detection status

// Motor 1 Status (0x0005-0x000C)
#define REG_MOTOR1_POSITION_ABS         0x0005  // Motor 1 absolute position
#define REG_MOTOR1_POSITION_REL         0x0006  // Motor 1 relative position
#define REG_MOTOR1_CURRENT_SPEED        0x0007  // Motor 1 current speed RPM
#define REG_MOTOR1_TARGET_SPEED         0x0008  // Motor 1 target speed RPM
#define REG_MOTOR1_PWM_DUTY             0x0009  // Motor 1 PWM duty cycle
#define REG_MOTOR1_MAX_SPEED            0x000A  // Motor 1 max speed RPM
#define REG_MOTOR1_ACCELERATION         0x000B  // Motor 1 acceleration (RPM/s)
#define REG_MOTOR1_DECELERATION         0x000C  // Motor 1 deceleration (RPM/s)

// Motor 2 Status (0x000D-0x0014)
#define REG_MOTOR2_POSITION_ABS         0x000D  // Motor 2 absolute position
#define REG_MOTOR2_POSITION_REL         0x000E  // Motor 2 relative position
#define REG_MOTOR2_CURRENT_SPEED        0x000F  // Motor 2 current speed RPM
#define REG_MOTOR2_TARGET_SPEED         0x0010  // Motor 2 target speed RPM
#define REG_MOTOR2_PWM_DUTY             0x0011  // Motor 2 PWM duty cycle
#define REG_MOTOR2_MAX_SPEED            0x0012  // Motor 2 max speed RPM
#define REG_MOTOR2_ACCELERATION         0x0013  // Motor 2 acceleration
#define REG_MOTOR2_DECELERATION         0x0014  // Motor 2 deceleration

// PID Control Parameters (0x0015-0x0017)
#define REG_PID_KP                     	0x0015  // PID Kp × 1000
#define REG_PID_KI                    	0x0016  // PID Ki × 1000
#define REG_PID_KD                     	0x0017  // PID Kd × 1000

// Sensor Values (0x0018-0x001B)
#define REG_PROXIMITY_1_STATE           0x0018  // Proximity sensor 1
#define REG_PROXIMITY_2_STATE           0x0019  // Proximity sensor 2
#define REG_PROXIMITY_3_STATE           0x001A  // Proximity sensor 3
#define REG_PROXIMITY_4_STATE           0x001B  // Proximity sensor 4

// Encoder Status (0x001C-0x001F)
#define REG_MOTOR1_ENCODER_COUNT        0x001C  // Motor 1 raw encoder
#define REG_MOTOR2_ENCODER_COUNT        0x001D  // Motor 2 raw encoder
#define REG_MOTOR1_ENCODER_ERRORS       0x001E  // Motor 1 encoder errors
#define REG_MOTOR2_ENCODER_ERRORS       0x001F  // Motor 2 encoder errors

// Fault Relay status (0x0020)
#define REG_FAULT_RL_STATUS             0x0020  // Fault relay status

// System Information (0x0021-0x0025)
#define REG_SYSTEM_UPTIME_HI            0x0021  // Uptime high word
#define REG_SYSTEM_UPTIME_LO            0x0022  // Uptime low word
#define REG_CPU_USAGE                   0x0023  // CPU usage %
#define REG_FREE_MEMORY                 0x0024  // Free memory bytes
#define REG_TASK_COUNT                  0x0025  // Active task count

// Communication Status (0x0026-0x0028)
#define REG_ERROR_CODE                  0x0026  // Current error code
#define REG_FAULT_FLAGS                 0x0027  // Fault flags bitmap
#define REG_WARNING_FLAGS               0x0028  // Warning flags bitmap

// System Configuration (0x0029-0x002C)
#define REG_SYSTEM_MODE                 0x0029  // System operating mode
#define REG_AUTO_CALIBRATE              0x002A  // Auto-calibration enable
#define REG_SAFETY_TIMEOUT              0x002B  // Safety timeout seconds
#define REG_EMERGENCY_STOP              0x002C  // Emergency stop control

// Position Limits (0x002D-0x0032)
#define REG_DOOR_OPEN_LIMIT             0x002D  // Door open position limit
#define REG_DOOR_CLOSE_LIMIT            0x002E  // Door close position limit
#define REG_MOTOR1_SOFT_LIMIT_MIN       0x002F  // Motor 1 min soft limit
#define REG_MOTOR1_SOFT_LIMIT_MAX       0x0030  // Motor 1 max soft limit
#define REG_MOTOR2_SOFT_LIMIT_MIN       0x0031  // Motor 2 min soft limit
#define REG_MOTOR2_SOFT_LIMIT_MAX       0x0032  // Motor 2 max soft limit

// Calibration Data (0x0033-0x003A)
#define REG_CALIBRATE_MOTOR1_HOME       0x0033  // Motor 1 home position
#define REG_CALIBRATE_MOTOR2_HOME       0x0034  // Motor 2 home position
#define REG_CALIBRATE_DOOR_OPEN_POS     0x0035  // Door open calibration
#define REG_CALIBRATE_DOOR_CLOSE_POS    0x0036  // Door close calibration
#define REG_START_CALIBRATION           0x0037  // Start calibration command (write 1 to start calibration)
#define REG_STOP_CALIBRATION            0x0038  // Stop calibration command (write 1 to stop calibration)
#define REG_CALIBRATION_STATUS          0x0039  // Calibration status (bit field)
#define REG_CALIBRATION_ERROR           0x003A  // Calibration error (global error code)
#define REG_RESERVED_3B                 0x003B  // Reserved (dummy for continuous read)

// System Registers
#define REG_DEVICE_ID         0x0100  // Device ID (Modbus slave address)
#define REG_CONFIG_BAUDRATE   0x0101  // Config baudrate (1=9600, 2=19200, 3=38400,...)
#define REG_CONFIG_PARITY     0x0102  // Config parity (0=None, 1=Even, 2=Odd)
#define REG_CONFIG_STOP_BITS  0x0103  // Config stop bits (1=1, 2=2)
#define REG_MODULE_TYPE       0x0104  // Module type (0x0007 = Cargo control Module)
#define REG_FIRMWARE_VERSION  0x0105  // Firmware version (e.g. 0x0101 = v1.01)
#define REG_HARDWARE_VERSION  0x0106  // Hardware version (e.g. 0x0101 = v1.01)
#define REG_SYSTEM_STATUS     0x0107  // System status (bit field)
#define REG_SYSTEM_ERROR      0x0108  // System error (global error code)
#define REG_RESET_ERROR_CMD   0x0109  // Reset error command (write 1 to reset all error flags)
#define REG_RESET_SYSTEM      0x010A  // Reset system command (write 1 to reset all system flags)


#endif /* INC_MODBUS_REGS_H_ */
