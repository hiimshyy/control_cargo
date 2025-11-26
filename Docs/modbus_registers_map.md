# Modbus Registers Map - Cargo Control System

## Overview
This document describes the Modbus RTU register map for the Cargo Control System. The system uses Modbus RTU protocol to communicate and control 2 motors, sensors, and safety functions.

## Register Structure

### 1. System Registers (0x0100-0x010A)
System registers and Modbus configuration

| Address | Name | Description | Data Type | Values / Notes | Default | Access |
|---------|------|-------------|-----------|----------------|---------|--------|
| 0x0100 | `REG_DEVICE_ID` | Modbus slave address | uint16 | 1-247 | 1 | R/W |
| 0x0101 | `REG_CONFIG_BAUDRATE` | Baudrate configuration | uint16 | 1=9600, 2=19200, 3=38400, 4=57600, 5=115200 | 5 (115200) | R/W |
| 0x0102 | `REG_CONFIG_PARITY` | Parity configuration | uint16 | 0=None, 1=Even, 2=Odd | 0 (None) | R/W |
| 0x0103 | `REG_CONFIG_STOP_BITS` | Stop bits configuration | uint16 | 1=1 bit, 2=2 bits | 1 | R/W |
| 0x0104 | `REG_MODULE_TYPE` | Module type identifier | uint16 | 0x0007 = Cargo Module | 0x0007 | R |
| 0x0105 | `REG_FIRMWARE_VERSION` | Firmware version | uint16 | 0xMMmm (Major.minor, e.g. 0x0101 = v1.01) | - | R |
| 0x0106 | `REG_HARDWARE_VERSION` | Hardware version | uint16 | 0xMMmm (Major.minor, e.g. 0x0101 = v1.01) | - | R |
| 0x0107 | `REG_SYSTEM_STATUS` | System status flags | uint16 | Bit field (see bitmap section) | 0x0000 | R |
| 0x0108 | `REG_SYSTEM_ERROR` | Global system error code | uint16 | Error code (0 = No error) | 0 | R |
| 0x0109 | `REG_RESET_ERROR_CMD` | Reset error command | uint16 | Write 1 to reset all error flags | 0 | W |
| 0x010A | `REG_RESET_SYSTEM` | Reset system command | uint16 | Write 1 to reset system | 0 | W |

### 2. Door States Registers (0x0000-0x0003)
Door state registers

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0000 | `REG_DOOR_STATE` | - | Door state | uint16 | 0=Closed, 1=Opening, 2=Open, 3=Closing, 4=Stopped, 5=Error | R |
| 0x0001 | `REG_DOOR_POSITION_PERCENT` | % | Current door position | uint16 | 0-100 (%) | R |
| 0x0002 | `REG_DOOR_TARGET_POSITION` | % | Target door position | uint16 | 0-100 (%) | R/W |
| 0x0003 | `REG_SENSOR_STATES` | - | Sensor states bitmap | uint16 | Bit field | R |

### 3. Object Detection (0x0004)
Object detection

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0004 | `REG_OBJECT_DETECTION` | - | Object detection status | uint16 | 0=No object, 1=Object detected | R |

### 4. Motor 1 Status (0x0005-0x000C)
Motor 1 status and control

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0005 | `REG_MOTOR1_POSITION_ABS` | counts | Motor 1 absolute position | int16 | 1 count | R |
| 0x0006 | `REG_MOTOR1_POSITION_REL` | counts | Motor 1 relative position | int16 | 1 count | R |
| 0x0007 | `REG_MOTOR1_CURRENT_SPEED` | RPM | Motor 1 current speed | int16 | 1 RPM | R |
| 0x0008 | `REG_MOTOR1_TARGET_SPEED` | RPM | Motor 1 target speed | int16 | 1 RPM | R/W |
| 0x0009 | `REG_MOTOR1_PWM_DUTY` | 0.1% | Motor 1 PWM duty cycle | uint16 | 0-1000 (0-100%) | R |
| 0x000A | `REG_MOTOR1_MAX_SPEED` | RPM | Motor 1 maximum speed | uint16 | 1 RPM | R/W |
| 0x000B | `REG_MOTOR1_ACCELERATION` | RPM/s | Motor 1 acceleration | uint16 | 1 RPM/s | R/W |
| 0x000C | `REG_MOTOR1_DECELERATION` | RPM/s | Motor 1 deceleration | uint16 | 1 RPM/s | R/W |

### 5. Motor 2 Status (0x000D-0x0014)
Motor 2 status and control

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x000D | `REG_MOTOR2_POSITION_ABS` | counts | Motor 2 absolute position | int16 | 1 count | R |
| 0x000E | `REG_MOTOR2_POSITION_REL` | counts | Motor 2 relative position | int16 | 1 count | R |
| 0x000F | `REG_MOTOR2_CURRENT_SPEED` | RPM | Motor 2 current speed | int16 | 1 RPM | R |
| 0x0010 | `REG_MOTOR2_TARGET_SPEED` | RPM | Motor 2 target speed | int16 | 1 RPM | R/W |
| 0x0011 | `REG_MOTOR2_PWM_DUTY` | 0.1% | Motor 2 PWM duty cycle | uint16 | 0-1000 (0-100%) | R |
| 0x0012 | `REG_MOTOR2_MAX_SPEED` | RPM | Motor 2 maximum speed | uint16 | 1 RPM | R/W |
| 0x0013 | `REG_MOTOR2_ACCELERATION` | RPM/s | Motor 2 acceleration | uint16 | 1 RPM/s | R/W |
| 0x0014 | `REG_MOTOR2_DECELERATION` | RPM/s | Motor 2 deceleration | uint16 | 1 RPM/s | R/W |

### 6. PID Control Parameters (0x0015-0x0017)
PID control parameters

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0015 | `REG_PID_KP` | - | PID proportional gain | uint16 | × 0.001 (e.g. 1500 = 1.5) | R/W |
| 0x0016 | `REG_PID_KI` | - | PID integral gain | uint16 | × 0.001 (e.g. 500 = 0.5) | R/W |
| 0x0017 | `REG_PID_KD` | - | PID derivative gain | uint16 | × 0.001 (e.g. 100 = 0.1) | R/W |

### 7. Sensor Values (0x0018-0x001B)
Proximity sensor values

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0018 | `REG_PROXIMITY_1_STATE` | - | Proximity sensor 1 state | uint16 | 0=Inactive, 1=Active | R |
| 0x0019 | `REG_PROXIMITY_2_STATE` | - | Proximity sensor 2 state | uint16 | 0=Inactive, 1=Active | R |
| 0x001A | `REG_PROXIMITY_3_STATE` | - | Proximity sensor 3 state | uint16 | 0=Inactive, 1=Active | R |
| 0x001B | `REG_PROXIMITY_4_STATE` | - | Proximity sensor 4 state | uint16 | 0=Inactive, 1=Active | R |

### 8. Encoder Status (0x001C-0x001F)
Encoder status

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x001C | `REG_MOTOR1_ENCODER_COUNT` | counts | Motor 1 raw encoder count | uint16 | 1 count | R |
| 0x001D | `REG_MOTOR2_ENCODER_COUNT` | counts | Motor 2 raw encoder count | uint16 | 1 count | R |
| 0x001E | `REG_MOTOR1_ENCODER_ERRORS` | - | Motor 1 encoder error count | uint16 | 1 error | R |
| 0x001F | `REG_MOTOR2_ENCODER_ERRORS` | - | Motor 2 encoder error count | uint16 | 1 error | R |

### 9. Fault Relay Status (0x0020)
Fault relay status

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0020 | `REG_FAULT_RL_STATUS` | - | Fault relay status | uint16 | 0=Normal, 1=Fault | R |

### 10. System Information (0x0021-0x0025)
System information

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0021 | `REG_SYSTEM_UPTIME_HI` | s | System uptime - high word | uint16 | 1 second (upper 16 bits) | R |
| 0x0022 | `REG_SYSTEM_UPTIME_LO` | s | System uptime - low word | uint16 | 1 second (lower 16 bits) | R |
| 0x0023 | `REG_CPU_USAGE` | % | CPU usage | uint16 | 0-100 (%) | R |
| 0x0024 | `REG_FREE_MEMORY` | bytes | Free memory | uint16 | 1 byte | R |
| 0x0025 | `REG_TASK_COUNT` | - | Active task count | uint16 | 1 task | R |

### 11. Communication Status (0x0026-0x0028)
Communication status and errors

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0026 | `REG_ERROR_CODE` | - | Current error code | uint16 | Error code value | R |
| 0x0027 | `REG_FAULT_FLAGS` | - | Fault flags bitmap | uint16 | Bit field | R |
| 0x0028 | `REG_WARNING_FLAGS` | - | Warning flags bitmap | uint16 | Bit field | R |

### 12. System Configuration (0x0029-0x002C)
System configuration

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0029 | `REG_SYSTEM_MODE` | - | Operating mode | uint16 | 0=Manual, 1=Auto, 2=Maintenance | R/W |
| 0x002A | `REG_AUTO_CALIBRATE` | - | Auto-calibration enable | uint16 | 0=Disabled, 1=Enabled | R/W |
| 0x002B | `REG_SAFETY_TIMEOUT` | s | Safety timeout | uint16 | 1 second | R/W |
| 0x002C | `REG_EMERGENCY_STOP` | - | Emergency stop control | uint16 | 0=Normal, 1=Emergency Stop | R/W |

### 13. Position Limits (0x002D-0x0032)
Position limits

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x002D | `REG_DOOR_OPEN_LIMIT` | counts | Door open position limit | int16 | 1 count | R/W |
| 0x002E | `REG_DOOR_CLOSE_LIMIT` | counts | Door close position limit | int16 | 1 count | R/W |
| 0x002F | `REG_MOTOR1_SOFT_LIMIT_MIN` | counts | Motor 1 minimum soft limit | int16 | 1 count | R/W |
| 0x0030 | `REG_MOTOR1_SOFT_LIMIT_MAX` | counts | Motor 1 maximum soft limit | int16 | 1 count | R/W |
| 0x0031 | `REG_MOTOR2_SOFT_LIMIT_MIN` | counts | Motor 2 minimum soft limit | int16 | 1 count | R/W |
| 0x0032 | `REG_MOTOR2_SOFT_LIMIT_MAX` | counts | Motor 2 maximum soft limit | int16 | 1 count | R/W |

### 14. Calibration Data (0x0033-0x003A)
Calibration data

| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| 0x0033 | `REG_CALIBRATE_MOTOR1_HOME` | counts | Motor 1 home position | int16 | 1 count | R/W |
| 0x0034 | `REG_CALIBRATE_MOTOR2_HOME` | counts | Motor 2 home position | int16 | 1 count | R/W |
| 0x0035 | `REG_CALIBRATE_DOOR_OPEN_POS` | counts | Door open calibration position | int16 | 1 count | R/W |
| 0x0036 | `REG_CALIBRATE_DOOR_CLOSE_POS` | counts | Door close calibration position | int16 | 1 count | R/W |
| 0x0037 | `REG_START_CALIBRATION` | - | Start calibration command | uint16 | Write 1 to start | W |
| 0x0038 | `REG_STOP_CALIBRATION` | - | Stop calibration command | uint16 | Write 1 to stop | W |
| 0x0039 | `REG_CALIBRATION_STATUS` | - | Calibration status | uint16 | Bit field | R |
| 0x003A | `REG_CALIBRATION_ERROR` | - | Calibration error code | uint16 | Error code value | R |

## Notes

### Register Types
- **R**: Read Only
- **W**: Write Only
- **R/W**: Read/Write

### Data Format
- All registers are 16-bit (2 bytes)
- PID values are multiplied by 1000 for integer storage
- PWM duty cycle: 0-1000 corresponds to 0-100%
- Door position: 0-100 corresponds to 0-100%

### Bitmap Flags

#### REG_SENSOR_STATES (0x0003)
```
Bit 0: Proximity Sensor 1
Bit 1: Proximity Sensor 2
Bit 2: Proximity Sensor 3
Bit 3: Proximity Sensor 4
Bit 4-15: Reserved
```

#### REG_FAULT_FLAGS (0x0027)
```
Bit 0: Motor 1 Fault
Bit 1: Motor 2 Fault
Bit 2: Encoder Error
Bit 3: Communication Error
Bit 4: Safety Timeout
Bit 5: Emergency Stop Active
Bit 6-15: Reserved
```

#### REG_WARNING_FLAGS (0x0028)
```
Bit 0: High Temperature
Bit 1: Low Voltage
Bit 2: High Current
Bit 3: Position Limit Warning
Bit 4-15: Reserved
```

#### REG_CALIBRATION_STATUS (0x0039)
```
Bit 0: Calibration in progress
Bit 1: Motor 1 calibrated
Bit 2: Motor 2 calibrated
Bit 3: Door open position calibrated
Bit 4: Door close position calibrated
Bit 5-15: Reserved
```

#### REG_SYSTEM_STATUS (0x0107)
```
Bit 0: System Ready
Bit 1: System Error
Bit 2: Calibration Required
Bit 3: Emergency Stop Active
Bit 4: Door Moving
Bit 5-15: Reserved
```

## Usage Examples

### 1. Read door status
```
Function Code: 0x03 (Read Holding Registers)
Starting Address: 0x0000
Quantity: 4
```

### 2. Control door to open 50%
```
Function Code: 0x06 (Write Single Register)
Register Address: 0x0002 (REG_DOOR_TARGET_POSITION)
Value: 50
```

### 3. Configure PID parameters
```
Function Code: 0x10 (Write Multiple Registers)
Starting Address: 0x0015
Quantity: 3
Values: [1500, 500, 100] (Kp=1.5, Ki=0.5, Kd=0.1)
```

### 4. Start calibration
```
Function Code: 0x06 (Write Single Register)
Register Address: 0x0037 (REG_START_CALIBRATION)
Value: 1
```

### 5. Emergency stop
```
Function Code: 0x06 (Write Single Register)
Register Address: 0x002C (REG_EMERGENCY_STOP)
Value: 1
```

## Important Notes

1. **Response time**: Maximum response time is 100ms
2. **Default baudrate**: 115200 bps, 8N1
3. **Default Modbus address**: 1
4. **Timeout**: 1000ms (1 second)
5. **Configuration persistence**: Configuration changes need to be saved to EEPROM/Flash to persist after reset
6. **Safety**: Always check `REG_FAULT_FLAGS` before controlling motors
7. **Calibration**: Calibration must be performed after first boot or when mechanical setup changes

## Version
- **Document Version**: 1.0
- **Date**: November 25, 2025
- **Author**: tiensy
