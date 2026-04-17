# Firmware Communication
Messages will be sent continuously in the format that follows. The message begins with a start byte encoded with 70 (01000110), signaling the start of the message stream for that frame. Then the body of the message is n bytes which follows a predetermined format. Last an end byte encoded with 57 (00111001) is sent, signaling the end of the message stream for that frame.
## General Message Format
```
byte start_byte

byte0...byte_n 

byte end_byte
```
## Message To Receive From Firmware

```
byte start_byte


byte end_byte
```

### Health
The current health of the robot and if it died or not.
```
std_msgs/Header header

float32 health
bool is_dead
```

### OdometryData
The current location of the robot chassis in the world frame.
```
std_msgs/Header header

float32 x_pos
float32 y_pos
```

## Messages To Send Firmware

### EnemyTarget
The current target we want to shoot at in the camera frame.
```
std_msgs/Header header

float32 x_pos
float32 y_pos
float32 z_pos
bool real_target
```

### DesiredPosition
The current position we want the chassis to move to in the world frame.
```
std_msgs/Header header

float32 x_pos
float32 y_pos

```

### CurrentPosition
The current position of the robot chassis in the world frame.
```
std_msgs/Header header

float32 x_pos
float32 y_pos
```