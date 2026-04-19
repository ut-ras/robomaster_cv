# Firmware Communication
Messages will be sent continuously in the format that follows. The message begins with a start byte encoded with 70 (01000110), signaling the start of the message stream for that frame. Then the body of the message is n bytes which follows a predetermined format. Last an end byte encoded with 57 (00111001) is sent, signaling the end of the message stream for that frame.
## General Message Format
```
byte start_byte

byte0...byte_n 

byte end_byte
```
## Message To Receive From Firmware
The byte order of the message we reviece from firmware
```
byte start_byte

byte odom_x_pos_0
byte odom_x_pos_1
byte odom_x_pos_2
byte odom_x_pos_3

byte odom_y_pos_0
byte odom_y_pos_1
byte odom_y_pos_2
byte odom_y_pos_3

byte health_0
byte health_1
byte health_2
byte health_3

byte color

byte end_byte
```

## Message To Send Firmware

```
byte start_byte

byte desired_x_pos_0
byte desired_x_pos_1
byte desired_x_pos_2
byte desired_x_pos_3

byte desired_y_pos_0
byte desired_y_pos_1
byte desired_y_pos_2
byte desired_y_pos_3

byte enemy_target_x_pos_0
byte enemy_target_x_pos_1
byte enemy_target_x_pos_2
byte enemy_target_x_pos_3

byte enemy_target_y_pos_0
byte enemy_target_y_pos_1
byte enemy_target_y_pos_2
byte enemy_target_y_pos_3

byte enemy_target_z_pos_0
byte enemy_target_z_pos_1
byte enemy_target_z_pos_2
byte enemy_target_z_pos_3

byte real_target

byte current_x_pos_0
byte current_x_pos_1
byte current_x_pos_2
byte current_x_pos_3

byte current_y_pos_0
byte current_y_pos_1
byte current_y_pos_2
byte current_y_pos_3

byte is_valid 

byte end_byte
```

### Health
The current health of the robot and if it died or not.
```
std_msgs/Header header

float32 health
```

### OdometryData
The current location of the robot chassis in the world frame.
```
std_msgs/Header header

float32 x_pos
float32 y_pos
```

### Color
The current color of the robot.
```
bool color
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