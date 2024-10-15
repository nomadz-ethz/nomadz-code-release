std_msgs/Header header

# An array of joint positions, corresponding to their indexes in the JointIndexes.msg.
float32[25] positions # radians

# Temperature status enums, computed accordingly to the temperature limitation to protect the motors.
int32 STATUS_NORMAL=0          # normal
int32 STATUS_HOT=1             # high, start to reduce stiffness
int32 STATUS_VERY_HOT=2        # very hot, stiffness reduced over 30%
int32 STATUS_CRITICALLY_HOT=3  # critically hot, stiffness is set to 0

int32[25] statuses  # Status codes, in order of JointIndexes.msg

float32[25] stiffnesses  # 0.0 - 1.0

float32[25] currents  # Amperes (A), in order of JointIndexes.msg

float32[25] temperatures  # Celcius (°C), in order of JointIndexes.msg
