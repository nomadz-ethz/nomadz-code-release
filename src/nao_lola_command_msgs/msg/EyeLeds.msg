# Message identifying colors for each RGB LED in the NAO's eyes.
# Expect ranges for R, G and B are 0.0 - 1.0.
# The alpha value (A) is ignored.

# LED locations and their ids are shown below
#                          _                         #
#      /                  (_)                 \      #
#     /                                        \     #
#    /         R0                    L0         \    #
#   |     R7  ____  R1          L1  ____  L7     |   #
#   |        /    \                /    \        |   #
#   |   R6  |  **  |  R2      L2  |  **  |  L6   |   #
#   |        \____/                \____/        |   #
#   |     R5        R3          L3        L5     |   #
#    \         R4                    L4         /    #
#     \                                        /     #
#      \                  ___                 /      #
#       \__               \_/             __/        #
#          \__                         ___/          #
#             \_______________________/              #

# For more details, refer to http://doc.aldebaran.com/2-1/family/robots/leds_robot.html#nao-v5-v4-and-v3-3
std_msgs/Header header

int32 L0=0
int32 L1=1
int32 L2=2
int32 L3=3
int32 L4=4
int32 L5=5
int32 L6=6
int32 L7=7

int32 R0=0
int32 R1=1
int32 R2=2
int32 R3=3
int32 R4=4
int32 R5=5
int32 R6=6
int32 R7=7

int32 NUM_LEDS=8

std_msgs/ColorRGBA[8] left_colors
std_msgs/ColorRGBA[8] right_colors
