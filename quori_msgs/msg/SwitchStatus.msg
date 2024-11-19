
# bit flag for SWITCH_STS_PACKET state and changed flags (for limit switches)
# Head Board limit Switches
uint32 LIMIT_SW_1_BIT = 0x00000001
uint32 LIMIT_SW_2_BIT = 0x00000002
uint32 LIMIT_SW_3_BIT = 0x00000004
uint32 LIMIT_SW_4_BIT = 0x00000008
uint32 LIMIT_SW_5_BIT = 0x00000010
uint32 LIMIT_SW_6_BIT = 0x00000020

# Main Board Limit Switches
uint32 LIMIT_SW_7_BIT = 0x00000040
uint32 LIMIT_SW_8_BIT = 0x00000080
uint32 LIMIT_SW_9_BIT = 0x00000100
uint32 LIMIT_SW_10_BIT =0x00000200


# bit flags for SWITCH_STS_PACKET state and changed flags (for OP sensors)
# Head Board Optical switches
uint32 OP_SENSOR_1_BIT = 0x00000400
uint32 OP_SENSOR_2_BIT = 0x00000800
uint32 OP_SENSOR_3_BIT = 0x00001000

# Main Board Optical switches
uint32 OP_SENSOR_4_BIT = 0x00002000
uint32 OP_SENSOR_5_BIT = 0x00004000

# bit flag for SWITCH_STS_PACKET state and changed flags (for cap touch sensor)
uint32 CAP_TOUCH_BIT = 0x00008000

# bit flag for USER_BUTTON state and changed flags (for user button sensors on each PCB)
uint32 USER_BTN_HEAD_BIT =  0x00010000
uint32 USER_BTN_MAIN_BIT =  0x00020000
uint32 USER_BTN_BASE_BIT =  0x00040000
uint32 USER_BTN_POWER_BIT = 0x00080000

#bit flag for BUMP_SWITCH state and changed flags (for bump/pressure  sensors on base motor PCB)
# If using analog pressure sensors these bits indicate pressure exceeds threshold
uint32 BUMP_SW_1_BIT = 0x00100000
uint32 BUMP_SW_2_BIT = 0x00200000
uint32 BUMP_SW_3_BIT = 0x00400000
uint32 BUMP_SW_4_BIT = 0x00800000


uint32 state  # switch status
uint32 changed # switch changed
