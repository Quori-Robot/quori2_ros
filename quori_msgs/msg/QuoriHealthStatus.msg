# Bit Flags for status of each component

uint32 HF_NONE=0

uint32 HF_BASE_BOARD_OK=1
uint32 HF_HEAD_BOARD_OK=2
uint32 HF_POWER_BOARD_OK=4
uint32 HF_BASE_MTR_BOARD_OK=8

uint32 HF_ORIN_OK=16

uint32 HF_RBTQ_OK=32

uint32 HF_TWIST_MOTOR_OK=64
uint32 HF_NECK_MOTOR_OK=128
uint32 HF_PAN_MOTOR_OK=256
uint32 HF_TILT_MOTOR_OK=512

# Message Structure
uint32 status_flags  # contains bit flags indicating the operating status of major compontents of the robot
