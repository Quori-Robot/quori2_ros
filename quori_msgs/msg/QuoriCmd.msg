# Quori CMD CMD IDs
uint32 QC_GET_VERSION = 1       # Request report of  all hardware/firmware versions - a QuariVersionInfo msg will be published
uint32 QC_MAIN_AUDIO_VOLUME = 2 # Arg1 = volume (0-63). Arg2 = AUDIO_CMD ->.b0 = NA, Arg2.b1 = Volume up (1=increment volume), Arg2.b2 = Volume down (1=decrement volume), Arg2.b3 (1: Set Volume=Arg1)
uint32 QC_HEAD_AUDIO_VOLUME = 3 # Arg1 = volume (0-63). Arg2 = AUDIO_CMD ->.b0 = Mute (1=muted, 0 =active), Arg2.b1 = Volume up (1=increment volume), Arg2.b2 = Volume down (1=decrement volume), Arg2.b3 (1: Set Volume=Arg1)
uint32 QC_POWER_SWITCH = 4      # Arg1 = switch number (one of POWER_SWITCH_ID), arg2 POWER_SWITCH_STATE: 0=off, 1=on
uint32 QC_SHUTDOWN = 5          # Arg1 = Hard -> 0=Soft, 1=Immediate
uint32 QC_CLEAR_FAULTS = 6      # Args N/A

# arguments for QC_[MAIN/HEAD]_AUDIO_VOLUME Audio commands
uint32 NONE=0
uint32 MUTE=1           # HEAD Audio only
uint32 VOLUME_UP=2
uint32 VOLUME_DOWN=4
uint32 SET_VOLUME=8    # arg2 = volume (0-63)

# POWER_SWITCH_ID for QC_POWER_SWITCH comamnd  arg1
uint32 PS_NONE=0
uint32 PS_MAIN_24V_BUS=1
uint32 PS_MAIN_12V_BUS=2
uint32 PS_MAIN_5V_EXTRA=3
uint32 PS_HEAD_12V=4
uint32 PS_ORIN=5
uint32 PS_RBTQ=6
uint32 PS_FRONT_LIDAR=7
uint32 PS_BACK_LIDAR=8
uint32 PS_RGB_LEDS=9
uint32 PS_USB_HUB=10
uint32 PS_ETHERNET=11
uint32 PS_NECK_AUDIO_12V=12
uint32 PS_MAIN_AUDIO_12V=13
uint32 PS_HEAD_AUDIO=14     #  head audio SHTDN
uint32 PS_HEAD_DISPLAY=15
uint32 PS_POWER_BOARD_12V=16
uint32 PS_POWER_BOARD_12V_EXTRA=17

# POWER_SWITCH_STATE for  QC_POWER_SWITCH comamnd  Arg2
uint32 PS_OFF=0
uint32 PS_ON=1
uint32 PS_INVALID=2

# Message Structure
uint32 cmd           # command from definitions above
uint32 arg1
uint32 arg2