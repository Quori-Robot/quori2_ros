# IVT_ID 
uint32 IVT_NONE=0
uint32 IVT_HEAD_STM_TEMP=1
uint32 IVT_BASE_STM_TEMP=2
uint32 IVT_MAIN_STM_TEMP=4
uint32 IVT_POWER_STM_TEMP=8
uint32 IVT_RBTQ_VMOT=16
uint32 IVT_24V_BUS_CURRENT=32
uint32 IVT_12V_BUS_CURRENT=64
uint32 IVT_5V_EXTRA_CURRENT=128
uint32 IVT_ORIN_CURRENT=256
uint32 IVT_POWER_BOARD_24V_BUS_CURRENT=512
uint32 IVT_POWER_BOARD_12V_BUS_CURRENT=1024
uint32 IVT_BATT_VOLTS=2048
uint32 IVT_BATT_CURRENT=4096
uint32 IVT_BATT_SOC=8192

# message structure
uint32 num_values    # number of valid entries in the arrays
uint32[32] ivt_id    # from IVT_ID above
float32[32] value    # Corresponding measurement value
bool[32] valid       # Corresponding measurement value is valid if true