
uint8 IVT_NONE=0
uint8 IVT_HEAD_STM_TEMP=1
uint8 IVT_BASE_STM_TEMP=2
uint8 IVT_MAIN_STM_TEMP=4
uint8 IVT_POWER_STM_TEMP=8
uint8 IVT_RBTQ_VMOT=16
uint8 IVT_24V_BUS_CURRENT=32
uint8 IVT_12V_BUS_CURRENT=64
uint8 IVT_5V_EXTRA_CURRENT=128
uint8 IVT_ORIN_CURRENT=256

uint32 num_values           # number of valid entries in the arrays
uint32[32] ivt_id
float32[32] value