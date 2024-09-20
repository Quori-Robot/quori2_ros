uint8[3] temp  # temperatures
uint8 state    # state machine state

float32[3]  volts         # voltages
float32[2]  amps          # reported currents

float32[2]  vel           # reported velocity
float32     pos           # reported position
float32     torque        # reported troque

uint32[2] encoder_count	  # raw encoder counts

uint32 din                # reported digital inputs

uint16[2] fm_faults
uint16    fs_flags
uint16    ff_faults

uint32 coms_error_count