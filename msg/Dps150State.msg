# Snapshot of the FNIRSI DPS-150 state.
# Float fields are NaN until the driver has received the matching register.

uint8 PROTECTION_OK=0
uint8 PROTECTION_OVP=1
uint8 PROTECTION_OCP=2
uint8 PROTECTION_OPP=3
uint8 PROTECTION_OTP=4
uint8 PROTECTION_LVP=5
uint8 PROTECTION_REP=6
uint8 PROTECTION_UNKNOWN=255

uint8 MODE_CC=0
uint8 MODE_CV=1
uint8 MODE_UNKNOWN=255

builtin_interfaces/Time stamp
bool connected

float32 input_voltage
float32 set_voltage
float32 set_current
float32 output_voltage
float32 output_current
float32 output_power
float32 temperature

float32[6] preset_voltages
float32[6] preset_currents

float32 over_voltage_protection
float32 over_current_protection
float32 over_power_protection
float32 over_temperature_protection
float32 low_voltage_protection

uint8 brightness
uint8 volume
bool metering_enabled
float32 output_capacity
float32 output_energy

bool output_enabled
uint8 protection_code
string protection
uint8 mode_code
string mode

float32 upper_limit_voltage
float32 upper_limit_current

string model_name
string hardware_version
string firmware_version

uint64 frames_received
uint64 checksum_errors
uint64 parse_errors
