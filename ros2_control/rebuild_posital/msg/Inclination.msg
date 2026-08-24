std_msgs/Header header

# Vehicle pitch in radians after the configured mounting transform.
float64 pitch_rad

# Raw 16-bit value from CANopen object 0x6010:00.
uint16 raw_value

# True only while fresh PDO data is available.
bool valid

# Age of the latest PDO sample according to a monotonic clock.
float64 sample_age_sec

# Latest CANopen NMT state (0 when no heartbeat has been received).
uint8 nmt_state
