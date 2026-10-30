# Output of the radar/IMU fusion filter on the tilt rig (radar_imu_fusion)

uint64 timestamp            # time since system start (microseconds)

float32 tilt                # [rad] estimated platform tilt, positive nose down
float32 tilt_std            # [rad]
float32 gyro_bias           # [rad/s] on the nose down rate
float32 height              # [m] estimated tilt axis height above the ground
float32 height_std          # [m]

float32 alt_innovation      # [m] last altimeter innovation
float32 alt_nis             # last altimeter normalised innovation squared
uint32 alt_rejected         # altimeter ranges rejected by the gate

float32 ground_range        # [m] expected ground echo range, negative when the beam misses the ground
float32 ground_gate         # [m] gate half width around ground_range

uint8 ground_count          # objects of the last radar cycle inside the gate
uint8 obstacle_count        # objects of the last radar cycle outside the gate
uint16 ground_mask          # bit i: radar_objects element i is a ground echo
float32 nearest_obstacle    # [m] nearest object outside the gate, negative when there is none

float32 nees                # normalised estimation error squared, simulated IMU only, otherwise NAN
