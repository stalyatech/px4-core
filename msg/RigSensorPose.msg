# Radar poses on the tilt rig, from the platform tilt (rig_pose)
#
# Rig frame: origin on the ground under the tilt axis, x forward (MR72
# boresight at zero tilt), z up. Positive tilt is nose down.

uint64 timestamp            # time since system start (microseconds)

uint8 SOURCE_IMU = 0        # vehicle_attitude pitch
uint8 SOURCE_SERVO = 1      # gimbal pitch command through a servo lag model
uint8 SOURCE_MANUAL = 2     # RIG_TILT_DEG or "rig_pose set"
uint8 source

float32 tilt                # [rad] platform tilt, positive nose down
float32 axis_height         # [m] tilt axis above the ground (RIG_AXIS_H)

float32 alt_face_x          # [m] altimeter antenna face centre, rig frame
float32 alt_face_z          # [m]
float32 alt_boresight       # [rad] altimeter boresight from straight down, positive towards +x

float32 mr72_face_x         # [m] MR72 antenna face centre, rig frame
float32 mr72_face_z         # [m]
float32 mr72_boresight      # [rad] MR72 boresight below the horizon, positive looking down
