uint64 timestamp  # timestamp of the system
 
float32 force_x   # X-axis force component (N)
float32 force_y   # Y-axis force component (N)
float32 force_z   # Z-axis force component (N)

float32 torque_x  # Torque about the X-axis (N*m)
float32 torque_y  # Torque about the X-axis (N*m)
float32 torque_z  # Torque about the X-axis (N*m)

uint8 ORB_QUEUE_LENGTH = 8  # Maximum queue of the topic