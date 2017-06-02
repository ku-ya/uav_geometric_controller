# header contains time stamp
Header header

# This message specifies the coordinate used for the program
# north east down (NED) or local tangent plane (LTP)
# east north up (ENU)
# desired position, velocity, acceleration in (ENU)
geometry_msgs/Vector3 xc # + xc_dot, xc_2dot, xc_3dot
geometry_msgs/Vector3 xc_dot
geometry_msgs/Vector3 xc_2dot
geometry_msgs/Vector3 xc_ned # desired command in NED frame
geometry_msgs/Vector3 xc_ned_dot
geometry_msgs/Vector3 xc_ned_2dot

# vicon pose, velocity (ENU)
geometry_msgs/Vector3 x_v
geometry_msgs/Quaternion q_v
float32[9] R_v
geometry_msgs/Vector3 v_v

# IMU attitude, angular velocity (NED)
geometry_msgs/Vector3 w_imu
geometry_msgs/Quaternion q_imu
float32[9] R_imu

# desired attitude (NED)
float32[9] Rc
float32[9] Rc_dot
float32[9] Rc_2dot
geometry_msgs/Vector3 Wc # + omaga_c_dot
geometry_msgs/Vector3 Wc_dot
geometry_msgs/Vector3 b1d # + b1d_dot, b1d_2dot

# position attitude in NED
geometry_msgs/Vector3 x
geometry_msgs/Vector3 v
geometry_msgs/Vector3 W
float32[9] R

# position, attitude error vectors (NED)
geometry_msgs/Vector3 ex # + ev
geometry_msgs/Vector3 ev
geometry_msgs/Vector3 eR # + eW
geometry_msgs/Vector3 eW
# Actuation value
# force: total thrust
# f_motor: force for each motor
# f_motor_sat: saturation applied to the f_motor
# throttle: motor command value
# moment: in (NED)
float32 force
float32[4] f_motor
float32[4] f_motor_sat
float32[4] throttle
geometry_msgs/Vector3 moment

# controller position and attitude gains
float32[3] gain_position
float32[3] gain_attitude

# sensor feed back from the motor
uint8[24] motor_power
