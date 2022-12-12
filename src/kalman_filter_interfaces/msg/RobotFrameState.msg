# Represents states tracked by Kalman Filter, in the Robot's frame of reference
std_msgs/Header header

# distance along path
float32 x
# speed along path (in forward direction of robot)
float32 x_dot
# orientation of robot wrt global frame
float32 theta
# angular velocity of robot
float32 omega