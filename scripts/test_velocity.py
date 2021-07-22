#!/usr/bin/env python3

# This is a test tool to drive forever open-loop under computer control.
#
# By default a constant linear velocity is used till script is stopped.
# In constant mode velocity and position errors are printed
#
# The second mode will switch between two speeds with a deadtime between.
# This mode is enabled when the 'switched_time' parameter is non-zero.
# The switched_time is how long to run in a given direction.
# The normal velocity can be set and the second, switched_velocity can be set
# where each velocity can be positive or negative.
# The dead time can be 0 but if not we go to 0 for the time specified.
# 

# VELOCITY can be positive (driving forward) or negative (driving backward)
VELOCITY = 0.20

# When SWITCH_TIME is non-zero we drive for this time then reverse direction
# This allows for evaluation of the PID parameters to go back and forth
SWITCH_TIME = 0.0
SWITCHED_VELOCITY = VELOCITY * -1.0

# When doing reversals this is the dead time at zero velocity between reversals of direction
SWITCH_DEAD_TIME = 2.0

# Initial turn angle (Z axis)
ANGLE = 0.0

LOOP_TIME = 0.05


import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
import argparse


parser = argparse.ArgumentParser(description='Ubiquity Robotics Automatic velocity tester')
parser.add_argument('--loop_time', help='Loop time delay in sec)', default='0.05')
parser.add_argument('--velocity', help='Running velocity in M/Sec)', default=str(VELOCITY))
parser.add_argument('--angle', help='Angular velocity in Radians/Sec)', default='0.0')
parser.add_argument('--switch_time', help='Time till direction switch (0.0 = no switch)', default='0.0')
parser.add_argument('--switched_velocity', help='Velocity in switched 2nd period', default=str(SWITCHED_VELOCITY))
parser.add_argument('--switch_dead_time', help='Time stopped between velocity switches', default='2.0')
args = parser.parse_args()

LOOP_TIME = float(args.loop_time)
VELOCITY = float(args.velocity)
ANGLE    = float(args.angle)
SWITCH_TIME = float(args.switch_time)
SWITCHED_VELOCITY = float(args.switched_velocity)
SWITCH_DEAD_TIME = float(args.switch_dead_time)

rospy.init_node('slow_motion', anonymous=True)


last_t = None
last_pos = None

if SWITCH_TIME > 0.0:
    print("""    -------------------------------------------------------------
    Auto-reverse velocity mode 
    -------------------------------------------------------------
    """)
else:
    print("""    -------------------------------------------------------------
    Odometer consistency check
    -------------------------------------------------------------
    """)
    print("Using velocity of " + str(VELOCITY))

def odometry_callback(msg):

    global SWITCH_TIME

    # Calculate velocity error (%)
    global last_t
    global last_pos

    now = rospy.Time.now().to_sec()
    cur_pos = msg.pose.pose.position

    if last_pos and (SWITCH_TIME == 0.0):
        pos_distance = ((cur_pos.x-last_pos.x)**2 + (cur_pos.y-last_pos.y)**2)**0.5
        t_distance = VELOCITY*(now-last_t)
        print("Velocity error: {}%".format(round(abs(msg.twist.twist.linear.x-VELOCITY)/VELOCITY*100,2)),\
               "  Position error: {}%".format(round(abs(pos_distance-t_distance)/t_distance*100,2)))
    
    last_pos = cur_pos
    last_t = now


def slow_motion():

    global SWITCH_TIME
    global SWITCH_DEAD_TIME
    global LOOP_TIME

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('odom',Odometry,odometry_callback)
    vel_msg = Twist()
    vel_msg.linear.x = VELOCITY
    vel_msg.angular.z = ANGLE

    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0

    initial = True


    drive_time = 0.0
    drive_forward = 1
  
    while not rospy.is_shutdown():

        # Publish the velocity
        pub.publish(vel_msg)

        if initial:
            vel_msg.angular.z = 0.0
            initial = False     

        rospy.sleep(LOOP_TIME) 

        drive_time += LOOP_TIME

        # implement auto-reversal of driving linear direction if enabled
        if ((SWITCH_TIME > 0.0) and (drive_time > SWITCH_TIME)):
            if drive_forward > 0:
                print "Linear VELOCITY is being set to rate 1 of ", VELOCITY, " M/sec for ", SWITCH_TIME, " sec"
                vel_msg.linear.x = VELOCITY
            else:
                vel_msg.linear.x = SWITCHED_VELOCITY
                print "Linear VELOCITY is being set to rate 2 of ", SWITCHED_VELOCITY, " M/sec for ", SWITCH_TIME, " sec"
            drive_forward = drive_forward * -1
            drive_time = 0.0
            if SWITCH_DEAD_TIME > 0.0:
                pub.publish(stop_msg)
                print "Linear VELOCITY deadtime active for next ", SWITCH_DEAD_TIME, " sec"
                rospy.sleep(SWITCH_DEAD_TIME) 


if __name__ == '__main__':
    if abs(VELOCITY)<0.000001:
        print("VELOCITY must be different from zero")
    else:

        try:
            slow_motion()
        except rospy.ROSInterruptException:
            pass
