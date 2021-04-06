#!/usr/bin/env python

# This is a test tool to drive open-loop under computer control.

# VELOCITY can be positive (driving forward) or negative (driving backward)
VELOCITY = 0.20

# When REVERSE_TIME is non-zero we drive for this time then reverse direction
# This allows for evaluation of the PID parameters to go back and forth
REVERSE_TIME = 0.0

# When doing reversals this is the dead time at zero velocity between reversals of direction
REVERSE_DEAD_TIME = 2.0

# Initial turn angle (Z axis)
ANGLE = 0.0

LOOP_TIME = 0.05


import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
import argparse


parser = argparse.ArgumentParser(description='Ubiquity Robotics Firmware Updater')
parser.add_argument('--velocity', help='Running velocity in M/Sec)', default='0.2')
parser.add_argument('--angle', help='Angular velocity in Radians/Sec)', default='0.0')
parser.add_argument('--reverse_time', help='Time till direction reverse (0.0 = no reversal)', default='0.0')
parser.add_argument('--reverse_dead_time', help='Time stopped between reversals', default='2.0')
args = parser.parse_args()

VELOCITY = float(args.velocity)
ANGLE    = float(args.angle)
REVERSE_TIME = float(args.reverse_time)
REVERSE_DEAD_TIME = float(args.reverse_dead_time)

rospy.init_node('slow_motion', anonymous=True)

VELOCITY     = rospy.get_param('velocity', VELOCITY)
REVERSE_TIME = rospy.get_param('reverse_time', REVERSE_TIME)
REVERSE_DEAD_TIME = rospy.get_param('reverse_dead_time', REVERSE_DEAD_TIME)

last_t = None
last_pos = None

if REVERSE_TIME > 0.0:
    print ("""    -------------------------------------------------------------
    Auto-reverse velocity mode 
    -------------------------------------------------------------
    """)
else:
    print ("""    -------------------------------------------------------------
    Odometer consistency check
    -------------------------------------------------------------
    """)
    print ("Using velocity of " + str(VELOCITY))

def odometry_callback(msg):

    global REVERSE_TIME

    # Calculate velocity error (%)
    global last_t
    global last_pos

    now = rospy.Time.now().to_sec()
    cur_pos = msg.pose.pose.position

    if last_pos and (REVERSE_TIME == 0.0):
        pos_distance = ((cur_pos.x-last_pos.x)**2 + (cur_pos.y-last_pos.y)**2)**0.5
        t_distance = VELOCITY*(now-last_t)
        print "Velocity error: {}%".format(round(abs(msg.twist.twist.linear.x-VELOCITY)/VELOCITY*100,2)),\
               "  Position error: {}%".format(round(abs(pos_distance-t_distance)/t_distance*100,2))
    
    last_pos = cur_pos
    last_t = now


def slow_motion():

    global REVERSE_TIME
    global REVERSE_DEAD_TIME
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
  
    while not rospy.is_shutdown():

        # Publish the velocity
        pub.publish(vel_msg)

        if initial:
            vel_msg.angular.z = 0.0
            initial = False     

        rospy.sleep(LOOP_TIME) 

        drive_time += LOOP_TIME

        # implement auto-reversal of driving linear direction if enabled
        if ((REVERSE_TIME > 0.0) and (drive_time > REVERSE_TIME)):
            vel_msg.linear.x = -1.0 * vel_msg.linear.x
            drive_time = 0.0
            if REVERSE_DEAD_TIME > 0.0:
                pub.publish(stop_msg)
                print "Linear VELOCITY deadtime active for next ", REVERSE_DEAD_TIME, " sec"
                rospy.sleep(REVERSE_DEAD_TIME) 
            print "Linear VELOCITY is being reversed now for next ", REVERSE_TIME, " sec"


if __name__ == '__main__':
    if abs(VELOCITY)<0.000001:
        print ("VELOCITY must be different from zero")
    else:

        try:
            slow_motion()
        except rospy.ROSInterruptException:
            pass
