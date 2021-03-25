#!/usr/bin/env python

# This is a test tool to drive open-loop under computer control.

# VELOCITY can be positive (driving forward) or negative (driving backward)
VELOCITY = 0.20

# When REVERSE_TIME is non-zero we drive for this time then reverse direction
# This allows for evaluation of the PID parameters to go back and forth
VEL_REV_TIME = 0.0
VEL_REV_DEAD_TIME = 2.0

# Initial turn angle (Z axis)
ANGLE = 0.0

LOOP_TIME = 0.05


import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry

rospy.init_node('slow_motion', anonymous=True)

last_t = None
last_pos = None

if VEL_REV_TIME > 0.0:
    print ("""    -------------------------------------------------------------
    Auto-reverse velocity mode 
    -------------------------------------------------------------
    """)
else:
    print ("""    -------------------------------------------------------------
    Odometer consistency check
    -------------------------------------------------------------
    """)

def odometry_callback(msg):

    global VEL_REV_TIME

    # Calculate velocity error (%)
    global last_t
    global last_pos

    now = rospy.Time.now().to_sec()
    cur_pos = msg.pose.pose.position

    if last_pos and (VEL_REV_TIME == 0.0):
        pos_distance = ((cur_pos.x-last_pos.x)**2 + (cur_pos.y-last_pos.y)**2)**0.5
        t_distance = VELOCITY*(now-last_t)
        print "Velocity error: {}%".format(round(abs(msg.twist.twist.linear.x-VELOCITY)/VELOCITY*100,2)),\
               "  Position error: {}%".format(round(abs(pos_distance-t_distance)/t_distance*100,2))
    
    last_pos = cur_pos
    last_t = now


def slow_motion():

    global VEL_REV_TIME
    global VEL_REV_DEAD_TIME
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
        if ((VEL_REV_TIME > 0.0) and (drive_time > VEL_REV_TIME)):
            vel_msg.linear.x = -1.0 * vel_msg.linear.x
            drive_time = 0.0
            if VEL_REV_DEAD_TIME > 0.0:
                pub.publish(stop_msg)
                print "Linear VELOCITY deadtime active for next ", VEL_REV_DEAD_TIME, " sec"
                rospy.sleep(VEL_REV_DEAD_TIME) 
            print "Linear VELOCITY is being reversed now for next ", VEL_REV_TIME, " sec"


if __name__ == '__main__':
    if abs(VELOCITY)<0.000001:
        print ("VELOCITY must be different from zero")
    else:

        try:
            slow_motion()
        except rospy.ROSInterruptException:
            pass
