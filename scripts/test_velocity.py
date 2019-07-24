#!/usr/bin/env python

# VELOCITY can be positive (driving forward) or negative (driving backward)
VELOCITY = 0.2

# Initial turn angle (Z axis)
ANGLE = 0.0


import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry

rospy.init_node('slow_motion', anonymous=True)

last_t = None
last_pos = None

print ("""-------------------------------------------------------------
Odometer consistency check
-------------------------------------------------------------
""")

def odometry_callback(msg):

    # Calculate velocity error (%)
    global last_t
    global last_pos

    now = rospy.Time.now().to_sec()
    cur_pos = msg.pose.pose.position

    if last_pos:
        pos_distance = ((cur_pos.x-last_pos.x)**2 + (cur_pos.y-last_pos.y)**2)**0.5
        t_distance = VELOCITY*(now-last_t)
        print "Velocity error: {}%".format(round(abs(msg.twist.twist.linear.x-VELOCITY)/VELOCITY*100,2)),\
               "  Position error: {}%".format(round(abs(pos_distance-t_distance)/t_distance*100,2))
    
    last_pos = cur_pos
    last_t = now


def slow_motion():


    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('odom',Odometry,odometry_callback)
    vel_msg = Twist()

    initial = True

    vel_msg.linear.x = VELOCITY
    vel_msg.angular.z = ANGLE
  
    while not rospy.is_shutdown():

        # Publish the velocity
        pub.publish(vel_msg)

        if initial:
            vel_msg.angular.z = 0.0
            initial = False     

        rospy.sleep(0.05) 



if __name__ == '__main__':
    if abs(VELOCITY)<0.000001:
        print ("VELOCITY must be different from zero")
    else:

        try:
            slow_motion()
        except rospy.ROSInterruptException:
            pass