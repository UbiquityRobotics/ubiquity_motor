#!/usr/bin/env python

# VELOCITY can be positive (driving forward) or negative (driving backward)
VELOCITY = 0.05

# Initial turn angle (Z axis)
ANGLE = 0.0

import roslib; roslib.load_manifest('ubiquity_motor')
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

print ("""-------------------------------------------------------------
Odometer consistency check
-------------------------------------------------------------
""")


def odometry_callback(msg):

    # Calculate velocity error (%)
    print ("Velocity error: {}%".format(round(abs(msg.twist.twist.linear.x-VELOCITY)/VELOCITY*100,2)))

def slow_motion():

    rospy.init_node('slow_motion', anonymous=True)
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
        
        rospy.sleep(0.001)


if __name__ == '__main__':
    try:
        slow_motion()
    except rospy.ROSInterruptException:
        pass