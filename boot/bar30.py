#!/usr/bin/env python

import rospy
from sensor_msgs.msg import FluidPressure, Temperature
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospkg
import sys

# get ms5837 path
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('ecn_bluerov') + '/ms5837-python/')
import ms5837

sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)

# We must initialize the sensor before reading it
if not sensor.init():
    rospy.logwarn("Sensor could not be initialized")
    exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    rospy.logwarn("Sensor read failed!")
    exit(1)


# init ROS stuff
rospy.init_node('barometer_interface')

temp_msg = Temperature()
pres_msg = FluidPressure()
pose_msg = PoseWithCovarianceStamped()
pose_msg.pose.covariance[14] = 0.2

temp_pub = rospy.Publisher('temperature', Temperature, queue_size=1)
pres_pub = rospy.Publisher('pressure', FluidPressure, queue_size=1)
pose_pub = rospy.Publisher('depth', PoseWithCovarianceStamped, queue_size=1)

# init density, used for depth
sensor.setFluidDensity(1000)

# Spew readings
while not rospy.is_shutdown():
    if sensor.read():
        temp_msg.header.stamp = pres_msg.header.stamp = pose_msg.header.stamp = rospy.Time.now()

        temp_msg.temperature = sensor.temperature() # centigrades
        pres_msg.fluid_pressure = sensor.pressure()       # mbar
        pose_msg.pose.pose.position.z = sensor.depth()   # meters

        temp_pub.publish(temp_msg)
        pres_pub.publish(pres_msg)
        pose_pub.publish(pose_msg)

        rospy.sleep(0.1)
    else:
        rospy.logwarn("Sensor read failed!")


