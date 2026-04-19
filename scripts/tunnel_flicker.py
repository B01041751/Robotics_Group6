#!/usr/bin/env python3
import rospy
import random
from gazebo_msgs.srv import SetLightProperties
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3

def flicker():
    rospy.init_node('tunnel_flicker')
    rospy.wait_for_service('/gazebo/set_light_properties')
    set_light = rospy.ServiceProxy('/gazebo/set_light_properties', SetLightProperties)

    base_r, base_g, base_b = 0.6, 0.5, 0.2
    rate = rospy.Rate(8)  # 8 Hz gives a natural flicker cadence

    while not rospy.is_shutdown():
        # Random intensity between 40% and 100% of base colour
        factor = random.uniform(0.4, 1.0)
        diffuse = ColorRGBA(base_r * factor, base_g * factor, base_b * factor, 1.0)
        try:
            set_light(
                light_name='tunnel_lamp',
                cast_shadows=True,
                diffuse=diffuse,
                specular=ColorRGBA(0.1, 0.1, 0.1, 1.0),
                attenuation_constant=0.3,
                attenuation_linear=0.0,
                attenuation_quadratic=0.0,
                direction=Vector3(0.0, 0.0, -1.0)
            )
        except rospy.ServiceException:
            pass
        rate.sleep()

if __name__ == '__main__':
    try:
        flicker()
    except rospy.ROSInterruptException:
        pass
