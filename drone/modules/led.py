import rospy
from clover.srv import SetLEDEffect

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
