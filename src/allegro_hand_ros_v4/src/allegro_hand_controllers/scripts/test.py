import rospy
from  sensor_msgs.msg import JointState
import time
if __name__=='__main__':
    rospy.init_node('pd_control_test')
    pub=rospy.Publisher('allegroHand/joint_cmd',JointState,queue_size=10)
    pd_value=JointState()

    rate=rospy.Rate(20)
    for idx in range(16):
        for deg in range(1,128):
            pd_value.position[idx]=deg
            pub.publish(pd_value)
            rate.sleep()
    
