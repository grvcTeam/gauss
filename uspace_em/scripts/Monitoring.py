#!/usr/bin/env python
# license removed for brevity
import rospy
from gauss_msgs.msg import Conflict

#create a new publisher. we specify the topic name, then type of message then the queue size
def Monitoring():

    pub = rospy.Publisher('conflict_topic', Conflict, queue_size=10)

    #we need to initialize the node
    rospy.init_node('Monitoring', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        conflict = Conflict()
        conflict.id = 1
        conflict.name = "UAS_IN_CV"
        #rospy.loginfo("I publish:")
        rospy.loginfo(conflict)
        pub.publish(conflict)
        rate.sleep()
if __name__=='__main__':
    try:
        Monitoring()
    except rospy.ROSInterruptException:
        pass