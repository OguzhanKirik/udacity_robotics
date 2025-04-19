#!/usr/bin/env python3
import rospy

from mars.msg import IRStamped

from geometry_msgs.msg import PoseWithCovarianceStamped


## the diagonal elements of the covariance matrix are the variances. The square root of these variances are the standard deviations. 
class ir_positions:
    def __init__(self):
        self.con_x = 0.0025
        self.pub = rospy.Publisher("ir_samples",PoseWithCovarianceStamped,queue_size=10)
        self.sub = rospy.Subscriber("ir_triggers",IRStamped,self.callback_converter)
    def callback_converter(self,msg):
        arr = PoseWithCovarianceStamped()
        arr.header = msg.header
        arr.pose.covariance[0] = self.con_x
        if(msg.beam_index == 0):
            arr.pose.pose.position.x = 0.1
        elif(msg.beam_index == 1):
            arr.pose.pose.position.x = 0.5
        elif(msg.beam_index == 2):
            arr.pose.pose.position.x = 1.8
        elif(msg.beam_index == 3):
            arr.pose.pose.position.x = 1.9
        else:
            print("something")
        rospy.loginfo("{} position beam is received and converted".format(msg.beam_index))
        self.pub.publish(arr)        


if __name__ == '__main__':
    rospy.init_node('ir_positions',anonymous=True)
    ir_positions = ir_positions()    
    print('ir_positions node is ready')
    rospy.spin()

