#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16

ir_motor = rospy.Publisher('/bob/ir_motor', UInt16, queue_size=5)

def servo_publisher():
    rospy.init_node('servo_pub', anonymous=True)
    rate = rospy.Rate(2)  # 40 Hz e 0.8

    posicao = 0

    while not rospy.is_shutdown():
        posicao = 0 
        rate.sleep()
        ir_motor.publish(posicao)

        posicao = 90 
        rate.sleep()
        ir_motor.publish(posicao)

        posicao = 180
        rate.sleep()
        ir_motor.publish(posicao)


if __name__ == '__main__':
    try:
        servo_publisher()
    except rospy.ROSInterruptException:
        pass
