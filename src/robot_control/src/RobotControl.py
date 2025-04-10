#!usr/bin/env python3

import rospy
from std_msgs.msg import String
import math 
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point
import arm64.robot_interface as sdk
import time
class RobotControlNode:
    def __init__(self):
        rospy.init_node('robot_control_node')

        rospy.Subscriber()#need to check later
                # Initialize UDP and command interface for robot control
        HIGHLEVEL = 0xee
        self.udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
        self.cmd = sdk.Highself.cmd()
        self.state = sdk.HighState()
        self.udp.Initself.cmdData(self.self.cmd)

        # Robot movement parameters
        self.cmd.mode = 0
        self.cmd.gaitType = 0
        self.cmd.mode = 0
        self.cmd.gaitType = 0
        self.cmd.speedLevel = 0
        self.cmd.footRaiseHeight = 0
        self.cmd.bodyHeight = 0
        self.cmd.euler = [0, 0, 0]
        self.cmd.velocity = [0, 0]
        self.cmd.yawSpeed = 0.0
        self.cmd.reserve = 0

        rospy.loginfo("Robot control node initialized and ready to move to waypoints.")
        
    # Low Power Sleep
    def lowPowerSleep(self):
        self.cmd.mode = 5
        self.cmd.velocity = [0, 0]
        self.cmd.gaitType = 0
        self.cmd.yawSpeed = 0
        self.udp.SetSend(self.cmd)
        self.udp.Send()
        self.cmd.mode = 7
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    # Sleep Flat
    def sleepFlat(self):
        self.cmd.mode = 5
        self.cmd.velocity = [0, 0]
        self.cmd.yawSpeed = 0
        self.cmd.gaitType = 0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    # Stand Up
    def standUp(self):
        self.cmd.mode = 6
        self.cmd.velocity = [0, 0]
        self.cmd.gaitType = 0
        self.cmd.yawSpeed = 0
        self.udp.SetSend(self.cmd)
        self.udp.Send()
        time.sleep(0.25)
        self.cmd.mode = 1
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    # Move Forward
    def moveForward(self):
        self.cmd.mode = 2
        self.cmd.velocity = [self.quantity, 0]
        self.cmd.gaitType = 1
        self.cmd.yawSpeed = 0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    # Move Backward
    def moveBackward(self):
        self.cmd.mode = 2
        self.cmd.velocity = [-1.0*self.quantity, 0]
        self.cmd.gaitType = 1
        self.cmd.yawSpeed = 0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def moveDirectionless(self):
        self.cmd.mode = 2
        self.cmd.velocity = [self.quantity, 0]
        self.cmd.gaitType = 1
        self.cmd.yawSpeed = 0
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    # Rotate ClockWise
    def rotateClockwise(self):
        self.cmd.mode = 2
        self.cmd.velocity = [0, 0]
        self.cmd.yawSpeed = -1.0*self.quantity
        self.cmd.gaitType = 1
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    # Rotate Counter ClockWise
    def rotateCounterClockwise(self):
        self.cmd.mode = 2
        self.cmd.velocity = [0, 0]
        self.cmd.yawSpeed = self.quantity
        self.cmd.gaitType = 1
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def rotateDirectionless(self):
        self.cmd.mode = 2
        self.cmd.velocity = [0, 0]
        self.cmd.yawSpeed = self.quantity
        self.cmd.gaitType = 1
        self.udp.SetSend(self.cmd)
        self.udp.Send()

    def orientDirectionless(self):
        self.cmd.mode = 2
        self.cmd.velocity = [0, 0]
        self.cmd.yawSpeed = self.quantity1
        self.cmd.velocity = [self.quantity, 0]
        self.cmd.gaitType = 1
        self.udp.SetSend(self.cmd)
        self.udp.Send()


if __name__ == '__main__':
    try:
        controller = RobotControlNode()
    except rospy.ROSInterruptException:
        pass

