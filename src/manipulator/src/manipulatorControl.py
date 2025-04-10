import sys
import cv2
from ultralytics import YOLO
import time
import rospy
from robot_control import utils
YOLO_Model = ()

from ik import *

# Distance Estimation using the Curve Fit Equation when the Arm is attached to the Robot
def distanceMap(boxX):
    out = 0.0000027821*(boxX**4) - 0.0016156*(boxX**3) + 0.35248*(boxX**2) - 35.944*boxX + 1699.2
    return out

class ManipulatorControlNode:
    def __init___(self):
        rospy.init_node('ManipulatorControlNode')

        self.frames = rospy.Subscriber('/cameraNode')
        self.frameNumber = 0
    
    def TrackBall(self):
        while not(self.frameNumber > 450):
            cap = self.frames
            utils.standUp(self.cmd, self.udp)
            while cap.IsOpened():
                success, frame = cap.read()
                if success:
                    self.frameNumber = self.frameNumber + 1
                    print(self.frameNumber)
                    results = YOLO_Model(source = frame,conf = 0.65,max_det=1,iou=0.5,imgsz=(640,480),verbose=False)
                    annonted_frame = results[0].plot()
                    box = results[0].boxes.xyxy.ravel().cpu().numpy()
                    if(box.size > 0):
                        self.start = 0
                        x1 = box[0]
                        x2 = box[2]
                        error = 120 - (x2-x1) #translation error
                        setPoint = ((640.0-(x2-x1)))/2.0 #SetPoint - The desired location where the bounding box should be...
                        #... so the ball is the center of the frame
                        rot_error = x1 - setPoint #Rotation error
                        utils.orientDirectionless(self.cmd, self.udp, error*0.004,rot_error*-0.003)
                    cv2.imshow('YOLOv8 INterface', annonted_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    break

                if(self.frameNumber > 450):
                    break    

            cap.release()
            cv2.destroyAllWindows()
        utils.sleepFlat(self.cmd, self.udp)


    def PickBall(self):
        while 1:
            cap = self.frame
            utils.setTorque(port, packet)#set torque on the arm
            utils.slowDOnw(port, packet, 1000)#slow down the 
            utils.positionSuitable(port,packet) # Place the Arm in the suitable position to manipulate

            self.start = 1
            self.sameCOunt = 0
            self.x2x1_prev = 0
            self.x1_prev = 0

            while cap.isOpened():
                success , frame = cap.read()
                if success:
                    results = YOLO_Model(source=frame, conf=0.65, max_det=1,iou=0.5,imgsz=(256,160),verbose=False)
                    annoted_frame = results[0].plot()
                    box = results[0].boxes.xyxy.ravel().cpu().numpy #get bounding box
                    if(box.size > 0):
                        self.start = 0
                        self.x1 = box[0]
                        self.x2 = box[2]
                        self.setPoint = ((640.0(x2-x1))/2.0) #SetPoint - The desired location where the bounding box should be...
                        #... so the ball is the center of the frame
                        error = self.x1 - self.setPoint # Error to correct to centre the ball to the centre of the frame
                        if(abs(error) < 5): # Set a deadzone to avoid oscillation when error is less enough
                            error = 0
                        if(error !=0 ):
                            utils.positionZControl(port,packet,readPosition(port,packet,11) + int(-0.5*error)) # The Z-Axis of the robot is corrected to center the ball (Proportional Controller)
                        if(abs((self.x2-self.x1) - self.x2x1_prev) < abs((self.x2-self.x1)*0.005)) and (abs((self.x1) - self.x1_prev) < abs((self.x1)*0.005)): # Check if the ball is in the centre and increment
                            self.sameCount = sameCount + 1
                        else:
                            self.x2x1_prev = self.x2 - self.x1
                            self.x1_prev = self.x1
                        if(self.sameCount == 100): # Checking if the ball was in the center of the frame for 100 consecutive frames
                            utils.positionOpenArm(port,packet) # Open the gripper
                            distanceCorrection = int(distanceMap(self.x2-self.x1)/10.0) # Convert Distance from millimeter to centimeter
                            print('boxX = ', end="")
                            print(self.x2-self.x1)
                            print('distance = ', end="")
                            print(distanceCorrection)
                            if(distanceCorrection >= 30): # Calibration to account for mechanical irregularity in the Arm and the Robot
                                distanceCorrection = distanceCorrection + 4
                            elif(distanceCorrection > 20):
                                distanceCorrection = distanceCorrection + 2
                            i12, j13, k14 = inverseKinematics(distanceCorrection) # Read the joint angle values from the pre-calculated IK values
                            utils.lowDown(port,packet,2000)
                            utils.writePosition(port,packet,-1,i12,j13,k14,-1) # Move the Arm to grab the ball
                            time.sleep(3)
                            utils.slowDown(port,packet,0)
                            utils.positionCloseArm(port,packet)
                            time.sleep(1)
                            utils.slowDown(port,packet,2000)
                            presentCurrent = utils.readCurrent(port,packet,15) # Read the current consumption of the gripper motor, to check if the ball is gripped or not
                            # print("presentCurrent : ")
                            # print(presentCurrent)
                            if(presentCurrent < 20): # The current consumed by gripper motor is less if the motor is not gripper anything
                                sameCount = 0
                                utils.slowDown(port,packet,1000)
                                utils.positionSuitable(port,packet)
                                continue # Restart the Algorithm because the gripping was not successful
                            time.sleep(2)
                            utils.slowDown(port,packet,2000)
                            utils.positionReturnBall(port,packet) # The gripping was successful hence returing the ball position
                            time.sleep(7)
                            utils.positionOpenArm(port,packet) # Release the ball
                            utils.positionHome(port,packet)
                            time.sleep(3)
                            utils.releaseTorque(port,packet)
                            break
                    else:
                        if(self.start == 1):
                            continue
                        presentPosition = utils.readPosition(port,packet,11)
                        if(self.x1 > self.setPoint): # Algorithm to scan the ball in all the positions reachable if the ball is out of the frame
                            utils.positionZControl(port,packet,presentPosition-50)
                            if(presentPosition <= 950):
                                self.x1 = 0
                        else:
                            utils.positionZControl(port,packet,presentPosition+50)
                            if(presentPosition >= 3150):
                                self.x1 = 640

                    cv2.imshow("YOLOv8 Inference", self.annotated_frame) # Show the bounding box on the ball and live correction to center the ball on screen

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                else:
                    break


            cap.release()
            cv2.destroyAllWindows()

    def RetriveBall():
        pass



if __name__ == '__main__':
    try:
        Manipulator = ManipulatorControlNode()
        #function that need to call
    except rospy.ROSInterruptException:
        pass

