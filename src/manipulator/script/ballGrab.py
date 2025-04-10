import sys
import cv2
from ultralytics import YOLO
import time
import rospy

from ik import *

YOLO_model = YOLO()

class ballGrabNode():
    def __init__(self):
      rospy.init_node('ballGrab')
      self.frames = rospy.Subscriber('/input_camera')

      # Distance Estimation using the Curve Fit Equation when the Arm is placed on ground
      def distanceMAPArmOnGround(self):
              self.out = 0
              if(self.boxX >= 157 and self.boxX <= 350):
                self.out = 0.0028*(self.boxX**2) - 1.9118*self.boxX + 480.5943
              if(self.boxX < 157 and self.boxX >= 80):
                    self.out = -0.00067437*(self.boxX**3) + 0.27581*(self.boxX**2) - 38.937*self.boxX + 2173.5
                    return self.out
          
      
      # Distance Estimation using the Curve Fit Equation when the Arm is attached to the Robot
      def distanceMap(boxX):
          out = 0.0000027821*(boxX**4) - 0.0016156*(boxX**3) + 0.35248*(boxX**2) - 35.944*boxX + 1699.2
          return out
      
      def GrabBall(self):
        cap = self.frames
        setTorque(self.port, self.packet)
        slowDown(port, packet, 1000)
        positionSuitable(port, packet)

        self.start = 1
        self.sameCount = 0
        self.x2x1_prev = 0
        self.x1_prev = 0

        while cap.isOpened():
             self.success, self.frame = cap.read()
             if self.success:
                  self.results = YOLO_model(source=self.frame, conf=0.65,max_det=1,iou=0.5,imgsz=(256,160),verbose=False)
                  self.annoted_frame = self.results[0].plot()
                  self.box = self.results[0].boxes.xyxy.ravel().cpu().numpy()
                  if(self.box > 0):
                      self.start = 0
                      self.x1 = self.box[0]
                      self.x2 = self.box[2]
                      self.setPoint = ((640.0-(self.x2-self.x1))/2.0)
                      self.error = self.x1 - self.setPoint
                      if(abs(self.error) < 5):
                          self.error = 0
                      if(self.error != 0):
                          # The Z-Axis of the robot is corrected to center the ball (Proportional Controller)
                          positionZControl(port,packet,readPosition(port,packet,11) + int(-0.5*self.error)) 
                      if(abs((x2-x1) - x2x1_prev) < abs((x2-x1)*0.005)) and (abs((x1) - x1_prev) < abs((x1)*0.005)): # Check if the ball is in the centre and increment
                          sameCount = sameCount + 1
                      else:
                          x2x1_prev = x2 - x1
                          x1_prev = x1
                      if(sameCount == 100): # Checking if the ball was in the center of the frame for 100 consecutive frames
                          positionOpenArm(port,packet) # Open the gripper
                          distanceCorrection = int(distanceMap(x2-x1)/10.0) # Convert Distance from millimeter to centimeter
                          print('boxX = ', end="")
                          print(x2-x1)
                          print('distance = ', end="")
                          print(distanceCorrection)
                          if(distanceCorrection >= 30): # Calibration to account for mechanical irregularity in the Arm and the Robot
                              distanceCorrection = distanceCorrection + 4
                          elif(distanceCorrection > 20):
                              distanceCorrection = distanceCorrection + 2
                          i12, j13, k14 = inverseKinematics(distanceCorrection) # Read the joint angle values from the pre-calculated IK values
                          slowDown(port,packet,2000)
                          writePosition(port,packet,-1,i12,j13,k14,-1) # Move the Arm to grab the ball
                          time.sleep(3)
                          slowDown(port,packet,0)
                          positionCloseArm(port,packet)
                          time.sleep(1)
                          slowDown(port,packet,2000)
                          presentCurrent = readCurrent(port,packet,15) # Read the current consumption of the gripper motor, to check if the ball is gripped or not
                          # print("presentCurrent : ")
                          # print(presentCurrent)
                          if(presentCurrent < 20): # The current consumed by gripper motor is less if the motor is not gripper anything
                              sameCount = 0
                              slowDown(port,packet,1000)
                              positionSuitable(port,packet)
                              continue # Restart the Algorithm because the gripping was not successful
                          time.sleep(2)
                          slowDown(port,packet,2000)
                          positionReturnBall(port,packet) # The gripping was successful hence returing the ball position
                          time.sleep(7)
                          positionOpenArm(port,packet) # Release the ball
                          positionHome(port,packet)
                          time.sleep(3)
                          releaseTorque(port,packet)
                          break
                  else:
                      if(start == 1):
                          continue
                      presentPosition = readPosition(port,packet,11)
                      if(x1 > setPoint): # Algorithm to scan the ball in all the positions reachable if the ball is out of the frame
                          positionZControl(port,packet,presentPosition-50)
                          if(presentPosition <= 950):
                              x1 = 0
                      else:
                          positionZControl(port,packet,presentPosition+50)
                          if(presentPosition >= 3150):
                              x1 = 640

                  cv2.imshow("YOLOv8 Inference", annotated_frame) # Show the bounding box on the ball and live correction to center the ball on screen

                  if cv2.waitKey(1) & 0xFF == ord("q"):
                      break
              else:
                  break
             
             
          self.cap.release()
          cv2.destroyAllWindows()

if __name__=='__main__':
        try:
        ball = ballGrabNode()
    except rospy.ROSInterruptException:
        pass