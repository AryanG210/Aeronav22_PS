from controller import Robot, Motor, Gyro, GPS, Camera, Compass, Keyboard, LED, InertialUnit, DistanceSensor
import math
import cv2
import numpy as np 
import time
from pyzbar.pyzbar import decode


SIGN = lambda x: int(x>0) - int(x<0)
CLAMP = lambda value, low, high : min(high, max(value, low))




class Drone:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        # front_left_led = robot.getDevice("front left led");
        # front_right_led = robot.getDevice("front right led");
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        self.camera_roll_motor = self.robot.getDevice('camera roll')
        self.camera_pitch_motor = self.robot.getDevice('camera pitch')

        self.front_left_motor = self.robot.getDevice("front left propeller")
        self.front_right_motor = self.robot.getDevice("front right propeller")
        self.rear_left_motor = self.robot.getDevice("rear left propeller")
        self.rear_right_motor = self.robot.getDevice("rear right propeller")
        self.motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]

        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        self.k_vertical_thrust = 68.5
        self.k_vertical_offset = 0.6 
        self.k_vertical_p = 3.0
        self.k_roll_p = 50.0
        self.k_pitch_p = 30.0

        self.target_altitude = 1.0

    def move(self,command,intensity):
        roll = self.imu.getRollPitchYaw()[0] #+ math.pi / 2.0
        pitch = self.imu.getRollPitchYaw()[1]
        altitude = self.gps.getValues()[2]
        roll_acceleration = self.gyro.getValues()[0]
        pitch_acceleration = self.gyro.getValues()[1]

        # led_state = int(time) % 2
        # front_left_led.set(led_state)
        # front_right_led.set(int(not led_state))
        
        # self.camera_roll_motor.setPosition(-0.115 * roll_acceleration)
        # self.camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)
        self.camera_roll_motor.setPosition(0)
        self.camera_pitch_motor.setPosition(math.pi/2)
        
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        if(command=='forward'):
            pitch_disturbance = -intensity  #2.0
        elif(command=='backward'):
            pitch_disturbance = intensity #-2.0
        elif(command=='right'):
            yaw_disturbance = -intensity  #1.3
        elif(command=='left'):
            yaw_disturbance = intensity  #-1.3
        elif(command=='sRight'):
            roll_disturbance = -intensity  #-1.0
        elif(command=='sLeft'):
            roll_disturbance = intensity  #1.0
        elif(command=='up'):
            self.target_altitude += intensity  #0.05
        elif(command=='down'):
            self.target_altitude -= intensity  #0.05

        roll_input = self.k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = self.k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = CLAMP(self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
        vertical_input = self.k_vertical_p * pow(clamped_difference_altitude, 3.0)

        
        front_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
        self.front_left_motor.setVelocity(front_left_motor_input)
        self.front_right_motor.setVelocity(-front_right_motor_input)
        self.rear_left_motor.setVelocity(-rear_left_motor_input)
        self.rear_right_motor.setVelocity(rear_right_motor_input)
        
    def get_image(self):
        self.camera.saveImage('image.jpg', 100)
        image = cv2.imread('image.jpg')
        return image
            
      


    def follow_center_of_road(self):
        # Get the camera image
        image = self.get_image()
        cv2.imshow("Ground image",image)

        # Split the image into left and right halves
        height, width, _ = image.shape
        left_half = image[:, :width // 2, :]
        right_half = image[:, width // 2:, :]

        # Process left and right halves to detect white lines
        left_white_mask = cv2.inRange(left_half, (200, 200, 200), (255, 255, 255))
        right_white_mask = cv2.inRange(right_half, (200, 200, 200), (255, 255, 255))
        cv2.imshow("Left White Mask", left_white_mask)
        cv2.imshow("Right White Mask", right_white_mask)
        cv2.waitKey(1)
        # Calculate the areas of white color in each half
        left_white_area = cv2.countNonZero(left_white_mask)
        right_white_area = cv2.countNonZero(right_white_mask)
        print("Left area: ",left_white_area)
        print("Right area: ",right_white_area)
        j=700
        diff = abs(left_white_area-right_white_area)+200
        # print()
        if diff>500:
            diff= 200
        diff=2
       
        # if j>0:
            # j-=1
        # self.move('sLeft',2)
        # Decide the direction based on the area of white color
        if left_white_area > right_white_area:
            while j > 0:
                j =j- 1
                if right_white_area==0:
                    # print('Here l')
                    # self.move('sLeft',2)
                    self.move('left', diff)
                    # self.move('backward',2)
                else:
                    self.move('left', diff)
        else:
            while j > 0:
                j =j- 1
                if left_white_area==0:
                    # print("Here r")
                    # self.move('sRight',2)
                    self.move('right', diff)
                    # self.move('backward',2)
                else:    
                    self.move('right', diff)
        # self.move('forward', 3)
        
    def follow_center_of_roadd(self):
        # Get the camera image
        image = self.get_image()
        if image is None:
            drone.move('forward', 0.1)
            return
        
        cv2.imshow("Ground image",image)

        # Split the image into left and right halves
        height, width, _ = image.shape
        left_half = image[:, :width // 2, :]
        right_half = image[:, width // 2:, :]

        # Process left and right halves to detect white lines
        left_white_mask = cv2.inRange(left_half, (200, 200, 200), (255, 255, 255))
        right_white_mask = cv2.inRange(right_half, (200, 200, 200), (255, 255, 255))
        cv2.imshow("Left White Mask", left_white_mask)
        cv2.imshow("Right White Mask", right_white_mask)
        cv2.waitKey(1)
        # Calculate the areas of white color in each half
        left_white_area = cv2.countNonZero(left_white_mask)
        right_white_area = cv2.countNonZero(right_white_mask)
        print("Left area: ",left_white_area)
        print("Right area: ",right_white_area)
        j=300
        diff = abs(left_white_area-right_white_area)+200
        # print()
        if diff>500:
            diff= 200
        diff=0.7
       
        # if j>0:
            # j-=1
        # self.move('sLeft',2)
        # Decide the direction based on the area of white color
        if left_white_area > right_white_area:
            while j > 0:
                j =j- 1
                if right_white_area==0:
                    # print('Here l')
                    # self.move('sLeft',2)
                    self.move('left', diff)
                    # self.move('backward',2)
                else:
                    self.move('left', diff)
        else:
            while j > 0:
                j =j- 1
                if left_white_area==0:
                    # print("Here r")
                    # self.move('sRight',2)
                    self.move('right', diff)
                    # self.move('backward',2)
                else:    
                    self.move('right', diff)
        # self.move('forward', 3)     

    def get_qr_code_data(self):
        # Get the camera image
        image = self.get_image()

        # Get the center region of the image
        height, width, _ = image.shape
        roi = image[:height, width // 4: 3 * width // 4]

        # Convert the ROI to grayscale
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Use QR code scanner to decode information
        qr_codes = decode(gray_roi)

        # Display the ROI with QR code detection
        cv2.imshow("ROI with QR Code Detection", gray_roi)
        cv2.waitKey(1)

        if qr_codes:
            # Extract and return the data from the QR code
            qr_data = qr_codes[0].data.decode("utf-8")
            # return qr_data
            print(qr_data)
            values = [float(val) for val in qr_data.split(",")]
            return values
        else:
            return None
    def count_objects_red(self,list1):
        # Define the RGB values for pure red
        pure_red = list1

        # Define a small range around pure red
        color_lower = np.array([pure_red[0] - 40, pure_red[1] - 40, pure_red[2] - 40])
        color_upper = np.array([pure_red[0] + 40, pure_red[1] + 40, pure_red[2] + 40])

        
        # Open the video source
        frame = self.get_image()

        

        # Create a binary mask based on the specified color range
        mask = cv2.inRange(frame, color_lower, color_upper)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize object count
        object_count = 0

        for contour in contours:
            # Get the area of each contour
            area = cv2.contourArea(contour)

            # Set a minimum area threshold to filter out small noise
            min_contour_area = 2

            if area >= min_contour_area:
                # Draw a bounding box around the detected object
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # Increment the object count
                object_count += 1


        

        # Break the loop if 'q' key is pressed
        

        # Release the video source and close all windows
        # cap.release()
        # cv2.destroyAllWindows()

        return object_count


drone=Drone()


i=0
# blue = 0000ff
# cyan = 00ffff
# red =  ff0000
# yellow = ffff00
# pink = ff00ff
rgb1 = (0,0,255)   #red
rgb2 = (255,0,0)   #blue

#rgb1=(255,255,0) #cyan
#rgb2=(0,255,255) #yellow
rgb1c=0
rgb2c=0
# input is in bgr
# Assuming the drone object is initialized and has methods like move and robot.step
while drone.robot.step(drone.timestep) != -1 and rgb1 is not None and rgb2 is not None:
    i = i + 1
    print(i)
    x=abs(2*rgb1c-rgb2c)
    # x=2
    # if i%500==0:
    #     print(f"Count of blue is {rgb1c} and count of red is {rgb2c}")
    #     print(f"x is {x}")
    if i < 100:
        drone.move('up', 0.1)
        # drone.move('sLeft',0.3)
    elif i<1000:  
            # drone.move('sLeft',1)
        drone.move('forward',0.5)
    # else:
        # drone.move('sRight',3)
   
        
    elif i<6500:
        ## print("in this area")
        # drone.move('left',1)
        # drone.move('forward',2)
        
        if i%20==0:
            drone.follow_center_of_road()
            # time.sleep(1)
            
        else:
            drone.move('forward',1.5)
        if i%700==0:
            rgb1c+=drone.count_objects_red(rgb1)
            rgb2c+=drone.count_objects_red(rgb2)
            
            
    elif i<64000:
        if i%5==0:
            drone.follow_center_of_roadd()
            # time.sleep(1)
            
        else:
            drone.move('forward',0.1)
        if i%15000==0:
            rgb1c+=drone.count_objects_red(rgb1)
            rgb2c+=drone.count_objects_red(rgb2)
            
    elif i <69000:
        if i%20==0:
            drone.follow_center_of_road()
            # time.sleep(1)
            
        else:
            drone.move('forward',1.5)
        if i%700==0:
            rgb1c+=drone.count_objects_red(rgb1)
            rgb2c+=drone.count_objects_red(rgb2)
            
            
    elif i<69032:
        drone.move('right',1)
        # drone.move('ba')
    elif i<69050:
        drone.move('down',0.1)
    elif i<70270:
        drone.move('sLeft',1)
    elif i<70950:
        drone.move('backward',0.5)
    else:
        z=drone.get_qr_code_data()
        if(z is not None):
            
            if x in z:
                for j in range(300):
                    drone.move('backward',0.4)
                for j in range(200):
                    drone.move('down',0.1)
                    # drone.move('backward',0.2)
                
            else:
                drone.move('sRight',1)
                continue
        else:
            drone.move('sRight',1)
            continue
            
        
        
        ## drone.follow_white_line()
        ## drone.move('sRight',1.5)

    # print(drone.imu.getRollPitchYaw())
    # print(drone.gps.getValues())
    # if i%100==0:
        # image=drone.get_image()
        
        
        # cv2.imshow("img",image)
        # cv2.waitKey(1)
        # cv2.destroyAllWindows()

    
    # image=drone.get_image()
    
   
    # cv2.imshow("img",image)
    # if cv2.waitKey() & 0xFF == ord('q'):
        # break

cv2.destroyAllWindows()

    


   


