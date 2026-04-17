# L3_color_tracking.py
# This program was designed to have SCUTTLE following a target using a USB camera input

import cv2              # For image capture and processing
import numpy as np      
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import netifaces as ni
from time import sleep
from math import radians, pi
import L3_path_template as path
import lidar_obstacle_avoidance as lidar

# Gets IP to grab MJPG stream
#def getIp():
#    return 0
    
#    Camera
#stream_ip = getIp()
#if not stream_ip: 
#    print("Failed to get IP for camera stream")
#    exit()

#camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

size_w  = 240   # Resized image width. This is the image width in pixels.
size_h = 160	# Resized image height. This is the image height in pixels.

fov = 1         # Camera field of view in rad (estimate)

#    Color Range, described in HSV

        #Values for Blue Volleyball
        #v1_min = 85        # Minimum H value
        #v2_min = 80        # Minimum S value
        #v3_min = 120       # Minimum V value
        #
        #v1_max = 115     # Maximum H value
        #v2_max = 255     # Maximum S value
        #v3_max = 255     # Maximum V value

# Values for Orange Ping Pong Ball
v1_min = 5          # Minimum H value
v2_min = 150        # Minimum S value
v3_min = 150        # Minimum V value

v1_max = 150     # Maximum H value
v2_max = 255     # Maximum S value
v3_max = 255     # Maximum V value

target_width = 25      # Target pixel width of tracked object
angle_margin = 0.2      # Radians object can be from image center to be considered "centered"
width_margin = 5       # Minimum width error to drive forward/back

def main():
    # Try opening camera with default method
    try:
        camera = cv2.VideoCapture(0)    
    except: pass

    # Try opening camera stream if default method failed
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)    

    camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    camera.set(4, size_h)                       # Set height of images that will be retrived from camera
    
    # initialize LiDAR
    robot = lidar.ObstacleAvoidance()
    robot.start()
    sleep(1)        # Wait for LiDAR to initialize

    try:
        while True:
            sleep(.05)                                          

            # Lidar Obstacle Check
            data = robot.lidar.get()

            # Filter only front angles
            front_data = []
            for d, angle in data:
                if -robot.front_angle/2 <= angle <= robot.front_angle/2:
                    if d > 0:  # ignore invalid (-1)
                        front_data.append([d, angle])

            obstacle, dist = robot.detect_obstacle(front_data)

            if obstacle:
                direction = robot.decide_direction(front_data)
                print(f"Obstacle at {dist:.2f} m. TURN {direction}")

            else:
                print("Path is clear. MOVE FORWARD")
                direction = False

            # Process Lidar Info
            if direction == "LEFT":
                sc.driveOpenLoop(np.array([2.0,5.0]))      # Robot rotates left
                sleep(0.25)
            elif direction == "RIGHT":
                sc.driveOpenLoop(np.array([5.0,2.0]))      # Robot rotates right
                sleep(0.25)
            elif direction == False:                         # If False, check camera

                # Color Tracking Code
                ret, image = camera.read()  # Get image from camera

                # Make sure image was grabbed
                if not ret:
                    print("Failed to retrieve image!")
                    break
                
                image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)              # Convert image to HSV

                height, width, channels = image.shape                       # Get shape of image

                thresh = cv2.inRange(image, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))   # Find all pixels in color range

                kernel = np.ones((5,5),np.uint8)                            # Set kernel size
                mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)     # Open morph: removes noise w/ erode followed by dilate
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)      # Close morph: fills openings w/ dilate followed by erode
                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
                
                if len(cnts) > 0:                             # If more than 0 and less than 3 closed shapes exist

                    c = max(cnts, key=cv2.contourArea)                      # return the largest target area
                    x,y,w,h = cv2.boundingRect(c)                           # Get bounding rectangle (x,y,w,h) of the largest contour
                    center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                    angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                    wheel_measured = kin.getPdCurrent()                     # Wheel speed measurements

                    # If robot is facing target
                    if abs(angle) < angle_margin:                                 
                        e_width = target_width - w                          # Find error in target width and measured width

                        # If error width is within acceptable margin
                        if abs(e_width) < width_margin:
                            sc.driveOpenLoop(np.array([0.,0.]))             # Stop when centered and aligned
                            print("Aligned! ",w)
                            continue

                        fwd_effort = e_width/target_width                   
                        
                        wheel_speed = ik.getPdTargets(np.array([0.8*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                        sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                        print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
                        continue

                    wheel_speed = ik.getPdTargets(np.array([0, -0.8*angle]))    # Find wheel speeds for only turning

                    sc.driveClosedLoop(wheel_speed, wheel_measured, 0)          # Drive robot
                    print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
                    

                else:
                    print("No targets")
                    sc.driveOpenLoop(np.array([-3.5,3.5]))         # scan if no targets detected
                    sleep (0.02)
                    continue
                    
                
                    
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        pass

    finally:
        print("Exiting Color Tracking.")
        sc.driveOpenLoop(np.array([0,0]))
        robot.stop()
        

if __name__ == '__main__':
    main()