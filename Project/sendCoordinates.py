from smbus2 import SMBus
import cv2 as cv
import numpy as np
from time import sleep

# Set bus address to 0x8
addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/i2c-1. Use this bus.

# '***' stands for things to modify for your own webcam, display, and ball if needed

# Define a function to detect a yellow ball
def detect_yellow_ball():
    # Start capturing video from the webcam. If multiple webcams connected, you may use 1,2, etc.
    cap = cv.VideoCapture(0)
    # *1 CAP_PROP_FPS sets the frame rate of the webcam to 30 fps here
    cap.set(cv.CAP_PROP_FPS, 30)
    
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # *2 Set the image resolution to 480x480. Note increasing resolution increases processing power used, and may slow down video feed.
        frame = cv.resize(frame, (1920, 1080))

        # Convert the frame from BGR to HSV color space to easily identify a colour
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 

        # *3 Define the range of yellow color in HSV [Hue, Saturation, Value]
        # SET THESE VALUES VIA THE METHOD EXPLAINED IN THE TUTORIAL
        ball_color_lower = np.array([20, 100, 100]) # [lower Hue, lower Saturation, lower Value]
        ball_color_upper = np.array([30, 255, 255]) # [upper Hue, upper Saturation, upper Value]

        # Threshold the HSV image to get the colors defined above
        # Pixels in the range are set to white (255) and those that aren't are set to black (0), creating a binary mask 
        mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)

        # Find contours in the mask
        # RETR_TREE retrieves all hierarchical contours and organizes them
        # CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments, leaving only their end points
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        if contours:
            # Determines the larget contour size using the cv.contour Area function
            largest_contour = max(contours, key=cv.contourArea)
            # Computes the minimum enclosing circle aroudn the largest contour
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            # * 4 Only consider large enough objects. If it only detects a small portion of your ball, you can test higher radius values to capture more of the ball
            if radius > 10:
                # Draw a yellow circle around the ball
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # Draw a red dot in the center of the ball
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)  # (image to draw dot on, x,y pixel coordinates, radius in pixels, RGB values in this case red, -1 indicates to fill the circle)
                # Display the position of the ball
                print(f"Yellow ball detected at position: ({int(x)}, {int(y)})")

                # send the coords to arudino
                bus.write_byte(addr,int(x))

  

        # Display the resulting frame
        cv.imshow('frame', frame)

        # Break the loop when 'q' is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture when everything is done
    cap.release()
    # Close all windows
    cv.destroyAllWindows()




# # if user inputs an integer other than 1 or 0, program ends
# while True:

#     ledstate = input(">>>>>>>     ")
#     # Switch on
#     if ledstate == "1":
# 	  # Sends a byte of data 0x1 to address ‘addr’ which is the Arduino Mega
#         bus.write_byte(addr, 0x1)
#     # Switch off
#     elif ledstate == "0":
#         bus.write_byte(addr, 0x0)
#     # If input ledstate is not 0 or 1, while loop is broken
#     else:
#         number = 0
