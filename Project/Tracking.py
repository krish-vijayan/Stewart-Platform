import cv2 as cv
import numpy as np
from time import sleep

# Define a function to detect a green ball
def detect_green_ball():
    # Start capturing video from the webcam
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FPS, 30)
    
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Set the image resolution if needed (e.g., 1920x1080 for high resolution)
        frame = cv.resize(frame, (1920, 1080))

        # Convert the frame from BGR to HSV color space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV) 

        # Define the range of green color in HSV
        ball_color_lower = np.array([35, 100, 100])  # Lower boundary for green
        ball_color_upper = np.array([85, 255, 255])  # Upper boundary for green

        # Threshold the HSV image to get only green colors
        mask = cv.inRange(hsv, ball_color_lower, ball_color_upper)

        # Find contours in the mask
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Find the index of the largest contour
        if contours:
            largest_contour = max(contours, key=cv.contourArea)
            ((x, y), radius) = cv.minEnclosingCircle(largest_contour)
            if radius > 10:
                # Draw a green circle around the ball
                cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                # Draw a red dot in the center of the ball
                cv.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
                
                # Display the position of the ball on the frame
                coordinates_text = f"Position: ({int(x)}, {int(y)})"
                cv.putText(frame, coordinates_text, (int(x), int(y) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Print the position of the ball in the console
                print(f"Green ball detected at position: ({int(x)}, {int(y)})")

        # Display the resulting frame
        cv.imshow('frame', frame)

        # Break the loop when 'q' is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close windows
    cap.release()
    cv.destroyAllWindows()

# Call the function to detect the green ball
detect_green_ball()
