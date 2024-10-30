# Import the opencv library
import cv2 as cv

# You can change *1 and *2 lines of code. Set resolution to a size you like and
# you can see this when you run the program, but it should not be bigger than 
# the maximum resolution of your webcam!

# Create variable that captures video feed. 
# ‘0’ refers to the only connected webcam. ‘1’, ‘2’, etc would refer to more 
# connected webcams
cap = cv.VideoCapture(0)
# Set video frame rate to 30 fps. Maximum fps is 30           *1
cap.set(cv.CAP_PROP_FPS, 30)

while True:
    # Captures an image 'pic1' and checks whether operation was successful
    ret, pic1 = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break
    
    # Resizes the captured pic1 into a 480x480 resolution     *2
    pic2 = cv.resize(pic1, (480,480))

    # Display resized frame
    cv.imshow('Frame', pic2)

    # If the key 'c' is pressed, the program is shut down
    if cv.waitKey(1) & 0xFF == ord('c'):
        break

# Shuts down video feed and closes video feed window
cap.release()
cv.destroyAllWindows()
