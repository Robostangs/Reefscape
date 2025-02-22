import cv2
import numpy as np

def drawDecorations(image, area):
    cv2.putText(image,
                f'PVC Pipe Area: {area:.2f}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 255, 0), 2, cv2.LINE_AA)

def runPipeline(image, llrobot):
    # Convert BGR image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define range for white/light gray color in HSV
    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 30, 255])
    
    # Threshold the HSV image to get only white/light gray colors
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Morphological operations to remove small noise
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize variables
    largest_contour = np.array([[]])
    area = 0
    x, y, w, h = 0, 0, 0, 0
    
    if contours:
        # Find the largest contour (assumed to be the PVC pipe)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get the bounding rectangle
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Draw the rectangle on the original image
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Calculate the area
        area = w * h
    
    # Draw the area on the image
    drawDecorations(image, area)
    
    # Prepare llpython array with useful information
    llpython = [1 if contours else 0, x, y, w, h, area, 0, 0]
    
    # Return the largest contour, the processed image, and the llpython array
    return largest_contour, image, llpython