import cv2
import numpy as np

def runPipeline(image, llrobot):
    # Convert BGR image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define range for purple color in HSV
    lower_purple = np.array([130, 50, 50])
    upper_purple = np.array([160, 255, 255])

    # Create a mask for purple color
    mask = cv2.inRange(hsv, lower_purple, upper_purple)

    # Noise reduction
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPHOLOGY_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPHOLOGY_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize variables
    largest_contour = np.array([[]])
    tx, ty = 0, 0
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]

    if contours:
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Filter based on aspect ratio (assuming the reef is more wide than tall)
        x, y, w, h = cv2.boundingRect(largest_contour)
        aspect_ratio = w / float(h)
        
        if aspect_ratio > 2:  # Adjust this value based on the actual reef's shape
            # Calculate center of the contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Calculate Tx and Ty
                tx = (cx - image.shape[1]/2) / (image.shape[1]/2)  # -1 to 1
                ty = (image.shape[0]/2 - cy) / (image.shape[0]/2)  # -1 to 1

                # Draw contour and center point
                cv2.drawContours(image, [largest_contour], 0, (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)

                # Draw Tx and Ty values on image
                cv2.putText(image, f"Tx: {tx:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(image, f"Ty: {ty:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                # Populate llpython array
                llpython = [1, tx, ty, aspect_ratio, 0, 0, 0, 0]

    # Draw crosshair at the center of the image
    h, w = image.shape[:2]
    cv2.line(image, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 0, 255), 2)
    cv2.line(image, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 0, 255), 2)

    return largest_contour, image, llpython