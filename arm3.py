import cv2
import numpy as np
import math
import time

# Camera Stream
CAMERA_URL = "http://192.168.0.100:4747/video"
FRAME_WIDTH = 600
FRAME_HEIGHT = 600

# Color Ranges
COLOR_RANGES = {
    "blue": np.array([[60, 30, 20], [112, 255, 255]]),
    "green": np.array([[69, 155, 0], [80, 255, 255]])
}

# Global Position Variables
POS_X, POS_Y = None, None
SELECTED_COLOR = ""

def get_color_input():
    global SELECTED_COLOR
    SELECTED_COLOR = input("Enter object color to detect (blue or green): ").strip().lower()
    while SELECTED_COLOR not in COLOR_RANGES:
        SELECTED_COLOR = input("Invalid color. Enter 'blue' or 'green': ").strip().lower()

def process_frame(frame, color_range):
    """Processes the frame to detect object and find its coordinates."""
    img = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
    img_cropped = img[35:570, 190:555]
    img_resized = cv2.resize(img_cropped, (FRAME_WIDTH, FRAME_HEIGHT))
    hsv_img = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)

    lower, upper = color_range
    mask = cv2.inRange(hsv_img, lower, upper)
    result = cv2.bitwise_and(img_resized, img_resized, mask=mask)
    result = cv2.GaussianBlur(result, (5, 5), 0)
    
    img_gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(img_gray, 50, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return img_resized, result, contours

def get_object_coordinates(contours, img_shape):
    """Finds the largest contour and calculates its centroid."""
    global POS_X, POS_Y
    if not contours:
        return None, None
    
    max_contour = max(contours, key=cv2.contourArea, default=None)
    if max_contour is None or cv2.contourArea(max_contour) < 500:
        return None, None

    M = cv2.moments(max_contour)
    if M["m00"] == 0:
        return None, None
    
    # Image coordinates of centroid
    img_x = int(M["m10"] / M["m00"])
    img_y = int(M["m01"] / M["m00"])
    
    # Flip Y-axis to convert from image coordinates (top-left origin) to Cartesian-like (bottom-left origin)
    flipped_img_y = FRAME_HEIGHT - img_y

    # Normalize coordinates to desired ranges
    normalized_x = (img_x / img_shape[1]) * 10  # X: 0 to 10 (left to right)
    normalized_y = (flipped_img_y / img_shape[0]) * 20 - 10  # Y: -10 to 10 (bottom to top)

    # Flip 180 degrees around the center (5, 0)
    flipped_x = 10 - normalized_x  # Invert X
    flipped_y = -normalized_y      # Invert Y

    POS_X, POS_Y = flipped_x, flipped_y
    return POS_X, POS_Y

def main():
    get_color_input()
    cap = cv2.VideoCapture(CAMERA_URL)
    
    if not cap.isOpened():
        print("Error: Unable to open video stream.")
        return
    
    frame_counter = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture video stream.")
            break

        img_resized, result, contours = process_frame(frame, COLOR_RANGES[SELECTED_COLOR])
        pos_x, pos_y = get_object_coordinates(contours, img_resized.shape)
        
        if pos_x is None or pos_y is None:
            print("Object not detected!")
        elif frame_counter % 100 == 0:
            print(f"Coordinates: X={pos_x:.2f}, Y={pos_y:.2f}")
        
        # Draw vertical line at X=0 (middle of the frame)
        middle_x = img_resized.shape[1] // 2
        cv2.line(img_resized, (middle_x, 0), (middle_x, img_resized.shape[0]), (0, 0, 255), 2)

        cv2.imshow("Original Image", img_resized)
        cv2.imshow("Processed Image", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        frame_counter += 1
        time.sleep(0.1)
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
