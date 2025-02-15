import cv2
import numpy as np
import time

# Camera Stream
CAMERA_URL = "http://192.168.0.130:4747/video"
FRAME_WIDTH = 600
FRAME_HEIGHT = 600

# Color Ranges
COLOR_RANGES = {
    "blue": np.array([[60, 30, 20], [112, 255, 255]]),
    "green": np.array([[50, 50, 60], [110, 255, 255]])
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
    img_cropped = img[70:500, 200:515]  # [width, height]
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
        return None, None, None, None
    
    max_contour = max(contours, key=cv2.contourArea, default=None)
    if max_contour is None or cv2.contourArea(max_contour) < 500:
        return None, None, None, None

    M = cv2.moments(max_contour)
    if M["m00"] == 0:
        return None, None, None, None
    
    img_x = int(M["m10"] / M["m00"])
    img_y = int(M["m01"] / M["m00"])
    
    flipped_img_y = FRAME_HEIGHT - img_y
    normalized_x = 20 - (img_x / (img_shape[1]-1)) * 20  # Corrected mapping
    normalized_y = (flipped_img_y / (img_shape[0]-1)) * 20 - 10
    
    POS_X, POS_Y = normalized_x, -normalized_y
    return POS_X, POS_Y, img_x, img_y

def transform_to_pixel_coordinates(x, y, img_shape):
    """Converts transformed coordinates to pixel coordinates."""
    img_x = int((20 - x) * (img_shape[1]-1) / 20)  # Corrected mapping
    img_y = (img_shape[0]-1) - int((y + 10) * (img_shape[0]-1) / 20)
    return img_x, img_y

def draw_grid(img):
    """Draws grid lines and labels on the image."""
    # Vertical lines (x from 0 to 20)
    for x in range(0, 21):
        pixel_x = int((20 - x) * (FRAME_WIDTH-1) / 20)
        color = (100, 100, 100) if x % 5 == 0 else (50, 50, 50)
        thickness = 2 if x % 5 == 0 else 1
        cv2.line(img, (pixel_x, 0), (pixel_x, FRAME_HEIGHT-1), color, thickness)
        if x % 5 == 0:
            cv2.putText(img, str(x), (pixel_x + 5, FRAME_HEIGHT - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    # Horizontal lines (y from -10 to 10)
    for y in range(-10, 11):
        pixel_y = int((FRAME_HEIGHT-1) - (y + 10) * (FRAME_HEIGHT-1) / 20)
        color = (100, 100, 100) if y % 5 == 0 else (50, 50, 50)
        thickness = 2 if y % 5 == 0 else 1
        cv2.line(img, (0, pixel_y), (FRAME_WIDTH-1, pixel_y), color, thickness)
        if y % 5 == 0:
            cv2.putText(img, str(y), (5, pixel_y - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

def main():
    get_color_input()
    cap = cv2.VideoCapture(CAMERA_URL)
    
    if not cap.isOpened():
        print("Error: Unable to open video stream.")
        return
    
    start_time = time.time()
    warmup_duration = 10  # 10-second warmup
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Unable to capture video stream.")
            break

        img_resized, result, contours = process_frame(frame, COLOR_RANGES[SELECTED_COLOR])
        
        # Draw grid system
        draw_grid(img_resized)
        
        pos_x, pos_y, img_x, img_y = get_object_coordinates(contours, img_resized.shape)
        
        elapsed = time.time() - start_time

        # Convert (0,0) in transformed coordinates to pixel coordinates
        zero_x, zero_y = transform_to_pixel_coordinates(0, 0, img_resized.shape)

        if elapsed >= warmup_duration:
            if pos_x is None or pos_y is None:
                print("Object not detected!")
            else:
                print(f"Coordinates: X={pos_x:.2f}, Y={pos_y:.2f}")
                cv2.putText(img_resized, f"({pos_x:.2f}, {pos_y:.2f})", (img_x, img_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(img_resized, (img_x, img_y), 5, (0, 0, 255), -1)  # Red dot at object centroid
        
        # Draw the (0,0) reference point
        cv2.circle(img_resized, (zero_x, zero_y), 6, (255, 0, 0), -1)  # Blue dot for (0,0)
        cv2.putText(img_resized, "(0,0)", (zero_x + 10, zero_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow("Original Image", img_resized)
        cv2.imshow("Processed Image", result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        time.sleep(0.1)
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()