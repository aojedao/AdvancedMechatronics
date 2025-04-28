import cv2
import time

# Load the image
image_path = r'image.jpg'  # Replace with the path to your image
image = cv2.imread(image_path)

# Check if the image was loaded successfully
if image is None:
    print("Error loading image.")
    exit()

# Perform Canny edge detection with different thresholds
thresholds = [(50, 200), (150, 200), (150, 250)]

# Create a single window and set its size to 640x480
cv2.namedWindow('Canny Edge Detection', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Canny Edge Detection', 640, 480)

while True:
    for low, high in thresholds:
        # Perform Canny edge detection
        edges = cv2.Canny(image, low, high)
        
        # Convert the single-channel edge image to a 3-channel image
        edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        
        # Add text showing the threshold values
        text = f"Thresholds: {low}, {high}"
        cv2.putText(edges_colored, text, (10, 30), 
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale=0.8, 
                    color=(0, 255, 0),  # Green text
                    thickness=2)
        
        # Display the edge-detected image with text
        cv2.imshow('Canny Edge Detection', edges_colored)

        # Wait 500 ms and check for key press
        key = cv2.waitKey(500)
        if key != -1:  # Break if any key is pressed
            break
    if key != -1:  # Break the outer loop if any key is pressed
        break

# Clean up and close all OpenCV windows
cv2.destroyAllWindows()
print("Program terminated.")
