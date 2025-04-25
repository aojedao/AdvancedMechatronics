import cv2
import time

image_path = r'C:\Users\Acer\Downloads\your_image.jpg'
image = cv2.imread(image_path)

if image is None:
    print("Error loading image.")
    exit()

# Perform Canny edge detection with different thresholds
thresholds = [(50, 200), (150, 200), (150, 250)]

# Create a single window
cv2.namedWindow('Canny Edge Detection', cv2.WINDOW_NORMAL)

while True:
    for low, high in thresholds:
        edges = cv2.Canny(image, low, high)
        
        # Add text showing the threshold values
        text = f"{low}, {high}"
        cv2.putText(edges, text, (10, 15), 
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                    fontScale=0.4, 
                    color=(255, 255, 255),  # White text
                    thickness=1)
        
        cv2.imshow('Canny Edge Detection', edges)

        # Wait 500 ms and check for key press
        key = cv2.waitKey(500)
        if key != -1:
            break
    if key != -1:
        break

cv2.destroyAllWindows()
print("Program terminated.")
