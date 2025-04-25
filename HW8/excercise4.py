import cv2

# Load the image
image = cv2.imread('image.jpg')  # Replace 'your_image.jpg' with the path to your image

# Check if the image was loaded successfully
if image is None:
    print("Error: Could not load image.")
    exit()

# Resize the image to 640x480 for display
resized_image = cv2.resize(image, (640, 480))

# Display the resized image
cv2.imshow('Pre Resized', resized_image)

# Wait for the user to press Enter
print("Press Enter to resize the image to half its size and convert to grayscale...")
cv2.waitKey(0)  # Wait for any key press

# Resize the image to half its original size
height, width = image.shape[:2]
half_resized_image = cv2.resize(image, (640 // 2, 480 // 2))

# Display the half-resized image
cv2.imshow('Half-Resized Image', half_resized_image)

# Wait for the user to press Enter
print("Press Enter to convert the image to grayscale...")
cv2.waitKey(0)  # Wait for any key press

# Convert the image to grayscale
gray_image = cv2.cvtColor(half_resized_image, cv2.COLOR_BGR2GRAY)

# Display the grayscale image
cv2.imshow('Grayscale Image', gray_image)

# Wait for the user to press Enter before closing
print("Press Enter to close the program...")
cv2.waitKey(0)

# Clean up and close all OpenCV windows
cv2.destroyAllWindows()
