import cv2


image_path = 'path/to/your/image.jpg'  # <-- image path. Please use Madonna 😉

# Load the image using the path
image = cv2.imread(image_path)

if image is None:
    print(f"Error: Could not load image from {image_path}")
    exit()

# Show the original image
cv2.imshow('Original Image', image)
cv2.waitKey(0)
print("Press any key to resize the image")

#Resize the image to half its size
resized_image = cv2.resize(image, (image.shape[1] // 2, image.shape[0] // 2))
cv2.imshow('Resized Image', resized_image)
cv2.waitKey(0)
print("Press any key to convert the image to greyscale")

#convert to grey image
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow('Grayscale Image', gray_image)
cv2.waitKey(0)

# Close all windows
cv2.destroyAllWindows()

# Final message
print("Program terminated.")
