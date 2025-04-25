import cv2

# Load the image
image = cv2.imread('your_image.jpg')

# Resize the image to half its original size
resized_image = cv2.resize(image, (image.shape[1] // 2, image.shape[0] // 2))
cv2.imshow('Resized Image', resized_image)
cv2.waitKey(0)

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imshow('Grayscale Image', gray_image)
cv2.waitKey(0)

cv2.destroyAllWindows()
