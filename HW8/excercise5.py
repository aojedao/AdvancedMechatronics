import cv2

# Load the image
image = cv2.imread('your_image.jpg')

# Perform Canny edge detection with different thresholds
thresholds = [(50, 200), (150, 200), (150, 250)]
for i, (low, high) in enumerate(thresholds):
    edges = cv2.Canny(image, low, high)
    cv2.imshow(f'Edges with thresholds {low}, {high}', edges)
    cv2.waitKey(0)

cv2.destroyAllWindows()
