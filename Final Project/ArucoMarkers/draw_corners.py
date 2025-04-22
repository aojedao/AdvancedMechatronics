import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
 
# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Detect corners on a chessboard
 
filename = '/home/user/Documents/NYU/AdvancedMechatronics/Final Project/ArucoMarkers/Calibration/2025-04-18-130937.jpg'
 
# Chessboard dimensions
number_of_squares_X = 10 # Number of chessboard squares along the x-axis
number_of_squares_Y = 7  # Number of chessboard squares along the y-axis
nX = number_of_squares_X - 1 # Number of interior corners along x-axis
nY = number_of_squares_Y - 1 # Number of interior corners along y-axis
 
def main():
     
  # Load an image
  image = cv2.imread(filename)
 
  # Convert the image to grayscale
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
 
  # Find the corners on the chessboard
  success, corners = cv2.findChessboardCorners(gray, (nY, nX), None)
     
  # If the corners are found by the algorithm, draw them
  if success == True:
 
    # Draw the corners
    cv2.drawChessboardCorners(image, (nY, nX), corners, success)
 
    # Create the output file name by removing the '.jpg' part
    size = len(filename)
    new_filename = filename[:size - 4]
    new_filename = new_filename + '_drawn_corners.jpg'     
     
    # Save the new image in the working directory
    cv2.imwrite(new_filename, image)
 
    # Display the image 
    cv2.imshow("Image", image) 
     
    # Display the window until any key is pressed
    cv2.waitKey(0) 
     
    # Close all windows
    cv2.destroyAllWindows() 
     
main()