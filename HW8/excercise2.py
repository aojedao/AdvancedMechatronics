import time
from picamera import PiCamera

#if we want to save the image to a specific folder
#save_folder = '/home/pi/captured_images/' 

camera = PiCamera()
camera.resolution = (640, 480)
camera.start_preview()
time.sleep(5)

contrast_values = [-75, -50, -25, 0, 25, 50, 75]
for contrast in contrast_values:
    camera.contrast = contrast
    #to save file in the folder.
    #filename = f'{save_folder}image_contrast_{contrast}.jpg'
    camera.capture(f'image_contrast_{contrast}.jpg')
    time.sleep(1)

camera.stop_preview()
