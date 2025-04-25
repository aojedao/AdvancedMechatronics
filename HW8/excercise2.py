import time
from picamera import PiCamera

camera = PiCamera()
camera.start_preview()
time.sleep(5)

contrast_values = [-75, -50, -25, 0, 25, 50, 75]
for contrast in contrast_values:
    camera.contrast = contrast
    camera.capture(f'image_contrast_{contrast}.jpg')
    time.sleep(1)

camera.stop_preview()
