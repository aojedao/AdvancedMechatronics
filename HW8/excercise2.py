import time
import picamera

contrast_values = [-75, -50, -25, 0, 25, 50, 75]

with picamera.PiCamera() as camera:
    camera.start_preview()
    time.sleep(5)  # Preview for 5 seconds
    
    for contrast in contrast_values:
        camera.contrast = contrast
        filename = f'image_contrast_{contrast}.jpg'
        camera.capture(filename)
        time.sleep(1)  # Capture at 1-second intervals
