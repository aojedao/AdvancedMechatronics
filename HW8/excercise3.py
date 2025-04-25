from picamera import PiCamera
import time

camera = PiCamera()
camera.start_preview()
time.sleep(2)

for i in range(10):
    letter = chr(65 + i)  # ASCII value for 'A' is 65
    camera.annotate_text = letter
    camera.capture(f'image_{letter}.jpg')
    time.sleep(1)

camera.stop_preview()
