from picamera import PiCamera
import time

camera = PiCamera()

camera.start_preview()
time.sleep(2)

# store letters from A to J in the list
letters = [chr(65 + i) for i in range(10)]  # ['A', 'B', ..., 'J']

# Capture images continuously
for filename in camera.capture_continuous('image_{counter}.jpg'):
    index = int(filename.split('_')[-1].split('.')[0])  # Get the current count from 'image_{counter}.jpg'. {counter} part is from 0 to n
    if index < len(letters): #check if the values is less than 10
        letter = letters[index]
        camera.annotate_text = letter
        new_filename = f'image_{letter}.jpg'
        camera.capture(new_filename)
        print(f"Captured {new_filename} with annotation '{letter}'")
        time.sleep(1)
    else:
        break

camera.stop_preview()
camera.close()

