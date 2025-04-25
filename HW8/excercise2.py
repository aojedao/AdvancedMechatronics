from picamera2 import Picamera2, Preview
import time

# Initialize the camera
picam2 = Picamera2()

# Configure the camera with a preview configuration
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))

# Start the camera preview using DRM (Direct Rendering Manager)
picam2.start_preview(Preview.DRM)  # Use DRM preview to avoid event loop conflicts
time.sleep(5)  # Allow the preview to display for 5 seconds

# Start the camera
picam2.start()

# List of contrast values to apply
contrast_values = [-75, -50, -25, 0, 25, 50, 75]

# Capture images with different contrast values
for contrast in contrast_values:
    # Adjust contrast using the camera's controls
    picam2.set_controls({"Contrast": contrast})
    # Capture and save the image with the contrast value in the filename
    picam2.capture_file(f"image_contrast_{contrast}.jpg")
    time.sleep(1)  # Wait for 1 second before capturing the next image

# Stop the camera and the preview
picam2.stop_preview()  # Stop the preview
picam2.stop()  # Stop the camera
