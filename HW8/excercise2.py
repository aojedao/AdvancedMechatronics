from picamera2 import Picamera2, Preview
import time

# Initialize the camera
picam2 = Picamera2()

# Step 1: Display the Preview
def display_preview():
    # Configure the camera for preview
    camera_config = picam2.create_preview_configuration()
    picam2.configure(camera_config)

    # Start the camera preview
    picam2.start_preview(Preview.QTGL)  # Use QTGL for graphical environments
    picam2.start()

    # Display the preview for 5 seconds
    print("Displaying preview for 5 seconds...")
    time.sleep(5)

    # Stop the preview
    picam2.stop_preview()
    picam2.stop()
    print("Preview stopped.")

# Step 2: Capture Images with Different Contrast Values
def capture_images():
    # Configure the camera for still image capture
    camera_config = picam2.create_still_configuration(main={"size": (640, 480)})
    picam2.configure(camera_config)

    # Start the camera
    picam2.start()

    # List of contrast values
    contrast_values = [-75, -50, -25, 0, 25, 50, 75]

    # Capture images with different contrast values
    for contrast in contrast_values:
        # Set the contrast value
        picam2.set_controls({"Contrast": contrast})

        # Wait for 1 second to apply the contrast setting
        time.sleep(1)

        # Capture the image and save it with the contrast value in the filename
        filename = f"image_contrast_{contrast}.jpg"
        picam2.capture_file(filename)
        print(f"Captured {filename}")

    # Stop the camera
    picam2.stop()
    print("Image capture completed.")

# Main function to execute both steps
if __name__ == "__main__":
    display_preview()  # Step 1: Display the preview
    capture_images()   # Step 2: Capture images
