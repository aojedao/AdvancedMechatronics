from picamera2 import Picamera2, Preview
import time

# Initialize the camera
camera = Picamera2()

# Step 1: Display the Preview
def display_preview():
    # Configure the camera for preview
    preview_config = camera.create_preview_configuration()
    camera.configure(preview_config)

    # Start the preview
    camera.start_preview(Preview.QTGL)  # Use QTGL for graphical environments
    camera.start()

    # Wait for 2 seconds to allow the preview to initialize
    print("Displaying preview for 2 seconds...")
    time.sleep(2)

    # Stop the preview
    camera.stop_preview()
    camera.stop()
    print("Preview stopped.")

# Step 2: Capture Images with Annotations
def capture_images():
    # Configure the camera for still image capture
    still_config = camera.create_still_configuration()
    camera.configure(still_config)

    # Start the camera
    camera.start()

    # Store letters from A to J in a list
    letters = [chr(65 + i) for i in range(10)]  # ['A', 'B', ..., 'J']

    # Capture images with annotations
    for index, letter in enumerate(letters):
        # Capture the image
        filename = f"image_{letter}.jpg"
        camera.capture_file(filename)
        print(f"Captured {filename} with annotation '{letter}'")

        # Wait for 1 second before capturing the next image
        time.sleep(1)

    # Stop the camera
    camera.stop()
    print("Image capture completed.")

# Main function to execute both steps
if __name__ == "__main__":
    display_preview()  # Step 1: Display the preview
    capture_images()   # Step 2: Capture images
