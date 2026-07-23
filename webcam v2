import cv2

# Open the default webcam (0)
cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press 'q' to quit.")

while True:
    # Read a frame
    ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame.")
        break

    # Display the frame
    cv2.imshow("USB Webcam", frame)

    # Press q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.release()
cv2.destroyAllWindows()
