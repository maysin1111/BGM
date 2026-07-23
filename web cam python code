import cv2
import numpy as np

# ==========================
# Camera Setup
# ==========================
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("ERROR: Could not open camera.")
    exit()

print("Camera started successfully!")
print("Press 'q' to quit.")

# ==========================
# Main Loop
# ==========================
while True:

    # Capture a frame
    ret, image = cap.read()

    if not ret:
        print("Failed to grab frame.")
        break

    # Convert image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Green color range
    lower_green = np.array([25, 30, 30])
    upper_green = np.array([110, 255, 255])

    # Create mask
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Remove noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask,
                                   cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:

        # Find the largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        # Ignore tiny objects
        if area > 500:

            # Bounding rectangle
            x, y, w, h = cv2.boundingRect(largest)

            # Center point
            center_x = x + w // 2
            center_y = y + h // 2

            # Draw contour
            cv2.drawContours(image, [largest], -1, (0, 0, 255), 2)

            # Draw bounding box
            cv2.rectangle(image,
                          (x, y),
                          (x + w, y + h),
                          (0, 255, 0),
                          2)

            # Label
            cv2.putText(image,
                        "Corn",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2)

            # Center dot
            cv2.circle(image,
                       (center_x, center_y),
                       6,
                       (255, 0, 0),
                       -1)

            # Crosshair
            cv2.line(image,
                     (center_x - 10, center_y),
                     (center_x + 10, center_y),
                     (255, 0, 0),
                     2)

            cv2.line(image,
                     (center_x, center_y - 10),
                     (center_x, center_y + 10),
                     (255, 0, 0),
                     2)

            # Coordinates on image
            cv2.putText(image,
                        f"({center_x}, {center_y})",
                        (center_x + 15, center_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 0, 0),
                        2)

            # Print information
            print("--------------------------------")
            print("Corn Detected")
            print("Center X :", center_x)
            print("Center Y :", center_y)
            print("Width    :", w)
            print("Height   :", h)
            print("Area     :", int(area))

    # Show camera
    cv2.imshow("Corn Detector", image)

    # Show mask (optional)
    cv2.imshow("Green Mask", mask)

    # Press Q to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ==========================
# Cleanup
# ==========================
cap.release()
cv2.destroyAllWindows()
