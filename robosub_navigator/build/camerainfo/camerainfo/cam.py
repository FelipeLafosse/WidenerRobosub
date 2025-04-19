import cv2

cap = cv2.VideoCapture(2)  # Try different indices (0, 1, 2, etc.)
if not cap.isOpened():
    print("Error: Camera not found")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
