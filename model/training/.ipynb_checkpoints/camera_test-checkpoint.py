from ultralytics import YOLO
import cv2


model = YOLO("runs/detect/robocon_test_model/weights/best.pt")


cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ Camera not working")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    
    results = model(frame, conf=0.5)

    
    annotated = results[0].plot()

    
    cv2.imshow("Detection", annotated)

    
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()