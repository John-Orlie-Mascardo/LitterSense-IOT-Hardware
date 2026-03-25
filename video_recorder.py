import cv2
import datetime
import os
from ultralytics import YOLO

# ── Configuration ──
STREAM_URL = 0  # Replace with your ESP32-CAM stream URL (e.g., "http://192.168.1.100:81/stream")
MODEL_PATH = "yolov8n.pt"  # Path to your YOLOv8 model
OUTPUT_DIR = "recordings"
CAT_CLASS_ID = 15
CONFIDENCE_THRESHOLD = 0.5
MOTION_THRESHOLD = 5000  # Minimum contour area to count as motion
COOLDOWN_FRAMES = 50  # Keep recording for this many frames after last trigger
CODEC = "XVID"
FPS = 20.0

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Load model & open stream ──
model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    raise RuntimeError("Cannot open video stream.")

frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# ── State variables ──
prev_gray = None
writer = None
cooldown_counter = 0

print("[INFO] Dual-trigger detection started. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("[WARN] Frame grab failed, retrying...")
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    # ── Primary Trigger: YOLOv8 cat detection ──
    ai_triggered = False
    results = model(frame, verbose=False)
    for box in results[0].boxes:
        if int(box.cls[0]) == CAT_CLASS_ID and float(box.conf[0]) >= CONFIDENCE_THRESHOLD:
            ai_triggered = True
            break

    # ── Fallback Trigger: Frame differencing (motion) ──
    motion_triggered = False
    if prev_gray is not None:
        delta = cv2.absdiff(prev_gray, gray)
        thresh = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) >= MOTION_THRESHOLD:
                motion_triggered = True
                break

    prev_gray = gray

    # ── Evaluate triggers ──
    triggered = ai_triggered or motion_triggered

    if triggered:
        if ai_triggered and motion_triggered:
            reason = "Triggered by Both"
        elif ai_triggered:
            reason = "Triggered by AI"
        else:
            reason = "Triggered by Motion"

        cooldown_counter = COOLDOWN_FRAMES

        # Start a new recording if not already recording
        if writer is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(OUTPUT_DIR, f"rec_{timestamp}.avi")
            fourcc = cv2.VideoWriter_fourcc(*CODEC)
            writer = cv2.VideoWriter(filename, fourcc, FPS, (frame_w, frame_h))
            print(f"[REC START] {filename} — {reason}")
        else:
            print(f"[REC] {reason}")

    # ── Write frame or wind down cooldown ──
    if writer is not None:
        writer.write(frame)
        if not triggered:
            cooldown_counter -= 1
            if cooldown_counter <= 0:
                writer.release()
                writer = None
                print("[REC STOP] Cooldown expired, recording saved.")

    # ── Display (optional, remove in headless deployment) ──
    status = "REC" if writer is not None else "IDLE"
    cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                (0, 0, 255) if status == "REC" else (0, 255, 0), 2)
    cv2.imshow("LitterSense", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# ── Cleanup ──
if writer is not None:
    writer.release()
cap.release()
cv2.destroyAllWindows()
print("[INFO] Shut down cleanly.")