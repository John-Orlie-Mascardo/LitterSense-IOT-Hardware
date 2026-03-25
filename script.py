import cv2
import os
import time
import threading
from ultralytics import YOLO


# --- BACKGROUND DELETION TASK ---
def schedule_deletion(filepath, delay_seconds):
    def delete_task():
        time.sleep(delay_seconds)
        if os.path.exists(filepath):
            os.remove(filepath)
            print(f"\n🗑️ Auto-deleted test video: {os.path.basename(filepath)}")

    thread = threading.Thread(target=delete_task)
    thread.start()


# --- SETUP YOLO AI & CAMERA ---
model = YOLO('yolov8n.pt')
stream_url = "http://192.168.68.115:81/stream"
print(f"Connecting to ESP32-CAM at {stream_url}...")

cap = cv2.VideoCapture(stream_url)

# FIX 1: Prevent the live stream from lagging behind reality
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("Error: Could not open the video stream.")
    exit()

# --- SETUP VIDEO VARIABLES ---
SAVE_DIR = 'cat_videos'
os.makedirs(SAVE_DIR, exist_ok=True)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# FIX 2: Lower the playback speed to match your laptop's AI processing speed
fps = 10.0
fourcc = cv2.VideoWriter.fourcc(*'XVID')

# --- SMART DVR LOGIC VARIABLES ---
is_recording = False
out = None
frames_missing = 0
COOLDOWN_FRAMES = 30  # Waits ~1.5 seconds after the cat leaves before cutting the video

print("\n🚀 LitterSense Smart DVR is active!")
print("Hold up a picture of a cat to your camera to start recording...")

while True:
    ret, frame = cap.read()
    if not ret: break

    # Run AI on the frame
    results = model(frame, conf=0.5, classes=[15])
    annotated_frame = results[0].plot()

    # Did the AI see a cat this frame? True or False
    cat_in_frame = len(results[0].boxes) > 0

    if cat_in_frame:
        frames_missing = 0  # Reset the countdown

        # If we aren't recording yet, START RECORDING!
        if not is_recording:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            save_path = os.path.join(SAVE_DIR, f'litter_event_{timestamp}.avi')
            out = cv2.VideoWriter(save_path, fourcc, fps, (frame_width, frame_height))
            is_recording = True
            print("\n🎥 CAT DETECTED! Recording started...")

    else:
        # If there is no cat, but we ARE recording...
        if is_recording:
            frames_missing += 1  # Start counting how long the cat has been gone

            # If the cat has been gone for more than 1.5 seconds, STOP RECORDING!
            if frames_missing > COOLDOWN_FRAMES:
                out.release()
                is_recording = False
                print(f"🛑 Cat left. Video saved to: {os.path.abspath(save_path)}")

                # Trigger the 10-second auto-delete for this specific video
                schedule_deletion(save_path, 120)
                print("Waiting for the next cat...")

    # If the camera is currently in "Recording Mode", write the frame to the file
    if is_recording:
        out.write(annotated_frame)

    # Show the live feed
    cv2.imshow('LitterSense Smart DVR', annotated_frame)

    # Press 'q' to quit the whole program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- FINAL CLEANUP ---
if is_recording:
    out.release()
cap.release()
cv2.destroyAllWindows()
print("\nSystem shut down.")