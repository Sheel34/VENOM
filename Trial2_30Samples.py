import cv2
import math
import time
import numpy as np
import mediapipe as mp
from datetime import datetime
import socket
import queue
import threading

ESP32_IP = "192.168.4.1"  # Replace with ESP32's IP address
ESP32_PORT = 12345
# ------------------------------> #client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Mediapipe Hands Setup
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.2,
    min_tracking_confidence=0.2
)

# Polynomial Regression Coefficients
x_vals = [300, 245, 200, 170, 145, 130, 112, 103, 93, 87, 80, 75, 70, 67, 62, 59, 57]
y_vals = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100]
coefficients = np.polyfit(x_vals, y_vals, 2)  # y = Ax^2 + Bx + C

# Camera Setup
cap = cv2.VideoCapture(1)  # Use the correct camera index
cap.set(3, 1920)   # Reduced resolution
cap.set(4, 1080)  # Reduced resolution
cap.set(cv2.CAP_PROP_FPS, 60)            # Set to camera's max FPS

# ---------------------------> # client_socket.connect((ESP32_IP, ESP32_PORT))

# Queue for frame buffering
frame_queue = queue.Queue(maxsize=10)

# Utility Function: Map Values to Range
def map_to_range(value, input_min, input_max, output_min, output_max):
    return output_min + (value - input_min) * (output_max - output_min) / (input_max - input_min)

# Utility Function: Calculate Distance
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Frame Capture Thread
def capture_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        # Flip frame (optional, can remove if not needed)
        #frame = cv2.flip(frame, 1)
        try:
            frame_queue.put_nowait((frame, time.perf_counter()))
        except queue.Full:
            pass  # Skip frame if queue is full

# Frame Processing Thread
def process_frames():
    reading_count = 0
    start_time = time.perf_counter()
    fps_update_interval = 1.0  # Update FPS every 1 second

    # Set up the window
    cv2.namedWindow('Hand Detection', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Hand Detection', 640, 480)

    while True:
        try:
            # Get frame from queue (non-blocking)
            frame, frame_time = frame_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        # Convert to RGB for Mediapipe
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)
        h, w, _ = frame.shape

        if results.multi_hand_landmarks:
            reading_count += 1  # Increment reading count when hand is detected
            for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
                try:
                    x_list, y_list = [], []

                    # Collect landmarks
                    for lm in hand_landmarks.landmark:
                        px, py = int(lm.x * w), int(lm.y * h)
                        x_list.append(px)
                        y_list.append(py)

                    # Wrist Mapping
                    wrist = hand_landmarks.landmark[9]
                    mapped_x = round (map_to_range((wrist.x * w), 0, w, -35, 35), 1)
                    mapped_y = round (map_to_range((wrist.y * h), 0, h, 31, -2), 1)

                    # Clamp values
                    mapped_x = round (min(24, max(-24, mapped_x)), 1)
                    mapped_y = round (max(0, min(24, mapped_y)), 1)

                    # Distance Calculation
                    dis = calculate_distance(x_list[5], y_list[5], x_list[17], y_list[17])
                    A, B, C = coefficients
                    distance_cm = round (abs(A * dis ** 2 + B * dis + C) - 40, 1)
                    distance_cm = round (max(-12, min(24, distance_cm)), 1)

                    # Output data
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    data = f"{mapped_x:.1f} {mapped_y:.1f} {distance_cm:.1f}\n"
                    # -----------------------> # client_socket.sendall(data.encode())
                    print(f"{timestamp} | {hand_info.classification[0].label} : {data}")

                except Exception as e:
                    print(f"Error in processing landmarks: {e}")
                    continue

        # Calculate and display performance metrics
        current_time = time.perf_counter()
        elapsed_time = current_time - start_time

        if elapsed_time >= fps_update_interval:
            readings_per_sec = reading_count / elapsed_time
            print(f"Readings per second: {readings_per_sec:.2f}")
            reading_count = 0
            start_time = current_time

        # Display the frame
        cv2.imshow('Hand Detection', frame)

        # Check for 'q' key to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()

# Main Function
if __name__ == "__main__":
    try:
        # Start capture thread
        capture_thread = threading.Thread(target=capture_frames, daemon=True)
        capture_thread.start()

        # Start processing thread
        process_thread = threading.Thread(target=process_frames, daemon=True)
        process_thread.start()

        # Keep main thread alive to handle keyboard interrupts
        while True:
            time.sleep(1)  # Reduce CPU usage in main thread
    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()