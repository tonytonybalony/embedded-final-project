import socket
import struct
import cv2
import numpy as np
from ultralytics import YOLO

# Configuration
HOST = '0.0.0.0'
PORT = 9999
IMG_W = 852
IMG_H = 480

def start_server():
    # 1. Load YOLO Model
    print("Loading YOLO model...")
    model = YOLO('yolo11n.pt')
    print("Model loaded. Waiting for connection...")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    try:
        while True:
            # 2. Receive Button Status (4 Bytes)
            status_data = conn.recv(4)
            if not status_data: break

            # Unpack 4 bytes: [ON/OFF, AI Explain, Snapshot, Emergency]
            btn_states = struct.unpack('4B', status_data)
            is_on = btn_states[0]

            # Debug: Print all button states
            # print(f"Btns: ON={btn_states[0]}, Explain={btn_states[1]}, Snap={btn_states[2]}, Emerg={btn_states[3]}")

            # 3. Receive Raw Image Bytes from C
            size = IMG_W * IMG_H * 3
            frame_data = b''
            while len(frame_data) < size:
                packet = conn.recv(size - len(frame_data))
                if not packet: break
                frame_data += packet

            if len(frame_data) < size: break

            # 4. Convert Bytes to Numpy Array (BGR format)
            # C sends BGR, and OpenCV uses BGR, so no conversion needed.
            frame_np = np.frombuffer(frame_data, dtype=np.uint8)
            frame_np = frame_np.reshape((IMG_H, IMG_W, 3))

            # 5. Run YOLO Inference (Only if ON)
            annotated_frame = frame_np
            result_str = ""

            if is_on == 1:
                # verbose=False keeps the terminal clean
                results = model(frame_np, conf=0.5, iou=0.25, verbose=False)

                # plot() returns the image as a BGR numpy array
                annotated_frame = results[0].plot()

                # Extract Text Result (Highest Confidence Object)
                if len(results[0].boxes) > 0:
                    # Get the first detection (usually highest confidence)
                    box = results[0].boxes[0]
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    name = results[0].names[cls_id]
                    result_str = f"{name}: {int(conf * 100)}%"

            print(f"Result: {result_str} | Buttons: [ON:{btn_states[0]}, EXP:{btn_states[1]}, SNP:{btn_states[2]}, EMG:{btn_states[3]}]")

            # 6. Send Processed Image BACK to C
            # Ensure the array is contiguous and bytes
            conn.sendall(annotated_frame.tobytes())

            # 7. Send Text Result (Fixed 64 bytes)
            conn.sendall(result_str.ljust(64).encode('utf-8'))

    except Exception as e:
        print(f"Error: {e}")
    finally:
        conn.close()
        server_socket.close()

if __name__ == "__main__":
    start_server()
