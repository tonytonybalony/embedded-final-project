import socket
import struct
import time
import random

# Configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 9999
IMG_W = 320
IMG_H = 240


def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"ML Server listening on {HOST}:{PORT}...")

    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    try:
        while True:
            # 1. Define Frame Size (320 * 240 * 3 bytes = 230400 bytes)
            # We removed the dynamic header for simplicity/speed
            size = IMG_W * IMG_H * 3

            # 2. Receive Frame Data
            frame_data = b''
            while len(frame_data) < size:
                packet = conn.recv(size - len(frame_data))
                if not packet:
                    break
                frame_data += packet

            if len(frame_data) < size:
                print("Incomplete frame received, closing connection.")
                break

            # --- ML PROCESSING HERE ---
            # For now, we simulate a result.
            # In real life, decode 'frame_data' with opencv: img = cv2.imdecode(...)

            classes = ["Person", "Cat", "Dog", "Laptop", "Phone"]
            detected = random.choice(classes)
            confidence = random.randint(70, 99)
            result_str = f"{detected}: {confidence}%"
            print(f"Received {size} bytes. Result: {result_str}")

            # 3. Send Result Back (Fixed 64 bytes for simplicity in C)
            response = result_str.ljust(64).encode('utf-8')
            conn.sendall(response)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        conn.close()
        server_socket.close()


if __name__ == "__main__":
    start_server()
