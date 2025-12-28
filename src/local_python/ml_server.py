import socket
import struct
import cv2
import numpy as np
import os
import datetime
import io
from ultralytics import YOLO
from google import genai
from google.genai import types
from PIL import Image
from gtts import gTTS

# Configuration
HOST = '0.0.0.0'
PORT = 9999
IMG_W = 852
IMG_H = 480
GEMINI_API_KEY_ENV = "GEMINI_API_KEY"


def _load_dotenv(dotenv_path: str) -> None:
    """Minimal .env loader.

    Python does not automatically read .env files. This loads KEY=VALUE pairs
    into process environment (without overwriting already-set variables).
    """
    try:
        with open(dotenv_path, "r", encoding="utf-8") as file:
            for raw_line in file:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip().strip('"').strip("'")
                if not key:
                    continue
                os.environ.setdefault(key, value)
    except FileNotFoundError:
        return


def _try_load_project_dotenv() -> None:
    # Try project-root (same folder as this file) first, then current working dir.
    here = os.path.dirname(os.path.abspath(__file__))
    _load_dotenv(os.path.join(here, ".env"))
    _load_dotenv(os.path.join(os.getcwd(), ".env"))


def _print_gemini_config_status() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    dotenv_here = os.path.join(here, ".env")
    dotenv_cwd = os.path.join(os.getcwd(), ".env")
    dotenv_found = os.path.exists(dotenv_here) or os.path.exists(dotenv_cwd)
    key_present = bool(_get_gemini_api_key())
    print(f".env found: {dotenv_found} | {GEMINI_API_KEY_ENV} present: {key_present}")


def _recv_exact(conn: socket.socket, size: int) -> bytes | None:
    data = b""
    while len(data) < size:
        try:
            chunk = conn.recv(size - len(data))
        except socket.timeout:
            continue
        if not chunk:
            return None
        data += chunk
    return data


def _get_gemini_api_key() -> str:
    return os.getenv(GEMINI_API_KEY_ENV, "").strip()

def start_server():
    _try_load_project_dotenv()
    # 1. Load YOLO Model
    print("Loading YOLO model...")
    model = YOLO('yolo11n.pt')

    _print_gemini_config_status()

    # Configure Gemini
    gemini_api_key = _get_gemini_api_key()
    if gemini_api_key:
        client = genai.Client(api_key=gemini_api_key)
        print("Gemini configured.")
    else:
        client = None
        print(f"Gemini API Key missing. Set {GEMINI_API_KEY_ENV} to enable AI Explain.")

    print("Model loaded. Waiting for connection...")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)

    # Make Ctrl+C responsive even during blocking socket calls on Windows.
    server_socket.settimeout(1.0)

    try:
        while True:
            try:
                conn, addr = server_socket.accept()
                break
            except socket.timeout:
                continue
        print(f"Connected by {addr}")
        conn.settimeout(1.0)
    except KeyboardInterrupt:
        print("Shutting down (Ctrl+C).")
        server_socket.close()
        return

    last_explain_state = 0
    cached_explanation = "Waiting for Explanation..."

    try:
        while True:
            # 2. Receive Button Status (4 Bytes)
            try:
                status_data = conn.recv(4)
            except socket.timeout:
                continue
            if not status_data:
                break

            # Unpack 4 bytes: [ON/OFF, AI Explain, Snapshot, Emergency]
            btn_states = struct.unpack('4B', status_data)
            is_on = btn_states[0]
            is_explain = btn_states[1]

            # 3. Receive Raw Image Bytes from C
            size = IMG_W * IMG_H * 3
            frame_data = _recv_exact(conn, size)
            if frame_data is None:
                break

            # 4. Convert Bytes to Numpy Array (BGR format)
            # C sends BGR, and OpenCV uses BGR, so no conversion needed.
            frame_np = np.frombuffer(frame_data, dtype=np.uint8)
            frame_np = frame_np.reshape((IMG_H, IMG_W, 3))

            # 5. Logic
            annotated_frame = frame_np
            yolo_str = ""
            explain_str = ""
            audio_bytes = b""

            # AI Explain Logic (Button 2)
            if is_explain == 1:
                if last_explain_state == 0:
                    # Trigger new explanation
                    if client:
                        try:
                            print("Requesting Gemini explanation...")
                            # Convert current frame (BGR) -> JPEG bytes for Gemini
                            img_rgb = cv2.cvtColor(frame_np, cv2.COLOR_BGR2RGB)
                            img_pil = Image.fromarray(img_rgb)
                            buf = io.BytesIO()
                            img_pil.save(buf, format="JPEG", quality=85)
                            image_bytes = buf.getvalue()

                            response = client.models.generate_content(
                                model="gemini-2.5-flash-lite",
                                contents=[
                                    types.Part.from_bytes(
                                        data=image_bytes,
                                        mime_type="image/jpeg",
                                    ),
                                    "You are a navigation assistant for a visually impaired user. "
                                    "Describe only what you can see in this photo. In 1 to 2 short sentences, "
                                    "state: (1) the main scene ahead, "
                                    "(2) any immediate hazards (obstacles, drop-offs/stairs, vehicles/bikes), "
                                    "and (3) whether it appears safe to continue straight. If you are not sure, say 'uncertain' and do not guess.(not necessary)",
                                ],
                            )

                            response_text = getattr(response, "text", None)
                            cached_explanation = (response_text or "").strip() or "(No text returned)"
                            print(f"Gemini: {cached_explanation}")

                            # Generate Audio
                            if cached_explanation and "Error" not in cached_explanation:
                                try:
                                    print("Generating audio...")
                                    tts = gTTS(cached_explanation, lang='en')
                                    # Write to memory buffer
                                    fp = io.BytesIO()
                                    tts.write_to_fp(fp)
                                    fp.seek(0)
                                    audio_bytes = fp.read()
                                    print(f"Audio generated: {len(audio_bytes)} bytes")
                                except Exception as e:
                                    print(f"TTS Error: {e}")
                        except Exception as e:
                            cached_explanation = f"Gemini Error: {str(e)}"
                            print(cached_explanation)
                    else:
                        cached_explanation = "No API Key Configured"

                explain_str = cached_explanation

            # YOLO Logic (Button 1) - Only if Explain is OFF
            elif is_on == 1:
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
                    yolo_str = f"{name}: {int(conf * 100)}%"

            last_explain_state = is_explain

            print(f"YOLO: {yolo_str} | Explain: {explain_str[:20]}... | Buttons: {btn_states}")

            # 6. Send Processed Image BACK to C
            # Ensure the array is contiguous and bytes
            conn.sendall(annotated_frame.tobytes())

            # 7. Send Text Results
            # Send YOLO (64 bytes)
            conn.sendall(yolo_str[:64].ljust(64, '\0').encode('utf-8'))
            # Send Explain (512 bytes)
            conn.sendall(explain_str[:512].ljust(512, '\0').encode('utf-8'))

            # 8. Send Audio Data
            # Send size (4 bytes)
            conn.sendall(struct.pack('I', len(audio_bytes)))
            # Send data
            if len(audio_bytes) > 0:
                conn.sendall(audio_bytes)

    except KeyboardInterrupt:
        print("Shutting down (Ctrl+C).")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        conn.close()
        server_socket.close()

if __name__ == "__main__":
    start_server()
