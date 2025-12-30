[Youtube Link](https://youtu.be/08xs3S92wzY)


# Embedded Final Project (LVGL + YOLO + Gemini + Voice)

This project is a **Linux desktop simulator** built with **LVGL + SDL2** that reads frames from a **virtual webcam** (`/dev/video0`), streams them to a **Python ML server**, and displays:

- a live annotated video preview (YOLO mode)
- object detection text
- an optional **AI “Explain”** description (Gemini)
- an optional **voice response** (MP3 generated on the server and played on the client)

The networking is a custom TCP protocol designed to safely transfer a full frame plus fixed-size text fields and optional audio.

## Features

- **LVGL UI (SDL2)**
	- Live video panel
	- Text labels for YOLO result and AI explanation
	- Buttons for modes (YOLO / Explain / Snapshot / Emergency)

- **Python ML server** (`src/local_python/ml_server.py`)
	- YOLO inference via `ultralytics`
	- AI scene explanation via Google Gemini (optional; requires `GEMINI_API_KEY`)
	- Text-to-speech via `gTTS` producing an MP3 returned to the client

- **Reliable TCP receive/send**
	- Uses `recv_exact()` / `send_exact()` to handle TCP fragmentation
	- Handles `EINTR` safely (important fix: prevents random disconnects during long transfers)

## System architecture

### Data flow

1. A video is looped into a virtual camera (`/dev/video0`) using `v4l2loopback` + `ffmpeg`.
2. The LVGL app (`bin/main`) reads frames from `/dev/video0`.
3. The app sends button state + raw RGB frame to the Python server.
4. The server returns:
	 - annotated frame (or original)
	 - YOLO text (fixed 64 bytes)
	 - Explain text (fixed 512 bytes)
	 - audio length + optional MP3 bytes
5. The app displays the results and (if present) plays `src/assets/audio/ai_response.mp3` via `mpg123`.

### TCP protocol (client ↔ server)

All integers are little-endian on typical x86 Linux; both sides currently treat the 4-byte audio length as a native `uint32_t`.

Client → Server:

- Button state: **4 bytes** (4× `uint8_t`)
- Frame: **IMG_W × IMG_H × 3 bytes** (BGR)

Server → Client:

- Frame: **IMG_W × IMG_H × 3 bytes**
- YOLO text: **64 bytes** (UTF-8, NUL padded/truncated)
- Explain text: **512 bytes** (UTF-8, NUL padded/truncated)
- Audio size: **4 bytes** (`uint32_t`)
- Audio bytes: **N bytes** (MP3)

> Important: the fixed 64/512 byte padding prevents “text overflow” from corrupting the audio length field.

## Prerequisites (Linux)

### Native packages

- A C toolchain + CMake
- SDL2 development headers
- `v4l2loopback` (virtual webcam)
- `ffmpeg` (feed a video into the virtual webcam)
- `mpg123` (play MP3 audio for AI Explain)

### Python packages

The server uses:

- `opencv-python`
- `numpy`
- `ultralytics`
- `google-genai`
- `Pillow`
- `gTTS`

(Install into your preferred environment.)

## Run it

### 1) Create a virtual webcam

```bash
sudo modprobe v4l2loopback exclusive_caps=1 card_label="VirtualCam"
```

### 2) Stream a video into `/dev/video0`

Examples are also listed in `src/README.txt`.

```bash
ffmpeg -re -stream_loop -1 -i /path/to/video.mp4 \
	-f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0
```

### 3) Start the Python ML server

If you want AI Explain, export your Gemini API key into .env in the same folder with ml_server.py

Then run:

```PowerShell
python ml_server.py
```

### 4) Build and run the LVGL app

This repo uses Makefile.

```bash
cd build
make -j
cd ..&&./build/bin/main
```

## UI behavior notes

- **YOLO mode**: annotated frame + short detection text.
- **Explain mode**: requests a Gemini explanation on the rising edge of the Explain button and caches it while the button remains pressed.
- **Voice**: when Explain is generated, the server sends an MP3. The client writes it to `src/assets/audio/ai_response.mp3` and plays it.
- **Snapshot**: freezes/ignores live frame updates while snapshot is active (labels can still update).

## Troubleshooting

### Connection drops after clicking Explain

This was caused by `recv()` being interrupted (`errno == EINTR`) mid-transfer and the client treating it as a hard failure.
The client now retries on `EINTR` in `recv_exact()` / `send_exact()`.

### No AI explanation shown

- Ensure `GEMINI_API_KEY` is set.
- Check the Python server terminal for Gemini errors.

### No audio playback

- Ensure `mpg123` is installed.
- Confirm `src/assets/audio/ai_response.mp3` is being written.

### Virtual webcam shows no frames

- Confirm `v4l2loopback` created `/dev/video0`.
- Confirm `ffmpeg` is streaming and not erroring.
- Confirm permissions on `/dev/video0`.

### Server exits when client disconnects

The current server exits its main loop on disconnect. If you want it to accept a new connection without restarting, we can adjust `ml_server.py` to wrap `accept()` / client handling in a reconnect loop.

## Key files

- `src/finalfunction.c` — LVGL app + camera capture + TCP client + audio playback
- `src/local_python/ml_server.py` — Python ML server (YOLO + Gemini + gTTS)
- `src/README.txt` — quick `ffmpeg` + `v4l2loopback` notes
