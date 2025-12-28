#include "../lvgl/examples/lv_examples.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <sys/stat.h>
#include <time.h>

// --- CONFIGURATION ---
#define SERVER_IP   "192.168.175.1"
#define SERVER_PORT 9999
#define VIDEO_DEV   "/dev/video0"  // <--- Make sure this is your loopback device
// loaded video size
#define IMG_W       852
#define IMG_H       480

// --- GLOBALS ---
static lv_obj_t * img_display;
static lv_obj_t * label_result;
static lv_obj_t * label_explain; // New label for AI Explain
static lv_img_dsc_t img_dsc;

// Button Globals
static lv_obj_t * ui_btns[5];
static bool btn_states[5] = {false, false, false, false, false};
static lv_timer_t * emergency_timer = NULL;
static bool emergency_flash_state = false;

// Buffers
static uint8_t * img_buf_rgb;    // DISPLAY buffer (Read by LVGL)
static uint8_t * capture_buf;    // CAMERA buffer (Written by Camera)
static uint8_t * shared_net_buf; // NETWORK buffer (Read by Network)
static uint8_t * sending_buf;    // SENDING/RECV buffer (Used by Socket)

// Synchronization
static pthread_mutex_t lock;     // Protects UI and Display buffers
static pthread_mutex_t net_lock; // Protects Network buffer
static volatile bool has_new_net_frame = false;
static volatile bool frame_ready = false;

static int sock_fd = -1;
static volatile bool running = true;
static char shared_yolo_result[64];
static char shared_explain_result[512];

// V4L2
struct buffer { void *start; size_t length; };
static struct buffer *buffers;
static unsigned int n_buffers;
static int fd_cam = -1;


// --- HELPERS ---
static void yuyv_to_rgb(const uint8_t *src, uint8_t *dst, int width, int height) {
    int i, j;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j += 2) {
            int y0 = src[(i * width + j) * 2];
            int u  = src[(i * width + j) * 2 + 1] - 128;
            int y1 = src[(i * width + j) * 2 + 2];
            int v  = src[(i * width + j) * 2 + 3] - 128;

            int r = y0 + (1.370705 * v);
            int g = y0 - (0.698001 * v) - (0.337633 * u);
            int b = y0 + (1.732446 * u);

            if (r < 0) r = 0; if (r > 255) r = 255;
            if (g < 0) g = 0; if (g > 255) g = 255;
            if (b < 0) b = 0; if (b > 255) b = 255;

            int idx = (i * width + j) * 3;
            dst[idx] = b; dst[idx + 1] = g; dst[idx + 2] = r;

            r = y1 + (1.370705 * v);
            g = y1 - (0.698001 * v) - (0.337633 * u);
            b = y1 + (1.732446 * u);

            if (r < 0) r = 0; if (r > 255) r = 255;
            if (g < 0) g = 0; if (g > 255) g = 255;
            if (b < 0) b = 0; if (b > 255) b = 255;

            idx = (i * width + j + 1) * 3;
            dst[idx] = b; dst[idx + 1] = g; dst[idx + 2] = r;
        }
    }
}

static int xioctl(int fh, int request, void *arg) {
    int r;
    do { r = ioctl(fh, request, arg); } while (-1 == r && EINTR == errno);
    return r;
}

static void save_snapshot_bmp(const uint8_t *buffer, int width, int height) {
    // Create directory if not exists
    struct stat st = {0};
    if (stat("snapshots", &st) == -1) {
        mkdir("snapshots", 0700);
    }

    // Generate filename with timestamp
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[64];
    snprintf(filename, sizeof(filename), "snapshots/snap_%04d%02d%02d_%02d%02d%02d.bmp",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

    FILE *f = fopen(filename, "wb");
    if (!f) {
        LV_LOG_ERROR("Failed to open file for snapshot");
        return;
    }

    // BMP Header
    uint32_t headers_size = 14 + 40;
    uint32_t pixel_data_size = width * height * 3;
    uint32_t filesize = headers_size + pixel_data_size;

    uint8_t bmpfileheader[14] = {
        'B','M',
        filesize & 0xFF, (filesize >> 8) & 0xFF, (filesize >> 16) & 0xFF, (filesize >> 24) & 0xFF,
        0,0, 0,0,
        headers_size & 0xFF, (headers_size >> 8) & 0xFF, (headers_size >> 16) & 0xFF, (headers_size >> 24) & 0xFF
    };

    uint8_t bmpinfoheader[40] = {
        40,0,0,0,
        width & 0xFF, (width >> 8) & 0xFF, (width >> 16) & 0xFF, (width >> 24) & 0xFF,
        height & 0xFF, (height >> 8) & 0xFF, (height >> 16) & 0xFF, (height >> 24) & 0xFF,
        1,0,
        24,0,
        0,0,0,0,
        pixel_data_size & 0xFF, (pixel_data_size >> 8) & 0xFF, (pixel_data_size >> 16) & 0xFF, (pixel_data_size >> 24) & 0xFF,
        0,0,0,0,
        0,0,0,0,
        0,0,0,0,
        0,0,0,0
    };

    fwrite(bmpfileheader, 1, 14, f);
    fwrite(bmpinfoheader, 1, 40, f);

    // BMP stores pixels bottom-to-top, so we write rows in reverse order
    // Also, BMP expects BGR. Our buffer is BGR (from Python/OpenCV), so we can write directly.
    // However, we need to flip vertically.
    uint8_t *row = malloc(width * 3);
    for (int y = height - 1; y >= 0; y--) {
        memcpy(row, &buffer[y * width * 3], width * 3);
        fwrite(row, 1, width * 3, f);
    }
    free(row);
    fclose(f);
    printf("Snapshot saved to %s\n", filename);
}

// --- NETWORK THREAD ---
static void *net_thread_entry(void *arg) {
    (void)arg;
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr);

    while (running) {
        if (sock_fd < 0) {
            sock_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                close(sock_fd); sock_fd = -1;
                usleep(1000000); continue;
            }
        }

        // 1. Get Frame from Camera Thread
        bool work_to_do = false;
        pthread_mutex_lock(&net_lock);
        if (has_new_net_frame) {
            memcpy(sending_buf, shared_net_buf, IMG_W * IMG_H * 3);
            has_new_net_frame = false;
            work_to_do = true;
        }
        pthread_mutex_unlock(&net_lock);

        if (!work_to_do) { usleep(5000); continue; }

        // 2. Send Frame to Python
        // Send Button Status (4 Bytes)
        uint8_t status[4];
        for(int i=0; i<4; i++) status[i] = btn_states[i] ? 1 : 0;

        if (send(sock_fd, status, 4, 0) < 0) {
            close(sock_fd); sock_fd = -1; continue;
        }

        if (send(sock_fd, sending_buf, IMG_W * IMG_H * 3, 0) < 0) {
            close(sock_fd); sock_fd = -1; continue;
        }

        // 3. Receive PROCESSED Frame from Python (Reuse sending_buf)
        int total_read = 0;
        int size = IMG_W * IMG_H * 3;
        while (total_read < size) {
            int n = recv(sock_fd, sending_buf + total_read, size - total_read, 0);
            if (n <= 0) {
                close(sock_fd); sock_fd = -1; total_read = -1; break;
            }
            total_read += n;
        }
        if (total_read < 0) continue;

        // 4. Receive Text Results (YOLO + Explain)
        char yolo_res[64] = {0};
        char explain_res[512] = {0};

        // Recv YOLO (64 bytes)
        if (recv(sock_fd, yolo_res, 64, 0) <= 0) {
            close(sock_fd); sock_fd = -1; continue;
        }
        // Recv Explain (512 bytes)
        if (recv(sock_fd, explain_res, 512, 0) <= 0) {
            close(sock_fd); sock_fd = -1; continue;
        }

        // 5. Update UI Buffers
        pthread_mutex_lock(&lock);

        // Always update text buffers
        strncpy(shared_yolo_result, yolo_res, 63);
        strncpy(shared_explain_result, explain_res, 511);

        // Only update image if Snapshot (Button 3) AND AI Explain (Button 2) are NOT active
        if (!btn_states[2] && !btn_states[1]) {
            memcpy(img_buf_rgb, sending_buf, IMG_W * IMG_H * 3);
            frame_ready = true;
        }
        // If AI Explain IS active, we still signal frame_ready to update labels, but don't update image
        else if (btn_states[1]) {
             frame_ready = true;
        }
        pthread_mutex_unlock(&lock);
    }
    if (sock_fd >= 0) close(sock_fd);
    return NULL;
}

// --- CAMERA THREAD ---
static void *cam_thread_entry(void *arg) {
    (void)arg;
    fd_cam = open(VIDEO_DEV, O_RDWR | O_NONBLOCK, 0);
    if (fd_cam == -1) { LV_LOG_ERROR("Cannot open camera"); return NULL; }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMG_W; fmt.fmt.pix.height = IMG_H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    xioctl(fd_cam, VIDIOC_S_FMT, &fmt);

    struct v4l2_requestbuffers req = {0};
    req.count = 2; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    xioctl(fd_cam, VIDIOC_REQBUFS, &req);

    buffers = calloc(req.count, sizeof(*buffers));
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        xioctl(fd_cam, VIDIOC_QUERYBUF, &buf);
        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_cam, buf.m.offset);
    }

    for (int i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        xioctl(fd_cam, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_cam, VIDIOC_STREAMON, &type);

    while (running) {
        fd_set fds; FD_ZERO(&fds); FD_SET(fd_cam, &fds);
        struct timeval tv = {2, 0};
        if (select(fd_cam + 1, &fds, NULL, NULL, &tv) <= 0) continue;

        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_cam, VIDIOC_DQBUF, &buf) == -1) continue;

        // 1. Convert to CAPTURE buffer
        yuyv_to_rgb((uint8_t*)buffers[buf.index].start, capture_buf, IMG_W, IMG_H);

        // 2. Copy to Network Buffer
        pthread_mutex_lock(&net_lock);
        memcpy(shared_net_buf, capture_buf, IMG_W * IMG_H * 3);
        has_new_net_frame = true;
        pthread_mutex_unlock(&net_lock);

        // NOTE: We do NOT signal 'frame_ready' here anymore.
        // The Network thread will do it after receiving the processed frame.

        xioctl(fd_cam, VIDIOC_QBUF, &buf);
    }
    close(fd_cam);
    return NULL;
}

// --- AUDIO HELPERS ---
#define ALARM_FILE "/home/tonytony/lv_port_pc_vscode/src/assets/audio/alarm.wav"
#define AI_VOICE_FILE "/home/tonytony/lv_port_pc_vscode/src/assets/audio/ai_response.mp3"

// UPDATED: Function to play the pre-generated MP3
static void speak_text(const char *text) {
    // 1. Check if the MP3 file exists (Generated by Python)
    if (access(AI_VOICE_FILE, F_OK) == 0) {
        // Stop any currently playing audio
        system("pkill mpg123");

        // Play MP3 (Redirect output to /dev/null to avoid "No such device" errors)
        char cmd[512];
        snprintf(cmd, sizeof(cmd), "mpg123 -q %s > /dev/null 2>&1 &", AI_VOICE_FILE);
        system(cmd);
    }
    // 2. Fallback to espeak if MP3 is missing (e.g. gTTS failed or Python server didn't update)
    else {
        if (text == NULL || strlen(text) == 0) return;
        char cmd[1024];
        snprintf(cmd, sizeof(cmd), "espeak -v en+m3 -s 150 -a 200 '%s' > /dev/null 2>&1 &", text);
        system(cmd);
    }
}

static void start_alarm(void) {
    char cmd[512];
    // Run a loop in background, save PID to a file
    snprintf(cmd, sizeof(cmd),
        "sh -c 'while true; do aplay -q %s; sleep 0.1; done' > /dev/null 2>&1 & echo $! > /tmp/lvgl_alarm.pid",
        ALARM_FILE);
    system(cmd);
}

static void stop_alarm(void) {
    // Kill the shell script (parent) and aplay (child)
    system("if [ -f /tmp/lvgl_alarm.pid ]; then "
           "PID=$(cat /tmp/lvgl_alarm.pid); "
           "pkill -P $PID; " // Kill children of the shell loop
           "kill $PID; "     // Kill the shell loop itself
           "rm /tmp/lvgl_alarm.pid; "
           "fi");
    // Ensure silence
    system("pkill -f 'aplay -q " ALARM_FILE "'");
}

// --- UI UPDATE TIMER ---
static char last_spoken_text[512]; // Add a static buffer to track the last spoken text

static void update_timer_cb(lv_timer_t * t) {
    (void)t;

    pthread_mutex_lock(&lock);

    // Update YOLO Label
    if (shared_yolo_result[0] != '\0') {
        lv_label_set_text(label_result, shared_yolo_result);
    }

    // Update Explain Label & Speak
    if (shared_explain_result[0] != '\0') {
        lv_label_set_text(label_explain, shared_explain_result);

        // NEW: Check if text changed, if so, speak it
        if (strcmp(shared_explain_result, last_spoken_text) != 0) {
            strncpy(last_spoken_text, shared_explain_result, 511);
            speak_text(shared_explain_result);

        }
    }
    // Clear last spoken if text is empty (e.g. Return button pressed)
    else {
        last_spoken_text[0] = '\0';
    }

    bool do_invalidate = false;
    if (frame_ready) {
        frame_ready = false;
        do_invalidate = true;
    }
    pthread_mutex_unlock(&lock);

    if (do_invalidate) lv_obj_invalidate(img_display);
}

static void emergency_timer_cb(lv_timer_t * t) {
    (void)t;
    emergency_flash_state = !emergency_flash_state;
    if (emergency_flash_state) {
        lv_obj_set_style_bg_color(ui_btns[3], lv_color_hex(0x8B2020), 0); // Red
        lv_obj_set_style_bg_grad_color(ui_btns[3], lv_color_hex(0x8B2020), 0);
    } else {
        lv_obj_set_style_bg_color(ui_btns[3], lv_color_hex(0x000000), 0); // Black
        lv_obj_set_style_bg_grad_color(ui_btns[3], lv_color_hex(0x000000), 0);
    }
}

static void btn_event_cb(lv_event_t * e) {
    lv_obj_t * btn = lv_event_get_target(e);
    lv_obj_t * label = lv_obj_get_child(btn, 0);

    int btn_idx = -1;
    for(int i=0; i<4; i++) {
        if(ui_btns[i] == btn) {
            btn_idx = i;
            break;
        }
    }    if(btn_idx == 0) { // Button 1: ON/OFF
        btn_states[0] = !btn_states[0];
        lv_label_set_text(label, btn_states[0] ? "OFF" : "ON");
        // TODO: Send command to Python to toggle YOLO
    }
    else if(btn_idx == 1) { // Button 2: AI Explain/Return
        btn_states[1] = !btn_states[1];
        lv_label_set_text(label, btn_states[1] ? "Return" : "AI Explain");

        if(btn_states[1]) { // AI Explain Active -> Disable 1 & 3
            lv_obj_add_state(ui_btns[0], LV_STATE_DISABLED);
            lv_obj_add_state(ui_btns[2], LV_STATE_DISABLED);
        } else { // Return -> Enable 1 & 3
            lv_obj_clear_state(ui_btns[0], LV_STATE_DISABLED);
            lv_obj_clear_state(ui_btns[2], LV_STATE_DISABLED);
            lv_label_set_text(label_explain, ""); // Clear explanation
        }
    }
    else if(btn_idx == 2) { // Button 3: Snapshot/Return
        btn_states[2] = !btn_states[2];
        lv_label_set_text(label, btn_states[2] ? "Return" : "Snapshot");

        if(btn_states[2]) { // Snapshot Active -> Disable 1 & 2
            lv_obj_add_state(ui_btns[0], LV_STATE_DISABLED);
            lv_obj_add_state(ui_btns[1], LV_STATE_DISABLED);

            // Save Snapshot immediately
            pthread_mutex_lock(&lock);
            save_snapshot_bmp(img_buf_rgb, IMG_W, IMG_H);
            pthread_mutex_unlock(&lock);

        } else { // Return -> Enable 1 & 2
            lv_obj_clear_state(ui_btns[0], LV_STATE_DISABLED);
            lv_obj_clear_state(ui_btns[1], LV_STATE_DISABLED);
        }
    }
    else if(btn_idx == 3) { // Button 4: Emergency
        btn_states[3] = !btn_states[3];
        if(btn_states[3]) {
            // Start Flashing
            if(emergency_timer == NULL) {
                emergency_timer = lv_timer_create(emergency_timer_cb, 200, NULL);
            }
            lv_timer_resume(emergency_timer);
            start_alarm();
        } else {
            // Stop Flashing
            if(emergency_timer) {
                lv_timer_pause(emergency_timer);
            }
            stop_alarm();
            // Reset to Dark
            lv_obj_set_style_bg_color(ui_btns[3], lv_color_hex(0x1A1A1A), 0);
            lv_obj_set_style_bg_grad_color(ui_btns[3], lv_color_hex(0x000000), 0);
        }
    }
}

// --- INIT ---
void final_project_init(void) {
    pthread_mutex_init(&lock, NULL);
    pthread_mutex_init(&net_lock, NULL);

    // 1. Setup Screen Styling
    lv_obj_t * scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x202020), 0); // Dark Grey Background

    // 2. Header Label
    lv_obj_t * header = lv_label_create(scr);
    lv_label_set_text(header, "Smart Surveillance System");
    lv_obj_set_style_text_font(header, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(header, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 10);

    // 3. Video Display Container
    img_buf_rgb = malloc(IMG_W * IMG_H * 3);
    capture_buf = malloc(IMG_W * IMG_H * 3);
    shared_net_buf = malloc(IMG_W * IMG_H * 3);
    sending_buf = malloc(IMG_W * IMG_H * 3);
    memset(img_buf_rgb, 0, IMG_W * IMG_H * 3);

    img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    img_dsc.header.w = IMG_W;
    img_dsc.header.h = IMG_H;
    img_dsc.header.stride = IMG_W * 3;
    img_dsc.data_size = IMG_W * IMG_H * 3;
    img_dsc.header.cf = LV_COLOR_FORMAT_RGB888;
    img_dsc.data = img_buf_rgb;

    lv_obj_t * video_cont = lv_obj_create(scr);
    lv_obj_set_size(video_cont, IMG_W + 10, IMG_H + 10);
    lv_obj_align(video_cont, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_set_style_bg_color(video_cont, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(video_cont, 2, 0);
    lv_obj_set_style_border_color(video_cont, lv_color_hex(0x404040), 0);
    lv_obj_clear_flag(video_cont, LV_OBJ_FLAG_SCROLLABLE);

    img_display = lv_img_create(video_cont);
    lv_img_set_src(img_display, &img_dsc);
    lv_obj_center(img_display);

    // 4. Result Label (Only)

    // Label under image, aligned left
    lv_obj_t * res_title = lv_label_create(scr);
    lv_label_set_text(res_title, "AI Detection Result:");
    lv_obj_set_style_text_color(res_title, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(res_title, video_cont, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 40);

    // Text output bar under the label, same length as video
    lv_obj_t * res_cont = lv_obj_create(scr);
    lv_obj_set_size(res_cont, IMG_W + 10, 100); // Increased height for explanation
    // Align to LABEL
    lv_obj_align_to(res_cont, res_title, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);
    lv_obj_set_style_bg_color(res_cont, lv_color_hex(0x303030), 0);
    lv_obj_set_style_border_color(res_cont, lv_color_hex(0x505050), 0);
    lv_obj_clear_flag(res_cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(res_cont, 10, 0); // Increased padding

    label_explain = lv_label_create(res_cont);
    lv_label_set_text(label_explain, "");
    lv_obj_align(label_explain, LV_ALIGN_TOP_LEFT, 0, 0); // Align top-left
    lv_obj_set_style_text_font(label_explain, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_explain, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_long_mode(label_explain, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_explain, IMG_W - 30); // Adjust width for padding


    label_result = lv_label_create(scr);
    lv_label_set_text(label_result, "Waiting for ML Server...");
    lv_obj_align_to(label_result, video_cont, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    lv_obj_set_style_text_font(label_result, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label_result, lv_color_hex(0x00FF00), 0);

    // 5. Buttons (4 Buttons)
    const char * btns[] = {"ON", "AI Explain", "Snapshot", "Emergency"};
    lv_obj_t * btn_cont = lv_obj_create(scr);
    lv_obj_set_size(btn_cont, IMG_W + 10, 60);
    lv_obj_align_to(btn_cont, res_cont, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20); // Align to res_cont instead of label_result
    lv_obj_set_style_bg_opa(btn_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_cont, 0, 0);
    lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_cont, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(btn_cont, LV_OBJ_FLAG_SCROLLABLE);

    for(int i=0; i<4; i++) {
        ui_btns[i] = lv_btn_create(btn_cont);
        lv_obj_set_size(ui_btns[i], (IMG_W / 4)-20, 45);
        lv_obj_add_event_cb(ui_btns[i], btn_event_cb, LV_EVENT_CLICKED, NULL);

        // Modern Button Styling (Dark & Metallic Gold)
        lv_obj_set_style_bg_color(ui_btns[i], lv_color_hex(0x1A1A1A), 0); // Dark Grey
        lv_obj_set_style_bg_grad_color(ui_btns[i], lv_color_hex(0x000000), 0); // Black
        lv_obj_set_style_bg_grad_dir(ui_btns[i], LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_radius(ui_btns[i], 8, 0);
        lv_obj_set_style_shadow_width(ui_btns[i], 15, 0);
        lv_obj_set_style_shadow_color(ui_btns[i], lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(ui_btns[i], LV_OPA_30, 0);
        lv_obj_set_style_shadow_ofs_y(ui_btns[i], 4, 0);
        lv_obj_set_style_border_width(ui_btns[i], 1, 0);
        lv_obj_set_style_border_color(ui_btns[i], lv_color_hex(0xD4AF37), 0); // Metallic Gold Border

        // Pressed State
        lv_obj_set_style_bg_color(ui_btns[i], lv_color_hex(0x000000), LV_STATE_PRESSED);
        lv_obj_set_style_bg_grad_color(ui_btns[i], lv_color_hex(0x1A1A1A), LV_STATE_PRESSED);
        lv_obj_set_style_shadow_ofs_y(ui_btns[i], 2, LV_STATE_PRESSED);
        lv_obj_set_style_translate_y(ui_btns[i], 2, LV_STATE_PRESSED); // Move down when pressed

        lv_obj_t * lbl = lv_label_create(ui_btns[i]);
        lv_label_set_text(lbl, btns[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xD4AF37), 0); // Metallic Gold Text
        lv_obj_center(lbl);
    }

    lv_timer_create(update_timer_cb, 30, NULL);

    pthread_t thread_id, net_thread_id;
    pthread_create(&thread_id, NULL, cam_thread_entry, NULL);
    pthread_detach(thread_id);
    pthread_create(&net_thread_id, NULL, net_thread_entry, NULL);
    pthread_detach(net_thread_id);
}
