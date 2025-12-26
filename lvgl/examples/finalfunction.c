#include "lv_examples.h"
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

// --- CONFIGURATION ---
#define SERVER_IP   "192.168.175.1"  // Host IP (VMware NAT usually ends in .1)
#define SERVER_PORT 9999
#define VIDEO_DEV   "/dev/video0"
#define IMG_W       320
#define IMG_H       240

// --- GLOBALS ---
static lv_obj_t * img_display;
static lv_obj_t * label_result;
static lv_img_dsc_t img_dsc;
static uint8_t * img_buf_rgb; // Buffer for LVGL (RGB888)

// --- NEW: Network Thread Globals ---
static uint8_t * shared_net_buf; // Buffer shared between cam and net threads
static uint8_t * sending_buf;    // Buffer used by net thread for sending
static pthread_mutex_t net_lock; // Mutex for the network buffers
static volatile bool has_new_net_frame = false;

static int sock_fd = -1;
static volatile bool running = true;

// Thread synchronization
static pthread_mutex_t lock;
static char shared_result[64];
static volatile bool frame_ready = false;

// V4L2 Structs
struct buffer {
    void   *start;
    size_t length;
};
static struct buffer *buffers;
static unsigned int n_buffers;
static int fd_cam = -1;

// --- HELPERS ---

// Simple YUYV to RGB888 conversion
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

            // LVGL RGB888: B, G, R (if LV_COLOR_DEPTH 32, usually BGRA or ARGB)
            // Adjust based on your LV_COLOR_16_SWAP or similar.
            // Assuming standard 32-bit ARGB or 24-bit RGB.
            // Let's write RGB for now.

            int idx = (i * width + j) * 3; // 3 bytes per pixel
            dst[idx]     = b;
            dst[idx + 1] = g;
            dst[idx + 2] = r;

            r = y1 + (1.370705 * v);
            g = y1 - (0.698001 * v) - (0.337633 * u);
            b = y1 + (1.732446 * u);

            if (r < 0) r = 0; if (r > 255) r = 255;
            if (g < 0) g = 0; if (g > 255) g = 255;
            if (b < 0) b = 0; if (b > 255) b = 255;

            idx = (i * width + j + 1) * 3;
            dst[idx]     = b;
            dst[idx + 1] = g;
            dst[idx + 2] = r;
        }
    }
}

static int xioctl(int fh, int request, void *arg) {
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);
    return r;
}

// --- NEW: Network Thread Loop ---
static void *net_thread_entry(void *arg) {
    (void)arg;
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr);

    while (running) {
        // 1. Manage Connection
        if (sock_fd < 0) {
            sock_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                close(sock_fd);
                sock_fd = -1;
                usleep(1000000); // Retry every 1s
                continue;
            } else {
                LV_LOG_USER("Connected to ML Server");
            }
        }

        // 2. Check for new frame (Thread Safe Copy)
        bool work_to_do = false;
        pthread_mutex_lock(&net_lock);
        if (has_new_net_frame) {
            memcpy(sending_buf, shared_net_buf, IMG_W * IMG_H * 3);
            has_new_net_frame = false;
            work_to_do = true;
        }
        pthread_mutex_unlock(&net_lock);

        if (!work_to_do) {
            usleep(5000); // Sleep 5ms if no new frame to save CPU
            continue;
        }

        // 3. Send and Recv (Blocking, but only blocks this thread)
        uint32_t size = IMG_W * IMG_H * 3;
        uint32_t net_size = htonl(size);

        if (send(sock_fd, &net_size, 4, 0) < 0 ||
            send(sock_fd, sending_buf, size, 0) < 0) {
            LV_LOG_ERROR("Send failed, closing socket");
            close(sock_fd);
            sock_fd = -1;
            continue;
        }

        char result[64] = {0};
        if (recv(sock_fd, result, 64, 0) <= 0) {
            LV_LOG_ERROR("Recv failed, closing socket");
            close(sock_fd);
            sock_fd = -1;
            continue;
        }

        // 4. Update UI Result
        pthread_mutex_lock(&lock);
        strncpy(shared_result, result, 63);
        pthread_mutex_unlock(&lock);
    }

    if (sock_fd >= 0) close(sock_fd);
    return NULL;
}

// --- THREAD LOOP ---
static void *cam_thread_entry(void *arg) {
    (void)arg;

    // 1. Open Camera
    fd_cam = open(VIDEO_DEV, O_RDWR | O_NONBLOCK, 0);
    if (fd_cam == -1) {
        LV_LOG_ERROR("Cannot open camera");
        return NULL;
    }

    // 2. Init V4L2
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMG_W;
    fmt.fmt.pix.height = IMG_H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    xioctl(fd_cam, VIDIOC_S_FMT, &fmt);

    struct v4l2_requestbuffers req = {0};
    req.count = 2;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
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

    // 4. Loop
    int frame_cnt = 0;
    while (running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_cam, &fds);
        struct timeval tv = {2, 0};
        int r = select(fd_cam + 1, &fds, NULL, NULL, &tv);
        if (r <= 0) continue;

        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_cam, VIDIOC_DQBUF, &buf) == -1) continue;

        // Convert YUYV -> RGB (Display Buffer)
        yuyv_to_rgb((uint8_t*)buffers[buf.index].start, img_buf_rgb, IMG_W, IMG_H);

        // Hand off to Network Thread (Non-blocking copy)
        pthread_mutex_lock(&net_lock);
        memcpy(shared_net_buf, img_buf_rgb, IMG_W * IMG_H * 3);
        has_new_net_frame = true;
        pthread_mutex_unlock(&net_lock);

        // Signal UI that a frame is ready for display
        pthread_mutex_lock(&lock);
        frame_ready = true;
        pthread_mutex_unlock(&lock);

        xioctl(fd_cam, VIDIOC_QBUF, &buf);
    }

    close(fd_cam);
    return NULL;
}

// --- UI UPDATE TIMER ---
static void update_timer_cb(lv_timer_t * t) {
    (void)t;
    if (frame_ready) {
        pthread_mutex_lock(&lock);
        if (shared_result[0] != '\0') {
            lv_label_set_text(label_result, shared_result);
        }
        frame_ready = false;
        pthread_mutex_unlock(&lock);

        lv_obj_invalidate(img_display);
    }
}

// --- UI SETUP ---
void final_project_init(void) {
    // Init Mutex
    pthread_mutex_init(&lock, NULL);
    pthread_mutex_init(&net_lock, NULL); // Init new mutex

    // 1. Create Image Object
    img_display = lv_img_create(lv_screen_active());

    // Allocate buffers
    img_buf_rgb = malloc(IMG_W * IMG_H * 3);
    shared_net_buf = malloc(IMG_W * IMG_H * 3); // Allocate shared
    sending_buf = malloc(IMG_W * IMG_H * 3);    // Allocate sending
    memset(img_buf_rgb, 0, IMG_W * IMG_H * 3);

    // Init Image Descriptor
    img_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    img_dsc.header.w = IMG_W;
    img_dsc.header.h = IMG_H;
    img_dsc.header.stride = IMG_W * 3;
    img_dsc.data_size = IMG_W * IMG_H * 3;
    img_dsc.header.cf = LV_COLOR_FORMAT_RGB888;
    img_dsc.data = img_buf_rgb;

    lv_img_set_src(img_display, &img_dsc);
    lv_obj_center(img_display);

    // 2. Create Result Label
    label_result = lv_label_create(lv_screen_active());
    lv_label_set_text(label_result, "Waiting for ML...");
    lv_obj_align(label_result, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_style_text_font(label_result, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(label_result, lv_color_hex(0xFF0000), 0);

    // 3. Create Timer for UI Updates
    lv_timer_create(update_timer_cb, 30, NULL);

    // 4. Start Capture Thread
    pthread_t thread_id;
    pthread_create(&thread_id, NULL, cam_thread_entry, NULL);
    pthread_detach(thread_id);

    // 5. Start Network Thread
    pthread_t net_thread_id;
    pthread_create(&net_thread_id, NULL, net_thread_entry, NULL);
    pthread_detach(net_thread_id);
}
