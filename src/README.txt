ffmpeg -re -stream_loop -1 -i {video.mp4}-f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0
