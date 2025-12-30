ffmpeg -re -stream_loop -1 -i {video.mp4} -f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0
ffmpeg -re -stream_loop -1 -i  -f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0








Create webcam device:
sudo modprobe v4l2loopback exclusive_caps=1 card_label="VirtualCam"

Load video to webcam:
ffmpeg -re -stream_loop -1 -i {video.mp4} -f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0



New York 1
ffmpeg -re -stream_loop -1 -i ~/Downloads/NewYork1_480p.mp4 -f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0
New York 2
ffmpeg -re -stream_loop -1 -i ~/Downloads/NewYork2_480p.mp4 -f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0
PacificPark
ffmpeg -re -stream_loop -1 -i ~/Downloads/PacificPark_480p.mp4 -f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0
Hollywood
ffmpeg -re -stream_loop -1 -i ~/Downloads/Hollywood_480p.mp4 -f v4l2 -vcodec rawvideo -pix_fmt yuyv422 /dev/video0
