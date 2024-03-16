#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

image_pub = rospy.Publisher("/go2_camera/color/image", Image, queue_size=10)
cv_bridge = CvBridge()

rospy.init_node("go2_camera_stream")

def main():

    gst_pipeline = (
        "udpsrc port=4002 "
        "caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! "
        "appsink"
    )

    cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open video stream.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to get frame.")
            break

        image_pub.publish(cv_bridge.cv2_to_imgmsg(frame, "bgr8"))

        cv2.imshow('Frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
