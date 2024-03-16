import cv2
import numpy as np
import subprocess
import time

# Revised FFmpeg command with the protocol whitelist specified at the beginning
ffmpeg_command = [
    'ffmpeg',
    '-protocol_whitelist', 'file,udp,rtp',  # Whitelist protocols at the start
    '-i', 'connection.sdp',  # Input from your .sdp file
    '-flags', 'low_delay',  # Low delay flag
    '-probesize', '32',  # Probesize set to 32
    '-vf', 'setpts=0',  # Video filter to set pts
    '-f', 'image2pipe',
    '-pix_fmt', 'bgr24',
    '-vcodec', 'rawvideo',
    'pipe:1'  # Use stdout as output
]

# Start the FFmpeg process 
pipe = subprocess.Popen(ffmpeg_command, stdout=subprocess.PIPE)

# Define the frame width and height (adjust according to the actual video resolution)
frame_width = 1280
frame_height = 720
time.sleep(2)

# Loop to read each frame from the stdout
while True:
    # Read a frame from the stdout
    raw_frame = pipe.stdout.read(frame_width * frame_height * 3)

    if not raw_frame:
        break

    # Convert the raw frame to a NumPy array
    frame = np.frombuffer(raw_frame, np.uint8).reshape((frame_height, frame_width, 3))

    # Display the frame using OpenCV
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# Cleanup
pipe.stdout.flush()
pipe.terminate()
cv2.destroyAllWindows()
