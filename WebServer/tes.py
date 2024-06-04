import cv2
import numpy as np

import pyscreenshot as imggrab

import time

import subprocess




p = subprocess.Popen("xwininfo -id $(xprop -root | awk '/_NET_ACTIVE_WINDOW\(WINDOW\)/{print $NF}')", stdout=subprocess.PIPE, shell=True)
(output, err) = p.communicate()
p_status = p.wait()
val = output.decode().strip().split("\n")
name = val[0].split("\"")[1]

while "Bullet Physics" not in name:
    time.sleep(0.7)
    p = subprocess.Popen("xwininfo -id $(xprop -root | awk '/_NET_ACTIVE_WINDOW\(WINDOW\)/{print $NF}')", stdout=subprocess.PIPE, shell=True)
    (output, err) = p.communicate()
    p_status = p.wait()
    val = output.decode().strip().split("\n")
    name = val[0].split("\"")[1]
    print("Not found bullet window yet")

x1 = int(val[2].split(":")[-1].strip())
y1 = int(val[3].split(":")[-1].strip())
width = int(val[6].split(":")[-1].strip())
height = int(val[7].split(":")[-1].strip())

print(f"Found bullet window: ({x1}, {y1}, {width}, {height})")

# box=(10, 10, 510, 510)
# from fastgrab import screenshot
# # take a full screen screenshot

# while True:
#     last_time = time.time()
#     #im = np.array(imggrab.grab(bbox=box))
#     img = screenshot.Screenshot().capture(bbox=box)
#     cv2.imshow("test", img)
#     if cv2.waitKey(25) & 0xFF == ord("q"):
#         cv2.destroyAllWindows()
#         break

#     print(f"fps: {1 / (time.time() - last_time)}")

# with mss.mss() as sct:
#     # Part of the screen to capture
#     monitor = {"top": 40, "left": 0, "width": 800, "height": 640}

#     while "Screen capturing":
#         last_time = time.time()

#         # Get raw pixels from the screen, save it to a Numpy array
#         img = numpy.array(sct.grab(monitor))

#         # Display the picture
#         cv2.imshow("OpenCV/Numpy normal", img)

#         # Display the picture in grayscale
#         # cv2.imshow('OpenCV/Numpy grayscale',
#         #            cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY))

#         print(f"fps: {1 / (time.time() - last_time)}")

#         # Press "q" to quit
#         if cv2.waitKey(25) & 0xFF == ord("q"):
#             cv2.destroyAllWindows()
#             break

