import cv2
import numpy as np

import pyscreenshot as imggrab

import time

"xwininfo -id $(xprop -root | awk '/_NET_ACTIVE_WINDOW\(WINDOW\)/{print $NF}')"
box=(10, 10, 510, 510)
while True:
    last_time = time.time()
    im = np.array(imggrab.grab(bbox=box))
    cv2.imshow("test", im)
    if cv2.waitKey(25) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break

    print(f"fps: {1 / (time.time() - last_time)}")

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

