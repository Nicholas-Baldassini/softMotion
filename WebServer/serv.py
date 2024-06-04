

import cv2, sys, os



import numpy as np
import time
import threading
import subprocess
from fastgrab import screenshot




p = subprocess.Popen("xwininfo -id $(xprop -root | awk '/_NET_ACTIVE_WINDOW\(WINDOW\)/{print $NF}')", stdout=subprocess.PIPE, shell=True)
(output, err) = p.communicate()
p_status = p.wait()
val = output.decode().strip().split("\n")
name = val[0].split("\"")[1]

while True:
    time.sleep(0.7)
    p = subprocess.Popen("xwininfo -id $(xprop -root | awk '/_NET_ACTIVE_WINDOW\(WINDOW\)/{print $NF}')", stdout=subprocess.PIPE, shell=True)
    (output, err) = p.communicate()
    p_status = p.wait()
    val = output.decode().strip().split("\n")
    name = val[0].split("\"")[1]
    if "Bullet Physics" in name:
        break
    print("Not found bullet window yet")

x1 = int(val[2].split(":")[-1].strip())
y1 = int(val[3].split(":")[-1].strip())
width = int(val[6].split(":")[-1].strip())
height = int(val[7].split(":")[-1].strip())

print(f"Found bullet window: ({x1}, {y1}, {width}, {height})")

#box = (10, 10, 510, 510)
box = (x1, y1, x1+width-50, y1+height-50)

im = screenshot.Screenshot().capture(bbox=box)
def record_screen():
    global im
    while True:
        
        im = screenshot.Screenshot().capture(bbox=box)



        
t1 = threading.Thread(target=record_screen, daemon=True)
t1.start()
from flask import Flask, render_template, Response
app = Flask(__name__)
def gen_frames():  
    while True:
        #frame = np.array(ms.grab(monitor))
        frame = im
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.05)
        # success, frame = camera.read()  # read the camera frame
        # if not success:
        #     break
        # else:
        #     ret, buffer = cv2.imencode('.jpg', frame)
        #     frame = buffer.tobytes()
        #     yield (b'--frame\r\n'
        #            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
            
            
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    app.run(debug=True)