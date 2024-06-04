#Import necessary libraries
from flask import Flask, render_template, Response
import cv2, sys, os

#from mss.linux import MSS as mss
#import mss
import pyscreenshot as imggrab
import numpy as np
import time
import threading

# sys.path.append('../')
# sys.path.append('../URDFS')
# import simulate
#Initialize the Flask app
app = Flask(__name__)

#camera = cv2.VideoCapture(0)

box = (10, 10, 510, 510)
im = np.array(imggrab.grab(bbox=box))
def record_screen():
    global im
    while True:
        im = np.array(imggrab.grab(bbox=box))
        
t1 = threading.Thread(target=record_screen, daemon=True)
t1.start()
def gen_frames():  
    while True:
        #frame = np.array(ms.grab(monitor))
        frame = im
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)
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