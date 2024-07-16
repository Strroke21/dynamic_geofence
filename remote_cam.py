from flask import Flask, Response
import cv2
import numpy as np

app = Flask(__name__)
cap = cv2.VideoCapture(0)  # Use 0 for default camera

# Set resolution
cap.set(3, 1280)  # Width
cap.set(4, 720)   # Height

def gen():
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # Encode frame to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            break
        # Convert JPEG to bytes
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n\r\n')

@app.route('/')
def video_feed():
    return Response(gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
