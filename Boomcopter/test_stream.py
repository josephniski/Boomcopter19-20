#!/usr/bin/python
"""
    Author: Igor Maculan - n3wtron@gmail.com
    A Simple mjpg stream http server
"""
import cv2
import Image
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import StringIO
import time
capture=None

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            i = 1
            while True:
                try:
                    #rc,img = capture.read()
                    #if not rc:
                    #	continue
                    #imgRGB=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
                    # jpg = Image.fromarray(imgRGB)
                    jpg = Image.open("test" + str(i) + ".jpg")
                    tmpFile = StringIO.StringIO()
                    jpg.save(tmpFile,'JPEG')
                    self.wfile.write("--jpgboundary")
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length',str(tmpFile.len))
                    self.end_headers()
                    jpg.save(self.wfile,'JPEG')
                    time.sleep(0.05)
                    i += 1
                    if i == 5:
                        i = 1
                except KeyboardInterrupt:
                    break
            return
        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>')
            self.wfile.write('<img src="http://boomcopter:8080/cam.mjpg"/>')
            self.wfile.write('</body></html>')
            return

def main():
    global capture
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    capture.set(cv2.CAP_PROP_SATURATION,0.2)
    global img
    try:
        server = HTTPServer(('',8080),CamHandler)
        print "server started"
        server.serve_forever()
    except KeyboardInterrupt:
        capture.release()
        server.socket.close()

if __name__ == '__main__':
    main()
