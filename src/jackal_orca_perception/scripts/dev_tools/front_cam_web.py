#!/usr/bin/env python3
"""전면 카메라(camera1 / RealSense D435I serial 344522070202, /dev/video4) MJPEG 웹 스트리머.
pyrealsense2/flask 없이 cv2 V4L2 + 표준 http.server만 사용."""
import cv2
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DEVICE = "/dev/video4"   # 전면 RGB (usb-1.2). 후면이면 /dev/video10 으로 변경
PORT = 8080

_frame = None
_lock = threading.Lock()


def capture_loop():
    global _frame
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print(f"[ERR] {DEVICE} 열기 실패")
        return
    print(f"[OK] {DEVICE} 캡처 시작")
    while True:
        ok, img = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _lock:
                _frame = jpg.tobytes()


PAGE = """<!doctype html><html><head><meta charset=utf-8>
<title>Front Camera</title>
<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center;margin:0}
h1{font-size:18px;padding:10px}img{max-width:100%;height:auto}</style></head>
<body><h1>Jackal 전면 카메라 (camera1 / video4)</h1>
<img src="/stream"></body></html>""".encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with _lock:
                    f = _frame
                if f is None:
                    time.sleep(0.03)
                    continue
                try:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(f)).encode() +
                                     b"\r\n\r\n" + f + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
                time.sleep(0.03)
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(PAGE)


if __name__ == "__main__":
    threading.Thread(target=capture_loop, daemon=True).start()
    time.sleep(1.5)
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"[OK] http://0.0.0.0:{PORT} 서빙 중")
    srv.serve_forever()
