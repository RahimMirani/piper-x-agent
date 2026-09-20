"""Small LAN-only camera viewer; it never initializes the arm backend."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import time

PAGE = """<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>
<title>PiPER-X camera check</title><style>body{font:16px system-ui;background:#111;color:#ddd;margin:16px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px}img{width:100%;background:#000}
h2{font-size:18px;font-weight:500}</style><h1>PiPER-X camera check</h1>
<p>Camera-only viewer. The arm is not connected by this service.</p><div class=grid>
<section><h2>Scene — CC1N16200TF</h2><img src=/stream/scene></section>
<section><h2>Wrist — CC1N16200F9</h2><img src=/stream/wrist></section></div>"""


def serve_camera_web(runtime, bind="0.0.0.0", port=8090):
    cameras = runtime._cameras()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print(f"[camera-web] {self.address_string()} {fmt % args}", flush=True)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                body = PAGE.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            parts = path.strip("/").split("/")
            if len(parts) == 2 and parts[0] == "snapshot" and parts[1] in ("scene", "wrist"):
                try:
                    frame = cameras.latest(parts[1])
                    self.send_response(200)
                    self.send_header("Content-Type", frame.mime)
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(frame.data)))
                    self.end_headers()
                    self.wfile.write(frame.data)
                except Exception as exc:
                    self.send_error(503, str(exc))
                return
            if len(parts) == 2 and parts[0] == "stream" and parts[1] in ("scene", "wrist"):
                self.stream(parts[1])
                return
            self.send_error(404)

        def stream(self, role):
            boundary = b"frame"
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store, no-cache")
            self.end_headers()
            try:
                while True:
                    frame = cameras.latest(role)
                    self.wfile.write(b"--" + boundary + b"\r\nContent-Type: " + frame.mime.encode()
                                    + b"\r\nContent-Length: " + str(len(frame.data)).encode()
                                    + b"\r\n\r\n" + frame.data + b"\r\n")
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError, RuntimeError, OSError):
                return

    server = ThreadingHTTPServer((bind, port), Handler)
    print(f"Camera viewer listening at http://{bind}:{port}/", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
