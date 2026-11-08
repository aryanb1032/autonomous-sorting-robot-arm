"""Exercise the phone-stream path using a local MJPEG server, no real camera."""
import http.server,json,subprocess,sys,threading,time
from pathlib import Path
import cv2
ROOT=Path(__file__).resolve().parents[1]

def test_mjpeg_source(tmp_path):
    img=cv2.imread(str(ROOT/'examples/synthetic_input.png'))
    ok,encoded=cv2.imencode('.jpg',img);assert ok
    payload=encoded.tobytes()
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.end_headers()
            try:
                for _ in range(60):
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(payload)).encode()+b'\r\n\r\n'+payload+b'\r\n');self.wfile.flush();time.sleep(.03)
            except (BrokenPipeError,ConnectionResetError):pass
    server=http.server.ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        result=subprocess.run([sys.executable,str(ROOT/'box_vision.py'),'--stream',f'http://127.0.0.1:{server.server_port}/video','--headless','--frames','3','--output',str(tmp_path)],capture_output=True,text=True,timeout=15)
        assert result.returncode==0,result.stderr
        report=json.loads((tmp_path/'latest.json').read_text())
        assert report['frame_index']==2
        assert report['counts']=={'black':2,'white':2}
        assert report['calibrated'] is False
        assert report['physical_control_enabled'] is False
        assert (tmp_path/'latest_raw.jpg').exists()
    finally:server.shutdown();server.server_close()
