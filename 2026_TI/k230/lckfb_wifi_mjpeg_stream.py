# -*- coding: utf-8 -*-
# LCKFB LushanPi K230 STA WiFi MJPEG stream for 2026 TI H problem.
#
# Usage:
# 1. Set WIFI_SSID and WIFI_PASSWORD below. Use a 2.4 GHz hotspot/router.
# 2. Copy this file to the K230 board.
# 3. Run it on CanMV K230. The serial console prints: http://<K230_IP>
# 4. Open that URL on a PC connected to the same hotspot/router.
#
# This file only handles video transmission. Ball recognition can later be added
# in run_ball_detect() using CAM_CHN_ID_2, without changing the MJPEG path.

from media.sensor import *
from media.display import *
from media.media import *
import image
import network
import usocket
import gc
import os
import time


# ---------------- User config ----------------
WIFI_SSID = "ICE"
WIFI_PASSWORD = "15013623467"

HTTP_PORT = 80

# Keep this modest. Higher values improve quality but consume more bandwidth and CPU.
STREAM_WIDTH = 640
STREAM_HEIGHT = 360
JPEG_QUALITY = 45
STREAM_FRAME_DELAY_MS = 100
SEND_CHUNK_SIZE = 1024
SEND_RETRY_LIMIT = 200
BACKLOG_SAFE = 600

# Set to "lcd3_5", "lcd2_4", or "hdmi".
DISPLAY = "lcd3_5"

WIFI_CONNECT_TIMEOUT_MS = 20000
ACCEPT_POLL_SLEEP_MS = 5
CLIENT_HEADER_TIMEOUT_MS = 1000
GC_EVERY_N_FRAMES = 30


DISPLAY_MAP = {
    "lcd3_5": ("lcd", 800, 480),
    "lcd2_4": ("lcd", 640, 480),
    "hdmi": ("hdmi", 1920, 1080),
}

MJPEG_HEADER = (
    b"HTTP/1.1 200 OK\r\n"
    b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    b"Cache-Control: no-cache, no-store, must-revalidate\r\n"
    b"Pragma: no-cache\r\n"
    b"Expires: 0\r\n"
    b"Connection: close\r\n"
    b"Access-Control-Allow-Origin: *\r\n"
    b"\r\n"
)

_IPPROTO_TCP = 6
_TCP_NODELAY = 1

stream_client = None
pending = b""
stream_frame = 0
stream_count = 0
backlog_cnt = 0


def ticks_ms():
    return time.ticks_ms()


def ticks_diff(a, b):
    return time.ticks_diff(a, b)


def sleep_ms(ms):
    time.sleep_ms(ms)


def exitpoint():
    fn = getattr(os, "exitpoint", None)
    if fn is None:
        return
    try:
        fn()
    except (KeyboardInterrupt, SystemExit):
        raise
    except OSError as e:
        if getattr(e, "errno", None) not in (-1, 4):
            raise


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print("[wifi] connecting to %s ..." % WIFI_SSID)
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        start = ticks_ms()
        while not wlan.isconnected():
            exitpoint()
            if ticks_diff(ticks_ms(), start) > WIFI_CONNECT_TIMEOUT_MS:
                raise RuntimeError("WiFi connect timeout. Check 2.4 GHz SSID/password.")
            sleep_ms(200)

    ip = wlan.ifconfig()[0]
    print("[wifi] connected, ip=%s" % ip)
    return wlan, ip


def display_config():
    mode, width, height = DISPLAY_MAP.get(DISPLAY, DISPLAY_MAP["lcd3_5"])
    return mode, width, height


def init_camera_and_display():
    display_mode, display_w, display_h = display_config()

    sensor = Sensor(id=2)
    sensor.reset()

    # Channel 0: hardware display. Keep it matched with the real display.
    sensor.set_framesize(width=display_w, height=display_h)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)

    # Channel 1: MJPEG stream source.
    sensor.set_framesize(width=STREAM_WIDTH, height=STREAM_HEIGHT, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

    # Channel 2: reserved for later AI recognition.
    sensor.set_framesize(width=STREAM_WIDTH, height=STREAM_HEIGHT, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    bind = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    bind["layer"] = Display.LAYER_VIDEO1
    Display.bind_layer(**bind)

    if display_mode == "hdmi":
        Display.init(Display.LT9611, to_ide=True)
    else:
        Display.init(Display.ST7701, to_ide=True)

    MediaManager.init()
    sensor.run()
    sleep_ms(300)
    return sensor


def init_server(port):
    addr = usocket.getaddrinfo("0.0.0.0", port)[0][-1]
    server = usocket.socket(usocket.AF_INET, usocket.SOCK_STREAM)
    server.setsockopt(usocket.SOL_SOCKET, usocket.SO_REUSEADDR, 1)
    server.bind(addr)
    server.listen(1)
    server.setblocking(False)
    return server


def set_nodelay(sock):
    try:
        sock.setsockopt(_IPPROTO_TCP, _TCP_NODELAY, 1)
    except Exception:
        pass


def build_index_html(ip):
    html = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>K230 WiFi MJPEG Stream</title>
<style>
body{margin:0;background:#101216;color:#e8eef6;font-family:system-ui,Segoe UI,Arial,sans-serif}
main{max-width:920px;margin:0 auto;padding:16px}
header{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:12px}
h1{font-size:20px;margin:0;font-weight:650}
.pill{font-size:13px;color:#8ee08e;border:1px solid #315a31;border-radius:999px;padding:4px 10px}
.viewer{background:#000;border:1px solid #2a3038;aspect-ratio:16/9;display:flex;align-items:center;justify-content:center;overflow:hidden}
img{width:100%;height:100%;object-fit:contain}
.bar{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin-top:12px}
button,a{border:1px solid #354150;background:#171c24;color:#e8eef6;border-radius:6px;padding:8px 12px;text-decoration:none;font-size:14px}
button:disabled{opacity:.45}
.rec{color:#ff8e8e}
.meta{color:#9ba8b8;font-size:13px;margin-top:8px}
</style>
</head>
<body>
<main>
<header><h1>K230 WiFi MJPEG Stream</h1><span id="state" class="pill">connecting</span></header>
<div class="viewer"><img id="stream" src="/stream" alt="K230 stream"></div>
<div class="bar">
<button id="start">Start PC Record</button>
<button id="stop" disabled>Stop Record</button>
<a href="/snapshot" target="_blank">Snapshot</a>
<span id="timer" class="rec"></span>
</div>
<div class="meta">Open from the same 2.4 GHz hotspot/router: http://__K230_IP__</div>
</main>
<script>
const img=document.getElementById('stream');
const state=document.getElementById('state');
const startBtn=document.getElementById('start');
const stopBtn=document.getElementById('stop');
const timer=document.getElementById('timer');
let rec=null,chunks=[],started=0,tick=null;
img.onerror=()=>{
  state.textContent='reconnecting';
  setTimeout(()=>{img.src='/stream?t='+Date.now()},2000);
};
img.onload=()=>{
  state.textContent='streaming';
};
function updateTimer(){const s=Math.floor((Date.now()-started)/1000);timer.textContent='REC '+s+'s'}
startBtn.onclick=()=>{
  const canvas=document.createElement('canvas');
  canvas.width=640;canvas.height=360;
  const ctx=canvas.getContext('2d');
  function draw(){try{ctx.drawImage(img,0,0,canvas.width,canvas.height)}catch(e){}requestAnimationFrame(draw)}
  draw();
  const stream=canvas.captureStream(20);
  chunks=[];
  let opts={};
  if(MediaRecorder.isTypeSupported('video/webm;codecs=vp9'))opts={mimeType:'video/webm;codecs=vp9'};
  rec=new MediaRecorder(stream,opts);
  rec.ondataavailable=e=>{if(e.data&&e.data.size)chunks.push(e.data)};
  rec.onstop=()=>{
    const blob=new Blob(chunks,{type:'video/webm'});
    const a=document.createElement('a');
    a.href=URL.createObjectURL(blob);
    a.download='k230-stream-'+new Date().toISOString().replace(/[:.]/g,'-')+'.webm';
    a.click();
    URL.revokeObjectURL(a.href);
  };
  rec.start();
  started=Date.now();
  tick=setInterval(updateTimer,500);
  updateTimer();
  startBtn.disabled=true;stopBtn.disabled=false;
};
stopBtn.onclick=()=>{
  if(rec&&rec.state==='recording')rec.stop();
  clearInterval(tick);
  timer.textContent='';
  startBtn.disabled=false;stopBtn.disabled=true;
};
</script>
</body>
</html>""".replace("__K230_IP__", ip)
    return html.encode()


def http_response(content_type, body):
    return (
        b"HTTP/1.1 200 OK\r\n"
        b"Content-Type: " + content_type + b"\r\n"
        b"Content-Length: " + str(len(body)).encode() + b"\r\n"
        b"Connection: close\r\n"
        b"Access-Control-Allow-Origin: *\r\n"
        b"\r\n" + body
    )


def build_mjpeg_part(jpeg):
    return (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n"
        b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" +
        jpeg + b"\r\n"
    )


def close_stream(reason=""):
    global stream_client, pending, stream_frame, backlog_cnt
    if stream_client is not None:
        try:
            stream_client.close()
        except Exception:
            pass
        print("[stream] closed %s frames=%d" % (reason, stream_frame))
    stream_client = None
    pending = b""
    stream_frame = 0
    backlog_cnt = 0


def try_send(sock, data):
    global pending
    if pending:
        data = pending + data
        pending = b""
    try:
        sent = sock.send(data)
    except OSError as e:
        errno = getattr(e, "errno", None)
        if errno is None and len(e.args) > 0:
            errno = e.args[0]
        if errno in (11, 35):
            pending = data
            return "block"
        print("[stream] send error:", e)
        return "error"
    if sent is None:
        sent = 0
    if sent < len(data):
        pending = data[sent:]
        return "block"
    return "ok"


def send_all(sock, data):
    sent = 0
    retry_count = 0
    while sent < len(data):
        exitpoint()
        end = sent + SEND_CHUNK_SIZE
        if end > len(data):
            end = len(data)
        try:
            n = sock.send(data[sent:end])
        except OSError as e:
            errno = getattr(e, "errno", None)
            if errno is None and len(e.args) > 0:
                errno = e.args[0]
            if errno == 11:
                retry_count += 1
                if retry_count > SEND_RETRY_LIMIT:
                    print("[http] send retry timeout:", e)
                    return False
                sleep_ms(5)
                continue
            print("[http] send error:", e)
            return False
        if n is None:
            n = 0
        if n <= 0:
            retry_count += 1
            if retry_count > SEND_RETRY_LIMIT:
                print("[http] send stalled")
                return False
            sleep_ms(2)
        else:
            sent += n
            retry_count = 0
    return True


def read_request(sock):
    req = b""
    deadline = ticks_ms() + CLIENT_HEADER_TIMEOUT_MS
    while b"\r\n\r\n" not in req:
        exitpoint()
        try:
            chunk = sock.recv(256)
        except OSError:
            chunk = None
        if chunk:
            req += chunk
            if len(req) > 1024:
                break
        elif ticks_diff(ticks_ms(), deadline) > 0:
            break
        else:
            sleep_ms(2)
    try:
        return req.decode()
    except Exception:
        return ""


def request_first_line(request):
    if not request:
        return "<empty>"
    return request.split("\r\n", 1)[0]


def snapshot_jpeg(sensor):
    img = sensor.snapshot(chn=CAM_CHN_ID_1)
    try:
        return bytes(img.compress(quality=JPEG_QUALITY))
    finally:
        img.__del__()


def run_ball_detect(sensor):
    # Reserved for later:
    # ai_img = sensor.snapshot(chn=CAM_CHN_ID_2)
    # Run KPU/YOLO here, then release ai_img.
    return None


def close_client(client):
    if client is None:
        return
    try:
        client.close()
    except Exception:
        pass


def serve_stream(client, sensor):
    if not send_all(client, MJPEG_HEADER):
        return
    frame_count = 0
    while True:
        exitpoint()
        jpeg = snapshot_jpeg(sensor)
        if not send_all(client, build_mjpeg_part(jpeg)):
            break
        frame_count += 1
        sleep_ms(STREAM_FRAME_DELAY_MS)
        if frame_count % GC_EVERY_N_FRAMES == 0:
            gc.collect()


def main():
    global stream_client, pending, stream_frame, stream_count, backlog_cnt

    wlan = None
    sensor = None
    server = None
    last_jpeg = None
    last_stream_tick = 0

    try:
        print("[boot] LCKFB K230 STA WiFi MJPEG stream")
        wlan, ip = connect_wifi()
        sensor = init_camera_and_display()
        server = init_server(HTTP_PORT)
        index_html = build_index_html(ip)

        print("[http] open http://%s" % ip)

        while True:
            exitpoint()
            now = ticks_ms()

            if stream_client is not None and ticks_diff(now, last_stream_tick) >= STREAM_FRAME_DELAY_MS:
                last_stream_tick = now
                if pending:
                    result = try_send(stream_client, b"")
                else:
                    try:
                        last_jpeg = snapshot_jpeg(sensor)
                    except Exception as e:
                        print("[stream] snapshot error:", e)
                        last_jpeg = last_jpeg

                    if not last_jpeg:
                        result = "block"
                    else:
                        result = try_send(stream_client, build_mjpeg_part(last_jpeg))

                if result == "error":
                    close_stream("send error")
                elif result == "block":
                    backlog_cnt += 1
                    if backlog_cnt > BACKLOG_SAFE:
                        close_stream("backlog timeout")
                else:
                    if not pending:
                        stream_frame += 1
                    backlog_cnt = 0

                if stream_frame % GC_EVERY_N_FRAMES == 0:
                    gc.collect()

            try:
                client, addr = server.accept()
            except OSError:
                if wlan is not None and not wlan.isconnected():
                    print("[wifi] disconnected, reconnecting")
                    wlan, ip = connect_wifi()
                    index_html = build_index_html(ip)
                    print("[http] open http://%s" % ip)
                sleep_ms(ACCEPT_POLL_SLEEP_MS)
                continue

            keep_client_open = False
            try:
                try:
                    client.setblocking(True)
                except Exception:
                    pass
                request = read_request(client)
                print("[http] %s from %s" % (request_first_line(request), addr))
                if "GET /stream" in request:
                    close_stream("replaced")
                    stream_client = client
                    keep_client_open = True
                    stream_frame = 0
                    backlog_cnt = 0
                    pending = b""
                    stream_count += 1
                    try:
                        stream_client.setblocking(False)
                    except Exception:
                        pass
                    set_nodelay(stream_client)
                    print("[stream #%d] client %s" % (stream_count, addr))
                    try_send(stream_client, MJPEG_HEADER)
                    if last_jpeg:
                        if try_send(stream_client, build_mjpeg_part(last_jpeg)) == "ok":
                            stream_frame = 1
                elif "GET /snapshot" in request:
                    jpeg = snapshot_jpeg(sensor)
                    if not send_all(client, http_response(b"image/jpeg", jpeg)):
                        print("[http] snapshot response failed")
                else:
                    if not send_all(client, http_response(b"text/html; charset=utf-8", index_html)):
                        print("[http] index response failed")
            except Exception as e:
                print("[http] request error:", e)
            finally:
                if not keep_client_open:
                    close_client(client)
                gc.collect()

    except KeyboardInterrupt:
        print("[exit] interrupted")
    finally:
        close_stream("exit")
        close_client(server)
        try:
            if sensor is not None:
                sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass
        gc.collect()
        print("[exit] resources released")


main()
