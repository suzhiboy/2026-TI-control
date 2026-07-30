# -*- coding: utf-8 -*-
# LCKFB LushanPi K230 STA WiFi H.264 RTSP stream for 2026 TI H problem.
#
# Run this on CanMV K230, then open the printed URL in VLC:
#   Media -> Open Network Stream -> rtsp://<K230_IP>:8554/ball
#
# This script is independent from the MJPEG script. It uses the official
# RtspServer path: Sensor -> VENC(H.264) -> RTSP.

import gc
import network
import os
import time

import multimedia as mm
from media.rtspserver import RtspServer


# ---------------- User config ----------------
WIFI_SSID = "ICE"
WIFI_PASSWORD = "15013623467"

RTSP_PORT = 8554
RTSP_SUFFIX = "ball"

STREAM_WIDTH = 640
STREAM_HEIGHT = 360
BITRATE_KBPS = 512
GOP = 30

WIFI_CONNECT_TIMEOUT_MS = 20000
HEARTBEAT_MS = 10000


def enable_exitpoint():
    fn = getattr(os, "exitpoint", None)
    mode = getattr(os, "EXITPOINT_ENABLE", None)
    if fn is None or mode is None:
        return
    try:
        fn(mode)
    except Exception as e:
        print("[exitpoint] enable failed:", e)


def enable_exitpoint_sleep():
    fn = getattr(os, "exitpoint", None)
    mode = getattr(os, "EXITPOINT_ENABLE_SLEEP", None)
    if fn is None or mode is None:
        return
    try:
        fn(mode)
    except Exception as e:
        print("[exitpoint] sleep mode failed:", e)


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


def set_default_network_device():
    set_default = getattr(network, "set_default_dev", None)
    if set_default is None:
        return

    # Firmware names vary across CanMV builds. Try harmless common names.
    for dev in ("wlan0", "w0", "sta0", "wlan"):
        try:
            set_default(dev)
            print("[net] default device=%s" % dev)
            return
        except Exception:
            pass
    print("[net] default device unchanged")


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
                raise RuntimeError("WiFi connect timeout. Use a 2.4 GHz hotspot/router.")
            sleep_ms(200)

    ip = wlan.ifconfig()[0]
    print("[wifi] connected, ip=%s" % ip)
    set_default_network_device()
    return wlan, ip


def make_rtsp_server():
    attempts = (
        (
            "new-api",
            {
                "session_name": RTSP_SUFFIX,
                "port": RTSP_PORT,
                "video_type": mm.multi_media_type.media_h264,
                "enable_audio": False,
                "width": STREAM_WIDTH,
                "height": STREAM_HEIGHT,
                "bit_rate": BITRATE_KBPS,
                "gop_len": GOP,
            },
        ),
        (
            "size-api",
            {
                "session_name": RTSP_SUFFIX,
                "port": RTSP_PORT,
                "video_type": mm.multi_media_type.media_h264,
                "enable_audio": False,
                "width": STREAM_WIDTH,
                "height": STREAM_HEIGHT,
            },
        ),
        (
            "legacy-api",
            {
                "session_name": RTSP_SUFFIX,
                "port": RTSP_PORT,
                "video_type": mm.multi_media_type.media_h264,
                "enable_audio": False,
            },
        ),
    )

    last_error = None
    for name, kwargs in attempts:
        try:
            server = RtspServer(**kwargs)
            print("[rtsp] RtspServer constructor=%s" % name)
            if name == "legacy-api":
                print("[rtsp] legacy API: resolution/bitrate use firmware defaults")
            elif name == "size-api":
                print("[rtsp] size API: bitrate/gop use firmware defaults")
            return server
        except TypeError as e:
            last_error = e
            print("[rtsp] constructor %s failed: %s" % (name, e))

    raise last_error


def get_rtsp_url(server, ip):
    getter = getattr(server, "get_rtsp_url", None)
    if getter is not None:
        try:
            return getter()
        except Exception:
            pass
    return "rtsp://%s:%d/%s" % (ip, RTSP_PORT, RTSP_SUFFIX)


def main():
    wlan = None
    rtsp = None
    last_heartbeat = 0

    try:
        enable_exitpoint()
        print("[boot] LCKFB K230 H.264 RTSP stream")
        wlan, ip = connect_wifi()
        rtsp = make_rtsp_server()
        rtsp.start()

        url = get_rtsp_url(rtsp, ip)
        print("[rtsp] open in VLC: %s" % url)

        while True:
            exitpoint()
            now = ticks_ms()
            if ticks_diff(now, last_heartbeat) >= HEARTBEAT_MS:
                last_heartbeat = now
                if wlan is not None and not wlan.isconnected():
                    print("[wifi] disconnected, reconnecting")
                    wlan, ip = connect_wifi()
                    print("[rtsp] open in VLC: %s" % get_rtsp_url(rtsp, ip))
                else:
                    print("[rtsp] running")
            sleep_ms(100)

    except KeyboardInterrupt:
        print("[exit] interrupted")
    finally:
        if rtsp is not None:
            try:
                rtsp.stop()
            except Exception:
                pass
        enable_exitpoint_sleep()
        sleep_ms(100)
        gc.collect()
        print("[exit] resources released")


main()
