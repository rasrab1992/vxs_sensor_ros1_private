#!/usr/bin/env python3
"""IMX219 publisher — fakesink + signal-handoffs.
On Argus TIMEOUT, restarts nvargus-daemon and re-execs this process so
libargus gets a fresh socket connection (in-process restart is not enough)."""

import os
import queue
import subprocess
import sys
import time

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)
except Exception as exc:
    print(f"GStreamer Python bindings unavailable: {exc}", file=sys.stderr)
    sys.exit(1)

# IMX219 sensor modes (JP 5.x):
#   mode=0: 3264x2464 @ 21fps   mode=1: 3264x1848 @ 28fps
#   mode=2: 1920x1080 @ 30fps   mode=3: 1640x1232 @ 30fps
#   mode=4: 1280x720  @ 60fps   mode=5: 1280x720  @ 120fps
# mode=4 triggers frequent Argus TIMEOUTs on JP 5.1.3; mode=2 is stable.
# Capture at 1920x1080 then scale to 1280x720 so existing calibration stays valid.
SENSOR_MODE = 2
CAP_WIDTH   = 1920
CAP_HEIGHT  = 1080
CAP_FPS     = 30
WIDTH       = 1280
HEIGHT      = 720
PUB_FPS     = 10

PIPELINE_STR = (
    f"nvarguscamerasrc sensor-id=0 sensor-mode={SENSOR_MODE} "
    f"! video/x-raw(memory:NVMM),width={CAP_WIDTH},height={CAP_HEIGHT},format=NV12,framerate={CAP_FPS}/1 "
    f"! nvvidconv "
    f"! video/x-raw(memory:NVMM),width={WIDTH},height={HEIGHT},format=NV12 "
    f"! nvvidconv "
    f"! video/x-raw,format=I420 "
    f"! queue leaky=1 max-size-buffers=4 "
    f"! videorate "
    f"! video/x-raw,framerate={PUB_FPS}/1 "
    f"! fakesink name=fsink sync=false signal-handoffs=true"
)

_frame_q = queue.Queue(maxsize=1)


def on_handoff(fakesink, buf, pad):
    ok, mapinfo = buf.map(Gst.MapFlags.READ)
    if not ok:
        return
    try:
        arr = np.frombuffer(mapinfo.data, dtype=np.uint8).copy()
    finally:
        buf.unmap(mapinfo)
    try:
        _frame_q.put_nowait(arr)
    except queue.Full:
        pass


def restart_daemon_and_reexec():
    """Restart nvargus-daemon then re-exec this process.
    In-process pipeline restart is not enough — libargus holds a stale
    socket to the old daemon instance that cannot be recovered."""
    rospy.logwarn("TIMEOUT: restarting nvargus-daemon and re-execing publisher...")
    try:
        subprocess.run(["sudo", "systemctl", "restart", "nvargus-daemon"],
                       timeout=15, check=True)
        for _ in range(20):
            result = subprocess.run(
                ["systemctl", "is-active", "nvargus-daemon"],
                capture_output=True, text=True)
            if result.stdout.strip() == "active":
                break
            time.sleep(1)
        time.sleep(5)  # extra buffer for Argus socket to accept connections
    except Exception as exc:
        rospy.logwarn("Could not restart daemon: %s — sleeping 10s", exc)
        time.sleep(10)

    rospy.loginfo("Re-execing process to get fresh libargus socket...")
    os.execv(sys.executable, [sys.executable] + sys.argv)


def build_pipeline():
    pipeline = Gst.parse_launch(PIPELINE_STR)
    fsink = pipeline.get_by_name("fsink")
    fsink.connect("handoff", on_handoff)
    return pipeline


def main():
    rospy.init_node("imx219_publisher")
    frame_id   = rospy.get_param("~frame_id", "camera")
    calib_file = rospy.get_param(
        "~camera_info_url",
        "/home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/calibration/imx219_camera_info.yaml"
    )
    pub_comp = rospy.Publisher("/camera/image_raw/compressed", CompressedImage, queue_size=1)
    pub_raw  = rospy.Publisher("/camera/image_raw",            Image,           queue_size=1)
    pub_info = rospy.Publisher("/camera/camera_info",          CameraInfo,      queue_size=1)

    with open(calib_file) as f:
        c = yaml.safe_load(f)
    _camera_info = CameraInfo()
    _camera_info.width            = c["image_width"]
    _camera_info.height           = c["image_height"]
    _camera_info.distortion_model = c["distortion_model"]
    _camera_info.D = c["distortion_coefficients"]["data"]
    _camera_info.K = c["camera_matrix"]["data"]
    _camera_info.R = c["rectification_matrix"]["data"]
    _camera_info.P = c["projection_matrix"]["data"]
    rospy.loginfo("Loaded camera_info from %s", calib_file)

    interval = 1.0 / PUB_FPS

    rospy.loginfo("Building pipeline...")
    pipeline = build_pipeline()
    bus = pipeline.get_bus()

    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        rospy.logerr("Pipeline failed to enter PLAYING state — restarting daemon and re-execing")
        pipeline.set_state(Gst.State.NULL)
        restart_daemon_and_reexec()
        return  # never reached — re-exec replaces process

    rospy.loginfo("Pipeline PLAYING")

    while not rospy.is_shutdown():
        t0 = time.time()

        msg = bus.timed_pop_filtered(0, Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if msg is not None:
            if msg.type == Gst.MessageType.ERROR:
                err, _ = msg.parse_error()
                rospy.logwarn("GStreamer ERROR: %s", err)
            else:
                rospy.logwarn("GStreamer EOS")
            pipeline.set_state(Gst.State.NULL)
            restart_daemon_and_reexec()
            return  # never reached — re-exec replaces process

        try:
            arr = _frame_q.get_nowait()
        except queue.Empty:
            time.sleep(max(0.0, interval - (time.time() - t0)))
            continue

        stamp = rospy.Time.now()
        yuv = arr.reshape((HEIGHT * 3 // 2, WIDTH))
        bgr = cv2.flip(cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420), -1)

        ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if ok:
            jpg_bytes = jpg.tobytes()

            cmsg = CompressedImage()
            cmsg.header.stamp    = stamp
            cmsg.header.frame_id = frame_id
            cmsg.format          = "jpeg"
            cmsg.data            = jpg_bytes
            pub_comp.publish(cmsg)

            raw = Image()
            raw.header.stamp    = stamp
            raw.header.frame_id = frame_id
            raw.height          = HEIGHT
            raw.width           = WIDTH
            raw.encoding        = "bgr8"
            raw.step            = WIDTH * 3
            raw.data            = bgr.tobytes()
            pub_raw.publish(raw)

            _camera_info.header.stamp    = stamp
            _camera_info.header.frame_id = frame_id
            pub_info.publish(_camera_info)

        time.sleep(max(0.0, interval - (time.time() - t0)))

    pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    main()
