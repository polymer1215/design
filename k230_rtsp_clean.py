"""K230 Yahboom: Wi-Fi AP + H.264 RTSP camera server.

Target firmware:
    CanMV v1.4.3, k230_canmv_yahboom
"""

import gc
import os
import time
import uctypes
import network
import multimedia as mm

from media.media import *
from media.sensor import *
from media.vencoder import *


AP_SSID = "K230D_BALL"
AP_KEY = "12345678"
RTSP_PORT = 8554
RTSP_SESSION = "test"
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
VENC_BUFFER_COUNT = 8
BOOT_DELAY_SECONDS = 5
NETWORK_RETRY_COUNT = 10


def start_wifi_ap():
    """Configure the RT-Smart network interface as a Wi-Fi access point."""
    ap = network.WLAN(network.AP_IF)
    last_error = None

    for attempt in range(NETWORK_RETRY_COUNT):
        try:
            ap.active(True)
            ap.config(ssid=AP_SSID, key=AP_KEY)
            time.sleep(1)

            ip_address = ap.ifconfig()[0]
            if ip_address and ip_address != "0.0.0.0":
                print("Wi-Fi SSID:", AP_SSID)
                print("Wi-Fi key :", AP_KEY)
                print("RTSP URL  : rtsp://%s:%d/%s" %
                      (ip_address, RTSP_PORT, RTSP_SESSION))
                return ap
        except BaseException as error:
            last_error = error

        print("Waiting for network:", attempt + 1, "/", NETWORK_RETRY_COUNT)
        time.sleep(1)

    if last_error is not None:
        raise last_error
    raise RuntimeError("Wi-Fi AP did not obtain an IP address")


class RtspCameraServer:
    def __init__(self):
        self.venc_chn = VENC_CHN_ID_0

        self.sensor = None
        self.encoder = None
        self.link = None
        self.rtsp = None

        self.media_initialized = False
        self.encoder_created = False
        self.encoder_started = False
        self.sensor_started = False
        self.rtsp_initialized = False
        self.rtsp_started = False

    def initialize(self):
        """Create the camera, VENC, media pools, and RTSP server."""
        width = ALIGN_UP(VIDEO_WIDTH, 16)
        height = VIDEO_HEIGHT

        # 1. Configure the camera before initializing MediaManager.
        self.sensor = Sensor()
        self.sensor.reset()
        self.sensor.set_framesize(
            width=width,
            height=height,
            alignment=12,
        )
        self.sensor.set_pixformat(Sensor.YUV420SP)

        # 2. Add the VENC output pool before initializing MediaManager.
        self.encoder = Encoder()
        self.encoder.SetOutBufs(
            self.venc_chn,
            VENC_BUFFER_COUNT,
            width,
            height,
        )

        # 3. Bind camera channel 0 to VENC channel 0.
        self.link = MediaManager.link(
            self.sensor.bind_info()["src"],
            (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, self.venc_chn),
        )

        # 4. Initialize all configured video-buffer pools exactly once.
        MediaManager.init()
        self.media_initialized = True

        # 5. Create the H.264 encoder.
        encoder_attr = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            width,
            height,
        )
        self.encoder.Create(self.venc_chn, encoder_attr)
        self.encoder_created = True

        # 6. Create and start the RTSP service.
        self.rtsp = mm.rtsp_server()
        self.rtsp.rtspserver_init(RTSP_PORT)
        self.rtsp_initialized = True
        self.rtsp.rtspserver_createsession(
            RTSP_SESSION,
            mm.multi_media_type.media_h264,
            False,
        )
        self.rtsp.rtspserver_start()
        self.rtsp_started = True

        # 7. Start VENC first, then start the camera.
        self.encoder.Start(self.venc_chn)
        self.encoder_started = True
        self.sensor.run()
        self.sensor_started = True

    def serve_forever(self):
        """Read encoded frames and send them to connected RTSP clients."""
        stream = StreamData()

        while True:
            os.exitpoint()
            stream_acquired = False

            try:
                self.encoder.GetStream(self.venc_chn, stream)
                stream_acquired = True

                for index in range(stream.pack_cnt):
                    size = stream.data_size[index]
                    data = bytes(
                        uctypes.bytearray_at(stream.data[index], size)
                    )
                    self.rtsp.rtspserver_sendvideodata(
                        RTSP_SESSION,
                        data,
                        size,
                        1000,
                    )
            finally:
                # Every successful GetStream must have one ReleaseStream.
                if stream_acquired:
                    self.encoder.ReleaseStream(self.venc_chn, stream)

    def close(self):
        """Release resources in the reverse order of initialization."""
        if self.sensor_started:
            try:
                self.sensor.stop()
            finally:
                self.sensor_started = False

        if self.link is not None:
            del self.link
            self.link = None

        if self.encoder_started:
            try:
                self.encoder.Stop(self.venc_chn)
            finally:
                self.encoder_started = False

        if self.encoder_created:
            try:
                self.encoder.Destroy(self.venc_chn)
            finally:
                self.encoder_created = False

        if self.media_initialized:
            try:
                # Force complete VB release; supported by the v1.4.x API.
                MediaManager.deinit(True)
            finally:
                self.media_initialized = False

        if self.rtsp_started:
            try:
                self.rtsp.rtspserver_stop()
            finally:
                self.rtsp_started = False

        if self.rtsp_initialized:
            try:
                self.rtsp.rtspserver_deinit()
            finally:
                self.rtsp_initialized = False

        self.sensor = None
        self.encoder = None
        self.rtsp = None
        gc.collect()


def main():
    os.exitpoint(os.EXITPOINT_ENABLE)

    # On a cold boot, main.py can run before network/MPP drivers are ready.
    print("Cold-start delay:", BOOT_DELAY_SECONDS, "seconds")
    time.sleep(BOOT_DELAY_SECONDS)
    gc.collect()

    # RT-Smart keeps the network interface active. Do not call active(False).
    start_wifi_ap()
    server = RtspCameraServer()

    try:
        server.initialize()
        print("RTSP server started")
        server.serve_forever()
    except KeyboardInterrupt:
        print("User stopped")
    except BaseException as error:
        import sys
        sys.print_exception(error)
    finally:
        server.close()
        print("Resources released")


if __name__ == "__main__":
    main()
