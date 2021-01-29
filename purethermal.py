import sys
import time
from ctypes import CFUNCTYPE, POINTER, byref, c_uint16, c_void_p, cast
from queue import Queue

import cv2
import numpy as np

from libuvc_wrapper import (PT_USB_PID, PT_USB_VID,  # print_device_formats,
                            UVC_FRAME_FORMAT_Y16, VS_FMT_GUID_Y16,
                            get_device_info, libuvc, uvc_context, uvc_device,
                            uvc_device_handle, uvc_frame,
                            uvc_get_frame_formats_by_guid, uvc_stream_ctrl)


class PureThermal:
    def __init__(self):
        self._q = Queue(2)

        self._ctx = ctx = POINTER(uvc_context)()
        self._dev = dev = POINTER(uvc_device)()
        self._devh = devh = POINTER(uvc_device_handle)()

        dev, devh, ctx, q = self._dev, self._devh, self._ctx, self._q
        self._ctrl = ctrl = uvc_stream_ctrl()

        self._uvc_init(ctx)
        self._find_device(ctx, dev)
        self._open_device(dev, devh)

        # print_device_formats(devh)
        self._serial_n = get_device_info(devh)  # lepton 3.5 has S/N: 500-0771-01

        frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
        self._check_frame_formats(frame_formats)

        libuvc.uvc_get_stream_ctrl_format_size(
            devh,
            byref(ctrl),
            UVC_FRAME_FORMAT_Y16,
            frame_formats[0].wWidth,
            frame_formats[0].wHeight,
            int(1e7 / frame_formats[0].dwDefaultFrameInterval),
        )

        self._ptr_frame_callback = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(
            self._frame_callback
        )

        self._start_streaming()

    def read(self):
        arr = self._q.get(True, 500)
        return arr

    def close(self):
        libuvc.uvc_stop_streaming(self._devh)
        libuvc.uvc_unref_device(self._dev)
        libuvc.uvc_exit(self._ctx)

    @property
    def SN(self):
        return self._serial_n

    def _frame_callback(self, frame, userptr):
        array_pointer = cast(
            frame.contents.data,
            POINTER(c_uint16 * (frame.contents.width * frame.contents.height)),
        )
        data = np.frombuffer(array_pointer.contents, dtype=np.dtype(np.uint16))
        data = data.reshape(frame.contents.height, frame.contents.width)

        assert frame.contents.data_bytes == (
            2 * frame.contents.width * frame.contents.height
        )

        if not self._q.full():
            self._q.put(data)

    def _start_streaming(self):
        res = libuvc.uvc_start_streaming(
            self._devh, byref(self._ctrl), self._ptr_frame_callback, None, 0
        )
        if res < 0:
            print("uvc_start_streaming failed: {0}".format(res))
            exit(1)

    @staticmethod
    def _uvc_init(ctx):
        res = libuvc.uvc_init(byref(ctx), 0)
        if res < 0:
            print("uvc_init error")
            exit(res)

    @staticmethod
    def _find_device(ctx, dev):
        res = libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0)
        if res < 0:
            print("uvc_find_device error")
            exit(res)

    @staticmethod
    def _open_device(dev, devh):
        res = libuvc.uvc_open(dev, byref(devh))
        if res < 0:
            print(f"uvc_open error {res}")
            exit(res)
        else:
            pass
        # print("device opened!")

    @staticmethod
    def _check_frame_formats(frame_formats):
        if len(frame_formats) == 0:
            print("device does not support Y16")
            exit(1)


if __name__ == "__main__":

    # example usage
    cam = PureThermal()
    print(f"FLIR device serial #: {cam.SN}")
    try:
        while True:
            a = cam.read()
            print(a.shape)
    finally:
        cam.close()