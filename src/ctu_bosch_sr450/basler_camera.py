#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2025-10-02
import numpy as np
from pypylon import pylon


class BaslerCamera:
    """A class to interface with a Basler camera.

    This class provides a simple interface to connect to, configure, and capture
    images from a Basler camera.

    Attributes:
        camera: The pylon.InstantCamera object.
        converter: The pylon.ImageFormatConverter object.
        connected: A boolean indicating if the camera is connected.
        opened: A boolean indicating if the camera connection is open.
    """

    def __init__(self):
        """Initializes the BaslerCamera instance."""
        self.camera: pylon.InstantCamera | None = None
        self.converter: pylon.ImageFormatConverter | None = None
        self.connected: bool = False
        self.opened: bool = False

    def __del__(self):
        """Destructor to ensure the camera is closed upon object deletion."""
        self.close()

    def connect_by_name(self, name: str):
        """Connects to a camera by its user-defined name.

        Args:
            name: The user-defined name of the camera to connect to.

        Raises:
            pylon.RuntimeException: If no camera with the specified name is found,
                or if no Basler cameras are detected.
            TypeError: If the camera name is not provided.
        """
        if not name:
            raise TypeError("Camera name must be provided.")

        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        if not devices:
            raise pylon.RuntimeException("No Basler cameras detected.")

        for device in devices:
            if device.GetUserDefinedName() == name:
                self.camera = pylon.InstantCamera(tl_factory.CreateDevice(device))
                self.camera.MaxNumBuffer = 5
                self.connected = True
                return

        raise pylon.RuntimeException(f"Camera with name '{name}' not found.")

    def connect_by_ip(self, ip_addr: str):
        """Connects to a camera by its IP address.

        Args:
            ip_addr: The IP address of the camera to connect to.

        Raises:
            pylon.RuntimeException: If no camera with the specified IP address is
                found, or if no Basler cameras are detected.
            TypeError: If the camera IP address is not provided.
        """
        if not ip_addr:
            raise TypeError("Camera IP address must be provided.")

        tl_factory = pylon.TlFactory.GetInstance()
        devices = tl_factory.EnumerateDevices()
        if not devices:
            raise pylon.RuntimeException("No Basler cameras detected.")

        for device in devices:
            if device.GetIpAddress() == ip_addr:
                self.camera = pylon.InstantCamera(tl_factory.CreateDevice(device))
                self.camera.MaxNumBuffer = 5
                self.connected = True
                return

        raise pylon.RuntimeException(f"Camera with IP address '{ip_addr}' not found.")

    def open(self):
        """Opens the connection to the camera.

        This method must be called after a successful connection has been made.
        It also initializes the image format converter to output BGR8 images.
        """
        if not self.connected:
            raise pylon.RuntimeException("Camera not connected.")
        self.camera.Open()
        self.opened = True
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def set_parameters(
        self, exposure_time: float = 0.0, gain: int = 0, gamma: float = 1.0
    ):
        """Sets the camera parameters.

        Args:
            exposure_time: The exposure time in microseconds. If 0,
                auto-exposure ("Continuous") is used. Otherwise, exposure is set
                manually.
            gain: The camera gain (raw value). Auto-gain is always turned off.
            gamma: The gamma correction value.
        """
        if not self.opened:
            return

        if exposure_time > 0:
            self.camera.ExposureAuto.SetValue("Off")
            self.camera.ExposureTime.SetValue(exposure_time)
        else:
            self.camera.ExposureAuto.SetValue("Continuous")

        self.camera.GainAuto.SetValue("Off")
        self.camera.GainRaw.SetValue(gain)

        self.camera.Gamma.SetValue(gamma)

    def grab_image(self, timeout: int = 10000) -> None | np.ndarray:
        """Grabs a single image from the camera.

        Args:
            timeout: The timeout in milliseconds for the grab operation.

        Returns:
            The captured image as a numpy array in BGR8 format, or None if the
            grab fails.
        """
        if not self.camera.IsGrabbing():
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        try:
            res = self.camera.RetrieveResult(
                timeout, pylon.TimeoutHandling_ThrowException
            )
            if res.GrabSucceeded():
                image = self.converter.Convert(res)
                return image.GetArray()
            else:
                return None
        finally:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()

    def close(self):
        """Closes the connection to the camera.

        If the camera is grabbing, it is stopped. If the camera is open, it is
        closed.
        """
        if self.opened:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            self.camera.Close()
            self.opened = False
