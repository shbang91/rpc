"""
Project:
    Data collection / Teleoprator

Title:
    Sensor base class

Description
    - base class of the sensor

* Copyrighted by Mingyo Seo
* Bonston Dynamics AI Institute, The University of Texas at Austin
"""

import ctypes
from multiprocessing import Process, Value


class Sensor:
    """
    a receiver's configuration for remote commands
    """

    def __init__(self, name=""):
        self._name = name

        self._flag_proc = Value(
            ctypes.c_bool,
        )
        self._time_proc = Value(
            ctypes.c_float,
        )

        print(f"{self._name} created")

    def _fn_init(self):
        raise NotImplementedError

    def start(self):
        # Start streaming with our callback
        self._flag_proc.value = True
        self._proc = Process(
            target=self._fn_proc,
        )
        self._proc.start()

        self._fn_init()

        print(f"{self._name} start!")

    def stop(self):
        self._flag_proc.value = False
        self._proc.terminate()

        print(f"{self._name} stop!")

    def _fn_proc(self):
        raise NotImplementedError

    @property
    def time(self):
        return self._time_proc.value
