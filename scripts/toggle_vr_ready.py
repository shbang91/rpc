import numpy as np
import zmq
import time
from output_data_manager import OutputDataManager

context = zmq.Context()
manager = OutputDataManager(context)
for i in range(100):
    manager.toggle_vr_ready()
    time.sleep(.01)
