import sys
import time

# from scooping.object import Ball
from models.object_grounding import Ball
from models.model import EE_manipulator

from ultralytics import YOLO

import signal
def signal_handler(signum, frame):
    raise KeyboardInterrupt

eP1 = [0.000, 0.000, 0.000, 0.000]
dP1 = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]

if __name__=='__main__':
    signal.signal(signal.SIGINT, signal_handler)  # 捕获 Ctrl+C 信号

    start_time = time.strftime("%Y%m%d_%H%M%S")
    ball = Ball()    

    # Load a model
    yolo_model = YOLO("./train2/weights/last.pt")  # pretrained YOLO11n model

    try:
        predictor = EE_manipulator()
        pose = predictor.infer(ball, start_time, yolo_model)
        
        
        
    finally:
        # cam.stop_recording()
        pass


