import threading

class DataBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.Motor_RPM = []
        self.Wheel1_RPM = []
        self.Wheel2_RPM = []
        self.Wheel3_RPM = []
        self.Wheel4_RPM = []
        self.Theta = []
        self.Drifting = []
        self.Send_Timestamp = []
        self.Receive_Timestamp = []
        self.Cumulative_Latency = []
