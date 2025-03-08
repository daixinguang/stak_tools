import math

class CarPose:
    def __init__(self, lo=0, la=0, z=0, yaw=0, roll=0, pitch=0, speed=0) -> None:
        self.lo = lo
        self.la = la
        self.z  = z
        self.yaw = yaw
        self.roll = roll
        self.pitch = pitch
        self.speed = speed
        if speed or la or lo:
            self.available = True
        else:
            self.available = False

    def is_move(self):
        if abs(self.speed) < 0.01:
            return False
        else:
            return True

    def is_available(self):
        return self.available

    def fix_time_diff(self, delta_time):
        self.la += self.speed * math.sin(-self.yaw) * delta_time
        self.lo += self.speed * math.cos(-self.yaw) * delta_time

    # 获取位置信息：纵向位置、横向位置、yaw角
    def get_pos(self):
        return self.lo, self.la, self.z, self.roll, self.pitch, self.yaw
