import numpy as np
import math

class ShelterInfo:
    def __init__(self, track_list) -> None:
        self.map = [np.inf for _ in range(361)]
        self.update_map_tracks(track_list)

    def get_angle_dis(self, box):
        lx, ly, rx, ry = box
        angle_list = [0 for _ in range(4)]
        angle_list[0] = int(math.atan2(ly, lx) / math.pi * 180)
        angle_list[1] = int(math.atan2(ly, rx) / math.pi * 180)
        angle_list[2] = int(math.atan2(ry, lx) / math.pi * 180)
        angle_list[3] = int(math.atan2(ry, rx) / math.pi * 180)
        angle_list = [i+180 for i in angle_list]
        x_len = (lx + rx) / 2
        y_len = (ly + ry) / 2
        dis = math.sqrt(x_len*x_len + y_len*y_len)
        start = min(angle_list)
        end = max(angle_list)
        return start, end, dis

    def update_map(self, box):
        start, end, dis = self.get_angle_dis(box)
        for i in range(start, end+1, 1):
            if self.map[i] > dis:
                self.map[i] = dis
        

    def check_shelter_box(self, box):
        start, end, dis = self.get_angle_dis(box)
        shelter_cnt = 0
        cnt_all = end - start + 1
        for i in range(start, end+1, 1):
            if dis > self.map[i]:
                shelter_cnt += 1
        if shelter_cnt / cnt_all > 0.3:
            return 1
        else:
            return 0

    def check_shelter_track(self, track):
        box = track.box
        return self.check_shelter_box(box)

    def update_map_tracks(self, track_list):
        for track in track_list:
            self.update_map(track.box)


if __name__ == "__main__":
    yxwh = [[-24.93442007,-4.407459868, 4.495186806, 2.207508087], 
    [-16.3515991, 0.095181216, 5.049917698, 2.144861698], 
    [-9.343179462, -0.109833696,4.571141243,2.163946152]]
    boxes = [[i[1]-i[3]/2, i[0]-i[2]/2, i[1]+i[3]/2, i[0]+i[2]/2] for i in yxwh]
    shelter = ShelterInfo([])
    for box in boxes:
        shelter.update_map(box)

    print(shelter.check_shelter_box(boxes[0]))