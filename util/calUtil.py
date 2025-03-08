'''
Author:  
Date: 2022-05-14 13:07:01
Description: 通用计算模块
'''
import numpy as np
import math
import time

# 计算iou，先暂时用这个实现，后续如果遇到性能瓶颈考虑用cpp或者其他算法代替
def cal_iou(box1, box2):
    # box format: [x1, y1, x2, y2]
    xa = max(box1[0], box2[0])
    ya = max(box1[1], box2[1])
    xb = min(box1[2], box2[2])
    yb = min(box1[3], box2[3])

    interArea = (xb-xa) * (yb-ya)

    if (xa >= xb or ya >= yb):
        return 0

    box1Area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2Area = (box2[2] - box2[0]) * (box2[3] - box2[1])

    iou = interArea / (box1Area + box2Area - interArea)
    return iou

def cal_iou_xylw(box1, box2):
    # box format: [center_x, center_y, l, w]
    x1 = box1[0] - box1[2] / 2
    y1 = box1[1] - box1[3] / 2
    x2 = box1[0] + box1[2] / 2
    y2 = box1[1] + box1[3] / 2
    box1 = [x1, y1, x2, y2]
    x1 = box2[0] - box2[2] / 2
    y1 = box2[1] - box2[3] / 2
    x2 = box2[0] + box2[2] / 2
    y2 = box2[1] + box2[3] / 2
    box2 = [x1, y1, x2, y2]
    return cal_iou(box1, box2)

def cal_area(box):
    width = box[2] - box[0]
    height = box[3] - box[1]
    return width * height

# 计算两个框之间的欧氏距离
def cal_box_distance(box1, box2, type='center'):
    x1 = (box1[0] + box1[2]) * 0.5
    x2 = (box2[0] + box2[2]) * 0.5
    if type == 'top':
        y1 = box1[1]
        y2 = box2[1]
    elif type == 'bottom':
        y1 = box1[3]
        y2 = box2[3]
    else:
        y1_t = box1[1]
        y2_t = box2[1]
        y1_b = box1[3]
        y2_b = box2[3]
        y1_c = (box1[1] + box1[3]) * 0.5
        y2_c = (box2[1] + box2[3]) * 0.5
        dlt_list = [y2_t - y1_t, y2_c - y1_c, y2_b - y1_b]
        dlt_list = [abs(x) for x in dlt_list]
        min_index = dlt_list.index(min(dlt_list))
        if min_index == 0:
            y1 = y1_t
            y2 = y2_t
        elif min_index == 1:
            y1 = y1_c
            y2 = y2_c
        else:
            y1 = y1_b
            y2 = y2_b
    vector_1 = np.asarray([x1, y1])
    vector_2 = np.asarray([x2, y2])
    return np.sqrt(np.sum(np.square(vector_1 - vector_2))), (vector_1 - vector_2)

def cal_box_distance_square(box1, box2):
    x1 = (box1[0] + box1[2]) * 0.5
    y1 = (box1[1] + box1[3]) * 0.5
    x2 = (box2[0] + box2[2]) * 0.5
    y2 = (box2[1] + box2[3]) * 0.5
    vector_1 = np.asarray([x1, y1])
    vector_2 = np.asarray([x2, y2])
    return np.sum(np.square(vector_1 - vector_2))

def cal_center_distance(x1, y1, x2, y2):
    vector_1 = np.asarray([x1, y1])
    vector_2 = np.asarray([x2, y2])
    return np.sqrt(np.sum(np.square(vector_1 - vector_2)))

# input->str, output->bool_res. If input is illegal return 0.
def decode_bool(input_data):
    if isinstance(input_data, str):
        true_list = ["1", "TRUE", "true", "True"]
        False_list = ["0", "FALSE", "False", "False"]
        if input_data in true_list:
            return True
        else:
            return False
    else:
        return False

def decode_float(input_data):
    if not input_data:
        return 0
    else:
        return float(input_data)
    
def decode_int(input_data):
    if not input_data:
        return 0
    else:
        return int(float(input_data))

# 计算目标位置与雷达所成角度，使用的坐标是原始自车坐标系，转换后的坐标系需要做对应的变换。
def cal_object_angle(lo, la, length, v_lo):
    front_install_lo = 3.75
    establish_time = 0.3
    lo -= front_install_lo
    if v_lo > 0:
        lo += length / 2
        lo -= v_lo * establish_time
    degree = math.atan2(la, lo) / math.pi * 180
    return degree

def time2unixStamp(year, month, day, hour, minute, second, millisecond):
    time_str = "{0}-{1}-{2} {3}:{4}:{5}".format(year, month, day, hour, minute, second)
    if time_str == "2000-0-0 0:0:0":
        return 0
    time_array = time.strptime(time_str, "%Y-%m-%d %H:%M:%S")
    time_stamp = time.mktime(time_array) + (millisecond / 1000)
    return time_stamp

def points_trans_local2global(positions, car_x, car_y, car_z, roll, pitch, yaw):
    tt = np.asarray([[1, 0, 0, 0], 
                    [0, 1, 0, 0], 
                    [0, 0, 1, 0], 
                    [car_x, car_y, car_z, 1]])
    tx = np.asarray([[1, 0, 0, 0], 
                    [0, math.cos(roll), math.sin(roll), 0], 
                    [0, -math.sin(roll), math.cos(roll), 0], 
                    [0, 0, 0, 1]])
    ty = np.asarray([[math.cos(pitch), 0, -math.sin(pitch), 0], 
                    [0, 1, 0, 0], 
                    [math.sin(pitch), 0, math.cos(pitch), 0],
                    [0, 0, 0, 1]])
    tz = np.asarray([[math.cos(yaw), math.sin(yaw), 0, 0], 
                    [-math.sin(yaw), math.cos(yaw), 0, 0], 
                    [0, 0, 1, 0], 
                    [0, 0, 0, 1]])
    T = np.matmul(tx, ty)
    T = np.matmul(T, tz)
    T = np.matmul(T, tt)
    prev = np.asarray([[x, y, z, 1] for x, y, z in positions])
    dest = np.matmul(prev, T)
    res = []
    for d in dest:
        dx = d[0]
        dy = d[1]
        dz = d[2]
        res.append((dx, dy, dz))
    return res

def points_trans_global2local(positions, car_x, car_y, car_z, roll, pitch, yaw):
    tt = np.asarray([[1, 0, 0, 0], 
                    [0, 1, 0, 0], 
                    [0, 0, 1, 0], 
                    [car_x, car_y, car_z, 1]])
    tx = np.asarray([[1, 0, 0, 0], 
                    [0, math.cos(roll), math.sin(roll), 0], 
                    [0, -math.sin(roll), math.cos(roll), 0], 
                    [0, 0, 0, 1]])
    ty = np.asarray([[math.cos(pitch), 0, -math.sin(pitch), 0], 
                    [0, 1, 0, 0], 
                    [math.sin(pitch), 0, math.cos(pitch), 0],
                    [0, 0, 0, 1]])
    tz = np.asarray([[math.cos(yaw), math.sin(yaw), 0, 0], 
                    [-math.sin(yaw), math.cos(yaw), 0, 0], 
                    [0, 0, 1, 0], 
                    [0, 0, 0, 1]])
    T = np.matmul(tx, ty)
    T = np.matmul(T, tz)
    T = np.matmul(T, tt)
    prev = np.asarray([[x, y, z, 1] for x, y, z in positions])
    T = np.linalg.inv(T)
    dest = np.matmul(prev, T)
    res = []
    for d in dest:
        dx = d[0]
        dy = d[1]
        dz = d[2]
        res.append((dx, dy, dz))
    return res

if __name__ == "__main__":
    time2unixStamp(2000, 0, 0, 0, 0, 0, 0)