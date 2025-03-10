import os
import json
import ctypes
import socket
import base64
import argparse
import pandas as pd
from tqdm import tqdm

from util import binTools as bt
from protocol.sta775s_intel.sta775sProtocol import Sta775sFrame
# from protocol.sta775s_intel_x86 import EthCombinePack_Str_ACC
from protocol.sta775s_changan.sta775s_changan_x64 import RdrObjListType_Replay_Fusion
class CheckDLL():
    def __init__(self, dll_x64_fp=r'./dll/fradar.dll'):
        self.track_check_flag = False
        self.fs_check_flag = True
        self.max_length = 130000
        self.port = 12345
        self.client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.host = socket.gethostname()
        self.client.connect((self.host, self.port))
        self.dll_x64 = ctypes.WinDLL(dll_x64_fp)
        self.track_match_pd = pd.DataFrame(columns=('frame','x64_ID', 'x64_age', 'x64_Heading', 'x64_ObjType', 'x64_obstacle',
                                                    'x64_rx', 'x64_ry', 'x64_RelVelX', 'x64_RelVelY', 'x64_AbsVelX', 'x64_AbsVelY', 
                                                                        'x64_RelAccX', 'x64_RelAccY', 'x64_AbsAccX', 'x64_AbsAccY',
                                                    'x64_IsStationary', 'x64_width', 'x64_length', 'x64_height',
                                                    'x86_ID', 'x86_age', 'x86_Heading', 'x86_ObjType', 'x86_obstacle',
                                                    'x86_rx', 'x86_ry', 'x86_RelVelX', 'x86_RelVelY', 'x86_AbsVelX', 'x86_AbsVelY', 
                                                                        'x86_RelAccX', 'x86_RelAccY', 'x86_AbsAccX', 'x86_AbsAccY',
                                                    'x86_IsStationary', 'x86_width', 'x86_length', 'x86_height',
                                                    'age', 'Yaw', 'TarType', 'obstacle', 'rx', 'ry', 'vx_rel', 'vy_rel', 'vx_abs', 'vy_abs', 'ax_rel', 'ay_rel', 'ax_abs', 'ay_abs', 'moveState', 'box_width', 'box_length', 'height', 'flag_sum'))
        self.track_unmatch_pd = pd.DataFrame(columns=('frame','x64_ID', 'x64_age', 'x64_Heading', 'x64_ObjType', 'x64_obstacle',
                                                    'x64_rx', 'x64_ry', 'x64_RelVelX', 'x64_RelVelY', 'x64_AbsVelX', 'x64_AbsVelY', 
                                                                        'x64_RelAccX', 'x64_RelAccY', 'x64_AbsAccX', 'x64_AbsAccY',
                                                    'x64_IsStationary', 'x64_width', 'x64_length', 'x64_height'))
        self.fs_match_pd = pd.DataFrame(columns=('frame','index','x64_range','x86_range'))
        self.fs_unmatch_pd = pd.DataFrame(columns=('frame','index','x64_range','x86_range'))
        self.fs_match_list = []
        self.fs_unmatch_list = []
        self.track_match_list = []
        self.track_unmatch_list = []
    
    def check(self, data_fp=r'data\FRADAR_20240730-190513_075_0.bin'):
        with open(data_fp, 'rb') as f:
            totalFrameHeader = f.read(2)
            totleVersion = f.read(1)
            totalDataLength = f.read(2)
            frame_length = bt.unpackUint16(totalDataLength)
            # f.seek(3024 * frame_length)
            f.seek(0)
            data = f.read(frame_length)
            # cal frame num: file total length / frame length
            frame_num = int(os.path.getsize(data_fp) / frame_length)
            name = b'test'
            # frame_num = 0
            for i in tqdm(range(frame_num)):
                bytes_data = f.read(frame_length)
                if not data:
                    break
                data_dict = {
                    "state": "msg",
                    "name" : "test",
                    "max_length" : 130000,
                    "input_data": base64.b64encode(bytes_data).decode('utf-8')
                }
                Message = json.dumps(data_dict, ensure_ascii=False)
                # decoded_ = base64.b64decode(data_dict["input_data"])
                # print(len(Message))
                self.client.send(Message.encode('utf-8'))
                res_x86 = self.client.recv(131000)
                res_x64 = self.run64(bytes_data, frame_length, name)
                # print(len(res_x86))
                
                # 解析x86数据
                data_sparse_x86 = Sta775sFrame(with_fs=True, with_points=True)
                data_sparse_x86.decodeProtocol(res_x86)
                # print(len(data_sparse_x86))
                
                # 解析x64数据
                data_sparse_x64 = RdrObjListType_Replay_Fusion.from_buffer_copy(res_x64)
                
                # 计算x64和x86时间戳的差(ms) 差值在100ms左右
                # x86_time = (data_sparse_x86.monitorProtocol['syncTime'] - data_sparse_x86.monitorProtocol['od_latency']*100)/(1000*1000)
                # print(f'frame:{data_sparse_x86.baseProtocol['frameCount']}')
                # print(f'x64_time:{data_sparse_x64.SensorStatusFlags_st.current_radar_time}')
                # print(f'x86_time:{x86_time}')
                # print(f'diff_time:{data_sparse_x64.SensorStatusFlags_st.current_radar_time - x86_time}')
                if(data_sparse_x64.SensorStatusFlags_st.RadarObj_num_u32 == 0):
                    # print(f'track num=0')
                    continue
                # else:
                #     print(f'frame:{data_sparse_x86.baseProtocol['frameCount']}, track num={data_sparse_x64.SensorStatusFlags_st.RadarObj_num_u32}.............................................')
                if self.track_check_flag:
                    # 校验track数据并保存校验结果 
                    self.track_check(data_sparse_x64, data_sparse_x86)
                if self.fs_check_flag:
                    # 校验fs
                    self.fs_check(data_sparse_x64, data_sparse_x86)

            # 保存csv
            if self.track_check_flag:
                self.track_match_pd = pd.concat([self.track_match_pd, pd.DataFrame(self.track_match_list)])
                self.track_match_pd['diff_Yaw'] = (self.track_match_pd['x64_Heading'] - self.track_match_pd['x86_Heading']).abs()
                self.track_match_pd['diff_obstacle'] = (self.track_match_pd['x64_obstacle'] - self.track_match_pd['x86_obstacle']).abs()
                self.track_match_pd['diff_rx'] = (self.track_match_pd['x64_rx'] - self.track_match_pd['x86_rx']).abs()
                self.track_match_pd['diff_ry'] = (self.track_match_pd['x64_ry'] - self.track_match_pd['x86_ry']).abs()
                self.track_match_pd['diff_vx_rel'] = (self.track_match_pd['x64_RelVelX'] - self.track_match_pd['x86_RelVelX']).abs()
                self.track_match_pd['diff_vy_rel'] = (self.track_match_pd['x64_RelVelY'] - self.track_match_pd['x86_RelVelY']).abs()
                self.track_match_pd['diff_vx_abs'] = (self.track_match_pd['x64_AbsVelX'] - self.track_match_pd['x86_AbsVelX']).abs()
                self.track_match_pd['diff_vy_abs'] = (self.track_match_pd['x64_AbsVelY'] - self.track_match_pd['x86_AbsVelY']).abs()
                self.track_match_pd['diff_ax_rel'] = (self.track_match_pd['x64_RelAccX'] - self.track_match_pd['x86_RelAccX']).abs()
                self.track_match_pd['diff_ay_rel'] = (self.track_match_pd['x64_RelAccY'] - self.track_match_pd['x86_RelAccY']).abs()
                self.track_match_pd['diff_ax_abs'] = (self.track_match_pd['x64_AbsAccX'] - self.track_match_pd['x86_AbsAccX']).abs()
                self.track_match_pd['diff_ay_abs'] = (self.track_match_pd['x64_AbsAccY'] - self.track_match_pd['x86_AbsAccY']).abs()
                self.track_match_pd['diff_width'] = (self.track_match_pd['x64_width'] - self.track_match_pd['x86_width']).abs()
                self.track_match_pd['diff_length'] = (self.track_match_pd['x64_length'] - self.track_match_pd['x86_length']).abs()
                self.track_match_pd['diff_height'] = (self.track_match_pd['x64_height'] - self.track_match_pd['x86_height']).abs()
                self.track_unmatch_pd = pd.concat([self.track_unmatch_pd, pd.DataFrame(self.track_unmatch_list)])
                self.track_match_pd.to_csv(data_fp.split('.')[-2] + '_match_track.csv', index=False)
                self.track_unmatch_pd.to_csv(data_fp.split('.')[-2] + '_unmatch_track.csv', index=False)
            if self.fs_check_flag:
                self.fs_match_pd = pd.concat([self.fs_match_pd, pd.DataFrame(self.fs_match_list)])
                self.fs_unmatch_pd = pd.concat([self.fs_unmatch_pd, pd.DataFrame(self.fs_unmatch_list)])
                self.fs_match_pd.to_csv(data_fp.split('.')[-2] + '_match_fs.csv', index=False)
                self.fs_unmatch_pd.to_csv(data_fp.split('.')[-2] + '_unmatch_fs.csv', index=False)
            # bin文件读取结束,关闭socket通信.
            data_dict = {
                "state": "off",
                "name" : "test",
                "max_length" : 130000,
                "input_data": 'None'
            }
            Message = json.dumps(data_dict, ensure_ascii=False)
            # print(len(Message))
            self.client.send(Message.encode('utf-8'))
            close_msg = self.client.recv(131000)
            print(close_msg.decode('utf-8'))


    def run64(self, input_data, input_length, name=b'test'):
        input_data = (ctypes.c_uint8 * self.max_length)(*input_data)
        output_data = (ctypes.c_uint8 * self.max_length)()
        # if name is a string, it will be converted to bytes
        if isinstance(name, str):
            name = name.encode('utf-8')
        tmp = self.dll_x64.fradar(input_data, input_length, output_data, self.max_length, name)
        if tmp > 0:
            return ctypes.string_at(output_data, tmp)
        else:
            raise Exception("error in dll")
        
    def track_check(self, data_sparse_x64, data_sparse_x86):
        # 校验目标(changan_x64最多32个目标所以要让x64去遍历x86)
        for i in range(data_sparse_x64.SensorStatusFlags_st.RadarObj_num_u32):
            match_list=[]
            for (j,track) in enumerate(data_sparse_x86.trackProtocol['trackList_intel']):
                flag_dict = {
                    'age': 0,
                    'Yaw':0,
                    'TarType':0,
                    'obstacle':0,
                    'rx':0,
                    'ry':0,
                    'vx_rel':0,
                    'vy_rel':0,
                    'vx_abs':0,
                    'vy_abs':0,
                    'ax_rel':0,
                    'ay_rel':0,
                    'ax_abs':0,
                    'ay_abs':0,
                    'moveState':0,
                    'box_width':0,
                    'box_length':0,
                    'height':0,
                }
                diff_dict = {
                    'diff_Yaw':0,
                    'diff_obstacle':0,
                    'diff_rx':0,
                    'diff_ry':0,
                    'diff_vx_rel':0,
                    'diff_vy_rel':0,
                    'diff_vx_abs':0,
                    'diff_vy_abs':0,
                    'diff_ax_rel':0,
                    'diff_ay_rel':0,
                    'diff_ax_abs':0,
                    'diff_ay_abs':0,
                    'diff_width':0,
                    'diff_length':0,
                    'diff_height':0,
                }
                # rx,ry(m)
                if abs(data_sparse_x64.SensorObject_st[i].PosX_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['Rx'] / 100) < 0.01:
                    flag_dict['rx'] = 1
                if abs(data_sparse_x64.SensorObject_st[i].PosY_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['Ry'] / 100) < 0.01:
                    flag_dict['ry'] = 1
                if(flag_dict['rx'] + flag_dict['ry'] == 2):
                    # 'age', 
                    if data_sparse_x64.SensorObject_st[i].Age_u32 == data_sparse_x86.trackProtocol['trackList_intel'][j]['lifeTime']:
                        flag_dict['age'] = 1
                    # 航向角Yaw(rad)
                    if(abs(data_sparse_x64.SensorObject_st[i].Heading_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['Yaw'] / 100 * 0.017453293) < 0.01):
                        flag_dict['Yaw'] = 1
                    # 目标类型TarType
                    if((data_sparse_x64.SensorObject_st[i].ObjType_u8 == 1 and track['TarType']== 5)
                       or (data_sparse_x64.SensorObject_st[i].ObjType_u8 == 2 and track['TarType']== 4)
                       or (data_sparse_x64.SensorObject_st[i].ObjType_u8 == 3 and track['TarType']== 6)
                       or (data_sparse_x64.SensorObject_st[i].ObjType_u8 == 4 and track['TarType']== 7)
                       or (data_sparse_x64.SensorObject_st[i].ObjType_u8 == 7 and (track['TarType']== 1 or track['TarType']== 2 or track['TarType']== 8))):
                        flag_dict['TarType'] = 1
                    # 置信度obstacle(0-100)
                    if(data_sparse_x64.SensorObject_st[i].ObstacleProb_f32 == data_sparse_x86.trackProtocol['trackList_intel'][j]['obstacle']):
                        flag_dict['obstacle'] = 1
                    # Rel vx,vy(m/s)
                    if abs(data_sparse_x64.SensorObject_st[i].RelVelX_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['longitu_speed'] / 100) < 0.02:
                        flag_dict['vx_rel'] = 1
                    if abs(data_sparse_x64.SensorObject_st[i].RelVelY_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['cross_speed'] / 100) < 0.02:
                        flag_dict['vy_rel'] = 1
                    # Abs vx.vy(m/s)
                    if abs(data_sparse_x64.SensorObject_st._b_base_.SensorObject_st[i].AbsVelX_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['vxAbs'] / 100) < 0.02:
                        flag_dict['vx_abs'] = 1
                    if abs(data_sparse_x64.SensorObject_st._b_base_.SensorObject_st[i].AbsVelY_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['vyAbs'] / 100) < 0.02:
                        flag_dict['vy_abs'] = 1
                    # Rel ax,ay(m/s2)
                    if abs(data_sparse_x64.SensorObject_st[i].RelAccX_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['Aceler_X'] / 100) < 0.02:
                        flag_dict['ax_rel'] = 1
                    if abs(data_sparse_x64.SensorObject_st[i].RelAccY_f32 - data_sparse_x86.trackProtocol['trackList_intel'][j]['Aceler_Y'] / 100) < 0.02:
                        flag_dict['ay_rel'] = 1
                    # Abs ax,ay(m/s2)
                    if abs(data_sparse_x64.SensorObject_st[i].AbsAccX_f32 - (data_sparse_x86.trackProtocol['trackList_intel'][j]['Aceler_X'] / 100 + data_sparse_x86.baseProtocol['DSP_Acceleration_X'])) < 0.02:
                        flag_dict['ax_abs'] = 1
                    if abs(data_sparse_x64.SensorObject_st[i].AbsAccY_f32 - (data_sparse_x86.trackProtocol['trackList_intel'][j]['Aceler_Y'] / 100 + data_sparse_x86.baseProtocol['DSP_Acceleration_Y'])) < 0.02:
                        flag_dict['ay_abs'] = 1
                    # 运动状态
                    if((data_sparse_x64.SensorObject_st[i].IsStationary_u8 == 1 and (track['moveState'] == 1 or track['moveState'] == 2 or track['moveState'] == 4))
                       or (data_sparse_x64.SensorObject_st[i].IsStationary_u8 == 2 and track['moveState'] == 3)
                       or (data_sparse_x64.SensorObject_st[i].IsStationary_u8 == 3 and track['moveState'] == 5)
                       or (data_sparse_x64.SensorObject_st[i].IsStationary_u8 == 4 and track['moveState'] == 0)):
                        flag_dict['moveState'] = 1
                    # box(m)
                    if (track['moveState'] == 5 or track['moveState'] == 6):
                        # 静止目标box都是0
                        if data_sparse_x64.SensorObject_st[i].Width_f32 == 0:
                            flag_dict['box_width'] = 1
                        if data_sparse_x64.SensorObject_st[i].Lenth_f32 == 0:
                            flag_dict['box_length'] = 1
                        if data_sparse_x64.SensorObject_st[i].Height_f32 == 0:
                            flag_dict['height'] = 1
                    else:
                        if abs(data_sparse_x64.SensorObject_st[i].Width_f32 - track['box_width'] / 100) < 0.1:
                            flag_dict['box_width'] = 1
                        if abs(data_sparse_x64.SensorObject_st[i].Lenth_f32 - track['box_length'] / 100) < 0.1:
                            flag_dict['box_length'] = 1
                        if abs(data_sparse_x64.SensorObject_st[i].Height_f32 - track['height'] / 100) < 0.1:
                            flag_dict['height'] = 1
                   
                    if flag_dict['rx']==1 and flag_dict['ry']==1 and flag_dict['TarType'] and flag_dict['moveState']:
                        match_list.append([i,j,data_sparse_x64.SensorObject_st._b_base_.SensorObject_st[i].ID_u8,data_sparse_x86.trackProtocol['trackList_intel'][j]['index']])
                        flag_sum = 0
                        for v in flag_dict.values():
                            flag_sum += v
                        self.track_match_list.append(
                            {
                                'frame': data_sparse_x86.baseProtocol['frameCount'],
                                'x64_ID': data_sparse_x64.SensorObject_st[i].ID_u8,
                                'x64_age': data_sparse_x64.SensorObject_st[i].Age_u32,
                                'x64_Heading': data_sparse_x64.SensorObject_st[i].Heading_f32,
                                'x64_ObjType': data_sparse_x64.SensorObject_st[i].ObjType_u8,
                                'x64_obstacle': data_sparse_x64.SensorObject_st[i].ObstacleProb_f32,
                                'x64_rx': data_sparse_x64.SensorObject_st[i].PosX_f32,
                                'x64_ry': data_sparse_x64.SensorObject_st[i].PosY_f32,
                                'x64_RelVelX': data_sparse_x64.SensorObject_st[i].RelVelX_f32,
                                'x64_RelVelY': data_sparse_x64.SensorObject_st[i].RelVelY_f32,
                                'x64_AbsVelX': data_sparse_x64.SensorObject_st[i].AbsVelX_f32,
                                'x64_AbsVelY': data_sparse_x64.SensorObject_st[i].AbsVelY_f32,
                                'x64_RelAccX': data_sparse_x64.SensorObject_st[i].RelAccX_f32,
                                'x64_RelAccY': data_sparse_x64.SensorObject_st[i].RelAccY_f32,
                                'x64_AbsAccX': data_sparse_x64.SensorObject_st[i].AbsAccX_f32,
                                'x64_AbsAccY': data_sparse_x64.SensorObject_st[i].AbsAccY_f32,
                                'x64_IsStationary': data_sparse_x64.SensorObject_st[i].IsStationary_u8,
                                'x64_width': data_sparse_x64.SensorObject_st[i].Width_f32,
                                'x64_length': data_sparse_x64.SensorObject_st[i].Lenth_f32,
                                'x64_height': data_sparse_x64.SensorObject_st[i].Height_f32,
                                'x86_ID': data_sparse_x86.trackProtocol['trackList_intel'][j]['index'],
                                'x86_age': data_sparse_x86.trackProtocol['trackList_intel'][j]['lifeTime'],
                                'x86_Heading': data_sparse_x86.trackProtocol['trackList_intel'][j]['Yaw'] / 100 * 0.017453293,
                                'x86_ObjType': data_sparse_x86.trackProtocol['trackList_intel'][j]['TarType'],
                                'x86_obstacle': data_sparse_x86.trackProtocol['trackList_intel'][j]['obstacle'],
                                'x86_rx': data_sparse_x86.trackProtocol['trackList_intel'][j]['Rx'] / 100,
                                'x86_ry': data_sparse_x86.trackProtocol['trackList_intel'][j]['Ry'] / 100,
                                'x86_RelVelX': data_sparse_x86.trackProtocol['trackList_intel'][j]['longitu_speed'] / 100,
                                'x86_RelVelY': data_sparse_x86.trackProtocol['trackList_intel'][j]['cross_speed'] / 100,
                                'x86_AbsVelX': data_sparse_x86.trackProtocol['trackList_intel'][j]['vxAbs'] /100,
                                'x86_AbsVelY': data_sparse_x86.trackProtocol['trackList_intel'][j]['vyAbs'] /100,
                                'x86_RelAccX': data_sparse_x86.trackProtocol['trackList_intel'][j]['Aceler_X'] / 100,
                                'x86_RelAccY': data_sparse_x86.trackProtocol['trackList_intel'][j]['Aceler_Y'] / 100,
                                'x86_AbsAccX': data_sparse_x86.trackProtocol['trackList_intel'][j]['Aceler_X'] / 100 + data_sparse_x86.baseProtocol['DSP_Acceleration_X'],
                                'x86_AbsAccY': data_sparse_x86.trackProtocol['trackList_intel'][j]['Aceler_Y'] / 100 + data_sparse_x86.baseProtocol['DSP_Acceleration_Y'],
                                'x86_IsStationary': data_sparse_x86.trackProtocol['trackList_intel'][j]['moveState'],
                                'x86_width': data_sparse_x86.trackProtocol['trackList_intel'][j]['box_width'] / 100,
                                'x86_length': data_sparse_x86.trackProtocol['trackList_intel'][j]['box_length'] / 100,
                                'x86_height': data_sparse_x86.trackProtocol['trackList_intel'][j]['height'] / 100,
                                'age': flag_dict['age'],
                                'Yaw': flag_dict['Yaw'],
                                'TarType': flag_dict['TarType'],
                                'obstacle': flag_dict['obstacle'],
                                'rx': flag_dict['rx'],
                                'ry': flag_dict['ry'],
                                'vx_rel': flag_dict['vx_rel'],
                                'vy_rel': flag_dict['vy_rel'],
                                'vx_abs': flag_dict['vx_abs'],
                                'vy_abs': flag_dict['vy_abs'],
                                'ax_rel': flag_dict['ax_rel'],
                                'ay_rel': flag_dict['ay_rel'],
                                'ax_abs': flag_dict['ax_abs'],
                                'ay_abs': flag_dict['ay_abs'],
                                'moveState': flag_dict['moveState'],
                                'box_width': flag_dict['box_width'],
                                'box_length': flag_dict['box_length'],
                                'height': flag_dict['height'],
                                'flag_sum': flag_sum
                            })
                        

                    else:
                        self.track_unmatch_list.append(
                            {
                                'frame': data_sparse_x86.baseProtocol['frameCount'],
                                'x64_ID': data_sparse_x64.SensorObject_st[i].ID_u8,
                                'x64_age': data_sparse_x64.SensorObject_st[i].Age_u32,
                                'x64_Heading': data_sparse_x64.SensorObject_st[i].Heading_f32,
                                'x64_ObjType': data_sparse_x64.SensorObject_st[i].ObjType_u8,
                                'x64_obstacle': data_sparse_x64.SensorObject_st[i].ObstacleProb_f32,
                                'x64_rx': data_sparse_x64.SensorObject_st[i].PosX_f32,
                                'x64_ry': data_sparse_x64.SensorObject_st[i].PosY_f32,
                                'x64_RelVelX': data_sparse_x64.SensorObject_st[i].RelVelX_f32,
                                'x64_RelVelY': data_sparse_x64.SensorObject_st[i].RelVelY_f32,
                                'x64_AbsVelX': data_sparse_x64.SensorObject_st[i].AbsVelX_f32,
                                'x64_AbsVelY': data_sparse_x64.SensorObject_st[i].AbsVelY_f32,
                                'x64_RelAccX': data_sparse_x64.SensorObject_st[i].RelAccX_f32,
                                'x64_RelAccY': data_sparse_x64.SensorObject_st[i].RelAccY_f32,
                                'x64_AbsAccX': data_sparse_x64.SensorObject_st[i].AbsAccX_f32,
                                'x64_AbsAccY': data_sparse_x64.SensorObject_st[i].AbsAccY_f32,
                                'x64_IsStationary': data_sparse_x64.SensorObject_st[i].IsStationary_u8,
                                'x64_width': data_sparse_x64.SensorObject_st[i].Width_f32,
                                'x64_length': data_sparse_x64.SensorObject_st[i].Lenth_f32,
                                'x64_height': data_sparse_x64.SensorObject_st[i].Height_f32
                            })
            if not match_list:
                print(f'frame: {data_sparse_x86.baseProtocol['frameCount']}, x64_index:{i}, fs not match............')
            # print(f'frame:{data_sparse_x86.baseProtocol['frameCount']}, current_radar_time:{data_sparse_x64.SensorStatusFlags_st.current_radar_time}.............................................')
    
    def fs_check(self, data_sparse_x64, data_sparse_x86):
        for i in range(360):
            if 1:
            # if data_sparse_x64.RadarFs.RadarFsType_Replay_st[i].FsConfidence_u8 != 0:
                if (abs(data_sparse_x64.RadarFs.RadarFsType_Replay_st[i].Range_f32 - data_sparse_x86.fsProtocol['fsList'][i][0] * 0.1) < 0.5):
                    self.fs_match_list.append({
                        'frame': data_sparse_x86.baseProtocol['frameCount'],
                        'index': i,
                        'x64_range': data_sparse_x64.RadarFs.RadarFsType_Replay_st[i].Range_f32,
                        'x64_confidence': data_sparse_x64.RadarFs.RadarFsType_Replay_st[i].FsConfidence_u8,
                        'x86_range': data_sparse_x86.fsProtocol['fsList'][i][0] * 0.01,
                        'x86_confidence': data_sparse_x86.fsProtocol['fsList'][i][1],
                    })
                else:
                    self.fs_unmatch_list.append({
                        'frame': data_sparse_x86.baseProtocol['frameCount'],
                        'index': i,
                        'x64_range': data_sparse_x64.RadarFs.RadarFsType_Replay_st[i].Range_f32,
                        'x64_confidence': data_sparse_x64.RadarFs.RadarFsType_Replay_st[i].FsConfidence_u8,
                        'x86_range': data_sparse_x86.fsProtocol['fsList'][i][0] * 0.1,
                        'x86_confidence': data_sparse_x86.fsProtocol['fsList'][i][1],
                    })
                    print(f'frame: {data_sparse_x86.baseProtocol['frameCount']}, x64_index:{i}, fs not match............')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dll64','-d', type=str, default=r'./dll/fradar.dll')
    parser.add_argument('--path', '-p', type=str, default=r'data\DTM20250217000263\FRADAR_20240716-172427_482_0.bin')
    args = parser.parse_args()
    dll_exp = CheckDLL(dll_x64_fp=args.dll64)
    dll_exp.check(data_fp=args.path)

if __name__ == '__main__':
    main()
    print('check end...')