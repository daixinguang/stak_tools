'''
通过同时加载修改前和修改后的dll，回灌验证修改前后对carinfo参数的影响。

775S
\\Hik-sjzyf-fs01\1.sta77-5测试$\STA77-5S\01 平台\10 客户演示\上汽测试用例\03 真实道路测试\以太网数据
775L
\\Hik-sjzyf-fs01\森思泰克\1.部门文件\1.车载雷达产品部\501.车载算法部\001-算法库2.0\07-STA77-5L\02-路测数据\2.新点云协议
777S
\\Hik-sjzyf-fs01\森思泰克\1.部门文件\1.车载雷达产品部\501.车载算法部\001-算法库2.0\06-STA77-7S\数据库\新协议
'''
import os
import json
import ctypes
import base64
import struct
import argparse
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm
class CheckDLL:
    def __init__(self, dllB_fp, dllA_fp, platform='775S'):
        self.dllB = ctypes.WinDLL(dllB_fp)
        self.dllA = ctypes.WinDLL(dllA_fp)
        self.max_length = 130000
        self.name = b'test'
        self.platform = platform
        
        plt.figure(figsize=(18, 6))
        plt.ion()
        
        self.columns_carinfo = ['frame', 'Ori-velocity', 'pose-velocity-old', 'pose-velocity-new', 'Ori-yawrate', 'pose-yawrate-old', 'pose-yawrate-new', 'adjustFlag']
        self.data_pd = pd.DataFrame(columns=self.columns_carinfo)
        if self.platform == '775S':
            self.struct_format_775S_init()
        elif self.platform == '775L':
            self.struct_format_775L_init()
        elif self.platform == '777S':
            self.struct_format_777S_init()
        else:
            raise Exception('please check the platform')
    def struct_format_775S_init(self):
        self.Protocol_Head_Str_ACC_format = '<HBI'
    def struct_format_775L_init(self):
        self.EPACK_HEAD_T_format = '<IIHIII'
    def struct_format_777S_init(self):
        self.EPACK_HEAD_T_format = '<IIHIII'
    
    def verify_frameCount(self, f_cnt):
        if self.f_cnt != f_cnt:
            raise Exception(f'sparse EPACK Error: frame: {self.f_cnt}, current frame unequal {f_cnt}')
    def verify_struct(self, struct_data, struct_name):
        if struct_data[:4] != b'\xa0\xa0\xa0\xa0' or  struct_data[4:8] != b'\xb0\xb0\xb0\xb0':
            raise Exception(f'sparse EPACK Error: frame: {self.f_cnt}, struct {struct_name} head error')
        if struct_data[-8:-4] != b'\xe0\xe0\xe0\xe0' or  struct_data[-4:] != b'\xf0\xf0\xf0\xf0':
            raise Exception(f'sparse EPACK Error: frame: {self.f_cnt}, struct {struct_name} tail error')
    def draw_CarInfo_LineGraph(self, data_pd):
            plt.subplot(4, 1, 1)
            plt.axhline(0, linestyle='--', c='gray')
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['Ori-velocity']], linewidth=5,color='b', label='Ori-velocity')
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['pose-velocity-old']], linestyle=':', marker='o', c='r', label='pose-velocity-old')
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['pose-velocity-new']], linestyle=':', marker='o', c='g', label='pose-velocity-new')
            plt.xticks(data_pd['frame'].astype('Int64'), rotation=270)
            plt.legend(loc='upper left')
            plt.subplot(4, 1, 2)
            plt.axhline(0, linestyle='--', c='gray')
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['pose-velocity-old'] - data_pd['Ori-velocity']],color='r', label='diff-velocity-old')
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['pose-velocity-new'] - data_pd['Ori-yawrate']],color='b', label='diff-velocity-new')
            plt.xticks(data_pd['frame'].astype('Int64'), rotation=270)
            plt.legend(loc='upper left')
            plt.subplot(4, 1, 3)
            plt.axhline(0, linestyle='--', c='gray')
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['Ori-yawrate']], linewidth=5,color='b', label='Ori-velocity')
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['pose-yawrate-old']], linestyle=':', marker='o', c='r', label='pose-yawrate-old')
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['pose-yawrate-new']], linestyle=':', marker='o', c='g', label='pose-yawrate-new')
            plt.xticks(data_pd['frame'].astype('Int64'), rotation=270)
            plt.legend(loc='upper left')
            plt.subplot(4, 1, 4)
            plt.plot([data_pd['frame'].astype('Int64')], [data_pd['adjustFlag']])
            plt.xticks(data_pd['frame'].astype('Int64'), rotation=270)
            plt.yticks([0, 1,2])
            plt.tight_layout()  # 调整子图布局，防止重叠
            plt.show()
            plt.pause(0.1)
            plt.clf()
    def sparse_EPACK_HEAD(self, data_bin):
        if self.platform == '775S':
            totalHead, totalVersion, totalDataLength = struct.unpack_from(self.Protocol_Head_Str_ACC_format, data_bin, 0)
            return totalDataLength
        elif self.platform == '775L':
            curFrameDataLength = 0
            start_0, start_1, packid, version, f_cnt, length0 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, 0)
            self.f_cnt = f_cnt
            self.EPACK_00_NORMAL_INFO_T = data_bin[curFrameDataLength:curFrameDataLength + length0]
            self.verify_struct(self.EPACK_00_NORMAL_INFO_T, 'EPACK_00_NORMAL_INFO_T')
            curFrameDataLength += length0
            start_0, start_1, packid, version, f_cnt, length3 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, curFrameDataLength)
            self.verify_frameCount(f_cnt)
            self.EPACK_03_VEHICLE_INFO_T = data_bin[curFrameDataLength:curFrameDataLength + length3]
            self.verify_struct(self.EPACK_03_VEHICLE_INFO_T, 'EPACK_03_VEHICLE_INFO_T')
            curFrameDataLength += length3
            start_0, start_1, packid, version, f_cnt, length4 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, curFrameDataLength)
            self.verify_frameCount(f_cnt)
            self.EPACK_04_DYNAMIC_TEST_T = data_bin[curFrameDataLength:curFrameDataLength + length4]
            self.verify_struct(self.EPACK_04_DYNAMIC_TEST_T, 'EPACK_04_DYNAMIC_TEST_T')
            curFrameDataLength += length4
            start_0, start_1, packid, version, f_cnt, length5 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, curFrameDataLength)
            self.verify_frameCount(f_cnt)
            self.EPACK_05_TRACE_INFO_T = data_bin[curFrameDataLength:curFrameDataLength + length5]
            self.verify_struct(self.EPACK_05_TRACE_INFO_T, 'EPACK_05_TRACE_INFO_T')
            curFrameDataLength += length5
            start_0, start_1, packid, version, f_cnt, length6 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, curFrameDataLength)
            self.verify_frameCount(f_cnt)
            self.EPACK_06_CLUSTER_INFO_T = data_bin[curFrameDataLength:curFrameDataLength + length6]
            self.verify_struct(self.EPACK_06_CLUSTER_INFO_T, 'EPACK_06_CLUSTER_INFO_T')
            curFrameDataLength += length6
            start_0, start_1, packid, version, f_cnt, length8 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, curFrameDataLength)
            self.verify_frameCount(f_cnt)
            self.EPACK_08_FREESPACE_T = data_bin[curFrameDataLength:curFrameDataLength + length8]
            self.verify_struct(self.EPACK_08_FREESPACE_T, 'EPACK_08_FREESPACE_T')
            curFrameDataLength += length8
            start_0, start_1, packid, version, f_cnt, length10 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, curFrameDataLength)
            self.verify_frameCount(f_cnt)
            self.EPACK_10_ROAD_EDGE_T = data_bin[curFrameDataLength:curFrameDataLength + length10]
            self.verify_struct(self.EPACK_10_ROAD_EDGE_T, 'EPACK_10_ROAD_EDGE_T')
            curFrameDataLength += length10
            start_0, start_1, packid, version, f_cnt, length12 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, curFrameDataLength)
            self.verify_frameCount(f_cnt)
            self.EPACK_12_GROUND_CLUTTER_T = data_bin[curFrameDataLength:curFrameDataLength + length12]
            self.verify_struct(self.EPACK_12_GROUND_CLUTTER_T, 'EPACK_12_GROUND_CLUTTER_T')
            curFrameDataLength += length12
            start_0, start_1, packid, version, f_cnt, length15 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, curFrameDataLength)
            self.verify_frameCount(f_cnt)
            self.EPACK_15_MONITOR_T = data_bin[curFrameDataLength:curFrameDataLength + length15]
            self.verify_struct(self.EPACK_15_MONITOR_T, 'EPACK_15_MONITOR_T')
            curFrameDataLength += length15
            return curFrameDataLength
        elif self.platform == '777S':
            totalDataLength = 0
            start_0, start_1, packid, version, f_cnt, length0 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, 0)
            totalDataLength += length0
            start_0, start_1, packid, version, f_cnt, length3 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length3
            start_0, start_1, packid, version, f_cnt, length4 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length4
            start_0, start_1, packid, version, f_cnt, length5 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length5
            start_0, start_1, packid, version, f_cnt, length8 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length8
            start_0, start_1, packid, version, f_cnt, length6 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length6
            start_0, start_1, packid, version, f_cnt, length7 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length7
            start_0, start_1, packid, version, f_cnt, length9 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length9
            start_0, start_1, packid, version, f_cnt, length10 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length10
            start_0, start_1, packid, version, f_cnt, length11 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length11
            start_0, start_1, packid, version, f_cnt, length12 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length12
            start_0, start_1, packid, version, f_cnt, length14 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length14
            start_0, start_1, packid, version, f_cnt, length15 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length15
            start_0, start_1, packid, version, f_cnt, length16 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length16
            start_0, start_1, packid, version, f_cnt, length17 = struct.unpack_from(self.EPACK_HEAD_T_format, data_bin, totalDataLength)
            totalDataLength += length17 + 4 + 4
            frame_num = len(data_bin) / totalDataLength
            return totalDataLength, frame_num
        else:
            raise Exception('please check the platform')
    def sparse_carinfo_775L(self, data):
        pass
    def check(self, data_fp):
        with open(data_fp, 'rb') as f:
            curDataIndex = 0
            data_bin = f.read()
            while True:
                if curDataIndex > 0:
                    curDataLength = self.sparse_EPACK_HEAD(data_bin[curDataIndex:])
                else:
                    curDataLength = self.sparse_EPACK_HEAD(data_bin)
                frame_data = data_bin[curDataIndex:curDataIndex+curDataLength]
                if not frame_data:
                    break
                input_data = (ctypes.c_uint8 * self.max_length)(*frame_data)
                output_dataB = (ctypes.c_uint8 * self.max_length)()
                tmp_B = self.dllB.dataProcess(input_data, curDataLength, output_dataB, self.max_length, self.name)
                if tmp_B > 0:
                    output_dataB = ctypes.string_at(output_dataB, tmp_B)
                    output_dataBLength = self.sparse_EPACK_HEAD(output_dataB)
                    # 解析车身信号yawrate和speed, alg_开头的需要修改算法库输出结构体赋值，重新生成配套的dll
                    speedB, yawrateB, alg_speedB, alg_yawrateB, alg_adjustFlagB = struct.unpack_from('<fffff', self.EPACK_03_VEHICLE_INFO_T, 22+1*2+4*4)
                else:
                    raise Exception("error in dllB")
                output_dataA = (ctypes.c_uint8 * self.max_length)()
                tmp_A = self.dllA.dataProcess(input_data, curDataLength, output_dataA, self.max_length, self.name)
                if tmp_A > 0:
                    output_dataA = ctypes.string_at(output_dataA, tmp_A)
                    output_dataALength = self.sparse_EPACK_HEAD(output_dataA)
                    speedA, yawrateA, alg_speedA, alg_yawrateA, alg_adjustFlagA = struct.unpack_from('<fffff', self.EPACK_03_VEHICLE_INFO_T, 22+1*2+4*4)
                else:
                    raise Exception("error in dllA")
                if (len(self.data_pd) > 0 and self.data_pd.iloc[-1]['frame'] != self.f_cnt) or len(self.data_pd) == 0:
                    data_list = [self.f_cnt, speedA, alg_speedB, alg_speedA, yawrateA, alg_yawrateB, alg_yawrateA, alg_adjustFlagA]
                    if len(self.data_pd) >= 40:
                        self.data_pd = pd.concat([self.data_pd.iloc[1:],pd.DataFrame([data_list], columns=self.columns_carinfo)], ignore_index=True)
                    else:
                        self.data_pd = pd.concat([self.data_pd,pd.DataFrame([data_list], columns=self.columns_carinfo)], ignore_index=True)
                self.draw_CarInfo_LineGraph(self.data_pd)
                curDataIndex += curDataLength

                
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dllB','-d1', type=str, default=r'dll\20250327\775L\DataProcessB.dll', help='dll Before')
    parser.add_argument('--dllA','-d2', type=str, default=r'dll\20250327\775L\DataProcessA.dll', help='dll After')
    # parser.add_argument('--path', '-p', type=str, default=r'data\CCRS\20240711_103104_684-CCRS-100%-20kph-1\dat\20240711_103104_868_9157-CCRS-100%-20kph-1-GPS.bin') # 775S
    parser.add_argument('--path', '-p', type=str, default=r'\\Hik-sjzyf-fs01\森思泰克\1.部门文件\1.车载雷达产品部\501.车载算法部\001-算法库2.0\07-STA77-5L\02-路测数据\2.新点云协议\20250226+STA77-5L-波导-VV6+APP_03ddb0a+ALG_db1595e+机场路线路测\20250226_090745_749-上午\dat\20250226_090746_021_797.bin') # 775L
    # parser.add_argument('--path', '-p', type=str, default=r'data\CCRS\20240711_103104_684-CCRS-100%-20kph-1\dat\20240711_103104_868_9157-CCRS-100%-20kph-1-GPS.bin') # 777S
    args = parser.parse_args()
    dll_exp = CheckDLL(dllB_fp=args.dllB, dllA_fp=args.dllA, platform='775L')
    dll_exp.check(data_fp=args.path)

if __name__ == '__main__':
    main()
    print('check end...')