import os
import struct
import ctypes
import argparse
from tqdm import tqdm
from util import binTools as bt

class AlglibDll:
    def __init__(self, 
                 bitflag: str, 
                 dll_fp: str = r'./dll/2_0_0/DataProcess.dll'):
        self.max_length = 130000
        # load dll
        self.bitflag = bitflag
        if self.bitflag=='32bit':
            self.dll_x86 = ctypes.WinDLL(dll_fp)
        elif self.bitflag=='64bit':
            self.dll_x64 = ctypes.WinDLL(dll_fp)
        else:
            raise Exception('must choice bitflag')

    def run(self, data_bin:bytes):
        if self.bitflag=='32bit':
            return self.dll_x86.dataprocess(data_bin)
        elif self.bitflag=='64bit':
            return self.dll_x64.fradar(data_bin)

def load_data(path):
    data_bin = open(path, 'rb').read()
    # 解析文件头信息 Protocol_Head_Str_ACC
    Protocol_Head_Str_ACC_format = '<HBI'
    totalHead, totalVersion, totalDataLength = struct.unpack_from(Protocol_Head_Str_ACC_format, data_bin, 0)
    Protocol_Head_Str_ACC_offset=struct.calcsize(Protocol_Head_Str_ACC_format) # 7
    subHead_format = '<HBH'
    # 帧头数据 Frame_Head_Date_Str_ACC
    subHead1, subVersion1, subDataLength1 = struct.unpack_from(subHead_format, data_bin, Protocol_Head_Str_ACC_offset)
    Frame_Head_Date_Str_ACC_offset=Protocol_Head_Str_ACC_offset + subDataLength1 # 7+81=88
    # RF_ConfigPara_Str_ACC
    subHead2, subVersion2, subDataLength2 = struct.unpack_from(subHead_format, data_bin, Frame_Head_Date_Str_ACC_offset)
    RF_ConfigPara_Str_ACC_offset=Frame_Head_Date_Str_ACC_offset + subDataLength2 # 88+117=205
    # 雷达状态参数 Radar_Status_Date_Str_ACC
    subHead3, subVersion3, subDataLength3 = struct.unpack_from(subHead_format, data_bin, RF_ConfigPara_Str_ACC_offset)
    Radar_Status_Date_Str_ACC_offset=RF_ConfigPara_Str_ACC_offset + subDataLength3 # 205+200=405
    # 监控数据 MonitorData_DSP_t_ACC
    subHead4, subVersion4, subDataLength4 = struct.unpack_from(subHead_format, data_bin, Radar_Status_Date_Str_ACC_offset)
    MonitorData_DSP_t_ACC_offset=Radar_Status_Date_Str_ACC_offset + subDataLength4 # 405+129=534
    # ECU_Moni_Str_ACC
    subHead5, subVersion5, subDataLength5 = struct.unpack_from(subHead_format, data_bin, MonitorData_DSP_t_ACC_offset)
    ECU_Moni_Str_ACC_offset=MonitorData_DSP_t_ACC_offset + subDataLength5 # 534+252=786
    # ADAS_Monitor_Str
    subHead6, subVersion6, subDataLength6 = struct.unpack_from(subHead_format, data_bin, ECU_Moni_Str_ACC_offset)
    ADAS_Monitor_Str_offset=ECU_Moni_Str_ACC_offset + subDataLength6 # 786+1959=2745
    return data_bin
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dll32','-d', type=str, default=r'dll\DataProcess.dll')
    parser.add_argument('--path','-p', type=str, default=r'data\DTM20250217000263\FRADAR_20240716-172427_482_0.bin')
    parser.add_argument('--platform','-x', type=str)
    args = parser.parse_args()
    if not args.platform:
        import platform
        args.platform, _ = platform.architecture()
    dll = AlglibDll(bitflag=args.platform, dll_fp=args.dll32)
    load_data(args.path)
    dll.run()

if __name__ == '__main__':
    main()