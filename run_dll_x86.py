import os
import ctypes
from tqdm import tqdm
from util import binTools as bt

class AlglibDll:
    def __init__(self):
        # load dll
        self.dll_x64 = None
        # self.dll_x86 = None
        self.max_length = 130000

    def run_file64(self, data_fp=r'data\FRADAR_20240730-190513_075_0.bin', dll_fp=r'./dll/fradar.dll'):
        self.dll_x64 = ctypes.WinDLL(dll_fp)
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

            # run speed test
            for i in tqdm(range(frame_num)):
                data = f.read(frame_length)
                if not data:
                    break
                res = self.run64(data, frame_length, name)

    def run64(self, input_data, input_length, name=b'test'):
        input_data = (ctypes.c_uint8 * self.max_length)(*input_data)
        output_data = (ctypes.c_uint8 * self.max_length)()
        # if name is a string, it will be converted to bytes
        if isinstance(name, str):
            name = name.encode('utf-8')
        tmp = self.dll_x64.fradar(input_data, input_length, output_data, self.max_length, name)
        if tmp > 0:
            return output_data[:tmp]
        else:
            raise Exception("error in dll")
        
    def run_file86(self, data_fp=r'data\FRADAR_20240730-190513_075_0.bin', dll_fp=r'./dll/DataProcess.dll'):
        self.dll_x86 = ctypes.WinDLL(dll_fp)

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

            # run speed test
            for i in tqdm(range(frame_num)):
                data = f.read(frame_length)
                if not data:
                    break
                res = self.run86(data, frame_length, name)
    def run86(self, input_data, input_length, name=b'test'):
        input_data = (ctypes.c_uint8 * self.max_length)(*input_data)
        output_data = (ctypes.c_uint8 * self.max_length)()
        # if name is a string, it will be converted to bytes
        if isinstance(name, str):
            name = name.encode('utf-8')
        tmp = self.dll_x86.dataProcess(input_data, input_length, output_data, self.max_length, name)
        if tmp > 0:
            return output_data[:tmp]
        else:
            raise Exception("error in dll")
def main():
    dll_x64_fp = r'./dll/fradar.dll'
    dll_x86_fp = r'./dll/DataProcess.dll'
    dll_exp = AlglibDll()
    dll_exp.run_file86()
if __name__ == '__main__':
    main()
    print('check end...')