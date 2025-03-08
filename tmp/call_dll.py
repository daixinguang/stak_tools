import ctypes
from util import binTools as bt
from tqdm import tqdm 
import os
import shutil
import time

def call_dll_sample():
    p_dll = ctypes.WinDLL("d:\\code\\sta77-5s-cross-platform-alg-lib\\cross-platform-vc-project\\Debug\\DataProcess.dll")
    data_dir = "c:\\Users\\yangruozhang\\Desktop\\eval\\20231226_154645_3831-新程序验证-常规路测\\dat"
    for root, sub_dirs, files in os.walk(data_dir):
        for file_name in files:
            if '.bin' in file_name:
                file_path = os.path.join(root, file_name)
                print("processing {0}".format(file_path))
                with open(file_path, 'rb') as f:
                    totalFrameHeader = f.read(2)
                    totleVersion = f.read(1)
                    totalDataLength = f.read(2)
                    frame_length = bt.unpackUint16(totalDataLength)
                    # f.seek(3024 * frame_length)
                    f.seek(0)
                    data = f.read(frame_length)
                    res=ctypes.c_uint8(13000)
                    name = b'test'
                    input_data = (ctypes.c_uint8 * 130000)(*data)
                    output_data = (ctypes.c_uint8 * 130000)()
                    cnt = 0
                    # run speed test
                    for i in tqdm(range(5000)):
                        cnt += 1
                        data = f.read(frame_length)
                        if not data:
                            break
                        tmp = p_dll.dataProcess(input_data, frame_length, output_data, 130000, name)
                        # print(tmp)
                        # if cnt > 100:
                        #     break

class AlglibDll:
    def __init__(self, dll_path = "d:\\code\\sta77-5s-cross-platform-alg-lib\\cross-platform-vc-project\\Debug\\DataProcess.dll", index = 0):
        self.load_dll(dll_path, index)
        self.cnt = 0
        self.max_length = 130000

    def load_dll(self, dll_path, index=0):
        if not os.path.exists(dll_path):
            raise Exception("dll path not exists")
        self.dll_path = dll_path
        if not os.path.exists("run_tmp"):
            os.mkdir("run_tmp")
        process_id = os.getpid()
        self.real_dll_path = os.path.join("run_tmp", "DataProcess_{1}_{0}.dll".format(process_id, index))
        if not os.path.exists(self.real_dll_path):
            shutil.copy(dll_path, self.real_dll_path)
            time.sleep(2)
        self.p_dll = ctypes.WinDLL(self.real_dll_path)

    def run(self, input_data, input_length, name=b'test'):
        self.cnt += 1
        input_data = (ctypes.c_uint8 * self.max_length)(*input_data)
        output_data = (ctypes.c_uint8 * self.max_length)()
        # if name is a string, it will be converted to bytes
        if isinstance(name, str):
            name = name.encode('utf-8')
        tmp = self.p_dll.fradar(input_data, input_length, output_data, self.max_length, name)
        if tmp > 0:
            return output_data[:tmp]
        else:
            raise Exception("error in dll")
    
    def run_file(self, file_path):
        with open(file_path, 'rb') as f:
            totalFrameHeader = f.read(2)
            totleVersion = f.read(1)
            totalDataLength = f.read(2)
            frame_length = bt.unpackUint16(totalDataLength)
            # f.seek(3024 * frame_length)
            f.seek(0)
            data = f.read(frame_length)
            # cal frame num: file total length / frame length
            frame_num = int(os.path.getsize(file_path) / frame_length)
            name = b'test'
            cnt = 0
            # run speed test
            for i in tqdm(range(frame_num)):
                cnt += 1
                data = f.read(frame_length)
                if not data:
                    break
                res = self.run(data, frame_length, name)
                # print(tmp)
                # if cnt > 100:
                #     break

    # def __del__(self):
        # release dll
        # win32api.FreeLibrary(self.p_dll._handle)
        # if os.path.exists(self.real_dll_path):
        #     os.remove(self.real_dll_path)
    
if __name__ == "__main__":
    # call_dll_sample()
    
    dll_x64_fp = r'./dll/fradar.dll'
    dll_x86_fp = r'./dll/DataProcess.dll'
    data_fp = r"E:\alg_V1.0\长安CD701发版文件\20250221\FRADAR_20240730-190513_075_0.bin"
    dll_path = dll_x64_fp
    dll = AlglibDll(dll_path)
    dll.run_file(data_fp)