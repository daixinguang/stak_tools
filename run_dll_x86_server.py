import json
import socket
import ctypes
import base64
import argparse
from util import binTools as bt

class dll_server:
    def __init__(self, dll_x86_fp = r'./dll/2_0_0/DataProcess.dll'):
        # init
        max_length = 130000
        # load dll
        self.dll_x86 = ctypes.WinDLL(dll_x86_fp)

    def run(self):
        # 创建socket
        # https://blog.csdn.net/Dontla/article/details/103679153
        server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)         # 创建 socket 对象
        host = socket.gethostbyname(socket.gethostname()) # 获取本地主机名
        try:
            if port == None:
                port = 12345
        except:
            port = 12345  # 设置端口
        server.bind((host, port))        # 绑定端口
        server.listen(1)                 # 等待客户端连接
        
        while True:
            conn,addr = server.accept()     # 建立客户端连接
            # print('Connect to Address:', addr)
            try:
                while True:
                    data = conn.recv(131000).decode('utf-8')
                    if not data:
                        break
                    data_json = json.loads(data)
                    name = bytes(data_json['name'], encoding = "utf-8")
                    max_length = int(data_json['max_length'])
                    input_length = int(data_json['max_length'])
                    input_data = base64.b64decode(data_json["input_data"])
                    input_data = (ctypes.c_uint8 * max_length)(*input_data)
                    output_data = (ctypes.c_uint8 * max_length)()
                    if data_json['state'] !="off":
                        out = self.dll_x86.dataProcess(input_data, input_length, output_data, max_length, name)
                        if out > 0:
                            out_bytes = ctypes.string_at(output_data, out)
                            conn.send(out_bytes)
                        # conn.close()
                        continue
                    else:
                        conn.send("process end".encode('utf-8'))
                        conn.close()
                        raise EOFError('server closed')
                        break
            except Exception as e:
                print(f"error: {e}")
                break
            # conn.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dll32','-d', type=str, default=r'dll\20250314\2_0_0_20250314_1\DataProcess.dll')
    args = parser.parse_args()
    ds = dll_server(dll_x86_fp=args.dll32)
    ds.run()

if __name__ == '__main__':
    main()