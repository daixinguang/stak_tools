import os
import tqdm
from util import binTools as bt

# struct 文档  https://docs.python.org/zh-cn/3.12/library/struct.html

def read_frames(f, frame_length):
    while True:
        data = f.read(frame_length)
        if not data:
            break
        yield data

# 定义文件路径和名称
data_fp = 'data\DTM20250217000263\FRADAR_20240716-172427_482_0.bin'
name = b'test'

# 定义生成器函数
def process_frames():
    with open(data_fp, 'rb') as f:
        totalFrameHeader = f.read(2)
        totleVersion = f.read(1)
        totalDataLength = f.read(2)
        frame_length = bt.unpackUint16(totalDataLength)
        f.seek(0)
        yield from read_frames(f, frame_length)

# 运行速度测试
for frame_data in tqdm.tqdm(process_frames()):
    # 处理每一帧的数据
    print(frame_data)
