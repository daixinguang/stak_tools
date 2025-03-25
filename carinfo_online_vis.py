'''
通过watchdog监控文件实现对车身信号可视化
plt折线图
https://blog.csdn.net/weixin_46707493/article/details/119722246
'''
import os
import re
import time
import pandas as pd
import matplotlib.pyplot as plt
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


'''
定义文件处理器, 继承文件事件处理器
'''
class MyFileHandler(FileSystemEventHandler):
    def __init__(self):
        pass
        
    
    # 重写on_modified方法, CALLED WHEN A FILE OR DIRECTORY IS MODIFIED.
    def on_modified(self, event):
        global data_list
        global data_pd
        global plt
        try:
            # 分析点云数据
            if re.search(r'CarInfoRealtime.txt', event.src_path):
                # print(f"file on modified: {event.src_path}")
                with open(event.src_path, 'r') as f:
                    data = f.read().strip()
                    if data != '':
                        data_list = data.split('\t')
                        data_list = [float(i.strip()) for i in data_list]
                        if (len(data_pd) > 0 and data_pd.iloc[-1]['frame'] != data_list[0]) or len(data_pd) == 0:
                            if len(data_pd) >= 40:
                                data_pd = pd.concat([data_pd.iloc[1:],pd.DataFrame([data_list], columns=columns_carinfo)], ignore_index=True)
                            else:
                                data_pd = pd.concat([data_pd,pd.DataFrame([data_list], columns=columns_carinfo)], ignore_index=True)
        except Exception as e:
            print(f'file path read err: {event.src_path}, {e}')
            
def draw():
    pass
def observerRun(path):
    global columns_carinfo 
    global data_pd
    columns_carinfo = ['frame', 'Ori-velocity', 'pose-velocity', 'Sensor-velocity', 'Sensor-adjustVelocity', 'Sensor-predictEgoSpeed', 'Ori-yawrate', 'pose-yawrate', 'Sensor-yawrate', 'Sensor-adjustYawRate', 'adjustFlag', 'wheelAngle', 'Sensor-vx', 'Sensor-vy', 'pose-ax', 'pose-ay']
    data_pd = pd.DataFrame(columns=columns_carinfo)
    plt.figure(figsize=(18, 6))
    plt.ion()
    # 声明
    event_handler = MyFileHandler()
    # 监控器
    observer = Observer()
    # 注册时间处理器, 配置监控目录
    observer.schedule(event_handler, path, recursive=False)
    # 监控器启动(创建线程)
    observer.start()
    # 以下代码保持主线程持续运行
    # plt.grid(linestyle='-.')
    # plt.show()
    try:
        while True:
            # time.sleep(0.01)
            plt.subplot(3, 1, 1)
            plt.axhline(0, linestyle='--', c='gray')
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['Ori-velocity'], linewidth=5,color='b', label='Ori-velocity')
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['Sensor-velocity'], label='Sensor-velocity')
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['Sensor-adjustVelocity'], c='r', label='Sensor-adjustVelocity')
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['Sensor-predictEgoSpeed'], linestyle='-', c='m', label='Sensor-predictEgoSpeed')
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['pose-velocity'], linestyle=':', marker='o', c='g', label='pose-velocity')
            plt.xticks(data_pd['frame'].astype('Int64'), rotation=270)
            plt.legend(loc='upper left')
            plt.subplot(3, 1, 2)
            plt.axhline(0, linestyle='--', c='gray')
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['Ori-yawrate'], linewidth=5,color='b', label='Ori-yawrate')
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['Sensor-adjustYawRate'], c='r', label='Sensor-adjustYawRate')
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['pose-yawrate'], linestyle=':', marker='o', c='g', label='pose-yawrate')
            plt.xticks(data_pd['frame'].astype('Int64'), rotation=270)
            plt.legend(loc='upper left')
            plt.subplot(3, 1, 3)
            plt.plot(data_pd['frame'].astype('Int64'), data_pd['adjustFlag'])
            plt.xticks(data_pd['frame'].astype('Int64'), rotation=270)
            plt.yticks([0, 1,2])
            plt.tight_layout()  # 调整子图布局，防止重叠
            plt.show()
            plt.pause(0.1)
            plt.clf()
    except KeyboardInterrupt:
        observer.stop()
    # 主线程结束后, 进入阻塞状态,等待其他线程结束后, 主线程终止
    observer.join()
def main():
    # 定义监控目录
    path = r"E:\code\cross-platform-alg-lib-v2\cross-platform-vc-project\DataProcess"
    observerRun(path)

if __name__ == '__main__':
    main()
