# STAK_TOOLS

c编译的dll输出内容使用python脚本进行处理。

## 工具列表

[1] 校验长安64位dll和通用32位dll输出内容是否一致。

    32位dll需要使用32位python环境，64位dll需要使用64位python环境。然后通过socket实现同时运行两个环境的dll，在64位dll进行比较。

```powershell
conda activate py32
python run_dll_x86_server.py

conda activate python312
python check_changan_x64.py
```

[2] 校验车身信号优化前后的连续帧可视化。

    需要同时提供优化前和优化后的dll，关闭调试信息打印。然后通过回灌统一数据，可视化车身信号 pose-velocity 和 pose-yawrate 修改前后的对比。

    可使用775S、775L、777S的dll进行回灌比较。
