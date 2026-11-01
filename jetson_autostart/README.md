# Jetson Orin Nano Autostart Package

这个目录用于把 RTMDet + RTMPose + 专用双手版 `camera_arm_rm65b_control_rtmpose.py` 改造成
Jetson Orin Nano 开机自启动程序。

默认策略：

- 使用 `systemd` 后台服务启动。
- 默认关闭 OpenCV GUI：`show_window:=false`。
- 所有路径和 ROS 参数集中放在 `/etc/rm_brainco_orbbec_demo/rm_brainco_orbbec_demo.env`。
- 程序异常退出后自动重启。
- 启动前等待机器人 IP、USB-RS485 设备等基础条件。

## 文件说明

| 文件 | 用途 |
| --- | --- |
| `rm_brainco_orbbec_demo.env` | 可编辑参数模板。 |
| `rm_brainco_orbbec_demo.service.in` | systemd 服务模板。 |
| `run_rm_brainco_orbbec_demo.sh` | 真正启动 Python 主控脚本的 launcher。 |
| `install_autostart.sh` | 安装并启用开机自启动。 |
| `uninstall_autostart.sh` | 停止并卸载自启动服务。 |

## 安装

把整个仓库放到 Jetson，例如：

```bash
/home/jetson/ws/rm_brainco_orbbec_demo
```

然后执行：

```bash
cd /home/jetson/ws/rm_brainco_orbbec_demo/jetson_autostart
sudo ./install_autostart.sh
```

安装脚本会：

1. 复制 launcher 到 `/usr/local/bin/rm_brainco_orbbec_demo`
2. 复制默认 env 到 `/etc/rm_brainco_orbbec_demo/rm_brainco_orbbec_demo.env`
3. 生成 systemd service
4. 执行 `systemctl enable`

## 修改参数

```bash
sudo nano /etc/rm_brainco_orbbec_demo/rm_brainco_orbbec_demo.env
sudo systemctl restart rm_brainco_orbbec_demo.service
```

## 图形窗口自启动

如果希望开机后打开 OpenCV 图像窗口，让用户看到自己是否进入识别范围，不要使用纯后台 systemd 服务来弹窗。推荐改成“桌面登录后自启动”：

```bash
cd /home/jetson/ws/rm_brainco_orbbec_demo/jetson_autostart
sudo ./install_gui_desktop_autostart.sh
```

这个脚本会：

1. 禁用后台 `rm_brainco_orbbec_demo.service`，避免同时启动两个控制程序。
2. 复制 launcher 到 `/usr/local/bin/rm_brainco_orbbec_demo`。
3. 把 `SHOW_WINDOW=true`、`AUTO_START_ON_READY=false` 写入 env。
4. 安装桌面自启动文件到 `~/.config/autostart/rm_brainco_orbbec_demo.desktop`。

图形窗口模式推荐参数：

```bash
SHOW_WINDOW=true
AUTO_START_ON_READY=false
```

这样程序启动后会进入等待画面，用户进入相机视野后，可以在窗口里看到识别范围，并手动按 `s` 开始控制，按 `h` 回原位。

如果你希望“有窗口，同时检测到最近人后自动开始”，可以改成：

```bash
SHOW_WINDOW=true
AUTO_START_ON_READY=true
AUTO_START_DELAY_S=1.0
```

取消桌面自启动：

```bash
cd /home/jetson/ws/rm_brainco_orbbec_demo/jetson_autostart
sudo ./uninstall_gui_desktop_autostart.sh
```

注意：桌面窗口自启动要求 Jetson 启动后进入图形桌面。如果希望完全无人值守，请在系统设置中打开 `jetson` 用户自动登录。

如果你的手动运行命令是：

```bash
python /home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT/scripts/camera_arm_rm65b_control_rtmpose.py ...
```

则 env 里至少应保持：

```bash
APP_ROOT=/home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT
CONTROL_SCRIPT=/home/jetson/ws/Jetson_Femto_Realman_Brainco_Demo_RTMPose_TensorRT/scripts/camera_arm_rm65b_control_rtmpose.py
HAND_RS485_PORT=auto
HAND_RS485_SLAVE_ID=0
HAND_RS485_DURATION_MS=60
WAIT_FOR_HAND_RS485_PORT=false
```

后台自启动没有 OpenCV 窗口接收 `s` 键。如果希望开机后最近人进入视野就自动开始识别和控制：

```bash
SHOW_WINDOW=false
AUTO_START_ON_READY=true
AUTO_START_DELAY_S=1.0
```

如果只是开机后进入等待、仍然想手动按 `s`，需要在桌面会话里运行并打开窗口：

```bash
SHOW_WINDOW=true
AUTO_START_ON_READY=false
```

## 手动启动/停止

```bash
sudo systemctl start rm_brainco_orbbec_demo.service
sudo systemctl stop rm_brainco_orbbec_demo.service
sudo systemctl restart rm_brainco_orbbec_demo.service
```

## 查看日志

```bash
journalctl -u rm_brainco_orbbec_demo.service -f
```

如果只想看最近一次启动：

```bash
journalctl -u rm_brainco_orbbec_demo.service -b
```

## 取消自启动

```bash
cd /home/jetson/ws/rm_brainco_orbbec_demo/jetson_autostart
sudo ./uninstall_autostart.sh
```

## 重要提醒

- systemd 开机自启动不适合显示 GUI，默认 `SHOW_WINDOW=false`。
- 如果要用 GUI 调参，请先停止服务，再手动运行 Python 脚本。
- 如果你使用 Conda 环境，设置 `CONDA_SH` 和 `CONDA_ENV`。
- 如果你使用 venv，设置 `VENV_ACTIVATE`。
- 如果灵巧手走 USB-RS485 且使用 `HAND_RS485_PORT=auto`，自启动脚本不会等待固定 `/dev/ttyUSB0`；如果改成固定端口，再把 `WAIT_FOR_HAND_RS485_PORT=true` 打开。
- 如果启用 Lingbot PyTorch 深度增强，开机启动耗时会明显增加，建议先用 `DEPTH_ENHANCEMENT_BACKEND=filter` 验证稳定性。
