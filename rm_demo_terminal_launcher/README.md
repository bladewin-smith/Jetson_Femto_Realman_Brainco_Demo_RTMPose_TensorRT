# RM BrainCo Orbbec Demo 启动器（C++ 版本）

这个项目用于在 Jetson 上编译一个 C++ 可执行文件，点击桌面图标即可打开新终端并运行：

```bash
DISPLAY=:0 /usr/local/bin/rm_brainco_orbbec_demo >> /home/jetson/.local/share/rm_brainco_orbbec_demo/autostart.log 2>&1
```

启动时终端会先打印：
- `按下 H 键可让机械臂回到设定原位`
- `按 Ctrl+C 退出程序`
- 以及启动命令与日志路径

## 目录结构

- `CMakeLists.txt`：工程构建脚本
- `src/main.cpp`：启动器源码
- `scripts/install_desktop_launcher.sh`：编译并安装可执行文件及桌面快捷方式
- `scripts/uninstall_desktop_launcher.sh`：卸载

## 编译与安装桌面快捷方式

```bash
cd /path/to/rm_demo_terminal_launcher
chmod +x scripts/install_desktop_launcher.sh scripts/uninstall_desktop_launcher.sh
./scripts/install_desktop_launcher.sh
```

安装后会在 `~/.local/share/applications/rm_brainco_orbbec_demo.desktop` 生成桌面入口，
可在应用菜单中直接点击启动。

## 命令参数（可选）

- `--command <cmd>`：覆盖要运行的主命令（默认 `/usr/local/bin/rm_brainco_orbbec_demo`）
- `--log <path>`：覆盖日志文件路径（默认 `~/.local/share/rm_brainco_orbbec_demo/autostart.log`）

示例：

```bash
rm_brainco_orbbec_demo_terminal --command "/usr/local/bin/rm_brainco_orbbec_demo --check" --log /tmp/rm_demo.log
```

## 注意事项

- 脚本默认使用 `DISPLAY=:0`，若你的显示不是 `:0` 可手动设置 `DISPLAY` 环境变量并重写 `--command`。
- 终端执行器会按优先级自动选择 `gnome-terminal / konsole / xfce4-terminal / mate-terminal / xterm`，请至少安装一个。
