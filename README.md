# 450expo · STM32 超声波 Chirp 测距

STM32G474RBT3 发射 38–42 kHz 的线性扫频，采集超声回波并在 MCU 上估计往返时间和距离。Python 上位机通过 UART 接收测距结果和波形，在本地浏览器中显示距离、温度、波形和频谱。

## 工作原理

1. **发射与采样。** MCU 先启动 ADC1 的 DMA 采集，由 TIM15 以 500 kS/s 触发 ADC，每帧采 16,384 个 12 位点，时长约 32.77 ms。随后 HRTIM1 Timer B 通过 DMA 更新周期和比较寄存器，生成 20 ms 的发射扫频。固件用接收 DMA 的已采样点数记录发射起点，供后续解调对齐。
2. **解调与滤波。** MCU 对回波去均值，用本地扫频参考做正交混频。回波延迟为 $\tau$ 秒时，低频拍频的绝对值满足 $|f_b| \approx S\tau$，其中 $S=(42\,000-38\,000)/0.02=200\,000\ \mathrm{Hz/s}$。I、Q 分别经过 129 抽头、5 kHz FIR，再抽取 16 倍，得到 31.25 kS/s 的 1,024 对 I/Q。
3. **估计距离。** 给 I/Q 加 Kaiser 窗（$\beta=4$），做 1,024 点 FFT，在对应 100–1,500 mm 的频带内寻找功率峰，再用峰及相邻两格的对数功率做三点插值。用公式 $R=c|f_b|/(2S)$ 得到单程候选距离，分母中的 2 来自超声往返。
4. **温度补偿。** ADC5 默认通过 PB14/OPAMP5 读取 NTC2，按 1 秒周期调度，声学收发忙时延后。每帧开始时锁存温度和声速，以 $c=331.3+0.606T$（m/s，$T$ 为摄氏度）换算距离。外部温度不可用时，默认按 343 m/s 估算并标记未温补。
5. **串口回传。** UART4 使用 115200、8N1。默认 Chirp 帧由帧头、1,024 对有符号时域 I/Q 和 CRC16 校验值组成。帧头带 MCU 估计的拍频、声速和温度。网页用拍频与声速保留距离的小数位，并显示本帧使用的温度。

| 要看什么 | 代码 |
| --- | --- |
| 主循环、收发调度和 UART 回传 | [main.c](STM32G474RBT3_FW/Core/Src/main.c) · [mcu_acquisition_control.c](STM32G474RBT3_FW/Core/Src/mcu_acquisition_control.c) |
| 混频、滤波、FFT、测距 | [rx_processing.c](STM32G474RBT3_FW/Core/Src/rx_processing.c) |
| ADC5 测温与声速换算 | [temperature_monitor.c](STM32G474RBT3_FW/Core/Src/temperature_monitor.c) · [temperature_compensation.c](STM32G474RBT3_FW/Core/Src/temperature_compensation.c) |
| 串口帧解析、本地服务和网页 | [rx_protocol.py](tools/rx_waveform/rx_protocol.py) · [rx_viewer.py](tools/rx_waveform/rx_viewer.py) · [viewer.html](tools/rx_waveform/viewer.html) |

## 用 STM32CubeIDE 编译固件

1. 克隆仓库并打开 STM32CubeIDE。用 **File → Import… → General → Existing Projects into Workspace**，在 **Select root directory** 选择仓库中的 `STM32G474RBT3_FW/`。
2. 勾选 `STM32G474RBT3_FW` 工程后点击 **Finish**。项目配置和 HAL/CMSIS 源码已随仓库提供，直接构建现有工程即可。
3. 右键工程，选 **Build Configurations → Set Active → Debug**，再点 **Build Project**。生成 `STM32G474RBT3_FW/Debug/STM32G474RBT3_FW.elf`。选择 **Release** 时，产物位于 `Release/`。
4. 工程以 STM32G474RBTx、Cortex-M4 硬浮点和 `-Os` 构建。使用随仓库提供的 `STM32G474RBTX_FLASH.ld`，应用位于从 `0x08000000` 开始的首个 64 KiB Flash 区间。[STM32CubeIDE 工程文档](https://dev.st.com/stm32cube-docs/stm32cubeide/2.3.0/en/docs/markup/QuickStart_Guide/QuickStart_Workspace.html)

## 在终端手动编译固件

需要 Python 3.10 或更高版本和带 Newlib Nano 的 Arm GNU Toolchain。可使用独立安装的工具链，也可使用 CubeIDE 随附的 `arm-none-eabi-gcc`、`arm-none-eabi-objcopy`、`arm-none-eabi-size`。[Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)

在仓库根目录运行。

```sh
python tools/build_firmware.py
```

若编译器尚未加入 `PATH`，用 `--gcc` 指定该工具链的 `arm-none-eabi-gcc` 可执行文件。Windows 示例中的路径请替换为自己的安装位置。

```powershell
python tools/build_firmware.py --gcc "<工具链 bin 目录>/arm-none-eabi-gcc.exe"
```

脚本从 C/汇编源码编译并链接，在 `build/` 生成 `STM32G474RBT3_FW.elf`、`STM32G474RBT3_FW.hex` 和链接映射文件 `STM32G474RBT3_FW.map`。编译阶段只需要电脑上的工具链。

## 通过 DAPLink 烧录

板子**只使用 DAPLink（CMSIS-DAP）通过 SWD 烧录**。给目标板按设计供电，用 DAPLink 连接以下信号。配置使用 DAPLink 标准 USB VID/PID `0D28:0204`。[DAPLink 项目](https://github.com/ARMmbed/DAPLink) · [DAPLink USB 配置](https://github.com/ARMmbed/DAPLink/blob/main/source/hic_hal/stm32/stm32f103xb/usb_config.c)

| DAPLink | 目标板 |
| --- | --- |
| SWDIO | PA13 / SWDIO |
| SWCLK | PA14 / SWCLK |
| GND | GND |
| VTref（若探针有此引脚） | 目标板的 3.3 V 逻辑参考 |

烧录前在查看器停止连续测量，并结束已有的调试会话。

### STM32CubeIDE 中设置 DAPLink

1. 打开 **Run → Debug Configurations… → GDB Hardware Debugging**，新建配置。在 **Main** 页选项目 `STM32G474RBT3_FW`、构建配置 **Debug**、应用程序 `Debug/STM32G474RBT3_FW.elf`，可启用 **Build before launching**。
2. 在 **Debugger** 页将 **GDB Command** 指向 CubeIDE 随附的 `arm-none-eabi-gdb`，勾选 **Use remote target**，在 **JTAG Device** 选择 **OpenOCD (via pipe)**，协议填 `remote`。在 **Connection** 中填写下列结构，保留开头的 `|`，把尖括号内容替换为实际路径。

   ```text
   | "<OpenOCD 可执行文件>" -s "<OpenOCD 脚本目录>" -f "<工程目录>/debug_support/daplink_pipe.cfg"
   ```

   使用 CubeIDE 随附工具时，在安装目录的 `plugins/` 查找。GDB 位于 `com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/`，OpenOCD 位于 `com.st.stm32cube.ide.mcu.externaltools.openocd.*/tools/bin/`，脚本目录为 `com.st.stm32cube.ide.mcu.debug.openocd_*/resources/openocd/st_scripts/`。[OpenOCD 的 GDB 管道模式](https://openocd.org/doc/html/GDB-and-OpenOCD.html)

3. 在 **Startup** 页的 **Initialization Commands** 填 `monitor reset halt`，勾选 **Load Image** 和 **Load Symbols**，两项都选 **Use project binary**。在 **Run Commands** 填 `monitor reset halt`，勾选 **Resume**。首次运行可清除已有断点并取消入口断点选项。
4. 点 **Apply → Debug**。在 GDB 控制台确认 ELF 的加载输出，随后确认目标状态为 **Running**。若命中断点，点击 **Resume**。

### 在终端用 DAPLink 烧录

先按上文完成手动编译，再从仓库根目录运行，使用支持 CMSIS-DAP 和 STM32G4 的 OpenOCD 0.12 或兼容版本。使用 CubeIDE 随附的 OpenOCD 时，在命令中加入 `-s "<OpenOCD 脚本目录>"`。使用 IDE 生成的镜像时，将 `program` 的 ELF 路径换成 `STM32G474RBT3_FW/Debug/STM32G474RBT3_FW.elf`。

```sh
openocd -f STM32G474RBT3_FW/debug_support/daplink_swd.cfg -c "program build/STM32G474RBT3_FW.elf verify reset exit"
```

完成时 OpenOCD 会校验镜像并复位运行。确认日志包含 `Verified OK`。[OpenOCD 烧录命令](https://openocd.org/doc/html/Flash-Programming.html)

## 启动串口网页查看器

网页由本地 Python 服务直接提供。Windows 可在 `tools/rx_waveform/` 运行 `install_dependencies.cmd`，然后运行 `start_viewer.cmd`。

也可手动安装和启动。在仓库根目录打开终端，先执行。

```sh
cd tools/rx_waveform
python -m venv .venv
```

Windows PowerShell。

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe rx_viewer.py
```

Linux 或 macOS。

```sh
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python rx_viewer.py
```

默认打开 [本地查看器](http://127.0.0.1:8765/)。给 `rx_viewer.py` 加 `--demo` 可离线查看模拟波形。若默认端口被占用，可加 `--port 0` 并打开终端输出的地址。

实测时使用 **3.3 V TTL USB-UART**。转换器 RXD 接板子 PC10（UART4_TX），TXD 接 PC11（UART4_RX），两端 GND 共地。根据设备描述选择数据串口，点击 **连接**。选择 **Chirp 测距 · 1024 I/Q 点**，再点 **采集一帧** 或 **开始连续**。温补有效时，候选距离下方会显示本帧的摄氏温度。

`Drivers/` 中保留了 STM32 HAL 和 CMSIS 的许可文件。
