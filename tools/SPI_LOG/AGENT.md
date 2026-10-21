# ASCENT：SOC 黑盒子开发记录（agent 交接文档）

更新：2026-10-21。当前采用 v2.1 单向流；以下历史记录有过期结论，以本节和当前代码为准。

## 当前修复：开始记录时崩溃（2026-10-21）

- 接手时实际 `ascent.c` 的 `busSegment_t segments[2]` 仍在 `ascentUpdate()` 栈上，
  与下文“栈段已修复”记录不符。异步 DMA 中断在函数返回后继续访问失效段表。
- 将测试 SPI 后端改为保留段表指针、下一 tick 才完成 DMA；ASan 稳定报
  `stack-use-after-return`，定位 `ascentUpdate` 的 segments，修复前失败、修复后通过。
- 段表改文件级 static；发送前检查总线忙和 TX DMA 可用性；DMA 超时 100ms 后
  停用本次启动的记录设备，并保留段表/缓冲供迟到中断使用。没有改成阻塞轮询。
- 修正 blackbox flush/working/reserve 入口错误依赖 USE_FLASHFS，恢复原 FLASH reserve 分支。
- `debug[0]`：7 已初始化，8 DMA 超时，9 DMA 不可用；`debug[2]` 为发送提交计数。
- 主机测试模拟异步段表生命周期、共享总线忙、无 DMA、迟到中断；两侧共 14 项通过。
- F405 和 G473 构建通过；F405 修复固件已用 DFU 烧录并验证。
- ALWAYS、1/4 采样率连续观察 60 秒：状态持续为 RUNNING (11)，
  发送计数从 453 增至 12833，缓冲溢出计数 0，MSP 响应约 7..22ms。
  ESP 重启后读取持久化 offset=942712，证明已收到并保存日志。
- 另一次 ALWAYS 测试中连续 3 次将 ESP 保持复位约 1.5 秒，飞控仍为 RUNNING，
  MSP 响应约 7..24ms，无驱动停用或缓冲溢出。此测试验证掉线不挂飞控，不保证掉线期间日志完整。
- 已恢复并保存 NORMAL、1/4。未解锁电机，未做试飞及本轮日志完整解码。
- F405 固件：`obj/betaflight_CADDXF405WING-balckboxtest.bin`，SHA256
  `79231d2d47647811e97c0e36a0f7562968a0a93fb6daa9c10f23e4d7ec643164`。
  上述 SHA 为板端回归所用固件。交接包随后按提交 aaae51a54 重新构建并更新双目标固件，
  已替换旧 G473 hex；包内准确校验值见 `packs/SHA256SUMS.txt`，本次更新包未再次烧板。
- 旧诊断进程可能永久卡在 USB 写入；已结束本次发现的占用进程，后续串口必须独占。
- 下文 v1 的 flashfs/export/擦除记录只作历史参考，不适用于当前单向流。

## 零、需求 v2（2026-10-20，覆盖此前所有设计目标）

- 不再伪装 flash 块设备。log 数据能传出来即可，内容 = UART-blackbox 的原始流
- VTX（SOC）自管存储/丢弃/导出；不用 Configurator 导出；不用管空间，满了 VTX 自己抛
- 协议收敛为 PING(0x5A01) / DATA(0x5A10, log 块→ACK) / STATUS(0x5A07,可选)；
  GEOMETRY/READ/WRITE/ERASE_* 作废；传输节奏（物理层+事务+CRC）沿用已验证的 v1
- 完整定义见 `ASCENT_需求.txt`（v2）

**v2 已实现（2026-10-20 下午）：**
1. ascent.c/h 重写为流式驱动：PING(0x5A01) 探活 + DATA(0x5A10) 流式推送 + 序列号去重；
   4KB 环形缓冲、满则丢弃、超时丢块不锁死、惰性初始化（taskMain 首圈探活，2s 窗口）
2. blackbox_io.c：7 处 ASCENT case 改流式语义（write/writeString/flushForce/open/
   working/headerBudget/reserve），其余 5 处堆叠已摘除
3. flash.c v1 挂接已移除（该文件已回原样，提交在 HEAD 的挂接是编辑删除的）
4. init/msp/usb_msc/cms/cli/flashfs 的 v1 挂接全部 git checkout 还原
5. config：balckboxtest + G473 已去掉 USE_FLASH_CHIP/USE_FLASHFS/USE_ASCENT_SPI，
   仅需 USE_ASCENT + ASCENT_SPI_CS_PIN + ASCENT_SPI_INSTANCE + DEFAULT_BLACKBOX_DEVICE
6. ESP32 从机 v2：PING/DATA/STATUS 三命令，分区追加存储（头扇区持久化长度，
   每 16 块落一次），满则丢弃仍 ACK；已构建+烧录验证（selftest PASS）
7. 单测重写：`python3 src/test/unit/ascent_test.py` 12/12 PASS（驱动 7 + 从机 5，ASan/UBSan）
8. 双目标构建通过：CADDXF405WING-balckboxtest + CADDXG473
9. 外发包 `tools/SPI_LOG/packs/` 已按 v2 整理（协议/ESP32工程/BF驱动源码），
   旧 data/ 目录已删除；packs 里的 G473 hex 是用户自放的旧产物，注意甄别

**v2.1 单向流已实现（2026-10-20 晚，按用户需求再简化）：**
- 需求变更：不需要 SOC ACK——BF 只管把 log 流推上 SPI，VTX 检测到数据就收，
  满了自行丢弃，导出 VTX 自理，不经 Configurator
- ascent.c 收敛为 ~200 行单向流驱动：DATA(0x5A10) 唯一必需命令；
  PING/STATUS 降级为可选诊断（BF 不使用）；删光应答扫描/重试/超时/探活
- 帧间隔 ASCENT_FRAME_GAP_US=3ms（SOC 落盘节奏，可调大换可靠）
- 单测 11/11 PASS（含 SOC 挂死不阻塞场景）
- 双目标构建通过；ESP32 现有 v2 从机线上完全兼容，无需重刷
- packs/ 已刷新为 v2.1 协议 + 最新两侧源码；ESP32参考实现/README 说明导出方法
- **待用户**：✅ 已手动刷入 v2.1 固件（17:47 构建）。注意：我此前一次 DFU 烧录烧进了坏构建
  （开机即崩，MSP 全死），用户重刷 17:47 构建后正常——发版前务必自测 MSP 响应
- 注意：刷机会重置配置，`blackbox_device` 默认即 ASCENT，但 `blackbox_mode` 默认 NORMAL
  （解锁才记录）。bench 验证可临时 `set blackbox_mode = ALWAYS`，测完恢复 NORMAL
- **已知问题：F405 的 VCP 在脚本反复 open/close + reboot 后会楔死**（端口在、MSP/CLI 全哑），
  拔插 USB 或 Configurator 重连可恢复。远程调试脚本应最小化开关口次数。
  bench 验证推荐全程用 Configurator 的 CLI 操作（稳定），ESP offset 验证由脚本读独立串口

**2026-10-21 较早现场记录：ALWAYS 死机（已由顶部修复替代）**
- 时间线：栈段修复构建刷入 → CLI 设 ALWAYS+save 成功 → 20s 内 MSP 全死，
  90s 采样无自愈；**ESP offset=0（独立证据：死机发生在第一帧到达 SOC 之前）**
- 已确认事实：① 死机与首帧 DMA 同时发生；② ALWAYS 已持久化 → 每次上电复现；
  ③ Configurator 重刷（配置重置回 NORMAL）即复活——用户两次以此解救
- **定位决策：改轮询 SPI 发送**（spiReadWriteBuf 阻塞式，单帧 ≤0.5ms 有界），
  完全绕开 DMA 异步层。轮询版不挂 = DMA 问题实锤且直接绕过；轮询版也挂 = 更深层问题
- **本轮按用户指示不动代码**：轮询版改动已实施后又回滚，当前树保持 DMA 版
  （与板上 17:47 固件同源）。下次继续时的实施方案已明确：
  删 SENDING 态与 spiSequence/segments，改 `spiReadWriteBuf` 轮询直发（帧长 ≤268B，约 0.5ms），
  单测后端从 spiSequence 桩换 spiReadWriteBuf 桩，场景去掉丢请求/重试相关项
- FC 死机时的恢复流程：按 BOOT → DFU → Configurator 重刷（配置重置 NORMAL）→ 复活

## 一、项目目标

MCU(BF) = SPI 主机 ↔ SOC(ESP32-S3) = SPI 从机，链路承载 MSP v2 帧。
blackbox 新增 `BLACKBOX_DEVICE_ASCENT`，SOC 对 BF 呈现为"逻辑 flash 驱动"，
上层 flashfs / configurator / MSC 全复用，configurator 零改动。
最终效果：飞行记录 log 落 SOC，事后可读出。

历史：伪装 SPI FLASH / 伪装 SPI SD 两方案已否决（协议层零等待时序不可行），
文档源码包也已删除。需求见同目录 `ASCENT_需求.txt`，
ESP 测试固件见 `ESP32blackboxtest/`（README.txt 是 SPI 事务契约 + 命令集权威）。

## 二、硬件接线（实测可用，勿动）

| F405 | ESP32-S3 | 信号 |
|------|----------|------|
| PB3 (SPI3_SCK)  | GPIO12 | SCK |
| PB4 (SPI3_SDI)  | GPIO11 | MISO（ESP→BF）|
| PB5 (SPI3_SDO)  | GPIO10 | MOSI（BF→ESP）|
| PC14 (ASCENT_SPI_CS) | GPIO13 | CS |

ESP 固件引脚宏：MOSI=10 / MISO=11 / SCK=12 / CS=13。1MB SPI 起频（`ASCENT_SPI_SPEED_HZ`），链路通后再上调。

## 三、BF 侧改动清单（全部只增不改）

- `src/config/configs/CADX/CADDXF405WING-balckboxtest/config.h`
  用户自建测试目标。关键行：`USE_FLASH_CHIP` + `USE_FLASHFS` + `USE_ASCENT` + `USE_ASCENT_SPI`、
  四个 `ASCENT_*_PIN`、`ASCENT_SPI_INSTANCE SPI3`、`DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_ASCENT`
  （SDCARD 相关已删。注意：USE_FLASH_CHIP/USE_FLASHFS 曾被误删导致全程不通，见第四节）
- `src/main/blackbox/blackbox.h`：枚举加 `BLACKBOX_DEVICE_ASCENT = 4`，VIRTUAL 挪到 5（settings lookup 表顺序必须一致）
- `src/main/cli/settings.c`：lookupTableBlackboxDevice 加 `"ASCENT"`（SERIAL 之后）
- `src/main/drivers/ascent.c` / `ascent.h`（新）：核心驱动，见下
- `src/main/blackbox/blackbox_io.c`：全部 `case BLACKBOX_DEVICE_FLASH:` 前叠 `#ifdef USE_ASCENT case BLACKBOX_DEVICE_ASCENT`
- `src/main/blackbox/blackbox.c`：blackboxValidateConfig 同样叠 case
- `src/main/drivers/flash/flash.c`：flashDeviceInit 里 ASCENT 模式走 `ascentInit()`，绕过硬件探测；注意 UNUSED(flashConfig) 守卫不要加 USE_ASCENT 条件
- `src/main/drivers/resource.h|.c`：`OWNER_ASCENT` + ownerNames 表（必须同步，有 STATIC_ASSERT）
- `mk/source.mk`：FLASH_SRC 加 `drivers/ascent.c`（FLASH_SRC 无条件编译，文件内 `#ifdef USE_ASCENT` 自守卫）

ascent.c 设计：
- 同步事务模型：请求帧一次突发发完 → 等 ≥2ms → 512B 轮询突发找 `$X>` 应答帧（帧不跨 CS 突发）
- 命令 ID 0x5A01-0x5A06（PING/GEOMETRY/READ/WRITE/ERASE_SECTOR/ERASE_ALL），与 ESP 固件一致，改动须两侧同步
- 实现 flashVTable → flashfs 无感接入；读写 256B 分片；ERASE_ALL 为 fire-and-forget（ESP 后台擦）
- 跨任务锁（ascentBusy，500ms 超时）；SOC 缺席仅设备不可用，不阻塞调度（需求红线）
- 调试打点直写 `debug[0..2]`（init 步骤/PING 次数/PING 结果），MSP_DEBUG(254) 可读

## 四、已解决的问题（踩坑记录，按时间序）

### ESP 侧
1. **ESP32-S3 芯片修订 v0.2 不支持 octal PSRAM**：检测能过、一进调度就静默挂死 → RTC WDT 复位循环。
   已放弃 PSRAM，改用 ESP 自带 16MB flash 的 `storage` 分区（14.6MB，partitions.csv），
   `esp_partition_read/write/erase_range` 实现，天然 NOR 语义 + 断电数据保留
2. **sdkconfig.defaults 改动不生效**：必须 `rm sdkconfig` + `idf.py set-target esp32s3` 重新生成
3. **IDF 是 EIM 布局**：`export.sh` 不可靠，`IDF_PYTHON_ENV_PATH` 单独设也不够，
   工具链发现靠 `bash -c '. /Users/cr/.espressif/tools/activate_idf_v5.4.3.sh && idf.py ...'`
4. **xtensa 下 uint32_t = unsigned long**：日志 `%u` 直接 -Werror，参数加 `(unsigned)` 强转
5. **IDF 5.4 的 `spi_slave_transaction_t` 没有 `rxlength` 成员**
6. ESP 构建产物：`build/esp32blackboxtest.bin`，串口 `/dev/cu.usbserial-A5069RR4`（bridge 口，esptool 在此口）
7. 串口读日志：DTR/RTS 复位序列 + 预启动读线程；看到 `Multicore app` 后静默 = 启动即挂
   （当时即 PSRAM 问题）

### BF 侧
8. **config.h 丢 `USE_FLASH_CHIP`/`USE_FLASHFS`**（最大坑）：flash.c 整个文件包在
   `#ifdef USE_FLASH_CHIP` 里，宏一丢整个 flash 子系统不编译，ascentInit 永不执行，
   MSP_DEBUG 全零、dataflash ready=0。已补回。验证手段：用真实编译 flags 预处理探针文件
9. **ascent.c 把 `#ifdef USE_ASCENT` 放在 `#include "platform.h"` 之前**：config.h 由
   platform.h 引入，守卫先于宏定义 → 整个文件被剔空 → 链接期 `undefined reference to ascentInit`
10. ascent.h 必须先 `flash.h` 再 `flash_impl.h`（后者依赖前者类型）
11. `pageProgramContinue` 的 `bufferSizes` 是 `const uint32_t *`，不能原地推进，改用 bufIdx/bufOff 游标
12. **`SPI_CFG_TO_DEV(SPI3)` 是错的**：SPI3 是外设指针宏，不是编号。正确链路：
    `SPI_DEV_TO_CFG(spiDeviceByInstance((const spiResource_t *)ASCENT_SPI_INSTANCE))`
13. **MSP_DEBUG(254) 只回 `debug[0..3]`**，且 `DEBUG_SET` 受 `debug_mode` 设置门控（默认 NONE 全吞）。
    要远程可读：直写 `debug[]` 全局数组（extern int16_t debug[]），不走 DEBUG_SET
14. macOS 自带 nm/objdump 读不了 LTO 对象文件（报 plugin needed 或静默空输出），
    用 `arm-none-eabi-nm`；V=1 抓真实编译行做预处理探针是验证宏的最可靠手段
15. 首次构建过 ≠ 宏链路对：链接成功只说明符号齐，不说明 ascent 路径被执行
16. **CS 引脚宏名是 `ASCENT_SPI_CS_PIN`**（不是 ASCENT_CS_PIN）：ascent.c 只认这一个名字，
    宏名拼错报 `DEFIO_TAG__ASCENT_SPI_CS_PIN undeclared`。G473 接入时踩过（已修，
    `make CONFIG=CADDXG473`；G4 平台 target.h 默认已开 USE_SPI_DEVICE_3，无需补）
17. **致命坑：异步 spiSequence 的段数组不能放栈上**——DMA 完成前函数已返回，栈段失效
    → 硬错误死机，且按配置每上电必复现（ALWAYS 模式开机即发帧）。段表必须文件级 static
    （m25p16 同款注释："The segment list cannot be in automatic storage..."）。
    v2.1 重写时引入、已修复；此类死机特征：设 ALWAYS 后卡死、重启复现、拔插无效
18. **Configurator/DFU 刷机会重置配置**：blackbox_mode 回 NORMAL（device 回默认 ASCENT）。
    bench 验证用 ALWAYS 时，测完记得恢复；刷机后 offset/配置"丢失"先想到这一条
19. **isBlackboxDeviceFull / blackboxDeviceReserveBufferSpace 的 ASCENT 分支语义**：
    reserve 装得下必须返回 SUCCESS（曾误写永久 TEMPORARY → SEND_HEADER 永远卡死、
    零写入）；isFull 对流式设备必须返回 false（回 true 则 ALWAYS 永不开始）

### 烧录
16. F405 烧录路径：VCP 发 MSP_REBOOT 无效（见第五节#2）→ **按住 BOOT 键重插 USB** 进 ROM DFU，
    `dfu-util -a 0 -s 0x08000000:leave -D obj/betaflight_CADDXF405WING-balckboxtest.bin`
    （hex 不行，dfu-util 要 .bin；`arm-none-eabi-objcopy -O binary` 从 elf 生成）
17. F405 的 VCP 频繁 open/close 会抽风（MSP 时通时断），等 10s 左右自愈；脚本里保持端口打开、少折腾

## 五、未解决问题

1. **ALWAYS 模式死机已修复**：实际源码仍残留栈上 DMA 段表，已修为 static 并通过实板回归。
   保留异步 DMA，未采用此前讨论的阻塞轮询方案。证据与验证范围见顶部。
2. **MSP_REBOOT(248) payload=1 远程进 DFU 仍未验证成功**：注意手拼 MSP 帧 CRC 是
   `len^cmd^payload`，此前多次测试 CRC 算错导致结论不可信；重测须用 msp_tool 的
   MSP 类（自动算 CRC）。且**串口严格独占**：两个进程同时开 VCP 会互相打死
3. flashfs used-size 记账可能略超容量（实测超 122 字节），msp_tool 导出长度已做
   `min(used, capacity)` 钳制；根因未查（疑似写缓冲跨界记账），无功能影响
4. CLI 进去必须 `exit` 退出，否则挡 MSP

## 五点五、2026-10-20 会话新增（前一会话完成）

- ascent.c 重写为异步状态机（`ascentUpdate()` 挂 taskMain）：busSegment DMA、
  限时重试、STATUS(0x5A07) 轮询擦除进度、SOC 应答缓存重放安全；SPI 实测 5.25MHz
- flashfs：ASCENT 模式写缓冲 2048/自动冲刷 256B
- 全套挂接：init.c(MSC+emfat)、usb_msc、cli(flash_erase 可用性检查)、cms、
  msp(DATAFLASH_READ 播放门控)
- 单测：`python3 src/test/unit/ascent_test.py`（把生产 ascent.c + ascent_slave.c
  用 stub 后端编译跑 8 个场景，ASan/UBSan）——当前 8/8 PASS
- 工具：`tools/SPI_LOG/msp_tool.py`（summary/debug/导出 dataflash）；
  解码器 `tools/SPI_LOG/blackbox_decode`（源码构建：github betaflight/blackbox-tools，make）

## 六、当前状态（2026-10-20 11:10）——v1 块设备方案实测通过（已按 v2 转向，下列为历史基线）

- ESP32-S3：正常（storage 14.6MB，selftest PASS）
- F405：ASCENT ready（debug=(7,1,1) 首试 PING 成功）
- **`blackbox_mode = NORMAL`**（已从 ALWAYS 恢复默认：解锁才记录，save 并重启验证）
- **flash 已擦空**：`flash_erase` 完成，MSP used=0；flash_info 显示
  JEDEC 0x41534301 / 3567 扇区 / 14,610,432B
- msp_tool.py 增强：导出长度钳制、请求级重试（4 次）、**断点续传 + 端口被占自动等待**；
  曾修两个自身 bug（mkdir/exists 未判空）
- 解码验证存档：1.7MB 片段解码 59,991 帧（817Hz / 23.8KB/s，8MHz 相比 1MHz 提升约 3.4 倍），
  gyro/acc/vbat/baro/PID/motor 数据真实；解码器 `tools/SPI_LOG/blackbox_decode`
- 历史导出：用户曾手动全量导出并确认数据；机器自动全量导出因 14.6MB 耗时过长放弃，
  需要时可再跑 `msp_tool.py --export <path>`（支持断点续传）
- 剩余可做：
  1. **试飞验证**（RX 之前未接，bench 数据 gyro 平静属正常）
  2. MSP_REBOOT 进 DFU 重测（CRC 须用 msp_tool 的 MSP 类；此前手拼帧 CRC 算错污染结论）
  3. 满片回绕/长飞压力摸底
  4. flashfs used-size 记账可超容量 122 字节（已钳制，根因未查）

## 七、环境速查

**SOC 工程师交接包**：`tools/SPI_LOG/packs/`
（v2.1 单向流协议 SPI_MSP协议.txt + ESP32 参考工程 + BF 主机侧 ascent.c/h，
随实现同步更新）

```
# ESP 构建/烧录（必须走 activate 脚本）
cd tools/SPI_LOG/ESP32blackboxtest
bash -c '. /Users/cr/.espressif/tools/activate_idf_v5.4.3.sh >/dev/null 2>&1;
  idf.py set-target esp32s3 && idf.py build && idf.py -p /dev/cu.usbserial-A5069RR4 flash'

# BF 构建
cd /Users/cr/Works/betaflight
make CONFIG=CADDXF405WING-balckboxtest
arm-none-eabi-objcopy -O binary obj/main/betaflight_STM32F405_CADDXF405WING-balckboxtest.elf \
  obj/betaflight_CADDXF405WING-balckboxtest.bin

# F405 烧录：BOOT 键 + 重插 USB → DFU
dfu-util -a 0 -s 0x08000000:leave -D obj/betaflight_CADDXF405WING-balckboxtest.bin

# 串口
ESP32: /dev/cu.usbserial-A5069RR4 (115200, 有日志)
F405:  /dev/cu.usbmodem0x80000001 (VCP, MSP/CLI)
```

验证用 MSP：BLACKBOX_CONFIG(80)/SET(81)、DATAFLASH_SUMMARY(70)/READ(71)、DEBUG(254)。
CLI 可从 VCP 进（发 `\n#\n`），`flash_info`/`get blackbox_device` 可用。
