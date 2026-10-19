# ESP32blackboxtest

ESP32-S3 ASCENT SPI 从机。MSP v2 提供逻辑 NOR flash，介质为内部 16MB flash
的 storage 分区。芯片修订 v0.2，禁用 PSRAM。

## 接线与串口

| 信号 | F405 SPI3 | ESP32-S3 |
|------|-----------|----------|
| SCK | PB3 | GPIO12 |
| MISO | PB4 | GPIO11 |
| MOSI | PB5 | GPIO10 |
| CS | PC14 | GPIO13 |

共地，3.3V，mode 0，MSB first。BF 配置上限 8MHz，F405 分频实际为 5.25MHz。
ESP 串口 /dev/cu.usbserial-A5069RR4，115200。
BF VCP /dev/cu.usbmodem0x80000001。

## 构建

    idf.py set-target esp32s3
    idf.py build
    idf.py -p <串口> flash

本机 IDF 5.4.3 环境：

    . /Users/cr/.espressif/tools/activate_idf_v5.4.3.sh

sdkconfig.defaults 配置 16MB flash、自定义分区；PSRAM 默认禁用。
storage 位于 0x210000，物理大小 14614528 字节，末尾 4096 字节专供启动自检。
对 BF 上报 14610432 字节、3567 个 4096 字节扇区。自检不会修改已上报容量内的数据。
启动必须出现 selftest PASS 和 spi slave ready。

## 协议

MSP v2：`$X dir flag cmd:u16LE len:u16LE payload crc8`。
请求方向 `<`，应答 `>`。CRC8_DVB_S2 初值 0，覆盖 flag/cmd/len/payload。
flag 是事务序号 1..255，应答原样回传。重试必须保持序号、命令和数据不变。
从机缓存最近一次成功请求及应答；完全相同的重试只重放应答，不重复写入或擦除。

1. 主机以一次 CS 突发发送完整请求，补 0xFF 到 4 字节对齐。
2. 至少等待 2ms，再以 512 字节 0xFF 突发收应答。
3. 一个应答必须完整位于一次 CS 突发中；CRC 和序号验证后才接受。
4. 未收到应答则延迟后继续轮询，每 4 次失败轮询重发原请求。
5. 相邻事务至少间隔 2ms，以便从机重新排队 DMA；每次 CS 最多 512 字节。

| ID | 命令 | 请求 | 应答 |
|----|------|------|------|
| 0x5A01 | PING | 空 | ASCENT01 (8 字节) |
| 0x5A02 | GEOMETRY | 空 | 总容量:u32LE, 扇区大小:u32LE |
| 0x5A03 | READ | 地址:u32LE, 长度:u32LE | 最多 256 字节数据 |
| 0x5A04 | WRITE | 地址:u32LE, 最多 256 字节数据 | 空 |
| 0x5A05 | ERASE_SECTOR | 对齐地址:u32LE | 空，扇区已擦完 |
| 0x5A06 | ERASE_ALL | 空 | 空，仅表示接受 |
| 0x5A07 | STATUS | 空 | 0 就绪 / 1 擦除中 / 2 擦除失败 |

全擦 ACK 发送后才启动后台任务。逐 4KB 扇区擦除并让出调度，实测约 140 秒；
BF 全擦期限 300 秒，STATUS 每 10ms 轮询。忙时只接受 PING、GEOMETRY、STATUS。
非法长度、越界、CRC 错误或存储错误不应答，主机超时后停用本次启动的记录设备。
介质保留 NOR 语义：写前擦除，擦后为 0xFF。

BF 运行期写入/擦除使用常驻 DMA 缓冲和状态机，无忙等；缺少双向 DMA 则停用记录。
flashVTable 的同步读取仅用于初始化和未解锁时导出，解锁期间拒绝读取。
启动探活上限约 2 秒；ESP 缺席时仅 ASCENT 不可用。

## 回归与导出

在仓库根目录：

    python3 src/test/unit/ascent_test.py
    python3 tools/SPI_LOG/msp_tool.py
    python3 tools/SPI_LOG/msp_tool.py --export /tmp/ascent.bfl

标准 MSP 70 查询、71 导出、72 全擦。MSP_REBOOT 是 68，payload=1 进入 ROM DFU。
测试使用 ALWAYS 模式无需解锁电机；结束后恢复 NORMAL、1/4 采样率。
