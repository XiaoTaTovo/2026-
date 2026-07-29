# 图传链路总方案（题目要求第 1 项，6 分）

> 负责人：队友（图传专项）
> 关联：`code/vision/maxicam/`（球位检测，控制环用，与本链路**物理分离**）
> 上位文档：`code/README.md`

---

## 0. 题目原文约束（逐条对照，验收就看这几句）

| 原文 | 落到设计上 |
|---|---|
| 「装置由发送和接收模块组成」 | 发送 = ESP32-CAM（车上）；接收 = 树莓派 WiFi（场外） |
| 「发送模块须稳固安装在小车上」 | 相机 + 支架刚性固定在摆杆凹槽正上方，车身内，不超出 35×25cm |
| 「接收模块可连接显示存储装置并整体置于环形线路以外」 | 树莓派 + 触摸屏 + 电源，整体放场边桌上，**不要放进赛道** |
| 「稳定实时显示钢球在凹槽中滚动的画面」 | 端到端延迟 ≤ 300ms，整圈 30s 内不黑屏；断线自动重连 |
| 「完整记录每次测试时钢球运动的视频」 | 一次测试 = 一个文件（+ 断线分片），文件名带测试项号和时间 |
| 「能按要求回放」 | 触摸屏上按测试项检索、播放/暂停/**逐帧**/0.25x 慢放/拖进度 |
| 说明 6：「回传画面能覆盖整个摆杆，能完整清晰看到钢球的滚动运行轨迹并判定其位置」 | 视野覆盖 25cm 全长 + 刻度线可辨；见 §3 光学参数 |
| 说明 6：「图传接收模块及其连接的显示存储装置在比赛结束后和作品一起封存」 | **树莓派、屏、电源、网线赛后一起上交**，别拆回去 |
| 说明 7：「凹槽内钢球位置检测必须采用摄像头」 | 那是控制环的活（MaixCAM）。图传相机**不参与控制**，两条链路互不拖累 |
| 说明 5：「显示屏尺寸不大于 2 英寸」 | 只约束**车上**那块（OLED，主控已有）。树莓派这块屏是场外接收装置，不受限，**绝对不要装到车上** |

---

## 1. 系统架构

```
        ┌──────────── 小车（赛道内）─────────────┐
        │  摆杆凹槽                              │
        │     ▲ 约 20cm                          │
        │  [ESP32-CAM] ── WiFi 2.4G STA ──┐      │
        │   +补光LED   独立 5V 供电        │      │
        └──────────────────────────────────┼─────┘
                                           │  MJPEG over HTTP
        ┌──────────── 场外（赛道以外）──────┼─────────────────────┐
        │  [树莓派]  wlan0 = AP（发热点）───┘                     │
        │     ├─ 后端 ballcam-api : 抓流 / 录制 / 文件管理         │
        │     ├─ 前端 Chromium kiosk : 实时 + 回放 + 设置          │
        │     └─ eth0 静态 192.168.50.1 ── 网线 ── 笔记本（SSH/备份显示）│
        │  [7" 触摸屏] ← 唯一现场操作入口，不需要键盘              │
        └─────────────────────────────────────────────────────────┘
```

**关键设计取向：现场零命令行。** 交接之后我不在场，所有比赛现场可能要改的东西（WiFi 信道、相机分辨率/曝光、重启相机、重启服务、安全关机、删录像）全部做进触摸屏设置页。SSH 只是排障后手。

---

## 2. 网络：三套方案，按优先级刷好，现场随时切

现场 2.4G 频段极度拥挤（几十支队伍 + 观众手机），**这是本项最大的失分风险**，所以准备可切换的三条链路。

### 方案 A（主推）：树莓派做 AP，ESP32-CAM 当客户端

树莓派 OS Bookworm/Trixie 用 NetworkManager，一条命令建 AP：

```bash
sudo nmcli con add type wifi ifname wlan0 con-name ap-ballcam autoconnect yes \
     ssid BALLCAM_H
sudo nmcli con mod ap-ballcam \
     802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 6 \
     wifi-sec.key-mgmt wpa-psk wifi-sec.psk "ballcam2026" \
     ipv4.method shared ipv6.method disabled \
     connection.autoconnect-priority 20
sudo nmcli con up ap-ballcam
```

- `ipv4.method shared`：NetworkManager 自动起 DHCP，网段 **10.42.0.1/24**，相机会拿到 `10.42.0.x`。
- **必做**：`sudo raspi-config` → Localisation → WLAN Country = **CN**。不设国家码 wlan0 被 rfkill 锁死，AP 起不来，这是最经典的坑。
- 关蓝牙减少同频干扰：`/boot/firmware/config.txt` 加 `dtoverlay=disable-bt`。
- 相机 IP 自动发现：后端读 `/var/lib/NetworkManager/dnsmasq-wlan0.leases`，不用手填 IP。
- 代价：wlan0 做了 AP 就**不能同时连外网**。所以所有软件必须交接前装完（见 HANDOVER §2）。

### 方案 B（备用）：ESP32-CAM 自己开 SoftAP，树莓派去连它

零额外设备，连树莓派 AP 配错了都能救。缺点：ESP32 SoftAP 吞吐低、并发差。固件里预留一个拨码/长按 BOOT 键切换 STA/AP 模式。树莓派侧：

```bash
sudo nmcli dev wifi connect ESP32CAM_AP password "esp32cam" ifname wlan0
```

UI 设置页给「链路模式 A / B」两个按钮，各对应一组 nmcli profile 切换，现场 10 秒切完。

### 方案 C（WiFi 模块到货后的正式方案）：外置 AP

那个还没到的 WiFi 模块 / 随身路由，接上电就是专用 AP。树莓派和 ESP32 都作为 STA 连它（树莓派 wlan0 改 STA，或直接用网线接路由 LAN 口更稳）。此时树莓派可以恢复上网，方便临时装东西。

> 不建议用「第二块 ESP32 当 AP」中转图传——ESP32 SoftAP 转发会成为瓶颈，画面会卡。那块 ESP32 留着做备用发送模块（摔了一个还有一个）更划算。

### 现场必做：先扫信道再定信道

```bash
sudo nmcli dev wifi rescan; nmcli -f SSID,CHAN,SIGNAL dev wifi list | sort -k2 -n
```

选 1/6/11 里占用最少的那个，改 `802-11-wireless.channel` 后 `nmcli con up ap-ballcam`。UI 设置页做成「扫描 → 一键换台」按钮。

---

## 3. 相机侧参数（ESP32-CAM）

### 光学与安装

- **高度**：OV2640 默认镜头水平 FOV 约 65°，要覆盖 25cm 摆杆全长 → 距离 ≈ 25 / (2·tan32.5°) ≈ **20cm**。装 18~22cm，装好后拍一张确认两端刻度都在画面里且留 1cm 余量。
- **朝向**：正俯视，画面长边平行摆杆。摆杆在画面里尽量水平居中，方便叠加刻度参考线。
- **支架**：车在跑会抖，支架必须刚（碳杆/铝型材，别用亚克力细条），底座垫橡胶减振。题目明确要求「稳固安装」，评委会看。
- **补光**：侧向漫射白光 LED 条（打在凹槽侧壁上），**不要**用板载闪光灯正打——钢球会出高光亮斑，反而看不清位置。补光后固定曝光。
- **供电**：ESP32-CAM 对电压跌落极敏感，WiFi 发射瞬间电流尖峰会 brownout 重启。**独立 DC-DC 出 5V/2A + 输入端 470µF 电解 + 100nF**，不要和电机共用一路。

### 固件设定（CameraWebServer 类固件为基线）

| 项 | 值 | 理由 |
|---|---|---|
| framesize | `SVGA 800×600`（保底 `VGA 640×480`） | 800px / 25cm ≈ 32px/cm，0.1cm 刻度 ≈ 3px，勉强可辨；VGA 25px/cm 是下限 |
| jpeg_quality | 10~12（数字越小越清晰） | 兼顾码率，先按 12 测 |
| fps | 目标 ≥ 12，越高越好 | 30s 整圈滚动轨迹要连续 |
| AEC/AGC | **关闭**，固定 exposure + gain | 自动曝光会随车姿抖动闪烂，钢球判位会飘 |
| AWB | 关闭，固定 | 同上 |
| `WiFi.setSleep(false)` | 必须 | 否则延迟从 100ms 抖到 300ms+ |
| 断线重连 | **无限重试**，退避 1s，不要「N 次失败后 restart」 | 树莓派 AP 比相机晚起来几秒，相机必须自己等 |
| hostname | `ballcam` | 便于 `ballcam.local` 和 lease 表识别 |
| 上电即工作 | 不需要按键、不需要串口、不需要手机配网 | 自启动硬要求 |

接口约定（后端按这个对接，实际以 `/status` 探测为准）：

```
http://<CAM_IP>:81/stream                     MJPEG 流（主用）
http://<CAM_IP>/capture                       单帧 JPEG（心跳探测/截图）
http://<CAM_IP>/status                        JSON 当前参数
http://<CAM_IP>/control?var=framesize&val=9   在线改参数（UI 设置页调它）
```

---

## 4. 树莓派侧软件

### 技术栈（选它是为了开发量最小 + 触摸最顺）

- 后端：**Python 3.11 + FastAPI + uvicorn**，单文件 `app.py`。
- 前端：**Chromium kiosk + 单文件 HTML/JS**，实时用 `<img>` 吃 MJPEG，回放用 `<video>`（原生 seek / 倍速 / 逐帧）。
- 录制：**ffmpeg 一进程双输出**，不重编码那路保底，编码那路给浏览器即时回放。

### ⚠️ 架构约束：ESP32-CAM 只接受一个 `/stream` 客户端

CameraWebServer 固件**同时只允许一路 MJPEG 客户端**。所以浏览器**不能**直连相机拉流 —— 那会和录制用的 ffmpeg 抢同一路连接，结果预览和录像双双断掉。

正确做法：**后端唯一持有上游连接**，按 JPEG 的 `FFD8`/`FFD9` 标记切出完整帧，再 fan-out 给两个下游：① 浏览器预览（`/api/stream`）② 录制中的 ffmpeg（写它的 stdin）。

顺带三个好处：相机只被连一次；录制启停与前端翻页解耦（**评委让你切到回放页不会中断录像**）；断线重连逻辑集中在一处。代价是多一次内存搬运，VGA 15fps 约 450KB/s，Pi 上不到 3% CPU。

### 录制命令（核心，已实现于 `rpi2026/app.py`）

```bash
ffmpeg -hide_banner -loglevel error \
  -f mjpeg -use_wallclock_as_timestamps 1 -i pipe:0 \
  -map 0:v -c copy   -f matroska  raw/T4_20260731_142513_part1.mkv \
  -map 0:v -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p \
          -g 15 -movflags +frag_keyframe+empty_moov \
          -f mp4       mp4/T4_20260731_142513_part1.mp4
```

输入是 `pipe:0`（后端把 hub 分发来的帧写进 stdin），不是直接连相机 —— 见上一节。

设计理由（报告里可以直接写）：

1. **`-c:v copy` → mkv**：原始 JPEG 帧零重编码、零画质损失、CPU 几乎为 0，mkv 容错强，掉电也能修。这是"完整记录"的保底证据。
2. **h264 fragmented mp4**：`+frag_keyframe+empty_moov` 让文件**边写边可播**，录制中断电也不会变成废文件。评委一句「回放刚才那次」就能立刻放，**不用等转码**——这是为什么不采用「先存 mkv 再后台转码」。
3. `-g 15`（每秒一个 I 帧）：逐帧步进和拖进度条才顺滑。
4. `-use_wallclock_as_timestamps 1`：时间轴对齐真实墙钟，回放时间 = 测试真实用时。
5. **不烧字幕水印**：drawtext 吃 CPU 且损画质。时间/帧号由 UI 覆盖层显示，更清晰也更准。
6. 每次录制同时写 sidecar `T4_20260731_142513.json`：测试项、起止 wallclock、帧数、平均 fps、丢帧数、分片列表、人工标记点、备注。设计报告"测试数据完整性"那 4 分靠它。

### 断线不丢录像

ffmpeg 输入断流会退出。后端 supervisor 检测到「进程退出 && UI 仍处于 REC 状态」→ 立即重启录一个 `..._part2`，sidecar 记下分片列表；回放页把同一次测试的多分片当一条连续记录顺序播放。这样"相机中途掉线"也仍然满足「完整记录每次测试」。

### 存储估算

VGA q12 MJPEG ≈ 22MB/min + h264 ≈ 15MB/min ≈ **37MB/min**。整天调试 60 分钟净录制 ≈ 2.2GB。32GB 卡足够，UI 顶栏常显剩余空间，低于 2GB 变红并提示清理。

### 目录约定

代码区和数据区分开（沿用本机既有约定，录像不进 git）：

```
~/workspace/ti-cup/vision/2026/     代码区（app.py / web / scripts / conf）
~/data/ti_vision/2026/
├── mp4/     可直接回放（h264 fragmented）
├── raw/     原始 MJPEG 保底（mkv）
├── meta/    *.json sidecar
├── shots/   截图
└── logs/    ffmpeg 日志
```

---

## 5. 自启动（三件事都要自己活过来）

| 层 | 机制 | 校验方法 |
|---|---|---|
| ① WiFi AP | `nmcli con mod ap-ballcam connection.autoconnect yes` | 冷启后 `nmcli con show --active` 有 ap-ballcam |
| ② 后端 | `ballcam-api.service`：`Restart=always`, `RestartSec=2`, `After=NetworkManager.service` | `systemctl is-active ballcam-api` |
| ③ 前端 kiosk | 桌面会话 autostart（见下）+ 自动登录桌面（`raspi-config` → System → Boot/Auto Login → **Desktop Autologin**） | 冷启后自己出画面 |
| ④ 相机 | 固件上电即连、无限重连 | 断电重上电 15s 内画面恢复 |
| ⑤ 不熄屏 | `raspi-config` → Display → Screen Blanking = **off** | 静置 20 分钟屏幕仍亮 |
| ⑥ 时间 | 离线无 NTP，靠 fake-hwclock；**建议加 DS3231 RTC**（¥10），否则录像时间戳每次开机会回退 | 冷启后 `date` 正确 |

kiosk 启动条目按桌面实际类型放（先 `echo $XDG_SESSION_TYPE`、`ps -e | grep -E 'labwc|wayfire|openbox'` 判断）：

- labwc：`~/.config/labwc/autostart`
- wayfire：`~/.config/wayfire.ini` 的 `[autostart]`
- X11/LXDE：`~/.config/autostart/ballcam-ui.desktop`

启动行：

```
chromium-browser --kiosk --noerrdialogs --disable-infobars \
  --disable-session-crashed-bubble --check-for-update-interval=31536000 \
  --autoplay-policy=no-user-gesture-required http://localhost:8000
```

UI 里的关机/重启按钮需要一条免密 sudo（`/etc/sudoers.d/ballcam`）：

```
pi ALL=(ALL) NOPASSWD: /sbin/poweroff, /sbin/reboot, /usr/bin/systemctl restart ballcam-api, /usr/bin/nmcli
```

**启动顺序坑**：树莓派 AP 起来要 8~15s，相机上电只要 2s，一定会先连不上。所以相机固件必须无限重连、后端必须轮询等相机出现，两边都不能"失败即放弃"。

---

## 6. 验收测试表（第 1 项 6 分 + 报告测试数据）

| # | 项目 | 方法 | 判定 |
|---|---|---|---|
| T1-1 | 冷启动自恢复 | 只给树莓派和小车上电，全程不碰键鼠 | ≤ 60s 自动出画面并可录制 |
| T1-2 | 端到端延迟 | UI 设置页开「延迟自测」：屏幕显示毫秒级大计时器，把车上相机对着屏幕拍，画面里读数 vs 当前读数之差 | ≤ 300ms，连续 60s 无 >1s 卡顿 |
| T1-3 | 判位清晰度 | 钢球静置 -10/-5/0/+5/+10cm 各拍，回放定格读刻度 | 5 点全部可读，误差 ≤ 0.2cm |
| T1-4 | 全程覆盖 | 车跑整圈 30s | 画面始终覆盖摆杆全长；丢帧率 < 5% |
| T1-5 | 录制完整性 | 每次测试一个文件 | 帧数 ≈ fps×时长；能在树莓派上直接播、逐帧、0.25x 慢放 |
| T1-6 | 断线恢复 | 录制中断相机电 3s 再上电 | 画面自动恢复；生成 part2 且可连续回放 |
| T1-7 | 掉电安全 | 录制中直接拔树莓派电源 | 重启后 mp4 仍可播放（验证 fragmented 有效） |
| T1-8 | 干扰余量 | 现场开一堆手机热点后重测 T1-2/T1-4 | 换信道后指标仍达标 |

T1-2 / T1-4 的数字直接进设计报告"测试方案与测试结果"。

---

## 7. 待队友确认/补齐的信息（第一天先填掉）

- [ ] 树莓派型号（`cat /proc/device-tree/model`）
- [ ] 系统版本（`cat /etc/os-release`）→ 决定 NetworkManager 还是老 hostapd 路线
- [ ] 桌面会话类型（`echo $XDG_SESSION_TYPE`）→ 决定 kiosk 放哪
- [ ] 屏幕分辨率（`wlr-randr` 或 `xrandr`）→ UI 断点，见 `UI_DESIGN.md`
- [ ] ESP32-CAM 具体型号/固件（`/status` 返回什么）
- [ ] SD 卡剩余容量、是否有备份卡
- [ ] 是否加 DS3231 RTC
