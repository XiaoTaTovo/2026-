# ballcam — 树莓派图传接收端（代码区）

**Pi 上的位置**：`~/workspace/ti-cup/vision/2026/`（代码区）
**数据落在**：`~/data/ti_vision/2026/`（数据区，录像不进代码区）
**上游文档**：`Desktop/2026/code/vision/picture/{README,UI_DESIGN,HANDOVER}.md`

```
2026/
├── app.py                  后端（单文件，~350 行）
├── web/index.html          触摸屏 UI（单文件，无外部依赖）
├── conf/ballcam.env         配置（相机 IP / 编码器 / 帧率 / 路径）
├── install.sh              一键安装（★必须在有网时跑）
├── systemd/ballcam-api.service
├── sudoers.d-ballcam       免密 sudo 白名单（换信道/重启/关机按钮要用）
└── scripts/
    ├── net-ap.sh           建 WiFi 热点
    ├── net-eth-direct.sh   eth0 备用固定 IP（网线直连 SSH）
    ├── kiosk-install.sh    开机自动全屏 UI
    ├── screen-fix.sh       屏幕不亮：诊断 + 修复
    └── diag.sh             一键体检
```

---

## 首次部署：六条命令

在**有网**的环境下（wlan0 连着路由或插网线），SSH 进 Pi：

```bash
cd ~/workspace/ti-cup/vision/2026
chmod +x install.sh scripts/*.sh          # ① 传上来的文件默认没有执行位
./install.sh                              # ② 装包 + 建服务 + 起后端（有网时跑）
./scripts/net-ap.sh                       # ③ 建热点 BALLCAM_H（跑完 wlan0 就没外网了）
./scripts/net-eth-direct.sh               # ④ eth0 加备用地址 192.168.50.1
./scripts/kiosk-install.sh                # ⑤ 开机自动全屏 UI
./scripts/diag.sh                         # ⑥ 体检，截图发群
```

**顺序不能换**：`install.sh` 要联网装包，`net-ap.sh` 一跑 wlan0 就变成热点、外网断掉。装漏了东西就得先 `nmcli con down ap-ballcam` 恢复上网。

`install.sh` 之后还差两个 `raspi-config` 里的开关（脚本改不了，必须手点）：

```
sudo raspi-config
  1 System Options   → S5 Boot/Auto Login  → B4 Desktop Autologin   ← 没这个开机停在登录界面，UI 出不来
  2 Display Options  → Screen Blanking     → No                     ← 没这个十几分钟熄屏
  5 Localisation     → WLAN Country        → CN                     ← 没这个 wlan0 被 rfkill 锁死，热点起不来
```

设完 `sudo reboot`，然后**只给电、不碰键鼠**，60 秒内应自动出现全屏 UI。

---

## 日常指令格式

```bash
# —— 服务 ——
systemctl status ballcam-api           # 看状态
sudo systemctl restart ballcam-api     # 重启后端（改完 app.py / conf 要重启）
journalctl -u ballcam-api -f           # 看实时日志（报错都在这）
journalctl -u ballcam-api -n 50        # 看最近 50 行

# —— 前台调试（改代码时用这个，比看日志快）——
sudo systemctl stop ballcam-api
python3 app.py                         # Ctrl+C 退出；报错直接打在终端
sudo systemctl start ballcam-api

# —— 热点 ——
nmcli con show --active                        # 看 ap-ballcam 在不在
sudo nmcli con up ap-ballcam                   # 起热点
sudo nmcli con down ap-ballcam                 # 关热点（要联网装包时用）
CH=11 ./scripts/net-ap.sh                      # 换到 11 信道
cat /var/lib/NetworkManager/dnsmasq-wlan0.leases   # 看谁连上来了（相机的 IP 在这）

# —— 相机 ——
curl -s http://10.42.0.23/status                             # 看当前参数
curl -s "http://10.42.0.23/control?var=framesize&val=9"      # 改成 SVGA 800x600
curl -s "http://10.42.0.23/control?var=quality&val=12"       # 改画质（数字越小越清晰）
curl -s "http://10.42.0.23/control?var=aec&val=0"            # 关自动曝光（必做）
ffplay http://10.42.0.23:81/stream                           # 脱离本系统单独验证图传通不通

# —— UI ——
./scripts/kiosk-run.sh                 # 手动起一次全屏 UI（不用重启）
pkill chromium                         # 退出 kiosk
# 笔记本上看：http://xiaotpi.local:8000  或  http://192.168.50.1:8000

# —— 录像 ——
ls -lh ~/data/ti_vision/2026/mp4/      # 可直接回放的（h264）
ls -lh ~/data/ti_vision/2026/raw/      # 原始 MJPEG 保底（mkv）
cat ~/data/ti_vision/2026/meta/T4_*.json   # 元数据：帧数/fps/丢帧/标记点

# —— 屏幕/体检 ——
./scripts/screen-fix.sh                # 只诊断，不改任何文件
./scripts/screen-fix.sh --apply        # 确认建议后再跑，会改启动参数（自动备份）
./scripts/diag.sh

# —— 关机（★永远不要直接拔电）——
sudo poweroff                          # 或在 UI 设置页长按「安全关机」
```

### 从 Windows 传代码上来

**首次上传**（目标目录还不存在，先建）：

```powershell
ssh pi@xiaotpi.local "mkdir -p ~/workspace/ti-cup/vision/2026"
scp -r "C:\Users\taowz\Desktop\2026\code\vision\picture\rpi2026\*" pi@xiaotpi.local:~/workspace/ti-cup/vision/2026/
ssh pi@xiaotpi.local "cd ~/workspace/ti-cup/vision/2026 && chmod +x install.sh scripts/*.sh && ls -R"
```

> PowerShell 里 `scp -r ...\rpi2026\*` 的通配符由 PowerShell 展开，会把**目录内容**铺到目标目录（而不是套一层 `rpi2026/`）。这正是我们要的效果。

**后续增量同步**（只改了一个文件就单传，最省事）：

```powershell
scp "C:\Users\taowz\Desktop\2026\code\vision\picture\rpi2026\app.py" pi@xiaotpi.local:~/workspace/ti-cup/vision/2026/app.py
ssh pi@xiaotpi.local "sudo systemctl restart ballcam-api"
```

只改了一个文件就单传：

```powershell
scp "C:\Users\taowz\Desktop\2026\code\vision\picture\rpi2026\web\index.html" pi@xiaotpi.local:~/workspace/ti-cup/vision/2026/web/index.html
# 前端是静态文件，不用重启服务，触摸屏上下拉刷新或 pkill chromium 重开即可
```

---

## 关键设计决策（改代码前先看这段，省得踩回同样的坑）

### 1. 浏览器不直连相机，一律走后端中继

ESP32-CAM 的 CameraWebServer 固件**同时只接受一个 `/stream` 客户端**。如果让浏览器 `<img>` 直连相机、录制的 ffmpeg 再连一次，两边会互相踢，结果是预览和录像都断。

所以 `app.py` 里的 `Hub` 唯一持有上游连接，按 JPEG 的 `FFD8`/`FFD9` 标记切出完整帧，再分发给预览客户端和 ffmpeg 的 stdin。

顺带三个好处：相机只被连一次；**录制启停和翻页互不影响**（评委让你切到回放页不会中断录像）；断线重连逻辑只有一处。代价是多一次内存搬运，VGA 15fps 约 450KB/s，Pi 上不到 3% CPU。

### 2. ffmpeg 一进程双输出

```
-c copy      → mkv    原始 JPEG 零重编码、零画质损失、CPU 近 0，掉电能修 → 保底证据
-c:v libx264 → mp4    +frag_keyframe+empty_moov = 边写边可播，掉电不成废文件
```

为什么不「先存 mkv，停止后再转码」：评委说「回放刚才那次」时**不能让人等转码**。所以录制当场就产出可直接播的 mp4。

`-g 15`（每秒一个 I 帧）是为了回放的逐帧步进和拖进度条够顺 —— 逐帧定格读刻度是「按要求回放」真正的得分点。

### 3. 断线自动续录分片

ffmpeg 的输入断流会导致它退出。`_watch_loop` 检测到「进程退出且仍在 REC 状态」就立刻开 `_part2` 续录，sidecar 记下分片列表，回放时前端顺序播完所有分片、对使用者透明（列表上标 `⚠2分片`）。这样相机中途掉线也仍然满足题目「完整记录每次测试」。

### 4. 相机 IP 自动发现

读 NetworkManager 共享模式的 DHCP lease 表（`/var/lib/NetworkManager/dnsmasq-wlan0.leases`），逐个探 `/status`。相机重连拿到新 IP 也能自己找到，队友永远不用手填。想固定就在 `conf/ballcam.env` 里填 `CAM_HOST=`。

### 5. eth0 用 `method auto` + 附加静态地址

见 `scripts/net-eth-direct.sh` 顶部注释：这样插路由器能拿 DHCP 上网装包，直连笔记本又永远有 `192.168.50.1` 这个固定入口。写成 `manual` 就没法联网装包了。

### 6. 现场零命令行

换信道、改相机参数、重连相机、清理存储、重启服务、安全关机全在触摸屏设置页。理由：交接之后原作者不在场，比赛全程不该需要 SSH。SSH 只是排障后手。

---

## 改动常见项

| 想改 | 改哪 |
|---|---|
| 热点名/密码/信道 | `SSID=xx PSK=yy CH=11 ./scripts/net-ap.sh` |
| 相机固定 IP | `conf/ballcam.env` 的 `CAM_HOST=` |
| 用硬件编码省 CPU | `conf/ballcam.env` 的 `H264_ENC=h264_v4l2m2m`（Pi4 可用，偶发不兼容就换回 libx264） |
| 测试项按钮 | `web/index.html` 里的 `ITEMS` 和 `FILT` 两个数组 |
| 刻度线显示哪几档 | `web/index.html` 的 `drawScale()` 里 `[-10,-5,0,5,10]` |
| 回放倍速档位 | `web/index.html` 的 `SPD` 数组 |
| UI 配色/按钮尺寸 | `web/index.html` 顶部 `:root` 的 CSS 变量 |
| 界面端口 | `conf/ballcam.env` 的 `UI_PORT` + `scripts/kiosk-install.sh` 里的 `URL` |
