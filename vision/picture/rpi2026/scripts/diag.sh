#!/usr/bin/env bash
# 一键体检：把「HANDOVER §9 要回填的信息」一次全打出来，截图发群里就行。
set -uo pipefail
hr() { printf '\n\033[1;36m── %s\033[0m\n' "$*"; }

hr "硬件与系统"
echo "   型号   $(tr -d '\0' < /proc/device-tree/model 2>/dev/null)"
echo "   系统   $(. /etc/os-release; echo "$PRETTY_NAME")"
echo "   内核   $(uname -r)  $(uname -m)"
echo "   主机名 $(hostname)"
echo "   温度   $(vcgencmd measure_temp 2>/dev/null || echo n/a)"
echo "   欠压   $(vcgencmd get_throttled 2>/dev/null || echo n/a)   （非 0x0 说明电源不够，会随机重启）"

hr "桌面 / 屏幕"
echo "   会话类型 ${XDG_SESSION_TYPE:-未知（SSH 里看不到是正常的）}"
echo "   合成器   $(ps -eo comm= | grep -m1 -E 'labwc|wayfire|openbox|mutter' || echo 未检测到)"
echo "   启动目标 $(systemctl get-default)"
echo "   seat0会话 $(loginctl list-sessions --no-legend 2>/dev/null | awk '$4=="seat0"' | wc -l)"
for p in /sys/class/drm/card*-*/status; do
  [ -e "$p" ] && printf '   %-22s %s\n' "$(basename "$(dirname "$p")")" "$(cat "$p")"
done

hr "网络"
nmcli -t -f NAME,TYPE,DEVICE,STATE con show --active | sed 's/^/   /'
ip -4 -br addr | sed 's/^/   /'
echo "   国家码 $(iw reg get 2>/dev/null | awk '/country/{print $2; exit}')  ★必须是 CN，否则 AP 起不来"
L=/var/lib/NetworkManager/dnsmasq-wlan0.leases
[ -f "$L" ] && { echo "   热点客户端（相机应该在这里）:"; sed 's/^/     /' "$L"; } \
            || echo "   （还没有 lease 文件：热点没起，或还没有设备连上来）"

hr "服务"
for s in ballcam-api NetworkManager; do
  printf '   %-16s %s\n' "$s" "$(systemctl is-active $s 2>/dev/null)/$(systemctl is-enabled $s 2>/dev/null)"
done
curl -sf http://localhost:8000/api/status >/dev/null && echo "   后端 HTTP  ✔ 通" || echo "   后端 HTTP  ✖ 不通"

hr "依赖"
for c in ffmpeg python3 chromium chromium-browser nmcli; do
  printf '   %-18s %s\n' "$c" "$(command -v $c || echo '✖ 缺')"
done
python3 -c "import fastapi,uvicorn,httpx;print('   python  ✔ fastapi/uvicorn/httpx 齐')" 2>/dev/null \
  || echo "   python  ✖ 依赖不全，跑 ./install.sh"
ffmpeg -hide_banner -encoders 2>/dev/null | grep -q h264_v4l2m2m && \
  echo "   h264硬编  可用（默认仍用 libx264，更稳）" || echo "   h264硬编  不可用，用 libx264 软编"

hr "存储"
df -h "$HOME" | sed 's/^/   /'
D="$HOME/data/ti_vision/2026"
[ -d "$D" ] && echo "   录像 $(ls "$D/meta" 2>/dev/null | wc -l) 条，占 $(du -sh "$D" 2>/dev/null | cut -f1)"

hr "相机"
IP="$(awk '{print $3}' "$L" 2>/dev/null | head -1)"
if [ -n "${IP:-}" ]; then
  echo "   探测 $IP ..."
  curl -s -m 3 "http://$IP/status" | head -c 400 | sed 's/^/   /'; echo
else
  echo "   没找到相机。先确认：相机上电了吗？固件里的 SSID/密码对吗？热点起来了吗？"
fi

hr "时间（★离线无 NTP，时间不对录像时间戳会乱，写报告对不上）"
echo "   $(date)"
timedatectl 2>/dev/null | sed 's/^/   /'
