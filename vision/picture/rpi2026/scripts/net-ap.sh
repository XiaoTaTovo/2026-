#!/usr/bin/env bash
# 建 WiFi 热点，供 ESP32-CAM 连入。幂等，重复跑只会更新参数。
#
# 为什么用 nmcli 而不是 hostapd+dnsmasq：
#   本机是 Debian + NetworkManager（内核 6.12 rpt-rpi-v8）。NM 的
#   `ipv4.method shared` 会自动起 dnsmasq 发 DHCP 并做 NAT，一条命令搞定，
#   比手写 hostapd.conf + dnsmasq.conf + dhcpcd 排除项少一半出错面。
#
# 网段固定 10.42.0.1/24（NM shared 模式的默认），相机会拿到 10.42.0.x。
set -euo pipefail

CON=ap-ballcam
SSID="${SSID:-BALLCAM_H}"
PSK="${PSK:-ballcam2026}"
CH="${CH:-6}"
IFACE="${IFACE:-wlan0}"

echo "== 检查 WiFi 国家码（★不设国家码 wlan0 会被 rfkill 锁死，AP 起不来，这是最经典的坑）"
REG="$(iw reg get 2>/dev/null | awk '/country/{print $2; exit}' | tr -d ':')"
echo "   当前 regulatory domain = ${REG:-未设置}"
if [ "${REG:-00}" = "00" ] || [ -z "${REG:-}" ]; then
  echo "   ✖ 未设置。跑一次： sudo raspi-config → 5 Localisation → WLAN Country → CN，然后重启"
  echo "     （也可临时： sudo iw reg set CN）"
fi
sudo rfkill unblock all || true

echo "== 建/更新连接 ${CON}"
if ! nmcli -t -f NAME con show | grep -qx "${CON}"; then
  sudo nmcli con add type wifi ifname "${IFACE}" con-name "${CON}" ssid "${SSID}"
fi
sudo nmcli con mod "${CON}" \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  802-11-wireless.channel "${CH}" \
  802-11-wireless.ssid "${SSID}" \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "${PSK}" \
  ipv4.method shared \
  ipv6.method disabled \
  connection.autoconnect yes \
  connection.autoconnect-priority 20

echo "== 启动"
sudo nmcli con up "${CON}"
sleep 2
nmcli -f GENERAL.STATE,IP4.ADDRESS con show "${CON}" || true
ip -4 addr show "${IFACE}" | sed -n 's/.*inet \([0-9.\/]*\).*/   本机 AP 地址: \1/p'

cat <<EOF

== 完成
   SSID      ${SSID}
   密码      ${PSK}
   信道      ${CH}
   网段      10.42.0.1/24（相机会拿到 10.42.0.x，后端自动发现，不用手填）

   ESP32-CAM 固件里填这个 SSID/密码即可，务必设成【无限重连】——
   树莓派热点要 8~15s 才起来，相机 2s 就启动，第一次必然连不上，必须自己重试。

   改信道： CH=11 $0        （或直接在触摸屏设置页点「扫描并推荐空闲信道」）
   关热点： sudo nmcli con down ${CON}
   看谁连了： cat /var/lib/NetworkManager/dnsmasq-${IFACE}.leases
EOF
