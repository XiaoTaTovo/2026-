#!/usr/bin/env bash
# 给 eth0 加一个固定的备用地址 192.168.50.1/24，用于「网线直连笔记本 → SSH」。
#
# 为什么这么配（关键）：
#   用的是 ipv4.method auto **同时** 挂 ipv4.addresses 192.168.50.1/24。
#   NetworkManager 在 auto 模式下也会把手动列出的地址一并配上，于是同时得到：
#     · 插路由器/交换机 → 照常拿 DHCP，能上网、能 apt install
#     · 直连笔记本（对面没有 DHCP）→ 仍然有 192.168.50.1 这个固定入口
#   如果写成 method manual，插到有 DHCP 的网络里就上不了网了，装包会很难受。
#
# mDNS（ssh pi@xiaotpi.local）在 Win11 上已实测可用，优先用它；
# 这个静态地址是兜底：换了队友的机器、mDNS 不灵、或者环境变了，还有一条死路子。
set -euo pipefail

IFACE="${IFACE:-eth0}"
ADDR="${ADDR:-192.168.50.1/24}"

CON="$(nmcli -t -f NAME,DEVICE,TYPE con show --active | awk -F: -v i="$IFACE" '$2==i && $3=="ethernet"{print $1; exit}')"
if [ -z "${CON}" ]; then
  CON="$(nmcli -t -f NAME,TYPE con show | awk -F: '$2=="802-3-ethernet"{print $1; exit}')"
fi
if [ -z "${CON}" ]; then
  CON="lan-direct"
  echo "== 没有现成的以太网连接，新建 ${CON}"
  sudo nmcli con add type ethernet ifname "${IFACE}" con-name "${CON}"
fi
echo "== 使用连接: ${CON}"

sudo nmcli con mod "${CON}" \
  ipv4.method auto \
  ipv4.addresses "${ADDR}" \
  ipv4.may-fail yes \
  connection.autoconnect yes
sudo nmcli con up "${CON}" || true
sleep 1
ip -4 addr show "${IFACE}" | sed -n 's/.*inet \([0-9.\/]*\).*/   地址: \1/p'

cat <<'EOF'

== 完成。笔记本这边要做的（只做一次）：

  Windows：设置 → 网络和 Internet → 以太网 → IP 分配「编辑」→ 手动 → 打开 IPv4
     IP 地址   192.168.50.2
     子网掩码  255.255.255.0
     网关/DNS  ★留空★（留空才不会抢走笔记本 WiFi 的上网，可以一边查资料一边 SSH）

  然后：
     ping 192.168.50.1
     ssh pi@192.168.50.1
     浏览器 http://192.168.50.1:8000      ← 笔记本也能当第二块屏/备用显示装置

  日常优先用 mDNS（不用配 IP，插上网线或同热点就能连）：
     ssh pi@xiaotpi.local
     http://xiaotpi.local:8000
EOF
