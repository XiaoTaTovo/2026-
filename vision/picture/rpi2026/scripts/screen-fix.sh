#!/usr/bin/env bash
# 屏幕插着但不亮 —— 诊断 + 修复。
#
# 默认只诊断、并尝试无损唤醒；不改任何配置文件。
# 要改启动参数才加 --apply（会先备份）。
#
# ★ 刻意不用 set -e / pipefail：这是诊断脚本，探测失败是常态（很多命令在
#   没有 X、没有 wayland、没有对应硬件时就是非零退出）。一个探测失败就整体
#   退出会让诊断变得毫无用处 —— 第一版就犯了这个错。
set -u

APPLY=0; [ "${1:-}" = "--apply" ] && APPLY=1
BOOT=/boot/firmware; [ -d "$BOOT" ] || BOOT=/boot
MODE="${MODE:-800x480M@60D}"     # 与实测面板一致；换屏要改这里
hr() { printf '\n\033[1;36m── %s\033[0m\n' "$*"; }
ok() { printf '   \033[1;32m✔\033[0m %s\n' "$*"; }
no() { printf '   \033[1;31m✖\033[0m %s\n' "$*"; }
warn() { printf '   \033[1;33m!\033[0m %s\n' "$*"; }

# ---------------------------------------------------------------- 找到本地 X

find_x() {   # 输出 "DISPLAY XAUTHORITY"，找不到则空
  local d a
  d="$(ps -eo args= | grep -oE '^/usr/lib/xorg/Xorg :[0-9]+' | grep -oE ':[0-9]+' | head -1)"
  [ -z "$d" ] && d=":0"
  for a in "$HOME/.Xauthority" /var/run/lightdm/root/"$d" /run/lightdm/root/"$d"; do
    if [ -r "$a" ] && DISPLAY="$d" XAUTHORITY="$a" xset q >/dev/null 2>&1; then
      echo "$d $a"; return 0
    fi
  done
  return 1
}
XINFO="$(find_x || true)"
XD="${XINFO%% *}"; XA="${XINFO##* }"

hr "1. 屏保 / DPMS（★最常见的原因：屏幕只是睡着了）"
if [ -n "$XINFO" ]; then
  MON="$(DISPLAY=$XD XAUTHORITY=$XA xset q 2>/dev/null | grep -oE 'Monitor is (On|Off|in .*)' | head -1)"
  echo "   X 会话 $XD   $MON"
  if echo "$MON" | grep -q 'Monitor is Off\|Monitor is in'; then
    no "屏幕被 DPMS 关掉了 —— 这就是黑屏的原因，和插拔顺序、和线材都无关"
    echo "   正在唤醒…"
    DISPLAY=$XD XAUTHORITY=$XA xset dpms force on 2>/dev/null
    DISPLAY=$XD XAUTHORITY=$XA xset s reset    2>/dev/null
    DISPLAY=$XD XAUTHORITY=$XA xset s off      2>/dev/null
    DISPLAY=$XD XAUTHORITY=$XA xset -dpms      2>/dev/null
    ok "已唤醒并临时关掉屏保。抬头看屏幕。"
    echo
    echo "   ▶ 永久生效（xset 只对当前会话有效，重启就没了）："
    echo "       sudo raspi-config nonint do_blanking 1"
    echo "     它会建 /etc/X11/xorg.conf.d/10-blanking.conf（DPMS Disable + 各 timeout=0）"
    echo "     校验： sudo raspi-config nonint get_blanking   # 输出 1 = 屏保已禁用"
  else
    ok "屏幕未被 DPMS 关闭"
  fi
  if [ -f /etc/X11/xorg.conf.d/10-blanking.conf ]; then
    ok "已有持久化禁用屏保的配置（重启后依然不熄屏）"
  else
    warn "没有 /etc/X11/xorg.conf.d/10-blanking.conf → 重启后 10 分钟又会熄屏"
    echo "     修： sudo raspi-config nonint do_blanking 1"
  fi
else
  warn "连不上本地 X（可能压根没起桌面，见第 3 步）"
fi

hr "2. 显示接口（DRM connector）"
for p in /sys/class/drm/card*-*/status; do
  [ -e "$p" ] || continue
  printf '   %-22s %s\n' "$(basename "$(dirname "$p")")" "$(cat "$p" 2>/dev/null)"
done
CONNECTED="$(for p in /sys/class/drm/card*-*/status; do
  [ -e "$p" ] && grep -qx connected "$p" 2>/dev/null && basename "$(dirname "$p")"; done | head -1)"
echo "   → 已连接: ${CONNECTED:-（无）}"
case "$CONNECTED" in
  *DSI*) warn "这是 DSI 排线屏：DSI 完全不支持热插拔，且带电插拔排线可能烧屏/烧板。"
         echo "     插拔排线必须先彻底断电。config.txt 里需要 dtoverlay=vc4-kms-dsi-7inch" ;;
esac

hr "3. 本地图形会话"
echo "   默认启动目标 $(systemctl get-default 2>/dev/null)"
SEAT0=0
while read -r id _ _ seat _; do
  [ "${seat:-}" = "seat0" ] && SEAT0=$((SEAT0+1))
done < <(loginctl list-sessions --no-legend 2>/dev/null || true)
echo "   seat0（本机屏幕+键鼠）上的会话数: ${SEAT0}"
for s in lightdm gdm3 sddm xrdp; do
  st="$(systemctl is-active $s 2>/dev/null || true)"
  [ "$st" = "active" ] && printf '   %-8s %s\n' "$s" "$st"
done
COMP="$(ps -eo comm= 2>/dev/null | grep -m1 -E 'labwc|wayfire|openbox|mutter' || true)"
echo "   合成器/WM ${COMP:-未检测到}   会话类型 ${XDG_SESSION_TYPE:-（SSH 里看不到是正常的）}"
if systemctl is-active --quiet xrdp 2>/dev/null; then
  warn "xrdp 在跑。它开的是独立远程会话，不会点亮本地 HDMI（但也不妨碍本地会话）"
fi

hr "4. 输出配置（合成器视角）"
if [ -n "$XINFO" ]; then
  DISPLAY=$XD XAUTHORITY=$XA xrandr 2>/dev/null | grep -E ' connected|\*' | sed 's/^/   /'
  echo "   （带 * 的是当前生效模式；有 * 就说明画面确实在往外送）"
elif command -v wlr-randr >/dev/null 2>&1 && [ -n "${WAYLAND_DISPLAY:-}" ]; then
  wlr-randr 2>/dev/null | sed 's/^/   /'
else
  echo "   （拿不到，看第 3 步结论）"
fi

hr "5. 内核日志"
{ dmesg 2>/dev/null || sudo dmesg 2>/dev/null; } | grep -iE 'hdmi|edid|drm.*connect' | tail -6 | sed 's/^/   /'

# ---------------------------------------------------------------- 结论

hr "结论"
if [ "$SEAT0" = "0" ]; then
cat <<'EOF'
   ▶ seat0 上没有会话 = 本机没启动桌面。和插拔顺序无关。
     修： sudo raspi-config → 1 System Options → S5 Boot/Auto Login
          → B4 Desktop Autologin，然后 sudo reboot
     （这一步也是 kiosk 自启的前提，见 scripts/kiosk-install.sh）
EOF
elif [ -z "$CONNECTED" ]; then
cat <<EOF
   ▶ 所有接口都 disconnected：屏没给出 HPD/EDID（便宜 HDMI 小屏常见）。
     必须强制输出再重启：  ./scripts/screen-fix.sh --apply
     分辨率不对就改本脚本顶部的 MODE（当前 ${MODE}）。
EOF
else
cat <<EOF
   ▶ ${CONNECTED} 已连接、本机有图形会话。
     如果第 1 步报了「Monitor is Off」并已唤醒 → 就是屏保，看屏幕现在亮没亮。
     如果第 4 步有带 * 的模式但屏还是黑：
       · 检查屏的【独立供电】。很多 7 寸 HDMI 屏的 EDID 芯片靠 HDMI 的 +5V 供电，
         所以即使不接 USB 电源也能被识别、也能配出模式 —— 但背光是暗的。
         插上屏自己的 USB 供电线试试。
       · 检查屏上的物理背光/亮度旋钮或按键。
       · 换一根 HDMI 线（劣质线常见只有 DDC 通、TMDS 不通）。
EOF
fi

if [ "$APPLY" = "1" ]; then
  hr "写入启动参数（先备份）"
  sudo cp -n "${BOOT}/cmdline.txt" "${BOOT}/cmdline.txt.bak.ballcam" 2>/dev/null
  sudo cp -n "${BOOT}/config.txt"  "${BOOT}/config.txt.bak.ballcam"  2>/dev/null
  grep -q 'video=HDMI' "${BOOT}/cmdline.txt" 2>/dev/null || \
    sudo sed -i "1s|\$| video=HDMI-A-1:${MODE}|" "${BOOT}/cmdline.txt"
  grep -q '^hdmi_force_hotplug' "${BOOT}/config.txt" 2>/dev/null || \
    echo 'hdmi_force_hotplug=1' | sudo tee -a "${BOOT}/config.txt" >/dev/null
  ok "已写入，备份在 ${BOOT}/*.bak.ballcam"
  echo "   sudo reboot 生效。万一开不了机：把 SD 卡插电脑，用 cmdline.txt.bak.ballcam 覆盖回去。"
fi
