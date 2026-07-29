#!/usr/bin/env bash
# 装「开机自动全屏显示 UI」（kiosk）。
#
# 树莓派桌面这几年换了三套合成器（LXDE/X11 → wayfire → labwc），自启动条目
# 放的位置各不相同，所以这里先探测再写对应的那一个，不瞎猜。
set -euo pipefail
cd "$(dirname "$0")/.."
APP="$(pwd)"
URL="http://localhost:8000"

BROWSER="$(command -v chromium || command -v chromium-browser || true)"
[ -z "${BROWSER}" ] && { echo "✖ 没装 chromium，先跑 ./install.sh"; exit 1; }

LAUNCH="${APP}/scripts/kiosk-run.sh"
cat > "${LAUNCH}" <<EOF
#!/usr/bin/env bash
# 等后端起来再开浏览器，否则会先弹一个错误页
for i in \$(seq 1 60); do
  curl -sf ${URL}/api/status >/dev/null && break
  sleep 1
done
exec ${BROWSER} --kiosk --noerrdialogs --disable-infobars \\
  --disable-session-crashed-bubble --disable-features=Translate \\
  --check-for-update-interval=31536000 --autoplay-policy=no-user-gesture-required \\
  --app=${URL}
EOF
chmod +x "${LAUNCH}"

SESSION="${XDG_SESSION_TYPE:-unknown}"
COMP="$(ps -eo comm= | grep -m1 -E 'labwc|wayfire|openbox|mutter' || true)"
echo "== 会话类型 ${SESSION}   合成器 ${COMP:-未检测到}"

case "${COMP}" in
  labwc)
    mkdir -p ~/.config/labwc
    grep -q kiosk-run ~/.config/labwc/autostart 2>/dev/null || echo "${LAUNCH} &" >> ~/.config/labwc/autostart
    chmod +x ~/.config/labwc/autostart
    echo "   → 写入 ~/.config/labwc/autostart" ;;
  wayfire)
    mkdir -p ~/.config
    F=~/.config/wayfire.ini
    grep -q '^\[autostart\]' "$F" 2>/dev/null || echo -e "\n[autostart]" >> "$F"
    grep -q kiosk-run "$F" || sed -i "/^\[autostart\]/a ballcam = ${LAUNCH}" "$F"
    echo "   → 写入 ${F} 的 [autostart]" ;;
  *)
    mkdir -p ~/.config/autostart
    cat > ~/.config/autostart/ballcam-ui.desktop <<EOF
[Desktop Entry]
Type=Application
Name=ballcam UI
Exec=${LAUNCH}
X-GNOME-Autostart-enabled=true
EOF
    echo "   → 写入 ~/.config/autostart/ballcam-ui.desktop（XDG 通用位置）" ;;
esac

cat <<'EOF'

== 还差两个开关（必须手动设，脚本不便代劳）：

  sudo raspi-config
    1 System Options  → S5 Boot / Auto Login → B4 Desktop Autologin
        没有自动登录到桌面，开机会停在登录界面，UI 不会出来。
    2 Display Options → Screen Blanking → No
        默认十几分钟熄屏，评委看着黑屏很尴尬。

  设完 sudo reboot 验证：只给电、不碰键鼠，60 秒内应自动出现全屏 UI。

  临时手动起一次（不用重启）： ./scripts/kiosk-run.sh
  退出 kiosk：接键盘按 Alt+F4，或 SSH 里 pkill chromium
EOF
