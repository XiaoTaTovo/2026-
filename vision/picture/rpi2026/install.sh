#!/usr/bin/env bash
# ballcam 一键安装。幂等，可以反复跑。
#
# ★ 必须在【有网】的时候跑一次。原因：树莓派只有一块无线网卡，wlan0 一旦拿去
#   做热点就没有外网了，赛场也不一定有网 —— 所以所有 apt 包必须现在装齐。
#
# 用法：
#   cd ~/workspace/ti-cup/vision/2026
#   chmod +x install.sh scripts/*.sh
#   ./install.sh
set -euo pipefail
cd "$(dirname "$0")"
APP="$(pwd)"
USER_NAME="$(id -un)"
DATA="${HOME}/data/ti_vision/2026"

say() { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }

say "1/7 检查是否有网（无网就装不了包，先连上 WiFi 或插网线）"
if ! ping -c1 -W3 deb.debian.org >/dev/null 2>&1; then
  echo "  ✖ 没有外网。请先连 WiFi/插网线再跑本脚本。"; exit 1
fi

say "2/7 安装系统包"
sudo apt-get update
# 优先用 apt 的 python3-* 包：Debian 的 Python 是 externally-managed，
# 直接 pip install 会被 PEP668 拦住；用系统包最省事。
sudo apt-get install -y --no-install-recommends \
  ffmpeg python3 python3-pip python3-httpx python3-fastapi python3-uvicorn \
  network-manager chromium fonts-noto-cjk unclutter || true
# 某些镜像里 chromium 叫 chromium-browser
command -v chromium >/dev/null || sudo apt-get install -y chromium-browser || true

say "3/7 校验 Python 依赖"
if ! python3 -c "import fastapi, uvicorn, httpx" 2>/dev/null; then
  echo "  apt 版本不全，退回 venv（--system-site-packages 复用已装的部分）"
  python3 -m venv --system-site-packages "${APP}/.venv"
  "${APP}/.venv/bin/pip" install -q fastapi uvicorn httpx
  PY="${APP}/.venv/bin/python3"
else
  PY="$(command -v python3)"
fi
echo "  Python = ${PY}"

say "4/7 建数据目录（代码区/数据区分离，符合本机既有约定）"
mkdir -p "${DATA}"/{mp4,raw,meta,shots,logs}
echo "  ${DATA}"

say "5/7 生成配置 conf/ballcam.env"
mkdir -p conf
if [ ! -f conf/ballcam.env ]; then
  cp conf/ballcam.env.example conf/ballcam.env
  sed -i "s|^DATA_DIR=.*|DATA_DIR=${DATA}|" conf/ballcam.env
fi
# 探测硬件 h264 编码器。Pi4 有 h264_v4l2m2m，但它在新内核上时好时坏，
# 而 libx264 ultrafast 编 480p@15fps 只占 30~50% 单核 —— 稳定优先，默认 libx264。
if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q h264_v4l2m2m; then
  echo "  检测到 h264_v4l2m2m（硬编）。默认仍用 libx264；卡顿时可手动改 conf/ballcam.env"
fi
grep -q '^DATA_DIR' conf/ballcam.env || echo "DATA_DIR=${DATA}" >> conf/ballcam.env

say "6/7 安装 systemd 服务 + 免密 sudo 白名单"
sed -e "s|@USER@|${USER_NAME}|g" -e "s|@APP@|${APP}|g" -e "s|@PY@|${PY}|g" \
    systemd/ballcam-api.service | sudo tee /etc/systemd/system/ballcam-api.service >/dev/null
# UI 上的换信道/重启/关机按钮需要这几条免密 sudo。范围收得很窄，只列具体命令。
sed -e "s|@USER@|${USER_NAME}|g" sudoers.d-ballcam | sudo tee /etc/sudoers.d/ballcam >/dev/null
sudo chmod 440 /etc/sudoers.d/ballcam
sudo visudo -c -f /etc/sudoers.d/ballcam
sudo systemctl daemon-reload
sudo systemctl enable --now ballcam-api

say "7/7 完成。下一步："
cat <<EOF

  ① 建 WiFi 热点        ./scripts/net-ap.sh
  ② 建网线直连备用 IP    ./scripts/net-eth-direct.sh
  ③ 装开机自启 kiosk     ./scripts/kiosk-install.sh
  ④ 屏幕不亮就跑         ./scripts/screen-fix.sh
  ⑤ 体检/回填信息        ./scripts/diag.sh

  服务状态：systemctl status ballcam-api
  本机打开：http://localhost:8000
  外部打开：http://xiaotpi.local:8000  或  http://192.168.50.1:8000
EOF
