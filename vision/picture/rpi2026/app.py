#!/usr/bin/env python3
"""ballcam 图传接收端后端（单文件）。

为什么是单文件：交接给一个人维护，拆成 8 个模块只会增加心智负担。
全部逻辑 ~350 行，改哪儿一眼能找到。

架构核心（这是本文件最重要的设计，改代码前必读）：
  ESP32-CAM 的 CameraWebServer 固件同时只接受 **一个** /stream 客户端。
  所以浏览器绝对不能直连相机拉流——那会和录制用的 ffmpeg 抢同一路连接，
  结果是两边都断流。

  本后端唯一持有上游连接，解析出完整 JPEG 帧后 fan-out 给两个下游：
    1) 浏览器预览（GET /api/stream，multipart/x-mixed-replace）
    2) 录制中的 ffmpeg（写它的 stdin）

  好处：相机只被连一次；预览和录制互不影响（评委翻页不会中断录像）；
  断线重连逻辑只有一处；帧率/丢帧统计天然可得。

  代价：多一次内存搬运。VGA q12 约 15fps×30KB = 450KB/s，Pi4 上不到 3% CPU。
"""
import asyncio
import json
import os
import shutil
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import httpx
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

# ---------------------------------------------------------------- 配置

HERE = Path(__file__).resolve().parent


def load_env():
    """读 conf/ballcam.env。手写解析，不引 dotenv——少一个依赖少一个装不上的风险。"""
    cfg = {
        "CAM_HOST": "",             # 留空 = 从 DHCP lease 自动发现
        "CAM_STREAM_PORT": "81",
        "CAM_CTRL_PORT": "80",
        "DATA_DIR": str(Path.home() / "data/ti_vision/2026"),
        "H264_ENC": "libx264",
        "TARGET_FPS": "15",
        "UI_PORT": "8000",
        "LEASE_FILE": "/var/lib/NetworkManager/dnsmasq-wlan0.leases",
        "AP_CON": "ap-ballcam",
    }
    f = HERE / "conf/ballcam.env"
    if f.exists():
        for line in f.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                cfg[k.strip()] = v.strip().strip('"')
    return cfg


CFG = load_env()
DATA = Path(CFG["DATA_DIR"])
for sub in ("mp4", "raw", "meta", "shots", "logs"):
    (DATA / sub).mkdir(parents=True, exist_ok=True)
CALIB_FILE = DATA / "calib.json"
FPS = int(CFG["TARGET_FPS"])

SOI, EOI = b"\xff\xd8", b"\xff\xd9"   # JPEG 起始/结束标记

# ---------------------------------------------------------------- 帧分发中枢


class Hub:
    """唯一上游连接 + fan-out。"""

    def __init__(self):
        self.subs: set[asyncio.Queue] = set()
        self.last: bytes | None = None
        self.stamps = deque(maxlen=30)     # 最近 30 帧到达时刻，用于算 fps
        self.online = False
        self.cam_ip = CFG["CAM_HOST"] or None
        self.err = ""
        self.total = 0

    @property
    def fps(self) -> float:
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return round((len(self.stamps) - 1) / span, 1) if span > 0 else 0.0

    def publish(self, frame: bytes):
        self.last = frame
        self.total += 1
        self.stamps.append(time.monotonic())
        for q in list(self.subs):
            if q.full():
                try:
                    q.get_nowait()        # 丢最旧的一帧，保预览低延迟
                except asyncio.QueueEmpty:
                    pass
            try:
                q.put_nowait(frame)
            except asyncio.QueueFull:
                pass
        REC.on_frame(frame)

    async def discover(self) -> str | None:
        """从 NetworkManager 的 DHCP lease 表里找相机，逐个探 /status。

        这样队友永远不用手填 IP——相机换网、重连拿到新 IP 都能自己找到。
        """
        if CFG["CAM_HOST"]:
            return CFG["CAM_HOST"]
        ips = []
        lf = Path(CFG["LEASE_FILE"])
        if lf.exists():
            for line in lf.read_text().splitlines():
                p = line.split()
                if len(p) >= 3:
                    ips.append(p[2])
        ips += ["10.42.0.23", "10.42.0.50"]   # 常见值兜底
        async with httpx.AsyncClient(timeout=1.2) as c:
            for ip in dict.fromkeys(ips):
                for path in ("/status", "/capture"):
                    try:
                        r = await c.get(f"http://{ip}:{CFG['CAM_CTRL_PORT']}{path}")
                        if r.status_code == 200:
                            return ip
                    except Exception:
                        pass
        return None

    async def run(self):
        """常驻任务：连上游、解析 MJPEG、断了就重连。无限重试，永不放弃。"""
        while True:
            ip = self.cam_ip or await self.discover()
            if not ip:
                self.online, self.err = False, "未发现相机（检查相机供电和 WiFi）"
                await asyncio.sleep(2)
                continue
            self.cam_ip = ip
            url = f"http://{ip}:{CFG['CAM_STREAM_PORT']}/stream"
            try:
                async with httpx.AsyncClient(timeout=httpx.Timeout(5, read=8)) as c:
                    async with c.stream("GET", url) as r:
                        r.raise_for_status()
                        self.online, self.err = True, ""
                        buf = bytearray()
                        async for chunk in r.aiter_bytes(8192):
                            buf += chunk
                            # 按 JPEG SOI/EOI 切帧，不依赖 multipart boundary 字符串。
                            # ESP32-CAM 输出不带 EXIF 缩略图，所以这样切是安全的。
                            while True:
                                i = buf.find(SOI)
                                if i < 0:
                                    if len(buf) > (1 << 20):
                                        buf.clear()
                                    break
                                j = buf.find(EOI, i + 2)
                                if j < 0:
                                    del buf[:i]
                                    break
                                self.publish(bytes(buf[i:j + 2]))
                                del buf[:j + 2]
            except Exception as e:
                self.online = False
                self.err = f"{type(e).__name__}: {e}"
                self.cam_ip = CFG["CAM_HOST"] or None   # 自动发现模式下重新找
                await asyncio.sleep(1.5)


# ---------------------------------------------------------------- 录制


class Recorder:
    """ffmpeg 一进程双输出；断线自动续录分片；写 sidecar JSON。"""

    def __init__(self):
        self.active = False
        self.item = ""
        self.rid = ""
        self.t0 = 0.0
        self.parts: list[dict] = []
        self.marks: list[float] = []
        self.proc = None
        self.q: asyncio.Queue = asyncio.Queue(maxsize=60)
        self.writer = None
        self.watch = None
        self.frames = 0
        self.dropped = 0

    def _args(self, raw: Path, mp4: Path):
        enc = CFG["H264_ENC"]
        a = ["ffmpeg", "-hide_banner", "-loglevel", "error",
             "-f", "mjpeg", "-use_wallclock_as_timestamps", "1", "-i", "pipe:0",
             # 第一路：原始 JPEG 帧直接封 mkv。零重编码、零画质损失、CPU 近 0。
             # mkv 容错强，录制中掉电也能修出来 —— 这是"完整记录"的保底证据。
             "-map", "0:v", "-c", "copy", "-f", "matroska", str(raw),
             # 第二路：h264 fragmented mp4。+frag_keyframe+empty_moov 让文件
             # 边写边可播，掉电不会变废文件；评委说"回放刚才那次"能立刻放，
             # 不用等转码 —— 这就是不采用"先存后转"的原因。
             "-map", "0:v", "-c:v", enc]
        if enc == "libx264":
            a += ["-preset", "ultrafast", "-tune", "zerolatency"]
        a += ["-pix_fmt", "yuv420p",
              "-g", str(FPS),      # 每秒一个 I 帧：逐帧步进和拖进度条才顺滑
              "-movflags", "+frag_keyframe+empty_moov", "-f", "mp4", str(mp4)]
        return a

    async def _spawn(self):
        n = len(self.parts) + 1
        stem = f"{self.rid}_part{n}"
        raw, mp4 = DATA / "raw" / f"{stem}.mkv", DATA / "mp4" / f"{stem}.mp4"
        self.proc = await asyncio.create_subprocess_exec(
            *self._args(raw, mp4),
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.DEVNULL,
            stderr=open(DATA / "logs" / f"{stem}.log", "wb"))
        self.parts.append({"part": n, "mp4": mp4.name, "raw": raw.name,
                           "t_offset": round(time.monotonic() - self.t0, 2)})

    async def _write_loop(self):
        while self.active:
            try:
                f = await asyncio.wait_for(self.q.get(), 1.0)
            except asyncio.TimeoutError:
                continue
            if self.proc and self.proc.stdin and not self.proc.stdin.is_closing():
                try:
                    self.proc.stdin.write(f)
                    await self.proc.stdin.drain()
                    self.frames += 1
                except Exception:
                    pass

    async def _watch_loop(self):
        """ffmpeg 意外退出（上游断流会导致）而仍在录 → 立刻开新分片续录。

        这样"相机中途掉线"也仍然满足题目"完整记录每次测试"。
        """
        while self.active:
            await asyncio.sleep(1)
            if self.proc and self.proc.returncode is not None and self.active:
                await self._spawn()

    def on_frame(self, f: bytes):
        if not self.active:
            return
        if self.q.full():
            self.dropped += 1
            return
        self.q.put_nowait(f)

    async def start(self, item: str):
        if self.active:
            raise HTTPException(409, "已在录制中")
        self.item, self.t0 = item, time.monotonic()
        self.rid = f"{item}_{datetime.now():%Y%m%d_%H%M%S}"
        self.parts, self.marks, self.frames, self.dropped = [], [], 0, 0
        self.q = asyncio.Queue(maxsize=60)
        self.active = True
        await self._spawn()
        self.writer = asyncio.create_task(self._write_loop())
        self.watch = asyncio.create_task(self._watch_loop())
        return self.rid

    async def stop(self):
        if not self.active:
            raise HTTPException(409, "当前未在录制")
        self.active = False
        for t in (self.writer, self.watch):
            if t:
                t.cancel()
        if self.proc and self.proc.stdin:
            try:
                self.proc.stdin.close()
            except Exception:
                pass
            try:
                await asyncio.wait_for(self.proc.wait(), 8)
            except asyncio.TimeoutError:
                self.proc.kill()
        dur = round(time.monotonic() - self.t0, 2)
        meta = {"id": self.rid, "item": self.item,
                "ended_at": datetime.now().isoformat(timespec="seconds"),
                "duration_s": dur, "frames": self.frames,
                "fps": round(self.frames / dur, 1) if dur else 0,
                "dropped": self.dropped, "parts": self.parts,
                "marks": self.marks, "note": ""}
        (DATA / "meta" / f"{self.rid}.json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
        return meta

    def status(self):
        return {"active": self.active, "item": self.item, "id": self.rid,
                "elapsed": round(time.monotonic() - self.t0, 1) if self.active else 0,
                "frames": self.frames, "dropped": self.dropped,
                "parts": len(self.parts), "marks": self.marks}


HUB, REC = Hub(), Recorder()
app = FastAPI(title="ballcam")


@app.on_event("startup")
async def _boot():
    asyncio.create_task(HUB.run())


# ---------------------------------------------------------------- 工具


async def sh(*args, timeout=15):
    p = await asyncio.create_subprocess_exec(
        *args, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    try:
        out, _ = await asyncio.wait_for(p.communicate(), timeout)
    except asyncio.TimeoutError:
        p.kill()
        return 1, "timeout"
    return p.returncode, out.decode("utf-8", "replace")


# ---------------------------------------------------------------- API


@app.get("/api/status")
async def status():
    du = shutil.disk_usage(DATA)
    ok, ap = await sh("nmcli", "-t", "-f", "NAME,DEVICE", "connection", "show", "--active")
    return {
        "cam": {"ip": HUB.cam_ip, "online": HUB.online, "fps": HUB.fps,
                "frames": HUB.total, "err": HUB.err},
        "rec": REC.status(),
        "disk": {"free_gb": round(du.free / 2**30, 1), "total_gb": round(du.total / 2**30, 1)},
        "ap": {"up": CFG["AP_CON"] in ap, "raw": ap.strip() if ok == 0 else ""},
        "time": datetime.now().strftime("%H:%M:%S"),
        "date": datetime.now().strftime("%Y-%m-%d"),
        "calib": json.loads(CALIB_FILE.read_text()) if CALIB_FILE.exists() else None,
        "fps_target": FPS,
    }


@app.get("/api/stream")
async def stream():
    """把 hub 的帧转成 multipart 流给浏览器。<img src> 直接吃这个。"""
    q: asyncio.Queue = asyncio.Queue(maxsize=2)
    HUB.subs.add(q)

    async def gen():
        try:
            if HUB.last:
                yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + HUB.last + b"\r\n"
            while True:
                f = await q.get()
                yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n"
        finally:
            HUB.subs.discard(q)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=f")


@app.post("/api/rec/start")
async def rec_start(item: str = "free"):
    if not HUB.online:
        raise HTTPException(409, "相机离线，无法录制")
    return {"id": await REC.start(item)}


@app.post("/api/rec/stop")
async def rec_stop():
    return await REC.stop()


@app.post("/api/rec/mark")
async def rec_mark():
    if not REC.active:
        raise HTTPException(409, "未在录制")
    t = round(time.monotonic() - REC.t0, 2)
    REC.marks.append(t)
    return {"t": t, "marks": REC.marks}


@app.get("/api/recordings")
async def recordings(item: str = ""):
    out = []
    for f in sorted((DATA / "meta").glob("*.json"), reverse=True):
        try:
            m = json.loads(f.read_text(encoding="utf-8"))
        except Exception:
            continue
        if item and m.get("item") != item:
            continue
        m["size_mb"] = round(sum(
            (DATA / "mp4" / p["mp4"]).stat().st_size
            for p in m.get("parts", []) if (DATA / "mp4" / p["mp4"]).exists()) / 2**20, 1)
        out.append(m)
    return out


@app.delete("/api/recordings/{rid}")
async def rec_del(rid: str):
    mf = DATA / "meta" / f"{rid}.json"
    if not mf.exists():
        raise HTTPException(404, "no such record")
    m = json.loads(mf.read_text(encoding="utf-8"))
    for p in m.get("parts", []):
        for sub, key in (("mp4", "mp4"), ("raw", "raw")):
            (DATA / sub / p[key]).unlink(missing_ok=True)
    mf.unlink()
    return {"ok": True}


@app.post("/api/shot")
async def shot():
    if not HUB.last:
        raise HTTPException(409, "无画面")
    p = DATA / "shots" / f"shot_{datetime.now():%Y%m%d_%H%M%S}.jpg"
    p.write_bytes(HUB.last)
    return {"file": p.name}


@app.post("/api/camera/control")
async def cam_ctl(var: str, val: str):
    if not HUB.cam_ip:
        raise HTTPException(409, "相机未连接")
    async with httpx.AsyncClient(timeout=3) as c:
        r = await c.get(f"http://{HUB.cam_ip}:{CFG['CAM_CTRL_PORT']}/control",
                        params={"var": var, "val": val})
    return {"code": r.status_code}


@app.post("/api/camera/reset")
async def cam_reset():
    HUB.cam_ip = CFG["CAM_HOST"] or None
    return {"ok": True, "note": "已清除缓存 IP，重连中"}


@app.get("/api/net/scan")
async def net_scan():
    """扫周边 AP，按信道统计拥挤度，推荐 1/6/11 里最空的。

    赛场几十支队伍抢 2.4G，这是本项最大失分风险，所以做成一键。
    """
    await sh("nmcli", "dev", "wifi", "rescan", timeout=20)
    ok, out = await sh("nmcli", "-t", "-f", "SSID,CHAN,SIGNAL", "dev", "wifi", "list")
    load: dict[int, int] = {}
    for line in out.splitlines():
        p = line.split(":")
        if len(p) >= 3 and p[1].isdigit():
            ch = int(p[1])
            try:
                load[ch] = load.get(ch, 0) + int(p[2])
            except ValueError:
                pass
    best = min((1, 6, 11), key=lambda c: load.get(c, 0))
    return {"load": load, "recommend": best}


@app.post("/api/net/channel")
async def net_ch(ch: int):
    if ch not in range(1, 14):
        raise HTTPException(400, "bad channel")
    await sh("sudo", "nmcli", "con", "mod", CFG["AP_CON"], "802-11-wireless.channel", str(ch))
    await sh("sudo", "nmcli", "con", "up", CFG["AP_CON"], timeout=30)
    return {"ok": True, "ch": ch}


@app.post("/api/net/ap-up")
async def ap_up():
    ok, out = await sh("sudo", "nmcli", "con", "up", CFG["AP_CON"], timeout=30)
    return {"ok": ok == 0, "out": out}


@app.post("/api/calib")
async def calib(x1: float, x2: float, cm: float = 20.0):
    """刻度标定：x1/x2 是画面上摆杆两端刻度的横向百分比，cm 是这两点间实际距离。"""
    d = {"x1": x1, "x2": x2, "cm": cm}
    CALIB_FILE.write_text(json.dumps(d), encoding="utf-8")
    return d


@app.post("/api/sys/clean")
async def clean(days: int = 7):
    cut = time.time() - days * 86400
    n = 0
    for f in (DATA / "meta").glob("*.json"):
        if f.stat().st_mtime < cut:
            await rec_del(f.stem)
            n += 1
    return {"deleted": n}


# ★ 这条必须放在 /api/sys/clean 之后：FastAPI 按声明顺序匹配，
#   若 {action} 在前，/api/sys/clean 会被它吞掉并 404。
@app.post("/api/sys/{action}")
async def sysact(action: str):
    cmds = {"poweroff": ("sudo", "/sbin/poweroff"),
            "reboot": ("sudo", "/sbin/reboot"),
            "restart-api": ("sudo", "/usr/bin/systemctl", "restart", "ballcam-api")}
    if action not in cmds:
        raise HTTPException(404)
    if REC.active:
        raise HTTPException(409, "正在录制，先停止录制")
    asyncio.create_task(sh(*cmds[action]))
    return {"ok": True}


# 视频文件直接给 <video> 用；静态 UI 挂在根路径
app.mount("/rec", StaticFiles(directory=str(DATA / "mp4")), name="rec")


@app.get("/")
async def index():
    return FileResponse(HERE / "web/index.html")


app.mount("/web", StaticFiles(directory=str(HERE / "web")), name="web")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(CFG["UI_PORT"]), log_level="warning")
