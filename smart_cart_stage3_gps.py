"""
AI電動スマート台車  段階3「GPSで決めた地点を順に走る」
smart_cart_stage1.py と同じフォルダに置く。
流れ: GPS経路追従AI → 【段階1の安全ルール(ソナー・バンパー・非常停止)】 → モーター制御
GPSが悪い・途絶・許可範囲の外 → 停止して待機(自動では再開しない)
"""
import math, random
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple
from smart_cart_stage1 import Command, Sensors, SafetyGovernor, MotorController, SimDriver

# ---------- 設定 ----------
NAV_SPEED = 0.3          # 自動走行の速度(最高速度より低くする)
ARRIVE_M = 2.0           # この距離に入ったら到着(GPS誤差より大きくする)
SLOW_M = 6.0             # 目標地点のこの距離から減速
TURN_IN_PLACE_DEG = 60   # 向きのズレがこれ以上なら、その場で向きを変える
GEOFENCE_M = 40.0        # スタート地点からこの距離を超えたら停止
HDOP_MAX = 2.5           # GPSの精度の悪さ(小さいほど良い)
SATS_MIN = 6             # 最低限必要な衛星数
GPS_TIMEOUT = 1.0        # GPS情報がこの秒数より古いと停止
R_EARTH = 6378137.0

class NavState(Enum):
    IDLE = "待機"
    RUNNING = "自動走行中"
    HOLD = "一時停止(要確認)"
    ARRIVED = "全地点に到着"

@dataclass
class GpsFix:
    lat: float
    lon: float
    fix_ok: bool
    hdop: float
    sats: int
    heading_deg: Optional[float]   # コンパス等から得た向き(北=0, 時計回り)。不明ならNone
    timestamp: float

# ---------- GPS経路追従AI ----------
class Navigator:
    def __init__(self, origin_lat: float, origin_lon: float):
        self.o_lat, self.o_lon = origin_lat, origin_lon
        self.wps: List[Tuple[float, float]] = []
        self.i, self.state, self.reason = 0, NavState.IDLE, "待機"

    def to_xy(self, lat, lon):
        """緯度経度 → スタート地点からの東(x)・北(y)メートル。数百m程度の範囲で使う近似"""
        x = math.radians(lon - self.o_lon) * R_EARTH * math.cos(math.radians(self.o_lat))
        y = math.radians(lat - self.o_lat) * R_EARTH
        return x, y

    def to_latlon(self, x, y):
        lat = self.o_lat + math.degrees(y / R_EARTH)
        lon = self.o_lon + math.degrees(x / (R_EARTH * math.cos(math.radians(self.o_lat))))
        return lat, lon

    def set_route_latlon(self, pts):
        self.wps = [self.to_xy(la, lo) for la, lo in pts]; self.i = 0; self.state = NavState.IDLE

    def start(self):
        """人が「開始」を押して初めて走り出す。HOLDからの再開もこれ"""
        if self.wps and self.i < len(self.wps):
            self.state, self.reason = NavState.RUNNING, "走行開始"

    def _hold(self, why, now):
        self.state, self.reason = NavState.HOLD, why
        return Command(0.0, 0.0, now)

    def compute(self, fix: Optional[GpsFix], now: float) -> Command:
        stop = Command(0.0, 0.0, now)
        if self.state != NavState.RUNNING:
            return stop
        # --- GPSの品質チェック(悪ければ走らない) ---
        if fix is None or now - fix.timestamp > GPS_TIMEOUT: return self._hold("GPS途絶", now)
        if not fix.fix_ok or fix.hdop > HDOP_MAX or fix.sats < SATS_MIN: return self._hold("GPS精度不足", now)
        if fix.heading_deg is None: return self._hold("向きが不明", now)
        x, y = self.to_xy(fix.lat, fix.lon)
        if math.hypot(x, y) > GEOFENCE_M: return self._hold("走行許可範囲の外", now)
        # --- 目標地点へ ---
        wx, wy = self.wps[self.i]
        dx, dy = wx - x, wy - y
        dist = math.hypot(dx, dy)
        if dist <= ARRIVE_M:
            self.i += 1
            if self.i >= len(self.wps):
                self.state, self.reason = NavState.ARRIVED, "全地点に到着"
            else:
                self.reason = f"地点{self.i}に到着、次へ"
            return stop
        bearing = math.degrees(math.atan2(dx, dy)) % 360
        err = (bearing - fix.heading_deg + 180) % 360 - 180   # +なら右へ向く
        turn = max(-1.0, min(1.0, err / 60))
        if abs(err) > TURN_IN_PLACE_DEG:
            speed = 0.0; self.reason = f"地点{self.i+1}へ向きを変更中"
        else:
            speed = NAV_SPEED * math.cos(math.radians(err))
            if dist < SLOW_M: speed *= max(0.4, dist / SLOW_M)
            self.reason = f"地点{self.i+1}へ走行 残り{dist:.0f}m"
        return Command(speed, turn, now)

# ---------- シミュレーション ----------
DT = 0.1
class SimWorld:
    """屋外の模擬世界。GPSの誤差(ゆっくり揺れる)・向きの誤差・前方障害物つき"""
    def __init__(self, nav, heading=90.0, obstacles=()):
        self.nav, self.x, self.y, self.h, self.v = nav, 0.0, 0.0, heading, 0.0
        self.obs, self.nx, self.ny = list(obstacles), 0.0, 0.0
        self.gps_down = False; self.hdop = 1.2; self.sats = 10; self.heading_ok = True
    def gps(self, now):
        if self.gps_down: return None
        self.nx = 0.95 * self.nx + random.gauss(0, 0.25); self.ny = 0.95 * self.ny + random.gauss(0, 0.25)
        la, lo = self.nav.to_latlon(self.x + self.nx, self.y + self.ny)
        hd = (self.h + random.gauss(0, 3)) % 360 if self.heading_ok else None
        return GpsFix(la, lo, True, self.hdop, self.sats, hd, now)
    def sonar_cm(self):
        best = 400.0
        hr = math.radians(self.h)
        for ox, oy, r in self.obs:
            dx, dy = ox - self.x, oy - self.y
            fwd = dx * math.sin(hr) + dy * math.cos(hr)
            lat = dx * math.cos(hr) - dy * math.sin(hr)
            if fwd > 0 and abs(lat) < r + 0.3: best = min(best, (fwd - r) * 100)
        return max(0.0, best)
    def sensors(self, now):
        d = self.sonar_cm()
        return Sensors(d, d, d, 200, 200, d <= 3, False, now)
    def move(self, driver):
        l, r = driver.left, driver.right
        avg = (l + r) / 2
        self.v += (avg * 1.0 - self.v) * 0.3            # 出力1.0で秒速1m
        self.h = (self.h + (l - r) * 86 * DT) % 360       # 旋回
        self.x += self.v * math.sin(math.radians(self.h)) * DT
        self.y += self.v * math.cos(math.radians(self.h)) * DT

def run(route_xy, seconds, obstacles=(), events=None, label=""):
    random.seed(1)
    nav = Navigator(35.0, 135.0)
    nav.set_route_latlon([nav.to_latlon(*p) for p in route_xy])
    world, gov, drv = SimWorld(nav, obstacles=obstacles), SafetyGovernor(), SimDriver()
    motor = MotorController(drv)
    nav.start()
    log, t, reached = [], 0.0, []
    while t < seconds:
        if events: events(t, world, nav)
        fix, s = world.gps(t), world.sensors(t)
        cmd = gov.decide(nav.compute(fix, t), s, t)
        motor.update(cmd, t); world.move(drv)
        if nav.i > len(reached): reached.append((t, world.x, world.y))
        log.append((t, world.x, world.y, world.v, nav.state, nav.reason, gov.reason, s.front_center))
        if nav.state == NavState.ARRIVED: break
        t += DT
    return nav, world, log, reached

def demo():
    print("=== 試験1: 四角いコース(15m四方)を順に走る ===")
    route = [(15, 0), (15, 15), (0, 15), (0, 0)]
    nav, w, log, rch = run(route, 600)
    for k, (t, x, y) in enumerate(rch):
        wx, wy = route[k]
        print(f"地点{k+1} 到着 {t:5.0f}秒  実際の位置との差 {math.hypot(x-wx, y-wy):.1f}m")
    print(f"結果: {nav.state.value} (所要 {log[-1][0]:.0f}秒)")

    print("\n=== 試験2: 進路上に人が立っている → 手前で止まるか ===")
    nav, w, log, _ = run([(30, 0)], 60, obstacles=[(15, 0, 0.3)])
    print(f"停止位置 x={w.x:.1f}m(人は15m) 前方距離 {log[-1][7]:.0f}cm 速度 {w.v:.2f}m/s 判断: {log[-1][6]}")

    print("\n=== 試験3: 走行中にGPSが途絶 → 停止、復旧しても自動再開しない ===")
    def ev(t, w, nav):
        w.gps_down = 10 <= t < 13
    nav, w, log, _ = run([(40, 0)], 40, events=ev)
    at15 = [l for l in log if abs(l[0]-15) < 0.05][0]
    print(f"10〜13秒GPS途絶 → 15秒時点: 速度{at15[3]:.2f} 状態:{at15[4].value}({at15[5]})")
    print(f"40秒時点: x={w.x:.1f}m 状態:{nav.state.value} → 人が『開始』を押すまで動かない")

    print("\n=== 試験4: GPS精度が悪化 / 向きが不明 ===")
    for name, f in (("精度悪化(HDOP 5)", lambda t,w,n: setattr(w,'hdop',5.0) if t>5 else None),
                    ("衛星数不足(3個)", lambda t,w,n: setattr(w,'sats',3) if t>5 else None),
                    ("向き不明", lambda t,w,n: setattr(w,'heading_ok',False) if t>5 else None)):
        nav, w, log, _ = run([(40, 0)], 15, events=f)
        print(f"{name}: {nav.state.value}「{nav.reason}」 速度{w.v:.2f}")

    print("\n=== 試験5: 許可範囲(40m)の外へ向かう経路 ===")
    nav, w, log, _ = run([(80, 0)], 200)
    print(f"停止位置 x={w.x:.1f}m → {nav.state.value}「{nav.reason}」")

if __name__ == "__main__":
    demo()
