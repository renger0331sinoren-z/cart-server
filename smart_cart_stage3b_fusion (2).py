"""
段階3b: RTKなしでGPS誤差を小さくする
  ① センサー融合: 車輪の回転量(オドメトリ)+ジャイロ+コンパス+GPS を組み合わせて位置・向きを推定
  ② スタート地点補正: 位置が分かっている場所で30秒測り、一定のずれを引き算する
smart_cart_stage1.py / smart_cart_stage3_gps.py と同じフォルダに置く。
"""
import math, random
import smart_cart_stage3_gps as g
from smart_cart_stage1 import SafetyGovernor, MotorController, SimDriver

DT = g.DT
GPS_BIAS = (1.5, -1.0)      # 模擬: GPSの一定のずれ(m)
def wrap(a): return (a + 180) % 360 - 180

class PositionEstimator:
    """位置: カルマンフィルタ(車輪の動きで予測 → GPSで少しずつ補正)
       向き: ジャイロで動かし、コンパスでゆっくり補正(相補フィルタ)"""
    def __init__(self, nav, r_gps=1.5, k_head=0.02):
        self.nav, self.R, self.kh = nav, r_gps ** 2, k_head
        self.x = self.y = self.h = 0.0; self.P = 0.0; self.ready = False
    def update(self, fix, odo_m, gyro_dps, dt):
        if not self.ready:
            if fix is not None and fix.heading_deg is not None:
                self.x, self.y = self.nav.to_xy(fix.lat, fix.lon)
                self.h, self.P, self.ready = fix.heading_deg, self.R, True
            return
        self.h = (self.h + gyro_dps * dt) % 360                      # 予測(ジャイロ)
        if fix is not None and fix.heading_deg is not None:           # 補正(コンパス)
            self.h = (self.h + self.kh * wrap(fix.heading_deg - self.h)) % 360
        hr = math.radians(self.h)                                     # 予測(車輪)
        self.x += odo_m * math.sin(hr); self.y += odo_m * math.cos(hr)
        self.P += (0.03 * abs(odo_m)) ** 2 + 1e-4
        if fix is not None:                                           # 補正(GPS)
            zx, zy = self.nav.to_xy(fix.lat, fix.lon)
            K = self.P / (self.P + self.R)
            self.x += K * (zx - self.x); self.y += K * (zy - self.y); self.P *= (1 - K)

# ---------- 模擬世界(一定のずれ・車輪の誤差・ジャイロの誤差つき) ----------
class World2(g.SimWorld):
    def gps(self, now):
        if self.gps_down: return None
        self.nx = 0.95 * self.nx + random.gauss(0, 0.25); self.ny = 0.95 * self.ny + random.gauss(0, 0.25)
        la, lo = self.nav.to_latlon(self.x + self.nx + GPS_BIAS[0], self.y + self.ny + GPS_BIAS[1])
        return g.GpsFix(la, lo, True, self.hdop, self.sats, (self.h + random.gauss(0, 3)) % 360, now)
    def move2(self, driver):
        h0, x0, y0 = self.h, self.x, self.y
        self.move(driver)
        odo = math.hypot(self.x - x0, self.y - y0) * 1.02 + random.gauss(0, 0.002)   # 車輪は2%多く数える
        rate = wrap(self.h - h0) / DT + 0.3 + random.gauss(0, 0.5)                   # ジャイロの偏り0.3°/s
        return odo, rate

def run2(route, fusion, arrive_m, calib=False, seed=1):
    random.seed(seed); g.ARRIVE_M = arrive_m
    nav = g.Navigator(35.0, 135.0); nav.set_route_latlon([nav.to_latlon(*p) for p in route])
    w, gov, drv = World2(nav), SafetyGovernor(), SimDriver()
    motor, est = MotorController(drv), PositionEstimator(nav)
    off, odo, rate, t, arr = (0.0, 0.0), 0.0, 0.0, 0.0, []
    raw_err, est_err = [], []
    if calib:   # 位置が分かっている(0,0)に止めて30秒測り、一定のずれを求める
        xs, ys = [], []
        for _ in range(300):
            f = w.gps(t); zx, zy = nav.to_xy(f.lat, f.lon); xs.append(zx); ys.append(zy); t += DT
        off = (sum(xs) / len(xs), sum(ys) / len(ys))
    nav.start()
    while t < 600:
        fix = w.gps(t)
        if fix is not None and calib:    # 求めたずれを引く
            fix.lat, fix.lon = nav.to_latlon(*[a - b for a, b in zip(nav.to_xy(fix.lat, fix.lon), off)])
        est.update(fix, odo, rate, DT)
        use = fix
        if fusion and fix is not None and est.ready:
            la, lo = nav.to_latlon(est.x, est.y)
            use = g.GpsFix(la, lo, fix.fix_ok, fix.hdop, fix.sats, est.h, fix.timestamp)
        if fix is not None and nav.state == g.NavState.RUNNING:
            rx, ry = nav.to_xy(fix.lat, fix.lon); raw_err.append((rx - w.x, ry - w.y))
            est_err.append((est.x - w.x, est.y - w.y))
        cmd = gov.decide(nav.compute(use, t), w.sensors(t), t)
        motor.update(cmd, t); odo, rate = w.move2(drv)
        if nav.i > len(arr): arr.append((w.x, w.y))
        if nav.state == g.NavState.ARRIVED: break
        t += DT
    return arr, raw_err, est_err

def stats(errs):
    n = len(errs); mx = sum(e[0] for e in errs) / n; my = sum(e[1] for e in errs) / n
    sd = math.sqrt(sum((e[0]-mx)**2 + (e[1]-my)**2 for e in errs) / n)
    return math.hypot(mx, my), sd

def report(name, route, **kw):
    arr, raw, est = run2(route, **kw)
    errs = [(a[0]-w[0], a[1]-w[1]) for a, w in zip(arr, route)]
    mags = [math.hypot(*e) for e in errs]
    bias, scat = stats(errs)
    print(f"{name}\n  各地点の到着誤差(m): " + ", ".join(f"{m:.1f}" for m in mags) +
          f"  / 平均 {sum(mags)/len(mags):.1f}m")
    return raw, est

if __name__ == "__main__":
    route = [(15, 0), (15, 15), (0, 15), (0, 0)]
    print("模擬: GPSに一定のずれ(1.5m, -1.0m)と揺れ、車輪2%誤差、ジャイロ偏り0.3°/s を入れています\n")
    report("A. GPSだけ(到着判定2.0m)", route, fusion=False, arrive_m=2.0)
    report("B. GPSだけ(到着判定1.0m)", route, fusion=False, arrive_m=1.0)
    raw, est = report("C. センサー融合(到着判定1.0m)", route, fusion=True, arrive_m=1.0)
    raw, est = report("D. 融合 + スタート地点補正(到着判定1.0m)", route, fusion=True, arrive_m=1.0, calib=True)
    print("\n--- 位置の推定精度(走行中、本当の位置との差。Dの補正後) ---")
    b1, s1 = stats(raw); b2, s2 = stats(est)
    print(f"GPSそのまま : 一定のずれ {b1:.2f}m / 揺れ {s1:.2f}m")
    print(f"融合した推定: 一定のずれ {b2:.2f}m / 揺れ {s2:.2f}m")
    print("\n--- 融合のみ(補正なし、Cの設定)の位置推定 ---")
    r, e = run2(route, fusion=True, arrive_m=1.0)[1:]
    b1, s1 = stats(r); b2, s2 = stats(e)
    print(f"GPSそのまま : 一定のずれ {b1:.2f}m / 揺れ {s1:.2f}m")
    print(f"融合した推定: 一定のずれ {b2:.2f}m / 揺れ {s2:.2f}m")
