"""
段階5: 曲がり角・信号の手前での案内と停止(経路の形と、地図の信号の位置から求める)
座標は、スタート地点からの東(x)・北(y)メートル。方位は、北=0度で時計回り(右に曲がる=増える)。
単独でも動く: python smart_cart_stage5_events.py
"""
import math

def wrap(d): return (d + 180) % 360 - 180
def bearing(a, b): return math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360

def clean(pts):
    """同じ場所の連続した点を1つにする"""
    out = [tuple(pts[0])]
    for p in pts[1:]:
        if math.dist(p, out[-1]) > 1e-6: out.append(tuple(p))
    return out

def cumulative(pts):
    cum = [0.0]
    for a, b in zip(pts, pts[1:]): cum.append(cum[-1] + math.dist(a, b))
    return cum

def polyline_len(pts): return cumulative(pts)[-1] if len(pts) > 1 else 0.0

def point_at(pts, cum, s):
    s = max(0.0, min(cum[-1], s))
    lo, hi = 0, len(cum) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if cum[mid] <= s: lo = mid
        else: hi = mid
    seg = cum[hi] - cum[lo]
    f = 0.0 if seg <= 0 else (s - cum[lo]) / seg
    return (pts[lo][0] + (pts[hi][0] - pts[lo][0]) * f, pts[lo][1] + (pts[hi][1] - pts[lo][1]) * f)

def project(pts, cum, p, lo=0.0, hi=1e18):
    """点pに一番近い、経路上の位置。戻り値: (経路の始点からの距離s, 経路からの距離)。lo〜hiの範囲だけ調べる"""
    if len(pts) < 2: return 0.0, math.dist(p, pts[0])
    best = (None, 1e18)
    for i in range(len(pts) - 1):
        if cum[i + 1] < lo: continue
        if cum[i] > hi: break
        a, b = pts[i], pts[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]; L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
        d = math.hypot(p[0] - (a[0] + u * dx), p[1] - (a[1] + u * dy))
        if d < best[1]: best = (cum[i] + u * math.sqrt(L2), d)
    if best[0] is None:                                              # 範囲に線分がないとき
        s, d = project(pts, cum, p); return s, d
    return best

def find_turns(pts, window=10.0, min_deg=35.0, step=0.5):
    """経路の曲がり角を見つける。前後window(m)の進行方向の差が、min_deg以上の所"""
    pts = clean(pts); cum = cumulative(pts); L = cum[-1]
    if L < 2 * window + 1: return []
    samples, s = [], window
    while s <= L - window + 1e-9:
        a = wrap(bearing(point_at(pts, cum, s), point_at(pts, cum, s + window)) -
                 bearing(point_at(pts, cum, s - window), point_at(pts, cum, s)))
        samples.append((s, a)); s += step
    turns, group = [], []
    def flush():
        if group:
            s_, a_ = max(group, key=lambda t: abs(t[1])); x, y = point_at(pts, cum, s_)
            turns.append({"kind": "turn", "s": s_, "x": x, "y": y, "angle": a_, "dir": "右" if a_ > 0 else "左"})
    for s_, a_ in samples:
        if abs(a_) >= min_deg and (not group or (a_ > 0) == (group[0][1] > 0)): group.append((s_, a_))
        else:
            flush(); group = [(s_, a_)] if abs(a_) >= min_deg else []
    flush()
    return turns

def make_events(path, signals=(), stop_turn_deg=60.0, stop_before=4.0, near_signal=12.0):
    """経路から、案内・停止のイベントを作る。
       signals: (x, y, 種類) の列。種類は 'signal'(信号) か 'crossing'(横断歩道)。経路から20m以内のものだけ使う"""
    path = clean(path); cum = cumulative(path)
    ev = find_turns(path)
    for t in ev: t["stop"] = abs(t["angle"]) >= stop_turn_deg; t["stage"] = 0
    sig = []
    for x, y, kind in signals:
        s, d = project(path, cum, (x, y))
        if d > 20 or s - stop_before < 2.0: continue
        sig.append((s, kind))
    sig.sort()
    merged = []
    for s, kind in sig:                                               # 近くの信号と横断歩道は1つにまとめる(信号を優先)
        if merged and s - merged[-1][0] < near_signal:
            if kind == "signal": merged[-1] = (merged[-1][0], "signal")
            continue
        merged.append((s, kind))
    for s, kind in merged:
        x, y = point_at(path, cum, s - stop_before)
        ev.append({"kind": kind, "s": s - stop_before, "x": x, "y": y, "stage": 0})
    return sorted(ev, key=lambda e: e["s"])

def demo():
    print("=== 1: 曲がり角の検出 ===")
    cases = {
        "まっすぐ": [(0, 0), (40, 0)],
        "右に90度(直角)": [(0, 0), (20, 0), (20, -20)],
        "左に90度": [(0, 0), (0, 20), (-20, 20)],
        "ゆるい曲がり(20度)": [(0, 0), (20, 0), (40, 7.3)],
        "右に45度": [(0, 0), (20, 0), (34, -14)],
        "S字(右→左)": [(0, 0), (20, 0), (20, -20), (40, -20)],
        "Uターン(右に180度)": [(0, 0), (20, 0), (20, -3), (0, -3)],
    }
    for name, p in cases.items():
        t = find_turns(p); print(f"{name}: " + (", ".join(f"{e['dir']}{abs(e['angle']):.0f}度(経路の{e['s']:.0f}m地点)" for e in t) or "曲がり角なし"))
    arc = [(30 * math.sin(a / 57.3), 30 * (1 - math.cos(a / 57.3))) for a in range(0, 91, 3)]
    arc = [(0, 0), (15, 0)] + [(15 + 12.7 * math.sin(a / 57.3), -12.7 * (1 - math.cos(a / 57.3))) for a in range(0, 91, 5)] + [(27.7, -27.7)]
    t = find_turns(arc); print("なめらかな90度カーブ(半径約13m): ", ", ".join(f"{e['dir']}{abs(e['angle']):.0f}度" for e in t) or "なし")
    print("\n=== 2: 道の点が密(5m間隔)でも、同じ結果になるか ===")
    def dens(p, st):
        out = [p[0]]
        for a, b in zip(p, p[1:]):
            n = max(1, math.ceil(math.dist(a, b) / st)); out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
        return out
    for name in ("右に90度(直角)", "S字(右→左)"):
        a = find_turns(cases[name]); b = find_turns(dens(cases[name], 5))
        print(f"{name}: 粗い点 {[(e['dir'], round(abs(e['angle']))) for e in a]} / 密な点 {[(e['dir'], round(abs(e['angle']))) for e in b]}")
    print("\n=== 3: イベント(曲がり角で止まる/信号で止まる) ===")
    path = [(0, 0), (60, 0), (60, -40)]
    ev = make_events(path, [(30, 3, "signal"), (33, 2, "crossing"), (60, -20, "crossing"), (30, 40, "signal"), (1, 0, "signal")])
    for e in ev: print(f"  経路の{e['s']:5.1f}m地点 ({e['x']:5.1f},{e['y']:6.1f}) " + (f"曲がり角 {e['dir']}{abs(e['angle']):.0f}度 → {'一時停止して案内' if e['stop'] else '案内のみ'}" if e['kind'] == 'turn' else {'signal': '信号の手前で停止', 'crossing': '横断歩道の手前で停止'}[e['kind']]))
    print("  (信号と横断歩道が近いものは1つに / 経路から20m以上離れたものと、スタート直後のものは除外)")
    print("\n=== 4: 経路上の位置の求め方 ===")
    cum = cumulative(path); print("経路の全長", polyline_len(path), "m / (30,4)の位置:", [round(v, 1) for v in project(path, cum, (30, 4))], "/ (65,-10):", [round(v, 1) for v in project(path, cum, (65, -10))])

if __name__ == "__main__":
    demo()
