"""
AI電動スマート台車  操作ページ付きサーバー(段階1〜3を内蔵)
モード: 手動 / 自動アシスト / 追従 / 自動運転(GPSで決めた地点を順に走る)
使い方:  python cart_server.py          (このパソコンだけで使う)
         python cart_server.py --lan    (同じWi-Fiのスマホから使う)
必要なファイル(同じフォルダ): smart_cart_stage1.py, stage2.py, stage3_gps.py, stage3b_fusion.py
起動すると4桁の暗証番号が表示される。ブラウザでページを開き、その番号を入力する。
安全: ブラウザとの通信が0.5秒途絶えると、運転指令は停止になり、自動運転は一時停止になる。
"""
import json, math, random, secrets, sys, threading, time, os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import smart_cart_stage3_gps as g
import smart_cart_stage4_shortest as sp
import smart_cart_stage5_events as ev5
from smart_cart_stage3b_fusion import World2, PositionEstimator, wrap
from smart_cart_stage1 import Command, Sensors, SafetyGovernor, MotorController, SimDriver, Mode, MAX_SPEED
from smart_cart_stage2 import FollowController, SpeedHold

g.GEOFENCE_M = 1e9                                  # 36mの円の許可範囲はやめて、下の「経路のそばの帯」で見張る
g.ARRIVE_M, g.SLOW_M, g.TURN_IN_PLACE_DEG = 1.0, 1.0, 15                      # 到着の判定(m)・減速を始める距離(m)・その場で向きを変える角度(度)
REPLAN_AFTER, MAX_REPLANS = 2.0, 8                 # 障害物で止まってから迂回路を作るまでの秒数 / 迂回路を作り直せる回数
HZ = 20; DT = 1 / HZ
CALIB_SEC = 30                                     # 位置合わせの時間(台車を動かさない)
MAX_POINTS = 2000                                  # 経路の点の上限(長い道に沿った経路は点が多い)
MAX_ROUTE_M = 5000.0                               # 経路の長さの上限(m)
CORRIDOR_M = 25.0                                  # 経路からこの距離より離れたら一時停止(m)
AHEAD_M, TURN_NOW_M, TURN_STOP_SEC = 8.0, 2.5, 3.0  # 曲がり角の何m手前で案内するか / 何m手前で一時停止するか / 一時停止の秒数
HOLD_SAY = {"GPS途絶": "GPSの電波が途切れました。一時停止します。", "GPS精度不足": "GPSの精度が下がりました。一時停止します。",
            "向きが不明": "向きが分からなくなりました。一時停止します。", "見守りの通信が途絶": "通信が切れました。一時停止します。",
            "手動操作で一時停止": "手動の操作を受け付けました。一時停止します。", "非常停止": "非常停止しました。",
            "進路がふさがれています": "前に進めません。一時停止します。", "通れる道がありません": "通れる道が見つかりません。一時停止します。",
            "経路から外れました": "経路から外れました。一時停止します。"}
REQ_TIMEOUT = 0.5                                  # 操作・見守りの通信がこの秒数途絶えたら停止
BLOCKED_LIMIT = 10.0                               # 自動運転で進路がこの秒数ふさがれたら一時停止
ROUTE = [(15, 0), (15, 15), (0, 15), (0, 0)]       # 自動運転の経路(スタートからの東・北メートル)
OBSTACLE = (8.0, 0.0, 0.3)                         # 模擬: 進路上の障害物(位置合わせ後、台車は線の上を走る)

# ---------- 世界(今は模擬。実機では本物のセンサー・ドライバに差し替える) ----------
class ServerWorld(World2):
    """2次元の屋外模擬世界。GPSの誤差、前方の人、障害物つき"""
    def __init__(self, nav):
        super().__init__(nav, heading=90.0); self.slope = 0.0; self.reset()
    def reset(self):
        self.x = self.y = self.v = self.nx = self.ny = 0.0; self.h = 90.0
        self.px, self.py, self.pspeed = 2.3, 0.0, 0.0
        self.static, self.slope, self.gps_down, self.hdop = [], 0.0, False, 1.2
    def place_person(self, ahead=2.3):
        hr = math.radians(self.h)
        self.px, self.py, self.pspeed = self.x + ahead * math.sin(hr), self.y + ahead * math.cos(hr), 0.0
    def read(self, now, estop):
        self.obs = self.static + [(self.px, self.py, 0.3)]
        d = self.sonar_cm()
        return Sensors(d, d, d, 200, 200, d <= 3, estop, now)
    def step(self, driver, dt):
        l, r = driver.left, driver.right; avg = (l + r) / 2
        tv = 0.0 if avg == 0 else avg - (self.slope if avg > 0 else 0.0)
        h0, x0, y0 = self.h, self.x, self.y
        self.v += (tv - self.v) * (1 - 0.7 ** (dt / 0.1))
        self.h = (self.h + (l - r) * 86 * dt) % 360
        hr = math.radians(self.h)
        self.x += self.v * math.sin(hr) * dt; self.y += self.v * math.cos(hr) * dt
        self.px += self.pspeed / 100 * dt
        for ox, oy, rr in self.static + [(self.px, self.py, 0.3)]:      # めり込み防止
            dx, dy = self.x - ox, self.y - oy; dd = math.hypot(dx, dy)
            if 0 < dd < 0.25 + rr: self.x, self.y = ox + dx / dd * (0.25 + rr), oy + dy / dd * (0.25 + rr); self.v = 0.0
        odo = math.hypot(self.x - x0, self.y - y0) * 1.02 + random.gauss(0, 0.002)
        rate = wrap(self.h - h0) / dt + 0.3 + random.gauss(0, 0.5)
        return odo, rate

# ---------- 台車アプリ本体 ----------
class CartApp:
    def __init__(self):
        self.lock = threading.Lock()
        self.driver = SimDriver(); self.nav = g.Navigator(35.0, 135.0)
        self.world = ServerWorld(self.nav)
        self.motor = MotorController(self.driver); self.gov = SafetyGovernor()
        self.follow = FollowController(); self.hold = SpeedHold(); self.use_hold = True
        self.route_xy = list(ROUTE)
        self.planner = sp.Planner(radius=1e9); self.detour = True
        self.say_seq, self.says, self.rng = 0, [], random.Random()
        self.cmd = Command(0, 0, 0); self.dist = 0.0
        self._fresh_state()

    def _fresh_state(self):
        self.nav.set_route_latlon([self.nav.to_latlon(*p) for p in self.route_xy])
        self.est = PositionEstimator(self.nav)
        self.mode, self.estop, self.blocked = "manual", False, 0.0
        self.calib, self.off, self.moved = None, (0.0, 0.0), False   # 位置合わせの状態
        self.planner.clear(); self.replans = 0                        # 見つけた障害物の記憶
        self.s_prog, self.wait, self.go_flag, self.pause_until = 0.0, None, False, 0.0
        self._ns = self.nav.state
        self._build_events([])
        self.req, self.req_time, self.odo, self.rate = (0.0, 0.0), 0.0, 0.0, 0.0

    def say(self, text, urgent=False):
        """案内の文を出す(ページが順番に読み上げる)"""
        self.say_seq += 1; self.says.append({"id": self.say_seq, "text": text, "u": urgent}); self.says = self.says[-12:]

    def _build_events(self, signals):
        """経路の形(曲がり角)と、地図の信号・横断歩道から、案内と停止の場所を作る"""
        self.corridor = ev5.clean([(0.0, 0.0)] + list(self.route_xy)); self.corr_cum = ev5.cumulative(self.corridor)
        self.events = ev5.make_events(self.corridor, signals); self.ev_i = 0

    def _events(self, now):
        """曲がり角・信号の案内と停止。戻り値は速度の倍率(0なら止まる)"""
        if self.ev_i >= len(self.events): return 1.0
        e = self.events[self.ev_i]; ahead = e["s"] - self.s_prog          # 経路に沿って、あと何m
        if e["kind"] == "turn":
            how = "大きく" if abs(e["angle"]) >= 120 else ""
            if ahead < -8.0 and e["stage"] < 2: self.ev_i += 1; return 1.0
            if e["stage"] == 0 and ahead <= AHEAD_M:
                e["stage"] = 1; self.say(self.rng.choice([f"この先、{e['dir']}に曲がります。", f"まもなく、{e['dir']}に曲がります。"]))
            if e["stage"] <= 1 and ahead <= TURN_NOW_M:
                if e["stop"]:
                    e["stage"] = 2; self.pause_until = now + TURN_STOP_SEC; self.say("曲がり角です。一時停止して、左右を確認します。")
                else:
                    e["stage"] = 3; self.say(f"{e['dir']}に{how}曲がります。"); self.ev_i += 1; return 1.0
            if e["stage"] == 2:
                if now < self.pause_until: return 0.0
                e["stage"] = 3; self.say(f"{e['dir']}に{how}曲がります。"); self.ev_i += 1; return 1.0
            return 0.5 if ahead <= 5.0 else 1.0
        sig = e["kind"] == "signal"; name = "信号" if sig else "横断歩道"
        if ahead < -8.0 and e["stage"] < 2:
            self.say(f"{name}の位置を通り過ぎました。一時停止します。", True); self.ev_i += 1; self.nav._hold(f"{name}を通り過ぎました", now); return 0.0
        if e["stage"] == 0 and ahead <= 12.0:
            e["stage"] = 1; self.say(f"この先に{name}があります。")
        if e["stage"] <= 1 and ahead <= 1.0:
            e["stage"] = 2; self.wait = e["kind"]
            self.say(f"{name}の手前で止まります。{'信号の色を確認して' if sig else '左右を確認して'}、安全なら「進む」を押してください。", True)
        if e["stage"] == 2:
            if self.go_flag:
                self.go_flag, self.wait, e["stage"] = False, None, 3; self.ev_i += 1; self.say("進みます。"); return 1.0
            return 0.0
        return 0.6 if ahead <= 6.0 else 1.0

    def set_mode(self, m):
        if m == "follow":
            s = self.world.read(time.time(), self.estop)
            if not self.follow.start(s):
                return "追従を始められません。正面80〜250cmに人を置いてください。"
        else:
            self.follow.stop()
        if m == "auto":
            self.nav.set_route_latlon([self.nav.to_latlon(*p) for p in self.route_xy])
            self.world.px, self.world.py, self.world.pspeed = -6.0, -6.0, 0.0   # 模擬の人を経路から外す
            self.blocked = 0.0
            msg = "自動運転を選びました。周囲の安全を確認して「開始」を押してください。"
        else:
            self.nav.state = g.NavState.IDLE; msg = ""
        self.mode = m
        self.gov.set_mode(Mode.ASSIST if m == "assist" else Mode.MANUAL)
        return msg

    def _calibrate(self, fix, now):
        """台車がスタート地点(地図でタップした場所)にいる前提で、GPSの一定のずれを測る"""
        c = self.calib
        if self.estop or self.mode != "auto" or abs(self.req[0]) > 0 or abs(self.req[1]) > 0:
            self.calib = None; self.nav.reason = "位置合わせを中止しました"; self.say("位置合わせを中止しました。"); return
        if fix is not None:
            x, y = self.nav.to_xy(fix.lat, fix.lon); c["xs"].append(x); c["ys"].append(y)
        left = CALIB_SEC - (now - c["t0"])
        if left > 0:
            self.nav.reason = f"位置合わせ中(あと{left:.0f}秒。台車を動かさないでください)"; return
        self.calib = None
        if len(c["xs"]) < 100:
            self.nav.reason = "位置合わせに失敗しました(GPSを十分に受信できません)"; self.say("位置合わせに失敗しました。GPSを十分に受信できません。", True); return
        self.off = (sum(c["xs"]) / len(c["xs"]), sum(c["ys"]) / len(c["ys"]))
        self.est = PositionEstimator(self.nav)
        self.say("位置合わせが終わりました。"); self.nav.start(); self.moved = True

    def _replan(self, s):
        """ソナーで見つけた障害物を覚えて、そこを避ける最短の道に作り直す(台車のまわり30mだけを計画する)"""
        if not self.est.ready: return False
        hr = math.radians(self.est.h); d = s.front_center / 100 + 0.45      # 表面までの距離 + 障害物の半径(仮)
        cx, cy = self.est.x, self.est.y
        self.planner.add_obstacle(cx + d * math.sin(hr), cy + d * math.cos(hr))
        pl = sp.Planner(radius=30.0)
        for ox, oy in self.planner.obs:
            if math.hypot(ox - cx, oy - cy) < 45: pl.add_obstacle(ox - cx, oy - cy, merge=0.0)
        tail, here, new, planned = self.nav.wps[self.nav.i:], (0.0, 0.0), [], 0
        for w in tail:
            lw = (w[0] - cx, w[1] - cy); far = math.hypot(*lw)
            if far > 25: lw = (lw[0] / far * 20, lw[1] / far * 20)          # 遠い目標は、20m先までを計画する
            path = pl.path(here, lw)
            if path is None: return False
            new += [(px + cx, py + cy) for px, py in sp.Planner.densify(path, 1.5)[1:]]; here = lw
            if far > 25: break
            planned += 1
        wps = new + tail[planned:]
        if len(wps) > MAX_POINTS: return False
        self.nav.wps, self.nav.i, self.route_xy = wps, 0, list(wps)
        self.replans += 1; self.blocked = 0.0
        self.nav.reason = f"障害物をよける最短の道を作りました({self.replans}回目)"
        self.say(self.rng.choice(["前方に障害物があります。よけて進みます。", "障害物を見つけました。迂回します。"]))
        return True

    def tick(self):
        now = time.time()
        with self.lock:
            s = self.world.read(now, self.estop); self.dist = s.front_center
            fix = self.world.gps(now)
            if self.calib is not None: self._calibrate(fix, now)
            if fix is not None and self.off != (0.0, 0.0):          # 位置合わせで求めたずれを引く
                x, y = self.nav.to_xy(fix.lat, fix.lon)
                fix.lat, fix.lon = self.nav.to_latlon(x - self.off[0], y - self.off[1])
            self.est.update(fix, self.odo, self.rate, DT)
            fresh = now - self.req_time < REQ_TIMEOUT
            manual_in = abs(self.req[0]) > 0 or abs(self.req[1]) > 0
            if self.mode == "follow":
                req = self.follow.compute(s, now)
            elif self.mode == "auto":
                if self.nav.state == g.NavState.RUNNING:
                    if not fresh: self.nav._hold("見守りの通信が途絶", now)
                    elif manual_in: self.nav._hold("手動操作で一時停止", now)
                if self.nav.state == g.NavState.RUNNING and self.est.ready:   # 経路のそばの帯から出ていないか・経路に沿った進み具合
                    s_c, dist = ev5.project(self.corridor, self.corr_cum, (self.est.x, self.est.y), max(0.0, self.s_prog - 10.0), self.s_prog + 60.0)
                    self.s_prog = max(self.s_prog, s_c)
                    if dist > CORRIDOR_M: self.nav._hold("経路から外れました", now)
                use = fix
                if fix is not None and self.est.ready:       # 融合した位置・向きを使う
                    la, lo = self.nav.to_latlon(self.est.x, self.est.y)
                    use = g.GpsFix(la, lo, fix.fix_ok, fix.hdop, fix.sats, self.est.h, fix.timestamp)
                req = self.nav.compute(use, now)
                if self.nav.state == g.NavState.RUNNING and self.est.ready:   # 曲がり角・信号の案内と停止
                    k = self._events(now)
                    if k != 1.0: req = Command(req.speed * k, req.turn if k > 0 else 0.0, now)
                    if self.wait: self.nav.reason = f"{'信号' if self.wait == 'signal' else '横断歩道'}の手前で待機中(安全を確認して「進む」を押してください)"
                    elif now < self.pause_until: self.nav.reason = "曲がり角で一時停止中"
                if self.nav.state != g.NavState.RUNNING: req = Command(0.0, 0.0, now)
            elif fresh: req = Command(self.req[0], self.req[1], now)
            else: req = Command(0.0, 0.0, now)
            if self.use_hold and req.speed > 0:
                req = Command(self.hold.adjust(min(req.speed, MAX_SPEED), self.world.v, DT), req.turn, now)
            self.cmd = self.gov.decide(req, s, now)
            if self.mode == "auto" and self.nav.state == g.NavState.RUNNING and req.speed > 0 and self.cmd.speed < 0.02:
                self.blocked += DT                            # 障害物で動けない時間
                if (self.detour and self.blocked > REPLAN_AFTER and self.replans < MAX_REPLANS and "障害物" in self.gov.reason
                        and s.front_center is not None and s.front_center < 150):
                    if not self._replan(s): self.nav._hold("通れる道がありません", now)
                if self.blocked > BLOCKED_LIMIT: self.nav._hold("進路がふさがれています", now)
            else:
                self.blocked = 0.0
            self.motor.update(self.cmd, now)
            self.odo, self.rate = self.world.step(self.driver, DT)
            if self.nav.state != self._ns:                           # 状態が変わったら、案内する
                if self.nav.state == g.NavState.RUNNING: self.say("走行を再開します。" if self._ns == g.NavState.HOLD else "出発します。")
                elif self.nav.state == g.NavState.HOLD: self.say(HOLD_SAY.get(self.nav.reason, f"一時停止します。{self.nav.reason}"), True)
                elif self.nav.state == g.NavState.ARRIVED: self.say("目的地に到着しました。")
                self._ns = self.nav.state

    def state(self):
        w, n = self.world, self.nav
        k = max(1, len(self.route_xy) // 150)                      # ミニ地図用に、点を間引く
        return {"mode": self.mode, "reason": self.gov.reason, "estop": self.estop,
                "cmd": round(self.cmd.speed, 2), "turn": round(self.cmd.turn, 2),
                "v": round(w.v * 100, 1), "dist": round(self.dist, 0),
                "follow": self.follow.state.value, "hold": self.use_hold,
                "slope": w.slope > 0, "target_speed": w.pspeed,
                "nav": n.state.value, "nav_reason": n.reason, "wp": [min(n.i, len(self.route_xy)), len(self.route_xy)],
                "pos": [round(w.x, 2), round(w.y, 2), round(w.h, 1)], "person": [round(w.px, 2), round(w.py, 2)],
                "route": self.route_xy[::k], "rstep": k, "ll": list(n.to_latlon(w.x, w.y)), "origin": [n.o_lat, n.o_lon],
                "route_ll": [list(n.to_latlon(*p)) for p in self.route_xy[::k]], "obstacle": [w.static[0][0], w.static[0][1]] if w.static else None, "gps_down": w.gps_down, "gps_bad": w.hdop > g.HDOP_MAX, "calib": self.calib is not None,
                "known_obs": [[round(x, 2), round(y, 2)] for x, y in self.planner.obs], "replans": self.replans, "shortest": self.detour,
                "wait": self.wait, "ev_left": len(self.events) - self.ev_i, "says": self.says[-8:]}

    def set_route(self, r):
        """地図で選んだ経路(緯度経度)を受け取る。日本国内で、長さ5km以内。信号・横断歩道の位置(signals)も受け取る"""
        if self.nav.state == g.NavState.RUNNING:
            return "走行中は経路を変更できません。先に一時停止してください。"
        try:
            olat, olon = float(r["origin"][0]), float(r["origin"][1])
            pts = [(float(a), float(b)) for a, b in r["points"]]
        except (KeyError, TypeError, ValueError, IndexError):
            return "経路のデータが正しくありません。"
        if not all(math.isfinite(v) for v in [olat, olon] + [c for p in pts for c in p]):
            return "経路のデータが正しくありません。"
        if not (20 <= olat <= 46 and 122 <= olon <= 154):
            return "日本国内の場所を選んでください。"
        if not 1 <= len(pts) <= MAX_POINTS:
            return f"経路の点は1〜{MAX_POINTS}か所にしてください。"
        tmp = g.Navigator(olat, olon)
        xy = [tmp.to_xy(a, b) for a, b in pts]
        if any(math.hypot(x, y) > MAX_ROUTE_M for x, y in xy):
            return f"スタート地点から{MAX_ROUTE_M / 1000:.0f}km以内を選んでください。"
        sig = []
        try:
            for q in list(r.get("signals", []))[:300]:
                la, lo = float(q["lat"]), float(q["lon"])
                if math.isfinite(la) and math.isfinite(lo):
                    x, y = tmp.to_xy(la, lo); sig.append((x, y, "signal" if q.get("type") == "signal" else "crossing"))
        except (KeyError, TypeError, ValueError, AttributeError):
            return "信号のデータが正しくありません。"
        shortest, road, note = bool(r.get("shortest", True)), bool(r.get("road", False)), ""
        if shortest and not road and 1 < len(xy) <= 8:        # 行き先の順番を、合計が最短になるように並べ替える
            pl = sp.Planner(radius=1e9); idx = pl.order((0.0, 0.0), xy)
            before, after = pl.order_length((0.0, 0.0), xy, list(range(len(xy)))), pl.order_length((0.0, 0.0), xy, idx)
            if idx != list(range(len(xy))):
                xy = [xy[k] for k in idx]
                note = f"順番を最適化しました({'→'.join(str(k + 1) for k in idx)}、約{before - after:.0f}m短縮)。"
        if ev5.polyline_len([(0.0, 0.0)] + xy) > MAX_ROUTE_M:
            return f"経路が長すぎます(最大{MAX_ROUTE_M / 1000:.0f}km)。"
        self.nav.o_lat, self.nav.o_lon, self.route_xy = olat, olon, xy
        estop = self.estop
        self.world.reset(); self.follow.stop(); self._fresh_state(); self.estop = estop
        self.world.px, self.world.py = -6.0, -6.0
        self._build_events(sig)
        self.set_mode("auto"); self.detour = shortest
        n_t = sum(1 for e in self.events if e["kind"] == "turn"); n_s = sum(1 for e in self.events if e["kind"] == "signal"); n_c = sum(1 for e in self.events if e["kind"] == "crossing")
        info = f"経路の長さは約{ev5.polyline_len(self.corridor):.0f}m。曲がり角{n_t}か所、信号{n_s}か所、横断歩道{n_c}か所。信号と横断歩道の手前では、必ず止まります。"
        return f"経路を設定しました({len(xy)}点)。{note}{info}周囲の安全を確認して、「開始」を押してください。"

    def handle(self, d):
        msg, now = "", time.time()
        with self.lock:
            if "speed" in d:
                self.req = (float(d["speed"]), float(d.get("turn", 0))); self.req_time = now
            if "estop" in d:
                self.estop = bool(d["estop"])
                if self.estop and self.nav.state == g.NavState.RUNNING: self.nav._hold("非常停止", now)
            if "mode" in d and d["mode"] in ("manual", "assist", "follow", "auto"): msg = self.set_mode(d["mode"])
            if d.get("auto_start"):
                if self.mode != "auto": msg = "先に「自動運転」モードを選んでください。"
                elif self.estop: msg = "非常停止中は開始できません。"
                elif self.nav.i == 0 and not self.moved and self.calib is None:
                    self.calib = {"t0": now, "xs": [], "ys": []}      # 最初の開始は、先に位置合わせをする
                    self.nav.reason = f"位置合わせを始めます(約{CALIB_SEC}秒。台車を動かさないでください)"
                    msg = f"位置合わせを始めます。約{CALIB_SEC}秒、台車に触れず、動かさないでください。"
                    self.say("位置合わせをします。台車を動かさないでください。")
                elif self.calib is None:
                    self.nav.start(); self.moved = True
            if d.get("go_on") and self.wait: self.go_flag = True       # 信号・横断歩道で待っているときだけ有効
            if d.get("auto_stop") and self.mode == "auto":
                self.nav.state, self.nav.reason = g.NavState.IDLE, "停止"
            if "hold" in d: self.use_hold = bool(d["hold"])
            if "slope" in d: self.world.slope = 0.2 if d["slope"] else 0.0
            if "target_speed" in d: self.world.pspeed = float(d["target_speed"])
            if "gps_down" in d: self.world.gps_down = bool(d["gps_down"])
            if "gps_bad" in d: self.world.hdop = 5.0 if d["gps_bad"] else 1.2
            if "obstacle" in d: self.world.static = [OBSTACLE] if d["obstacle"] else []
            if d.get("place_person"): self.world.place_person()
            if "route" in d: msg = self.set_route(d["route"])
            if d.get("reset"):
                self.world.reset(); self.follow.stop(); self.nav.state = g.NavState.IDLE; self._fresh_state()
                self.gov.set_mode(Mode.MANUAL)
        return msg

app = CartApp(); PIN = f"{secrets.randbelow(10000):04d}"

def control_loop():
    while True:
        t0 = time.time(); app.tick()
        time.sleep(max(0, DT - (time.time() - t0)))

# ---------- Webサーバー ----------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _send(self, code, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code); self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
    def _auth(self, pin): return secrets.compare_digest(str(pin), PIN)
    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/": return self._send(200, PAGE, "text/html")
        if u.path == "/api/state":
            if not self._auth(parse_qs(u.query).get("pin", [""])[0]): return self._send(403, '{"error":"pin"}')
            return self._send(200, json.dumps(app.state()))
        self._send(404, "{}")
    def do_POST(self):
        if urlparse(self.path).path != "/api/cmd": return self._send(404, "{}")
        try: d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        except Exception: return self._send(400, '{"error":"bad json"}')
        if not self._auth(d.get("pin", "")): return self._send(403, '{"error":"pin"}')
        try: msg = app.handle(d)
        except (ValueError, TypeError): return self._send(400, '{"error":"bad value"}')
        s = app.state(); s["msg"] = msg; self._send(200, json.dumps(s))

PAGE = """<!DOCTYPE html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><title>台車 操作ページ</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>:root{--bg:#f4f6f9;--cd:#fff;--tx:#1b2230;--mu:#5b6575;--bd:#d6dbe3;--ac:#1f6feb;--ng:#cf222e;--ok:#1a7f37;--wn:#b25e00;color-scheme:light dark}
@media(prefers-color-scheme:dark){:root{--bg:#0e131a;--cd:#171e28;--tx:#e6eaf0;--mu:#97a1b1;--bd:#2c3644;--ac:#58a6ff;--ng:#ff7b72;--ok:#3fb950;--wn:#e3a008}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:system-ui,"Hiragino Sans","Noto Sans JP",sans-serif;padding:12px}
.c{background:var(--cd);border:1px solid var(--bd);border-radius:12px;padding:12px;margin:0 auto 10px;max-width:520px}
button{font:inherit;padding:10px 14px;border-radius:8px;border:1px solid var(--bd);background:var(--cd);color:var(--tx)}
button.on{background:var(--ac);color:#fff;border-color:var(--ac)}#es.on{background:var(--ng);border-color:var(--ng)}
.r{display:flex;flex-wrap:wrap;gap:8px}.pad{display:grid;grid-template-columns:repeat(3,72px);gap:8px;justify-content:center}
.pad button{height:64px;font-size:1.4rem;touch-action:none;user-select:none}.big{font-size:1.1rem;font-weight:600}.mu{color:var(--mu);font-size:.85rem}
input{font:inherit;padding:8px;width:7em}label{font-size:.9rem;display:flex;gap:4px;align-items:center}
canvas{width:100%;height:auto;border-radius:8px;background:var(--bg);display:block}
.pin{width:24px;height:24px;border-radius:12px;background:#1f6feb;color:#fff;font:600 12px/20px sans-serif;text-align:center;border:2px solid #fff}.pin.s{background:#1a7f37}select{font:inherit;padding:8px}</style></head><body>
<div class="c" id="lg">暗証番号(起動時に表示された4桁): <input id="pin" inputmode="numeric" maxlength="4"> <button onclick="go()">接続</button></div>
<div class="c"><div class="r" id="md"><button data-m="manual">手動</button><button data-m="assist">自動アシスト</button><button data-m="follow">追従</button><button data-m="auto">自動運転</button><button id="es">非常停止</button></div>
<label style="margin-top:8px"><input type="checkbox" id="vc" style="width:auto">音声案内(アナウンス)</label>
<p class="mu" id="msg"></p></div>
<div class="c"><b>自動運転</b>
<div class="r" style="margin:8px 0"><button id="as">▶ 開始・再開</button><button id="ap">■ 一時停止</button><button id="go">▶ 進む(信号・横断歩道の確認後)</button></div>
<div>状態: <span class="big" id="nv">-</span></div><div class="mu" id="nr"></div><div class="mu" id="sy"></div>
<canvas id="cv" width="480" height="320"></canvas>
<p class="mu">周囲の安全を確認し、台車から目を離さないでください。最初の「開始」では、約30秒の位置合わせ(GPSのずれの測定)をします。台車を、地図でタップしたスタート地点に置き、動かさないでください。ページを閉じる・通信が切れる・手動で操作すると、一時停止します。</p></div>
<div class="c"><b>地図で場所を選ぶ(日本・無料)</b>
<div class="r" style="margin:8px 0"><select id="mp"><option value="gstd">国土地理院 地図</option><option value="gpho">国土地理院 航空写真</option></select></div>
<label style="margin-bottom:6px"><input type="checkbox" id="sh" checked style="width:auto">最短ルート(行き先の順番を最適化・障害物はよけて最短で迂回)</label>
<label style="margin-bottom:6px"><input type="checkbox" id="nosig" style="width:auto">信号・横断歩道の情報を取得できないときは、この経路に信号・横断歩道がないことを、自分で確認した</label>
<div class="r"><button id="m_o" class="on">① スタート地点</button><button id="m_w">② 行き先を追加</button><button id="m_r">🛣 道に沿う</button><button id="m_c">消去</button><button id="m_s">台車に送る</button></div>
<div class="r" style="margin-top:8px"><input id="q" placeholder="場所・住所(例: 東京駅)" style="flex:1;min-width:10em;width:auto"><button id="qb">検索</button></div>
<div id="qr" style="margin-top:6px;max-height:180px;overflow:auto;display:flex;flex-direction:column;gap:4px"></div>
<div id="map" style="height:320px;margin-top:8px;border-radius:8px;background:var(--bg)"></div>
<p class="mu" id="mm">場所を検索するか、地図を拡大して、台車を置く場所をタップしてください。</p>
<p class="mu">検索: 国土地理院と OpenStreetMap(© OpenStreetMap contributors)の両方から、似た名前も含めて探します。緯度,経度(例: 35.68, 139.76)も入力できます。</p>
<p class="mu">「道に沿う」: 行き先まで、地図の道(歩行者向け)に沿った経路を作ります(OpenStreetMap / FOSSGIS の無料サービス)。地図の「道」は、人や車がいない道とは限りません。管理された敷地内の道だけで使い、台車から目を離さないでください。</p></div>
<div class="c"><div class="pad"><span></span><button data-k="u">▲</button><span></span><button data-k="l">◀</button><button data-k="d">▼</button><button data-k="r">▶</button></div>
<p class="mu">ボタンを押している間だけ指令が出ます。指を離す・通信が切れると停止します。</p></div>
<div class="c"><div>AIの判断: <span class="big" id="rs">-</span></div>
<div>前方の距離: <b id="ds">-</b> cm / 速度: <b id="vv">-</b> cm/s / 追従: <b id="fl">-</b></div></div>
<div class="c"><div class="r"><label><input type="checkbox" id="hd" checked style="width:auto">速度維持</label>
<label><input type="checkbox" id="sl" style="width:auto">坂道(模擬)</label>
<label><input type="checkbox" id="gd" style="width:auto">GPS途絶(模擬)</label>
<label><input type="checkbox" id="gb" style="width:auto">GPS精度悪化(模擬)</label>
<label><input type="checkbox" id="ob" style="width:auto">進路に障害物(模擬)</label></div>
<div class="r" style="margin-top:8px"><button id="pp">前の人を正面に置く</button><button id="t0">前の人: 止まる</button><button id="t1">前の人: 歩く</button><button id="rst">リセット</button></div>
<p class="mu">このカードは模擬世界の操作です。実機では使いません。</p></div>
<script>
let pin="",k={u:0,d:0,l:0,r:0},extra={},ok=false;const $=i=>document.getElementById(i);
let vc=false,lk="",lm="",lf="",down=false,lastSay={},lastId=0,voice=null,firstPoll=true;
function pickVoice(){
 const vs=(window.speechSynthesis?speechSynthesis.getVoices():[]).filter(v=>(v.lang||"").toLowerCase().replace("_","-").startsWith("ja"));
 const pref=["Microsoft Nanami","Microsoft Haruka","Microsoft Ayumi","Google 日本語","Kyoko","O-Ren","Haruka","Sayaka","Nanami"];
 for(const p of pref){const v=vs.find(x=>x.name.includes(p));if(v){voice=v;return}}
 voice=vs[0]||null;
}
if(window.speechSynthesis){pickVoice();speechSynthesis.onvoiceschanged=pickVoice}
function say(t,u){
 if(!vc||!window.speechSynthesis)return;
 const n=Date.now();if(lastSay[t]&&n-lastSay[t]<3000)return;lastSay[t]=n;
 if(u)speechSynthesis.cancel();else if(speechSynthesis.pending)return;      // 急ぎでない案内は、たまっているときは飛ばす
 const x=new SpeechSynthesisUtterance(t);x.lang="ja-JP";if(voice)x.voice=voice;x.rate=0.95;x.pitch=1;x.volume=1;
 speechSynthesis.speak(x);
}
$("vc").onchange=e=>{vc=e.target.checked;if(vc)say("音声案内をオンにしました。",1)};
const MN={manual:"手動モードにしました。",assist:"自動アシストモードにしました。",follow:"追従モードにしました。",auto:"自動運転モードにしました。"};
const AT={es:["非常停止しました。",1],bp:["バンパーに当たりました。停止します。",1],fault:["センサーの異常を検知しました。停止します。",1],stop:["前に障害物があるので、停止しました。",1],slow:["前に障害物があります。速度を落とします。",0]};
function ann(s){const r=s.reason||"";let q="ok";
if(r.includes("非常停止"))q="es";else if(r.includes("バンパー"))q="bp";else if(r.includes("異常"))q="fault";
else if(/障害物.*停止/.test(r))q="stop";else if(r.includes("減速"))q="slow";
if(q!=lk){if(AT[q])say(AT[q][0],AT[q][1]);lk=q}
if(s.mode!=lm){if(lm)say(MN[s.mode],0);lm=s.mode}
if(s.follow!=lf){if(s.follow=="追従中")say("人についていきます。",0);else if(s.follow.startsWith("見失い"))say("人を見失いました。止まります。",1);lf=s.follow}
const sy=s.says||[];                                                   // サーバーからの案内(曲がり角・信号・出発・到着など)
if(sy.length){const top=sy[sy.length-1];if(top.id<lastId)lastId=0;
 sy.forEach(m=>{if(m.id>lastId){lastId=m.id;if(!firstPoll)say(m.text,m.u)}});$("sy").textContent="案内: "+top.text}
firstPoll=false}

function go(){pin=$("pin").value;ok=true;$("lg").style.display="none"}
document.querySelectorAll("[data-m]").forEach(b=>b.onclick=()=>extra.mode=b.dataset.m);
let es=false;$("es").onclick=()=>{es=!es;extra.estop=es};
$("go").onclick=()=>extra.go_on=true;$("as").onclick=()=>extra.auto_start=true;$("ap").onclick=()=>extra.auto_stop=true;
$("hd").onchange=e=>extra.hold=e.target.checked;$("sl").onchange=e=>extra.slope=e.target.checked;
$("gd").onchange=e=>extra.gps_down=e.target.checked;$("gb").onchange=e=>extra.gps_bad=e.target.checked;$("ob").onchange=e=>extra.obstacle=e.target.checked;
$("pp").onclick=()=>extra.place_person=true;$("t0").onclick=()=>extra.target_speed=0;$("t1").onclick=()=>extra.target_speed=30;
$("rst").onclick=()=>{extra.reset=true;es=false;["sl","gd","gb","ob"].forEach(i=>$(i).checked=false)};
document.querySelectorAll("[data-k]").forEach(b=>{const n=b.dataset.k;b.onpointerdown=e=>{e.preventDefault();k[n]=1};
["onpointerup","onpointerleave","onpointercancel"].forEach(t=>b[t]=()=>k[n]=0)});
const M={ArrowUp:"u",ArrowDown:"d",ArrowLeft:"l",ArrowRight:"r"};
onkeydown=e=>{if(e.target.tagName==="INPUT")return;if(M[e.key]){k[M[e.key]]=1;e.preventDefault()}};onkeyup=e=>{if(M[e.key])k[M[e.key]]=0};
function draw(s){const c=$("cv"),x=c.getContext("2d"),W=c.width,H=c.height,cs=getComputedStyle(document.documentElement);
const tx=cs.getPropertyValue("--tx"),ac=cs.getPropertyValue("--ac"),mu=cs.getPropertyValue("--mu"),ok2=cs.getPropertyValue("--ok");
x.clearRect(0,0,W,H);const P=[[0,0],...s.route,[s.pos[0],s.pos[1]],s.person];
const x0=Math.min(...P.map(p=>p[0]))-4,x1=Math.max(...P.map(p=>p[0]))+4,y0=Math.min(...P.map(p=>p[1]))-4,y1=Math.max(...P.map(p=>p[1]))+4;
const sc=Math.min(W/(x1-x0),H/(y1-y0)),X=a=>(a-x0)*sc,Y=b=>H-(b-y0)*sc;
x.strokeStyle=mu;x.lineWidth=2;x.setLineDash([6,4]);x.beginPath();x.moveTo(X(0),Y(0));s.route.forEach(p=>x.lineTo(X(p[0]),Y(p[1])));x.stroke();x.setLineDash([]);
x.font="13px sans-serif";x.textAlign="center";x.textBaseline="middle";
const dn=s.route.length>12;s.route.forEach((p,i)=>{x.fillStyle=(i==Math.floor(s.wp[0]/(s.rstep||1))&&s.nav!="全地点に到着")?ac:mu;x.beginPath();x.arc(X(p[0]),Y(p[1]),dn?3:9,0,7);x.fill();if(!dn){x.fillStyle="#fff";x.fillText(i+1,X(p[0]),Y(p[1]))}});
if(s.obstacle){x.fillStyle=mu;x.beginPath();x.arc(X(s.obstacle[0]),Y(s.obstacle[1]),7,0,7);x.fill()}
(s.known_obs||[]).forEach(o=>{x.globalAlpha=0.5;x.fillStyle=cs.getPropertyValue("--ng");x.beginPath();x.arc(X(o[0]),Y(o[1]),Math.max(5,0.5*sc),0,7);x.fill();x.globalAlpha=1});
x.fillStyle=ok2;x.beginPath();x.arc(X(s.person[0]),Y(s.person[1]),6,0,7);x.fill();
x.save();x.translate(X(s.pos[0]),Y(s.pos[1]));x.rotate(s.pos[2]*Math.PI/180);x.fillStyle=s.estop?cs.getPropertyValue("--ng"):tx;
x.beginPath();x.moveTo(0,-11);x.lineTo(8,9);x.lineTo(-8,9);x.closePath();x.fill();x.restore()}
async function tick(){if(!ok)return;const body={pin,speed:k.u?1:k.d?-1:0,turn:k.r?1:k.l?-1:0,...extra};extra={};
try{const r=await fetch("/api/cmd",{method:"POST",body:JSON.stringify(body)});
if(r.status==403){$("msg").textContent="暗証番号が違います。";ok=false;$("lg").style.display="";return}
const s=await r.json();down=false;ann(s);draw(s);if(s.ll)updateCart(s.ll);
$("rs").textContent=s.reason||"-";$("ds").textContent=s.dist;$("vv").textContent=s.v;$("fl").textContent=s.follow;
$("nv").textContent=s.nav+"(地点 "+s.wp[0]+"/"+s.wp[1]+")";$("nr").textContent=s.nav_reason;
es=s.estop;$("es").classList.toggle("on",es);$("es").textContent=es?"非常停止中(押して解除)":"非常停止";
document.querySelectorAll("[data-m]").forEach(b=>b.classList.toggle("on",b.dataset.m==s.mode));
$("msg").textContent=s.msg||"";}catch(e){$("msg").textContent="通信が切れました。台車は自動で停止します。";if(!down){down=true;say("通信が切れました。台車は止まります。",1)}}}
/* ---------- 地図(国土地理院の地図。無料・キー不要) ---------- */
let MP=null,mode2="o",orig=null,pts=[],ov=[],cartMk=null,provider="gstd",roadLL=null,srOv=[];
const MAXKM=5;
function mm(t){$("mm").textContent=t}
function hav(a,b){const R=6371000,t=Math.PI/180,p=(b[0]-a[0])*t,l=(b[1]-a[1])*t,x=Math.sin(p/2)**2+Math.cos(a[0]*t)*Math.cos(b[0]*t)*Math.sin(l/2)**2;return 2*R*Math.asin(Math.sqrt(x))}
function mapBuild(){
 if(MP)MP.remove();MP=null;ov=[];srOv=[];cartMk=null;
 if(!window.L){mm("地図の部品を読み込めません。インターネットに接続してください。");return}
 const u=provider==="gpho"?"https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg":"https://cyberjapandata.gsi.go.jp/xyz/std/{z}/{x}/{y}.png";
 MP=L.map("map").setView(orig||[36,138],orig?19:5);
 L.tileLayer(u,{maxZoom:18,attribution:"国土地理院"}).addTo(MP);
 MP.on("click",e=>onMap(e.latlng.lat,e.latlng.lng));
 redraw();
}
function mkPin(ll,label,start){return L.marker(ll,{icon:L.divIcon({html:'<div class="pin'+(start?' s':'')+'">'+label+'</div>',className:"",iconSize:[24,24],iconAnchor:[12,12]})}).addTo(MP)}
function redraw(){
 if(!MP)return;ov.forEach(o=>o.remove());ov=[];if(!orig)return;
 ov.push(mkPin(orig,"S",true));
 pts.forEach((p,i)=>ov.push(mkPin(p,String(i+1),false)));
 if(roadLL)ov.push(L.polyline(roadLL,{color:"#cf222e",weight:4}).addTo(MP));
}
function updateCart(ll){
 if(!MP||!orig)return;
 if(!cartMk)cartMk=L.circleMarker(ll,{radius:7,color:"#fff",weight:2,fillColor:"#cf222e",fillOpacity:1}).addTo(MP);
 else cartMk.setLatLng(ll);
}
function onMap(lat,lon){
 if(mode2==="o"){
  orig=[lat,lon];pts=[];roadLL=null;mode2="w";$("m_o").classList.remove("on");$("m_w").classList.add("on");
  MP.setView(orig,19);
  mm("スタート地点を決めました。行き先を、地図をタップして追加してください(最大8か所。日本のどこでも選べます)。");
 }else{
  if(!orig)return mm("先に「① スタート地点」を決めてください。");
  if(pts.length>=8)return mm("行き先は最大8か所です。");
  if(hav(orig,[lat,lon])>MAXKM*1000)return mm("スタート地点から"+MAXKM+"km以内を選んでください。");
  pts.push([lat,lon]);roadLL=null;mm("行き先 "+pts.length+" か所。決まったら「台車に送る」を押してください。");
 }
 redraw();
}
$("m_o").onclick=()=>{mode2="o";$("m_o").classList.add("on");$("m_w").classList.remove("on");mm("台車を置く場所を、地図でタップしてください。")};
$("m_w").onclick=()=>{mode2="w";$("m_w").classList.add("on");$("m_o").classList.remove("on");mm(orig?"地図をタップして、行き先を追加してください。":"先にスタート地点を決めてください。")};
$("m_c").onclick=()=>{pts=[];roadLL=null;orig=null;mode2="o";$("m_o").classList.add("on");$("m_w").classList.remove("on");redraw();mm("消しました。台車を置く場所を、地図でタップしてください。")};
async function getSignals(line){
 const samp=[];line.forEach((p,i)=>{if(!samp.length||i===line.length-1||hav(samp[samp.length-1],p)>=25)samp.push(p)});
 const co=samp.slice(0,400).map(p=>p[0].toFixed(5)+","+p[1].toFixed(5)).join(",");
 const q='[out:json][timeout:25];(node["highway"="traffic_signals"](around:20,'+co+');node["crossing"="traffic_signals"](around:20,'+co+');node["highway"="crossing"](around:20,'+co+'););out body;';
 const r=await fetch("https://overpass-api.de/api/interpreter",{method:"POST",body:"data="+encodeURIComponent(q),headers:{"Content-Type":"application/x-www-form-urlencoded"}});
 if(!r.ok)throw new Error("http");
 const j=await r.json();
 return(j.elements||[]).filter(e=>Number.isFinite(e.lat)&&Number.isFinite(e.lon)).map(e=>({lat:e.lat,lon:e.lon,type:(e.tags&&(e.tags.highway==="traffic_signals"||e.tags.crossing==="traffic_signals"))?"signal":"crossing"}));
}
$("m_s").onclick=async()=>{
 if(!orig||!pts.length)return mm("スタート地点と、行き先を1か所以上選んでください。");
 const line=roadLL?[orig,...roadLL]:[orig,...pts];
 mm("信号と横断歩道を、地図データから調べています…");
 let sigs=[],got=true;
 try{sigs=await getSignals(line)}catch(e){got=false}
 if(!got&&!$("nosig").checked)return mm("信号・横断歩道の情報を取得できませんでした。信号の位置が分からないまま走るのは危険です。もう一度試すか、この経路に信号・横断歩道がないことを確認して、上のチェックを入れてください。");
 extra.route={origin:orig,points:roadLL||pts,shortest:$("sh").checked,road:!!roadLL,signals:sigs};
 const nS=sigs.filter(x=>x.type==="signal").length,nC=sigs.length-nS;
 mm((roadLL?"道に沿った経路("+roadLL.length+"点)":"直線の経路("+pts.length+"点)")+"を台車に送りました。"+(got?"信号"+nS+"か所、横断歩道"+nC+"か所が、地図データにありました。":"信号の情報は取得できませんでした(確認済みとして送信)。")+"上の「自動運転」の「開始」で走り出します。");
};
const JP=x=>x.ll.every(Number.isFinite)&&x.ll[0]>=20&&x.ll[0]<=46&&x.ll[1]>=122&&x.ll[1]<=154;
async function getJson(u){const r=await fetch(u);if(!r.ok)throw new Error("http");return r.json()}
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
function variants(q){const n=q.normalize("NFKC"),s=new Set();[q,n,n.replace(/[ 　]+/g,""),...n.split(/[ 　]+/)].forEach(v=>{v=v.trim();if(v)s.add(v)});return[...s].slice(0,5)}
async function search(){
 const q=$("q").value.trim();if(!q)return;
 if(!MP)return mm("地図が読み込まれていません。");
 const qn=q.normalize("NFKC"),m=qn.match(/^(-?[0-9]+(?:[.][0-9]+)?)[ ,，]+(-?[0-9]+(?:[.][0-9]+)?)$/);
 let res=[];
 if(m)res=[{name:"緯度経度 "+m[1]+", "+m[2],ll:[+m[1],+m[2]],src:"入力"}];
 else{
  mm("検索中…");
  const g=await Promise.all(variants(q).map(v=>getJson("https://msearch.gsi.go.jp/address-search/AddressSearch?q="+encodeURIComponent(v)).then(j=>j.slice(0,40).map(f=>({name:String(f.properties.title),ll:[+f.geometry.coordinates[1],+f.geometry.coordinates[0]],src:"地理院"}))).catch(()=>[])));
  res=g.flat();
  const nq=[qn,qn.replace(/[ 　]+/g,"")].filter((v,i,a)=>a.indexOf(v)===i);   // OpenStreetMapは1秒に1回まで
  for(let i=0;i<nq.length;i++){
   if(i>0&&res.length>=8)break;
   try{const j=await getJson("https://nominatim.openstreetmap.org/search?format=jsonv2&limit=10&countrycodes=jp&accept-language=ja&q="+encodeURIComponent(nq[i]));
    res=res.concat(j.map(x=>({name:String(x.display_name),ll:[+x.lat,+x.lon],src:"OSM"})))}catch(e){}
   await sleep(1100);
  }
 }
 const seen=new Set();
 res=res.filter(JP).filter(x=>{const k=x.name+x.ll[0].toFixed(3)+x.ll[1].toFixed(3);if(seen.has(k))return false;seen.add(k);return true});
 const c=MP.getCenter(),sc=x=>x.name===qn?0:x.name.startsWith(qn)?1:x.name.includes(qn)?2:3;
 res.sort((a,b)=>sc(a)-sc(b)||hav([c.lat,c.lng],a.ll)-hav([c.lat,c.lng],b.ll));
 res=res.slice(0,40);
 const box=$("qr");box.innerHTML="";srOv.forEach(o=>o.remove());srOv=[];
 if(!res.length)return mm("見つかりませんでした。別の言葉や、緯度,経度(例: 35.68, 139.76)を試してください。");
 res.forEach(x=>{const b=document.createElement("button");const nm=x.name.length>44?x.name.slice(0,44)+"…":x.name;b.textContent="["+x.src+"] "+nm;b.style.textAlign="left";
  b.onclick=()=>{MP.setView(x.ll,17);mm("「"+nm+"」を表示しました。台車を置く場所を、地図でタップしてください。")};box.appendChild(b);
  srOv.push(L.circleMarker(x.ll,{radius:5,color:"#555",weight:1,fillOpacity:0.6}).addTo(MP))});
 if(res.length>1)MP.fitBounds(res.map(x=>x.ll),{maxZoom:17,padding:[20,20]});else MP.setView(res[0].ll,17);
 mm(res.length+"件見つかりました(似た名前を含む)。一覧から選ぶと、その場所に拡大します。");
}
function resample(line,step){
 const R=6371000,t=Math.PI/180,la0=line[0][0],lo0=line[0][1],pr=p=>[(p[1]-lo0)*t*R*Math.cos(la0*t),(p[0]-la0)*t*R];
 const xy=line.map(pr),out=[line[0]];let acc=0,next=step;
 for(let i=1;i<line.length;i++){
  const a=xy[i-1],b=xy[i],seg=Math.hypot(b[0]-a[0],b[1]-a[1]);
  while(seg>0&&acc+seg>=next){const f=(next-acc)/seg;out.push([line[i-1][0]+(line[i][0]-line[i-1][0])*f,line[i-1][1]+(line[i][1]-line[i-1][1])*f]);next+=step}
  acc+=seg;
 }
 const last=line[line.length-1];if(hav(out[out.length-1],last)>1.5)out.push(last);
 return out;
}
async function makeRoad(){
 if(!orig||!pts.length)return mm("先にスタート地点と、行き先を選んでください。");
 mm("道に沿った経路を作っています…");
 const co=[orig,...pts].map(p=>p[1].toFixed(6)+","+p[0].toFixed(6)).join(";");
 let j;
 try{j=await getJson("https://routing.openstreetmap.de/routed-foot/route/v1/driving/"+co+"?overview=full&geometries=geojson")}
 catch(e){return mm("道の検索に失敗しました。インターネットの接続を確認してください。直線の経路は、そのまま「台車に送る」で送れます。")}
 if(j.code!=="Ok"||!j.routes||!j.routes.length)return mm("選んだ場所の間に、道が見つかりませんでした。");
 const far=Math.max(...j.waypoints.map(w=>w.distance||0));
 if(far>30)return mm("選んだ場所の近くに道がありません(最寄りの道まで約"+Math.round(far)+"m)。道の近くを選んでください。");
 let rs=resample(j.routes[0].geometry.coordinates.map(c=>[c[1],c[0]]),5);
 if(rs.length>1&&hav(orig,rs[0])<3)rs=rs.slice(1);
 if(j.routes[0].distance>MAXKM*1000)return mm("経路が長すぎます(最大"+MAXKM+"km)。行き先を近くにしてください。");
 if(rs.length>2000)return mm("経路の点が多すぎます。行き先を近くにしてください。");
 roadLL=rs;redraw();
 mm("道に沿った経路ができました(赤い線、"+rs.length+"点、約"+Math.round(j.routes[0].distance)+"m)。「台車に送る」を押してください。");
}
$("m_r").onclick=makeRoad;
$("qb").onclick=search;$("q").onkeydown=e=>{if(e.key==="Enter")search()};
$("mp").onchange=e=>{provider=e.target.value;mapBuild()};
mapBuild();
setInterval(tick,100);</script></body></html>"""

if __name__ == "__main__":
    host = "0.0.0.0"
    port = int(os.environ.get("PORT", 8000))

    threading.Thread(target=control_loop, daemon=True).start()

    print(f"サーバー起動: {host}:{port}", flush=True)

    ThreadingHTTPServer((host, port), Handler).serve_forever()
