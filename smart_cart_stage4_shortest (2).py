"""
段階4: どんなときでも最短ルート
  ① 最短の道を探す(A*)  ② 障害物を避ける最短の迂回路  ③ 複数の行き先の、最短の順番
座標は、スタート地点からの東(x)・北(y)メートル。単独でも動く: python smart_cart_stage4_shortest.py
安全のための考え方:
  障害物は「半径0.7m」の硬い芯(絶対に入らない)と、「半径1.6m」の柔らかい余白(なるべく避ける)で囲む。
  余白を広くとるのは、台車が目標点の手前で曲がり始める(角を切る)ぶんを見込むため。
  そのため「最短」は、障害物を余白ごと避けた中での最短になる。
"""
import heapq, itertools, math, random, time

CELL = 0.5            # 探す地図の1マスの大きさ(m)

class Planner:
    def __init__(self, radius=40.0, r_hard=0.7, r_soft=1.6, escape=2.0):
        self.R, self.r_hard, self.r_soft, self.escape = radius, r_hard, r_soft, escape
        self.obs = []; self.last_cost = None

    def clear(self): self.obs = []
    def add_obstacle(self, x, y, merge=1.0):
        """障害物を覚える。すでに覚えている障害物に近い(merge m以内)ものは、同じ障害物として1つにまとめる"""
        if any(math.hypot(x - ox, y - oy) < merge for ox, oy in self.obs): return False
        self.obs.append((x, y)); return True

    def _cell(self, p): return (int(round((p[0] + self.R) / CELL)), int(round((p[1] + self.R) / CELL)))
    def _center(self, c): return (c[0] * CELL - self.R, c[1] * CELL - self.R)

    def _blocked_fn(self, start):
        """点(x,y)が通れないか。スタート付近だけは、柔らかい余白に入っていても出られるようにする"""
        obs, hard, soft, esc, R = self.obs, self.r_hard, self.r_soft, self.escape, self.R
        def blocked(x, y):
            if math.hypot(x, y) > R: return True                      # 許可範囲の外
            near = math.hypot(x - start[0], y - start[1]) <= esc
            for ox, oy in obs:
                d = math.hypot(x - ox, y - oy)
                if d < hard or (d < soft and not near): return True
            return False
        return blocked

    @staticmethod
    def length(path): return sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))

    def path(self, start, goal, _h=1.0):
        """startからgoalへの最短の道(点の列)。通れなければNone"""
        blocked = self._blocked_fn(start)
        if blocked(*goal): return None
        if not self.obs: self.last_cost = math.dist(start, goal); return [start, goal]   # 障害物がなければ、まっすぐ(長い距離でも速い)
        if any(math.hypot(start[0] - ox, start[1] - oy) < self.r_hard - 0.15 for ox, oy in self.obs): return None   # 位置の誤差ぶんの余裕
        def los(a, b):
            n = max(1, int(math.dist(a, b) / (CELL / 2)))
            return not any(blocked(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1))
        if los(start, goal): self.last_cost = math.dist(start, goal); return [start, goal]
        cache = {}
        def bl(c):
            v = cache.get(c)
            if v is None: v = cache[c] = blocked(*self._center(c))
            return v
        s, g = self._cell(start), self._cell(goal)
        def h(c): dx, dy = abs(c[0] - g[0]), abs(c[1] - g[1]); return _h * ((dx + dy) + (math.sqrt(2) - 2) * min(dx, dy))
        openq, best, prev = [(h(s), 0.0, s)], {s: 0.0}, {}
        while openq:
            f, gc, c = heapq.heappop(openq)
            if c == g: break
            if gc > best.get(c, 1e18): continue
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                n = (c[0] + dx, c[1] + dy)
                if n != g and bl(n): continue
                if dx and dy and ((bl((c[0] + dx, c[1])) and (c[0] + dx, c[1]) not in (s, g)) or (bl((c[0], c[1] + dy)) and (c[0], c[1] + dy) not in (s, g))): continue
                ng = gc + (math.sqrt(2) if dx and dy else 1.0)
                if ng < best.get(n, 1e18):
                    best[n] = ng; prev[n] = c; heapq.heappush(openq, (ng + h(n), ng, n))
        if g not in best: return None
        self.last_cost = best[g] * CELL
        cells = [g]
        while cells[-1] != s: cells.append(prev[cells[-1]])
        pts = [start] + [self._center(c) for c in reversed(cells)][1:-1] + [goal]
        out, i = [pts[0]], 0                                      # 見通せる限り、まっすぐに(角を減らす)
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not los(pts[i], pts[j]): j -= 1
            out.append(pts[j]); i = j
        return out

    @staticmethod
    def densify(path, step=1.5):
        """長い線分を、step以下の間隔の点に分ける(台車が角を切りすぎないように)"""
        out = [path[0]]
        for a, b in zip(path, path[1:]):
            n = max(1, math.ceil(math.dist(a, b) / step))
            out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
        return out

    def order(self, start, pts):
        """複数の行き先を、合計の道のりが最短になる順番に並べる(最後はどこでもよい)。8か所までは完全に最適"""
        n = len(pts)
        if n <= 1: return list(range(n))
        nodes = [start] + list(pts)
        D = [[0.0] * (n + 1) for _ in range(n + 1)]
        for i in range(n + 1):
            for j in range(i + 1, n + 1):
                p = self.path(nodes[i], nodes[j]); D[i][j] = D[j][i] = self.length(p) if p else 1e9
        if n > 8:                                                 # 多いときは、近い順につなぐ(最適とは限らない)
            left, cur, seq = set(range(1, n + 1)), 0, []
            while left: nxt = min(left, key=lambda k: D[cur][k]); seq.append(nxt); left.discard(nxt); cur = nxt
            return [k - 1 for k in seq]
        best, bl = None, 1e18
        for perm in itertools.permutations(range(1, n + 1)):
            L = D[0][perm[0]] + sum(D[perm[k]][perm[k + 1]] for k in range(n - 1))
            if L < bl: bl, best = L, perm
        return [k - 1 for k in best]

    def order_length(self, start, pts, idx):
        cur, tot = start, 0.0
        for k in idx:
            p = self.path(cur, pts[k]); tot += self.length(p) if p else 1e9; cur = pts[k]
        return tot

# ---------- 単独の試験 ----------
def demo():
    random.seed(3)
    print("=== 1: 障害物がなければ、まっすぐ(これが最短) ===")
    P = Planner(); p = P.path((0, 0), (20, 10)); print(f"道のり {P.length(p):.2f}m / 直線 {math.dist((0,0),(20,10)):.2f}m / 点の数 {len(p)}")

    print("\n=== 2: 道の真ん中に障害物 → 避ける最短の道(計算で求めた正解と比べる) ===")
    for L in (6.0, 10.0):
        P = Planner(); P.add_obstacle(L, 0.0); p = P.path((0, 0), (2 * L, 0)); R = P.r_soft
        t = math.sqrt(L * L - R * R); ideal = 2 * t + R * (math.pi - 2 * math.acos(R / L))
        print(f"障害物まで{L:.0f}m: 探した道 {P.length(p):.2f}m / 理論上の最短 {ideal:.2f}m → 差 {100*(P.length(p)/ideal-1):+.1f}%  (直線なら {2*L:.0f}m)")

    print("\n=== 3: A*が本当に最短か(探し方を変えて同じ答えになるか) ===")
    P = Planner()
    for ox, oy in [(5, 0), (5, 2.5), (5, -2.5), (5, 5), (5, -5), (9, 4), (9, 1.5), (9, -1)]: P.add_obstacle(ox, oy)
    P.path((0, 0), (14, 0)); a = P.last_cost; P.path((0, 0), (14, 0), _h=0.0); d = P.last_cost
    print(f"A*(速い探し方)のマス目の道のり {a:.2f}m / 全部調べる方法 {d:.2f}m → {'一致' if abs(a-d)<1e-6 else '不一致!'}")
    p = P.path((0, 0), (14, 0)); print(f"壁のように並んだ障害物を迂回: 道のり {P.length(p):.1f}m(直線14m)")

    print("\n=== 4: 通れない場合は、通れないと答える ===")
    P = Planner(); P.add_obstacle(5, 0)
    print("目的地が障害物の中:", P.path((0, 0), (5.3, 0)))
    P2 = Planner(radius=6.0)
    for y in range(-6, 7): P2.add_obstacle(3, y)
    print("通路が全部ふさがれている:", P2.path((0, 0), (5, 0)))
    print("許可範囲(40m)の外へ:", Planner().path((0, 0), (60, 0)))

    print("\n=== 5: 複数の行き先の順番(8か所のランダムな配置を20回) ===")
    sv_tap, sv_nn = [], []
    for _ in range(20):
        P = Planner(); pts = [(random.uniform(-30, 30), random.uniform(-30, 30)) for _ in range(8)]
        t0 = time.time(); best = P.order((0, 0), pts); dt = time.time() - t0
        tap = P.order_length((0, 0), pts, list(range(8))); opt = P.order_length((0, 0), pts, best)
        cur, left, nn = (0, 0), set(range(8)), []
        while left: k = min(left, key=lambda i: math.dist(cur, pts[i])); nn.append(k); left.discard(k); cur = pts[k]
        nnl = P.order_length((0, 0), pts, nn)
        sv_tap.append(100 * (1 - opt / tap)); sv_nn.append(100 * (1 - opt / nnl))
    print(f"タップした順に比べて、平均 {sum(sv_tap)/20:.0f}% 短い(最大 {max(sv_tap):.0f}%)")
    print(f"『一番近い所から順に』より、平均 {sum(sv_nn)/20:.1f}% 短い(最大 {max(sv_nn):.0f}%) / 最適な順番を探す時間 {dt:.2f}秒")

    print("\n=== 6: 計算の速さ(障害物6個、40m先) ===")
    P = Planner()
    for ox, oy in [(5, 0), (12, 1), (18, -1), (24, 0), (30, 2), (35, -2)]: P.add_obstacle(ox, oy)
    t0 = time.time(); p = P.path((0, 0), (38, 0)); print(f"{time.time()-t0:.2f}秒 / 道のり {P.length(p):.1f}m")

if __name__ == "__main__":
    demo()
