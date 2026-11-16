"""
AI電動スマート台車  段階2「ついてくる台車」
段階1(smart_cart_stage1.py)と同じフォルダに置いて使う。
流れ:  追従AI → 速度維持 → 【段階1の安全ルール】 → モーター制御
追従AIが何を指令しても、最後は必ず安全ルールを通る。
"""
from enum import Enum
from typing import Optional
from smart_cart_stage1 import (Command, Sensors, SafetyGovernor, MotorController,
                               SimDriver, Mode, MAX_SPEED, STOP_CM)

# ---------- 設定 ----------
FOLLOW_TARGET_CM = 160   # この距離を保って追従する
FOLLOW_LOCK_MIN = 80     # 追従開始できる距離の範囲
FOLLOW_LOCK_MAX = 250
FOLLOW_LOST_CM = 300     # これより遠いと見失ったと判断
JUMP_LIMIT_CM = 80       # 1回で距離がこれ以上飛んだら別の物体と判断
DEADBAND_CM = 10         # 距離のズレがこれ以内なら動かない(ガタつき防止)
KP_DIST = 0.004          # 距離のズレ1cmあたりの速度
KP_TURN = 0.005          # 左右センサー差1cmあたりの旋回

class FollowState(Enum):
    IDLE = "待機"
    FOLLOWING = "追従中"
    LOST = "見失い(停止)"

# ---------- 追従AI ----------
class FollowController:
    def __init__(self):
        self.state = FollowState.IDLE
        self.last_dist: Optional[float] = None

    def start(self, s: Sensors) -> bool:
        """人が前に立った状態で開始。範囲内にいなければ開始しない"""
        d = s.front_center
        if d is not None and FOLLOW_LOCK_MIN <= d <= FOLLOW_LOCK_MAX:
            self.state, self.last_dist = FollowState.FOLLOWING, d
            return True
        return False

    def stop(self):
        self.state, self.last_dist = FollowState.IDLE, None

    def compute(self, s: Sensors, now: float) -> Command:
        if self.state != FollowState.FOLLOWING:
            return Command(0.0, 0.0, now)
        d = s.front_center
        # 測れない / 遠すぎる / 距離が急に飛んだ(別の物体が割り込んだ) → 見失い
        if (d is None or d > FOLLOW_LOST_CM
                or abs(d - self.last_dist) > JUMP_LIMIT_CM):
            self.state = FollowState.LOST
            return Command(0.0, 0.0, now)
        self.last_dist = d

        err = d - FOLLOW_TARGET_CM
        speed = 0.0 if abs(err) < DEADBAND_CM else max(0.0, min(MAX_SPEED, KP_DIST * err))
        turn = 0.0
        if s.front_left is not None and s.front_right is not None:
            turn = max(-0.5, min(0.5, KP_TURN * (s.front_left - s.front_right)))
        return Command(speed, turn, now)

# ---------- 速度維持アシスト(PI制御) ----------
class SpeedHold:
    """目標速度と実際の速度(車輪の回転センサー)の差を見て、出力を補正する"""
    def __init__(self, kp=1.0, ki=1.5):
        self.kp, self.ki, self.integral = kp, ki, 0.0

    def adjust(self, desired: float, measured: float, dt: float) -> float:
        if desired <= 0:                       # 停止指令なら補正しない
            self.integral = 0.0
            return desired
        err = desired - measured
        out = desired + self.kp * err + self.ki * self.integral
        if 0.0 <= out <= 1.0:                  # 出力が限界の時は積分を止める
            self.integral += err * dt
        return max(0.0, min(1.0, out))

# ---------- シミュレーション ----------
DT = 0.1
class SimCart:
    """簡易モデル: 出力1.0で秒速100cm。slope_cm_s>0なら坂で押し戻される"""
    def __init__(self): self.pos = 0.0; self.v = 0.0
    def step(self, driver, slope=0.0):
        cmd = (driver.left + driver.right) / 2
        target = 0.0 if cmd == 0 else cmd * 100 - slope
        self.v += (target - self.v) * 0.5
        self.pos += self.v * DT

def follow_demo():
    print("=== 試験1: 人が歩いて止まる → 台車は付いてきて、止まる ===")
    gov, drv = SafetyGovernor(), SimDriver()
    motor, follow, cart = MotorController(drv), FollowController(), SimCart()
    person, t = 200.0, 0.0
    s0 = Sensors(200,200,200,200,200,False,False,t)
    print("追従開始:", "成功" if follow.start(s0) else "失敗")
    for i in range(300):
        t = i * DT
        person += (30 if 2 <= t < 20 else 0) * DT   # 2〜20秒に秒速30cmで歩く
        d = person - cart.pos
        s = Sensors(d,d,d,200,200,False,False,t)
        cmd = gov.decide(follow.compute(s, t), s, t)
        motor.update(cmd, t); cart.step(drv)
        if i % 40 == 0:
            print(f"t={t:4.1f}s 距離{d:5.0f}cm 台車速度{cart.v:4.1f}cm/s [{follow.state.value}]")
    print(f"最終距離 {d:.0f}cm (目標{FOLLOW_TARGET_CM}cm / 停止ライン{STOP_CM}cm)")

    print("\n=== 試験2: 人が台車より速く去る → 見失って停止 ===")
    gov, drv = SafetyGovernor(), SimDriver()
    motor, follow, cart = MotorController(drv), FollowController(), SimCart()
    person, follow.state, follow.last_dist = 200.0, FollowState.FOLLOWING, 200.0
    for i in range(200):
        t = i * DT
        person += 90 * DT                            # 秒速90cm(台車の最高は50)
        d = person - cart.pos
        s = Sensors(d,d,d,200,200,False,False,t)
        cmd = gov.decide(follow.compute(s, t), s, t)
        motor.update(cmd, t); cart.step(drv)
        if follow.state == FollowState.LOST:
            print(f"t={t:.1f}s 距離{d:.0f}cm → {follow.state.value}。台車速度 {cart.v:.1f}cm/s"); break

    print("\n=== 試験3: 別の人・物が間に割り込む → 見失って停止 ===")
    follow = FollowController(); follow.state, follow.last_dist = FollowState.FOLLOWING, 200.0
    s = Sensors(90,90,90,200,200,False,False,0)
    c = follow.compute(s, 0)
    print(f"距離 200→90cm に急変 → {follow.state.value} / 速度={c.speed}")

    print("\n=== 試験4: 追従開始の条件 ===")
    for dist in (50, 150, 400):
        f = FollowController()
        ok = f.start(Sensors(dist,dist,dist,200,200,False,False,0))
        print(f"前方{dist}cm → 追従開始 {'できる' if ok else 'できない'}")

    print("\n=== 試験5: 人が左にいる → 左に曲がるか ===")
    f = FollowController(); f.state, f.last_dist = FollowState.FOLLOWING, 200.0
    c = f.compute(Sensors(180,200,230,200,200,False,False,0), 0)
    print(f"左180/右230cm → 旋回 {c.turn:+.2f} (マイナス=左) 速度 {c.speed:.2f}")

def speed_hold_demo():
    print("\n=== 試験6: 速度維持アシスト(目標0.30、5〜15秒は坂道で押し戻される) ===")
    for use_hold in (False, True):
        drv, cart, hold = SimDriver(), SimCart(), SpeedHold()
        cart.v = 30.0; worst = 0.0
        for i in range(250):
            t = i * DT
            slope = 20 if 5 <= t < 15 else 0
            desired = 0.30
            out = hold.adjust(desired, cart.v / 100, DT) if use_hold else desired
            drv.set_wheels(out, out); cart.step(drv, slope)
            if 8 <= t < 15: worst = max(worst, abs(cart.v - 30))
        print(f"{'速度維持 あり' if use_hold else '速度維持 なし'}: 坂の途中の最大ズレ {worst:.1f}cm/s, 最終速度 {cart.v:.1f}cm/s")

if __name__ == "__main__":
    follow_demo()
    speed_hold_demo()
