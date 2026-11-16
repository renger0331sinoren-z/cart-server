"""
AI電動スマート台車  段階1「止まれる台車」
- 判断部分(SafetyGovernor)とモーター制御部分(MotorController)を分離
- 実機がなくてもシミュレーションで動作確認できる
速度は -1.0〜+1.0(前進が正)、旋回は -1.0(左)〜+1.0(右)
"""
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

# ---------- 設定(ここの数値を変えて調整する) ----------
STOP_CM = 40          # これより近い物体があれば停止
SLOW_CM = 150         # これより近いと減速を始める
MAX_SPEED = 0.5       # 最高速度(最初は低く!)
REVERSE_MAX = 0.15    # 後ろにはセンサーがないので後退は超低速
SENSOR_TIMEOUT = 0.3  # センサー情報がこの秒数より古いと停止
COMMAND_TIMEOUT = 0.3 # 指令がこの秒数より古いと停止

class Mode(Enum):
    MANUAL = "手動"
    ASSIST = "自動アシスト"

@dataclass
class Sensors:
    """距離はcm。測定失敗はNone(=危険扱いにする)"""
    front_left: Optional[float]
    front_center: Optional[float]
    front_right: Optional[float]
    left: Optional[float]
    right: Optional[float]
    bumper_pressed: bool
    estop_pressed: bool
    timestamp: float

@dataclass
class Command:
    speed: float
    turn: float
    timestamp: float

# ---------- 判断部分(AIの脳:安全ルールが最優先) ----------
class SafetyGovernor:
    def __init__(self):
        self.mode = Mode.MANUAL
        self.reason = ""

    def set_mode(self, mode: Mode):
        self.mode = mode

    def decide(self, request: Command, s: Sensors, now: float) -> Command:
        """運転者の指令を、安全ルールで制限して返す"""
        stop = Command(0.0, 0.0, now)

        # 1. 絶対停止の条件(どのモードでも有効)
        if s.estop_pressed:
            self.reason = "非常停止"; return stop
        if s.bumper_pressed:
            self.reason = "バンパー接触"; return stop
        if now - s.timestamp > SENSOR_TIMEOUT:
            self.reason = "センサー情報が古い"; return stop

        speed = max(-REVERSE_MAX, min(MAX_SPEED, request.speed))
        turn = max(-1.0, min(1.0, request.turn))

        # 2. 前進時のみ前方を確認
        if speed > 0:
            front = [s.front_left, s.front_center, s.front_right]
            if any(d is None for d in front):   # 測れない=安全とは言えない
                self.reason = "前方センサー異常"; return stop
            nearest = min(front)
            if nearest <= STOP_CM:
                self.reason = f"障害物 {nearest:.0f}cm 停止"; return stop
            if nearest < SLOW_CM:
                ratio = (nearest - STOP_CM) / (SLOW_CM - STOP_CM)
                speed *= ratio                   # 近いほど遅く
                self.reason = f"障害物 {nearest:.0f}cm 減速"
            else:
                self.reason = "通常走行"

            # 3. 自動アシスト時のみ:左右の壁から離れる方向へ操舵補正
            if self.mode == Mode.ASSIST and s.left is not None and s.right is not None:
                if s.left < 80 or s.right < 80:
                    turn += max(-0.3, min(0.3, (s.left - s.right) / 200 * -1))
                    turn = max(-1.0, min(1.0, turn))
        else:
            self.reason = "停止または後退"
        return Command(speed, turn, now)

# ---------- モーター制御部分(判断部分とは指令だけでつながる) ----------
class MotorController:
    def __init__(self, driver):
        self.driver = driver
        self.last: Optional[Command] = None

    def update(self, cmd: Optional[Command], now: float):
        """指令が古い/無いときは必ず停止(見張り機能)"""
        if cmd is None or now - cmd.timestamp > COMMAND_TIMEOUT:
            self.driver.set_wheels(0.0, 0.0)
            return
        left = max(-1.0, min(1.0, cmd.speed + cmd.turn * 0.5))
        right = max(-1.0, min(1.0, cmd.speed - cmd.turn * 0.5))
        self.driver.set_wheels(left, right)

class SimDriver:
    """実機の代わり。後で本物のモータードライバ用に差し替える"""
    def __init__(self): self.left = self.right = 0.0
    def set_wheels(self, l, r): self.left, self.right = l, r

# ---------- シミュレーション(前方1.5m先に壁) ----------
def demo():
    gov, driver = SafetyGovernor(), SimDriver()
    motor = MotorController(driver)
    pos, wall = 0.0, 300.0   # cm
    print("手動で全速前進を指令し続ける → 壁の手前で止まるか確認")
    for step in range(60):
        now = time.time()
        d = wall - pos
        s = Sensors(d, d, d, 200, 200, False, False, now)
        cmd = gov.decide(Command(1.0, 0.0, now), s, now)
        motor.update(cmd, now)
        pos += (driver.left + driver.right) / 2 * 12   # 簡易移動モデル
        if step % 4 == 0 or cmd.speed == 0:
            print(f"{step:2d} 壁まで{d:5.0f}cm  速度{cmd.speed:4.2f}  {gov.reason}")
        if cmd.speed == 0 and d < 100: break
    print(f"最終: 壁まで{wall-pos:.0f}cm で停止 (停止距離{STOP_CM}cm)")

    print("\n--- 安全テスト ---")
    now = time.time()
    ok = Sensors(200,200,200,200,200,False,False,now)
    tests = {
        "センサー断線(None)": Sensors(None,200,200,200,200,False,False,now),
        "バンパー接触": Sensors(200,200,200,200,200,True,False,now),
        "非常停止": Sensors(200,200,200,200,200,False,True,now),
        "データが古い": Sensors(200,200,200,200,200,False,False,now-1),
    }
    for name, s in tests.items():
        c = gov.decide(Command(0.5,0,now), s, now)
        print(f"{name}: 速度={c.speed} → {'OK(停止)' if c.speed==0 else 'NG!'}")
    motor.update(Command(0.5,0,now-1), now)
    print(f"指令が古い: 車輪={driver.left},{driver.right} → {'OK(停止)' if driver.left==0 else 'NG!'}")

if __name__ == "__main__":
    demo()
