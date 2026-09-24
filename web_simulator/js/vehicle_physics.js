// Simple differential-drive physics for the AIFormula sample_vehicle.
// Values are chosen to match vehicles/sample_vehicle/xacro/*.xacro and
// config/wheel.yaml (wheel radius / track width), with the overall vehicle
// mass and wheel-load values set to the measured vehicle specifications.

export const VEHICLE = {
  // Static wheel loads [kg].  Their sum is the vehicle mass used by the
  // longitudinal dynamics below.
  wheelLoadsKg: {
    right: 25.0,
    left: 23.4,
    rear: 23.2,
  },
  massKg: 71.6,
  wheelRadius: 0.12, // [m] vehicles/sample_vehicle xacro: WHEEL_RADIUS
  track: 0.6, // [m] config/wheel.yaml: tread
  wheelbase: 0.815, // [m] front drive-wheel axle to rear-wheel axle
};

// Forces are expressed in Newtons and converted to acceleration through
// VEHICLE.massKg, so the 71.6kg vehicle mass genuinely affects how quickly the
// vehicle speeds up, brakes and coasts to a stop.
const DRIVE_FORCE_N = 154; // W held -> forward accel ~2.2 m/s^2
const REVERSE_FORCE_N = 98; // S held while stopped/reversing -> ~1.4 m/s^2
const BRAKE_FORCE_N = 231; // S held while still moving forward -> ~3.3 m/s^2
const COAST_RESISTANCE_N = 35; // rolling resistance + drag while coasting -> ~0.5 m/s^2

// 実機 motor_controller の加減速制限 (control/motor_controller/config/
// motor_controller.yaml の max_linear_accel / max_linear_decel /
// max_angular_accel と同じ値にすること)。実機は受けた速度指令に対し、|v| が
// 大きくなる向きは ACCEL、小さくなる向き (減速・停止・前後反転の手前まで) は
// DECEL を超えない範囲でしか追従しないので、シミュレータの車もこれ以上速く
// 速度を変えない (例: S ブレーキ 3.2 m/s^2 も実機同様 1.5 m/s^2 に制限される)。
export const MOTOR_MAX_LINEAR_ACCEL = 2.2; // [m/s^2]
export const MOTOR_MAX_LINEAR_DECEL = 1.5; // [m/s^2]
export const MOTOR_MAX_ANGULAR_ACCEL = 4.0; // [rad/s^2]

function motorLimitLinear(previous, next, dt) {
  const toward = previous * next < 0 ? 0 : next; // 前後反転は一度 0 まで減速
  const decreasing = Math.abs(toward) < Math.abs(previous) || previous * next < 0;
  const maxStep = (decreasing ? MOTOR_MAX_LINEAR_DECEL : MOTOR_MAX_LINEAR_ACCEL) * dt;
  return previous + clamp(toward - previous, -maxStep, maxStep);
}

function motorLimitAngular(previous, next, dt) {
  const maxStep = MOTOR_MAX_ANGULAR_ACCEL * dt;
  return previous + clamp(next - previous, -maxStep, maxStep);
}

// Exported so js/simulator.js can pass them to the autonomous-driving
// lane navigator (lane_navigator.js) as its max speed / angular clamp --
// per instruction, autonomous driving keeps these same limits rather than
// oit_navigation's own real-vehicle defaults.
export const MAX_SPEED = 1.5; // [m/s]
const MAX_REVERSE_SPEED = -0.75; // [m/s]

const ANGULAR_ACCEL = 2.5; // [rad/s^2] while A/D held
const ANGULAR_DAMPING = 4.0; // [rad/s^2] while A/D released (coasts back to 0)
export const MAX_ANGULAR = 1.2; // [rad/s]

// 車輪スリップ誤差モデル (CLAUDE.md: 「機体はスリップ誤差が8%程度ある」)。
// 左右輪独立に、時定数付きランダムウォークで±8%以内のスリップ率を持たせる。
// 真の物理位置(this.x/y/yaw)には影響させず、measuredWheelSpeeds()だけに
// 反映することで、CAN配信・オドメトリ推定(js/simulator.jsのlocalizer)と
// 真の位置が実車と同じように乖離していくようにする。
const SLIP_TAU_S = 2.0; // [s] 時定数
const SLIP_NOISE = 0.05; // [1/sqrt(s)] ノイズ強度
const SLIP_MAX = 0.08; // ±8%にクランプ

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function approachZero(value, decel, dt) {
  if (value > 0) return Math.max(0, value - decel * dt);
  if (value < 0) return Math.min(0, value + decel * dt);
  return 0;
}

const INITIAL_X = 0;
const INITIAL_Y = 0;
const INITIAL_YAW = 0;

export class VehiclePhysics {
  constructor() {
    this.x = INITIAL_X; // [m] ROS convention, world/odom frame
    this.y = INITIAL_Y; // [m]
    this.yaw = INITIAL_YAW; // [rad], 0 = facing +X, positive = CCW (REP103)
    this.v = 0; // [m/s] forward speed
    this.omega = 0; // [rad/s] yaw rate
    this.linearAccel = 0; // [m/s^2] forward (body x) accel, for IMU simulation
    this.slipL = 0; // 左輪スリップ率 (-0.08〜0.08、measuredWheelSpeeds()だけに影響)
    this.slipR = 0; // 右輪スリップ率
  }

  reset() {
    this.x = INITIAL_X;
    this.y = INITIAL_Y;
    this.yaw = INITIAL_YAW;
    this.v = 0;
    this.omega = 0;
    this.linearAccel = 0;
    this.slipL = 0;
    this.slipR = 0;
  }

  // keys: { forward, backward, left, right } booleans
  step(keys, dt) {
    const mass = VEHICLE.massKg;
    const previousV = this.v;

    // --- Longitudinal (linear.x) ---
    if (keys.forward && !keys.backward) {
      this.v += (DRIVE_FORCE_N / mass) * dt;
    } else if (keys.backward && !keys.forward) {
      if (this.v > 0) {
        this.v -= (BRAKE_FORCE_N / mass) * dt;
      } else {
        this.v -= (REVERSE_FORCE_N / mass) * dt;
      }
    } else {
      this.v = approachZero(this.v, COAST_RESISTANCE_N / mass, dt);
    }
    this.v = motorLimitLinear(previousV, clamp(this.v, MAX_REVERSE_SPEED, MAX_SPEED), dt);
    this.linearAccel = dt > 0 ? (this.v - previousV) / dt : 0;

    // --- Yaw rate (angular.z) ---
    const previousOmega = this.omega;
    if (keys.left && !keys.right) {
      this.omega += ANGULAR_ACCEL * dt;
    } else if (keys.right && !keys.left) {
      this.omega -= ANGULAR_ACCEL * dt;
    } else {
      this.omega = approachZero(this.omega, ANGULAR_DAMPING, dt);
    }
    this.omega = motorLimitAngular(previousOmega, clamp(this.omega, -MAX_ANGULAR, MAX_ANGULAR), dt);

    // --- Integrate pose (ROS convention: x forward, y left, yaw about z) ---
    this.x += this.v * Math.cos(this.yaw) * dt;
    this.y += this.v * Math.sin(this.yaw) * dt;
    this.yaw += this.omega * dt;
    // Normalize yaw to [-PI, PI] (REP 103 standard)
    this.yaw = Math.atan2(Math.sin(this.yaw), Math.cos(this.yaw));

    this._stepSlip(dt);
  }

  // Drives the vehicle from a commanded (v, omega) instead of WASD key
  // state -- used by the autonomous-driving toggle (js/simulator.js), fed
  // from the ported oit_navigation lane navigator (js/lane_navigator.js's
  // LaneNavigator.step). Ramps toward the
  // command using the same force/accel budget as manual driving (so
  // autonomous driving has the same inertia "feel"), then clamps to the
  // same MAX_SPEED/MAX_REVERSE_SPEED/MAX_ANGULAR limits WASD is bound by --
  // per instruction, autonomous mode keeps these limits unchanged (1.5 m/s
  // forward, 1.2 rad/s turning) rather than adopting oit_navigation's own
  // real-vehicle defaults (target_linear_speed=1.0, max_angular_speed=1.5).
  stepAutonomous(vCmd, omegaCmd, dt) {
    const mass = VEHICLE.massKg;
    const previousV = this.v;

    vCmd = clamp(vCmd, MAX_REVERSE_SPEED, MAX_SPEED);
    omegaCmd = clamp(omegaCmd, -MAX_ANGULAR, MAX_ANGULAR);

    if (this.v < vCmd) {
      const accel = (vCmd > 0 ? DRIVE_FORCE_N : REVERSE_FORCE_N) / mass;
      this.v = Math.min(vCmd, this.v + accel * dt);
    } else if (this.v > vCmd) {
      const decel = (this.v > 0 ? BRAKE_FORCE_N : REVERSE_FORCE_N) / mass;
      this.v = Math.max(vCmd, this.v - decel * dt);
    }
    this.v = motorLimitLinear(previousV, clamp(this.v, MAX_REVERSE_SPEED, MAX_SPEED), dt);
    this.linearAccel = dt > 0 ? (this.v - previousV) / dt : 0;

    if (this.omega < omegaCmd) {
      this.omega = Math.min(omegaCmd, this.omega + ANGULAR_ACCEL * dt);
    } else if (this.omega > omegaCmd) {
      this.omega = Math.max(omegaCmd, this.omega - ANGULAR_ACCEL * dt);
    }
    this.omega = clamp(this.omega, -MAX_ANGULAR, MAX_ANGULAR);

    this.x += this.v * Math.cos(this.yaw) * dt;
    this.y += this.v * Math.sin(this.yaw) * dt;
    this.yaw += this.omega * dt;
    this.yaw = Math.atan2(Math.sin(this.yaw), Math.cos(this.yaw));

    this._stepSlip(dt);
  }

  // Box-Mullerで標準正規乱数を1つ作る (スリップのランダムウォークのノイズ項)。
  static _randn() {
    const u1 = Math.max(Math.random(), 1e-9);
    const u2 = Math.random();
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  }

  // 左右輪のスリップ率を1ステップ進める。step()/stepAutonomous()の末尾から呼ぶ。
  _stepSlip(dt) {
    this.slipL += (-this.slipL / SLIP_TAU_S + SLIP_NOISE * VehiclePhysics._randn()) * dt;
    this.slipR += (-this.slipR / SLIP_TAU_S + SLIP_NOISE * VehiclePhysics._randn()) * dt;
    this.slipL = clamp(this.slipL, -SLIP_MAX, SLIP_MAX);
    this.slipR = clamp(this.slipR, -SLIP_MAX, SLIP_MAX);
  }

  // Standard differential-drive wheel speed decomposition (actual/measured).
  wheelSpeeds() {
    const halfTrack = VEHICLE.track / 2;
    return {
      left: this.v - this.omega * halfTrack,
      right: this.v + this.omega * halfTrack,
    };
  }

  // CAN RPM配信・オドメトリ推定(js/simulator.jsのlocalizer)が使う、
  // スリップ込みの「計測される」車輪速度。wheelSpeeds()(真値、当たり判定
  // や描画に使う)とは別に用意し、両者の乖離が8%程度のスリップを再現する。
  measuredWheelSpeeds() {
    const { left, right } = this.wheelSpeeds();
    return { left: left * (1 + this.slipL), right: right * (1 + this.slipR) };
  }

  // Theoretical / commanded target wheel speeds from active key inputs.
  targetWheelSpeeds(keys) {
    let vTarget = 0;
    let omegaTarget = 0;
    if (keys.forward && !keys.backward) {
      vTarget = MAX_SPEED;
    } else if (keys.backward && !keys.forward) {
      vTarget = MAX_REVERSE_SPEED;
    }
    if (keys.left && !keys.right) {
      omegaTarget = MAX_ANGULAR;
    } else if (keys.right && !keys.left) {
      omegaTarget = -MAX_ANGULAR;
    }
    const halfTrack = VEHICLE.track / 2;
    return {
      vTarget,
      omegaTarget,
      left: vTarget - omegaTarget * halfTrack,
      right: vTarget + omegaTarget * halfTrack,
    };
  }
}
