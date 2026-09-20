// Simple differential-drive physics for the AIFormula sample_vehicle.
// Values are chosen to match vehicles/sample_vehicle/xacro/*.xacro and
// config/wheel.yaml (wheel radius / track width), with the overall vehicle
// mass set to the ~70kg spec given for this simulator.

export const VEHICLE = {
  massKg: 70,
  wheelRadius: 0.12, // [m] vehicles/sample_vehicle xacro: WHEEL_RADIUS
  track: 0.6, // [m] config/wheel.yaml: tread
};

// Forces are expressed in Newtons and converted to acceleration through
// VEHICLE.massKg, so the 70kg body mass genuinely affects how quickly the
// vehicle speeds up, brakes and coasts to a stop.
const DRIVE_FORCE_N = 154; // W held -> forward accel ~2.2 m/s^2
const REVERSE_FORCE_N = 98; // S held while stopped/reversing -> ~1.4 m/s^2
const BRAKE_FORCE_N = 231; // S held while still moving forward -> ~3.3 m/s^2
const COAST_RESISTANCE_N = 35; // rolling resistance + drag while coasting -> ~0.5 m/s^2

// Exported so js/simulator.js can pass them to the autonomous-driving
// lane navigator (lane_navigator.js) as its max speed / angular clamp --
// per instruction, autonomous driving keeps these same limits rather than
// oit_navigation's own real-vehicle defaults.
export const MAX_SPEED = 1.5; // [m/s]
const MAX_REVERSE_SPEED = -0.75; // [m/s]

const ANGULAR_ACCEL = 2.5; // [rad/s^2] while A/D held
const ANGULAR_DAMPING = 4.0; // [rad/s^2] while A/D released (coasts back to 0)
export const MAX_ANGULAR = 1.2; // [rad/s]

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
  }

  reset() {
    this.x = INITIAL_X;
    this.y = INITIAL_Y;
    this.yaw = INITIAL_YAW;
    this.v = 0;
    this.omega = 0;
    this.linearAccel = 0;
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
    this.v = clamp(this.v, MAX_REVERSE_SPEED, MAX_SPEED);
    this.linearAccel = dt > 0 ? (this.v - previousV) / dt : 0;

    // --- Yaw rate (angular.z) ---
    if (keys.left && !keys.right) {
      this.omega += ANGULAR_ACCEL * dt;
    } else if (keys.right && !keys.left) {
      this.omega -= ANGULAR_ACCEL * dt;
    } else {
      this.omega = approachZero(this.omega, ANGULAR_DAMPING, dt);
    }
    this.omega = clamp(this.omega, -MAX_ANGULAR, MAX_ANGULAR);

    // --- Integrate pose (ROS convention: x forward, y left, yaw about z) ---
    this.x += this.v * Math.cos(this.yaw) * dt;
    this.y += this.v * Math.sin(this.yaw) * dt;
    this.yaw += this.omega * dt;
    // Normalize yaw to [-PI, PI] (REP 103 standard)
    this.yaw = Math.atan2(Math.sin(this.yaw), Math.cos(this.yaw));
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
    this.v = clamp(this.v, MAX_REVERSE_SPEED, MAX_SPEED);
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
  }

  // Standard differential-drive wheel speed decomposition (actual/measured).
  wheelSpeeds() {
    const halfTrack = VEHICLE.track / 2;
    return {
      left: this.v - this.omega * halfTrack,
      right: this.v + this.omega * halfTrack,
    };
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
