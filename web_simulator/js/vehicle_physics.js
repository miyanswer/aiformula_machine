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

const MAX_SPEED = 1.5; // [m/s]
const MAX_REVERSE_SPEED = -0.75; // [m/s]

const ANGULAR_ACCEL = 2.5; // [rad/s^2] while A/D held
const ANGULAR_DAMPING = 4.0; // [rad/s^2] while A/D released (coasts back to 0)
const MAX_ANGULAR = 1.2; // [rad/s]

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function approachZero(value, decel, dt) {
  if (value > 0) return Math.max(0, value - decel * dt);
  if (value < 0) return Math.min(0, value + decel * dt);
  return 0;
}

export class VehiclePhysics {
  constructor() {
    this.x = 0; // [m] ROS convention, world/odom frame
    this.y = 0; // [m]
    this.yaw = 0; // [rad], 0 = facing +X, positive = CCW (REP103)
    this.v = 0; // [m/s] forward speed
    this.omega = 0; // [rad/s] yaw rate
  }

  reset() {
    this.x = 0;
    this.y = 0;
    this.yaw = 0;
    this.v = 0;
    this.omega = 0;
  }

  // keys: { forward, backward, left, right } booleans
  step(keys, dt) {
    const mass = VEHICLE.massKg;

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
  }

  // Standard differential-drive wheel speed decomposition.
  wheelSpeeds() {
    const halfTrack = VEHICLE.track / 2;
    return {
      left: this.v - this.omega * halfTrack,
      right: this.v + this.omega * halfTrack,
    };
  }
}
