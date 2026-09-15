import * as THREE from 'three';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { VehiclePhysics, VEHICLE, MAX_SPEED, MAX_ANGULAR } from './vehicle_physics.js';
import { createCourseTexture, COURSE_WIDTH_M, COURSE_DEPTH_M } from './course.js';
import { ThresholdLaneDetector } from './lane_threshold_detector.js';
import { ModelLaneDetector } from './lane_model_detector.js';
import { TwistMux } from './twist_mux.js';
import { BevTransformer, BevLaneExtractor, stepPurePursuitControl, applyLaneOverlay } from './oit_lane_pipeline.js';

// ---------------------------------------------------------------------------
// Geometry taken directly from vehicles/sample_vehicle/xacro/ai_car1.xacro
// (base_footprint -> base_link joint is identity, so every offset below is
// expressed directly in the base_footprint frame, in ROS units/convention).
// ---------------------------------------------------------------------------
// Loaded relative to index.html; serve the repo root so this resolves
// (see README.md for the exact command).
const MESH_DIR = '../vehicles/sample_vehicle/xacro/meshes/';

const BODY_OFFSET = { x: 0.0, y: 0.0, z: VEHICLE.wheelRadius, roll: 0, pitch: 0, yaw: Math.PI };
const WHEEL_LEFT_JOINT = { x: 0.0, y: 0.3, z: VEHICLE.wheelRadius, roll: Math.PI / 2, pitch: 0, yaw: Math.PI / 2 };
const WHEEL_RIGHT_JOINT = { x: 0.0, y: -0.3, z: VEHICLE.wheelRadius, roll: Math.PI / 2, pitch: 0, yaw: Math.PI / 2 };
const CASTER_JOINT = { x: -0.76, y: 0.0, z: 0.10, roll: Math.PI / 2, pitch: 0, yaw: Math.PI / 2 };
const WHEEL_SCALE = { thickness: 0.05, size: 0.12 }; // wheel_macro.xacro THICKNESS_SCALE/SIZE_SCALE
const CASTER_SCALE = 0.10; // caster_back_macro.xacro SCALE

// ZED camera mount, relative to base_link (config/zedx/extrinsic/extrinsic.yaml
// "position"; the camera_joint origin has rpy=0, so it faces straight ahead).
const CAMERA_MOUNT = { x: 0.055, y: 0.0, z: 0.44 };

// ---------------------------------------------------------------------------
// Scene setup
// ---------------------------------------------------------------------------
const container = document.getElementById('scene-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1d21);
scene.fog = new THREE.Fog(0x1a1d21, 20, 80);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 500);
camera.position.set(-2.5, 2.0, 2.5);

// Onboard (vehicle-mounted) camera, rendered picture-in-picture top-right.
// Vertical FOV = 70.6 deg, matching the real ZED X HD1080 intrinsic parameters
// (see config/zedx/intrinsic/SN48442725/HD1080.yaml: 2 * atan(540 / 763.17) ≈ 70.6 deg).
const onboardCamera = new THREE.PerspectiveCamera(70.6, 16 / 9, 0.05, 500);
const pipFrame = document.getElementById('pip-frame');

// Offscreen capture of the onboard camera, published as a compressed image
// topic mirroring the real vehicle's naming (see topic_list.yaml
// sensing.zedx.left_image.undistorted) with the standard image_transport
// "/compressed" suffix. Rendered separately from the on-screen PiP view so
// its resolution/rate can be tuned independently of the display.
const CAPTURE_WIDTH = 640;
const CAPTURE_HEIGHT = 360;
const CAPTURE_JPEG_QUALITY = 0.7;
const captureCanvas = document.createElement('canvas');
captureCanvas.width = CAPTURE_WIDTH;
captureCanvas.height = CAPTURE_HEIGHT;
const captureRenderer = new THREE.WebGLRenderer({
  canvas: captureCanvas,
  antialias: true,
  preserveDrawingBuffer: true, // required so toDataURL() can read back this frame
});
captureRenderer.setPixelRatio(1);
captureRenderer.setSize(CAPTURE_WIDTH, CAPTURE_HEIGHT, false);
captureRenderer.toneMapping = THREE.ACESFilmicToneMapping;
captureRenderer.toneMappingExposure = 1.0;

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.1;
controls.minDistance = 1.0;
controls.maxDistance = 40;
controls.maxPolarAngle = Math.PI * 0.49;
controls.autoRotate = false;
controls.autoRotateSpeed = 6; // ~10s per revolution at 60fps (default 2 = 30s)

scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const sun = new THREE.DirectionalLight(0xffffff, 0.9);
sun.position.set(6, 10, 4);
scene.add(sun);
// Dim fill light from roughly the opposite side, so the side facing away
// from the sun isn't left almost black -- a single directional light
// otherwise makes one side of the body look "broken"/unlit by comparison.
const fill = new THREE.DirectionalLight(0xffffff, 0.35);
fill.position.set(-6, 4, -4);
scene.add(fill);

// Unlit so it stays a flat, readable black regardless of light intensity
// (MeshStandardMaterial was blowing out to near-white under the sun light).
const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(200, 200),
  new THREE.MeshBasicMaterial({ color: 0x050608 })
);
ground.rotation.x = -Math.PI / 2;
scene.add(ground);

const grid = new THREE.GridHelper(200, 200, 0x4a5058, 0x2a2e34);
scene.add(grid);

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ---------------------------------------------------------------------------
// ROS -> Three.js frame conversion (Z-up, X-forward  ->  Y-up), the same
// -90deg X rotation used by ros3d.js / URDF viewers. Every child below is
// positioned using raw ROS xyz/rpy numbers.
// ---------------------------------------------------------------------------
const rosRoot = new THREE.Group();
rosRoot.rotation.x = -Math.PI / 2;
scene.add(rosRoot);

const vehicleRoot = new THREE.Group();
rosRoot.add(vehicleRoot);

// Course layout the user asked to add, textured from their own course image
// (see course.js). Real-world scale not modeled yet -- visual only, no
// collision or lap/gate logic. A child of rosRoot (like everything else
// placed in ROS coordinates) so it can be positioned directly with ROS
// x/y/yaw: THREE.PlaneGeometry already lies in its local XY plane with a
// +Z normal by default, which is exactly "flat on the ground, facing up"
// in ROS convention -- unlike `ground` above (added straight to `scene`,
// Three.js's own Y-up world), this needs no extra rotation.x tilt.
const COURSE_POSE = { x: 13.22, y: 35.41, z: 0.01, roll: 0, pitch: 0, yaw: Math.PI / 2 };
const course = new THREE.Mesh(
  new THREE.PlaneGeometry(COURSE_WIDTH_M, COURSE_DEPTH_M),
  new THREE.MeshBasicMaterial({ map: createCourseTexture() })
);
setPose(course, COURSE_POSE); // setPose is defined just below; hoisted, so usable here
rosRoot.add(course);

function setPose(object3d, pose) {
  object3d.position.set(pose.x, pose.y, pose.z);
  // URDF rpy is R = Rz(yaw)*Ry(pitch)*Rx(roll) (fixed-axis/extrinsic).
  // Three.js Euler order 'ZYX' with (x,y,z)=(roll,pitch,yaw) reproduces
  // that exact matrix (verified against Matrix4.makeRotationFromEuler) --
  // 'XYZ' does NOT, and only coincidentally looked right for the body
  // mesh above because its roll and pitch are both zero.
  object3d.rotation.set(pose.roll, pose.pitch, pose.yaw, 'ZYX');
}

const bodyGroup = new THREE.Group();
setPose(bodyGroup, BODY_OFFSET);
vehicleRoot.add(bodyGroup);

const wheelLeftJoint = new THREE.Group();
setPose(wheelLeftJoint, WHEEL_LEFT_JOINT);
vehicleRoot.add(wheelLeftJoint);
const wheelLeftSpin = new THREE.Group();
wheelLeftJoint.add(wheelLeftSpin);

const wheelRightJoint = new THREE.Group();
setPose(wheelRightJoint, WHEEL_RIGHT_JOINT);
vehicleRoot.add(wheelRightJoint);
const wheelRightSpin = new THREE.Group();
wheelRightJoint.add(wheelRightSpin);

const casterJoint = new THREE.Group();
setPose(casterJoint, CASTER_JOINT);
vehicleRoot.add(casterJoint);
const casterSpin = new THREE.Group();
casterJoint.add(casterSpin);

// ---------------------------------------------------------------------------
// Load meshes (COLLADA, same assets referenced by the xacro model)
// ---------------------------------------------------------------------------
const loader = new ColladaLoader();

// tire.dae/AIF_body.dae carry Phong materials with a specular highlight
// baked in by the exporter; at certain camera/light angles that highlight
// blows out to solid white. Kill specular entirely so shading stays flat
// and readable from any angle.
//
// tire.dae's own color texture (tire_color.png) is not real tire artwork --
// it's a rainbow UV-checker placeholder image -- so instead of using it
// (which would render the wheels as a rainbow grid rather than fixing
// anything), wheel/caster loads are given a plain dark rubber color and
// their texture map is dropped.
//
// tire.dae also has 32 unmaterialed <lines> baked in by the exporter (edge
// lines with no material binding), which ColladaLoader defaults to plain
// white LineBasicMaterial (see buildObjects: `materials.push(new
// LineBasicMaterial())`). Those render as LineSegments, not Mesh, so they
// were previously skipped here entirely -- leaving bright white lines
// overlaid on the tire regardless of the mesh tint below. Style those too.
//
// tire.dae's 320 vertices sit in 3 clean concentric radius bands around the
// local rotation axis (measured as sqrt(y^2+z^2) in its own object space,
// checked directly against the .dae's raw position array): ~0.20 (center
// hub cap), ~0.56-0.60 (spokes), ~0.96-1.0 (outer tread), with real gaps
// between them -- so a single radius threshold cleanly separates "hub cap"
// vertices from everything else. hubRadius/hubColor below paint just that
// inner disc a different (non-rubber) color via per-vertex colors.
function styleMaterials(object3d, { tintColor, hubColor, hubRadius } = {}) {
  object3d.traverse((obj) => {
    if (!obj.isMesh && !obj.isLine && !obj.isLineSegments && !obj.isPoints) return;

    if (tintColor === undefined) {
      const materials = Array.isArray(obj.material) ? obj.material : [obj.material];
      materials.forEach((mat) => {
        if (!mat) return;
        if (mat.specular) mat.specular.setRGB(0, 0, 0);
        if (typeof mat.shininess === 'number') mat.shininess = 0;
      });
      return;
    }

    // Replace with a fully unlit material so the tint is immune to
    // directional-light angle, specular, and tone-mapping entirely -- no
    // dependence on how the source Lambert material shades per pixel.
    if (obj.isMesh && hubColor !== undefined) {
      applyHubTint(obj, tintColor, hubColor, hubRadius);
    } else {
      const flat = new THREE.MeshBasicMaterial({ color: tintColor });
      obj.material = Array.isArray(obj.material) ? obj.material.map(() => flat) : flat;
    }
  });
}

function applyHubTint(mesh, treadColor, hubColor, hubRadius) {
  const position = mesh.geometry.attributes.position;
  const tread = new THREE.Color(treadColor);
  const hub = new THREE.Color(hubColor);
  const colors = new Float32Array(position.count * 3);
  for (let i = 0; i < position.count; i++) {
    const y = position.getY(i);
    const z = position.getZ(i);
    const radius = Math.sqrt(y * y + z * z);
    const c = radius < hubRadius ? hub : tread;
    colors[i * 3] = c.r;
    colors[i * 3 + 1] = c.g;
    colors[i * 3 + 2] = c.b;
  }
  mesh.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  mesh.material = new THREE.MeshBasicMaterial({ vertexColors: true });
}

function loadInto(parent, url, styleOptions) {
  loader.load(
    url,
    (collada) => {
      // These .dae assets declare <up_axis>Z_UP</up_axis>, so ColladaLoader
      // already rotates collada.scene by -90deg about X to present it as
      // Y-up. rosRoot (below) does that same Z-up -> Y-up conversion once
      // for the whole vehicle, so undo ColladaLoader's own rotation here to
      // avoid applying it twice (which was pitching/mispositioning meshes).
      collada.scene.rotation.x = 0;
      styleMaterials(collada.scene, styleOptions);
      parent.add(collada.scene);
    },
    undefined,
    (err) => console.error(`Failed to load ${url}`, err)
  );
}

const WHEEL_RUBBER_COLOR = 0x1c1c1c;
const WHEEL_HUB_COLOR = 0x9a9ea3; // center cap / hub, distinct from the tire rubber
const WHEEL_HUB_RADIUS = 0.4; // tire.dae local units; hub cluster sits at r~0.20, next band starts ~0.56
const WHEEL_STYLE = { tintColor: WHEEL_RUBBER_COLOR, hubColor: WHEEL_HUB_COLOR, hubRadius: WHEEL_HUB_RADIUS };

wheelLeftSpin.scale.set(WHEEL_SCALE.thickness, WHEEL_SCALE.size, WHEEL_SCALE.size);
wheelRightSpin.scale.set(WHEEL_SCALE.thickness, WHEEL_SCALE.size, WHEEL_SCALE.size);
casterSpin.scale.set(CASTER_SCALE, CASTER_SCALE, CASTER_SCALE);

loadInto(bodyGroup, MESH_DIR + 'AIF_body.dae');
loadInto(wheelLeftSpin, MESH_DIR + 'tire.dae', WHEEL_STYLE);
loadInto(wheelRightSpin, MESH_DIR + 'tire.dae', WHEEL_STYLE);
loadInto(casterSpin, MESH_DIR + 'tire.dae', WHEEL_STYLE);

// ---------------------------------------------------------------------------
// Keyboard input (held-down state; ignored while typing in an HUD input)
// ---------------------------------------------------------------------------
const keys = { forward: false, backward: false, left: false, right: false };
const keyEls = {};
document.querySelectorAll('#keys .key').forEach((el) => {
  keyEls[el.dataset.key] = el;
});

function isTypingTarget(target) {
  return target && (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA');
}

window.addEventListener('keydown', (e) => {
  if (isTypingTarget(e.target)) return;
  switch (e.code) {
    case 'KeyW': keys.forward = true; break;
    case 'KeyS': keys.backward = true; break;
    case 'KeyA': keys.left = true; break;
    case 'KeyD': keys.right = true; break;
    case 'KeyR':
      physics.reset();
      odomTrailPoints.length = 0;
      odomTrailGeometry.setFromPoints([]);
      break;
    default: return;
  }
  if (keyEls[e.code]) keyEls[e.code].classList.add('active');
  e.preventDefault();
});

window.addEventListener('keyup', (e) => {
  switch (e.code) {
    case 'KeyW': keys.forward = false; break;
    case 'KeyS': keys.backward = false; break;
    case 'KeyA': keys.left = false; break;
    case 'KeyD': keys.right = false; break;
    default: return;
  }
  if (keyEls[e.code]) keyEls[e.code].classList.remove('active');
});

// ---------------------------------------------------------------------------
// rosbridge connection (roslib.js, loaded globally as ROSLIB via <script>)
// ---------------------------------------------------------------------------
let ros = null;
let cmdVelTopic = null;
let compressedImageTopic = null;
let imuTopic = null;
const IMAGE_TOPIC_NAME = '/aiformula_sensing/zed_node/left_image/undistorted/compressed';
const IMAGE_FRAME_ID = 'zed_left_camera_optical_frame'; // matches zed_macro.xacro
const IMU_TOPIC_NAME = '/aiformula_sensing/vectornav/imu'; // topic_list.yaml sensing.vectornav.imu
// The real vectornav driver publishes with frame_id "vectornav" (see
// sensing/vectornav/vectornav/config/vectornav.yaml), but its mounting
// offset relative to base_link isn't known here, so -- per instruction --
// the simulator treats the sensor as coincident with base_link and computes
// its readings directly from the vehicle's own pose/velocity.
const IMU_FRAME_ID = 'vectornav';
const GRAVITY = 9.81; // [m/s^2]

let odomTopic = null;
// Matches sensing/odometry_publisher's gyro_odometry_publisher exactly:
// topic_list.yaml sensing.odometry.gyro, and the frame ids it's launched
// with (odom_frame_id=FRAME_IDS.odom, vehicle_frame_id=FRAME_IDS.base_footprint).
// Note: the real node actually subscribes to the ZED camera's IMU for this
// (see odometry_publisher/launch/gyro_odometry_publisher.launch.py:
// sub_imu -> sensing.zedx.imu), not vectornav -- but this simulator only
// models one IMU, so per instruction it reuses the simulated vectornav IMU
// (physics.yaw / physics.omega) as the gyro input instead.
const ODOM_TOPIC_NAME = '/aiformula_sensing/gyro_odometry_publisher/odom';
const ODOM_FRAME_ID = 'odom';
const ODOM_CHILD_FRAME_ID = 'base_footprint';

let canTopic = null;
let maskImageTopic = null;
let annotatedCameraImageTopic = null;
let bevAnnotatedImageTopic = null;
let annotatedMaskImageTopic = null;
let laneLeftTopic = null;
let laneRightTopic = null;
let laneCenterTopic = null;
let targetTrajectoryTopic = null;
let autonomousCmdVelTopic = null;
let laneTrackerStatusTopic = null;
let muxedCmdVelTopic = null;
// "ROS2連携" detector mode subscriptions: when selected, the heavy YOLOP/BEV/
// Pure Pursuit computation runs on the ROS 2 side (see
// src/oit_navigation/launch/simulator_test.launch.py) instead of in the
// browser, and the simulator just displays what it publishes back + drives
// the vehicle from its cmd_vel -- see setDetectorMode('ros2') below.
let rosMaskImageSub = null;
let rosBevAnnotatedImageSub = null;
let rosAutonomousCmdVelSub = null;
let rosLaneTrackerStatusSub = null;
// Wheel-speed CAN frame, decoded on the real robot by
// odometry_publisher/include/odometry_publisher/wheel.hpp:
//   RPM_ID = 1809; data[0..3] = right wheel RPM, data[4..7] = left wheel RPM,
//   both int32 little-endian; speed[m/s] = rpm * (1/60) * (diameter * PI).
// Encoding uses config/wheel.yaml's diameter (0.254m, what that decoder
// actually uses) rather than the xacro's WHEEL_RADIUS (0.12m, used only for
// this simulator's own wheel-spin animation) so a real consumer decodes the
// exact wheelSpeeds() m/s values back out, regardless of that pre-existing
// xacro/yaml radius mismatch.
const CAN_TOPIC_NAME = '/aiformula_sensing/vehicle_info';
const CAN_RPM_ID = 1809;
const CAN_WHEEL_DIAMETER = 0.254; // [m] config/wheel.yaml wheel.diameter
const CAN_PUBLISH_HZ = 100; // matches the real CAN bus's ~10ms measurement cycle

// ---------------------------------------------------------------------------
// oit_navigation lane pipeline: white-line detection -> BEV transform ->
// Pure Pursuit control, ported to run client-side (see
// js/lane_threshold_detector.js and js/oit_lane_pipeline.js). Topic names
// below are copied verbatim from
// src/oit_navigation/config/navigation_params.yaml / the node's declared
// parameter defaults, so this simulator's output is a drop-in match for the
// real vehicle's oit_navigation stack.
const ROBOT_FRAME_ID = 'base_link';
const MASK_IMAGE_TOPIC = '/aiformula_perception/object_road_detector/mask_image';
const ANNOTATED_CAMERA_IMAGE_TOPIC = '/aiformula_visualization/object_road_detector/annotated_image';
const BEV_ANNOTATED_IMAGE_TOPIC = '/aiformula_visualization/bev_annotated_image';
const ANNOTATED_MASK_IMAGE_TOPIC = '/aiformula_perception/lane_line_publisher/annotated_mask_image';
const LANE_LINE_LEFT_TOPIC = '/aiformula_perception/lane_line_publisher/lane_lines/left';
const LANE_LINE_RIGHT_TOPIC = '/aiformula_perception/lane_line_publisher/lane_lines/right';
const LANE_LINE_CENTER_TOPIC = '/aiformula_perception/lane_line_publisher/lane_lines/center';
const TARGET_TRAJECTORY_TOPIC = '/aiformula_visualization/target_trajectory';
// "mpc" input of twist_mux (see launchers/sample_launchers/launch/twist_mux.launch.py) --
// the real vehicle's autonomous lane-tracker output topic. This simulator
// always publishes it once the pipeline is running (mirroring how the real
// node always publishes regardless of twist_mux arbitration); whether the
// simulated vehicle itself obeys it is decided by the twistMux instance
// below (js/twist_mux.js), gated by the "自動運転" HUD toggle.
const AUTONOMOUS_CMD_VEL_TOPIC = '/aiformula_control/extremum_seeking_mpc/cmd_vel';
const LANE_TRACKER_STATUS_TOPIC = '/aiformula_control/lane_tracker/status';
// twist_mux's arbitrated output (topic_list.yaml control.speed_command.multiplexed).
const MUXED_CMD_VEL_TOPIC = '/aiformula_control/twist_mux/cmd_vel';

// Priorities/timeouts copied verbatim from
// launchers/sample_launchers/config/twist_mux.yaml -- gamepad (150) always
// outranks mpc (50), so a human on WASD instantly overrides autonomous
// driving, and control reverts to autonomous once no key has been held for
// timeoutSec. (The real config's handle_controller (250, physical steering
// wheel) and handle_controller_coasting (1) have no equivalent input in
// this simulator and are omitted.)
const TWIST_MUX_SOURCES = [
  { name: 'gamepad', priority: 150, timeoutSec: 0.3 },
  { name: 'mpc', priority: 50, timeoutSec: 0.3 },
];

// Autonomous control law gains/limits (navigation_params.yaml), except
// target speed / max angular speed: per instruction, those stay the
// simulator's own WASD limits (MAX_SPEED=1.5, MAX_ANGULAR=1.2) rather than
// the real vehicle's defaults (target_linear_speed=1.0, max_angular_speed=1.5).
const PURE_PURSUIT_PARAMS = {
  lookaheadDistance: 5.0,
  targetSpeed: MAX_SPEED,
  angularGain: 0.85,
  crossTrackGain: 0.95,
  maxAngularSpeed: MAX_ANGULAR,
  maxAngularAccel: 4.0,
};
const LANE_DATA_TIMEOUT_MS = 800; // navigation_params.yaml data_timeout

const urlInput = document.getElementById('ros-url');
const topicInput = document.getElementById('ros-topic');
const connectBtn = document.getElementById('connect-btn');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');

function setStatus(state, label) {
  statusDot.className = state;
  statusText.textContent = label;
}

function disconnect() {
  if (ros) ros.close();
  ros = null;
  cmdVelTopic = null;
  compressedImageTopic = null;
  imuTopic = null;
  odomTopic = null;
  canTopic = null;
  maskImageTopic = null;
  annotatedCameraImageTopic = null;
  bevAnnotatedImageTopic = null;
  annotatedMaskImageTopic = null;
  laneLeftTopic = null;
  laneRightTopic = null;
  laneCenterTopic = null;
  targetTrajectoryTopic = null;
  autonomousCmdVelTopic = null;
  laneTrackerStatusTopic = null;
  muxedCmdVelTopic = null;
  rosMaskImageSub = null;
  rosBevAnnotatedImageSub = null;
  rosAutonomousCmdVelSub = null;
  rosLaneTrackerStatusSub = null;
  connectBtn.textContent = '接続';
  setStatus('', '未接続');
}

function connect() {
  setStatus('connecting', '接続中...');
  connectBtn.disabled = true;

  ros = new ROSLIB.Ros({ url: urlInput.value });

  ros.on('connection', () => {
    setStatus('connected', '接続済み');
    connectBtn.disabled = false;
    connectBtn.textContent = '切断';
    cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: topicInput.value,
      messageType: 'geometry_msgs/msg/Twist',
    });
    compressedImageTopic = new ROSLIB.Topic({
      ros,
      name: IMAGE_TOPIC_NAME,
      messageType: 'sensor_msgs/msg/CompressedImage',
    });
    imuTopic = new ROSLIB.Topic({
      ros,
      name: IMU_TOPIC_NAME,
      messageType: 'sensor_msgs/msg/Imu',
    });
    odomTopic = new ROSLIB.Topic({
      ros,
      name: ODOM_TOPIC_NAME,
      messageType: 'nav_msgs/msg/Odometry',
    });
    canTopic = new ROSLIB.Topic({
      ros,
      name: CAN_TOPIC_NAME,
      messageType: 'can_msgs/msg/Frame',
    });
    maskImageTopic = new ROSLIB.Topic({ ros, name: MASK_IMAGE_TOPIC, messageType: 'sensor_msgs/msg/Image' });
    annotatedCameraImageTopic = new ROSLIB.Topic({ ros, name: ANNOTATED_CAMERA_IMAGE_TOPIC, messageType: 'sensor_msgs/msg/Image' });
    bevAnnotatedImageTopic = new ROSLIB.Topic({ ros, name: BEV_ANNOTATED_IMAGE_TOPIC, messageType: 'sensor_msgs/msg/Image' });
    annotatedMaskImageTopic = new ROSLIB.Topic({ ros, name: ANNOTATED_MASK_IMAGE_TOPIC, messageType: 'sensor_msgs/msg/Image' });
    laneLeftTopic = new ROSLIB.Topic({ ros, name: LANE_LINE_LEFT_TOPIC, messageType: 'nav_msgs/msg/Path' });
    laneRightTopic = new ROSLIB.Topic({ ros, name: LANE_LINE_RIGHT_TOPIC, messageType: 'nav_msgs/msg/Path' });
    laneCenterTopic = new ROSLIB.Topic({ ros, name: LANE_LINE_CENTER_TOPIC, messageType: 'nav_msgs/msg/Path' });
    targetTrajectoryTopic = new ROSLIB.Topic({ ros, name: TARGET_TRAJECTORY_TOPIC, messageType: 'nav_msgs/msg/Path' });
    autonomousCmdVelTopic = new ROSLIB.Topic({ ros, name: AUTONOMOUS_CMD_VEL_TOPIC, messageType: 'geometry_msgs/msg/Twist' });
    laneTrackerStatusTopic = new ROSLIB.Topic({ ros, name: LANE_TRACKER_STATUS_TOPIC, messageType: 'std_msgs/msg/String' });
    muxedCmdVelTopic = new ROSLIB.Topic({ ros, name: MUXED_CMD_VEL_TOPIC, messageType: 'geometry_msgs/msg/Twist' });

    // "ROS2連携" mode subscriptions (see onRos*() callbacks above) -- always
    // subscribed once connected, regardless of the current detectorMode;
    // each callback itself no-ops unless detectorMode === 'ros2'. Separate
    // Topic instances from the publish-side ones above (same topic names)
    // since roslib.js topics are one-directional.
    rosMaskImageSub = new ROSLIB.Topic({ ros, name: MASK_IMAGE_TOPIC, messageType: 'sensor_msgs/msg/Image' });
    rosMaskImageSub.subscribe(onRosMaskImage);
    rosBevAnnotatedImageSub = new ROSLIB.Topic({ ros, name: BEV_ANNOTATED_IMAGE_TOPIC, messageType: 'sensor_msgs/msg/Image' });
    rosBevAnnotatedImageSub.subscribe(onRosBevAnnotatedImage);
    rosAutonomousCmdVelSub = new ROSLIB.Topic({ ros, name: AUTONOMOUS_CMD_VEL_TOPIC, messageType: 'geometry_msgs/msg/Twist' });
    rosAutonomousCmdVelSub.subscribe(onRosAutonomousCmdVel);
    rosLaneTrackerStatusSub = new ROSLIB.Topic({ ros, name: LANE_TRACKER_STATUS_TOPIC, messageType: 'std_msgs/msg/String' });
    rosLaneTrackerStatusSub.subscribe(onRosLaneTrackerStatus);
  });

  ros.on('error', () => {
    setStatus('error', 'エラー');
    connectBtn.disabled = false;
    connectBtn.textContent = '接続';
  });

  ros.on('close', () => {
    cmdVelTopic = null;
    compressedImageTopic = null;
    imuTopic = null;
    odomTopic = null;
    canTopic = null;
    maskImageTopic = null;
    annotatedCameraImageTopic = null;
    bevAnnotatedImageTopic = null;
    annotatedMaskImageTopic = null;
    laneLeftTopic = null;
    laneRightTopic = null;
    laneCenterTopic = null;
    targetTrajectoryTopic = null;
    autonomousCmdVelTopic = null;
    laneTrackerStatusTopic = null;
    muxedCmdVelTopic = null;
    rosMaskImageSub = null;
    rosBevAnnotatedImageSub = null;
    rosAutonomousCmdVelSub = null;
    rosLaneTrackerStatusSub = null;
    connectBtn.disabled = false;
    connectBtn.textContent = '接続';
    setStatus('', '未接続');
  });
}

connectBtn.addEventListener('click', () => {
  if (ros) {
    disconnect();
  } else {
    connect();
  }
});

const PUBLISH_HZ = 10;
const PUBLISH_INTERVAL = 1 / PUBLISH_HZ;
let publishAccumulator = 0;

// Mirrors twist_mux's own timeout for this input (topic_list.yaml: gamepad/
// keyboard timeout: 0.3s) -- but applied to the *simulated vehicle itself*,
// not just the published topic. On the real robot, if cmd_vel stops
// arriving for 0.3s, twist_mux treats that source as timed out and the
// vehicle gets zero velocity from it. Previously this simulator only
// mirrored that on the wire (publishCmdVel no-ops once disconnected) while
// the local physics kept driving forever off raw key state, regardless of
// connection status -- so a dropped rosbridge connection here never looked
// like what it actually causes on the real vehicle. Now, once armed (after
// the first successful publish), if CMD_VEL_TIMEOUT_SEC elapses without a
// fresh publish (disconnect, or the publish loop stalling), held keys are
// ignored and the vehicle coasts to a stop via the normal coast-resistance
// physics, same as releasing every key.
const CMD_VEL_TIMEOUT_SEC = 0.3;
const NO_KEYS = { forward: false, backward: false, left: false, right: false };
let lastCmdVelPublishTime = null;

function isCmdVelTimedOut() {
  return lastCmdVelPublishTime !== null && (performance.now() - lastCmdVelPublishTime) / 1000 > CMD_VEL_TIMEOUT_SEC;
}

function publishCmdVel(v, omega) {
  if (!cmdVelTopic) return;
  cmdVelTopic.publish(
    new ROSLIB.Message({
      linear: { x: v, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: omega },
    })
  );
  lastCmdVelPublishTime = performance.now();
}

// twist_mux's arbitrated output (topic_list.yaml control.speed_command.multiplexed).
// Since this simulator's vehicle_physics *is* the single shared vehicle
// state (there's no separate motor-controller tracking loop to observe),
// the muxed output is simply whatever physics.v/physics.omega ended up
// being this step -- that's already the result of twistMux's own arbitration
// (see animate()'s activeSource branch above).
function publishMuxedCmdVel(v, omega) {
  if (!muxedCmdVelTopic) return;
  muxedCmdVelTopic.publish(
    new ROSLIB.Message({
      linear: { x: v, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: omega },
    })
  );
}

const IMAGE_PUBLISH_HZ = 15;
const IMAGE_PUBLISH_INTERVAL = 1 / IMAGE_PUBLISH_HZ;
let imagePublishAccumulator = 0;

// Renders the onboard camera to an offscreen canvas and publishes it as a
// sensor_msgs/msg/CompressedImage. rosbridge decodes a base64 *string* into
// the message's uint8[] "data" field for byte-array fields, so a stripped
// canvas.toDataURL() output is exactly what's expected here.
// Renders the onboard camera into captureCanvas. Split out from publishing
// so the oit_navigation lane pipeline below can run off the same frame
// (renderOnboardCapture() is called once per accumulator tick regardless of
// rosbridge connection state, so the two panels work locally even when not
// connected -- same as the existing camera PiP).
function renderOnboardCapture() {
  onboardCamera.aspect = CAPTURE_WIDTH / CAPTURE_HEIGHT;
  onboardCamera.updateProjectionMatrix();
  captureRenderer.render(scene, onboardCamera);
}

function publishCompressedImage() {
  if (!compressedImageTopic) return;

  const dataUrl = captureCanvas.toDataURL('image/jpeg', CAPTURE_JPEG_QUALITY);
  const base64Data = dataUrl.slice(dataUrl.indexOf(',') + 1);
  const nowMs = Date.now();

  compressedImageTopic.publish(
    new ROSLIB.Message({
      header: {
        stamp: { sec: Math.floor(nowMs / 1000), nanosec: (nowMs % 1000) * 1e6 },
        frame_id: IMAGE_FRAME_ID,
      },
      format: 'jpeg',
      data: base64Data,
    })
  );
}

// ---------------------------------------------------------------------------
// oit_navigation lane pipeline wiring: threshold-based white-line mask ->
// BEV warp -> lane extraction -> Pure Pursuit, rendered into the two HUD
// panels and published on the unified topic names declared above.
// ---------------------------------------------------------------------------
const thresholdDetector = new ThresholdLaneDetector();
const modelDetector = new ModelLaneDetector();
const bevTransformer = new BevTransformer();
const laneExtractor = new BevLaneExtractor();

const laneCanvas = document.getElementById('lane-canvas');
const laneCtx = laneCanvas.getContext('2d', { willReadFrequently: true });
const bevCanvas = document.getElementById('bev-canvas');
const bevCtx = bevCanvas.getContext('2d', { willReadFrequently: true });
const oitModeEl = document.getElementById('oit-mode');
const detectorStatusEl = document.getElementById('oit-detector-status');
const thresholdBtn = document.getElementById('detector-threshold-btn');
const modelBtn = document.getElementById('detector-model-btn');
const ros2Btn = document.getElementById('detector-ros2-btn');
const autonomousBtn = document.getElementById('autonomous-btn');

// Model file produced by src/oit_navigation/oit_navigation/export_onnx_web.py
// from models/honda_shihou_finetuned_best.pth (see js/lane_model_detector.js).
const MODEL_ONNX_URL = 'models/honda_shihou_finetuned.onnx';
const MODEL_WASM_DIR = 'vendor/onnxruntime-web/';

let activeDetector = thresholdDetector;
let modelLoadPromise = null;

// 'threshold' | 'model' | 'ros2'. In 'ros2' mode, the heavy YOLOP/BEV/Pure
// Pursuit computation runs on the ROS 2 side instead of in the browser (see
// src/oit_navigation/launch/simulator_test.launch.py) -- updateLanePipeline()
// below becomes a no-op, and the panels + autonomous "mpc" cmd instead come
// from subscribing to what that pipeline publishes back over rosbridge
// (onRosMaskImage/onRosBevAnnotatedImage/onRosAutonomousCmdVel below).
let detectorMode = 'threshold';
let lastRosCmdVelTime = 0;

// Switches where the "mpc" source's mask/BEV/cmd_vel comes from. The ONNX
// model is loaded lazily on first switch to "model" mode (not at startup)
// so the default threshold mode never pays its download/WASM-init cost. If
// loading fails, falls back to threshold mode so the simulator keeps working.
function setDetectorMode(mode) {
  detectorMode = mode;
  thresholdBtn.classList.toggle('active', mode === 'threshold');
  modelBtn.classList.toggle('active', mode === 'model');
  ros2Btn.classList.toggle('active', mode === 'ros2');

  if (mode === 'threshold') {
    activeDetector = thresholdDetector;
    detectorStatusEl.textContent = '閾値処理';
    return;
  }

  if (mode === 'ros2') {
    detectorStatusEl.textContent = ros ? 'ROS2連携 (信号待ち)' : 'ROS2連携 (rosbridge未接続)';
    return;
  }

  if (modelDetector.session) {
    activeDetector = modelDetector;
    detectorStatusEl.textContent = 'モデル (読込済)';
    return;
  }

  detectorStatusEl.textContent = 'モデル読込中...';
  if (!modelLoadPromise) {
    modelLoadPromise = modelDetector.load(MODEL_ONNX_URL, MODEL_WASM_DIR);
  }
  modelLoadPromise
    .then(() => {
      // Only switch over if the user hasn't switched to a different mode
      // while the model was loading.
      if (modelBtn.classList.contains('active')) {
        activeDetector = modelDetector;
        detectorStatusEl.textContent = 'モデル (読込済)';
      }
    })
    .catch((err) => {
      console.error('Failed to load lane detection ONNX model', err);
      detectorStatusEl.textContent = 'モデル読込エラー(閾値処理へ切替)';
      setDetectorMode('threshold');
    });
}

thresholdBtn.addEventListener('click', () => setDetectorMode('threshold'));
modelBtn.addEventListener('click', () => setDetectorMode('model'));
ros2Btn.addEventListener('click', () => setDetectorMode('ros2'));
setDetectorMode('threshold');

let autonomousMode = false;
let latestAutonomousCmd = { v: 0, omega: 0 };
let lastOmegaCmd = 0;
let lastControlTime = performance.now();
let lastMaskTime = 0;

// Arbitrates between WASD ("gamepad") and Pure Pursuit ("mpc") exactly like
// the real vehicle's twist_mux (see js/twist_mux.js and TWIST_MUX_SOURCES
// above): a human on WASD always overrides autonomous driving instantly,
// and control reverts to autonomous once no key has been held for 0.3s.
// "mpc" starts disabled since autonomousMode starts false. This is the same
// mux regardless of detectorMode -- only *where* "mpc" commands come from
// (local computation vs. subscribed ROS 2 cmd_vel) changes.
const twistMux = new TwistMux(TWIST_MUX_SOURCES);
twistMux.setEnabled('mpc', autonomousMode);

const twistMuxGamepadStateEl = document.getElementById('twist-mux-gamepad-state');
const twistMuxMpcStateEl = document.getElementById('twist-mux-mpc-state');
const twistMuxActiveEl = document.getElementById('twist-mux-active');

autonomousBtn.addEventListener('click', () => {
  autonomousMode = !autonomousMode;
  autonomousBtn.textContent = `自動運転: ${autonomousMode ? 'ON' : 'OFF'}`;
  autonomousBtn.classList.toggle('active', autonomousMode);
  twistMux.setEnabled('mpc', autonomousMode);
});

// ---------------------------------------------------------------------------
// "ROS2連携" mode: subscription callbacks for what
// src/oit_navigation/launch/simulator_test.launch.py's yolop_lane_detector /
// bev_pure_pursuit_node publish back over rosbridge. These subscriptions are
// always live once connected (see connect() below); each callback no-ops
// unless detectorMode is actually 'ros2', so switching modes never needs to
// coordinate subscribe/unsubscribe timing with the connection state.
// ---------------------------------------------------------------------------

// mask_image (mono8): same shape/semantics as the local detectors' output,
// so it's rendered the same way onto the white-line panel (camera + green
// overlay via applyLaneOverlay -- see js/oit_lane_pipeline.js).
function onRosMaskImage(msg) {
  if (detectorMode !== 'ros2') return;
  const width = msg.width;
  const height = msg.height;
  const bytes = base64ToUint8Array(msg.data);
  const mask = new Uint8Array(width * height);
  for (let i = 0; i < mask.length; i++) mask[i] = bytes[i] > 0 ? 1 : 0;

  laneCtx.drawImage(captureCanvas, 0, 0, width, height);
  const imageData = laneCtx.getImageData(0, 0, width, height);
  applyLaneOverlay(imageData, mask, 0.5);
  laneCtx.putImageData(imageData, 0, 0);
}

// bev_annotated_image (bgr8): bev_pure_pursuit_node's own fully-rendered BEV
// visualization (sliding windows, left/right lane pixels, target line, EGO
// marker) -- displayed as-is rather than recomputed locally, so the BEV
// panel shows exactly what the real ROS 2 pipeline computed.
function onRosBevAnnotatedImage(msg) {
  if (detectorMode !== 'ros2') return;
  const width = msg.width;
  const height = msg.height;
  const bytes = base64ToUint8Array(msg.data);
  const imageData = bevCtx.createImageData(width, height);
  for (let i = 0, j = 0; j < imageData.data.length; i += 3, j += 4) {
    // bgr8: byte order is [B, G, R] per pixel.
    imageData.data[j] = bytes[i + 2];
    imageData.data[j + 1] = bytes[i + 1];
    imageData.data[j + 2] = bytes[i];
    imageData.data[j + 3] = 255;
  }
  bevCtx.putImageData(imageData, 0, 0);
}

// extremum_seeking_mpc/cmd_vel: the ROS 2 side's actual Pure Pursuit output.
// Fed directly into twistMux's "mpc" source (no local recomputation) --
// this is the real control link this mode exists for.
function onRosAutonomousCmdVel(msg) {
  if (detectorMode !== 'ros2') return;
  const v = msg.linear.x;
  const omega = msg.angular.z;
  const now = performance.now();
  latestAutonomousCmd = { v, omega };
  twistMux.update('mpc', v, omega, now);
  lastRosCmdVelTime = now;
  detectorStatusEl.textContent = 'ROS2連携 (受信中)';
}

// lane_tracker/status: "[oit_navigation] Mode=Both Lanes | Speed=... | ...".
// Only the Mode= field is pulled out, to reuse the same oit-mode HUD field
// the local pipeline already fills in.
function onRosLaneTrackerStatus(msg) {
  if (detectorMode !== 'ros2') return;
  const match = /Mode=([^|]+)/.exec(msg.data);
  oitModeEl.textContent = match ? match[1].trim() : msg.data;
}

// rosbridge decodes a base64 *string* into a message's uint8[] field (same
// trick publishCompressedImage above relies on for CompressedImage.data).
function uint8ArrayToBase64(bytes) {
  let binary = '';
  const chunkSize = 0x8000;
  for (let i = 0; i < bytes.length; i += chunkSize) {
    binary += String.fromCharCode.apply(null, bytes.subarray(i, i + chunkSize));
  }
  return btoa(binary);
}

// Inverse of uint8ArrayToBase64 -- rosbridge base64-encodes a subscribed
// message's uint8[] fields (sensor_msgs/Image.data) the same way it decodes
// a *published* base64 string back into bytes, so this is what "ROS2連携"
// mode needs to read the ROS-side yolop_lane_detector/bev_pure_pursuit_node
// nodes' Image messages back out.
function base64ToUint8Array(base64) {
  const binary = atob(base64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
  return bytes;
}

function nowStamp() {
  const nowMs = Date.now();
  return { sec: Math.floor(nowMs / 1000), nanosec: (nowMs % 1000) * 1e6 };
}

function canvasToRgb8Bytes(ctx, width, height) {
  const imgData = ctx.getImageData(0, 0, width, height);
  const rgb = new Uint8Array(width * height * 3);
  for (let i = 0, j = 0; i < imgData.data.length; i += 4, j += 3) {
    rgb[j] = imgData.data[i];
    rgb[j + 1] = imgData.data[i + 1];
    rgb[j + 2] = imgData.data[i + 2];
  }
  return rgb;
}

function publishImageTopic(topic, width, height, encoding, bytesPerPixel, dataBytes, frameId) {
  if (!topic) return;
  topic.publish(
    new ROSLIB.Message({
      header: { stamp: nowStamp(), frame_id: frameId },
      height,
      width,
      encoding,
      is_bigendian: 0,
      step: width * bytesPerPixel,
      data: uint8ArrayToBase64(dataBytes),
    })
  );
}

function publishPathTopic(topic, points) {
  if (!topic) return;
  const header = { stamp: nowStamp(), frame_id: ROBOT_FRAME_ID };
  topic.publish(
    new ROSLIB.Message({
      header,
      poses: (points || []).map(([x, y]) => ({
        header,
        pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      })),
    })
  );
}

// Triggered from the same accumulator as the camera image publish (15Hz).
// The threshold detector is synchronous and fast, but the ONNX model
// detector is not -- lanePipelineBusy is a single-slot guard (same idea as
// yolop_lane_detector.py's worker thread) so a slow model inference never
// gets a second one queued behind it.
let lanePipelineBusy = false;

async function updateLanePipeline() {
  // In "ROS2連携" mode, the ROS 2 side does this work instead (see
  // onRosMaskImage/onRosBevAnnotatedImage/onRosAutonomousCmdVel above) --
  // running the local detector here too would be wasted CPU (the exact
  // browser jank this mode exists to avoid) and would fight over the same
  // mask/BEV canvases and twistMux "mpc" slot.
  if (detectorMode === 'ros2') return;
  if (lanePipelineBusy) return;
  lanePipelineBusy = true;
  try {
    const { mask, width, height } = await activeDetector.infer(captureCanvas);
    lastMaskTime = performance.now();

    // Panel 1: camera + green lane overlay (draw_lane_lines port)
    laneCtx.drawImage(captureCanvas, 0, 0, width, height);
    const camImageData = laneCtx.getImageData(0, 0, width, height);
    applyLaneOverlay(camImageData, mask, 0.5);
    laneCtx.putImageData(camImageData, 0, 0);

    const maskBytes255 = new Uint8Array(mask.length);
    for (let i = 0; i < mask.length; i++) maskBytes255[i] = mask[i] * 255;
    publishImageTopic(maskImageTopic, width, height, 'mono8', 1, maskBytes255, IMAGE_FRAME_ID);
    publishImageTopic(
      annotatedCameraImageTopic,
      width,
      height,
      'rgb8',
      3,
      canvasToRgb8Bytes(laneCtx, width, height),
      IMAGE_FRAME_ID
    );

    // Panel 2: BEV transform + sliding-window lane extraction
    const bevMask = bevTransformer.warpToBev(mask, width, height);
    const { targetCenter, leftPts, rightPts } = laneExtractor.extractLaneTrajectories(bevMask, bevCtx);

    // Pure Pursuit control step (real elapsed time since the last control
    // update, matching bev_pure_pursuit_node.py's wall-clock dt).
    const now = performance.now();
    const dt = Math.min((now - lastControlTime) / 1000, 0.5);
    lastControlTime = now;

    const cmd = stepPurePursuitControl(targetCenter, lastOmegaCmd, dt, PURE_PURSUIT_PARAMS);
    lastOmegaCmd = cmd.omega;
    latestAutonomousCmd = { v: cmd.v, omega: cmd.omega };
    twistMux.update('mpc', cmd.v, cmd.omega, now);

    // Draw the Pure Pursuit trajectory + EGO marker onto the BEV panel
    // *after* the control step, so the line drawn is the origin-anchored
    // trajectory that actually connects to the vehicle's own origin (see
    // BevLaneExtractor.drawTrajectory's docstring).
    laneExtractor.drawTrajectory(bevCtx, cmd.farPt ? cmd.originAnchoredTrajectory : null, cmd.farPt);

    const bevBytes = canvasToRgb8Bytes(bevCtx, bevCanvas.width, bevCanvas.height);
    publishImageTopic(bevAnnotatedImageTopic, bevCanvas.width, bevCanvas.height, 'rgb8', 3, bevBytes, ROBOT_FRAME_ID);
    publishImageTopic(annotatedMaskImageTopic, bevCanvas.width, bevCanvas.height, 'rgb8', 3, bevBytes, ROBOT_FRAME_ID);
    publishPathTopic(laneLeftTopic, leftPts);
    publishPathTopic(laneRightTopic, rightPts);
    publishPathTopic(laneCenterTopic, targetCenter);

    if (autonomousCmdVelTopic) {
      autonomousCmdVelTopic.publish(
        new ROSLIB.Message({ linear: { x: cmd.v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: cmd.omega } })
      );
    }

    if (cmd.farPt) {
      publishPathTopic(targetTrajectoryTopic, cmd.originAnchoredTrajectory);
      const mode = leftPts && rightPts ? 'Both Lanes' : leftPts ? 'Left Anchor' : 'Right Anchor';
      oitModeEl.textContent = mode;
      if (laneTrackerStatusTopic) {
        const omegaStr = `${cmd.omega >= 0 ? '+' : ''}${cmd.omega.toFixed(2)}`;
        const originEy = cmd.crossTrackError;
        const farYStr = `${cmd.farPt[1] >= 0 ? '+' : ''}${cmd.farPt[1].toFixed(2)}`;
        laneTrackerStatusTopic.publish(
          new ROSLIB.Message({
            data:
              `[oit_navigation] Mode=${mode} | Speed=${cmd.v.toFixed(2)}m/s | Omega=${omegaStr}rad/s | ` +
              `Origin_ey=${originEy >= 0 ? '+' : ''}${originEy.toFixed(2)}m | Far_5m=(${cmd.farPt[0].toFixed(2)}, ${farYStr}m)`,
          })
        );
      }
    } else {
      oitModeEl.textContent = 'Fallback (No Lane)';
    }
  } catch (err) {
    console.error('oit_navigation lane pipeline error', err);
  } finally {
    lanePipelineBusy = false;
  }
}

// Watchdog: if no mask has completed for LANE_DATA_TIMEOUT_MS (model still
// loading, inference stalled, tab throttled), decelerate the autonomous
// command toward a stop instead of latching the last successful cmd_vel
// forever -- mirrors bev_pure_pursuit_node.py's watchdog_timer/_handle_fallback.
setInterval(() => {
  // In "ROS2連携" mode this local fallback doesn't apply -- twistMux's own
  // 0.3s timeout on the "mpc" source (fed directly by onRosAutonomousCmdVel)
  // already hands control back the same way if the ROS 2 side goes quiet.
  if (detectorMode === 'ros2') {
    if (lastRosCmdVelTime !== 0 && performance.now() - lastRosCmdVelTime > LANE_DATA_TIMEOUT_MS) {
      detectorStatusEl.textContent = 'ROS2連携 (信号途絶)';
    }
    return;
  }
  if (lastMaskTime !== 0 && performance.now() - lastMaskTime <= LANE_DATA_TIMEOUT_MS) return;
  const now = performance.now();
  const dt = Math.min((now - lastControlTime) / 1000, 0.5);
  lastControlTime = now;
  const cmd = stepPurePursuitControl(null, lastOmegaCmd, dt, PURE_PURSUIT_PARAMS);
  lastOmegaCmd = cmd.omega;
  latestAutonomousCmd = { v: cmd.v, omega: cmd.omega };
  // Matches the real bev_pure_pursuit_node.py: its watchdog still publishes
  // to the mpc topic while decelerating, so twist_mux still sees "mpc" as
  // alive/fresh even though no lane is currently tracked.
  twistMux.update('mpc', cmd.v, cmd.omega, now);
}, LANE_DATA_TIMEOUT_MS / 2);

const IMU_PUBLISH_HZ = 100;

// Synthesizes sensor_msgs/msg/Imu from the vehicle's own 2D physics state.
// The vehicle never rolls/pitches in this sim, so orientation is yaw-only,
// gyro is just the yaw rate, and linear_acceleration is: forward accel from
// physics.linearAccel, lateral accel approximated as the centripetal term
// v*omega (turning while moving), and +g on Z (a level, stationary
// accelerometer reads ~+9.81 as the reaction to gravity). This is noise-free
// synthetic data, so all covariances are left at zero rather than modeling
// real VectorNav sensor noise.
function publishImu() {
  if (!imuTopic) return;

  const halfYaw = physics.yaw / 2;
  const lateralAccel = physics.v * physics.omega;
  const nowMs = Date.now();

  imuTopic.publish(
    new ROSLIB.Message({
      header: {
        stamp: { sec: Math.floor(nowMs / 1000), nanosec: (nowMs % 1000) * 1e6 },
        frame_id: IMU_FRAME_ID,
      },
      orientation: { x: 0, y: 0, z: Math.sin(halfYaw), w: Math.cos(halfYaw) },
      orientation_covariance: new Array(9).fill(0),
      angular_velocity: { x: 0, y: 0, z: physics.omega },
      angular_velocity_covariance: new Array(9).fill(0),
      linear_acceleration: { x: physics.linearAccel, y: lateralAccel, z: GRAVITY },
      linear_acceleration_covariance: new Array(9).fill(0),
    })
  );
}

// Decoupled from the requestAnimationFrame render loop (which tops out at
// the display's refresh rate, typically 60Hz, and would otherwise cap the
// achievable publish rate) since publishing is cheap -- just reading the
// current physics state and sending JSON over the WebSocket -- so it can
// safely run on its own high-frequency timer without affecting render
// performance. Note: physics itself is still only integrated once per
// rendered frame (~60Hz), so consecutive IMU messages within the same
// frame will carry identical values; only the transmission rate is 100Hz.
setInterval(publishImu, 1000 / IMU_PUBLISH_HZ);

// Reproduces gyro_odometry_publisher's fusion: heading + yaw rate come from
// the (simulated) IMU, forward speed comes from the vehicle's own speed
// (standing in for the real node's CAN wheel-RPM input), integrated exactly
// as OdometryPublisher::updatePosition does: vx=v*cos(yaw), vy=v*sin(yaw),
// pos += (vx,vy)*dt -- which is the same integration vehicle_physics.js
// already does for physics.x/y. With no wheel-slip or gyro-drift noise
// modeled, this "sensor" output is therefore numerically identical to
// ground truth here, unlike the real robot where it would drift.
const ODOM_PUBLISH_HZ = 100; // matches gyro_odometry_publisher.yaml publish_timer_loop_duration: 10ms

function publishOdometry() {
  if (!odomTopic) return;

  const halfYaw = physics.yaw / 2;
  const vx = physics.v * Math.cos(physics.yaw);
  const vy = physics.v * Math.sin(physics.yaw);
  const nowMs = Date.now();
  const zeroCovariance36 = new Array(36).fill(0);

  odomTopic.publish(
    new ROSLIB.Message({
      header: {
        stamp: { sec: Math.floor(nowMs / 1000), nanosec: (nowMs % 1000) * 1e6 },
        frame_id: ODOM_FRAME_ID,
      },
      child_frame_id: ODOM_CHILD_FRAME_ID,
      pose: {
        pose: {
          position: { x: physics.x, y: physics.y, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(halfYaw), w: Math.cos(halfYaw) },
        },
        covariance: zeroCovariance36,
      },
      twist: {
        twist: {
          linear: { x: vx, y: vy, z: 0 },
          angular: { x: 0, y: 0, z: physics.omega },
        },
        covariance: zeroCovariance36,
      },
    })
  );
}

setInterval(publishOdometry, 1000 / ODOM_PUBLISH_HZ);

function int32ToLittleEndianBytes(value) {
  const buffer = new ArrayBuffer(4);
  new DataView(buffer).setInt32(0, Math.round(value), /* littleEndian= */ true);
  return Array.from(new Uint8Array(buffer));
}

// Publishes a can_msgs/msg/Frame carrying wheel RPM, in the exact byte
// layout odometry_publisher/include/odometry_publisher/wheel.hpp expects
// (see CAN_* constants above): data[0..3]=right RPM, data[4..7]=left RPM,
// both int32 little-endian, derived from physics.wheelSpeeds() (m/s).
function publishVehicleInfoCan() {
  if (!canTopic) return;

  const { left, right } = physics.wheelSpeeds();
  const wheelCircumference = CAN_WHEEL_DIAMETER * Math.PI;
  const toRpm = (speedMetersPerSecond) => (speedMetersPerSecond / wheelCircumference) * 60;
  const data = [...int32ToLittleEndianBytes(toRpm(right)), ...int32ToLittleEndianBytes(toRpm(left))];
  const nowMs = Date.now();

  canTopic.publish(
    new ROSLIB.Message({
      header: {
        stamp: { sec: Math.floor(nowMs / 1000), nanosec: (nowMs % 1000) * 1e6 },
        frame_id: '',
      },
      id: CAN_RPM_ID,
      is_rtr: false,
      is_extended: false,
      is_error: false,
      dlc: 8,
      data,
    })
  );
}

setInterval(publishVehicleInfoCan, 1000 / CAN_PUBLISH_HZ);

// ---------------------------------------------------------------------------
// Physics + render loop
// ---------------------------------------------------------------------------
const physics = new VehiclePhysics();
const speedVal = document.getElementById('speed-val');
const yawRateVal = document.getElementById('yaw-rate-val');
const imuAxVal = document.getElementById('imu-ax');
const imuAyVal = document.getElementById('imu-ay');
const imuAzVal = document.getElementById('imu-az');
const imuGzVal = document.getElementById('imu-gz');
const imuYawVal = document.getElementById('imu-yaw');
const odomXVal = document.getElementById('odom-x');
const odomYVal = document.getElementById('odom-y');
const odomYawVal = document.getElementById('odom-yaw');
const odomVxVal = document.getElementById('odom-vx');
const odomVyVal = document.getElementById('odom-vy');
const odomWzVal = document.getElementById('odom-wz');
const canRpmRVal = document.getElementById('can-rpm-r');
const canRpmLVal = document.getElementById('can-rpm-l');
const canTargetRpmRVal = document.getElementById('can-target-rpm-r');
const canTargetRpmLVal = document.getElementById('can-target-rpm-l');

// Odometry trail: the gyro_odometry_publisher's estimated ground track,
// drawn on the ground plane. In this noise-free sim it coincides exactly
// with the vehicle's true path (see publishOdometry() above, which reads
// physics.x/y/yaw directly) -- there's no wheel-slip or gyro-drift model --
// but the same drawing code would show real drift if that's ever added.
const ODOM_TRAIL_MAX_POINTS = 300;
const ODOM_TRAIL_SAMPLE_INTERVAL = 0.15; // [s] between recorded points
const odomTrailPoints = [];
const odomTrailGeometry = new THREE.BufferGeometry();
const odomTrailLine = new THREE.Line(odomTrailGeometry, new THREE.LineBasicMaterial({ color: 0x35d0ff }));
// geometry.setFromPoints() (used below to grow/slide the trail) only
// rewrites the position attribute -- it never touches geometry.boundingSphere,
// which Three.js computes once (lazily, from whichever points happened to
// exist at the first frustum-culling check) and then never recomputes. As
// the trail grows/moves away from that stale sphere, the line gets
// incorrectly frustum-culled depending on camera angle/distance, making it
// flicker in and out of view. Disabling frustum culling for this object
// sidesteps the stale-bounds check entirely.
odomTrailLine.frustumCulled = false;
scene.add(odomTrailLine);
let odomTrailAccumulator = 0;

// Camera stays a fixed distance behind/above the vehicle, expressed in ROS
// (x-forward, z-up) space, then converted to Three.js space the same way
// rosRoot converts every model transform (threeX=x, threeY=z, threeZ=-y).
const CAM_BACK = 3.5;
const CAM_HEIGHT = 2.2;
function rosToThree(x, y, z) {
  return new THREE.Vector3(x, z, -y);
}

// Default "behind and above" chase position, in current vehicle-relative
// world coordinates. Used to seed the camera on load and by the view-reset
// button; ongoing frame-to-frame following is handled in animate() below by
// translating camera.position with the vehicle instead of re-snapping here,
// so it doesn't fight the user's manual orbit/zoom.
function chaseCameraPosition() {
  return rosToThree(
    physics.x - CAM_BACK * Math.cos(physics.yaw),
    physics.y - CAM_BACK * Math.sin(physics.yaw),
    CAM_HEIGHT
  );
}

const followTarget = new THREE.Vector3();
let cameraInitialized = false;

// --- View controls: rotate (auto-orbit toggle) / reset ---
const rotateViewBtn = document.getElementById('rotate-view-btn');
const resetViewBtn = document.getElementById('reset-view-btn');

rotateViewBtn.addEventListener('click', () => {
  controls.autoRotate = !controls.autoRotate;
  rotateViewBtn.classList.toggle('active', controls.autoRotate);
});

resetViewBtn.addEventListener('click', () => {
  camera.position.copy(chaseCameraPosition());
  followTarget.copy(rosToThree(physics.x, physics.y, 0.3));
  controls.target.copy(followTarget);
  controls.update();
});

const clock = new THREE.Clock();

function animate() {
  requestAnimationFrame(animate);
  const dt = Math.min(clock.getDelta(), 0.1);

  // twist_mux-style arbitration (js/twist_mux.js): refresh "gamepad"
  // freshness whenever the human is actually driving right now (same
  // effective-keys signal the existing cmd_vel-timeout fallback already
  // uses), then let the highest-priority still-fresh source drive the
  // vehicle. gamepad (150) always wins over mpc (50) the instant a key is
  // held; letting go hands control back to autonomous once gamepad's own
  // 0.3s timeout elapses -- exactly like the real vehicle's twist_mux.
  const effectiveKeys = isCmdVelTimedOut() ? NO_KEYS : keys;
  if (effectiveKeys.forward || effectiveKeys.backward || effectiveKeys.left || effectiveKeys.right) {
    twistMux.update('gamepad', physics.v, physics.omega, performance.now());
  }
  const { activeSource } = twistMux.mux(performance.now());

  if (activeSource === 'mpc') {
    physics.stepAutonomous(latestAutonomousCmd.v, latestAutonomousCmd.omega, dt);
  } else {
    physics.step(effectiveKeys, dt);
  }

  vehicleRoot.position.set(physics.x, physics.y, 0);
  vehicleRoot.rotation.set(0, 0, physics.yaw, 'ZYX'); // pure yaw; order is a no-op here but kept consistent with setPose

  const { left, right } = physics.wheelSpeeds();
  wheelLeftSpin.rotation.x += (left / VEHICLE.wheelRadius) * dt;
  wheelRightSpin.rotation.x += (right / VEHICLE.wheelRadius) * dt;
  casterSpin.rotation.x += (physics.v / CASTER_JOINT.z) * dt; // CASTER_JOINT.z == CASTER_RADIUS

  // The orbit target always tracks the vehicle. camera.position is only
  // ever translated by the vehicle's own movement (not re-snapped to a
  // fixed offset) so free orbiting via mouse drag / autoRotate / the view
  // buttons isn't fought every frame -- whatever relative angle/distance
  // the user has set is preserved while still following the car around.
  const targetThree = rosToThree(physics.x, physics.y, 0.3);

  if (!cameraInitialized) {
    camera.position.copy(chaseCameraPosition());
    followTarget.copy(targetThree);
    controls.target.copy(followTarget);
    cameraInitialized = true;
  } else {
    const smoothing = 1 - Math.pow(0.001, dt);
    const previousTarget = followTarget.clone();
    followTarget.lerp(targetThree, smoothing);
    camera.position.add(followTarget.clone().sub(previousTarget));
    controls.target.copy(followTarget);
  }

  controls.update();

  // Onboard (ZED mount) camera: rigidly attached to the vehicle.
  // Pitch down angle is 1.8 deg, matching real extrinsic.yaml orientation (r: -88.2 -> ~1.8 deg tilt).
  const PITCH_DOWN_RAD = (1.8 * Math.PI) / 180;
  const LOOK_DIST = 5.0;
  const mountRos = {
    x: physics.x + CAMERA_MOUNT.x * Math.cos(physics.yaw) - CAMERA_MOUNT.y * Math.sin(physics.yaw),
    y: physics.y + CAMERA_MOUNT.x * Math.sin(physics.yaw) + CAMERA_MOUNT.y * Math.cos(physics.yaw),
    z: VEHICLE.wheelRadius + CAMERA_MOUNT.z, // 0.12m (base_link height) + 0.44m = 0.56m above ground
  };
  const lookAheadRos = {
    x: mountRos.x + Math.cos(physics.yaw) * LOOK_DIST,
    y: mountRos.y + Math.sin(physics.yaw) * LOOK_DIST,
    z: mountRos.z - LOOK_DIST * Math.tan(PITCH_DOWN_RAD),
  };
  onboardCamera.position.copy(rosToThree(mountRos.x, mountRos.y, mountRos.z));
  onboardCamera.lookAt(rosToThree(lookAheadRos.x, lookAheadRos.y, lookAheadRos.z));

  publishAccumulator += dt;
  if (publishAccumulator >= PUBLISH_INTERVAL) {
    publishAccumulator = 0;
    publishCmdVel(physics.v, physics.omega);
    publishMuxedCmdVel(physics.v, physics.omega);
  }

  imagePublishAccumulator += dt;
  if (imagePublishAccumulator >= IMAGE_PUBLISH_INTERVAL) {
    imagePublishAccumulator = 0;
    renderOnboardCapture();
    publishCompressedImage();
    updateLanePipeline(); // async, single-slot-buffered; fire-and-forget
  }

  // publishImu() itself runs on a separate setInterval (see below), not here.

  // Lateral (centripetal) acceleration component, still shown in the IMU HUD panel.
  const imuLateralAccel = physics.v * physics.omega;

  // Odometry ground-track trail (see publishOdometry()/odom-panel setup for
  // why this coincides with the vehicle's true path in this simulator).
  odomTrailAccumulator += dt;
  if (odomTrailAccumulator >= ODOM_TRAIL_SAMPLE_INTERVAL) {
    odomTrailAccumulator = 0;
    odomTrailPoints.push(rosToThree(physics.x, physics.y, 0.05));
    if (odomTrailPoints.length > ODOM_TRAIL_MAX_POINTS) odomTrailPoints.shift();
    odomTrailGeometry.setFromPoints(odomTrailPoints);
  }

  const muxNow = performance.now();
  twistMuxGamepadStateEl.textContent = twistMux.isFresh('gamepad', muxNow) ? '有効' : '-';
  twistMuxMpcStateEl.textContent = twistMux.isFresh('mpc', muxNow) ? '有効' : '-';
  twistMuxActiveEl.textContent = activeSource === 'gamepad' ? 'gamepad (WASD)' : activeSource === 'mpc' ? 'mpc (自動運転)' : 'なし';

  speedVal.textContent = physics.v.toFixed(2);
  yawRateVal.textContent = physics.omega.toFixed(2);
  imuAxVal.textContent = physics.linearAccel.toFixed(2);
  imuAyVal.textContent = imuLateralAccel.toFixed(2);
  imuAzVal.textContent = GRAVITY.toFixed(2);
  imuGzVal.textContent = physics.omega.toFixed(2);
  imuYawVal.textContent = `${THREE.MathUtils.radToDeg(physics.yaw).toFixed(1)}°`;

  const odomYawDeg = THREE.MathUtils.radToDeg(physics.yaw).toFixed(1);
  odomXVal.textContent = `${physics.x.toFixed(2)} m`;
  odomYVal.textContent = `${physics.y.toFixed(2)} m`;
  odomYawVal.textContent = `${odomYawDeg}°`;
  odomVxVal.textContent = `${(physics.v * Math.cos(physics.yaw)).toFixed(2)} m/s`;
  odomVyVal.textContent = `${(physics.v * Math.sin(physics.yaw)).toFixed(2)} m/s`;
  odomWzVal.textContent = `${physics.omega.toFixed(2)} rad/s`;

  const wheelCircumference = CAN_WHEEL_DIAMETER * Math.PI;
  const toRpm = (speedMps) => (speedMps / wheelCircumference) * 60;
  if (canRpmRVal) canRpmRVal.textContent = `${Math.round(toRpm(right))} rpm`;
  if (canRpmLVal) canRpmLVal.textContent = `${Math.round(toRpm(left))} rpm`;

  const activeKeys = isCmdVelTimedOut() ? NO_KEYS : keys;
  const targetSpeeds = physics.targetWheelSpeeds(activeKeys);
  if (canTargetRpmRVal) canTargetRpmRVal.textContent = `${Math.round(toRpm(targetSpeeds.right))} rpm`;
  if (canTargetRpmLVal) canTargetRpmLVal.textContent = `${Math.round(toRpm(targetSpeeds.left))} rpm`;

  render();
}

function render() {
  const w = window.innerWidth;
  const h = window.innerHeight;

  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, w, h);
  renderer.render(scene, camera);

  // Picture-in-picture onboard view, positioned to match #pip-frame exactly.
  const rect = pipFrame.getBoundingClientRect();
  const pipX = rect.left;
  const pipY = h - rect.bottom; // Three.js viewport origin is bottom-left
  const pipW = rect.width;
  const pipH = rect.height;

  onboardCamera.aspect = pipW / pipH;
  onboardCamera.updateProjectionMatrix();

  renderer.setScissorTest(true);
  renderer.setScissor(pipX, pipY, pipW, pipH);
  renderer.setViewport(pipX, pipY, pipW, pipH);
  renderer.render(scene, onboardCamera);
  renderer.setScissorTest(false);
}

animate();
