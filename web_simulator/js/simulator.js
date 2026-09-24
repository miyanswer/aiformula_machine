import * as THREE from 'three';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { VehiclePhysics, VEHICLE, MAX_SPEED, MAX_ANGULAR } from './vehicle_physics.js';
import { loadCourse } from './course.js';
import { resolveCollisions, VEHICLE_COLLIDERS, PathTracker, DepartureMonitor } from './collision.js';
import { addMyLapsGantry, MYLAPS_COLLIDERS, mylapsPoseOnPath, worldColliders, setSignalLight } from './course_props.js';
import { createConeEditor } from './cone_editor.js';
import { loadConeTemplate, addCone, coneWorldColliders } from './cone_props.js';
import { TwistMux } from './twist_mux.js';
import { UfldLaneDetector } from './ufld_lane_detector.js';
import { IdealLaneDetector } from './ideal_lane_detector.js';
import { ModelLaneDetector } from './lane_model_detector.js';
import {
  DEFAULT_CAMERA, projectToGround, fitLine, LineTracker, LINE_TRACKER_PARAMS, LaneNavigator,
  NAVIGATOR_PARAMS, RACELINE_PARAMS, TRACKER_PARAMS, MAPPING, RACING, ROLES, extractMaskLines,
} from './lane_navigator.js';
import { ConeDetector } from './cone_detector.js';
import { reactiveAvoid, ConeRecorder, applyRacelineDeflection, coneLandmarkCorrection } from './cone_avoidance.js';
import { SixLanePlanner, LanePolicyNet, SIX_LANE_PARAMS, N_LANES, laneY, explainJa } from './six_lane_planner.js';
import { TrafficLightDetector } from './traffic_light_detector.js';
import { TrafficLightStop, TL_STATE_JA } from './traffic_light_stop.js';

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
const CASTER_JOINT = { x: -VEHICLE.wheelbase, y: 0.0, z: 0.10, roll: Math.PI / 2, pitch: 0, yaw: Math.PI / 2 };
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
// course.glb's asphalt slab occupies z = -0.1 .. 0 (its top face is the z = 0
// ground plane the vehicle drives on), so the backdrop sits below it: at z = 0
// it would z-fight with the slab's top face.
ground.position.y = -0.11;
scene.add(ground);

const grid = new THREE.GridHelper(200, 200, 0x4a5058, 0x2a2e34);
grid.position.y = -0.105;
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

// The course: models/course.glb (js/course.js). It is placed so that the START
// of the centre white line is the odom origin (0, 0) with +x along the line --
// which is where VehiclePhysics starts the vehicle (0, 0, 0). So the vehicle's
// initial position, the odometry's (0, 0) and the start of the centre line are
// all the same point.
const course = await loadCourse(rosRoot);

// MyLaps timing gantry on the centre line (js/course_props.js). Its posts and
// cones are solid -- the vehicle cannot pass between the 0.46m posts, which is
// intentional: it is the obstacle the planned YOLO cone detector will have to
// steer around.
const mylapsPose = mylapsPoseOnPath(course.centerPath);
const mylapsRoot = addMyLapsGantry(rosRoot, setPose, mylapsPose);
const obstacles = worldColliders(mylapsPose, MYLAPS_COLLIDERS);

// Free-placement cones (js/cone_editor.js): click to add, drag to move,
// click-without-drag to remove, persisted to localStorage. The 3D instances
// are rebuilt from scratch on every change (rebuildConeInstances) and their
// circular colliders are merged into applyCollisionAndDeparture()'s obstacle
// list below, alongside the MyLaps gantry's.
const coneTemplate = await loadConeTemplate();
let coneInstances = []; // three.jsオブジェクト、コーン変更のたびに作り直す
let coneColliders = []; // collision.jsに渡す円コライダー

function rebuildConeInstances(cones) {
  for (const inst of coneInstances) rosRoot.remove(inst);
  coneInstances = cones.map((c) => addCone(rosRoot, coneTemplate, c));
  coneColliders = coneWorldColliders(cones);
}

const coneEditor = createConeEditor({
  raycastTarget: ground, // simulator.js既存の200x200背景プレーン(104行目付近)、sceneの直接の子
  rosRoot, // ground自体はrosRootの子ではないので、ヒット点をROS座標に戻すために渡す
  camera,
  domElement: renderer.domElement,
  onChange: rebuildConeInstances,
  orbitControls: controls,
});
rebuildConeInstances(coneEditor.cones); // ページ読み込み時、保存済みコーンを復元

// Set by applyCollisionAndDeparture() whenever the vehicle overlaps an
// obstacle at the most recent physics step, and cleared otherwise -- read by
// Task 9's HUD indicator, so it must never latch.
let contactActive = false;

// Course departure: reported, never blocked. The vehicle can leave the
// track and drive back on; the HUD counts each excursion once. Both track
// state against the corrected pose (after obstacle resolution) and are
// updated from the same call sites as the obstacle correction itself --
// see applyCollisionAndDeparture() below.
const pathTracker = new PathTracker(course.centerPath);
const departureMonitor = new DepartureMonitor(course);
const departureStateEl = document.getElementById('departure-state');
const departureCountEl = document.getElementById('departure-count');
const contactStateEl = document.getElementById('contact-state');

// Last HUD values actually written to the DOM. applyCollisionAndDeparture()
// runs once per physics step -- including inside fastForward()'s tight
// while loop, which can call it thousands of times per invocation -- so
// writes are skipped unless the displayed value changed. null forces the
// first call to sync the DOM to the elements' static HTML defaults.
let hudOutside = null;
let hudCount = null;
let hudContact = null;

// Writes the departure/contact HUD indicators and syncs the write-guard
// cache above, so applyCollisionAndDeparture() and resetNavigation() (the
// two sites that touch this HUD) can't drift apart on the literal
// strings/colours.
function writeCollisionHud(outside, count, contact) {
  hudOutside = outside;
  hudCount = count;
  hudContact = contact;
  departureStateEl.textContent = outside ? '逸脱中' : 'コース内';
  departureStateEl.style.color = outside ? '#ff6b6b' : '#8fd18f';
  departureCountEl.textContent = String(count);
  contactStateEl.style.display = contact ? 'inline' : 'none';
}

// Pushes the vehicle back out of anything it overlaps, applied after
// integration as a position correction so VehiclePhysics itself stays a
// clean kinematic model. Sliding falls out of the correction (only the
// component along the contact normal is cancelled); a near head-on contact
// additionally kills forward speed. Then checks course departure against
// that same corrected pose (Task 9), so a vehicle pushed out of an obstacle
// is evaluated at the position it actually ends up at -- folded into this
// one function, rather than a second one, so both call sites below stay in
// lockstep; a departure check wired only into animate() would make a
// fast-forwarded lap silently stop counting excursions.
// Called once per physics step -- from animate() after its step()/
// stepAutonomous() branch, and from fastForward()'s own physics loop after
// its stepAutonomous() call -- so a fast-forwarded lap collides with the
// gantry and is checked for departures exactly like a real-time one.
// Gated on mylapsRoot.userData.collisionDisabled, which nothing sets by
// default (undefined is falsy, so collision starts enabled); it exists so
// live verification can disable it to drive a clean lap.
function applyCollisionAndDeparture() {
  if (mylapsRoot.userData.collisionDisabled) {
    contactActive = false;
  } else {
    const hit = resolveCollisions(physics, [...obstacles, ...coneColliders], VEHICLE_COLLIDERS);
    if (hit.maxPenetration > 0) {
      physics.x += hit.dx;
      physics.y += hit.dy;
      if (hit.headOn && physics.v > 0) physics.v = 0;
    }
    contactActive = hit.maxPenetration > 0;
  }

  const { offset } = pathTracker.update(physics.x, physics.y);
  const departure = departureMonitor.update(offset);

  if (departure.outside !== hudOutside || departure.count !== hudCount || contactActive !== hudContact) {
    writeCollisionHud(departure.outside, departure.count, contactActive);
  }
}

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

// AIF_body.dae has an authoring-time point light and camera baked in by the
// exporter, and ColladaLoader adds both to the scene graph along with the
// meshes. The light ends up parented to the vehicle, so it travels with it,
// and its attenuation term evaluates absurdly high: measured against the
// MyLaps gantry ~5m ahead, intensity 1 and intensity 0.01 both render it a
// flat #ffffff, while intensity 0 gives the expected shading (black frame
// rgb(0,0,0), orange cone rgb(151,18,8)). So every *lit* material near the
// car blows out to pure white -- which is why the body renders as a
// featureless white blob, and why the gantry lost all of its colour. Nothing
// here wants the exporter's lights or cameras (the scene lights itself), so
// drop them on load.
function stripSceneExtras(object3d) {
  const extras = [];
  object3d.traverse((obj) => {
    if (obj.isLight || obj.isCamera) extras.push(obj);
  });
  extras.forEach((obj) => obj.removeFromParent());
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
      stripSceneExtras(collada.scene);
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

// キーを押したまま別タブ/別ウィンドウに移ると keyup が届かず、押しっぱなし
// 扱いのまま残る (実機操縦中なら前進指令を送り続ける)。フォーカスを失った
// 時点で全キーを離した扱いにし、実機操縦中は即座に停止指令を送る
// (非表示タブでは requestAnimationFrame が間引かれ、通常の送信ループを
// 待つと停止が遅れるため)。
function releaseAllKeys() {
  keys.forward = keys.backward = keys.left = keys.right = false;
  Object.values(keyEls).forEach((el) => el.classList.remove('active'));
  if (teleopOnly && cmdVelTopic) {
    publishCmdVel(0, 0);
    teleopStopRemaining = TELEOP_STOP_REPEAT;
  }
}
window.addEventListener('blur', releaseAllKeys);
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState !== 'hidden') return;
  releaseAllKeys();
  // 非表示タブでは描画 (=シミュレータの車) が止まり、自律走行指令も古い画面から
  // しか計算されないため、実機への mpc 指令も止める (publishAutonomousCmd 側でも抑止)。
  if (autonomousMode) publishAutonomousStop();
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
let laneLeftTopic = null;
let laneRightTopic = null;
let laneCenterTopic = null;
let laneDetectorAnnotatedImageTopic = null;
let targetTrajectoryTopic = null;
let leftBoundaryTopic = null;
let rightBoundaryTopic = null;
let autonomousCmdVelTopic = null;
let laneTrackerStatusTopic = null;
let muxedCmdVelTopic = null;
let sixLaneStatusTopic = null;
let sixLaneTargetPathTopic = null;
let sixLaneReseedTopic = null;
let rosSixLaneStatusSub = null;
let redDistanceTopic = null;
let conesTopic = null;
let greenDistanceTopic = null;
let trafficLightStopStatusTopic = null;
let rosTrafficLightStopStatusSub = null;
// "ROS2連携" detector mode subscriptions: when selected, UFLD inference +
// the lap-mapping/QP navigator run on the ROS 2 side (see
// src/oit_navigation/launch/simulator_test.launch.py) instead of in the
// browser, and the simulator just displays what it publishes back + drives
// the vehicle from its cmd_vel -- see setDetectorMode('ros2') below.
let rosLaneDetectorAnnotatedImageSub = null;
let rosAutonomousCmdVelSub = null;
let rosLaneTrackerStatusSub = null;
let rosTargetTrajectorySub = null;
let rosLeftBoundarySub = null;
let rosRightBoundarySub = null;
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
// oit_navigation pipeline: UFLD white-line detection -> left/center/right
// line tracking -> lap 1 center-line tracking + boundary recording -> QP
// min-curvature raceline -> lap 2+ raceline following, ported to run
// client-side (js/lane_model_detector.js (YOLOP), js/ufld_lane_detector.js,
// js/lane_navigator.js). Topic names
// below are copied verbatim from src/oit_navigation/config/navigation_params.yaml
// (lane_detector / lane_navigator node parameters), so this
// simulator's output is a drop-in match for the real vehicle's stack.
const ROBOT_FRAME_ID = 'base_link';
const ODOM_NAV_FRAME_ID = 'odom';
const LANE_DETECTOR_ANNOTATED_IMAGE_TOPIC = '/aiformula_visualization/lane_detector/annotated_image';
const LANE_LINE_LEFT_TOPIC = '/aiformula_perception/lane_line_publisher/lane_lines/left';
const LANE_LINE_RIGHT_TOPIC = '/aiformula_perception/lane_line_publisher/lane_lines/right';
const LANE_LINE_CENTER_TOPIC = '/aiformula_perception/lane_line_publisher/lane_lines/center';
const TARGET_TRAJECTORY_TOPIC = '/aiformula_visualization/target_trajectory';
const LEFT_BOUNDARY_TOPIC = '/aiformula_visualization/lane_navigator/left_boundary';
const RIGHT_BOUNDARY_TOPIC = '/aiformula_visualization/lane_navigator/right_boundary';
// "mpc" input of twist_mux (see launchers/sample_launchers/launch/twist_mux.launch.py) --
// the real vehicle's lane_navigator output topic (unchanged from the old
// Pure Pursuit node, so twist_mux needs no change). This simulator always
// publishes it once the pipeline is running (mirroring how the real node
// always publishes regardless of twist_mux arbitration); whether the
// simulated vehicle itself obeys it is decided by the twistMux instance
// below (js/twist_mux.js), gated by the "自動運転" HUD toggle.
const AUTONOMOUS_CMD_VEL_TOPIC = '/aiformula_control/extremum_seeking_mpc/cmd_vel';
// lane_navigator status (std_msgs/String, JSON -- LaneNavigator.status()).
const LANE_TRACKER_STATUS_TOPIC = '/aiformula_control/lane_tracker/status';
// 6レーン走行 (six_lane_planner ノード, src/oit_navigation/oit_navigation/6lane/) の
// 状態 JSON と目標経路。指令は lane_navigator と同じ twist_mux "mpc" 入力
// (AUTONOMOUS_CMD_VEL_TOPIC) に出す -- 実機でもどちらか一方だけを起動する。
const SIX_LANE_STATUS_TOPIC = '/aiformula_control/six_lane_planner/status';
const SIX_LANE_TARGET_PATH_TOPIC = '/aiformula_visualization/six_lane_planner/target_path';
// 6レーン走行が白線の役割取り違えを検出したときの横位置 (レーン座標 F, std_msgs/Float64)。
// lane_detector が購読して LineTracker を置き直す (tracker.seed_lane_position)。
const SIX_LANE_RESEED_TOPIC = '/aiformula_control/six_lane_planner/lane_reseed';
// 実機操縦モードの cmd_vel 送信。実機のゲームパッド (teleop_twist_joy: 有効
// ボタンを押している間だけ publish し、離すとゼロを送る) と同じ振る舞いにする:
// キーを押している間だけ現在の速度を送り、離したらゼロを数回送って送信を止める。
// 以降は実機の twist_mux が gamepad 入力をタイムアウト (0.3s) させ、下位の
// 入力 (自律走行など) に制御が戻る。シミュレータの車も同時に停止させて、
// 画面上の車と実機への指令を一致させる (惰性で走り続ける速度を送らない)。
const TELEOP_STOP_REPEAT = 3;
let teleopStopRemaining = 0;

function publishTeleopCmdVel() {
  if (!cmdVelTopic) return;
  if (keys.forward || keys.backward || keys.left || keys.right) {
    publishCmdVel(physics.v, physics.omega);
    teleopStopRemaining = TELEOP_STOP_REPEAT;
    return;
  }
  if (!autonomousMode) {
    physics.v = 0;
    physics.omega = 0;
  }
  if (teleopStopRemaining > 0) {
    publishCmdVel(0, 0);
    teleopStopRemaining -= 1;
  } else {
    // 送信停止中はローカルの cmd_vel タイムアウト (isCmdVelTimedOut) を
    // 解除しておく。残したままだと次のキー入力が NO_KEYS 扱いで無視され、
    // 二度と publish されなくなる。
    lastCmdVelPublishTime = null;
  }
}

// twist_mux's arbitrated output (topic_list.yaml control.speed_command.multiplexed).
const MUXED_CMD_VEL_TOPIC = '/aiformula_control/twist_mux/cmd_vel';
// 信号機: traffic_light_distance_node (traffic_light_params.yaml base_topic) の出力と、
// 赤信号停止 (utils/traffic_light_stop_ros.py, lane_navigator / six_lane_planner 共通) の状態。
// ブラウザ内の検出モードではシミュレータ自身が同じトピックを出す。
const RED_DISTANCE_TOPIC = '/aiformula_perception/traffic_light/red_distance';
const GREEN_DISTANCE_TOPIC = '/aiformula_perception/traffic_light/green_distance';
const TRAFFIC_LIGHT_STOP_STATUS_TOPIC = '/aiformula_control/traffic_light_stop/status';
// 実機の cone_detector ノード (oit_navigation/cone_detector_node.py) と同じトピック: 検出コーンの位置 (base_link)。
// ブラウザ内の検出モードではシミュレータの cone_detector.js の結果を出す (RViz で確認できるように)。
const CONES_TOPIC = '/aiformula_perception/cone_detector/cones';

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

// Navigator parameters = navigation_params.yaml defaults, except speed /
// angular limits: per instruction, those stay the simulator's own WASD
// limits (MAX_SPEED=1.5, MAX_ANGULAR=1.2) rather than the real vehicle's.
// lane_width: this course's center line <-> boundary line distance, measured
// from course.glb by js/course.js (3.5 m).
const SIM_LANE_WIDTH = course.laneWidthM;
const SIM_NAVIGATOR_PARAMS = {
  ...NAVIGATOR_PARAMS,
  raceline: { ...RACELINE_PARAMS, vMax: MAX_SPEED },
  tracker: { ...TRACKER_PARAMS, maxAngularSpeed: MAX_ANGULAR },
};
const LANE_DATA_TIMEOUT_MS = 800; // navigation_params.yaml lines_timeout
// Start pose: the start of the centre white line, which course.js made the odom
// origin, facing along the line (+x). The lap-1 method drives on top of the
// centre line, so it begins here.
const SIM_START_POSE = { x: 0, y: 0, yaw: 0 };

const urlInput = document.getElementById('ros-url');
const topicInput = document.getElementById('ros-topic');
const connectBtn = document.getElementById('connect-btn');
const teleopOnlyInput = document.getElementById('ros-teleop-only');
const statusDot = document.getElementById('status-dot');

// 実機操縦モード: rosbridge の接続先が実機 (Jetson) のとき、キー操作中の
// cmd_vel (gamepad 入力) *だけ* を送る。通常の接続はシミュレータ用に
// カメラ画像 (JPEG + 無圧縮RGBの注釈画像)・IMU・オドメトリ・CAN車輪速・
// twist_mux出力までを実機と同じトピック名で publish するため、実機に
// 繋ぐと (1) 実センサのトピックに偽データが混ざり、(2) twist_mux 出力
// (= motor_controller 入力) を直接上書きしてしまい、(3) 画像で rosbridge
// が飽和して cmd_vel が数百ms遅延する。接続先 URL が localhost 以外なら
// 既定でオン (接続前ならいつでも手動で切替可)。
let teleopOnly = false;
function isLocalRosUrl(url) {
  try {
    return ['localhost', '127.0.0.1', '[::1]', '::1'].includes(new URL(url).hostname);
  } catch (err) {
    return true;
  }
}
function syncTeleopOnlyDefault() {
  if (!ros) teleopOnlyInput.checked = !isLocalRosUrl(urlInput.value);
}
urlInput.addEventListener('input', syncTeleopOnlyDefault);
syncTeleopOnlyDefault();
const statusText = document.getElementById('status-text');

function setStatus(state, label) {
  statusDot.className = state;
  statusText.textContent = label;
}

function clearRosTopics() {
  teleopOnlyInput.disabled = false;
  cmdVelTopic = null;
  compressedImageTopic = null;
  imuTopic = null;
  odomTopic = null;
  canTopic = null;
  laneLeftTopic = null;
  laneRightTopic = null;
  laneCenterTopic = null;
  laneDetectorAnnotatedImageTopic = null;
  targetTrajectoryTopic = null;
  leftBoundaryTopic = null;
  rightBoundaryTopic = null;
  autonomousCmdVelTopic = null;
  laneTrackerStatusTopic = null;
  muxedCmdVelTopic = null;
  sixLaneStatusTopic = null;
  sixLaneTargetPathTopic = null;
  sixLaneReseedTopic = null;
  rosSixLaneStatusSub = null;
  redDistanceTopic = null;
  conesTopic = null;
  greenDistanceTopic = null;
  trafficLightStopStatusTopic = null;
  rosTrafficLightStopStatusSub = null;
  rosLaneDetectorAnnotatedImageSub = null;
  rosAutonomousCmdVelSub = null;
  rosLaneTrackerStatusSub = null;
  rosTargetTrajectorySub = null;
  rosLeftBoundarySub = null;
  rosRightBoundarySub = null;
}

function disconnect() {
  if (ros) ros.close();
  ros = null;
  clearRosTopics();
  connectBtn.textContent = '接続';
  setStatus('', '未接続');
}

function connect() {
  setStatus('connecting', '接続中...');
  connectBtn.disabled = true;

  ros = new ROSLIB.Ros({ url: urlInput.value });
  teleopOnly = teleopOnlyInput.checked;
  teleopOnlyInput.disabled = true;

  ros.on('connection', () => {
    setStatus('connected', teleopOnly ? '接続済み (実機操縦)' : '接続済み');
    connectBtn.disabled = false;
    connectBtn.textContent = '切断';
    cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: topicInput.value,
      messageType: 'geometry_msgs/msg/Twist',
    });
    // 実機操縦: gamepad cmd_vel と、自動運転 ON 時の extremum_seeking_mpc/cmd_vel
    // 以外は一切 publish / subscribe しない (上のコメント参照)。他の *Topic は
    // null のままなので各 publish 関数は no-op になる。
    if (teleopOnly) {
      autonomousCmdVelTopic = new ROSLIB.Topic({ ros, name: AUTONOMOUS_CMD_VEL_TOPIC, messageType: 'geometry_msgs/msg/Twist' });
      return;
    }
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
    laneDetectorAnnotatedImageTopic = new ROSLIB.Topic({ ros, name: LANE_DETECTOR_ANNOTATED_IMAGE_TOPIC, messageType: 'sensor_msgs/msg/Image' });
    laneLeftTopic = new ROSLIB.Topic({ ros, name: LANE_LINE_LEFT_TOPIC, messageType: 'nav_msgs/msg/Path' });
    laneRightTopic = new ROSLIB.Topic({ ros, name: LANE_LINE_RIGHT_TOPIC, messageType: 'nav_msgs/msg/Path' });
    laneCenterTopic = new ROSLIB.Topic({ ros, name: LANE_LINE_CENTER_TOPIC, messageType: 'nav_msgs/msg/Path' });
    targetTrajectoryTopic = new ROSLIB.Topic({ ros, name: TARGET_TRAJECTORY_TOPIC, messageType: 'nav_msgs/msg/Path' });
    leftBoundaryTopic = new ROSLIB.Topic({ ros, name: LEFT_BOUNDARY_TOPIC, messageType: 'nav_msgs/msg/Path' });
    rightBoundaryTopic = new ROSLIB.Topic({ ros, name: RIGHT_BOUNDARY_TOPIC, messageType: 'nav_msgs/msg/Path' });
    autonomousCmdVelTopic = new ROSLIB.Topic({ ros, name: AUTONOMOUS_CMD_VEL_TOPIC, messageType: 'geometry_msgs/msg/Twist' });
    laneTrackerStatusTopic = new ROSLIB.Topic({ ros, name: LANE_TRACKER_STATUS_TOPIC, messageType: 'std_msgs/msg/String' });
    muxedCmdVelTopic = new ROSLIB.Topic({ ros, name: MUXED_CMD_VEL_TOPIC, messageType: 'geometry_msgs/msg/Twist' });
    sixLaneStatusTopic = new ROSLIB.Topic({ ros, name: SIX_LANE_STATUS_TOPIC, messageType: 'std_msgs/msg/String' });
    sixLaneTargetPathTopic = new ROSLIB.Topic({ ros, name: SIX_LANE_TARGET_PATH_TOPIC, messageType: 'nav_msgs/msg/Path' });
    sixLaneReseedTopic = new ROSLIB.Topic({ ros, name: SIX_LANE_RESEED_TOPIC, messageType: 'std_msgs/msg/Float64' });
    rosSixLaneStatusSub = new ROSLIB.Topic({ ros, name: SIX_LANE_STATUS_TOPIC, messageType: 'std_msgs/msg/String' });
    rosSixLaneStatusSub.subscribe(onRosSixLaneStatus);
    redDistanceTopic = new ROSLIB.Topic({ ros, name: RED_DISTANCE_TOPIC, messageType: 'std_msgs/msg/Float32' });
    conesTopic = new ROSLIB.Topic({ ros, name: CONES_TOPIC, messageType: 'geometry_msgs/msg/PoseArray' });
    greenDistanceTopic = new ROSLIB.Topic({ ros, name: GREEN_DISTANCE_TOPIC, messageType: 'std_msgs/msg/Float32' });
    trafficLightStopStatusTopic = new ROSLIB.Topic({ ros, name: TRAFFIC_LIGHT_STOP_STATUS_TOPIC, messageType: 'std_msgs/msg/String' });
    rosTrafficLightStopStatusSub = new ROSLIB.Topic({ ros, name: TRAFFIC_LIGHT_STOP_STATUS_TOPIC, messageType: 'std_msgs/msg/String' });
    rosTrafficLightStopStatusSub.subscribe(onRosTrafficLightStopStatus);

    // "ROS2連携" mode subscriptions (see onRos*() callbacks below) -- always
    // subscribed once connected, regardless of the current detectorMode;
    // each callback itself no-ops unless detectorMode === 'ros2'. Separate
    // Topic instances from the publish-side ones above (same topic names)
    // since roslib.js topics are one-directional.
    rosLaneDetectorAnnotatedImageSub = new ROSLIB.Topic({ ros, name: LANE_DETECTOR_ANNOTATED_IMAGE_TOPIC, messageType: 'sensor_msgs/msg/Image' });
    rosLaneDetectorAnnotatedImageSub.subscribe(onRosLaneDetectorAnnotatedImage);
    rosAutonomousCmdVelSub = new ROSLIB.Topic({ ros, name: AUTONOMOUS_CMD_VEL_TOPIC, messageType: 'geometry_msgs/msg/Twist' });
    rosAutonomousCmdVelSub.subscribe(onRosAutonomousCmdVel);
    rosLaneTrackerStatusSub = new ROSLIB.Topic({ ros, name: LANE_TRACKER_STATUS_TOPIC, messageType: 'std_msgs/msg/String' });
    rosLaneTrackerStatusSub.subscribe(onRosLaneTrackerStatus);
    rosTargetTrajectorySub = new ROSLIB.Topic({ ros, name: TARGET_TRAJECTORY_TOPIC, messageType: 'nav_msgs/msg/Path' });
    rosTargetTrajectorySub.subscribe((msg) => onRosMapPath('raceline', msg));
    rosLeftBoundarySub = new ROSLIB.Topic({ ros, name: LEFT_BOUNDARY_TOPIC, messageType: 'nav_msgs/msg/Path' });
    rosLeftBoundarySub.subscribe((msg) => onRosMapPath('left', msg));
    rosRightBoundarySub = new ROSLIB.Topic({ ros, name: RIGHT_BOUNDARY_TOPIC, messageType: 'nav_msgs/msg/Path' });
    rosRightBoundarySub.subscribe((msg) => onRosMapPath('right', msg));
  });

  ros.on('error', () => {
    setStatus('error', 'エラー');
    connectBtn.disabled = false;
    connectBtn.textContent = '接続';
  });

  ros.on('close', () => {
    clearRosTopics();
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
// oit_navigation pipeline wiring: UFLD (browser ONNX) -> ground projection ->
// left/center/right line tracking -> LaneNavigator (lap 1: center-line
// tracking + boundary recording, lap 2+: QP raceline following), rendered
// into the two HUD panels and published on the same topic names the real
// lane_detector / lane_navigator nodes use.
// ---------------------------------------------------------------------------
const ufldDetector = new UfldLaneDetector();
const idealDetector = new IdealLaneDetector(course.lines);
// YOLOP white-line segmentation (same weights as the real vehicle's
// lane_detector backend=yolop), exported by export_onnx_web.py.
const yolopDetector = new ModelLaneDetector();
const YOLOP_ONNX_URL = 'models/honda_shihou_finetuned.onnx';
let yolopLoadPromise = null;
// Generated locally by `ros2 run oit_navigation export_ufld_onnx` (~245MB,
// not committed -- see web_simulator/README.md).
const UFLD_ONNX_URL = 'models/ufld.onnx';
const UFLD_META_URL = 'models/ufld.json';
const MODEL_WASM_DIR = 'vendor/onnxruntime-web/';
let ufldLoadPromise = null;

// コーン検知 (models/cone.onnx, export_cone_onnx.pyで生成) -- 未生成でも
// 白線追従・衝突判定など他機能はそのまま動くよう、読込失敗はHUD表示のみで握りつぶす。
const coneDetector = new ConeDetector();
const CONE_ONNX_URL = 'models/cone.onnx';
let coneLoadPromise = null;
let latestConeDetections = [];
const coneRecorder = new ConeRecorder();
let prevReactiveBias = 0;
let previousNavState = null;
const coneStatusEl = document.getElementById('oit-cone-status');
const avoidanceDebugEl = document.getElementById('avoidance-debug');

function ensureConeDetectorLoading() {
  if (coneDetector.session || coneLoadPromise) return;
  coneLoadPromise = coneDetector.load(CONE_ONNX_URL, MODEL_WASM_DIR)
    .then(() => { coneStatusEl.textContent = 'コーン検知: 読込済'; })
    .catch((err) => {
      console.warn('ConeDetector load failed (models/cone.onnx missing?)', err);
      coneStatusEl.textContent = 'コーン検知: 読込エラー (models/cone.onnx を生成してください)';
      coneLoadPromise = null;
    });
}
ensureConeDetectorLoading();

// 信号機検知 (models/traffic_light.onnx = models/traffic_light.pt を export_cone_onnx.py で変換)。
// 実機の traffic_light_distance_node と同じく、赤/青信号までの距離をバウンディングボックスの
// 縦の画面占有率から逆算し、TrafficLightStop (実機の utils/traffic_light_stop.py と同一) が
// 走行方式によらず最終 cmd_vel に速度上限を掛ける。未生成でも他機能は動く。
const trafficLightDetector = new TrafficLightDetector();
const TRAFFIC_LIGHT_ONNX_URL = 'models/traffic_light.onnx';
let trafficLightLoadError = null;
const trafficStop = new TrafficLightStop();
let latestTrafficLight = { red: null, green: null, detections: [] };
let rosTrafficLightStopStatus = null;
const trafficLightStatusEl = document.getElementById('oit-traffic-light-status');
trafficLightDetector.load(TRAFFIC_LIGHT_ONNX_URL, MODEL_WASM_DIR).catch((err) => {
  console.warn('TrafficLightDetector load failed (models/traffic_light.onnx missing?)', err);
  trafficLightLoadError = '読込エラー (models/traffic_light.onnx を生成してください)';
});

// コース上の信号 (MyLaps パネルの LED 面)。'cycle' は SIGNAL_PERIOD 秒ごとに赤 <-> 緑、'red' は赤のまま。
// 時計はシミュレーション時間 (animate / fastForward の dt の積算) で進める。
const SIGNAL_PERIOD = 10.0; // [s]
let signalMode = 'cycle';
let signalClock = 0;
function currentSignal() {
  if (signalMode === 'red') return 'red';
  return Math.floor(signalClock / SIGNAL_PERIOD) % 2 === 0 ? 'red' : 'green';
}
function advanceSignal(dt) {
  signalClock += dt;
  const color = currentSignal();
  if (mylapsRoot.userData.signal !== color) setSignalLight(mylapsRoot, color);
}
function setSignalMode(mode) {
  signalMode = mode;
  signalClock = 0;
  document.getElementById('signal-cycle-btn').classList.toggle('active', mode === 'cycle');
  document.getElementById('signal-red-btn').classList.toggle('active', mode === 'red');
  advanceSignal(0);
}
document.getElementById('signal-cycle-btn').addEventListener('click', () => setSignalMode('cycle'));
document.getElementById('signal-red-btn').addEventListener('click', () => setSignalMode('red'));
setSignalLight(mylapsRoot, currentSignal());

function trafficLightStatusText(st) {
  const sig = currentSignal() === 'red' ? '赤' : '緑';
  const left = signalMode === 'cycle' ? ` (切替まで ${(SIGNAL_PERIOD - (signalClock % SIGNAL_PERIOD)).toFixed(0)}秒)` : ' (固定)';
  const head = `コースの信号: ${sig}${left}`;
  if (!st) return `${head} / 検出器 ${trafficLightLoadError ?? (trafficLightDetector.session ? '読込済' : '読込中...')}`;
  const det = [st.red_distance !== null && st.red_count > 0 ? `赤 ${st.red_distance.toFixed(1)}m` : null,
    st.green_distance !== null && st.green_count > 0 ? `青 ${st.green_distance.toFixed(1)}m` : null].filter(Boolean).join(' ');
  return `${head}\n停止制御: ${TL_STATE_JA[st.state] ?? st.state}${det ? ` / 検出 ${det}` : ''}\n${st.reason}`;
}

// 1 フレームの信号機検出 -> TrafficLightStop へ観測を渡し, 実機と同じトピックに距離を出す。
async function runTrafficLightDetection(now) {
  if (detectorMode === 'ros2' || !trafficLightDetector.session) {
    latestTrafficLight = { red: null, green: null, detections: [] };
    return;
  }
  try {
    latestTrafficLight = await trafficLightDetector.infer(captureCanvas);
  } catch (err) {
    console.error('traffic light detector inference error', err);
    latestTrafficLight = { red: null, green: null, detections: [] };
    return;
  }
  trafficStop.observe(now, latestTrafficLight.red, latestTrafficLight.green);
  if (redDistanceTopic && latestTrafficLight.red !== null) redDistanceTopic.publish(new ROSLIB.Message({ data: latestTrafficLight.red }));
  if (greenDistanceTopic && latestTrafficLight.green !== null) greenDistanceTopic.publish(new ROSLIB.Message({ data: latestTrafficLight.green }));
}

// 走行方式の最終指令に赤信号の速度上限を掛ける (実機は lane_navigator / six_lane_planner の中で同じことをする)。
function applyTrafficLightStop(now, dt, vMeas, cmd) {
  const out = trafficStop.apply(now, dt, vMeas, cmd.v, cmd.omega);
  const st = trafficStop.status();
  if (trafficLightStopStatusTopic) trafficLightStopStatusTopic.publish(new ROSLIB.Message({ data: JSON.stringify(st) }));
  return out;
}

// HUD の信号表示 (コースの信号の色・残り秒数 + 停止制御の状態)。ROS2連携では実機ノードの状態を出す。
setInterval(() => {
  if (!trafficLightStatusEl || fastForwarding) return;
  const st = detectorMode === 'ros2' ? rosTrafficLightStopStatus : (trafficLightDetector.session ? trafficStop.status() : null);
  trafficLightStatusEl.textContent = trafficLightStatusText(st);
}, 250);

// "ROS2連携": 実機ノードの停止状態を表示する。
function onRosTrafficLightStopStatus(msg) {
  if (detectorMode !== 'ros2') return;
  try {
    rosTrafficLightStopStatus = JSON.parse(msg.data);
  } catch (err) {
    console.warn('traffic_light_stop status parse error', err);
  }
}

const laneCanvas = document.getElementById('lane-canvas');
const laneCtx = laneCanvas.getContext('2d', { willReadFrequently: true });
const mapCanvas = document.getElementById('map-canvas');
const mapCtx = mapCanvas.getContext('2d');
const detectorStatusEl = document.getElementById('oit-detector-status');
const oitStateEl = document.getElementById('oit-state');
const oitLapEl = document.getElementById('oit-lap');
const oitSamplesEl = document.getElementById('oit-samples');
const oitMessageEl = document.getElementById('oit-message');
const ufldBtn = document.getElementById('detector-ufld-btn');
const yolopBtn = document.getElementById('detector-yolop-btn');
const idealBtn = document.getElementById('detector-ideal-btn');
const ros2Btn = document.getElementById('detector-ros2-btn');
const autonomousBtn = document.getElementById('autonomous-btn');
const finishMappingBtn = document.getElementById('finish-mapping-btn');
const navResetBtn = document.getElementById('nav-reset-btn');
const startPoseBtn = document.getElementById('start-pose-btn');

// 'yolop' | 'ufld' | 'ideal' | 'ros2'. 'yolop' is what the real vehicle
// runs (models/ in git). 'ideal' replaces the detector with the course's true
// white lines + noise/dropouts (js/ideal_lane_detector.js) to verify the
// driving method independently of UFLD's accuracy on rendered images. In
// 'ros2' mode, UFLD + lane_navigator run on the ROS 2 side (src/oit_navigation/launch/simulator_test.launch.py) --
// updateLanePipeline() below becomes a no-op, and the panels + autonomous
// "mpc" cmd instead come from subscribing to what those nodes publish back
// over rosbridge (onRos*() below).
let detectorMode = 'yolop';
let lastRosCmdVelTime = 0;

// ---------------------------------------------------------------------------
// Browser stand-in for the real vehicle's odom_imu_localizer node: dead
// reckoning from wheel speed (CAN, here physics.v) + IMU yaw rate (here
// physics.omega), midpoint integration -- NOT the simulator's ground-truth
// pose, so the navigator sees the same kind of estimate as on the vehicle.
// Its frame starts at the vehicle pose at reset (a frame choice only; the
// map panel then overlays the course naturally).
const localizer = { x: 0, y: 0, yaw: 0, v: 0, omega: 0, s: 0 };
function resetLocalizer() {
  Object.assign(localizer, { x: physics.x, y: physics.y, yaw: physics.yaw, v: 0, omega: 0, s: 0 });
}
function integrateLocalizer(v, omega, dt) {
  const yawMid = localizer.yaw + 0.5 * omega * dt;
  localizer.x += v * Math.cos(yawMid) * dt;
  localizer.y += v * Math.sin(yawMid) * dt;
  localizer.yaw = Math.atan2(Math.sin(localizer.yaw + omega * dt), Math.cos(localizer.yaw + omega * dt));
  localizer.s += Math.abs(v) * dt;
  localizer.v = v;
  localizer.omega = omega;
}

const lineTracker = new LineTracker({ ...LINE_TRACKER_PARAMS, laneWidthInit: SIM_LANE_WIDTH });
const laneNavigator = new LaneNavigator(SIM_NAVIGATOR_PARAMS);
let latestTracked = null;
let lastNavStepTime = null;
let lastPipelineTime = 0;
let lastMapPublishTime = 0;
const localizerTrail = [];
const laneTrace = [];

// ---------------------------------------------------------------------------
// 走行方式: 'qp' = 既存の 1周目マップ作成 + 2周目QPレーシングライン (LaneNavigator)、
// 'sixlane' = 地図なし・オドメトリなしの 6レーン動的選択 (js/six_lane_planner.js)。
// どちらも同じ白線検出 (理想検出/YOLOP/UFLD) -> LineTracker の出力を使い、
// 同じ twist_mux "mpc" 入力に指令を出す。
// ---------------------------------------------------------------------------
// NN の重みは実機ノードと共用 (train_policy.py が生成)。serve.py はリポジトリの
// ルートを配信するので ../src/... で読める。
const SIX_LANE_POLICY_URL = '../src/oit_navigation/oit_navigation/6lane/six_lane_policy.json';
let navMethod = 'qp';
let sixLanePlanner = null;
let sixLaneLoadError = null;
const sixLaneReady = LanePolicyNet.load(SIX_LANE_POLICY_URL)
  .then((net) => {
    sixLanePlanner = new SixLanePlanner(net, { ...SIX_LANE_PARAMS, vMax: MAX_SPEED, maxAngularSpeed: MAX_ANGULAR });
  })
  .catch((err) => {
    console.error('six_lane_policy.json load failed', err);
    sixLaneLoadError = String(err);
  });
let latestSixLaneLines = null; // 最後に見えた白線 (BEV 描画用)

const sixLaneDebugEl = document.getElementById('six-lane-debug');
const sixLaneDebugTextEl = document.getElementById('six-lane-debug-text');
const sixLaneDebugBarsEl = document.getElementById('six-lane-debug-bars');
const mapLabelEl = document.getElementById('map-label');
const navMethodQpBtn = document.getElementById('nav-method-qp-btn');
const navMethodSixLaneBtn = document.getElementById('nav-method-6lane-btn');

function resetNavigation() {
  laneNavigator.reset();
  if (sixLanePlanner) sixLanePlanner.reset();
  lineTracker.reset();
  resetLocalizer();
  localizerTrail.length = 0;
  latestTracked = null;
  lastNavStepTime = null;
  rosMap.left = rosMap.right = rosMap.raceline = null;
  lastMapPublishTime = 0;
  pathTracker.reset();
  departureMonitor.reset();
  coneRecorder.cones.length = 0;
  prevReactiveBias = 0;
  previousNavState = null;
  window.__sim.coneMapPoints = [];
  latestSixLaneLines = null;
  trafficStop.reset();
  // Reflect the reset in the HUD immediately, rather than waiting for the
  // next physics step.
  writeCollisionHud(false, 0, false);
}

function setDetectorMode(mode) {
  detectorMode = mode;
  yolopBtn.classList.toggle('active', mode === 'yolop');
  ufldBtn.classList.toggle('active', mode === 'ufld');
  idealBtn.classList.toggle('active', mode === 'ideal');
  ros2Btn.classList.toggle('active', mode === 'ros2');
  if (mode === 'ideal') {
    detectorStatusEl.textContent = '理想検出 (コース形状 + ノイズ)';
    return;
  }
  if (mode === 'ros2') {
    detectorStatusEl.textContent = ros ? 'ROS2連携 (信号待ち)' : 'ROS2連携 (rosbridge未接続)';
    return;
  }
  if (mode === 'yolop') {
    if (yolopDetector.session) {
      detectorStatusEl.textContent = 'YOLOP (読込済)';
      return;
    }
    detectorStatusEl.textContent = 'YOLOP 読込中...';
    if (!yolopLoadPromise) yolopLoadPromise = yolopDetector.load(YOLOP_ONNX_URL, MODEL_WASM_DIR);
    yolopLoadPromise
      .then(() => { if (detectorMode === 'yolop') detectorStatusEl.textContent = 'YOLOP (読込済)'; })
      .catch((err) => {
        console.error('Failed to load YOLOP ONNX model', err);
        yolopLoadPromise = null;
        detectorStatusEl.textContent = 'YOLOP 読込エラー';
      });
    return;
  }
  if (ufldDetector.session) {
    detectorStatusEl.textContent = 'UFLD (読込済)';
    return;
  }
  detectorStatusEl.textContent = 'UFLD 読込中...';
  if (!ufldLoadPromise) ufldLoadPromise = ufldDetector.load(UFLD_ONNX_URL, UFLD_META_URL, MODEL_WASM_DIR);
  ufldLoadPromise
    .then(() => {
      if (detectorMode === 'ufld') detectorStatusEl.textContent = 'UFLD (読込済)';
    })
    .catch((err) => {
      console.error('Failed to load UFLD ONNX model', err);
      ufldLoadPromise = null;
      detectorStatusEl.textContent = 'UFLD 読込エラー (models/ufld.onnx を生成してください)';
    });
}

yolopBtn.addEventListener('click', () => setDetectorMode('yolop'));
ufldBtn.addEventListener('click', () => setDetectorMode('ufld'));
idealBtn.addEventListener('click', () => setDetectorMode('ideal'));
ros2Btn.addEventListener('click', () => setDetectorMode('ros2'));

function setNavMethod(method) {
  navMethod = method;
  navMethodQpBtn.classList.toggle('active', method === 'qp');
  navMethodSixLaneBtn.classList.toggle('active', method === 'sixlane');
  sixLaneDebugEl.style.display = method === 'sixlane' ? 'block' : 'none';
  mapLabelEl.textContent = method === 'sixlane' ? '6レーン 白線点群 (俯瞰)' : '周回マップ';
  mapCanvas.parentElement.classList.toggle('sixlane', method === 'sixlane');
  finishMappingBtn.disabled = method === 'sixlane';
  if (sixLanePlanner) sixLanePlanner.reset();
  lastNavStepTime = null;
  if (method === 'sixlane') {
    oitStateEl.textContent = sixLaneLoadError ? `6レーン: 重み読込エラー` : '6レーン走行 (地図なし)';
    oitLapEl.textContent = '-';
  }
}
navMethodQpBtn.addEventListener('click', () => setNavMethod('qp'));
navMethodSixLaneBtn.addEventListener('click', () => setNavMethod('sixlane'));

let autonomousMode = false;
let latestAutonomousCmd = { v: 0, omega: 0 };

// Arbitrates between WASD ("gamepad") and lane_navigator ("mpc") exactly
// like the real vehicle's twist_mux (see js/twist_mux.js and
// TWIST_MUX_SOURCES above): a human on WASD always overrides autonomous
// driving instantly, and control reverts to autonomous once no key has been
// held for 0.3s. "mpc" starts disabled since autonomousMode starts false.
const twistMux = new TwistMux(TWIST_MUX_SOURCES);
twistMux.setEnabled('mpc', autonomousMode);

const twistMuxGamepadStateEl = document.getElementById('twist-mux-gamepad-state');
const twistMuxMpcStateEl = document.getElementById('twist-mux-mpc-state');
const twistMuxActiveEl = document.getElementById('twist-mux-active');
const modeBadgeEl = document.getElementById('mode-badge');

autonomousBtn.addEventListener('click', () => {
  autonomousMode = !autonomousMode;
  autonomousBtn.textContent = `自動運転: ${autonomousMode ? 'ON' : 'OFF'}`;
  autonomousBtn.classList.toggle('active', autonomousMode);
  twistMux.setEnabled('mpc', autonomousMode);
  if (!autonomousMode) publishAutonomousStop();
});

const conePlaceBtn = document.getElementById('cone-place-btn');
const coneClearBtn = document.getElementById('cone-clear-btn');
conePlaceBtn.addEventListener('click', () => {
  if (coneEditor.isEnabled()) {
    coneEditor.disable();
    resetChaseView();
    conePlaceBtn.textContent = 'コーン配置: OFF';
  } else {
    coneEditor.enable();
    showConePlacementView();
    conePlaceBtn.textContent = 'コーン配置: ON';
  }
});
coneClearBtn.addEventListener('click', () => coneEditor.clearAll());

// Same as the real lane_navigator's ~/finish_mapping service: closes lap 1
// by hand (e.g. when odometry drift keeps the automatic lap detection from
// firing) and builds the map + QP raceline from what was recorded so far.
finishMappingBtn.addEventListener('click', () => {
  if (detectorMode === 'ros2') {
    callRosTrigger(FINISH_MAPPING_SERVICE);
    return;
  }
  laneNavigator.finishMapping();
});
navResetBtn.addEventListener('click', () => {
  if (detectorMode === 'ros2') {
    callRosTrigger(RESET_SERVICE);
    rosMap.left = rosMap.right = rosMap.raceline = null;
    return;
  }
  resetNavigation();
});
startPoseBtn.addEventListener('click', () => {
  physics.x = SIM_START_POSE.x;
  physics.y = SIM_START_POSE.y;
  physics.yaw = SIM_START_POSE.yaw;
  physics.v = 0;
  physics.omega = 0;
  odomTrailPoints.length = 0;
  odomTrailGeometry.setFromPoints([]);
  resetNavigation();
});

// ---------------------------------------------------------------------------
// "ROS2連携" mode: subscription callbacks for what
// src/oit_navigation/launch/simulator_test.launch.py's lane_detector /
// lane_navigator publish back over rosbridge. These subscriptions are always
// live once connected (see connect() above); each callback no-ops unless
// detectorMode is actually 'ros2'.
// ---------------------------------------------------------------------------
const FINISH_MAPPING_SERVICE = '/lane_navigator/finish_mapping';
const RESET_SERVICE = '/lane_navigator/reset';
const rosMap = { left: null, right: null, raceline: null };
let rosStatus = null;

function callRosTrigger(name) {
  if (!ros) return;
  const srv = new ROSLIB.Service({ ros, name, serviceType: 'std_srvs/srv/Trigger' });
  srv.callService({}, (res) => { oitMessageEl.textContent = res.message || name; });
}

// lane_detector's annotated image (bgr8): camera + detected points +
// role-colored tracked lines, displayed as-is.
function onRosLaneDetectorAnnotatedImage(msg) {
  if (detectorMode !== 'ros2') return;
  const { width, height } = msg;
  const bytes = base64ToUint8Array(msg.data);
  const imageData = laneCtx.createImageData(width, height);
  const bgr = msg.encoding === 'bgr8';
  for (let i = 0, j = 0; j < imageData.data.length; i += 3, j += 4) {
    imageData.data[j] = bytes[bgr ? i + 2 : i];
    imageData.data[j + 1] = bytes[i + 1];
    imageData.data[j + 2] = bytes[bgr ? i : i + 2];
    imageData.data[j + 3] = 255;
  }
  if (laneCanvas.width !== width || laneCanvas.height !== height) {
    laneCanvas.width = width;
    laneCanvas.height = height;
  }
  laneCtx.putImageData(imageData, 0, 0);
}

// extremum_seeking_mpc/cmd_vel: the ROS 2 lane_navigator's actual output.
// Fed directly into twistMux's "mpc" source (no local recomputation).
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

// lane_tracker/status: JSON from LaneNavigator.status().
function onRosLaneTrackerStatus(msg) {
  if (detectorMode !== 'ros2') return;
  try {
    rosStatus = JSON.parse(msg.data);
    showNavStatus(rosStatus);
  } catch (err) {
    oitMessageEl.textContent = msg.data;
  }
}

function onRosMapPath(kind, msg) {
  if (detectorMode !== 'ros2') return;
  rosMap[kind] = msg.poses.map((ps) => [ps.pose.position.x, ps.pose.position.y]);
}

function showNavStatus(st) {
  oitStateEl.textContent = { MAPPING: '1周目: 中央線走行 + 境界記録', OPTIMIZING: 'QP 計算中', RACING: 'レーシングライン走行', STOPPED: '停止' }[st.state] || st.state;
  oitLapEl.textContent = `${st.lap}`;
  oitSamplesEl.textContent = `${st.samples} 点 (次の間隔 ${st.spacing}m, κ=${st.kappa})`;
  oitMessageEl.textContent = st.message || '-';
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
// message's uint8[] fields (sensor_msgs/Image.data).
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

function publishPathTopic(topic, points, frameId = ROBOT_FRAME_ID) {
  if (!topic) return;
  const header = { stamp: nowStamp(), frame_id: frameId };
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

// ---------------------------------------------------------------------------
// Drawing: lane panel (camera + UFLD) and course-map panel (top-down).
// ---------------------------------------------------------------------------
const ROLE_COLORS = { left: '#35d0ff', center: '#ffd400', right: '#ff5ad8' };

// Inverse of projectToGround(): base_link ground point -> image pixel.
function groundToImage(x, y, width, height) {
  const cam = DEFAULT_CAMERA;
  const sx = width / cam.refWidth, sy = height / cam.refHeight;
  const s = Math.sin(cam.pitchDown), c = Math.cos(cam.pitchDown);
  const xr = x - cam.camX, zr = -cam.camHeight;
  const zc = c * xr - s * zr;
  if (zc < 0.3) return null;
  const yDown = -(s * xr + c * zr);
  return [cam.cx * sx + (cam.fx * sx * -y) / zc, cam.cy * sy + (cam.fy * sy * yDown) / zc];
}

function sampleLine(fit, step = 0.5) {
  const pts = [];
  if (!fit) return pts;
  const x0 = Math.max(0.8, fit.inferred ? 1.0 : fit.xMin);
  const x1 = Math.min(12.0, fit.inferred ? 8.0 : fit.xMax);
  for (let x = x0; x <= x1 + 1e-6; x += step) pts.push([x, fit.yAt(x)]);
  return pts;
}

function drawLanePanel(lanes, tracked, width, height, mask = null) {
  if (laneCanvas.width !== width || laneCanvas.height !== height) {
    laneCanvas.width = width;
    laneCanvas.height = height;
  }
  laneCtx.drawImage(captureCanvas, 0, 0, width, height);
  if (mask) {
    // YOLOP white-line mask, lightly tinted green
    const img = laneCtx.getImageData(0, 0, width, height);
    for (let i = 0; i < mask.length; i++) {
      if (!mask[i]) continue;
      img.data[i * 4] *= 0.5;
      img.data[i * 4 + 1] = 0.5 * img.data[i * 4 + 1] + 127;
      img.data[i * 4 + 2] *= 0.5;
    }
    laneCtx.putImageData(img, 0, 0);
  }
  // Raw UFLD points (per slot, white)
  laneCtx.fillStyle = 'rgba(255,255,255,0.85)';
  lanes.forEach((l) => {
    if (!l) return;
    for (let k = 0; k < l.u.length; k++) {
      laneCtx.beginPath();
      laneCtx.arc(l.u[k], l.v[k], 2.5, 0, 2 * Math.PI);
      laneCtx.fill();
    }
  });
  if (!tracked) return;
  // Tracked role lines re-projected into the image (dashed = inferred)
  for (const role of ROLES) {
    const fit = tracked.lines[role];
    if (!fit) continue;
    const px = sampleLine(fit).map(([x, y]) => groundToImage(x, y, width, height)).filter(Boolean);
    if (px.length < 2) continue;
    laneCtx.strokeStyle = ROLE_COLORS[role];
    laneCtx.lineWidth = 3;
    laneCtx.setLineDash(tracked.detected[role] ? [] : [8, 6]);
    laneCtx.beginPath();
    px.forEach(([u, v], i) => (i ? laneCtx.lineTo(u, v) : laneCtx.moveTo(u, v)));
    laneCtx.stroke();
  }
  laneCtx.setLineDash([]);
  laneCtx.font = '14px sans-serif';
  ROLES.forEach((role, i) => {
    laneCtx.fillStyle = ROLE_COLORS[role];
    const tag = tracked.detected[role] ? '検出' : tracked.lines[role] ? '補完' : 'なし';
    laneCtx.fillText(`${{ left: '左', center: '中央', right: '右' }[role]}: ${tag}`, 8, 18 + 17 * i);
  });
}

function drawMapPanel(map, pose) {
  const W = mapCanvas.width, H = mapCanvas.height;
  mapCtx.fillStyle = '#0b0d10';
  mapCtx.fillRect(0, 0, W, H);
  const layers = [map.left, map.right, map.raceline, map.trail, pose ? [[pose[0], pose[1]]] : null].filter((l) => l && l.length);
  if (!layers.length) return;
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const l of layers) for (const [x, y] of l) {
    minX = Math.min(minX, x); maxX = Math.max(maxX, x); minY = Math.min(minY, y); maxY = Math.max(maxY, y);
  }
  const pad = 4;
  const span = Math.max(maxX - minX, maxY - minY, 10) + 2 * pad;
  const scale = Math.min(W, H) / span;
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
  const toPx = ([x, y]) => [W / 2 + (x - cx) * scale, H / 2 - (y - cy) * scale];

  const polyline = (pts, color, width, closed = false) => {
    if (!pts || pts.length < 2) return;
    mapCtx.strokeStyle = color;
    mapCtx.lineWidth = width;
    mapCtx.beginPath();
    pts.forEach((p, i) => { const [u, v] = toPx(p); i ? mapCtx.lineTo(u, v) : mapCtx.moveTo(u, v); });
    if (closed) mapCtx.closePath();
    mapCtx.stroke();
  };
  const dots = (pts, color, r) => {
    if (!pts) return;
    mapCtx.fillStyle = color;
    for (const p of pts) { const [u, v] = toPx(p); mapCtx.beginPath(); mapCtx.arc(u, v, r, 0, 2 * Math.PI); mapCtx.fill(); }
  };
  polyline(map.trail, 'rgba(90,140,255,0.6)', 1.5);
  dots(map.left, ROLE_COLORS.left, 3);
  dots(map.right, ROLE_COLORS.right, 3);
  if (map.raceline) {
    polyline(map.raceline, '#ff9f1a', 2, true);
    mapCtx.strokeStyle = '#ff9f1a';
    mapCtx.lineWidth = 1.5;
    for (const p of map.raceline) { const [u, v] = toPx(p); mapCtx.beginPath(); mapCtx.arc(u, v, 4, 0, 2 * Math.PI); mapCtx.stroke(); }
  }
  if (pose) {
    const [u, v] = toPx(pose);
    const a = -pose[2];
    mapCtx.fillStyle = '#ff3b3b';
    mapCtx.beginPath();
    mapCtx.moveTo(u + 10 * Math.cos(a), v + 10 * Math.sin(a));
    mapCtx.lineTo(u + 6 * Math.cos(a + 2.4), v + 6 * Math.sin(a + 2.4));
    mapCtx.lineTo(u + 6 * Math.cos(a - 2.4), v + 6 * Math.sin(a - 2.4));
    mapCtx.closePath();
    mapCtx.fill();
  }
  mapCtx.font = '14px sans-serif';
  mapCtx.fillStyle = '#e8eaed';
  mapCtx.fillText(`${(span - 2 * pad).toFixed(0)}m 四方 | ●左境界 ●右境界 ○QPウェイポイント`, 8, H - 10);
}

// 6レーン走行の俯瞰図 (base_link, 前方が上・左が左): 仮想6レーンの塗り分け
// (現在レーン=緑, 目標レーン=橙, コーンで塞がれたレーン=赤)、白線の点群
// (左=水色/中央=黄/右=桃, 補完線は破線)、各レーン上端に NN の確率バー、
// Pure Pursuit の注視点と予定軌跡、コーン、車体。
const BEV = { xMin: -1.5, xMax: 13.0, xLane: 12.0 };

function drawSixLaneBev(lines, st, cones) {
  const W = mapCanvas.width, H = mapCanvas.height;
  const scale = H / (BEV.xMax - BEV.xMin);
  const toPx = (x, y) => [W / 2 - y * scale, H - (x - BEV.xMin) * scale];
  const ctx = mapCtx;
  ctx.fillStyle = '#0b0d10';
  ctx.fillRect(0, 0, W, H);
  ctx.strokeStyle = 'rgba(255,255,255,0.07)';
  ctx.lineWidth = 1;
  for (let x = 0; x <= BEV.xMax; x += 2) {
    const [, v] = toPx(x, 0);
    ctx.beginPath(); ctx.moveTo(0, v); ctx.lineTo(W, v); ctx.stroke();
    ctx.fillStyle = 'rgba(255,255,255,0.35)';
    ctx.font = '13px sans-serif';
    ctx.fillText(`${x}m`, 4, v - 3);
  }
  if (lines) {
    const xs = [];
    for (let x = 0; x <= BEV.xLane + 1e-6; x += 0.5) xs.push(x);
    const blocked = new Set(st && st.blocked ? st.blocked : []);
    for (let k = 1; k <= N_LANES; k++) {
      const pts = [...xs.map((x) => toPx(x, laneY(lines, x, k - 1))), ...xs.slice().reverse().map((x) => toPx(x, laneY(lines, x, k)))];
      let fill = k % 2 ? 'rgba(255,255,255,0.05)' : 'rgba(255,255,255,0.10)';
      if (st && k === st.currentLane) fill = 'rgba(80,220,120,0.30)';
      if (st && k === st.targetLane) fill = st.targetLane === st.currentLane ? 'rgba(160,220,90,0.40)' : 'rgba(255,159,26,0.35)';
      if (blocked.has(k)) fill = 'rgba(255,70,70,0.35)';
      ctx.fillStyle = fill;
      ctx.beginPath();
      pts.forEach(([u, v], i) => (i ? ctx.lineTo(u, v) : ctx.moveTo(u, v)));
      ctx.closePath();
      ctx.fill();
    }
    // レーン境界 (白線以外は細い破線)
    for (let F = 0; F <= N_LANES; F++) {
      const role = F === 0 ? 'left' : F === 3 ? 'center' : F === 6 ? 'right' : null;
      ctx.strokeStyle = role ? ROLE_COLORS[role] : 'rgba(255,255,255,0.35)';
      ctx.lineWidth = role ? 2.5 : 1;
      ctx.setLineDash(role ? (lines[role].detected ? [] : [10, 7]) : [4, 6]);
      ctx.beginPath();
      xs.forEach((x, i) => { const [u, v] = toPx(x, laneY(lines, x, F)); i ? ctx.lineTo(u, v) : ctx.moveTo(u, v); });
      ctx.stroke();
    }
    ctx.setLineDash([]);
    // 白線の点群
    for (const role of ROLES) {
      const ln = lines[role];
      if (!ln || !ln.px) continue;
      ctx.fillStyle = ROLE_COLORS[role];
      for (let i = 0; i < ln.px.length; i++) {
        const [u, v] = toPx(ln.px[i], ln.py[i]);
        ctx.beginPath(); ctx.arc(u, v, 3.5, 0, 2 * Math.PI); ctx.fill();
      }
    }
    // レーン番号 + NN 確率バー (上端)
    ctx.font = 'bold 16px sans-serif';
    ctx.textAlign = 'center';
    for (let k = 1; k <= N_LANES; k++) {
      const [u, v] = toPx(BEV.xLane - 0.6, laneY(lines, BEV.xLane - 0.6, k - 0.5));
      const q = st && st.probs ? st.probs[k - 1] : 0;
      ctx.fillStyle = 'rgba(255,159,26,0.9)';
      ctx.fillRect(u - 9, v - 8 - q * 60, 18, q * 60);
      ctx.fillStyle = '#e8eaed';
      ctx.fillText(`${k}`, u, v + 14);
    }
    ctx.textAlign = 'start';
    // Pure Pursuit の予定軌跡 (円弧) と注視点
    if (st && st.lookahead) {
      const [tx, ty] = st.lookahead;
      const kap = (2 * ty) / (tx * tx + ty * ty);
      ctx.strokeStyle = '#ff9f1a';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      const L = Math.hypot(tx, ty) * 1.05;
      for (let sArc = 0; sArc <= L; sArc += 0.1) {
        const x = Math.abs(kap) < 1e-6 ? sArc : Math.sin(kap * sArc) / kap;
        const y = Math.abs(kap) < 1e-6 ? 0 : (1 - Math.cos(kap * sArc)) / kap;
        const [u, v] = toPx(x, y);
        sArc ? ctx.lineTo(u, v) : ctx.moveTo(u, v);
      }
      ctx.stroke();
      const [u, v] = toPx(tx, ty);
      ctx.beginPath(); ctx.arc(u, v, 6, 0, 2 * Math.PI); ctx.stroke();
    }
  }
  // コーン
  ctx.fillStyle = '#ff7a1a';
  for (const c of cones || []) {
    if (c.x < BEV.xMin || c.x > BEV.xMax) continue;
    const [u, v] = toPx(c.x, c.y);
    ctx.beginPath(); ctx.arc(u, v, 0.15 * scale, 0, 2 * Math.PI); ctx.fill();
  }
  // 車体 (base_link = 駆動輪軸, 後輪は wheelbase 後方)
  const body = [[0.35, 0.35], [0.35, -0.35], [-VEHICLE.wheelbase - 0.1, -0.35], [-VEHICLE.wheelbase - 0.1, 0.35]].map(([x, y]) => toPx(x, y));
  ctx.fillStyle = '#ff3b3b';
  ctx.beginPath();
  body.forEach(([u, v], i) => (i ? ctx.lineTo(u, v) : ctx.moveTo(u, v)));
  ctx.closePath();
  ctx.fill();
  ctx.font = '15px sans-serif';
  ctx.fillStyle = '#e8eaed';
  const head = !lines ? '白線なし' : st && st.currentLane ? `現在 L${st.currentLane} → 目標 L${st.targetLane}` : '判断待ち';
  ctx.fillText(`${head}　●左 ●中央 ●右 点群`, 8, H - 10);
}

function navMapLayers() {
  const nav = laneNavigator;
  const samples = nav.recorder.samples;
  const m = nav.courseMap;
  return {
    left: m ? m.left : samples.map((s) => s.left),
    right: m ? m.right : samples.map((s) => s.right),
    raceline: nav.raceline ? nav.raceline.points : null,
    trail: localizerTrail,
  };
}

// ---------------------------------------------------------------------------
// Pipeline tick (15Hz, same timer as the camera publish). lanePipelineBusy is
// a single-slot guard so a slow inference never gets a second one queued.
// ---------------------------------------------------------------------------
let lanePipelineBusy = false;

// 自律走行指令 (extremum_seeking_mpc/cmd_vel = 実機 twist_mux の "mpc" 入力) の送信。
// 通常モードは従来どおりパイプラインの出力を常に送る。実機操縦モードでは
// 「自動運転: ON」でシミュレータ自身の検出器が走っているときだけ送り、実機に
// シミュレータの車と同じ指令を与える (ROS2連携=実機ノードが指令元の時・早送り中・
// タブ非表示で画面が止まっている時は送らない)。
function publishAutonomousCmd(cmd) {
  if (!autonomousCmdVelTopic) return;
  if (teleopOnly && !(autonomousMode && detectorMode !== 'ros2' && !fastForwarding
      && document.visibilityState !== 'hidden')) return;
  autonomousCmdVelTopic.publish(
    new ROSLIB.Message({ linear: { x: cmd.v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: cmd.omega } })
  );
}

// 実機の motor_controller は最後に受けた指令を保持し続け、twist_mux も入力が
// 途絶えただけでは 0 を出さない。実機操縦中に自動運転を止めるときは、mpc 入力に
// 明示的に速度 0 を送って止める。
function publishAutonomousStop() {
  if (!teleopOnly || !autonomousCmdVelTopic) return;
  for (let i = 0; i < TELEOP_STOP_REPEAT; i += 1) {
    autonomousCmdVelTopic.publish(
      new ROSLIB.Message({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } })
    );
  }
}

function stepNavigator(tracked, now = performance.now() / 1000, sixLaneLines = null) {
  if (navMethod === 'sixlane') return stepSixLane(tracked ? sixLaneLines : null, now);
  const dt = lastNavStepTime === null ? 0 : Math.min(now - lastNavStepTime, 0.5);
  lastNavStepTime = now;
  const pose = [localizer.x, localizer.y, localizer.yaw];
  const rawCmd = laneNavigator.step(now, dt, pose, localizer.v, localizer.omega, localizer.s, tracked);
  const avoided = reactiveAvoid(rawCmd, coneDetectionsForAvoidance(), prevReactiveBias, dt);
  prevReactiveBias = avoided.bias;
  avoidanceDebugEl.textContent = avoided.debug;
  const cmd = applyTrafficLightStop(now, dt, localizer.v, { v: avoided.v, omega: avoided.omega });

  // MAPPING -> RACING遷移を検知したら、1回だけレーシングラインをコーン回避
  // 後処理版に差し替え、コーン地図を確定する。
  if (previousNavState !== RACING && laneNavigator.state === RACING && laneNavigator.raceline) {
    const finalizedCones = coneRecorder.finalize(laneNavigator.recorder.samples, laneNavigator.yawDrift,
      laneNavigator.courseMap, laneNavigator.p.lap);
    applyRacelineDeflection(laneNavigator, finalizedCones, laneNavigator.p.tracker);
    window.__sim.coneMapPoints = finalizedCones; // デバッグ確認用
  }
  previousNavState = laneNavigator.state;

  if (laneNavigator.state === RACING && window.__sim.coneMapPoints && window.__sim.coneMapPoints.length) {
    coneLandmarkCorrection(laneNavigator, pose, latestConeDetections, window.__sim.coneMapPoints);
  }

  latestAutonomousCmd = { v: cmd.v, omega: cmd.omega };
  twistMux.update('mpc', cmd.v, cmd.omega, performance.now());
  publishAutonomousCmd(cmd);
  const st = laneNavigator.status();
  showNavStatus(st);
  if (laneTrackerStatusTopic) laneTrackerStatusTopic.publish(new ROSLIB.Message({ data: JSON.stringify(st) }));
  return cmd;
}

// 6レーン走行の1制御周期: 白線 (null = 観測なし) -> SixLanePlanner -> コーン反応回避
// (QP 方式と同じ最終安全層) -> twist_mux "mpc"。オドメトリ (localizer) は使わず、
// 速度だけ車輪速 (CAN 相当, スリップ込み) を使う。
function stepSixLane(lines, now) {
  const dt = lastNavStepTime === null ? 0 : Math.min(now - lastNavStepTime, 0.5);
  lastNavStepTime = now;
  if (!sixLanePlanner) {
    oitMessageEl.textContent = sixLaneLoadError ? `6レーン: ${sixLaneLoadError}` : '6レーン: NN 読込中...';
    return { v: 0, omega: 0 };
  }
  const measured = physics.measuredWheelSpeeds();
  const vMeas = (measured.left + measured.right) / 2;
  const cones = coneDetectionsForAvoidance();
  const st = sixLanePlanner.step(dt, lines, vMeas, cones, !!(lines && lines.reanchored));
  // 白線の役割取り違えを検出したら、LineTracker を追跡済みの横位置で置き直して正しい割り当てに戻す
  // (実機では six_lane_planner が lane_reseed トピックで lane_detector に同じことをさせる)
  if (st.lateralRejected) {
    lineTracker.seedLanePosition(st.F);
    if (sixLaneReseedTopic) sixLaneReseedTopic.publish(new ROSLIB.Message({ data: st.F }));
  }
  const avoided = reactiveAvoid({ v: st.v, omega: st.omega }, cones, prevReactiveBias, dt);
  prevReactiveBias = avoided.bias;
  if (!fastForwarding) avoidanceDebugEl.textContent = avoided.debug;
  const cmd = applyTrafficLightStop(now, dt, vMeas, { v: avoided.v, omega: avoided.omega });
  latestAutonomousCmd = cmd;
  twistMux.update('mpc', cmd.v, cmd.omega, performance.now());
  publishAutonomousCmd(cmd);
  if (sixLaneStatusTopic) sixLaneStatusTopic.publish(new ROSLIB.Message({ data: JSON.stringify(sixLaneStatusJson(st)) }));
  if (sixLaneTargetPathTopic && lines && st.targetLane) {
    const pts = [];
    for (let x = 0; x <= 10 + 1e-6; x += 0.5) pts.push([x, laneY(lines, x, st.targetLane - 0.5)]);
    publishPathTopic(sixLaneTargetPathTopic, pts);
  }
  if (!fastForwarding) showSixLaneDebug(st);
  return cmd;
}

// 実機ノード (six_lane_planner_node.py) の status JSON と同じキー。
function sixLaneStatusJson(st) {
  const r = (v, d = 3) => (typeof v === 'number' ? +v.toFixed(d) : v);
  return {
    phase: st.phase, current_lane: st.currentLane ?? null, target_lane: st.targetLane ?? null,
    pending_lane: st.pendingLane ?? null, pending_count: st.pendingCount ?? 0, sign: st.sign ?? 0,
    teacher_target: r(st.teacherTarget), intensity: r(st.intensity), F: r(st.F), F_meas: r(st.FMeas),
    lateral_rejected: !!st.lateralRejected,
    kappas: (st.kappas || []).map((k) => r(k, 4)), confidence: r(st.confidence),
    nn_probs: (st.nnProbs || []).map((q) => r(q)), probs: (st.probs || []).map((q) => r(q)),
    blocked: st.blocked || [], v: r(st.v), omega: r(st.omega), lost_time: r(st.lostTime),
  };
}

// 右下の思考結果パネル: 日本語の判断説明 + 6レーンの確率バー。
function showSixLaneDebug(st) {
  const header = st.currentLane
    ? `<b>現在 レーン${st.currentLane}</b> (横位置 F=${st.F.toFixed(2)}) → <b>目標 レーン${st.targetLane}</b>`
    : '<b>現在レーン: 不明</b>';
  // 赤信号の停止制御が効いているときは, その判断も並べる (速度は NN ではなく信号で決まっている)
  const tl = detectorMode === 'ros2' ? rosTrafficLightStopStatus : trafficStop.status();
  const tlLine = tl && tl.state !== 'NORMAL' ? [`<b>信号: ${tl.reason}</b>`] : [];
  sixLaneDebugTextEl.innerHTML = [header, ...tlLine, ...explainJa(st, sixLanePlanner.p)].map((l) => `<div>${l}</div>`).join('');
  const probs = st.probs || new Array(N_LANES).fill(0);
  const nn = st.nnProbs || probs;
  sixLaneDebugBarsEl.innerHTML = probs.map((q, i) => {
    const k = i + 1;
    const cls = [k === st.targetLane ? 'target' : '', k === st.currentLane ? 'current' : '', (st.blocked || []).includes(k) ? 'blocked' : ''].join(' ');
    const mark = `${k === st.currentLane ? '●' : ''}${k === st.targetLane ? '★' : ''}`;
    return `<div class="sl-bar ${cls}" title="NN出力 ${(nn[i] * 100).toFixed(1)}% / コーン補正後 ${(q * 100).toFixed(1)}%">`
      + `<div class="sl-fill" style="height:${Math.max(2, q * 100)}%"></div>`
      + `<div class="sl-pct">${(q * 100).toFixed(0)}%</div><div class="sl-name">L${k}${mark}</div></div>`;
  }).join('');
}

// "ROS2連携" 検出モード + 6レーン: 実機ノードの status JSON を表示する。
function onRosSixLaneStatus(msg) {
  if (detectorMode !== 'ros2' || navMethod !== 'sixlane' || !sixLanePlanner) return;
  try {
    const j = JSON.parse(msg.data);
    showSixLaneDebug({
      phase: j.phase, currentLane: j.current_lane ?? undefined, targetLane: j.target_lane, pendingLane: j.pending_lane,
      pendingCount: j.pending_count, sign: j.sign, teacherTarget: j.teacher_target, intensity: j.intensity, F: j.F,
      FMeas: j.F_meas, lateralRejected: j.lateral_rejected,
      kappas: j.kappas, confidence: j.confidence, nnProbs: j.nn_probs, probs: j.probs, blocked: j.blocked,
      v: j.v, omega: j.omega, lostTime: j.lost_time,
    });
  } catch (err) {
    sixLaneDebugTextEl.textContent = msg.data;
  }
}

// エディタで置いたコーンはシミュレータが正確な位置を知っているため、ONNX
// モデルの有無・検出の一時的な失敗に関係なく回避対象へ渡す。カメラ検出結果も
// 残し、同じコーンと思われる近接点は重複させない。
function coneDetectionsForAvoidance() {
  const c = Math.cos(physics.yaw);
  const s = Math.sin(physics.yaw);
  const placed = coneEditor.cones.map((cone) => {
    const dx = cone.x - physics.x;
    const dy = cone.y - physics.y;
    return {
      x: c * dx + s * dy,
      y: -s * dx + c * dy,
      conf: 1,
    };
  });
  const detections = [...placed];
  for (const detection of latestConeDetections) {
    if (!detections.some((known) => Math.hypot(known.x - detection.x, known.y - detection.y) < 0.35)) {
      detections.push(detection);
    }
  }
  return detections;
}

async function updateLanePipeline() {
  if (fastForwarding) return;
  if (detectorMode === 'ros2') {
    // 6レーン走行は判断結果 (右下パネル) だけ ROS 側の status から表示する (周回マップは無い)
    if (navMethod !== 'sixlane') drawMapPanel({ ...rosMap, trail: null }, null);
    return;
  }
  if (lanePipelineBusy || (detectorMode === 'ufld' && !ufldDetector.session)
    || (detectorMode === 'yolop' && !yolopDetector.session)) return;
  lanePipelineBusy = true;
  try {
    await runPerception(performance.now() / 1000);
  } catch (err) {
    console.error('oit_navigation UFLD pipeline error', err);
  } finally {
    lanePipelineBusy = false;
  }
}

// One perception + navigation tick: detect lines (UFLD or ideal), track
// roles, step the navigator at time `now` [s], draw panels, publish topics.
async function runPerception(now) {
  {
    const width = captureCanvas.width, height = captureCanvas.height;
    let lanes;
    let fits;
    let mask = null;
    let groundPts = []; // fits[i] の元になった地面点群 (base_link), 6レーンの曲率推定・BEV 用
    const toFits = (ls) => ls.map((l, i) => {
      if (!l) return null;
      const g = projectToGround(DEFAULT_CAMERA, l.u, l.v, width, height);
      groundPts[i] = g;
      return fitLine(g.x, g.y);
    });
    if (detectorMode === 'yolop') {
      ({ mask } = await yolopDetector.infer(captureCanvas));
      lanes = extractMaskLines(mask, width, height);
      fits = toFits(lanes);
    } else if (detectorMode === 'ideal') {
      const obs = idealDetector.detect({ x: physics.x, y: physics.y, yaw: physics.yaw });
      fits = obs.fits;
      groundPts = obs.points;
      // Observed ground points re-projected into the image, for the lane panel.
      lanes = obs.points.map(({ x, y }) => {
        const px = x.map((xi, i) => groundToImage(xi, y[i], width, height)).filter(Boolean);
        return { u: px.map((q) => q[0]), v: px.map((q) => q[1]) };
      });
    } else {
      lanes = await ufldDetector.detect(captureCanvas);
      fits = toFits(lanes);
    }
    const tracked = lineTracker.update(fits);
    // 6レーン用: 役割ごとの線 (補完線も含む) に、検出線なら元の点群を添える。
    const sixLaneLines = {};
    for (const role of ROLES) {
      const fit = tracked.lines[role];
      const k = fit ? fits.indexOf(fit) : -1;
      sixLaneLines[role] = fit ? {
        yAt: (x) => fit.yAt(x), inferred: !!fit.inferred, detected: tracked.detected[role],
        px: k >= 0 && groundPts[k] ? groundPts[k].x : null, py: k >= 0 && groundPts[k] ? groundPts[k].y : null,
      } : null;
    }
    const sixLaneLinesOk = ROLES.every((r) => sixLaneLines[r]) ? sixLaneLines : null;
    if (sixLaneLinesOk) sixLaneLinesOk.reanchored = !!tracked.reanchored;
    latestSixLaneLines = sixLaneLinesOk;
    if (detectorMode !== 'ros2' && coneDetector.session) {
      try {
        latestConeDetections = await coneDetector.infer(captureCanvas);
      } catch (err) {
        console.error('cone detector inference error', err);
        latestConeDetections = [];
      }
    } else {
      latestConeDetections = [];
    }
    if (conesTopic && detectorMode !== 'ros2') {
      const nowMs = Date.now();
      conesTopic.publish(new ROSLIB.Message({
        header: { stamp: { sec: Math.floor(nowMs / 1000), nanosec: (nowMs % 1000) * 1e6 }, frame_id: ROBOT_FRAME_ID },
        poses: latestConeDetections.map((c) => ({ position: { x: c.x, y: c.y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } })),
      }));
    }
    // ONNX Runtime Web はセッションをまたいでも同時に 1 推論しか走らせられないので、コーンの後に順番に推論する
    await runTrafficLightDetection(now);
    if (navMethod === 'qp' && laneNavigator.state === MAPPING) {
      coneRecorder.update(localizer.s, [localizer.x, localizer.y, localizer.yaw], latestConeDetections);
    }
    latestTracked = tracked;
    // Debug trace (last ~60s) for automated verification from the console.
    laneTrace.push({
      t: +now.toFixed(2),
      pose: [+physics.x.toFixed(2), +physics.y.toFixed(2), +physics.yaw.toFixed(3)],
      meas: fits.map((f) => (f ? +f.yAt(LINE_TRACKER_PARAMS.xRef).toFixed(2) : null)),
      det: ROLES.map((r) => (tracked.detected[r] ? 1 : 0)).join(''),
      off: ROLES.map((r) => +tracked.offsets[r].toFixed(2)),
    });
    if (laneTrace.length > 900) laneTrace.shift();
    if (fastForwarding) {
      stepNavigator(tracked, now, sixLaneLinesOk);
      return;
    }
    lastPipelineTime = performance.now();
    stepNavigator(tracked, now, sixLaneLinesOk);

    drawLanePanel(lanes, tracked, width, height, mask);
    publishImageTopic(laneDetectorAnnotatedImageTopic, width, height, 'rgb8', 3, canvasToRgb8Bytes(laneCtx, width, height), IMAGE_FRAME_ID);
    publishPathTopic(laneLeftTopic, sampleLine(tracked.lines.left));
    publishPathTopic(laneCenterTopic, sampleLine(tracked.lines.center));
    publishPathTopic(laneRightTopic, sampleLine(tracked.lines.right));

    if (navMethod === 'sixlane') {
      drawSixLaneBev(sixLaneLinesOk, sixLanePlanner ? sixLanePlanner.last : null, coneDetectionsForAvoidance());
      return;
    }
    const layers = navMapLayers();
    drawMapPanel(layers, [localizer.x, localizer.y, localizer.yaw]);
    if (performance.now() - lastMapPublishTime > 1000) {
      lastMapPublishTime = performance.now();
      publishPathTopic(leftBoundaryTopic, layers.left, ODOM_NAV_FRAME_ID);
      publishPathTopic(rightBoundaryTopic, layers.right, ODOM_NAV_FRAME_ID);
      if (layers.raceline) publishPathTopic(targetTrajectoryTopic, layers.raceline, ODOM_NAV_FRAME_ID);
    }
  }
}

// ---------------------------------------------------------------------------
// Deterministic fast-forward for automated verification: steps physics
// (autonomous command only), the localizer and perception+navigation at
// IMAGE_PUBLISH_HZ in *simulated* time, without depending on
// requestAnimationFrame (which browsers throttle/stop in hidden tabs).
//   await window.__sim.fastForward(120)   // simulate 120 s
// Works with the 'ideal', 'yolop' and 'ufld' detector modes (the models render the
// onboard camera each perception tick).
// ---------------------------------------------------------------------------
let fastForwarding = false;
let simClock = 0;

async function fastForward(seconds, physicsDt = 1 / 60) {
  if (detectorMode === 'ros2') throw new Error('fastForward is for the in-browser detector modes');
  fastForwarding = true;
  // Let an in-flight regular pipeline tick finish first (an ONNX session
  // cannot run two inferences at once).
  while (lanePipelineBusy) await new Promise((r) => setTimeout(r, 5));
  try {
    const end = simClock + seconds;
    let nextPerception = simClock;
    lastNavStepTime = null;
    while (simClock < end) {
      if (simClock >= nextPerception) {
        nextPerception += 1 / IMAGE_PUBLISH_HZ;
        // カメラ画像を使う検出 (白線モデル・コーン・信号機) があれば、その時点の姿勢で描画し直す
        if (detectorMode === 'ufld' || detectorMode === 'yolop' || coneDetector.session || trafficLightDetector.session) {
          updateOnboardCameraPose();
          renderOnboardCapture();
        }
        await runPerception(simClock);
      }
      const cmd = autonomousMode ? latestAutonomousCmd : { v: 0, omega: 0 };
      physics.stepAutonomous(cmd.v, cmd.omega, physicsDt);
      advanceSignal(physicsDt);
      applyCollisionAndDeparture();
      // odom_imu_localizer stand-in (see animate()'s own call site for the
      // rationale): use measuredWheelSpeeds() so fastForward()-driven runs
      // also see 8%-slip-induced odometry drift instead of ground truth for
      // forward speed. omegaMeas stays physics.omega (not wheel-derived):
      // the real gyro_odometry_publisher (sensing/odometry_publisher)
      // derives yaw/yaw rate entirely from IMU orientation/angular-velocity
      // interpolation, never from wheel differential (see
      // gyro_odometry_publisher.cpp, odometry_publisher.cpp, wheel.hpp),
      // and this sim has no separate IMU noise model, so the vehicle's true
      // omega is the correct stand-in.
      const measured = physics.measuredWheelSpeeds();
      const vMeas = (measured.left + measured.right) / 2;
      const omegaMeas = physics.omega;
      integrateLocalizer(vMeas, omegaMeas, physicsDt);
      recordLocalizerTrail();
      simClock += physicsDt;
    }
  } finally {
    fastForwarding = false;
    lastNavStepTime = null;
    if (navMethod === 'sixlane') {
      if (sixLanePlanner) showSixLaneDebug(sixLanePlanner.last);
      drawSixLaneBev(latestSixLaneLines, sixLanePlanner ? sixLanePlanner.last : null, coneDetectionsForAvoidance());
    } else {
      drawMapPanel(navMapLayers(), [localizer.x, localizer.y, localizer.yaw]);
    }
  }
  return navMethod === 'sixlane' ? sixLaneStatusJson(sixLanePlanner ? sixLanePlanner.last : {}) : laneNavigator.status();
}

function recordLocalizerTrail() {
  const last = localizerTrail[localizerTrail.length - 1];
  if (!last || Math.hypot(localizer.x - last[0], localizer.y - last[1]) > 0.5) {
    localizerTrail.push([localizer.x, localizer.y]);
    if (localizerTrail.length > 4000) localizerTrail.shift();
  }
}

// Watchdog: if no UFLD frame has completed for LANE_DATA_TIMEOUT_MS (model
// still loading, inference stalled, tab throttled), keep stepping the
// navigator without a new observation -- lap 1 decelerates to a stop (no
// center line to follow), lap 2+ keeps following the raceline on odometry
// -- instead of latching the last cmd_vel forever. Mirrors the real
// lane_navigator node's control timer.
setInterval(() => {
  if (detectorMode === 'ros2') {
    if (lastRosCmdVelTime !== 0 && performance.now() - lastRosCmdVelTime > LANE_DATA_TIMEOUT_MS) {
      detectorStatusEl.textContent = 'ROS2連携 (信号途絶)';
    }
    return;
  }
  if (fastForwarding || performance.now() - lastPipelineTime <= LANE_DATA_TIMEOUT_MS / 2) return;
  stepNavigator(null);
}, LANE_DATA_TIMEOUT_MS / 4);

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

  const { left, right } = physics.measuredWheelSpeeds();
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

// Also decoupled from the requestAnimationFrame render loop, but for a
// different reason than the timers above: browsers throttle/suspend rAF
// almost entirely in a backgrounded tab (unlike setInterval, which is only
// clamped to a lower rate, not stopped), which used to be driven from inside
// animate() via an accumulator. Backgrounding the simulator tab for even a
// few seconds - switching to another tab/window, minimizing - froze this
// camera feed for exactly that long, so oit_navigation's whole pipeline
// (lane_detector, lane_navigator) received nothing and every downstream
// visualization appeared to just stop updating. Running the capture+publish tick on
// its own timer keeps frames flowing (against whatever scene state is
// current - stale while backgrounded, since physics integration is still
// tied to animate() - real work resumes once the tab regains focus).
setInterval(() => {
  renderOnboardCapture();
  publishCompressedImage();
  updateLanePipeline(); // async, single-slot-buffered; fire-and-forget
}, 1000 / IMAGE_PUBLISH_HZ);

// ---------------------------------------------------------------------------
// Physics + render loop
// ---------------------------------------------------------------------------
const physics = new VehiclePhysics();
// Debug hook for automated verification (browser console / test harness).
window.__sim = {
  physics, captureCanvas, renderOnboardCapture, updateOnboardCameraPose, laneNavigator, lineTracker, localizer, ufldDetector,
  idealDetector, laneTrace, localizerTrail, fastForward: (sec) => fastForward(sec),
  setDetectorMode: (m) => setDetectorMode(m), resetNavigation: () => resetNavigation(),
  course, obstacles, mylapsRoot, pathTracker, departureMonitor, coneEditor,
  coneDetector, coneRecorder, latestConeDetections: () => latestConeDetections, coneMapPoints: [],
  setNavMethod: (m) => setNavMethod(m), sixLane: () => sixLanePlanner, sixLaneReady,
  seedLineTrackerFromLane: (F) => lineTracker.seedLanePosition(F),
  trafficLightDetector, trafficStop, latestTrafficLight: () => latestTrafficLight,
  setSignalMode: (m) => setSignalMode(m), currentSignal: () => currentSignal(),
  signalClock: () => signalClock, setSignalClock: (t) => { signalClock = t; advanceSignal(0); },
};
resetLocalizer();
setDetectorMode('yolop');
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
const slipLEl = document.getElementById('slip-l');
const slipREl = document.getElementById('slip-r');
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

// コーン配置では、中心線の外接矩形からカメラの高さを求め、コース全体を
// 収める俯瞰視点にする。真上を避けてわずかに傾けることで、OrbitControlsの
// 視線とupベクトルが平行になる特異点を回避する。
function showConePlacementView() {
  const xs = course.centerPath.map(([x]) => x);
  const ys = course.centerPath.map(([, y]) => y);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const width = maxX - minX;
  const depth = maxY - minY;
  const target = rosToThree((minX + maxX) / 2, (minY + maxY) / 2, 0);
  const aspect = window.innerWidth / window.innerHeight;
  const span = Math.max(depth, width / aspect) * 1.25; // 周囲にも少し余白を残す
  const height = span / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2));

  followTarget.copy(target);
  controls.maxDistance = Math.max(controls.maxDistance, height * 1.05);
  camera.position.copy(target).add(new THREE.Vector3(0, height, height * 0.025));
  controls.target.copy(target);
  controls.autoRotate = false;
  rotateViewBtn.classList.remove('active');
  controls.update();
}

// --- View controls: rotate (auto-orbit toggle) / reset ---
const rotateViewBtn = document.getElementById('rotate-view-btn');
const resetViewBtn = document.getElementById('reset-view-btn');

function resetChaseView() {
  controls.maxDistance = 40;
  camera.position.copy(chaseCameraPosition());
  followTarget.copy(rosToThree(physics.x, physics.y, 0.3));
  controls.target.copy(followTarget);
  controls.update();
}

rotateViewBtn.addEventListener('click', () => {
  controls.autoRotate = !controls.autoRotate;
  rotateViewBtn.classList.toggle('active', controls.autoRotate);
});

resetViewBtn.addEventListener('click', () => {
  resetChaseView();
});

const clock = new THREE.Clock();

function updateOnboardCameraPose() {
  // Onboard (ZED mount) camera: rigidly attached to the vehicle.
  // Pitch down angle is 7.3 deg, matching real extrinsic.yaml orientation (r: -82.7 -> ~7.3 deg tilt).
  const PITCH_DOWN_RAD = (7.3 * Math.PI) / 180;
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
}

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

  if (fastForwarding) {
    // fastForward() owns the physics/localizer while it runs.
  } else if (activeSource === 'mpc') {
    physics.stepAutonomous(latestAutonomousCmd.v, latestAutonomousCmd.omega, dt);
  } else {
    physics.step(effectiveKeys, dt);
  }

  if (!fastForwarding) advanceSignal(dt);

  if (!fastForwarding) {
    // fastForward() calls this itself, once per physics step in its own
    // loop; calling it again here too would double up (and, since
    // contactActive is reset each call, could clear it between
    // fastForward's own calls and flicker the 接触中 indicator).
    applyCollisionAndDeparture();

    // odom_imu_localizer stand-in: wheel speed (slip-affected) + IMU yaw
    // rate dead reckoning. CAN(RPM)相当のmeasuredWheelSpeeds()から v を
    // 再構成する -- 真のphysics.vではなく、8%スリップが乗った計測値を
    // 使うことで、実車と同じようにオドメトリ推定(localizer)の位置が
    // 真の位置からズレていく。一方yaw/yaw rateはomegaMeas=physics.omega
    // とし、真の値をそのまま使う: 実車のgyro_odometry_publisher
    // (sensing/odometry_publisher)はyaw/yaw rateを完全にIMU由来
    // (orientation/angular_velocityの補間)で求めており、ホイール差動から
    // 算出することはない (gyro_odometry_publisher.cpp,
    // odometry_publisher.cpp, wheel.hpp を参照)。このシムには別途IMUノイズ
    // モデルが無いため、真のphysics.omegaがIMU相当の代替として妥当。
    const measured = physics.measuredWheelSpeeds();
    const vMeas = (measured.left + measured.right) / 2;
    const omegaMeas = physics.omega;
    integrateLocalizer(vMeas, omegaMeas, dt);
    recordLocalizerTrail();
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
  } else if (!coneEditor.isEnabled()) {
    const smoothing = 1 - Math.pow(0.001, dt);
    const previousTarget = followTarget.clone();
    followTarget.lerp(targetThree, smoothing);
    camera.position.add(followTarget.clone().sub(previousTarget));
    controls.target.copy(followTarget);
  }

  controls.update();

  updateOnboardCameraPose();

  publishAccumulator += dt;
  if (publishAccumulator >= PUBLISH_INTERVAL) {
    publishAccumulator = 0;
    if (teleopOnly) {
      publishTeleopCmdVel();
    } else {
      publishCmdVel(physics.v, physics.omega);
      publishMuxedCmdVel(physics.v, physics.omega);
    }
  }

  // Camera capture/publish + the oit_navigation lane pipeline itself run on a
  // separate setInterval below, not here (see the comment next to it) --
  // publishImu() itself runs on a separate setInterval too (see below), not here.

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
  // Header badge: who is driving right now (the twist_mux winner).
  modeBadgeEl.textContent = activeSource === 'gamepad' ? '手動' : activeSource === 'mpc' ? '自動運転' : '待機';
  modeBadgeEl.className = activeSource === 'gamepad' ? 'manual' : activeSource === 'mpc' ? 'auto' : '';

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
  slipLEl.textContent = `${(physics.slipL * 100).toFixed(1)}%`;
  slipREl.textContent = `${(physics.slipR * 100).toFixed(1)}%`;

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
