import * as THREE from 'three';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { VehiclePhysics, VEHICLE } from './vehicle_physics.js';

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
const CAMERA_MOUNT = { x: 0.055, y: 0.0, z: 0.54 };

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
const onboardCamera = new THREE.PerspectiveCamera(80, 16 / 9, 0.05, 500);
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
    case 'KeyR': physics.reset(); break;
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
const IMAGE_TOPIC_NAME = '/aiformula_sensing/zed_node/left_image/undistorted/compressed';
const IMAGE_FRAME_ID = 'zed_left_camera_optical_frame'; // matches zed_macro.xacro

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
  });

  ros.on('error', () => {
    setStatus('error', 'エラー');
    connectBtn.disabled = false;
    connectBtn.textContent = '接続';
  });

  ros.on('close', () => {
    cmdVelTopic = null;
    compressedImageTopic = null;
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

const PUBLISH_HZ = 20;
const PUBLISH_INTERVAL = 1 / PUBLISH_HZ;
let publishAccumulator = 0;

function publishCmdVel(v, omega) {
  if (!cmdVelTopic) return;
  cmdVelTopic.publish(
    new ROSLIB.Message({
      linear: { x: v, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: omega },
    })
  );
}

const IMAGE_PUBLISH_HZ = 10;
const IMAGE_PUBLISH_INTERVAL = 1 / IMAGE_PUBLISH_HZ;
let imagePublishAccumulator = 0;

// Renders the onboard camera to an offscreen canvas and publishes it as a
// sensor_msgs/msg/CompressedImage. rosbridge decodes a base64 *string* into
// the message's uint8[] "data" field for byte-array fields, so a stripped
// canvas.toDataURL() output is exactly what's expected here.
function publishCompressedImage() {
  if (!compressedImageTopic) return;

  onboardCamera.aspect = CAPTURE_WIDTH / CAPTURE_HEIGHT;
  onboardCamera.updateProjectionMatrix();
  captureRenderer.render(scene, onboardCamera);

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
// Physics + render loop
// ---------------------------------------------------------------------------
const physics = new VehiclePhysics();
const speedVal = document.getElementById('speed-val');
const yawRateVal = document.getElementById('yaw-rate-val');

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

  physics.step(keys, dt);

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

  // Onboard (ZED mount) camera: rigidly attached to the vehicle, looking
  // straight ahead along its heading (mount joint has rpy=0 in the xacro).
  const mountRos = {
    x: physics.x + CAMERA_MOUNT.x * Math.cos(physics.yaw) - CAMERA_MOUNT.y * Math.sin(physics.yaw),
    y: physics.y + CAMERA_MOUNT.x * Math.sin(physics.yaw) + CAMERA_MOUNT.y * Math.cos(physics.yaw),
    z: CAMERA_MOUNT.z,
  };
  const lookAheadRos = {
    x: mountRos.x + Math.cos(physics.yaw) * 5,
    y: mountRos.y + Math.sin(physics.yaw) * 5,
    z: mountRos.z - 0.5,
  };
  onboardCamera.position.copy(rosToThree(mountRos.x, mountRos.y, mountRos.z));
  onboardCamera.lookAt(rosToThree(lookAheadRos.x, lookAheadRos.y, lookAheadRos.z));

  publishAccumulator += dt;
  if (publishAccumulator >= PUBLISH_INTERVAL) {
    publishAccumulator = 0;
    publishCmdVel(physics.v, physics.omega);
  }

  imagePublishAccumulator += dt;
  if (imagePublishAccumulator >= IMAGE_PUBLISH_INTERVAL) {
    imagePublishAccumulator = 0;
    publishCompressedImage();
  }

  speedVal.textContent = physics.v.toFixed(2);
  yawRateVal.textContent = physics.omega.toFixed(2);

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
