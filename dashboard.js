
/* ================================================================
 *  ROBOTIC ARM DASHBOARD
 *  Features: Manual joint control, Record/Playback, Presets,
 *            Inverse Kinematics with 3D real-time visualizer
 * ================================================================ */

// ── Real arm link lengths (mm) ────────────────────────────────
const L = {
  BASE: 56.0,
  L1: 166.7,   // upper arm        (pitch)
  L2: 115.0,   // forearm          (pitch)
  L3: 46.0,   // Arm_03           (ROLL — not a pitch joint)
  L4: 77.0,   // gripper base     (pitch offset)
  L5: 67.3    // gripper tip      (pitch offset)
};
// L3 is a roll joint: merged into effective forearm for position calculation.
// Roll has NO effect on reach — it is an orientation-only DOF (theta[4]).
const L2_EFF = L.L2 + L.L3;        // 161.0 mm — effective 2-link forearm
const L_WRIST = L.L4 + L.L5;        // 144.3 mm — wrist offset along arm axis

// ── Config ────────────────────────────────────────────────────
const WS_URL = `ws://${window.location.host}/ws/browser`;
const JOINTS = ['base', 'shoulder', 'elbow', 'wrist_pitch', 'wrist_roll', 'gripper'];
const JOINT_LABELS = ['Base', 'Shoulder', 'Elbow', 'Wrist Pitch', 'Wrist Roll', 'Gripper'];
const ANGLE_MIN = [0, 0, 0, 0, 0, 80]; // gripper min = 80° (matches servo_controller.h)
const ANGLE_MAX = [180, 90, 90, 150, 180, 180];  // elbow (ch2) capped at 90°

// Must match HOME_ANGLES in servo_controller.h exactly.
// base=90, shoulder=90, elbow=45, wrist_pitch=43, wrist_roll=110, gripper=80
const HOME_ANGLES = [90, 90, 45, 43, 110, 80];

// ── State ─────────────────────────────────────────────────────
let ws = null;
// Display angles — driven by ESP32 feedback via lerp; initialised to HOME
// so the 3D model shows the correct pose before the first WS message arrives.
let angles = [...HOME_ANGLES];
let isDragging = Array(JOINTS.length).fill(false);
const dragGraceTimers = Array(JOINTS.length).fill(null);
let eStop = false, recState = 'idle', frameCount = 0, speed = 1.0;
const sendQueue = [];

// IK state
let ikAngles = [90, 90, 90, 90, 90, 90]; // last solved IK angles
let ikReachable = false;

/* ================================================================
 *  IK SOLVER (JS mirror of ik_solver.h)
 *  2-link planar IK using L1 and L2_EFF (= L2+L3).
 *  L3 roll (theta[4]) is orientation-only — NOT included here.
 * ================================================================ */
function solveIK(x, y, z, pitchDeg, gripperDeg) {
  const DEG = Math.PI / 180;
  const result = {
    reachable: false,
    approximate: false,
    clamped: false,
    theta: Array(6).fill(90),
    achieved: [0, 0, 0],
    errorMm: 0,
    msg: ''
  };

  // ── Base angle ────────────────────────────────────────────────
  // atan2(y,x) gives the horizontal angle of (x,y) in world frame.
  // Servo convention: 90° = arm pointing straight forward (+X axis).
  //   servo 0°   = arm pointing full-left  (world angle = -90°, i.e. -Y)
  //   servo 90°  = arm pointing forward    (world angle =   0°, +X)
  //   servo 180° = arm pointing full-right (world angle = +90°, +Y)
  //
  // The BASE is a YOKE servo (0–180°), so it covers a HALF-HEMISPHERE only:
  // world angles from -90° to +90° (i.e. anything with x >= 0).
  // Points with x < 0 are truly behind the base and unreachable.
  //
  // However atan2 returns -180…+180, so we must handle the wrap correctly:
  //   world angle = atan2(y, x)  [-90°…+90°] → reachable
  //   world angle outside that range → unreachable
  //
  // To give the widest valid workspace:
  //   - If x >= 0: standard mapping, theta0 = worldAngle + 90
  //   - If x < 0 and |worldAngle| <= 90: arm can reach by going to 0° or 180°
  //     but x < 0 means the target is behind — clamp to the nearest edge
  //     and report the closest reachable point (let the arm try).
  //   - Pure x<0 with large negative x: truly behind, reject.

  const worldAngle = Math.atan2(y, x) / DEG;   // -180 to +180

  let theta0;
  if (x >= 0) {
    // Front hemisphere — normal mapping
    theta0 = worldAngle + 90;  // 0° to 180°
  } else {
    // Target is behind the base (x < 0).
    // Check if it's just barely negative (rounding/slider edge):
    // if worldAngle is between -90 and +90 it means x>=0 so this branch
    // should never hit that. When x<0, worldAngle is in (-180,-90) or (90,180).
    // Clamp to nearest reachable base angle edge (0° or 180°).
    if (worldAngle > 90) {
      theta0 = 180;  // snap to right edge
    } else {
      theta0 = 0;    // snap to left edge
    }
    // Re-project rHoriz onto the clamped base direction for the planar IK.
    // This lets the arm reach toward (0,±y,z) which IS reachable.
  }

  if (theta0 < 0 || theta0 > 180) {
    result.msg = `Base out of range: ${theta0.toFixed(1)}°`; return result;
  }

  const zRel = z - L.BASE;
  const pitchRad = pitchDeg * DEG;
  // rHoriz is ALWAYS positive — it's the distance in the arm's own plane.
  // When base is clamped (x<0 case), we still use the full planar reach.
  const rHoriz = Math.sqrt(x * x + y * y);

  // Wrist chain endpoint — subtract L4+L5 offset only (L3 is roll, in L2_EFF)
  const wx = rHoriz - L_WRIST * Math.cos(pitchRad);
  const wz = zRel - L_WRIST * Math.sin(pitchRad);
  const d = Math.sqrt(wx * wx + wz * wz);

  // Reachability check using L2_EFF
  if (d > L.L1 + L2_EFF) {
    result.msg = `Unreachable: dist=${d.toFixed(0)}mm > ${(L.L1 + L2_EFF).toFixed(0)}mm`; return result;
  }
  if (d < Math.abs(L.L1 - L2_EFF)) {
    result.msg = `Too close: dist=${d.toFixed(0)}mm`; return result;
  }

  // Law of cosines — uses L2_EFF, not L.L2
  const cosElbow = (L.L1 * L.L1 + L2_EFF * L2_EFF - d * d) / (2 * L.L1 * L2_EFF);
  const elbowRad = Math.acos(Math.max(-1, Math.min(1, cosElbow)));

  const alpha = Math.atan2(wz, wx);
  const beta = Math.acos(Math.max(-1, Math.min(1,
    (L.L1 * L.L1 + d * d - L2_EFF * L2_EFF) / (2 * L.L1 * d))));
  const shoulderRad = alpha + beta;

  let theta1 = shoulderRad / DEG;
  let theta2 = 180 - (elbowRad / DEG);

  const theta1Raw = theta1;
  const theta2Raw = theta2;
  theta1 = Math.max(0, Math.min(90, theta1));
  theta2 = Math.max(0, Math.min(90, theta2));   // elbow hard-capped at 90°

  // Mirror the firmware: wrist pitch must be derived from the clamped
  // shoulder/elbow angles, otherwise the preview drifts near joint limits.
  const effectiveShoulderRad = theta1 * DEG;
  const effectiveElbowRad = (180 - theta2) * DEG;
  const armPitch = effectiveShoulderRad + (effectiveElbowRad - Math.PI);
  const wristRad = pitchRad - armPitch;
  let theta3 = 90 + (wristRad / DEG);
  theta3 = Math.max(0, Math.min(150, theta3));

  // theta[4] = Arm_03 roll — read from slider, orientation only, no position effect
  const theta4 = parseFloat(document.getElementById('ikWr').value);
  const theta5 = Math.max(0, Math.min(180, gripperDeg));
  const wasClamped = (theta1 !== theta1Raw || theta2 !== theta2Raw);
  const clampNote = wasClamped ? ' [CLAMPED]' : '';

  result.theta = [theta0, theta1, theta2, theta3, theta4, theta5];
  result.clamped = wasClamped;

  const achieved = forwardKinematics(result.theta)[4];
  const errX = achieved[0] - x;
  const errY = achieved[1] - y;
  const errZ = achieved[2] - z;
  const errorMm = Math.sqrt(errX * errX + errY * errY + errZ * errZ);

  result.achieved = achieved;
  result.errorMm = errorMm;
  result.reachable = !wasClamped && errorMm <= 2.0;
  result.approximate = !result.reachable;
  result.msg = `Base=${theta0.toFixed(0)}° Sh=${theta1.toFixed(0)}° El=${theta2.toFixed(0)}° Wr=${theta3.toFixed(0)}°${clampNote}`;
  return result;
}

// Forward kinematics — returns joint positions for 3D drawing.
// Uses L2_EFF (L2+L3) for the elbow segment. Roll (theta[4]) is
// orientation-only and does NOT shift the end-effector position.
function forwardKinematics(th) {
  const DEG = Math.PI / 180;

  // ── Base: servo 90° = forward (+X), 0° = full-left (-Y), 180° = full-right (+Y)
  const baseRad = (th[0] - 90) * DEG;

  // ── Shoulder: match ik_solver.h exactly.
  //    Servo 0° = arm horizontal forward, servo 90° = arm vertical up.
  const shRad = th[1] * DEG;

  // ── Elbow: servo 180° = arm fully extended (forearm aligned with upper arm, cumulative 0° bend).
  //    servo 90° = 90° interior angle between upper arm and forearm.
  //    The elbow adds an angle RELATIVE to the upper arm direction.
  //    elRad = 0 when servo=180 (straight), increases as servo decreases (folds).
  const elRad = (180 - th[2]) * DEG;

  // ── Wrist pitch: servo 90° = wrist neutral (aligned with forearm direction).
  //    Positive wrist servo tilts tip upward relative to forearm.
  const wrRad = (th[3] - 90) * DEG;

  // th[4] = Arm_03 roll — orientation only, not used in FK position chain

  // All positions in 3D world frame [x=forward, y=left, z=up]
  const pts = [];
  pts.push([0, 0, 0]);            // floor / base origin
  pts.push([0, 0, L.BASE]);       // shoulder pivot (raised by base height)

  // Project 2D side-plane point into 3D world using base rotation
  function pt2d_to_3d(x2d, z2d) {
    return [x2d * Math.cos(baseRad), x2d * Math.sin(baseRad), L.BASE + z2d];
  }

  // Upper arm — shoulder angle measured from horizontal (0° = forward-horizontal)
  const s1x = L.L1 * Math.cos(shRad);
  const s1z = L.L1 * Math.sin(shRad);
  pts.push(pt2d_to_3d(s1x, s1z));    // elbow joint

  // Forearm — match ik_solver.h exactly.
  // The firmware FK uses: shRad + elRad - PI.
  const cumEl = shRad + elRad - Math.PI;
  const s2x = s1x + L2_EFF * Math.cos(cumEl);
  const s2z = s1z + L2_EFF * Math.sin(cumEl);
  pts.push(pt2d_to_3d(s2x, s2z));    // wrist joint

  // Wrist + tip offset along wrist direction — also match ik_solver.h.
  const cumWr = cumEl + wrRad;
  const s3x = s2x + L_WRIST * Math.cos(cumWr);
  const s3z = s2z + L_WRIST * Math.sin(cumWr);
  pts.push(pt2d_to_3d(s3x, s3z));    // end-effector tip

  return pts;
}

/* ================================================================
 *  THREE.JS — FULL 3D WORKSPACE
 *  • RAF render loop  • Orbit / pan / zoom
 *  • Double-click floor → move IK target
 *  • Raycaster for target drag in 3D
 *  • Gripper jaws, joint rings, floor shadow, workspace sphere
 * ================================================================ */
let threeScene, threeCamera, threeRenderer, threeTarget;
let ikJointMeshes = [], ikBoneMeshes = [], ikTargetMesh;
let gripperL, gripperR, tcpMarker;
let _rafId = null;
let orbitState = {
  dragging: false, button: -1, lastX: 0, lastY: 0,
  theta: 0.6, phi: 1.0, radius: 700, panX: 0, panY: 50
};
const GRIPPER_JAW_REACH = 38;

function worldToThree(p) {
  return new THREE.Vector3(p[0], p[2], -p[1]);
}

// Plane for raycasting (floor XZ and vertical XZ)
let floorPlane, wallPlane;
const _ray = { raycaster: null };

function initThreeJS() {
  const canvas = document.getElementById('ikCanvas');
  threeRenderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  threeRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  threeRenderer.shadowMap.enabled = true;
  threeRenderer.shadowMap.type = THREE.PCFSoftShadowMap;
  threeRenderer.setClearColor(0x080b10, 1);
  resizeThree();

  threeScene = new THREE.Scene();
  threeScene.fog = new THREE.Fog(0x080b10, 800, 2500);
  threeCamera = new THREE.PerspectiveCamera(50, canvas.clientWidth / canvas.clientHeight, 1, 4000);

  _ray.raycaster = new THREE.Raycaster();
  floorPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  wallPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);

  // ── Lighting ──────────────────────────────────────────────────
  const ambient = new THREE.AmbientLight(0x1a2540, 3.0);
  threeScene.add(ambient);

  const keyLight = new THREE.DirectionalLight(0x00c8ff, 2.5);
  keyLight.position.set(400, 600, 300);
  keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(2048, 2048);
  keyLight.shadow.camera.near = 10;
  keyLight.shadow.camera.far = 2000;
  keyLight.shadow.camera.left = -600;
  keyLight.shadow.camera.right = 600;
  keyLight.shadow.camera.top = 600;
  keyLight.shadow.camera.bottom = -600;
  keyLight.shadow.bias = -0.001;
  threeScene.add(keyLight);

  const fillLight = new THREE.DirectionalLight(0x334488, 1.0);
  fillLight.position.set(-300, 200, -400);
  threeScene.add(fillLight);

  const rimLight = new THREE.PointLight(0x39ff14, 0.8, 800);
  rimLight.position.set(-200, 300, -200);
  threeScene.add(rimLight);

  // ── Floor ─────────────────────────────────────────────────────
  const floorGeo = new THREE.PlaneGeometry(1200, 1200);
  const floorMat = new THREE.MeshStandardMaterial({
    color: 0x0a0f18, roughness: 0.9, metalness: 0.1,
  });
  const floorMesh = new THREE.Mesh(floorGeo, floorMat);
  floorMesh.rotation.x = -Math.PI / 2;
  floorMesh.receiveShadow = true;
  threeScene.add(floorMesh);

  // Floor grid
  const grid = new THREE.GridHelper(1000, 40, 0x0d2040, 0x0d1a2e);
  grid.position.y = 0.5;
  threeScene.add(grid);

  // ── Workspace envelope — FRONT HEMISPHERE ONLY ────────────────
  // Base servo is 0–180°, which sweeps a half-circle in front of the arm.
  // World coords: +X = forward, +Y = left, -Y = right.
  // The reachable zone is everything with world-angle in [-90°, +90°],
  // i.e. x >= 0 half of the horizontal plane.
  // We draw this as a hemisphere (open toward -X / behind).
  const maxR = L.L1 + L2_EFF + L_WRIST;
  // Front hemisphere using a half-sphere (rotate so flat face is at x=0 plane)
  const wsGeo = new THREE.SphereGeometry(maxR, 32, 24, 0, Math.PI);  // half sphere, phi 0..π
  const wsMat = new THREE.MeshBasicMaterial({
    color: 0x00c8ff, wireframe: true, transparent: true, opacity: 0.05,
    side: THREE.DoubleSide
  });
  const wsMesh = new THREE.Mesh(wsGeo, wsMat);
  // SphereGeometry with phiLength=PI gives right half (x>0 in THREE coords = forward)
  // Rotate so the hemisphere faces +Z (forward in world = +X, but Three uses +Z for "into screen")
  // Our convention: world +X forward maps to Three +X. The half-sphere already faces +X.
  threeScene.add(wsMesh);

  // Workspace arc rings — show the 3 principal arcs of the hemisphere
  [0, Math.PI / 4, Math.PI / 2].forEach(tilt => {
    const arcR = maxR * Math.cos(tilt);
    if (arcR < 10) return;
    // Half-circle arc (front only)
    const pts = [];
    for (let a = -Math.PI / 2; a <= Math.PI / 2; a += 0.05) {
      pts.push(new THREE.Vector3(arcR * Math.cos(a), maxR * Math.sin(tilt), arcR * Math.sin(a)));
    }
    const arcGeo = new THREE.BufferGeometry().setFromPoints(pts);
    threeScene.add(new THREE.Line(arcGeo,
      new THREE.LineBasicMaterial({ color: 0x1f3050, transparent: true, opacity: 0.4 })));
  });

  // Workspace equator ring (half only, flat on floor level)
  const ringPts = [];
  for (let a = -Math.PI / 2; a <= Math.PI / 2; a += 0.04) {
    ringPts.push(new THREE.Vector3(maxR * Math.cos(a), 0, maxR * Math.sin(a)));
  }
  const ringGeo2 = new THREE.BufferGeometry().setFromPoints(ringPts);
  threeScene.add(new THREE.Line(ringGeo2,
    new THREE.LineBasicMaterial({ color: 0x1f4060, transparent: true, opacity: 0.6 })));

  // ── Base platform ─────────────────────────────────────────────
  const platGeo = new THREE.CylinderGeometry(50, 55, 12, 24);
  const platMat = new THREE.MeshStandardMaterial({ color: 0x0f1e30, roughness: 0.7, metalness: 0.4 });
  const platMesh = new THREE.Mesh(platGeo, platMat);
  platMesh.position.y = 6;
  platMesh.castShadow = true; platMesh.receiveShadow = true;
  threeScene.add(platMesh);

  // Keep the decorative pedestal aligned with the kinematic shoulder pivot:
  // the top of the base stack should land at y = L.BASE.
  const pedestalHeight = Math.max(8, L.BASE - 12);
  const baseGeo = new THREE.CylinderGeometry(28, 32, pedestalHeight, 20);
  const baseMat = new THREE.MeshStandardMaterial({
    color: 0x1a2535, roughness: 0.6, metalness: 0.5,
    emissive: 0x001020, emissiveIntensity: 0.3
  });
  const baseMesh = new THREE.Mesh(baseGeo, baseMat);
  baseMesh.position.y = 12 + pedestalHeight / 2;
  baseMesh.castShadow = true;
  threeScene.add(baseMesh);

  // Base ring emitter glow
  const baseRingGeo = new THREE.TorusGeometry(32, 2.5, 8, 32);
  const baseRingMat = new THREE.MeshBasicMaterial({ color: 0x00c8ff, transparent: true, opacity: 0.6 });
  const baseRingMesh = new THREE.Mesh(baseRingGeo, baseRingMat);
  baseRingMesh.position.y = 13;
  baseRingMesh.rotation.x = Math.PI / 2;
  threeScene.add(baseRingMesh);

  // ── Arm bone segments ─────────────────────────────────────────
  // Segment 0: L1 upper arm (pitch)
  // Segment 1: L2+L3 effective forearm (L3 is roll, merged into forearm visually)
  // Segment 2: L4 wrist offset (pitch)
  // Segment 3: L5 gripper tip (pitch)
  const boneSpecs = [
    { len: L.L1, r0: 11, r1: 9, color: 0x00c8ff, emissive: 0x003344 },  // upper arm
    { len: L2_EFF, r0: 9, r1: 7, color: 0x0099cc, emissive: 0x002233 },  // forearm (L2+L3)
    { len: L.L4, r0: 7, r1: 5, color: 0x006699, emissive: 0x001122 },  // wrist offset
    { len: L.L5, r0: 5, r1: 3, color: 0x39ff14, emissive: 0x0a2200 },  // gripper
  ];
  ikBoneMeshes = boneSpecs.map(spec => {
    const geo = new THREE.CylinderGeometry(spec.r0, spec.r1, spec.len, 12);
    const mat = new THREE.MeshStandardMaterial({
      color: spec.color, emissive: spec.emissive, roughness: 0.4, metalness: 0.6
    });
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = true;
    threeScene.add(m);
    return m;
  });

  // ── Joint spheres ─────────────────────────────────────────────
  const jointSpecs = [
    { r: 16, color: 0x00c8ff }, { r: 13, color: 0x00b0ee },
    { r: 11, color: 0x0099cc }, { r: 9, color: 0x006699 },
  ];
  ikJointMeshes = jointSpecs.map(spec => {
    const geo = new THREE.SphereGeometry(spec.r, 16, 12);
    const mat = new THREE.MeshStandardMaterial({
      color: spec.color, emissive: new THREE.Color(spec.color).multiplyScalar(0.25),
      roughness: 0.3, metalness: 0.7
    });
    const m = new THREE.Mesh(geo, mat);
    m.castShadow = true;
    threeScene.add(m);
    // Joint ring
    const rGeo = new THREE.TorusGeometry(spec.r + 3, 1.5, 6, 20);
    const rMat = new THREE.MeshBasicMaterial({ color: spec.color, transparent: true, opacity: 0.5 });
    const ring = new THREE.Mesh(rGeo, rMat);
    m.add(ring);
    return m;
  });

  // ── Gripper jaws ──────────────────────────────────────────────
  const jawMat = new THREE.MeshStandardMaterial({ color: 0x39ff14, emissive: 0x0a2200, roughness: 0.4, metalness: 0.5 });
  gripperL = new THREE.Group(); gripperR = new THREE.Group();
  [gripperL, gripperR].forEach((g, side) => {
    const jawBody = new THREE.Mesh(new THREE.BoxGeometry(4, 28, 8), jawMat);
    jawBody.position.y = 14;
    g.add(jawBody);
    const jawTip = new THREE.Mesh(new THREE.ConeGeometry(3, 10, 6), jawMat);
    jawTip.position.y = 33;
    jawTip.rotation.z = Math.PI;
    g.add(jawTip);
    g.castShadow = true;
    threeScene.add(g);
  });

  // ── Target marker ─────────────────────────────────────────────
  const tGroup = new THREE.Group();

  // Pulsing sphere
  const tGeo = new THREE.SphereGeometry(12, 16, 12);
  const tMat = new THREE.MeshStandardMaterial({
    color: 0xff4060, emissive: 0x440010, roughness: 0.3, metalness: 0.2,
    transparent: true, opacity: 0.85
  });
  ikTargetMesh = new THREE.Mesh(tGeo, tMat);
  tGroup.add(ikTargetMesh);

  // Crosshair lines
  const crossMat = new THREE.LineBasicMaterial({ color: 0xff4060, transparent: true, opacity: 0.6 });
  const sz = 35;
  [[sz, 0, 0], [-sz, 0, 0], [0, sz, 0], [0, -sz, 0], [0, 0, sz], [0, 0, -sz]].forEach(d => {
    const pts = [new THREE.Vector3(0, 0, 0), new THREE.Vector3(...d)];
    tGroup.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), crossMat));
  });

  // Circle rings around target
  [1, 1.5].forEach(scale => {
    const cGeo = new THREE.TorusGeometry(18 * scale, 1, 6, 32);
    const cMat = new THREE.MeshBasicMaterial({ color: 0xff4060, transparent: true, opacity: 0.3 / scale });
    const cMesh = new THREE.Mesh(cGeo, cMat);
    tGroup.add(cMesh);
  });

  // Floor shadow ring under target
  const shadowRingGeo = new THREE.RingGeometry(14, 22, 32);
  const shadowRingMat = new THREE.MeshBasicMaterial({ color: 0xff4060, transparent: true, opacity: 0.15, side: THREE.DoubleSide });
  const shadowRing = new THREE.Mesh(shadowRingGeo, shadowRingMat);
  shadowRing.rotation.x = -Math.PI / 2;
  shadowRing.name = 'shadowRing';
  tGroup.add(shadowRing);

  // Vertical projection line to floor
  const projLineMat = new THREE.LineBasicMaterial({ color: 0xff4060, transparent: true, opacity: 0.2 });
  const projLineGeo = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, -500, 0)
  ]);
  tGroup.add(new THREE.Line(projLineGeo, projLineMat));

  threeScene.add(tGroup);
  threeTarget = tGroup;

  // Solved TCP marker — shows where the IK pose actually places the gripper
  // pinch point, separate from the requested target marker.
  const tcpGeo = new THREE.SphereGeometry(6, 12, 10);
  const tcpMat = new THREE.MeshBasicMaterial({
    color: 0x39ff14, transparent: true, opacity: 0.85
  });
  tcpMarker = new THREE.Mesh(tcpGeo, tcpMat);
  tcpMarker.visible = false;
  threeScene.add(tcpMarker);

  // ── Mouse / touch controls ────────────────────────────────────
  let isDblClick = false;
  canvas.addEventListener('dblclick', e => {
    // Double-click: raycast to floor, move IK target there
    const rect = canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1
    );
    _ray.raycaster.setFromCamera(ndc, threeCamera);
    const hit = new THREE.Vector3();
    if (_ray.raycaster.ray.intersectPlane(floorPlane, hit)) {
      // X must be >= 0 (front hemisphere only — base servo is 0–180°)
      const newX = Math.round(Math.max(0, Math.min(450, hit.x)));
      const newY = Math.round(Math.max(-450, Math.min(450, -hit.z)));
      document.getElementById('ikX').value = newX;
      document.getElementById('ikY').value = newY;
      onIkSlider();
      log(`IK target → floor (${newX}, ${newY}, ${parseInt(document.getElementById('ikZ').value)})mm`, 'info');
    }
  });

  canvas.addEventListener('mousedown', e => {
    orbitState.dragging = true; orbitState.button = e.button;
    orbitState.shiftKey = e.shiftKey;
    orbitState.lastX = e.clientX; orbitState.lastY = e.clientY;
    e.preventDefault();
  });
  canvas.addEventListener('contextmenu', e => e.preventDefault());
  window.addEventListener('mouseup', () => { orbitState.dragging = false; });
  window.addEventListener('mousemove', e => {
    if (!orbitState.dragging) return;
    const dx = e.clientX - orbitState.lastX;
    const dy = e.clientY - orbitState.lastY;
    orbitState.lastX = e.clientX; orbitState.lastY = e.clientY;
    if (orbitState.button === 0 && orbitState.shiftKey) {
      // Shift+drag: translate IK target XZ
      let nx = parseFloat(document.getElementById('ikX').value) + dx * 1.2;
      let nz = parseFloat(document.getElementById('ikZ').value) - dy * 1.2;
      nx = Math.round(Math.max(0, Math.min(450, nx)));
      nz = Math.round(Math.max(20, Math.min(450, nz)));
      document.getElementById('ikX').value = nx;
      document.getElementById('ikZ').value = nz;
      onIkSlider();
    } else if (orbitState.button === 0) {
      orbitState.theta -= dx * 0.008;
      orbitState.phi = Math.max(0.05, Math.min(Math.PI * 0.85, orbitState.phi + dy * 0.008));
    } else if (orbitState.button === 2) {
      const right = new THREE.Vector3(-Math.cos(orbitState.theta), 0, Math.sin(orbitState.theta));
      const up2 = new THREE.Vector3(0, 1, 0);
      orbitState.panX -= right.x * dx * 0.4 + up2.x * dy * 0.4;
      orbitState.panY -= dy * 0.4;
    }
  });
  canvas.addEventListener('wheel', e => {
    orbitState.radius = Math.max(80, Math.min(2500, orbitState.radius + e.deltaY * 0.5));
    e.preventDefault();
  }, { passive: false });

  // Touch orbit
  let lastTouch = null, lastTouchDist = 0;
  canvas.addEventListener('touchstart', e => {
    if (e.touches.length === 1) lastTouch = { x: e.touches[0].clientX, y: e.touches[0].clientY };
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      lastTouchDist = Math.sqrt(dx * dx + dy * dy);
    }
  }, { passive: true });
  canvas.addEventListener('touchmove', e => {
    if (e.touches.length === 1 && lastTouch) {
      const dx = e.touches[0].clientX - lastTouch.x;
      const dy = e.touches[0].clientY - lastTouch.y;
      lastTouch = { x: e.touches[0].clientX, y: e.touches[0].clientY };
      orbitState.theta -= dx * 0.01;
      orbitState.phi = Math.max(0.05, Math.min(Math.PI * 0.85, orbitState.phi + dy * 0.01));
    }
    if (e.touches.length === 2) {
      const dx = e.touches[0].clientX - e.touches[1].clientX;
      const dy = e.touches[0].clientY - e.touches[1].clientY;
      const dist = Math.sqrt(dx * dx + dy * dy);
      orbitState.radius = Math.max(80, Math.min(2500, orbitState.radius - (dist - lastTouchDist) * 1.2));
      lastTouchDist = dist;
    }
    e.preventDefault();
  }, { passive: false });

  updateThreeArm(angles);
  startRenderLoop();
}

function resizeThree() {
  if (!threeRenderer) return;
  const canvas = document.getElementById('ikCanvas');
  const w = canvas.clientWidth || canvas.parentElement.clientWidth;
  const h = canvas.clientHeight || canvas.parentElement.clientHeight;
  threeRenderer.setSize(w, h, false);
  if (threeCamera) { threeCamera.aspect = w / h; threeCamera.updateProjectionMatrix(); }
}

let _rafId2 = null;
let _tRotY = 0;
function renderThree() {
  if (!threeRenderer) return;
  _tRotY += 0.008;
  if (ikTargetMesh) ikTargetMesh.rotation.y = _tRotY;

  const o = orbitState;
  const cx = o.radius * Math.sin(o.phi) * Math.sin(o.theta) + o.panX;
  const cy = o.radius * Math.cos(o.phi) + o.panY;
  const cz = o.radius * Math.sin(o.phi) * Math.cos(o.theta);
  threeCamera.position.set(cx, cy, cz);
  threeCamera.lookAt(o.panX, o.panY, 0);
  threeRenderer.render(threeScene, threeCamera);
}

function startRenderLoop() {
  function loop() {
    _rafId2 = requestAnimationFrame(loop);
    renderThree();
  }
  if (_rafId2) cancelAnimationFrame(_rafId2);
  loop();
}

function updateThreeArm(th) {
  if (!threeRenderer) return;
  const pts = forwardKinematics(th);
  const tp = pts.map(worldToThree);
  const wristToTip = new THREE.Vector3().subVectors(tp[4], tp[3]);
  const wristDir = wristToTip.lengthSq() > 1e-9
    ? wristToTip.clone().normalize()
    : new THREE.Vector3(1, 0, 0);
  const gripBase = tp[3].clone().addScaledVector(wristDir, L.L4);

  // Joint spheres at: shoulder[1], elbow[2], wrist[3], tip[4]
  [1, 2, 3, 4].forEach((pi, ji) => {
    if (ikJointMeshes[ji]) ikJointMeshes[ji].position.copy(tp[pi]);
  });

  // Bone cylinders — split the wrist chain into L4 then L5 so the rendered
  // gripper tip lands exactly on the solved end-effector point.
  const segments = [
    [tp[1], tp[2], L.L1],
    [tp[2], tp[3], L2_EFF],
    [tp[3], gripBase, L.L4],
    [gripBase, tp[4], L.L5],
  ];
  for (let i = 0; i < ikBoneMeshes.length; i++) {
    const [start, end, nominalLen] = segments[i];
    const mid = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5);
    const dir = new THREE.Vector3().subVectors(end, start);
    const len = dir.length() || 1;
    ikBoneMeshes[i].position.copy(mid);
    const q = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0), dir.clone().normalize()
    );
    ikBoneMeshes[i].setRotationFromQuaternion(q);
    ikBoneMeshes[i].scale.set(1, len / nominalLen, 1);
  }

  // Gripper jaws
  if (gripperL && gripperR) {
    const tip = tp[4];
    const dir = wristDir.clone();
    const gripAngle = ((parseFloat(document.getElementById('ikG').value) || 90) / 180) * 0.5;
    const right3 = new THREE.Vector3().crossVectors(dir, new THREE.Vector3(0, 1, 0)).normalize();
    if (right3.length() < 0.01) right3.set(1, 0, 0);
    const spread = 10 + gripAngle * 20;
    // The jaw meshes extend forward from their local origin by ~38 mm.
    // Offset them backward so the visible pinch point coincides with tp[4].
    const jawBase = tip.clone().addScaledVector(dir, -GRIPPER_JAW_REACH);
    gripperL.position.copy(jawBase).addScaledVector(right3, spread);
    gripperR.position.copy(jawBase).addScaledVector(right3, -spread);
    const q2 = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    gripperL.setRotationFromQuaternion(q2);
    gripperR.setRotationFromQuaternion(q2);
  }

  if (tcpMarker) {
    tcpMarker.position.copy(tp[4]);
    tcpMarker.visible = true;
    tcpMarker.material.color.setHex(ikReachable ? 0x39ff14 : 0xffb020);
  }

  // IK target group position
  const ix = parseFloat(document.getElementById('ikX').value);
  const iy = parseFloat(document.getElementById('ikY').value);
  const iz = parseFloat(document.getElementById('ikZ').value);
  if (threeTarget) {
    threeTarget.position.set(ix, iz, -iy);
    // Shadow ring stays on floor
    const sr = threeTarget.getObjectByName('shadowRing');
    if (sr) sr.position.y = -iz + 1;
  }

  // Update viewport coord display
  const vpX = document.getElementById('vpX');
  const vpY = document.getElementById('vpY');
  const vpZ = document.getElementById('vpZ');
  if (vpX) { vpX.textContent = Math.round(ix); vpY.textContent = Math.round(iy); vpZ.textContent = Math.round(iz); }
}

// ── IK Slider handler ─────────────────────────────────────────
function onIkSlider() {
  const x = parseFloat(document.getElementById('ikX').value);
  const y = parseFloat(document.getElementById('ikY').value);
  const z = parseFloat(document.getElementById('ikZ').value);
  const p = parseFloat(document.getElementById('ikP').value);
  const g = parseFloat(document.getElementById('ikG').value);
  const wr = parseFloat(document.getElementById('ikWr').value);

  // Sync number inputs with slider values
  syncNumInput('ikXNum', x);
  syncNumInput('ikYNum', y);
  syncNumInput('ikZNum', z);
  syncNumInput('ikPNum', p);
  syncNumInput('ikGNum', g);
  syncNumInput('ikWrNum', wr);

  _solveAndUpdate(x, y, z, p, g, wr);
}

// ── IK Number Input handler (typed values) ─────────────────────
function onIkNumInput(sliderId, val) {
  const numVal = parseFloat(val);
  if (isNaN(numVal)) return;
  // Clamp to slider min/max
  const slider = document.getElementById(sliderId);
  const min = parseFloat(slider.min);
  const max = parseFloat(slider.max);
  const clamped = Math.max(min, Math.min(max, numVal));
  slider.value = clamped;
  onIkSlider();
}

// ── Sync a number input field (skip if it's currently focused to avoid fighting user typing)
function syncNumInput(id, val) {
  const el = document.getElementById(id);
  if (el && document.activeElement !== el) {
    el.value = Math.round(val);
  }
}

// ── Common IK solve + UI update ───────────────────────────────
function _solveAndUpdate(x, y, z, p, g, wr) {
  const result = solveIK(x, y, z, p, g);
  result.theta[4] = wr;

  const sbEl = document.getElementById('ikStatusSb');
  const sbTxt = document.getElementById('ikStatusSbTxt');
  const vpEl = document.getElementById('ikStatus');
  const vpTxt = document.getElementById('ikStatusTxt');

  if (result.reachable) {
    ikAngles = result.theta;
    ikReachable = true;
    if (sbEl) { sbEl.className = 'ik-status ok'; sbTxt.textContent = result.msg; }
    if (vpEl) { vpEl.className = 'vp-status ok'; vpTxt.textContent = '● REACHABLE'; }
    updateThreeArm(result.theta);
    updateArmSvg(result.theta);
    renderIkReadout(result.theta);
  } else if (result.approximate) {
    ikAngles = result.theta;
    ikReachable = false;
    if (sbEl) { sbEl.className = 'ik-status err'; sbTxt.textContent = `${result.msg}  ERR=${result.errorMm.toFixed(1)}mm`; }
    if (vpEl) { vpEl.className = 'vp-status err'; vpTxt.textContent = `△ LIMITED (${result.errorMm.toFixed(0)}mm off)`; }
    updateThreeArm(result.theta);
    updateArmSvg(result.theta);
    renderIkReadout(result.theta);
  } else {
    ikReachable = false;
    if (sbEl) { sbEl.className = 'ik-status err'; sbTxt.textContent = result.msg; }
    if (vpEl) { vpEl.className = 'vp-status err'; vpTxt.textContent = '✗ ' + result.msg; }
    if (threeTarget) threeTarget.position.set(x, z, -y);
    if (tcpMarker) tcpMarker.visible = false;
  }
}

function renderIkReadout(th) {
  const el = document.getElementById('ikAngleReadout');
  if (!el) return;
  el.innerHTML = JOINT_LABELS.map((label, i) => `
    <div class="angle-cell">
      <div class="angle-cell-lbl">${label}</div>
      <div class="angle-cell-val">${Math.round(th[i])}<span>°</span></div>
    </div>`).join('');
}

function ikSendCommand() {
  if (!ikReachable) { log('IK target unreachable — cannot send', 'err'); return; }
  const x = parseFloat(document.getElementById('ikX').value);
  const y = parseFloat(document.getElementById('ikY').value);
  const z = parseFloat(document.getElementById('ikZ').value);
  const p = parseFloat(document.getElementById('ikP').value);
  const g = parseFloat(document.getElementById('ikG').value);
  const wr = parseFloat(document.getElementById('ikWr').value);

  // Step 1: Open gripper first
  send({ cmd: 'set_servo', ch: 5, angle: g });
  log(`Gripper → ${g}° (opening first)`, 'info');

  // Step 2: After gripper settles, send ik_move to reach the position
  setTimeout(() => {
    send({ cmd: 'ik_move', x, y, z, pitch: p, grip: g, roll: wr });
    log(`IK → (${x},${y},${z})mm pitch=${p}° → sent`, 'ok');
  }, 600);
}

function ikReadFromArm() {
  // Run FK on current arm angles and back-fill IK sliders + number inputs
  const pts = forwardKinematics(angles);
  const tip = pts[pts.length - 1];
  document.getElementById('ikX').value = Math.round(tip[0]);
  document.getElementById('ikY').value = Math.round(tip[1]);
  document.getElementById('ikZ').value = Math.round(tip[2]);
  onIkSlider();
  log(`IK ← read position (${Math.round(tip[0])},${Math.round(tip[1])},${Math.round(tip[2])})mm`, 'info');
}

/* ================================================================
 *  PICK & PLACE SEQUENCE
 *  Workflow: PICK → GRIP → HOME → DROP → RELEASE
 *
 *  1. PICK  — Open gripper, move to IK pick position (from sliders)
 *  2. GRIP  — Close gripper (grab the object)
 *  3. HOME  — Move to home position (gripper stays closed)
 *  4. DROP  — Move to user-specified drop position (gripper closed)
 *  5. RELEASE — Open gripper (release object), then go home
 * ================================================================ */
const PP_STEPS = ['pick', 'grip', 'home', 'drop', 'release'];
const GRIPPER_OPEN_ANGLE  = 170;   // fully open
const GRIPPER_CLOSE_ANGLE = 80;    // closed (matches servo min)
let _ppRunning = false;
let _ppTimers = [];

function ppSetStep(stepId, statusText) {
  // Clear all step states
  PP_STEPS.forEach(s => {
    const el = document.getElementById('pp-' + s);
    if (el) el.className = 'pp-step';
  });
  // Mark steps before current as done
  for (const s of PP_STEPS) {
    if (s === stepId) break;
    const el = document.getElementById('pp-' + s);
    if (el) el.className = 'pp-step done';
  }
  // Mark current step active
  const active = document.getElementById('pp-' + stepId);
  if (active) active.className = 'pp-step active';
  // Update status text
  const statusEl = document.getElementById('ppStatus');
  const statusTxt = document.getElementById('ppStatusTxt');
  if (statusEl) statusEl.className = 'ik-status ok';
  if (statusTxt) statusTxt.textContent = statusText;
}

function ppReset(msg) {
  PP_STEPS.forEach(s => {
    const el = document.getElementById('pp-' + s);
    if (el) el.className = 'pp-step';
  });
  const statusEl = document.getElementById('ppStatus');
  const statusTxt = document.getElementById('ppStatusTxt');
  if (statusEl) statusEl.className = 'ik-status';
  if (statusTxt) statusTxt.textContent = msg || 'READY';
  document.getElementById('ppPlayBtn').disabled = false;
  document.getElementById('ppStopBtn').disabled = true;
  _ppRunning = false;
}

function ppAllDone() {
  PP_STEPS.forEach(s => {
    const el = document.getElementById('pp-' + s);
    if (el) el.className = 'pp-step done';
  });
  const statusEl = document.getElementById('ppStatus');
  const statusTxt = document.getElementById('ppStatusTxt');
  if (statusEl) statusEl.className = 'ik-status ok';
  if (statusTxt) statusTxt.textContent = '✓ SEQUENCE COMPLETE';
  document.getElementById('ppPlayBtn').disabled = false;
  document.getElementById('ppStopBtn').disabled = true;
  _ppRunning = false;
}

// Schedule a function after delay, tracking the timer so it can be cancelled
function ppDelay(fn, ms) {
  return new Promise(resolve => {
    const t = setTimeout(() => {
      if (!_ppRunning) return;
      fn();
      resolve();
    }, ms);
    _ppTimers.push(t);
  });
}

// Smoothly animate the 3D arm from its current pose to targetAngles over `duration` ms
function ppAnimateTo(targetAngles, duration) {
  return new Promise(resolve => {
    if (!_ppRunning) { resolve(); return; }
    const startAngles = [...angles];
    const startTime = performance.now();

    function tick(now) {
      if (!_ppRunning) { resolve(); return; }
      const elapsed = now - startTime;
      const t = Math.min(elapsed / duration, 1);
      // Smooth ease-in-out
      const ease = t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;

      for (let i = 0; i < 6; i++) {
        angles[i] = startAngles[i] + (targetAngles[i] - startAngles[i]) * ease;
      }
      updateThreeArm(angles);
      renderIkReadout(angles);

      // Update servo sliders to follow the animation
      for (let i = 0; i < 6; i++) {
        updateSliderUI(i, angles[i]);
      }

      if (t < 1) {
        requestAnimationFrame(tick);
      } else {
        // Snap to exact target
        for (let i = 0; i < 6; i++) angles[i] = targetAngles[i];
        updateThreeArm(angles);
        renderIkReadout(angles);
        resolve();
      }
    }
    requestAnimationFrame(tick);
  });
}

async function ikPlaySequence() {
  if (_ppRunning) return;
  if (!ikReachable) {
    log('Cannot play: IK pick position is unreachable', 'err');
    return;
  }

  // Read pick position from IK sliders
  const pickX = parseFloat(document.getElementById('ikX').value);
  const pickY = parseFloat(document.getElementById('ikY').value);
  const pickZ = parseFloat(document.getElementById('ikZ').value);
  const pickP = parseFloat(document.getElementById('ikP').value);
  const pickWr = parseFloat(document.getElementById('ikWr').value);

  // Read drop position from drop inputs
  const dropX = parseFloat(document.getElementById('dropX').value);
  const dropY = parseFloat(document.getElementById('dropY').value);
  const dropZ = parseFloat(document.getElementById('dropZ').value);
  const dropP = parseFloat(document.getElementById('dropP').value);
  const dropWr = parseFloat(document.getElementById('dropWr').value);

  // Solve IK for pick and drop positions
  const pickIK = solveIK(pickX, pickY, pickZ, pickP, GRIPPER_OPEN_ANGLE);
  pickIK.theta[4] = pickWr;  // wrist roll
  pickIK.theta[5] = GRIPPER_OPEN_ANGLE;

  const dropIK = solveIK(dropX, dropY, dropZ, dropP, GRIPPER_CLOSE_ANGLE);
  dropIK.theta[4] = dropWr;  // wrist roll from drop input
  dropIK.theta[5] = GRIPPER_CLOSE_ANGLE;

  if (!pickIK.reachable && !pickIK.approximate) {
    log(`Cannot play: pick position unreachable — ${pickIK.msg}`, 'err');
    return;
  }
  if (!dropIK.reachable && !dropIK.approximate) {
    log(`Cannot play: drop position (${dropX},${dropY},${dropZ}) unreachable — ${dropIK.msg}`, 'err');
    return;
  }

  _ppRunning = true;
  _ppTimers = [];
  document.getElementById('ppPlayBtn').disabled = true;
  document.getElementById('ppStopBtn').disabled = false;
  log('▶ Starting Pick & Place sequence', 'ok');

  try {
    // ── STEP 1: PICK — Open gripper, move to pick position ──
    ppSetStep('pick', 'Opening gripper & moving to pick position…');

    // First animate gripper open at current position
    const prePickAngles = [...angles];
    prePickAngles[5] = GRIPPER_OPEN_ANGLE;
    send({ cmd: 'set_servo', ch: 5, angle: GRIPPER_OPEN_ANGLE });
    log(`[P&P] Gripper → ${GRIPPER_OPEN_ANGLE}° (open)`, 'info');
    await ppAnimateTo(prePickAngles, 500);
    if (!_ppRunning) return;

    // Then animate to pick position
    send({ cmd: 'ik_move', x: pickX, y: pickY, z: pickZ, pitch: pickP, grip: GRIPPER_OPEN_ANGLE, roll: pickWr });
    log(`[P&P] Moving to pick (${pickX},${pickY},${pickZ})mm`, 'info');
    await ppAnimateTo(pickIK.theta, 1800);
    if (!_ppRunning) return;
    await ppDelay(() => {}, 300);  // settle
    if (!_ppRunning) return;

    // ── STEP 2: GRIP — Close gripper to grab ──
    ppSetStep('grip', 'Closing gripper — grabbing object…');
    const gripAngles = [...pickIK.theta];
    gripAngles[5] = GRIPPER_CLOSE_ANGLE;
    send({ cmd: 'set_servo', ch: 5, angle: GRIPPER_CLOSE_ANGLE });
    log(`[P&P] Gripper → ${GRIPPER_CLOSE_ANGLE}° (close)`, 'info');
    await ppAnimateTo(gripAngles, 2000);
    if (!_ppRunning) return;
    await ppDelay(() => {}, 300);  // let gripper settle
    if (!_ppRunning) return;

    // ── STEP 3: HOME — Go home with gripper closed ──
    ppSetStep('home', 'Moving to home position…');
    const homeAngles = [...HOME_ANGLES];
    homeAngles[5] = GRIPPER_CLOSE_ANGLE;  // keep gripper closed
    send({ cmd: 'home' });
    // Override gripper after home command
    await ppDelay(() => {
      send({ cmd: 'set_servo', ch: 5, angle: GRIPPER_CLOSE_ANGLE });
    }, 200);
    if (!_ppRunning) return;
    log('[P&P] Moving to HOME (gripper closed)', 'info');
    await ppAnimateTo(homeAngles, 2000);
    if (!_ppRunning) return;
    await ppDelay(() => {}, 500);  // settle at home
    if (!_ppRunning) return;

    // ── STEP 4: DROP — Move to drop position ──
    ppSetStep('drop', `Moving to drop (${dropX},${dropY},${dropZ})mm…`);
    const dropAngles = [...dropIK.theta];
    dropAngles[5] = GRIPPER_CLOSE_ANGLE;  // still holding
    send({ cmd: 'ik_move', x: dropX, y: dropY, z: dropZ, pitch: dropP, grip: GRIPPER_CLOSE_ANGLE, roll: dropWr });
    log(`[P&P] Moving to drop (${dropX},${dropY},${dropZ})mm`, 'info');
    await ppAnimateTo(dropAngles, 1800);
    if (!_ppRunning) return;
    await ppDelay(() => {}, 300);  // settle
    if (!_ppRunning) return;

    // ── STEP 5: RELEASE — Open gripper to release ──
    ppSetStep('release', 'Opening gripper — releasing object…');
    const releaseAngles = [...dropAngles];
    releaseAngles[5] = GRIPPER_OPEN_ANGLE;
    send({ cmd: 'set_servo', ch: 5, angle: GRIPPER_OPEN_ANGLE });
    log(`[P&P] Gripper → ${GRIPPER_OPEN_ANGLE}° (release)`, 'info');
    await ppAnimateTo(releaseAngles, 2000);
    if (!_ppRunning) return;
    await ppDelay(() => {}, 400);
    if (!_ppRunning) return;

    // Return home after release
    send({ cmd: 'home' });
    log('[P&P] Returning to HOME', 'info');
    await ppAnimateTo([...HOME_ANGLES], 1800);
    if (!_ppRunning) return;

    // ── DONE ──
    ppAllDone();
    log('✓ Pick & Place sequence complete!', 'ok');

  } catch (e) {
    log(`[P&P] Error: ${e}`, 'err');
    ppReset('ERROR — sequence aborted');
  }
}

function ikStopSequence() {
  _ppRunning = false;
  _ppTimers.forEach(t => clearTimeout(t));
  _ppTimers = [];
  send({ cmd: 'estop' });
  ppReset('⏹ STOPPED by user');
  log('⏹ Pick & Place sequence stopped', 'warn');
}
/* ================================================================
 *  WEBSOCKET
 * ================================================================ */
function connect() {
  log('Connecting to ' + WS_URL, 'info');
  ws = new WebSocket(WS_URL);
  ws.onopen = () => {
    log('WebSocket connected', 'ok');
    setConnected(true);
    document.getElementById('connOverlay').classList.add('hidden');
    while (sendQueue.length > 0) ws.send(JSON.stringify(sendQueue.shift()));
  };
  ws.onclose = () => {
    log('Connection lost — retrying in 3s…', 'err');
    setConnected(false);
    document.getElementById('connOverlay').classList.remove('hidden');
    setTimeout(connect, 3000);
  };
  ws.onerror = () => log('WebSocket error', 'err');
  ws.onmessage = (ev) => {
    try {
      const msg = JSON.parse(ev.data);
      // Backend wraps ESP32 state as {type:'esp32_state', angles:{...}, ...}
      // Server status arrives as {type:'server_status', esp32_connected:bool}
      if (msg.type === 'esp32_state') {
        applyState(msg);
      } else if (msg.type === 'server_status') {
        const dot = document.getElementById('wsDot');
        const label = document.getElementById('wsLabel');
        if (msg.esp32_connected) {
          dot.classList.add('connected');
          label.textContent = 'ESP32 ONLINE';
        } else {
          dot.classList.remove('connected');
          label.textContent = 'ESP32 OFFLINE';
        }
        log('ESP32 ' + (msg.esp32_connected ? 'connected' : 'disconnected'), msg.esp32_connected ? 'ok' : 'warn');
      } else if (msg.type === 'auto_status') {
        handleAutoStatus(msg);
      } else if (msg.type === 'error') {
        log('Server: ' + msg.msg, 'err');
      } else {
        // Legacy / direct state message (no type field) — pass through
        applyState(msg);
      }
    } catch (e) { log('Bad message: ' + ev.data.slice(0, 40), 'err'); }
  };
}




function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  } else {
    sendQueue.push(obj);
    if (sendQueue.length > 20) sendQueue.shift();
    log('WS not connected — queued', 'warn');
  }
}

function setConnected(ok) {
  document.getElementById('wsDot').className = 'ws-dot' + (ok ? ' connected' : '');
  document.getElementById('wsLabel').textContent = ok ? 'CONNECTED' : 'OFFLINE';
}

/* ================================================================
 *  applyState — real-time closed-loop digital twin
 *
 *  Called every ~50ms from WS onmessage with ESP32's currentAngle[].
 *  Uses exponential lerp (α=0.2) so the 3D model glides smoothly
 *  toward the actual servo positions rather than snapping.
 *
 *  Drag guard: if the user is actively dragging a slider, that joint
 *  is skipped entirely so WS feedback can't fight the user's input.
 *  A per-joint grace timer (1 s after release) prevents a stale
 *  broadcast from snapping a just-released slider back.
 * ================================================================ */
const LERP_ALPHA = 0.2;   // interpolation factor per frame (0=frozen, 1=instant)

function applyState(s) {
  if (s.angles) {
    let visualDirty = false;

    JOINTS.forEach((j, i) => {
      // Skip this joint while the user is dragging it
      if (isDragging[i]) return;

      const received = s.angles[j];
      if (received === undefined) return;

      // Lerp display angle toward received value
      const prev = angles[i];
      angles[i] = prev + (received - prev) * LERP_ALPHA;

      // Only repaint slider if change is visually meaningful (>0.4°)
      if (Math.abs(angles[i] - prev) > 0.4) {
        updateSliderUI(i, angles[i]);
        visualDirty = true;
      }
    });

    // Rebuild 3D model whenever any joint moved
    if (visualDirty) {
      updateThreeArm(angles);
    }
  }

  if (s.eStop !== undefined) {
    eStop = s.eStop;
    document.getElementById('estopFloat').querySelector('button').style.background =
      eStop ? 'rgba(255,64,96,.4)' : '';
  }
  if (s.recState) {
    recState = s.recState;
    document.getElementById('recIndicator').className =
      'rec-indicator' + (recState === 'recording' ? ' active' : '');
    document.getElementById('pbStatus').style.display =
      recState === 'playing' ? '' : 'none';
    document.getElementById('recBtn').disabled = recState === 'recording';
    document.getElementById('stopRecBtn').disabled = recState !== 'recording';
  }
  if (s.frameCount !== undefined) {
    frameCount = s.frameCount;
    document.getElementById('fCount').textContent = frameCount;
    document.getElementById('frameInfo').textContent = frameCount + ' FRAMES';
  }
  if (s.presets) updatePresetList(s.presets);
}

/* ================================================================
 *  SERVO SLIDER UI
 * ================================================================ */
function buildServoGrid() {
  const grid = document.getElementById('servoGrid');
  grid.innerHTML = '';
  JOINTS.forEach((j, i) => {
    const card = document.createElement('div');
    card.className = 'servo-card'; card.id = `card-${i}`;
    card.innerHTML = `
      <div class="servo-header">
        <span class="servo-name">${JOINT_LABELS[i]}</span>
        <span class="servo-idx">CH${i}</span>
      </div>
      <div class="servo-angle" id="angle-${i}">${HOME_ANGLES[i]}<span>°</span></div>
      <input type="range" id="slider-${i}"
        min="${ANGLE_MIN[i]}" max="${ANGLE_MAX[i]}" value="${HOME_ANGLES[i]}"
        onmousedown="startDrag(${i})" onmouseup="endDrag(${i})"
        ontouchstart="startDrag(${i})" ontouchend="endDrag(${i})"
        oninput="onSlider(${i},this.value)"
        onchange="onSliderCommit(${i},this.value)" />`;
    grid.appendChild(card);
    updateSliderUI(i, HOME_ANGLES[i]);
  });
}

function updateSliderUI(i, deg) {
  const slider = document.getElementById(`slider-${i}`);
  const label = document.getElementById(`angle-${i}`);
  if (!slider) return;
  slider.value = deg;
  const pct = ((deg - ANGLE_MIN[i]) / (ANGLE_MAX[i] - ANGLE_MIN[i]) * 100).toFixed(1);
  slider.style.setProperty('--pct', pct + '%');
  label.innerHTML = `${Math.round(deg)}<span>°</span>`;
}

function onSlider(i, val) {
  // Local update for immediate visual feel while dragging.
  // applyState() lerp is suppressed for this joint (isDragging[i]=true),
  // so ESP32 feedback cannot fight the user. On release, feedback takes over.
  angles[i] = parseFloat(val);
  updateSliderUI(i, angles[i]);
  updateThreeArm(angles);
  send({ cmd: 'set_servo', ch: i, angle: angles[i] });
}
function onSliderCommit(i, val) {
  angles[i] = parseFloat(val);
  send({ cmd: 'set_servo', ch: i, angle: angles[i] });
}
function startDrag(i) {
  if (dragGraceTimers[i]) clearTimeout(dragGraceTimers[i]);
  isDragging[i] = true;
  document.getElementById(`card-${i}`).classList.add('active');
}
function endDrag(i) {
  send({ cmd: 'set_servo', ch: i, angle: angles[i] });
  log(`CH${i} ${JOINT_LABELS[i]} → ${Math.round(angles[i])}°`, 'ok');
  dragGraceTimers[i] = setTimeout(() => {
    isDragging[i] = false;
    document.getElementById(`card-${i}`).classList.remove('active');
  }, 1000);
}

/* ================================================================
 *  COMMANDS
 * ================================================================ */
function emergencyStop() { send({ cmd: 'estop' }); log('⬛ EMERGENCY STOP', 'err'); }

function goHome() {
  HOME_ANGLES.forEach((a, i) => {
    angles[i] = a;
    updateSliderUI(i, a);
    send({ cmd: 'set_servo', ch: i, angle: a });
  });
  updateThreeArm(angles);
  log('Moving to HOME position', 'info');
}
function startRecording() {
  send({ cmd: 'record_start' });
  log('⏺ Recording started', 'warn');
  document.getElementById('recIndicator').style.display = 'block';
  document.getElementById('recBtn').disabled = true;
  document.getElementById('stopRecBtn').disabled = false;
}
function stopRecording() {
  send({ cmd: 'record_stop' });
  log('⏹ Recording stopped', 'info');
  document.getElementById('recIndicator').style.display = 'none';
  document.getElementById('recBtn').disabled = false;
  document.getElementById('stopRecBtn').disabled = true;
}
function startPlayback() {
  const sp = parseFloat(document.getElementById('speedSlider').value) / 100;
  const loop = document.getElementById('loopChk').checked;
  send({ cmd: 'playback_start', speed: sp, loop });
  log(`▶ Playback @ ${sp}× ${loop ? 'LOOP' : ''}`, 'ok');
}
function stopPlayback() { send({ cmd: 'playback_stop' }); log('⏹ Playback stopped', 'info'); }
function updateSpeed(v) { speed = parseFloat(v) / 100; document.getElementById('speedVal').textContent = speed.toFixed(2) + '×'; }
function savePreset() {
  const name = document.getElementById('presetName').value.trim();
  if (!name) { log('Enter a preset name', 'warn'); return; }
  send({ cmd: 'preset_save', name, angles: [...angles] });
  log(`💾 Saved preset "${name}"`, 'ok');
  document.getElementById('presetName').value = '';
}
function gotoPreset(name) { send({ cmd: 'preset_goto', name }); log(`→ Go to preset "${name}"`, 'info'); }
function updatePresetList(presets) {
  const el = document.getElementById('presetList');
  if (!presets || !presets.length) { el.innerHTML = '<div style="font-family:var(--mono);font-size:10px;color:var(--text-dim);">No presets saved.</div>'; return; }
  el.innerHTML = presets.map(p => `
    <div class="preset-item"><span>${p.name}</span>
    <button class="go-btn" onclick="gotoPreset('${p.name}')">GO →</button></div>`).join('');
}

/* ================================================================
 *  2D ARM VISUALIZER — stub (3D is primary now)
 * ================================================================ */
function updateArmSvg(th) { /* no-op — 3D viewport is the visualizer */ }

/* ── Collapsible sidebar sections ── */
function toggleSection(id) {
  const body = document.getElementById('sec-' + id);
  const tog = document.getElementById('tog-' + id);
  if (!body) return;
  const collapsed = body.classList.toggle('collapsed');
  if (tog) tog.textContent = collapsed ? '▶' : '▼';
}

/* ================================================================
 *  LOG
 * ================================================================ */
const MAX_LOGS = 60;
function log(msg, type = 'info') {
  const box = document.getElementById('logBox');
  const time = new Date().toTimeString().slice(0, 8);
  const el = document.createElement('div');
  el.className = `entry ${type}`; el.textContent = `[${time}] ${msg}`;
  box.insertBefore(el, box.firstChild);
  while (box.children.length > MAX_LOGS) box.removeChild(box.lastChild);
}
function clearLog() { document.getElementById('logBox').innerHTML = ''; }

/* ================================================================
 *  VISION & AUTONOMOUS MODE
 * ================================================================ */
let _visionPollTimer = null;
let _camMode = false;
let _calibrated = false;

function startCalibration() {
  send({ cmd: 'calibrate' });
  log('📐 Calibration requested — detecting workspace markers…', 'warn');
  const btn = document.getElementById('calibrateBtn');
  if (btn) { btn.disabled = true; btn.textContent = '⏳ CALIBRATING…'; }
}

function startScan() {
  if (!_calibrated) {
    log('⚠ Cannot scan: workspace not calibrated!', 'err');
    return;
  }
  send({ cmd: 'scan' });
  log('🔍 Scan requested — looking for object markers…', 'warn');
  const btn = document.getElementById('scanBtn');
  if (btn) { btn.disabled = true; btn.textContent = '⏳ SCANNING…'; }
}

function toggleCamera(enable) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ cmd: "toggle_camera", enable }));
  }
  const img = document.getElementById('videoFeed');
  if (img) {
    if (enable) {
      img.onerror = function () {
        this.onerror = null;
        this.src = 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIzMjAiIGhlaWdodD0iMjQwIj48cmVjdCB3aWR0aD0iMTAwJSIgaGVpZ2h0PSIxMDAlIiBmaWxsPSIjMDUwNTA1Ii8+PHRleHQgeD0iNTAlIiB5PSI1MCUiIGZvbnQtZmFtaWx5PSJtb25vc3BhY2UiIGZvbnQtc2l6ZT0iMTIiIGZpbGw9IiM2YmI4Y2IiIHRleHQtYW5jaG9yPSJtaWRkbGUiIGR5PSIuM2VtIj5DQU1FUkEgT0ZGTElORTwvdGV4dD48L3N2Zz4=';
      };
      img.src = '/video_feed?' + Date.now();
    } else {
      img.src = 'data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIzMjAiIGhlaWdodD0iMjQwIj48cmVjdCB3aWR0aD0iMTAwJSIgaGVpZ2h0PSIxMDAlIiBmaWxsPSIjMDUwNTA1Ii8+PHRleHQgeD0iNTAlIiB5PSI1MCUiIGZvbnQtZmFtaWx5PSJtb25vc3BhY2UiIGZvbnQtc2l6ZT0iMTIiIGZpbGw9IiM2YmI4Y2IiIHRleHQtYW5jaG9yPSJtaWRkbGUiIGR5PSIuM2VtIj5DQU1FUkEgT0ZGTElORTwvdGV4dD48L3N2Zz4=';
    }
  }
}

function updateArmState(state) {
  // Update state machine nodes
  document.querySelectorAll('.sm-node').forEach(el => el.classList.remove('active'));
  const activeSmNode = document.getElementById('sm-' + state);
  if (activeSmNode) activeSmNode.classList.add('active');
}

function updateCalibrationUI(calibrated) {
  _calibrated = calibrated;
  const statusEl = document.getElementById('calibStatusBig');
  const scanBtn = document.getElementById('scanBtn');
  const scanMsg = document.getElementById('scanStatusMsg');
  const calibBtn = document.getElementById('calibrateBtn');

  if (calibrated) {
    if (statusEl) {
      statusEl.textContent = '✓ CALIBRATED';
      statusEl.style.color = 'var(--green)';
      statusEl.style.background = 'rgba(34,197,94,0.08)';
      statusEl.style.borderColor = 'rgba(34,197,94,0.3)';
    }
    if (scanBtn) { scanBtn.disabled = false; }
    if (scanMsg) { scanMsg.textContent = 'Ready — place object and click SCAN'; scanMsg.style.color = 'var(--green)'; }
    if (calibBtn) { calibBtn.disabled = false; calibBtn.textContent = '📐 RE-CALIBRATE'; }
  } else {
    if (statusEl) {
      statusEl.textContent = '✗ NOT CALIBRATED';
      statusEl.style.color = 'var(--red)';
      statusEl.style.background = 'rgba(239,68,68,0.08)';
      statusEl.style.borderColor = 'rgba(239,68,68,0.2)';
    }
    if (scanBtn) { scanBtn.disabled = true; }
    if (scanMsg) { scanMsg.textContent = 'Calibrate workspace first'; scanMsg.style.color = 'var(--text-dim)'; }
    if (calibBtn) { calibBtn.disabled = false; calibBtn.textContent = '📐 CALIBRATE WORKSPACE'; }
  }
}

function updateWorkspaceMarkers(visibleIds) {
  for (let i = 0; i <= 3; i++) {
    const dot = document.getElementById('wsm-' + i);
    if (!dot) continue;
    if (visibleIds.includes(i)) {
      dot.style.color = 'var(--green)';
      dot.style.background = 'rgba(34,197,94,0.15)';
      dot.style.borderColor = 'var(--green)';
    } else {
      dot.style.color = 'var(--text-dim)';
      dot.style.background = 'var(--bg-hover)';
      dot.style.borderColor = 'var(--border)';
    }
  }
}

function pollVisionStatus() {
  const base = window.location.protocol + '//' + window.location.host;
  fetch(base + '/vision_status')
    .then(r => r.json())
    .then(data => {
      // Sync camera mode state
      const camBtn = document.getElementById('camToggleBig');
      if (camBtn && data.camera_running !== _camMode) {
        _camMode = data.camera_running;
        if (_camMode) {
          camBtn.style.color = 'var(--blue)';
          camBtn.style.borderColor = 'var(--blue)';
          camBtn.textContent = '📷 CAM: ON';
        } else {
          camBtn.style.color = 'var(--text-dim)';
          camBtn.style.borderColor = 'var(--border)';
          camBtn.textContent = '📷 CAM: OFF';
        }
      }

      // Calibration & workspace markers
      updateCalibrationUI(data.calibrated);
      updateWorkspaceMarkers(data.workspace_markers || []);

      // State Machine Sync
      updateArmState(data.arm_state);

      // Re-enable buttons if arm returned to IDLE
      if (data.arm_state === 'IDLE') {
        const calibBtn = document.getElementById('calibrateBtn');
        const scanBtn = document.getElementById('scanBtn');
        if (calibBtn && calibBtn.textContent.includes('CALIBRATING')) {
          calibBtn.disabled = false;
          calibBtn.textContent = data.calibrated ? '📐 RE-CALIBRATE' : '📐 CALIBRATE WORKSPACE';
        }
        if (scanBtn && scanBtn.textContent.includes('SCANNING')) {
          scanBtn.disabled = false;
          scanBtn.textContent = '🔍 SCAN & PICK';
        }
      }

      // Render Detected Markers
      const ml = document.getElementById('markerListBig');
      if (ml && data.detections) {
        if (data.detections.length === 0) {
          ml.innerHTML = '<div style="text-align:center;color:var(--text-dim);font-size:12px;padding:20px;">No markers detected</div>';
        } else {
          ml.innerHTML = data.detections.map(d => {
            const hasCoords = !!d.center_mm;
            const coordsTxt = hasCoords ? `[${d.center_mm[0].toFixed(0)}, ${d.center_mm[1].toFixed(0)}]` : `—`;
            const isWorkspace = d.role === 'workspace';
            const isObject = d.role === 'object';
            const roleTag = isWorkspace ? '<span style="color:var(--blue);font-size:9px;">WS</span>'
              : isObject ? '<span style="color:var(--green);font-size:9px;">OBJ</span>'
                : '<span style="color:var(--text-muted);font-size:9px;">?</span>';
            const color = isObject && hasCoords ? 'var(--green)' : isWorkspace ? 'var(--blue)' : 'var(--text-dim)';
            return `
              <div class="marker-item">
                ${roleTag}
                <span style="font-family:var(--mono);font-size:12px;font-weight:600;">${d.label}</span>
                <span style="font-family:var(--mono);font-size:11px;color:${color};margin-left:auto;">${coordsTxt}</span>
              </div>
            `;
          }).join('');
        }
      }
    })
    .catch(() => { /* server not available yet */ });
}

function initVisionFeed() {
  const base = window.location.protocol + '//' + window.location.host;
  const img = document.getElementById('videoFeed');
  if (img) img.src = base + '/video_feed';
}

function startVisionPolling() {
  pollVisionStatus();
  _visionPollTimer = setInterval(pollVisionStatus, 2000);
}

/* ================================================================
 *  INIT
 * ================================================================ */
buildServoGrid();
window.addEventListener('resize', () => { resizeThree(); });
document.getElementById('speedSlider').addEventListener('input', function () { updateSpeed(this.value); });

window.addEventListener('load', () => {
  if (typeof THREE !== 'undefined') {
    initThreeJS();
    onIkSlider();
  } else {
    document.getElementById('ikCanvas').style.display = 'none';
    log('Three.js failed to load — 3D view unavailable', 'err');
  }
  connect();
  initVisionFeed();
  startVisionPolling();
});

// MODE SWITCHING LOGIC
function switchAppMode(mode) {
  document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('modeBtn' + (mode === 'manual' ? 'Manual' : 'Auto')).classList.add('active');

  if (mode === 'manual') {
    document.getElementById('mode-manual').style.display = 'flex';
    document.getElementById('mode-auto').style.display = 'none';
    if (typeof resizeThree === 'function') setTimeout(resizeThree, 50);
  } else {
    document.getElementById('mode-manual').style.display = 'none';
    document.getElementById('mode-auto').style.display = 'flex';
  }
}

