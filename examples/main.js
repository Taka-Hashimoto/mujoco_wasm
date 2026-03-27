
import * as THREE           from 'three';
import { GUI              } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls    } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { setupGUI, downloadExampleScenesFolder, loadSceneFromURL, getPosition, getQuaternion, toMujocoPos, standardNormal } from './mujocoUtils.js';
import { HexapodCPG, GAITS } from './hexapodCPG.js';
import   load_mujoco        from '../dist/mujoco_wasm.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
// Boot with humanoid (no mesh dependencies), then switch to hexapod after all assets load.
var bootScene   = "humanoid.xml";
var initialScene = "phantomx_hexapod/phantomx_hexapod.xml";
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');
mujoco.FS.writeFile("/working/" + bootScene, await(await fetch("./examples/scenes/" + bootScene)).text());

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    // Load in the state from XML (boot scene for initial setup)
    this.model      = new mujoco.Model("/working/" + bootScene);
    this.state      = new mujoco.State(this.model);
    this.simulation = new mujoco.Simulation(this.model, this.state);

    // Define Random State Variables
    this.params = { scene: initialScene, paused: false, help: false, ctrlnoiserate: 0.0, ctrlnoisestd: 0.0, keyframeNumber: 0, cpgEnabled: true, gait: "tripod" };
    this.cpg = new HexapodCPG("tripod");
    this.cpgEnabled = true;
    this.simTime = 0.0;         // simulation time in seconds (for CPG)
    this.cpgTime = 0.0;         // CPG phase time (only advances when moving)
    this.mujoco_time = 0.0;
    this.bodies  = {}, this.lights = {};
    this.tmpVec  = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    // Keyboard state for locomotion control
    this.keys = {};
    this.forward = 0.0;
    this.turn = 0.0;
    document.addEventListener('keydown', (e) => {
      this.keys[e.code] = true;
      // Blur GUI inputs so keyboard events always reach the simulation
      if (document.activeElement && document.activeElement.tagName === 'INPUT') {
        document.activeElement.blur();
      }
    });
    document.addEventListener('keyup', (e) => { this.keys[e.code] = false; });
    // Also clear keys when window loses focus (e.g. iframe → parent)
    window.addEventListener('blur', () => { this.keys = {}; });

    this.container = document.createElement( 'div' );
    document.body.appendChild( this.container );

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera( 45, window.innerWidth / window.innerHeight, 0.001, 100 );
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(0.5, 0.4, 0.5);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5 );

    this.ambientLight = new THREE.AmbientLight( 0xffffff, 0.1 );
    this.ambientLight.name = 'AmbientLight';
    this.scene.add( this.ambientLight );

    this.renderer = new THREE.WebGLRenderer( { antialias: true } );
    this.renderer.setPixelRatio( window.devicePixelRatio );
    this.renderer.setSize( window.innerWidth, window.innerHeight );
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap; // default THREE.PCFShadowMap
    this.renderer.setAnimationLoop( this.render.bind(this) );

    this.container.appendChild( this.renderer.domElement );

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.1, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager.
    this.dragStateManager = new DragStateManager(this.scene, this.renderer, this.camera, this.container.parentElement, this.controls);
  }

  async init() {
    // Download the the examples to MuJoCo's virtual file system
    await downloadExampleScenesFolder(mujoco);

    // Load the hexapod scene (all mesh assets are now available in the virtual FS)
    [this.model, this.state, this.simulation, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);

    this.gui = new GUI();
    setupGUI(this);

    // On-screen control hints
    let hint = document.createElement('div');
    hint.innerHTML =
      '<div style="display:grid;grid-template-columns:repeat(3,36px);grid-template-rows:repeat(2,36px);gap:3px;font:bold 14px monospace;text-align:center;line-height:36px">' +
      '<div></div><div id="k-w" style="background:rgba(255,255,255,0.15);border-radius:5px;color:#fff">W</div><div></div>' +
      '<div id="k-a" style="background:rgba(255,255,255,0.15);border-radius:5px;color:#fff">A</div>' +
      '<div id="k-s" style="background:rgba(255,255,255,0.15);border-radius:5px;color:#fff">S</div>' +
      '<div id="k-d" style="background:rgba(255,255,255,0.15);border-radius:5px;color:#fff">D</div></div>' +
      '<div style="margin-top:6px;font:11px sans-serif;color:rgba(255,255,255,0.5)">W/S: forward/back &nbsp; A/D: turn</div>';
    hint.style.cssText = 'position:fixed;bottom:20px;left:20px;z-index:100;pointer-events:none;user-select:none';
    document.body.appendChild(hint);
    this.hintKeys = { w: document.getElementById('k-w'), a: document.getElementById('k-a'), s: document.getElementById('k-s'), d: document.getElementById('k-d') };
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize( window.innerWidth, window.innerHeight );
  }

  render(timeMS) {
    this.controls.update();

    // Read keyboard input for locomotion commands
    let targetFwd  = 0.0;
    let targetTurn = 0.0;
    if (this.keys['KeyW'] || this.keys['ArrowUp']   ) { targetFwd  += 1.0; }
    if (this.keys['KeyS'] || this.keys['ArrowDown'] ) { targetFwd  -= 1.0; }
    if (this.keys['KeyA'] || this.keys['ArrowLeft'] ) { targetTurn += 1.0; }
    if (this.keys['KeyD'] || this.keys['ArrowRight']) { targetTurn -= 1.0; }
    // Smooth the commands for natural acceleration
    let alpha = 0.08;
    this.forward += (targetFwd  - this.forward) * alpha;
    this.turn    += (targetTurn - this.turn)    * alpha;
    // Deadzone: snap to zero when very small
    if (Math.abs(this.forward) < 0.01) { this.forward = 0.0; }
    if (Math.abs(this.turn)    < 0.01) { this.turn    = 0.0; }

    // Update control hint key highlights
    if (this.hintKeys) {
      let on = 'background:rgba(180,230,80,0.6);border-radius:5px;color:#fff';
      let off = 'background:rgba(255,255,255,0.15);border-radius:5px;color:#fff';
      this.hintKeys.w.style.cssText = (this.keys['KeyW'] || this.keys['ArrowUp'])    ? on : off;
      this.hintKeys.a.style.cssText = (this.keys['KeyA'] || this.keys['ArrowLeft'])  ? on : off;
      this.hintKeys.s.style.cssText = (this.keys['KeyS'] || this.keys['ArrowDown'])  ? on : off;
      this.hintKeys.d.style.cssText = (this.keys['KeyD'] || this.keys['ArrowRight']) ? on : off;
    }

    if (!this.params["paused"]) {
      let timestep = this.model.getOptions().timestep;
      if (timeMS - this.mujoco_time > 35.0) { this.mujoco_time = timeMS; }
      while (this.mujoco_time < timeMS) {

        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.simulation.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones.
        for (let i = 0; i < this.simulation.qfrc_applied.length; i++) { this.simulation.qfrc_applied[i] = 0.0; }
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition  (this.simulation.xpos , b, this.bodies[b].position);
              getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update(); // Update the world-space force origin
          let force = toMujocoPos(this.dragStateManager.currentWorld.clone().sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass[bodyID] * 250));
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, bodyID);

          // TODO: Apply pose perturbations (mocap bodies only).
        }

        // Apply CPG control targets for hexapod locomotion
        if (this.cpgEnabled && this.cpg && this.model.nu === 18) {
          if (this.simTime < 2.0) {
            // Hold default pose (ctrl = 0) to let the robot settle on the ground
            let ctrl = this.simulation.ctrl;
            for (let i = 0; i < 18; i++) { ctrl[i] = 0.0; }
          } else {
            let isMoving = Math.abs(this.forward) > 0.0 || Math.abs(this.turn) > 0.0;
            if (isMoving) {
              // Advance CPG phase only while moving
              this.cpgTime += timestep;
              let targets = this.cpg.getTargets(this.cpgTime, this.forward, this.turn);
              let ctrl = this.simulation.ctrl;
              for (let i = 0; i < 18; i++) { ctrl[i] = targets[i]; }
            } else {
              // Idle: hold default pose
              let ctrl = this.simulation.ctrl;
              for (let i = 0; i < 18; i++) { ctrl[i] = 0.0; }
            }
          }
        }

        this.simulation.step();
        this.simTime += timestep;

        this.mujoco_time += timestep * 1000.0;
      }

    } else if (this.params["paused"]) {
      this.dragStateManager.update(); // Update the world-space force origin
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        getPosition  (this.simulation.xpos , b, this.tmpVec , false); // Get raw coordinate from MuJoCo
        getQuaternion(this.simulation.xquat, b, this.tmpQuat, false); // Get raw coordinate from MuJoCo

        let offset = toMujocoPos(this.dragStateManager.currentWorld.clone()
          .sub(this.dragStateManager.worldHit).multiplyScalar(0.3));
        if (this.model.body_mocapid[b] >= 0) {
          // Set the root body's mocap position...
          console.log("Trying to move mocap body", b);
          let addr = this.model.body_mocapid[b] * 3;
          let pos  = this.simulation.mocap_pos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;
        } else {
          // Set the root body's position directly...
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos  = this.simulation.qpos;
          pos[addr+0] += offset.x;
          pos[addr+1] += offset.y;
          pos[addr+2] += offset.z;

          //// Save the original root body position
          //let x  = pos[addr + 0], y  = pos[addr + 1], z  = pos[addr + 2];
          //let xq = pos[addr + 3], yq = pos[addr + 4], zq = pos[addr + 5], wq = pos[addr + 6];

          //// Clear old perturbations, apply new ones.
          //for (let i = 0; i < this.simulation.qfrc_applied().length; i++) { this.simulation.qfrc_applied()[i] = 0.0; }
          //for (let bi = 0; bi < this.model.nbody(); bi++) {
          //  if (this.bodies[b]) {
          //    getPosition  (this.simulation.xpos (), bi, this.bodies[bi].position);
          //    getQuaternion(this.simulation.xquat(), bi, this.bodies[bi].quaternion);
          //    this.bodies[bi].updateWorldMatrix();
          //  }
          //}
          ////dragStateManager.update(); // Update the world-space force origin
          //let force = toMujocoPos(this.dragStateManager.currentWorld.clone()
          //  .sub(this.dragStateManager.worldHit).multiplyScalar(this.model.body_mass()[b] * 0.01));
          //let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          //// This force is dumped into xrfc_applied
          //this.simulation.applyForce(force.x, force.y, force.z, 0, 0, 0, point.x, point.y, point.z, b);
          //this.simulation.integratePos(this.simulation.qpos(), this.simulation.qfrc_applied(), 1);

          //// Add extra drag to the root body
          //pos[addr + 0] = x  + (pos[addr + 0] - x ) * 0.1;
          //pos[addr + 1] = y  + (pos[addr + 1] - y ) * 0.1;
          //pos[addr + 2] = z  + (pos[addr + 2] - z ) * 0.1;
          //pos[addr + 3] = xq + (pos[addr + 3] - xq) * 0.1;
          //pos[addr + 4] = yq + (pos[addr + 4] - yq) * 0.1;
          //pos[addr + 5] = zq + (pos[addr + 5] - zq) * 0.1;
          //pos[addr + 6] = wq + (pos[addr + 6] - wq) * 0.1;


        }
      }

      this.simulation.forward();
    }

    // Update body transforms.
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition  (this.simulation.xpos , b, this.bodies[b].position);
        getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Camera follows the hexapod body (MP_BODY = body index 1)
    if (this.cpgEnabled && this.bodies[1]) {
      let bodyPos = this.bodies[1].position;
      let camOffset = new THREE.Vector3().subVectors(this.camera.position, this.controls.target);
      this.controls.target.lerp(bodyPos, 0.05);
      this.camera.position.copy(this.controls.target).add(camOffset);
    }

    // Update light transforms.
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.simulation.light_xpos, l, this.lights[l].position);
        getPosition(this.simulation.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Update tendon transforms.
    let numWraps = 0;
    if (this.mujocoRoot && this.mujocoRoot.cylinders) {
      let mat = new THREE.Matrix4();
      for (let t = 0; t < this.model.ntendon; t++) {
        let startW = this.simulation.ten_wrapadr[t];
        let r = this.model.tendon_width[t];
        for (let w = startW; w < startW + this.simulation.ten_wrapnum[t] -1 ; w++) {
          let tendonStart = getPosition(this.simulation.wrap_xpos, w    , new THREE.Vector3());
          let tendonEnd   = getPosition(this.simulation.wrap_xpos, w + 1, new THREE.Vector3());
          let tendonAvg   = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

          let validStart = tendonStart.length() > 0.01;
          let validEnd   = tendonEnd  .length() > 0.01;

          if (validStart) { this.mujocoRoot.spheres.setMatrixAt(numWraps    , mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validEnd  ) { this.mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd  , new THREE.Quaternion(), new THREE.Vector3(r, r, r))); }
          if (validStart && validEnd) {
            mat.compose(tendonAvg, new THREE.Quaternion().setFromUnitVectors(
              new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()),
              new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
            this.mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
            numWraps++;
          }
        }
      }
      this.mujocoRoot.cylinders.count = numWraps;
      this.mujocoRoot.spheres  .count = numWraps > 0 ? numWraps + 1: 0;
      this.mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
      this.mujocoRoot.spheres  .instanceMatrix.needsUpdate = true;
    }

    // Render!
    this.renderer.render( this.scene, this.camera );
  }
}

let demo = new MuJoCoDemo();
await demo.init();
