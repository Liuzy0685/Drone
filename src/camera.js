
import { THREE } from './three.js'
import { RAPIER } from './rapier.js'
import { deg, clamp } from './utils.js'
import { dt } from './config.js'
import { rpyDegToQuat } from './utils.js'

export function createCameraAnchor(camConfig, avoidCollision = false, world = null, ignoreBody = null) {
    // physical position, computed in the worker
    const camTarget = new THREE.Object3D() // follows the drone with first order smoothing
    const camAnchor = new THREE.Object3D() // actual camera pose
    camTarget.add(camAnchor)
    camAnchor.position.set(...camConfig.position)
    camAnchor.quaternion.copy(rpyDegToQuat(camConfig.rollPitchYaw))

    function update(camConfig, dronePosition, droneQuaternion) {
        const alpha = camConfig.poseTimeConstant == 0 ? 1 : 1.0 - Math.exp(-dt / camConfig.poseTimeConstant)
        camTarget.position.lerp(dronePosition, alpha)
        camTarget.quaternion.slerp(droneQuaternion, alpha)

        camAnchor.position.set(...camConfig.position)
        camAnchor.quaternion.copy(rpyDegToQuat(camConfig.rollPitchYaw))

        if (avoidCollision) {
            const ray = new RAPIER.Ray(
                camTarget.position,
                camAnchor.getWorldPosition(new THREE.Vector3()).sub(camTarget.position)
            );

            const maxToi = 1.0;
            const solid = false;

            const hit = world.castRay(ray, maxToi, solid, RAPIER.QueryFilterFlags.EXCLUDE_SENSORS, null, null, ignoreBody);
            if (hit != null) {
                const hitPoint = ray.pointAt(hit.timeOfImpact);
                const localHitPoint = camTarget.worldToLocal(new THREE.Vector3(hitPoint.x, hitPoint.y, hitPoint.z));
                camAnchor.position.copy(localHitPoint);
            }
        }
    }

    return { camTarget, camAnchor, update }
}

export function createCamera(camConfig) {
    // for rendering
    const mount = new THREE.Object3D()
    const camera = new THREE.PerspectiveCamera(90, 1, 0.1, 1000);
    mount.add(camera)
    camera.quaternion.set(-0.5, -0.5, 0.5, 0.5) // rotate to match drone coordinate system
    camera.userData.fishEyeStrength = camConfig.fishEyeStrength
    camera.userData.exposure = 1 / camConfig.shutterSpeed

    function resize() {
        const aspect = window.innerWidth / window.innerHeight
        const halfDiagonal = Math.tan(camConfig.fieldOfView * deg / 2)
        const halfVertical = halfDiagonal / Math.sqrt(aspect * aspect + 1)
        const vfov = 2 * Math.atan(halfVertical) / deg
        camera.aspect = aspect;
        camera.fov = vfov;
        camera.updateProjectionMatrix();
    }
    window.addEventListener("resize", resize)
    resize()

    return { mount, camera, resize }
}

// ── Free Camera (god‑view) ────────────────────────────────────────────

export function createFreeCamera() {
    const camera = new THREE.PerspectiveCamera(70, 1, 0.1, 2000);
    camera.userData.fishEyeStrength = 0;      // no fisheye in free cam
    camera.userData.exposure = 0;              // no motion blur in free cam

    // spherical coords for orbit‑style rotation
    let yaw = 0;
    let pitch = -0.4; // slight downward look

    // accumulated mouse delta
    let mouseDX = 0;
    let mouseDY = 0;

    function resize() {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
    }
    window.addEventListener('resize', resize);
    resize();

    /**
     * Call each frame.  Moves the camera based on keys and mouse.
     *
     * @param {number} delta    seconds since last frame (or dt)
     * @param {object} keys     keyDown map (e.g. {w:true, a:false, …})
     * @param {object} dronePos THREE.Vector3 — drone position, for framing
     * @param {number} mouseRawDX  raw mouse movement X (pixels) since last frame
     * @param {number} mouseRawDY  raw mouse movement Y (pixels) since last frame
     */
    function update(delta, keys, dronePos, mouseRawDX, mouseRawDY) {
        const SENS = 0.005;     // mouse rotation sensitivity
        const BASE_SPEED = 15;  // m/s
        const FAST_MUL = 4;     // hold Shift

        // ── rotation: mouse direction matches view direction ──
        yaw   += mouseRawDX * SENS;   // mouse right → view right
        pitch += mouseRawDY * SENS;   // mouse down  → view down
        // arrow keys also rotate
        if (keys['arrowleft'])  yaw   += 2.0 * delta;
        if (keys['arrowright']) yaw   -= 2.0 * delta;
        if (keys['arrowup'])    pitch -= 2.0 * delta;
        if (keys['arrowdown'])  pitch += 2.0 * delta;

        pitch = clamp(pitch, -Math.PI / 2 + 0.02, Math.PI / 2 - 0.02);

        // ── camera basis vectors (project coords: XY = horizontal, -Z = up, +Z = down) ──
        const cosPitch = Math.cos(pitch);
        const forward = new THREE.Vector3(
            -Math.sin(yaw) * cosPitch,  // X
             Math.cos(yaw) * cosPitch,  // Y
             Math.sin(pitch),           // Z: +down, -up
        );
        const right = new THREE.Vector3(
             Math.cos(yaw),   // X
             Math.sin(yaw),   // Y
             0,               // Z: stays level
        );
        const up = new THREE.Vector3(0, 0, -1);  // -Z = world up

        // ── movement ──
        const speed = BASE_SPEED * delta * (keys['shift'] ? FAST_MUL : 1);
        if (keys['w']) camera.position.addScaledVector(forward,  speed);
        if (keys['s']) camera.position.addScaledVector(forward, -speed);
        if (keys['a']) camera.position.addScaledVector(right,    speed);
        if (keys['d']) camera.position.addScaledVector(right,   -speed);
        if (keys['r']) camera.position.addScaledVector(up,       speed);  // R = rise (-Z)
        if (keys['f']) camera.position.addScaledVector(up,      -speed);  // F = fall (+Z)

        // ── look‑at ──
        camera.up.set(0, 0, -1);  // project up = -Z
        const lookTarget = camera.position.clone().add(forward);
        camera.lookAt(lookTarget);
    }

    /** call once when switching TO free‑cam so it starts near the drone */
    function snapTo(dronePos) {
        // position behind/above drone (in project coords: -Z = up)
        camera.position.set(dronePos.x - 5, dronePos.y - 3, dronePos.z - 8);
        // look at drone, using project up direction (-Z)
        camera.up.set(0, 0, -1);
        camera.lookAt(dronePos);
        // extract yaw/pitch from the new orientation
        const dir = new THREE.Vector3();
        camera.getWorldDirection(dir);
        pitch = Math.asin(clamp(dir.z, -1, 1));   // Z component = pitch (sin)
        yaw = Math.atan2(dir.x, dir.y);            // XY plane angle
    }

    return { camera, update, resize, snapTo };
}