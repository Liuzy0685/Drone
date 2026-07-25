import { RAPIER } from './rapier.js'
import { createDroneBody, updateDroneBody } from './dronebody.js'
import { dt } from './config.js'
import { clamp, deg } from './utils.js'
import { initControls, controlDrone, initFlightState, controlAutoPhase, initNavState, controlNavigation } from './control.js'

import { THREE } from './three.js'
import { createCameraAnchor } from './camera.js'

async function main() {

    console.log("worker started")

    // init physics
    await RAPIER.init();

    // signal window
    postMessage("ready")

    // await specific message
    function receive(fieldName) {
        return new Promise(resolve => {
            function handler(e) {
                if (fieldName in e.data) {
                    self.removeEventListener("message", handler);
                    resolve(e.data[fieldName]);
                }
            }
            self.addEventListener("message", handler);
        });
    }

    // config
    const config = await receive("config")
    const configListeners = []
    function onConfigUpdate(callback, immediate = false) {
        configListeners.push(callback)
        callback()
    }
    function triggerConfigUpdate() { for (const cb of configListeners) { cb() } }

    // world
    const world = new RAPIER.World({ x: config.map.gravity[0], y: config.map.gravity[1], z: config.map.gravity[2] });
    onConfigUpdate(() => { world.timestep = dt * config.map.timeScale }, true)

    // drone
    const droneBody = createDroneBody(config, world)
    onConfigUpdate(() => { updateDroneBody(droneBody, config) })

    // flight state (takeoff / landing / cruising)
    const flightState = initFlightState(config)

    // terrain
    const terrain = await receive("terrain")
    const colliderNames = new Map()
    for (const part of terrain) {
        const trimeshDesc = RAPIER.ColliderDesc.trimesh(
            new Float32Array(part.vertices),
            new Uint32Array(part.faces)
        );
        if (part.isSensor) {
            trimeshDesc.setSensor(true)
        }
        const collider = world.createCollider(trimeshDesc);
        colliderNames.set(collider, part.name)
    }

    // cameras
    const mockScene = new THREE.Scene()
    const fpv = createCameraAnchor(config.aircraft.camera.firstPerson)
    mockScene.add(fpv.camTarget)
    const tpv = createCameraAnchor(config.aircraft.camera.thirdPerson, true, world, droneBody)
    mockScene.add(tpv.camTarget)

    // inputs
    let controlData
    onConfigUpdate(() => { controlData = initControls(config, dt) }, true)
    let trace = {
        checkpoints: [], // for backtracking when stuck
        nextCheckpoint: Math.ceil(0.5 / dt) // number of steps til next checkpoint
    }
    let inputs = null
    let debug = false
    let firstDebugMessage = true
    self.addEventListener("message", (e) => {
        if (Object.hasOwn(e.data, "inputs")) {
            inputs = e.data.inputs
            debug = e.data.debug
        }
        else if (Object.hasOwn(e.data, "config")) {
            Object.assign(config, e.data.config)
            triggerConfigUpdate()
        }
        else if (Object.hasOwn(e.data, "navCommand")) {
            if (e.data.navCommand === 'start') {
                game.type = 'navigation'
                game.navState = {
                    startPos: new THREE.Vector3(...e.data.startPos),
                    endPos: new THREE.Vector3(...e.data.endPos),
                    reached: false,
                }
                game.navigationActive = true
                flightState.phase = 'cruising'
                // teleport drone to start position (upright)
                droneBody.setTranslation({ x: e.data.startPos[0], y: e.data.startPos[1], z: e.data.startPos[2] });
                droneBody.setLinvel({ x: 0, y: 0, z: 0 });
                droneBody.setAngvel({ x: 0, y: 0, z: 0 });
                // reset rotation to level (identity quaternion = upright)
                droneBody.setRotation({ x: 0, y: 0, z: 0, w: 1 });
            } else if (e.data.navCommand === 'stop') {
                game.navigationActive = false
            }
        }
    })

    // game logic
    // a bit unelegant but as the game evolves this will surely be refactored, surely!
    let game = { type: config.map.mission.type, finished: false }
    if (game.type === "navigation") {
        game.navState = initNavState(config)
        game.navigationActive = false  // default manual, press N to enable auto-nav
        // place drone at start position
        droneBody.setTranslation({
            x: game.navState.startPos.x,
            y: game.navState.startPos.y,
            z: game.navState.startPos.z,
        });
    }
    else if (game.type === "race") {
        game.checkpoints = config.map.mission.checkpoints
        if (config.map.mission.mode === "random") {
            game.mode = "random"
            game.checkpointsTodo = new Set()
            for (const cp of game.checkpoints) {
                game.checkpointsTodo.add(cp)
            }
            postMessage({ type: "initCheckpoints", active: Array.from(game.checkpointsTodo) })
        }
        else if (config.map.mission.mode === "point-to-point") {
            game.mode = "point-to-point"
            game.activeCheckpoint = game.checkpoints[0]
            postMessage({ type: "initCheckpoints", active: [game.activeCheckpoint] })
        }
        else { // mode must be a {laps:n} object
            game.mode = "laps"
            game.lapsLeft = config.map.mission.mode.laps
            game.activeCheckpoint = game.checkpoints[0]
            postMessage({ type: "initCheckpoints", active: [game.activeCheckpoint] })
        }
    }
    function handleSensorHit(sensorHit) {
        if (game.type === "race") {
            if (game.mode === "random") {
                if (game.checkpointsTodo.delete(sensorHit)) {
                    if (game.checkpointsTodo.size === 0) {
                        game.finished = true
                    }
                    postMessage({
                        type: "checkpoint",
                        active: Array.from(game.checkpointsTodo),
                        finished: game.finished
                    })
                }
            }
            else { // mode === "point-to-point" or "laps"
                if (game.activeCheckpoint === sensorHit) {
                    if (game.activeCheckpoint === game.checkpoints[game.checkpoints.length - 1]) { // last checkpoint
                        if (game.mode === "point-to-point") {
                            game.finished = true
                            game.activeCheckpoint = null
                        }
                        else { // mode === "laps"
                            game.lapsLeft -= 1
                            game.activeCheckpoint = game.checkpoints[0]
                        }
                    }
                    else if ( // first checkpoint after last lap -> finished
                        game.mode === "laps"
                        && game.activeCheckpoint === game.checkpoints[0]
                        && game.lapsLeft === 0
                    ) {
                        game.finished = true
                        game.activeCheckpoint = null
                    }
                    else {
                        game.activeCheckpoint = game.checkpoints[game.checkpoints.indexOf(game.activeCheckpoint) + 1]
                    }
                    postMessage({
                        type: "checkpoint",
                        active: game.activeCheckpoint ? [game.activeCheckpoint] : [],
                        finished: game.finished
                    })
                }
            }
        }
    }

    // run engine
    let ingameTime = 0.0
    let tNextStep = performance.now()
    function stepPhysics() {

        world.step()

        // ── collision detection ── (checked below after distances computed) ──

        let sensorHit = null
        world.intersectionPairsWith(droneBody.collider(0), (sensorCollider) => {
            const intersecting = world.intersectionPair(droneBody.collider(0), sensorCollider);

            if (intersecting) {
                sensorHit = colliderNames.get(sensorCollider)
                handleSensorHit(sensorHit)
            }
        });

        if (inputs) {
            // ── handle one-shot commands ──
            if (inputs.command === 'takeoff' && flightState.phase === 'ground') {
                flightState.phase = 'takingOff';
                flightState.takeoffTimer = 0;
                const p = droneBody.translation();
                flightState.spawnPos = [p.x, p.y, p.z];
            }
            if (inputs.command === 'land' && flightState.phase === 'cruising') {
                flightState.phase = 'landing';
                flightState.landingSettleCount = 0;
            }
            if (inputs.command === 'toggleNav' && game.type === 'navigation') {
                game.navigationActive = !game.navigationActive;
            }

            // ── dispatch controller ──
            const autoInputs = controlAutoPhase(flightState, controlData, droneBody, config, dt, world);
            if (autoInputs) {
                // taking off or landing → auto control
                controlDrone(autoInputs, controlData, droneBody, trace, config, dt);
            } else if (flightState.phase === 'ground') {
                // on ground: allow manual control (user can just throttle up)
                controlDrone(inputs, controlData, droneBody, trace, config, dt);
                // if user gives strong throttle, jump straight to cruising
                if (inputs.throttleInput > 0.6) {
                    flightState.phase = 'cruising';
                }
            } else if (
                game.type === 'navigation' &&
                game.navigationActive &&
                !game.navState.reached &&
                flightState.phase === 'cruising'
            ) {
                // navigation autopilot
                const navInputs = controlNavigation(game.navState, controlData, droneBody, config, dt, world);
                controlDrone(navInputs, controlData, droneBody, trace, config, dt);
            } else if (flightState.phase === 'cruising') {
                // manual control
                controlDrone(inputs, controlData, droneBody, trace, config, dt);
            }
        }

        // ── W/S horizontal translation force (Mode 5) ──
        if (inputs && inputs.forwardInput && inputs.forwardInput !== 0) {
            const dRot = droneBody.rotation();
            const dQ = new THREE.Quaternion(dRot.x, dRot.y, dRot.z, dRot.w);
            // camera quaternion that maps drone-local to view direction
            const camQ = new THREE.Quaternion(-0.5, -0.5, 0.5, 0.5);
            const viewQ = dQ.clone().multiply(camQ);
            // forward in world space = viewQ rotates (0,0,-1)
            const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(viewQ);
            // project to horizontal plane (XY)
            const fwdH = new THREE.Vector3(fwd.x, fwd.y, 0);
            if (fwdH.length() > 0.01) {
                fwdH.normalize();
                const FORCE = 8.0;
                droneBody.addForce(
                    new RAPIER.Vector3(fwdH.x * FORCE * inputs.forwardInput, fwdH.y * FORCE * inputs.forwardInput, 0),
                    true,
                );
            }

        }

        const pos = droneBody.translation();
        const rot = droneBody.rotation();
        const dronePosition = new THREE.Vector3(pos.x, pos.y, pos.z)
        const droneQuaternion = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w)

        // ── telemetry ──
        const linvel = droneBody.linvel();
        const speed = Math.sqrt(linvel.x*linvel.x + linvel.y*linvel.y + linvel.z*linvel.z);

        // altitude along gravity from spawn
        const gv = config.map.gravity;
        const gNorm = Math.sqrt(gv[0]*gv[0]+gv[1]*gv[1]+gv[2]*gv[2]);
        const sp = config.map.spawn.position;
        const altitude = ((pos.x-sp[0])*gv[0] + (pos.y-sp[1])*gv[1] + (pos.z-sp[2])*gv[2]) / gNorm;

        // euler angles from quaternion
        const euler = new THREE.Euler().setFromQuaternion(droneQuaternion, 'ZYX');
        const rpyDeg = { roll: euler.x/deg, pitch: euler.y/deg, yaw: euler.z/deg };

        // distances to obstacles in 6 body-frame directions (rotated by drone + camera quaternion)
        const camQ = new THREE.Quaternion(-0.5, -0.5, 0.5, 0.5);
        const viewQ = droneQuaternion.clone().multiply(camQ);
        const bodyDirs = [
            { name:'前', v: new THREE.Vector3(0,0,-1).applyQuaternion(viewQ) },
            { name:'后', v: new THREE.Vector3(0,0,1).applyQuaternion(viewQ) },
            { name:'左', v: new THREE.Vector3(-1,0,0).applyQuaternion(viewQ) },
            { name:'右', v: new THREE.Vector3(1,0,0).applyQuaternion(viewQ) },
            { name:'上', v: new THREE.Vector3(0,-1,0).applyQuaternion(viewQ) },
            { name:'下', v: new THREE.Vector3(0,1,0).applyQuaternion(viewQ) },
        ];
        const distances = {};
        for (const dir of bodyDirs) {
            const ray = new RAPIER.Ray(
                { x:pos.x, y:pos.y, z:pos.z },
                { x:dir.v.x, y:dir.v.y, z:dir.v.z },
            );
            const hit = world.castRay(ray, 200, true, RAPIER.QueryFilterFlags.EXCLUDE_SENSORS, null, null, droneBody);
            distances[dir.name] = hit ? hit.timeOfImpact.toFixed(1) : '∞';
        }

        // collision: any direction < 0.25m (drone half-size + margin) = touching obstacle
        let isColliding = false;
        for (const d of Object.values(distances)) {
            if (d !== '∞' && parseFloat(d) < 0.25) { isColliding = true; break; }
        }

        // update camera
        fpv.update(config.aircraft.camera.firstPerson, dronePosition, droneQuaternion)
        tpv.update(config.aircraft.camera.thirdPerson, dronePosition, droneQuaternion)

        // update graphics
        const message = {
            type: "step",
            wallTime: performance.now(),
            ingameTime: ingameTime,
            drone: {
                xyz: dronePosition.toArray(),
                qxyzw: droneQuaternion.toArray()
            },
            fpvCamera: {
                xyz: fpv.camAnchor.getWorldPosition(new THREE.Vector3()).toArray(),
                qxyzw: fpv.camAnchor.getWorldQuaternion(new THREE.Quaternion()).toArray()
            },
            tpvCamera: {
                xyz: tpv.camAnchor.getWorldPosition(new THREE.Vector3()).toArray(),
                qxyzw: tpv.camAnchor.getWorldQuaternion(new THREE.Quaternion()).toArray()
            },
            flightPhase: flightState.phase,
            navActive: game.type === 'navigation' ? game.navigationActive : false,
            navReached: game.type === 'navigation' ? game.navState.reached : false,
            isColliding: isColliding,
            telemetry: {
                speed: speed.toFixed(1),
                altitude: altitude.toFixed(1),
                roll: rpyDeg.roll.toFixed(1),
                pitch: rpyDeg.pitch.toFixed(1),
                yaw: rpyDeg.yaw.toFixed(1),
                distances,
            },
            debug: { isActive: debug },
        }
        if (debug) {
            let buffers = []
            if (firstDebugMessage) {
                // since the map is likely massive, we only render it once
                const { vertices, colors } = world.debugRender(RAPIER.QueryFilterFlags.ONLY_FIXED)
                message.debug.fixedVertices = vertices.buffer
                message.debug.fixedColors = colors.buffer
                buffers.push(vertices.buffer)
                buffers.push(colors.buffer)
                firstDebugMessage = false
            }
            const { vertices, colors } = world.debugRender(RAPIER.QueryFilterFlags.EXCLUDE_FIXED)
            message.debug.dynamicVertices = vertices.buffer
            message.debug.dynamicColors = colors.buffer
            buffers.push(vertices.buffer)
            buffers.push(colors.buffer)
            postMessage(message, buffers)
        }
        else {
            postMessage(message)
        }

        const tNow = performance.now()
        tNextStep = tNextStep + dt * 1000
        tNextStep = Math.max(tNextStep, tNow - 100) // if we lag by more than 0.1s, we slow down
        setTimeout(stepPhysics, clamp(tNextStep - tNow, 0, Infinity))

        ingameTime += dt
    }
    setTimeout(stepPhysics, dt * 1000)


}
main()