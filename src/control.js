
import { THREE } from './three.js'
import { RAPIER } from './rapier.js'
import { inch, deg, clamp, lerp, rpyDegToQuat } from './utils.js'
import { calcDrownInertia } from './dronebody.js'
// import { readInputs } from './inputs.js'
// import { updateSound } from './sound.js'

export function initControls(config) {

    const g = Math.sqrt(
        Math.pow(config.map.gravity[0], 2)
        + Math.pow(config.map.gravity[1], 2)
        + Math.pow(config.map.gravity[2], 2)
    )

    const droneInertia = calcDrownInertia(config)

    const maxSingleMotorThrust = config.aircraft.maxCombinedThrust / 4
    const lever = config.aircraft.wheelbase * 0.5 * Math.SQRT1_2 // from center of mass to motor along x or y
    const maxMotorDrag = 0.07 * maxSingleMotorThrust * (config.aircraft.propSize * inch) // according to ChatGPT

    const controlData = {
        droneInertia,
        maxPitchRollTorque: 4 * lever * maxSingleMotorThrust,
        //maxYawTorque: 2 * (config.aircraft.wheelbase / 2) * maxMotorDrag,
        maxYawTorque: 4 * lever * maxSingleMotorThrust, // TODO: not realistic
        hoverThrottle: g * config.aircraft.mass / config.aircraft.maxCombinedThrust,
    }

    return controlData
}

export function controlDrone(inputs, controlData, droneBody, trace, config, dt) {

    const {
        droneInertia,
        maxPitchRollTorque,
        maxYawTorque,
        hoverThrottle
    } = controlData

    const [ixx, iyy, izz] = droneInertia

    trace.nextCheckpoint--
    if (trace.nextCheckpoint <= 0) {
        trace.nextCheckpoint = Math.ceil(0.5 / dt)
        trace.checkpoints.push([droneBody.translation(), droneBody.rotation()])
        if (trace.checkpoints.length > 3600) { trace.checkpoints.shift() }
    }

    const { throttleInput, rollInput, pitchInput, yawInput, reset } = inputs

    if (reset) {
        droneBody.resetForces(true);
        droneBody.resetTorques(true);
        droneBody.setLinvel({ x: 0, y: 0, z: 0 });
        droneBody.setAngvel({ x: 0, y: 0, z: 0 });
        if (trace.checkpoints.length >= 2) {
            const [t, r] = trace.checkpoints[trace.checkpoints.length - 2]
            droneBody.setTranslation(t);
            droneBody.setRotation(r);
            trace.checkpoints.pop()
            trace.checkpoints.pop()
        }
        else {
            droneBody.setTranslation(
                { x: config.map.spawn.position[0], y: config.map.spawn.position[1], z: config.map.spawn.position[2] }
            );
            droneBody.setRotation(rpyDegToQuat(config.map.spawn.rollPitchYaw))
        }
        return
    }

    // make it so that the drone hovers when the stick is in the middle
    let throttle;
    if (throttleInput <= 0.5) {
        throttle = throttleInput * 2 * hoverThrottle
    }
    else {
        throttle = hoverThrottle + (throttleInput * 2 - 1) * (1 - hoverThrottle)
    }

    const rotation = droneBody.rotation();
    const q = new THREE.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
    const qInv = q.clone().invert();

    const linvel = droneBody.linvel();
    const angvel = droneBody.angvel();
    const localLinearVelocity = new THREE.Vector3(linvel.x, linvel.y, linvel.z).applyQuaternion(qInv);
    const localAngularVelocity = new THREE.Vector3(angvel.x, angvel.y, angvel.z).applyQuaternion(qInv);

    let localTargetAngularVelocity = new THREE.Vector3(
        rollInput * config.aircraft.maxRollRate * deg,
        pitchInput * config.aircraft.maxPitchRate * deg,
        yawInput * config.aircraft.maxYawRate * deg,
    );

    if (config.aircraft.angleLimit < 180 || config.aircraft.stabilization > 0) {
        const gInDroneFrame = new THREE.Vector3(...config.map.gravity).normalize().applyQuaternion(qInv)
        const polarAngle = Math.acos(clamp(gInDroneFrame.z, -1, 1))

        let restoreAxis = new THREE.Vector3(gInDroneFrame.y, -gInDroneFrame.x, 0).normalize()
        let restoreAmount = 0

        if (config.aircraft.angleLimit < 180) {
            let factor = 0;
            if (
                polarAngle > config.aircraft.angleLimit * 0.9 * deg
                && polarAngle < config.aircraft.angleLimit * 1.1 * deg
            ) {
                factor = lerp(polarAngle, config.aircraft.angleLimit * 0.9 * deg, config.aircraft.angleLimit * deg * 1.1, 0, 1)
            }
            else if (polarAngle > config.aircraft.angleLimit * 1.1 * deg) {
                factor = 1
            }
            restoreAmount += clamp(localTargetAngularVelocity.dot(restoreAxis) * factor, 0, Infinity)
        }

        if (config.aircraft.stabilization > 0) {
            restoreAmount += polarAngle * config.aircraft.stabilization
        }

        localTargetAngularVelocity.addScaledVector(restoreAxis, -restoreAmount)
    }



    const localForce = new RAPIER.Vector3(0, 0, -throttle * config.aircraft.maxCombinedThrust);
    const localTorque = new RAPIER.Vector3(
        (localTargetAngularVelocity.x - localAngularVelocity.x) * ixx / config.aircraft.rollTimeConstant,
        (localTargetAngularVelocity.y - localAngularVelocity.y) * iyy / config.aircraft.pitchTimeConstant,
        (localTargetAngularVelocity.z - localAngularVelocity.z) * izz / config.aircraft.yawTimeConstant,
    );

    if (Math.abs(localTorque.x) > maxPitchRollTorque) { localTorque.x = maxPitchRollTorque * Math.sign(localTorque.x) }
    if (Math.abs(localTorque.y) > maxPitchRollTorque) { localTorque.y = maxPitchRollTorque * Math.sign(localTorque.y) }
    if (Math.abs(localTorque.z) > maxYawTorque) { localTorque.z = maxYawTorque * Math.sign(localTorque.z) }

    const thrustForce = new THREE.Vector3(localForce.x, localForce.y, localForce.z).applyQuaternion(q);
    const thrustTorque = new THREE.Vector3(localTorque.x, localTorque.y, localTorque.z).applyQuaternion(q);

    const [vx, vy, vz] = localLinearVelocity.toArray();
    const d = config.aircraft.dragForceOverSpeed
    const d2 = config.aircraft.dragForceOverSpeedSquared
    const dragForce = new THREE.Vector3(
        -d[0] * vx - d2[0] * vx * Math.abs(vx),
        -d[1] * vy - d2[1] * vy * Math.abs(vy),
        -d[2] * vz - d2[2] * vz * Math.abs(vz),
    ).applyQuaternion(q);

    droneBody.resetForces(true);
    droneBody.resetTorques(true);

    droneBody.addForce(new RAPIER.Vector3(thrustForce.x, thrustForce.y, thrustForce.z), true);
    droneBody.addTorque(new RAPIER.Vector3(thrustTorque.x, thrustTorque.y, thrustTorque.z), true);
    droneBody.addForce(new RAPIER.Vector3(dragForce.x, dragForce.y, dragForce.z), true);

}

// ── Flight State Machine ──────────────────────────────────────────────

export function initFlightState(config) {
    const hoverAlt = (config.autoFlight && config.autoFlight.takeoffHoverAltitude) || 3.0;
    return {
        phase: 'ground',           // 'ground' | 'takingOff' | 'cruising' | 'landing'
        takeoffTimer: 0,
        takeoffTargetAlt: hoverAlt,
        landingSettleCount: 0,
        spawnPos: [...config.map.spawn.position],
    };
}

/**
 * Compute artificial inputs for automated takeoff / landing phases.
 * Returns null when normal manual control should be used.
 */
export function controlAutoPhase(flightState, controlData, droneBody, config, dt, world) {

    if (flightState.phase === 'cruising') {
        return null; // no auto control
    }

    if (flightState.phase === 'ground') {
        // auto-detect manual lift-off or takeoff command → transition to cruising
        const alt = altitudeAboveSpawn(droneBody, flightState, config);
        if (alt > 0.5) {
            flightState.phase = 'cruising';
        }
        return null;
    }

    const pos = droneBody.translation();
    const grav = config.map.gravity;
    const gNorm = Math.sqrt(grav[0] * grav[0] + grav[1] * grav[1] + grav[2] * grav[2]);

    if (flightState.phase === 'takingOff') {
        flightState.takeoffTimer += dt;

        // smooth throttle ramp over 0.5 s
        const ramp = Math.min(flightState.takeoffTimer / 0.5, 1.0);

        const alt = altitudeAboveSpawn(droneBody, flightState, config);

        if (alt >= flightState.takeoffTargetAlt) {
            flightState.phase = 'cruising';
            return null;
        }

        // auto-level, climb with 1.3× hover throttle
        const climbThrottle = Math.min(controlData.hoverThrottle * 1.3 * ramp, 1.0);
        return {
            throttleInput: throttleToInput(climbThrottle, controlData.hoverThrottle),
            rollInput: 0, pitchInput: 0, yawInput: 0, reset: false,
        };
    }

    if (flightState.phase === 'landing') {
        // cast ray downward (gravity direction) to find ground
        const rayDir = {
            x: grav[0] / gNorm,
            y: grav[1] / gNorm,
            z: grav[2] / gNorm,
        };
        const ray = new RAPIER.Ray(
            { x: pos.x, y: pos.y, z: pos.z },
            rayDir,
        );
        const hit = world.castRay(ray, 20.0, true, RAPIER.QueryFilterFlags.EXCLUDE_SENSORS, null, null, droneBody);

        let groundDist = Infinity;
        if (hit) groundDist = hit.timeOfImpact;

        let desiredThrottle;
        if (groundDist < 0.5) {
            // final soft touchdown
            desiredThrottle = controlData.hoverThrottle * 0.3;
            flightState.landingSettleCount++;
            if (flightState.landingSettleCount > 30) {
                flightState.phase = 'ground';
                desiredThrottle = 0;
            }
        } else {
            desiredThrottle = controlData.hoverThrottle * 0.85;
            flightState.landingSettleCount = 0;
        }

        return {
            throttleInput: throttleToInput(desiredThrottle, controlData.hoverThrottle),
            rollInput: 0, pitchInput: 0, yawInput: 0, reset: false,
        };
    }

    return null;
}

/** Height above spawn along gravity direction (m) */
function altitudeAboveSpawn(droneBody, flightState, config) {
    const pos = droneBody.translation();
    const g = config.map.gravity;
    const gNorm = Math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    return Math.abs(
        (pos.x - flightState.spawnPos[0]) * g[0] / gNorm +
        (pos.y - flightState.spawnPos[1]) * g[1] / gNorm +
        (pos.z - flightState.spawnPos[2]) * g[2] / gNorm,
    );
}

/** Convert a desired thrust fraction [0,1] to throttleInput [0,1] */
function throttleToInput(desiredThrottle, hoverThrottle) {
    if (desiredThrottle <= 0) return 0;
    if (desiredThrottle <= hoverThrottle) {
        return desiredThrottle / (2 * hoverThrottle);
    } else {
        return (desiredThrottle - hoverThrottle) / (2 * (1 - hoverThrottle)) + 0.5;
    }
}

// ── Navigation Controller ─────────────────────────────────────────────

export function initNavState(config) {
    const nav = config.map.mission.navigation;
    return {
        startPos: new THREE.Vector3(...(nav.startPosition || config.map.spawn.position)),
        endPos: new THREE.Vector3(...nav.endPosition),
        reached: false,
    };
}

/**
 * Simple straight-line navigation:
 *  1. Yaw toward target
 *  2. Fly forward (pitch + throttle proportional to distance)
 *  3. Obstacle ahead → climb (increase throttle, reduce pitch) to go OVER it
 */
export function controlNavigation(navState, controlData, droneBody, config, dt, world) {

    const pos = droneBody.translation();
    const rot = droneBody.rotation();
    const q = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w);
    const qInv = q.clone().invert();

    const target = navState.endPos;
    const toTarget = new THREE.Vector3(target.x - pos.x, target.y - pos.y, target.z - pos.z);
    const distToTarget = toTarget.length();

    // ── reached target? ──
    if (distToTarget < 2.0) {
        navState.reached = true;
        return { throttleInput: 0.5, rollInput: 0, pitchInput: 0, yawInput: 0, reset: false };
    }

    // ── target direction in world & drone-local frames ──
    const toTargetDir = toTarget.clone().normalize();

    // Drone forward direction in world (drone -Z = forward)
    const droneFwd = new THREE.Vector3(0, 0, -1).applyQuaternion(q).normalize();

    // ── Yaw: signed angle between drone forward and target (horizontal only) ──
    // Project both onto horizontal plane (XY, since gravity = +Z)
    const fwdH = new THREE.Vector3(droneFwd.x, droneFwd.y, 0);
    const tgtH = new THREE.Vector3(toTargetDir.x, toTargetDir.y, 0);
    let yawInput = 0;
    if (fwdH.length() > 0.001 && tgtH.length() > 0.001) {
        fwdH.normalize();
        tgtH.normalize();
        const cross = fwdH.x * tgtH.y - fwdH.y * tgtH.x; // z-component of cross product
        yawInput = clamp(cross * 3.0, -1, 1);  // P-controller with gain 3
    }

    // ── Obstacle detection: ray-cast forward ──
    let obstacleClose = false;
    const fwdRay = new RAPIER.Ray(
        { x: pos.x, y: pos.y, z: pos.z },
        { x: droneFwd.x, y: droneFwd.y, z: droneFwd.z },
    );
    const hit = world.castRay(fwdRay, 15.0, true, RAPIER.QueryFilterFlags.EXCLUDE_SENSORS, null, null, droneBody);
    if (hit && hit.timeOfImpact < 6.0) {
        obstacleClose = true;
    }

    // ── Assemble control inputs ──
    let pitchInput, throttleInput;

    if (obstacleClose) {
        // Climb over: nose up (pitch back), extra throttle
        pitchInput = -0.4;
        throttleInput = 0.7;  // above hover
    } else {
        // Normal cruise: pitch forward proportional to distance, throttle above hover
        pitchInput = clamp(distToTarget / 40.0, 0.1, 0.6);
        const speedFrac = Math.min(distToTarget / 15.0, 1.0);
        throttleInput = 0.5 + 0.25 * speedFrac;
    }

    const rollInput = 0;

    return { throttleInput, rollInput, pitchInput, yawInput, reset: false };
}
