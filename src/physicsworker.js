import { RAPIER } from './rapier.js'
import { createDroneBody, updateDroneBody } from './dronebody.js'
import { dt } from './config.js'
import { clamp, deg, rpyDegToQuat } from './utils.js'
import { initControls, controlDrone, initFlightState, controlAutoPhase, initNavState, controlNavigation } from './control.js'
import { RapierEnvironmentAdapter } from './navigation/adapters/RapierEnvironmentAdapter.js'
import { AvoidanceSupervisor } from './navigation/AvoidanceSupervisor.js'
import { ThreatAssessor } from './navigation/ThreatAssessor.js'
import { HperD3qnClient } from './navigation/avoidance/HperD3qnClient.js'

import { THREE } from './three.js'
import { createCameraAnchor } from './camera.js'

async function main() {

    console.log("worker started")

    await RAPIER.init();
    postMessage("ready")

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

    const config = await receive("config")
    const configListeners = []
    function onConfigUpdate(callback) {
        configListeners.push(callback)
        callback()
    }
    function triggerConfigUpdate() {
        for (const cb of configListeners) {
            cb()
        }
    }

    const world = new RAPIER.World({ x: config.map.gravity[0], y: config.map.gravity[1], z: config.map.gravity[2] });
    onConfigUpdate(() => { world.timestep = dt * config.map.timeScale })

    const droneBody = createDroneBody(config, world)
    onConfigUpdate(() => { updateDroneBody(droneBody, config) })

    const flightState = initFlightState(config)
    const testScene = config.navigation?.testScene ?? {}
    const testStaticObstacles = []
    const testDynamicObstacles = []
    if (testScene.enabled) {
        for (const obstacle of testScene.staticObstacles ?? []) {
            const body = world.createRigidBody(
                new RAPIER.RigidBodyDesc(RAPIER.RigidBodyType.Fixed)
                    .setTranslation(...obstacle.position)
            )
            world.createCollider(
                RAPIER.ColliderDesc.cuboid(
                    obstacle.size[0] * 0.5,
                    obstacle.size[1] * 0.5,
                    obstacle.size[2] * 0.5,
                ),
                body,
            )
            testStaticObstacles.push({
                id: obstacle.id,
                type: 'building',
                position: [...obstacle.position],
                velocity: [0, 0, 0],
                size: Math.max(obstacle.size[0], obstacle.size[1], obstacle.size[2]) * 0.5,
                footprintHalfExtents: [
                    obstacle.size[0] * 0.5,
                    obstacle.size[1] * 0.5,
                    obstacle.size[2] * 0.5,
                ],
                dynamic: false,
                confidence: 1.0,
            })
        }
        for (const obstacle of testScene.dynamicObstacles ?? []) {
            const body = world.createRigidBody(
                new RAPIER.RigidBodyDesc(RAPIER.RigidBodyType.KinematicPositionBased)
                    .setTranslation(...obstacle.position)
            )
            world.createCollider(
                RAPIER.ColliderDesc.cuboid(
                    obstacle.size[0] * 0.5,
                    obstacle.size[1] * 0.5,
                    obstacle.size[2] * 0.5,
                ),
                body,
            )
            testDynamicObstacles.push({
                id: obstacle.id,
                body,
                size: obstacle.size,
                path: obstacle.path,
                speed: obstacle.speed ?? 1.5,
                phase: obstacle.phase ?? 0,
                position: [...obstacle.position],
                velocity: [0, 0, 0],
                dynamic: true,
                type: 'uav',
                confidence: 1.0,
            })
        }
    }

    const terrain = await receive("terrain")
    const colliderNames = new Map()
    const terrainPlannerObstacles = []
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

        if (!part.isSensor) {
            const vertices = new Float32Array(part.vertices)
            let minX = Infinity
            let minY = Infinity
            let minZ = Infinity
            let maxX = -Infinity
            let maxY = -Infinity
            let maxZ = -Infinity
            for (let i = 0; i < vertices.length; i += 3) {
                const x = vertices[i]
                const y = vertices[i + 1]
                const z = vertices[i + 2]
                if (x < minX) minX = x
                if (y < minY) minY = y
                if (z < minZ) minZ = z
                if (x > maxX) maxX = x
                if (y > maxY) maxY = y
                if (z > maxZ) maxZ = z
            }

            const sizeX = maxX - minX
            const sizeY = maxY - minY
            const sizeZ = maxZ - minZ
            const maxHorizontal = Math.max(sizeX, sizeY)
            const isLikelyGround = sizeZ < 1.0 && maxHorizontal > 20.0
            const isTiny = maxHorizontal < 0.35 && sizeZ < 0.35

            if (!isLikelyGround && !isTiny && Number.isFinite(sizeX) && Number.isFinite(sizeY) && Number.isFinite(sizeZ)) {
                terrainPlannerObstacles.push({
                    id: `terrain-${part.name}`,
                    type: 'building',
                    position: [
                        (minX + maxX) * 0.5,
                        (minY + maxY) * 0.5,
                        (minZ + maxZ) * 0.5,
                    ],
                    velocity: [0, 0, 0],
                    size: Math.max(sizeX, sizeY, sizeZ) * 0.5,
                    footprintHalfExtents: [
                        sizeX * 0.5,
                        sizeY * 0.5,
                        sizeZ * 0.5,
                    ],
                    dynamic: false,
                    confidence: 0.9,
                })
            }
        }
    }

    const mockScene = new THREE.Scene()
    const fpv = createCameraAnchor(config.aircraft.camera.firstPerson)
    mockScene.add(fpv.camTarget)
    const tpv = createCameraAnchor(config.aircraft.camera.thirdPerson, true, world, droneBody)
    mockScene.add(tpv.camTarget)

    let controlData
    onConfigUpdate(() => { controlData = initControls(config, dt) })
    const environmentAdapter = new RapierEnvironmentAdapter(config, RAPIER)
    const threatAssessor = new ThreatAssessor(config)
    const avoidanceSupervisor = new AvoidanceSupervisor(config)
    const d3qnClient = new HperD3qnClient(config)
    const plannerWorker = new Worker(new URL('./workers/globalPlanner.worker.js', import.meta.url), { type: 'module' })
    let latestGlobalPath = []
    let lastPlannerRequestTime = -Infinity
    let lastPlannerGoal = null
    let plannerGeneration = 0
    plannerWorker.postMessage({ type: 'init', config })
    plannerWorker.addEventListener('message', (e) => {
        if (e.data?.type !== 'planResult') {
            return
        }
        if ((e.data.generation ?? 0) !== plannerGeneration) {
            return
        }
        latestGlobalPath = Array.isArray(e.data.path) ? e.data.path : []
        avoidanceSupervisor.acceptGlobalPath(latestGlobalPath)
    })
    let trace = {
        checkpoints: [],
        nextCheckpoint: Math.ceil(0.5 / dt)
    }
    let inputs = null
    let appliedInputs = null
    let debug = false
    let firstDebugMessage = true

    function toVec3Array(v) {
        if (!v) return [0, 0, 0]
        if (Array.isArray(v)) return [v[0], v[1], v[2]]
        return [v.x, v.y, v.z]
    }

    function wrapAngle(angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle))
    }

    function horizontalDistance(a, b) {
        return Math.hypot(a[0] - b[0], a[1] - b[1])
    }

    function makeLevelBodyQuaternionFromHeading(heading) {
        const worldUp = new THREE.Vector3(0, 0, -1)
        const forward = new THREE.Vector3(Math.cos(heading), Math.sin(heading), 0)
        if (forward.lengthSq() < 1e-9) {
            forward.set(0, 1, 0)
        }
        forward.normalize()

        const right = new THREE.Vector3().crossVectors(forward, worldUp).normalize()
        const correctedUp = new THREE.Vector3().crossVectors(right, forward).normalize()
        const back = forward.clone().multiplyScalar(-1)

        const viewMatrix = new THREE.Matrix4().makeBasis(right, correctedUp, back)
        const viewQuaternion = new THREE.Quaternion().setFromRotationMatrix(viewMatrix)

        const cameraToBody = new THREE.Quaternion(-0.5, -0.5, 0.5, 0.5).invert()
        return viewQuaternion.multiply(cameraToBody)
    }

    function blendVelocity(baseVelocity, overrideVelocity, weight, horizontalLimit, verticalLimit) {
        const blended = [
            baseVelocity[0] * (1 - weight) + overrideVelocity[0] * weight,
            baseVelocity[1] * (1 - weight) + overrideVelocity[1] * weight,
            baseVelocity[2] * (1 - weight) + overrideVelocity[2] * weight,
        ]
        const horizontal = new THREE.Vector3(blended[0], blended[1], 0)
        if (horizontal.length() > horizontalLimit) {
            horizontal.normalize().multiplyScalar(horizontalLimit)
        }
        return [
            horizontal.x,
            horizontal.y,
            clamp(blended[2], -verticalLimit, verticalLimit),
        ]
    }

    function limitVelocity(velocity, horizontalLimit, verticalLimit) {
        const horizontal = new THREE.Vector3(velocity[0], velocity[1], 0)
        if (horizontal.length() > horizontalLimit) {
            horizontal.normalize().multiplyScalar(horizontalLimit)
        }
        return [
            horizontal.x,
            horizontal.y,
            clamp(velocity[2], -verticalLimit, verticalLimit),
        ]
    }

    function magnitude3(vec) {
        return Math.hypot(vec[0], vec[1], vec[2])
    }

    function obstacleHalfExtents(obstacle) {
        if (Array.isArray(obstacle?.footprintHalfExtents) && obstacle.footprintHalfExtents.length >= 2) {
            return {
                x: Math.max(obstacle.footprintHalfExtents[0] ?? 0, 0),
                y: Math.max(obstacle.footprintHalfExtents[1] ?? 0, 0),
                z: Math.max(obstacle.footprintHalfExtents[2] ?? 0, 0),
            }
        }
        const radius = Math.max(obstacle?.size ?? 0, 0)
        return { x: radius, y: radius, z: radius }
    }

    function obstacleHorizontalClearance(position, obstacle) {
        const extents = obstacleHalfExtents(obstacle)
        const dx = Math.max(Math.abs(position[0] - obstacle.position[0]) - extents.x, 0)
        const dy = Math.max(Math.abs(position[1] - obstacle.position[1]) - extents.y, 0)
        return Math.hypot(dx, dy)
    }

    function closestPointOnObstacleXY(position, obstacle, z = 0) {
        const extents = obstacleHalfExtents(obstacle)
        return new THREE.Vector3(
            clamp(position[0], obstacle.position[0] - extents.x, obstacle.position[0] + extents.x),
            clamp(position[1], obstacle.position[1] - extents.y, obstacle.position[1] + extents.y),
            z,
        )
    }

    function obstacleProjectedExtent(obstacle, axis) {
        const extents = obstacleHalfExtents(obstacle)
        return Math.abs(axis.x) * extents.x + Math.abs(axis.y) * extents.y
    }

    function isRayObstacle(obstacle) {
        return typeof obstacle?.id === 'string' && obstacle.id.startsWith('ray-')
    }

    function isMassiveTerrainObstacle(obstacle, maxSpan = 45.0, maxHeight = 24.0) {
        if (typeof obstacle?.id !== 'string' || !obstacle.id.startsWith('terrain-')) {
            return false
        }
        const extents = obstacleHalfExtents(obstacle)
        const span = Math.max(extents.x * 2, extents.y * 2)
        const height = extents.z * 2
        return span > maxSpan || height > maxHeight
    }

    function findVerticalEscapeCandidate(observation, position, triggerDistance, maxWidth = Infinity, maxHeight = Infinity) {
        let best = null
        let bestDistance = Infinity
        for (const obstacle of observation.staticObstacles ?? []) {
            if (isRayObstacle(obstacle)) {
                continue
            }
            if (isMassiveTerrainObstacle(obstacle)) {
                continue
            }
            const extents = obstacleHalfExtents(obstacle)
            const width = Math.max(extents.x * 2, extents.y * 2)
            const height = extents.z * 2
            if (width > maxWidth || height > maxHeight) {
                continue
            }
            const horizontalDistance = obstacleHorizontalClearance(position, obstacle)
            if (horizontalDistance > triggerDistance) {
                continue
            }
            if (horizontalDistance < bestDistance) {
                bestDistance = horizontalDistance
                best = obstacle
            }
        }
        return best
    }

    function computeRayEscapeDirection(observation, currentYaw, goal = null) {
        const rayDistances = observation.rayDistances ?? {}
        const headingForward = new THREE.Vector3(Math.cos(currentYaw), Math.sin(currentYaw), 0)
        const headingRight = new THREE.Vector3(-Math.sin(currentYaw), Math.cos(currentYaw), 0)
        const headingLeft = headingRight.clone().multiplyScalar(-1)
        const headingBack = headingForward.clone().multiplyScalar(-1)
        const worldUp = new THREE.Vector3(0, 0, -1)
        const ego = observation?.ego?.position ?? [0, 0, 0]
        const goalVector = goal
            ? new THREE.Vector3(goal[0] - ego[0], goal[1] - ego[1], 0)
            : headingForward.clone()
        const goalDirection = goalVector.lengthSq() > 1e-9
            ? goalVector.normalize()
            : headingForward.clone()

        const frontClear = Math.min(
            rayDistances.front ?? Infinity,
            rayDistances.frontLeft ?? rayDistances.front ?? Infinity,
            rayDistances.frontRight ?? rayDistances.front ?? Infinity,
        )
        const backClear = Math.min(
            rayDistances.back ?? Infinity,
            rayDistances.backLeft ?? rayDistances.back ?? Infinity,
            rayDistances.backRight ?? rayDistances.back ?? Infinity,
        )
        const leftClear = Math.min(
            rayDistances.left ?? Infinity,
            rayDistances.frontLeft ?? rayDistances.left ?? Infinity,
            rayDistances.leftUp ?? rayDistances.left ?? Infinity,
        )
        const rightClear = Math.min(
            rayDistances.right ?? Infinity,
            rayDistances.frontRight ?? rayDistances.right ?? Infinity,
            rayDistances.rightUp ?? rayDistances.right ?? Infinity,
        )
        const upClear = Math.min(
            rayDistances.up ?? Infinity,
            rayDistances.frontUp ?? rayDistances.up ?? Infinity,
            rayDistances.leftUp ?? rayDistances.up ?? Infinity,
            rayDistances.rightUp ?? rayDistances.up ?? Infinity,
        )

        const frontLeft = headingForward.clone().addScaledVector(headingLeft, 0.92).normalize()
        const frontRight = headingForward.clone().addScaledVector(headingRight, 0.92).normalize()
        const backLeft = headingBack.clone().addScaledVector(headingLeft, 0.85).normalize()
        const backRight = headingBack.clone().addScaledVector(headingRight, 0.85).normalize()

        const candidates = [
            { name: 'front', direction: headingForward.clone(), clearance: rayDistances.front ?? 0 },
            { name: 'frontLeft', direction: frontLeft, clearance: Math.min(rayDistances.frontLeft ?? 0, rayDistances.left ?? rayDistances.frontLeft ?? 0) },
            { name: 'frontRight', direction: frontRight, clearance: Math.min(rayDistances.frontRight ?? 0, rayDistances.right ?? rayDistances.frontRight ?? 0) },
            { name: 'left', direction: headingLeft.clone(), clearance: rayDistances.left ?? 0 },
            { name: 'right', direction: headingRight.clone(), clearance: rayDistances.right ?? 0 },
            { name: 'up', direction: worldUp.clone(), clearance: rayDistances.up ?? 0 },
            { name: 'backLeft', direction: backLeft, clearance: Math.min(rayDistances.backLeft ?? 0, rayDistances.left ?? rayDistances.backLeft ?? 0) },
            { name: 'backRight', direction: backRight, clearance: Math.min(rayDistances.backRight ?? 0, rayDistances.right ?? rayDistances.backRight ?? 0) },
            { name: 'back', direction: headingBack.clone(), clearance: rayDistances.back ?? 0 },
        ]

        const bestSideClear = Math.max(leftClear, rightClear)
        const upPreferred =
            upClear > Math.max(frontClear, bestSideClear) + 1.5 &&
            bestSideClear < 2.4

        let best = null
        let bestScore = -Infinity
        for (const candidate of candidates) {
            const horizontal = new THREE.Vector3(candidate.direction.x, candidate.direction.y, 0)
            const horizontalDirection = horizontal.lengthSq() > 1e-9 ? horizontal.normalize() : null
            const goalAlignment = horizontalDirection ? horizontalDirection.dot(goalDirection) : 0
            const upwardBias = Math.max(-candidate.direction.z, 0)
            const sideBias =
                candidate.name === 'frontLeft' || candidate.name === 'frontRight'
                    ? 0.8
                    : candidate.name === 'left' || candidate.name === 'right'
                        ? 0.55
                        : 0
            const backPenalty =
                candidate.name === 'back'
                    ? 2.8
                    : candidate.name === 'backLeft' || candidate.name === 'backRight'
                        ? 1.4
                        : 0

            let score =
                candidate.clearance * 1.0 +
                goalAlignment * 2.4 +
                sideBias -
                backPenalty

            if (candidate.name === 'up') {
                score += upPreferred ? 2.2 + upwardBias * 1.6 : -2.6
            } else if (frontClear < 1.6 && (candidate.name === 'frontLeft' || candidate.name === 'frontRight')) {
                score += 1.0
            } else if (frontClear < 1.1 && (candidate.name === 'left' || candidate.name === 'right')) {
                score += 0.7
            }

            if (candidate.clearance < 0.35) {
                score -= 6.0
            }

            if (score > bestScore) {
                bestScore = score
                best = candidate
            }
        }

        const direction = best?.direction?.clone?.() ?? new THREE.Vector3()
        if (direction.lengthSq() < 1e-9) {
            direction.copy(headingForward)
        }

        if (best && best.name !== 'up' && !best.name.startsWith('back')) {
            direction.addScaledVector(goalDirection, 0.9)
        } else if (best && (best.name === 'backLeft' || best.name === 'backRight')) {
            direction.addScaledVector(goalDirection, 0.35)
        }

        if (upPreferred && frontClear < 0.9 && bestSideClear < 1.4) {
            direction.addScaledVector(worldUp, 0.85)
        }

        if (direction.lengthSq() < 1e-9) {
            direction.copy(goalDirection)
        }

        return direction
    }

    function enforceGoalProgress(desiredVelocity, position, goal, minForwardSpeed) {
        if (minForwardSpeed <= 0) {
            return desiredVelocity
        }
        const goalForward = new THREE.Vector3(goal[0] - position[0], goal[1] - position[1], 0)
        if (goalForward.lengthSq() < 1e-9) {
            return desiredVelocity
        }
        goalForward.normalize()
        const forwardSpeed = desiredVelocity[0] * goalForward.x + desiredVelocity[1] * goalForward.y
        if (forwardSpeed >= minForwardSpeed) {
            return desiredVelocity
        }
        const boost = minForwardSpeed - forwardSpeed
        return [
            desiredVelocity[0] + goalForward.x * boost,
            desiredVelocity[1] + goalForward.y * boost,
            desiredVelocity[2],
        ]
    }

    function computeEmergencyOverrideVelocity(navState, observation, currentYaw, goal, maxCruiseSpeed, maxVerticalSpeed) {
        const auto = config.navigation?.autopilot ?? {}
        const position = observation.ego.position
        const baseDirection = computeRayEscapeDirection(observation, currentYaw, goal)
        if (baseDirection.lengthSq() < 1e-9) {
            return computeStuckEscapeVelocity(navState, observation, goal, maxCruiseSpeed, maxVerticalSpeed)
        }

        const escapeSpeed = auto.emergencyOverrideSpeed ?? Math.min(maxCruiseSpeed, 2.8)
        const direction = baseDirection.normalize()
        let desiredVelocity = [
            direction.x * escapeSpeed,
            direction.y * escapeSpeed,
            clamp(direction.z * escapeSpeed, -maxVerticalSpeed, maxVerticalSpeed),
        ]

        desiredVelocity = enforceGoalProgress(
            desiredVelocity,
            position,
            goal,
            auto.emergencyOverrideMinForwardSpeed ?? 0.75,
        )

        if (Math.abs(desiredVelocity[2]) < 0.15) {
            desiredVelocity[2] = clamp(
                (navState.cruiseZ - position[2]) * (auto.verticalCruiseGain ?? 0.55),
                -maxVerticalSpeed,
                maxVerticalSpeed,
            )
        }

        return limitVelocity(desiredVelocity, maxCruiseSpeed, maxVerticalSpeed)
    }

    function computeAdaptiveCruiseZ(observation, navState, goal, horizontalGoalDistance) {
        const auto = config.navigation?.autopilot ?? {}
        const position = observation.ego.position
        const altitudeTrackGoalRadius = auto.altitudeTrackGoalRadius
            ?? Math.max((auto.verticalDescentLockRadius ?? 1.2) * 2.5, 3.0)
        const fixedCruiseZ = Number.isFinite(navState?.cruiseZ)
            ? navState.cruiseZ
            : position[2]
        const phase = navState?.phase ?? 'cruise'

        if (
            phase === 'arrive' ||
            phase === 'hold' ||
            horizontalGoalDistance <= altitudeTrackGoalRadius
        ) {
            return Number.isFinite(goal?.[2]) ? goal[2] : fixedCruiseZ
        }

        return fixedCruiseZ
    }

    function nearestPathWaypointIndex(path, position) {
        if (!path || path.length === 0) {
            return -1
        }

        let bestIndex = 0
        let bestDistance = Infinity
        for (let i = 0; i < path.length; i += 1) {
            const waypoint = path[i]
            const distance = Math.hypot(
                waypoint[0] - position[0],
                waypoint[1] - position[1],
                waypoint[2] - position[2],
            )
            if (distance < bestDistance) {
                bestDistance = distance
                bestIndex = i
            }
        }
        return bestIndex
    }

    function targetAlongPath(path, position, lookaheadDistance) {
        if (!path || path.length === 0) {
            return null
        }
        if (path.length === 1) {
            return path[0]
        }

        const nearestIndex = nearestPathWaypointIndex(path, position)
        let remaining = Math.max(lookaheadDistance, 0)
        let current = path[Math.max(nearestIndex, 0)]
        for (let i = Math.max(nearestIndex, 0); i < path.length - 1; i += 1) {
            const next = path[i + 1]
            const segmentLength = Math.hypot(
                next[0] - current[0],
                next[1] - current[1],
                next[2] - current[2],
            )
            if (segmentLength >= remaining && segmentLength > 1e-6) {
                const t = remaining / segmentLength
                return [
                    current[0] + (next[0] - current[0]) * t,
                    current[1] + (next[1] - current[1]) * t,
                    current[2] + (next[2] - current[2]) * t,
                ]
            }
            remaining -= segmentLength
            current = next
        }
        return path[path.length - 1]
    }

    function computePathDirection(path, position, goal) {
        const fallback = new THREE.Vector3(goal[0] - position[0], goal[1] - position[1], 0)
        if (fallback.lengthSq() < 1e-9) {
            fallback.set(1, 0, 0)
        } else {
            fallback.normalize()
        }

        if (!path || path.length === 0) {
            return fallback
        }

        const nearestIndex = nearestPathWaypointIndex(path, position)
        const nextIndex = Math.min(Math.max(nearestIndex, 0) + 1, path.length - 1)
        const nextWaypoint = path[nextIndex]
        const tangent = new THREE.Vector3(
            nextWaypoint[0] - position[0],
            nextWaypoint[1] - position[1],
            0,
        )
        if (tangent.lengthSq() < 1e-9) {
            return fallback
        }
        return tangent.normalize()
    }

    function nearestBlockingObstacle(observation, pathDirection, maxDistance = 8.0) {
        const ego = observation.ego.position
        let best = null
        let bestScore = -Infinity
        for (const obstacle of observation.staticObstacles ?? []) {
            if (isMassiveTerrainObstacle(obstacle)) {
                continue
            }
            const closestPoint = closestPointOnObstacleXY(ego, obstacle, ego[2])
            let dx = closestPoint.x - ego[0]
            let dy = closestPoint.y - ego[1]
            if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
                dx = obstacle.position[0] - ego[0]
                dy = obstacle.position[1] - ego[1]
            }
            const distance = Math.max(obstacleHorizontalClearance(ego, obstacle), 0)
            if (distance > maxDistance) {
                continue
            }
            const direction = new THREE.Vector3(dx, dy, 0)
            if (direction.lengthSq() < 1e-9) {
                continue
            }
            direction.normalize()
            const alignment = pathDirection.dot(direction)
            if (alignment < 0.25) {
                continue
            }
            const score = alignment * 0.65 + (1 - distance / maxDistance) * 0.35
            if (score > bestScore) {
                best = obstacle
                bestScore = score
            }
        }
        return best
    }

    function synthesizeFallbackPath(observation, navState = null) {
        const auto = config.navigation?.autopilot ?? {}
        const position = observation.ego.position
        const goal = observation.goal
        const cruiseZ = Number.isFinite(navState?.cruiseZ) ? navState.cruiseZ : position[2]
        const pathDirection = computePathDirection([], position, goal)
        const blocker = nearestBlockingObstacle(
            observation,
            pathDirection,
            auto.fallbackPathObstacleDistance ?? 10.0,
        )

        const start = [position[0], position[1], cruiseZ]
        const finish = [goal[0], goal[1], cruiseZ]
        if (!blocker) {
            return [start, finish]
        }

        let lateralSign = (navState?.escapeAttemptCount ?? 0) % 2 === 0 ? 1 : -1
        const relX = blocker.position[0] - position[0]
        const relY = blocker.position[1] - position[1]
        const cross = pathDirection.x * relY - pathDirection.y * relX
        lateralSign = cross >= 0 ? -1 : 1

        const lateral = new THREE.Vector3(-pathDirection.y, pathDirection.x, 0)
            .normalize()
            .multiplyScalar(lateralSign)

        const detourLateralDistance =
            Math.max(obstacleProjectedExtent(blocker, lateral), 1.0) + (auto.fallbackPathLateralMargin ?? 6.0)
        const detourForwardDistance =
            Math.max(obstacleProjectedExtent(blocker, pathDirection), 1.0) + (auto.fallbackPathForwardDistance ?? 5.0)
        const obstacleCenter = new THREE.Vector3(blocker.position[0], blocker.position[1], cruiseZ)
        const startVec = new THREE.Vector3(position[0], position[1], cruiseZ)
        const goalVec = new THREE.Vector3(goal[0], goal[1], cruiseZ)

        const detourEntry = startVec
            .clone()
            .addScaledVector(pathDirection, detourForwardDistance * 0.6)
            .addScaledVector(lateral, detourLateralDistance)

        const detourExit = obstacleCenter
            .clone()
            .addScaledVector(pathDirection, detourForwardDistance)
            .addScaledVector(lateral, detourLateralDistance)

        const detourClear = detourExit
            .clone()
            .addScaledVector(pathDirection, Math.max((auto.fallbackPathForwardDistance ?? 5.0) * 0.8, 3.0))

        return [
            [startVec.x, startVec.y, startVec.z],
            [detourEntry.x, detourEntry.y, detourEntry.z],
            [detourExit.x, detourExit.y, detourExit.z],
            [detourClear.x, detourClear.y, detourClear.z],
            [goalVec.x, goalVec.y, goalVec.z],
        ]
    }

    function updateNavigationProgressWatchdog(navState, observation, setpoint, goalDistance, horizontalGoalDistance, minObstacleDistance) {
        const auto = config.navigation?.autopilot ?? {}
        const now = observation.timestamp
        const horizontalSpeed = Math.hypot(observation.ego.linearVelocity[0], observation.ego.linearVelocity[1])
        const progressEpsilon = auto.stuckProgressEpsilon ?? 0.03
        const stuckSpeedThreshold = auto.stuckSpeedThreshold ?? 0.45
        const stuckDetectionSeconds = auto.stuckDetectionSeconds ?? 1.6
        const stuckObstacleDistance = auto.stuckObstacleDistance ?? 7.5
        const goalFarDistance = auto.stuckGoalDistance ?? 4.0

        const previousGoalDistance = navState.lastGoalDistance ?? goalDistance
        const distanceProgress = previousGoalDistance - goalDistance
        const pathBlocked =
            minObstacleDistance < stuckObstacleDistance ||
            ['apf', 'd3qn', 'recovery'].includes(setpoint?.source)

        const canBeStuck =
            navState.phase === 'cruise' &&
            goalDistance > goalFarDistance &&
            horizontalGoalDistance > (auto.verticalDescentLockRadius ?? 1.2)

        if (canBeStuck && pathBlocked && distanceProgress < progressEpsilon && horizontalSpeed < stuckSpeedThreshold) {
            navState.stuckTime = (navState.stuckTime ?? 0) + dt
        } else {
            navState.stuckTime = Math.max((navState.stuckTime ?? 0) - dt * 2, 0)
        }

        if ((navState.stuckTime ?? 0) >= stuckDetectionSeconds) {
            navState.stuckTime = 0
            navState.escapeUntil = now + (auto.stuckEscapeDuration ?? 1.2)
            navState.forceReplan = true
            navState.escapeAttemptCount = (navState.escapeAttemptCount ?? 0) + 1
        }

        navState.lastGoalDistance = goalDistance
    }

    function computeStuckEscapeVelocity(navState, observation, goal, maxCruiseSpeed, maxVerticalSpeed) {
        const auto = config.navigation?.autopilot ?? {}
        const path = latestGlobalPath.length > 0 ? latestGlobalPath : observation.globalPath
        const position = observation.ego.position
        const pathDirection = computePathDirection(path, position, goal)
        const obstacle = nearestBlockingObstacle(
            observation,
            pathDirection,
            auto.stuckObstacleDistance ?? 7.5,
        )

        let lateralSign = (navState.escapeAttemptCount ?? 0) % 2 === 0 ? 1 : -1
        if (obstacle) {
            const relX = obstacle.position[0] - position[0]
            const relY = obstacle.position[1] - position[1]
            const cross = pathDirection.x * relY - pathDirection.y * relX
            lateralSign = cross >= 0 ? -1 : 1
            if ((navState.escapeAttemptCount ?? 0) > 1) {
                lateralSign *= -1
            }
        }

        const lateral = new THREE.Vector3(-pathDirection.y, pathDirection.x, 0).multiplyScalar(lateralSign)
        const forwardBias = auto.stuckForwardBias ?? 1.15
        const lateralBias = auto.stuckLateralBias ?? 1.35
        const altitudeBias = clamp((navState.cruiseZ - position[2]) * 0.35, -maxVerticalSpeed, maxVerticalSpeed)
        const escapeVector = pathDirection
            .clone()
            .multiplyScalar(forwardBias)
            .add(lateral.multiplyScalar(lateralBias))

        if (escapeVector.lengthSq() < 1e-9) {
            escapeVector.set(1, 0, 0)
        }
        escapeVector.normalize().multiplyScalar(auto.stuckEscapeSpeed ?? Math.min(maxCruiseSpeed, 2.6))

        return limitVelocity(
            [escapeVector.x, escapeVector.y, Math.min(altitudeBias, 0)],
            maxCruiseSpeed,
            maxVerticalSpeed,
        )
    }

    function applyBasicNavigationAutopilot(navState, observation, setpoint) {
        const auto = config.navigation?.autopilot ?? {}
        const hoverAltitude = auto.hoverAltitude ?? ((config.autoFlight && config.autoFlight.takeoffHoverAltitude) || 3.0)
        const takeoffClimbSpeed = auto.takeoffClimbSpeed ?? 1.3
        const maxCruiseSpeed = auto.maxCruiseSpeed ?? (config.map.mission?.navigation?.cruiseSpeed ?? 3.0)
        const maxApproachSpeed = auto.maxApproachSpeed ?? 1.2
        const maxVerticalSpeed = auto.maxVerticalSpeed ?? 1.0
        const yawRateLimit = auto.alignYawRateLimit ?? 1.2
        const cruiseArrivalRadius = auto.cruiseArrivalRadius ?? 1.6
        const finalArrivalRadius = auto.finalArrivalRadius ?? 0.8
        const brakingDistance = auto.brakingDistance ?? 6.0
        const velocityBlend = auto.velocityBlend ?? 0.2
        const holdVelocityBlend = auto.holdVelocityBlend ?? 0.35
        const holdSnapSpeed = auto.holdSnapSpeed ?? 0.2
        const verticalDescentLockRadius = auto.verticalDescentLockRadius ?? 1.2
        const arriveObstacleClearance = auto.arriveObstacleClearance ?? 4.5
        const arriveVerticalWindow = auto.arriveVerticalWindow ?? 1.8
        const avoidanceTriggerDistance = auto.avoidanceTriggerDistance ?? 10.0
        const avoidanceHardDistance = auto.avoidanceHardDistance ?? 4.0
        const avoidanceContactEscapeSpeed = auto.avoidanceContactEscapeSpeed ?? 1.8
        const cruiseSetpointWeight = auto.cruiseSetpointWeight ?? 0.9
        const d3qnOverrideWeight = auto.d3qnOverrideWeight ?? 0.88
        const recoveryOverrideWeight = auto.recoveryOverrideWeight ?? 0.82
        const apfOverrideWeight = auto.apfOverrideWeight ?? 0.98
        const position = observation.ego.position
        const velocity = observation.ego.linearVelocity
        const goal = observation.goal ?? navState.endPos.toArray()
        const collision = observation.collision ?? { isColliding: false, minDistance: Infinity, hasPhysicalContact: false }
        const minObstacleDistance = collision.minDistance ?? Infinity
        const hasPhysicalContact = !!collision.hasPhysicalContact
        const avoidanceSource = setpoint && ['apf', 'd3qn', 'recovery'].includes(setpoint.source)
        const avoidanceVelocity = toVec3Array(setpoint?.velocityWorld ?? [0, 0, 0])
        const hasCruiseSetpoint = setpoint?.source === 'cruise' && magnitude3(toVec3Array(setpoint.velocityWorld ?? [0, 0, 0])) > 1e-3
        const cruiseVelocityWorld = toVec3Array(setpoint?.velocityWorld ?? [0, 0, 0])
        const groundProtectionZ = auto.groundProtectionZ ?? -2.8
        const groundDownwardDeadband = auto.groundDownwardDeadband ?? 0.05
        const cruiseAltitudeTolerance = auto.cruiseAltitudeTolerance ?? 0.4
        const cruiseAltitudeRecoverySpeed = auto.cruiseAltitudeRecoverySpeed ?? 0.7
        const airborneSkipTakeoffDistance = auto.airborneSkipTakeoffDistance ?? 2.5
        const verticalEscapeTriggerDistance = auto.verticalEscapeTriggerDistance ?? 0.7
        const verticalEscapeClearance = auto.verticalEscapeClearance ?? 0.8
        const verticalEscapeSpeed = auto.verticalEscapeSpeed ?? 1.0
        const verticalEscapeMaxRise = auto.verticalEscapeMaxRise ?? 2.5
        const verticalEscapeMaxObstacleWidth = auto.verticalEscapeMaxObstacleWidth ?? 6.0
        const verticalEscapeMaxObstacleHeight = auto.verticalEscapeMaxObstacleHeight ?? 8.0
        const rayEscapeTriggerDistance = auto.rayEscapeTriggerDistance ?? 0.45
        const rayEscapeSpeed = auto.rayEscapeSpeed ?? 2.2
        const rayEscapeWeight = auto.rayEscapeWeight ?? 0.96
        const avoidanceGoalMinForwardSpeed = auto.avoidanceGoalMinForwardSpeed ?? 0.55
        const emergencyOverrideDistance = auto.emergencyOverrideDistance ?? 0.85
        const emergencyOverrideContactSeconds = auto.emergencyOverrideContactSeconds ?? 0.55
        const emergencyOverrideClearSeconds = auto.emergencyOverrideClearSeconds ?? 0.25
        const altitudeOnlyArriveRadius = auto.altitudeOnlyArriveRadius ?? Math.max(verticalDescentLockRadius * 1.25, 1.0)
        const altitudeOnlyArriveClearance = auto.altitudeOnlyArriveClearance ?? 0.9
        const altitudeOnlyVerticalGain = auto.altitudeOnlyVerticalGain ?? 0.9
        const takeoffProtectedUpClearance = auto.takeoffProtectedUpClearance ?? 1.5
        const takeoffAvoidanceHardDistance = auto.takeoffAvoidanceHardDistance ?? 0.28

        const hoverZ = navState.startPos.z - hoverAltitude
        if (!Number.isFinite(navState.cruiseZ)) {
            navState.cruiseZ = hoverZ
        }
        const toGoal = [
            goal[0] - position[0],
            goal[1] - position[1],
            goal[2] - position[2],
        ]
        const horizontalGoalDistance = Math.hypot(toGoal[0], toGoal[1])
        const goalDistance = Math.hypot(toGoal[0], toGoal[1], toGoal[2])
        const altitudeError = hoverZ - position[2]
        const currentYaw = observation.ego.orientation?.[2] ?? 0
        const now = observation.timestamp

        if (!Number.isFinite(navState.emergencyContactTime)) {
            navState.emergencyContactTime = 0
        }
        if (!Number.isFinite(navState.lastEmergencyClearTime)) {
            navState.lastEmergencyClearTime = now
        }

        if (!navState.phase) {
            navState.phase = 'takeoff'
        }

        if (!navState.altitudeModeInitialized) {
            navState.altitudeModeInitialized = true
            const groundDistance = observation.rayDistances?.down ?? Infinity
            if (groundDistance > airborneSkipTakeoffDistance) {
                navState.cruiseZ = position[2]
                navState.phase = 'align'
                navState.alignTimer = 0
            } else {
                navState.cruiseZ = hoverZ
                navState.phase = 'takeoff'
            }
        }

        if (navState.phase === 'takeoff' && position[2] <= hoverZ + 0.25) {
            navState.phase = 'align'
            navState.alignTimer = 0
        }

        if (navState.phase === 'align') {
            const headingTarget = Math.atan2(toGoal[1], toGoal[0] || 1e-6)
            const yawError = wrapAngle(headingTarget - currentYaw)
            navState.alignTimer = Math.abs(yawError) < 0.1 ? (navState.alignTimer ?? 0) + dt : 0
            if ((navState.alignTimer ?? 0) >= 0.25) {
                navState.phase = 'cruise'
            }
        }

        if (navState.phase === 'cruise' || navState.phase === 'arrive') {
            navState.cruiseZ = computeAdaptiveCruiseZ(
                observation,
                navState,
                goal,
                horizontalGoalDistance,
            )
        }

        const arrivalPathClear =
            minObstacleDistance > arriveObstacleClearance &&
            !hasPhysicalContact &&
            !avoidanceSource
        const altitudeOnlyReady =
            horizontalGoalDistance < altitudeOnlyArriveRadius &&
            !hasPhysicalContact &&
            minObstacleDistance > altitudeOnlyArriveClearance
        const readyForArrive =
            horizontalGoalDistance < verticalDescentLockRadius &&
            Math.abs(toGoal[2]) < arriveVerticalWindow
        const closeEnoughForArrive =
            goalDistance < Math.max(cruiseArrivalRadius, brakingDistance * 0.28) &&
            horizontalGoalDistance < verticalDescentLockRadius * 1.35

        if (
            navState.phase === 'cruise' &&
            (
                (arrivalPathClear && (readyForArrive || closeEnoughForArrive))
                || altitudeOnlyReady
            )
        ) {
            navState.phase = 'arrive'
            navState.approachYaw = navState.lastYaw ?? currentYaw
        }

        if (navState.phase === 'arrive' && goalDistance < finalArrivalRadius) {
            navState.phase = 'hold'
            navState.reached = true
            navState.holdYaw = currentYaw
        }

        let desiredVelocity = [0, 0, 0]
        let desiredYaw = navState.lastYaw ?? currentYaw

        if (navState.phase === 'takeoff') {
            desiredVelocity = [0, 0, -takeoffClimbSpeed]
        } else if (navState.phase === 'align') {
            desiredVelocity = [0, 0, clamp(altitudeError * 0.8, -0.35, 0.35)]
            desiredYaw = Math.atan2(toGoal[1], toGoal[0] || 1e-6)
        } else if (navState.phase === 'cruise') {
            if (hasCruiseSetpoint) {
                const speedScale = clamp(horizontalGoalDistance / brakingDistance, 0.35, 1.0)
                desiredVelocity = limitVelocity([
                    cruiseVelocityWorld[0] * cruiseSetpointWeight * speedScale,
                    cruiseVelocityWorld[1] * cruiseSetpointWeight * speedScale,
                    clamp((navState.cruiseZ - position[2]) * (auto.verticalCruiseGain ?? 0.55), -maxVerticalSpeed, maxVerticalSpeed),
                ], maxCruiseSpeed, maxVerticalSpeed)
            } else {
                const direct = new THREE.Vector3(toGoal[0], toGoal[1], 0)
                const directLen = direct.length() || 1
                direct.multiplyScalar(1 / directLen)
                const speedScale = clamp(horizontalGoalDistance / brakingDistance, 0.3, 1.0)
                desiredVelocity = [
                    clamp(direct.x * maxCruiseSpeed * speedScale, -maxCruiseSpeed, maxCruiseSpeed),
                    clamp(direct.y * maxCruiseSpeed * speedScale, -maxCruiseSpeed, maxCruiseSpeed),
                    clamp((navState.cruiseZ - position[2]) * (auto.verticalCruiseGain ?? 0.55), -maxVerticalSpeed, maxVerticalSpeed),
                ]
            }
            desiredYaw = Math.atan2(desiredVelocity[1], desiredVelocity[0] || 1e-6)
        } else if (navState.phase === 'arrive') {
            const dist3d = Math.max(goalDistance, 1e-6)
            if (horizontalGoalDistance < verticalDescentLockRadius) {
                desiredVelocity = [
                    clamp(toGoal[0] * 0.4, -0.2, 0.2),
                    clamp(toGoal[1] * 0.4, -0.2, 0.2),
                    clamp(toGoal[2] * 0.8, -maxVerticalSpeed, maxVerticalSpeed),
                ]
                desiredYaw = navState.approachYaw ?? navState.lastYaw ?? currentYaw
            } else {
                desiredVelocity = [
                    clamp((toGoal[0] / dist3d) * maxApproachSpeed, -maxApproachSpeed, maxApproachSpeed),
                    clamp((toGoal[1] / dist3d) * maxApproachSpeed, -maxApproachSpeed, maxApproachSpeed),
                    clamp((toGoal[2] / dist3d) * maxApproachSpeed, -maxVerticalSpeed, maxVerticalSpeed),
                ]
                desiredYaw = Math.atan2(toGoal[1], toGoal[0] || 1e-6)
                navState.approachYaw = desiredYaw
            }
        } else {
            desiredVelocity = [0, 0, 0]
            desiredYaw = navState.holdYaw ?? navState.lastYaw ?? currentYaw
        }

        updateNavigationProgressWatchdog(
            navState,
            observation,
            setpoint,
            goalDistance,
            horizontalGoalDistance,
            minObstacleDistance,
        )

        if (avoidanceSource) {
            let avoidanceWeight = setpoint.source === 'apf'
                ? apfOverrideWeight
                : setpoint.source === 'd3qn'
                    ? d3qnOverrideWeight
                    : recoveryOverrideWeight

            const takeoffProtected =
                (navState.phase === 'takeoff' || navState.phase === 'align') &&
                !hasPhysicalContact &&
                (observation.rayDistances?.up ?? Infinity) > takeoffProtectedUpClearance
            if (takeoffProtected && minObstacleDistance > takeoffAvoidanceHardDistance) {
                desiredVelocity = [0, 0, desiredVelocity[2]]
                desiredYaw = navState.lastYaw ?? currentYaw
            } else {
                if (hasPhysicalContact) {
                    avoidanceWeight = 1.0
                } else if (minObstacleDistance < avoidanceHardDistance) {
                    avoidanceWeight = Math.max(avoidanceWeight, 0.96)
                } else if (minObstacleDistance < avoidanceTriggerDistance) {
                    avoidanceWeight = Math.max(avoidanceWeight, 0.84)
                } else if (navState.phase !== 'cruise') {
                    avoidanceWeight *= 0.72
                }

                const limitedAvoidanceVelocity = limitVelocity(avoidanceVelocity, maxCruiseSpeed, maxVerticalSpeed)
                if (navState.phase !== 'arrive') {
                    limitedAvoidanceVelocity[2] = Math.min(limitedAvoidanceVelocity[2], 0)
                }
                if (
                    navState.phase === 'cruise' &&
                    setpoint.source === 'recovery' &&
                    !hasPhysicalContact
                ) {
                    desiredVelocity = limitVelocity(
                        [
                            limitedAvoidanceVelocity[0],
                            limitedAvoidanceVelocity[1],
                            clamp((navState.cruiseZ - position[2]) * (auto.verticalCruiseGain ?? 0.55), -maxVerticalSpeed, maxVerticalSpeed),
                        ],
                        maxCruiseSpeed,
                        maxVerticalSpeed,
                    )
                } else if (
                    navState.phase === 'cruise' &&
                    setpoint.source === 'apf' &&
                    !hasPhysicalContact
                ) {
                    desiredVelocity = blendVelocity(
                        [
                            desiredVelocity[0],
                            desiredVelocity[1],
                            clamp((navState.cruiseZ - position[2]) * (auto.verticalCruiseGain ?? 0.55), -maxVerticalSpeed, maxVerticalSpeed),
                        ],
                        [
                            limitedAvoidanceVelocity[0],
                            limitedAvoidanceVelocity[1],
                            clamp((navState.cruiseZ - position[2]) * (auto.verticalCruiseGain ?? 0.55), -maxVerticalSpeed, maxVerticalSpeed),
                        ],
                        Math.max(avoidanceWeight, 0.92),
                        maxCruiseSpeed,
                        maxVerticalSpeed,
                    )
                } else if (avoidanceWeight >= 0.92 && navState.phase === 'cruise') {
                    desiredVelocity = limitedAvoidanceVelocity
                } else if (avoidanceWeight > 0) {
                    desiredVelocity = blendVelocity(
                        desiredVelocity,
                        limitedAvoidanceVelocity,
                        avoidanceWeight,
                        maxCruiseSpeed,
                        maxVerticalSpeed,
                    )
                }

                if (navState.phase === 'cruise') {
                    desiredVelocity = enforceGoalProgress(
                        desiredVelocity,
                        position,
                        goal,
                        avoidanceGoalMinForwardSpeed,
                    )
                }
            }
        }

        if (navState.phase === 'cruise' && !hasPhysicalContact) {
            desiredVelocity[2] = clamp(
                (navState.cruiseZ - position[2]) * (auto.verticalCruiseGain ?? 0.55),
                -maxVerticalSpeed,
                maxVerticalSpeed,
            )
        }

        if (
            navState.phase === 'cruise' &&
            avoidanceSource &&
            minObstacleDistance < verticalEscapeTriggerDistance
        ) {
            const climbObstacle = findVerticalEscapeCandidate(
                observation,
                position,
                Math.max(avoidanceHardDistance, verticalEscapeTriggerDistance * 2),
                verticalEscapeMaxObstacleWidth,
                verticalEscapeMaxObstacleHeight,
            )
            if (climbObstacle) {
                const extents = obstacleHalfExtents(climbObstacle)
                const obstacleTopZ = climbObstacle.position[2] - extents.z
                const targetClimbZ = obstacleTopZ - verticalEscapeClearance
                const requiredRise = position[2] - targetClimbZ
                if (requiredRise > 0 && requiredRise <= verticalEscapeMaxRise) {
                    desiredVelocity[2] = -Math.min(
                        verticalEscapeSpeed,
                        Math.max(requiredRise * 0.8, 0.35),
                    )
                }
            }
        }

        if ((navState.escapeUntil ?? -Infinity) > now && navState.phase === 'cruise') {
            desiredVelocity = computeStuckEscapeVelocity(
                navState,
                observation,
                goal,
                maxCruiseSpeed,
                maxVerticalSpeed,
            )
            desiredYaw = Math.atan2(desiredVelocity[1], desiredVelocity[0] || 1e-6)
        }

        const nearCollision = hasPhysicalContact || minObstacleDistance < rayEscapeTriggerDistance
        if (nearCollision) {
            const escapeDirection = computeRayEscapeDirection(observation, currentYaw, goal)
            if (escapeDirection.lengthSq() > 1e-6) {
                const escapeSpeed = hasPhysicalContact
                    ? avoidanceContactEscapeSpeed
                    : rayEscapeSpeed
                escapeDirection.normalize().multiplyScalar(escapeSpeed)
                const escapeVector = [
                    escapeDirection.x,
                    escapeDirection.y,
                    Math.min(escapeDirection.z, 0),
                ]
                desiredVelocity = blendVelocity(
                    desiredVelocity,
                    escapeVector,
                    hasPhysicalContact ? 0.9 : rayEscapeWeight,
                    maxCruiseSpeed,
                    maxVerticalSpeed,
                )
                if (navState.phase === 'cruise') {
                    desiredVelocity = enforceGoalProgress(
                        desiredVelocity,
                        position,
                        goal,
                        Math.max(avoidanceGoalMinForwardSpeed * 0.85, 0.35),
                    )
                }
                desiredYaw = Math.atan2(desiredVelocity[1], desiredVelocity[0] || 1e-6)
            }
        }

        const emergencyPinned =
            navState.phase === 'cruise' &&
            avoidanceSource &&
            (hasPhysicalContact || minObstacleDistance < emergencyOverrideDistance)

        if (emergencyPinned) {
            navState.emergencyContactTime += dt
        } else {
            navState.emergencyContactTime = Math.max(navState.emergencyContactTime - dt * 2, 0)
            navState.lastEmergencyClearTime = now
        }

        if (
            navState.phase === 'cruise' &&
            avoidanceSource &&
            navState.emergencyContactTime >= emergencyOverrideContactSeconds
        ) {
            navState.forceReplan = true
            navState.escapeAttemptCount = (navState.escapeAttemptCount ?? 0) + 1
            desiredVelocity = computeEmergencyOverrideVelocity(
                navState,
                observation,
                currentYaw,
                goal,
                maxCruiseSpeed,
                maxVerticalSpeed,
            )
            desiredYaw = Math.atan2(desiredVelocity[1], desiredVelocity[0] || 1e-6)
            navState.escapeUntil = Math.max(
                navState.escapeUntil ?? -Infinity,
                now + emergencyOverrideClearSeconds,
            )
        }

        if (
            navState.phase !== 'hold' &&
            altitudeOnlyReady
        ) {
            navState.phase = 'arrive'
            desiredVelocity = [
                clamp(toGoal[0] * 0.35, -0.18, 0.18),
                clamp(toGoal[1] * 0.35, -0.18, 0.18),
                clamp(toGoal[2] * altitudeOnlyVerticalGain, -maxVerticalSpeed, maxVerticalSpeed),
            ]
            desiredYaw = navState.approachYaw ?? navState.lastYaw ?? currentYaw
        }

        const allowGroundDirectionMotion =
            navState.phase === 'arrive'
            && horizontalGoalDistance < verticalDescentLockRadius
        if (!allowGroundDirectionMotion && desiredVelocity[2] > 0) {
            desiredVelocity[2] = 0
        }
        if (
            navState.phase === 'cruise'
            && position[2] > navState.cruiseZ + cruiseAltitudeTolerance
        ) {
            desiredVelocity[2] = Math.min(
                desiredVelocity[2],
                -cruiseAltitudeRecoverySpeed,
            )
        }
        desiredVelocity = limitVelocity(desiredVelocity, maxCruiseSpeed, maxVerticalSpeed)
        if (position[2] > groundProtectionZ && desiredVelocity[2] > groundDownwardDeadband) {
            desiredVelocity[2] = 0
        }
        if (Math.hypot(desiredVelocity[0], desiredVelocity[1]) > 0.08) {
            desiredYaw = Math.atan2(desiredVelocity[1], desiredVelocity[0])
        }

        const yawError = wrapAngle(desiredYaw - currentYaw)
        const nextYaw = currentYaw + clamp(yawError, -yawRateLimit * dt, yawRateLimit * dt)
        navState.lastYaw = nextYaw

        const blend = navState.phase === 'hold' ? holdVelocityBlend : velocityBlend
        const nextVelocity = {
            x: velocity[0] * (1 - blend) + desiredVelocity[0] * blend,
            y: velocity[1] * (1 - blend) + desiredVelocity[1] * blend,
            z: velocity[2] * (1 - blend) + desiredVelocity[2] * blend,
        }
        const speed = Math.hypot(nextVelocity.x, nextVelocity.y, nextVelocity.z)
        if (navState.phase === 'hold' && speed < holdSnapSpeed) {
            nextVelocity.x = 0
            nextVelocity.y = 0
            nextVelocity.z = 0
        }

        droneBody.resetForces(true)
        droneBody.resetTorques(true)
        droneBody.setRotation(makeLevelBodyQuaternionFromHeading(nextYaw), true)
        droneBody.setAngvel({ x: 0, y: 0, z: 0 }, true)
        droneBody.setLinvel(nextVelocity, true)

        return {
            throttleInput: 0.5,
            rollInput: 0,
            pitchInput: 0,
            yawInput: 0,
            reset: false,
            mode: 5,
            forwardInput: 0,
        }
    }

    function buildCruiseSetpoint(observation, navState = null) {
        const path = observation.globalPath
        const currentPosition = observation.ego.position
        const lookaheadDistance = config.navigation?.autopilot?.pathLookaheadDistance ?? 6.0
        let target = observation.goal
        if (path && path.length > 0) {
            target = targetAlongPath(path, currentPosition, lookaheadDistance) ?? path[path.length - 1]
        }
        const egoPosition = observation.ego.position
        const horizontalGoalDistance = Math.hypot(
            observation.goal[0] - egoPosition[0],
            observation.goal[1] - egoPosition[1],
        )
        const cruiseZ = computeAdaptiveCruiseZ(observation, navState ?? {}, observation.goal, horizontalGoalDistance)
        const toTarget = [target[0] - egoPosition[0], target[1] - egoPosition[1], cruiseZ - egoPosition[2]]
        const horizontalLength = Math.sqrt(toTarget[0] ** 2 + toTarget[1] ** 2) || 1
        const fullLength = Math.sqrt(toTarget[0] ** 2 + toTarget[1] ** 2 + toTarget[2] ** 2) || 1
        const cruiseSpeed = config.map.mission?.navigation?.cruiseSpeed ?? 4.0
        const verticalCruiseGain = config.navigation?.autopilot?.verticalCruiseGain ?? 0.55
        const maxVerticalSpeed = config.navigation?.autopilot?.maxVerticalSpeed ?? 1.0
        return {
            velocityWorld: [
                (toTarget[0] / horizontalLength) * cruiseSpeed,
                (toTarget[1] / horizontalLength) * cruiseSpeed,
                clamp((toTarget[2] / Math.max(fullLength, 1)) * cruiseSpeed * verticalCruiseGain, -maxVerticalSpeed, maxVerticalSpeed),
            ],
            yawRate: 0,
            source: 'cruise',
            priority: avoidanceSupervisor.parameters.cruisePriority ?? 20,
            validUntil: observation.timestamp + 0.2,
        }
    }

    function sameVec3(a, b) {
        if (!a || !b) return false
        return Math.abs(a[0] - b[0]) < 1e-6 && Math.abs(a[1] - b[1]) < 1e-6 && Math.abs(a[2] - b[2]) < 1e-6
    }

    function resetNavigationAvoidanceState(now = ingameTime) {
        plannerGeneration += 1
        latestGlobalPath = []
        lastPlannerRequestTime = -Infinity
        lastPlannerGoal = null
        avoidanceSupervisor.reset(now)
        threatAssessor.reset()
        d3qnClient.reset()
        plannerWorker.postMessage({ type: 'reset' })
        plannerWorker.postMessage({ type: 'init', config })
    }

    function maybeRequestGlobalPlan(observation) {
        const plannerConfig = config.navigation?.globalPlanner ?? {}
        const minInterval = plannerConfig.replanTimeThreshold ?? 0.5
        const forceReplan = !!game.navState?.forceReplan
        const navPhase = game.navState?.phase ?? 'idle'
        const plannerCruiseZ = Number.isFinite(game.navState?.cruiseZ)
            ? game.navState.cruiseZ
            : observation.ego.position[2]
        const altitudeTrackGoalRadius = config.navigation?.autopilot?.altitudeTrackGoalRadius
            ?? 3.0
        const horizontalGoalDistance = Math.hypot(
            observation.goal[0] - observation.ego.position[0],
            observation.goal[1] - observation.ego.position[1],
        )
        const useCruisePlanePlan =
            game.type === 'navigation' &&
            game.navigationActive &&
            navPhase !== 'arrive' &&
            navPhase !== 'hold' &&
            horizontalGoalDistance > altitudeTrackGoalRadius

        const plannerStart = useCruisePlanePlan
            ? [observation.ego.position[0], observation.ego.position[1], plannerCruiseZ]
            : [...observation.ego.position]
        const plannerGoal = useCruisePlanePlan
            ? [observation.goal[0], observation.goal[1], plannerCruiseZ]
            : [...observation.goal]

        const goalChanged = !sameVec3(lastPlannerGoal, plannerGoal)
        if (!forceReplan && !goalChanged && (observation.timestamp - lastPlannerRequestTime) < minInterval) {
            return
        }
        if (game.navState) {
            game.navState.forceReplan = false
        }
        lastPlannerRequestTime = observation.timestamp
        lastPlannerGoal = plannerGoal ? [...plannerGoal] : null
        plannerWorker.postMessage({
            type: 'plan',
            request: {
                start: plannerStart,
                goal: plannerGoal,
                staticObstacles: [
                    ...terrainPlannerObstacles,
                    ...(testStaticObstacles ?? []),
                ],
                timestamp: observation.timestamp,
                generation: plannerGeneration,
            },
            config,
        })
    }

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
                resetNavigationAvoidanceState(ingameTime)
                game.type = 'navigation'
                game.navState = {
                    startPos: new THREE.Vector3(...e.data.startPos),
                    endPos: new THREE.Vector3(...e.data.endPos),
                    cruiseZ: e.data.startPos[2] - (config.navigation?.autopilot?.hoverAltitude ?? 3.0),
                    reached: false,
                    phase: 'takeoff',
                    alignTimer: 0,
                    stuckTime: 0,
                    lastGoalDistance: Infinity,
                    escapeUntil: -Infinity,
                    escapeAttemptCount: 0,
                    emergencyContactTime: 0,
                    forceReplan: false,
                    lastYaw: 0,
                }
                game.navigationActive = true
                flightState.phase = 'cruising'
                flightState.spawnPos = [...e.data.startPos]
                droneBody.setTranslation({ x: e.data.startPos[0], y: e.data.startPos[1], z: e.data.startPos[2] });
                droneBody.setLinvel({ x: 0, y: 0, z: 0 });
                droneBody.setAngvel({ x: 0, y: 0, z: 0 });
                const spawnRotation = rpyDegToQuat(config.map.spawn.rollPitchYaw)
                droneBody.setRotation({
                    x: spawnRotation.x,
                    y: spawnRotation.y,
                    z: spawnRotation.z,
                    w: spawnRotation.w,
                });
            } else if (e.data.navCommand === 'stop') {
                resetNavigationAvoidanceState(ingameTime)
                game.navigationActive = false
                if (game.navState) {
                    game.navState.reached = false
                    game.navState.phase = 'idle'
                }
            }
        }
    })

    let game = { type: config.map.mission.type, finished: false }
    if (game.type === "navigation") {
        game.navState = initNavState(config)
        game.navigationActive = false
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
        else {
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
            else {
                if (game.activeCheckpoint === sensorHit) {
                    if (game.activeCheckpoint === game.checkpoints[game.checkpoints.length - 1]) {
                        if (game.mode === "point-to-point") {
                            game.finished = true
                            game.activeCheckpoint = null
                        }
                        else {
                            game.lapsLeft -= 1
                            game.activeCheckpoint = game.checkpoints[0]
                        }
                    }
                    else if (
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

    let ingameTime = 0.0
    let tNextStep = performance.now()
    let stepId = 0
    function stepPhysics() {

        world.step()
        stepId += 1

        let sensorHit = null
        world.intersectionPairsWith(droneBody.collider(0), (sensorCollider) => {
            const intersecting = world.intersectionPair(droneBody.collider(0), sensorCollider);
            if (intersecting) {
                sensorHit = colliderNames.get(sensorCollider)
                handleSensorHit(sensorHit)
            }
        });

        const navGoal = game.type === 'navigation' && game.navState?.endPos
            ? game.navState.endPos
            : null
        if (testScene.enabled) {
            for (const obstacle of testDynamicObstacles) {
                const [ax, ay, az] = obstacle.path.a
                const [bx, by, bz] = obstacle.path.b
                const t = 0.5 + 0.5 * Math.sin(obstacle.phase + ingameTime * obstacle.speed)
                const nextPos = {
                    x: ax + (bx - ax) * t,
                    y: ay + (by - ay) * t,
                    z: az + (bz - az) * t,
                }
                const prev = obstacle.body.translation()
                obstacle.body.setNextKinematicTranslation(nextPos)
                obstacle.position = [nextPos.x, nextPos.y, nextPos.z]
                obstacle.velocity = [
                    (nextPos.x - prev.x) / dt,
                    (nextPos.y - prev.y) / dt,
                    (nextPos.z - prev.z) / dt,
                ]
            }
        }

        const observation = environmentAdapter.sample({
            stepId,
            world,
            droneBody,
            goal: navGoal,
            extraStaticObstacles: [
                ...terrainPlannerObstacles,
                ...testStaticObstacles,
            ],
            extraDynamicObstacles: testDynamicObstacles.map(obstacle => ({
                id: obstacle.id,
                type: obstacle.type,
                position: obstacle.position,
                velocity: obstacle.velocity,
                size: Math.max(obstacle.size[0] ?? obstacle.size, obstacle.size[1] ?? obstacle.size, obstacle.size[2] ?? obstacle.size),
                dynamic: true,
                confidence: obstacle.confidence,
            })),
            dt,
            timestamp: ingameTime,
        })
        const effectivePath = navGoal
            ? (
                latestGlobalPath.length > 0
                    ? latestGlobalPath.map(waypoint => [...waypoint])
                    : synthesizeFallbackPath(observation, game.navState)
            )
            : []
        observation.globalPath = effectivePath
        observation.cruiseZ = Number.isFinite(game.navState?.cruiseZ)
            ? game.navState.cruiseZ
            : observation.ego.position[2]
        observation.navPhase = game.navState?.phase ?? 'idle'
        const threats = threatAssessor.update(observation)
        avoidanceSupervisor.acceptThreatAssessments(threats)
        maybeRequestGlobalPlan(observation)
        d3qnClient.maybeRequestInference(observation, threats)
        avoidanceSupervisor.acceptRlResult(d3qnClient.latestFreshResult(config.navigation?.d3qn?.resultTtlMs ?? 150, observation.timestamp))
        const cruiseSetpoint = buildCruiseSetpoint(observation, game.navState)
        const navSetpoint = avoidanceSupervisor.update(observation, cruiseSetpoint)
        appliedInputs = null

        if (inputs) {
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

            const navOwnsControl =
                game.type === 'navigation' &&
                game.navigationActive

            if (navOwnsControl) {
                appliedInputs = applyBasicNavigationAutopilot(game.navState, observation, navSetpoint)
                flightState.phase = game.navState.phase === 'takeoff' ? 'takingOff' : 'cruising'
            } else {
                const autoInputs = controlAutoPhase(flightState, controlData, droneBody, config, dt, world);
                if (autoInputs) {
                    appliedInputs = autoInputs
                    controlDrone(autoInputs, controlData, droneBody, trace, config, dt);
                } else if (flightState.phase === 'ground') {
                appliedInputs = inputs
                controlDrone(inputs, controlData, droneBody, trace, config, dt);
                if (inputs.throttleInput > 0.6) {
                    flightState.phase = 'cruising';
                }
                } else if (flightState.phase === 'cruising') {
                    appliedInputs = inputs
                    controlDrone(inputs, controlData, droneBody, trace, config, dt);
                }
            }
        }

        if (appliedInputs && appliedInputs.forwardInput && appliedInputs.forwardInput !== 0) {
            const dRot = droneBody.rotation();
            const dQ = new THREE.Quaternion(dRot.x, dRot.y, dRot.z, dRot.w);
            const camQ = new THREE.Quaternion(-0.5, -0.5, 0.5, 0.5);
            const viewQ = dQ.clone().multiply(camQ);
            const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(viewQ);
            const fwdH = new THREE.Vector3(fwd.x, fwd.y, 0);
            if (fwdH.length() > 0.01) {
                fwdH.normalize();
                const FORCE = 8.0;
                droneBody.addForce(
                    new RAPIER.Vector3(
                        fwdH.x * FORCE * appliedInputs.forwardInput,
                        fwdH.y * FORCE * appliedInputs.forwardInput,
                        0,
                    ),
                    true,
                );
            }
        }

        const pos = droneBody.translation();
        const rot = droneBody.rotation();
        const dronePosition = new THREE.Vector3(pos.x, pos.y, pos.z)
        const droneQuaternion = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w)

        const linvel = droneBody.linvel();
        const speed = Math.sqrt(linvel.x * linvel.x + linvel.y * linvel.y + linvel.z * linvel.z);

        const gv = config.map.gravity;
        const gNorm = Math.sqrt(gv[0] * gv[0] + gv[1] * gv[1] + gv[2] * gv[2]);
        const sp = config.map.spawn.position;
        const altitude = gNorm > 1e-6
            ? ((pos.x - sp[0]) * gv[0] + (pos.y - sp[1]) * gv[1] + (pos.z - sp[2]) * gv[2]) / gNorm
            : (pos.z - sp[2]);

        const euler = new THREE.Euler().setFromQuaternion(droneQuaternion, 'ZYX');
        const rpyDeg = { roll: euler.x / deg, pitch: euler.y / deg, yaw: euler.z / deg };

        const distances = {
            front: observation.rayDistances.front.toFixed(1),
            back: observation.rayDistances.back.toFixed(1),
            left: observation.rayDistances.left.toFixed(1),
            right: observation.rayDistances.right.toFixed(1),
            up: observation.rayDistances.up.toFixed(1),
            down: observation.rayDistances.down.toFixed(1),
        }
        const uiCollisionWarningDistance = config.navigation?.sensing?.uiCollisionWarningDistance ?? 0.6
        const isColliding =
            !!observation.collision.hasPhysicalContact ||
            (observation.collision.minDistance ?? Infinity) < uiCollisionWarningDistance
        const supervisorStatus = avoidanceSupervisor.status()
        const navDebug = {
            supervisorMode: supervisorStatus.mode,
            setpointSource: navSetpoint?.source ?? 'none',
            setpointVelocity: toVec3Array(navSetpoint?.velocityWorld ?? [0, 0, 0]),
            navPhase: game.type === 'navigation' ? game.navState?.phase ?? 'idle' : 'idle',
            goalDistance: game.type === 'navigation' && game.navState?.endPos ? game.navState.endPos.distanceTo(dronePosition) : 0,
            obstacleDistance: observation.collision?.minDistance ?? Infinity,
            stuckTime: game.type === 'navigation' ? (game.navState?.stuckTime ?? 0) : 0,
            escapeRemaining: game.type === 'navigation'
                ? Math.max((game.navState?.escapeUntil ?? -Infinity) - ingameTime, 0)
                : 0,
            escapeAttempts: game.type === 'navigation' ? (game.navState?.escapeAttemptCount ?? 0) : 0,
            forceReplan: game.type === 'navigation' ? !!game.navState?.forceReplan : false,
            pathPoints: observation.globalPath.length,
            plannerPathPoints: latestGlobalPath.length,
            localDetourPoints: supervisorStatus.localDetourPoints ?? 0,
            localDetourIndex: supervisorStatus.localDetourIndex ?? 0,
            localDetourObstacleId: supervisorStatus.localDetourObstacleId ?? 'none',
        }

        fpv.update(config.aircraft.camera.firstPerson, dronePosition, droneQuaternion)
        tpv.update(config.aircraft.camera.thirdPerson, dronePosition, droneQuaternion)

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
            navPhase: game.type === 'navigation' ? game.navState.phase : 'idle',
            navHorizontalDistance: game.type === 'navigation' && game.navState?.endPos
                ? horizontalDistance(
                    [dronePosition.x, dronePosition.y],
                    [game.navState.endPos.x, game.navState.endPos.y],
                )
                : 0,
            isColliding: isColliding,
            telemetry: {
                speed: speed.toFixed(1),
                altitude: altitude.toFixed(1),
                roll: rpyDeg.roll.toFixed(1),
                pitch: rpyDeg.pitch.toFixed(1),
                yaw: rpyDeg.yaw.toFixed(1),
                distances,
            },
            testScene: testScene.enabled ? {
                staticObstacles: testStaticObstacles.map(obstacle => ({
                    id: obstacle.id,
                    position: obstacle.position,
                    size: obstacle.size,
                })),
                dynamicObstacles: testDynamicObstacles.map(obstacle => ({
                    id: obstacle.id,
                    position: obstacle.position,
                    size: obstacle.size,
                })),
            } : null,
            navDebug,
            debug: { isActive: debug },
        }
        if (debug) {
            let buffers = []
            if (firstDebugMessage) {
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
        tNextStep = Math.max(tNextStep, tNow - 100)
        setTimeout(stepPhysics, clamp(tNextStep - tNow, 0, Infinity))

        ingameTime += dt
    }
    setTimeout(stepPhysics, dt * 1000)
}
main()
