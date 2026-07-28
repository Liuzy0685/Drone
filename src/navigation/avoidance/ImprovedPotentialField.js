import { THREE } from '../../three.js'

/**
 * @typedef {[number, number, number]} Vec3
 */

/**
 * @typedef {{
 *   ego: {
 *     position: Vec3,
 *     orientation: Vec3,
 *     linearVelocity: Vec3,
 *     angularVelocity: Vec3,
 *   },
 *   goal: Vec3,
 *   globalPath: Vec3[],
 *   staticObstacles: Array<{
 *     id: string,
 *     type: string,
 *     position: Vec3,
 *     velocity: Vec3,
 *     size: number,
 *     dynamic: boolean,
 *     confidence: number,
 *   }>,
 *   dynamicObstacles: Array<{
 *     id: string,
 *     type: string,
 *     position: Vec3,
 *     velocity: Vec3,
 *     size: number,
 *     dynamic: boolean,
 *     confidence: number,
 *   }>,
 *   localPointCloud?: Float32Array,
 *   dt: number,
 *   timestamp: number,
 * }} NavigationObservation
 */

/**
 * @typedef {{
 *   velocityWorld: Vec3,
 *   yawRate?: number,
 *   source: 'cruise' | 'd3qn' | 'apf' | 'recovery' | 'hover',
 *   priority: number,
 *   validUntil: number,
 * }} MotionSetpoint
 */

function vec3FromArray(v) {
    return new THREE.Vector3(v[0], v[1], v[2])
}

function clampMagnitude(vector, maxMagnitude) {
    if (vector.lengthSq() <= maxMagnitude * maxMagnitude) {
        return vector
    }
    return vector.normalize().multiplyScalar(maxMagnitude)
}

function safeNormalize(vector, fallback = new THREE.Vector3(0, 0, 0)) {
    if (vector.lengthSq() < 1e-12) {
        return fallback.clone()
    }
    return vector.clone().normalize()
}

function pointCloudObstacles(pointCloud, pointRadius) {
    if (!pointCloud || pointCloud.length < 3) {
        return []
    }

    const obstacles = []
    for (let i = 0; i <= pointCloud.length - 3; i += 3) {
        obstacles.push({
            id: `point-${i / 3}`,
            type: 'unknown',
            position: [pointCloud[i], pointCloud[i + 1], pointCloud[i + 2]],
            velocity: [0, 0, 0],
            size: pointRadius,
            dynamic: false,
            confidence: 0.25,
        })
    }
    return obstacles
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

function isMassiveTerrainObstacle(obstacle, maxSpan, maxHeight) {
    if (typeof obstacle?.id !== 'string' || !obstacle.id.startsWith('terrain-')) {
        return false
    }
    const extents = obstacleHalfExtents(obstacle)
    const span = Math.max(extents.x * 2, extents.y * 2)
    const height = extents.z * 2
    return span > maxSpan || height > maxHeight
}

function projectPointToSegment(point, start, end) {
    const segment = end.clone().sub(start)
    const lengthSq = segment.lengthSq()
    if (lengthSq < 1e-12) {
        return start.clone()
    }
    const t = THREE.MathUtils.clamp(point.clone().sub(start).dot(segment) / lengthSq, 0, 1)
    return start.clone().addScaledVector(segment, t)
}

/**
 * Improved Artificial Potential Field safety layer.
 * 这一版除了传统吸引/斥力，还加入了：
 * - 沿全局路径切线的走廊吸引
 * - 主障碍物绕行偏置
 * - 静态建筑前向阻塞增益
 * - 绕行方向的短时记忆，减少左右来回抖动
 */
export class ImprovedPotentialField {
    /**
     * @param {any} config
     */
    constructor(config = {}) {
        this.config = config
        this.lastBypassSign = 1
        this.lastBypassObstacleId = null
        this.lastBypassUpdateTime = -Infinity
        this.bypassLockUntil = -Infinity
        this.bypassLockClearance = Infinity
    }

    reset() {
        this.lastBypassSign = 1
        this.lastBypassObstacleId = null
        this.lastBypassUpdateTime = -Infinity
        this.bypassLockUntil = -Infinity
        this.bypassLockClearance = Infinity
    }

    /**
     * @returns {any}
     */
    get parameters() {
        const defaults = {
            enabled: true,
            priority: 100,
            validityMs: 120,
            maxSpeed: 6.0,
            maxAcceleration: 8.0,
            hoverDeadband: 0.35,
            safeDistance: 14.0,
            emergencyDistance: 8.0,
            repulsiveGain: 18.0,
            attractiveGain: 1.8,
            pathAttractionGain: 2.1,
            dampingGain: 1.2,
            tangentialGain: 4.6,
            relativeVelocityGain: 2.5,
            verticalBiasGain: 0.8,
            forwardBlockGain: 6.0,
            dominantObstacleBoost: 1.8,
            pathCorridorWidth: 5.0,
            pointCloudPointRadius: 0.35,
            yawRateGain: 1.4,
            maxYawRate: 1.2,
            localGoalLookahead: 4,
            bypassMemorySeconds: 0.8,
            bypassLockSeconds: 2.0,
            bypassLockReleaseDistance: 11.0,
            wallFollowDistance: 10.0,
            wallFollowGain: 2.4,
            upAxisSign: -1,
            localObstacleMaxSpan: 45.0,
            localObstacleMaxHeight: 24.0,
        }
        return {
            ...defaults,
            ...(this.config.navigation?.apf ?? {}),
        }
    }

    /**
     * @param {NavigationObservation} observation
     * @returns {MotionSetpoint}
     */
    update(observation) {
        const params = this.parameters
        const now = observation.timestamp
        const egoPosition = vec3FromArray(observation.ego.position)
        const egoVelocity = vec3FromArray(observation.ego.linearVelocity)
        const pathFrame = this.computePathFrame(observation, egoPosition)
        const localGoal = pathFrame.goal

        const attractive = safeNormalize(localGoal.clone().sub(egoPosition)).multiplyScalar(params.attractiveGain)
        const pathAttraction = pathFrame.tangent.clone().multiplyScalar(params.pathAttractionGain)
        const damping = egoVelocity.clone().multiplyScalar(-params.dampingGain)

        const staticObstacles = observation.staticObstacles ?? []
        const dynamicObstacles = observation.dynamicObstacles ?? []
        const cloudObstacles = pointCloudObstacles(observation.localPointCloud, params.pointCloudPointRadius)
        const filteredStaticObstacles = staticObstacles.filter(obstacle =>
            !isMassiveTerrainObstacle(
                obstacle,
                params.localObstacleMaxSpan,
                params.localObstacleMaxHeight,
            )
        )
        const allObstacles = [...filteredStaticObstacles, ...dynamicObstacles, ...cloudObstacles]

        const dominantObstacle = this.findDominantObstacle(allObstacles, egoPosition, pathFrame.tangent, params)
        const bypassSign = this.selectBypassSign(dominantObstacle, egoPosition, pathFrame.tangent, now, params)

        const repulsive = new THREE.Vector3()
        const tangential = new THREE.Vector3()
        let nearestDistance = Infinity

        for (const obstacle of allObstacles) {
            const obstaclePosition = vec3FromArray(obstacle.position)
            const relativePosition = egoPosition.clone().sub(obstaclePosition)
            const centerDistance = relativePosition.length()
            const clearance = Math.max(centerDistance - (obstacle.size ?? 0), 1e-3)
            nearestDistance = Math.min(nearestDistance, clearance)

            if (clearance > params.safeDistance) {
                continue
            }

            const away = relativePosition.clone().normalize()
            const toObstacle = away.clone().negate()
            const weight = Math.max(obstacle.confidence ?? 0.5, 0.1)
            const alignment = Math.max(pathFrame.tangent.dot(toObstacle), 0)
            const corridorPenalty = Math.max(
                1 - pathFrame.projected.distanceTo(obstaclePosition) / Math.max(params.pathCorridorWidth, 1),
                0,
            )
            const isDominant = dominantObstacle && dominantObstacle.id === obstacle.id
            const boost = isDominant ? params.dominantObstacleBoost : 1.0

            const distanceScale = (1 / clearance) - (1 / params.safeDistance)
            const repulsiveMagnitude =
                params.repulsiveGain * weight * boost * distanceScale / (clearance * clearance)
            repulsive.add(away.multiplyScalar(repulsiveMagnitude))

            if (alignment > 0.2) {
                repulsive.add(
                    away.clone().multiplyScalar(
                        params.forwardBlockGain * weight * boost * alignment * corridorPenalty / Math.max(clearance, 1.0)
                    )
                )
            }

            const obstacleVelocity = vec3FromArray(obstacle.velocity ?? [0, 0, 0])
            const relativeVelocity = egoVelocity.clone().sub(obstacleVelocity)
            const closingSpeed = Math.max(relativeVelocity.dot(toObstacle), 0)
            if (closingSpeed > 0) {
                repulsive.add(
                    away.clone().multiplyScalar(
                        params.relativeVelocityGain * closingSpeed * weight * boost / clearance
                    )
                )
            }

            const tangentAxis = new THREE.Vector3(-relativePosition.y, relativePosition.x, 0)
            if (tangentAxis.lengthSq() > 1e-12) {
                const tangent = tangentAxis.normalize().multiplyScalar(bypassSign)
                const emergencyBoost = clearance < params.emergencyDistance ? 2.0 : 1.0
                const wallFollowBoost = alignment > 0.25 ? params.wallFollowGain : 1.0
                tangential.add(
                    tangent.multiplyScalar(
                        params.tangentialGain *
                        weight *
                        boost *
                        emergencyBoost *
                        wallFollowBoost /
                        Math.max(clearance, 1.0)
                    )
                )
            }

            if (alignment > 0.15 && clearance < params.wallFollowDistance) {
                tangential.add(
                    pathFrame.tangent.clone().multiplyScalar(
                        alignment * weight * params.wallFollowGain / Math.max(clearance, 1.0)
                    )
                )
            }

            const verticalBiasScale = obstacle.dynamic ? 1.0 : 0.2
            if (obstaclePosition.z >= egoPosition.z && clearance < params.emergencyDistance) {
                tangential.z += params.verticalBiasGain * weight * params.upAxisSign * verticalBiasScale
            } else if (obstaclePosition.z < egoPosition.z && clearance < params.emergencyDistance) {
                tangential.z -= params.verticalBiasGain * weight * params.upAxisSign * verticalBiasScale
            }
        }

        const desiredAcceleration = attractive
            .add(pathAttraction)
            .add(repulsive)
            .add(tangential)
            .add(damping)

        clampMagnitude(desiredAcceleration, params.maxAcceleration)

        const desiredVelocity = egoVelocity
            .clone()
            .addScaledVector(desiredAcceleration, observation.dt)

        clampMagnitude(desiredVelocity, params.maxSpeed)

        if (desiredVelocity.length() < params.hoverDeadband && nearestDistance >= params.safeDistance) {
            desiredVelocity.set(0, 0, 0)
        }

        const desiredYaw = Math.atan2(desiredVelocity.y, desiredVelocity.x)
        const currentYaw = observation.ego.orientation[2] ?? 0
        const yawError = Math.atan2(Math.sin(desiredYaw - currentYaw), Math.cos(desiredYaw - currentYaw))
        const yawRate = THREE.MathUtils.clamp(yawError * params.yawRateGain, -params.maxYawRate, params.maxYawRate)

        return {
            velocityWorld: [desiredVelocity.x, desiredVelocity.y, desiredVelocity.z],
            yawRate,
            source: 'apf',
            priority: params.priority,
            validUntil: now + params.validityMs / 1000,
        }
    }

    computePathFrame(observation, egoPosition) {
        const params = this.parameters
        const globalPath = observation.globalPath ?? []
        if (!globalPath || globalPath.length < 2) {
            const goal = vec3FromArray(observation.goal)
            const tangent = safeNormalize(goal.clone().sub(egoPosition), new THREE.Vector3(1, 0, 0))
            return {
                projected: egoPosition.clone(),
                tangent,
                goal,
            }
        }

        let bestSegment = {
            start: vec3FromArray(globalPath[0]),
            end: vec3FromArray(globalPath[1]),
            projected: vec3FromArray(globalPath[0]),
            distance: Infinity,
            index: 0,
        }
        for (let i = 0; i < globalPath.length - 1; i += 1) {
            const start = vec3FromArray(globalPath[i])
            const end = vec3FromArray(globalPath[i + 1])
            const projected = projectPointToSegment(egoPosition, start, end)
            const distance = projected.distanceTo(egoPosition)
            if (distance < bestSegment.distance) {
                bestSegment = { start, end, projected, distance, index: i }
            }
        }

        const tangent = safeNormalize(
            bestSegment.end.clone().sub(bestSegment.start),
            safeNormalize(vec3FromArray(observation.goal).sub(egoPosition), new THREE.Vector3(1, 0, 0)),
        )

        const lookaheadIndex = Math.min(bestSegment.index + params.localGoalLookahead, globalPath.length - 1)
        const goal = vec3FromArray(globalPath[lookaheadIndex])
        return {
            projected: bestSegment.projected,
            tangent,
            goal,
        }
    }

    findDominantObstacle(obstacles, egoPosition, pathTangent, params) {
        let best = null
        let bestScore = -Infinity
        for (const obstacle of obstacles) {
            const obstaclePosition = vec3FromArray(obstacle.position)
            const toObstacle = obstaclePosition.clone().sub(egoPosition)
            const clearance = Math.max(toObstacle.length() - (obstacle.size ?? 0), 1e-3)
            if (clearance > params.safeDistance) continue

            const alignment = Math.max(pathTangent.dot(safeNormalize(toObstacle)), 0)
            const score =
                alignment * 1.8 +
                (1 - clearance / params.safeDistance) * 1.6 +
                (obstacle.dynamic ? 0.25 : 0.5)

            if (score > bestScore) {
                best = {
                    ...obstacle,
                    clearance,
                    alignment,
                    score,
                }
                bestScore = score
            }
        }
        return best
    }

    selectBypassSign(dominantObstacle, egoPosition, pathTangent, now, params) {
        if (!dominantObstacle) {
            if (now > this.bypassLockUntil) {
                this.lastBypassObstacleId = null
                this.bypassLockClearance = Infinity
            }
            return this.lastBypassSign
        }

        const obstaclePosition = vec3FromArray(dominantObstacle.position)
        const toObstacle = obstaclePosition.clone().sub(egoPosition)
        const crossZ = pathTangent.x * toObstacle.y - pathTangent.y * toObstacle.x
        let sign = crossZ >= 0 ? -1 : 1

        const lockStillActive =
            now < this.bypassLockUntil &&
            dominantObstacle.clearance < Math.max(params.bypassLockReleaseDistance, this.bypassLockClearance)

        if (
            this.lastBypassObstacleId === dominantObstacle.id &&
            (
                (now - this.lastBypassUpdateTime) < params.bypassMemorySeconds
                || lockStillActive
            )
        ) {
            sign = this.lastBypassSign
        } else if (lockStillActive) {
            sign = this.lastBypassSign
        }

        this.lastBypassSign = sign
        this.lastBypassObstacleId = dominantObstacle.id
        this.lastBypassUpdateTime = now
        if (dominantObstacle.clearance < params.wallFollowDistance) {
            this.bypassLockUntil = now + params.bypassLockSeconds
            this.bypassLockClearance = dominantObstacle.clearance + params.bypassLockReleaseDistance
        }
        return sign
    }
}
