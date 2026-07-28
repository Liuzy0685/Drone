import { THREE } from '../three.js'
import { ImprovedPotentialField } from './avoidance/ImprovedPotentialField.js'

/**
 * @typedef {[number, number, number]} Vec3
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

/**
 * @typedef {{
 *   obstacleId: string,
 *   type: 'uav' | 'car' | 'person' | 'building' | 'unknown',
 *   threatScore: number,
 *   ttc: number,
 *   dcpa: number,
 *   range: number,
 *   closingSpeed: number,
 *   lastUpdated: number,
 *   obstacle: {
 *     id: string,
 *     position: Vec3,
 *     velocity: Vec3,
 *     size: number,
 *     dynamic: boolean,
 *     confidence: number,
 *   },
 * }} ThreatAssessment
 */

function vec3FromArray(v) {
    return new THREE.Vector3(v[0], v[1], v[2])
}

function clampMagnitude(vector, maxMagnitude) {
    if (vector.lengthSq() > maxMagnitude * maxMagnitude) {
        vector.normalize().multiplyScalar(maxMagnitude)
    }
    return vector
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

function pathDeviation(position, path) {
    if (!path || path.length === 0) {
        return Infinity
    }
    if (path.length === 1) {
        return position.distanceTo(vec3FromArray(path[0]))
    }

    let best = Infinity
    for (let i = 0; i < path.length - 1; i += 1) {
        const start = vec3FromArray(path[i])
        const end = vec3FromArray(path[i + 1])
        const projected = projectPointToSegment(position, start, end)
        best = Math.min(best, projected.distanceTo(position))
    }
    return best
}

function hoverSetpoint(timestamp, validityMs) {
    return {
        velocityWorld: [0, 0, 0],
        yawRate: 0,
        source: 'hover',
        priority: 0,
        validUntil: timestamp + validityMs / 1000,
    }
}

function currentObstacleDistance(observation) {
    return observation?.collision?.minDistance ?? Infinity
}

function clamp01(value) {
    return THREE.MathUtils.clamp(value, 0, 1)
}

function safeDirection(from, to, fallback = new THREE.Vector3(1, 0, 0)) {
    const vector = to.clone().sub(from)
    if (vector.lengthSq() < 1e-12) {
        return fallback.clone()
    }
    return vector.normalize()
}

function lateralDirection(direction, sign = 1) {
    const lateral = new THREE.Vector3(-direction.y, direction.x, 0)
    if (lateral.lengthSq() < 1e-12) {
        return new THREE.Vector3(0, sign, 0)
    }
    return lateral.normalize().multiplyScalar(sign)
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

function horizontalClearanceToObstacle(position, obstacle) {
    const extents = obstacleHalfExtents(obstacle)
    const dx = Math.max(Math.abs(position.x - obstacle.position[0]) - extents.x, 0)
    const dy = Math.max(Math.abs(position.y - obstacle.position[1]) - extents.y, 0)
    return Math.hypot(dx, dy)
}

function closestHorizontalPointOnObstacle(position, obstacle) {
    const extents = obstacleHalfExtents(obstacle)
    const center = obstacle.position
    return new THREE.Vector3(
        THREE.MathUtils.clamp(position.x, center[0] - extents.x, center[0] + extents.x),
        THREE.MathUtils.clamp(position.y, center[1] - extents.y, center[1] + extents.y),
        position.z,
    )
}

function isRayObstacle(obstacle) {
    return typeof obstacle?.id === 'string' && obstacle.id.startsWith('ray-')
}

function preferredStaticObstacles(observation) {
    const obstacles = observation?.staticObstacles ?? []
    const maxSpan = 45.0
    const maxHeight = 24.0
    const structural = obstacles.filter(obstacle =>
        !isRayObstacle(obstacle) &&
        !isMassiveTerrainObstacle(obstacle, maxSpan, maxHeight)
    )
    return structural
}

function clampForwardProgress(point, origin, forward, minProgress) {
    const offset = point.clone().sub(origin)
    const progress = offset.dot(forward)
    if (progress >= minProgress) {
        return point
    }
    return point.add(forward.clone().multiplyScalar(minProgress - progress))
}

function selectBestSetpoint(candidates, fallback) {
    const validCandidates = candidates.filter(Boolean)
    if (validCandidates.length === 0) {
        return fallback
    }
    validCandidates.sort((a, b) => {
        if (b.priority !== a.priority) {
            return b.priority - a.priority
        }
        return b.validUntil - a.validUntil
    })
    return validCandidates[0]
}

function frontRayProfile(observation, fallbackDistance = Infinity) {
    const rays = observation?.rayDistances ?? {}
    return {
        front: Math.min(
            rays.front ?? fallbackDistance,
            rays.frontLeft ?? rays.front ?? fallbackDistance,
            rays.frontRight ?? rays.front ?? fallbackDistance,
        ),
        left: Math.min(
            rays.left ?? fallbackDistance,
            rays.frontLeft ?? rays.left ?? fallbackDistance,
            rays.leftUp ?? rays.left ?? fallbackDistance,
        ),
        right: Math.min(
            rays.right ?? fallbackDistance,
            rays.frontRight ?? rays.right ?? fallbackDistance,
            rays.rightUp ?? rays.right ?? fallbackDistance,
        ),
        up: Math.min(
            rays.up ?? fallbackDistance,
            rays.frontUp ?? rays.up ?? fallbackDistance,
            rays.leftUp ?? rays.up ?? fallbackDistance,
            rays.rightUp ?? rays.up ?? fallbackDistance,
        ),
    }
}

/**
 * Hierarchical avoidance mode scheduler.
 */
export class AvoidanceSupervisor {
    /**
     * @param {any} config
     */
    constructor(config = {}) {
        this.config = config
        this.apf = new ImprovedPotentialField(config)
        this.mode = 'CRUISE'
        this.modeSince = 0
        this.lastGlobalPath = []
        this.lastRlResult = null
        this.lastThreats = []
        this.lastSetpoint = null
        this.localDetourPath = []
        this.localDetourIndex = 0
        this.localDetourObstacleId = null
        this.localDetourSign = 1
        this.localDetourLockUntil = -Infinity
    }

    /**
     * @returns {any}
     */
    get parameters() {
        const defaults = {
            hysteresisEnterSeconds: 0.5,
            hysteresisExitSeconds: 0.75,
            dynamicThreatScoreThreshold: 0.6,
            dynamicTtcThreshold: 5.0,
            emergencyTtcThreshold: 1.5,
            emergencyDistanceThreshold: 8.0,
            staticWarningDistance: 5.0,
            staticEmergencyDistance: 3.0,
            staticBlockedAheadDistance: 8.0,
            staticBlockedAheadConeDot: 0.45,
            staticApfOnlyDistance: 1.6,
            staticHardEmergencyDistance: 0.7,
            recoveryClearSeconds: 0.5,
            recoveryPathDeviationThreshold: 3.0,
            rlResultTtlMs: 150,
            hoverValidityMs: 120,
            recoverySpeed: 4.0,
            cruisePriority: 20,
            recoveryPriority: 40,
            detourLateralGain: 2.2,
            detourForwardGain: 3.4,
            detourObstacleInfluenceDistance: 7.0,
            detourLateralClearance: 3.5,
            detourForwardClearance: 4.5,
            detourExtraClearance: 2.5,
            detourWaypointReachDistance: 2.8,
            detourLockSeconds: 1.4,
            recoveryLookaheadDistance: 8.0,
            recoveryMinForwardSpeed: 1.8,
            recoveryVerticalGain: 0.45,
            recoveryVerticalDeadband: 0.25,
            recoveryVerticalLimit: 0.45,
            localDetourMaxObstacleSpan: 45.0,
            localDetourMaxObstacleHeight: 24.0,
            proactiveFrontDistance: 9.0,
            proactiveFrontHardDistance: 4.8,
            proactiveSidePreferenceGain: 2.6,
            proactiveForwardSuppression: 0.85,
            proactiveClimbDistance: 4.0,
            proactiveClimbSpeed: 0.42,
            proactiveClimbSideThreshold: 2.0,
        }
        return {
            ...defaults,
            ...(this.config.navigation?.supervisor ?? {}),
        }
    }

    /**
     * @param {Vec3[]} path
     */
    acceptGlobalPath(path) {
        this.lastGlobalPath = Array.isArray(path) ? path : []
    }

    /**
     * @param {MotionSetpoint | null} result
     */
    acceptRlResult(result) {
        this.lastRlResult = result ?? null
    }

    /**
     * @param {ThreatAssessment[]} threats
     */
    acceptThreatAssessments(threats) {
        this.lastThreats = Array.isArray(threats) ? threats : []
    }

    reset(now = 0) {
        if (typeof this.apf.reset === 'function') {
            this.apf.reset()
        }
        this.mode = 'CRUISE'
        this.modeSince = now
        this.lastGlobalPath = []
        this.lastRlResult = null
        this.lastThreats = []
        this.lastSetpoint = null
        this.localDetourPath = []
        this.localDetourIndex = 0
        this.localDetourObstacleId = null
        this.localDetourSign = 1
        this.localDetourLockUntil = -Infinity
    }

    /**
     * @param {any} observation
     * @param {MotionSetpoint | null} cruiseSetpoint
     * @returns {MotionSetpoint}
     */
    update(observation, cruiseSetpoint = null) {
        const now = observation.timestamp
        const params = this.parameters
        const navPhase = observation?.navPhase ?? 'cruise'

        if (navPhase === 'takeoff' || navPhase === 'align') {
            this.mode = 'CRUISE'
            this.modeSince = now
            this.lastSetpoint = cruiseSetpoint ?? hoverSetpoint(now, params.hoverValidityMs)
            return this.lastSetpoint
        }

        const threats = this.lastThreats
        const apfSetpoint = this.apf.update({
            ...observation,
            globalPath: this.lastGlobalPath.length > 0 ? this.lastGlobalPath : observation.globalPath,
        })
        const rlFresh = this.latestFreshRlResult(now)
        const staticBlockedAhead = this.isStaticBlockedAhead(observation)

        this.transitionMode(observation, threats, now)
        this.refreshLocalDetour(observation, staticBlockedAhead)

        let selected = null
        if (this.mode === 'EMERGENCY_AVOID') {
            selected = this.shouldApfOwnControl(observation, threats)
                ? apfSetpoint
                : this.buildRecoverySetpoint(observation, now)
        } else if (this.mode === 'DYNAMIC_AVOID') {
            selected = this.selectDynamicAvoidanceSetpoint(observation, rlFresh, apfSetpoint, cruiseSetpoint, threats, now)
        } else if (this.mode === 'RECOVERY') {
            const recovery = this.buildRecoverySetpoint(observation, now)
            selected = this.shouldApfOwnControl(observation, threats)
                ? selectBestSetpoint([apfSetpoint, recovery], apfSetpoint ?? recovery)
                : selectBestSetpoint([recovery, cruiseSetpoint], recovery)
        } else {
            const recovery = this.buildRecoverySetpoint(observation, now)
            if (this.shouldApfOwnControl(observation, threats)) {
                selected = selectBestSetpoint([apfSetpoint, recovery, cruiseSetpoint], apfSetpoint ?? recovery ?? cruiseSetpoint)
            } else if (staticBlockedAhead) {
                selected = selectBestSetpoint([apfSetpoint, recovery, cruiseSetpoint], apfSetpoint ?? recovery ?? cruiseSetpoint)
            } else {
                selected = cruiseSetpoint ?? recovery ?? apfSetpoint
            }
        }

        if (!selected) {
            selected = hoverSetpoint(now, params.hoverValidityMs)
        }

        if (this.mode !== 'EMERGENCY_AVOID' && selected.source === 'd3qn' && !rlFresh) {
            selected = apfSetpoint ?? hoverSetpoint(now, params.hoverValidityMs)
        }

        this.lastSetpoint = selected
        return selected
    }

    /**
     * PDF-aligned dual-layer local avoidance:
     * - Mid/long range dynamic threat: prefer RL result
     * - Near/emergency threat: fall back to APF quick response
     */
    selectDynamicAvoidanceSetpoint(observation, rlFresh, apfSetpoint, cruiseSetpoint, threats, now) {
        const params = this.parameters
        const recovery = this.buildRecoverySetpoint(observation, now)
        if (this.shouldApfOwnControl(observation, threats)) {
            return apfSetpoint ?? recovery ?? rlFresh ?? cruiseSetpoint ?? hoverSetpoint(now, params.hoverValidityMs)
        }
        const nearestThreat = threats.length > 0
            ? threats.reduce((best, current) => current.range < best.range ? current : best, threats[0])
            : null
        const nearThreat = nearestThreat && (
            nearestThreat.range < params.emergencyDistanceThreshold * 1.5
            || nearestThreat.ttc < params.dynamicTtcThreshold * 0.6
        )

        if (nearThreat) {
            return rlFresh ?? recovery ?? apfSetpoint ?? cruiseSetpoint ?? hoverSetpoint(now, params.hoverValidityMs)
        }

        if (rlFresh) {
            return rlFresh
        }

        return recovery ?? cruiseSetpoint ?? apfSetpoint ?? hoverSetpoint(now, params.hoverValidityMs)
    }

    /**
     * @returns {{ mode: string, since: number }}
     */
    status() {
        return {
            mode: this.mode,
            since: this.modeSince,
            localDetourPoints: this.localDetourPath.length,
            localDetourIndex: this.localDetourIndex,
            localDetourObstacleId: this.localDetourObstacleId,
        }
    }

    /**
     * @param {number} now
     * @returns {MotionSetpoint | null}
     */
    latestFreshRlResult(now) {
        if (!this.lastRlResult) {
            return null
        }
        if ((this.lastRlResult.validUntil ?? -Infinity) < now) {
            return null
        }
        return this.lastRlResult
    }

    /**
     * @param {any} observation
     * @param {ThreatAssessment[]} threats
     * @param {number} now
     */
    transitionMode(observation, threats, now) {
        const params = this.parameters
        const inModeFor = now - this.modeSince
        const dynamicEmergency = this.isEmergencyThreat(threats)
        const dynamic = this.isDynamicThreat(threats)
        const staticEmergency = this.isStaticEmergency(observation)
        const staticWarning = this.isStaticWarning(observation)
        const staticBlockedAhead = this.isStaticBlockedAhead(observation)
        const emergency = dynamicEmergency || staticEmergency
        const clearOfEmergency = !emergency
        const onPath = this.isBackOnPath(observation)

        switch (this.mode) {
            case 'CRUISE':
                if (emergency) {
                    this.setMode('EMERGENCY_AVOID', now)
                } else if (staticBlockedAhead && inModeFor >= params.hysteresisEnterSeconds * 0.4) {
                    this.setMode('RECOVERY', now)
                } else if (dynamic && inModeFor >= params.hysteresisEnterSeconds) {
                    this.setMode('DYNAMIC_AVOID', now)
                }
                break

            case 'DYNAMIC_AVOID':
                if (emergency) {
                    this.setMode('EMERGENCY_AVOID', now)
                } else if (staticBlockedAhead && !dynamic) {
                    this.setMode('RECOVERY', now)
                } else if (!dynamic && !staticBlockedAhead && inModeFor >= params.hysteresisExitSeconds) {
                    this.setMode('RECOVERY', now)
                }
                break

            case 'EMERGENCY_AVOID':
                if (clearOfEmergency && inModeFor >= params.recoveryClearSeconds) {
                    this.setMode('RECOVERY', now)
                }
                break

            case 'RECOVERY':
                if (emergency) {
                    this.setMode('EMERGENCY_AVOID', now)
                } else if (dynamic) {
                    this.setMode('DYNAMIC_AVOID', now)
                } else if (onPath && !staticWarning && inModeFor >= params.hysteresisExitSeconds) {
                    this.setMode('CRUISE', now)
                }
                break
        }
    }

    /**
     * @param {ThreatAssessment[]} threats
     * @returns {boolean}
     */
    isDynamicThreat(threats) {
        const params = this.parameters
        return threats.some(threat =>
            threat.threatScore > params.dynamicThreatScoreThreshold
            || threat.ttc < params.dynamicTtcThreshold
        )
    }

    /**
     * @param {ThreatAssessment[]} threats
     * @returns {boolean}
     */
    isEmergencyThreat(threats) {
        const params = this.parameters
        return threats.some(threat =>
            threat.ttc < params.emergencyTtcThreshold
            || threat.range < params.emergencyDistanceThreshold
        )
    }

    /**
     * @param {any} observation
     * @returns {boolean}
     */
    isBackOnPath(observation) {
        const params = this.parameters
        if (this.localDetourPath.length > 0 && this.localDetourIndex < this.localDetourPath.length - 1) {
            return false
        }
        if (!this.lastGlobalPath || this.lastGlobalPath.length === 0) {
            return true
        }
        const position = vec3FromArray(observation.ego.position)
        const deviation = pathDeviation(position, this.lastGlobalPath)
        return deviation < params.recoveryPathDeviationThreshold
    }

    /**
     * @param {any} observation
     * @returns {boolean}
     */
    isStaticWarning(observation) {
        const params = this.parameters
        return !!observation && (
            observation?.collision?.hasPhysicalContact
            || observation?.collision?.isColliding
            || currentObstacleDistance(observation) < params.staticWarningDistance
        )
    }

    /**
     * @param {any} observation
     * @returns {boolean}
     */
    isStaticEmergency(observation) {
        const params = this.parameters
        return !!observation && (
            observation?.collision?.hasPhysicalContact
            || currentObstacleDistance(observation) < params.staticEmergencyDistance
        )
    }

    shouldApfOwnControl(observation, threats) {
        const params = this.parameters
        const dynamicEmergency = this.isEmergencyThreat(threats)
        const hasContact = !!observation?.collision?.hasPhysicalContact
        const obstacleDistance = currentObstacleDistance(observation)
        const rayProfile = frontRayProfile(observation, Infinity)
        const hasCommittedDetour =
            this.localDetourPath.length > 0 &&
            this.localDetourIndex < this.localDetourPath.length

        if (dynamicEmergency) {
            return true
        }

        if (rayProfile.front < params.proactiveFrontHardDistance) {
            return true
        }

        if (obstacleDistance < params.staticHardEmergencyDistance) {
            return true
        }

        if (hasContact && !hasCommittedDetour) {
            return true
        }

        return (
            hasContact && obstacleDistance < Math.max(params.staticEmergencyDistance * 0.6, params.staticHardEmergencyDistance)
        )
    }

    /**
     * @param {any} observation
     * @returns {boolean}
     */
    isStaticBlockedAhead(observation) {
        const params = this.parameters
        const tangent = this.pathDirection(observation)
        const ego = vec3FromArray(observation.ego.position)
        let severity = 0
        const rayProfile = frontRayProfile(observation, params.staticBlockedAheadDistance)

        if (rayProfile.front < params.proactiveFrontDistance) {
            const frontScore = clamp01(1 - rayProfile.front / Math.max(params.proactiveFrontDistance, 1))
            const sideScore = clamp01(Math.max(rayProfile.left, rayProfile.right) / Math.max(params.proactiveFrontDistance, 1))
            severity = Math.max(severity, frontScore * 0.82 + sideScore * 0.18)
        }

        for (const obstacle of preferredStaticObstacles(observation)) {
            const point = closestHorizontalPointOnObstacle(ego, obstacle)
            let offset = point.clone().sub(ego)
            if (offset.lengthSq() < 1e-9) {
                offset = vec3FromArray(obstacle.position).sub(ego)
            }
            const distance = Math.max(horizontalClearanceToObstacle(ego, obstacle), 0)
            if (distance > params.staticBlockedAheadDistance) {
                continue
            }
            const direction = offset.lengthSq() > 1e-9 ? offset.clone().normalize() : tangent.clone()
            const alignment = tangent.dot(direction)
            if (alignment < params.staticBlockedAheadConeDot) {
                continue
            }
            const score =
                clamp01(1 - distance / params.staticBlockedAheadDistance) * 0.65 +
                clamp01((alignment - params.staticBlockedAheadConeDot) / (1 - params.staticBlockedAheadConeDot)) * 0.35
            severity = Math.max(severity, score)
        }

        return severity > 0.45
    }

    pathDirection(observation, preferLocalDetour = true) {
        if (preferLocalDetour && this.localDetourPath.length > 0) {
            const ego = vec3FromArray(observation.ego.position)
            const target = vec3FromArray(this.localDetourPath[Math.min(this.localDetourIndex, this.localDetourPath.length - 1)])
            return safeDirection(ego, target, new THREE.Vector3(1, 0, 0))
        }

        if (this.lastGlobalPath && this.lastGlobalPath.length > 1) {
            const ego = vec3FromArray(observation.ego.position)
            let bestIndex = 0
            let bestDistance = Infinity
            for (let i = 0; i < this.lastGlobalPath.length; i += 1) {
                const point = vec3FromArray(this.lastGlobalPath[i])
                const distance = point.distanceTo(ego)
                if (distance < bestDistance) {
                    bestDistance = distance
                    bestIndex = i
                }
            }
            const nextIndex = Math.min(bestIndex + 1, this.lastGlobalPath.length - 1)
            return safeDirection(ego, vec3FromArray(this.lastGlobalPath[nextIndex]))
        }

        const ego = vec3FromArray(observation.ego.position)
        return safeDirection(ego, vec3FromArray(observation.goal))
    }

    /**
     * @param {any} observation
     * @param {number} now
     * @returns {MotionSetpoint}
     */
    buildRecoverySetpoint(observation, now) {
        const params = this.parameters
        const egoPosition = vec3FromArray(observation.ego.position)
        const egoVelocity = vec3FromArray(observation.ego.linearVelocity)
        const direction = this.pathDirection(observation, false)
        const rayProfile = frontRayProfile(observation, params.detourObstacleInfluenceDistance)
        const cruiseZ = Number.isFinite(observation.cruiseZ)
            ? observation.cruiseZ
            : observation.ego.position[2]

        let target = null
        this.advanceLocalDetour(observation)
        if (this.localDetourPath.length > 0) {
            target = vec3FromArray(this.localDetourPath[Math.min(this.localDetourIndex, this.localDetourPath.length - 1)])
        }
        if (this.lastGlobalPath && this.lastGlobalPath.length > 0) {
            target = target ?? this.targetAlongPath(
                this.lastGlobalPath,
                observation.ego.position,
                params.recoveryLookaheadDistance,
            ) ?? vec3FromArray(this.lastGlobalPath[this.lastGlobalPath.length - 1])
        }
        if (!target) {
            target = vec3FromArray(observation.goal)
        }

        const blocker = this.nearestStaticAhead(observation, direction)
        let desiredVector = direction.clone().multiplyScalar(params.detourForwardGain)
        if (target) {
            const toTarget = safeDirection(egoPosition, target, direction)
            desiredVector = toTarget.multiplyScalar(params.recoverySpeed)
        }

        if (!this.localDetourPath.length && blocker) {
            const blockerPoint = closestHorizontalPointOnObstacle(egoPosition, blocker)
            const offset = blockerPoint.clone().sub(egoPosition)
            const cross = direction.x * offset.y - direction.y * offset.x
            const sign = cross >= 0 ? -1 : 1
            const lateral = lateralDirection(direction, sign)
            const proximity = clamp01(1 - blocker.distance / params.detourObstacleInfluenceDistance)
            desiredVector.add(lateral.clone().multiplyScalar(params.detourLateralGain * Math.max(proximity, 0.8)))
        }

        desiredVector.z = 0

        if (rayProfile.front < params.proactiveFrontDistance) {
            const sign = this.preferredDetourSign(
                observation,
                direction,
                rayProfile.right > rayProfile.left ? 1 : -1,
            )
            const lateral = lateralDirection(direction, sign)
            const frontProximity = clamp01(1 - rayProfile.front / Math.max(params.proactiveFrontDistance, 1))
            const sideGap = Math.abs(rayProfile.right - rayProfile.left)
            const sideBias = clamp01(sideGap / Math.max(params.proactiveFrontDistance, 1))

            desiredVector.add(
                lateral.multiplyScalar(
                    params.proactiveSidePreferenceGain * (0.9 + frontProximity * 1.4 + sideBias * 0.8),
                ),
            )

            desiredVector.add(
                direction.clone().multiplyScalar(
                    -params.recoverySpeed * params.proactiveForwardSuppression * frontProximity,
                ),
            )

            const narrowSides = Math.max(rayProfile.left, rayProfile.right) < params.proactiveClimbSideThreshold
            if (
                rayProfile.front < params.proactiveClimbDistance &&
                rayProfile.up > Math.max(rayProfile.front, rayProfile.left, rayProfile.right) + 0.5 &&
                narrowSides
            ) {
                desiredVector.z = -params.proactiveClimbSpeed * (1 + frontProximity)
            }
        }

        const goalForward = safeDirection(egoPosition, vec3FromArray(observation.goal), direction)
        const forwardSpeed = desiredVector.dot(goalForward)
        if (forwardSpeed < params.recoveryMinForwardSpeed) {
            desiredVector.add(goalForward.multiplyScalar(params.recoveryMinForwardSpeed - forwardSpeed))
        }
        if (rayProfile.front >= params.proactiveClimbDistance || Math.abs(desiredVector.z) < 1e-6) {
            desiredVector.z = 0
        }

        const desiredVelocity = clampMagnitude(
            desiredVector.sub(egoVelocity.clone().multiplyScalar(0.15)),
            params.recoverySpeed,
        )

        const desiredYaw = Math.atan2(desiredVelocity.y, desiredVelocity.x)
        const currentYaw = observation.ego.orientation[2] ?? 0
        const yawError = Math.atan2(Math.sin(desiredYaw - currentYaw), Math.cos(desiredYaw - currentYaw))

        return {
            velocityWorld: [desiredVelocity.x, desiredVelocity.y, desiredVelocity.z],
            yawRate: THREE.MathUtils.clamp(yawError, -1.0, 1.0),
            source: 'recovery',
            priority: params.recoveryPriority,
            validUntil: now + params.hoverValidityMs / 1000,
        }
    }

    refreshLocalDetour(observation, staticBlockedAhead) {
        const params = this.parameters
        const now = observation.timestamp
        const baseDirection = this.pathDirection(observation, false)
        const blocker = this.nearestStaticAhead(observation, baseDirection)
        const needsDetour = !!blocker && (
            staticBlockedAhead
            || blocker.distance < params.detourObstacleInfluenceDistance * 0.9
        )

        if (!needsDetour) {
            if (this.localDetourPath.length > 0) {
                const ego = vec3FromArray(observation.ego.position)
                const finalPoint = vec3FromArray(this.localDetourPath[this.localDetourPath.length - 1])
                const nearFinal = ego.distanceTo(finalPoint) < Math.max(params.recoveryPathDeviationThreshold, 2.5)
                if (nearFinal || this.isBackOnPath({ ...observation, collision: observation.collision })) {
                    this.clearLocalDetour()
                }
            }
            return
        }

        if (
            this.localDetourPath.length > 0 &&
            now < this.localDetourLockUntil &&
            this.localDetourIndex < this.localDetourPath.length - 1
        ) {
            return
        }

        if (
            this.localDetourPath.length === 0
            || this.localDetourObstacleId !== blocker.id
        ) {
            this.buildLocalDetourPath(observation, blocker, baseDirection, now)
        }
    }

    clearLocalDetour() {
        this.localDetourPath = []
        this.localDetourIndex = 0
        this.localDetourObstacleId = null
        this.localDetourLockUntil = -Infinity
    }

    advanceLocalDetour(observation) {
        if (this.localDetourPath.length === 0) {
            return
        }
        const ego = vec3FromArray(observation.ego.position)
        while (this.localDetourIndex < this.localDetourPath.length - 1) {
            const waypoint = vec3FromArray(this.localDetourPath[this.localDetourIndex])
            if (ego.distanceTo(waypoint) > this.parameters.detourWaypointReachDistance) {
                break
            }
            this.localDetourIndex += 1
        }
    }

    buildLocalDetourPath(observation, blocker, baseDirection, now = observation.timestamp) {
        const params = this.parameters
        const ego = vec3FromArray(observation.ego.position)
        const cruiseZ = Number.isFinite(observation.cruiseZ)
            ? observation.cruiseZ
            : observation.ego.position[2]
        const blockerPoint = closestHorizontalPointOnObstacle(ego, blocker)
        const blockerCenter = vec3FromArray(blocker.position).setZ(cruiseZ)
        const offset = blockerPoint.clone().sub(ego)
        const cross = baseDirection.x * offset.y - baseDirection.y * offset.x
        const sign = this.preferredDetourSign(observation, baseDirection, cross >= 0 ? -1 : 1)
        this.localDetourSign = sign

        const lateral = lateralDirection(baseDirection, sign)
        const extents = obstacleHalfExtents(blocker)
        const lateralExtent = Math.abs(lateral.x) * extents.x + Math.abs(lateral.y) * extents.y
        const forwardExtent = Math.abs(baseDirection.x) * extents.x + Math.abs(baseDirection.y) * extents.y
        const rayDistances = observation?.rayDistances ?? {}
        const frontDistance = Math.min(
            rayDistances.front ?? params.detourObstacleInfluenceDistance,
            rayDistances.frontLeft ?? params.detourObstacleInfluenceDistance,
            rayDistances.frontRight ?? params.detourObstacleInfluenceDistance,
            params.detourObstacleInfluenceDistance,
        )
        const proximityBoost = clamp01(1 - blocker.distance / Math.max(params.detourObstacleInfluenceDistance, 1))
        const extraClear =
            params.detourExtraClearance +
            proximityBoost * 2.5 +
            clamp01(1 - frontDistance / Math.max(params.detourObstacleInfluenceDistance, 1)) * 2.0
        const lateralClear = lateralExtent + params.detourLateralClearance + extraClear
        const forwardClear = forwardExtent + params.detourForwardClearance + extraClear

        const preEntry = clampForwardProgress(
            ego.clone()
                .add(lateral.clone().multiplyScalar(Math.max(lateralClear * 0.55, 3.0)))
                .setZ(cruiseZ),
            ego,
            baseDirection,
            1.0,
        )
        const entry = clampForwardProgress(
            blockerCenter.clone()
            .add(lateral.clone().multiplyScalar(lateralClear))
            .add(baseDirection.clone().multiplyScalar(-Math.max(forwardClear, 5.0))),
            ego,
            baseDirection,
            2.0,
        )
        const sideNear = clampForwardProgress(
            blockerCenter.clone()
            .add(lateral.clone().multiplyScalar(lateralClear))
            .add(baseDirection.clone().multiplyScalar(-Math.max(forwardClear * 0.35, 2.5))),
            ego,
            baseDirection,
            3.5,
        )
        const side = clampForwardProgress(
            blockerCenter.clone()
                .add(lateral.clone().multiplyScalar(lateralClear)),
            ego,
            baseDirection,
            5.0,
        )
        const exit = clampForwardProgress(
            blockerCenter.clone()
            .add(lateral.clone().multiplyScalar(lateralClear))
            .add(baseDirection.clone().multiplyScalar(forwardClear)),
            ego,
            baseDirection,
            7.0,
        )
        const clear = clampForwardProgress(
            exit.clone()
                .add(baseDirection.clone().multiplyScalar(Math.max(params.detourForwardClearance + extraClear * 0.5, 4.0))),
            ego,
            baseDirection,
            9.5,
        )
        const mergeTarget = this.targetAlongPath(
            this.lastGlobalPath,
            observation.ego.position,
            params.recoveryLookaheadDistance * 1.8,
        )
        const merge = mergeTarget
            ? mergeTarget.clone()
            : exit.clone().add(baseDirection.clone().multiplyScalar(Math.max(params.detourForwardClearance, 4.0)))

        const rawPath = [preEntry, entry, sideNear, side, exit, clear, merge]
            .filter(point => Number.isFinite(point.x) && Number.isFinite(point.y) && Number.isFinite(point.z))
            .map(point => [point.x, point.y, point.z])

        this.localDetourPath = rawPath.filter((point, index, array) => {
            if (index === 0) return true
            const prev = array[index - 1]
            return Math.hypot(point[0] - prev[0], point[1] - prev[1], point[2] - prev[2]) > 0.6
        })
        this.localDetourIndex = 0
        this.localDetourObstacleId = blocker.id
        this.localDetourLockUntil = now + params.detourLockSeconds
        this.advanceLocalDetour(observation)
    }

    preferredDetourSign(observation, direction, fallbackSign = 1) {
        const rays = observation?.rayDistances ?? {}
        const leftClear =
            (rays.left ?? 0) * 0.8 +
            (rays.frontLeft ?? rays.left ?? 0) * 1.2 +
            (rays.leftUp ?? rays.left ?? 0) * 0.35 +
            (rays.leftDown ?? rays.left ?? 0) * 0.2
        const rightClear =
            (rays.right ?? 0) * 0.8 +
            (rays.frontRight ?? rays.right ?? 0) * 1.2 +
            (rays.rightUp ?? rays.right ?? 0) * 0.35 +
            (rays.rightDown ?? rays.right ?? 0) * 0.2

        if (Math.abs(rightClear - leftClear) < 0.75) {
            return this.localDetourObstacleId ? this.localDetourSign : fallbackSign
        }

        const signFromClearance = rightClear > leftClear ? 1 : -1
        if (direction.lengthSq() < 1e-9) {
            return signFromClearance
        }
        return signFromClearance
    }

    nearestStaticAhead(observation, direction) {
        const params = this.parameters
        const ego = vec3FromArray(observation.ego.position)
        let best = null
        let bestDistance = Infinity
        for (const obstacle of preferredStaticObstacles(observation)) {
            const point = closestHorizontalPointOnObstacle(ego, obstacle)
            let offset = point.clone().sub(ego)
            if (offset.lengthSq() < 1e-12) {
                offset = vec3FromArray(obstacle.position).sub(ego)
            }
            const distance = Math.max(horizontalClearanceToObstacle(ego, obstacle), 0)
            if (distance > params.detourObstacleInfluenceDistance) {
                continue
            }
            const dir = offset.lengthSq() > 1e-12 ? offset.clone().normalize() : direction.clone()
            const alignment = direction.dot(dir)
            if (alignment < params.staticBlockedAheadConeDot) {
                continue
            }
            if (distance < bestDistance) {
                bestDistance = distance
                best = { ...obstacle, distance, alignment }
            }
        }
        return best
    }

    targetAlongPath(path, position, lookaheadDistance) {
        if (!path || path.length === 0) {
            return null
        }
        if (path.length === 1) {
            return vec3FromArray(path[0])
        }

        const positionVec = vec3FromArray(position)
        let nearestIndex = 0
        let bestDistance = Infinity
        for (let i = 0; i < path.length; i += 1) {
            const point = vec3FromArray(path[i])
            const distance = point.distanceTo(positionVec)
            if (distance < bestDistance) {
                bestDistance = distance
                nearestIndex = i
            }
        }

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
                return new THREE.Vector3(
                    current[0] + (next[0] - current[0]) * t,
                    current[1] + (next[1] - current[1]) * t,
                    current[2] + (next[2] - current[2]) * t,
                )
            }
            remaining -= segmentLength
            current = next
        }

        return vec3FromArray(path[path.length - 1])
    }

    /**
     * @param {'CRUISE' | 'DYNAMIC_AVOID' | 'EMERGENCY_AVOID' | 'RECOVERY'} mode
     * @param {number} now
     */
    setMode(mode, now) {
        if (this.mode === mode) {
            return
        }
        this.mode = mode
        this.modeSince = now
    }
}
