/**
 * @typedef {[number, number, number]} Vec3
 */

/**
 * @typedef {{
 *   seq: number,
 *   observation: {
 *     ego: {
 *       position: Vec3,
 *       orientation: Vec3,
 *       linearVelocity: Vec3,
 *       angularVelocity: Vec3,
 *     },
 *     goal: Vec3,
 *     globalPath: Vec3[],
 *     staticObstacles: Array<{
 *       id: string,
 *       type: string,
 *       position: Vec3,
 *       velocity: Vec3,
 *       size: number,
 *       dynamic: boolean,
 *       confidence: number,
 *     }>,
 *     dynamicObstacles: Array<{
 *       id: string,
 *       type: string,
 *       position: Vec3,
 *       velocity: Vec3,
 *       size: number,
 *       dynamic: boolean,
 *       confidence: number,
 *     }>,
 *     localPointCloud?: Float32Array,
 *     dt: number,
 *     timestamp: number,
 *   },
 *   config: any,
 *   generation?: number,
 *   threats?: Array<{
 *     obstacleId: string,
 *     type: string,
 *     threatScore: number,
 *     ttc: number,
 *     dcpa: number,
 *     range: number,
 *     closingSpeed: number,
 *     lastUpdated: number,
 *     obstacle: any,
 *   }>,
 * }} InferenceRequest
 */

const DEFAULT_CONFIG = {
    maxSpeed: 5.5,
    yawRateGain: 1.2,
    maxYawRate: 1.0,
    validityMs: 150,
    predictionHorizon: 2.4,
    staticAvoidDistance: 9.0,
    topThreatCount: 3,
    actionSet: [
        [1.0, 0.0, 0.0],
        [0.85, 0.35, 0.0],
        [0.85, -0.35, 0.0],
        [0.45, 0.85, 0.0],
        [0.45, -0.85, 0.0],
        [0.4, 0.0, -0.75],
        [0.4, 0.0, 0.75],
        [0.0, 0.0, 0.0],
    ],
    threatConfig: {
        lookaheadSeconds: 12,
        safeDistance: 12,
        emergencyDistance: 8,
        typeWeights: {
            uav: 1.0,
            car: 0.85,
            person: 0.7,
            building: 0.4,
            unknown: 0.6,
        },
    },
}

let latestPending = null
let running = false

function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max)
}

function vecLength(vec) {
    return Math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])
}

function normalize(vec, fallback = [0, 0, 0]) {
    const length = vecLength(vec)
    if (length < 1e-9) {
        return [...fallback]
    }
    return [vec[0] / length, vec[1] / length, vec[2] / length]
}

function add(a, b) {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

function sub(a, b) {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

function scale(vec, factor) {
    return [vec[0] * factor, vec[1] * factor, vec[2] * factor]
}

function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

function limit(vec, maxSpeed) {
    const length = vecLength(vec)
    if (length <= maxSpeed) {
        return vec
    }
    return scale(normalize(vec), maxSpeed)
}

function cross2d(a, b) {
    return a[0] * b[1] - a[1] * b[0]
}

function selectGoal(observation) {
    if (observation.globalPath && observation.globalPath.length > 0) {
        return observation.globalPath[Math.min(3, observation.globalPath.length - 1)]
    }
    return observation.goal
}

function pathDirection(observation) {
    return normalize(sub(selectGoal(observation), observation.ego.position), [1, 0, 0])
}

function assessThreat(egoPosition, egoVelocity, obstacle, config) {
    const threatConfig = {
        ...DEFAULT_CONFIG.threatConfig,
        ...(config.threatConfig ?? {}),
        typeWeights: {
            ...DEFAULT_CONFIG.threatConfig.typeWeights,
            ...(config.threatConfig?.typeWeights ?? {}),
        },
    }
    const obstaclePosition = obstacle.position
    const obstacleVelocity = obstacle.velocity ?? [0, 0, 0]
    const relativePosition = sub(obstaclePosition, egoPosition)
    const relativeVelocity = sub(obstacleVelocity, egoVelocity)
    const combinedRadius = Math.max(obstacle.size ?? 0, 0.4)
    const range = Math.max(vecLength(relativePosition) - combinedRadius, 0)

    const relativeSpeedSq = dot(relativeVelocity, relativeVelocity)
    let tcpa = Infinity
    let dcpa = range
    if (relativeSpeedSq > 1e-9) {
        tcpa = -dot(relativePosition, relativeVelocity) / relativeSpeedSq
        if (tcpa < 0) {
            tcpa = Infinity
        } else {
            const cpaVector = add(relativePosition, scale(relativeVelocity, tcpa))
            dcpa = Math.max(vecLength(cpaVector) - combinedRadius, 0)
        }
    }

    const typeWeight = threatConfig.typeWeights[obstacle.type] ?? threatConfig.typeWeights.unknown
    const tcpaScore = Number.isFinite(tcpa)
        ? clamp((threatConfig.lookaheadSeconds - tcpa) / threatConfig.lookaheadSeconds, 0, 1)
        : 0
    const dcpaScore = clamp((threatConfig.safeDistance - dcpa) / threatConfig.safeDistance, 0, 1)
    const rangeScore = clamp((threatConfig.safeDistance - range) / threatConfig.safeDistance, 0, 1)
    const confidenceScore = clamp(obstacle.confidence ?? 0.5, 0, 1)

    let threatScore =
        tcpaScore * 0.45 +
        dcpaScore * 0.35 +
        rangeScore * 0.15 +
        confidenceScore * 0.05
    threatScore *= typeWeight

    if (range <= threatConfig.emergencyDistance) {
        threatScore = Math.max(threatScore, 0.85 * typeWeight)
    }
    if (Number.isFinite(tcpa) && tcpa <= 1.5) {
        threatScore = Math.max(threatScore, 0.95 * typeWeight)
    }

    return {
        obstacle,
        threatScore: clamp(threatScore, 0, 1),
        range,
        tcpa,
        dcpa,
        relativePosition,
        relativeVelocity,
    }
}

function staticPointPenalty(candidateDirection, egoPosition, pointCloud, config) {
    if (!pointCloud || pointCloud.length < 3 || vecLength(candidateDirection) < 1e-6) {
        return 0
    }

    const direction = normalize(candidateDirection)
    let penalty = 0
    const sampleCount = Math.min(pointCloud.length / 3, 48)
    for (let i = 0; i < sampleCount * 3; i += 3) {
        const point = [pointCloud[i], pointCloud[i + 1], pointCloud[i + 2]]
        const offset = sub(point, egoPosition)
        const distance = vecLength(offset)
        if (distance < 1e-6 || distance > config.staticAvoidDistance) {
            continue
        }

        const alignment = dot(normalize(offset), direction)
        if (alignment <= 0.35) {
            continue
        }

        penalty += alignment * (1 - clamp(distance / config.staticAvoidDistance, 0, 1))
    }

    return penalty
}

function deriveThreatSet(request, config) {
    const threats = []
    if (Array.isArray(request.threats) && request.threats.length > 0) {
        const sortedThreats = [...request.threats].sort((a, b) => b.threatScore - a.threatScore)
        for (const ranked of sortedThreats.slice(0, config.topThreatCount ?? 3)) {
            if (!ranked?.obstacle) continue
            threats.push(assessThreat(
                request.observation.ego.position,
                request.observation.ego.linearVelocity,
                ranked.obstacle,
                config,
            ))
        }
        return threats
    }

    const egoPosition = request.observation.ego.position
    const egoVelocity = request.observation.ego.linearVelocity
    for (const obstacle of request.observation.dynamicObstacles ?? []) {
        threats.push(assessThreat(egoPosition, egoVelocity, obstacle, config))
    }
    threats.sort((a, b) => b.threatScore - a.threatScore)
    return threats.slice(0, config.topThreatCount ?? 3)
}

function evaluateAction(action, request, config, threatSet) {
    const observation = request.observation
    const egoPosition = observation.ego.position
    const egoVelocity = observation.ego.linearVelocity
    const goal = selectGoal(observation)
    const goalDir = normalize(sub(goal, egoPosition), [1, 0, 0])
    const routeDir = pathDirection(observation)
    const candidateVelocity = limit(scale(action, config.maxSpeed), config.maxSpeed)
    const candidateDirection = normalize(candidateVelocity, goalDir)
    const candidateSpeed = vecLength(candidateVelocity)

    const goalAlignment = dot(candidateDirection, goalDir)
    const routeAlignment = dot(candidateDirection, routeDir)
    const currentGoalAlignment = dot(normalize(egoVelocity, goalDir), goalDir)
    const alignmentGain = goalAlignment - currentGoalAlignment * 0.35
    const verticalPenalty = Math.abs(action[2]) * 0.12
    const staticPenalty = staticPointPenalty(candidateDirection, egoPosition, observation.localPointCloud, config)

    let score =
        goalAlignment * 0.85 +
        routeAlignment * 0.55 +
        alignmentGain * 0.45 -
        verticalPenalty -
        staticPenalty * 0.8

    if (!threatSet || threatSet.length === 0) {
        const cruiseBias = candidateSpeed > 0.15 ? 0.2 : -0.15
        return score + cruiseBias
    }

    let aggregateThreatUtility = 0
    let brakingBonus = 0
    for (let i = 0; i < threatSet.length; i += 1) {
        const rankedThreat = threatSet[i]
        const currentThreat = assessThreat(egoPosition, egoVelocity, rankedThreat.obstacle, config)
        const candidateThreat = assessThreat(egoPosition, candidateVelocity, rankedThreat.obstacle, config)
        const awayDirection = normalize(scale(currentThreat.relativePosition, -1), [0, 0, 0])
        const awayScore = dot(candidateDirection, awayDirection)
        const lateralReference = [
            -currentThreat.relativePosition[1],
            currentThreat.relativePosition[0],
            0,
        ]
        const lateralDir = normalize(lateralReference, [0, 1, 0])
        const lateralScore = Math.abs(dot(candidateDirection, lateralDir))
        const tcpaImprovement = Number.isFinite(currentThreat.tcpa) && Number.isFinite(candidateThreat.tcpa)
            ? clamp((candidateThreat.tcpa - currentThreat.tcpa) / Math.max(config.predictionHorizon, 1), -1, 1)
            : (Number.isFinite(candidateThreat.tcpa) ? 0.15 : 0.35)
        const dcpaImprovement = clamp(
            (candidateThreat.dcpa - currentThreat.dcpa) / Math.max(config.threatConfig.safeDistance ?? 12, 1),
            -1,
            1,
        )
        const rangeGain = clamp(
            (candidateThreat.range - currentThreat.range) / Math.max(config.threatConfig.safeDistance ?? 12, 1),
            -1,
            1,
        )
        const sidePreference = Math.sign(cross2d(currentThreat.relativePosition, currentThreat.relativeVelocity)) || 1
        const directionalSide = dot(candidateDirection, scale(lateralDir, sidePreference))
        const obstacleVerticalOffset = currentThreat.relativePosition[2]
        const climbPreference = obstacleVerticalOffset > 0 ? -1 : 1
        const verticalEscapeScore = candidateDirection[2] * climbPreference
        const rankWeight = (rankedThreat.threatScore ?? 0) * Math.max(1 - i * 0.18, 0.45)

        aggregateThreatUtility += rankWeight * (
            awayScore * 1.25 +
            lateralScore * 0.65 +
            directionalSide * 0.22 +
            verticalEscapeScore * 0.4 +
            tcpaImprovement * 0.9 +
            dcpaImprovement * 1.1 +
            rangeGain * 0.6
        )

        if (candidateSpeed < 0.3 && currentThreat.threatScore > 0.85) {
            brakingBonus += 0.25 * rankWeight
        }

        if (currentThreat.range < (config.threatConfig.emergencyDistance ?? 8) * 0.8) {
            score -= staticPenalty * 0.15 * rankWeight
            aggregateThreatUtility += awayScore * 0.35 * rankWeight
        }
    }

    score += aggregateThreatUtility + brakingBonus
    return score
}

function decodeAction(bestAction, observation, config) {
    const velocity = limit(scale(bestAction, config.maxSpeed), config.maxSpeed)
    const desiredYaw = Math.atan2(velocity[1], velocity[0] || 1e-9)
    const currentYaw = observation.ego.orientation?.[2] ?? 0
    const yawError = Math.atan2(Math.sin(desiredYaw - currentYaw), Math.cos(desiredYaw - currentYaw))

    return {
        velocityWorld: velocity,
        yawRate: clamp(yawError * config.yawRateGain, -config.maxYawRate, config.maxYawRate),
    }
}

/**
 * HPER-D3QN style browser worker.
 * 当前版本仍然不是训练后的神经网络权重推理，而是：
 * 1. 使用 DTPA 风险估计抽取主威胁；
 * 2. 对离散动作集合做确定性评分；
 * 3. 输出与接口兼容的速度指令。
 * 后续只需要把 `evaluateAction` 前的评分器替换成真实 dueling DQN 前向推理即可。
 */
function runThreatAwareInference(request) {
    const config = {
        ...DEFAULT_CONFIG,
        ...(request.config ?? {}),
        threatConfig: {
            ...DEFAULT_CONFIG.threatConfig,
            ...(request.config?.threatConfig ?? {}),
            typeWeights: {
                ...DEFAULT_CONFIG.threatConfig.typeWeights,
                ...(request.config?.threatConfig?.typeWeights ?? {}),
            },
        },
    }

    const threatSet = deriveThreatSet(request, config)
    const dominantThreat = threatSet[0] ?? null
    const qValues = config.actionSet.map(action => evaluateAction(action, request, config, threatSet))

    let bestIndex = 0
    let bestValue = -Infinity
    for (let i = 0; i < qValues.length; i += 1) {
        if (qValues[i] > bestValue) {
            bestValue = qValues[i]
            bestIndex = i
        }
    }

    const selectedAction = config.actionSet[bestIndex]
    const decoded = decodeAction(selectedAction, request.observation, config)

    return {
        seq: request.seq,
        generation: request.generation ?? 0,
        qValues,
        selectedAction,
        velocityWorld: decoded.velocityWorld,
        yawRate: decoded.yawRate,
        source: 'd3qn',
        priority: 60,
        validUntil: request.observation.timestamp + config.validityMs / 1000,
        computedAt: request.observation.timestamp,
        dominantThreat: dominantThreat ? {
            obstacleId: dominantThreat.obstacle.id,
            threatScore: dominantThreat.threatScore,
            range: dominantThreat.range,
            tcpa: dominantThreat.tcpa,
            dcpa: dominantThreat.dcpa,
        } : null,
        trackedThreatCount: threatSet.length,
    }
}

async function processLoop() {
    if (running) {
        return
    }
    running = true

    while (latestPending) {
        const request = latestPending
        latestPending = null
        const result = runThreatAwareInference(request)
        postMessage({ type: 'd3qnResult', result })
        await Promise.resolve()
    }

    running = false
}

self.addEventListener('message', (event) => {
    const data = event.data ?? {}
    if (data.type === 'infer') {
        latestPending = data.payload
        processLoop()
        return
    }

    if (data.type === 'reset') {
        latestPending = null
    }
})
