import { THREE } from '../three.js'

/**
 * @typedef {[number, number, number]} Vec3
 */

/**
 * @typedef {{
 *   id: string,
 *   type: 'uav' | 'car' | 'person' | 'building' | 'unknown',
 *   position: Vec3,
 *   velocity: Vec3,
 *   size: number,
 *   dynamic: boolean,
 *   confidence: number,
 * }} ObstacleTrack
 */

/**
 * @typedef {{
 *   ego: {
 *     position: Vec3,
 *     orientation: Vec3,
 *     linearVelocity: Vec3,
 *     angularVelocity: Vec3,
 *   },
 *   dynamicObstacles: ObstacleTrack[],
 *   timestamp: number,
 * }} NavigationObservation
 */

/**
 * @typedef {{
 *   obstacleId: string,
 *   type: ObstacleTrack['type'],
 *   threatScore: number,
 *   ttc: number,
 *   dcpa: number,
 *   range: number,
 *   closingSpeed: number,
 *   lastUpdated: number,
 *   obstacle: ObstacleTrack,
 * }} ThreatAssessment
 */

function vec3FromArray(v) {
    return new THREE.Vector3(v[0], v[1], v[2])
}

function clamp01(value) {
    return THREE.MathUtils.clamp(value, 0, 1)
}

/**
 * Dynamic Threat and Proximity Assessment cache.
 * Update rate is intentionally lower than the physics loop.
 */
export class ThreatAssessor {
    /**
     * @param {any} config
     */
    constructor(config = {}) {
        this.config = config
        this.lastUpdateTime = -Infinity
        /** @type {Map<string, ThreatAssessment>} */
        this.cache = new Map()
    }

    /**
     * @returns {any}
     */
    get parameters() {
        const defaults = {
            updateHz: 25,
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
            scoreWeights: {
                tcpa: 0.45,
                dcpa: 0.35,
                range: 0.15,
                confidence: 0.05,
            },
        }
        return {
            ...defaults,
            ...(this.config.navigation?.dtpa ?? {}),
            typeWeights: {
                ...defaults.typeWeights,
                ...(this.config.navigation?.dtpa?.typeWeights ?? {}),
            },
            scoreWeights: {
                ...defaults.scoreWeights,
                ...(this.config.navigation?.dtpa?.scoreWeights ?? {}),
            },
        }
    }

    /**
     * @param {NavigationObservation} observation
     * @returns {ThreatAssessment[]}
     */
    update(observation) {
        const params = this.parameters
        const minUpdateInterval = 1 / Math.max(params.updateHz, 1)

        if ((observation.timestamp - this.lastUpdateTime) < minUpdateInterval) {
            return this.latest()
        }

        this.lastUpdateTime = observation.timestamp
        this.cache.clear()

        const egoPosition = vec3FromArray(observation.ego.position)
        const egoVelocity = vec3FromArray(observation.ego.linearVelocity)

        for (const obstacle of observation.dynamicObstacles ?? []) {
            const assessment = this.assessObstacle(
                egoPosition,
                egoVelocity,
                obstacle,
                observation.timestamp,
            )
            this.cache.set(obstacle.id, assessment)
        }

        return this.latest()
    }

    /**
     * @returns {ThreatAssessment[]}
     */
    latest() {
        return [...this.cache.values()].sort((a, b) => b.threatScore - a.threatScore)
    }

    /**
     * @param {string} obstacleId
     * @returns {ThreatAssessment | null}
     */
    get(obstacleId) {
        return this.cache.get(obstacleId) ?? null
    }

    reset() {
        this.lastUpdateTime = -Infinity
        this.cache.clear()
    }

    /**
     * @param {THREE.Vector3} egoPosition
     * @param {THREE.Vector3} egoVelocity
     * @param {ObstacleTrack} obstacle
     * @param {number} timestamp
     * @returns {ThreatAssessment}
     */
    assessObstacle(egoPosition, egoVelocity, obstacle, timestamp) {
        const params = this.parameters
        const obstaclePosition = vec3FromArray(obstacle.position)
        const obstacleVelocity = vec3FromArray(obstacle.velocity)

        const relativePosition = obstaclePosition.clone().sub(egoPosition)
        const relativeVelocity = obstacleVelocity.clone().sub(egoVelocity)
        const combinedRadius = Math.max(obstacle.size ?? 0, 0) + (this.config.aircraft?.boundingBox?.size?.[0] ?? 0.3) * 0.5
        const range = Math.max(relativePosition.length() - combinedRadius, 0)

        const relativeSpeedSq = relativeVelocity.lengthSq()
        let tcpa = Infinity
        let dcpa = range

        if (relativeSpeedSq > 1e-9) {
            tcpa = -relativePosition.dot(relativeVelocity) / relativeSpeedSq
            if (tcpa < 0) {
                tcpa = Infinity
            } else {
                const cpaVector = relativePosition.clone().addScaledVector(relativeVelocity, tcpa)
                dcpa = Math.max(cpaVector.length() - combinedRadius, 0)
            }
        }

        const lineOfSight = relativePosition.lengthSq() > 1e-12
            ? relativePosition.clone().normalize()
            : new THREE.Vector3()
        const closingSpeed = Math.max(egoVelocity.clone().sub(obstacleVelocity).dot(lineOfSight), 0)

        const tcpaScore = Number.isFinite(tcpa)
            ? clamp01((params.lookaheadSeconds - tcpa) / params.lookaheadSeconds)
            : 0
        const dcpaScore = clamp01((params.safeDistance - dcpa) / params.safeDistance)
        const rangeScore = clamp01((params.safeDistance - range) / params.safeDistance)
        const confidenceScore = clamp01(obstacle.confidence ?? 0.5)
        const typeWeight = params.typeWeights[obstacle.type] ?? params.typeWeights.unknown

        let threatScore =
            params.scoreWeights.tcpa * tcpaScore +
            params.scoreWeights.dcpa * dcpaScore +
            params.scoreWeights.range * rangeScore +
            params.scoreWeights.confidence * confidenceScore

        threatScore *= typeWeight

        if (range <= params.emergencyDistance) {
            threatScore = Math.max(threatScore, 0.85 * typeWeight)
        }
        if (Number.isFinite(tcpa) && tcpa <= 1.5) {
            threatScore = Math.max(threatScore, 0.95 * typeWeight)
        }

        return {
            obstacleId: obstacle.id,
            type: obstacle.type,
            threatScore: clamp01(threatScore),
            ttc: Number.isFinite(tcpa) ? tcpa : Infinity,
            dcpa,
            range,
            closingSpeed,
            lastUpdated: timestamp,
            obstacle,
        }
    }
}
