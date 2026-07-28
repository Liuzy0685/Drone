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
 * Browser-side client for the asynchronous HPER-D3QN inference worker.
 */
export class HperD3qnClient {
    /**
     * @param {any} config
     * @param {{ workerUrl?: string }} options
     */
    constructor(config = {}, options = {}) {
        this.config = config
        this.workerUrl = options.workerUrl ?? new URL('../../workers/d3qn.worker.js', import.meta.url)
        this.worker = new Worker(this.workerUrl, { type: 'module' })
        this.seq = 0
        this.generation = 0
        this.lastRequestTime = -Infinity
        this.latestResult = null
        this.latestSeq = -1

        this.worker.addEventListener('message', (event) => {
            const data = event.data ?? {}
            if (data.type !== 'd3qnResult' || !data.result) {
                return
            }
            if ((data.result.generation ?? 0) !== this.generation) {
                return
            }
            if ((data.result.seq ?? -1) < this.latestSeq) {
                return
            }
            this.latestSeq = data.result.seq
            this.latestResult = {
                velocityWorld: data.result.velocityWorld,
                yawRate: data.result.yawRate,
                source: 'd3qn',
                priority: data.result.priority ?? this.parameters.priority,
                validUntil: data.result.validUntil,
                qValues: data.result.qValues ?? [],
                selectedAction: data.result.selectedAction ?? null,
                computedAt: data.result.computedAt ?? 0,
                seq: data.result.seq ?? -1,
            }
        })
    }

    /**
     * @returns {any}
     */
    get parameters() {
        const defaults = {
            requestHz: 15,
            resultTtlMs: 150,
            priority: 60,
            maxSpeed: 5.5,
            yawRateGain: 1.2,
            maxYawRate: 1.0,
            validityMs: 150,
        }
        return {
            ...defaults,
            ...(this.config.navigation?.d3qn ?? {}),
        }
    }

    /**
     * @param {any} observation
     */
    maybeRequestInference(observation, threats = []) {
        const params = this.parameters
        const minInterval = 1 / Math.max(params.requestHz, 1)
        if ((observation.timestamp - this.lastRequestTime) < minInterval) {
            return
        }

        this.lastRequestTime = observation.timestamp
        this.seq += 1

        this.worker.postMessage({
            type: 'infer',
            payload: {
                seq: this.seq,
                observation: {
                    ...observation,
                    localPointCloud: observation.localPointCloud ?? new Float32Array(0),
                },
                config: {
                    maxSpeed: params.maxSpeed,
                    yawRateGain: params.yawRateGain,
                    maxYawRate: params.maxYawRate,
                    validityMs: params.validityMs,
                    actionSet: params.actionSet,
                    threatConfig: this.config.navigation?.dtpa ?? {},
                },
                threats,
                generation: this.generation,
            },
        })
    }

    /**
     * @param {number} ttlMs
     * @returns {MotionSetpoint | null}
     */
    latestFreshResult(ttlMs = 150, nowSeconds = performance.now() / 1000) {
        if (!this.latestResult) {
            return null
        }
        const ttlSeconds = ttlMs / 1000
        if ((nowSeconds - this.latestResult.computedAt) > ttlSeconds) {
            return null
        }
        if ((nowSeconds - this.latestResult.computedAt) < 0) {
            return null
        }
        if ((this.latestResult.validUntil ?? -Infinity) < nowSeconds) {
            return null
        }
        return this.latestResult
    }

    reset() {
        this.generation += 1
        this.latestResult = null
        this.latestSeq = -1
        this.lastRequestTime = -Infinity
        this.worker.postMessage({ type: 'reset' })
    }

    dispose() {
        this.worker.terminate()
    }
}
