import { THREE } from '../../three.js'

/**
 * @typedef {[number, number, number]} Vec3
 */

/**
 * @typedef {{
 *   position: Vec3,
 *   orientation: Vec3,
 *   linearVelocity: Vec3,
 *   angularVelocity: Vec3,
 * }} VehicleState
 */

/**
 * @typedef {{
 *   id: string,
 *   type: 'uav'|'car'|'person'|'building'|'unknown',
 *   position: Vec3,
 *   velocity: Vec3,
 *   size: number,
 *   dynamic: boolean,
 *   confidence: number,
 * }} ObstacleTrack
 */

/**
 * @typedef {{
 *   ego: VehicleState,
 *   goal: Vec3,
 *   globalPath: Vec3[],
 *   staticObstacles: ObstacleTrack[],
 *   dynamicObstacles: ObstacleTrack[],
 *   localPointCloud?: Float32Array,
 *   dt: number,
 *   timestamp: number,
 *   rayDistances: Record<string, number>,
 *   collision: { isColliding: boolean, minDistance: number },
 * }} NavigationObservation
 */

/**
 * Shared Rapier sampling adapter.
 */
export class RapierEnvironmentAdapter {
    /**
     * @param {any} config
     * @param {any} rapier
     */
    constructor(config, rapier) {
        this.config = config
        this.RAPIER = rapier
        this.lastStepId = -1
        this.lastObservation = null
    }

    /**
     * @returns {{
     *   maxRayDistance: number,
     *   pointCloudRange: number,
     *   collisionDistance: number,
     *   contactConfirmationDistance: number,
     *   includeDiagonalRays: boolean,
     *   diagonalRaySpreadDeg: number,
     *   verticalRaySpreadDeg: number,
     * }}
     */
    get sensingConfig() {
        const sensing = this.config.navigation?.sensing ?? {}
        return {
            maxRayDistance: sensing.maxRayDistance ?? 200,
            pointCloudRange: sensing.pointCloudRange ?? 50,
            collisionDistance: sensing.collisionDistance ?? 0.25,
            contactConfirmationDistance: sensing.contactConfirmationDistance ?? 0.6,
            includeDiagonalRays: sensing.includeDiagonalRays ?? true,
            diagonalRaySpreadDeg: sensing.diagonalRaySpreadDeg ?? 35,
            verticalRaySpreadDeg: sensing.verticalRaySpreadDeg ?? 24,
        }
    }

    /**
     * @param {number} yaw
     * @returns {Array<{ name: string, direction: THREE.Vector3, primary: boolean }>}
     */
    buildDirections(yaw) {
        const { includeDiagonalRays, diagonalRaySpreadDeg, verticalRaySpreadDeg } = this.sensingConfig
        const diagonalFactor = Math.tan(THREE.MathUtils.degToRad(diagonalRaySpreadDeg))
        const verticalFactor = Math.tan(THREE.MathUtils.degToRad(verticalRaySpreadDeg))
        const forward = new THREE.Vector3(Math.cos(yaw), Math.sin(yaw), 0)
        const right = new THREE.Vector3(-Math.sin(yaw), Math.cos(yaw), 0)
        const up = new THREE.Vector3(0, 0, -1)
        const down = new THREE.Vector3(0, 0, 1)
        const directions = [
            { name: 'front', direction: forward.clone(), primary: true },
            { name: 'back', direction: forward.clone().multiplyScalar(-1), primary: true },
            { name: 'left', direction: right.clone().multiplyScalar(-1), primary: true },
            { name: 'right', direction: right.clone(), primary: true },
            { name: 'up', direction: up.clone(), primary: true },
            { name: 'down', direction: down.clone(), primary: true },
        ]

        if (includeDiagonalRays) {
            const extras = [
                { name: 'frontLeft', direction: forward.clone().add(right.clone().multiplyScalar(-diagonalFactor)) },
                { name: 'frontRight', direction: forward.clone().add(right.clone().multiplyScalar(diagonalFactor)) },
                { name: 'backLeft', direction: forward.clone().multiplyScalar(-1).add(right.clone().multiplyScalar(-diagonalFactor)) },
                { name: 'backRight', direction: forward.clone().multiplyScalar(-1).add(right.clone().multiplyScalar(diagonalFactor)) },
                { name: 'frontUp', direction: forward.clone().add(up.clone().multiplyScalar(verticalFactor)) },
                { name: 'frontDown', direction: forward.clone().add(down.clone().multiplyScalar(verticalFactor)) },
                { name: 'leftUp', direction: right.clone().multiplyScalar(-1).add(up.clone().multiplyScalar(verticalFactor)) },
                { name: 'rightUp', direction: right.clone().add(up.clone().multiplyScalar(verticalFactor)) },
                { name: 'leftDown', direction: right.clone().multiplyScalar(-1).add(down.clone().multiplyScalar(verticalFactor)) },
                { name: 'rightDown', direction: right.clone().add(down.clone().multiplyScalar(verticalFactor)) },
            ]

            for (const sample of extras) {
                directions.push({
                    name: sample.name,
                    direction: sample.direction.normalize(),
                    primary: false,
                })
            }
        }

        return directions.map(({ name, direction, primary }) => ({
            name,
            direction: direction.clone().normalize(),
            primary,
        }))
    }

    /**
     * @param {{
     *   stepId: number,
     *   world: any,
     *   droneBody: any,
     *   goal?: Vec3 | THREE.Vector3 | null,
     *   extraStaticObstacles?: ObstacleTrack[],
     *   extraDynamicObstacles?: ObstacleTrack[],
     *   dt: number,
     *   timestamp: number,
     * }} params
     * @returns {NavigationObservation}
     */
    sample({ stepId, world, droneBody, goal, extraStaticObstacles = [], extraDynamicObstacles = [], dt, timestamp }) {
        if (this.lastObservation && this.lastStepId === stepId) {
            return this.lastObservation
        }

        const pos = droneBody.translation()
        const rot = droneBody.rotation()
        const linvel = droneBody.linvel()
        const angvel = droneBody.angvel()

        const position = new THREE.Vector3(pos.x, pos.y, pos.z)
        const quaternion = new THREE.Quaternion(rot.x, rot.y, rot.z, rot.w)
        const euler = new THREE.Euler().setFromQuaternion(quaternion, 'ZYX')
        const directions = this.buildDirections(euler.z)

        const { maxRayDistance, pointCloudRange, collisionDistance, contactConfirmationDistance } = this.sensingConfig
        /** @type {Record<string, number>} */
        const rayDistances = {}
        /** @type {ObstacleTrack[]} */
        const staticObstacles = []
        /** @type {number[]} */
        const localPointCloud = []
        let minDistance = maxRayDistance
        let broadPhysicalContact = false
        const nonCollisionRays = new Set(['down', 'frontDown', 'leftDown', 'rightDown'])

        for (const { name, direction, primary } of directions) {
            const ray = new this.RAPIER.Ray(
                { x: position.x, y: position.y, z: position.z },
                { x: direction.x, y: direction.y, z: direction.z },
            )
            const hit = world.castRay(
                ray,
                maxRayDistance,
                true,
                this.RAPIER.QueryFilterFlags.EXCLUDE_SENSORS,
                null,
                null,
                droneBody,
            )

            const distance = hit ? hit.timeOfImpact : maxRayDistance
            rayDistances[name] = distance
            if (!nonCollisionRays.has(name)) {
                minDistance = Math.min(minDistance, distance)
            }

            if (!hit) {
                continue
            }

            const hitPoint = position.clone().addScaledVector(direction, distance)
            if (distance <= pointCloudRange) {
                localPointCloud.push(hitPoint.x, hitPoint.y, hitPoint.z)
            }

            if (!nonCollisionRays.has(name)) {
                staticObstacles.push({
                    id: `ray-${name}`,
                    type: 'unknown',
                    position: [hitPoint.x, hitPoint.y, hitPoint.z],
                    velocity: [0, 0, 0],
                    size: 0.5,
                    dynamic: false,
                    confidence: primary ? 0.35 : 0.22,
                })
            }
        }

        for (const obstacle of extraStaticObstacles) {
            staticObstacles.push(obstacle)
        }

        if (typeof world.contactPairsWith === 'function') {
            world.contactPairsWith(droneBody.collider(0), () => {
                broadPhysicalContact = true
            })
        } else if (typeof world.intersectionPairsWith === 'function') {
            world.intersectionPairsWith(droneBody.collider(0), (otherCollider) => {
                const intersecting = world.intersectionPair(droneBody.collider(0), otherCollider)
                if (intersecting) {
                    broadPhysicalContact = true
                }
            })
        }

        const hasPhysicalContact =
            broadPhysicalContact &&
            minDistance < Math.max(contactConfirmationDistance, collisionDistance)

        /** @type {Vec3} */
        const goalVec = goal
            ? Array.isArray(goal)
                ? [goal[0], goal[1], goal[2]]
                : [goal.x, goal.y, goal.z]
            : [position.x, position.y, position.z]

        const observation = {
            ego: {
                position: [position.x, position.y, position.z],
                orientation: [euler.x, euler.y, euler.z],
                linearVelocity: [linvel.x, linvel.y, linvel.z],
                angularVelocity: [angvel.x, angvel.y, angvel.z],
            },
            goal: goalVec,
            globalPath: [],
            staticObstacles,
            dynamicObstacles: [...extraDynamicObstacles],
            localPointCloud: localPointCloud.length > 0 ? new Float32Array(localPointCloud) : new Float32Array(0),
            dt,
            timestamp,
            rayDistances,
            collision: {
                isColliding: hasPhysicalContact || minDistance < collisionDistance,
                minDistance,
                hasPhysicalContact,
            },
        }

        this.lastStepId = stepId
        this.lastObservation = observation
        return observation
    }
}
