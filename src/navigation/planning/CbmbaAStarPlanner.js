import { THREE } from '../../three.js'

function keyOf(cell) {
    return `${cell.x}|${cell.y}|${cell.z}`
}

function vec3ToCell(vec, origin, resolution) {
    return {
        x: Math.round((vec[0] - origin[0]) / resolution),
        y: Math.round((vec[1] - origin[1]) / resolution),
        z: Math.round((vec[2] - origin[2]) / resolution),
    }
}

function cellToVec3(cell, origin, resolution) {
    return [
        origin[0] + cell.x * resolution,
        origin[1] + cell.y * resolution,
        origin[2] + cell.z * resolution,
    ]
}

function keyEquals(a, b) {
    return a.x === b.x && a.y === b.y && a.z === b.z
}

function heuristic(a, b, verticalWeight = 1.0) {
    const dx = a.x - b.x
    const dy = a.y - b.y
    const dz = (a.z - b.z) * verticalWeight
    return Math.sqrt(dx * dx + dy * dy + dz * dz)
}

function vec3FromArray(v) {
    return new THREE.Vector3(v[0], v[1], v[2])
}

function normalizeCellDirection(from, to) {
    const dx = Math.sign(to.x - from.x)
    const dy = Math.sign(to.y - from.y)
    const dz = Math.sign(to.z - from.z)
    return `${dx}|${dy}|${dz}`
}

function obstacleHalfExtents(obstacle) {
    if (Array.isArray(obstacle?.footprintHalfExtents) && obstacle.footprintHalfExtents.length >= 3) {
        return {
            x: Math.max(obstacle.footprintHalfExtents[0] ?? 0, 0),
            y: Math.max(obstacle.footprintHalfExtents[1] ?? 0, 0),
            z: Math.max(obstacle.footprintHalfExtents[2] ?? 0, 0),
        }
    }
    const radius = Math.max(obstacle?.size ?? 0, 0)
    return { x: radius, y: radius, z: radius }
}

export class CbmbaAStarPlanner {
    constructor(config = {}) {
        this.config = config
        this.grid = new Set()
        this.occupiedCells = []
        this.lastPath = []
        this.lastPlanTime = -Infinity
        this.lastStart = null
        this.lastGoal = null
        this.lastOrigin = [0, 0, 0]
    }

    get parameters() {
        const defaults = {
            enabled: true,
            resolution: 1.5,
            inflationRadius: 1.5,
            maxSearchNodes: 16000,
            replanDistanceThreshold: 2.0,
            replanTimeThreshold: 0.5,
            mapPadding: 8.0,
            weightedHeuristic: 1.15,
            verticalMoveCost: 1.4,
            verticalHeuristicWeight: 1.35,
            turnPenalty: 0.2,
            wallPenaltyRadius: 2,
            wallPenaltyWeight: 0.3,
            goalLayerCount: 2,
            maxGoalVerticalOffset: 4.0,
            lineOfSightSamples: 20,
            lineOfSightInflation: 0.8,
            freeCellSearchRadius: 3,
            adaptiveLongStepCells: 2,
            sectorBiasWeight: 0.4,
            buildingMinHeight: 1.2,
            buildingDownwardSealDepth: 6.0,
        }
        return {
            ...defaults,
            ...(this.config.navigation?.globalPlanner ?? {}),
        }
    }

    plan(obstacles, start, goal) {
        const params = this.parameters
        if (!params.enabled) {
            this.lastPath = [start, goal]
            return this.lastPath
        }

        const origin = this.computeOrigin(start, goal, obstacles)
        this.lastOrigin = origin
        const { occupied, occupiedCells } = this.buildOccupancyGrid(
            obstacles,
            origin,
            params.resolution,
            params.inflationRadius,
            params,
        )
        this.grid = occupied
        this.occupiedCells = occupiedCells

        const rawStartCell = vec3ToCell(start, origin, params.resolution)
        const rawGoalCell = vec3ToCell(goal, origin, params.resolution)
        const startCell = this.ensureFreeCell(rawStartCell, params.freeCellSearchRadius)
        const goalCells = this.buildGoalCells(rawGoalCell, params)
        const pathCells = this.runAStar(startCell, goalCells, params.maxSearchNodes, params)

        if (!pathCells || pathCells.length === 0) {
            this.lastPath = [start, goal]
            this.lastPlanTime = performance.now() / 1000
            this.lastStart = start
            this.lastGoal = goal
            return this.lastPath
        }

        const sampledPath = pathCells.map(cell => cellToVec3(cell, origin, params.resolution))
        this.lastPath = this.smoothPath([start, ...sampledPath, goal], params)
        this.lastPlanTime = performance.now() / 1000
        this.lastStart = start
        this.lastGoal = goal
        return this.lastPath
    }

    maybeReplan(start, goal, obstacles) {
        const now = performance.now() / 1000
        const params = this.parameters
        const shouldReplan =
            this.lastPath.length === 0
            || !this.lastStart
            || !this.lastGoal
            || this.distanceBetween(start, this.lastStart) > params.replanDistanceThreshold
            || this.distanceBetween(goal, this.lastGoal) > params.replanDistanceThreshold
            || (now - this.lastPlanTime) > params.replanTimeThreshold
            || this.isPathBlocked(obstacles, this.lastPath, params.inflationRadius)

        if (!shouldReplan) {
            return this.lastPath
        }
        return this.plan(obstacles, start, goal)
    }

    buildOccupancyGrid(obstacles, origin, resolution, inflationRadius, params = this.parameters) {
        const occupied = new Set()
        const occupiedCells = []
        for (const obstacle of obstacles ?? []) {
            const extents = obstacleHalfExtents(obstacle)
            const isBuildingVolume =
                obstacle?.type === 'building' &&
                (extents.z * 2) >= params.buildingMinHeight

            const minX = obstacle.position[0] - extents.x - inflationRadius
            const maxX = obstacle.position[0] + extents.x + inflationRadius
            const minY = obstacle.position[1] - extents.y - inflationRadius
            const maxY = obstacle.position[1] + extents.y + inflationRadius
            const minZ = obstacle.position[2] - extents.z - inflationRadius
            const maxZ =
                obstacle.position[2] +
                extents.z +
                inflationRadius +
                (isBuildingVolume ? params.buildingDownwardSealDepth : 0)

            const x0 = Math.floor((Math.min(minX, maxX) - origin[0]) / resolution)
            const x1 = Math.ceil((Math.max(minX, maxX) - origin[0]) / resolution)
            const y0 = Math.floor((Math.min(minY, maxY) - origin[1]) / resolution)
            const y1 = Math.ceil((Math.max(minY, maxY) - origin[1]) / resolution)
            const z0 = Math.floor((Math.min(minZ, maxZ) - origin[2]) / resolution)
            const z1 = Math.ceil((Math.max(minZ, maxZ) - origin[2]) / resolution)

            for (let x = x0; x <= x1; x += 1) {
                for (let y = y0; y <= y1; y += 1) {
                    for (let z = z0; z <= z1; z += 1) {
                        const cell = { x, y, z }
                        const key = keyOf(cell)
                        if (!occupied.has(key)) {
                            occupied.add(key)
                            occupiedCells.push(cell)
                        }
                    }
                }
            }
        }
        return { occupied, occupiedCells }
    }

    buildGoalCells(goalCell, params) {
        const cells = []
        const stepCount = Math.max(1, params.goalLayerCount)
        const verticalCellOffset = Math.max(1, Math.round(params.maxGoalVerticalOffset / params.resolution))
        for (let dz = -stepCount; dz <= stepCount; dz += 1) {
            const offset = Math.round((dz / stepCount) * verticalCellOffset)
            const candidate = this.ensureFreeCell({
                x: goalCell.x,
                y: goalCell.y,
                z: goalCell.z + offset,
            }, params.freeCellSearchRadius)
            if (!cells.some(existing => keyEquals(existing, candidate))) {
                cells.push(candidate)
            }
        }
        return cells
    }

    ensureFreeCell(cell, maxRadius = 2) {
        if (!this.grid.has(keyOf(cell))) {
            return cell
        }

        for (let radius = 1; radius <= maxRadius; radius += 1) {
            for (let dx = -radius; dx <= radius; dx += 1) {
                for (let dy = -radius; dy <= radius; dy += 1) {
                    for (let dz = -radius; dz <= radius; dz += 1) {
                        const candidate = {
                            x: cell.x + dx,
                            y: cell.y + dy,
                            z: cell.z + dz,
                        }
                        if (!this.grid.has(keyOf(candidate))) {
                            return candidate
                        }
                    }
                }
            }
        }

        return cell
    }

    orderedNeighbors(cell, goalCell, currentDir, params) {
        const candidates = new Map()
        const longStep = Math.max(1, params.adaptiveLongStepCells ?? 1)
        const stepSet = longStep > 1 ? [1, longStep] : [1]
        const goalDir = {
            x: Math.sign(goalCell.x - cell.x),
            y: Math.sign(goalCell.y - cell.y),
            z: Math.sign(goalCell.z - cell.z),
        }

        for (const step of stepSet) {
            for (let dx = -1; dx <= 1; dx += 1) {
                for (let dy = -1; dy <= 1; dy += 1) {
                    for (let dz = -1; dz <= 1; dz += 1) {
                        if (dx === 0 && dy === 0 && dz === 0) continue
                        const neighbor = {
                            x: cell.x + dx * step,
                            y: cell.y + dy * step,
                            z: cell.z + dz * step,
                        }
                        candidates.set(keyOf(neighbor), neighbor)
                    }
                }
            }
        }

        const result = [...candidates.values()]
        result.sort((a, b) => {
            const dirA = normalizeCellDirection(cell, a).split('|').map(Number)
            const dirB = normalizeCellDirection(cell, b).split('|').map(Number)
            const biasA =
                (dirA[0] === goalDir.x ? 1 : 0) +
                (dirA[1] === goalDir.y ? 1 : 0) +
                (dirA[2] === goalDir.z ? 1 : 0) +
                (currentDir && normalizeCellDirection(cell, a) === currentDir ? params.sectorBiasWeight : 0)
            const biasB =
                (dirB[0] === goalDir.x ? 1 : 0) +
                (dirB[1] === goalDir.y ? 1 : 0) +
                (dirB[2] === goalDir.z ? 1 : 0) +
                (currentDir && normalizeCellDirection(cell, b) === currentDir ? params.sectorBiasWeight : 0)
            const scoreA = heuristic(a, goalCell) - biasA
            const scoreB = heuristic(b, goalCell) - biasB
            return scoreA - scoreB
        })
        return result
    }

    runAStar(startCell, goalCells, maxSearchNodes, params) {
        const open = new Map()
        const cameFrom = new Map()
        const gScore = new Map()
        const fScore = new Map()
        const closed = new Set()
        const startKey = keyOf(startCell)
        const goalKeys = new Set(goalCells.map(keyOf))
        const goalReference = goalCells[0]

        open.set(startKey, startCell)
        gScore.set(startKey, 0)
        fScore.set(startKey, heuristic(startCell, goalReference, params.verticalHeuristicWeight))

        let iterations = 0
        while (open.size > 0 && iterations < maxSearchNodes) {
            iterations += 1
            let currentKey = null
            let currentCell = null
            let bestScore = Infinity
            for (const [key, cell] of open.entries()) {
                const score = fScore.get(key) ?? Infinity
                if (score < bestScore) {
                    bestScore = score
                    currentKey = key
                    currentCell = cell
                }
            }
            if (!currentCell || !currentKey) break
            if (goalKeys.has(currentKey)) {
                return this.reconstructPath(cameFrom, currentKey)
            }
            open.delete(currentKey)
            closed.add(currentKey)

            const nearestGoal = goalCells.reduce((best, candidate) =>
                heuristic(currentCell, candidate, params.verticalHeuristicWeight) <
                heuristic(currentCell, best, params.verticalHeuristicWeight)
                    ? candidate
                    : best,
            goalReference)

            const currentDir = (() => {
                const parentKey = cameFrom.get(currentKey)
                if (!parentKey) return null
                return normalizeCellDirection(this.parseKey(parentKey), currentCell)
            })()

            for (const neighbor of this.orderedNeighbors(currentCell, nearestGoal, currentDir, params)) {
                const neighborKey = keyOf(neighbor)
                if (closed.has(neighborKey) || this.grid.has(neighborKey)) continue

                const currentCost = gScore.get(currentKey) ?? Infinity
                const traversalCost = this.computeTraversalCost(
                    currentCell,
                    neighbor,
                    currentKey,
                    cameFrom,
                    params,
                )
                const tentativeG = currentCost + traversalCost
                if (tentativeG >= (gScore.get(neighborKey) ?? Infinity)) continue

                cameFrom.set(neighborKey, currentKey)
                gScore.set(neighborKey, tentativeG)
                const h = heuristic(neighbor, nearestGoal, params.verticalHeuristicWeight)
                fScore.set(neighborKey, tentativeG + h * params.weightedHeuristic)
                open.set(neighborKey, neighbor)
            }
        }

        return []
    }

    computeTraversalCost(currentCell, neighborCell, currentKey, cameFrom, params) {
        const dx = neighborCell.x - currentCell.x
        const dy = neighborCell.y - currentCell.y
        const dz = neighborCell.z - currentCell.z
        const base = Math.sqrt(
            dx * dx +
            dy * dy +
            (dz * params.verticalMoveCost) * (dz * params.verticalMoveCost)
        )

        let turnPenalty = 0
        const parentKey = cameFrom.get(currentKey)
        if (parentKey) {
            const parentCell = this.parseKey(parentKey)
            const previousDir = normalizeCellDirection(parentCell, currentCell)
            const nextDir = normalizeCellDirection(currentCell, neighborCell)
            if (previousDir !== nextDir) {
                turnPenalty = params.turnPenalty
            }
        }

        const wallPenalty = this.localObstaclePenalty(neighborCell, params.wallPenaltyRadius) * params.wallPenaltyWeight
        return base + turnPenalty + wallPenalty
    }

    localObstaclePenalty(cell, radius) {
        if (radius <= 0) {
            return 0
        }

        let penalty = 0
        for (let dx = -radius; dx <= radius; dx += 1) {
            for (let dy = -radius; dy <= radius; dy += 1) {
                for (let dz = -radius; dz <= radius; dz += 1) {
                    const sample = { x: cell.x + dx, y: cell.y + dy, z: cell.z + dz }
                    if (!this.grid.has(keyOf(sample))) continue
                    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz) || 1
                    penalty += 1 / distance
                }
            }
        }
        return penalty
    }

    reconstructPath(cameFrom, currentKey) {
        const path = [this.parseKey(currentKey)]
        while (cameFrom.has(currentKey)) {
            currentKey = cameFrom.get(currentKey)
            path.push(this.parseKey(currentKey))
        }
        path.reverse()
        return path
    }

    parseKey(key) {
        const [x, y, z] = key.split('|').map(Number)
        return { x, y, z }
    }

    smoothPath(points, params) {
        if (!points || points.length < 3) return points

        const compact = [points[0]]
        let anchorIndex = 0
        while (anchorIndex < points.length - 1) {
            let furthestIndex = anchorIndex + 1
            for (let i = points.length - 1; i > anchorIndex + 1; i -= 1) {
                if (this.hasLineOfSight(points[anchorIndex], points[i], params)) {
                    furthestIndex = i
                    break
                }
            }
            compact.push(points[furthestIndex])
            anchorIndex = furthestIndex
        }

        const smooth = [compact[0]]
        for (let i = 1; i < compact.length - 1; i += 1) {
            const prev = vec3FromArray(compact[i - 1])
            const curr = vec3FromArray(compact[i])
            const next = vec3FromArray(compact[i + 1])
            const v1 = curr.clone().sub(prev).normalize()
            const v2 = next.clone().sub(curr).normalize()
            if (v1.angleTo(v2) > 0.12) {
                smooth.push(compact[i])
            }
        }
        smooth.push(compact[compact.length - 1])
        return smooth
    }

    hasLineOfSight(start, end, params) {
        const a = vec3FromArray(start)
        const b = vec3FromArray(end)
        const samples = Math.max(4, params.lineOfSightSamples)
        for (let i = 1; i < samples; i += 1) {
            const t = i / samples
            const point = a.clone().lerp(b, t)
            const cell = vec3ToCell(point.toArray(), this.lastOrigin, params.resolution)
            if (this.grid.has(keyOf(cell))) {
                return false
            }

            if (params.lineOfSightInflation > 0) {
                const radius = Math.ceil(params.lineOfSightInflation / params.resolution)
                for (let dx = -radius; dx <= radius; dx += 1) {
                    for (let dy = -radius; dy <= radius; dy += 1) {
                        const expanded = { x: cell.x + dx, y: cell.y + dy, z: cell.z }
                        if (this.grid.has(keyOf(expanded))) {
                            return false
                        }
                    }
                }
            }
        }
        return true
    }

    isPathBlocked(obstacles, path, inflationRadius) {
        if (!path || path.length < 2) return false
        for (const obstacle of obstacles ?? []) {
            const center = vec3FromArray(obstacle.position)
            for (let i = 0; i < path.length - 1; i += 1) {
                const a = vec3FromArray(path[i])
                const b = vec3FromArray(path[i + 1])
                const projection = this.projectToSegment(center, a, b)
                const clearance = projection.distanceTo(center) - (obstacle.size + inflationRadius)
                if (clearance < 0) return true
            }
        }
        return false
    }

    projectToSegment(point, start, end) {
        const segment = end.clone().sub(start)
        const segmentLengthSq = segment.lengthSq()
        if (segmentLengthSq < 1e-12) return start.clone()
        const t = THREE.MathUtils.clamp(point.clone().sub(start).dot(segment) / segmentLengthSq, 0, 1)
        return start.clone().addScaledVector(segment, t)
    }

    computeOrigin(start, goal, obstacles) {
        const params = this.parameters
        const xs = [start[0], goal[0]]
        const ys = [start[1], goal[1]]
        const zs = [start[2], goal[2]]
        for (const obstacle of obstacles ?? []) {
            xs.push(obstacle.position[0])
            ys.push(obstacle.position[1])
            zs.push(obstacle.position[2])
        }
        return [
            Math.min(...xs) - params.mapPadding,
            Math.min(...ys) - params.mapPadding,
            Math.min(...zs) - params.mapPadding,
        ]
    }

    distanceBetween(a, b) {
        const dx = a[0] - b[0]
        const dy = a[1] - b[1]
        const dz = a[2] - b[2]
        return Math.sqrt(dx * dx + dy * dy + dz * dz)
    }
}
