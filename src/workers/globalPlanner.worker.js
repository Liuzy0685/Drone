import { CbmbaAStarPlanner } from '../navigation/planning/CbmbaAStarPlanner.js'

let planner = null

self.addEventListener('message', (event) => {
    const data = event.data ?? {}

    if (data.type === 'init') {
        planner = new CbmbaAStarPlanner(data.config ?? {})
        postMessage({ type: 'ready' })
        return
    }

    if (data.type === 'plan') {
        if (!planner) {
            planner = new CbmbaAStarPlanner(data.config ?? {})
        }
        const req = data.request ?? {}
        const path = planner.maybeReplan(req.start, req.goal, req.staticObstacles ?? [])
        postMessage({
            type: 'planResult',
            path,
            timestamp: req.timestamp ?? 0,
            generation: req.generation ?? 0,
        })
        return
    }

    if (data.type === 'reset') {
        planner = null
    }
})
