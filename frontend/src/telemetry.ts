export type Vec3 = { x: number; y: number; z: number };
export type Drone = {
  id: string;
  position: Vec3;
  velocity: Vec3;
  yaw: number; // radians, NED heading
  goal: Vec3;
  status: 'navigating' | 'yielding' | 'arrived' | 'offline';
  trail: Vec3[];
  camera_url?: string;
};
export type Obstacle = { id: string; x: number; y: number; width: number; depth: number };
export type TelemetryFrame = {
  type: 'telemetry';
  timestamp: number; // Unix milliseconds
  frame: 'world_ned';
  drones: Drone[];
  obstacles: Obstacle[];
};
export const COLORS = ['#348779', '#597ee2', '#b57bd8', '#df9c42', '#df7383', '#55a3bb'];
const finite = (n: unknown): n is number => typeof n === 'number' && Number.isFinite(n);
const object = (v: unknown): v is Record<string, unknown> => !!v && typeof v === 'object' && !Array.isArray(v);
const vector = (v: unknown): v is Vec3 => object(v) && finite(v.x) && finite(v.y) && finite(v.z);
export function cameraUrl(value: unknown): string | undefined {
  if (value === undefined || value === '') return undefined;
  if (typeof value !== 'string') throw new Error('camera_url 必须是 HTTP(S) 地址');
  const url = new URL(value);
  if (!['http:', 'https:'].includes(url.protocol)) throw new Error('camera_url 仅支持 HTTP(S) 图像流');
  if (location.protocol === 'https:' && url.protocol !== 'https:') throw new Error('HTTPS 页面需要 HTTPS 图像流');
  return url.href;
}
export function parseFrame(raw: string): TelemetryFrame {
  if (raw.length > 2_000_000) throw new Error('遥测帧过大');
  const data: unknown = JSON.parse(raw);
  if (!object(data) || data.type !== 'telemetry' || data.frame !== 'world_ned' || !finite(data.timestamp)) {
    throw new Error('需要 telemetry / world_ned 格式和毫秒时间戳');
  }
  if (data.timestamp < Date.now() - 10_000 || data.timestamp > Date.now() + 5_000) throw new Error('遥测时间戳过期或时钟偏差过大');
  if (!Array.isArray(data.drones) || !data.drones.length || data.drones.length > 32) throw new Error('无人机数量应为 1–32');
  const ids = new Set<string>();
  const drones: Drone[] = data.drones.map((d: unknown) => {
    if (!object(d) || typeof d.id !== 'string' || !d.id.trim() || d.id.length > 64 || ids.has(d.id)) throw new Error('无人机 ID 无效或重复');
    ids.add(d.id);
    if (!vector(d.position) || !vector(d.velocity) || !vector(d.goal) || !finite(d.yaw)) throw new Error(`${d.id} 的位置、速度、目标或朝向无效`);
    if (!['navigating', 'yielding', 'arrived', 'offline'].includes(String(d.status))) throw new Error(`${d.id} 的状态无效`);
    if (d.trail !== undefined && (!Array.isArray(d.trail) || !d.trail.every(vector))) throw new Error(`${d.id} 的航迹无效`);
    return { id: d.id, position: d.position, velocity: d.velocity, goal: d.goal, yaw: d.yaw,
      status: d.status as Drone['status'], trail: (d.trail as Vec3[] | undefined)?.slice(-600) ?? [], camera_url: cameraUrl(d.camera_url) };
  });
  if (!Array.isArray(data.obstacles) || data.obstacles.length > 500) throw new Error('obstacles 必须是最多 500 项的数组');
  const obstacles: Obstacle[] = data.obstacles.map((o: unknown) => {
    if (!object(o) || typeof o.id !== 'string' || !finite(o.x) || !finite(o.y) || !finite(o.width) || !finite(o.depth) || o.width <= 0 || o.depth <= 0) throw new Error('障碍物几何信息无效');
    return { id: o.id, x: o.x, y: o.y, width: o.width, depth: o.depth };
  });
  return { type: 'telemetry', timestamp: data.timestamp, frame: 'world_ned', drones, obstacles };
}
export const DEMO_OBSTACLES: Obstacle[] = [
  { id: 'A', x: 20, y: -27, width: 13, depth: 10 },
  { id: 'B', x: 21, y: 18, width: 14, depth: 10 },
  { id: 'C', x: 1, y: 1, width: 11, depth: 13 },
  { id: 'D', x: -20, y: -24, width: 15, depth: 10 },
  { id: 'E', x: -20, y: 25, width: 10, depth: 11 },
];
const routes = [
  [{ x: -30, y: -43, z: -1 }, { x: -7, y: -36, z: -1 }, { x: -7, y: -19, z: -1 }, { x: -11, y: 1, z: -1 }, { x: -5, y: 19, z: -1 }, { x: 7, y: 36, z: -1 }, { x: 30, y: 43, z: -1 }],
  [{ x: 30, y: 43, z: -1 }, { x: 8, y: 36, z: -1 }, { x: 8, y: 19, z: -1 }, { x: 13, y: 0, z: -1 }, { x: 5, y: -19, z: -1 }, { x: 9, y: -36, z: -1 }, { x: -30, y: -43, z: -1 }],
];
function routePoint(route: Vec3[], t: number): Vec3 {
  const p = Math.max(0, Math.min(0.999999, t)) * (route.length - 1);
  const index = Math.floor(p), k = p - index;
  const a = route[index], b = route[index + 1];
  return { x: a.x + (b.x - a.x) * k, y: a.y + (b.y - a.y) * k, z: a.z };
}
export function demoFrame(seconds: number): TelemetryFrame {
  const cycle = seconds % 100;
  return { type: 'telemetry', timestamp: Date.now(), frame: 'world_ned', obstacles: DEMO_OBSTACLES.map(o => ({ ...o })),
    drones: routes.map((route, i) => {
      const duration = 88 + i * 5, t = Math.min(cycle / duration, 1);
      const position = routePoint(route, t);
      const before = routePoint(route, Math.max(0, t - 0.001));
      const after = routePoint(route, Math.min(1, t + 0.001));
      const dx = after.x - before.x, dy = after.y - before.y, length = Math.hypot(dx, dy);
      const yielding = i === 1 && cycle > 38 && cycle < 44;
      const speed = t >= 1 ? 0 : yielding ? 0.35 : 1.6;
      return { id: `Drone${i + 1}`, position, goal: route[route.length - 1],
        velocity: { x: length ? dx / length * speed : 0, y: length ? dy / length * speed : 0, z: 0 }, yaw: Math.atan2(dy, dx),
        status: t >= 1 ? 'arrived' : yielding ? 'yielding' : 'navigating',
        trail: Array.from({ length: Math.max(2, Math.ceil(t * 100)) }, (_, n) => routePoint(route, t * n / Math.max(1, Math.ceil(t * 100) - 1))) };
    }) };
}
export function distance(a: Vec3, b: Vec3) { return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z); }
export function speed(drone: Drone) { return Math.hypot(drone.velocity.x, drone.velocity.y, drone.velocity.z); }
