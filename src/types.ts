// ============================================================
// 共享类型定义 —— 三人之间的接口契约
// 修改此文件前必须 PR 并经另外两人 Review
// ============================================================

/** 三维向量 */
export type Vec3 = [number, number, number];

/** 四元数 */
export type Quaternion = [number, number, number, number];

// ---------- 人 1 输出 → 人 2 消费 ----------

/** 单个建筑物的有向包围盒 (OBB) */
export interface BuildingOBB {
  id: string;
  center: Vec3;
  halfExtents: Vec3;   // 半边长
  rotation: Vec3;      // 欧拉角 [x, y, z]
}

/** 场景障碍物数据 */
export interface SceneData {
  buildings: BuildingOBB[];
  bounds: {
    min: Vec3;
    max: Vec3;
  };
}

// ---------- 人 2 输出 → 人 3 消费 ----------

/** 实时控制指令 */
export interface ControlCommand {
  waypoints: Vec3[];          // 全局参考路径点
  targetVelocity: Vec3;       // 当前时刻期望速度 (DWA 输出)
  targetPosition: Vec3;       // 当前时刻期望位置
  obstacleDistances: number[];// 传感器读数 (各方障碍距离)
}

// ---------- 人 3 输出 → 人 1 + 人 2 消费 ----------

/** 无人机实时状态 */
export interface DroneState {
  position: Vec3;
  orientation: Quaternion;
  velocity: Vec3;
  angularVelocity: Vec3;
  timestamp: number;
}

// ---------- 仿真配置 ----------

export interface SimConfig {
  startPosition: Vec3;
  goalPosition: Vec3;
  maxSpeed: number;
  sensorRange: number;
  timeStep: number;      // 仿真步长 (秒)
}
