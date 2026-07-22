// ============================================================
// 人 2 模块：避障路径规划
// 负责：A* / RRT* / DWA、碰撞检测、传感器模拟
// ============================================================
// TODO: 实现以下方法
// ============================================================

import type { SceneData, ControlCommand, DroneState, SimConfig, Vec3 } from '../types';

export class PathPlanner {
  private sceneData: SceneData | null = null;
  private config: SimConfig;

  constructor(config: SimConfig) {
    this.config = config;
  }

  /** 加载场景障碍数据 */
  public loadScene(sceneData: SceneData): void {
    this.sceneData = sceneData;
    console.log('[PathPlanner] 场景数据已加载, 建筑物数量:', sceneData.buildings.length);
  }

  /**
   * 全局路径规划：从起点到终点的全局路径
   * TODO: 人2 - 实现 A* 或 RRT* 算法
   *
   * @param start 起点位置
   * @param goal  终点位置
   * @returns 路径点数组
   */
  public planGlobalPath(start: Vec3, goal: Vec3): Vec3[] {
    // TODO: 实现 A* / RRT*
    // 先用直线路径作为占位
    console.warn('[PathPlanner] planGlobalPath 未实现，返回直线路径');
    return [start, goal];
  }

  /**
   * 局部实时避障：根据当前状态和环境计算下一时刻的控制指令
   * TODO: 人2 - 实现 DWA 动态窗口法
   *
   * @param state 无人机当前状态
   * @param globalPath 全局参考路径
   * @returns 控制指令
   */
  public computeControl(
    state: DroneState,
    globalPath: Vec3[]
  ): ControlCommand {
    // TODO: 实现 DWA
    // 1. 模拟传感器扫描（基于当前位姿 + 场景包围盒做射线检测）
    // 2. 在速度空间采样
    // 3. 评估每个采样轨迹的安全性、目标趋近度、速度代价
    // 4. 返回最优的速度指令

    // 占位：朝目标点以恒定速度飞行
    const goal = globalPath[globalPath.length - 1];
    const dx = goal[0] - state.position[0];
    const dy = goal[1] - state.position[1];
    const dz = goal[2] - state.position[2];
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const speed = Math.min(this.config.maxSpeed, dist * 0.5);

    return {
      waypoints: globalPath,
      targetVelocity: [
        (dx / dist) * speed,
        (dy / dist) * speed,
        (dz / dist) * speed,
      ],
      targetPosition: goal,
      obstacleDistances: [],
    };
  }

  /**
   * 碰撞检测：判断给定位置是否与建筑物碰撞
   * TODO: 人2 - 实现 OBB 碰撞检测 / BVH 加速
   */
  public checkCollision(position: Vec3): boolean {
    // TODO: 实现 OBB 碰撞检测
    return false;
  }
}
