// ============================================================
// 仿真主循环：串联 场景 → 规划 → 控制 → 渲染
// ============================================================

import { SceneManager } from '../scene/SceneManager';
import { PathPlanner } from '../planning/PathPlanner';
import { FlightController } from '../flight/FlightController';
import type { SimConfig, Vec3, DroneState } from '../types';

export class SimulationLoop {
  private sceneManager: SceneManager;
  private pathPlanner: PathPlanner;
  private flightController: FlightController;
  private config: SimConfig;

  private running = false;
  private animationId = 0;
  private lastTime = 0;
  private globalPath: Vec3[] = [];

  // UI 回调
  private onStateUpdate?: (state: DroneState) => void;

  constructor(
    sceneManager: SceneManager,
    config: SimConfig
  ) {
    this.sceneManager = sceneManager;
    this.config = config;

    this.pathPlanner = new PathPlanner(config);
    this.flightController = new FlightController(config.startPosition, config);
  }

  /** 初始化：加载场景数据 + 创建无人机模型 */
  public init(): void {
    // 将场景包围盒数据交给人 2
    const sceneData = this.sceneManager.getSceneData();
    this.pathPlanner.loadScene(sceneData);

    // 创建无人机 3D 模型
    this.sceneManager.createDroneModel();
    const startState = this.flightController.getState();
    this.sceneManager.updateDrone(startState);

    // 全局路径规划
    this.globalPath = this.pathPlanner.planGlobalPath(
      this.config.startPosition,
      this.config.goalPosition
    );
    console.log('[Sim] 全局路径点数:', this.globalPath.length);

    // 渲染初始帧
    this.sceneManager.render();
  }

  /** 设置状态更新回调（用于 UI 面板） */
  public setOnStateUpdate(cb: (state: DroneState) => void): void {
    this.onStateUpdate = cb;
  }

  /** 开始仿真 */
  public start(): void {
    if (this.running) return;
    this.running = true;
    this.lastTime = performance.now();
    this.tick();
  }

  /** 暂停 */
  public pause(): void {
    this.running = false;
    if (this.animationId) {
      cancelAnimationFrame(this.animationId);
    }
  }

  /** 重置 */
  public reset(): void {
    this.pause();
    this.flightController.reset(this.config.startPosition);
    this.sceneManager.reset();

    const startState = this.flightController.getState();
    this.sceneManager.updateDrone(startState);

    this.globalPath = this.pathPlanner.planGlobalPath(
      this.config.startPosition,
      this.config.goalPosition
    );

    this.sceneManager.render();
  }

  /** 主循环 */
  private tick = (): void => {
    if (!this.running) return;

    const now = performance.now();
    let dt = (now - this.lastTime) / 1000;
    this.lastTime = now;

    // 限制最大步长，防止切标签页后跳帧
    if (dt > 0.1) dt = 0.1;

    // ---- 算法层 (人 2) ----
    const droneState = this.flightController.getState();
    const control = this.pathPlanner.computeControl(droneState, this.globalPath);

    // ---- 飞控层 (人 3) ----
    this.flightController.setCommand(control);
    const newState = this.flightController.step(dt);

    // ---- 渲染层 (人 1) ----
    this.sceneManager.updateDrone(newState);
    this.sceneManager.render();

    // UI 回调
    this.onStateUpdate?.(newState);

    // 检查是否到达目标
    const dist = this.distance(newState.position, this.config.goalPosition);
    if (dist < 2.0) {
      console.log('[Sim] 已到达目标点!');
      this.pause();
      return;
    }

    this.animationId = requestAnimationFrame(this.tick);
  };

  private distance(a: Vec3, b: Vec3): number {
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    const dz = a[2] - b[2];
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
}
