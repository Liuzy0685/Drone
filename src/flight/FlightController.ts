// ============================================================
// 人 3 模块：飞控系统
// 负责：无人机动力学模型、PID/MPC 控制器、UI 面板
// ============================================================
// TODO: 实现以下方法
// ============================================================

import type { DroneState, ControlCommand, SimConfig, Vec3, Quaternion } from '../types';

/** 无人机内部物理状态 */
interface PhysicsState {
  position: Vec3;
  velocity: Vec3;
  acceleration: Vec3;
  orientation: Quaternion;  // [x, y, z, w]
  angularVelocity: Vec3;
}

export class FlightController {
  private state: PhysicsState;
  private config: SimConfig;
  private targetCommand: ControlCommand | null = null;

  constructor(startPos: Vec3, config: SimConfig) {
    this.config = config;
    this.state = {
      position: [...startPos] as Vec3,
      velocity: [0, 0, 0],
      acceleration: [0, 0, 0],
      orientation: [0, 0, 0, 1],
      angularVelocity: [0, 0, 0],
    };
  }

  /** 接收来自人 2 的控制指令 */
  public setCommand(cmd: ControlCommand): void {
    this.targetCommand = cmd;
  }

  /**
   * 仿真步进：根据控制指令更新无人机物理状态
   * TODO: 人3 - 实现完整的四旋翼动力学 + PID 控制器
   *
   * 当前占位实现：简单的一阶惯性模型
   */
  public step(dt: number): DroneState {
    if (!this.targetCommand) {
      return this.getState();
    }

    const { targetVelocity } = this.targetCommand;
    const gain = 3.0; // 控制增益

    // TODO: 替换为完整四旋翼动力学
    // 1. 根据期望速度计算期望推力
    // 2. 计算期望姿态（倾转角）
    // 3. PID 姿态控制 → 力矩 → 角速度 → 四元数更新
    // 4. 推力 → 加速度 → 速度 → 位置

    // 占位：一阶惯性跟踪
    this.state.acceleration = [
      (targetVelocity[0] - this.state.velocity[0]) * gain,
      (targetVelocity[1] - this.state.velocity[1]) * gain,
      (targetVelocity[2] - this.state.velocity[2]) * gain,
    ];

    // 数值积分
    for (let i = 0; i < 3; i++) {
      this.state.velocity[i] += this.state.acceleration[i] * dt;
      this.state.position[i] += this.state.velocity[i] * dt;
    }

    return this.getState();
  }

  /** 获取当前状态（符合共享接口格式） */
  public getState(): DroneState {
    return {
      position: [...this.state.position] as Vec3,
      orientation: [...this.state.orientation] as Quaternion,
      velocity: [...this.state.velocity] as Vec3,
      angularVelocity: [...this.state.angularVelocity] as Vec3,
      timestamp: performance.now(),
    };
  }

  /** 重置 */
  public reset(startPos: Vec3): void {
    this.state = {
      position: [...startPos] as Vec3,
      velocity: [0, 0, 0],
      acceleration: [0, 0, 0],
      orientation: [0, 0, 0, 1],
      angularVelocity: [0, 0, 0],
    };
    this.targetCommand = null;
  }
}
