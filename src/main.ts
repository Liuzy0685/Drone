// ============================================================
// 入口文件
// ============================================================

import { SceneManager } from './scene/SceneManager';
import { SimulationLoop } from './sim/SimulationLoop';
import type { SimConfig } from './types';

// ---- 仿真参数 ----
const config: SimConfig = {
  startPosition: [-60, 2, 60],
  goalPosition: [60, 2, -50],
  maxSpeed: 8,
  sensorRange: 15,
  timeStep: 1 / 60,
};

// ---- 初始化 ----
const container = document.getElementById('app')!;
const sceneManager = new SceneManager(container);
const sim = new SimulationLoop(sceneManager, config);
sim.init();

// ---- HUD 更新 ----
const hud = document.getElementById('hud')!;
sim.setOnStateUpdate((state) => {
  hud.textContent =
    `位置: (${state.position[0].toFixed(1)}, ${state.position[1].toFixed(1)}, ${state.position[2].toFixed(1)}) ` +
    `| 速度: ${Math.sqrt(state.velocity[0]**2 + state.velocity[1]**2 + state.velocity[2]**2).toFixed(1)} m/s`;
});

// ---- 按钮事件 ----
document.getElementById('btn-start')!.addEventListener('click', () => {
  sim.start();
});

document.getElementById('btn-reset')!.addEventListener('click', () => {
  sim.reset();
});
