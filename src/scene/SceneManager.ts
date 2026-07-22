// ============================================================
// 人 1 模块：3D 场景管理
// 负责：场景搭建、建筑模型加载、包围盒生成、实时渲染
// ============================================================

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import type { BuildingOBB, SceneData, DroneState } from '../types';

export class SceneManager {
  public scene: THREE.Scene;
  public camera: THREE.PerspectiveCamera;
  public renderer: THREE.WebGLRenderer;
  public controls: OrbitControls;

  private buildings: THREE.Group;
  private buildingOBBs: BuildingOBB[] = [];
  private droneModel: THREE.Group | null = null;
  private trajectoryLine: THREE.Line | null = null;
  private trajectoryPoints: THREE.Vector3[] = [];
  private sensorLines: THREE.Line[] = [];

  constructor(container: HTMLElement) {
    // ---- 渲染器 ----
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    this.renderer.shadowMap.enabled = true;
    container.appendChild(this.renderer.domElement);

    // ---- 场景 ----
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x87ceeb);
    this.scene.fog = new THREE.Fog(0x87ceeb, 100, 600);

    // ---- 相机 ----
    this.camera = new THREE.PerspectiveCamera(
      55, container.clientWidth / container.clientHeight, 1, 1000
    );
    this.camera.position.set(80, 60, 100);
    this.camera.lookAt(0, 0, 0);

    // ---- 轨道控制器 ----
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 10, 0);
    this.controls.maxPolarAngle = Math.PI / 2.2;

    // ---- 建筑物容器 ----
    this.buildings = new THREE.Group();
    this.scene.add(this.buildings);

    this.setupLights();
    this.setupGround();
    this.createDemoBuildings();
    this.setupGrid();

    // ---- 窗口缩放 ----
    window.addEventListener('resize', () => {
      this.camera.aspect = container.clientWidth / container.clientHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(container.clientWidth, container.clientHeight);
    });
  }

  // ---- 光照 ----
  private setupLights(): void {
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambient);

    const sun = new THREE.DirectionalLight(0xffffff, 1.2);
    sun.position.set(80, 120, 40);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.near = 1;
    sun.shadow.camera.far = 400;
    sun.shadow.camera.left = -100;
    sun.shadow.camera.right = 100;
    sun.shadow.camera.top = 100;
    sun.shadow.camera.bottom = -100;
    this.scene.add(sun);
  }

  // ---- 地面 ----
  private setupGround(): void {
    const geometry = new THREE.PlaneGeometry(300, 300);
    const material = new THREE.MeshLambertMaterial({ color: 0x4a7c4f });
    const ground = new THREE.Mesh(geometry, material);
    ground.rotation.x = -Math.PI / 2;
    ground.receiveShadow = true;
    this.scene.add(ground);
  }

  // ---- 网格参考线 ----
  private setupGrid(): void {
    const grid = new THREE.GridHelper(300, 30, 0x888888, 0xcccccc);
    this.scene.add(grid);
  }

  // ---- 创建演示建筑群 ----
  private createDemoBuildings(): void {
    // 不同尺寸和颜色的建筑，模拟复杂建筑群
    const buildingDefs = [
      { pos: [ -40, 15, -40] as const, size: [16, 30, 14] as const, color: 0x8899aa },
      { pos: [ -20, 22, -35] as const, size: [12, 44, 10] as const, color: 0xaabbcc },
      { pos: [   0, 12, -40] as const, size: [18, 24, 16] as const, color: 0x99aabb },
      { pos: [  25, 25, -30] as const, size: [10, 50, 12] as const, color: 0x7788aa },
      { pos: [  45, 18, -45] as const, size: [14, 36, 18] as const, color: 0x8899bb },
      { pos: [ -45, 20,   0] as const, size: [20, 40, 20] as const, color: 0x667788 },
      { pos: [ -25, 10,  10] as const, size: [10, 20, 10] as const, color: 0x99aacc },
      { pos: [   5, 18,   5] as const, size: [16, 36, 14] as const, color: 0xaabbdd },
      { pos: [  35, 12,  15] as const, size: [12, 24, 16] as const, color: 0x8899cc },
      { pos: [  50, 16,   0] as const, size: [22, 32, 18] as const, color: 0x7788bb },
      { pos: [ -15, 28,  35] as const, size: [14, 56, 12] as const, color: 0x667799 },
      { pos: [  20, 14,  40] as const, size: [18, 28, 14] as const, color: 0x8899aa },
      { pos: [ -50, 20, -25] as const, size: [16, 40, 12] as const, color: 0x99aabb },
      { pos: [  10, 30, -20] as const, size: [20, 60, 16] as const, color: 0x778899 },
    ];

    this.buildingOBBs = buildingDefs.map((def, i) => {
      const geometry = new THREE.BoxGeometry(def.size[0], def.size[1], def.size[2]);
      const material = new THREE.MeshLambertMaterial({ color: def.color });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.set(def.pos[0], def.pos[1], def.pos[2]);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.buildings.add(mesh);

      // 边缘线，让建筑轮廓更清晰
      const edges = new THREE.EdgesGeometry(geometry);
      const line = new THREE.LineSegments(
        edges,
        new THREE.LineBasicMaterial({ color: 0x333333 })
      );
      mesh.add(line);

      return {
        id: `building_${i}`,
        center: [...def.pos] as [number, number, number],
        halfExtents: def.size.map(s => s / 2) as [number, number, number],
        rotation: [0, 0, 0],
      };
    });
  }

  // ---- 暴露包围盒数据给人 2 ----
  public getSceneData(): SceneData {
    return {
      buildings: this.buildingOBBs,
      bounds: { min: [-150, 0, -150], max: [150, 80, 150] },
    };
  }

  // ---- 加载无人机 3D 模型 ----
  // 人 1 在阶段 3 完善：替换为 GLTF 模型
  public createDroneModel(): THREE.Group {
    const group = new THREE.Group();

    // 机身：扁平长方体
    const bodyGeo = new THREE.BoxGeometry(1.2, 0.2, 1.2);
    const bodyMat = new THREE.MeshLambertMaterial({ color: 0x333333 });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    group.add(body);

    // 四个机臂
    const armGeo = new THREE.CylinderGeometry(0.06, 0.06, 2.2, 8);
    const armMat = new THREE.MeshLambertMaterial({ color: 0x555555 });
    for (const angle of [0.785, 2.356, 3.927, 5.498]) {
      const arm = new THREE.Mesh(armGeo, armMat);
      arm.rotation.x = Math.PI / 2;
      arm.rotation.z = angle;
      group.add(arm);
    }

    // 四个旋翼盘
    const rotorGeo = new THREE.CylinderGeometry(0.5, 0.5, 0.05, 16);
    const rotorMat = new THREE.MeshLambertMaterial({ color: 0x888888 });
    for (const angle of [0.785, 2.356, 3.927, 5.498]) {
      const rotor = new THREE.Mesh(rotorGeo, rotorMat);
      rotor.position.set(
        Math.cos(angle) * 1.1,
        0.15,
        Math.sin(angle) * 1.1
      );
      group.add(rotor);
    }

    this.droneModel = group;
    this.scene.add(group);
    return group;
  }

  // ---- 更新无人机位置 ----
  public updateDrone(state: DroneState): void {
    if (!this.droneModel) return;

    this.droneModel.position.set(...state.position);
    const [x, y, z, w] = state.orientation;
    this.droneModel.quaternion.set(x, y, z, w);

    // 更新飞行轨迹
    this.trajectoryPoints.push(this.droneModel.position.clone());
    this.updateTrajectoryLine();
  }

  // ---- 飞行轨迹线 ----
  private updateTrajectoryLine(): void {
    if (this.trajectoryLine) {
      this.scene.remove(this.trajectoryLine);
      this.trajectoryLine.geometry.dispose();
    }

    if (this.trajectoryPoints.length < 2) return;

    const geometry = new THREE.BufferGeometry().setFromPoints(
      this.trajectoryPoints
    );
    const material = new THREE.LineBasicMaterial({ color: 0xff6600 });
    this.trajectoryLine = new THREE.Line(geometry, material);
    this.scene.add(this.trajectoryLine);
  }

  // ---- 传感器射线可视化 ----
  // 人 1 在阶段 3 完善：展示激光雷达扫描效果
  public updateSensorRays(
    origin: THREE.Vector3,
    directions: THREE.Vector3[],
    distances: number[]
  ): void {
    // 先清除旧射线
    this.sensorLines.forEach(l => {
      this.scene.remove(l);
      l.geometry.dispose();
    });
    this.sensorLines = [];

    for (let i = 0; i < directions.length; i++) {
      const end = origin.clone().add(
        directions[i].clone().multiplyScalar(distances[i])
      );
      const geometry = new THREE.BufferGeometry().setFromPoints([
        origin, end,
      ]);
      const material = new THREE.LineBasicMaterial({
        color: distances[i] < 5 ? 0xff0000 : 0x00ff00,
      });
      const line = new THREE.Line(geometry, material);
      this.scene.add(line);
      this.sensorLines.push(line);
    }
  }

  // ---- 重置 ----
  public reset(): void {
    this.trajectoryPoints = [];
    if (this.trajectoryLine) {
      this.scene.remove(this.trajectoryLine);
      this.trajectoryLine.geometry.dispose();
      this.trajectoryLine = null;
    }
    this.sensorLines.forEach(l => {
      this.scene.remove(l);
      l.geometry.dispose();
    });
    this.sensorLines = [];
  }

  // ---- 渲染帧 ----
  public render(): void {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  // ---- 清理 ----
  public dispose(): void {
    this.renderer.dispose();
  }
}
