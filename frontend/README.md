# Fleet · 多无人机监控前端

本地运行的 React + TypeScript + Vite 前端。默认使用两机模拟遥测和 Canvas 生成的相机示意画面，**未连接 AirSim，也不执行真实 ORCA 仿真**。不发送任何飞行控制指令。

## 启动

需要 Node.js 20.19+ 或 22.12+。

```powershell
cd D:\30817\Adrone\AirsimAlgorithm-feature-wang\frontend
npm.cmd install
npm.cmd run dev
```

打开终端输出的本地地址，通常为 http://127.0.0.1:5173 。停止服务用 Ctrl+C。

```powershell
npm.cmd run build
npm.cmd run preview
```

`build` 先进行 TypeScript 检查，再生成 `dist/`。前端与原有 Python 导航进程独立，不修改飞行配置或 Python 依赖。

构建还会生成 `dist/standalone.html`，内含全部脚本和样式，可双击在浏览器中离线查看演示，无需启动服务。真实本地遥测建议通过 `npm.cmd run dev` 接入，以免浏览器对本地文件的网络访问施加额外限制。

使用 Node.js 22.18+ 时，可执行 `npm.cmd test` 验证遥测校验、坐标距离、历史航迹上限和演示航线。

## 第一版功能

- 局部场景地图：拖动、缩放、全局适配、选择无人机、跟随、图层开关。
- 多机航迹、目标点、障碍物矩形与半径 2 m 的演示安全圈。
- 各机相机卡片及放大查看；真实模式支持 JPEG / MJPEG 的 HTTP(S) 图像地址。
- 状态表：三维速度、相对共同原点高度、三维距目标距离、NED 位置。
- 暂停 / 继续、重新播放、0.5× / 1× / 2× 演示速度；真实模式禁用演示控制。
- WebSocket 遥测连接入口、帧校验、断连和超过 3 秒的数据过期提示。
- 桌面、平板和手机响应式布局。

地图不是地理卫星地图。它使用共同世界 NED 坐标，X 向北、Y 向东、Z 向下，单位 m；−Z 为原点上方高度，不是离地高度。实际 AirSim 每机局部位置必须先由后端转换到共同原点，不能直接叠加。障碍物当前是演示矩形；真实场景应由后端传入已知场景几何或建图结果，前端不从相机自动建图。

## 接入真实数据

后续新增本地 Python 采集服务（本版未实现），通过独立 AirSim RPC 客户端读取位置和相机，订阅各导航进程的规划/避让状态，再转发 WebSocket。不要把图像读取和编码加入飞行控制主循环。

界面右上角“连接 AirSim 数据”默认连接 `ws://127.0.0.1:8000/ws/telemetry`。连接尝试开始后清除模拟数据；错误、断连和等待首帧会明确显示，不自动切回模拟。点击“返回演示”显式切换。

WebSocket 每帧发送一个 JSON 文本对象，建议遥测 5–10 Hz。所有字段必填，只有 `trail` 和 `camera_url` 可省略；每个 `id` 必须唯一。`timestamp` 为 Unix 毫秒，需保持前后端时钟同步。过期超过 10 秒、超前超过 5 秒、乱序或格式错误的帧会被拒绝。

```json
{
  "type": "telemetry",
  "timestamp": 1793318400000,
  "frame": "world_ned",
  "drones": [
    {
      "id": "Drone1",
      "position": { "x": 0, "y": 0, "z": -1 },
      "velocity": { "x": 1.2, "y": 0.1, "z": 0 },
      "yaw": 0.083,
      "goal": { "x": 30, "y": 40, "z": -1 },
      "status": "navigating",
      "trail": [{ "x": 0, "y": 0, "z": -1 }],
      "camera_url": "http://127.0.0.1:8000/cameras/Drone1.mjpg"
    }
  ],
  "obstacles": [
    { "id": "building-A", "x": 10, "y": 15, "width": 8, "depth": 6 }
  ]
}
```

示例时间戳仅说明格式，发送时应替换为当前时间：`int(time.time() * 1000)`。

- `yaw`：弧度，0 指北（+X），正方向由北朝东。
- `status`：`navigating` / `yielding` / `arrived` / `offline`。
- `trail`：共同世界坐标的历史位置，前端最多保留每机最新 600 点。
- `obstacles`：可为空数组。`x/y` 为矩形中心，`width` 沿 Y（东），`depth` 沿 X（北），均为正数。
- `camera_url`：仅允许 HTTP(S) 图像地址；不支持直接播放 RTSP、WebRTC 或普通 MP4。HTTPS 页面需要 WSS 和 HTTPS 资源。
- 仅接收遥测不会自动产生图像流；后端还需单独提供 JPEG/MJPEG 端点。遥测在线不保证相机在线。
- 真实模式不假设避障算法为 ORCA；策略字段尚未加入协议，因此界面显示“尚未接入策略遥测”。
- 安全圈目前固定为 2 m 的界面示意，不代表从后端读取的真实安全阈值。前端的机间距离提醒也只用固定 2 m。

## 文件

- `src/main.tsx`：监控界面、交互、地图、演示相机、WebSocket 生命周期。
- `src/telemetry.ts`：类型、运行时校验、模拟数据。
- `src/style.css`：布局与响应式样式。

页面无地图服务和 API Key 依赖，核心界面不依赖外部服务。
