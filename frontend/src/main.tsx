import React, { useCallback, useEffect, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import { COLORS, cameraUrl, demoFrame, distance, parseFrame, speed } from './telemetry';
import type { Drone, TelemetryFrame } from './telemetry';
import './style.css';

type IconName = 'grid' | 'map' | 'drone' | 'activity' | 'settings' | 'chevron' | 'play' | 'pause' | 'reset' | 'plus' | 'minus' | 'fit' | 'layers' | 'expand' | 'close' | 'link' | 'arrow' | 'check' | 'help';
function Icon({ name, size = 19 }: { name: IconName; size?: number }) {
  const paths: Record<IconName, React.ReactNode> = {
    grid: <><rect x="3" y="3" width="7" height="7" rx="1.5"/><rect x="14" y="3" width="7" height="7" rx="1.5"/><rect x="3" y="14" width="7" height="7" rx="1.5"/><rect x="14" y="14" width="7" height="7" rx="1.5"/></>,
    map: <><path d="m3 6 6-3 6 3 6-3v15l-6 3-6-3-6 3Z"/><path d="M9 3v15m6-12v15"/></>,
    drone: <><circle cx="5" cy="5" r="3"/><circle cx="19" cy="5" r="3"/><circle cx="5" cy="19" r="3"/><circle cx="19" cy="19" r="3"/><path d="m7 7 10 10m0-10L7 17"/><rect x="9" y="9" width="6" height="6" rx="2"/></>,
    activity: <path d="M2 12h5l3-8 4 16 3-8h5"/>,
    settings: <><path d="m10 3-1 3-3 1-3 3v4l3 1 1 3 3 3h4l1-3 3-1 3-3v-4l-3-1-1-3-3-3Z"/><circle cx="12" cy="12" r="3"/></>,
    chevron: <path d="m9 5 7 7-7 7"/>, play: <path d="m8 4 12 8-12 8Z"/>, pause: <><path d="M8 5v14M16 5v14"/></>,
    reset: <><path d="M3 10a9 9 0 1 1 1 7M3 4v6h6"/></>, plus: <path d="M12 5v14M5 12h14"/>, minus: <path d="M5 12h14"/>,
    fit: <><path d="M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5"/><circle cx="12" cy="12" r="3"/></>,
    layers: <><path d="m12 3 10 6-10 6L2 9Z"/><path d="m2 13 10 6 10-6M2 17l10 6 10-6"/></>,
    expand: <path d="M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5"/>, close: <path d="m6 6 12 12M6 18 18 6"/>,
    link: <><path d="m10 7 3-3a5 5 0 0 1 7 7l-3 3M14 17l-3 3a5 5 0 0 1-7-7l3-3M8 16l8-8"/></>,
    arrow: <path d="M4 12h16m-6-6 6 6-6 6"/>, check: <path d="m5 12 4 4L19 6"/>, help: <><circle cx="12" cy="12" r="9"/><path d="M9 9a3 3 0 1 1 5 2c-2 1-2 1-2 3m0 3h.01"/></>,
  };
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.65" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{paths[name]}</svg>;
}
const statusText = { navigating: '自主导航', yielding: '减速让行', arrived: '已到达', offline: '离线' };
const clock = (date: number) => new Date(date).toLocaleTimeString('zh-CN', { hour12: false });

function Camera({ drone, index, seconds, demo, large = false, stale }: { drone: Drone; index: number; seconds: number; demo: boolean; large?: boolean; stale: boolean }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [failed, setFailed] = useState(false);
  useEffect(() => setFailed(false), [drone.camera_url]);
  useEffect(() => {
    if (!demo) return;
    const canvas = canvasRef.current, ctx = canvas?.getContext('2d');
    if (!canvas || !ctx) return;
    const w = 960, h = 500; canvas.width = w; canvas.height = h;
    const horizon = 190;
    const sky = ctx.createLinearGradient(0, 0, 0, h); sky.addColorStop(0, '#8ca9ba'); sky.addColorStop(0.38, '#d8e2df'); sky.addColorStop(0.39, '#899b8d'); sky.addColorStop(1, '#5e7466');
    ctx.fillStyle = sky; ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = '#93a99f';
    for (let n = 0; n < 35; n++) { const x = n * 35; const height = 16 + Math.sin(n * 4.5) * 12; ctx.fillRect(x, horizon - height, 32, height); }
    ctx.strokeStyle = '#b5c1ae45'; ctx.lineWidth = 1;
    for (let n = -8; n <= 8; n++) { ctx.beginPath(); ctx.moveTo(w / 2 + n * 25, horizon); ctx.lineTo(w / 2 + n * 260, h); ctx.stroke(); }
    for (let n = 1; n < 9; n++) { const y = horizon + ((n + (seconds % 3) / 3) / 9) ** 2 * 350; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
    // A procedural preview illustrates the layout; it never represents an AirSim image.
    const sway = Math.sin(seconds * 0.09 + index * 2) * 75;
    const blocks = index ? [[70, 240, 225, 155], [610, 265, 200, 112], [414, 210, 75, 55]] : [[-40, 280, 285, 170], [670, 260, 220, 150], [310, 200, 90, 70]];
    for (const [bx, base, bw, bh] of blocks) {
      const x = bx + sway, top = base - bh, side = 35;
      ctx.fillStyle = '#b8b9aa'; ctx.fillRect(x, top, bw, bh);
      ctx.fillStyle = '#858e80'; ctx.beginPath(); ctx.moveTo(x + bw, top); ctx.lineTo(x + bw + side, top - 15); ctx.lineTo(x + bw + side, base - 15); ctx.lineTo(x + bw, base); ctx.fill();
      ctx.fillStyle = '#d3d2c2'; ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x + side, top - 15); ctx.lineTo(x + bw + side, top - 15); ctx.lineTo(x + bw, top); ctx.fill();
      ctx.fillStyle = '#45594e28'; ctx.beginPath(); ctx.moveTo(x, base); ctx.lineTo(x + bw, base); ctx.lineTo(x + bw + 80, base + 35); ctx.lineTo(x + 40, base + 35); ctx.fill();
    }
    ctx.strokeStyle = '#d4f9dd'; ctx.lineWidth = 3; ctx.setLineDash([10, 10]); ctx.beginPath(); ctx.moveTo(480, 510); ctx.quadraticCurveTo(470 + sway, 300, 490, 205); ctx.stroke(); ctx.setLineDash([]);
    const shade = ctx.createLinearGradient(0, 0, 0, h); shade.addColorStop(0, '#071c1438'); shade.addColorStop(0.4, '#00000000'); shade.addColorStop(1, '#04130c77'); ctx.fillStyle = shade; ctx.fillRect(0, 0, w, h);
  }, [demo, index, seconds]);
  const url = drone.camera_url ? cameraUrl(drone.camera_url) : undefined;
  return <div className={`camera-image ${large ? 'large' : ''}`}>
    {demo ? <canvas ref={canvasRef} aria-label={`${drone.id} 程序生成的演示视角，非 AirSim 画面`} /> : url && !failed ? <img src={url} alt={`${drone.id} 实时相机`} onError={() => setFailed(true)} /> : <div className="camera-empty"><Icon name="drone" size={32}/><span>{failed ? '图像流无法加载' : '尚未接入相机'}</span><small>需要 HTTP(S) JPEG / MJPEG 图像地址</small></div>}
    <div className="camera-top"><span><i className={`dot ${stale ? 'gray' : ''}`}/>{demo ? '模拟视角' : stale ? '数据已过期' : '相机画面'}</span><span>FRONT · 01</span></div>
    {(demo || (url && !failed)) && <><div className="reticle"><span/><span/></div><div className="camera-bottom"><span>ALT <b>{(-drone.position.z).toFixed(1)}</b> m</span><span>SPD <b>{speed(drone).toFixed(1)}</b> m/s</span><span>{demo ? 'DEMO' : 'LIVE'}</span></div></>}
  </div>;
}

type Layers = { obstacles: boolean; trails: boolean; safety: boolean; goals: boolean };
function WorldMap({ frame, selected, select, follow, setFollow, layers, setLayers, stale }: {
  frame: TelemetryFrame; selected: string; select: (id: string) => void; follow: boolean; setFollow: (f: boolean) => void;
  layers: Layers; setLayers: React.Dispatch<React.SetStateAction<Layers>>; stale: boolean;
}) {
  const [view, setView] = useState({ x: -65, y: -43, w: 130, h: 86 });
  const [layerMenu, setLayerMenu] = useState(false);
  const [mapSize, setMapSize] = useState({ width: 850, height: 435 });
  const svgRef = useRef<SVGSVGElement>(null);
  const drag = useRef<{ x: number; y: number; vx: number; vy: number; moved: boolean } | null>(null);
  const chosen = frame.drones.find(d => d.id === selected) ?? frame.drones[0];
  useEffect(() => {
    const svg = svgRef.current;
    if (!svg) return;
    const observer = new ResizeObserver(([entry]) => setMapSize({ width: entry.contentRect.width, height: entry.contentRect.height }));
    observer.observe(svg);
    return () => observer.disconnect();
  }, []);
  const centerX = follow && chosen ? chosen.position.y : view.x + view.w / 2;
  const centerY = follow && chosen ? -chosen.position.x : view.y + view.h / 2;
  const x0 = centerX - view.w / 2, y0 = centerY - view.h / 2;
  function zoom(factor: number) { setView(v => { const w = Math.max(20, Math.min(600, v.w * factor)), h = w * 86 / 130; return { x: centerX - w / 2, y: centerY - h / 2, w, h }; }); }
  const fit = () => {
    const points = frame.drones.flatMap(d => [d.position, d.goal, ...d.trail]);
    frame.obstacles.forEach(o => points.push({ x: o.x - o.depth / 2, y: o.y - o.width / 2, z: 0 }, { x: o.x + o.depth / 2, y: o.y + o.width / 2, z: 0 }));
    if (!points.length) return;
    const xs = points.map(p => p.y), ys = points.map(p => -p.x);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    const w = Math.max(60, maxX - minX + 26, (maxY - minY + 26) * 130 / 86), h = w * 86 / 130;
    setFollow(false); setView({ x: (minX + maxX - w) / 2, y: (minY + maxY - h) / 2, w, h });
  };
  // Fit once when the source changes, without following every telemetry update.
  const initialIds = frame.drones.map(d => d.id).join('|');
  useEffect(() => { fit(); }, [initialIds]); // eslint intentionally not configured
  // Keep drone markers and labels readable at every viewport and zoom level.
  const scale = view.w / Math.max(1, Math.min(mapSize.width, mapSize.height * 130 / 86)) * 6;
  const polyline = (points: Drone['trail']) => points.map(p => `${p.y},${-p.x}`).join(' ');
  return <div className="map-stage">
    <svg ref={svgRef} className="world-map" viewBox={`${x0} ${y0} ${view.w} ${view.h}`} role="img" aria-label="无人机全局地图，横轴 Y 向东，纵轴 X 向北；支持拖动和缩放"
      onWheel={e => zoom(e.deltaY > 0 ? 1.12 : 0.89)}
      onPointerDown={e => { if (e.button !== 0) return; drag.current = { x: e.clientX, y: e.clientY, vx: x0, vy: y0, moved: false }; e.currentTarget.setPointerCapture(e.pointerId); }}
      onPointerMove={e => { const d = drag.current, svg = svgRef.current; if (!d || !svg) return; const rect = svg.getBoundingClientRect(); const ratio = Math.max(view.w / rect.width, view.h / rect.height); if (Math.hypot(e.clientX - d.x, e.clientY - d.y) > 3) { d.moved = true; setFollow(false); setView(v => ({ ...v, x: d.vx - (e.clientX - d.x) * ratio, y: d.vy - (e.clientY - d.y) * ratio })); } }}
      onPointerUp={() => { drag.current = null; }} onPointerCancel={() => { drag.current = null; }}>
      <defs><pattern id="grid-small" width="2" height="2" patternUnits="userSpaceOnUse"><path d="M 2 0 L 0 0 0 2" fill="none" stroke="#dfe5e6" strokeWidth=".07"/></pattern><pattern id="grid-major" width="10" height="10" patternUnits="userSpaceOnUse"><rect width="10" height="10" fill="url(#grid-small)"/><path d="M 10 0 L 0 0 0 10" fill="none" stroke="#d4dcdf" strokeWidth=".13"/></pattern><pattern id="hatch" width="1" height="1" patternTransform="rotate(35)" patternUnits="userSpaceOnUse"><path d="M0 0v1" stroke="#b6c0c5" strokeWidth=".17"/></pattern></defs>
      <rect x={x0} y={y0} width={view.w} height={view.h} fill="#f3f6f6"/><rect x={x0} y={y0} width={view.w} height={view.h} fill="url(#grid-major)"/>
      <rect x="-51" y="-33" width="102" height="66" rx="1" fill="#ffffff30" stroke="#b8c5c8" strokeWidth=".2" strokeDasharray="1 1"/>
      <text x="-49" y="-30" fontSize="1.5" fill="#9aa8ad" letterSpacing=".25">SIMULATION AREA / 01</text>
      <path d="M-80 0h160M0-60V60" stroke="#bdcbcf" strokeWidth=".15" strokeDasharray=".7 1"/>
      {[-40, -20, 0, 20, 40].map(n => <g key={n}><text x={n + 0.7} y="2.2" fontSize="1.4" fill="#9ba8af">{n}</text><text x="1" y={n - 0.5} fontSize="1.4" fill="#9ba8af">{-n}</text></g>)}
      {layers.obstacles && frame.obstacles.map(o => <g key={o.id}>
        <rect x={o.y - o.width / 2 + 0.65} y={-o.x - o.depth / 2 + 0.8} width={o.width} height={o.depth} rx=".5" fill="#27383e0d"/>
        <rect x={o.y - o.width / 2} y={-o.x - o.depth / 2} width={o.width} height={o.depth} rx=".5" fill="#dce2e4" stroke="#b8c3c8" strokeWidth=".16"/>
        <rect x={o.y - o.width / 2 + 0.6} y={-o.x - o.depth / 2 + 0.6} width={Math.max(0, o.width - 1.2)} height={Math.max(0, o.depth - 1.2)} rx=".2" fill="url(#hatch)"/>
        <text x={o.y} y={-o.x + 0.6} textAnchor="middle" fontSize="1.7" fill="#87959e">{o.id}</text>
      </g>)}
      {frame.drones.map((d, i) => {
        const color = COLORS[i % COLORS.length], active = d.id === selected;
        return <g key={d.id} opacity={stale || d.status === 'offline' ? 0.45 : 1}>
          {layers.goals && <><line x1={d.position.y} y1={-d.position.x} x2={d.goal.y} y2={-d.goal.x} stroke={color} strokeWidth=".2" strokeDasharray=".7 1" opacity=".32"/><g transform={`translate(${d.goal.y} ${-d.goal.x})`}><circle r="1.7" fill="white" stroke={color} strokeWidth=".22"/><circle r=".6" fill={color}/><text x="3" y=".5" fill={color} fontSize="1.8">{d.id} 目标</text></g></>}
          {layers.trails && <polyline points={polyline(d.trail)} fill="none" stroke={color} strokeWidth={active ? '.55' : '.35'} strokeLinejoin="round" strokeLinecap="round"/>}
          {layers.safety && <circle cx={d.position.y} cy={-d.position.x} r="2" fill={`${color}12`} stroke={color} strokeWidth=".16" strokeDasharray=".6 .5"/>}
          <g transform={`translate(${d.position.y} ${-d.position.x})`} role="button" tabIndex={0} aria-label={`在地图上选择 ${d.id}`} className="map-drone" onClick={() => select(d.id)} onPointerDown={e => e.stopPropagation()} onKeyDown={e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); select(d.id); } }}>
            <circle r={3.7 * scale} fill="transparent"/>
            {active && <circle r={3.1 * scale} fill={`${color}12`} stroke={color} strokeWidth=".15"/>}
            <g transform={`rotate(${d.yaw * 180 / Math.PI}) scale(${scale})`}><path d="M0-2.25 1.65 1.7 0 .9-1.65 1.7Z" fill={color} stroke="white" strokeWidth=".4"/></g>
            <g transform={`translate(${4 * scale} ${-4 * scale}) scale(${scale})`}><rect x="-1" y="-2" width="14" height="5" rx="1" fill="white" stroke="#dde4e8" strokeWidth=".15"/><circle cx="1" cy=".5" r=".4" fill={color}/><text x="2.4" y="1" fontSize="1.9" fill="#263841" fontWeight="600">{d.id}</text></g>
          </g>
        </g>;
      })}
      <g transform={`translate(${x0 + view.w - 20 * scale} ${y0 + view.h - 6 * scale})`} pointerEvents="none">
        <path d="M0 -0.7V0H10V-0.7" fill="none" stroke="#91a2aa" strokeWidth={0.12 * scale}/>
        <text x="5" y={2.2 * scale} textAnchor="middle" fill="#91a2aa" fontSize={1.3 * scale}>10 m</text>
      </g>
    </svg>
    <div className="map-meta"><span><i className="dot"/>局部场景地图</span><span>NED · 米制坐标</span></div>
    <div className="map-compass"><b>N</b><svg width="23" height="32" viewBox="0 0 23 32"><path d="M11.5 2 19 25 11.5 20Z" fill="#34454c"/><path d="M11.5 2 4 25 11.5 20Z" fill="#b2bdc4"/></svg></div>
    <div className="map-tools"><button aria-label="放大地图" onClick={() => zoom(.8)}><Icon name="plus"/></button><button aria-label="缩小地图" onClick={() => zoom(1.25)}><Icon name="minus"/></button><div/><button aria-label="显示全部无人机" onClick={fit}><Icon name="fit"/></button><button aria-label="地图图层" aria-expanded={layerMenu} onClick={() => setLayerMenu(!layerMenu)}><Icon name="layers"/></button></div>
    {layerMenu && <div className="layer-menu"><strong>地图图层</strong>{(Object.keys(layers) as (keyof Layers)[]).map(k => <label key={k}><input type="checkbox" checked={layers[k]} onChange={e => setLayers(v => ({ ...v, [k]: e.target.checked }))}/>{{ obstacles: '场景障碍物', trails: '历史航迹', safety: '2 m 演示安全圈', goals: '目标点与引导线' }[k]}</label>)}</div>}
    <div className="map-legend">{frame.drones.map((d, i) => <button key={d.id} onClick={() => select(d.id)} className={selected === d.id ? 'active' : ''}><i style={{ background: COLORS[i % COLORS.length] }}/>{d.id}</button>)}<span><i className="obstacle-key"/>障碍物</span></div>
    <div className="map-hint">拖动平移 · 滚轮缩放</div>
  </div>;
}

function App() {
  const [source, setSource] = useState<'demo' | 'live'>('demo');
  const [frame, setFrame] = useState<TelemetryFrame>(() => demoFrame(23));
  const [selected, setSelected] = useState('Drone1');
  const [seconds, setSeconds] = useState(23);
  const [paused, setPaused] = useState(false), [rate, setRate] = useState(1);
  const [follow, setFollow] = useState(false);
  const [layers, setLayers] = useState<Layers>({ obstacles: true, trails: true, safety: true, goals: true });
  const [modal, setModal] = useState<'connect' | 'help' | null>(null), [expanded, setExpanded] = useState<string | null>(null);
  const [url, setUrl] = useState('ws://127.0.0.1:8000/ws/telemetry');
  const [connection, setConnection] = useState('未连接'), [error, setError] = useState('');
  const [now, setNow] = useState(Date.now()), [lastReceived, setLastReceived] = useState(0);
  const [events, setEvents] = useState<{ at: number; message: string; type: 'info' | 'warn' }[]>([{ at: Date.now(), message: '演示场景已载入 · 两机独立导航', type: 'info' }]);
  const ws = useRef<WebSocket | null>(null), statusRef = useRef(''), lastTimestamp = useRef(0);
  const overviewRef = useRef<HTMLElement>(null), fleetRef = useRef<HTMLElement>(null), eventsRef = useRef<HTMLElement>(null);
  const addEvent = useCallback((message: string, type: 'info' | 'warn' = 'info') => setEvents(v => [{ at: Date.now(), message, type }, ...v].slice(0, 40)), []);
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(timer); }, []);
  useEffect(() => {
    if (source !== 'demo' || paused) return;
    const timer = setInterval(() => setSeconds(v => v + .1 * rate), 100);
    return () => clearInterval(timer);
  }, [source, paused, rate]);
  useEffect(() => { if (source === 'demo') setFrame(demoFrame(seconds)); }, [seconds, source]);
  useEffect(() => {
    const key = frame.drones.map(d => `${d.id}:${d.status}`).join('|');
    if (key !== statusRef.current) { if (statusRef.current) frame.drones.forEach(d => addEvent(`${source === 'demo' ? '[演示] ' : ''}${d.id} · ${statusText[d.status]}`, d.status === 'yielding' || d.status === 'offline' ? 'warn' : 'info')); statusRef.current = key; }
  }, [frame, source, addEvent]);
  useEffect(() => () => { if (ws.current) { ws.current.onclose = null; ws.current.close(); } }, []);
  useEffect(() => { if (!frame.drones.some(d => d.id === selected)) setSelected(frame.drones[0]?.id ?? ''); }, [frame.drones, selected]);
  useEffect(() => {
    const handler = (event: KeyboardEvent) => { if (event.key === 'Escape') { setModal(null); setExpanded(null); } };
    document.addEventListener('keydown', handler); return () => document.removeEventListener('keydown', handler);
  }, []);
  function demo() {
    if (ws.current) { ws.current.onclose = null; ws.current.close(); ws.current = null; }
    setSource('demo'); setConnection('未连接'); setError(''); setModal(null); setSeconds(23); setPaused(false); setLastReceived(0); lastTimestamp.current = 0; statusRef.current = ''; addEvent('已切换至演示数据');
  }
  function connect() {
    try {
      const endpoint = new URL(url);
      if (!['ws:', 'wss:'].includes(endpoint.protocol)) throw new Error('请输入 ws:// 或 wss:// 地址');
      if (location.protocol === 'https:' && endpoint.protocol !== 'wss:') throw new Error('HTTPS 页面需要 wss:// 服务');
      if (ws.current) { ws.current.onclose = null; ws.current.close(); }
      const socket = new WebSocket(endpoint.href); ws.current = socket; setConnection('连接中'); setError('');
      setSource('live'); setFrame({ type: 'telemetry', timestamp: 0, frame: 'world_ned', drones: [], obstacles: [] }); setFollow(false); setLastReceived(0); lastTimestamp.current = 0; statusRef.current = '';
      const timeout = window.setTimeout(() => { if (ws.current === socket && socket.readyState === WebSocket.CONNECTING) { setError('连接超时，请检查本地遥测服务'); socket.close(); } }, 8000);
      socket.onopen = () => { clearTimeout(timeout); setConnection('等待数据'); addEvent('遥测连接已建立，等待首帧'); };
      socket.onmessage = event => {
        if (ws.current !== socket) return;
        try { if (typeof event.data !== 'string') throw new Error('遥测需要 JSON 文本帧'); const next = parseFrame(event.data); if (next.timestamp < lastTimestamp.current) throw new Error('已丢弃乱序遥测帧'); lastTimestamp.current = next.timestamp; setFrame(next); setLastReceived(Date.now()); setConnection('已连接'); setError(''); }
        catch (e) { setError(e instanceof Error ? e.message : '无法解析遥测'); }
      };
      socket.onerror = () => { if (ws.current === socket) setError('无法连接遥测服务，请检查地址、端口与服务状态'); };
      socket.onclose = () => { clearTimeout(timeout); if (ws.current === socket) { setConnection('已断开'); addEvent('遥测连接已断开', 'warn'); } };
    } catch (e) { setError(e instanceof Error ? e.message : '连接失败'); }
  }
  const stale = source === 'live' && (!lastReceived || now - lastReceived > 3000 || now - frame.timestamp > 3000 || connection === '已断开');
  const drones = frame.drones, chosen = drones.find(d => d.id === selected) ?? drones[0];
  const online = stale ? 0 : drones.filter(d => d.status !== 'offline').length;
  const minDistance = drones.length > 1 ? Math.min(...drones.flatMap((d, i) => drones.slice(i + 1).map(b => distance(d.position, b.position)))) : null;
  const expandedDrone = drones.find(d => d.id === expanded);
  const age = lastReceived ? Math.max(0, now - frame.timestamp) / 1000 : null;
  return <div className="app-shell">
    <aside className="rail"><a className="brand-mark" href="#overview" aria-label="Fleet 首页"><Icon name="drone" size={27}/></a><nav><button className="rail-button active" aria-label="监控总览" title="监控总览" onClick={() => overviewRef.current?.scrollIntoView({ behavior: 'smooth' })}><Icon name="grid"/></button><button className="rail-button" aria-label="定位场景地图" title="场景地图" onClick={() => document.querySelector('#scene-map')?.scrollIntoView({ behavior: 'smooth' })}><Icon name="map"/></button><button className="rail-button" aria-label="无人机状态" title="无人机状态" onClick={() => fleetRef.current?.scrollIntoView({ behavior: 'smooth' })}><Icon name="drone"/></button><button className="rail-button" aria-label="事件日志" title="事件日志" onClick={() => eventsRef.current?.scrollIntoView({ behavior: 'smooth' })}><Icon name="activity"/></button></nav><div className="rail-bottom"><button className="rail-button" aria-label="连接设置" title="连接设置" onClick={() => setModal('connect')}><Icon name="settings"/></button><button className="rail-button" aria-label="使用说明" title="使用说明" onClick={() => setModal('help')}><Icon name="help"/></button><div className="avatar">LY</div></div></aside>
    <div className="workspace"><header className="topbar"><div className="wordmark">FLEET<span>/</span><span>多无人机监控台</span></div><div className="topbar-right"><span className="environment"><i className={`dot ${source === 'live' && stale ? 'gray' : ''}`}/>{source === 'demo' ? '本地演示环境' : stale ? '遥测未就绪' : 'AirSim 遥测'}</span><span className="header-divider"/><time>{clock(now)}</time><span className="version">v0.1</span></div></header>
    <main id="overview" ref={overviewRef}>
      <div className="page-heading"><div><div className="eyebrow">MISSION CONTROL <span>/</span> OVERVIEW</div><h1>飞行态势总览<span className={`mode-tag ${source === 'live' ? 'live-tag' : ''}`}>{source === 'demo' ? 'DEMO' : 'LIVE'}</span></h1><p>每一个视角，都在同一张地图上。</p></div><div className="heading-actions"><button className="button" onClick={() => setModal('help')}><Icon name="help" size={16}/>使用说明</button><button className="button primary" onClick={() => setModal('connect')}><Icon name="link" size={16}/>连接 AirSim 数据<Icon name="arrow" size={16}/></button></div></div>
      <div className={`demo-banner ${source === 'live' ? 'live-banner' : ''}`}><div><span className="banner-icon"><Icon name={source === 'demo' ? 'activity' : 'link'} size={16}/></span><strong>{source === 'demo' ? '演示模式' : `${connection}${stale ? ' · 暂无有效实时数据' : ''}`}</strong><span>{source === 'demo' ? '地图、航迹与相机均为模拟数据，可直接体验界面交互。' : '只读监控 · 本页面不会向无人机发送飞行指令。'}</span></div><button onClick={() => source === 'demo' ? setModal('connect') : demo()}>{source === 'demo' ? '配置数据源' : '返回演示'}<Icon name="chevron" size={13}/></button></div>
      {source === 'live' && error && <div className="error-banner" role="alert">{error}</div>}
      <div className="stats-grid"><div className="stat"><span>在线无人机<Icon name="drone"/></span><div><b>{online.toString().padStart(2, '0')}</b><em>/ {drones.length.toString().padStart(2, '0')}</em><small className={online ? 'good' : ''}>{online ? '数据正常' : '等待遥测'}</small></div></div><div className="stat"><span>协同避障策略<Icon name="layers"/></span><div><b className="text-value">{source === 'demo' ? 'ORCA' : '—'}</b><small>{source === 'demo' ? '水平速度协同 · 演示' : '尚未接入策略遥测'}</small></div></div><div className="stat"><span>最小机间距离<Icon name="fit"/></span><div><b>{stale || minDistance === null ? '—' : minDistance.toFixed(1)}</b><em>m</em><small className={!stale && minDistance !== null && minDistance < 2 ? 'warn' : ''}>{stale ? '数据过期' : minDistance === null ? '至少需要两机' : minDistance < 2 ? '距离提醒' : '三维欧氏距离'}</small></div></div><div className="stat"><span>{source === 'demo' ? '演示时间' : '数据帧龄'}<Icon name="activity"/></span><div><b className="mono-value">{source === 'demo' ? `${Math.floor(seconds / 60).toString().padStart(2, '0')}:${Math.floor(seconds % 60).toString().padStart(2, '0')}` : age === null ? '—' : age.toFixed(1)}</b><em>{source === 'live' ? 's' : ''}</em><small>{source === 'demo' ? paused ? '已暂停' : `${rate}× 播放中` : stale ? '等待新帧' : '每帧校验'}</small></div></div></div>
      <div className="monitor-grid"><section className="panel map-panel" id="scene-map"><div className="panel-header"><div><Icon name="map" size={18}/><h2>全局态势地图</h2><span className="subtle-label">TOP VIEW</span></div><button className={`follow-toggle ${follow ? 'enabled' : ''}`} disabled={!chosen} onClick={() => setFollow(!follow)}><span/>{follow ? `跟随 ${chosen?.id}` : '跟随选中无人机'}</button></div>
        <WorldMap key={source} frame={frame} selected={selected} select={setSelected} follow={follow} setFollow={setFollow} layers={layers} setLayers={setLayers} stale={stale}/>
        <div className="map-footer"><span><span className="footer-symbol">⌖</span>{chosen ? <><b>{chosen.id}</b><span>X {chosen.position.x.toFixed(1)}</span><span>Y {chosen.position.y.toFixed(1)}</span><span>Z {chosen.position.z.toFixed(1)} m</span></> : '等待世界坐标数据'}</span><span className="map-footer-right">X 北 · Y 东 · Z 向下</span></div></section>
      <section className="panel views-panel"><div className="panel-header"><div><Icon name="drone" size={18}/><h2>机载视角</h2></div><span className="count-tag">{drones.length} 路</span></div><div className="camera-list">{drones.length ? drones.map((d, i) => <article className={`camera-card ${selected === d.id ? 'selected' : ''}`} key={d.id}>
        <div className="camera-card-header"><button onClick={() => setSelected(d.id)}><i style={{ background: COLORS[i % COLORS.length] }}/><b>{d.id}</b><span>{stale ? '数据过期' : statusText[d.status]}</span></button><button aria-label={`放大 ${d.id} 相机`} onClick={() => setExpanded(d.id)}><Icon name="expand" size={15}/></button></div>
        <button className="camera-select" onClick={() => setSelected(d.id)} aria-label={`选择 ${d.id} 视角`}><Camera drone={d} index={i} seconds={seconds} demo={source === 'demo'} stale={stale}/></button>
        <div className="camera-caption"><span>{source === 'demo' ? '程序生成 · 非 AirSim 实拍' : '前向相机 · HTTP 图像流'}</span><button onClick={() => { setSelected(d.id); setFollow(true); }}>地图定位<Icon name="chevron" size={11}/></button></div>
      </article>) : <div className="empty-state"><Icon name="drone" size={38}/><strong>等待无人机接入</strong><span>连接遥测后在这里显示机载视角</span><button className="button" onClick={() => setModal('connect')}>配置数据源</button></div>}</div><div className="views-footnote"><i className="dot"/>{source === 'demo' ? '视角与地图同步演示' : stale ? '连接状态或数据已过期' : '已接收有效遥测'}</div></section></div>
      <div className="lower-grid"><section className="panel fleet-panel" ref={fleetRef}><div className="panel-header"><div><Icon name="drone" size={18}/><h2>无人机状态</h2></div><span className="subtle-label">{source === 'demo' ? 'SIMULATED TELEMETRY' : 'READ-ONLY TELEMETRY'}</span></div><div className="table-scroll"><table><thead><tr><th>无人机</th><th>飞行状态</th><th>速度</th><th>相对高度</th><th>距目标</th><th>位置 · X / Y</th><th/></tr></thead><tbody>{drones.map((d, i) => <tr key={d.id} className={selected === d.id ? 'selected-row' : ''} onClick={() => setSelected(d.id)}><td><span className="drone-name"><i style={{ background: COLORS[i % COLORS.length] }}/>{d.id}</span></td><td><span className={`status-pill ${d.status === 'yielding' ? 'yielding' : stale || d.status === 'offline' ? 'offline' : ''}`}><i/>{stale ? '数据过期' : statusText[d.status]}</span></td><td>{speed(d).toFixed(1)}<small> m/s</small></td><td>{(-d.position.z).toFixed(1)}<small> m</small></td><td>{distance(d.position, d.goal).toFixed(1)}<small> m</small></td><td className="coordinate">{d.position.x.toFixed(1)} / {d.position.y.toFixed(1)}</td><td><button className="icon-button" aria-label={`跟随 ${d.id}`} onClick={e => { e.stopPropagation(); setSelected(d.id); setFollow(true); document.querySelector('#scene-map')?.scrollIntoView({ behavior: 'smooth' }); }}><Icon name="chevron" size={15}/></button></td></tr>)}</tbody></table>{!drones.length && <div className="table-empty">暂无遥测数据</div>}</div><div className="table-note">高度为共同 NED 原点上方高度（−Z），不代表离地高度。{source === 'demo' ? '安全圈半径 2 m，仅作界面演示。' : '障碍物由遥测服务提供。'}</div></section>
      <section className="panel events-panel" ref={eventsRef}><div className="panel-header"><div><Icon name="activity" size={18}/><h2>事件动态</h2></div><span className="subtle-label">最近 {events.length} 条</span></div><div className="events-list">{events.map((event, i) => <div className="event" key={`${event.at}-${i}`}><span className={`event-marker ${event.type}`}><Icon name={event.type === 'warn' ? 'activity' : 'check'} size={11}/></span><div><span>{event.message}</span><time>{clock(event.at)}</time></div></div>)}</div></section></div>
      <footer className="page-footer"><span>FLEET CONSOLE <span>·</span> AirSim 多无人机可视化</span><span>本地场景坐标地图 · 前端原型 v0.1</span></footer>
    </main><div className="playback-bar"><div className="playback-label"><Icon name="activity" size={17}/><b>{source === 'demo' ? '演示播放器' : '实时遥测'}</b><span>{source === 'demo' ? 'DEMO SESSION / 01' : connection}</span></div><div className="playback-controls"><button className="icon-button" aria-label="重置演示" disabled={source !== 'demo'} onClick={() => { setSeconds(0); setPaused(false); addEvent('演示已从起点重新播放'); }}><Icon name="reset" size={17}/></button><button className="play-button" disabled={source !== 'demo'} aria-label={paused ? '播放演示' : '暂停演示'} onClick={() => setPaused(!paused)}><Icon name={paused ? 'play' : 'pause'} size={16}/><span>{paused ? '继续' : '暂停'}</span></button><button className="speed-button" disabled={source !== 'demo'} aria-label="切换演示播放速度" onClick={() => setRate(rate === 1 ? 2 : rate === 2 ? .5 : 1)}>{rate}×</button></div><div className="playback-status"><i className={`dot ${source === 'live' && stale ? 'gray' : ''}`}/>{source === 'demo' ? '仅演示 · 不控制飞行' : '只读连接 · 不控制飞行'}</div></div></div>
    {modal && <Modal title={modal === 'connect' ? '连接 AirSim 数据' : '关于这版监控台'} close={() => setModal(null)}>{modal === 'connect' ? <><p className="modal-intro">前端通过遥测服务读取数据。AirSim RPC 需要由本地 Python 服务转换为 WebSocket；本版未包含该服务。</p><label className="input-label" htmlFor="ws-url">遥测 WebSocket 地址</label><input id="ws-url" className="url-input" value={url} onChange={e => setUrl(e.target.value)} placeholder="ws://127.0.0.1:8000/ws/telemetry"/><div className="connection-state"><i className={`dot ${connection === '已连接' ? '' : 'gray'}`}/>{connection}{source === 'live' && stale ? ' · 等待有效新帧' : ''}</div>{error && <div className="connection-error" role="alert">{error}</div>}<div className="connection-notes"><b>数据接入约定</b><p>统一使用 world_ned 世界坐标；位置单位 m，速度 m/s，yaw 单位 rad。相机使用 HTTP(S) JPEG/MJPEG 地址。详见 frontend/README.md。</p><p>地图障碍物也需由服务传入，本版没有从图像自动建图。断连后会保留最后一帧并标记过期。</p></div><div className="modal-actions"><button className="button" onClick={demo}>使用演示数据</button><button className="button primary" onClick={connect} disabled={connection === '连接中'}><Icon name="link" size={15}/>{connection === '连接中' ? '连接中…' : '连接遥测服务'}</button></div></> : <><p className="modal-intro">这是一版面向双机避障调试的前端原型，默认使用程序生成的演示数据。</p><div className="help-items"><p><b>01 / 地图</b>拖动平移、滚轮缩放；右侧工具可以显示全部无人机、开关图层。点击无人机或卡片选择它，再开启跟随。</p><p><b>02 / 视角</b>相机右上角可放大。演示视角是程序生成的示意图，接入相机地址后才是 AirSim 实时画面。</p><p><b>03 / 数据</b>地图使用 NED 共同世界坐标，X 北、Y 东、Z 向下；它是仿真局部地图，不是地理卫星地图。</p><p><b>04 / 回放</b>底部可暂停、重置和调整演示速度。真实数据模式仅做只读显示，不会控制无人机。</p></div><button className="button primary full-width" onClick={() => setModal(null)}>开始查看<Icon name="arrow" size={16}/></button></>}</Modal>}
    {expandedDrone && <Modal title={`${expandedDrone.id} · 前向相机`} close={() => setExpanded(null)} wide><Camera drone={expandedDrone} index={drones.indexOf(expandedDrone)} seconds={seconds} demo={source === 'demo'} large stale={stale}/><p className="expanded-caption">{source === 'demo' ? '程序生成的演示画面 · 非真实相机采集' : 'HTTP 图像流 · 遥测状态不等于图像流状态'}<span>X {expandedDrone.position.x.toFixed(1)} · Y {expandedDrone.position.y.toFixed(1)} · Z {expandedDrone.position.z.toFixed(1)} m</span></p></Modal>}
  </div>;
}

function Modal({ title, close, children, wide = false }: { title: string; close: () => void; children: React.ReactNode; wide?: boolean }) {
  const ref = useRef<HTMLDialogElement>(null);
  useEffect(() => { const dialog = ref.current; dialog?.showModal(); return () => { dialog?.close(); }; }, []);
  return <dialog className={`modal ${wide ? 'wide-modal' : ''}`} ref={ref} onCancel={e => { e.preventDefault(); close(); }} onClick={e => { if (e.target === e.currentTarget) { const r = e.currentTarget.getBoundingClientRect(); if (e.clientX < r.left || e.clientX > r.right || e.clientY < r.top || e.clientY > r.bottom) close(); } }} aria-label={title}><div className="modal-heading"><h2>{title}</h2><button className="icon-button" aria-label="关闭弹窗" onClick={close}><Icon name="close"/></button></div>{children}</dialog>;
}

createRoot(document.getElementById('root')!).render(<React.StrictMode><App/></React.StrictMode>);
