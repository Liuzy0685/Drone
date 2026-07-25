
import GUI from 'three/addons/libs/lil-gui.module.min.js';

let ignore = [
    "version",
    "aircraft.type",
]

const zh = {
    settings: '设置', debug: '调试线框', antiAlias: '抗锯齿',
    composerResolutionScale: '合成分辨率', rendererResolutionScale: '渲染分辨率',
    aircraft: '飞机参数', wheelbase: '轴距', mass: '质量', propSize: '桨叶尺寸',
    maxCombinedThrust: '最大推力', stabilization: '自稳强度', angleLimit: '角度限制',
    maxRollRate: '最大横滚角速度', maxPitchRate: '最大俯仰角速度', maxYawRate: '最大偏航角速度',
    rollTimeConstant: '横滚响应', pitchTimeConstant: '俯仰响应', yawTimeConstant: '偏航响应',
    dragForceOverSpeed: '线性阻力', dragForceOverSpeedSquared: '平方阻力',
    model: '模型', path: '路径', position: '位置', rollPitchYaw: '姿态角', scale: '缩放',
    boundingBox: '碰撞盒', size: '尺寸',
    propSound: '桨叶声音', recordingFrequency: '录音频率', volume: '音量',
    numBlades: '桨叶数', maxThrustRPM: '最大转速', tiltDeltaRPM: '倾斜转速差', oscillateRPM: '波动转速',
    camera: '相机', firstPerson: '第一视角', thirdPerson: '第三视角', preselected: '默认视角',
    fieldOfView: '视场角', fishEyeStrength: '鱼眼强度', shutterSpeed: '快门速度', poseTimeConstant: '跟随平滑',
    map: '地图', spawn: '出生点', gravity: '重力', timeScale: '时间倍率', mission: '任务',
    checkpointScale: '检查点缩放', checkpointSound: '检查点音效',
    background: '背景', backgroundMap: '天空盒', environmentMap: '环境光照',
    intensity: '强度', music: '音乐', type: '类型',
    autoFlight: '自动飞行', takeoffHoverAltitude: '起飞悬停高度',
};

function getController(gui, path) {
    const parts = path.split('.');
    const prop = parts.pop();
    let folder = gui;
    for (const p of parts) {
        const zhP = zh[p] || p;
        folder = folder.folders.find(f => f._title === p || f._title === zhP);
        if (!folder) return null;
    }
    let field = folder.controllers.find(c => c._name === prop);
    if (field) return field;
    const zhProp = zh[prop] || prop;
    let sub = folder.folders.find(f => f._title === prop || f._title === zhProp);
    if (sub) return sub;
    return null;
}

export function createGui(config) {
    const gui = new GUI({ closeFolders: true });

    function walk(parent, obj) {
        for (const [key, val] of Object.entries(obj)) {
            const label = zh[key] || key;
            switch (typeof val) {
                case 'number': parent.add(obj, key).name(label); break;
                case 'string': parent.add(obj, key).name(label); break;
                case 'boolean': parent.add(obj, key).name(label); break;
                case 'object': {
                    if (val === null) continue;
                    if (Array.isArray(val)) continue;  // skip arrays in GUI
                    const f = parent.addFolder(label);
                    walk(f, val);
                    f.close();
                    break;
                }
            }
        }
    }
    walk(gui, config);

    for (const path of ignore) {
        const ctrl = getController(gui, path);
        if (ctrl) ctrl.destroy();
    }
    return gui;
}

const _guiChangeCallbacks = new WeakMap();

export function onGuiChange(gui, paths, callback, immediate = false) {
    for (const path of paths) {
        const controller = path ? getController(gui, path) : gui;
        if (!controller) return;
        let list = _guiChangeCallbacks.get(controller);
        if (!list) {
            list = [];
            _guiChangeCallbacks.set(controller, list);
            controller.onFinishChange(v => { for (const cb of list) cb(v); });
        }
        list.push(callback);
        if (immediate) callback(controller.getValue ? controller.getValue() : null);
    }
}
