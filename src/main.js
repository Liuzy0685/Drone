
import { loadResources, HDRLoader } from './resources.js'
import { THREE } from './three.js'
import { initDebugRender, updateDebugRender } from './debug.js'
import { setFromRPYdeg } from './utils.js'
import { keyPressed } from './inputs.js'
import { loadConfig, dt } from './config.js'
import { createStats } from './ui.js'
import { createTerrain } from './terrain.js'
import { createDroneVisuals } from './dronevisuals.js'
import { initSound, updateSound } from './sound.js'
import { createCamera, createFreeCamera } from './camera.js'
import { createGui, onGuiChange } from './gui.js'
import { createRenderPipeline } from './renderer.js'
import { readInputs, setCommand, keyDown, setMouseEnabled } from './inputs.js'

window.THREE = THREE;

async function main() {

    // load config
    const config = await loadConfig()
    console.assert(config.version == 1.0)
    console.assert(config.aircraft.type == "quadcopter")

    // gui
    const gui = createGui(config)
    gui.hide()

    // scene
    const scene = new THREE.Scene();
    window.scene = scene
    scene.background = new THREE.Color(0x87ceeb);
    const origin = new THREE.AxesHelper(1);
    scene.add(origin);
    onGuiChange(gui, ["settings.debug"], (debug) => { origin.visible = debug }, true)
    const gVector = new THREE.Vector3(...config.map.gravity)
    THREE.Object3D.DEFAULT_UP = gVector.clone().multiplyScalar(-1).normalize()
    scene.visible = false

    // renderer
    const { render, canvas, stepMotionBlurCamera } = createRenderPipeline(config, gui, scene)

    // resources
    let {
        physicsWorker,
        droneModel,
        propWav,
        checkpointWav,
        terrainModel,
        bgMap,
        envMap,
        musicWav
    } = await loadResources(config, canvas)
    document.getElementById('battery').style.display = 'none'
    gui.show()

    // capture mouse
    canvas.addEventListener("click", e => {
        if (e.button !== 0) return;

        if (document.pointerLockElement === null) {
            canvas.requestPointerLock();
        } else {
            document.exitPointerLock();
        }
    });

    // takeoff / landing buttons
    document.getElementById('takeoffBtn').addEventListener('click', () => {
        setCommand('takeoff');
    });
    document.getElementById('landBtn').addEventListener('click', () => {
        setCommand('land');
    });

    // ── navigation panel ──
    const navPanel = document.getElementById('navPanel');
    const navStartBtn = document.getElementById('navStartBtn');
    const navStopBtn = document.getElementById('navStopBtn');
    const navStatus = document.getElementById('navStatus');

    // N key toggles nav panel visibility, auto-fills start pos with current drone position
    window.addEventListener('keydown', (e) => {
        if (e.key === 'n' || e.key === 'N') {
            if (!e.repeat) {
                const showing = navPanel.style.display === 'block';
                navPanel.style.display = showing ? 'none' : 'block';
                if (!showing) {
                    // auto-fill start position with current drone location
                    document.getElementById('navStartX').value = latestDronePos.x.toFixed(1);
                    document.getElementById('navStartY').value = latestDronePos.y.toFixed(1);
                    document.getElementById('navStartZ').value = latestDronePos.z.toFixed(1);
                }
            }
        }
    });

    navStartBtn.addEventListener('click', () => {
        const sx = parseFloat(document.getElementById('navStartX').value);
        const sy = parseFloat(document.getElementById('navStartY').value);
        const sz = parseFloat(document.getElementById('navStartZ').value);
        const ex = parseFloat(document.getElementById('navEndX').value);
        const ey = parseFloat(document.getElementById('navEndY').value);
        const ez = parseFloat(document.getElementById('navEndZ').value);
        physicsWorker.postMessage({
            navCommand: 'start',
            startPos: [sx, sy, sz],
            endPos: [ex, ey, ez],
        });
        navStatus.innerText = '状态: 导航中...';
        navStatus.style.color = '#0f0';
    });

    navStopBtn.addEventListener('click', () => {
        physicsWorker.postMessage({ navCommand: 'stop' });
        navStatus.innerText = '状态: 已停止';
        navStatus.style.color = '#888';
        setCommand('toggleNav');  // also toggle off the old nav system
    });

    // raw mouse delta for free camera — only when pointer is locked (click canvas)
    let rawMouseDX = 0, rawMouseDY = 0;
    window.addEventListener('mousemove', (e) => {
        if (document.pointerLockElement !== null) {
            rawMouseDX += e.movementX;
            rawMouseDY += e.movementY;
        }
    });

    // stats
    const { engineStats, graphicsStats } = createStats()
    if (config.map.mission.type === "race" || config.map.mission.type === "navigation") {
        document.getElementById('timer').style.visibility = "visible"
    }

    // lighting
    const hdrLoader = new HDRLoader();

    envMap.mapping = THREE.EquirectangularReflectionMapping;
    scene.environment = envMap;
        onGuiChange(gui, ["background.environmentMap.path"], (path) => {
        hdrLoader.load(path, (hdrMap) => { scene.environment = hdrMap; })
    })
    onGuiChange(gui, ["background.environmentMap.rollPitchYaw"], () => {
        setFromRPYdeg(scene.environmentRotation, config.background.environmentMap.rollPitchYaw);
    }, true)
    onGuiChange(gui, ["background.environmentMap.intensity"], (intensity) => {
        scene.environmentIntensity = intensity
    }, true)

    bgMap.mapping = THREE.EquirectangularReflectionMapping;
    scene.background = bgMap;
        onGuiChange(gui, ["background.backgroundMap.path"], (path) => {
        hdrLoader.load(path, (hdrMap) => { scene.background = hdrMap; })
    })
    onGuiChange(gui, ["background.backgroundMap.rollPitchYaw"], () => {
        setFromRPYdeg(scene.backgroundRotation, config.background.backgroundMap.rollPitchYaw);
    }, true)
    onGuiChange(gui, ["background.backgroundMap.intensity"], (intensity) => {
        scene.backgroundIntensity = intensity
    }, true)

    // drone
    const drone = createDroneVisuals(droneModel, scene, config, gui)

    // cameras
    const fpv = createCamera(config.aircraft.camera.firstPerson)
    scene.add(fpv.mount)
    const tpv = createCamera(config.aircraft.camera.thirdPerson)
    scene.add(tpv.mount)
    const freeCam = createFreeCamera()

    // 3-way camera toggle: FPV → TPV → Free → FPV …
    const allCams = [fpv.camera, tpv.camera, freeCam.camera];
    let camIndex = config.aircraft.camera.preselected === "firstPerson" ? 0 : 1;
    let selectedCamera = allCams[camIndex];
    let latestDronePos = new THREE.Vector3(); // for free-cam snap

    onGuiChange(gui, ["aircraft.camera.firstPerson.fieldOfView"], (fov) => { fpv.resize() })
    onGuiChange(gui, ["aircraft.camera.firstPerson.fishEyeStrength"], (s) => { fpv.camera.userData.fishEyeStrength = s })
    onGuiChange(gui, ["aircraft.camera.firstPerson.shutterSpeed"], (s) => { fpv.camera.userData.exposure = 1 / s })
    onGuiChange(gui, ["aircraft.camera.thirdPerson.fieldOfView"], (fov) => { tpv.resize() })
    onGuiChange(gui, ["aircraft.camera.thirdPerson.fishEyeStrength"], (s) => { tpv.camera.userData.fishEyeStrength = s })
    onGuiChange(gui, ["aircraft.camera.thirdPerson.shutterSpeed"], (s) => { tpv.camera.userData.exposure = 1 / s })

    // sound
    const { propSounds, checkpointSound, music } = initSound(config, gui, tpv.camera, drone.node, propWav, checkpointWav, musicWav)

    // physics world
    let finished = false
    physicsWorker.postMessage({ "config": config })
    physicsWorker.addEventListener("message", (e) => {
        if (e.data.type === "step") {
            // update flight status display
            const phaseLabels = {
                'ground': '⏹ 地面 GROUND',
                'takingOff': '▲ 起飞中 TAKING OFF',
                'cruising': '✈ 飞行中 CRUISING',
                'landing': '▼ 降落中 LANDING',
            };
            let statusText = phaseLabels[e.data.flightPhase] || '';
            if (e.data.flightPhase === 'cruising' && e.data.navActive) {
                statusText += e.data.navReached ? ' | ✅ 已到达' : ' | 🎯 导航中';
            }
            document.getElementById('flightStatus').innerText = statusText;

            // mode indicator
            document.getElementById('modeIndicator').style.display = 'block';

            scene.visible = true

            // collision warning
            document.getElementById('collisionWarning').style.display =
                e.data.isColliding ? 'block' : 'none';

            latestDronePos.fromArray(e.data.drone.xyz);
            drone.updatePose(e.data.drone.xyz, e.data.drone.qxyzw)

            // update telemetry in key panel
            const tm = e.data.telemetry;
            if (tm) {
                const dp = latestDronePos;
                document.getElementById('tX').innerText = dp.x.toFixed(1);
                document.getElementById('tY').innerText = dp.y.toFixed(1);
                document.getElementById('tZ').innerText = dp.z.toFixed(1);
                document.getElementById('tSpeed').innerText = tm.speed;
                document.getElementById('tAlt').innerText = tm.altitude;
                document.getElementById('tRoll').innerText = tm.roll;
                document.getElementById('tPitch').innerText = tm.pitch;
                document.getElementById('tYaw').innerText = tm.yaw;
                const d = tm.distances;
                document.getElementById('dFront').innerText = d['前'];
                document.getElementById('dBack').innerText = d['后'];
                document.getElementById('dLeft').innerText = d['左'];
                document.getElementById('dRight').innerText = d['右'];
                document.getElementById('dUp').innerText = d['上'];
                document.getElementById('dDown').innerText = d['下'];
            }
            fpv.mount.position.fromArray(e.data.fpvCamera.xyz)
            fpv.mount.quaternion.fromArray(e.data.fpvCamera.qxyzw)
            tpv.mount.position.fromArray(e.data.tpvCamera.xyz)
            tpv.mount.quaternion.fromArray(e.data.tpvCamera.qxyzw)
            fpv.mount.updateMatrixWorld()
            tpv.mount.updateMatrixWorld()
            selectedCamera.updateProjectionMatrix()
            selectedCamera.updateMatrixWorld()
            stepMotionBlurCamera(selectedCamera)
            let t = e.data.ingameTime
            let mins = Math.floor(t / 60)
            let secs = t - mins * 60
            if (!finished) {
                document.getElementById('timer').innerText = `${mins.toString().padStart(2, '0')}:${secs.toFixed(2).padStart(5, '0')}`.replace('.', "'")
            }
            // navigation completion
            if (e.data.navReached && !finished) {
                finished = true;
                document.getElementById('timer').style.color = "lime";
                document.getElementById('navStatus').innerText = '状态: ✅ 已到达';
                document.getElementById('navStatus').style.color = '#0f0';
            }
            fixedDebugLines.visible = e.data.debug.isActive
            dynamicDebugLines.visible = e.data.debug.isActive
            if (e.data.debug.isActive) {
                if (e.data.debug.fixedVertices) {
                    updateDebugRender(fixedDebugLines, e.data.debug.fixedVertices, e.data.debug.fixedColors)
                }
                updateDebugRender(dynamicDebugLines, e.data.debug.dynamicVertices, e.data.debug.dynamicColors)
            }
            engineStats.update()
        }
        else if (e.data.type === "initCheckpoints") {
            setActiveCheckpoints(e.data.active)
        }
        else if (e.data.type === "checkpoint") {
            setActiveCheckpoints(e.data.active)
            finished = e.data.finished
            if (finished) {
                document.getElementById('timer').style.color = "lime"
            }
            checkpointSound.play()
        }
    })
    onGuiChange(gui, [null], () => {
        physicsWorker.postMessage({ "config": config })
    })

    // debug
    const fixedDebugLines = initDebugRender(scene)
    const dynamicDebugLines = initDebugRender(scene)

    // trrain
    const { terrainObject, terrainMeshData, setActiveCheckpoints } = createTerrain(terrainModel, config, scene)
    physicsWorker.postMessage({ "terrain": terrainMeshData }, terrainMeshData.flatMap(({ vertices, faces }) => [vertices, faces]))

    // run graphics
    function animate() {
        requestAnimationFrame(animate);
        drone.updateAnimation()

        if (keyPressed[' ']) {
            keyPressed[' '] = false
            camIndex = (camIndex + 1) % 3;
            selectedCamera = allCams[camIndex];
            if (camIndex === 2) {
                // snap free cam near current drone position
                freeCam.snapTo(latestDronePos);
            }
        }

        // mouse only rotates view in free‑cam; in FPV / TPV it is disabled
        setMouseEnabled(camIndex === 2);

        const inputs = readInputs()

        // Free camera: override drone inputs, update camera position
        if (camIndex === 2) {
            // send neutral (hover) to drone so it stays aloft,
            // but still forward commands (takeoff/land/toggleNav/reset)
            physicsWorker.postMessage({
                inputs: { throttleInput: 0.5, rollInput: 0, pitchInput: 0, yawInput: 0, reset: inputs.reset, mode: inputs.mode, command: inputs.command },
                debug: config.settings.debug,
            });
            // update free camera from keys + mouse (always, no pointer lock needed)
            freeCam.update(Math.min(1/60, 0.05), keyDown, latestDronePos, rawMouseDX, rawMouseDY);
        } else {
            physicsWorker.postMessage({ inputs, debug: config.settings.debug });
        }
        rawMouseDX = 0; rawMouseDY = 0;

        // update mode indicator
        const modeNames = { 1: 'Mode 1', 2: 'Mode 2', 3: 'Mode 3', 4: 'Mode 4', 5: 'Mode 5 (键盘)' };
        const camNames = ['FPV', '第三视角', '自由视角'];
        document.getElementById('modeIndicator').innerText =
            (modeNames[inputs.mode] || 'Mode ?') + ' | ' + (camNames[camIndex] || '');

        updateSound(config, propSounds, inputs)

        render(selectedCamera)
        graphicsStats.update()
    }

    animate();

}
main()
