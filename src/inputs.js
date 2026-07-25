
import { clamp } from './utils.js'
import { dt } from './config.js'

// globals
let kbdState = { x: 0, y: 0 };
let keyDown = {}
export let keyPressed = {}
export { keyDown }

let mouse = { x: 0, y: 0 };
let mouseEnabled = true; // false = mouse does NOT control drone
export function setMouseEnabled(v) { mouseEnabled = v; }

// config
const TAU_KBD = 0.2;
const MOUSE_SENS = 0.05;
let mode = 2; // 1,2,3,4,5
let command = null;  // 'takeoff' | 'land' | null

// allow external triggers (e.g. HTML buttons) to set commands
export function setCommand(cmd) {
    command = cmd
}

// input listeners
window.addEventListener('keydown', (e) => {
    keyDown[e.key.toLowerCase()] = true
    if (!e.repeat) { keyPressed[e.key.toLowerCase()] = true }
    if (e.key === 'Tab') e.preventDefault(); // keep focus on canvas
});
window.addEventListener('keyup', (e) => {
    keyDown[e.key.toLowerCase()] = false
});

let mouseCaptured = false;
document.addEventListener("pointerlockchange", () => {
    mouseCaptured = document.pointerLockElement !== null
});

window.addEventListener('mousemove', e => {
    if (mouseCaptured) {
        mouse.x += e.movementX * MOUSE_SENS;
        mouse.y += e.movementY * MOUSE_SENS;
        mouse.x = clamp(mouse.x, -1, 1);
        mouse.y = clamp(mouse.y, -1, 1);
    }
});

window.addEventListener('keydown', e => {
    if (e.repeat) return;
    if (e.key === '1') mode = 1;
    else if (e.key === '2') mode = 2;
    else if (e.key === '3') mode = 3;
    else if (e.key === '4') mode = 4;
    else if (e.key === '5') mode = 5;
});

// drone control inputs
let gpBefore = new Array(100).fill(false)
export function readInputs() {

    let reset = false
    if (keyPressed['tab']) {
        reset = true
        keyPressed['tab'] = false
    }

    // consume one-shot commands
    let cmd = command
    command = null

    // takeoff / landing / navigation triggers
    if (keyPressed['t']) {
        cmd = 'takeoff'
        keyPressed['t'] = false
    }
    if (keyPressed['g']) {
        cmd = 'land'
        keyPressed['g'] = false
    }

    const gamepad = navigator.getGamepads()[0];
    const applyDeadzone = (v, dz = 0.15) => Math.abs(v) < dz ? 0 : v;

    let leftStickX = 0, leftStickY = 0, rightStickX = 0, rightStickY = 0;

    // gamepad
    if (gamepad) {
        leftStickX = applyDeadzone(gamepad.axes[0]);
        leftStickY = applyDeadzone(-gamepad.axes[1]);
        rightStickX = applyDeadzone(gamepad.axes[2]);
        rightStickY = applyDeadzone(-gamepad.axes[3]);
        if (gamepad.buttons[5].pressed && !gpBefore[5]) {
            reset = true
        }
        gpBefore = gamepad.buttons.map(b => b.pressed)
    }

    // wasd = left stick
    let targetX = (keyDown['d'] || keyDown['arrowright'] ? 1 : 0) - (keyDown['a'] || keyDown['arrowleft'] ? 1 : 0);
    let targetY = (keyDown['w'] || keyDown['arrowup'] ? 1 : 0) - (keyDown['s'] || keyDown['arrowdown'] ? 1 : 0);

    let q = dt / (TAU_KBD + dt);

    kbdState.x = (1 - q) * kbdState.x + q * targetX
    kbdState.y = (1 - q) * kbdState.y + q * targetY

    if (reset) {
        kbdState.x = 0;
        kbdState.y = 0;
        targetX = 0;
        targetY = 0;
    }

    leftStickX += kbdState.x;
    leftStickY += kbdState.y;

    // mouse = right stick (only when not in free‑cam)
    if (mouseEnabled) {
        rightStickX += mouse.x;
        rightStickY -= mouse.y;
        mouse.x = 0;
        mouse.y = 0;
    }

    leftStickX = clamp(leftStickX, -1, 1)
    leftStickY = clamp(leftStickY, -1, 1)
    rightStickX = clamp(rightStickX, -1, 1)
    rightStickY = clamp(rightStickY, -1, 1)

    let throttleInput, yawInput, pitchInput, rollInput, forwardInput = 0;

    switch (mode) {
        case 1:
            throttleInput = rightStickY;
            yawInput = leftStickX;
            pitchInput = -leftStickY;
            rollInput = rightStickX;
            break;
        case 2:
            throttleInput = leftStickY;
            yawInput = leftStickX;
            pitchInput = -rightStickY;
            rollInput = rightStickX;
            break;
        case 3:
            throttleInput = leftStickY;
            yawInput = rightStickX;
            pitchInput = -rightStickY;
            rollInput = leftStickX;
            break;
        case 4:
            throttleInput = rightStickY;
            yawInput = rightStickX;
            pitchInput = -leftStickY;
            rollInput = leftStickX;
            break;
        case 5:
            // ── W: forward · S: backward (horizontal translation) ──
            forwardInput = (keyDown['w'] ? 1 : 0) - (keyDown['s'] ? 1 : 0);
            // ── E: pitch forward (nose down) · Q: pitch backward (nose up) ──
            pitchInput = (keyDown['e'] ? 1 : 0) - (keyDown['q'] ? 1 : 0);

            // ── Yaw (偏航角 / 水平转向) ──
            // D → yaw right    A → yaw left
            yawInput = (keyDown['d'] ? 1 : 0) - (keyDown['a'] ? 1 : 0);

            // ── Roll (横滚角 / 左右倾斜) ──
            // X → roll right    Z → roll left
            rollInput = (keyDown['x'] ? 1 : 0) - (keyDown['z'] ? 1 : 0);

            // ── Throttle (升降) ──
            // R → climb (上升)    F → descend (下降)     neither → hover
            if (keyDown['r']) {
                throttleInput = 1.0;
            } else if (keyDown['f']) {
                throttleInput = -1.0;
            } else {
                throttleInput = 0.0;
            }
            break;

    }

    throttleInput = throttleInput * 0.5 + 0.5

    return { throttleInput, rollInput, pitchInput, yawInput, reset, mode, command: cmd, forwardInput };
}
