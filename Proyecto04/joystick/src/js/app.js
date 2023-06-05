import { Joystick } from "./joystick.js";
import { WebSocketC } from "./websocket.js";
const minVel = 140, maxVel = 255;
let px = 10, py = 10;
const joystick = new Joystick("stick1", 64, 8);
let webSocket = new WebSocketC("ws://192.168.0.126/ws", "webSocketMessage");
function update(v1, v2) {
    let { x, y } = joystick.value;
    let angle = Math.atan2(-y, x);
    let joyValues = document.getElementById("joyValues");
    let pwmValues = document.getElementById("pwmValues");
    if (joyValues)
        joyValues.innerText = `Joystick values -> X: ${x}, Y: ${-y}, ANGLE: ${angle}`;
    if (pwmValues)
        pwmValues.innerText = `PWM values -> M1: ${v1}, M2: ${v2}`;
}
function compare(x, y) {
    let change = false;
    if (Math.abs(px - x) > 0.1) {
        px = x;
        change = true;
    }
    if (Math.abs(py - y) > 0.1) {
        py = y;
        change = true;
    }
    return change;
}
function mapValues(val, minI, maxI, minO, maxO) {
    return ((val - minI) * (maxO - minO)) / (maxI - minI) + minO;
}
function loop() {
    requestAnimationFrame(loop);
    let { x, y } = joystick.value;
    let in1 = 0, in2 = 0, in3 = 0, in4 = 0, ena = 0, enb = 0;
    // if (webSocket.isOpened && compare(x, y)) {
    if (compare(x, y)) {
        if (-py >= 0) {
            in1++;
            in3++;
        }
        else {
            in2++;
            in4++;
        }
        ena = minVel;
        enb = minVel;
        if (px > 0)
            ena = mapValues(px, 0, 1, minVel, maxVel);
        else if (px < 0)
            enb = mapValues(px, -1, 0, maxVel, minVel);
        if (py == 0) {
            ena += minVel;
            enb += minVel;
        }
        else if (py > 0) {
            ena += mapValues(py, 0, 1, minVel, maxVel);
            enb += mapValues(py, 0, 1, minVel, maxVel);
        }
        else {
            ena += mapValues(py, -1, 0, maxVel, minVel);
            enb += mapValues(py, -1, 0, maxVel, minVel);
        }
        ena = Math.floor(mapValues(ena, minVel * 2, maxVel * 2 - 50, minVel, maxVel));
        enb = Math.floor(mapValues(enb, minVel * 2, maxVel * 2 - 50, minVel, maxVel));
        let message = `${in1},${in2},${in3},${in4},${ena},${enb}`;
        console.log(message);
        // webSocket.sendMessage(message);
        update(ena, enb);
    }
}
loop();
