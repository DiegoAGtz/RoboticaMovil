import { Joystick } from "./joystick.js";
import { WebSocketC } from "./websocket.js";

const minVel = 140,
  maxVel = 255;
let px = 10,
  py = 10;

const joystick = new Joystick("stick1", 64, 8);
let webSocket = new WebSocketC("ws://192.168.193.103/ws", "webSocketMessage");

function update(v1: number, v2: number): void {
  let { x, y } = joystick.value;
  let angle = Math.atan2(-y, x);
  let joyValues = document.getElementById("joyValues");
  let pwmValues = document.getElementById("pwmValues");
  if (joyValues)
    joyValues.innerText = `Joystick values -> X: ${x}, Y: ${-y}, ANGLE: ${angle}`;
  if (pwmValues) pwmValues.innerText = `PWM values -> M1: ${v1}, M2: ${v2}`;
}

function compare(x: number, y: number): boolean {
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

function mapValues(
  val: number,
  minI: number,
  maxI: number,
  minO: number,
  maxO: number
): number {
  return ((val - minI) * (maxO - minO)) / (maxI - minI) + minO;
}

function loop() {
  requestAnimationFrame(loop);
  let { x, y } = joystick.value;
  let in1 = 0,
    in2 = 0,
    in3 = 0,
    in4 = 0,
    ena = 0,
    enb = 0;

  if (compare(x, y)) {
    if (-py >= 0) {
      in1++;
      in3++;
    } else {
      in2++;
      in4++;
    }

    let enab = [0, 0];    
    let i = px > 0 ? 0:1;
    
    if(Math.abs(px) > Math.abs(py)){
      enab[i] = Math.abs(px);
    }else{
      enab[i] = Math.abs(py);
    }

    enab[i == 0 ? 1:0] = Math.abs(py) - Math.abs(py/2.0)*Math.abs(px);

    enab[0] = Math.floor(mapValues(enab[0], 0, 1, minVel, maxVel));
    enab[1] = Math.floor(mapValues(enab[1], 0, 1, minVel, maxVel));

    let message  = `${in1},${in2},${in3},${in4},${enab[0]},${enab[1]}`;
    //webSocket.sendMessage(message);
    update(enab[0], enab[1]);
  }
}

loop();
