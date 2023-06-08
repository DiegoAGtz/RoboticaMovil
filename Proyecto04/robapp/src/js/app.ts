import { Joystick } from "./joystick.js";
import { WebSocketC } from "./websocket.js";
import Alpine from "alpinejs";

const minVel = 140,
  maxVel = 255;
let px = 0,
  py = 0;

const joystick = new Joystick("stick1", 64, 8);
let webSocket: WebSocketC;

interface wsInterface {
  ip: string,
  alert: boolean;
  success: boolean;
  connect: () => void;
}
interface distanceInterface {
  value: number[],
  changeStyle: (value: number) => void;
  changeClasses: (value: number) => void;
}

window.Alpine = Alpine;

Alpine.store('ws', {
  ip: "",
  alert: true,
  success: false,
  connect(ip: string) {
    webSocket = new WebSocketC(`ws://${ip}/ws`, "webSocketMessage",wsDistance.value);
    wsStore.alert = true;
    setTimeout(() => {
      if (webSocket.webSocket.readyState !== 1) {
        wsStore.alert = false;
        webSocket.webSocket.close()
        wsStore.success = false;
      } else {
        wsStore.success = true;
      }
    }, 1500);

  }
});
const wsStore = (Alpine.store('ws') as unknown) as wsInterface;

Alpine.store('distance', {
  value: [0],
  changeStyle(value: number) {
    return "width:" + value.toString() + "%";
  },
  changeClasses(value: number) {
    if (value <= 50) {
      return 'bg-green-500';
    } else {
      let res = 100 - (value - 40);
      return `bg-gradient-to-r from-green-500 to-red-500 from-[${res}%]`;
    }
  }
});
const wsDistance = (Alpine.store('distance') as unknown) as distanceInterface;

Alpine.store('directions', {
  forward() { update(255, 255); wsSend('1,0,1,0,255,255'); },
  reverse() { update(255, 255); wsSend('0,1,0,1,255,255'); },
  left() { update(160, 255); wsSend('1,0,1,0,160,255'); },
  right() { update(255, 160); wsSend('1,0,1,0,255,160'); },
  stop() { update(140, 140); wsSend('1,0,1,0,140,140'); }
})

Alpine.start()

function wsSend(message: string) {
  if (webSocket?.isOpened) webSocket.sendMessage(message);
  else wsStore.alert = false;
}

function update(v1: number, v2: number): void {
  let pwmValues = document.getElementById("pwmValues");
  if (pwmValues && webSocket?.isOpened) pwmValues.innerText = `PWM enviados a los motores [140 - 255] => M1: ${v1}, M2: ${v2}`;
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
    in4 = 0;

  if (compare(x, y)) {
    if (-py >= 0) {
      in1++;
      in3++;
    } else {
      in2++;
      in4++;
    }

    let enab = [0, 0];
    let i = px > 0 ? 0 : 1;

    if (Math.abs(px) > Math.abs(py)) {
      enab[i] = Math.abs(px);
    } else {
      enab[i] = Math.abs(py);
    }

    enab[i == 0 ? 1 : 0] = Math.abs(py) - Math.abs(py / 2.0) * Math.abs(px);

    enab[0] = Math.floor(mapValues(enab[0], 0, 1, minVel, maxVel));
    enab[1] = Math.floor(mapValues(enab[1], 0, 1, minVel, maxVel));
    update(enab[0], enab[1]);
    wsSend(`${in1},${in2},${in3},${in4},${enab[0]},${enab[1]}`);
  }
}

loop();