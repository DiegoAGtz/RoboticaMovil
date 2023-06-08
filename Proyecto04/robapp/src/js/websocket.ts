export class WebSocketC {
  gateway: string;
  webSocket: WebSocket;
  isOpened: boolean;
  html: HTMLElement | null;
  distance: number[];

  constructor(gateway: string, htmlId: string, distance: number[]) {
    this.isOpened = false;
    console.log("Trying to open a WebSocket connection...");
    this.gateway = gateway;
    this.webSocket = new WebSocket(gateway);
    this.webSocket.onopen = this.onOpen
    this.webSocket.onclose = this.onClose;
    this.webSocket.onmessage = this.onMessage;
    this.html = document.getElementById(htmlId);
    this.distance = distance;
  }

  onOpen = (_: Event) => {
    console.log("Connection opened");
    this.isOpened = true;
  };

  onMessage = (event: MessageEvent) => {
    let message = event.data;
    let distance = 1000;
    if (Number(message) < 1000) distance = Number(message);
    this.distance[0] = this.mapValues(distance, 0, 1000, 100, 0);
    if (this.html)
      this.html.innerText = `Distancia detectada al objeto => ${(distance >= 1000) ? 'Fuera de rango' : distance.toString().concat(' mm')}`;
  };

  onClose = (_: Event) => {
    console.log("Connection closed");
    this.isOpened = false;
  };

  sendMessage = (message: string) => {
    this.webSocket.send(message);
  };

  mapValues = (
    val: number,
    minI: number,
    maxI: number,
    minO: number,
    maxO: number
  ): number => {
    return ((val - minI) * (maxO - minO)) / (maxI - minI) + minO;
  }

}