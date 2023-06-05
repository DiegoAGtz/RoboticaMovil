export class WebSocketC {
  gateway: string;
  webSocket: WebSocket;
  isOpened: boolean;
  html: HTMLElement | null;

  constructor(gateway: string, htmlId: string) {
    this.isOpened = false;
    console.log("Trying to open a WebSocket connection...");
    this.gateway = gateway;
    this.webSocket = new WebSocket(gateway);
    this.webSocket.onopen = this.onOpen;
    this.webSocket.onclose = this.onClose;
    this.webSocket.onmessage = this.onMessage;
    this.html = document.getElementById(htmlId);
  }

  onOpen = (_: Event) => {
    console.log("Connection opened");
    this.isOpened = true;
  };

  onMessage = (event: MessageEvent) => {
    if (this.html)
      this.html.innerText = `Server says -> ${event.data}mm`;
  };

  onClose = (_: Event) => {
    console.log("Connection closed");
    this.isOpened = false;
  };

  sendMessage = (message: string) => {
    this.webSocket.send(message);
  };
}