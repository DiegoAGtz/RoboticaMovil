export class WebSocketC {
    constructor(gateway, htmlId) {
        this.onOpen = (_) => {
            console.log("Connection opened");
            this.isOpened = true;
        };
        this.onMessage = (event) => {
            console.log(event.data);
            if (this.html)
                this.html.innerText = `Server says -> ${event.data}mm`;
        };
        this.onClose = (_) => {
            console.log("Connection closed");
            this.isOpened = false;
        };
        this.sendMessage = (message) => {
            this.webSocket.send(message);
        };
        this.isOpened = false;
        console.log("Trying to open a WebSocket connection...");
        this.gateway = gateway;
        this.webSocket = new WebSocket(gateway);
        this.webSocket.onopen = this.onOpen;
        this.webSocket.onclose = this.onClose;
        this.webSocket.onmessage = this.onMessage;
        this.html = document.getElementById(htmlId);
    }
}
