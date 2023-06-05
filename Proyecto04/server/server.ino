#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char *ssid = "TP-Link_FA5C";
const char *password = "83898006";

const char *right_parameter= "right";
const char *left_parameter = "left";

AsyncWebServer server(80);
const int led = 13; // For see that it's on

const char index_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
  body {
  overflow: hidden;
  user-select: none;

  /* stuff that sass won't compile to */
  -webkit-touch-callout: none;
  -khtml-user-select: none;
}

canvas {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 0;
}
  </style>
</head>
<body class="grid grid-cols-1">
  <canvas id="canvas"></canvas>
  <script>
    const canvas = document.getElementById("canvas");
    const ctx = canvas.getContext("2d");
    const pi = Math.PI;
    
    // background gradient
    var gradient;
    function fixDpiResizeCanvas() {
      const dpr = window.devicePixelRatio || 1;
      canvas.style.width = window.innerWidth + "px";
      canvas.style.height = window.innerHeight + "px";
      canvas.width = window.innerWidth * dpr;
      canvas.height = window.innerHeight * dpr;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    
      // sorry for not using css
      gradient = ctx.createLinearGradient(0, canvas.height, canvas.width, 0);
      gradient.addColorStop(0, "#88d8ff");
      gradient.addColorStop(1, "#d899ff");
    }
    fixDpiResizeCanvas();
    window.addEventListener("resize", fixDpiResizeCanvas);
    
    //
    
    /////////////////////////
    //  JOYSTICK UPDATING  //
    /////////////////////////
    
    var positions = {
      // Here, fixed is the outer circle and inner is the small circle that moves
      fixedX: undefined,
      fixedY: undefined,
      innerX: undefined,
      innerY: undefined
    };
    
    var angle = undefined;
    
    function touchStart(x, y) {
    if (positions.fixedX || positions.fixedY) return;
    positions.fixedX = positions.innerX = x;
    positions.fixedY = positions.innerY = y;
    }
    
    function touchMove(x, y) {
      if (!(positions.fixedX || positions.fixedY)) return;
    
      positions.innerX = x;
      positions.innerY = y;
    
      angle = Math.atan2(
          positions.innerY - positions.fixedY,
          positions.innerX - positions.fixedX
          );
    
      // If inner circle is outside joystick radius, reduce it to the circumference
      if (!(
            (x - positions.fixedX) ** 2 +
            (y - positions.fixedY) ** 2 < 10000
           ))
      {
        positions.innerX = Math.round(Math.cos(angle) * 100 + positions.fixedX);
        positions.innerY = Math.round(Math.sin(angle) * 100 + positions.fixedY);
      }
    }
    
    function touchEndOrCancel() {
      positions.fixedX
        = positions.fixedY
        = positions.innerX
        = positions.innerY
        = angle
        = undefined;
    }
    
    canvas.addEventListener("touchstart", function(e) {
        touchStart(e.touches[0].clientX, e.touches[0].clientY);
        });
    
    canvas.addEventListener("touchmove", function(e) {
        touchMove(e.touches[0].clientX, e.touches[0].clientY)
        });
    
    canvas.addEventListener("touchend", touchEndOrCancel);
    canvas.addEventListener("touchcancel", touchEndOrCancel);
    
    // TODO: test mouse on pc
    canvas.addEventListener("mousedown", function (e) {
        touchStart(e.offsetX, e.offsetY);
        });
    
    canvas.addEventListener("mousemove", function (e) {
        touchMove(e.offsetX, e.offsetY);
        });
    
    canvas.addEventListener("mouseup", touchEndOrCancel);
    
    function renderLoop() {
      requestAnimationFrame(renderLoop);
    
    
      ctx.clearRect(0, 0, canvas.width, canvas.height);
    
      // Background gradient
      ctx.fillStyle = gradient;
      ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    
      // Invert Y axis and turn into positive
      var displayAngle = (-angle + 2*pi) % (2*pi);
    
      ctx.fillStyle = "#0008";
      if (!(positions.fixedX || positions.fixedY)) {
        ctx.fillText("Robótica Móvil | Proyecto 04", 20, 20);
        ctx.fillText("Toca la pantalla", 20, 60);
        return;
      };
    
      // Display data
      ctx.fillText(
          `Angle: ${Math.round((displayAngle * 180) / pi)} degrees (${
              Math.round(displayAngle * 100) / 100
              } radians)`,
          20,
          20
          );
    
      ctx.fillText(`Inner joystick: (${positions.innerX},${positions.innerY})`, 20, 80);
    
      // Draw joystick outer circle
      ctx.beginPath();
      ctx.fillStyle = "#0004";
      ctx.arc(positions.fixedX, positions.fixedY, 100, 0, 2 * pi);
      ctx.fill();
      ctx.closePath();
    
      // Draw inner circle
      ctx.beginPath();
      ctx.fillStyle = "#0008";
      ctx.arc(positions.innerX, positions.innerY, 30, 0, 2 * pi);
      ctx.fill();
      ctx.closePath();
    }
    
    renderLoop();
    
    ctx.font = "20px Helvetica";
    ctx.textBaseline = "top";
  </script>
</body>
</html>
)rawliteral";

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  Serial.begin(115200);

  // Wait for connection
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send_P(200, "text/html", index_html);  
      });

  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}
