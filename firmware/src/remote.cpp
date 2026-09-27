#include "remote.h"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

namespace remote {
namespace {

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Written from the async_tcp task, read from the control loop.
volatile float cmd_v = 0, cmd_w = 0;
volatile uint32_t cmd_ms = 0;
volatile bool arm_req = false, stop_req = false;

const char kPage[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Pendulum</title>
<style>
body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#eee;touch-action:none;
  display:flex;flex-direction:column;align-items:center;height:100vh;user-select:none;-webkit-user-select:none}
#st{margin:12px;font-size:15px;font-variant-numeric:tabular-nums;text-align:center}
#pad{width:min(80vw,60vh);height:min(80vw,60vh);border-radius:50%;background:#222;border:2px solid #444;
  position:relative;margin:8px}
#knob{width:30%;height:30%;border-radius:50%;background:#4a8;position:absolute;left:35%;top:35%}
.row{display:flex;gap:16px;margin:12px}
button{font-size:20px;padding:14px 28px;border-radius:12px;border:0;color:#fff}
#arm{background:#2a6}#stop{background:#c33}
</style></head><body>
<div id="st">connecting...</div>
<div id="pad"><div id="knob"></div></div>
<div class="row"><button id="arm">ARM</button><button id="stop">STOP</button></div>
<script>
const st=document.getElementById('st'),pad=document.getElementById('pad'),knob=document.getElementById('knob');
let ws,jx=0,jy=0,active=false;
function connect(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.onopen=()=>st.textContent='connected';
  ws.onclose=()=>{st.textContent='disconnected - retrying';setTimeout(connect,1000)};
  ws.onmessage=e=>{const p=e.data.split(',');
    if(p[0]==='s')st.textContent=p[1]+'  tilt '+p[2]+' deg  v '+p[4]+' m/s  '+p[3]+' V';};
}
connect();
function send(m){if(ws&&ws.readyState===1)ws.send(m)}
function setKnob(){knob.style.left=(35+jx*35)+'%';knob.style.top=(35-jy*35)+'%'}
function move(e){const r=pad.getBoundingClientRect(),t=e.touches?e.touches[0]:e;
  let x=(t.clientX-r.left)/r.width*2-1,y=-((t.clientY-r.top)/r.height*2-1);
  const n=Math.hypot(x,y);if(n>1){x/=n;y/=n}jx=x;jy=y;setKnob();e.preventDefault()}
function start(e){active=true;move(e)}
function end(e){active=false;jx=jy=0;setKnob();send('j,0,0');e&&e.preventDefault()}
pad.addEventListener('touchstart',start,{passive:false});pad.addEventListener('touchmove',move,{passive:false});
pad.addEventListener('touchend',end,{passive:false});pad.addEventListener('touchcancel',end,{passive:false});
pad.addEventListener('mousedown',start);window.addEventListener('mousemove',e=>{if(active)move(e)});
window.addEventListener('mouseup',()=>{if(active)end()});
setInterval(()=>{if(active)send('j,'+jy.toFixed(2)+','+jx.toFixed(2))},50);
document.getElementById('arm').onclick=()=>send('arm');
document.getElementById('stop').onclick=()=>{end();send('stop')};
</script></body></html>)HTML";

void onEvent(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_DISCONNECT) {
    cmd_v = cmd_w = 0;
    return;
  }
  if (type != WS_EVT_DATA) return;
  auto* info = static_cast<AwsFrameInfo*>(arg);
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT || len > 40) return;
  char buf[41];
  memcpy(buf, data, len);
  buf[len] = 0;
  if (buf[0] == 'j' && buf[1] == ',') {
    float v = 0, w = 0;
    if (sscanf(buf + 2, "%f,%f", &v, &w) == 2) {
      cmd_v = constrain(v, -1.0f, 1.0f);
      cmd_w = constrain(w, -1.0f, 1.0f);
      cmd_ms = millis();
    }
  } else if (!strcmp(buf, "arm")) {
    arm_req = true;
  } else if (!strcmp(buf, "stop")) {
    cmd_v = cmd_w = 0;
    stop_req = true;
  }
}

}  // namespace

void begin(const char* ssid, const char* password) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) { req->send(200, "text/html", kPage); });
  server.begin();
}

Cmd get(uint32_t now_ms, uint32_t timeout_ms) {
  Cmd c;
  if (now_ms - cmd_ms < timeout_ms) {
    c.v = cmd_v;
    c.w = cmd_w;
  }
  return c;
}

bool takeArm() {
  const bool r = arm_req;
  arm_req = false;
  return r;
}

bool takeStop() {
  const bool r = stop_req;
  stop_req = false;
  return r;
}

void publish(const char* state, float th_deg, float vin, float v_ref) {
  ws.cleanupClients();
  if (!ws.count()) return;
  char msg[64];
  snprintf(msg, sizeof(msg), "s,%s,%.1f,%.2f,%.2f", state, th_deg, vin, v_ref);
  ws.textAll(msg);
}

}  // namespace remote
