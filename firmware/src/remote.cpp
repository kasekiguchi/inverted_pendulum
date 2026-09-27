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
QueueHandle_t queue = nullptr;  // Msg from the page to the control loop

LogSource log_src{};
volatile bool dl_active = false;

const char kPage[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Pendulum</title>
<style>
body{margin:0;font-family:system-ui,sans-serif;background:#111;color:#eee;touch-action:manipulation;
  display:flex;flex-direction:column;align-items:center;min-height:100vh;user-select:none;-webkit-user-select:none}
#st{margin:8px;font-size:14px;font-variant-numeric:tabular-nums;text-align:center}
#warn{font-size:20px;font-weight:bold;border-radius:8px;padding:0 12px}
#warn.low{background:#d80}#warn.cut{background:#c00}
#tabs{display:flex;gap:4px;margin:4px}
#tabs button{font-size:16px;padding:8px 18px;border-radius:8px;border:0;background:#333;color:#ccc}
#tabs button.on{background:#4a8;color:#fff}
section{display:none;width:100%;max-width:520px;flex-direction:column;align-items:center}
section.on{display:flex}
#pad{width:min(80vw,55vh);height:min(80vw,55vh);border-radius:50%;background:#222;border:2px solid #444;
  position:relative;margin:8px;touch-action:none}
#knob{width:30%;height:30%;border-radius:50%;background:#4a8;position:absolute;left:35%;top:35%}
.row{display:flex;gap:12px;margin:10px;flex-wrap:wrap;justify-content:center}
.big{font-size:20px;padding:14px 28px;border-radius:12px;border:0;color:#fff}
#arm{background:#2a6}#stop{background:#c33}
.btn{font-size:16px;padding:10px 18px;border-radius:10px;border:0;background:#456;color:#fff;text-decoration:none}
#rec.on{background:#c33}
canvas{width:96%;height:140px;background:#1a1a1a;border-radius:6px}
.p{width:94%;margin:6px 0}
.p .h{display:flex;justify-content:space-between;align-items:center;font-size:14px;gap:6px}
.p input[type=range]{width:100%}
.ed{display:flex;align-items:center;gap:4px}
.ed input{width:92px;font-size:16px;background:#222;color:#eee;border:1px solid #555;border-radius:6px;padding:4px;text-align:right}
.ed button{width:34px;height:32px;font-size:18px;border:0;border-radius:6px;background:#345;color:#fff}
.x{color:#999;font-size:12px;min-width:44px;text-align:right}
#msg{min-height:1.2em;color:#8c8;font-size:14px}
</style></head><body>
<div id="st">connecting...</div>
<div id="warn"></div>
<div id="tabs"><button data-t="drive" class="on">Drive</button><button data-t="tune">Tune</button><button data-t="log">Log</button></div>
<section id="drive" class="on"><div id="pad"><div id="knob"></div></div></section>
<section id="tune">
  <canvas id="plot" width="480" height="140"></canvas>
  <div style="font-size:12px;color:#999">tilt [deg] (green, &plusmn;10) / command u [rad/s] (orange, &plusmn;30)</div>
  <div id="params" style="width:100%;display:flex;flex-direction:column;align-items:center"></div>
  <div class="row"><button class="btn" id="save">SAVE</button><button class="btn" id="revert">REVERT</button></div>
</section>
<section id="log">
  <div id="recinfo" style="margin:10px">-</div>
  <div class="row"><button class="btn" id="rec">REC</button><button class="btn" id="mark">MARK</button>
  <a class="btn" id="dl" href="/log.csv" download="pendulum_log.csv">Download CSV</a></div>
  <div style="font-size:13px;color:#999;padding:0 16px">Without REC, runs (ARM to stop) are recorded automatically.
  REC records continuously from now. The recorder keeps the last 60 s. MARK (and every tuning change) sets a marker.</div>
</section>
<div id="msg"></div>
<div class="row"><button class="big" id="arm">ARM</button><button class="big" id="stop">STOP</button></div>
<script>
const $=id=>document.getElementById(id);
const st=$('st'),pad=$('pad'),knob=$('knob'),msg=$('msg');
let ws,jx=0,jy=0,active=false;
// Tuning parameters. Each has a value (cur) edited by a slider, a number box or
// -/+ buttons. Slider: rel = 0..2x the saved value, off = saved +-range,
// otherwise absolute. f = step of the -/+ buttons (rel: 1 % of the saved value).
const P=[{n:'K1 tilt',k:'rel'},{n:'K2 wheel angle',k:'rel'},{n:'K3 tilt rate',k:'rel'},{n:'K4 wheel speed',k:'rel'},
 {n:'TRIM [deg]',k:'off',min:-3,max:3,f:0.05,d:2},{n:'top speed [m/s]',min:0.1,max:1.2,f:0.05,d:2},
 {n:'accel [m/s2]',min:0.2,max:2,f:0.05,d:2},{n:'jerk [m/s3]',min:0.5,max:10,f:0.1,d:1},
 {n:'lean FF [deg/(m/s2)]',min:0,max:25,f:0.1,d:1},{n:'turn [rad/s]',min:-8,max:8,f:0.1,d:1}];
const base=new Array(P.length).fill(0),cur=new Array(P.length).fill(0),dirty=new Set();
const clamp=(v,a,b)=>Math.min(b,Math.max(a,v));
P.forEach((p,i)=>{const d=document.createElement('div');d.className='p';
  const rel=p.k==='rel';
  d.innerHTML='<div class="h"><span>'+p.n+'</span><span class="ed"><span class="x" id="x'+i+'"></span>'
   +'<button id="m'+i+'">&minus;</button><input type="number" inputmode="decimal" step="any" id="n'+i+'">'
   +'<button id="q'+i+'">+</button></span></div><input type="range" id="r'+i+'" min="'
   +(rel?0:p.min)+'" max="'+(rel?2:p.max)+'" step="'+(rel?0.01:'any')+'">';
  $('params').appendChild(d);
  $('r'+i).addEventListener('input',()=>{const r=parseFloat($('r'+i).value);
    cur[i]=rel?base[i]*r:p.k==='off'?base[i]+r:r;sync(i,'r')});
  $('n'+i).addEventListener('change',()=>{const v=parseFloat($('n'+i).value);
    // a gain must keep the sign of the saved one (the robot enforces it too): reject, show the current value
    if(isFinite(v)&&!(rel&&base[i]*v<0)){cur[i]=v;sync(i,'n')}else sync(i,undefined,true)});
  $('m'+i).onclick=()=>{cur[i]-=stepOf(i);sync(i)};
  $('q'+i).onclick=()=>{cur[i]+=stepOf(i);sync(i)};});
function stepOf(i){const p=P[i];return p.k==='rel'?(Math.abs(base[i])*0.01||0.001):p.f}
function digits(i){const p=P[i];if(p.k!=='rel')return p.d;const a=Math.abs(base[i]);return a>=10?2:a>=1?3:4}
// Refresh the widgets from cur[i] (except the one being edited) and queue it for sending.
function sync(i,from,quiet){const p=P[i];
  if(p.k==='rel'&&base[i]&&cur[i]*base[i]<0){cur[i]=0;from=undefined}  // -/+ stepped past zero
  if(from!=='r')$('r'+i).value=p.k==='rel'?clamp(base[i]?cur[i]/base[i]:1,0,2):p.k==='off'?clamp(cur[i]-base[i],p.min,p.max):clamp(cur[i],p.min,p.max);
  if(from!=='n')$('n'+i).value=cur[i].toFixed(digits(i));
  $('x'+i).textContent=p.k==='rel'&&base[i]?'x'+(cur[i]/base[i]).toFixed(2):p.k==='off'?(cur[i]-base[i]>=0?'+':'')+(cur[i]-base[i]).toFixed(2):'';
  if(!quiet)dirty.add(i)}
setInterval(()=>{dirty.forEach(i=>send('set,'+i+','+cur[i].toPrecision(6)));dirty.clear()},100);
function setParams(a){a.forEach((v,i)=>{if(i>=P.length)return;base[i]=cur[i]=v;sync(i,undefined,true)});}
// plot
const cv=$('plot'),cx=cv.getContext('2d'),TH=[],U=[];
function plot(){const w=cv.width,h=cv.height;cx.clearRect(0,0,w,h);cx.strokeStyle='#333';cx.beginPath();cx.moveTo(0,h/2);cx.lineTo(w,h/2);cx.stroke();
  [[TH,10,'#4c8'],[U,30,'#e93']].forEach(([a,s,c])=>{cx.strokeStyle=c;cx.beginPath();
    a.forEach((y,k)=>{const X=k/99*w,Y=h/2-y/s*h/2;k?cx.lineTo(X,Y):cx.moveTo(X,Y)});cx.stroke();});}
function connect(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.onopen=()=>{st.textContent='connected';send('getp')};
  ws.onclose=()=>{st.textContent='disconnected - retrying';setTimeout(connect,1000)};
  ws.onmessage=e=>{const p=e.data.split(',');
    if(p[0]==='p'){setParams(p.slice(1).map(parseFloat));return}
    if(p[0]==='m'){msg.textContent=p.slice(1).join(',');setTimeout(()=>msg.textContent='',3000);return}
    if(p[0]!=='s')return;
    st.textContent=p[1]+'  tilt '+p[2]+' deg  v '+p[4]+' m/s  '+p[3]+' V';
    const w=p[5]||'',el=$('warn');
    if(w!==el.textContent&&w&&navigator.vibrate)navigator.vibrate(300);
    el.textContent=w;el.className=w.includes('STOP')?'cut':(w?'low':'');
    TH.push(parseFloat(p[2]));U.push(parseFloat(p[6]));if(TH.length>100){TH.shift();U.shift()}
    if($('tune').classList.contains('on'))plot();
    const recAll=p[8]==='1';$('rec').classList.toggle('on',recAll);$('rec').textContent=recAll?'STOP REC':'REC';
    $('recinfo').textContent=(recAll?'recording continuously':'recording runs only')+' - '+p[7]+' s in memory';};
}
connect();
function send(m){if(ws&&ws.readyState===1)ws.send(m)}
// tabs
document.querySelectorAll('#tabs button').forEach(b=>b.onclick=()=>{
  document.querySelectorAll('#tabs button').forEach(x=>x.classList.toggle('on',x===b));
  document.querySelectorAll('section').forEach(s=>s.classList.toggle('on',s.id===b.dataset.t));});
// joystick
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
$('arm').onclick=()=>send('arm');
$('stop').onclick=()=>{end();send('stop')};
$('save').onclick=()=>send('save');
$('revert').onclick=()=>send('revert');
$('rec').onclick=()=>send($('rec').classList.contains('on')?'recstop':'recstart');
$('mark').onclick=()=>send('mark');
</script></body></html>)HTML";

void post(Action a, uint8_t param = 0, float value = 0) {
  Msg m;
  m.action = a;
  m.param = param;
  m.value = value;
  if (queue) xQueueSend(queue, &m, 0);
}

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
  } else if (!strncmp(buf, "set,", 4)) {
    int id = -1;
    float v = 0;
    if (sscanf(buf + 4, "%d,%f", &id, &v) == 2 && id >= 0 && id < kParamCount && isfinite(v)) {
      post(Action::kSet, id, v);
    }
  } else if (!strcmp(buf, "arm")) {
    arm_req = true;
  } else if (!strcmp(buf, "stop")) {
    cmd_v = cmd_w = 0;
    stop_req = true;
  } else if (!strcmp(buf, "save")) {
    post(Action::kSave);
  } else if (!strcmp(buf, "revert")) {
    post(Action::kRevert);
  } else if (!strcmp(buf, "getp")) {
    post(Action::kParamsRequest);
  } else if (!strcmp(buf, "recstart")) {
    post(Action::kRecStart);
  } else if (!strcmp(buf, "recstop")) {
    post(Action::kRecStop);
  } else if (!strcmp(buf, "mark")) {
    post(Action::kMark);
  }
}

// /log.csv streaming: header, then one row per recorder sample. A row can be
// split across chunks, so the current row is staged in `line`.
struct Download {
  size_t next = 0, total = 0;  // next row index (0 = header), rows in the recorder
  char line[192];
  size_t len = 0, off = 0;
} dl;

void endDownload() {
  if (!dl_active) return;
  dl_active = false;
  if (log_src.freeze) log_src.freeze(false);
}

size_t fillChunk(uint8_t* buf, size_t max_len, size_t) {
  size_t n = 0;
  while (n < max_len) {
    if (dl.off >= dl.len) {
      if (dl.next > dl.total) break;
      if (dl.next == 0) {
        dl.len = snprintf(dl.line, sizeof(dl.line), "%s\n", log_src.header);
      } else {
        dl.len = log_src.line(dl.next - 1, dl.line, sizeof(dl.line));
      }
      dl.off = 0;
      ++dl.next;
    }
    const size_t k = min(max_len - n, dl.len - dl.off);
    memcpy(buf + n, dl.line + dl.off, k);
    n += k;
    dl.off += k;
  }
  if (n == 0) endDownload();
  return n;
}

}  // namespace

void begin(const char* ssid, const char* password) {
  queue = xQueueCreate(32, sizeof(Msg));
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) { req->send(200, "text/html", kPage); });
  server.on("/log.csv", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!log_src.count || dl_active) {
      req->send(503, "text/plain", "busy");
      return;
    }
    dl_active = true;
    log_src.freeze(true);
    dl.next = 0;
    dl.total = log_src.count();
    dl.len = dl.off = 0;
    AsyncWebServerResponse* resp = req->beginChunkedResponse("text/csv", fillChunk);
    resp->addHeader("Content-Disposition", "attachment; filename=pendulum_log.csv");
    req->onDisconnect([] { endDownload(); });
    req->send(resp);
  });
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

bool take(Msg& m) { return queue && xQueueReceive(queue, &m, 0) == pdTRUE; }

void publish(const Status& s) {
  ws.cleanupClients();
  if (!ws.count()) return;
  char msg[112];
  snprintf(msg, sizeof(msg), "s,%s,%.1f,%.2f,%.2f,%s,%.1f,%.1f,%d", s.state, s.th_deg, s.vin, s.v_ref, s.warning, s.u,
           s.rec_s, s.rec_all ? 1 : 0);
  ws.textAll(msg);
}

void sendText(const char* msg) {
  if (ws.count()) ws.textAll(msg);
}

void setLogSource(const LogSource& src) { log_src = src; }

}  // namespace remote
