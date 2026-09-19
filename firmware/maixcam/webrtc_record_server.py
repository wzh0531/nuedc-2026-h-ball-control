"""为MaixPy WebRTC提供带录像/回放/保存按钮的本地网页。"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


RECORD_HTTP_PORT = 8080
RECORD_MAX_SECONDS = 10 * 60

_zero_lock = threading.Lock()
_zero_state = {
    "request_id": 0,
    "action": "",
    "state": "success",
    "message": "尚未操作",
    "selected_zero_cm": 0.0,
    "calibration_valid": False,
    "claimed": False,
}


def _zero_snapshot_locked():
    return {
        "request_id": _zero_state["request_id"],
        "action": _zero_state["action"],
        "state": _zero_state["state"],
        "message": _zero_state["message"],
        "selected_zero_cm": _zero_state["selected_zero_cm"],
        "calibration_valid": _zero_state["calibration_valid"],
    }


def _request_zero_action(action):
    with _zero_lock:
        if _zero_state["state"] == "pending":
            return _zero_snapshot_locked()
        _zero_state["request_id"] += 1
        _zero_state["action"] = action
        _zero_state["state"] = "pending"
        _zero_state["message"] = "等待AI主循环处理"
        _zero_state["claimed"] = False
        return _zero_snapshot_locked()


def request_set_zero():
    """Submit a non-blocking request to use the current stable ball position."""
    return _request_zero_action("set")


def request_reset_zero():
    """Submit a non-blocking request to restore the physical 0 cm zero."""
    return _request_zero_action("reset")


def claim_zero_request():
    """Let the AI loop atomically claim one pending web request."""
    with _zero_lock:
        if (
            _zero_state["state"] != "pending"
            or _zero_state["claimed"]
        ):
            return None
        _zero_state["claimed"] = True
        return {
            "request_id": _zero_state["request_id"],
            "action": _zero_state["action"],
        }


def complete_zero_request(
    request_id,
    success,
    message,
    selected_zero_cm,
):
    """Publish the AI loop result without exposing detector state to HTTP."""
    with _zero_lock:
        if (
            request_id != _zero_state["request_id"]
            or _zero_state["state"] != "pending"
        ):
            return False
        _zero_state["state"] = "success" if success else "failed"
        _zero_state["message"] = str(message)
        _zero_state["selected_zero_cm"] = float(selected_zero_cm)
        _zero_state["claimed"] = False
        return True


def update_zero_runtime_status(selected_zero_cm, calibration_valid):
    """Update display-only values; it never changes the active zero."""
    with _zero_lock:
        _zero_state["selected_zero_cm"] = float(selected_zero_cm)
        _zero_state["calibration_valid"] = bool(calibration_valid)


def get_zero_status():
    """Return a thread-safe status snapshot for the web page."""
    with _zero_lock:
        return _zero_snapshot_locked()


RECORD_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>MaixCAM WebRTC录像</title>
<style>
:root{color-scheme:dark;--bg:#080a0f;--panel:#171b25;--ok:#18a66f;--bad:#d94c60;--blue:#3478ee;--amber:#e9a23b}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;min-height:100%;background:var(--bg);color:#f5f7fa;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}
body{padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}
.bar{position:sticky;top:0;z-index:5;padding:9px;background:rgba(23,27,37,.96);border-bottom:1px solid #303746;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
button{min-height:43px;padding:8px 14px;border:0;border-radius:9px;color:white;font-size:16px;font-weight:650}
button:disabled{opacity:.38}
#start{background:var(--ok)}#stop{background:var(--bad)}#play{background:var(--blue)}#save{background:var(--amber);color:#171109}#fullscreen{background:#555d6e}
#setZero{background:#7757d8}#resetZero{background:#3f8c9b}
#status{font-size:15px;white-space:nowrap}
.zero-panel{width:min(100%,1280px);padding:10px;border-radius:9px;background:var(--panel);display:flex;gap:9px;align-items:center;flex-wrap:wrap}
.zero-value{font-variant-numeric:tabular-nums;font-weight:700}
#zeroStatus{color:#b9c1ce}
.stage{padding:7px;display:grid;gap:8px;justify-items:center}
.video-wrap{position:relative;width:min(100%,1280px)}
#video{display:block;width:100%;height:auto;max-height:calc(100vh - 78px);object-fit:contain;background:#000;border-radius:7px}
.badge{position:absolute;left:9px;bottom:9px;padding:5px 8px;border-radius:7px;background:rgba(0,0,0,.62);font-size:13px}
#playback{display:none;width:min(100%,960px);max-height:35vh;background:#000;border:2px solid #fff;border-radius:7px}
.hint{width:min(100%,1280px);color:#b9c1ce;font-size:13px;line-height:1.45}
.online{color:#59d9a1}.offline{color:#ff7d8c}.recording{color:#ff6d7d}
@media (orientation:landscape){
 .bar{padding:6px 9px}
 button{min-height:37px;font-size:14px;padding:6px 11px}
 .stage{padding:4px}
 #video{max-height:calc(100vh - 57px)}
 #playback{position:fixed;z-index:8;right:8px;bottom:8px;width:34vw;max-height:32vh}
}
</style>
</head>
<body>
<div class="bar">
  <button id="start" disabled>开始录像</button>
  <button id="stop" disabled>停止</button>
  <button id="play" disabled>回放</button>
  <button id="save" disabled>保存/分享</button>
  <button id="fullscreen">全屏</button>
  <span id="status">正在连接WebRTC…</span>
</div>
<main class="stage">
  <section class="zero-panel">
    <button id="setZero">将当前钢球位置设为零点</button>
    <button id="resetZero">恢复中心零点</button>
    <span>当前零点：<span id="zeroValue" class="zero-value">0.00 cm</span></span>
    <span id="zeroStatus">正在读取零点状态…</span>
  </section>
  <div class="video-wrap">
    <video id="video" autoplay playsinline muted></video>
    <div class="badge"><span id="resolution">-- × --</span> <span id="codec">unknown</span></div>
  </div>
  <video id="playback" controls playsinline></video>
  <div class="hint">
    录像直接读取WebRTC视频流，不录浏览器界面。录像暂存在本页内存中；
    刷新或关闭页面前必须“保存/分享”，并在iPad面板中选择“存储到文件”。
  </div>
</main>
<script>
"use strict";
const SIGNAL_PORT=8001;
const MAX_RECORD_MS=__MAX_RECORD_MS__;
const video=document.getElementById("video");
const playback=document.getElementById("playback");
const startBtn=document.getElementById("start");
const stopBtn=document.getElementById("stop");
const playBtn=document.getElementById("play");
const saveBtn=document.getElementById("save");
const fullscreenBtn=document.getElementById("fullscreen");
const setZeroBtn=document.getElementById("setZero");
const resetZeroBtn=document.getElementById("resetZero");
const zeroValue=document.getElementById("zeroValue");
const zeroStatus=document.getElementById("zeroStatus");
const statusText=document.getElementById("status");
const resolutionText=document.getElementById("resolution");
const codecText=document.getElementById("codec");

let websocket=null;
let peer=null;
let recorder=null;
let chunks=[];
let recordedBlob=null;
let recordedUrl=null;
let recordStartMs=0;
let recordTimer=null;
let zeroPollBusy=false;

function setStatus(text,kind=""){
  statusText.textContent=text;
  statusText.className=kind;
}

function renderZeroStatus(status){
  zeroValue.textContent=`${Number(status.selected_zero_cm).toFixed(2)} cm`;
  const pending=status.state==="pending";
  setZeroBtn.disabled=pending;
  resetZeroBtn.disabled=pending;
  const calibrationText=status.calibration_valid?"已标定":"尚未完成标定";
  zeroStatus.textContent=`${status.state}: ${status.message}（${calibrationText}）`;
  zeroStatus.className=status.state==="failed"?"offline":(
    status.state==="success"?"online":""
  );
}

async function refreshZeroStatus(){
  if(zeroPollBusy)return;
  zeroPollBusy=true;
  try{
    const response=await fetch("/api/zero/status",{cache:"no-store"});
    if(!response.ok)throw new Error(`HTTP ${response.status}`);
    renderZeroStatus(await response.json());
  }catch(error){
    zeroStatus.textContent=`零点状态读取失败：${error.message}`;
    zeroStatus.className="offline";
  }finally{
    zeroPollBusy=false;
  }
}

async function submitZeroRequest(path){
  setZeroBtn.disabled=true;
  resetZeroBtn.disabled=true;
  try{
    const response=await fetch(path,{method:"POST",cache:"no-store"});
    if(!response.ok)throw new Error(`HTTP ${response.status}`);
    renderZeroStatus(await response.json());
  }catch(error){
    zeroStatus.textContent=`零点请求失败：${error.message}`;
    zeroStatus.className="offline";
    setZeroBtn.disabled=false;
    resetZeroBtn.disabled=false;
  }
}

function randomId(length){
  const chars="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  const values=new Uint32Array(length);
  if(window.crypto&&crypto.getRandomValues)crypto.getRandomValues(values);
  else for(let i=0;i<length;i++)values[i]=Math.floor(Math.random()*0xffffffff);
  return Array.from(values,value=>chars[value%chars.length]).join("");
}

function updateVideoInfo(){
  resolutionText.textContent=video.videoWidth&&video.videoHeight
    ?`${video.videoWidth} × ${video.videoHeight}`:"-- × --";
  try{
    const receiver=peer?.getReceivers()?.find(item=>item.track?.kind==="video");
    const params=receiver?.getParameters();
    const mime=params?.codecs?.[0]?.mimeType||"";
    codecText.textContent=mime?mime.replace("video/","").toUpperCase():"unknown";
  }catch(_error){}
}

function createPeerConnection(){
  const connection=new RTCPeerConnection({
    bundlePolicy:"max-bundle",
    iceServers:[{urls:["stun:stun.l.google.com:19302"]}]
  });
  connection.ontrack=event=>{
    const stream=event.streams[0]||new MediaStream([event.track]);
    video.srcObject=stream;
    video.play().catch(()=>{});
    startBtn.disabled=false;
    setStatus("WebRTC已连接，可开始录像","online");
    video.addEventListener("loadedmetadata",updateVideoInfo,{once:true});
    setTimeout(updateVideoInfo,500);
  };
  connection.onconnectionstatechange=()=>{
    const state=connection.connectionState;
    if(state==="failed"||state==="disconnected"||state==="closed"){
      startBtn.disabled=true;
      setStatus(`WebRTC连接${state}`,"offline");
    }
  };
  return connection;
}

function waitIceGatheringComplete(connection){
  return new Promise(resolve=>{
    if(connection.iceGatheringState==="complete"){resolve();return;}
    const listener=()=>{
      if(connection.iceGatheringState==="complete"){
        connection.removeEventListener("icegatheringstatechange",listener);
        resolve();
      }
    };
    connection.addEventListener("icegatheringstatechange",listener);
  });
}

async function handleOffer(offer){
  if(peer)peer.close();
  peer=createPeerConnection();
  await peer.setRemoteDescription(offer);
  await peer.setLocalDescription(await peer.createAnswer());
  await waitIceGatheringComplete(peer);
  websocket.send(JSON.stringify({
    id:"server",
    type:peer.localDescription.type,
    sdp:peer.localDescription.sdp
  }));
}

function connectWebRTC(){
  const wsUrl=`ws://${window.location.hostname}:${SIGNAL_PORT}/${randomId(10)}`;
  websocket=new WebSocket(wsUrl);
  websocket.onopen=()=>{
    websocket.send(JSON.stringify({id:"server",type:"request"}));
  };
  websocket.onmessage=async event=>{
    if(typeof event.data!=="string")return;
    const message=JSON.parse(event.data);
    if(message.type==="offer"){
      try{await handleOffer(message);}
      catch(error){setStatus(`WebRTC协商失败：${error.message}`,"offline");}
    }
  };
  websocket.onerror=()=>setStatus("WebSocket信令连接失败","offline");
  websocket.onclose=()=>{
    if(!video.srcObject)setStatus("WebSocket信令已断开","offline");
  };
}

function supportedMime(){
  if(!window.MediaRecorder||!MediaRecorder.isTypeSupported)return "";
  const candidates=[
    "video/mp4;codecs=avc1.42E01E",
    "video/mp4",
    "video/webm;codecs=vp8",
    "video/webm"
  ];
  return candidates.find(type=>MediaRecorder.isTypeSupported(type))||"";
}

function clearOldRecording(){
  if(recordedUrl){URL.revokeObjectURL(recordedUrl);recordedUrl=null;}
  recordedBlob=null;
  playback.pause();
  playback.removeAttribute("src");
  playback.load();
  playback.style.display="none";
  playBtn.disabled=true;
  saveBtn.disabled=true;
}

function stopTimer(){
  if(recordTimer){clearInterval(recordTimer);recordTimer=null;}
}

startBtn.addEventListener("click",()=>{
  if(!video.srcObject||video.srcObject.getVideoTracks().length===0){
    alert("WebRTC视频尚未连接");
    return;
  }
  if(!window.MediaRecorder){
    alert("此iPad浏览器不支持MediaRecorder，请升级iPadOS或更换已验证设备。");
    return;
  }
  clearOldRecording();
  chunks=[];
  const mime=supportedMime();
  try{
    recorder=mime
      ?new MediaRecorder(video.srcObject,{mimeType:mime,videoBitsPerSecond:3000000})
      :new MediaRecorder(video.srcObject);
  }catch(error){
    alert(`无法启动录像：${error.message}`);
    return;
  }
  recorder.ondataavailable=event=>{
    if(event.data&&event.data.size>0)chunks.push(event.data);
  };
  recorder.onerror=event=>{
    setStatus(`录像错误：${event.error?.message||"未知错误"}`,"offline");
  };
  recorder.onstop=()=>{
    stopTimer();
    const type=recorder.mimeType||mime||"video/mp4";
    recordedBlob=new Blob(chunks,{type});
    recordedUrl=URL.createObjectURL(recordedBlob);
    playback.src=recordedUrl;
    playback.style.display="block";
    startBtn.disabled=false;
    stopBtn.disabled=true;
    playBtn.disabled=false;
    saveBtn.disabled=false;
    setStatus(`录像完成 ${(recordedBlob.size/1048576).toFixed(1)} MB`,"online");
  };
  recorder.start(1000);
  recordStartMs=Date.now();
  startBtn.disabled=true;
  stopBtn.disabled=false;
  setStatus("正在录像 0.0 秒","recording");
  recordTimer=setInterval(()=>{
    const elapsed=Date.now()-recordStartMs;
    setStatus(`正在录像 ${(elapsed/1000).toFixed(1)} 秒`,"recording");
    if(elapsed>=MAX_RECORD_MS&&recorder.state==="recording")recorder.stop();
  },200);
});

stopBtn.addEventListener("click",()=>{
  if(recorder&&recorder.state==="recording")recorder.stop();
});

playBtn.addEventListener("click",async()=>{
  if(!recordedUrl)return;
  playback.currentTime=0;
  try{await playback.play();}
  catch(error){alert(`请点击回放视频上的播放键：${error.message}`);}
});

saveBtn.addEventListener("click",async()=>{
  if(!recordedBlob)return;
  const extension=recordedBlob.type.includes("mp4")?"mp4":"webm";
  const name=`maixcam_${new Date().toISOString().replace(/[:.]/g,"-")}.${extension}`;
  const file=new File([recordedBlob],name,{type:recordedBlob.type});
  if(navigator.share&&navigator.canShare&&navigator.canShare({files:[file]})){
    try{
      await navigator.share({files:[file],title:"MaixCAM钢球录像"});
      return;
    }catch(error){
      if(error.name==="AbortError")return;
    }
  }
  const link=document.createElement("a");
  link.href=recordedUrl;
  link.download=name;
  link.target="_blank";
  document.body.appendChild(link);
  link.click();
  link.remove();
});

fullscreenBtn.addEventListener("click",()=>{
  if(!document.fullscreenElement){
    (video.requestFullscreen||video.webkitRequestFullscreen)?.call(video);
  }else{
    (document.exitFullscreen||document.webkitExitFullscreen)?.call(document);
  }
});

setZeroBtn.addEventListener("click",()=>{
  submitZeroRequest("/api/zero/set");
});

resetZeroBtn.addEventListener("click",()=>{
  submitZeroRequest("/api/zero/reset");
});

window.addEventListener("beforeunload",event=>{
  if(recorder&&recorder.state==="recording"){
    event.preventDefault();
    event.returnValue="";
  }
});

connectWebRTC();
refreshZeroStatus();
setInterval(refreshZeroStatus,500);
</script>
</body>
</html>
""".replace(
    "__MAX_RECORD_MS__",
    str(RECORD_MAX_SECONDS * 1000),
)


class RecordPageHandler(BaseHTTPRequestHandler):
    """仅服务单页录像器；视频与信令仍走WebRTC端口。"""

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = RECORD_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/zero/status":
            self._send_json(get_zero_status())
            return
        if self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
            return
        self.send_error(404)

    def do_POST(self):
        if self.path == "/api/zero/set":
            self._send_json(request_set_zero(), status=202)
            return
        if self.path == "/api/zero/reset":
            self._send_json(request_reset_zero(), status=202)
            return
        self.send_error(404)

    def _send_json(self, payload, status=200):
        body = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format, *_args):
        return


def start_record_server():
    """在后台启动录像页面HTTP服务器，返回(server, thread)。"""
    server = ThreadingHTTPServer(("", RECORD_HTTP_PORT), RecordPageHandler)
    thread = threading.Thread(
        target=server.serve_forever,
        name="webrtc-record-page",
        daemon=True,
    )
    thread.start()
    return server, thread


def stop_record_server(server, thread):
    """停止录像页面服务器，不触碰WebRTC视频服务器。"""
    if server is None:
        return
    server.shutdown()
    server.server_close()
    if thread is not None:
        thread.join(timeout=2.0)
