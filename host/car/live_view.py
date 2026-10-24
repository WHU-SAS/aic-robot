"""Local browser preview. Serial ownership remains with the calling thread."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


PAGE = '''<!doctype html><meta charset="utf-8"><title>K230 蓝色方块实时检测</title>
<style>body{font:17px sans-serif;background:#edf1f5;color:#14283d;margin:20px}main{display:flex;gap:20px;align-items:flex-start}img{width:100%;background:#16283b}article{flex:1;min-width:480px}aside{width:280px;line-height:1.9}button{padding:12px 24px;font-size:18px}#status{font-size:24px;font-weight:bold}pre{white-space:pre-wrap}small{color:#526476}</style>
<h2>K230 蓝色方块 · 连续视觉测试</h2><p>小车和机械臂保持静止。左右移动 → 远近移动 → 拿出画面 → 重新放回。</p>
<main><article><img id="preview" alt="等待K230画面"><small>检测框和红色中心十字由K230绘制，与画面对应；右侧是最近收到的检测结果。</small></article>
<aside><div id="status">等待相机预热</div><pre id="details"></pre><button id="stop">结束测试</button><p><small>平面距离为模型估计。更新率分别统计观测与电脑预览，不代表测距精度。</small></p></aside></main>
<script>const names={tracking:'正在跟踪',confirming:'正在确认',lost:'目标丢失',no_target:'未发现目标'};
let lastSequence=-1, ended=false;
document.getElementById('stop').onclick=async()=>{await fetch('/stop',{method:'POST'});document.getElementById('status').textContent='正在结束并恢复语音…';ended=true};
async function refresh(){if(ended)return;try{let s=await(await fetch('/state',{cache:'no-store'})).json();let o=s.observation||{};
document.getElementById('status').textContent=s.finished?s.message:(s.observation_age_s>1?'检测结果停更':(s.message||names[o.status]||'等待相机预热'));
const num=(x,n=1)=>x===null||x===undefined?'—':Number(x).toFixed(n);
document.getElementById('details').textContent=`检测帧：${o.frame_id??'—'}\n中心：${o.center_pixel?o.center_pixel.map(x=>num(x,0)).join(', '):'—'} px\n中线偏差：${num(o.image_midline_error_px)} px\n蓝色面积：${num(o.pixels,0)} px\n估计平面距离：${num(o.estimated_table_distance_mm,0)} mm\n单次检测：${num(o.processing_ms)} ms\n观测更新率：${o.frame_period_ms>0?num(1000/o.frame_period_ms):'—'} Hz\n预览接收率：${num(s.preview_hz)} Hz\n观测已停更：${num(s.observation_age_s)} 秒`;
if(s.preview_sequence!==lastSequence&&s.preview_sequence>0){lastSequence=s.preview_sequence;document.getElementById('preview').src='/frame.jpg?v='+lastSequence}
if(s.finished)ended=true;
}catch(e){if(!ended)document.getElementById('status').textContent='连接已结束，请查看终端记录'}
if(!ended)setTimeout(refresh,200)}refresh();</script>'''


class LiveView:
    def __init__(self, port=0):
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.jpeg = None
        self.state = {'observation': None, 'preview_sequence': 0,
                      'preview_hz': 0, 'finished': False, 'message': '等待相机预热'}
        view = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_GET(self):
                path = self.path.split('?', 1)[0]
                with view.lock:
                    if path == '/': data, kind = PAGE.encode(), 'text/html; charset=utf-8'
                    elif path == '/state':
                        state = dict(view.state)
                        stamp = (state.get('observation') or {}).get('host_received_monotonic_s')
                        state['observation_age_s'] = time.monotonic() - stamp if stamp is not None else None
                        data, kind = json.dumps(state).encode(), 'application/json'
                    elif path == '/frame.jpg' and view.jpeg is not None: data, kind = view.jpeg, 'image/jpeg'
                    else:
                        self.send_error(404); return
                self.send_response(200)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try: self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError): pass
            def do_POST(self):
                if self.path != '/stop': self.send_error(404); return
                view.stop.set(); self.send_response(204); self.end_headers()
        self.server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    @property
    def url(self): return 'http://127.0.0.1:%d/' % self.server.server_port

    def start(self): self.thread.start()

    def update(self, **values):
        with self.lock: self.state.update(values)

    def frame(self, jpeg, hz):
        with self.lock:
            self.jpeg = jpeg
            self.state['preview_sequence'] += 1
            self.state['preview_hz'] = hz

    def close(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=2)
