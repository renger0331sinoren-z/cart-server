            if "estop" in d:
                self.estop = bool(d["estop"])
                if self.estop and self.nav.state == g.NavState.RUNNING: self.nav._hold("非常停止", now)
            if "mode" in d and d["mode"] in ("manual", "assist", "follow", "auto"): msg = self.set_mode(d["mode"])
            if d.get("auto_start"):
                if self.mode != "auto": msg = "先に「自動運転」モードを選んでください。"
                elif self.estop: msg = "非常停止中は開始できません。"
                elif self.nav.i == 0 and not self.moved and self.calib is None:
                    self.calib = {"t0": now, "xs": [], "ys": []}      # 最初の開始は、先に位置合わせをする
                    self.nav.reason = f"位置合わせを始めます(約{CALIB_SEC}秒。台車を動かさないでください)"
                    msg = f"位置合わせを始めます。約{CALIB_SEC}秒、台車に触れず、動かさないでください。"
                elif self.calib is None:
                    self.nav.start(); self.moved = True
            if d.get("auto_stop") and self.mode == "auto":
                self.nav.state, self.nav.reason = g.NavState.IDLE, "停止"
            if "hold" in d: self.use_hold = bool(d["hold"])
            if "slope" in d: self.world.slope = 0.2 if d["slope"] else 0.0
            if "target_speed" in d: self.world.pspeed = float(d["target_speed"])
            if "gps_down" in d: self.world.gps_down = bool(d["gps_down"])
            if "gps_bad" in d: self.world.hdop = 5.0 if d["gps_bad"] else 1.2
            if "obstacle" in d: self.world.static = [OBSTACLE] if d["obstacle"] else []
            if d.get("place_person"): self.world.place_person()
            if "route" in d: msg = self.set_route(d["route"])
            if d.get("reset"):
                self.world.reset(); self.follow.stop(); self.nav.state = g.NavState.IDLE; self._fresh_state()
                self.gov.set_mode(Mode.MANUAL)
        return msg

app = CartApp(); PIN = f"{secrets.randbelow(10000):04d}"

def control_loop():
    while True:
        t0 = time.time(); app.tick()
        time.sleep(max(0, DT - (time.time() - t0)))

# ---------- Webサーバー ----------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _send(self, code, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code); self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
    def _auth(self, pin): return secrets.compare_digest(str(pin), PIN)
    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/": return self._send(200, PAGE, "text/html")
        if u.path == "/api/state":
            if not self._auth(parse_qs(u.query).get("pin", [""])[0]): return self._send(403, '{"error":"pin"}')
            return self._send(200, json.dumps(app.state()))
        self._send(404, "{}")
    def do_POST(self):
        if urlparse(self.path).path != "/api/cmd": return self._send(404, "{}")
        try: d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
        except Exception: return self._send(400, '{"error":"bad json"}')
        if not self._auth(d.get("pin", "")): return self._send(403, '{"error":"pin"}')
        try: msg = app.handle(d)
        except (ValueError, TypeError): return self._send(400, '{"error":"bad value"}')
        s = app.state(); s["msg"] = msg; self._send(200, json.dumps(s))

PAGE = """<!DOCTYPE html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><title>台車 操作ページ</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>:root{--bg:#f4f6f9;--cd:#fff;--tx:#1b2230;--mu:#5b6575;--bd:#d6dbe3;--ac:#1f6feb;--ng:#cf222e;--ok:#1a7f37;--wn:#b25e00;color-scheme:light dark}
@media(prefers-color-scheme:dark){:root{--bg:#0e131a;--cd:#171e28;--tx:#e6eaf0;--mu:#97a1b1;--bd:#2c3644;--ac:#58a6ff;--ng:#ff7b72;--ok:#3fb950;--wn:#e3a008}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font-family:system-ui,"Hiragino Sans","Noto Sans JP",sans-serif;padding:12px}
.c{background:var(--cd);border:1px solid var(--bd);border-radius:12px;padding:12px;margin:0 auto 10px;max-width:520px}
button{font:inherit;padding:10px 14px;border-radius:8px;border:1px solid var(--bd);background:var(--cd);color:var(--tx)}
button.on{background:var(--ac);color:#fff;border-color:var(--ac)}#es.on{background:var(--ng);border-color:var(--ng)}
.r{display:flex;flex-wrap:wrap;gap:8px}.pad{display:grid;grid-template-columns:repeat(3,72px);gap:8px;justify-content:center}
.pad button{height:64px;font-size:1.4rem;touch-action:none;user-select:none}.big{font-size:1.1rem;font-weight:600}.mu{color:var(--mu);font-size:.85rem}
input{font:inherit;padding:8px;width:7em}label{font-size:.9rem;display:flex;gap:4px;align-items:center}
canvas{width:100%;height:auto;border-radius:8px;background:var(--bg);display:block}
.pin{width:24px;height:24px;border-radius:12px;background:#1f6feb;color:#fff;font:600 12px/20px sans-serif;text-align:center;border:2px solid #fff}.pin.s{background:#1a7f37}select{font:inherit;padding:8px}</style></head><body>
<div class="c" id="lg">暗証番号(起動時に表示された4桁): <input id="pin" inputmode="numeric" maxlength="4"> <button onclick="go()">接続</button></div>
<div class="c"><div class="r" id="md"><button data-m="manual">手動</button><button data-m="assist">自動アシスト</button><button data-m="follow">追従</button><button data-m="auto">自動運転</button><button id="es">非常停止</button></div>
<label style="margin-top:8px"><input type="checkbox" id="vc" style="width:auto">音声案内(アナウンス)</label>
<p class="mu" id="msg"></p></div>
<div class="c"><b>自動運転</b>
<div class="r" style="margin:8px 0"><button id="as">▶ 開始・再開</button><button id="ap">■ 一時停止</button></div>
<div>状態: <span class="big" id="nv">-</span></div><div class="mu" id="nr"></div>
<canvas id="cv" width="480" height="320"></canvas>
<p class="mu">周囲の安全を確認し、台車から目を離さないでください。最初の「開始」では、約30秒の位置合わせ(GPSのずれの測定)をします。台車を、地図でタップしたスタート地点に置き、動かさないでください。ページを閉じる・通信が切れる・手動で操作すると、一時停止します。</p></div>
<div class="c"><b>地図で場所を選ぶ(日本・無料)</b>
<div class="r" style="margin:8px 0"><select id="mp"><option value="gstd">国土地理院 地図</option><option value="gpho">国土地理院 航空写真</option></select></div>
<label style="margin-bottom:6px"><input type="checkbox" id="sh" checked style="width:auto">最短ルート(行き先の順番を最適化・障害物はよけて最短で迂回)</label>
<div class="r"><button id="m_o" class="on">① スタート地点</button><button id="m_w">② 行き先を追加</button><button id="m_r">🛣 道に沿う</button><button id="m_c">消去</button><button id="m_s">台車に送る</button></div>
<div class="r" style="margin-top:8px"><input id="q" placeholder="場所・住所(例: 東京駅)" style="flex:1;min-width:10em;width:auto"><button id="qb">検索</button></div>
<div id="qr" style="margin-top:6px;max-height:180px;overflow:auto;display:flex;flex-direction:column;gap:4px"></div>
<div id="map" style="height:320px;margin-top:8px;border-radius:8px;background:var(--bg)"></div>
<p class="mu" id="mm">場所を検索するか、地図を拡大して、台車を置く場所をタップしてください。</p>
<p class="mu">検索: 国土地理院と OpenStreetMap(© OpenStreetMap contributors)の両方から、似た名前も含めて探します。緯度,経度(例: 35.68, 139.76)も入力できます。</p>
<p class="mu">「道に沿う」: 行き先まで、地図の道(歩行者向け)に沿った経路を作ります(OpenStreetMap / FOSSGIS の無料サービス)。地図の「道」は、人や車がいない道とは限りません。管理された敷地内の道だけで使い、台車から目を離さないでください。</p></div>
<div class="c"><div class="pad"><span></span><button data-k="u">▲</button><span></span><button data-k="l">◀</button><button data-k="d">▼</button><button data-k="r">▶</button></div>
<p class="mu">ボタンを押している間だけ指令が出ます。指を離す・通信が切れると停止します。</p></div>
<div class="c"><div>AIの判断: <span class="big" id="rs">-</span></div>
<div>前方の距離: <b id="ds">-</b> cm / 速度: <b id="vv">-</b> cm/s / 追従: <b id="fl">-</b></div></div>
<div class="c"><div class="r"><label><input type="checkbox" id="hd" checked style="width:auto">速度維持</label>
<label><input type="checkbox" id="sl" style="width:auto">坂道(模擬)</label>
<label><input type="checkbox" id="gd" style="width:auto">GPS途絶(模擬)</label>
<label><input type="checkbox" id="gb" style="width:auto">GPS精度悪化(模擬)</label>
<label><input type="checkbox" id="ob" style="width:auto">進路に障害物(模擬)</label></div>
<div class="r" style="margin-top:8px"><button id="pp">前の人を正面に置く</button><button id="t0">前の人: 止まる</button><button id="t1">前の人: 歩く</button><button id="rst">リセット</button></div>
<p class="mu">このカードは模擬世界の操作です。実機では使いません。</p></div>
<script>
let pin="",k={u:0,d:0,l:0,r:0},extra={},ok=false;const $=i=>document.getElementById(i);
let vc=false,lk="",lm="",lf="",ln="",lr=0,down=false,lastSay={};
function say(t,u){if(!vc||!window.speechSynthesis)return;const n=Date.now();if(lastSay[t]&&n-lastSay[t]<3000)return;lastSay[t]=n;
if(u)speechSynthesis.cancel();const x=new SpeechSynthesisUtterance(t);x.lang="ja-JP";speechSynthesis.speak(x)}
$("vc").onchange=e=>{vc=e.target.checked;if(vc)say("音声案内をオンにしました",1)};
const MN={manual:"手動モード",assist:"自動アシストモード",follow:"追従モード",auto:"自動運転モード"};
const AT={es:["非常停止しました",1],bp:["バンパーが接触しました。停止します",1],fault:["センサー異常です。停止します",1],stop:["障害物を検知しました。停止しました",1],slow:["前方に障害物があります。減速します",0]};
function ann(s){const r=s.reason||"";let q="ok";
if(r.includes("非常停止"))q="es";else if(r.includes("バンパー"))q="bp";else if(r.includes("異常"))q="fault";
else if(/障害物.*停止/.test(r))q="stop";else if(r.includes("減速"))q="slow";
if(q!=lk){if(AT[q])say(AT[q][0],AT[q][1]);lk=q}
if(s.mode!=lm){if(lm)say(MN[s.mode]+"です",0);lm=s.mode}
if(s.follow!=lf){if(s.follow=="追従中")say("追従を開始します",0);else if(s.follow.startsWith("見失い"))say("人を見失いました。停止します",1);lf=s.follow}
if(s.replans>lr)say("障害物を見つけました。最短の迂回路を作りました",0);lr=s.replans;
if(s.nav!=ln){if(s.mode=="auto"){if(s.nav=="自動走行中")say("自動運転を開始します",0);else if(s.nav.startsWith("一時停止"))say("自動運転を一時停止します。"+s.nav_reason,1);else if(s.nav=="全地点に到着")say("全地点に到着しました",0)}ln=s.nav}}
function go(){pin=$("pin").value;ok=true;$("lg").style.display="none"}
document.querySelectorAll("[data-m]").forEach(b=>b.onclick=()=>extra.mode=b.dataset.m);
let es=false;$("es").onclick=()=>{es=!es;extra.estop=es};
$("as").onclick=()=>extra.auto_start=true;$("ap").onclick=()=>extra.auto_stop=true;
$("hd").onchange=e=>extra.hold=e.target.checked;$("sl").onchange=e=>extra.slope=e.target.checked;
$("gd").onchange=e=>extra.gps_down=e.target.checked;$("gb").onchange=e=>extra.gps_bad=e.target.checked;$("ob").onchange=e=>extra.obstacle=e.target.checked;
$("pp").onclick=()=>extra.place_person=true;$("t0").onclick=()=>extra.target_speed=0;$("t1").onclick=()=>extra.target_speed=30;
$("rst").onclick=()=>{extra.reset=true;es=false;["sl","gd","gb","ob"].forEach(i=>$(i).checked=false)};
document.querySelectorAll("[data-k]").forEach(b=>{const n=b.dataset.k;b.onpointerdown=e=>{e.preventDefault();k[n]=1};
["onpointerup","onpointerleave","onpointercancel"].forEach(t=>b[t]=()=>k[n]=0)});
const M={ArrowUp:"u",ArrowDown:"d",ArrowLeft:"l",ArrowRight:"r"};
onkeydown=e=>{if(e.target.tagName==="INPUT")return;if(M[e.key]){k[M[e.key]]=1;e.preventDefault()}};onkeyup=e=>{if(M[e.key])k[M[e.key]]=0};
function draw(s){const c=$("cv"),x=c.getContext("2d"),W=c.width,H=c.height,cs=getComputedStyle(document.documentElement);
const tx=cs.getPropertyValue("--tx"),ac=cs.getPropertyValue("--ac"),mu=cs.getPropertyValue("--mu"),ok2=cs.getPropertyValue("--ok");
x.clearRect(0,0,W,H);const P=[[0,0],...s.route,[s.pos[0],s.pos[1]],s.person];
const x0=Math.min(...P.map(p=>p[0]))-4,x1=Math.max(...P.map(p=>p[0]))+4,y0=Math.min(...P.map(p=>p[1]))-4,y1=Math.max(...P.map(p=>p[1]))+4;
const sc=Math.min(W/(x1-x0),H/(y1-y0)),X=a=>(a-x0)*sc,Y=b=>H-(b-y0)*sc;
x.strokeStyle=mu;x.lineWidth=2;x.setLineDash([6,4]);x.beginPath();x.moveTo(X(0),Y(0));s.route.forEach(p=>x.lineTo(X(p[0]),Y(p[1])));x.stroke();x.setLineDash([]);
x.font="13px sans-serif";x.textAlign="center";x.textBaseline="middle";
const dn=s.route.length>12;s.route.forEach((p,i)=>{x.fillStyle=(i==s.wp[0]&&s.nav!="全地点に到着")?ac:mu;x.beginPath();x.arc(X(p[0]),Y(p[1]),dn?3:9,0,7);x.fill();if(!dn){x.fillStyle="#fff";x.fillText(i+1,X(p[0]),Y(p[1]))}});
if(s.obstacle){x.fillStyle=mu;x.beginPath();x.arc(X(s.obstacle[0]),Y(s.obstacle[1]),7,0,7);x.fill()}
(s.known_obs||[]).forEach(o=>{x.globalAlpha=0.5;x.fillStyle=cs.getPropertyValue("--ng");x.beginPath();x.arc(X(o[0]),Y(o[1]),Math.max(5,0.5*sc),0,7);x.fill();x.globalAlpha=1});
x.fillStyle=ok2;x.beginPath();x.arc(X(s.person[0]),Y(s.person[1]),6,0,7);x.fill();
x.save();x.translate(X(s.pos[0]),Y(s.pos[1]));x.rotate(s.pos[2]*Math.PI/180);x.fillStyle=s.estop?cs.getPropertyValue("--ng"):tx;
x.beginPath();x.moveTo(0,-11);x.lineTo(8,9);x.lineTo(-8,9);x.closePath();x.fill();x.restore()}
async function tick(){if(!ok)return;const body={pin,speed:k.u?1:k.d?-1:0,turn:k.r?1:k.l?-1:0,...extra};extra={};
try{const r=await fetch("/api/cmd",{method:"POST",body:JSON.stringify(body)});
if(r.status==403){$("msg").textContent="暗証番号が違います。";ok=false;$("lg").style.display="";return}
const s=await r.json();down=false;ann(s);draw(s);if(s.ll)updateCart(s.ll);
$("rs").textContent=s.reason||"-";$("ds").textContent=s.dist;$("vv").textContent=s.v;$("fl").textContent=s.follow;
$("nv").textContent=s.nav+"(地点 "+s.wp[0]+"/"+s.wp[1]+")";$("nr").textContent=s.nav_reason;
es=s.estop;$("es").classList.toggle("on",es);$("es").textContent=es?"非常停止中(押して解除)":"非常停止";
document.querySelectorAll("[data-m]").forEach(b=>b.classList.toggle("on",b.dataset.m==s.mode));
$("msg").textContent=s.msg||"";}catch(e){$("msg").textContent="通信が切れました。台車は自動で停止します。";if(!down){down=true;say("通信が切れました。台車は停止します",1)}}}
/* ---------- 地図(国土地理院の地図。無料・キー不要) ---------- */
let MP=null,mode2="o",orig=null,pts=[],ov=[],cartMk=null,provider="gstd",roadLL=null,srOv=[];
const FENCE = 4000, LIMIT = 4000
function mm(t){$("mm").textContent=t}
function hav(a,b){const R=6371000,t=Math.PI/180,p=(b[0]-a[0])*t,l=(b[1]-a[1])*t,x=Math.sin(p/2)**2+Math.cos(a[0]*t)*Math.cos(b[0]*t)*Math.sin(l/2)**2;return 2*R*Math.asin(Math.sqrt(x))}
function mapBuild(){
