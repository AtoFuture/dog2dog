import RFB from '/vendor/novnc/core/rfb.js';

const $ = id => document.getElementById(id);
let interactionRequested = true;
let rfb, connected = false, state, filter = 'all', retry, toastTimer, lastMapPng, mapImage;
const missions = {standby:'待机', exploring:'探索中', returning:'返航中', paused:'已暂停', estop:'急停'};
const statuses = {waiting_for_server:'等待导航服务',goal_request_sent:'已请求导航',cancel_requested:'请求取消',sending:'下发目标', accepted:'目标已接收', succeeded:'执行成功', failed:'执行失败', canceled:'目标已取消', nav_succeeded_pending_verification:'导航到达，正在复核', localization_rejected:'定位复核未通过', localization_failed:'返航定位失败', home_reference_captured:'已记录归位基准', verification_started:'开始归位复核', correction_requested:'请求归位修正', return_replan_requested:'定位变化，准备重规划', return_replan_started:'返航已重规划', integrity_fault:'定位跳变', integrity_recovered:'定位已稳定', odometry_home_rebased:'里程计基准已校正', map_home_rebased:'地图基准已校正', return_verified:'真实归位已确认', brain_requested:'请求决策', brain_selected:'决策完成', exploration_ended:'探索结束', search_unavailable:'暂无可用候选'};
const guardReasons = {allowed:'运动许可', clearance_recovery:'净空恢复 · 安全方向前移', footprint_clearance:'机身净空保护', predicted_footprint_collision:'预测净空保护', nav_command_unavailable:'导航请求过期', front_obstacle:'前方障碍', excessive_tilt:'倾角保护', 'mission:standby':'待机归零', 'mission:estop':'急停归零'};
function toast(message) { $('toast').textContent=message; $('toast').hidden=false; clearTimeout(toastTimer); toastTimer=setTimeout(()=>$('toast').hidden=true,3500); }
function syncInteraction() {
  const allowed=Boolean(state?.gui_interactive), active=connected&&allowed&&interactionRequested;
  if(rfb)rfb.viewOnly=!active;
  const button=$('interaction');button.disabled=!connected||!allowed;
  $('focus-robot').disabled=!connected||!allowed||focusing;
  button.setAttribute('aria-pressed',String(active));
  button.querySelector('span').textContent=active?'交互已开启':'仅查看';
  button.title=allowed?'切换 Gazebo 鼠标与键盘交互':'服务端已禁用交互';
  $('interaction-hint').textContent=!connected?'等待连接':active?'拖动平移 · 滚轮缩放 · 中键旋转':'仅查看 · 交互已关闭';
}
function streamStatus(value, detail) {
  connected=value;
  syncInteraction();
  $('scene-empty').hidden=value;
  $('scene-live').hidden=!value;
  $('capture').disabled=!value;
  $('stream-state').textContent=value?'画面已连接':'画面未连接';
  $('stream-dot').style.background=value?'var(--positive)':'var(--muted)';
  $('connection-badge').classList.toggle('connected',value);
  $('connection-badge').lastElementChild.textContent=value?'画面已连接':'画面未连接';
  if (detail) { $('scene-empty-title').textContent='等待仿真画面'; $('scene-empty-description').textContent=detail; }
}
function connect() {
  clearTimeout(retry);
  if (rfb) { const old=rfb; rfb=null; old.disconnect(); }
  if (!state?.window.available) { streamStatus(false,state?.window.message || '等待控制台连接信息。'); return; }
  $('vnc-screen').replaceChildren();
  streamStatus(false,'正在连接现有 Gazebo 窗口…');
  try {
    const client = new RFB($('vnc-screen'),`${location.protocol==='https:'?'wss':'ws'}://${location.host}/vnc`,{shared:true});
    rfb=client; client.viewOnly=true; client.scaleViewport=true; client.resizeSession=false; client.clipViewport=false;
    client.addEventListener('connect',()=>{ if(rfb===client)streamStatus(true); });
    client.addEventListener('disconnect',()=>{ if(rfb!==client)return; streamStatus(false,'窗口转发已断开，5 秒后重试。'); retry=setTimeout(connect,5000); });
    client.addEventListener('securityfailure',()=>streamStatus(false,'窗口连接失败，请检查控制台日志。'));
  } catch(error) { streamStatus(false,error.message); }
}
function fresh(key, limit=3) {
  const age=(state?.telemetry.ages?.[key] ?? Infinity)+(state?.bridge_age_s ?? Infinity);
  return age<limit ? state.telemetry.values[key] : undefined;
}
function number(value, digits=2) { return Number.isFinite(value)?value.toFixed(digits):'—'; }
function ageText(age) { return Number.isFinite(age)?age<3?`${age.toFixed(1)} s`:'已过期':'未接入'; }
function setTag(id, text, live=false, warning=false) { const e=$(id);e.textContent=text;e.classList.toggle('live',live);e.classList.toggle('warning',warning); }
const commandLabels={EXPLORE:'开始探索',PAUSE:'暂停',RESUME:'继续',RETURN:'返航',ESTOP:'急停',RESET:'解除急停',STANDBY:'待机'};
const commandStatuses={sent:'已发送，等待 G3 回执',accepted:'G3 已接受',rejected:'G3 已拒绝',unconfirmed:'回执未确认'};
let taskSending=0;
function renderTasks() {
  const control=state?.task_control;
  const records=control?.records||[], pending=taskSending>0||records.some(r=>r.status==='sent');
  document.querySelectorAll('[data-command]').forEach(button=>{
    button.disabled=!control?.allowed?.includes(button.dataset.command)||(pending&&button.dataset.command!=='ESTOP');
  });
  $('task-badge').textContent=!state?'连接中断':!control?.enabled?'任务控制未启用':control.mission_fresh?(missions[fresh('mission')]||'已连接'):'任务状态已过期';
  const latest=records.at(-1);
  $('task-feedback').textContent=latest?`${commandLabels[latest.command]}：${commandStatuses[latest.status]}${latest.reason?' · '+latest.reason:''}`:
    !state?'连接后可操作任务。':!control?.enabled?'服务端未启用任务控制。':control.mission_fresh?'操作交给 G3 执行；收到回执后显示接受或拒绝，任务执行结果见下方事件。':'等待实时任务状态；ROS 连接存活时仍可发送急停。';
  const history=$('task-history');history.replaceChildren();
  records.slice(-5).reverse().forEach(record=>{
    const row=document.createElement('li');
    [commandLabels[record.command],commandStatuses[record.status],record.reason||(record.state?`确认状态：${missions[record.state]||record.state}`:'等待回执')].forEach(text=>{
      const span=document.createElement('span');span.textContent=text;row.append(span);
    });history.append(row);
  });
}
document.querySelectorAll('[data-command]').forEach(button=>button.addEventListener('click',async()=>{
  const command=button.dataset.command;taskSending++;renderTasks();
  try {
    const response=await fetch(`/api/mission/${command}`,{method:'POST',signal:AbortSignal.timeout(5000)});
    const record=await response.json();if(!response.ok)throw new Error(record.error||'指令发送失败');
    if(state?.task_control&&!state.task_control.records.some(r=>r.request_id===record.request_id))state.task_control.records.push(record);
  }catch(error){toast(error.message||'指令状态未知，请检查任务状态。');}
  finally{taskSending--;renderTasks();}
}));
function render() {
  if(!state)return;
  const telemetry=state.telemetry, mission=fresh('mission'), guard=fresh('guard'), odom=fresh('odometry'), battery=fresh('battery',5);
  const corridor=fresh('corridor',2), geometry=corridor?.scan_fresh&&corridor?.confirmed?corridor.geometry:undefined;
  const corridorModes={observing:'通道观测 · 导航控制',centering:'居中行驶',approaching:'入口居中',entry_aligning:'入口停车对准',entry_stabilizing:'入口稳定确认',direction_unconfirmed:'等待通道方向',reversal_turning:'通道掉头 · 方向锁定',turn_settling:'停车等待步态收尾',turn_recovery_settling:'回中前停车等待',turn_recovery_orienting:'转向通道中心',turn_recentering:'掉头侧移回中',turn_recovery_complete:'回中完成',turn_recovery_timeout:'回中超时 · 已停车',turn_recovery_exhausted:'恢复次数已用完 · 已停车',turn_sensor_clearance:'雷达转向空间不足 · 已停车',turning:'受控转向',aligning:'停车调整方向',stationary:'已识别 · 静止',inactive:'已识别 · 待机',awaiting_path:'等待导航路径',goal_approach:'接近目标',planner_obstacle_avoidance:'通道内避障'};
  $('corridor-state').textContent=geometry?(corridorModes[corridor.mode]||'已识别窄通道'):corridor?.scan_fresh?'未确认窄通道':'等待有效雷达';
  $('corridor-geometry').textContent=geometry?`${number(geometry.width_m)} m / ${geometry.center_offset_m>=0?'左':'右'} ${number(Math.abs(geometry.center_offset_m))} m`:'—';
  const hasData=Object.keys(telemetry.ages||{}).some(key=>fresh(key)!==undefined);
  setTag('ros-badge',hasData?'ROS 在线':state.bridge_alive?'等待 ROS':'ROS 未连接',hasData);
  $('mission-state').textContent=missions[mission]||mission||'未接入';
  $('domain-value').textContent=state.domain;
  $('display-label').textContent=`DISPLAY ${state.window.display}`;
  $('window-label').textContent=`WINDOW ${state.window.id||'—'}`;
  $('linear-value').textContent=number(odom?.linear);
  $('clearance-value').textContent=number(guard?.footprint_clearance_m);
  $('battery-value').textContent=Number.isFinite(battery)?`${Math.round(battery*100)}%`:'—';
  $('battery-bar').style.width=Number.isFinite(battery)?`${Math.min(100,Math.max(0,battery*100))}%`:'0%';
  const safe=guard?.reason==='allowed'||guard?.reason==='mission:standby';
  setTag('guard-badge',guard?safe?'正常':'保护中':'未接入',!!guard&&safe,!!guard&&!safe);
  $('guard-reason').textContent=guardReasons[guard?.reason]||guard?.reason||'等待状态';
  $('pose-age').textContent=ageText((telemetry.ages?.pose??Infinity)+(state.bridge_age_s??Infinity));
  $('nav-age').textContent=ageText(guard ? (guard.inputs_age_s?.nav_command??Infinity)+(telemetry.ages?.guard??Infinity)+(state.bridge_age_s??Infinity) : Infinity);
  $('dialog-domain').textContent=state.domain;
  $('dialog-display').textContent=`${state.window.display} / ${state.window.id||'未找到窗口'}`;
  $('dialog-ros').textContent=hasData?'正在接收数据':state.bridge_alive?'观察节点已连接，等待话题数据':'观察节点未连接';
  $('dialog-window').textContent=connected?(state.gui_interactive?'已连接 · 可交互':'已连接 · 只读'):state.window.message;
  syncInteraction();renderTasks();renderMap(); renderEvents();
}
function renderMap() {
  const map=fresh('map',20), pose=fresh('pose'), canvas=$('map-canvas'), ctx=canvas.getContext('2d');
  if(!map) {ctx.clearRect(0,0,canvas.width,canvas.height);$('map-empty').hidden=false;setTag('map-badge','等待数据');$('map-details').textContent='MAP —';return;}
  setTag('map-badge','真实地图',true);$('map-empty').hidden=true;
  $('map-details').textContent=`${map.resolution.toFixed(2)} m/px · ${map.width}×${map.height}`;
  if(map.png!==lastMapPng) { lastMapPng=map.png;mapImage=new Image();mapImage.onload=()=>renderMap();mapImage.src=`data:image/png;base64,${map.png}`; }
  if(!mapImage?.complete||!mapImage.naturalWidth)return;
  ctx.clearRect(0,0,canvas.width,canvas.height);
  const scale=Math.min((canvas.width-40)/map.width,(canvas.height-30)/map.height);
  const x=(canvas.width-map.width*scale)/2,y=(canvas.height-map.height*scale)/2;
  ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImage,x,y,map.width*scale,map.height*scale);
  if(pose) {
    const [ox,oy,yaw]=map.origin,dx=pose.x-ox,dy=pose.y-oy;
    const px=(dx*Math.cos(yaw)+dy*Math.sin(yaw))/map.resolution,py=map.height-(-dx*Math.sin(yaw)+dy*Math.cos(yaw))/map.resolution;
    ctx.save();ctx.translate(x+px*scale,y+py*scale);ctx.rotate(-(pose.yaw-yaw));const palette=getComputedStyle(document.documentElement);ctx.fillStyle=palette.getPropertyValue('--map-robot').trim();ctx.strokeStyle=palette.getPropertyValue('--map-outline').trim();ctx.lineWidth=2;
    ctx.beginPath();ctx.moveTo(11,0);ctx.lineTo(-7,7);ctx.lineTo(-4,0);ctx.lineTo(-7,-7);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
  }
}
function renderEvents() {
  const events=state.telemetry.events||[]; $('event-count').textContent=events.length;
  const rows=events.filter(e=>filter==='all'||e.purpose===filter).slice().reverse();
  const root=$('event-list');root.replaceChildren();
  if(!rows.length) {
    const box=document.createElement('div');box.className='events-empty';
    const icon=document.createElement('span');icon.className='event-empty-icon';icon.textContent='≋';
    const group=document.createElement('div'),title=document.createElement('strong'),text=document.createElement('p');
    title.textContent=filter==='all'?'等待探索事件':'暂无匹配事件';text.textContent='接入运行栈后，目标下发、扫描与执行结果会出现在这里。';group.append(title,text);box.append(icon,group);root.append(box);return;
  }
  rows.forEach(e=>{
    const row=document.createElement('div');row.className='event-row';
    const values=[['event-index',e.attempt_id?`#${String(e.attempt_id).padStart(3,'0')}`:'—'],['event-status',statuses[e.status]||e.status],['event-target',e.target?`x ${number(e.target.x)}  y ${number(e.target.y)}`:e.reason||'—'],['event-purpose',e.purpose==='scan'?'扫描':e.purpose==='frontier'?'探索目标':e.purpose==='return'?'返航':e.purpose==='localization'?'定位复核':'系统']];
    values.forEach(([cls,text])=>{const span=document.createElement('span');span.className=cls;span.textContent=text;span.title=text;row.append(span);});root.append(row);
  });
}
let firstPoll=true;
async function poll() {
  try {
    const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(4000)});if(!response.ok)throw new Error('状态连接失败');
    state=await response.json();render();
    if(firstPoll){firstPoll=false;connect();}
    if(!state.window.available&&connected){rfb?.disconnect();streamStatus(false,state.window.message);}
  } catch(error) {
    state=undefined;syncInteraction();renderTasks(); ['linear-value','clearance-value','battery-value'].forEach(id=>$(id).textContent='—');
    $('battery-bar').style.width='0%';$('map-details').textContent='MAP —';$('dialog-ros').textContent='状态连接已断开';
    $('mission-state').textContent='状态连接已断开';setTag('ros-badge','连接中断',false,true);setTag('guard-badge','数据已过期',false,true);
    $('guard-reason').textContent='状态连接已断开';$('pose-age').textContent='已过期';$('nav-age').textContent='已过期';
    $('map-canvas').getContext('2d').clearRect(0,0,640,320);$('map-empty').hidden=false;setTag('map-badge','连接中断',false,true);
  } finally {setTimeout(poll,1500);}
}
let focusing=false;
$('focus-robot').addEventListener('click',async()=>{
  if(focusing)return;focusing=true;$('focus-robot').disabled=true;
  try {
    const response=await fetch('/api/view/robot',{method:'POST',signal:AbortSignal.timeout(20000)});
    const result=await response.json();if(!response.ok)throw new Error(result.error||'相机定位失败');
    toast('镜头已移向机器狗。');
  }catch(error){toast(error.message||'相机定位失败，请重试。');}
  finally{focusing=false;syncInteraction();}
});
$('interaction').addEventListener('click',()=>{interactionRequested=!interactionRequested;syncInteraction();});
$('reconnect').addEventListener('click',connect);
$('fullscreen').addEventListener('click',async()=>{try{if(document.fullscreenElement)await document.exitFullscreen();else await $('viewport').requestFullscreen();}catch{toast('当前浏览器不支持全屏。');}});
$('capture').addEventListener('click',()=>{if(!connected||!rfb)return;try{const data=rfb.getImageData(),canvas=document.createElement('canvas');canvas.width=data.width;canvas.height=data.height;canvas.getContext('2d').putImageData(data,0,0);canvas.toBlob(blob=>{if(!blob)return;const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=`go2-gazebo-${Date.now()}.png`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);toast('当前画面已保存。');});}catch{toast('暂时无法获取画面，请重连后再试。');}});
['settings','settings-rail'].forEach(id=>$(id).addEventListener('click',()=>$('connection-dialog').showModal()));
$('close-dialog').addEventListener('click',()=>$('connection-dialog').close());
$('connection-dialog').addEventListener('click',e=>{if(e.target===$('connection-dialog')){const r=e.target.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)e.target.close();}});
document.querySelectorAll('[data-filter]').forEach(button=>button.addEventListener('click',()=>{filter=button.dataset.filter;document.querySelectorAll('[data-filter]').forEach(e=>e.classList.toggle('selected',e===button));if(state)renderEvents();}));
// Repaint the current framebuffer after fullscreen/layout changes as well as
// rescaling it. A paused simulator may not send another damaged rectangle.
function fitStream() {
  requestAnimationFrame(()=>{
    if(!connected||!rfb)return;
    rfb.scaleViewport=true;
    const canvas=$('vnc-screen').querySelector('canvas');
    try {const frame=rfb.getImageData();if(canvas&&frame.width&&frame.height)canvas.getContext('2d').putImageData(frame,0,0);} catch {}
  });
}
new ResizeObserver(fitStream).observe($('viewport'));
document.addEventListener('fullscreenchange',fitStream);
window.addEventListener('beforeunload',()=>rfb?.disconnect());
poll();
