'use strict';
const $=id=>document.getElementById(id);
const flagNames=['servo','remote','mode','alarm','error','running','hold'];
const axes=['x','y','z','rx','ry','rz'];
let state=null,pending=false,initialized=false,uiError='',claimAttempted=false;
let connectionAttempted=false,connecting=false;
const tri=v=>v===true?'ON':v===false?'OFF':'-';
function robotMode(robot){
const {play,teach}=robot.flags;
if(robot.connected!==true||typeof play!=='boolean'||typeof teach!=='boolean')return '—';
if(play===true&&teach===false)return 'PLAY';
if(play===false&&teach===true)return 'TEACH';
return 'UNKNOWN';
}
const text=(id,value)=>{const content=String(value??'').replace(/\bUNKNOWN\b/gi,'-');if(id==='notice'){showNotice(content);return;}$(id).textContent=content;};
const notificationBar=document.querySelector('.notificationBar');
let noticeTimer=null,lastNotice=null,queuedNotice='',noticeQueued=false;
function showNotice(content){
queuedNotice=content;
if(noticeQueued)return;
noticeQueued=true;
// Render may choose a more specific message; display only its final choice.
queueMicrotask(()=>{
noticeQueued=false;
if(queuedNotice===lastNotice)return;
lastNotice=queuedNotice;
clearTimeout(noticeTimer);
$('notice').textContent=queuedNotice;
notificationBar.hidden=!queuedNotice;
if(queuedNotice)noticeTimer=setTimeout(()=>{notificationBar.hidden=true;},15000);
});
}
new ResizeObserver(()=>{
document.documentElement.style.setProperty('--notification-height',`${notificationBar.offsetHeight}px`);
if(fullscreenPages)requestAnimationFrame(fitFullscreenPages);
}).observe(notificationBar);
let fullscreenPages=null;
function fitFullscreenPages(){
if(!fullscreenPages)return;
const {scroller,live,settings}=fullscreenPages;
scroller.classList.remove('fullscreenRelaxed');
const robot=live.querySelector('.status');
const relaxed=innerWidth<=1000||innerHeight<=700||live.scrollHeight>live.clientHeight+1||settings.scrollHeight>settings.clientHeight+1||robot.scrollHeight>robot.clientHeight+1;
scroller.classList.toggle('fullscreenRelaxed',relaxed);
}
function layoutFullscreen(active){
if(active&&!fullscreenPages){
const scroller=document.createElement('div'),live=document.createElement('section'),settings=document.createElement('section');
scroller.className='fullscreenScreens';live.className='fullscreenPage fullscreenLive';settings.className='fullscreenPage fullscreenSettings';
const nodes=[document.querySelector('header.statusBar'),document.querySelector('main'),document.querySelector('.settings'),document.querySelector('.handControls')];
const originals=nodes.map(node=>{const marker=document.createComment('fullscreen layout');node.before(marker);return {node,marker};});
fullscreenPages={scroller,live,settings,originals,scroll:[scrollX,scrollY]};
live.append(nodes[0],nodes[1]);settings.append(nodes[2],nodes[3]);scroller.append(live,settings);document.body.append(scroller);
fitFullscreenPages();
}else if(!active&&fullscreenPages){
const {scroller,originals,scroll}=fullscreenPages;
for(const {node,marker}of originals)marker.replaceWith(node);
scroller.remove();fullscreenPages=null;window.scrollTo(...scroll);
}
}
window.addEventListener('resize',()=>{if(fullscreenPages)requestAnimationFrame(fitFullscreenPages);});
function renderFullscreen(){
const active=Boolean(document.fullscreenElement),button=$('fullscreen');
layoutFullscreen(active);
button.querySelector('path').setAttribute('d',active?'M3 8h5V3m8 0v5h5M8 21v-5H3m18 0h-5v5':'M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5');
button.setAttribute('aria-label',active?'Thoát toàn màn hình':'Toàn màn hình');
button.setAttribute('aria-pressed',String(active));
button.disabled=!document.fullscreenEnabled;
button.title=document.fullscreenEnabled?(active?'Thoát toàn màn hình (Esc)':'Toàn màn hình'):'Trình duyệt không hỗ trợ toàn màn hình';
}
document.addEventListener('fullscreenchange',renderFullscreen);
$('fullscreen').onclick=async()=>{
try{
$('fullscreen').disabled=true;
if(document.fullscreenElement)await document.exitFullscreen();
else await document.documentElement.requestFullscreen();
}catch(error){text('notice',`Không thể đổi chế độ toàn màn hình: ${error.message}`);}
finally{renderFullscreen();}
};
renderFullscreen();
function renderConnection(id,label,connected){
const el=$(id),status=connected===true?'ONLINE':connected===false?'OFFLINE':'UNKNOWN';
el.textContent=status==='UNKNOWN'?'-':status;el.className=`connectionStatus ${connected===true?'is-online':connected===false?'is-offline':'is-unknown'}`;
el.setAttribute('aria-label',`${label} ${status.toLowerCase()}`);
}
let stackCountDirty=false;
async function claimControl(){
const send=token=>fetch('/api/control/claim',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token})});
let response=await send('');
if(response.status===403){
const token=window.prompt('Nhập mã điều khiển HMI');
if(token!==null)response=await send(token);
}
if(!response.ok){const result=await response.json();throw Error(result.detail||'Không nhận được quyền điều khiển');}
}
function renderLaser(robot){
text('laser',tri(robot.laser));$('laser').className=robot.laser===true?'on':robot.laser===false?'off':'unknown';
for(let i=0;i<2;i++){
const value=robot.vacuum_inputs?.[i]??(Number.isInteger(robot.laser_raw_byte)?Boolean(robot.laser_raw_byte&(1<<(i+1))):null);
const el=$(`pressure${i+1}`);
el.textContent=tri(value);el.className=value===true?'on':value===false?'off':'unknown';
}
}
function renderDrop(s){
const d=s.drop_height;
if(!d)return;
text('nextDropZ',d.next_drop_z_mm);
if(!stackCountDirty)$('stackedCount').value=d.stacked_count;
const editable=!pending&&s.control&&!s.busy&&['IDLE','COMPLETE'].includes(s.state);
$('stackedCount').disabled=!editable;$('saveStackCount').disabled=!editable;
}
for(const key of flagNames){const box=document.createElement('div');box.className='flag';box.innerHTML=`<span>${key.toUpperCase()}</span><b id="flag-${key}" class="unknown">${key==='mode'?'—':'-'}</b>`;$('flags').insertBefore(box,document.querySelector('.sensorFirst'));}
for(const key of axes){const box=document.createElement('div');box.className='axis';box.innerHTML=`<span>${key.toUpperCase()}</span><b id="pos-${key}">—</b>`;$('position').append(box);}
async function refresh(){try{const r=await fetch('/api/state');if(!r.ok)throw Error('Mất kết nối server');state=await r.json();if(!state.control&&!claimAttempted){claimAttempted=true;await claimControl();const latest=await fetch('/api/state');if(!latest.ok)throw Error('HMI state unavailable');state=await latest.json();}render();}catch(e){uiError=e.message;text('notice',uiError);text('ready','-');renderConnection('robotLink','Robot',null);renderConnection('cameraLink','Camera',null);text('connect',connectionAttempted?'KẾT NỐI LẠI':'CONNECT');$('connect').className='connectButton';for(const k of flagNames){text(`flag-${k}`,k==='mode'?'—':'-');$(`flag-${k}`).className='unknown';}document.querySelectorAll('button:not([data-local])').forEach(b=>b.disabled=true);renderLaser({});$('preview').style.display='none';}}
function render(){const s=state;const connected=s.robot.connected&&s.camera.connected;const gate=s.capabilities.full_cycle;const ready=gate.allowed&&connected&&s.state==='IDLE'&&['servo','remote','play'].every(k=>s.robot.flags[k]===true)&&['teach','alarm','error','running','hold'].every(k=>s.robot.flags[k]===false);text('mode',s.mode);renderConnection('robotLink','Robot',s.robot.connected);renderConnection('cameraLink','Camera',s.camera.connected);text('ready',s.state==='FAULT'?'FAULT':s.busy?'BUSY':ready?'READY':'NOT READY');text('state',s.state);text('fps',`${s.camera.fps} FPS`);
for(const k of flagNames){const el=$(`flag-${k}`);if(k==='mode'){const mode=robotMode(s.robot);el.textContent=mode;el.className=mode==='PLAY'||mode==='TEACH'?'on':'unknown';continue;}const v=s.robot.flags[k];el.textContent=tri(v);el.className=v===true?'on':v===false?'off':'unknown';}
for(const k of axes)text(`pos-${k}`,s.robot.position?Number(s.robot.position[k]).toFixed(2):'—');
text('target',s.target?['x','y','z'].map(a=>s.target.pose[a].toFixed(1)).join(' / '):'— / — / —');text('laser',tri(s.robot.laser));renderLaser(s.robot);
text('notice',uiError||s.error||s.robot.error||gate.reason||s.real_limit||(s.state==='JOB_DISPATCHED'?`Đã gửi START ${s.job_dispatch?.name||'MOVEUNTI'}. Robot có thể vẫn đang chạy JOB.`:s.state==='COMPLETE'?(s.drop_release?.counted||s.drop_release?.board_release_confirmed?'Cycle hoàn tất · H2OFF đã xong, bộ đếm đã cập nhật':'Cycle hoàn tất'):s.mode==='MOCK'?'MOCK · Camera và robot được mô phỏng.':'REAL · Xác minh khu vực trước khi điều khiển.'));
if(!uiError&&!s.error&&!s.robot.error&&s.state==='COMPLETE'&&s.stop_reason==='no_boards')text('notice',`Đã dừng: không còn phát hiện ván · Đã xếp ${s.drop_height.stacked_count} tấm`);
if(!uiError&&!s.error&&!s.robot.error&&s.state==='IDLE'&&s.servo_enable?.verified&&s.servo_target===s.robot.flags.servo)text('notice',`Servo đã ${s.robot.flags.servo?'ON':'OFF'}.`);
const detections=s.camera.detections||[];text('detected',detections.length?detections.map(d=>`ID ${d.id} · ${(100*d.confidence).toFixed(0)}%`).join('   /   '):'Chưa có vật hợp lệ');text('calib',s.mode==='MOCK'?'MOCK · tọa độ mô phỏng':`Calibration: ${s.camera.calibration}`);
if(s.camera.connected){$('preview').src=`/api/preview.jpg?t=${Date.now()}`;$('preview').style.display='block';$('cameraNote').style.display='none';}else{$('preview').style.display='none';$('cameraNote').style.display='block';}
if(!initialized){for(const [key,value]of Object.entries({...s.settings,first_drop_z_mm:s.drop_height.first_drop_z_mm,board_thickness_mm:s.drop_height.board_thickness_mm})){const input=$('settings').elements[key];if(input)input.value=value;};initialized=true;}
document.querySelectorAll('button:not([data-local])').forEach(b=>b.disabled=pending||!s.control);$('start').disabled=pending||!s.control||!ready||stackCountDirty;$('ack').disabled=pending||!s.control||s.busy||s.robot.flags.running!==false||!['COMPLETE','JOB_DISPATCHED','HOLD','FAULT'].includes(s.state);
if(s.robot.connected===true||s.camera.connected===true)connectionAttempted=true;
text('connect',connecting?'ĐANG KẾT NỐI…':connected?'ĐÃ KẾT NỐI':connectionAttempted?'KẾT NỐI LẠI':'CONNECT');
$('connect').className=`connectButton${connected?' is-connected':''}`;
$('connect').disabled=pending||!s.control||s.busy||Boolean(connected)||['SERVO_ENABLING','SERVO_DISABLING'].includes(s.state);
$('connect').title=connected?'Camera và robot đã kết nối':'Kết nối camera và robot';
const servoState=s.robot.flags.servo,servoKnown=typeof servoState==='boolean';
const servoCapability=servoState===true?s.capabilities.servo_off:s.capabilities.servo_on;
text('servoOn',servoKnown?(servoState?'SERVO OFF':'SERVO ON'):'SERVO -');
$('servoOn').className=`servoButton ${!servoKnown?'is-unknown':servoState?'turn-off':'turn-on'}`;
$('servoOn').disabled=pending||!s.control||!servoKnown||!servoCapability?.allowed;
$('servoOn').title=servoCapability?.reason||(servoState?'Tắt nguồn servo':'Bật nguồn servo');
if(s.state==='SERVO_ENABLING'||s.state==='SERVO_DISABLING')text('notice',`Đang ${s.servo_target?'bật':'tắt'} servo, chờ xác nhận trạng thái SERVO ${s.servo_target?'ON':'OFF'}…`);
const releasing=s.robot.flags.hold_command===true;
text('hold',releasing?'UNHOLD':'HOLD');
$('hold').className=releasing?'releaseHold':'holdButton';
$('hold').setAttribute('aria-pressed',String(releasing));
$('hold').disabled=pending||!s.control||(releasing&&!s.capabilities.release_hold.allowed);
$('hold').title=releasing?(s.capabilities.release_hold.reason||'Nhả command HOLD'):'';
document.querySelectorAll('[data-channel]').forEach(b=>b.disabled=pending||!s.control||s.busy||!(s.capabilities.manual_vacuum||s.capabilities.vacuum_only).allowed||!['IDLE','COMPLETE'].includes(s.state));
for(const el of $('settings').elements)el.disabled=pending||!s.control||s.busy||s.state!=='IDLE';
renderDrop(s);

}
async function command(path,extra={}){if(pending||!state)return;pending=true;uiError='';render();try{const body={request_id:crypto.randomUUID(),revision:state.revision,...extra};const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const result=await r.json();if(!r.ok)throw Error(result.detail||'Yêu cầu thất bại');}catch(e){uiError=e.message;}finally{pending=false;await refresh();}}

$('connect').onclick=async()=>{connectionAttempted=true;connecting=true;try{await command('/api/system/connect');}finally{connecting=false;if(state)render();}};
$('start').onclick=()=>command('/api/cycle/start',{stage:'full_cycle'});$('ack').onclick=()=>command('/api/cycle/acknowledge',{inspected:true});
$('servoOn').onclick=()=>{if(typeof state?.robot.flags.servo!=='boolean')return;const on=!state.robot.flags.servo;const operation=command(`/api/robot/servo-${on?'on':'off'}`);text('notice',`Đang ${on?'bật':'tắt'} servo, chờ xác nhận trạng thái SERVO ${on?'ON':'OFF'}…`);return operation;};
document.querySelectorAll('[data-channel]').forEach(b=>b.onclick=()=>command(`/api/vacuum/${b.dataset.channel}`,{on:b.dataset.on==='true'}));
$('hold').onclick=()=>{
if(state.robot.flags.hold_command!==true){command('/api/cycle/hold');return;}
if(!state.capabilities.release_hold.allowed)return;
if(window.confirm('Nhả HOLD có thể cho phép chuyển động đang treo tiếp tục. Xác nhận khu vực đã sẵn sàng và gửi NHẢ HOLD?'))command('/api/cycle/release-hold',{confirmed:true});
};
$('settings').onsubmit=async e=>{e.preventDefault();if(!e.target.reportValidity())return;const settings={},drop_settings={};for(const [k,v]of new FormData(e.target)){const values=['first_drop_z_mm','board_thickness_mm'].includes(k)?drop_settings:settings;values[k]=Number(v);}await command('/api/config',{settings,drop_settings});if(!uiError)text('notice','Đã lưu thông số vận hành.');};
$('stackedCount').oninput=()=>{stackCountDirty=true;if(state)render();};
$('stackedCount').onkeydown=e=>{if(e.key==='Escape'){stackCountDirty=false;if(state)render();}};
$('stackCountForm').onsubmit=async e=>{
e.preventDefault();if(!$('stackCountForm').reportValidity())return;
const count=Number($('stackedCount').value);
if(!Number.isSafeInteger(count)||count<0)return;
await command('/api/drop/count',{stacked_count:count});
if(!uiError){stackCountDirty=false;render();}
};
setInterval(()=>{if(!pending)refresh();},500);refresh();


