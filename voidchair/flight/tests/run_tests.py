"""Offline CC:Tweaked flight tests. Run with Python and lupa (Lua 5.2)."""
from pathlib import Path
from lupa.lua52 import LuaRuntime

ROOT = Path(__file__).resolve().parents[1]
lua = LuaRuntime(unpack_returned_tuples=True)
for name in ('config', 'route', 'survey', 'checkpoint', 'core', 'pilot', 'setup', 'dashboard'):
    lua.globals()[name + '_source'] = (ROOT / (name + '.lua')).read_text(encoding='utf-8')
checks = lua.execute(r'''
local sources = {config=config_source, route=route_source, survey=survey_source,
  checkpoint=checkpoint_source, core=core_source, pilot=pilot_source, setup=setup_source, dashboard=dashboard_source}
function dofile(path)
  local name = path:match('([^/]+)%.lua$')
  return assert(load(assert(sources[name], path), '@' .. path))()
end
local config, R, S, C = dofile('config.lua'), dofile('route.lua'),
  dofile('survey.lua'), dofile('checkpoint.lua')
local checks = 0
local function check(ok, message)
  checks = checks + 1
  assert(ok, message)
end
local function near(a,b) return math.abs(a-b)<1e-6 end
local function mission(size, chunks)
  return {x1=-size/2,z1=-size/2,x2=size/2,z2=size/2,chunks=chunks,altitude=120}
end
local function covered(plan,x,z)
  local p = plan.points[1]
  if math.max(math.abs(x-p.x),math.abs(z-p.z)) <= plan.halfWidth then return true end
  for i=2,#plan.points do
    local a,b = plan.points[i-1],plan.points[i]
    local dx = math.max(math.min(a.x,b.x)-x,0,x-math.max(a.x,b.x))
    local dz = math.max(math.min(a.z,b.z)-z,0,z-math.max(a.z,b.z))
    if math.max(dx,dz) <= plan.halfWidth - config.waypointRadius then return true end
  end
  return false
end
for _, size in ipairs({1,32,32.01,64,128,600,2048}) do
  for _,chunks in ipairs({1,2,4,8,16,32}) do
    local plan = R.build(mission(size,chunks),config)
    check(plan.scanStep <= chunks*32*.9+1e-6, 'lane spacing')
    for ix=0,40 do for iz=0,40 do
      check(covered(plan,-size/2+size*ix/40,-size/2+size*iz/40),
        'coverage hole: '..size..'/'..chunks..'/'..ix..'/'..iz)
    end end
  end
end
check(not pcall(R.build,mission(0,4),config),'zero area rejected')
check(not pcall(R.build,mission(100,0),config),'zero radius rejected')
local bad=mission(100,1); bad.x2=80
check(not pcall(R.build,bad,config),'rectangle rejected')
for _,height in ipairs({501,560,1024,-64}) do
  local high=mission(64,1); high.altitude=height
  local highPlan=R.build(high,config)
  check(highPlan.altitude==height,'cruise Y independent of build height')
  local highTask=S.start(highPlan)
  local highPos={x=0,y=70,z=0}
  for _=1,10000 do
    local velocity=S.update(highTask,highPos,config,.05)
    highPos={x=highPos.x+velocity[1]*.05,y=highPos.y+velocity[2]*.05,
      z=highPos.z+velocity[3]*.05}
    if highTask.mode=='complete' then break end
  end
  check(highTask.mode=='complete' and math.abs(highPos.y-height)<=config.altitudeTolerance,
    'mission completes at custom cruise height')
end
local invalidHeight=mission(64,1); invalidHeight.altitude=math.huge
check(not pcall(R.build,invalidHeight,config),'infinite cruise Y rejected')
invalidHeight.altitude=0/0
check(not pcall(R.build,invalidHeight,config),'NaN cruise Y rejected')
local plan = R.build(mission(600,4),config)
local t = S.start(plan)
local pos = {x=-1000,y=70,z=800}
local scanTicks,maxTransit,maxScan=0,0,0
local paused=false
for tick=1,30000 do
  local oldMode,oldIndex=t.mode,t.index
  local v=S.update(t,pos,config,.05)
  local speed=math.sqrt(v[1]^2+v[2]^2+v[3]^2)
  check(speed<=340+1e-6,'total speed cap')
  check(t.index>=oldIndex,'progress monotonic')
  if oldMode=='climb_outbound' or oldMode=='resume_climb' then
    check(v[1]==0 and v[3]==0,'climb before horizontal transit')
  end
  if oldMode=='scan' then
    maxScan=math.max(maxScan,speed); scanTicks=scanTicks+1
    check(speed<=80+1e-6,'scan speed cap')
  else maxTransit=math.max(maxTransit,speed) end
  pos={x=pos.x+v[1]*.05,y=pos.y+v[2]*.05,z=pos.z+v[3]*.05}
  if t.mode=='scan' and scanTicks>15 and not paused then
    paused=true
    local index=t.index
    S.pause(t,pos)
    local anchor=t.pausePoint
    pos={x=pos.x+900,y=40,z=pos.z-500}
    S.resume(t)
    S.pause(t,pos)
    check(t.pausePoint==anchor and t.index==index,'repeat pause retains scan anchor')
    local snap=S.snapshot(t,pos)
    check(snap.pausePoint==anchor,'snapshot retains original anchor')
    S.resume(t)
    t.mode='resume_return'
    S.pause(t,pos)
    check(t.pausePoint==anchor and t.index==index,'return-phase re-pause retains anchor')
    S.resume(t)
  end
  if t.mode=='complete' then break end
end
check(t.mode=='complete','mission completes')
check(near(maxTransit,340),'transit reaches 340')
check(near(maxScan,80),'scan reaches 80')
check(t.index==#plan.points+1,'all segments complete')
local v=S.update(t,pos,config,.05)
check(v[1]==0 and v[2]==0 and v[3]==0,'complete hover')

local D = dofile('dashboard.lua')
local dTask=S.start(plan); dTask.mode='scan'; dTask.index=3
local dPos={x=plan.points[2].x,y=120,z=plan.points[2].z}
local percent=D.progress(dTask,dPos)
S.pause(dTask,dPos)
check(D.progress(dTask,{x=10000,y=0,z=-10000})==percent,'paused display progress stays at scan anchor')
S.resume(dTask)
check(D.progress(dTask,{x=-10000,y=0,z=10000})==percent,'return transit cannot advance scan progress')
for _,mode in ipairs({'idle','scan','paused','climb_outbound','transit_center','resume_climb','resume_return','complete'}) do
  dTask.mode=mode
  local rows=D.lines({position=dPos},340,11,11,true,dTask,nil)
  check(#rows==4,'dashboard has four rows')
  for _,row in ipairs(rows) do check(#row<=15,'dashboard row fits 15 columns') end
end
check(D.progress(dTask,dPos)==100,'complete display progress')
check(D.lines({position=dPos},0,1,11,false,dTask,'full')[3]=='SAVE ERROR','save error visible')
local highDisplayPlan=R.build(mission(64,1),config); highDisplayPlan.altitude=560
local highDisplayTask=S.start(highDisplayPlan)
local highRows=D.lines({position={x=0,y=560,z=0}},0,2,11,true,highDisplayTask,nil)
check(highRows[2]:find('Y0560',1,true)~=nil,'current Y above 500 displayed')
check(highRows[4]:find('H560',1,true)~=nil,'cruise Y above 500 displayed')

-- CC filesystem/serialization mocks. All tests stay in memory.
local files={}
local failWrites=false
fs={exists=function(p) return files[p]~=nil end,
 combine=function(a,b) return a..'/'..b end,
 getDir=function(p) return p:match('^(.*)/') or '' end}
fs.open=function(p,mode)
  if mode=='r' then
    if not files[p] then return nil end
    return {readAll=function() return files[p] end,close=function() end}
  end
  if failWrites then error('disk full') end
  files[p]=''
  return {write=function(s) files[p]=files[p]..s end,close=function() end}
end
local function serialize(v)
  if type(v)=='table' then
    local out={'{'}
    for k,value in pairs(v) do out[#out+1]='['..serialize(k)..']='..serialize(value)..',' end
    out[#out+1]='}'; return table.concat(out)
  elseif type(v)=='string' then return string.format('%q',v)
  else return tostring(v) end
end
textutils={serialize=serialize,unserialize=function(s)
  local f=load('return '..s,'snapshot','t',{})
  if not f then return nil end
  local ok,v=pcall(f); return ok and v or nil
end}
local store,saved,corrupt=C.open('test',S.validate)
check(not saved and not corrupt,'new journal')
t=S.start(plan); t.mode='scan'; t.index=3
store.save(S.snapshot(t,pos))
t.index=4; store.save(S.snapshot(t,pos))
local _,latest=C.open('test',S.validate)
check(latest.index==4 and latest.mode=='paused','latest durable segment')
files['test.0']='truncated!'
local _,fallback=C.open('test',S.validate)
check(fallback.index==3,'torn snapshot fallback')
files['test.1']='truncated!'
local _,nothing,isCorrupt=C.open('test',S.validate)
check(not nothing and isCorrupt,'all-corrupt detection')
failWrites=true
check(not pcall(store.save,S.snapshot(t,pos)),'write error surfaced')
failWrites=false
local m=mission(100,2); m.x1=1e-8; m.x2=100+1e-8; m.z1=0; m.z2=100
R.save('mission',m); check(near(R.load('mission').x1,m.x1),'scientific notation roundtrip')
m.altitude=560; R.save('mission',m)
check(R.load('mission').altitude==560,'high cruise Y persists in mission settings')

-- Run the actual pilot with mocked rigid-body physics and cable inputs.
local clock,tick,body,vel,hook,steps
local commands={}
local logs={}
local displayRows={}
local cursor=1
local display={setCursorPos=function(x,y) check(x==1 and y>=1 and y<=4,'valid display cursor'); cursor=y end,
 clearLine=function() displayRows[cursor]='' end,
 write=function(s) check(#s<=15,'pilot display fits'); displayRows[cursor]=s end,
 update=function() check(#displayRows==4,'four display rows sent together') end}
print=function(s) logs[#logs+1]=tostring(s) end
term={clear=function() end,setCursorPos=function() end}
shell={getRunningProgram=function() return 'flight/pilot.lua' end}
os.clock=function() return clock end
peripheral={getNames=function() return {'sensor','display'} end,
 getType=function(name) return name=='sensor' and 'integrated_flight_sensor' or 'Create_DisplayLink' end,
 hasType=function(name,kind) return name=='sensor' and kind=='integrated_flight_sensor' end,
 wrap=function(name) if name=='display' then return display end
 return {getAllInputs=function() return commands[tick] or {} end} end}
createfc=setmetatable({}, {__index=function() return function() end end})
createfc.getBodyState=function()
  return {mass=10,vBody={x=vel[1],y=vel[2],z=vel[3]},
    position={x=body.x,y=body.y,z=body.z},bodyForward={x=0,y=0,z=-1}}
end
createfc.applyImpulseBody=function(x,y,z)
  vel={vel[1]+x/10,vel[2]+y/10,vel[3]+z/10}
end
sublevel={getLogicalPose=function() return {orientation={0,0,0,1}} end}
sleep=function(dt)
  if hook then hook(tick,body,vel) end
  body={x=body.x+vel[1]*dt,y=body.y+vel[2]*dt,z=body.z+vel[3]*dt}
  clock=clock+dt; tick=tick+1; coroutine.yield()
end
parallel={waitForAny=function(_,control)
  local co=coroutine.create(control)
  for _=1,steps do
    local ok,err=coroutine.resume(co); assert(ok,err)
  end
end}
local function run(n,newHook)
  tick,clock,steps,hook=0,0,n,newHook
  dofile('pilot.lua')
end
files={}; R.save('flight/mission.txt',mission(600,4))
body={x=0,y=120,z=0}; vel={0,0,0}
commands={[1]={[12]=1},[20]={[13]=1,[1]=1},[21]={[1]=1},[23]={[13]=1},
 [27]={[13]=1},[30]={[13]=1}}
local interrupted,pausedIndex
run(160,function(i,p,v)
  if i==20 then
    check(near(v[1],0) and near(v[2],0) and near(v[3],0),'X immediately cancels held motion')
    local _,s=C.open('flight/survey-progress',S.validate)
    interrupted=s.pausePoint; pausedIndex=s.index
  elseif i==21 then
    check(v[3]~=0,'manual movement while paused')
  elseif i==22 then
    p.x=p.x+900; p.y=40
  elseif i==27 then
    local _,s=C.open('flight/survey-progress',S.validate)
    check(near(s.pausePoint.x,interrupted.x) and near(s.pausePoint.z,interrupted.z),
      'pilot re-pause keeps original point')
    check(s.index==pausedIndex,'return phase preserves next segment')
  end
end)
local _,restart=C.open('flight/survey-progress',S.validate)
check(restart.mode=='paused' and restart.index>=pausedIndex,'shutdown saves progress')
local originalCount=#restart.plan.points
-- Reconfigure the mission; recovery must still use its original route.
R.save('flight/mission.txt',mission(64,1))
body={x=900,y=10,z=900}; vel={30,0,0}; commands={}
run(5,function(_,_,v)
  check(near(v[1],0) and near(v[2],0) and near(v[3],0),'restart hover until X')
end)
local _,restored=C.open('flight/survey-progress',S.validate)
check(#restored.plan.points==originalCount and restored.index==restart.index,
  'restart retains original route and segment index')
commands={[1]={[13]=1}}
run(2000)
local _,finished=C.open('flight/survey-progress',S.validate)
check(finished.mode=='complete','pilot completes recovered mission')
commands={[0]={[1]=1},[1]={[1]=1}}
run(3,function(_,_,v)
  check(v[1]==0 and v[2]==0 and v[3]==0,'completion ignores movement and hovers')
end)
-- Failed storage must pause before any new segment is flown.
files={}; R.save('flight/mission.txt',mission(600,4))
body={x=0,y=120,z=0}; vel={0,0,0}; commands={[1]={[12]=1}}
run(80,function(i,_,v)
  if i==10 then failWrites=true end
  if i==65 then
    check(near(v[1],0) and near(v[2],0) and near(v[3],0),'storage failure parks aircraft')
    failWrites=false
  end
end)
local _,stopped=C.open('flight/survey-progress',S.validate)
check(stopped.mode=='paused','write failure persists paused recovery after disk restored')
return checks
''')
print(f'PASS: {checks} assertions; Lua 5.2, coverage, state machine, journal, and pilot integration.')
