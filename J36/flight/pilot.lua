local args={...}
local dir=fs.getDir(shell.getRunningProgram())
local C=dofile(fs.combine(dir,'config.lua'))
local M=dofile(fs.combine(dir,'core.lua'))
local A=dofile(fs.combine(dir,'aero.lua'))
local bus,busName,displayLink,displayName,owned,basis,origin,model,shipId
local aeroModel,pointOrigin
local armed,quit=false,false
local throttle=0
local held,remotePrevious,blocked={},{},{}
local lastOutputs,previous={},nil
local message='R: engine ON/OFF. Throttle comes from the accumulator.'
local omega={0,0,0}
local rateCommand={0,0,0}
local rateTrim={0,0,0}
local speed=0
local altitude=0
local velocity={0,0,0}
local attitude={0,0,0}
local pressure,aeroForce,aeroTorque,aeroInverseTorque=0,{0,0,0},{0,0,0},{0,0,0}
local aeroLimited,authorityLimited=false,false
local surfaceLeft,surfaceRight=0,0
local rollSurfaceFactor=0
local mass,com,gravity,throttleCap
local function all(jobs) if #jobs>0 then parallel.waitForAll(table.unpack(jobs)) end end
local function cut(reason)
  armed=false;previous=nil;rateCommand={0,0,0}
  rateTrim={0,0,0}
  surfaceLeft=0;surfaceRight=0;rollSurfaceFactor=0
  message=reason or 'ENGINES OFF'
end
local function stop()
  if owned then bus.clear();lastOutputs={} end
end
local directions={east={1,0,0},west={-1,0,0},up={0,1,0},down={0,-1,0},south={0,0,1},north={0,0,-1}}
local faces={{0,1,0},{1,0,0},{-1,0,0},{0,0,1},{0,0,-1}}
local function same(a,b) return M.norm(M.sub(a,b))<0.01 end
local function setup()
  local found={}
  for _,n in ipairs(peripheral.getNames()) do
    if peripheral.hasType(n,C.busType) then found[#found+1]=n end
  end
  assert(#found==1,'Connect exactly one dedicated multi_channel_cable_bus')
  busName=found[1];bus=peripheral.wrap(busName)
  local displayLinks={}
  for _,name in ipairs(peripheral.getNames()) do
    local kind=peripheral.getType(name)
    if type(kind)=='string' and kind:lower()=='create_displaylink' then
      displayLinks[#displayLinks+1]=name
    end
  end
  assert(#displayLinks<=1,'Connect at most one display link')
  if #displayLinks==1 then
    displayName=displayLinks[1]
    displayLink=peripheral.wrap(displayName)
    for _,methodName in ipairs({'setCursorPos','clearLine','write','update'}) do
      assert(type(displayLink[methodName])=='function','Display link lacks '..methodName)
    end
  end
  for _,name in ipairs({'getAllInputs','getChannelStatus','setChannel','setInterceptAll','clear'}) do
    assert(type(bus[name])=='function','Cable bus lacks '..name)
  end
  local statuses,jobs={},{}
  for ch=1,20 do jobs[#jobs+1]=function() statuses[ch]=bus.getChannelStatus(ch) end end
  all(jobs)
  local first=statuses[1]
  assert(first.connectionCount==1,'Load backup drive first: output 1 requires one connection')
  local source={first.sourceX,first.sourceY,first.sourceZ}
  local forward=M.mul(M.sub(source,M.v(first.connections[1])),1/5)
  assert(math.abs(M.norm(forward)-1)<1e-6 and forward[2]==0 and
    (math.abs(forward[1])==1 or math.abs(forward[3])==1),'Unexpected aircraft layout')
  basis={forward,{0,1,0},M.cross(forward,{0,1,0})}
  origin=M.sub(source,M.toPlot(basis,C.busPosition))
  for _,e in ipairs(C.engines) do for i=1,5 do
    local ch=e.first+i-1
    local s=statuses[ch]
    assert(s.connectionCount==1 and #s.connections==1,'Wrong sink count on channel '..ch)
    local sink=s.connections[1]
    assert(same({s.sourceX,s.sourceY,s.sourceZ},source),'Bus source changed')
    assert(same(M.v(sink),M.add(origin,M.toPlot(basis,e.proxy))),'Wrong target on channel '..ch)
    assert(sink.sinkChannel=='' and directions[sink.face] and
      same(directions[sink.face],M.toPlot(basis,faces[i])),'Wrong face on channel '..ch)
  end end
  local gate=statuses[C.engineGate.channel]
  assert(gate.connectionCount==3 and #gate.connections==3,'Engine gate requires three connections on channel 16')
  local expected={}
  for _,f in ipairs({{0,1,0},{1,0,0},{-1,0,0}}) do
    local dir=M.toPlot(basis,f)
    for name,d in pairs(directions) do if same(d,dir) then expected[name]=true end end
  end
  for _,sink in ipairs(gate.connections) do
    assert(same(M.v(sink),M.add(origin,M.toPlot(basis,C.engineGate.position))) and
      sink.sinkChannel=='' and expected[sink.face],'Wrong engine gate target/face')
    expected[sink.face]=nil
  end
  assert(next(expected)==nil,'Incomplete engine gate')
  for _,surface in ipairs(C.surfaces) do
    for _,input in ipairs({surface.positive,surface.negative}) do
      local status=statuses[input.channel]
      assert(status.connectionCount==1 and #status.connections==1,
        'Control surface channel '..input.channel..' requires one connection')
      local sink=status.connections[1]
      assert(same(M.v(sink),M.add(origin,M.toPlot(basis,surface.position))) and
        same(directions[sink.face],M.toPlot(basis,input.face)),
        'Wrong control surface route on channel '..input.channel)
    end
  end
  assert(sublevel and aero and sublevel.isInPlotGrid(),'Assemble aircraft; CC:Sable sublevel/aero APIs are required')
  assert(type(aero.getAirPressure)=='function','CC:Sable aero.getAirPressure is unavailable')
  assert(type(aero.getGravity)=='function','CC:Sable aero.getGravity is unavailable')
  shipId=sublevel.getUniqueId()
  -- Block indices rotate about block centres; COM/force points are continuous.
  local half={0.5,0.5,0.5}
  pointOrigin=M.add(origin,M.sub(half,M.toPlot(basis,half)))
  local inv,pose
  all({function() mass=sublevel.getMass() end,
    function() com=M.toBody(basis,M.sub(M.v(sublevel.getCenterOfMass()),pointOrigin)) end,
    function() inv=M.bodyMatrix(basis,M.matrix(sublevel.getInverseInertiaTensor())) end,
    function() pose=sublevel.getLogicalPose() end})
  assert(M.finite(mass) and mass>0,'Invalid mass')
  gravity=M.v(aero.getGravity())
  local gravityMagnitude=M.norm(gravity)
  assert(M.finite(gravityMagnitude) and gravityMagnitude>0,'Invalid gravity')
  assert(com[1]>0 and com[1]<20 and com[2]>-2 and com[2]<6 and com[3]>0 and com[3]<21,'Unexpected COM/layout')
  assert(same(M.v(pose.scale),{1,1,1}),'Scaled aircraft unsupported')
  for i=1,3 do assert(inv[i][i]>0,'Invalid inverse inertia') end
  model=M.model(C,com,inv)
  model.inverseInertia=inv
  model.com=com
  throttleCap=M.clamp(C.targetThrustToWeight*mass*gravityMagnitude/model.total,0,1)
  aeroModel=M.aeroModel(A,com)
  local plotCom=M.add(pointOrigin,M.toPlot(basis,com))
  local worldCom=M.worldPoint(pose,plotCom)
  pressure=assert(aero.getAirPressure(vector.new(worldCom[1],worldCom[2],worldCom[3])),'Air pressure unavailable')
end
local function samplePressure(pose)
  local plotCom=M.add(pointOrigin,M.toPlot(basis,com))
  local worldCom=M.worldPoint(pose,plotCom)
  local value=aero.getAirPressure(vector.new(worldCom[1],worldCom[2],worldCom[3]))
  assert(M.finite(value) and value>=0,'Invalid air pressure')
  return value
end
local function inputs()
  local raw=bus.getAllInputs()
  local k={}
  for name,ch in pairs(C.inputs) do
    assert(M.finite(raw[ch]),'Invalid bus input '..ch)
    k[name]=raw[ch]>0
  end
  local level=raw[C.throttleInput]
  assert(M.finite(level) and level>=0 and level<=15 and level%1==0,'Invalid accumulator output')
  return k,level/15
end
local function terminalKeys(k)
  for name in pairs(C.inputs) do
    if keys[name] and held[keys[name]] and os.clock()-held[keys[name]]<0.8 then k[name]=true end
  end
end
local function readAttitude(p)
  return M.attitudeAngles(basis,p.orientation)
end
local function write(outputs,surfaceOutputs)
  local jobs={}
  for ch=1,15 do
    local value=armed and not quit and outputs[ch] or 0
    if value~=lastOutputs[ch] then
      jobs[#jobs+1]=function() bus.setChannel(ch,value);lastOutputs[ch]=value end
    end
  end
  for ch,value in pairs(surfaceOutputs) do
    local safe=armed and not quit and value or 0
    if safe~=lastOutputs[ch] then
      jobs[#jobs+1]=function() bus.setChannel(ch,safe);lastOutputs[ch]=safe end
    end
  end
  all(jobs)
  if not armed or quit then stop()
  elseif lastOutputs[C.engineGate.channel]~=15 then
    -- Load nozzle commands while the engine gate is still OFF, then enable.
    bus.setChannel(C.engineGate.channel,15);lastOutputs[C.engineGate.channel]=15
    if not armed or quit then stop() end
  end
end
local function draw()
  term.setCursorPos(1,1);term.clear()
  print('THREE-ENGINE FIGHTER 2.4  '..(armed and 'ON' or 'OFF'))
  print('R: toggle engine ON/OFF')
  print('W/S: nose down/up   Q/E: yaw left/right')
  print('A/D: roll left/right')
  print('Typewriter Left Shift/Ctrl: throttle +/-')
  print(string.format('Accumulator %2d/15 (%3d%%)   Mass %.1f',math.floor(throttle*15+0.5),math.floor(throttle*100+0.5),mass))
  print(string.format('TWR cap %.2f   Max throttle %.1f%%',C.targetThrustToWeight,throttleCap*100))
  print(string.format('Speed %.1f m/s   Limit %.0f m/s',speed,C.maxSpeed))
  print(string.format('Pitch %5.1f  Roll %5.1f  Heading %5.1f deg',math.deg(attitude[2]),math.deg(attitude[1]),math.deg(attitude[3])))
  print(string.format('Velocity %.1f %.1f %.1f  Speed %.1f',velocity[1],velocity[2],velocity[3],speed))
  print(string.format('Rates deg/s  roll %6.1f yaw %6.1f pitch %6.1f',math.deg(omega[1]),-math.deg(omega[2]),math.deg(omega[3])))
  print(string.format('Surfaces deg  left %5.1f  right %5.1f  authority %3d%%',
    surfaceLeft,surfaceRight,math.floor(rollSurfaceFactor*100+0.5)))
  print(string.format('Aero p %.3f F %.0f %.0f %.0f T %.0f %.0f %.0f',pressure,
    aeroForce[1],aeroForce[2],aeroForce[3],aeroTorque[1],aeroTorque[2],aeroTorque[3]))
  print(message)
  print('Static sails add drag; lift scaling remains zero in Sable.')
  if displayLink then
    local gear=math.floor(throttle*15+0.5)
    local line=string.format('GEAR %02d/15   ALT %7.1f   SPD %6.1f m/s',
      gear,altitude,speed)
    local ok=pcall(function()
      displayLink.setCursorPos(1,1)
      displayLink.clearLine()
      displayLink.write(line)
      displayLink.update()
    end)
    if not ok then
      displayLink=nil
      message='Dashboard link lost'
    end
  end
end
local function events()
  while not quit do
    local event,key,repeated=os.pullEventRaw()
    if event=='terminate' then cut('Stopped');quit=true;return end
    if event=='peripheral_detach' and key==busName then error('Cable bus detached',0) end
    if event=='peripheral_detach' and key==displayName then displayLink=nil end
    if event=='key' then
      held[key]=os.clock()
    elseif event=='key_up' then held[key]=nil end
  end
end
local function controller()
  local last=os.clock();local lastDraw=-math.huge
  local firstKeys=inputs()
  for name,value in pairs(firstKeys) do if value then blocked[name]=true end end
  while not quit do
    local k,rate,pose
    all({function() k,throttle=inputs() end,function() rate=M.v(sublevel.getAngularVelocity()) end,
      function() velocity=M.v(sublevel.getLinearVelocity()) end,function() pose=sublevel.getLogicalPose() end})
    local now=os.clock();local dt=now-last;last=now
    assert(dt>=0 and dt<0.6,'Control cycle stalled; restart controller')
    dt=math.max(dt,0.001)
    throttle=math.min(throttle,throttleCap)
    -- CC:Sable 1.3.4 derives this from inverse(last orientation)*current.
    -- It is already expressed on the rigid body's axes.
    omega=M.v(rate)
    attitude=readAttitude(pose)
    speed=M.norm(velocity)
    altitude=M.v(pose.position)[2]
    pressure=samplePressure(pose)
    local velocityPlot=M.rotate(M.qconj(pose.orientation),velocity)
    local velocityBody=M.toBody(basis,velocityPlot)
    aeroForce,aeroTorque=M.aeroWrench(aeroModel,pressure,velocityBody,omega)
    terminalKeys(k)
    for name in pairs(blocked) do if not k[name] then blocked[name]=nil end end
    local toggleEdge=k.r and not remotePrevious.r and not blocked.r
    if toggleEdge then
      if armed then cut('R: engines OFF; accumulator setting retained')
      else
        previous=nil;rateCommand={0,0,0};armed=true
        rateTrim={0,0,0}
        message='R: engines ON at accumulator throttle'
      end
    end
    remotePrevious=k
    if armed then
      local inputRate=M.targetRate(C,k)
      rateCommand=M.slewRate(C,rateCommand,inputRate,dt)
      local alpha=M.rateAcceleration(C,rateCommand,omega,rateTrim,dt)
      local thrustAlpha
      thrustAlpha,aeroInverseTorque,aeroLimited=M.aeroAdjustedAlpha(
        C,alpha,aeroTorque,model.inverseInertia,A)
      local blended
      blended,rollSurfaceFactor=M.rollThrustBlend(C,A,speed,thrustAlpha)
      thrustAlpha=blended
      local driveThrottle=math.min(throttle,throttleCap)
      driveThrottle=driveThrottle*M.speedThrottleFactor(C,speed)
      local forceTarget=M.aeroForceTarget(model,driveThrottle,aeroForce,A)
      previous=M.allocate(model,driveThrottle,thrustAlpha,previous,forceTarget)
      local outputs=M.commands(model,driveThrottle,thrustAlpha,previous,forceTarget)
      local surfaceOutputs
      surfaceOutputs,surfaceLeft,surfaceRight=M.surfaceChannels(C,alpha,
        rateCommand,speed,surfaceLeft,surfaceRight,dt)
      local _,realisedAlpha=M.realised(model,outputs)
      authorityLimited=M.norm(M.sub(realisedAlpha,thrustAlpha))>A.authorityWarningThreshold
      if aeroLimited then message='AERO: compensation reaches command limit'
      elseif authorityLimited then message='AERO: thruster authority saturated'
      elseif speed>=C.maxSpeed then
        message='ENVELOPE: speed limit; hold SPACE to brake'
      elseif message:find('AERO:',1,true) or message:find('ENVELOPE:',1,true) then
        message='Engines active'
      end
      write(outputs,surfaceOutputs)
    elseif next(lastOutputs)~=nil then stop() end
    if now-lastDraw>=0.25 then draw();lastDraw=now end
    sleep(C.period)
  end
end
local ok,err=pcall(function()
  setup()
  if args[1]=='check' then
    print('PASS: 15 thruster outputs, 4 control-surface outputs and engine gate routes match.')
    print('Bus: '..busName..'  Ship: '..shipId)
    print(string.format('Mass %.2f; blueprint COM %.3f %.3f %.3f',mass,com[1],com[2],com[3]))
    print(string.format('Gravity %.2f; TWR cap %.2f; max throttle %.1f%%',
      M.norm(gravity),C.targetThrustToWeight,throttleCap*100))
    print(string.format('Speed limit: fixed %.0f m/s',C.maxSpeed))
    print('Display link: '..(displayLink and displayName or 'not connected'))
    print(string.format('Aero model: %d sails; pressure %.3f',aeroModel.sailCount,pressure))
    local _,level=inputs()
    print(string.format('Accumulator: %d/15',math.floor(level*15+0.5)))
    local rate=M.v(sublevel.getAngularVelocity())
    local vel=M.v(sublevel.getLinearVelocity())
    local att=readAttitude(sublevel.getLogicalPose())
    print(string.format('Attitude: pitch %.1f roll %.1f heading %.1f deg',
      math.deg(att[2]),math.deg(att[1]),math.deg(att[3])))
    print(string.format('Velocity: %.3f %.3f %.3f',vel[1],vel[2],vel[3]))
    print(string.format('Angular velocity: %.3f %.3f %.3f rad/s',rate[1],rate[2],rate[3]))
    print('Read-only live Sub-Level API check. Fuel needs an in-game check.')
    return
  end
  owned=true
  bus.setInterceptAll(true) -- A dedicated bus: external inputs cannot override outputs.
  stop()
  parallel.waitForAny(events,controller)
end)
armed=false;quit=true
local stopped,stopError=pcall(stop)
if not stopped then printError('Could not clear bus: '..tostring(stopError)..'; engine state could not be confirmed') end
if owned then print('Flight controller stopped.') end
if not ok then error(tostring(err),0) end
