-- Pure aircraft-frame math. No world or peripheral writes.
local M={}
function M.finite(x) return type(x)=='number' and x==x and math.abs(x)<1e100 end
function M.clamp(x,a,b) assert(M.finite(x),'Invalid number'); return math.max(a,math.min(b,x)) end
function M.v(t)
  assert(type(t)=='table','Missing vector')
  local r={t.x or t[1],t.y or t[2],t.z or t[3]}
  for i=1,3 do assert(M.finite(r[i]),'Invalid vector') end
  return r
end
function M.add(a,b) return {a[1]+b[1],a[2]+b[2],a[3]+b[3]} end
function M.sub(a,b) return {a[1]-b[1],a[2]-b[2],a[3]-b[3]} end
function M.mul(a,s) return {a[1]*s,a[2]*s,a[3]*s} end
function M.dot(a,b) return a[1]*b[1]+a[2]*b[2]+a[3]*b[3] end
function M.cross(a,b) return {a[2]*b[3]-a[3]*b[2],a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]} end
function M.norm(a) return math.sqrt(M.dot(a,a)) end
function M.q(t)
  local q=t.v and {t.v.x,t.v.y,t.v.z,t.a} or {t.x or t[1],t.y or t[2],t.z or t[3],t.w or t[4]}
  local norm=0
  for i=1,4 do assert(M.finite(q[i]),'Invalid orientation');norm=norm+q[i]^2 end
  assert(norm>1e-12,'Zero quaternion')
  norm=math.sqrt(norm)
  for i=1,4 do q[i]=q[i]/norm end
  return q
end
function M.rotate(q,v)
  local cross=M.cross({q[1],q[2],q[3]},v)
  local twice=M.mul(cross,2)
  return M.add(v,M.add(M.mul(twice,q[4]),M.cross({q[1],q[2],q[3]},twice)))
end
function M.qconj(q)
  q=M.q(q)
  return {-q[1],-q[2],-q[3],q[4]}
end
function M.worldPoint(pose,point)
  local q=M.q(pose.orientation)
  local delta=M.sub(point,M.v(pose.rotationPoint))
  return M.add(M.v(pose.position),M.rotate(q,delta))
end
function M.attitudeAngles(basis,q)
  q=M.q(q)
  local forward=M.rotate(q,basis[1])
  local up=M.rotate(q,basis[2])
  local right=M.rotate(q,basis[3])
  return {
    math.atan2(-right[2],up[2]), -- positive is right-wing down
    math.asin(M.clamp(forward[2],-1,1)),
    math.atan2(forward[3],forward[1]),
  }
end
function M.matvec(a,v) return {M.dot(a[1],v),M.dot(a[2],v),M.dot(a[3],v)} end
function M.matrix(a)
  assert(type(a)=='table','Missing inertia')
  local r={}
  for i=1,3 do
    r[i]={}
    for j=1,3 do assert(a[j] and M.finite(a[j][i]),'Invalid inertia');r[i][j]=a[j][i] end
  end -- CCSableUtils exposes JOML column first.
  return r
end
function M.toBody(b,v) return {M.dot(b[1],v),M.dot(b[2],v),M.dot(b[3],v)} end
function M.toPlot(b,v) return M.add(M.add(M.mul(b[1],v[1]),M.mul(b[2],v[2])),M.mul(b[3],v[3])) end
function M.bodyMatrix(b,a)
  local r={{},{},{}}
  for i=1,3 do for j=1,3 do r[i][j]=M.dot(b[i],M.matvec(a,b[j])) end end
  return r
end
function M.targetRate(c,k)
  local function down(n) return k[n] and 1 or 0 end
  return {(down('d')-down('a'))*c.rate[1],
    (down('q')-down('e'))*c.rate[2],
    (down('s')-down('w'))*c.rate[3]}
end
function M.slewRate(c,current,target,dt)
  local out={current[1],current[2],current[3]}
  dt=math.max(dt or 0,0)
  for i=1,3 do
    local step=c.rateSlew[i]*dt
    out[i]=out[i]+M.clamp(target[i]-out[i],-step,step)
  end
  return out
end
function M.rateAcceleration(c,target,omega,trim,dt)
  trim=trim or {0,0,0}
  dt=math.max(dt or 0,0)
  local a={}
  for i=1,3 do
    local e=target[i]-omega[i]
    local deadband=target[i]==0 and c.rateDeadband or c.rateTrackDeadband
    if deadband and math.abs(e)<deadband then e=0 end
    local previous=trim[i]
    if target[i]==0 then
      trim[i]=trim[i]*math.max(0,1-dt/0.5)
    elseif e~=0 then
      local limit=c.acceleration[i]*c.integralLimit
      trim[i]=M.clamp(trim[i]+c.integralGain[i]*e*dt,-limit,limit)
    end
    local requested=c.gain[i]*e+trim[i]
    if math.abs(requested)>c.acceleration[i] and requested*e>0 then
      trim[i]=previous
      requested=c.gain[i]*e+trim[i]
    end
    a[i]=M.clamp(requested,-c.acceleration[i],c.acceleration[i])
  end
  return a
end
-- Euclidean projection onto a forward circular cone intersected with a ball.
function M.project(v,angle,cap)
  local x,y,z=v[1],v[2],v[3]
  local lateral=math.sqrt(y*y+z*z)
  local ca,sa=math.cos(angle),math.sin(angle)
  if lateral*ca>x*sa then
    local length=math.max(0,x*ca+lateral*sa)
    x=length*ca
    local scale=lateral>1e-12 and length*sa/lateral or 0
    y,z=y*scale,z*scale
  end
  local length=math.sqrt(x*x+y*y+z*z)
  local scale=length>cap and cap/length or 1
  return {x*scale,y*scale,z*scale}
end
function M.model(c,com,inverseInertia)
  local model={columns={},angles={},total=0,config=c}
  for _,e in ipairs(c.engines) do model.total=model.total+e.capacity end
  -- Normalised force and angular acceleration objectives. Translational side
  -- force is deliberately weak: tail-vector pitch/yaw necessarily produce it.
  model.weights={1,0.03,0.03,2,2,2}
  model.lipschitz=0.002
  for j,e in ipairs(c.engines) do
    model.angles[j]=math.rad(e.controlCone)
    for axis=1,3 do
      local force={0,0,0};force[axis]=e.capacity
      local a=M.matvec(inverseInertia,M.cross(M.sub(e.position,com),force))
      local col={force[1]/model.total,force[2]/model.total,force[3]/model.total,
        a[1]/c.acceleration[1],a[2]/c.acceleration[2],a[3]/c.acceleration[3]}
      for i=1,6 do col[i]=col[i]*model.weights[i];model.lipschitz=model.lipschitz+col[i]^2 end
      model.columns[#model.columns+1]=col
    end
  end
  return model
end
local function targetVector(model,throttle,alpha,forceTarget)
  local c=model.config
  throttle=M.clamp(throttle,0,1)
  local force=forceTarget or {throttle,0,0}
  if forceTarget then
    local length=math.sqrt(force[1]^2+force[2]^2+force[3]^2)
    if length>1 then force=M.mul(force,1/length) end
  end
  local target={force[1],force[2],force[3],alpha[1]/c.acceleration[1],
    alpha[2]/c.acceleration[2],alpha[3]/c.acceleration[3]}
  for i=1,6 do target[i]=target[i]*model.weights[i] end
  return target
end
function M.aeroForceTarget(model,throttle,aeroForce,aeroConfig)
  local side=M.clamp(-aeroConfig.sideslipCompensation*aeroForce[3]/model.total,-1,1)
  return {throttle,0,side}
end
function M.allocate(model,throttle,alpha,previous,forceTarget)
  local c=model.config
  local target=targetVector(model,throttle,alpha,forceTarget)
  local x,y={},{}
  for i=1,9 do x[i]=previous and previous[i] or (i%3==1 and throttle or 0);y[i]=x[i] end
  local t=1
  for step=1,c.solverIterations do
    local residual={}
    for i=1,6 do
      residual[i]=-target[i]
      for j=1,9 do residual[i]=residual[i]+model.columns[j][i]*y[j] end
    end
    local nextX={}
    for j=1,9 do
      local grad=0.002*y[j]
      for i=1,6 do grad=grad+model.columns[j][i]*residual[i] end
      nextX[j]=y[j]-grad/model.lipschitz
    end
    for j=1,3 do
      local k=(j-1)*3
      local v=M.project({nextX[k+1],nextX[k+2],nextX[k+3]},model.angles[j],throttle)
      for i=1,3 do nextX[k+i]=v[i] end
    end
    local nextT=(1+math.sqrt(1+4*t*t))/2
    for j=1,9 do y[j]=nextX[j]+(t-1)/nextT*(nextX[j]-x[j]) end
    x,t=nextX,nextT
  end
  return x
end
local function round(x) return math.floor(x+0.5) end
function M.encode(e,f)
  local n=M.norm(f)
  local power=round(M.clamp(n,0,1)*15)
  if power==0 then return {0,0,0,0,0} end
  local lat=math.sqrt(f[2]^2+f[3]^2)
  local up,right=0,0
  if lat>1e-10 then
    local angle=math.acos(M.clamp(f[1]/n,-1,1))
    local scale=15*angle/math.rad(e.cone)/lat
    up=round(math.abs(f[2]*scale))*(f[2]>=0 and 1 or -1)
    right=round(math.abs(f[3]*scale))*(f[3]>=0 and 1 or -1)
  end
  return {power,M.clamp(up,0,15),M.clamp(-up,0,15),M.clamp(-right,0,15),M.clamp(right,0,15)}
end
function M.decode(e,s)
  local h,v=(s[5]-s[4])/15,(s[2]-s[3])/15
  local n=math.sqrt(h*h+v*v)
  local a=math.rad(e.cone)*math.min(1,n)
  local scale=n>1e-12 and math.sin(a)/n or 0
  return M.mul({math.cos(a),v*scale,h*scale},s[1]/15)
end
function M.channels(c,x)
  local out={}
  for j,e in ipairs(c.engines) do
    local k=(j-1)*3
    local s=M.encode(e,{x[k+1],x[k+2],x[k+3]})
    for i=1,5 do out[e.first+i-1]=s[i] end
  end
  return out
end
-- Joint integer refinement: independent rounding wastes the main nozzle's
-- finer 2-degree steps and the side nozzles' 6-degree steps. Optimise the
-- realised wrench after quantisation, preserving the throttle cap.
function M.commands(model,throttle,alpha,x,forceTarget)
  local c=model.config
  local target=targetVector(model,throttle,alpha,forceTarget)
  local output=M.channels(c,x)
  local residual={};for i=1,6 do residual[i]=-target[i] end
  local contribution={}
  local function effect(j,s)
    local v=M.decode(c.engines[j],s);local r={}
    for i=1,6 do
      r[i]=0
      for a=1,3 do r[i]=r[i]+model.columns[(j-1)*3+a][i]*v[a] end
    end
    return r
  end
  for j,e in ipairs(c.engines) do
    local s={};for i=1,5 do s[i]=output[e.first+i-1] end
    contribution[j]=effect(j,s)
    for i=1,6 do residual[i]=residual[i]+contribution[j][i] end
  end
  for sweep=1,3 do for j,e in ipairs(c.engines) do
    local start=e.first
    local p=output[start]
    local v=output[start+1]-output[start+2]
    local h=output[start+4]-output[start+3]
    local best,bestEffect,bestCost=nil,nil,math.huge
    for dp=-1,1 do for dv=-1,1 do for dh=-1,1 do
      local np,nv,nh=p+dp,v+dv,h+dh
      if np>=0 and np<=math.floor(throttle*15+0.5) and math.abs(nv)<=15 and math.abs(nh)<=15
        and math.sqrt(nv*nv+nh*nh)<=15*e.controlCone/e.cone+0.75 then
        local s={np,math.max(nv,0),math.max(-nv,0),math.max(-nh,0),math.max(nh,0)}
        if np==0 then s={0,0,0,0,0} end
        local a=effect(j,s)
        local cost=0.00002*(np/15)^2
        for i=1,6 do cost=cost+(residual[i]-contribution[j][i]+a[i])^2 end
        if cost<bestCost then best,bestEffect,bestCost=s,a,cost end
      end
    end end end
    if best then
      for i=1,6 do residual[i]=residual[i]-contribution[j][i]+bestEffect[i] end
      contribution[j]=bestEffect
      for i=1,5 do output[start+i-1]=best[i] end
    end
  end end
  return output
end
function M.realised(model,outputs)
  local c=model.config
  local force,torque={0,0,0},{0,0,0}
  for j,e in ipairs(c.engines) do
    local s={};for i=1,5 do s[i]=outputs[e.first+i-1] end
    local f=M.mul(M.decode(e,s),e.capacity)
    local t=M.cross(M.sub(e.position,model.com),f)
    force=M.add(force,f);torque=M.add(torque,t)
  end
  return force,M.matvec(model.inverseInertia,torque)
end
-- Compress the rigidly attached Sable lift providers into a body-frame
-- damping matrix D: [force;torque] = -air_pressure * D * [velocity;omega].
function M.aeroModel(A,com)
  assert(type(A)=='table' and type(A.sails)=='table','Missing aero geometry')
  assert(#A.sails==A.sailCount,'Sail geometry changed; regenerate aero.lua')
  local model={matrix={},sailCount=#A.sails,config=A}
  for row=1,6 do model.matrix[row]={} end
  for row=1,6 do for col=1,6 do model.matrix[row][col]=0 end end
  local unit={{1,0,0},{0,1,0},{0,0,1}}
  for _,sail in ipairs(A.sails) do
    local position=M.v(sail.position)
    local normal=M.v(sail.normal)
    local r=M.sub(position,com)
    for input=1,6 do
      local velocity
      if input<=3 then velocity=unit[input]
      else velocity=M.cross(unit[input-3],r) end
      local normalVelocity=M.dot(normal,velocity)
      local force={}
      for axis=1,3 do
        force[axis]=A.parallelDrag*normal[axis]*normalVelocity
          +A.directionlessDrag*velocity[axis]
      end
      local torque=M.cross(r,force)
      for axis=1,3 do
        model.matrix[axis][input]=model.matrix[axis][input]+force[axis]
        model.matrix[axis+3][input]=model.matrix[axis+3][input]+torque[axis]
      end
    end
  end
  return model
end
function M.aeroWrench(aero,pressure,velocity,omega)
  assert(M.finite(pressure) and pressure>=0,'Invalid air pressure')
  local state={velocity[1],velocity[2],velocity[3],omega[1],omega[2],omega[3]}
  local out={}
  for row=1,6 do
    local sum=0
    for col=1,6 do sum=sum+aero.matrix[row][col]*state[col] end
    out[row]=-pressure*sum
  end
  return {out[1],out[2],out[3]},{out[4],out[5],out[6]}
end
function M.aeroAdjustedAlpha(c,alpha,torque,inverseInertia,aeroConfig)
  local inverseTorque=M.matvec(inverseInertia,torque)
  local correction={}
  local gain=aeroConfig.torqueCompensation
  local limited=false
  for i=1,3 do
    local max=c.acceleration[i]*aeroConfig.maxAccelerationScale
    local requested=alpha[i]-gain*inverseTorque[i]
    correction[i]=M.clamp(requested,-max,max)
    if math.abs(requested-correction[i])>1e-9 then limited=true end
  end
  return correction,inverseTorque,limited
end
function M.surfaceSpeedFactor(c,speed)
  local control=c.surfaceControl
  return M.clamp((speed-control.surfaceEngageSpeed)/
    (control.surfaceFullSpeed-control.surfaceEngageSpeed),0,1)
end
function M.surfaceAngles(c,alpha,targetRate,speed,previousLeft,previousRight,dt)
  local control=c.surfaceControl
  -- Active response surfaces follow the pilot input. On release they return
  -- neutral; the rate damper remains on the vectored thrusters.
  local pitch=targetRate and targetRate[3]~=0
    and M.clamp(alpha[3]/c.acceleration[3]*control.pitchGain,-1,1) or 0
  local roll=targetRate and targetRate[1]~=0
    and M.clamp(alpha[1]/c.acceleration[1]*control.rollGain,-1,1) or 0
  local speedFactor=M.surfaceSpeedFactor(c,speed or 0)
  pitch,roll=pitch*speedFactor,roll*speedFactor
  local left=(control.leftPitchSign*pitch+control.leftRollSign*roll)*control.maxAngle
  local right=(control.rightPitchSign*pitch+control.rightRollSign*roll)*control.maxAngle
  left=M.clamp(left,-control.maxAngle,control.maxAngle)
  right=M.clamp(right,-control.maxAngle,control.maxAngle)
  local step=math.rad(control.commandRate)*math.max(dt or 0,0)
  left=(previousLeft or 0)+M.clamp(left-(previousLeft or 0),-step,step)
  right=(previousRight or 0)+M.clamp(right-(previousRight or 0),-step,step)
  return left,right
end
local function angleSignals(spec,angle,maxAngle)
  local amount=math.floor(math.abs(angle)/maxAngle*15+0.5)
  if angle>=0 then
    return {[spec.positive.channel]=amount,[spec.negative.channel]=0}
  end
  return {[spec.positive.channel]=0,[spec.negative.channel]=amount}
end
function M.surfaceChannels(c,alpha,targetRate,speed,previousLeft,previousRight,dt)
  local left,right=M.surfaceAngles(c,alpha,targetRate,speed,
    previousLeft,previousRight,dt)
  local out={}
  for name,signals in pairs(angleSignals(c.surfaces[1],left,c.surfaceControl.maxAngle)) do
    out[name]=signals
  end
  for name,signals in pairs(angleSignals(c.surfaces[2],right,c.surfaceControl.maxAngle)) do
    out[name]=signals
  end
  return out,left,right
end
function M.rollThrustBlend(c,aeroConfig,speed,alpha)
  local control=c.surfaceControl
  local factor=M.surfaceSpeedFactor(c,speed)
  local rollReduction=control.rollThrustReduction*factor
  local pitchReduction=control.pitchThrustReduction*factor
  return {alpha[1]*(1-rollReduction),alpha[2],
    alpha[3]*(1-pitchReduction)},factor,rollReduction
end
function M.speedThrottleFactor(c,speed)
  return M.clamp((c.maxSpeed-speed)/c.speedMargin,0,1)
end
return M
