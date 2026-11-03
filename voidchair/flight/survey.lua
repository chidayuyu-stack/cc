-- Survey state machine. No peripheral or filesystem access.
local S = {}
local phases = {climb_outbound = true, transit_center = true, scan = true}

function S.new()
  return {mode = "idle", index = 1, message = "AUTO IDLE"}
end

function S.active(task)
  return phases[task.mode] or task.mode == "resume_climb"
    or task.mode == "resume_return"
end

local function point(position)
  return {x = position.x, y = position.y, z = position.z}
end

function S.start(plan)
  return {mode = "climb_outbound", plan = plan, index = 1,
    message = "CLIMB CRUISE", dwellRemaining = 0}
end

function S.pause(task, position)
  if not S.active(task) then return end
  -- A second pause during recovery must retain the original scan anchor.
  if task.mode ~= "resume_climb" and task.mode ~= "resume_return" then
    task.pausePoint = point(position)
    task.pausePhase = task.mode
  end
  task.mode, task.message = "paused", "PAUSED"
end

function S.resume(task)
  if task.mode ~= "paused" then return end
  task.mode, task.message = "resume_climb", "RESUME CLIMB"
end

local function finite(n)
  return type(n) == "number" and n == n and math.abs(n) < 1e100
end

function S.validate(task)
  assert(type(task) == "table" and type(task.plan) == "table", "Missing saved route")
  local p = task.plan
  assert(finite(p.altitude) and type(p.points) == "table" and #p.points > 0,
    "Invalid saved route")
  for _, v in ipairs(p.points) do
    assert(type(v) == "table" and finite(v.x) and finite(v.z), "Invalid route point")
  end
  assert(finite(task.index) and task.index % 1 == 0 and task.index >= 1
    and task.index <= #p.points + 1, "Invalid saved progress")
  assert(task.mode == "paused" or task.mode == "complete", "Invalid checkpoint mode")
  assert(finite(task.dwellRemaining) and task.dwellRemaining >= 0, "Invalid saved dwell")
  if task.mode == "paused" then
    assert(phases[task.pausePhase], "Invalid saved phase")
    local v = task.pausePoint
    assert(type(v) == "table" and finite(v.x) and finite(v.y) and finite(v.z),
      "Invalid interruption point")
    assert(task.index <= #p.points, "Invalid unfinished waypoint")
  else
    assert(task.index == #p.points + 1, "Invalid completed progress")
  end
  return task
end

function S.snapshot(task, position)
  if not task.plan then return nil end
  local saved = {mode = task.mode, plan = task.plan, index = task.index,
    dwellRemaining = task.dwellRemaining or 0, pausePoint = task.pausePoint,
    pausePhase = task.pausePhase, message = task.message}
  if S.active(saved) then S.pause(saved, position) end
  return S.validate(saved)
end

local function horizontal(position, target, limit, config, dt)
  local dx, dz = target.x - position.x, target.z - position.z
  local distance = math.sqrt(dx * dx + dz * dz)
  if distance <= config.waypointRadius then return 0, 0, true end
  -- Cap each step to its remaining length; run at the fixed scan limit
  -- on straight sections and brake only when approaching a corner.
  local speed = math.min(limit, distance / math.max(dt, config.controlPeriod))
  return dx / distance * speed, dz / distance * speed, false
end

function S.update(task, position, config, dt)
  local plan = assert(task.plan, "Missing survey plan")
  local errorY = plan.altitude - position.y
  local atAltitude = math.abs(errorY) <= config.altitudeTolerance
  local vy = atAltitude and 0 or math.max(-config.verticalSpeed,
    math.min(config.verticalSpeed, errorY * config.altitudeGain))
  local vx, vz, arrived = 0, 0, false
  local mode = task.mode
  if mode == "climb_outbound" or mode == "resume_climb" then
    if atAltitude then
      task.mode = mode == "climb_outbound" and "transit_center" or "resume_return"
      task.message = mode == "climb_outbound" and "TO CENTER" or "RETURN TO PAUSE"
    end
  elseif mode == "transit_center" or mode == "resume_return" then
    vx, vz, arrived = horizontal(position,
      mode == "transit_center" and plan.points[1] or task.pausePoint,
      math.min(config.transitSpeed, config.maxSpeed), config, dt)
    if arrived and atAltitude then
      if mode == "transit_center" then
        task.index = 2
        task.mode = #plan.points > 1 and "scan" or "complete"
        task.dwellRemaining = config.waypointDwell
      else
        task.mode = task.pausePhase
      end
      task.message = task.mode == "complete" and "COMPLETE" or "SCAN"
      vx, vz = 0, 0
    end
  elseif mode == "scan" then
    if task.dwellRemaining > 0 then
      task.dwellRemaining = math.max(0, task.dwellRemaining - dt)
    else
      vx, vz, arrived = horizontal(position, assert(plan.points[task.index]),
        config.scanSpeed, config, dt)
      if arrived and atAltitude then
        task.index = task.index + 1
        task.dwellRemaining = config.waypointDwell
        if task.index > #plan.points then
          task.mode, task.message = "complete", "COMPLETE"
        end
        vx, vz = 0, 0
      end
    end
  elseif mode == "complete" or mode == "paused" then
    return {0, 0, 0}
  else
    error("Invalid survey phase: " .. tostring(mode))
  end
  local speed = math.sqrt(vx * vx + vy * vy + vz * vz)
  local limit = mode == "scan" and config.scanSpeed or config.maxSpeed
  if speed > limit then
    local scale = limit / speed
    vx, vy, vz = vx * scale, vy * scale, vz * scale
  end
  return {vx, vy, vz}
end

return S
