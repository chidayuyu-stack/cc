local args = {...}
local dir = fs.getDir(shell.getRunningProgram())
local config = dofile(fs.combine(dir, "config.lua"))
local mathx = dofile(fs.combine(dir, "core.lua"))
local routeLib = dofile(fs.combine(dir, "route.lua"))
local survey = dofile(fs.combine(dir, "survey.lua"))
local checkpoint = dofile(fs.combine(dir, "checkpoint.lua"))
local dashboard = dofile(fs.combine(dir, "dashboard.lua"))

assert(createfc, "Create Flight Control API 'createfc' is unavailable")
for _, method in ipairs({
  "getBodyState",
  "applyImpulseBody",
  "setBodyForwardFacing",
  "enableController",
  "setHover",
  "setHoverDamper",
  "setPointForceHover",
  "setAltitudeHold",
  "setLevelHold",
  "setVelocityTarget",
  "setAngularVelocityTarget",
  "setAngularGains",
}) do
  assert(type(createfc[method]) == "function", "createfc lacks " .. method)
end

local typewriter, typewriterName
local sensorBus, sensorBusName
local displayLink, displayName
local lastDisplayLine
local held = {}
local polled = {}
local previous = {
  gearUp = false,
  gearDown = false,
  autoStart = false,
  autoPause = false,
  virtualForce = false,
}
local gear = config.defaultGear
local running = true
local inputSource = "terminal"
local checkOnly = args[1] == "check"
local forceEnabled = false
local auto = survey.new()
local progressStore
local progressError
local lastSave = -math.huge
local lastPosition

local function saveProgress(position, now)
  if not auto.plan or not progressStore then return true end
  local ok, err = pcall(function()
    progressStore.save(survey.snapshot(auto, position))
  end)
  if ok then
    lastSave = now
    progressError = nil
  else
    progressError = tostring(err)
  end
  return ok
end

local function findAllPeripherals()
  for _, name in ipairs(peripheral.getNames()) do
    local kind = peripheral.getType(name)
    if kind == "linked_typewriter_hub"
      or peripheral.hasType(name, "linked_typewriter_hub")
      or peripheral.hasType(name, "linked_typewriter") then
      if not typewriter then
        typewriterName = name
        typewriter = peripheral.wrap(name)
      end
    elseif kind == "integrated_flight_sensor"
      or peripheral.hasType(name, "integrated_flight_sensor") then
      if not sensorBus then
        sensorBusName = name
        sensorBus = peripheral.wrap(name)
      end
    elseif (type(kind) == "string" and kind:lower() == "create_displaylink")
      or peripheral.hasType(name, "create_displaylink") then
      if not displayLink then
        local candidate = peripheral.wrap(name)
        if type(candidate.setCursorPos) == "function"
          and type(candidate.clearLine) == "function"
          and type(candidate.write) == "function"
          and type(candidate.update) == "function" then
          displayName = name
          displayLink = candidate
          lastDisplayLine = nil
        end
      end
    end
  end

  if typewriter and type(typewriter.setPromiscuousMode) == "function" then
    pcall(typewriter.setPromiscuousMode, true)
  end
end

local function fallbackBodyState()
  local state = createfc.getBodyState()
  assert(type(state) == "table", "createfc.getBodyState returned no state")
  assert(mathx.finite(state.mass) and state.mass > 0, "Invalid rigid-body mass")
  mathx.vec(state.vBody)
  mathx.vec(state.bodyForward)
  mathx.vec(state.position)
  return state
end

local function pollTypewriter()
  local current = {}
  if typewriter and type(typewriter.getPressedKeyCodes) == "function" then
    local ok, codes = pcall(typewriter.getPressedKeyCodes)
    if ok and type(codes) == "table" then
      for _, code in ipairs(codes) do current[tonumber(code)] = true end
    end
  end
  polled = current
end

local function readCableInputs()
  local result = {}
  if not sensorBus or type(sensorBus.getAllInputs) ~= "function" then
    return result
  end

  local ok, values = pcall(sensorBus.getAllInputs)
  if not ok or type(values) ~= "table" then return result end

  for name, channel in pairs(config.channels) do
    local value = tonumber(values[channel]) or 0
    result[name] = value > 0
  end
  return result
end

local function keyDown(codes, inputState, name)
  if inputState[name] then return true end
  for _, code in ipairs(codes) do
    if held[code] or polled[code] then return true end
  end
  return false
end

local function collectInput(cable)
  local input = {
    forward = keyDown(config.keyCodes.forward, cable, "forward"),
    backward = keyDown(config.keyCodes.backward, cable, "backward"),
    left = keyDown(config.keyCodes.left, cable, "left"),
    right = keyDown(config.keyCodes.right, cable, "right"),
    yawLeft = keyDown(config.keyCodes.yawLeft, cable, "yawLeft"),
    yawRight = keyDown(config.keyCodes.yawRight, cable, "yawRight"),
    up = keyDown(config.keyCodes.up, cable, "up"),
    down = keyDown(config.keyCodes.down, cable, "down"),
    gearUp = keyDown(config.keyCodes.gearUp, cable, "gearUp"),
    gearDown = keyDown(config.keyCodes.gearDown, cable, "gearDown"),
    virtualForce = keyDown(config.keyCodes.virtualForce, cable, "virtualForce"),
    autoStart = keyDown(config.keyCodes.autoStart, cable, "autoStart"),
    autoPause = keyDown(config.keyCodes.autoPause, cable, "autoPause"),
  }

  if input.gearUp and not input.gearDown then
    if not previous.gearUp then
      gear = mathx.nextGear(config, gear, true, false)
    end
  elseif input.gearDown and not input.gearUp then
    if not previous.gearDown then
      gear = mathx.nextGear(config, gear, false, true)
    end
  end
  previous.gearUp = input.gearUp
  previous.gearDown = input.gearDown
  input.gear = gear
  return input
end

local function activate()
  createfc.setBodyForwardFacing(config.forwardFacing)
  createfc.setLevelHold(false)
  createfc.setAltitudeHold(false)
  createfc.setPointForceHover(false)
  createfc.setHoverDamper(false)
  createfc.enableController(true)
  createfc.setHover(true)
  createfc.setAngularVelocityTarget(0, 0, 0)
  createfc.setAngularGains(
    config.angularGain[1],
    config.angularGain[2],
    config.angularGain[3]
  )
end

local function disableForce()
  createfc.setAngularVelocityTarget(0, 0, 0)
  createfc.setVelocityTarget(0, 0, 0)
  createfc.setHoverDamper(false)
  createfc.setHover(false)
  createfc.enableController(false)
end

local function setForceEnabled(enabled)
  forceEnabled = enabled
  if enabled then
    activate()
  else
    disableForce()
  end
end

local function safePark()
  if not forceEnabled then
    pcall(disableForce)
    return
  end
  pcall(function()
    local state = createfc.getBodyState()
    local velocity = mathx.vec(state.vBody)
    if mathx.finite(state.mass) and state.mass > 0 then
      createfc.applyImpulseBody(
        -velocity[1] * state.mass,
        -velocity[2] * state.mass,
        -velocity[3] * state.mass
      )
    end
    createfc.setAngularVelocityTarget(0, 0, 0)
    createfc.setVelocityTarget(0, 0, 0)
    createfc.enableController(true)
    createfc.setHover(true)
    createfc.setHoverDamper(true)
  end)
end

local function autoActive()
  return survey.active(auto)
end

local function autoLabel()
  if progressError then return "SAVE ERROR: " .. progressError end
  if auto.mode == "scan" and auto.plan then
    return string.format("SCAN %d/%d", auto.index - 1, #auto.plan.points - 1)
  end
  return auto.message
end

local function startAuto(state)
  local mission = routeLib.load(fs.combine(dir, config.missionFile))
  local plan = routeLib.build(mission, config)
  -- Persist the original immutable route before allowing motion.
  local pending = survey.start(plan)
  progressStore.save(survey.snapshot(pending, state.position))
  auto = pending
  lastSave = os.clock()
  setForceEnabled(true)
end

local function pauseAuto(state)
  if not autoActive() then return end
  survey.pause(auto, state.position)
  -- Brake before disk I/O. The caller discards its stale velocity sample.
  local velocity = mathx.vec(state.vBody)
  createfc.applyImpulseBody(-velocity[1] * state.mass,
    -velocity[2] * state.mass, -velocity[3] * state.mass)
  createfc.setAngularVelocityTarget(0, 0, 0)
  saveProgress(state.position, os.clock())
end

local function resumeAuto()
  if auto.mode ~= "paused" then return end
  assert(not progressError, "Resolve the checkpoint save error first")
  setForceEnabled(true)
  survey.resume(auto)
end

local function draw(state, input, target, actualSpeed)
  term.setCursorPos(1, 1)
  term.clear()
  print("VOIDCHAIR v1.0        LIMIT 340 m/s")
  print("W/S FWD/BACK  A/D LEFT/RIGHT")
  print("Q/E YAW       R/F UP/DOWN")
  print("SHIFT/CTRL    SPEED GEAR +/-")
  print("SPACE         VIRTUAL FORCE " .. (forceEnabled and "ON" or "OFF"))
  print("Z START AUTO  X PAUSE/RESUME")
  print("AUTO          " .. autoLabel())
  print(string.format("GEAR %2d/%d  SET %3.0f  NOW %6.1f",
    gear, #config.gears, mathx.gearSpeed(config, gear), actualSpeed))
  print(string.format("VBODY  X %6.1f  Y %6.1f  Z %6.1f",
    state.vBody.x, state.vBody.y, state.vBody.z))
  print(string.format("TARGET X %6.1f  Y %6.1f  Z %6.1f",
    target[1], target[2], target[3]))
  print(string.format("YAW RATE %6.0f deg/s  MASS %.1f",
    math.deg(mathx.yawTarget(config, input)), state.mass))
  print("TYPEWRITER " .. (typewriterName and "ON" or "OFF")
    .. "   CABLE " .. (sensorBusName and "ON" or "OFF"))
  print("CABLE 1..13 = ... 12=Z 13=X")
  if auto.plan then
    print(string.format("RADIUS %.0fm STEP %.1fm Y %.1f",
      auto.plan.halfWidth, auto.plan.scanStep, auto.plan.altitude))
  end
end

local function updateDisplay(state, actualSpeed)
  if not displayLink then return end

  local rows = dashboard.lines(state, actualSpeed, gear, #config.gears,
    forceEnabled, auto, progressError)
  local line = table.concat(rows, "\n")
  if line == lastDisplayLine then return end

  local ok = pcall(function()
    for i, row in ipairs(rows) do
      displayLink.setCursorPos(1, i)
      displayLink.clearLine()
      displayLink.write(row)
    end
    displayLink.update()
  end)
  if ok then
    lastDisplayLine = line
  else
    displayLink = nil
    displayName = nil
    lastDisplayLine = nil
  end
end

local function eventLoop()
  while running do
    local event, key = os.pullEventRaw()
    if event == "terminate" then
      running = false
      return
    elseif event == "peripheral_detach" then
      if key == typewriterName then
        typewriter = nil
        typewriterName = nil
        polled = {}
      elseif key == sensorBusName then
        sensorBus = nil
        sensorBusName = nil
      elseif key == displayName then
        displayLink = nil
        displayName = nil
        lastDisplayLine = nil
      end
    elseif event == "peripheral" then
      findAllPeripherals()
    elseif event == "key" then
      held[tonumber(key)] = true
    elseif event == "key_up" then
      held[tonumber(key)] = nil
    elseif type(event) == "string" and event:match("_key_up$") then
      held[tonumber(key)] = nil
    elseif type(event) == "string" and event:match("_key$") then
      held[tonumber(key)] = true
    end
  end
end

local function controlLoop()
  local lastDraw = -math.huge
  local forceInputInitialized = false
  local autoInputInitialized = false
  local lastTick = os.clock()
  while running do
    pollTypewriter()
    local cable = readCableInputs()
    local input = collectInput(cable)
    local state = fallbackBodyState()
    local pose = sublevel.getLogicalPose()
    local velocity = mathx.vec(state.vBody)
    local now = os.clock()
    local dt = math.max(config.controlPeriod, now - lastTick)
    lastTick = now
    lastPosition = state.position
    local pausedThisTick = false

    local forceToggleEdge = false
    if forceInputInitialized then
      forceToggleEdge = input.virtualForce and not previous.virtualForce
    else
      forceInputInitialized = true
    end
    previous.virtualForce = input.virtualForce

    local autoStartEdge, autoPauseEdge = false, false
    if autoInputInitialized then
      autoStartEdge = input.autoStart and not previous.autoStart
      autoPauseEdge = input.autoPause and not previous.autoPause
    else
      autoInputInitialized = true
    end
    previous.autoStart = input.autoStart
    previous.autoPause = input.autoPause

    if forceToggleEdge then
      if forceEnabled and autoActive() then
        pauseAuto(state)
        safePark()
      end
      setForceEnabled(not forceEnabled)
    end

    if autoStartEdge and (auto.mode == "idle" or auto.mode == "complete") then
      local ok, err = pcall(startAuto, state)
      if not ok then
        auto.message = "MISSION ERROR: " .. tostring(err)
      end
    end

    if autoPauseEdge then
      if auto.mode == "paused" then
        local ok, err = pcall(resumeAuto)
        if not ok then auto.message = "RESUME ERROR: " .. tostring(err) end
      elseif autoActive() then
        pauseAuto(state)
        velocity = {0, 0, 0}
        pausedThisTick = true
      end
    end

    local oldMode, oldIndex = auto.mode, auto.index
    local target
    if autoActive() then
      target = mathx.worldToBody(pose.orientation,
        survey.update(auto, state.position, config, dt))
    elseif auto.mode == "complete" or pausedThisTick then
      target = {0, 0, 0}
    else
      target = mathx.targetVelocity(config, state, input)
    end
    if auto.plan and (oldMode ~= auto.mode or oldIndex ~= auto.index
      or now - lastSave >= config.progressSavePeriod) then
      if not saveProgress(state.position, now) and autoActive() then
        survey.pause(auto, state.position)
        target = {0, 0, 0}
      end
    end
    local error = mathx.sub(target, velocity)
    local impulse = mathx.mul(error, state.mass)

    if forceEnabled then
      -- Direct velocity replacement is the "waterdrop" part of the controller.
      -- Hover independently removes gravity every physics step.
      if mathx.length(impulse) > 1e-6 then
        createfc.applyImpulseBody(impulse[1], impulse[2], impulse[3])
      end
      local yaw = autoActive() and 0 or mathx.yawTarget(config, input)
      createfc.setAngularVelocityTarget(0, yaw, 0)
    end

    if now - lastDraw >= config.displayPeriod then
      draw(state, input, target, mathx.length(velocity))
      updateDisplay(state, mathx.length(velocity))
      lastDraw = now
    end
    sleep(config.controlPeriod)
  end
end

local ok, err = pcall(function()
  findAllPeripherals()
  if checkOnly then
    local state = fallbackBodyState()
    print("PASS: createfc impulse and flight-state APIs found.")
    print("Mass: " .. state.mass)
    print("Forward facing: " .. tostring(state.bodyForwardFacing)
      .. " (flight config: " .. config.forwardFacing .. ")")
    print("Typewriter: " .. (typewriterName or "not connected"))
    print("Sensor bus: " .. (sensorBusName or "not connected"))
    print("Display link: " .. (displayName or "not connected"))
    print("Speed limit: 340 m/s; gears: " .. #config.gears)
    print("Auto mission: " .. (fs.exists(fs.combine(dir, config.missionFile))
      and "configured" or "missing; run flight/setup.lua"))
    print("Channels: 1-10 movement/gear, 11 Space, 12 Z, 13 X")
    return
  end

  setForceEnabled(false)
  local state = fallbackBodyState()
  lastPosition = state.position
  local saved, corrupt
  progressStore, saved, corrupt = checkpoint.open(
    fs.combine(dir, config.progressFile), survey.validate)
  if corrupt then
    setForceEnabled(true)
    safePark()
    error("Both survey checkpoints are unreadable. Restore a backup or remove "
      .. config.progressFile .. ".0/.1 to explicitly discard progress.", 0)
  end
  if saved then
    auto = saved
    auto.message = saved.mode == "complete" and "COMPLETE" or "SAVED TASK: X RESUME"
    setForceEnabled(true)
    safePark()
  end
  if typewriterName then
    inputSource = "typewriter"
  elseif sensorBusName then
    inputSource = "cable"
  end
  print("Waterdrop chair online via " .. inputSource .. ".")
  print("SPACE: force ON/OFF; saved tasks park until X. Ctrl+T stops.")
  parallel.waitForAny(eventLoop, controlLoop)
end)

running = false
if not checkOnly and lastPosition and auto.plan then
  local stateOK, finalState = pcall(fallbackBodyState)
  if stateOK then lastPosition = finalState.position end
  safePark()
  if autoActive() then survey.pause(auto, lastPosition) end
  saveProgress(lastPosition, os.clock())
end
if not ok then
  if not checkOnly then safePark() end
  error(tostring(err), 0)
end
if not checkOnly then
  local parkingEnabled = forceEnabled
  safePark()
  if parkingEnabled then
    print("Flight controller stopped. Hover/parking controller remains active.")
  else
    print("Flight controller stopped. Virtual force is OFF.")
  end
end
