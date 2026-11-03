-- Waterdrop flight chair / Create Flight Control 0.7.7 / CC:Tweaked.
-- Body axes: +X is local right, +Y is local up, -Z is the default
-- north-facing body forward. The chair itself faces south.
return {
  forwardFacing = "south",
  controlPeriod = 0.05,
  displayPeriod = 0.20,
  maxSpeed = 340,
  defaultGear = 2,
  gears = {0, 34, 68, 102, 136, 170, 204, 238, 272, 306, 340},

  -- The built-in angular controller uses the real Sable inertia tensor.
  -- Yaw is deliberately gentler than roll/pitch stabilization.
  yawRate = math.rad(90),
  angularGain = {20, 6, 20},

  -- Vertical motion is independent of horizontal gear changes.
  verticalSpeed = 18,

  -- Automatic survey task. Mission values are entered with setup.lua.
  missionFile = "mission.txt",
  progressFile = "survey-progress",
  progressSavePeriod = 2,
  scanSpeed = 80,
  transitSpeed = 340,
  scanOverlap = 0.90,
  waypointRadius = 0.25,
  altitudeTolerance = 0.5,
  altitudeGain = 3,
  waypointDwell = 0.15,

  -- GLFW key codes are what the linked typewriter sends. ComputerCraft's
  -- own keyboard codes are also included so the terminal remains usable.
  keyCodes = {
    forward  = {87, 17},   -- W
    backward = {83, 31},   -- S
    left     = {65, 30},   -- A
    right    = {68, 32},   -- D
    yawLeft  = {81, 16},   -- Q
    yawRight = {69, 18},   -- E
    up       = {82, 19},   -- R
    down     = {70, 33},   -- F
    gearUp   = {340, 344, 42, 54}, -- Left/Right Shift
    gearDown = {341, 345, 29, 157}, -- Left/Right Control
    virtualForce = {32, 57}, -- Space
    autoStart = {90, 44}, -- Z
    autoPause = {88, 45}, -- X
  },

  -- Optional Drive-By-Sable integrated flight sensor channels. Bind the
  -- linked-typewriter signals to these inputs if a cable bus is used.
  channels = {
    forward = 1,
    backward = 2,
    left = 3,
    right = 4,
    yawLeft = 5,
    yawRight = 6,
    up = 7,
    down = 8,
    gearUp = 9,
    gearDown = 10,
    virtualForce = 11,
    autoStart = 12,
    autoPause = 13,
  },
}
