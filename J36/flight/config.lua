-- J36 v1.1 / Three-engine fighter / CFC 0.7.7 / Drive-By-Sable r2.0.3.
return {
  busType = 'multi_channel_cable_bus',
  busPosition = {10,1,9},
  -- Aircraft axes: +X nose, +Y up, +Z right. Positions include force offsets.
  engines = {
    {name='main', position={0.5,1.5,9.5}, capacity=2224.5, cone=30, controlCone=30, proxy={5,1,9}, first=1},
    {name='left', position={0.5,1.5,8.5}, capacity=741.5, cone=90, controlCone=75, proxy={6,1,9}, first=6},
    {name='right',position={0.5,1.5,10.5},capacity=741.5, cone=90, controlCone=75, proxy={11,1,9},first=11},
  },
  -- Inputs arrive from the existing typewriter via cable module sinks.
  inputs={w=21,s=22,a=23,d=24,q=25,e=26,r=27},
  throttleInput=28, -- Actual redstone_accumulator world output, 0..15.
  engineGate={channel=16,position={12,1,9}},
  -- Advanced tilt adapters: one bipolar angle per wing. Positive and
  -- negative input sides are wired separately because their angle limits
  -- are independently configured.
  surfaces={
    {name='left', position={2,1,7}, positive={channel=17,face={1,0,0}},
      negative={channel=18,face={-1,0,0}}},
    {name='right',position={2,1,11},positive={channel=19,face={-1,0,0}},
      negative={channel=20,face={1,0,0}}},
  },
  surfaceControl={
    maxAngle=45,
    pitchGain=1.25,
    rollGain=1.2,
    -- The right adapter is reflected across Z, so its output angle is
    -- opposite. Pitch is common-mode; roll is differential.
    leftPitchSign=1,rightPitchSign=-1,
    leftRollSign=-1,rightRollSign=-1,
    surfaceEngageSpeed=3,
    surfaceFullSpeed=18,
    rollThrustReduction=0.75,
    pitchThrustReduction=0.30,
    commandRate=180, -- Maximum commanded surface travel, degrees/second.
  },
  targetThrustToWeight=2.5,
  maxSpeed=340,
  speedMargin=20,
  period=0.05,
  rate={math.rad(60),math.rad(25),math.rad(40)}, -- roll, yaw, pitch
  rateSlew={math.rad(360),math.rad(240),math.rad(300)}, -- Command ramp, deg/s^2.
  gain={2.0,1.6,2.6},
  integralGain={0.25,0.3,0.2},
  integralLimit=0.35, -- Fraction of each axis's acceleration limit.
  acceleration={math.rad(120),math.rad(50),math.rad(90)},
  rateDeadband=math.rad(0.7),
  rateTrackDeadband=math.rad(0.6),
  solverIterations=160,
}
