local dir = fs.getDir(shell.getRunningProgram())
local route = dofile(fs.combine(dir, "route.lua"))
local config = dofile(fs.combine(dir, "config.lua"))
local output = fs.combine(dir, config.missionFile)

if shell.getRunningProgram():match("pilot%.lua$") then
  error("Configure the mission from the shell after stopping pilot.lua.", 0)
end

local function readNumber(label)
  write(label)
  local value = tonumber(read())
  if not value then error("Expected a number.", 0) end
  return value
end

print("Void Chair auto-survey setup")
print("Enter the two opposite corners in world X/Z.")
local mission = {
  x1 = readNumber("Corner 1 X: "),
  z1 = readNumber("Corner 1 Z: "),
  x2 = readNumber("Corner 2 X: "),
  z2 = readNumber("Corner 2 Z: "),
  chunks = readNumber("Server simulation distance (chunks): "),
  altitude = readNumber("Cruise altitude/Y: "),
}

-- Validate the entire route before replacing the last valid mission.
local plan = route.build(mission, config)
route.save(output, mission)
print(string.format(
  "Saved: %.0f x %.0f blocks, radius %d chunks, altitude %.1f",
  plan.maxX - plan.minX,
  plan.maxZ - plan.minZ,
  plan.chunks,
  plan.altitude
))
print("Waypoints: " .. #plan.points)
print(string.format("Brush radius: %.0f blocks; lane spacing: %.2f blocks",
  plan.halfWidth, plan.scanStep))
print("Scan: 80 m/s; transit: up to 340 m/s.")
print("Existing saved progress keeps its original route until completed.")
print("Press Z in flight to start. Press X to pause/resume.")
