-- Pure mission parsing and square-spiral route generation.
local R = {}

local function number(value, name)
  local parsed = tonumber(value)
  assert(parsed and parsed == parsed and math.abs(parsed) < 1e100,
    "Invalid mission value: " .. name)
  return parsed
end

function R.validate(mission)
  for _, name in ipairs({"x1", "z1", "x2", "z2", "chunks", "altitude"}) do
    mission[name] = number(mission[name], name)
  end
  assert(mission.chunks % 1 == 0 and mission.chunks >= 1 and mission.chunks <= 32,
    "Simulation radius must be an integer from 1 to 32 chunks")
  -- Cruise Y is a flight coordinate, independent of the world's build limits.
  -- The numeric validation above still rejects NaN and infinite values.
  return mission
end

function R.save(path, mission)
  R.validate(mission)
  local handle = assert(fs.open(path, "w"))
  handle.write(string.format(
    "x1=%s\nz1=%s\nx2=%s\nz2=%s\nchunks=%s\naltitude=%s\n",
    tostring(mission.x1),
    tostring(mission.z1),
    tostring(mission.x2),
    tostring(mission.z2),
    tostring(mission.chunks),
    tostring(mission.altitude)
  ))
  handle.close()
end

function R.load(path)
  assert(fs.exists(path), "No auto-survey mission. Run flight/setup.lua first.")
  local handle = assert(fs.open(path, "r"))
  local contents = handle.readAll()
  handle.close()

  local values = {}
  for name, value in contents:gmatch("([%a_][%w_]*)%s*=%s*([^%s]+)") do
    values[name] = value
  end
  for _, name in ipairs({"x1", "z1", "x2", "z2", "chunks", "altitude"}) do
    assert(values[name], "Mission is missing " .. name)
  end

  local mission = {
    x1 = number(values.x1, "x1"),
    z1 = number(values.z1, "z1"),
    x2 = number(values.x2, "x2"),
    z2 = number(values.z2, "z2"),
    chunks = number(values.chunks, "chunks"),
    altitude = number(values.altitude, "altitude"),
  }
  return R.validate(mission)
end

function R.build(mission, config)
  R.validate(mission)
  local minX = math.min(mission.x1, mission.x2)
  local maxX = math.max(mission.x1, mission.x2)
  local minZ = math.min(mission.z1, mission.z2)
  local maxZ = math.max(mission.z1, mission.z2)
  local width = maxX - minX
  local depth = maxZ - minZ
  assert(width > 0 and depth > 0, "The two corner points must describe an area")
  assert(math.abs(width - depth) <= 1,
    "Target area must be square within 1 block")

  local centerX = (minX + maxX) / 2
  local centerZ = (minZ + maxZ) / 2
  local halfWidth = mission.chunks * 16
  -- Reserve enough boundary coverage for the waypoint arrival tolerance.
  local maxOffset = math.max(width, depth) / 2 - halfWidth
    + 2 * config.waypointRadius
  local points = {{x = centerX, z = centerZ}}
  local scanStep = 0

  if maxOffset > 0 then
    local preferredStep = math.max(1, 2 * halfWidth * config.scanOverlap)
    local rings = math.max(1, math.ceil(maxOffset / preferredStep))
    assert(rings <= 10000, "Survey exceeds 10000 rings; split the target area")
    local step = maxOffset / rings
    scanStep = step
    local directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}}
    local x, z = 0, 0
    local length = 1

    for _ = 1, rings do
      for side = 1, 4 do
        local direction = directions[side]
        x = x + direction[1] * length * step
        z = z + direction[2] * length * step
        points[#points + 1] = {x = centerX + x, z = centerZ + z}
        if side % 2 == 0 then length = length + 1 end
      end
    end
    -- The ordinary spiral ends at the bottom-left corner. Finish the outer
    -- bottom edge so the square brush covers the final top/right boundary.
    points[#points + 1] = {
      x = centerX + maxOffset,
      z = centerZ - maxOffset,
    }
  end

  return {
    minX = minX,
    maxX = maxX,
    minZ = minZ,
    maxZ = maxZ,
    centerX = centerX,
    centerZ = centerZ,
    altitude = mission.altitude,
    chunks = mission.chunks,
    halfWidth = halfWidth,
    scanStep = scanStep,
    points = points,
  }
end

return R
