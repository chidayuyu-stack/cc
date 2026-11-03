-- Create Connected dashboard: 4 rows, at most 15 characters per row.
local D = {}
local lengths = setmetatable({}, {__mode = "k"})

function D.progress(task, position)
  if task.mode == "complete" then return 100 end
  if not task.plan or task.index < 2 then return 0 end
  local plan = task.plan
  local cumulative = lengths[plan]
  if not cumulative then
    cumulative = {0}
    for i = 2, #plan.points do
      local a, b = plan.points[i - 1], plan.points[i]
      cumulative[i] = cumulative[i - 1]
        + math.sqrt((b.x - a.x)^2 + (b.z - a.z)^2)
    end
    lengths[plan] = cumulative
  end
  local index = math.min(task.index, #plan.points)
  local a, b = plan.points[index - 1], plan.points[index]
  if task.mode == "paused" or task.mode == "resume_climb"
    or task.mode == "resume_return" then
    position = task.pausePoint
  end
  local dx, dz = b.x - a.x, b.z - a.z
  local squared = dx * dx + dz * dz
  local fraction = squared > 0 and math.max(0, math.min(1,
    ((position.x - a.x) * dx + (position.z - a.z) * dz) / squared)) or 0
  local total = cumulative[#plan.points]
  if total == 0 then return 0 end
  return math.min(99, math.floor(100 * (cumulative[index - 1]
    + fraction * math.sqrt(squared)) / total))
end

function D.lines(state, speed, gear, gearCount, enabled, task, saveError)
  local labels = {
    idle = "Z:START X:PAUSE", paused = "PAUSE X:RESUME",
    complete = "COMPLETE", climb_outbound = "CLIMB CRUISE",
    transit_center = "TO CENTER", resume_climb = "RESUME CLIMB",
    resume_return = "RETURN TO PAUSE",
  }
  local status = labels[task.mode] or "SCAN"
  if task.mode == "scan" then
    status = string.format("SCAN %d/%d", task.index - 1, #task.plan.points - 1)
    if #status > 15 then status = "SCAN ACTIVE" end
  elseif task.mode == "idle" and task.message ~= "AUTO IDLE" then
    status = "MISSION ERROR"
  end
  if saveError then status = "SAVE ERROR" end
  local detail = "80/340m/s v1.0"
  if task.plan then
    detail = string.format("P%03d%% H%03.0f R%02d",
      D.progress(task, state.position), task.plan.altitude, task.plan.chunks)
  end
  local rows = {
    string.format("V%05.1f F:%s", speed, enabled and "ON" or "OFF"),
    string.format("Y%04.0f G%02d/%02d", state.position.y, gear, gearCount),
    status, detail,
  }
  for i, row in ipairs(rows) do rows[i] = row:sub(1, 15) end
  return rows
end

return D
