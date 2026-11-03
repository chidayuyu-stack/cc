-- Pure flight math. This file performs no peripheral or world writes.
local M = {}

function M.finite(value)
  return type(value) == "number"
    and value == value
    and value ~= math.huge
    and value ~= -math.huge
end

function M.clamp(value, low, high)
  assert(M.finite(value), "Non-finite number")
  if value < low then return low end
  if value > high then return high end
  return value
end

function M.vec(value)
  assert(type(value) == "table", "Missing vector")
  local result = {
    value.x or value[1],
    value.y or value[2],
    value.z or value[3],
  }
  for i = 1, 3 do
    assert(M.finite(result[i]), "Invalid vector component")
  end
  return result
end

function M.add(a, b)
  return {a[1] + b[1], a[2] + b[2], a[3] + b[3]}
end

function M.sub(a, b)
  return {a[1] - b[1], a[2] - b[2], a[3] - b[3]}
end

function M.mul(a, scale)
  assert(M.finite(scale), "Invalid vector scale")
  return {a[1] * scale, a[2] * scale, a[3] * scale}
end

function M.dot(a, b)
  return a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
end

function M.cross(a, b)
  return {
    a[2] * b[3] - a[3] * b[2],
    a[3] * b[1] - a[1] * b[3],
    a[1] * b[2] - a[2] * b[1],
  }
end

function M.length(a)
  return math.sqrt(M.dot(a, a))
end

function M.normalize(a)
  local length = M.length(a)
  if length < 1e-12 then return {0, 0, 0} end
  return M.mul(a, 1 / length)
end

function M.quaternion(value)
  assert(type(value) == "table", "Missing orientation")
  local q
  if value.v then
    q = {value.v.x, value.v.y, value.v.z, value.a}
  else
    q = {
      value.x or value[1],
      value.y or value[2],
      value.z or value[3],
      value.w or value.a or value[4],
    }
  end
  local length = 0
  for i = 1, 4 do
    assert(M.finite(q[i]), "Invalid orientation")
    length = length + q[i] * q[i]
  end
  assert(length > 1e-12, "Zero orientation quaternion")
  length = math.sqrt(length)
  for i = 1, 4 do q[i] = q[i] / length end
  return q
end

function M.quaternionConjugate(q)
  q = M.quaternion(q)
  return {-q[1], -q[2], -q[3], q[4]}
end

function M.rotate(q, value)
  q = M.quaternion(q)
  local xyz = {q[1], q[2], q[3]}
  local twiceCross = M.mul(M.cross(xyz, value), 2)
  return M.add(
    value,
    M.add(M.mul(twiceCross, q[4]), M.cross(xyz, twiceCross))
  )
end

function M.worldToBody(orientation, value)
  return M.rotate(M.quaternionConjugate(orientation), value)
end

function M.axisActive(positive, negative)
  if positive == negative then return 0 end
  if positive then return 1 end
  if negative then return -1 end
  return 0
end

function M.gearSpeed(config, gear)
  local speed = config.gears[gear] or config.gears[1]
  return M.clamp(speed, 0, config.maxSpeed)
end

function M.nextGear(config, gear, up, down)
  if up == down then return gear end
  if up then return math.min(#config.gears, gear + 1) end
  return math.max(1, gear - 1)
end

-- Converts the requested body-relative controls to an exact body velocity.
-- Diagonal input is normalized, so total speed can never exceed the gear.
function M.targetVelocity(config, state, input)
  local forward = M.normalize(M.vec(state.bodyForward))
  local up = {0, 1, 0}
  local right = M.normalize(M.cross(forward, up))

  local forwardAmount = M.axisActive(input.forward, input.backward)
  local rightAmount = M.axisActive(input.right, input.left)
  local upAmount = M.axisActive(input.up, input.down)

  local horizontal = M.add(
    M.mul(forward, forwardAmount),
    M.mul(right, rightAmount)
  )
  local horizontalLength = M.length(horizontal)
  local gearSpeed = M.gearSpeed(config, input.gear)
  local direction = {0, 0, 0}
  if horizontalLength >= 1e-12 then
    local horizontalSpeed = gearSpeed
    direction = M.add(direction, M.mul(horizontal, horizontalSpeed / horizontalLength))
  end

  local verticalSpeed = M.clamp(config.verticalSpeed or gearSpeed, 0, config.maxSpeed)
  direction = M.add(
    direction,
    M.mul(up, upAmount * math.min(gearSpeed, verticalSpeed))
  )

  local directionLength = M.length(direction)
  if directionLength < 1e-12 then return {0, 0, 0} end
  if directionLength > gearSpeed then
    direction = M.mul(direction, gearSpeed / directionLength)
  end
  return direction
end

function M.yawTarget(config, input)
  return M.axisActive(input.yawLeft, input.yawRight) * config.yawRate
end

return M
