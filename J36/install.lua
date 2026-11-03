local base = "https://raw.githubusercontent.com/chidayuyu-stack/cc/master/J36/"
local files = {
  {name = "config.lua", marker = "engineGate"},
  {name = "core.lua", marker = "aeroModel"},
  {name = "pilot.lua", marker = "THREE-ENGINE FIGHTER"},
  {name = "aero.lua", marker = "sailCount ="},
}
local downloaded = {}
local startupProgram

if not http then
  error("HTTP API is disabled on this server.", 0)
end
http.checkURL(base .. "install.lua")

print("Downloading CC fighter flight controller...")
for _, file in ipairs(files) do
  local response, failure = http.get(base .. "flight/" .. file.name, nil, true)
  if not response then
    error("Download failed for " .. file.name .. ": " .. tostring(failure), 0)
  end
  local body = response.readAll()
  response.close()
  if not body or #body < 100 or not body:find(file.marker, 1, true) then
    error("Invalid download for " .. file.name, 0)
  end
  downloaded[file.name] = body
end

if not fs.exists("startup.lua") then
  local response, failure = http.get(base .. "startup.lua", nil, true)
  if not response then
    error("Download failed for startup.lua: " .. tostring(failure), 0)
  end
  startupProgram = response.readAll()
  response.close()
  if not startupProgram or #startupProgram < 40 or not startupProgram:find("shell.run", 1, true) then
    error("Invalid download for startup.lua", 0)
  end
end

if fs.exists("flight") and not fs.isDir("flight") then
  error("A file named 'flight' already exists.", 0)
end
if not fs.exists("flight") then fs.makeDir("flight") end
for name, body in pairs(downloaded) do
  local handle = assert(fs.open(fs.combine("flight", name), "wb"))
  handle.write(body)
  handle.close()
end
if fs.exists("startup.lua") then
  print("Keeping existing /startup.lua")
else
  local handle = assert(fs.open("startup.lua", "wb"))
  handle.write(startupProgram)
  handle.close()
  print("Installed /startup.lua; the controller starts automatically on boot.")
end

print("Installed to /flight")
print("Running read-only check...")
sleep(1)
local success = shell.run("/flight/pilot.lua", "check")
if not success then
  error("Aero controller check failed. Fix the reported issue before flying.", 0)
end
print("Starting flight controller...")
sleep(1)
shell.run("/flight/pilot.lua")
