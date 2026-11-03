local base = "https://raw.githubusercontent.com/chidayuyu-stack/cc/master/voidchair/"
local files = {
  {name = "config.lua", marker = "virtualForce"},
  {name = "core.lua", marker = "targetVelocity"},
  {name = "pilot.lua", marker = "VIRTUAL FORCE"},
  {name = "route.lua", marker = "R.build"},
  {name = "survey.lua", marker = "S.snapshot"},
  {name = "checkpoint.lua", marker = "C.open"},
  {name = "dashboard.lua", marker = "D.lines"},
  {name = "setup.lua", marker = "Cruise altitude/Y"},
  {name = "BINDINGS.txt", marker = "CABLE INPUT MAP"},
  {name = "README.md", marker = "Voidchair v1.0"},
}
local downloaded = {}

if not http then
  error("HTTP API is disabled on this server.", 0)
end
http.checkURL(base .. "install.lua")

print("Downloading Voidchair v1.0 flight controller...")
for _, file in ipairs(files) do
  local response, failure = http.get(base .. "flight/" .. file.name, nil, true)
  if not response then
    error("Download failed for " .. file.name .. ": " .. tostring(failure), 0)
  end
  local body = response.readAll()
  response.close()
  if not body or #body < 40 or not body:find(file.marker, 1, true) then
    error("Invalid download for " .. file.name, 0)
  end
  downloaded[file.name] = body
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

print("Installed to /flight")
print("Running read-only check...")
sleep(1)
local success = shell.run("/flight/pilot.lua", "check")
if not success then
  error("Void Chair controller check failed.", 0)
end
print("For a new survey: Ctrl+T, flight/setup.lua, then flight/pilot.lua.")
print("Z starts a survey; X pauses/resumes. Saved tasks park until X.")
print("Starting flight controller; SPACE toggles virtual force.")
sleep(1)
shell.run("/flight/pilot.lua")
