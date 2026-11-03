-- Two alternating snapshots. An interrupted write leaves the other intact.
local C = {}

function C.open(path, validate)
  local sequence = 0
  local latest
  local found = false
  for slot = 0, 1 do
    local name = path .. "." .. slot
    if fs.exists(name) then
      found = true
      local ok, saved = pcall(function()
        local handle = assert(fs.open(name, "r"))
        local contents = handle.readAll()
        handle.close()
        local value = textutils.unserialize(contents)
        assert(type(value) == "table" and value.version == 1
          and type(value.sequence) == "number" and value.sequence >= 1
          and value.sequence < 1e15 and value.sequence % 1 == 0, "Invalid snapshot")
        validate(value.task)
        return value
      end)
      if ok and saved.sequence > sequence then
        sequence, latest = saved.sequence, saved.task
      end
    end
  end
  local store = {}
  function store.save(task)
    validate(task)
    local nextSequence = sequence + 1
    local contents = textutils.serialize({version = 1, sequence = nextSequence, task = task})
    local handle = assert(fs.open(path .. "." .. (nextSequence % 2), "w"))
    handle.write(contents)
    handle.close()
    sequence = nextSequence
  end
  return store, latest, found and not latest
end

return C
