"""Run installer in Lua 5.2 with in-memory downloads and computer storage."""
from pathlib import Path
from lupa.lua52 import LuaRuntime

ROOT = Path(__file__).resolve().parents[2]
REQUIRED = (
    "config.lua", "core.lua", "pilot.lua", "route.lua", "survey.lua",
    "checkpoint.lua", "dashboard.lua", "setup.lua", "BINDINGS.txt", "README.md",
)
PAYLOADS = {name: (ROOT / "flight" / name).read_text(encoding="utf-8") for name in REQUIRED}


def run(missing=None, invalid=None):
    lua = LuaRuntime(unpack_returned_tuples=True)
    lua.globals().payloads = lua.table_from(PAYLOADS)
    lua.globals().missing = missing
    lua.globals().invalid = invalid
    lua.execute(r'''
        files = {
          ["flight/mission.txt"] = "private mission settings",
          ["flight/survey-progress.0"] = "saved checkpoint 0",
          ["flight/survey-progress.1"] = "saved checkpoint 1",
          ["flight/pilot.lua"] = "previous working pilot",
          ["startup.lua"] = "existing custom startup",
        }
        requests, calls, writes = {}, {}, 0
        fs = {
          exists = function(p) return p == "flight" or files[p] ~= nil end,
          isDir = function(p) return p == "flight" end,
          combine = function(a,b) return a .. "/" .. b end,
          open = function(p, mode)
            assert(mode == "wb")
            writes = writes + 1
            return {write = function(s) files[p] = s end, close = function() end}
          end,
        }
        http = {
          checkURL = function() return true end,
          get = function(url)
            local name = url:match("/flight/(.*)$")
            assert(name and payloads[name], "unexpected download")
            requests[name] = true
            if name == missing then return nil, "network unavailable" end
            local body = name == invalid and "404 Not Found" or payloads[name]
            return {readAll = function() return body end, close = function() end}
          end,
        }
        shell = {run = function(...)
          calls[#calls + 1] = {...}
          return true
        end}
        print = function() end
        sleep = function() end
    ''')
    try:
        lua.execute((ROOT / "install.lua").read_text(encoding="utf-8"))
        success = True
    except Exception:
        success = False
    files = dict(lua.globals().files.items())
    assert files["flight/mission.txt"] == "private mission settings"
    assert files["flight/survey-progress.0"] == "saved checkpoint 0"
    assert files["flight/survey-progress.1"] == "saved checkpoint 1"
    assert files["startup.lua"] == "existing custom startup"
    if missing or invalid:
        assert not success
        assert lua.globals().writes == 0
        assert files["flight/pilot.lua"] == "previous working pilot"
        assert len(lua.globals().calls) == 0
    else:
        assert success
        assert set(dict(lua.globals().requests.items())) == set(REQUIRED)
        assert lua.globals().writes == len(REQUIRED)
        for name, body in PAYLOADS.items():
            assert files["flight/" + name] == body
        calls = lua.globals().calls
        assert len(calls) == 2
        assert calls[1][1] == "/flight/pilot.lua" and calls[1][2] == "check"
        assert calls[2][1] == "/flight/pilot.lua"


run()
for name in REQUIRED:
    run(missing=name)
    run(invalid=name)
print("PASS: installer success, 20 download failures, and runtime-data preservation.")
