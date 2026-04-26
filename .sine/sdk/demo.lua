-- demo.lua -- Sine SDK demo: request Pose at 2 Hz
-- Deploy to scripts/demo.lua

local sine = require("sine")

-- Subscribe to viGetPose at 2 Hz
sine.request_pose(500)

-- Callback path: called by sine.update() on every fresh pose
sine.on_pose(function(pose)
    gcs:send_text(6, string.format(
        "[cb] ts=%u valid=%s conf=%d lat=%.6f lon=%.6f alt=%dm",
        pose.ts, tostring(pose.valid), pose.confidence,
        pose.lat, pose.lon, pose.alt))
end)

sine.init()

local function update()
    sine.update()   -- pump TX + RX; fires on_pose callback when a response arrives
    return update, 100
end

return update, 100
