-- pos_inject.lua -- Sine SDK demo: inject External Position Estimate at 0.1 Hz
-- Deploy to scripts/pos_inject.lua  (run standalone, not alongside demo.lua)

local sine = require("sine")

local MAV_CMD_EXTERNAL_POSITION_ESTIMATE = 43003
local MAV_FRAME_GLOBAL                   = 0
local NaN                                = 0.0 / 0.0

-- Thresholds for straight-flight detection
local YAW_RATE_MAX_RAD = math.rad(3)   -- 3 °/s  yaw rate
local ROLL_MAX_RAD     = math.rad(5)   -- 5 °    bank angle

-- Returns true when the aircraft is in straight, unaccelerated flight.
-- Uses body-frame gyro yaw rate and roll angle — both sensitive and low-latency.
local function is_straight_flight()
    local gyro = ahrs:get_gyro()
    if not gyro then return false end
    return math.abs(gyro:z())        < YAW_RATE_MAX_RAD
       and math.abs(ahrs:get_roll_rad()) < ROLL_MAX_RAD
end

local INJECT_DEADLINE_MS = 60000   -- force inject if this long without a correction

local last_inject_ms = -INJECT_DEADLINE_MS   -- prime to allow immediate first inject

-- Request pose every 10 seconds
sine.request_pose(10000)

sine.on_pose(function(pose)
    if not pose.valid then
        gcs:send_text(5, "SNS: pose not valid, skipping")
        return
    end

    local now = millis():tofloat()
    local overdue = (now - last_inject_ms) >= INJECT_DEADLINE_MS

    if not overdue and not is_straight_flight() then
        gcs:send_text(6, "SNS: maneuver detected, skipping")
        return
    end

    -- lat/lon must be integer (e7 units) for run_command_int x/y fields
    local lat_i7 = math.tointeger(math.floor(pose.lat * 1e7 + 0.5))
    local lon_i7 = math.tointeger(math.floor(pose.lon * 1e7 + 0.5))
    if not lat_i7 or not lon_i7 then
        gcs:send_text(4, "SNS: lat/lon out of integer range")
        return
    end

    -- transmit timestamp in seconds, wrapped at 250 s to fit float precision
    local ts_s = (millis():tofloat() * 0.001) % 250.0
    local result = gcs:run_command_int(MAV_CMD_EXTERNAL_POSITION_ESTIMATE, {
        p1    = ts_s,       -- transmit timestamp (s, wrapping)
        p2    = 0.0,        -- processing delay (s)
        p3    = 0.0,        -- 1-sigma accuracy (m)
        x     = lat_i7,     -- latitude  × 1e7 (integer degrees)
        y     = lon_i7,     -- longitude × 1e7 (integer degrees)
        z     = NaN,        -- altitude: not used
        frame = MAV_FRAME_GLOBAL,
    })

    last_inject_ms = millis():tofloat()
    gcs:send_text(6, string.format(
        "SNS: lat=%.6f lon=%.6f rmse=%.1fm%s",
        pose.lat, pose.lon, pose.rmse,
        overdue and " [deadline]" or ""))
end)

sine.init()

local function update()
    sine.update()
    return update, 100
end

return update, 100
