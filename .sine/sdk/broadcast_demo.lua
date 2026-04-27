-- broadcast_demo.lua -- Send viSendBroadcastMessage every second with live telemetry
-- Deploy to scripts/broadcast_demo.lua

local sine = require("sine")

local ORIGIN       = 1      -- fixed sentinel: "autopilot"
local BROADCAST_MS = 1000

sine.init()

local last_tx = -BROADCAST_MS

-- 26-byte payload (little-endian):
--   [0-3]  float  bat_v    battery voltage (V)
--   [4]    uint8  bat_pct  battery capacity remaining (%)
--   [5-6]  int16  alt_m    altitude (m, from cm)
--   [7-10] float  gspd     ground speed (m/s)
--   [11]   uint8  mode     flight mode
--   [12]   uint8  sats     GPS satellite count
--   [13]   uint8  fix      GPS fix type (0=none .. 6=RTK)
--   [14-25] 12 bytes reserved (zeros)
local function make_payload()
    local bat_v   = battery:voltage(0) or 0.0
    local bat_pct = battery:capacity_remaining_pct(0) or 0
    local pos     = ahrs:get_position()
    local alt_m   = pos and math.floor(pos:alt() * 0.01 + 0.5) or 0
    local gspd    = gps:ground_speed(0) or 0.0
    local mode    = vehicle:get_mode() or 0
    local sats    = gps:num_sats(0) or 0
    local fix     = gps:status(0) or 0
    return string.pack('<fBhfBBB', bat_v, bat_pct, alt_m, gspd, mode, sats, fix)
        .. string.rep('\0', 12)
end

local function update()
    sine.update()
    local now = millis():tofloat()
    if now - last_tx >= BROADCAST_MS then
        last_tx = now
        sine.broadcast_msg(ORIGIN, make_payload())
        gcs:send_text(6, string.format(
            "BCast: bat=%.1fV/%d%% alt=%dm spd=%.1fm/s mode=%d sats=%d fix=%d",
            battery:voltage(0) or 0, battery:capacity_remaining_pct(0) or 0,
            math.floor(((ahrs:get_position() and ahrs:get_position():alt() * 0.01) or 0) + 0.5),
            gps:ground_speed(0) or 0, vehicle:get_mode() or 0,
            gps:num_sats(0) or 0, gps:status(0) or 0))
    end
    return update, 100
end

return update, 100
