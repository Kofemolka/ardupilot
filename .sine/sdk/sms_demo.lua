-- sms_demo.lua -- Send a 25-char SMS via sine.send_sms() every 10 seconds
-- Deploy to scripts/sms_demo.lua

local sine = require("sine")

local ORIGIN      = 1
local SEND_MS     = 10000

sine.init()

local last_tx = -SEND_MS

local function update()
    sine.update()
    local now = millis():tofloat()
    if now - last_tx >= SEND_MS then
        last_tx = now
        local bat_v   = battery:voltage(0) or 0.0
        local bat_pct = battery:capacity_remaining_pct(0) or 0
        local pos     = ahrs:get_position()
        local alt_m   = pos and math.floor(pos:alt() * 0.01 + 0.5) or 0
        local gspd    = gps:ground_speed(0) or 0.0
        -- "12.6V 99% 250m 0.0m/s" fits in 25 chars
        local text = string.format("%.1fV %d%% %dm %.1fm/s",
            bat_v, bat_pct, alt_m, gspd)
        sine.send_sms(ORIGIN, text)
        gcs:send_text(6, text)
    end
    return update, 100
end

return update, 100
