-- sine.lua -- Sine modem SDK (module)
-- Deploy to scripts/modules/sine.lua; consume with require("sine")
--
-- Sine wire layout inside FILE_TRANSFER_PROTOCOL.payload[251]:
--   [0x00 x8] [uuid LE 4] [sys_id 1] [comp_id 1] [raw_size 1] [0x00]
--   [raw ...] [crc_lo] [crc_hi] [0xFF]
--
-- CRC: CRC-MCRF4XX(raw) then accumulate MAVFTP crc_extra (84).

local MAVFTP_MSGID     = 110
local MAVFTP_CRC_EXTRA = 84
local MODEM_CHAN       = 1    -- GCS channel index for the modem serial port
local MODEM_SYS_ID     = 83   -- Sine devices: always 83
local MODEM_COMP_ID    = 76   -- Link modem component ID
local MODEM_UUID       = 0    -- 0 = any device; set to specific UUID if known

-- link.v1 Type enum (subset)
local TYPE_VI_GET_POSE           = 0x4C
local TYPE_VI_SEND_BROADCAST_MSG = 0x53
local TYPE_INVALID           = 0xFF
local TYPE_ERR_PASSWORD      = 0xFE
local TYPE_ERR_RELOAD        = 0xFD
local TYPE_ERR_FPV           = 0xFC
local TYPE_ERR_SLAVE_NOCONN  = 0xFB

-- Broadcast payload type constants
local BRD_SMS = 0x00

local ERR_NAMES = {
    [TYPE_INVALID]          = "InvalidObject",
    [TYPE_ERR_PASSWORD]     = "errPasswordRequired",
    [TYPE_ERR_RELOAD]       = "errReloadRequired",
    [TYPE_ERR_FPV]          = "errFpvModeEnabled",
    [TYPE_ERR_SLAVE_NOCONN] = "errSlaveNotConnected",
}

-- ── CRC-MCRF4XX (X25) ─────────────────────────────────────────────────────

local function crc_step(byte, crc)
    local tmp = byte ~ (crc & 0xFF)
    tmp = (tmp ~ ((tmp << 4) & 0xFF)) & 0xFF
    return ((crc >> 8) ~ (tmp << 8) ~ (tmp << 3) ~ (tmp >> 4)) & 0xFFFF
end

local function crc_buf(data)
    local crc = 0xFFFF
    for i = 1, #data do crc = crc_step(data:byte(i), crc) end
    return crc
end

-- ── Sine payload pack / unpack ─────────────────────────────────────────────

local function sine_pack(raw, sys_id, comp_id, uuid)
    local crc = crc_step(MAVFTP_CRC_EXTRA, crc_buf(raw))
    local buf = string.rep('\0', 8)
             .. string.pack('<I4', uuid)
             .. string.char(sys_id, comp_id, #raw, 0)
             .. raw
             .. string.char(crc & 0xFF, (crc >> 8) & 0xFF, 0xFF)
    if #buf < 251 then buf = buf .. string.rep('\0', 251 - #buf) end
    return buf
end

-- Returns raw, sys_id, comp_id, uuid  or  nil, err_string
local function sine_unpack(buf)
    if #buf < 19 then return nil, "too short" end
    for i = 1, 8 do
        if buf:byte(i) ~= 0 then return nil, "bad header" end
    end
    local uuid    = string.unpack('<I4', buf, 9)
    local sys_id  = buf:byte(13)
    local comp_id = buf:byte(14)
    local n       = buf:byte(15)
    if 16 + n + 3 > #buf then return nil, "truncated" end
    local raw     = buf:sub(17, 16 + n)
    if buf:byte(19 + n) ~= 0xFF then return nil, "bad terminator" end
    local crc_exp  = buf:byte(17 + n) | (buf:byte(18 + n) << 8)
    local crc_calc = crc_step(MAVFTP_CRC_EXTRA, crc_buf(raw))
    if crc_calc ~= crc_exp then
        return nil, string.format("CRC %04X != %04X", crc_calc, crc_exp)
    end
    return raw, sys_id, comp_id, uuid
end

-- ── link.v1 protocol ──────────────────────────────────────────────────────

-- Build a read request header (3 bytes): size=0, is_remote=0
local function link_req(type_byte, obj_index)
    return string.char(type_byte, 0x00, (obj_index or 0) & 0x3F)
end

-- Parse a link.v1 Header from raw bytes (1-based)
-- Returns type, body_size, is_remote, obj_index  or  nil, err_string
local function link_parse_header(raw)
    if #raw < 3 then return nil, "too short for header" end
    local t   = raw:byte(1)
    local b2  = raw:byte(2)
    local b3  = raw:byte(3)
    -- byte2: bits[0:5]=size, bit[6]=is_remote, bit[7]=_res0
    local body_size = b2 & 0x3F
    local is_remote = (b2 >> 6) & 0x01
    local obj_index = b3 & 0x3F
    return t, body_size, is_remote, obj_index
end

-- Parse viPose response body (bytes 4..32 of raw, after 3-byte header)
-- viPose body layout (packed, all LE):
--   [0-3]  ts          uint32
--   [4-7]  lat         float
--   [8-11] lon         float
--   [12-13] alt        uint16
--   [14-17] vn         float
--   [18-21] ve         float
--   [22-25] cog        float
--   [26]   valid       uint8 (bool)
--   [27]   _res        uint8
--   [28]   confidence  uint8
local function parse_vi_pose(raw)
    if #raw < 32 then
        return nil, string.format("viPose too short: %d bytes", #raw)
    end
    local ts, lat, lon, alt, vn, ve, cog, off
    ts,  off = string.unpack('<I4', raw, 4)
    lat, off = string.unpack('<f',  raw, off)
    lon, off = string.unpack('<f',  raw, off)
    alt, off = string.unpack('<I2', raw, off)
    vn,  off = string.unpack('<f',  raw, off)
    ve,  off = string.unpack('<f',  raw, off)
    cog, off = string.unpack('<f',  raw, off)
    local valid = raw:byte(off) ~= 0
    local conf  = raw:byte(off + 2)
    return { ts=ts, lat=lat, lon=lon, alt=alt, vn=vn, ve=ve, cog=cog,
             valid=valid, confidence=conf }
end

-- ── Private mutable state (closure upvalues) ──────────────────────────────

local _initialized = false
local _last_pose   = nil
local _pose_cb     = nil

-- Polling table keyed by message type.
-- rate_ms = nil means disabled; last_ms primed to fire on first tick.
local _requests = {
    [TYPE_VI_GET_POSE] = { rate_ms = nil, last_ms = -math.huge },
}

-- ── Private helpers ────────────────────────────────────────────────────────

local function _send_req(req_raw)
    local pl  = sine_pack(req_raw, MODEM_SYS_ID, MODEM_COMP_ID, MODEM_UUID)
    local pkt = string.char(0, MODEM_SYS_ID, MODEM_COMP_ID) .. pl
    if not mavlink:send_chan(MODEM_CHAN, MAVFTP_MSGID, pkt) then
        gcs:send_text(4, "Sine: tx failed (chan=" .. MODEM_CHAN .. ")")
    end
end

local function _handle_raw(raw)
    local t, _, _, _ = link_parse_header(raw)
    if not t then
        gcs:send_text(4, "Sine: bad link header")
        return
    end

    local err_name = ERR_NAMES[t]
    if err_name then
        gcs:send_text(4, "Sine: modem error: " .. err_name)
        return
    end

    if t == TYPE_VI_GET_POSE then
        local pose, err = parse_vi_pose(raw)
        if not pose then
            gcs:send_text(4, "Sine: parse error: " .. err)
            return
        end
        _last_pose = pose
        if _pose_cb then _pose_cb(pose) end

    elseif t == TYPE_VI_SEND_BROADCAST_MSG then
        -- modem echoes the broadcast back; nothing to do

    else
        gcs:send_text(5, string.format("Sine: unexpected type 0x%02X", t))
    end
end

-- ── Public API ─────────────────────────────────────────────────────────────

local M = {}

-- Must be called once before update(). Safe to call multiple times.
function M.init()
    if _initialized then return end
    mavlink:init(5, 1)
    mavlink:register_rx_msgid(MAVFTP_MSGID)
    _initialized = true
    gcs:send_text(6, string.format("Sine SDK ready (modem=%d/%d chan=%d)",
        MODEM_SYS_ID, MODEM_COMP_ID, MODEM_CHAN))
end

-- Drive TX and RX. Call every tick from the consumer's update loop.
function M.update()
    if not _initialized then return end
    local now_ms = millis():tofloat()

    for type_byte, req in pairs(_requests) do
        if req.rate_ms and (now_ms - req.last_ms >= req.rate_ms) then
            req.last_ms = now_ms
            _send_req(link_req(type_byte))
        end
    end

    while true do
        local msg, _ = mavlink:receive_chan()
        if not msg then break end
        -- mavlink_message_t struct layout (packed, 1-based Lua indices):
        --   bytes 1-2: checksum, 3: magic, 4: len, 5-6: flags,
        --   7: seq, 8: sysid, 9: compid, 10-12: msgid(24-bit LE)
        --   bytes 13+: FILE_TRANSFER_PROTOCOL payload fields:
        --     13: target_network, 14: target_system, 15: target_component
        --     16-266: payload[251]  ← Sine protocol data lives here
        local sine_pl = msg:sub(16, 266)
        local raw, err = sine_unpack(sine_pl)
        if raw then
            _handle_raw(raw)
        else
            gcs:send_text(4, "Sine: bad pkt: " .. tostring(err))
        end
    end
end

-- Subscribe to viGetPose at the given interval. Pass nil to cancel.
function M.request_pose(rate_ms)
    _requests[TYPE_VI_GET_POSE].rate_ms = rate_ms
end

-- Returns the last received pose table, or nil.
function M.get_pose()
    return _last_pose
end

-- Register a callback invoked on every fresh pose: callback(pose)
-- Pass nil to clear.
function M.on_pose(callback)
    _pose_cb = callback
end

-- Send a viSendBroadcastMessage. origin is a uint32; payload is a Lua string ≤ 26 bytes.
function M.broadcast_msg(origin, payload)
    if not _initialized then return end
    local pl26 = payload:sub(1, 26)
    if #pl26 < 26 then pl26 = pl26 .. string.rep('\0', 26 - #pl26) end
    -- byte2: body_size=30 (0x1E), is_remote=0, obj_index=0
    local raw = string.char(TYPE_VI_SEND_BROADCAST_MSG, 0x1E, 0x00)
             .. string.pack('<I4', origin)
             .. pl26
    _send_req(raw)
end

-- Send an SMS broadcast. text is truncated/padded to 25 bytes.
function M.send_sms(origin, text)
    local t25 = text:sub(1, 25)
    if #t25 < 25 then t25 = t25 .. string.rep('\0', 25 - #t25) end
    M.broadcast_msg(origin, string.char(BRD_SMS) .. t25)
end

return M
