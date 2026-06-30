'''
MAVLink RANGING_BEACON simulator for AP_Beacon_Sine (BCN_TYPE=4).

AP_FLAKE8_CLEAN
'''

import math
import random
import struct


_MAV_COMP_ID_USER66 = 90        # AP_Beacon_Sine filter requires this component ID
_MAVLINK_MSG_ID_RANGING_BEACON = 513   # development.xml ID
_BEACON_SIM_SYSID = 200         # source system ID used in outgoing packets


def _x25crc(buf):
    crc = 0xFFFF
    for b in (buf.encode() if isinstance(buf, str) else buf):
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def _x25crc_accumulate_byte(crc, b):
    tmp = b ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def _crc_extra(msg_name, wire_order_fields):
    '''Compute MAVLink CRC_EXTRA for a message.

    Per pymavlink generator/mavparse.py message_checksum(), the accumulation
    runs over msg_name + space, then each field in WIRE ORDER (sorted by type
    size, declaration-order tiebreak within each size group), not declaration order.
    '''
    crc = _x25crc(msg_name + ' ')
    for ftype, fname in wire_order_fields:
        for b in (ftype + ' ').encode():
            crc = _x25crc_accumulate_byte(crc, b)
        for b in (fname + ' ').encode():
            crc = _x25crc_accumulate_byte(crc, b)
    return (crc & 0xFF) ^ (crc >> 8)


# RANGING_BEACON fields in wire order (sorted by type size, declaration-order tiebreak)
# This matches the C struct layout in mavlink_msg_ranging_beacon.h and gives CRC_EXTRA = 99.
_RANGING_BEACON_WIRE_FIELDS = [
    ('uint64_t', 'time_usec'),    # 8 bytes
    ('uint32_t', 'range'),        # 4 bytes
    ('int32_t',  'lat'),          # 4 bytes
    ('int32_t',  'lon'),          # 4 bytes
    ('float',    'alt'),          # 4 bytes
    ('uint32_t', 'hacc_est'),     # 4 bytes
    ('uint32_t', 'vacc_est'),     # 4 bytes
    ('uint32_t', 'range_accuracy'),  # 4 bytes
    ('uint16_t', 'beacon_id'),    # 2 bytes
    ('uint16_t', 'carrier_freq'), # 2 bytes
    ('uint8_t',  'target_system'),  # 1 byte
    ('uint8_t',  'target_component'),  # 1 byte
    ('uint8_t',  'alt_type'),     # 1 byte
    ('uint8_t',  'sequence'),     # 1 byte
    ('uint8_t',  'status'),       # 1 byte
]
_RANGING_BEACON_CRC_EXTRA = _crc_extra('RANGING_BEACON', _RANGING_BEACON_WIRE_FIELDS)  # == 99


def _haversine_m(lat1, lng1, lat2, lng2):
    '''Horizontal great-circle distance in metres between two WGS84 points.'''
    R = 6371008.8
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lng2 - lng1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.asin(math.sqrt(a))


class SITLBeaconSimulator:
    '''Simulate RANGING_BEACON MAVLink messages for AP_Beacon_Sine (BCN_TYPE=4).

    Install as a message hook via vehicle.install_message_hook_context(sim).
    Fires synchronously in the main test thread on every GLOBAL_POSITION_INT —
    no background thread, no locking required.

    beacons:       list of up to 4 (lat_deg, lon_deg, alt_m_MSL) tuples
    noise_sigma_m: 1σ Gaussian range noise added to each measurement (metres)
    rate_hz:       per-beacon message rate in simulation Hz
    '''

    def __init__(self, vehicle, beacons, noise_sigma_m=0.5, rate_hz=10.0):
        if len(beacons) > 4:
            raise ValueError("AP_Beacon supports at most 4 beacons (AP_BEACON_MAX_BEACONS=4)")
        self._vehicle = vehicle
        self._beacons = list(beacons)
        self._noise_sigma_m = noise_sigma_m
        self._rate_hz = rate_hz
        self._seq = 0
        self._last_sent = {}   # beacon_idx -> last sim-time float

    def __call__(self, mav, msg):
        '''Message hook entry point — called for every received MAVLink message.'''
        if msg.get_type() != 'GLOBAL_POSITION_INT':
            return
        # Skip uninitialized GPS frames (before SITL establishes a position fix)
        if msg.lat == 0 and msg.lon == 0:
            return

        veh_lat = msg.lat * 1e-7   # degE7 → degrees
        veh_lng = msg.lon * 1e-7
        veh_alt = msg.alt * 1e-3   # mm → m MSL

        sim_now = self._vehicle.get_sim_time_cached()
        interval = 1.0 / self._rate_hz

        for i, (blat, blon, balt) in enumerate(self._beacons):
            if sim_now - self._last_sent.get(i, -999.0) < interval:
                continue
            self._last_sent[i] = sim_now

            h = _haversine_m(veh_lat, veh_lng, blat, blon)
            v = veh_alt - balt
            range_m = math.sqrt(h * h + v * v)
            if self._noise_sigma_m > 0:
                range_m += random.gauss(0, self._noise_sigma_m)
            range_m = max(0.1, range_m)

            self._seq = (self._seq + 1) & 0xFF
            self._send(i, range_m, blat, blon, balt)

    def _send(self, beacon_id, range_m, blat, blon, balt):
        '''Build and send a raw MAVLink 2.0 RANGING_BEACON frame.

        Wire field order (sorted by type size, declaration-order tiebreak):
          Q time_usec | I range | i lat | i lon | f alt |
          I hacc_est | I vacc_est | I range_accuracy |
          H beacon_id | H carrier_freq |
          B target_system | B target_component | B alt_type | B sequence | B status
        Total payload: 45 bytes; total frame: 57 bytes.
        '''
        range_mm = min(int(range_m * 1000), 0xFFFFFFFF)  # clamp to uint32 max
        payload = struct.pack(
            '<QIiifIIIHHBBBBB',
            0,                                        # time_usec
            range_mm,                                 # range (mm)
            int(blat * 1e7),                          # lat (degE7)
            int(blon * 1e7),                          # lon (degE7)
            float(balt),                              # alt (m MSL)
            0,                                        # hacc_est (mm)
            0,                                        # vacc_est (mm)
            int(self._noise_sigma_m * 1000),          # range_accuracy (mm)
            beacon_id & 0xFFFF,                       # beacon_id
            0,                                        # carrier_freq
            1,                                        # target_system
            _MAV_COMP_ID_USER66,                      # target_component = 90
            0,                                        # alt_type = 0 (WGS84)
            self._seq,                                # sequence
            0,                                        # status
        )

        mid = _MAVLINK_MSG_ID_RANGING_BEACON          # 513
        header = bytes([
            0xFD,                                     # STX (MAVLink 2.0)
            len(payload),                             # LEN
            0x00,                                     # INCOMPAT flags
            0x00,                                     # COMPAT flags
            self._seq,                                # SEQ
            _BEACON_SIM_SYSID,                        # SYSID
            _MAV_COMP_ID_USER66,                      # COMPID
            mid & 0xFF,                               # MSGID byte 0
            (mid >> 8) & 0xFF,                        # MSGID byte 1
            (mid >> 16) & 0xFF,                       # MSGID byte 2
        ])

        # CRC-16/X.25 over bytes 1..end_of_payload (everything after STX)
        crc = _x25crc(header[1:] + payload)
        # Accumulate the message-specific CRC_EXTRA byte
        crc = _x25crc_accumulate_byte(crc, _RANGING_BEACON_CRC_EXTRA)
        packet = header + payload + struct.pack('<H', crc)

        try:
            self._vehicle.mav.write(packet)
        except Exception:
            pass
