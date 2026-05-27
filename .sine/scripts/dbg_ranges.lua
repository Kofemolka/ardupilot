-- Things to log
-- aid.mode ABS 
-- ekf.flag.using_gps
-- ekf.flag.horiz_pos_rel
-- ekf.flag.horiz_pos_abs
-- ekf.pos.x/y
--  + wind_estimate


-- rng.alig.done
-- rng.alig.start
-- rng.mode
-- rng.innov - not exposed
-- rng.x/y

-- sine.rng[0-5]
-- sine.warmup.done

local function dbg_wind()
    wind = ahrs:wind_estimate()
    wind_dir_rad = math.atan(wind:y(), wind:x())+math.pi
    wind_dir_180 = math.floor(wrap_180(math.deg(wind_dir_rad)))

    gcs:send_text(6, string.format("wnd: %.0f", wind_dir_180))
end

local function dbg_ekf()
    local src_set = ahrs:get_posvelyaw_source_set()

    local inno = ahrs:get_vel_innovations_and_variances_for_source(4)
    if (inno) then
        gcs:send_text(6, string.format("inno: %.1f/%.1f", inno:x(), inno:y())) -- NO SHOW
    end

    gcs:send_text(6, string.format("src: %d", src_set))
end


local function update()
    dbg_wind()
    dbg_ekf()

    return update, 5000
end

function wrap_360(angle)
  local res = math.fmod(angle, 360.0)
   if res < 0 then
       res = res + 360.0
   end
   return res
end

function wrap_180(angle)
  local res = wrap_360(angle)
  if res > 180 then
     res = res - 360
  end
  return res
end

return update, 5000