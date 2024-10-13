local sensor_addr = 0x76 -- the sensor has two addresses; 0x76 and 0x77
local T, P, H = {}, {}, {} -- calib coefficient tables
local p, t, h
local i2c_bus = i2c:get_device(3, sensor_addr) -- get device handler, bus 3 = i2ca
local v1, v2, v3, tfine
local counter = 0
local file_name = "SENSOR_DATA.csv"
local file

-- index for table logging
local temp = 1
local pres = 2
local humi = 3
local coeff = 4
local sensor_data = {}

-- Initialize CSV file
file = io.open(file_name, "a")
if not file then
    error("Could not create file")
end
-- Write CSV header
file:write('Time Stamp(ms), Temperature(C), Pressure(Pa), Humidity(%rH), Coefficients\n')
file:flush()

-- Logging to SD Card (CSV)
local function write_to_file()
    if not file then
        error("Could not open file")
    end
    -- Write sensor data to the file
    file:write(tostring(millis()) .. ", " .. table.concat(sensor_data, ", ") .. "\n")
    file:flush()
end

-- Logging to data flash
local function write_to_dataflash()
    -- Logging temperature, pressure, humidity
    logger:write('SENS', 'temp(C),pressure(Pa),humidity(%rH)', 'fff', sensor_data[temp], sensor_data[pres], sensor_data[humi])
    -- Logging coefficients (as a summary string)
    logger:write('COEF', 'coefficients', 's', table.concat(sensor_data[coeff], ", "))
end

-- Calibration function
function calib()
    local c
    local function int16_t(uint, nbits)
        local first_neg = ({ [8] = 0x80, [16] = 0x8000 })[nbits or 16]
        return uint - (uint & first_neg) * 2
    end

    local function bytes_to_string(byte_table)
        local str = ""
        for i = 1, #byte_table do
            str = str .. string.char(byte_table[i])
        end
        return str
    end

    -- Read calibration data
    local calib1 = bytes_to_string(i2c_bus:read_registers(0x88, 24))
    local calib2 = bytes_to_string(i2c_bus:read_registers(0xA1, 1))
    local calib3 = bytes_to_string(i2c_bus:read_registers(0xE1, 7))
    local mode = (i2c_bus:read_registers(0xF4)) -- check mode
    gcs:send_text(0, "mode : " .. mode)

    -- Concatenate calibration data
    c = calib1 .. calib2 .. calib3

    -- Calibration coefficients
    T[1] = c:byte(1) + c:byte(2) * 256
    T[2] = int16_t(c:byte(3) + c:byte(4) * 256)
    T[3] = int16_t(c:byte(5) + c:byte(6) * 256)
    P[1] = c:byte(7) + c:byte(8) * 256
    P[2] = int16_t(c:byte(9) + c:byte(10) * 256)
    P[3] = int16_t(c:byte(11) + c:byte(12) * 256)
    P[4]=int16_t(c:byte(13)+c:byte(14)*256) -- 0x94,0x95; (signed) short
    P[5]=int16_t(c:byte(15)+c:byte(16)*256) -- 0x96,0x97; (signed) short
    P[6]=int16_t(c:byte(17)+c:byte(18)*256) -- 0x98,0x99; (signed) short
    P[7]=int16_t(c:byte(19)+c:byte(20)*256) -- 0x9A,0x9B; (signed) short
    P[8]=int16_t(c:byte(21)+c:byte(22)*256) -- 0x9C,0x9D; (signed) short
    P[9]=int16_t(c:byte(23)+c:byte(24)*256) -- 0x9E,0x9F; (signed) short
    
    -- H coefficients
    H[1] = c:byte(25)
    H[2] = int16_t(c:byte(26) + c:byte(27) * 256)
    H[3]=        c:byte(28)                 -- 0xE3     ; unsigned char
    H[4]=(c:byte(30)&0x0F)          -- 0xE5[3:0],...
    H[4]=int16_t(H[4]+c:byte(29)*16)      --  ...,0xE4; (signed) short
    H[5]=(c:byte(30)>>4)           -- 0xE5[7:4],...
    H[5]=int16_t(H[5]+c:byte(31)*16)      --  ...,0xE6; (signed) short
    H[6]=int16_t(c:byte(32),8)              -- 0xE7     ; (signed) char
    c=nil

    
    -- Save coefficients as string for logging
    sensor_data[coeff] = { "T1=" .. T[1], "T2=" .. T[2], "T3=" .. T[3], "P1=" .. P[1], "P2=" .. P[2] } -- and so on...
end
calib()

-- Supplementary function for Lua 5.3 arithmetic shift
function arshift(value, shift)
    if value >= 0 then
        return value >> shift
    else
        return ((value >> shift) | (0xFFFFFFFF << (32 - shift)))
    end
end

-- Read sensor data
function read()
    i2c_bus:write_register(0xF4, 0x27) -- put device in normal mode
    local c = i2c_bus:read_registers(0xF7, 8) -- Read raw data
     
    p = c[1] * 4096 + c[2] * 16 + c[3] / 16 -- uncompensated pressure
    t = c[4] * 4096 + c[5] * 16 + c[6] / 16 -- uncompensated temperature
    h = c[7] * 256 + c[8] -- uncompensated humidity
    c = nil
    return convert, 1000
end

-- Convert raw data
function convert()
    v1 = t / 8 - T[1] * 2
    v2 = arshift(v1 * T[2], 11)
    v3 = ((v1 / 2) * (v1 / 2) >> 12)
    tfine = v2 + arshift(v3 * T[3], 14)
    t = arshift(tfine * 5 + 128, 8)

    -- Pressure calculation
    v1 = tfine - 128000
    v2 = (math.floor((v1 / 8) * (v1 / 8)) >> 12)
    v3 = (v2 * P[3] / 8 + v1 * P[2] >> 20) + 32768
    v2 = (v2 * P[6] + v1 * P[5]) / 4 + (P[4] << 16)
    v1 = (v3 * P[1] >> 15)
    v3 = nil
    if v1==0 then -- p/0 will lua-panic
      p = nil
    else
      p = (1048576 - p - (v2 >> 12)) * 3125
      if p*2>0 then -- avoid overflow (signed) int32
        p = p*2/v1
      else
        p = p/v1*2
      end
    v1 = (math.floor((p/8)*(p/8))>>13)
    v1 = arshift(v1*P[9],12)
    v2 = arshift(p*P[8],15)
    p = p + arshift(v1 + v2 + P[7],4)
  end


    -- Humidity calculation
    v1 = tfine - 76800
    v2 = (v1*H[6]>>10)*((v1*H[3]>>11) + 32768)
    v1 = (h<<14) - (H[4]<<20) - H[5]*v1
    v2 =(v2>>10) + 2097152
    v1 = (v1 +16384>>15)*(v2*H[2] + 8192>>14)
    v2 = (v1>>15)
    v2 = (v2*v2>>7)
    v1 = v1 - (v2*H[1]>>4)
-- v1 between 0 and 100*2^22
    if v1 < 0 then
      h = 0                   --   0 %rH
    elseif v1 > 0x19000000 then
      h = 10000               -- 100 %rH
    else
      h = (v1>>12)   -- Q22.10, ie 42313 means 42313/1024=41.321 %rH
      h = (h*25>>8)  -- 0.01 %, ie 4132.1 means 41.321 %rH
    end
    v1=nil
    v2=nil
    tfine=nil
    counter=counter+1
    gcs:send_text(0, "temp result : ".. t)
    gcs:send_text(0, "pressure result : ".. p)
    gcs:send_text(0, "humidity result : ".. h)
    gcs:send_text(0, "counter : ".. counter)  


    -- Prepare data for logging
    sensor_data[temp] = t / 100 -- temperature in C
    sensor_data[pres] = p -- pressure in Pa
    sensor_data[humi] = h / 1024 -- humidity in %rH

    -- Log data to SD card and data flash
    write_to_file()
    write_to_dataflash()

    -- Reset sensor values
    t, p, h = nil, nil, nil
    return read, 8000
end

calib() -- Initialize calibration data
return read() -- Start sensor reading
