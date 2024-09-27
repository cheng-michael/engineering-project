-- Taken mostly from https://github.com/avaldebe/AQmon/blob/master/lua_modules/bme280.lua

local sensor_addr = 0x76 -- The sensor has two addresses: 0x76 and 0x77
local T,P,H = {},{},{} -- Calibration coefficients
local p,t,h 
local i2c_bus = i2c:get_device(3, sensor_addr) -- Get device handler, bus 3 = i2ca
local v1,v2,v3,tfine
local counter = 0

-- CSV Logging Setup
local file_name = "sensor_data_log.csv" -- The name of the CSV file

-- Function to append data to the CSV file
local function log_to_csv(temperature, humidity, pressure, calibration_coefficients)
    local file = io.open(file_name, "a") -- append mode
    if not file then
        gcs:send_text(0, "Failed to open log file!")
        return
    end

    -- write data to CSV
    -- write the header
    if file:seek("end") == 0 then
        file:write("Timestamp,Temperature,Humidity,Pressure,Calibration Coefficients\n")
    end

    -- Get the current timestamp
    local timestamp = os.date("%Y-%m-%d %H:%M:%S")

    -- Write the data
    file:write(string.format("%s,%d,%d,%d,%s\n", 
        timestamp, 
        temperature, 
        humidity, 
        pressure, 
        table.concat(calibration_coefficients, ";"))) -- semicolon for the CCS

    file:close() -- Close the file
end

-- Calibration function
function calib()
    local c

    local function int16_t(uint, nbits)
        local first_neg = ({[8] = 0x80, [16] = 0x8000})[nbits or 16]
        return uint - (uint & first_neg) * 2
    end

    -- Function to convert a table of bytes to a string
    local function bytes_to_string(byte_table)
        local str = ""
        for i = 1, #byte_table do
            str = str .. string.char(byte_table[i])
        end
        return str
    end

    -- Read data and convert each table to a string
    local calib1 = bytes_to_string(i2c_bus:read_registers(0x88, 24))
    local calib2 = bytes_to_string(i2c_bus:read_registers(0xA1, 1))
    local calib3 = bytes_to_string(i2c_bus:read_registers(0xE1, 7))
    local mode = (i2c_bus:read_registers(0xF4)) -- Check mode
    gcs:send_text(0, "mode : ".. mode)

    -- Concatenate the strings
    c = calib1 .. calib2 .. calib3

    -- Calibration coefficients (for temperature, pressure, humidity)
    T[1] = c:byte(1) + c:byte(2) * 256
    T[2] = int16_t(c:byte(3) + c:byte(4) * 256)
    T[3] = int16_t(c:byte(5) + c:byte(6) * 256)
    P[1] = c:byte(7) + c:byte(8) * 256
    P[2] = int16_t(c:byte(9) + c:byte(10) * 256)
    P[3] = int16_t(c:byte(11) + c:byte(12) * 256)
    P[4] = int16_t(c:byte(13) + c:byte(14) * 256)
    P[5] = int16_t(c:byte(15) + c:byte(16) * 256)
    P[6] = int16_t(c:byte(17) + c:byte(18) * 256)
    P[7] = int16_t(c:byte(19) + c:byte(20) * 256)
    P[8] = int16_t(c:byte(21) + c:byte(22) * 256)
    P[9] = int16_t(c:byte(23) + c:byte(24) * 256)
    H[1] = c:byte(25)
    H[2] = int16_t(c:byte(26) + c:byte(27) * 256)
    H[3] = c:byte(28)
    H[4] = (c:byte(30) & 0x0F) + c:byte(29) * 16
    H[4] = int16_t(H[4])
    H[5] = (c:byte(30) >> 4) + c:byte(31) * 16
    H[5] = int16_t(H[5])
    H[6] = int16_t(c:byte(32), 8)

    c = nil
end
calib()

-- Supplementary function for Lua 5.3, arithmetic shift
function arshift(value, shift)
    if value >= 0 then
        return value >> shift -- Logical right shift for non-negative values
    else
        -- For negative values, replicate arithmetic shift
        return ((value >> shift) | (0xFFFFFFFF << (32 - shift)))
    end
end

-- Function to read sensor data
function read()
    -- Taking raw measurements
    i2c_bus:write_register(0xF4, 0x27) -- Putting the device into normal mode
    local c = i2c_bus:read_registers(0xF7, 8) -- REG_PRESSURE_MSB 0xF7 .. REG_HUMIDITY_LSB 0xFE
     
    p = c[1] * 4096 + c[2] * 16 + c[3] / 16 -- Uncompensated pressure
    t = c[4] * 4096 + c[5] * 16 + c[6] / 16 -- Uncompensated temperature
    h = c[7] * 256 + c[8] -- Uncompensated humidity
    c = nil
    return convert, 1000 -- Re-run in 1000 ms (1 second)
end

-- Function to convert and calculate sensor data
function convert()
    -- Temperature calculation...
    v1 = t / 8 - T[1] * 2
    v2 = arshift(v1 * T[2], 11)
    v3 = ((v1 / 2) * (v1 / 2) >> 12)
    tfine = v2 + arshift(v3 * T[3], 14)
    t = arshift(tfine * 5 + 128, 8)

    -- Pressure calculation...
    v1 = tfine - 128000
    v2 = math.floor((v1 / 8) * (v1 / 8)) >> 12
    v3 = (v2 * P[3] / 8 + v1 * P[2] >> 20) + 32768
    v2 = (v2 * P[6] + v1 * P[5]) / 4 + (P[4] << 16)
    v1 = v3 * P[1] >> 15
    if v1 == 0 then
        p = nil
    else
        p = (1048576 - p - (v2 >> 12)) * 3125
        if p * 2 > 0 then
            p = p * 2 / v1
        else
            p = p / v1 * 2
        end
        v1 = math.floor((p / 8) * (p / 8)) >> 13
        v1 = arshift(v1 * P[9], 12)
        v2 = arshift(p * P[8], 15)
        p = p + arshift(v1 + v2 + P[7], 4)
    end

    -- Humidity calculation...
    v1 = tfine - 76800
    v2 = (v1 * H[6] >> 10) * ((v1 * H[3] >> 11) + 32768)
    v1 = (h << 14) - (H[4] << 20) - H[5] * v1
    v2 = (v2 >> 10) + 2097152
    v1 = (v1 + 16384 >> 15) * (v2 * H[2] + 8192 >> 14)
    v2 = v1 >> 15
    v2 = v2 * v2 >> 7
    v1 = v1 - v2 * H[1] >> 4
    if v1 < 0 then
        h = 0
    elseif v1 > 419430400 then
        h = 100000
    else
        h = arshift(v1, 12)
    end

    -- Debugging info: Send values to GCS
    gcs:send_text(0, "Temp: " .. t)
    gcs:send_text(0, "Pres: " .. p)
    gcs:send_text(0, "Humid: " .. h)
    
    -- Log the data to the CSV file
    log_to_csv(t, h, p, T)  -- Log temperature (t), humidity (h), pressure (p), and calibration coefficients (T)

    return read, 1000 -- Re-run in 1000 ms (1 second)
end

return read, 1000 -- Start reading immediately, and run every 1000ms
