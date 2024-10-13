--taken mostly from https://github.com/avaldebe/AQmon/blob/master/lua_modules/bme280.lua


-- logging to a file on the SD card and to data flash
local file_name = "LOG.csv"
local file

-- index for the data and table
local pressure = 1
local temperature = 2
local humidity = 3
local calibcoeff = 4
local interesting_data = {}

local function write_to_file()

  if not file then
    error("Could not open file")
  end

  -- write data
  -- separate with comas and add a carriage return
  file:write(tostring(millis()) .. ", " .. table.concat(data,", ") .. "\n")

  -- make sure file is upto date
  file:flush()

end

local function write_to_dataflash()

  -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
  -- format characters specify the type of variable to be logged, see AP_Logger/README.md
  -- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
  -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, and N
  -- lua automatically adds a timestamp in micro seconds
  logger:write('SCR1','press,temp,hum,cc','ffff',interesting_data[pressure],interesting_data[temperature],interesting_data[humidity],interesting_data[calibcoeff])

  -- it is also possible to give units and multipliers
  logger:write('SCR2','press,temp,hum,cc','fff','Pa,°C,%,coef','---',interesting_data[pressure],interesting_data[temperature],interesting_data[humidity],interesting_data[calibcoeff])

end

function update()

  -- get data
  data[pressure] = p
  data[temperature] = t
  data[humidity] = h 
  data[calibcoeff] = c
  -- write to then new file the SD card
  write_to_file()

  -- write to a new log in the data flash log
  write_to_dataflash()

  return update, 1000 -- reschedules the loop
end

-- make a file
file = io.open(file_name, "a")
if not file then
  error("Could not make file")
end

-- write the CSV header
file:write('time(ms), pressure(Pa), temperature(°C), humidity(%), c?\n')
file:flush()

return update, 10000

local sensor_addr = 0x76 --the sensor has two address; 0x76 and 0x77
local T,P,H={},{},{} --calib coefficient
local p,t,h 
local i2c_bus = i2c:get_device(3,sensor_addr) --get device handler, bus 3 = i2ca
local v1,v2,v3,tfine
local counter=0
function calib()

    local c 

    local function int16_t(uint,nbits)  --conversion is needed, see page 24 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
        -- first negative number
          -- uint8_t (unsigned char ): 2^7
          -- uint16_t(unsigned short): 2^15
          -- uint32_t(unsigned long ): 2^31
          local first_neg=({[8]=0x80,[16]=0x8000})[nbits or 16]
          return uint-(uint&first_neg)*2
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
    local mode = (i2c_bus:read_registers(0xF4)) --check mode
    gcs:send_text(0, "mode : ".. mode)


    -- Concatenate the strings
    c = calib1 .. calib2 .. calib3


    --little modifications here and there to for lua version 5.3, in particular bit library to normal operators like & and >> bit.
    T[1]=        c:byte( 1)+c:byte( 2)*256  -- 0x88,0x89; unsigned short
    T[2]=int16_t(c:byte( 3)+c:byte( 4)*256) -- 0x8A,0x8B; (signed) short
    T[3]=int16_t(c:byte( 5)+c:byte( 6)*256) -- 0x8C,0x8D; (signed) short
    P[1]=        c:byte( 7)+c:byte( 8)*256  -- 0x8E,0x8F; unsigned short
    P[2]=int16_t(c:byte( 9)+c:byte(10)*256) -- 0x90,0x91; (signed) short
    P[3]=int16_t(c:byte(11)+c:byte(12)*256) -- 0x92,0x93; (signed) short
    P[4]=int16_t(c:byte(13)+c:byte(14)*256) -- 0x94,0x95; (signed) short
    P[5]=int16_t(c:byte(15)+c:byte(16)*256) -- 0x96,0x97; (signed) short
    P[6]=int16_t(c:byte(17)+c:byte(18)*256) -- 0x98,0x99; (signed) short
    P[7]=int16_t(c:byte(19)+c:byte(20)*256) -- 0x9A,0x9B; (signed) short
    P[8]=int16_t(c:byte(21)+c:byte(22)*256) -- 0x9C,0x9D; (signed) short
    P[9]=int16_t(c:byte(23)+c:byte(24)*256) -- 0x9E,0x9F; (signed) short
    H[1]=        c:byte(25)                 -- 0xA1     ; unsigned char
    H[2]=int16_t(c:byte(26)+c:byte(27)*256) -- 0xE1,0xE2; (signed) short
    H[3]=        c:byte(28)                 -- 0xE3     ; unsigned char
    H[4]=(c:byte(30)&0x0F)          -- 0xE5[3:0],...
    H[4]=int16_t(H[4]+c:byte(29)*16)      --  ...,0xE4; (signed) short
    H[5]=(c:byte(30)>>4)           -- 0xE5[7:4],...
    H[5]=int16_t(H[5]+c:byte(31)*16)      --  ...,0xE6; (signed) short
    H[6]=int16_t(c:byte(32),8)              -- 0xE7     ; (signed) char
    c=nil

end
calib()

--supplementary function for 5.3 lua, arithmetic shift, shifting the bits and replacing the front bits with zeros   
function arshift(value, shift)
    -- Check if the value is negative
    if value >= 0 then
        return value >> shift -- Logical right shift for non-negative values
    else
        -- For negative values, replicate arithmetic shift
        return ((value >> shift) | (0xFFFFFFFF << (32 - shift)))
    end
end


function read()

    --taking raw measurements
    i2c_bus:write_register(0xF4,0x27) --putting the device to normal mode
    local c = i2c_bus:read_registers(0xF7,8) -- REG_PRESSURE_MSB 0xF7 .. REG_HUMIDITY_LSB 0xFE
     
    p=c[1]*4096+c[2]*16+c[3]/16       -- uncompensated pressure
    t=c[4]*4096+c[5]*16+c[6]/16       -- uncompensatedtemperature
    h=c[7]* 256+c[8]                  -- uncompensated humidity
    c=nil
    return convert, 1000 --milisec
end

function convert()
    

    v1 = t/8 - T[1]*2
    v2 = arshift(v1*T[2],11)
    v3 = ((v1/2)*(v1/2)>>12)
    tfine = v2 + arshift(v3*T[3],14)
    t = arshift(tfine*5 + 128,8)
    
    --[[ Pressure: Adapted from bme280_compensate_pressure_int32.
  Calculate actual pressure from uncompensated pressure.
  Returns the value in Pascal (Pa),
  and output value of "96386" equals 96386 Pa = 963.86 hPa. ]]
  v1 = tfine - 128000

  v2 = (math.floor((v1/8)*(v1/8))>>12)
  
  v3 = (v2*P[3]/8 + v1*P[2]>>20) + 32768
  v2 = (v2*P[6] + v1*P[5])/4 + (P[4]<<16)
  v1 = (v3*P[1]>>15)
  v3 = nil
  if v1==0 then -- p/0 will lua-panic
    p = nil
  else
    p = (1048576 - p - (v2>>12))*3125
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

--[[ Humidity: Adapted from bme280_compensate_humidity_int32.
  Calculte actual humidity from uncompensated humidity.
  Returns the value in 0.01 %rH.
  An output value of "4132.1" represents 41.321 %rH ]]
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
  t=nil
  p=nil
  h=nil
  

return read, 8000
end





-- not needed?
--[[

--]]



--NEW


return read()
