--NEW
-- logging to a file on the SD card and to data flash
local file_name = "LOG.csv"
local file

-- index for the data and table
local pp = 1
local tt = 2
local hh = 3
local cc = 4
local data = {}
--NEW

local sensor_addr = 0x76 --the sensor has two address; 0x76 and 0x77
local T,P,H={},{},{} --calib coefficient
local p,t,h 
local i2c_bus = i2c:get_device(3,sensor_addr) --get device handler, bus 3 = i2ca
local v1,v2,v3,tfine
local counter=0

function calib()
    local c 

    local function int16_t(uint,nbits)  --conversion is needed, see page 24 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
        local first_neg=({[8]=0x80,[16]=0x8000})[nbits or 16]
        return uint-(uint&first_neg)*2
    end

    local function bytes_to_string(byte_table)
        local str = ""
        for i = 1, #byte_table do
            str = str .. string.char(byte_table[i])
        end
        return str
    end

    local calib1 = bytes_to_string(i2c_bus:read_registers(0x88, 24))
    local calib2 = bytes_to_string(i2c_bus:read_registers(0xA1, 1))
    local calib3 = bytes_to_string(i2c_bus:read_registers(0xE1, 7))
    local mode = (i2c_bus:read_registers(0xF4)) --check mode
    gcs:send_text(0, "mode : ".. mode)

    c = calib1 .. calib2 .. calib3

    T[1]=        c:byte( 1)+c:byte( 2)*256  
    T[2]=int16_t(c:byte( 3)+c:byte( 4)*256) 
    T[3]=int16_t(c:byte( 5)+c:byte( 6)*256) 
    P[1]=        c:byte( 7)+c:byte( 8)*256  
    P[2]=int16_t(c:byte( 9)+c:byte(10)*256) 
    P[3]=int16_t(c:byte(11)+c:byte(12)*256) 
    P[4]=int16_t(c:byte(13)+c:byte(14)*256) 
    P[5]=int16_t(c:byte(15)+c:byte(16)*256) 
    P[6]=int16_t(c:byte(17)+c:byte(18)*256) 
    P[7]=int16_t(c:byte(19)+c:byte(20)*256) 
    P[8]=int16_t(c:byte(21)+c:byte(22)*256) 
    P[9]=int16_t(c:byte(23)+c:byte(24)*256) 
    H[1]=        c:byte(25)                 
    H[2]=int16_t(c:byte(26)+c:byte(27)*256) 
    H[3]=        c:byte(28)                 
    H[4]=(c:byte(30)&0x0F)          
    H[4]=int16_t(H[4]+c:byte(29)*16)      
    H[5]=(c:byte(30)>>4)           
    H[5]=int16_t(H[5]+c:byte(31)*16)      
    H[6]=int16_t(c:byte(32),8)              

    c=nil
end
calib()

function arshift(value, shift)
    if value >= 0 then
        return value >> shift 
    else
        return ((value >> shift) | (0xFFFFFFFF << (32 - shift)))
    end
end

function read()
    i2c_bus:write_register(0xF4,0x27)
    local c = i2c_bus:read_registers(0xF7,8) 
     
    p=c[1]*4096+c[2]*16+c[3]/16       
    t=c[4]*4096+c[5]*16+c[6]/16       
    h=c[7]* 256+c[8]                  
    c=nil
    return convert, 1000 
end

function convert()
    v1 = t/8 - T[1]*2
    v2 = arshift(v1*T[2],11)
    v3 = ((v1/2)*(v1/2)>>12)
    tfine = v2 + arshift(v3*T[3],14)
    t = arshift(tfine*5 + 128,8)

    v1 = tfine - 128000
    v2 = (math.floor((v1/8)*(v1/8))>>12)
  
    v3 = (v2*P[3]/8 + v1*P[2]>>20) + 32768
    v2 = (v2*P[6] + v1*P[5])/4 + (P[4]<<16)
    v1 = (v3*P[1]>>15)

    if v1==0 then
        p = nil
    else
        p = (1048576 - p - (v2>>12))*3125
        if p*2>0 then
            p = p*2/v1
        else
            p = p/v1*2
        end
        v1 = (math.floor((p/8)*(p/8))>>13)
        v1 = arshift(v1*P[9],12)
        v2 = arshift(p*P[8],15)
        p = p + arshift(v1 + v2 + P[7],4)
    end

    v1 = tfine - 76800
    v2 = (v1*H[6]>>10)*((v1*H[3]>>11) + 32768)
    v1 = (h<<14) - (H[4]<<20) - H[5]*v1
    v2 =(v2>>10) + 2097152
    v1 = (v1 +16384>>15)*(v2*H[2] + 8192>>14)
    v2 = (v1>>15)
    v2 = (v2*v2>>7)
    v1 = v1 - (v2*H[1]>>4)

    if v1 < 0 then
        h = 0                   
    elseif v1 > 0x19000000 then
        h = 10000               
    else
        h = (v1>>12)   
        h = (h*25>>8)  
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

--NEW
local function write_to_file()
    if not file then
        gcs:send_text(0, "Error: File not opened")
        return
    end

    -- Attempt to write data
    local success, err = file:write(tostring(millis()) .. ", " .. table.concat(data, ", ") .. "\n")
    if not success then
        gcs:send_text(0, "Error writing to file: " .. err)
    else
        gcs:send_text(0, "Successfully wrote to file")
    end

    -- Flush the buffer to ensure data is written
    file:flush()
end

function update()
    data[pp] = p
    data[tt] = t
    data[hh] = h 
    data[cc] = c

    -- Check if the data table is populated
    if next(data) == nil then
        gcs:send_text(0, "Data table is empty")
        return update, 1000
    end

    -- Write to the CSV file
    write_to_file()

    return update, 1000 -- reschedules the loop
end

-- make a file
file = io.open(file_name, "a")
if not file then
    gcs:send_text(0, "Error: Could not make file")
    error("Could not make file")
else
    gcs:send_text(0, "File created successfully: " .. file_name)
end

-- write the CSV header
file:write('time(ms), pressure(Pa), temperature(°C), humidity(%), c?\n')
file:flush()

return update, 10000
