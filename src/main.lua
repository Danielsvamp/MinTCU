-- Configuration & Pins
local sol = {
    y3 = 22, -- 1-2/4-5
    y4 = 23, -- 3-4
    y5 = 24, -- 2-3
    mpc = 11,
    spc = 12,
    tcc = 13
}

local pwmSol = {
   
}

-- Shifter Constants (Enums)
local Shifter = {
    P = 0x08,
    R = 0x07,
    N = 0x06,
    D = 0x05,
    UP = 0x09,
    DN = 0x0A
}

-- State Variables
local currentGear = 2
local targetGear = 2

local newGear = -1
local shiftActive = false
local shiftStart = 0
local shiftDuration = 500
local shifterPosition = Shifter.P

local engineSpeed = 0
local pedalPosition = 0

-- Shift Calibration Table
-- In Lua, tables are much easier to read and modify
local shifts = {
    {from = 1, to = 2, pin = sSol.y3},
    {from = 2, to = 1, pin = sSol.y3},
    {from = 2, to = 3, pin = sSol.y5},
    {from = 3, to = 2, pin = sSol.y5},
    {from = 3, to = 4, pin = sSol.y4},
    {from = 4, to = 3, pin = sSol.y4},
    {from = 4, to = 5, pin = sSol.y3},
    {from = 5, to = 4, pin = sSol.y3}
}


local function startShift(from, to)
    print("startShift")
    for _, s in ipairs(shifts) do
        if s.from == from and s.to == to then
            shiftStart = tmr.now() / 1000 -- NodeMCU uses microseconds, converting to ms
            gpio.write(s.pin, gpio.HIGH)
            shiftDuration = readMap(upshiftDurMap, pedalPosition, engineSpeed)
            print("Solenoid HIGH: " .. s.pin)
            return
        end
    end
    print("No valid shift profile found!")
end

local function requestShift(direction)
    if shiftActive then return end

    newGear = currentGear + direction
    if newGear < 1 or newGear > 5 then return end

    shiftActive = true
    startShift(currentGear, newGear)
    print("Shifting to " .. newGear)
end

local function handleShifterMessage(msg)
    -- Range Selection
    if msg == Shifter.P or msg == Shifter.R or msg == Shifter.N or msg == Shifter.D then
        shifterPosition = msg
    
    -- Sequential Shifting logic
    elseif msg == Shifter.UP and shifterPosition == Shifter.D and not shiftActive and currentGear < 5 then
        requestShift(1)
        print("Requested upshift")
    elseif msg == Shifter.DN and shifterPosition == Shifter.D and not shiftActive and currentGear > 1 then
        requestShift(-1)
        print("Requested downshift")
    end
end

-- Main Loop Logic (Timer-based in NodeMCU/Lua)
local function mainLoop()
    -- Handle Shift Timing
    if shiftActive and (tmr.now() / 1000 - shiftStart >= shiftDuration) then
        for _, pin in pairs(sSol) do gpio.write(pin, gpio.LOW) end
        print("pins low")
        currentGear = newGear
        shiftActive = false
        print("Shift completed")
    end
    
    -- CAN logic would be called here via a callback/interrupt in Lua
end

-- Example of how you'd process the CAN pedal data in Lua:
local function processPedal(data)
    -- Lua doesn't need complex bitwise shifts as often, but bit lib is there:
    local raw = bit.bor(bit.lshift(data[3], 8), data[4])
    local percent = (raw * 100.0) / 65535.0
    print(string.format("Pedal: %d (%.2f%%)", raw, percent))
end

