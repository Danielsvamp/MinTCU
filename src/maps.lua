

Clutch get_clutch_to_apply(GearChange change) {
    switch(change) {
        case GearChange::_1_2:
        case GearChange::_5_4:
            return Clutch::K1;
        case GearChange::_2_3:
            return Clutch::K2;
        case GearChange::_3_4:
        case GearChange::_3_2:
            return Clutch::K3;
        case GearChange::_4_3:
            return Clutch::B2;
        case GearChange::_4_5:
        case GearChange::_2_1:
        default:
            return Clutch::B1;
    }
}

const Clutch HOLDING_CLUTCHES[8][2] = {
    {Clutch::B2, Clutch::K3}, // 1-2 (B2 + K3)
    {Clutch::B2, Clutch::K1}, // 2-3 (B2 + K1)
    {Clutch::K1, Clutch::K2}, // 3-4 (K1 + K2)
    {Clutch::K2, Clutch::K3}, // 4-5 (K2 + K3)

    {Clutch::B2, Clutch::K3}, // 2-1 (B2 + K3)
    {Clutch::B2, Clutch::K1}, // 3-2 (B2 + K1)
    {Clutch::K1, Clutch::K2}, // 4-3 (K1 + K2)
    {Clutch::K2, Clutch::K3}, // 5-4 (K2 + K3)
};



local upshiftDurMapData = { -- Pedal Position (%), Engine Speed (rpm), Time (ms)
    6, 5,
    
    0, 20, 40, 60, 80, 100,
    
    1000, 2000, 3000, 4000, 5000,
    
    2000, 1500, 1000, 750, 500, 500,
    1000, 900,  800,  600, 475, 450,
    700,  650,  600,  500, 450, 400,
    500,  475,  450,  425, 400, 375,
    450,  425,  400,  375, 350, 350
}

local clutchFillTimeMapData = { -- ATF Temp (°C), Clutches and Brakes in order, TIME (ms)
    4, 5,

    -20, 5, 25, 60,
    
    1, 2, 3, 4, 5,

    600, 360, 220, 160, -- K1 clutch
    1620, 560, 260, 160, -- K2 clutch
    860, 500, 160, 160, -- K3 clutch
    600, 380, 220, 180, -- B1 brake
    820, 680, 260, 120  -- B2 brake
}

local tccPWM = { -- Pressure (Unit: Bar, Factor: 0.005), ATF Temp (Unit: °C, Offset: -50), PWM Duty Cycle)
    7, 5,

    0, 400, 800, 1000, 1500, 2000, 3000,
    
    50,	80,	110, 140, 170,

    0,	7680,	15360,	20480,	30720,	40960,	64535,
    0,	8960,	16640,	20480,	30720,	40960,	64535,
    0,	10240,	17920,	20480,	30720,	40960,	64535,
    0,	10240,	17920,	20480,	30720,	40960,	64535,
    0,	10240,	17920,	20480,	30720,	40960,	64535,
}

local friction = { -- Clutches in order K1-B3, Gears in order (N, D1-D5, R1-R2), Raw value for factor
    6, 8,

    K1, K2, K3, B1, B2, B3,
    
    1, 2, 3, 4, 5, 6, 7, 8

    4709,	0,	    0,	    3574,	0,  	0,
    0,	    0,	    3076,	2303,	2685,	0,
    1845,	0,	    1871,	0,	    1633,	0,
    0,	    1101,	0,	    0,	    1109,	0,
    958,	1673,	971,	0,	    0,  	0,
    0,	    1390,	807,	604,	0,	    0,
    0,	    0,	    3076,	2303,	0,	    3387,
    1845,	0,	    1871,	0,	    0,	    2060,
}

local lookupMap = {}
lookupMap.__index = lookupMap

function lookupMap.new(mapName)
    local self = setmetatable({}, lookupMap)
    
    self.map = mapName
    self.cols = mapName[1]
    self.rows = mapName[2]
    
    -- S = Start
    self.xS = 3
    self.yS = self.xS + self.cols
    self.zS = self.yS + self.rows

    return self
end

function lookupMap:get(inputX, inputY)
    
    local map = self.map
    local rows = self.rows
    local cols = self.cols
    
    local xS = self.xS
    local yS = self.yS
    local zS = self.zS
    
    local xIdx = 1
    for i = 0, cols - 2 do
        if inputX >= map[xS + i] then xIdx = i + 1 end
    end
    
    local yIdx = 1
    for i = 0, rows - 2 do
        if inputY >= map[yS + i] then yIdx = i + 1 end
    end

    local yFrac = (inputY - map[yS + yIdx - 1]) / (map[yS + yIdx] - map[yS + yIdx - 1])
    local xFrac = (inputX - map[xS + xIdx - 1]) / (map[xS + xIdx] - map[xS + xIdx - 1])

    local i11 = zS + (yIdx - 1) * cols + (xIdx - 1)
    local i21 = i11 + 1
    local i12 = i11 + cols
    local i22 = i12 + 1

    local v11, v21 = map[i11], map[i21]
    local v12, v22 = map[i12], map[i22]

    -- Bilinear Interpolation
    local top = v11 + xFrac * (v21 - v11)
    local bottom = v12 + xFrac * (v22 - v12)
    
    return top + yFrac * (bottom - top)
end

-- Store maps
local upshiftDurMapData = lookupMap.new(upshiftDurMapData)
local clutchFillTimeMap = lookupMap.new(clutchFillTimeMapData)
