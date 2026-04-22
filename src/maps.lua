local shifts

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



local upshiftDurMap = { -- Pedal Position (%), Engine Speed (rpm), Time (ms)
    5, 6,
    
    0, 20, 40, 60, 80, 100,
    
    1000, 2000, 3000, 4000, 5000,
    
    2000, 1500, 1000, 750, 500, 500,
    1000, 900,  800,  600, 475, 450,
    700,  650,  600,  500, 450, 400,
    500,  475,  450,  425, 400, 375,
    450,  425,  400,  375, 350, 350
}

local clutchFillTimeMap = { -- ATF Temp (°C), Clutches and Brakes in order, TIME (ms)
    4, 5,

    -20, 5, 25, 60,
    
    1, 2, 3, 4, 5,

    600, 360, 220, 160, -- K1 clutch
    1620, 560, 260, 160, -- K2 clutch
    860, 500, 160, 160, -- K3 clutch
    600, 380, 220, 180, -- B1 brake
    820, 680, 260, 120  -- B2 brake
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



function readMap.new(mapName, inputX, inputY)
    local self = setmetatable({}, readMap)
    
    self.map = mapName
    self.valX = inputX
    self.valY = inputY

    local rows = map[1]
    local cols = map[2]
    
    local xStart = 3
    local yStart = xStart + cols
    local zStart = yStart + rows
    
    local xIdx = 1
    for i = 0, cols - 2 do
        if valX >= map[xStart + i] then xIdx = i + 1 end
    end
    
     local yIdx = 1
    for i = 0, rows - 2 do
        if valY >= map[yStart + i] then yIdx = i + 1 end
    end

    local yFrac = (valY - map[yStart + yIdx - 1]) / (map[yStart + yIdx] - map[yStart + yIdx - 1])
    local xFrac = (valX - map[xStart + xIdx - 1]) / (map[xStart + xIdx] - map[xStart + xIdx - 1])

    local i11 = zStart + (yIdx - 1) * cols + (xIdx - 1)
    local i21 = i11 + 1
    local i12 = i11 + cols
    local i22 = i12 + 1

    local v11, v21 = map[i11], map[i21]
    local v12, v22 = map[i12], map[i22]

    -- Bilinear Interpolation
    local top = v11 + xFrac * (v21 - v11)
    local bottom = v12 + xFrac * (v22 - v12)
    
    return top + yFrac * (bottom - top)
    
    return self
end
