

    function update_io_layer()

    end
    


    local parking_lock = 0
    local atf_temperature = 0
    local battery_mv = 0
    
    local r2_1 = 1.61
    local n2_rpm = 0
    local n3_rpm = 0
    local output_rpm = 0
    local motor_temperature = 0
    local motor_oil_temperature = 0

    local function setr2_1(ratio)
        r2_1 = ratio
    end
    
    local function calcInputSpeed(n2, n3)
        return math.max(0, (n2 * r2_1) + (n3 - (r2_1 * n3)))
    end
