local rawSensors = {
    n2 = 0
    n3 = 0
    nOutput = 0
    vbat = 0
    atfTemp = 0
}

local hasDedicatednOutput = false

local nPulsesPerRev = 60

local nSensor = {}
nSensor.__index = nSensor

function nSensor.new(ppr)
    local self = setmetatable({}, nSensor)
    
    self.ppr = ppr or 1
    self.last_time = 0
    self.last_count = 0
    self.current_rpm = 0
    self.pulseCount = 0
    
    return self
end

local n2Sensor = nSensor.new(nPulsesPerRev)
local n2rpm = n2Sensor:update(


-- The 'Update' method: 
-- raw_pulses: the total counter from the hardware
-- now_ms: the current system time in milliseconds
function nSensor:update(currentTime)
    local delta_t = now_us - self.last_time -- Time in microseconds
    local delta_p = self.pulseCount - self.last_count
    
    -- Handle self.pulseCount rolling over at 4,294,967,295 (uint32)
    if delta_p < 0 then 
        delta_p = (4294967295 - self.last_count) + self.pulseCount 
    end

    -- Math: (Pulses / PPR) / (Time_us / 1,000,000) * 60
    -- Simplified to one line for the Mega's sanity:
    if delta_t > 0 then
        self.current_rpm = (delta_p * 60000000) / (self.ppr * delta_t)
    end

    self.last_time = now_us
    self.last_count = self.pulseCount

    return self.current_rpm
end



local function calc_rpm(count)
    noInterrupts()
    local currentCount = count
    count = 0
    interrupts();

    // 2. Do the math based on how much time has passed
    // RPM = (pulses / elapsed_time_seconds) * 60 / pulses_per_revolution
    // For 722.6, N2/N3 have a specific number of "teeth" (usually 30)
    
    local scaling = (currentCount / 0.02) * 60 / nPulsesPerRev
    return (scaling); 
end


local function update()
    dest->rpm_n2 = UINT16_MAX;
    dest->rpm_n3 = UINT16_MAX;
    dest->rpm_out = UINT16_MAX;
    dest->battery_mv = UINT16_MAX;
    dest->atf_temp_c = INT_MAX;
    dest->parking_lock = UINT8_MAX;

    // RPM Sensors
    dest->rpm_n2 = calc_rpm(&cb_data_n2);
    dest->rpm_n3 = calc_rpm(&cb_data_n3);
    if (output_rpm_ok) then
        dest->rpm_out = calc_rpm(&cb_data_out);
    end
end





-- Pulse counter
volatile uint32_t pulseCount = 0;

void onPulse() {
    pulseCount++;
}

-- Usage
void setup() {
    attachInterrupt(digitalPinToInterrupt(18), onPulse, RISING);
}


local pulseCount = 0
local lastPulseTime = 0

local function onPulse()
    pulseCount = pulseCount + 1
end

