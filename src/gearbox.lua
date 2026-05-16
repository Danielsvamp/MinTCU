local currentGear = 2
local targetGear = 2



while true do
   
end


-- Crossover Shift start -- For upshifts under positive load and downshifts when coasting

-- PREPARE FOR SHIFT 
-- Phase 0 Bleed: SPC pressure ramps from "SPC_MAX" to "high fill pressure" over time "60 ms", MPC holds releasing clutch
--      Wait for timer

-- Shift solenoid ON here

-- Phase 1.1 Fill 1 (High fill): SPC pressure from "fill pressure map" for time "fill time hold map"
-- Phase 1.2 Fill 2 (Ramp to low fill): SPC ramps pressure to "low fill pressure" over time "low fill time varable (60 ms)"
-- Phase 1.3 Fill 3 (Low fill hold): SPC pressure "low fill pressure map" for time "low fill hold time map (100 ms)"
-- "KISSING POINT"

-- Phase 2 Overlap1: SPC ramps to "torque holding pressure per gear" (or "SPC_MAX"?), MPC reduced (quantity?) for releasing clutch to slip, TCC unlocked for damping
-- Wait for releasing clutch to slip (until? and how much?)
-- Phase 3 Overlap2: I don't know what I should do here, I won't have PID control or torque requesting yet

-- Phase 4 Max pressure: SPC ramps to "SPC_MAX" locks applying clutch
-- Shift solenoid OFF here

-- Phase 5 End control: SPC pressure still "SPC_MAX", MPC ramps to working pressure (MAX)
-- SHIFT COMPLETE

-- Crossover Shift end --



