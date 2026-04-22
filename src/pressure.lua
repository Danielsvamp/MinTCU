

function PressureManager:get_p_solenoid_current(request_mbar)
    if self.pressure_pwm_map == nil then
        return 0 -- 10% (Failsafe)
    end
    return self.pressure_pwm_map:get_value(request_mbar, self.sensor_data.atf_temp)
end

function PressureManager:get_tcc_solenoid_pwm_duty(request_mbar)
    if request_mbar == 0 then
        return 0 -- Shortcut for when off
    end
    if self.tcc_pwm_map == nil then
        return 0 -- 0% (Failsafe - TCC off)
    end
    
    return self.tcc_pwm_map:get_value(request_mbar, self.sensor_data.atf_temp)
end



function PressureManager:update_pressures(current_gear, change_state)
    -- Ignore
    if CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE) then
        -- Slave mode logic handled elsewhere or ignored
    else
        -- -- Set solenoid currents --
        if self.shift_sol_en then
            self.corrected_spc_pressure = self:calc_current_linear_sol(self.target_shift_pressure, current_gear, change_state)
            sol_spc:set_current_target(self.pressure_pwm_map:get_value(self.corrected_spc_pressure, self.sensor_data.atf_temp + 50.0))
        else
            self.corrected_spc_pressure = self:get_max_solenoid_pressure()
            sol_spc:set_current_target(0)
        end
        self.corrected_mpc_pressure = self:calc_current_linear_sol(self.target_modulating_pressure, current_gear, change_state)
        sol_mpc:set_current_target(self.pressure_pwm_map:get_value(self.corrected_mpc_pressure, self.sensor_data.atf_temp + 50.0))
        sol_tcc:set_duty(self:get_tcc_solenoid_pwm_duty(self.target_tcc_pressure))
    end
end








