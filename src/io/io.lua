

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
    
    local function updatenSensors() {
    // INPUT SHAFT CALCULATION
    calc_rpm = UINT16_MAX;
    add_to_smoothed_sensor(&smoothed_sensor_n2_rpm, raw_sensors.rpm_n2);
    add_to_smoothed_sensor(&smoothed_sensor_n3_rpm, raw_sensors.rpm_n3);
    
    // OUTPUT SHAFT RPM CALCULATION
    if (Sensors::using_dedicated_output_rpm()) {
        add_to_smoothed_sensor(&smoothed_sensor_out_rpm, raw_sensors.rpm_out);
    } else {
        // Poll CANBUS
        add_to_onepoll_sensor(&onepoll_rl_speed, egs_can_hal->get_rear_left_wheel(100));
        add_to_onepoll_sensor(&onepoll_rr_speed, egs_can_hal->get_rear_right_wheel(100));
        uint16_t rl = TCUIO::wheel_rl_2x_rpm();
        uint16_t rr = TCUIO::wheel_rr_2x_rpm();
        calc_rpm = UINT16_MAX;
        if (UINT16_MAX != rl || UINT16_MAX != rr) {
            if (unlikely(UINT16_MAX == rl)) {
                // RL signal is faulty
                calc_rpm = rr;
            } else if (unlikely(UINT16_MAX == rr)) {
                // RR signal is faulty
                calc_rpm = rl;
            } else {
                // Both signals OK, take an average
                calc_rpm = (rl+rr)/2;
            }
            calc_rpm *= DIFF_RATIO_F;
            // Check transfer case if present
            if (
                VEHICLE_CONFIG.is_four_matic && 
                (VEHICLE_CONFIG.transfer_case_high_ratio != 0 && VEHICLE_CONFIG.transfer_case_low_ratio != 0)
            ) {
                if (VEHICLE_CONFIG.transfer_case_high_ratio == VEHICLE_CONFIG.transfer_case_low_ratio) {
                    // For 4Matic cars without variable ratio (Like W211)
                    //
                    // NOTE: I have never seen a vehicle with locked ratios that are not 1.0,
                    //       but, we still multiply by one of the ratios, just in case
                    //       this configuration exists somewhere
                    calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_high_ratio) / 1000.0);
                } else {
                    TransferCaseState state = egs_can_hal->get_transfer_case_state(500);
                    if (TransferCaseState::Switching == state) {
                        // Switching - Use last state
                        state = last_transfer_case_pos;
                        block_shifting = true;
                    } else {
                        block_shifting = false;
                    }
                    switch (state)
                    {
                    case TransferCaseState::Hi:
                        calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_high_ratio) / 1000.0);
                        last_transfer_case_pos = state;
                        break;
                    case TransferCaseState::Low:
                        calc_rpm *= ((float)(VEHICLE_CONFIG.transfer_case_low_ratio) / 1000.0);
                        last_transfer_case_pos = state;
                        break;
                    case TransferCaseState::Neither:
                        last_transfer_case_pos = state;
                        break; // Transfer case is disengaged, ignore
                    case TransferCaseState::Switching:
                        break; // Transfer case is switching, ignore
                    default:
                        calc_rpm = UINT16_MAX; // uh oh (Transfer case in invalid state)
                        break;
                    }
                }
            }
            if (UINT16_MAX != calc_rpm) {
                calc_rpm /= 2; // Since wheel speed is 2x
            }
        }
        add_to_smoothed_sensor(&smoothed_sensor_out_rpm, calc_rpm);
    }
}




