







class nSensor {
  public:
    // This is your "Odometer" - must be volatile for interrupts
    volatile uint32_t pulseCount = 0;
    
    uint8_t ppr;             // Pulses Per Revolution
    uint32_t last_time = 0;    // Last time we checked (micros)
    uint32_t last_count = 0;   // Last odometer reading
    uint16_t current_rpm = 0;

    // The Constructor (The .new equivalent)
    SpeedSensor(uint16_t pulses_per_rev) {
        ppr = pulses_per_rev;
    }

    // The logic function
    uint16_t update(uint32_t now_us) {
        noInterrupts(); 
        uint32_t currentCount = pulseCount; 
        interrupts();

        uint32_t delta_t = now_us - last_time;
        uint32_t delta_p = currentCount - last_count;

        if (delta_t > 0) {
            current_rpm = (delta_p * 60000000.0f) / (ppr * delta_t);
        }

        last_time = now_us;
        last_count = total;
        return current_rpm;
    }
};

// Now you create your instances globally
SpeedSensor n2(30);
SpeedSensor n3(30);

void n2_ISR() { n2.pulseCount++; }
void n3_ISR() { n3.pulseCount++; }

void setup() {
    attachInterrupt(digitalPinToInterrupt(18), n2_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(19), n3_ISR, RISING);
}

