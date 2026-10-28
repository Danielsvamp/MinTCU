#ifndef SHIFTER_H
#define SHIFTER_H

#include <stdint.h>
#include "programselector/programselector.hpp"

/**
 * @brief Gear shifter style
 */
enum class ShifterStyle : uint8_t {
    /**
     * @brief EWM TipTronic CAN based shifter (has +/-)
     */
    EWM = 0u,
	/**
	 * @brief TRRS hard-wired shifter (Has 4/3/2/1)
	 */
	TRRS = 1u,
	/**
	 * @brief SLR Mclaran shifter
	 */
	SLR = 2u,
    /**
     * ISM - Steering column selector, EGS53 only!
     */
    ISM = 3u
};

/**
 * @brief SLR profile knob position
 */
enum class SLRProfileWheel : uint8_t {
    /**
     * @brief Right (S)
     */
    Right = 0u,
    /**
     * @brief Center (M)
     */
    Center = 1u,
    /**
     * @brief Left (C)
     */
    Left = 2u,
    /**
     * @brief Could not determine position
     */
    SNV = UINT8_MAX,
};

/**
 * @brief Gear selector position
 */
enum class ShifterPosition : uint8_t {
    /**
     * @brief Park
     */
    P,

    /**
     * @brief Between Park and Reverse
     */
    P_R,

    /**
     * @brief Reverse
     */
    R,

    /**
     * @brief Between Reverse and Neutral
     */
    R_N,

    /**
     * @brief Neutral
     */
    N,

    /**
     * @brief Between Neutral and Drive
     */
    N_D,

    /**
     * @brief Drive
     */
    D,

    /**
     * @brief EWM Shifter only - TipTronic Plus position
     */
    PLUS,

    /**
     * @brief EWM Shifter only - TipTronic Minus position
     */
    MINUS,

    /**
     * @brief TRRS Shifter only - Range restrict position '4'
     */
    FOUR,

    /**
     * @brief TRRS Shifter only - Range restrict position '3'
     */
    THREE,

    /**
     * @brief TRRS Shifter only - Range restrict position '2'
     */
    TWO,

    /**
     * @brief TRRS Shifter only - Range restrict position '1'
     */
    ONE,

    /**
     * @brief Could not determine shifter position
     */
    SignalNotAvailable = UINT8_MAX, // SNV
};

#endif
