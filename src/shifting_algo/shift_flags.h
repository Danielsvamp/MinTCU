#ifndef __SHIFT_FLAGS_H_
#define __SHIFT_FLAGS_H_

#ifndef BIT
#define BIT(nr) (1UL << (nr)) // From ESP-IDF
#endif

#define SHIFT_FLAG_COAST        BIT(0)
#define SHIFT_FLAG_COAST_54_43  BIT(1)
#define SHIFT_FLAG_COAST_32_21  BIT(2)
#define SHIFT_FLAG_STATIONARY   BIT(3)

#endif