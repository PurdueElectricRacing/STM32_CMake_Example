/**
 * @file common_defs.h
 * @author Adam Busch (busch8@purdue.edu)
 * @brief Common defs for the entire firmware repository. Dont let this get too out of control please.
 * @version 0.1
 * @date 2022-01-26
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef COMMON_DEFS_H_
#define COMMON_DEFS_H_

/* Math Functions */
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) < (y) ? (y) : (x))
#define CLAMP(x, min, max)  MAX((min), MIN((x), (max)))
#define ABS(x) ((x) < 0 ? (-1 * x) : x)

// Base-2 logarithm that rounds down
#define LOG2_DOWN(x) (31U - __builtin_clzl((x)))
// Base-2 logarithm that rounds up
#define LOG2_UP(x) (LOG2_DOWN((x) - 1) + 1)

#define ROUNDDOWN(a, n)						    \
({								                \
	uint32_t __a = (uint32_t) (a);		        \
	(typeof(a)) (__a - __a % (n));				\
})
// Round up to the nearest multiple of n
#define ROUNDUP(a, n)						                \
({								                            \
	uint32_t __n = (uint32_t) (n);				            \
	(typeof(a)) (ROUNDDOWN((uint32_t) (a) + __n - 1, __n));	\
})

/* Constants */
#define PI (3.14159f)

/* Unit Conversions */
#define DEG_TO_RAD (PI / 180.0f)
#define G_TO_M_S (9.80665f)

/* CAN Bus Frequency */
#define VCAN_BPS 500000 // Main vehicle CAN bus
#define MCAN_BPS 500000 // Steering angle, DAQ, (wheel temps)
#define CCAN_BPS 500000 // Charger (ELCON, A_Box, Orion)

/* Per-Node HSI RCC Trim Constants */
#define HSI_TRIM_TORQUE_VECTOR 	15
#define HSI_TRIM_MAIN_MODULE	15
#define HSI_TRIM_PDU			16
#define HSI_TRIM_DASHBOARD		15
#define HSI_TRIM_DAQ			17
#define HSI_TRIM_A_BOX			16

#endif /* COMMON_DEFS_H_ */
