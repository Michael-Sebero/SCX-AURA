/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 Andrea Righi <andrea.righi@linux.dev>
 *
 * This software may be used and distributed according to the terms of the GNU
 * General Public License version 2.
 */
#ifndef __INTF_H
#define __INTF_H

#include <limits.h>

#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define CLAMP(val, lo, hi) MIN(MAX(val, lo), hi)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

enum consts {
	NSEC_PER_USEC = 1000ULL,
	NSEC_PER_MSEC = (1000ULL * NSEC_PER_USEC),
	NSEC_PER_SEC = (1000ULL * NSEC_PER_MSEC),

	/* Kernel definitions */
	CLOCK_BOOTTIME		= 7,
};

/*
 * QoS tiers, highest priority first. They map onto the XNU Clutch
 * timeshare root buckets FG, DF, UT and BG. NR_TIERS must stay a power of
 * two: tier indexes are masked with (NR_TIERS - 1) to bound array accesses
 * for the verifier.
 */
enum aura_tier {
	TIER_INTERACTIVE	= 0,
	TIER_DEFAULT		= 1,
	TIER_UTILITY		= 2,
	TIER_BACKGROUND		= 3,
	NR_TIERS		= 4,
};

#ifndef __VMLINUX_H__
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long u64;

typedef signed char s8;
typedef signed short s16;
typedef signed int s32;
typedef signed long s64;

typedef int pid_t;
#endif /* __VMLINUX_H__ */

struct cpu_arg {
	s32 cpu_id;
};

struct domain_arg {
	s32 cpu_id;
	s32 sibling_cpu_id;
};

#endif /* __INTF_H */
