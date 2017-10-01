/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 platform timer support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General	 Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ASM_MACH_JZ4740_TIMER
#define __ASM_MACH_JZ4740_TIMER

#define JZ_REG_TIMER_STOP		0x0C
#define JZ_REG_TIMER_STOP_SET		0x1C
#define JZ_REG_TIMER_STOP_CLEAR		0x2C
#define JZ_REG_TIMER_ENABLE		0x00
#define JZ_REG_TIMER_ENABLE_SET		0x04
#define JZ_REG_TIMER_ENABLE_CLEAR	0x08
#define JZ_REG_TIMER_FLAG		0x10
#define JZ_REG_TIMER_FLAG_SET		0x14
#define JZ_REG_TIMER_FLAG_CLEAR		0x18
#define JZ_REG_TIMER_MASK		0x20
#define JZ_REG_TIMER_MASK_SET		0x24
#define JZ_REG_TIMER_MASK_CLEAR		0x28

#define JZ_REG_TIMER_DFR(x) (((x) * 0x10) + 0x30)
#define JZ_REG_TIMER_DHR(x) (((x) * 0x10) + 0x34)
#define JZ_REG_TIMER_CNT(x) (((x) * 0x10) + 0x38)
#define JZ_REG_TIMER_CTRL(x) (((x) * 0x10) + 0x3C)

#define JZ_TIMER_IRQ_HALF(x) BIT((x) + 0x10)
#define JZ_TIMER_IRQ_FULL(x) BIT(x)

#define JZ_TIMER_CTRL_PWM_ABBRUPT_SHUTDOWN	BIT(9)
#define JZ_TIMER_CTRL_PWM_ACTIVE_LOW		BIT(8)
#define JZ_TIMER_CTRL_PWM_ENABLE		BIT(7)
#define JZ_TIMER_CTRL_PRESCALE_MASK		0x1c
#define JZ_TIMER_CTRL_PRESCALE_OFFSET		0x3
#define JZ_TIMER_CTRL_PRESCALE_1		(0 << 3)
#define JZ_TIMER_CTRL_PRESCALE_4		(1 << 3)
#define JZ_TIMER_CTRL_PRESCALE_16		(2 << 3)
#define JZ_TIMER_CTRL_PRESCALE_64		(3 << 3)
#define JZ_TIMER_CTRL_PRESCALE_256		(4 << 3)
#define JZ_TIMER_CTRL_PRESCALE_1024		(5 << 3)

#define JZ_TIMER_CTRL_PRESCALER(x) ((x) << JZ_TIMER_CTRL_PRESCALE_OFFSET)

#define JZ_TIMER_CTRL_SRC_EXT		BIT(2)
#define JZ_TIMER_CTRL_SRC_RTC		BIT(1)
#define JZ_TIMER_CTRL_SRC_PCLK		BIT(0)

#endif
