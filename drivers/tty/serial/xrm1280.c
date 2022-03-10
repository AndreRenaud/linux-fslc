/*
 * XRM1280 tty serial driver - Copyright (C) 2014 GridPoint
 * Author: Martin xu <martin.xu@exar.com>
 *
 *  Based on SC16IS7xx.c, by Jon Ringle <jringle@gridpoint.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
//#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>

//#define ANDRE(a,b...) pr_err("ANDRE:%s:%d: " a "\n", __FUNCTION__, __LINE__ , ##b)
#define ANDRE(a,b...) do {} while(0)

//#ifdef CONFIG_GPIOLIB
//#undef CONFIG_GPIOLIB
//#endif

#define XRM1280_NAME			"xrm1280"

/* XRM1280 register definitions */
#define XRM1280_RHR_REG		(0x00) /* RX FIFO */
#define XRM1280_THR_REG		(0x00) /* TX FIFO */

#define XRM1280_IER_REG		(0x01) /* Interrupt enable */
#define XRM1280_FCTR_REG		(0x01) /* FCTR register */
#define XRM1280_IIR_REG		(0x02) /* Interrupt Identification */
#define XRM1280_FCR_REG		(0x02) /* FIFO control */
#define XRM1280_LCR_REG		(0x03) /* Line Control */
#define XRM1280_MCR_REG		(0x04) /* Modem Control */
#define XRM1280_LSR_REG		(0x05) /* Line Status */
#define XRM1280_MSR_REG		(0x06) /* Modem Status */
#define XRM1280_SPR_REG		(0x07) /* Scratch Pad */
#define XRM1280_FC_REG		(0x07) /* TX/RX FIFO Counter Read only*/
#define XRM1280_EMSR_REG		(0x07) /* Enhanced Mode Select Register write only*/


#define XRM1280_IODIR_REG		(0x0a) /* I/O Direction
						* - only on 75x/76x
						*/
#define XRM1280_IOSTATE_REG		(0x0b) /* I/O State
						* - only on 75x/76x
						*/
#define XRM1280_IOINTENA_REG		(0x0c) /* I/O Interrupt Enable
						* - only on 75x/76x
						*/
#define XRM1280_IOCONTROL_REG		(0x0e) /* I/O Control
						* - only on 75x/76x
						*/
#define XRM1280_EFCR_REG		(0x0f) /* Extra Features Control */

/* TCR/TLR Register set: Only if ((MCR[2] == 1) && (EFR[4] == 1)) */
#define XRM1280_TCR_REG		(0x06) /* Transmit control */
#define XRM1280_TLR_REG		(0x07) /* Trigger level */

/* Special Register set: Only if ((LCR[7] == 1) && (LCR != 0xBF)) */
#define XRM1280_DLL_REG		(0x00) /* Divisor Latch Low */
#define XRM1280_DLH_REG		(0x01) /* Divisor Latch High */
#define XRM1280_DLD_REG     (0x02) /* Divisor Fractional */

/* Enhanced Register set: Only if (LCR == 0xBF) */
#define XRM1280_EFR_REG		(0x02) /* Enhanced Features */
#define XRM1280_XON1_REG		(0x04) /* Xon1 word */
#define XRM1280_XON2_REG		(0x05) /* Xon2 word */
#define XRM1280_XOFF1_REG		(0x06) /* Xoff1 word */
#define XRM1280_XOFF2_REG		(0x07) /* Xoff2 word */

/* IER register bits */
#define XRM1280_IER_RDI_BIT		(1 << 0) /* Enable RX data interrupt */
#define XRM1280_IER_THRI_BIT		(1 << 1) /* Enable TX holding register
						  * interrupt */
#define XRM1280_IER_RLSI_BIT		(1 << 2) /* Enable RX line status
						  * interrupt */
#define XRM1280_IER_MSI_BIT		(1 << 3) /* Enable Modem status
						  * interrupt */

/* IER register bits - write only if (EFR[4] == 1) */
#define XRM1280_IER_SLEEP_BIT		(1 << 4) /* Enable Sleep mode */
#define XRM1280_IER_XOFFI_BIT		(1 << 5) /* Enable Xoff interrupt */
#define XRM1280_IER_RTSI_BIT		(1 << 6) /* Enable nRTS interrupt */
#define XRM1280_IER_CTSI_BIT		(1 << 7) /* Enable nCTS interrupt */

/* FCR register bits */
#define XRM1280_FCR_FIFO_BIT		(1 << 0) /* Enable FIFO */
#define XRM1280_FCR_RXRESET_BIT	(1 << 1) /* Reset RX FIFO */
#define XRM1280_FCR_TXRESET_BIT	(1 << 2) /* Reset TX FIFO */
#define XRM1280_FCR_RXLVLL_BIT	(1 << 6) /* RX Trigger level LSB */
#define XRM1280_FCR_RXLVLH_BIT	(1 << 7) /* RX Trigger level MSB */

/* FCR register bits - write only if (EFR[4] == 1) */
#define XRM1280_FCR_TXLVLL_BIT	(1 << 4) /* TX Trigger level LSB */
#define XRM1280_FCR_TXLVLH_BIT	(1 << 5) /* TX Trigger level MSB */

/* IIR register bits */
#define XRM1280_IIR_NO_INT_BIT	(1 << 0) /* No interrupts pending */
#define XRM1280_IIR_ID_MASK		0x3e     /* Mask for the interrupt ID */
#define XRM1280_IIR_THRI_SRC		0x02     /* TX holding register empty */
#define XRM1280_IIR_RDI_SRC		0x04     /* RX data interrupt */
#define XRM1280_IIR_RLSE_SRC		0x06     /* RX line status error */
#define XRM1280_IIR_RTOI_SRC		0x0c     /* RX time-out interrupt */
#define XRM1280_IIR_MSI_SRC		0x00     /* Modem status interrupt
						  * - only on 75x/76x
						  */
#define XRM1280_IIR_INPIN_SRC		0x30     /* Input pin change of state
						  * - only on 75x/76x
						  */
#define XRM1280_IIR_XOFFI_SRC		0x10     /* Received Xoff */
#define XRM1280_IIR_CTSRTS_SRC	0x20     /* nCTS,nRTS change of state
						  * from active (LOW)
						  * to inactive (HIGH)
						  */
/* LCR register bits */
#define XRM1280_LCR_LENGTH0_BIT	(1 << 0) /* Word length bit 0 */
#define XRM1280_LCR_LENGTH1_BIT	(1 << 1) /* Word length bit 1
						  *
						  * Word length bits table:
						  * 00 -> 5 bit words
						  * 01 -> 6 bit words
						  * 10 -> 7 bit words
						  * 11 -> 8 bit words
						  */
#define XRM1280_LCR_STOPLEN_BIT	(1 << 2) /* STOP length bit
						  *
						  * STOP length bit table:
						  * 0 -> 1 stop bit
						  * 1 -> 1-1.5 stop bits if
						  *      word length is 5,
						  *      2 stop bits otherwise
						  */
#define XRM1280_LCR_PARITY_BIT	(1 << 3) /* Parity bit enable */
#define XRM1280_LCR_EVENPARITY_BIT	(1 << 4) /* Even parity bit enable */
#define XRM1280_LCR_FORCEPARITY_BIT	(1 << 5) /* 9-bit multidrop parity */
#define XRM1280_LCR_TXBREAK_BIT	(1 << 6) /* TX break enable */
#define XRM1280_LCR_DLAB_BIT		(1 << 7) /* Divisor Latch enable */
#define XRM1280_LCR_WORD_LEN_5	(0x00)
#define XRM1280_LCR_WORD_LEN_6	(0x01)
#define XRM1280_LCR_WORD_LEN_7	(0x02)
#define XRM1280_LCR_WORD_LEN_8	(0x03)
#define XRM1280_LCR_CONF_MODE_A	XRM1280_LCR_DLAB_BIT /* Special
								* reg set */
#define XRM1280_LCR_CONF_MODE_B	0xBF                   /* Enhanced
								* reg set */

/* MCR register bits */
#define XRM1280_MCR_DTR_BIT		(1 << 0) /* DTR complement
						  * - only on 75x/76x
						  */
#define XRM1280_MCR_RTS_BIT		(1 << 1) /* RTS complement */
#define XRM1280_MCR_TCRTLR_BIT	(1 << 2) /* TCR/TLR register enable */
#define XRM1280_MCR_LOOP_BIT		(1 << 4) /* Enable loopback test mode */
#define XRM1280_MCR_XONANY_BIT	(1 << 5) /* Enable Xon Any
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define XRM1280_MCR_IRDA_BIT		(1 << 6) /* Enable IrDA mode
						  * - write enabled
						  * if (EFR[4] == 1)
						  */
#define XRM1280_MCR_CLKSEL_BIT	(1 << 7) /* Divide clock by 4
						  * - write enabled
						  * if (EFR[4] == 1)
						  */

/* LSR register bits */
#define XRM1280_LSR_DR_BIT		(1 << 0) /* Receiver data ready */
#define XRM1280_LSR_OE_BIT		(1 << 1) /* Overrun Error */
#define XRM1280_LSR_PE_BIT		(1 << 2) /* Parity Error */
#define XRM1280_LSR_FE_BIT		(1 << 3) /* Frame Error */
#define XRM1280_LSR_BI_BIT		(1 << 4) /* Break Interrupt */
#define XRM1280_LSR_BRK_ERROR_MASK	0x1E     /* BI, FE, PE, OE bits */
#define XRM1280_LSR_THRE_BIT		(1 << 5) /* TX holding register empty */
#define XRM1280_LSR_TEMT_BIT		(1 << 6) /* Transmitter empty */
#define XRM1280_LSR_FIFOE_BIT		(1 << 7) /* Fifo Error */

/* MSR register bits */
#define XRM1280_MSR_DCTS_BIT		(1 << 0) /* Delta CTS Clear To Send */
#define XRM1280_MSR_DDSR_BIT		(1 << 1) /* Delta DSR Data Set Ready
						  * or (IO4)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_DRI_BIT		(1 << 2) /* Delta RI Ring Indicator
						  * or (IO7)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_DCD_BIT		(1 << 3) /* Delta CD Carrier Detect
						  * or (IO6)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_CTS_BIT		(1 << 0) /* CTS */
#define XRM1280_MSR_DSR_BIT		(1 << 1) /* DSR (IO4)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_RI_BIT		(1 << 2) /* RI (IO7)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_CD_BIT		(1 << 3) /* CD (IO6)
						  * - only on 75x/76x
						  */
#define XRM1280_MSR_DELTA_MASK	0x0F     /* Any of the delta bits! */

/*
 * TCR register bits
 * TCR trigger levels are available from 0 to 60 characters with a granularity
 * of four.
 * The programmer must program the TCR such that TCR[3:0] > TCR[7:4]. There is
 * no built-in hardware check to make sure this condition is met. Also, the TCR
 * must be programmed with this condition before auto RTS or software flow
 * control is enabled to avoid spurious operation of the device.
 */
#define XRM1280_TCR_RX_HALT(words)	((((words) / 4) & 0x0f) << 0)
#define XRM1280_TCR_RX_RESUME(words)	((((words) / 4) & 0x0f) << 4)

/*
 * TLR register bits
 * If TLR[3:0] or TLR[7:4] are logical 0, the selectable trigger levels via the
 * FIFO Control Register (FCR) are used for the transmit and receive FIFO
 * trigger levels. Trigger levels from 4 characters to 60 characters are
 * available with a granularity of four.
 *
 * When the trigger level setting in TLR is zero, the SC16IS740/750/760 uses the
 * trigger level setting defined in FCR. If TLR has non-zero trigger level value
 * the trigger level defined in FCR is discarded. This applies to both transmit
 * FIFO and receive FIFO trigger level setting.
 *
 * When TLR is used for RX trigger level control, FCR[7:6] should be left at the
 * default state, that is, '00'.
 */
#define XRM1280_TLR_TX_TRIGGER(words)	((((words) / 4) & 0x0f) << 0)
#define XRM1280_TLR_RX_TRIGGER(words)	((((words) / 4) & 0x0f) << 4)

/* IOControl register bits (Only 750/760) */
#define XRM1280_IOCONTROL_LATCH_BIT	(1 << 0) /* Enable input latching */
#define XRM1280_IOCONTROL_GPIO_BIT	(1 << 1) /* Enable GPIO[7:4] */
#define XRM1280_IOCONTROL_SRESET_BIT	(1 << 3) /* Software Reset */

/* EFCR register bits */
#define XRM1280_EFCR_9BIT_MODE_BIT	(1 << 0) /* Enable 9-bit or Multidrop
						  * mode (RS485) */
#define XRM1280_EFCR_RXDISABLE_BIT	(1 << 1) /* Disable receiver */
#define XRM1280_EFCR_TXDISABLE_BIT	(1 << 2) /* Disable transmitter */
#define XRM1280_EFCR_AUTO_RS485_BIT	(1 << 4) /* Auto RS485 RTS direction */
#define XRM1280_EFCR_RTS_INVERT_BIT	(1 << 5) /* RTS output inversion */
#define XRM1280_EFCR_IRDA_MODE_BIT	(1 << 7) /* IrDA mode
						  * 0 = rate upto 115.2 kbit/s
						  *   - Only 750/760
						  * 1 = rate upto 1.152 Mbit/s
						  *   - Only 760
						  */

/* EFR register bits */
#define XRM1280_EFR_AUTORTS_BIT	(1 << 6) /* Auto RTS flow ctrl enable */
#define XRM1280_EFR_AUTOCTS_BIT	(1 << 7) /* Auto CTS flow ctrl enable */
#define XRM1280_EFR_XOFF2_DETECT_BIT	(1 << 5) /* Enable Xoff2 detection */
#define XRM1280_EFR_ENABLE_BIT	(1 << 4) /* Enable enhanced functions
						  * and writing to IER[7:4],
						  * FCR[5:4], MCR[7:5]
						  */
#define XRM1280_EFR_SWFLOW3_BIT	(1 << 3) /* SWFLOW bit 3 */
#define XRM1280_EFR_SWFLOW2_BIT	(1 << 2) /* SWFLOW bit 2
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no transmitter flow
						  *       control
						  * 01 -> transmitter generates
						  *       XON2 and XOFF2
						  * 10 -> transmitter generates
						  *       XON1 and XOFF1
						  * 11 -> transmitter generates
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */
#define XRM1280_EFR_SWFLOW1_BIT	(1 << 1) /* SWFLOW bit 2 */
#define XRM1280_EFR_SWFLOW0_BIT	(1 << 0) /* SWFLOW bit 3
						  *
						  * SWFLOW bits 3 & 2 table:
						  * 00 -> no received flow
						  *       control
						  * 01 -> receiver compares
						  *       XON2 and XOFF2
						  * 10 -> receiver compares
						  *       XON1 and XOFF1
						  * 11 -> receiver compares
						  *       XON1, XON2, XOFF1 and
						  *       XOFF2
						  */

/* Misc definitions */
#define XRM1280_INTERNAL_FIFO_SIZE	(128) /* Size of the RX/TX fifos in the XRM1280 */
#define XRM1280_REG_SHIFT		3

struct xrm1280_devtype {
	char	name[10];
	int	nr_gpio;
	int	nr_uart;
};

#define XRM1280_FLAGS_SET_MCTRL (1 << 0)
#define XRM1280_FLAGS_IER_CLEAR (1 << 1)
#define XRM1280_FLAGS_SHUTDOWN (1 << 1)

struct xrm1280_one {
	struct uart_port		port;
	struct work_struct		tx_work;
	struct work_struct		md_work;

	struct serial_rs485		rs485;

	u32 flags;
	u8 ier;
};

struct xrm1280_port {
	struct uart_driver		uart;
	const struct xrm1280_devtype	*devtype;
	struct regmap			*regmap;
	struct mutex			mutex;
	struct clk			*clk;
#ifdef CONFIG_GPIOLIB
	struct gpio_chip		gpio;
#endif
	struct xrm1280_one		p[0];
};

#define to_xrm1280_one(p,e)	((container_of((p), struct xrm1280_one, e)))

static u8 xrm1280_port_read(struct uart_port *port, u8 reg)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	unsigned int val = 0;

	regmap_read(s->regmap,
		    (reg << XRM1280_REG_SHIFT) | port->line, &val);

	return val;
}

static void xrm1280_port_write(struct uart_port *port, u8 reg, u8 val)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);

	regmap_write(s->regmap,
		     (reg << XRM1280_REG_SHIFT) | port->line, val);
}

static void xrm1280_port_update(struct uart_port *port, u8 reg,
				  u8 mask, u8 val)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	regmap_update_bits(s->regmap,
			   (reg << XRM1280_REG_SHIFT) | port->line,
			   mask, val);
}


static void xrm1280_power(struct uart_port *port, int on)
{
	xrm1280_port_update(port, XRM1280_IER_REG,
			      XRM1280_IER_SLEEP_BIT,
			      on ? 0 : XRM1280_IER_SLEEP_BIT);
}

static const struct xrm1280_devtype xrm1280_devtype = {
	.name		= "XRM1280",
	.nr_gpio	= 0,
	.nr_uart	= 1,
};

static bool xrm1280_regmap_volatile(struct device *dev, unsigned int reg)
{
	switch (reg >> XRM1280_REG_SHIFT) {
	case XRM1280_RHR_REG:
	case XRM1280_IIR_REG:
	case XRM1280_LSR_REG:
	case XRM1280_MSR_REG:
	//case XRM1280_FC_REG:
	//case XRM1280_TXLVL_REG:
	//case XRM1280_RXLVL_REG:
	case XRM1280_IOSTATE_REG:
		return true;
	default:
		break;
	}

	return false;
}

static bool xrm1280_regmap_precious(struct device *dev, unsigned int reg)
{
	switch (reg >> XRM1280_REG_SHIFT) {
	case XRM1280_RHR_REG:
		return true;
	default:
		break;
	}

	return false;
}

static int xrm1280_set_baud(struct uart_port *port, int baud)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	u8 lcr;
	u8 prescaler = 0;
	unsigned long clk = port->uartclk, div = clk / 16 / baud;

    ANDRE("xrm1280_set_baud clk:%lu baud=%d div=%lu",clk,baud, div);

	if (div > 0xffff) {
		prescaler = XRM1280_MCR_CLKSEL_BIT;
		div /= 4;
	}

	lcr = xrm1280_port_read(port, XRM1280_LCR_REG);

	/* Open the LCR divisors for configuration */
	xrm1280_port_write(port, XRM1280_LCR_REG,
			     XRM1280_LCR_CONF_MODE_B);

	/* Enable enhanced features */
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_write(port, XRM1280_EFR_REG,
			     XRM1280_EFR_ENABLE_BIT);
	regcache_cache_bypass(s->regmap, false);

	/* Put LCR back to the normal mode */
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);

	xrm1280_port_update(port, XRM1280_MCR_REG,
			      XRM1280_MCR_CLKSEL_BIT,
			      prescaler);

	/* Open the LCR divisors for configuration */
	xrm1280_port_write(port, XRM1280_LCR_REG,
			     XRM1280_LCR_CONF_MODE_A);

	/* Write the new divisor */
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_write(port, XRM1280_DLH_REG, div / 256);
	xrm1280_port_write(port, XRM1280_DLL_REG, div % 256);
	ANDRE("dld: 0x%x", xrm1280_port_read(port, XRM1280_DLD_REG));
	ANDRE("baud:%d msb:%lu lsb=%lu lcr:%d\n",baud,div / 256,div % 256,lcr);
	
	regcache_cache_bypass(s->regmap, false);

	/* Put LCR back to the normal mode */
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);

	return DIV_ROUND_CLOSEST(clk / 16, div);
}

static void xrm1280_handle_rx(struct uart_port *port, unsigned long rxlen,
				unsigned int iir)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	unsigned int lsr = 0, ch, flag, bytes_read, i;
	bool read_lsr = (iir == XRM1280_IIR_RLSE_SRC) ? true : false;
	u8 buf[XRM1280_INTERNAL_FIFO_SIZE];

#if 0
	if (unlikely(rxlen >= sizeof(s->buf))) {
		dev_warn_ratelimited(port->dev,
				     "Port %i: Possible RX FIFO overrun: %d\n",
				     port->line, rxlen);
		port->icount.buf_overrun++;
		/* Ensure sanity of RX level */
		rxlen = sizeof(s->buf);
	}
#endif

	while (rxlen) {
		/* Only read lsr if there are possible errors in FIFO */
		if (read_lsr) {
			lsr = xrm1280_port_read(port, XRM1280_LSR_REG);
			if (!(lsr & XRM1280_LSR_FIFOE_BIT))
				read_lsr = false; /* No errors left in FIFO */
		} else
			lsr = 0;

		if (read_lsr) {
			buf[0] = xrm1280_port_read(port, XRM1280_RHR_REG);
			bytes_read = 1;
		} else {
			bytes_read = min(rxlen, sizeof(buf));
			regcache_cache_bypass(s->regmap, true);
			regmap_raw_read(s->regmap, XRM1280_RHR_REG << XRM1280_REG_SHIFT,
					buf, bytes_read);
			regcache_cache_bypass(s->regmap, false);
		}

		lsr &= XRM1280_LSR_BRK_ERROR_MASK;

		port->icount.rx += bytes_read;
		flag = TTY_NORMAL;

		if (unlikely(lsr)) {
			if (lsr & XRM1280_LSR_BI_BIT) {
				port->icount.brk++;
				if (uart_handle_break(port))
					continue;
			} else if (lsr & XRM1280_LSR_PE_BIT)
				port->icount.parity++;
			else if (lsr & XRM1280_LSR_FE_BIT)
				port->icount.frame++;
			else if (lsr & XRM1280_LSR_OE_BIT)
				port->icount.overrun++;

			lsr &= port->read_status_mask;
			if (lsr & XRM1280_LSR_BI_BIT)
				flag = TTY_BREAK;
			else if (lsr & XRM1280_LSR_PE_BIT)
				flag = TTY_PARITY;
			else if (lsr & XRM1280_LSR_FE_BIT)
				flag = TTY_FRAME;
			else if (lsr & XRM1280_LSR_OE_BIT)
				flag = TTY_OVERRUN;
		}

		ANDRE("f=0x%x br=%d lsr=0x%x", flag, bytes_read, lsr);
		for (i = 0; i < bytes_read; ++i) {
			ch = buf[i];
			ANDRE("%d:%c", i, ch);
			if (uart_handle_sysrq_char(port, ch))
				continue;

			if (lsr & port->ignore_status_mask)
				continue;

			uart_insert_char(port, lsr, XRM1280_LSR_OE_BIT, ch,
					 flag);
		}
		rxlen -= bytes_read;
	}
	
      tty_flip_buffer_push(&port->state->port);
}

static void xrm1280_handle_tx(struct uart_port *port)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int txlen, to_send, i;
	char buf[XRM1280_INTERNAL_FIFO_SIZE];

  	if (unlikely(port->x_char)) {
		xrm1280_port_write(port, XRM1280_THR_REG, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return;

	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);
	if (likely(to_send)) {
		/* Limit to size of TX FIFO */
		#if 1
		xrm1280_port_write(port, XRM1280_EMSR_REG, 1);// read tx_fifo counter
		txlen = xrm1280_port_read(port, XRM1280_FC_REG);
	   	#else
		txlen = xrm1280_port_read(port, XRM1280_TXLVL_REG);
		#endif
		ANDRE("txlen:%d -> %d",txlen, XRM1280_INTERNAL_FIFO_SIZE - txlen);
		txlen = XRM1280_INTERNAL_FIFO_SIZE - txlen;
		to_send = min(txlen, to_send);

		/* Add data to send */
		port->icount.tx += to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			buf[i] = xmit->buf[xmit->tail];
			ANDRE("%d:%c", i, buf[i]);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}
		regcache_cache_bypass(s->regmap, true);
		regmap_raw_write(s->regmap, XRM1280_THR_REG << XRM1280_REG_SHIFT, buf, to_send);
		regcache_cache_bypass(s->regmap, false);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void xrm1280_port_irq(struct xrm1280_port *s, int portno)
{
	struct uart_port *port = &s->p[portno].port;
    //u8 fctr;
	do {
		unsigned int iir, msr, rxlen;

		iir = xrm1280_port_read(port, XRM1280_IIR_REG);
		if (iir & XRM1280_IIR_NO_INT_BIT)
			break;

		iir &= XRM1280_IIR_ID_MASK;

		switch (iir) {
		case XRM1280_IIR_RDI_SRC:
		case XRM1280_IIR_RLSE_SRC:
		case XRM1280_IIR_RTOI_SRC:
		case XRM1280_IIR_XOFFI_SRC:
		#if 1
		regcache_cache_bypass(s->regmap, true);
		xrm1280_port_write(port, XRM1280_EMSR_REG, 0);//set FCTR[7] = 0 for read rx_fifo counter
		rxlen = xrm1280_port_read(port, XRM1280_FC_REG);
		regcache_cache_bypass(s->regmap, false);
		#else
			rxlen = xrm1280_port_read(port, XRM1280_RXLVL_REG);
		#endif
			if (rxlen) {
				mutex_lock(&s->mutex);
				xrm1280_handle_rx(port, rxlen, iir);
				mutex_unlock(&s->mutex);
			}
			break;

		case XRM1280_IIR_CTSRTS_SRC:
			msr = xrm1280_port_read(port, XRM1280_MSR_REG);
			uart_handle_cts_change(port,
					       !!(msr & XRM1280_MSR_CTS_BIT));
			break;
		case XRM1280_IIR_THRI_SRC:
			mutex_lock(&s->mutex);
			xrm1280_handle_tx(port);
			mutex_unlock(&s->mutex);
			break;
		default:
			dev_err_ratelimited(port->dev,
					    "Port %i: Unexpected interrupt: %x",
					    port->line, iir);
			break;
		}
	} while (1);
}

static irqreturn_t xrm1280_ist(int irq, void *dev_id)
{
	struct xrm1280_port *s = (struct xrm1280_port *)dev_id;
	int i;

	for (i = 0; i < s->uart.nr; ++i)
		xrm1280_port_irq(s, i);

	return IRQ_HANDLED;
}

static void xrm1280_wq_proc(struct work_struct *ws)
{
	struct xrm1280_one *one = to_xrm1280_one(ws, tx_work);
	struct xrm1280_port *s = dev_get_drvdata(one->port.dev);

	mutex_lock(&s->mutex);
	xrm1280_handle_tx(&one->port);
	mutex_unlock(&s->mutex);
}

static void xrm1280_stop_tx(struct uart_port* port)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);
	struct circ_buf *xmit = &one->port.state->xmit;

	/* handle rs485 */
	if (one->rs485.flags & SER_RS485_ENABLED) {
		/* do nothing if current tx not yet completed */
		int lsr = xrm1280_port_read(port, XRM1280_LSR_REG);
		if (!(lsr & XRM1280_LSR_TEMT_BIT))
			return;

		if (uart_circ_empty(xmit) &&
		    (one->rs485.delay_rts_after_send > 0))
			mdelay(one->rs485.delay_rts_after_send);
	}

	xrm1280_port_update(port, XRM1280_IER_REG,
			      XRM1280_IER_THRI_BIT,
			      0);
}

static void xrm1280_stop_rx(struct uart_port* port)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	one->port.read_status_mask &= ~XRM1280_LSR_DR_BIT;
	one->flags |= XRM1280_FLAGS_IER_CLEAR;
	one->ier |= XRM1280_LSR_DR_BIT;
	schedule_work(&one->md_work);
}

static void xrm1280_start_tx(struct uart_port *port)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	/* handle rs485 */
	if ((one->rs485.flags & SER_RS485_ENABLED) &&
	    (one->rs485.delay_rts_before_send > 0)) {
		mdelay(one->rs485.delay_rts_before_send);
	}

	if (!work_pending(&one->tx_work))
		schedule_work(&one->tx_work);
}
#if 0
static unsigned int xrm1280_tx_empty(struct uart_port *port)
{
	unsigned int lvl, lsr;

	lvl = xrm1280_port_read(port, XRM1280_TXLVL_REG);
	
	lsr = xrm1280_port_read(port, XRM1280_LSR_REG);

	return ((lsr & XRM1280_LSR_THRE_BIT) && !lvl) ? TIOCSER_TEMT : 0;
}
#else
static unsigned int xrm1280_tx_empty(struct uart_port *port)
{
	//unsigned int lvl;
    unsigned int lsr;

	//lvl = xrm1280_port_read(port, XRM1280_TXLVL_REG);
	
	lsr = xrm1280_port_read(port, XRM1280_LSR_REG);

	return (lsr & XRM1280_LSR_THRE_BIT) ? TIOCSER_TEMT : 0;
}

#endif
static unsigned int xrm1280_get_mctrl(struct uart_port *port)
{
	/* DCD and DSR are not wired and CTS/RTS is handled automatically
	 * so just indicate DSR and CAR asserted
	 */
	return TIOCM_DSR | TIOCM_CAR;
}

static void xrm1280_md_proc(struct work_struct *ws)
{
	struct xrm1280_one *one = to_xrm1280_one(ws, md_work);

	if (one->flags & XRM1280_FLAGS_IER_CLEAR) {
		xrm1280_port_update(&one->port, XRM1280_IER_REG, one->ier, 0);
		one->ier = 0;
	}
	if (one->flags & XRM1280_FLAGS_SHUTDOWN) {
		/* Disable TX/RX */
		xrm1280_port_write(&one->port, XRM1280_EFCR_REG,
			     XRM1280_EFCR_RXDISABLE_BIT |
			     XRM1280_EFCR_TXDISABLE_BIT);

		xrm1280_power(&one->port, 0);
	}
	if (one->flags & XRM1280_FLAGS_SET_MCTRL)
		xrm1280_port_update(&one->port, XRM1280_MCR_REG,
			      XRM1280_MCR_LOOP_BIT,
			      (one->port.mctrl & TIOCM_LOOP) ?
				      XRM1280_MCR_LOOP_BIT : 0);
	one->flags = 0;
}

static void xrm1280_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	one->flags |= XRM1280_FLAGS_SET_MCTRL;

	schedule_work(&one->md_work);
}

static void xrm1280_break_ctl(struct uart_port *port, int break_state)
{
	xrm1280_port_update(port, XRM1280_LCR_REG,
			      XRM1280_LCR_TXBREAK_BIT,
			      break_state ? XRM1280_LCR_TXBREAK_BIT : 0);
}

static void xrm1280_set_termios(struct uart_port *port,
				  struct ktermios *termios,
				  struct ktermios *old)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	unsigned int lcr, flow = 0;
	int baud;
	/* Mask termios capabilities we don't support */
	termios->c_cflag &= ~CMSPAR;

	/* Word size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = XRM1280_LCR_WORD_LEN_5;
		break;
	case CS6:
		lcr = XRM1280_LCR_WORD_LEN_6;
		break;
	case CS7:
		lcr = XRM1280_LCR_WORD_LEN_7;
		break;
	case CS8:
		lcr = XRM1280_LCR_WORD_LEN_8;
		break;
	default:
		lcr = XRM1280_LCR_WORD_LEN_8;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= CS8;
		break;
	}

	/* Parity */
	if (termios->c_cflag & PARENB) {
		lcr |= XRM1280_LCR_PARITY_BIT;
		if (!(termios->c_cflag & PARODD))
			lcr |= XRM1280_LCR_EVENPARITY_BIT;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB)
		lcr |= XRM1280_LCR_STOPLEN_BIT; /* 2 stops */

	/* Set read status mask */
	port->read_status_mask = XRM1280_LSR_OE_BIT;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= XRM1280_LSR_PE_BIT |
					  XRM1280_LSR_FE_BIT;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= XRM1280_LSR_BI_BIT;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= XRM1280_LSR_BI_BIT;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= XRM1280_LSR_BRK_ERROR_MASK;

	xrm1280_port_write(port, XRM1280_LCR_REG,
			     XRM1280_LCR_CONF_MODE_B);

	/* Configure flow control */
	regcache_cache_bypass(s->regmap, true);
	xrm1280_port_write(port, XRM1280_XON1_REG, termios->c_cc[VSTART]);
	xrm1280_port_write(port, XRM1280_XOFF1_REG, termios->c_cc[VSTOP]);
	if (termios->c_cflag & CRTSCTS)
		flow |= XRM1280_EFR_AUTOCTS_BIT |
			XRM1280_EFR_AUTORTS_BIT;
	if (termios->c_iflag & IXON)
		flow |= XRM1280_EFR_SWFLOW3_BIT;
	if (termios->c_iflag & IXOFF)
		flow |= XRM1280_EFR_SWFLOW1_BIT;

	xrm1280_port_write(port, XRM1280_EFR_REG, flow);
	regcache_cache_bypass(s->regmap, false);

	/* Update LCR register */
	xrm1280_port_write(port, XRM1280_LCR_REG, lcr);
    
	/* Get baud rate generator configuration */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 4 / 0xffff,
				  port->uartclk);

	/* Setup baudrate generator */
	baud = xrm1280_set_baud(port, baud);

	/* Update timeout according to new baud rate */
	uart_update_timeout(port, termios->c_cflag, baud);
}

#if defined(TIOCSRS485) && defined(TIOCGRS485)
static void xrm1280_config_rs485(struct uart_port *port,
				   struct serial_rs485 *rs485)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	one->rs485 = *rs485;

	if (one->rs485.flags & SER_RS485_ENABLED) {
		xrm1280_port_update(port, XRM1280_EFCR_REG,
				      XRM1280_EFCR_AUTO_RS485_BIT,
				      XRM1280_EFCR_AUTO_RS485_BIT);
	} else {
		xrm1280_port_update(port, XRM1280_EFCR_REG,
				      XRM1280_EFCR_AUTO_RS485_BIT,
				      0);
	}
}
#endif

static int xrm1280_ioctl(struct uart_port *port, unsigned int cmd,
			   unsigned long arg)
{
#if defined(TIOCSRS485) && defined(TIOCGRS485)
		struct serial_rs485 rs485;
	
		switch (cmd) {
		case TIOCSRS485:
			if (copy_from_user(&rs485, (void __user *)arg, sizeof(rs485)))
				return -EFAULT;
	
			xrm1280_config_rs485(port, &rs485);
			return 0;
		case TIOCGRS485:
			if (copy_to_user((void __user *)arg,
					 &(to_xrm1280_one(port, port)->rs485),
					 sizeof(rs485)))
				return -EFAULT;
			return 0;
		default:
			break;
		}
#endif
	   return -ENOIOCTLCMD;


	return -ENOIOCTLCMD;
}

static int xrm1280_startup(struct uart_port *port)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);
	unsigned int val;
	
	u8 fifo_count;
   	xrm1280_power(port, 1);
	/* Reset FIFOs*/
	val = XRM1280_FCR_RXRESET_BIT | XRM1280_FCR_TXRESET_BIT;
	xrm1280_port_write(port, XRM1280_FCR_REG, val);
	udelay(5);
	xrm1280_port_write(port, XRM1280_FCR_REG,
			     XRM1280_FCR_FIFO_BIT);

	/* Enable EFR */
	xrm1280_port_write(port, XRM1280_LCR_REG,
			     XRM1280_LCR_CONF_MODE_B);

	regcache_cache_bypass(s->regmap, true);

	/* Enable write access to enhanced features and internal clock div */
	xrm1280_port_write(port, XRM1280_EFR_REG,
			     XRM1280_EFR_ENABLE_BIT);

	/* Enable TCR/TLR */
	xrm1280_port_update(port, XRM1280_MCR_REG,
			      XRM1280_MCR_TCRTLR_BIT,
			      XRM1280_MCR_TCRTLR_BIT);


	/* Configure flow control levels */
	/* Flow control halt level 48, resume level 24 */
/*	
	xrm1280_port_write(port, XRM1280_TCR_REG,
			     XRM1280_TCR_RX_RESUME(24) |
			     XRM1280_TCR_RX_HALT(48));*/
			     
	xrm1280_port_write(port, XRM1280_FCTR_REG,0x40);
	regcache_cache_bypass(s->regmap, false);

	/* Now, initialize the UART */
	xrm1280_port_write(port, XRM1280_LCR_REG, XRM1280_LCR_WORD_LEN_8);


	/* Enable the Rx and Tx FIFO */
	/*
	xrm1280_port_update(port, XRM1280_EFCR_REG,
			      XRM1280_EFCR_RXDISABLE_BIT |
			      XRM1280_EFCR_TXDISABLE_BIT,
			      0);*/
			      
	regcache_cache_bypass(s->regmap, true);
    xrm1280_port_write(port, XRM1280_EMSR_REG,1);//set FCTR[7] = 0 for read tx_fifo counter
	fifo_count = xrm1280_port_read(port, XRM1280_FC_REG);
	//printk("tx fifo_count =%d\n",fifo_count);
	xrm1280_port_write(port, XRM1280_EMSR_REG,0);//set FCTR[7] = 0 for read rx_fifo counter
	fifo_count = xrm1280_port_read(port, XRM1280_FC_REG);
	//printk("rx fifo_count =%d\n",fifo_count);
	regcache_cache_bypass(s->regmap, false);
	
	/* Enable RX, TX, CTS change interrupts */
	val = XRM1280_IER_RDI_BIT | XRM1280_IER_THRI_BIT |
	      XRM1280_IER_CTSI_BIT;
	xrm1280_port_write(port, XRM1280_IER_REG, val);

	return 0;
}

static void xrm1280_shutdown(struct uart_port *port)
{
	struct xrm1280_one *one = to_xrm1280_one(port, port);

	/* Disable all interrupts */
	one->ier = 0xff;
	one->flags |= XRM1280_FLAGS_SHUTDOWN | XRM1280_FLAGS_IER_CLEAR;

	schedule_work(&one->md_work);
}

static const char *xrm1280_type(struct uart_port *port)
{
	struct xrm1280_port *s = dev_get_drvdata(port->dev);

	return (port->type == PORT_XRM1280) ? s->devtype->name : NULL;
}

static int xrm1280_request_port(struct uart_port *port)
{
	/* Do nothing */
	return 0;
}

static void xrm1280_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_XRM1280;
}

static int xrm1280_verify_port(struct uart_port *port,
				 struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_XRM1280))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

static void xrm1280_pm(struct uart_port *port, unsigned int state,
			 unsigned int oldstate)
{
	xrm1280_power(port, (state == UART_PM_STATE_ON) ? 1 : 0);
}

static void xrm1280_null_void(struct uart_port *port)
{
	/* Do nothing */
}

static const struct uart_ops xrm1280_ops = {
	.tx_empty	= xrm1280_tx_empty,
	.set_mctrl	= xrm1280_set_mctrl,
	.get_mctrl	= xrm1280_get_mctrl,
	.stop_tx	= xrm1280_stop_tx,
	.start_tx	= xrm1280_start_tx,
	.stop_rx	= xrm1280_stop_rx,
	.break_ctl	= xrm1280_break_ctl,
	.startup	= xrm1280_startup,
	.shutdown	= xrm1280_shutdown,
	.set_termios	= xrm1280_set_termios,
	.type		= xrm1280_type,
	.request_port	= xrm1280_request_port,
	.release_port	= xrm1280_null_void,
	.config_port	= xrm1280_config_port,
	.verify_port	= xrm1280_verify_port,
	.ioctl		= xrm1280_ioctl,
	.pm		= xrm1280_pm,
};

#ifdef CONFIG_GPIOLIB
static int xrm1280_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	unsigned int val;
	struct xrm1280_port *s = container_of(chip, struct xrm1280_port,
						gpio);
	struct uart_port *port = &s->p[0].port;

	val = xrm1280_port_read(port, XRM1280_IOSTATE_REG);

	return !!(val & BIT(offset));
}

static void xrm1280_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct xrm1280_port *s = container_of(chip, struct xrm1280_port,
						gpio);
	struct uart_port *port = &s->p[0].port;

	xrm1280_port_update(port, XRM1280_IOSTATE_REG, BIT(offset),
			      val ? BIT(offset) : 0);
}

static int xrm1280_gpio_direction_input(struct gpio_chip *chip,
					  unsigned offset)
{
	struct xrm1280_port *s = container_of(chip, struct xrm1280_port,
						gpio);
	struct uart_port *port = &s->p[0].port;

	xrm1280_port_update(port, XRM1280_IODIR_REG, BIT(offset), 0);

	return 0;
}

static int xrm1280_gpio_direction_output(struct gpio_chip *chip,
					   unsigned offset, int val)
{
	struct xrm1280_port *s = container_of(chip, struct xrm1280_port,
						gpio);
	struct uart_port *port = &s->p[0].port;

	xrm1280_port_update(port, XRM1280_IOSTATE_REG, BIT(offset),
			      val ? BIT(offset) : 0);
	xrm1280_port_update(port, XRM1280_IODIR_REG, BIT(offset),
			      BIT(offset));

	return 0;
}
#endif

static int xrm1280_probe(struct device *dev,
			   const struct xrm1280_devtype *devtype,
			   struct regmap *regmap, int irq)
{
	unsigned long freq, *pfreq = dev_get_platdata(dev);
	int i, ret;
	struct xrm1280_port *s;
	u32 uartclk = 0;

  	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Alloc port structure */
	s = devm_kzalloc(dev, sizeof(*s) +
			 sizeof(struct xrm1280_one) * devtype->nr_uart,
			 GFP_KERNEL);
	if (!s) {
		dev_err(dev, "Error allocating port structure\n");
		return -ENOMEM;
	}

	/* Always ask for fixed clock rate from a property. */
	device_property_read_u32(dev, "clock-frequency", &uartclk);

 	s->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(s->clk)) {
		if (uartclk)
			freq = uartclk;
		if (pfreq)
			freq = *pfreq;
	} else {
		freq = clk_get_rate(s->clk);
	}
	if (!freq)
		return PTR_ERR(s->clk);

	s->regmap = regmap;
	s->devtype = devtype;
	dev_set_drvdata(dev, s);

	/* Register UART driver */
	s->uart.owner		= THIS_MODULE;
	s->uart.dev_name	= "ttyXRM";
	s->uart.nr		= devtype->nr_uart;
	ret = uart_register_driver(&s->uart);
	if (ret) {
		dev_err(dev, "Registering UART driver failed\n");
		goto out_clk;
	}

#ifdef CONFIG_GPIOLIB
	if (devtype->nr_gpio) {
		/* Setup GPIO cotroller */
		s->gpio.owner		 = THIS_MODULE;
		s->gpio.parent		 = dev;
		s->gpio.label		 = dev_name(dev);
		s->gpio.direction_input	 = xrm1280_gpio_direction_input;
		s->gpio.get		 = xrm1280_gpio_get;
		s->gpio.direction_output = xrm1280_gpio_direction_output;
		s->gpio.set		 = xrm1280_gpio_set;
		s->gpio.base		 = -1;
		s->gpio.ngpio		 = devtype->nr_gpio;
		s->gpio.can_sleep	 = 1;
		ret = gpiochip_add(&s->gpio);
		if (ret)
			goto out_uart;
	}
#endif

	mutex_init(&s->mutex);

	for (i = 0; i < devtype->nr_uart; ++i) {
		/* Initialize port data */
		s->p[i].port.line	= i;
		s->p[i].port.dev	= dev;
		s->p[i].port.irq	= irq;
		s->p[i].port.type	= PORT_XRM1280;
		s->p[i].port.fifosize	= 128;
		s->p[i].port.flags	= UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		s->p[i].port.iotype	= UPIO_PORT;
		s->p[i].port.uartclk	= freq;
		s->p[i].port.ops	= &xrm1280_ops;
		/* Disable all interrupts */
		xrm1280_port_write(&s->p[i].port, XRM1280_IER_REG, 0);
		/* Disable TX/RX */
		xrm1280_port_write(&s->p[i].port, XRM1280_EFCR_REG,
				     XRM1280_EFCR_RXDISABLE_BIT |
				     XRM1280_EFCR_TXDISABLE_BIT);
		/* Initialize queue for start TX */
		INIT_WORK(&s->p[i].tx_work, xrm1280_wq_proc);
		/* Initialize queue for changing mode */
		INIT_WORK(&s->p[i].md_work, xrm1280_md_proc);
		/* Register port */
		uart_add_one_port(&s->uart, &s->p[i].port);
		/* Go to suspend mode */
		xrm1280_power(&s->p[i].port, 0);
	}
    
	/* Setup interrupt */
	ret = devm_request_threaded_irq(dev, irq, NULL, xrm1280_ist,
					IRQF_ONESHOT | IRQF_TRIGGER_LOW, dev_name(dev), s);
	//printk("xrm1280_probe devm_request_threaded_irq =%d\n",ret);
	
	if (ret != 0)
		goto out_mutex;

	return 0;

out_mutex:
	mutex_destroy(&s->mutex);

#ifdef CONFIG_GPIOLIB
	if (devtype->nr_gpio)
		gpiochip_remove(&s->gpio);

out_uart:
#endif
	uart_unregister_driver(&s->uart);

out_clk:
	if (!IS_ERR(s->clk))
		clk_disable_unprepare(s->clk);

	return ret;
}

static int xrm1280_remove(struct device *dev)
{
	struct xrm1280_port *s = dev_get_drvdata(dev);
	int i;

#ifdef CONFIG_GPIOLIB
	if (s->devtype->nr_gpio)
		gpiochip_remove(&s->gpio);
#endif

	for (i = 0; i < s->uart.nr; i++) {
		cancel_work_sync(&s->p[i].tx_work);
		cancel_work_sync(&s->p[i].md_work);
		uart_remove_one_port(&s->uart, &s->p[i].port);
		xrm1280_power(&s->p[i].port, 0);
	}

	mutex_destroy(&s->mutex);
	uart_unregister_driver(&s->uart);
	if (!IS_ERR(s->clk))
		clk_disable_unprepare(s->clk);

	return 0;
}


static const struct of_device_id __maybe_unused xrm1280_dt_ids[] = {
	{ .compatible = "exar,xrm1280",	.data = &xrm1280_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(of, xrm1280_dt_ids);

static struct regmap_config regcfg = {
	.reg_bits = 8,
	.pad_bits = 0,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = xrm1280_regmap_volatile,
	.precious_reg = xrm1280_regmap_precious,
};

#if 0
static int xrm1280_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
{
	struct xrm1280_devtype *devtype;
	struct regmap *regmap;
	int ret;
   	if (i2c->dev.of_node) {
		const struct of_device_id *of_id =
				of_match_device(xrm1280_dt_ids, &i2c->dev);

		devtype = (struct xrm1280_devtype *)of_id->data;
	} else {
		devtype = (struct xrm1280_devtype *)id->driver_data;
	}

	regcfg.max_register = (0xf << XRM1280_REG_SHIFT) |
			      (devtype->nr_uart - 1);
	regmap = devm_regmap_init_i2c(i2c, &regcfg);
	ret = xrm1280_probe(&i2c->dev, devtype, regmap, i2c->irq);
	return ret;
}

static int xrm1280_i2c_remove(struct i2c_client *client)
{
	return xrm1280_remove(&client->dev);
}

static const struct i2c_device_id xrm1280_i2c_id_table[] = {
	{ "xrm1280",	(kernel_ulong_t)&xrm1280_devtype, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, xrm1280_i2c_id_table);

static struct i2c_driver xrm1280_i2c_uart_driver = {
	.driver = {
		.name		= XRM1280_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(xrm1280_dt_ids),
	},
	.probe		= xrm1280_i2c_probe,
	.remove		= xrm1280_i2c_remove,
	.id_table	= xrm1280_i2c_id_table,
};
module_i2c_driver(xrm1280_i2c_uart_driver);
MODULE_ALIAS("i2c:xrm1280");
#endif

static int xrm1280_spi_probe(struct spi_device *spi)
{
	const struct xrm1280_devtype *devtype;
	struct regmap *regmap;
	int ret;

	/* Setup SPI bus */
	spi->bits_per_word	= 8;
	/* only supports mode 0 on XRM1280 */
	spi->mode		= spi->mode ? : SPI_MODE_0;
	spi->max_speed_hz	= spi->max_speed_hz ? : 15000000;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	if (spi->dev.of_node) {
		devtype = device_get_match_data(&spi->dev);
		if (!devtype)
			return -ENODEV;
	} else {
		const struct spi_device_id *id_entry = spi_get_device_id(spi);

		devtype = (struct xrm1280_devtype *)id_entry->driver_data;
	}

	regcfg.max_register = (0xf << XRM1280_REG_SHIFT) |
			      (devtype->nr_uart - 1);
	regmap = devm_regmap_init_spi(spi, &regcfg);

	return xrm1280_probe(&spi->dev, devtype, regmap, spi->irq);
}

static int xrm1280_spi_remove(struct spi_device *spi)
{
	return xrm1280_remove(&spi->dev);
}

static const struct spi_device_id xrm1280_spi_id_table[] = {
	{ "xrm1280",	(kernel_ulong_t)&xrm1280_devtype, },
	{ }
};

MODULE_DEVICE_TABLE(spi, xrm1280_spi_id_table);

static struct spi_driver xrm1280_spi_uart_driver = {
	.driver = {
		.name		= XRM1280_NAME,
		.of_match_table	= xrm1280_dt_ids,
	},
	.probe		= xrm1280_spi_probe,
	.remove		= xrm1280_spi_remove,
	.id_table	= xrm1280_spi_id_table,
};

module_spi_driver(xrm1280_spi_uart_driver);
MODULE_ALIAS("spi:xrm1280");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin xu <Martin.xu@exar.com>");
MODULE_DESCRIPTION("XR20M1280 serial driver");
