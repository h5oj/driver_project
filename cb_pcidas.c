/*
 * Analogy driver for Measurement Computing Boards
 *
 * Copyright (C) 2009 Simon Boulay <simon.boulay@gmail.com>
 *
 * Derived from comedi:
 * Copyright (C) 2000 David A. Schleef <ds@schleef.org>
 *               2006 Everett Wang <everett.wang@everteq.com>
 *               2009 Ian Abbott <abbotti@mev.co.uk>
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
/*
 * Original code comes from comedi linux-next staging driver (2009.12.20)
 * Board documentation: http://www.sensoray.com/products/526data.htm
 * Everything should work as in comedi:
 *   - Encoder works
 *   - Analog input works
 *   - Analog output works
 *   - PWM output works
 *   - Commands are not supported yet.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <asm/byteorder.h>
#include <rtdm/analogy/device.h>

#include "cb_pcidas.h"

/* 8255 related */
#define _8255_SIZE	4
#define _8255_BASE      4
#define _8255_DATA	0
#define _8255_CR	3

#define CR_C_LO_IO	0x01
#define CR_B_IO		0x02
#define CR_B_MODE	0x04
#define CR_C_HI_IO	0x08
#define CR_A_IO		0x10
#define CR_A_MODE(a)	((a)<<5)
#define CR_CW		0x80

/* Board description */
#define S526_GPCT_CHANS	4
#define S526_GPCT_BITS	24
#define S526_AI_CHANS	10	/* 8 regular differential inputs
				 * channel 8 is "reference 0" (+10V)
				 * channel 9 is "reference 1" (0V) */
#define S526_AI_BITS	16
#define S526_AI_TIMEOUT 100
#define S526_AO_CHANS	4
#define S526_AO_BITS	16

#define CB_PCIDAS_DIO_CHANS	8
#define CB_PCIDAS_DIO_BITS	1

/* Ports */
#define S526_IOSIZE		0x40  /* 64 bytes */
#define S526_DEFAULT_ADDRESS	0x2C0 /* Manufacturing default */

/* Registers */
#define REG_TCR 0x00
#define REG_WDC 0x02
#define REG_DAC 0x04
#define REG_ADC 0x06
#define REG_ADD 0x08
#define REG_DIO 0x0A
#define REG_IER 0x0C
#define REG_ISR 0x0E
#define REG_MSC 0x10
#define REG_C0L 0x12
#define REG_C0H 0x14
#define REG_C0M 0x16
#define REG_C0C 0x18
#define REG_C1L 0x1A
#define REG_C1H 0x1C
#define REG_C1M 0x1E
#define REG_C1C 0x20
#define REG_C2L 0x22
#define REG_C2H 0x24
#define REG_C2M 0x26
#define REG_C2C 0x28
#define REG_C3L 0x2A
#define REG_C3H 0x2C
#define REG_C3M 0x2E
#define REG_C3C 0x30
#define REG_EED 0x32
#define REG_EEC 0x34

#define ISR_ADC_DONE 0x4

struct counter_mode_register_t {
#if defined (__LITTLE_ENDIAN_BITFIELD)
	unsigned short coutSource:1;
	unsigned short coutPolarity:1;
	unsigned short autoLoadResetRcap:3;
	unsigned short hwCtEnableSource:2;
	unsigned short ctEnableCtrl:2;
	unsigned short clockSource:2;
	unsigned short countDir:1;
	unsigned short countDirCtrl:1;
	unsigned short outputRegLatchCtrl:1;
	unsigned short preloadRegSel:1;
	unsigned short reserved:1;
#elif defined(__BIG_ENDIAN_BITFIELD)
	unsigned short reserved:1;
	unsigned short preloadRegSel:1;
	unsigned short outputRegLatchCtrl:1;
	unsigned short countDirCtrl:1;
	unsigned short countDir:1;
	unsigned short clockSource:2;
	unsigned short ctEnableCtrl:2;
	unsigned short hwCtEnableSource:2;
	unsigned short autoLoadResetRcap:3;
	unsigned short coutPolarity:1;
	unsigned short coutSource:1;
#else
#error Unknown bit field order
#endif
};

union cmReg {
	struct counter_mode_register_t reg;
	unsigned short value;
};

/* Application Classes for GPCT Subdevices */
enum S526_GPCT_APP_CLASS {
	CountingAndTimeMeasurement,
	SinglePulseGeneration,
	PulseTrainGeneration,
	PositionMeasurement,
	Miscellaneous
};

/* GPCT subdevices configuration */
#define MAX_GPCT_CONFIG_DATA 6
struct s526GPCTConfig {
	enum S526_GPCT_APP_CLASS app;
	int data[MAX_GPCT_CONFIG_DATA];
};

typedef struct cb_pcidas_priv {
    unsigned long io_base;
    struct cb_pcidas_struct *cb;
    struct cb_pcidas_board *board;  
    /* base addresses */
    unsigned long s5933_config;
    unsigned long control_status;
    unsigned long adc_fifo;
    unsigned long pacer_counter_dio;
    unsigned long ao_registers;
    /* divisors of master clock for analog input pacing */
    unsigned int divisor1;
    unsigned int divisor2;
    /* number of analog input samples remaining */
    unsigned int count;
    /* bits to write to registers */
    unsigned int adc_fifo_bits;
    unsigned int s5933_intcsr_bits;
    unsigned int ao_control_bits;
    /* read and write subdevices */
    struct a4l_subdevice *write_subdev;
    struct a4l_subdevice *read_subdev;
} cb_pcidas_priv_t;

/*
struct s526_subd_gpct_priv {
	struct s526GPCTConfig config[4];
};

struct s526_subd_ai_priv {
	uint16_t config;
};

struct s526_subd_ao_priv {
	uint16_t readback[2];
};
*/
struct cb_pcidas_subd_dio_priv {
	int io_bits;
	unsigned int state;
};

#define devpriv ((cb_pcidas_priv_t*)(dev->priv))

#define ADDR_REG(reg) (devpriv->io_base + (reg))
#define ADDR_CHAN_REG(reg, chan) (devpriv->io_base + (reg) + (chan) * 8)

static LIST_HEAD(cb_pcidas_devices);

#if 0
static int s526_gpct_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_gpct_priv *subdpriv =
	    (struct s526_subd_gpct_priv *)subd->priv;
	unsigned int *data = (unsigned int *)insn->data;
	int subdev_channel = CR_CHAN(insn->chan_desc);
	int i;
	short value;
	union cmReg cmReg;

	a4l_dbg(1, drv_dbg, dev,
		"s526_gpct_insn_config: Configuring Channel %d\n",
		subdev_channel);

	for (i = 0; i < MAX_GPCT_CONFIG_DATA; i++) {
		subdpriv->config[subdev_channel].data[i] = data[i];
		a4l_dbg(1, drv_dbg, dev, "data[%d]=%x\n", i, data[i]);
	}

	switch (data[0]) {
	case A4L_INSN_CONFIG_GPCT_QUADRATURE_ENCODER:
		/*
		 * data[0]: Application Type
		 * data[1]: Counter Mode Register Value
		 * data[2]: Pre-load Register Value
		 * data[3]: Conter Control Register
		 */
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_insn_config: Configuring Encoder\n");
		subdpriv->config[subdev_channel].app = PositionMeasurement;

		/* Set Counter Mode Register */
		cmReg.value = data[1] & 0xFFFF;

		a4l_dbg(1, drv_dbg, dev, "Counter Mode register=%x\n", cmReg.value);
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Reset the counter if it is software preload */
		if (cmReg.reg.autoLoadResetRcap == 0) {
			outw(0x8000, ADDR_CHAN_REG(REG_C0C, subdev_channel)); /* Reset the counter */
			/* outw(0x4000, ADDR_CHAN_REG(REG_C0C, subdev_channel));	/\* Load the counter from PR0 *\/ */
		}
		break;

	case A4L_INSN_CONFIG_GPCT_SINGLE_PULSE_GENERATOR:
		/*
		 * data[0]: Application Type
		 * data[1]: Counter Mode Register Value
		 * data[2]: Pre-load Register 0 Value
		 * data[3]: Pre-load Register 1 Value
		 * data[4]: Conter Control Register
		 */
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_insn_config: Configuring SPG\n");
		subdpriv->config[subdev_channel].app = SinglePulseGeneration;

		/* Set Counter Mode Register */
		cmReg.value = (short)(data[1] & 0xFFFF);
		cmReg.reg.preloadRegSel = 0; /* PR0 */
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Load the pre-load register 0 high word */
		value = (short)((data[2] >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));

		/* Load the pre-load register 0 low word */
		value = (short)(data[2] & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));

		/* Set Counter Mode Register */
		cmReg.value = (short)(data[1] & 0xFFFF);
		cmReg.reg.preloadRegSel = 1; /* PR1 */
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Load the pre-load register 1 high word */
		value = (short)((data[3] >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));

		/* Load the pre-load register 1 low word */
		value = (short)(data[3] & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));

		/* Write the Counter Control Register */
		if (data[4] != 0) {
			value = (short)(data[4] & 0xFFFF);
			outw(value, ADDR_CHAN_REG(REG_C0C, subdev_channel));
		}
		break;

	case A4L_INSN_CONFIG_GPCT_PULSE_TRAIN_GENERATOR:
		/*
		 * data[0]: Application Type
		 * data[1]: Counter Mode Register Value
		 * data[2]: Pre-load Register 0 Value
		 * data[3]: Pre-load Register 1 Value
		 * data[4]: Conter Control Register
		 */
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_insn_config: Configuring PTG\n");
		subdpriv->config[subdev_channel].app = PulseTrainGeneration;

		/* Set Counter Mode Register */
		cmReg.value = (short)(data[1] & 0xFFFF);
		cmReg.reg.preloadRegSel = 0; /* PR0 */
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Load the pre-load register 0 high word */
		value = (short)((data[2] >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));

		/* Load the pre-load register 0 low word */
		value = (short)(data[2] & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));

		/* Set Counter Mode Register */
		cmReg.value = (short)(data[1] & 0xFFFF);
		cmReg.reg.preloadRegSel = 1; /* PR1 */
		outw(cmReg.value, ADDR_CHAN_REG(REG_C0M, subdev_channel));

		/* Load the pre-load register 1 high word */
		value = (short)((data[3] >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));

		/* Load the pre-load register 1 low word */
		value = (short)(data[3] & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));

		/* Write the Counter Control Register */
		if (data[4] != 0) {
			value = (short)(data[4] & 0xFFFF);
			outw(value, ADDR_CHAN_REG(REG_C0C, subdev_channel));
		}
		break;

	default:
		a4l_err(dev, "s526_gpct_insn_config: unsupported GPCT_insn_config\n");
		return -EINVAL;
		break;
	}

	return 0;
}

static int s526_gpct_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	uint32_t *data = (uint32_t *)insn->data;
	int counter_channel = CR_CHAN(insn->chan_desc);
	unsigned short datalow;
	unsigned short datahigh;
	int i;

	if (insn->data_size <= 0) {
		a4l_err(dev, "s526_gpct_rinsn: data size should be > 0\n");
		return -EINVAL;
	}

	for (i = 0; i < insn->data_size / sizeof(uint32_t); i++) {
		datalow = inw(ADDR_CHAN_REG(REG_C0L, counter_channel));
		datahigh = inw(ADDR_CHAN_REG(REG_C0H, counter_channel));
		data[i] = (int)(datahigh & 0x00FF);
		data[i] = (data[i] << 16) | (datalow & 0xFFFF);
		a4l_dbg(1, drv_dbg, dev,
			"s526_gpct_rinsn GPCT[%d]: %x(0x%04x, 0x%04x)\n",
			counter_channel, data[i], datahigh, datalow);
	}

	return 0;
}

static int s526_gpct_winsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_gpct_priv *subdpriv =
	    (struct s526_subd_gpct_priv *)subd->priv;
	uint32_t *data = (uint32_t *)insn->data;
	int subdev_channel = CR_CHAN(insn->chan_desc);
	short value;
	union cmReg cmReg;

	a4l_dbg(1, drv_dbg, dev,
		"s526_gpct_winsn: GPCT_INSN_WRITE on channel %d\n",
		subdev_channel);

	cmReg.value = inw(ADDR_CHAN_REG(REG_C0M, subdev_channel));
	a4l_dbg(1, drv_dbg, dev,
		"s526_gpct_winsn: Counter Mode Register: %x\n", cmReg.value);

	/* Check what Application of Counter this channel is configured for */
	switch (subdpriv->config[subdev_channel].app) {
	case PositionMeasurement:
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_winsn: INSN_WRITE: PM\n");
		outw(0xFFFF & ((*data) >> 16), ADDR_CHAN_REG(REG_C0H,
							     subdev_channel));
		outw(0xFFFF & (*data),
		     ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;

	case SinglePulseGeneration:
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_winsn: INSN_WRITE: SPG\n");
		outw(0xFFFF & ((*data) >> 16), ADDR_CHAN_REG(REG_C0H,
							     subdev_channel));
		outw(0xFFFF & (*data),
		     ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;

	case PulseTrainGeneration:
		/*
		 * data[0] contains the PULSE_WIDTH
		 * data[1] contains the PULSE_PERIOD
		 * @pre PULSE_PERIOD > PULSE_WIDTH > 0
		 * The above periods must be expressed as a multiple of the
		 * pulse frequency on the selected source
		 */
		a4l_dbg(1, drv_dbg, dev, "s526_gpct_winsn: INSN_WRITE: PTG\n");
		if ((data[1] > data[0]) && (data[0] > 0)) {
			(subdpriv->config[subdev_channel]).data[0] = data[0];
			(subdpriv->config[subdev_channel]).data[1] = data[1];
		} else {
			a4l_err(dev,
				"s526_gpct_winsn: INSN_WRITE: PTG: Problem with Pulse params -> %du %du\n",
				data[0], data[1]);
			return -EINVAL;
		}

		value = (short)((*data >> 16) & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0H, subdev_channel));
		value = (short)(*data & 0xFFFF);
		outw(value, ADDR_CHAN_REG(REG_C0L, subdev_channel));
		break;
	default:		/* Impossible */
		a4l_err(dev,
			"s526_gpct_winsn: INSN_WRITE: Functionality %d not implemented yet\n",
			 subdpriv->config[subdev_channel].app);
		return -EINVAL;
	}

	return 0;
}

static int s526_ai_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_ai_priv *subdpriv =
	    (struct s526_subd_ai_priv *)subd->priv;
	unsigned int *data = (unsigned int *)insn->data;

	if (insn->data_size < sizeof(unsigned int))
		return -EINVAL;

	/* data[0] : channels was set in relevant bits.
	 * data[1] : delay
	 */
	/* COMMENT: abbotti 2008-07-24: I don't know why you'd want to
	 * enable channels here.  The channel should be enabled in the
	 * INSN_READ handler. */

	/* Enable ADC interrupt */
	outw(ISR_ADC_DONE, ADDR_REG(REG_IER));
	a4l_dbg(1, drv_dbg, dev,
		"s526_ai_insn_config: ADC current value: 0x%04x\n",
		inw(ADDR_REG(REG_ADC)));

	subdpriv->config = (data[0] & 0x3FF) << 5;
	if (data[1] > 0)
		subdpriv->config |= 0x8000; /* set the delay */

	subdpriv->config |= 0x0001; /* ADC start bit. */

	return 0;
}

static int s526_ai_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_ai_priv *subdpriv =
	    (struct s526_subd_ai_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;
	int n, i;
	int chan = CR_CHAN(insn->chan_desc);
	uint16_t value;
	uint16_t d;
	uint16_t status;

	/* Set configured delay, enable channel for this channel only,
	 * select "ADC read" channel, set "ADC start" bit. */
	value = (subdpriv->config & 0x8000) |
	    ((1 << 5) << chan) | (chan << 1) | 0x0001;

	/* convert n samples */
	for (n = 0; n < insn->data_size / sizeof(uint16_t); n++) {
		/* trigger conversion */
		outw(value, ADDR_REG(REG_ADC));
		a4l_dbg(1, drv_dbg, dev, "s526_ai_rinsn: Wrote 0x%04x to ADC\n",
			value);

		/* wait for conversion to end */
		for (i = 0; i < S526_AI_TIMEOUT; i++) {
			status = inw(ADDR_REG(REG_ISR));
			if (status & ISR_ADC_DONE) {
				outw(ISR_ADC_DONE, ADDR_REG(REG_ISR));
				break;
			}
		}
		if (i == S526_AI_TIMEOUT) {
			a4l_warn(dev, "s526_ai_rinsn: ADC(0x%04x) timeout\n",
				 inw(ADDR_REG(REG_ISR)));
			return -ETIMEDOUT;
		}

		/* read data */
		d = inw(ADDR_REG(REG_ADD));
		a4l_dbg(1, drv_dbg, dev, "s526_ai_rinsn: AI[%d]=0x%04x\n",
			n, (uint16_t)(d & 0xFFFF));

		/* munge data */
		data[n] = d ^ 0x8000;
	}

	return 0;
}

static int s526_ao_winsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	struct s526_subd_ao_priv *subdpriv =
	    (struct s526_subd_ao_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;
	int i;
	int chan = CR_CHAN(insn->chan_desc);
	uint16_t val;

	val = chan << 1;
	outw(val, ADDR_REG(REG_DAC));

	for (i = 0; i < insn->data_size / sizeof(uint16_t); i++) {
		outw(data[i], ADDR_REG(REG_ADD)); /* write the data to preload register */
		subdpriv->readback[chan] = data[i];
		outw(val + 1, ADDR_REG(REG_DAC)); /* starts the D/A conversion. */
	}

	return 0;
}

static int s526_ao_rinsn(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct s526_subd_ao_priv *subdpriv =
		(struct s526_subd_ao_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;
	int i;
	int chan = CR_CHAN(insn->chan_desc);

	for (i = 0; i < insn->data_size / sizeof(uint16_t); i++)
		data[i] = subdpriv->readback[chan];

	return 0;
}
#endif

/* analog output insn for pcidas-1602 series */
static int cb_pcidas_ao_fifo_winsn(struct a4l_subdevice *subd,
				   struct a4l_kernel_instruction *insn)
{
    struct a4l_device *dev = subd->dev;	
    unsigned int chan = CR_CHAN(insn->chan_desc);
    unsigned int range = CR_RNG(insn->chan_desc);
    uint16_t *data = (uint16_t *)insn->data;
    unsigned long flags;

    /* clear dac fifo */
    outw(0, devpriv->ao_registers + DACFIFOCLR);
    
    /* set channel and range */
    rtdm_lock_get_irqsave(&dev->lock, flags);
    devpriv->ao_control_bits &= (~DAC_CHAN_EN(0) & ~DAC_CHAN_EN(1) &
				 ~DAC_RANGE_MASK(chan) & ~DAC_PACER_MASK);
    devpriv->ao_control_bits |= (DACEN | DAC_RANGE(chan, range) |
				 DAC_CHAN_EN(chan) | DAC_START);
    outw(devpriv->ao_control_bits, devpriv->control_status + DAC_CSR);
    rtdm_lock_put_irqrestore(&dev->lock, flags);
    
    /* remember value for readback */
    //devpriv->ao_value[chan] = data[0];
    
    /* send data */
    outw(data[0], devpriv->ao_registers + DACDATA);
    
    return insn->data_size;
}

static int cb_pcidas_dio_insn_config(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	//struct cb_pcidas_subd_dio_priv *subdpriv = (struct cb_pcidas_subd_dio_priv *)subd->priv;
	unsigned int *data = (unsigned int *)insn->data;
	int chan = CR_CHAN(insn->chan_desc);
	int mask, mask_config, config;
	
	/* Find which bank the channel is part of*/
	if (chan < 8) {
	  mask = 0x0000ff;
	  mask_config = 0x0010;
	} else if (chan < 16) {
	  mask = 0x00ff00;
	  mask_config = 0x0002;
	} else if (chan < 20) {
	  mask = 0x0f0000;
	  mask_config = 0x0001;
	} else {
	  mask = 0xf00000;
	  mask_config = 0x0080;
	}

	config = CR_CW;
	/* Output = 0, Input = 1 */
	switch (data[0]) {
	case A4L_INSN_CONFIG_DIO_OUTPUT:
	  config &= !mask_config;
	  break;
	case A4L_INSN_CONFIG_DIO_INPUT:
	  config |= mask_config;
	  break;
	case A4L_INSN_CONFIG_DIO_QUERY:
	  data[1] = inb(pci_resource_start(devpriv->cb->pcidev,3)+_8255_BASE+_8255_CR);
	  data[1] = !((data[1] | mask_config) >> (mask_config-1));
	  return 0;
	default:
		return -EINVAL;
	}

	outb(config,pci_resource_start(devpriv->cb->pcidev,3)+_8255_BASE+_8255_CR);
	return 0;
}

static int cb_pcidas_dio_insn_bits(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct a4l_device *dev = subd->dev;
	//struct cb_pcidas_subd_dio_priv *subdpriv = (struct cb_pcidas_subd_dio_priv *)subd->priv;
	uint8_t *data = (uint8_t *)insn->data;
	/* Not finished - needs to be completed */
#if 0
	if (insn->data_size != 2 * sizeof(uint8_t))
		return -EINVAL;

	if (data[0]) {
		subdpriv->state &= ~(data[0]);
		subdpriv->state |= data[0] & data[1];

		outw(subdpriv->state, ADDR_REG(REG_DIO));
	}

	data[1] = inw(ADDR_REG(REG_DIO)) & 0xFF; /* low 8 bits are the data */
#endif
	printk("cb_pcidas_dio_insn_bits: writing %d\n",data[0]);
	outb(data[0],pci_resource_start(devpriv->cb->pcidev,3)+4);
	return 0;
}

/* --- Channels descriptor --- */
/*
static struct a4l_channels_desc s526_chan_desc_gpct = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S526_GPCT_CHANS,
	.chans = {
		{A4L_CHAN_AREF_GROUND, S526_GPCT_BITS},
	},
};

static struct a4l_channels_desc s526_chan_desc_ai = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S526_AI_CHANS,
	.chans = {
		{A4L_CHAN_AREF_GROUND, S526_AI_BITS},
	},
};

static struct a4l_channels_desc cb_pcidas_chan_desc_ao = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = S526_AO_CHANS,
	.chans = {
		{A4L_CHAN_AREF_GROUND, S526_AO_BITS},
	},
};
*/
static struct a4l_channels_desc cb_pcidas_chan_desc_dio = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = CB_PCIDAS_DIO_CHANS,
	.chans = {
		{A4L_CHAN_AREF_GROUND, CB_PCIDAS_DIO_BITS},
	},
};

/* --- Subdevice initialization functions --- */

/* General purpose counter/timer (gpct)
static void setup_subd_gpct(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_COUNTER;
	subd->chan_desc = &s526_chan_desc_gpct;
	subd->insn_read = s526_gpct_rinsn;
	subd->insn_config = s526_gpct_insn_config;
	subd->insn_write = s526_gpct_winsn;
	}*/

/* Analog input subdevice 
static void setup_subd_ai(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_AI;
	subd->chan_desc = &s526_chan_desc_ai;
	subd->rng_desc = &a4l_range_bipolar10;
	subd->insn_read = s526_ai_rinsn;
	subd->insn_config = s526_ai_insn_config;
}*/

/* Analog output subdevice */
#if 0
static void setup_subd_ao(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_AO;
	subd->chan_desc = &s526_chan_desc_ao;
	subd->rng_desc = &a4l_range_bipolar10;
	subd->insn_write = s526_ao_winsn;
	subd->insn_read = s526_ao_rinsn;
}
#endif

/* Digital i/o subdevice */
static void setup_subd_dio(struct a4l_subdevice *subd)
{
	subd->flags = A4L_SUBD_DIO;
	subd->chan_desc = &cb_pcidas_chan_desc_dio;
	subd->rng_desc = &range_digital;
	subd->insn_bits = cb_pcidas_dio_insn_bits;
	subd->insn_config = cb_pcidas_dio_insn_config;
}

struct setup_subd {
	void (*setup_func) (struct a4l_subdevice *);
	int sizeof_priv;
};


static struct setup_subd setup_subds[4] = {
  /*	{
		.setup_func = setup_subd_gpct,
		.sizeof_priv = sizeof(struct s526_subd_gpct_priv),
	},
	{
		.setup_func = setup_subd_ai,
		.sizeof_priv = sizeof(struct s526_subd_ai_priv),
	},
	{
		.setup_func = setup_subd_ao,
		.sizeof_priv = sizeof(struct s526_subd_ao_priv),
		},*/
	{
		.setup_func = setup_subd_dio,
		.sizeof_priv = sizeof(struct cb_pcidas_subd_dio_priv),
	},
};

#if 0
static void handle_ao_interrupt(struct a4l_device *dev, unsigned int status)
{
	const struct cb_pcidas_board *thisboard = devpriv->board;
	struct comedi_subdevice *s = dev->write_subdev;
	struct comedi_async *async = s->async;
	struct comedi_cmd *cmd = &async->cmd;
	unsigned int half_fifo = thisboard->fifo_size / 2;
	unsigned int num_points;
	unsigned long flags;

	async->events = 0;

	if (status & DAEMI) {
		/*  clear dac empty interrupt latch */
		spin_lock_irqsave(&dev->spinlock, flags);
		outw(devpriv->adc_fifo_bits | DAEMI,
		     devpriv->control_status + INT_ADCFIFO);
		spin_unlock_irqrestore(&dev->spinlock, flags);
		if (inw(devpriv->ao_registers + DAC_CSR) & DAC_EMPTY) {
			if (cmd->stop_src == TRIG_NONE ||
			    (cmd->stop_src == TRIG_COUNT
			     && devpriv->ao_count)) {
				comedi_error(dev, "dac fifo underflow");
				cb_pcidas_ao_cancel(dev, s);
				async->events |= COMEDI_CB_ERROR;
			}
			async->events |= COMEDI_CB_EOA;
		}
	} else if (status & DAHFI) {
		unsigned int num_bytes;

		/*  figure out how many points we are writing to fifo */
		num_points = half_fifo;
		if (cmd->stop_src == TRIG_COUNT &&
		    devpriv->ao_count < num_points)
			num_points = devpriv->ao_count;
		num_bytes =
		    cfc_read_array_from_buffer(s, devpriv->ao_buffer,
					       num_points * sizeof(short));
		num_points = num_bytes / sizeof(short);

		if (async->cmd.stop_src == TRIG_COUNT)
			devpriv->ao_count -= num_points;
		/*  write data to board's fifo */
		outsw(devpriv->ao_registers + DACDATA, devpriv->ao_buffer,
		      num_points);
		/*  clear half-full interrupt latch */
		spin_lock_irqsave(&dev->spinlock, flags);
		outw(devpriv->adc_fifo_bits | DAHFI,
		     devpriv->control_status + INT_ADCFIFO);
		spin_unlock_irqrestore(&dev->spinlock, flags);
	}

	comedi_event(dev, s);
}
#endif

int cb_pcidas_interrupt(unsigned int irq, void *d)
{
    struct a4l_device *dev = (struct a4l_device *)d;
    //struct cb_pcidas_board *thisboard = devpriv->board;
    //struct cb_pcidas_private *devpriv = dev->private;
    
    //struct comedi_subdevice *s = dev->read_subdev;
    //struct comedi_async *async;
    int status, s5933_status;
    //int half_fifo = thisboard->fifo_size / 2;
    //unsigned int num_samples, i;
    //static const int timeout = 10000;
    //unsigned long flags;
    
#if 0    
    if (!dev->attached) {
	return IRQ_NONE;
    }
#endif
    
    s5933_status = inl(devpriv->s5933_config + AMCC_OP_REG_INTCSR);
    
    if ((INTCSR_INTR_ASSERTED & s5933_status) == 0) {
	return IRQ_NONE;
    }
    
    /*  make sure mailbox 4 is empty */
    inl_p(devpriv->s5933_config + AMCC_OP_REG_IMB4);
    /*  clear interrupt on amcc s5933 */
    outl(devpriv->s5933_intcsr_bits | INTCSR_INBOX_INTR_STATUS,
	 devpriv->s5933_config + AMCC_OP_REG_INTCSR);
    
    status = inw(devpriv->control_status + INT_ADCFIFO);
    
    /*  check for analog output interrupt */
    if (status & (DAHFI | DAEMI)) {
	//handle_ao_interrupt(dev, status);
    }
    
#if 0	
    async = s->async;
    async->events = 0;
    /*  check for analog input interrupts */
    /*  if fifo half-full */
    if (status & ADHFI) {
	/*  read data */
	num_samples = half_fifo;
	if (async->cmd.stop_src == TRIG_COUNT &&
	    num_samples > devpriv->count) {
	    num_samples = devpriv->count;
	}
	insw(devpriv->adc_fifo + ADCDATA, devpriv->ai_buffer,
	     num_samples);
	cfc_write_array_to_buffer(s, devpriv->ai_buffer,
				  num_samples * sizeof(short));
	devpriv->count -= num_samples;
	if (async->cmd.stop_src == TRIG_COUNT && devpriv->count == 0) {
	    async->events |= COMEDI_CB_EOA;
	    cb_pcidas_cancel(dev, s);
	}
	/*  clear half-full interrupt latch */
	spin_lock_irqsave(&dev->spinlock, flags);
	outw(devpriv->adc_fifo_bits | INT,
	     devpriv->control_status + INT_ADCFIFO);
	spin_unlock_irqrestore(&dev->spinlock, flags);
	/*  else if fifo not empty */
    } else if (status & (ADNEI | EOBI)) {
	for (i = 0; i < timeout; i++) {
	    /*  break if fifo is empty */
	    if ((ADNE & inw(devpriv->control_status +
			    INT_ADCFIFO)) == 0)
		break;
	    cfc_write_to_buffer(s, inw(devpriv->adc_fifo));
	    if (async->cmd.stop_src == TRIG_COUNT &&
		--devpriv->count == 0) {
		/* end of acquisition */
		cb_pcidas_cancel(dev, s);
		async->events |= COMEDI_CB_EOA;
		break;
	    }
	}
	/*  clear not-empty interrupt latch */
	spin_lock_irqsave(&dev->spinlock, flags);
	outw(devpriv->adc_fifo_bits | INT,
	     devpriv->control_status + INT_ADCFIFO);
	spin_unlock_irqrestore(&dev->spinlock, flags);
    } else if (status & EOAI) {
	comedi_error(dev,
		     "bug! encountered end of acquisition interrupt?");
	/*  clear EOA interrupt latch */
	spin_lock_irqsave(&dev->spinlock, flags);
	outw(devpriv->adc_fifo_bits | EOAI,
	     devpriv->control_status + INT_ADCFIFO);
	spin_unlock_irqrestore(&dev->spinlock, flags);
    }
    /* check for fifo overflow */
    if (status & LADFUL) {
	comedi_error(dev, "fifo overflow");
	/*  clear overflow interrupt latch */
	spin_lock_irqsave(&dev->spinlock, flags);
	outw(devpriv->adc_fifo_bits | LADFUL,
	     devpriv->control_status + INT_ADCFIFO);
	spin_unlock_irqrestore(&dev->spinlock, flags);
	cb_pcidas_cancel(dev, s);
	async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
    }
    
    comedi_event(dev, s);
#endif
    return IRQ_HANDLED;
}

int dev_cb_pcidas_device_intable(struct pci_dev *pcidev)
{
  int i, ret = -1;

  for(i = 0; i < ARRAY_SIZE(cb_pcidas_boards); i++) {
    if(pcidev->device == cb_pcidas_boards[i].device_id) {
      ret = i;
    }
  }
  return ret;
}

static int dev_cb_pcidas_attach(struct a4l_device *dev, a4l_lnkdesc_t *arg)
{
    int io_base, board_idx = -1;
    struct list_head *this;
    struct cb_pcidas_struct *cb = NULL;
    struct cb_pcidas_board *thisboard = NULL;
    struct a4l_subdevice *subd = NULL;
    
    printk("analogy_cb_pcidas: attach\n");
    if (arg->opts == NULL || arg->opts_size < sizeof(unsigned long)) {
	a4l_info(dev,"dev_cb_pcidas_attach: no options specified\n");
	io_base = 0;
    } else {
	io_base = ((unsigned long *)arg->opts)[0];
	a4l_info(dev,"dev_cb_pcidas_attach: option provided = %d\n",io_base);
    }
    /* Find PCI device from list created in probe */
    list_for_each(this, &cb_pcidas_devices) {
	cb = list_entry(this, struct cb_pcidas_struct, list);
	/* Test if device is within device table */
	if ((board_idx = dev_cb_pcidas_device_intable(cb->pcidev)) >= 0) {
	    break;
	}
    }
    if (board_idx < 0) {
	a4l_err(dev,"Device 0x%04x not found in device table\n",cb->pcidev->device);
	return -EIO;
    }
    
    devpriv->cb = cb;
    devpriv->board = thisboard = &cb_pcidas_boards[board_idx];
    
    a4l_info(dev,"Trying to attach %s\n",devpriv->board->name);
    
    /* Request all regions of PCI device */
    if (pci_request_regions(cb->pcidev,"cb_pcidas") != 0) {
	a4l_err(dev,"dev_cb_pcidas_attach: unable to request PCI region\n");
	return -EIO;
    }
    
    /* Fill private data struct */
    devpriv->s5933_config = pci_resource_start(cb->pcidev, 0);
    devpriv->control_status = pci_resource_start(cb->pcidev, 1);
    devpriv->adc_fifo = pci_resource_start(cb->pcidev, 2);
    devpriv->pacer_counter_dio = pci_resource_start(cb->pcidev, 3);
    if (thisboard->ao_nchan) {
	devpriv->ao_registers = pci_resource_start(cb->pcidev, 4);
    }

    /* Setup interrupts */
    /*  disable and clear interrupts on amcc s5933 */
    outl(INTCSR_INBOX_INTR_STATUS,
	 devpriv->s5933_config + AMCC_OP_REG_INTCSR);
    if(a4l_request_irq(dev,
		       cb->pcidev->irq,
		       cb_pcidas_interrupt,
		       RTDM_IRQTYPE_SHARED,
		     dev)) {
	a4l_err(dev,"unable to allocate irq %d\n",cb->pcidev->irq);
	return -EINVAL;
    }
    
    /* Allocate and add AO subdevice */
    subd = a4l_alloc_subd(0,NULL);
    if (subd == NULL) {
	return -ENOMEM;
    }
    if (thisboard->ao_nchan) {
	a4l_info(dev," attaching AO subdevice...");
      	subd->flags = A4L_SUBD_AO | A4L_SUBD_CMD;
	subd->chan_desc = kmalloc(sizeof(struct a4l_channels_desc) +
				  sizeof(struct a4l_channel), GFP_KERNEL);
	subd->chan_desc->mode = A4L_CHAN_GLOBAL_CHANDESC;
	subd->chan_desc->length = thisboard->ao_nchan;
	subd->chan_desc->chans[0].flags = A4L_CHAN_AREF_GROUND | A4L_CHAN_AREF_DIFF;
	subd->chan_desc->chans[0].nb_bits = thisboard->ao_bits;
	
	subd->rng_desc = thisboard->ranges_ao;

	/* Callbacks */
	subd->insn_write = cb_pcidas_ao_fifo_winsn;
    }
    if (a4l_add_subd(dev,subd) < 0) {
	return -1;
	a4l_info(dev,"registering AO subdevice failed\n");
    }
    a4l_info(dev,"registered AO subdevice\n");
    
    /* Allocate and DIO subdevice */
    subd = a4l_alloc_subd(setup_subds[0].sizeof_priv,
			  setup_subds[0].setup_func);
    if (subd == NULL) {
	return -ENOMEM;
    }
    if (a4l_add_subd(dev,subd) < 0) {
	return -1;
    }
    
    /* Assign driver? */
    dev->driver->driver_name = devpriv->board->name;
    
    a4l_info(dev,"Attach finished\n");
    return 0;
}

static int dev_cb_pcidas_detach(struct a4l_device *dev)
{
	int err = 0;	
	if (devpriv->cb != NULL) {
	    if (devpriv->s5933_config) {
			outl(INTCSR_INBOX_INTR_STATUS,
			     devpriv->s5933_config + AMCC_OP_REG_INTCSR);
	    }
	  pci_release_regions(devpriv->cb->pcidev);
	}
	if (devpriv->cb->pcidev->irq) {
	    a4l_free_irq(dev,devpriv->cb->pcidev->irq);
	}
	dev->driver->driver_name = NULL;
	a4l_info(dev,"dev_cb_pcidas_detach: pci_release_regions\n");
	return err;
}

static const struct pci_device_id cb_pcidas_pci_table[] = {
	{ PCI_VDEVICE(CB, 0x0001), BOARD_PCIDAS1602_16 },
	{ PCI_VDEVICE(CB, 0x000f), BOARD_PCIDAS1200 },
	{ PCI_VDEVICE(CB, 0x0010), BOARD_PCIDAS1602_12 },
	{ PCI_VDEVICE(CB, 0x0019), BOARD_PCIDAS1200_JR },
	{ PCI_VDEVICE(CB, 0x001c), BOARD_PCIDAS1602_16_JR },
	{ PCI_VDEVICE(CB, 0x004c), BOARD_PCIDAS1000 },
	{ PCI_VDEVICE(CB, 0x001a), BOARD_PCIDAS1001 },
	{ PCI_VDEVICE(CB, 0x001b), BOARD_PCIDAS1002 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, cb_pcidas_pci_table);

static int cb_pcidas_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
  int err = 0;
  struct cb_pcidas_struct *cb;

  printk("analogy_cb_pcidas: cb_pcidas_probe\n");

  cb = kmalloc(sizeof(struct cb_pcidas_struct), GFP_KERNEL);
  if(cb == NULL) {
    return -ENOMEM;
  }

  memset(cb, 0, sizeof(struct cb_pcidas_struct));
  cb->pcidev = dev;

  if(pci_enable_device(dev) < 0) {
    __a4l_err("error enabling cb_pcidas\n");
    err = -EIO;
    goto out;
  }

  list_add(&cb->list, &cb_pcidas_devices);
  printk("analogy_cb_pcidas: vendor id 0x%04x device id 0x%04x\n",
	 cb->pcidev->vendor,cb->pcidev->device);
  printk("analogy_cb_pcidas: slot 0x%04x function 0x%04x\n",
	 PCI_SLOT(cb->pcidev->devfn),PCI_FUNC(cb->pcidev->devfn));

 out:
  if (err < 0){
    kfree(cb);
  }

  return err;
}

static void cb_pcidas_remove(struct pci_dev *dev)
{
  struct list_head *this;

  printk("analogy_cb_pcidas: cb_pcidas_remove\n");

  list_for_each(this, &cb_pcidas_devices) {
    struct cb_pcidas_struct *cb = list_entry(this, struct cb_pcidas_struct, list);
    if (cb->pcidev == dev) {
      list_del(this);
      kfree(cb);
      break;
    }
  }
}

static struct pci_driver cb_pcidas_driver = {
  .name = "analogy_cb_pcidas_pci",
  .id_table = cb_pcidas_pci_table,
  .probe = cb_pcidas_probe,
  .remove = cb_pcidas_remove,
};

static struct a4l_driver drv_cb_pcidas = {
	.owner = THIS_MODULE,
	.board_name = "analogy_cb_pcidas",
	.driver_name = NULL,
	.attach = dev_cb_pcidas_attach,
	.detach = dev_cb_pcidas_detach,
	.privdata_size = sizeof(cb_pcidas_priv_t),
};

static int __init drv_cb_pcidas_init(void)
{
  printk("analogy_cb_pcidas: __init\n");
  pci_register_driver(&cb_pcidas_driver);
  return a4l_register_drv(&drv_cb_pcidas);
}

static void __exit drv_cb_pcidas_cleanup(void)
{
  printk("analogy_cb_pcidas: __cleanup\n");
  pci_unregister_driver(&cb_pcidas_driver);
  a4l_unregister_drv(&drv_cb_pcidas);
}

MODULE_DESCRIPTION("Analogy driver for Measurement Computing.");
MODULE_LICENSE("GPL");

module_init(drv_cb_pcidas_init);
module_exit(drv_cb_pcidas_cleanup);
