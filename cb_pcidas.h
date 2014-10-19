#ifndef __CB_PCIDAS_H_
#define __CB_PCIDAS_H_

#if __GNUC__ >= 3
#define GCC_ZERO_LENGTH_ARRAY
#else
#define GCC_ZERO_LENGTH_ARRAY 0
#endif

enum cb_pcidas_boardid {
	BOARD_PCIDAS1602_16,
	BOARD_PCIDAS1200,
	BOARD_PCIDAS1602_12,
	BOARD_PCIDAS1200_JR,
	BOARD_PCIDAS1602_16_JR,
	BOARD_PCIDAS1000,
	BOARD_PCIDAS1001,
	BOARD_PCIDAS1002,
};

struct cb_pcidas_struct {
  struct list_head list;
  struct pci_dev *pcidev;
};

/****************************************************************************/
/* AMCC Operation Register Offsets - PCI                                    */
/****************************************************************************/

#define AMCC_OP_REG_OMB1         0x00
#define AMCC_OP_REG_OMB2         0x04
#define AMCC_OP_REG_OMB3         0x08
#define AMCC_OP_REG_OMB4         0x0c
#define AMCC_OP_REG_IMB1         0x10
#define AMCC_OP_REG_IMB2         0x14
#define AMCC_OP_REG_IMB3         0x18
#define AMCC_OP_REG_IMB4         0x1c
#define AMCC_OP_REG_FIFO         0x20
#define AMCC_OP_REG_MWAR         0x24
#define AMCC_OP_REG_MWTC         0x28
#define AMCC_OP_REG_MRAR         0x2c
#define AMCC_OP_REG_MRTC         0x30
#define AMCC_OP_REG_MBEF         0x34
#define AMCC_OP_REG_INTCSR       0x38
#define  AMCC_OP_REG_INTCSR_SRC  (AMCC_OP_REG_INTCSR + 2)	/* INT source */
#define  AMCC_OP_REG_INTCSR_FEC  (AMCC_OP_REG_INTCSR + 3)	/* FIFO ctrl */
#define AMCC_OP_REG_MCSR         0x3c
#define  AMCC_OP_REG_MCSR_NVDATA (AMCC_OP_REG_MCSR + 2)	/* Data in byte 2 */
#define  AMCC_OP_REG_MCSR_NVCMD  (AMCC_OP_REG_MCSR + 3)	/* Command in byte 3 */

#define AMCC_FIFO_DEPTH_DWORD	8
#define AMCC_FIFO_DEPTH_BYTES	(8 * sizeof (u32))

/****************************************************************************/
/* AMCC - PCI Interrupt Control/Status Register                            */
/****************************************************************************/
#define INTCSR_OUTBOX_BYTE(x)	((x) & 0x3)
#define INTCSR_OUTBOX_SELECT(x)	(((x) & 0x3) << 2)
#define INTCSR_OUTBOX_EMPTY_INT	0x10	/*  enable outbox empty interrupt */
#define INTCSR_INBOX_BYTE(x)	(((x) & 0x3) << 8)
#define INTCSR_INBOX_SELECT(x)	(((x) & 0x3) << 10)
#define INTCSR_INBOX_FULL_INT	0x1000	/*  enable inbox full interrupt */
#define INTCSR_INBOX_INTR_STATUS	0x20000	/*  read, or write clear inbox full interrupt */
#define INTCSR_INTR_ASSERTED	0x800000	/*  read only, interrupt asserted */

/* range stuff */

#define CRANGE(a, b)		{(a)*1e6, (b)*1e6, 0}
#define CRANGE_ext(a, b)		{(a)*1e6, (b)*1e6, RF_EXTERNAL}
#define CRANGE_mA(a, b)		{(a)*1e6, (b)*1e6, UNIT_mA}
#define CRANGE_unitless(a, b)	{(a)*1e6, (b)*1e6, 0}
#define CBIP_RANGE(a)		{-(a)*1e6, (a)*1e6, 0}
#define CUNI_RANGE(a)		{0, (a)*1e6, 0}

#define RF_EXTERNAL		(1<<8)
#define UNIT_volt		0
#define UNIT_mA			1
#define UNIT_none		2


#define AI_BUFFER_SIZE		1024	/* max ai fifo size */
#define AO_BUFFER_SIZE		1024	/* max ao fifo size */
#define NUM_CHANNELS_8800	8
#define NUM_CHANNELS_7376	1
#define NUM_CHANNELS_8402	2
#define NUM_CHANNELS_DAC08	1

/* Control/Status registers */
#define INT_ADCFIFO		0	/* INTERRUPT / ADC FIFO register */
#define   INT_EOS		0x1	/* int end of scan */
#define   INT_FHF		0x2	/* int fifo half full */
#define   INT_FNE		0x3	/* int fifo not empty */
#define   INT_MASK		0x3	/* mask of int select bits */
#define   INTE			0x4	/* int enable */
#define   DAHFIE		0x8	/* dac half full int enable */
#define   EOAIE			0x10	/* end of acq. int enable */
#define   DAHFI			0x20	/* dac half full status / clear */
#define   EOAI			0x40	/* end of acq. int status / clear */
#define   INT			0x80	/* int status / clear */
#define   EOBI			0x200	/* end of burst int status */
#define   ADHFI			0x400	/* half-full int status */
#define   ADNEI			0x800	/* fifo not empty int status (latch) */
#define   ADNE			0x1000	/* fifo not empty status (realtime) */
#define   DAEMIE		0x1000	/* dac empty int enable */
#define   LADFUL		0x2000	/* fifo overflow / clear */
#define   DAEMI			0x4000	/* dac fifo empty int status / clear */

#define ADCMUX_CONT		2	/* ADC CHANNEL MUX AND CONTROL reg */
#define   BEGIN_SCAN(x)		((x) & 0xf)
#define   END_SCAN(x)		(((x) & 0xf) << 4)
#define   GAIN_BITS(x)		(((x) & 0x3) << 8)
#define   UNIP			0x800	/* Analog front-end unipolar mode */
#define   SE			0x400	/* Inputs in single-ended mode */
#define   PACER_MASK		0x3000	/* pacer source bits */
#define   PACER_INT		0x1000	/* int. pacer */
#define   PACER_EXT_FALL	0x2000	/* ext. falling edge */
#define   PACER_EXT_RISE	0x3000	/* ext. rising edge */
#define   EOC			0x4000	/* adc not busy */

#define TRIG_CONTSTAT		 4	/* TRIGGER CONTROL/STATUS register */
#define   SW_TRIGGER		0x1	/* software start trigger */
#define   EXT_TRIGGER		0x2	/* ext. start trigger */
#define   ANALOG_TRIGGER	0x3	/* ext. analog trigger */
#define   TRIGGER_MASK		0x3	/* start trigger mask */
#define   TGPOL			0x04	/* invert trigger (1602 only) */
#define   TGSEL			0x08	/* edge/level trigerred (1602 only) */
#define   TGEN			0x10	/* enable external start trigger */
#define   BURSTE		0x20	/* burst mode enable */
#define   XTRCL			0x80	/* clear external trigger */

#define CALIBRATION_REG		6	/* CALIBRATION register */
#define   SELECT_8800_BIT	0x100	/* select 8800 caldac */
#define   SELECT_TRIMPOT_BIT	0x200	/* select ad7376 trim pot */
#define   SELECT_DAC08_BIT	0x400	/* select dac08 caldac */
#define   CAL_SRC_BITS(x)	(((x) & 0x7) << 11)
#define   CAL_EN_BIT		0x4000	/* calibration source enable */
#define   SERIAL_DATA_IN_BIT	0x8000	/* serial data bit going to caldac */

#define DAC_CSR			0x8	/* dac control and status register */
#define   DACEN			0x02	/* dac enable */
#define   DAC_MODE_UPDATE_BOTH	0x80	/* update both dacs */

static inline unsigned int DAC_RANGE(unsigned int channel, unsigned int range)
{
	return (range & 0x3) << (8 + 2 * (channel & 0x1));
}

static inline unsigned int DAC_RANGE_MASK(unsigned int channel)
{
	return 0x3 << (8 + 2 * (channel & 0x1));
};

/* bits for 1602 series only */
#define   DAC_EMPTY		0x1	/* fifo empty, read, write clear */
#define   DAC_START		0x4	/* start/arm fifo operations */
#define   DAC_PACER_MASK	0x18	/* bits that set pacer source */
#define   DAC_PACER_INT		0x8	/* int. pacing */
#define   DAC_PACER_EXT_FALL	0x10	/* ext. pacing, falling edge */
#define   DAC_PACER_EXT_RISE	0x18	/* ext. pacing, rising edge */

static inline unsigned int DAC_CHAN_EN(unsigned int channel)
{
	return 1 << (5 + (channel & 0x1));	/*  enable channel 0 or 1 */
};

/* analog input fifo */
#define ADCDATA			0	/* ADC DATA register */
#define ADCFIFOCLR		2	/* ADC FIFO CLEAR */

/* pacer, counter, dio registers */
#define ADC8254			0
#define DIO_8255		4
#define DAC8254			8

/* analog output registers for 100x, 1200 series */
static inline unsigned int DAC_DATA_REG(unsigned int channel)
{
    return 2 * (channel & 0x1);
}

/* analog output registers for 1602 series*/
#define DACDATA			0	/* DAC DATA register */
#define DACFIFOCLR		2	/* DAC FIFO CLEAR */

#define IS_UNIPOLAR		0x4	/* unipolar range mask */

#if 0
struct comedi_krange {
    int min;	/* fixed point, multiply by 1e-6 */
	int max;	/* fixed point, multiply by 1e-6 */
    unsigned int flags;
};

struct comedi_lrange {
    int length;
    struct comedi_krange range[GCC_ZERO_LENGTH_ARRAY];
};
#endif

#define CB_PCIDAS_AI_RANGE_IDX 0
#define CB_PCIDAS_AO_RANGE_IDX 2

/* analog input ranges for most boards */
struct a4l_rngtab cb_pcidas_ai_ranges = {
length: 8, 
    rngs: {
RANGE_V(-10,10),
    RANGE_V(-5,5),
    RANGE_V(-2.5,2.5),
    RANGE_V(-1.25,1.25),
    RANGE_V(0,10),
    RANGE_V(0,5),
    RANGE_V(0,2.5),
    RANGE_V(0,1.25),
    }
    };

/* pci-das1001 input ranges */
struct a4l_rngtab cb_pcidas_alt_ranges = {8, {
RANGE_V(-10,10),
    RANGE_V(-1,1),
    RANGE_V(-0.1,0.1),
    RANGE_V(-0.01,0.01),
    RANGE_V(0,10),
    RANGE_V(0,1),
    RANGE_V(0,0.1),
    RANGE_V(0,0.01),
}};

/* analog output ranges */
struct a4l_rngtab cb_pcidas_ao_ranges = {4, {
    RANGE_V(-5,5),
    RANGE_V(-10,10),
    RANGE_V(0,5),
    RANGE_V(0,10),
}};

struct a4l_rngdesc cb_pcidas_range_ai_table = RNG_GLOBAL(cb_pcidas_ai_ranges);
struct a4l_rngdesc cb_pcidas_range_alt_table = RNG_GLOBAL(cb_pcidas_alt_ranges);
struct a4l_rngdesc cb_pcidas_range_ao_table = RNG_GLOBAL(cb_pcidas_ao_ranges);

enum trimpot_model {
	AD7376,
	AD8402,
};

struct cb_pcidas_board {
char *name;
int device_id;  /*  PCI Device ID */
int ai_nchan;		/*  Inputs in single-ended mode */
int ai_bits;		/*  analog input resolution */
int ai_speed;		/*  fastest conversion period in ns */
int ao_nchan;		/*  number of analog out channels */
int ao_bits;
int has_ao_fifo;	/*  analog output has fifo */
int ao_scan_speed;	/*  analog output scan speed for 1602 series */
int fifo_size;		/*  number of samples fifo can hold */
struct a4l_rngdesc *ranges_ao;
struct a4l_rngdesc *ranges_ai;
enum trimpot_model trimpot;
unsigned has_dac08:1;
unsigned is_1602:1;
};


static struct cb_pcidas_board cb_pcidas_boards[] = {
  {
.name		= "pci-das1602/16",
     .device_id         = 0x0001,
     .ai_nchan	        = 16,
     .ai_bits	        = 16,
     .ai_speed	        = 5000,
     .ao_nchan	        = 2,
     .ao_bits           = 16,
     .has_ao_fifo	= 1,
     .ao_scan_speed	= 10000,
     .fifo_size	        = 512,
     .ranges_ao	        = &(cb_pcidas_range_ao_table),
     .ranges_ai         = &(cb_pcidas_range_ai_table),
     .trimpot	        = AD8402,
     .has_dac08	        = 1,
     .is_1602	        = 1,
	},
  {
		.name		= "pci-das1200",
		.device_id      = 0x000f,
		.ai_nchan	= 16,
		.ai_bits	= 12,
		.ai_speed	= 3200,
		.ao_nchan	= 2,
		.fifo_size	= 1024,
		 //.ranges		= &cb_pcidas_ranges,
		.trimpot	= AD7376,
	},
  {
		.name		= "pci-das1602/12",
		.device_id      = 0x0010,
		.ai_nchan	= 16,
		.ai_bits	= 12,
		.ai_speed	= 3200,
		.ao_nchan	= 2,
		.has_ao_fifo	= 1,
		.ao_scan_speed	= 4000,
		.fifo_size	= 1024,
		 //.ranges		= &cb_pcidas_ranges,
		.trimpot	= AD7376,
		.is_1602	= 1,
	},
  {
		.name		= "pci-das1200/jr",
		.device_id      = 0x0019,
		.ai_nchan	= 16,
		.ai_bits	= 12,
		.ai_speed	= 3200,
		.fifo_size	= 1024,
		 //.ranges		= &cb_pcidas_ranges,
		.trimpot	= AD7376,
	},
  {
		.name		= "pci-das1602/16/jr",
		.device_id      = 0x001c,
		.ai_nchan	= 16,
		.ai_bits	= 16,
		.ai_speed	= 5000,
		.fifo_size	= 512,
		 //.ranges		= &cb_pcidas_ranges,
		.trimpot	= AD8402,
		.has_dac08	= 1,
		.is_1602	= 1,
	},
  {
		.name		= "pci-das1000",
		.device_id      = 0x004c,
		.ai_nchan	= 16,
		.ai_bits	= 12,
		.ai_speed	= 4000,
		.fifo_size	= 1024,
		 //.ranges		= &cb_pcidas_ranges,
		.trimpot	= AD7376,
	},
  {
		.name		= "pci-das1001",
		.device_id      = 0x001a,
		.ai_nchan	= 16,
		.ai_bits	= 12,
		.ai_speed	= 6800,
		.ao_nchan	= 2,
		.fifo_size	= 1024,
		 //.ranges		= &cb_pcidas_alt_ranges,
		.trimpot	= AD7376,
	},
  {
		.name		= "pci-das1002",
		.device_id      = 0x001b,
		.ai_nchan	= 16,
		.ai_bits	= 12,
		.ai_speed	= 6800,
		.ao_nchan	= 2,
		.fifo_size	= 1024,
		 //.ranges		= &cb_pcidas_ranges,
		.trimpot	= AD7376,
	},
};

struct cb_pcidas_private {
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
	/* fifo buffers */
	unsigned short ai_buffer[AI_BUFFER_SIZE];
	unsigned short ao_buffer[AO_BUFFER_SIZE];
	/* divisors of master clock for analog output pacing */
	unsigned int ao_divisor1;
	unsigned int ao_divisor2;
	/* number of analog output samples remaining */
	unsigned int ao_count;
	/* cached values for readback */
	unsigned short ao_value[2];
	unsigned int caldac_value[NUM_CHANNELS_8800];
	unsigned int trimpot_value[NUM_CHANNELS_8402];
	unsigned int dac08_value;
	unsigned int calibration_source;
};

#endif /*__CB_PCIDAS_H_*/
