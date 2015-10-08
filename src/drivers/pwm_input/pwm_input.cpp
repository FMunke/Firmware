/****************************************************************************
 *
 *   Copyright (c) 2015 Andrew Tridgell. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file pwm_input.cpp
 *
 * PWM input driver based on earlier driver from Evan Slatyer,
 * which in turn was based on drv_hrt.c
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <board_config.h>
#include <drivers/drv_pwm_input.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include <uORB/uORB.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/subsystem_info.h>

#include <drivers/drv_device.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define MINIMAL_DISTANCE 200
#define MAXIMAL_DISTANCE 10000

#if HRT_TIMER == PWMIN_TIMER
#error cannot share timer between HRT and PWMIN
#endif

#if !defined(GPIO_PWM_IN) || !defined(PWMIN_TIMER) || !defined(PWMIN_TIMER_CHANNEL)
#error PWMIN defines are needed in board_config.h for this board
#endif

/* PWMIN configuration */
#if   PWMIN_TIMER == 1
# define PWMIN_TIMER_BASE	STM32_TIM1_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM1EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1CC
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM1_CLKIN
#elif PWMIN_TIMER == 2
# define PWMIN_TIMER_BASE	STM32_TIM2_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM2EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM2
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM2_CLKIN
#elif PWMIN_TIMER == 3
# define PWMIN_TIMER_BASE	STM32_TIM3_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM3EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM3
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM3_CLKIN
#elif PWMIN_TIMER == 4
# define PWMIN_TIMER_BASE	STM32_TIM4_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB1ENR_TIM4EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM4
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM4_CLKIN
#elif PWMIN_TIMER == 5
# define PWMIN_TIMER_BASE	STM32_TIM5_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM5EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM5
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM5_CLKIN
#elif PWMIN_TIMER == 8
# define PWMIN_TIMER_BASE	STM32_TIM8_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB2ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM8EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM8CC
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM8_CLKIN
#elif PWMIN_TIMER == 9
# define PWMIN_TIMER_BASE	STM32_TIM9_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM9EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1BRK
# define PWMIN_TIMER_CLOCK	STM32_APB1_TIM9_CLKIN
#elif PWMIN_TIMER == 10
# define PWMIN_TIMER_BASE	STM32_TIM10_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM10EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1UP
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM10_CLKIN
#elif PWMIN_TIMER == 11
# define PWMIN_TIMER_BASE	STM32_TIM11_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM11EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1TRGCOM
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM11_CLKIN
#elif PWMIN_TIMER == 12
# define PWMIN_TIMER_BASE	STM32_TIM12_BASE
# define PWMIN_TIMER_POWER_REG	STM32_RCC_APB1ENR
# define PWMIN_TIMER_POWER_BIT	RCC_APB2ENR_TIM12EN
# define PWMIN_TIMER_VECTOR	STM32_IRQ_TIM1TRGCOM
# define PWMIN_TIMER_CLOCK	STM32_APB2_TIM12_CLKIN
#else
# error PWMIN_TIMER must be a value between 1 and 12
#endif

/*
 * HRT clock must be at least 1MHz
 */
#if PWMIN_TIMER_CLOCK <= 1000000
# error PWMIN_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(PWMIN_TIMER_BASE + _reg))

#define rCR1		REG(STM32_GTIM_CR1_OFFSET)
#define rCR2		REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR		REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER		REG(STM32_GTIM_DIER_OFFSET)
#define rSR		REG(STM32_GTIM_SR_OFFSET)
#define rEGR		REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1		REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2		REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER		REG(STM32_GTIM_CCER_OFFSET)
#define rCNT		REG(STM32_GTIM_CNT_OFFSET)
#define rPSC		REG(STM32_GTIM_PSC_OFFSET)
#define rARR		REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1		REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2		REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3		REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4		REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR		REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR		REG(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if PWMIN_TIMER_CHANNEL == 1
 #define rCCR_PWMIN_A		rCCR1			/* compare register for PWMIN */
 #define DIER_PWMIN_A		(GTIM_DIER_CC1IE) 	/* interrupt enable for PWMIN */
 #define SR_INT_PWMIN_A		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
 #define rCCR_PWMIN_B		rCCR2 			/* compare register for PWMIN */
 #define SR_INT_PWMIN_B		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
 #define CCMR1_PWMIN		((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT))
 #define CCMR2_PWMIN		0
 #define CCER_PWMIN		(GTIM_CCER_CC2P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
 #define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
 #define SMCR_PWMIN_1		(0x05 << GTIM_SMCR_TS_SHIFT)
 #define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#elif PWMIN_TIMER_CHANNEL == 2
 #define rCCR_PWMIN_A		rCCR2			/* compare register for PWMIN */
 #define DIER_PWMIN_A		(GTIM_DIER_CC2IE)	/* interrupt enable for PWMIN */
 #define SR_INT_PWMIN_A		GTIM_SR_CC2IF		/* interrupt status for PWMIN */
 #define rCCR_PWMIN_B		rCCR1			/* compare register for PWMIN */
 #define DIER_PWMIN_B		GTIM_DIER_CC1IE		/* interrupt enable for PWMIN */
 #define SR_INT_PWMIN_B		GTIM_SR_CC1IF		/* interrupt status for PWMIN */
 #define CCMR1_PWMIN		((0x01 << GTIM_CCMR1_CC2S_SHIFT) | (0x02 << GTIM_CCMR1_CC1S_SHIFT))
 #define CCMR2_PWMIN		0
 #define CCER_PWMIN		(GTIM_CCER_CC1P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
 #define SR_OVF_PWMIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
 #define SMCR_PWMIN_1		(0x06 << GTIM_SMCR_TS_SHIFT)
 #define SMCR_PWMIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_PWMIN_1)
#else
 #error PWMIN_TIMER_CHANNEL must be either 1 and 2.
#endif

#define TIMEOUT     300000 /* reset after no responce over this time in microseconds [0.3 secs] */

float lpf_value;

class PWMIN : device::CDev
{
public:
	PWMIN();
	virtual ~PWMIN();

	virtual int init();
	virtual int open(struct file *filp);
	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);

	void _publish(uint16_t status, uint32_t period, uint32_t pulse_width);
	void _print_info(void);
    void hard_reset();

private:
	uint32_t error_count;
	uint32_t pulses_captured;
	uint32_t last_period;
	uint32_t last_width;

    uint32_t list_width[5];
    uint64_t last_poll_time;

    float low_filtering_coeff;
    float hight_filtering_coeff;

	RingBuffer *reports;
	bool timer_started;

    range_finder_report data;
	orb_advert_t		range_finder_pub;

	hrt_call hard_reset_call;	// HRT callout for note completion
	hrt_call freeze_test_call;	// HRT callout for note completion

	void timer_init(void);

    void turn_on();
    void turn_off();
    void freeze_test();

};

static int pwmin_tim_isr(int irq, void *context);
static void pwmin_start(bool full_start);
static void pwmin_info(void);
static void pwmin_test(void);
static void pwmin_reset(void);
uint32_t pwm_median_filtering(uint32_t*, uint16_t);
float pwm_lpf_filtering(float, float, float, float);

static PWMIN *g_dev;

PWMIN::PWMIN() :
	CDev("pwmin", PWMIN0_DEVICE_PATH),
	error_count(0),
	pulses_captured(0),
	last_period(0),
	last_width(0),
	reports(nullptr),
	timer_started(false),
    range_finder_pub(-1)
{
	memset(&data, 0, sizeof(data));
}

PWMIN::~PWMIN()
{
	if (reports != nullptr)
		delete reports;
}

/*
  initialise the driver. This doesn't actually start the timer (that
  is done on open). We don't start the timer to allow for this driver
  to be started in init scripts when the user may be using the input
  pin as PWM output
 */
int
PWMIN::init()
{
	// we just register the device in /dev, and only actually
	// activate the timer when requested to when the device is opened
	CDev::init();

    float param_minimal_distance = MINIMAL_DISTANCE * 1e-3f;
    float param_maximal_distance = MAXIMAL_DISTANCE * 1e-3f;
    if (param_get(param_find("SENS_RANGE_MIN"), &param_minimal_distance))
    {
        fprintf(stderr, "ERROR! SENS_RANGE_MIN not read, setting lidar minimum to predefined value.\n");
    }
    if (param_get(param_find("SENS_RANGE_MAX"), &param_maximal_distance))
    {
        fprintf(stderr, "ERRORR! SENS_RANGE_MAX not read, setting lidar maximum to predefined value.\n");
    }

    data.type = RANGE_FINDER_TYPE_LASER;
    data.minimum_distance = param_minimal_distance;
    data.maximum_distance = param_maximal_distance;

    range_finder_pub = orb_advertise(ORB_ID(sensor_range_finder), &data);
    DOG_PRINT("[pwm_input] advertising %d\n"
            ,range_finder_pub);

	reports = new RingBuffer(2, sizeof(struct pwm_input_s));
	if (reports == nullptr) {
		return -ENOMEM;
	}

    // Schedule freeze check to invoke periodically
    hrt_call_every(&freeze_test_call, 0, TIMEOUT, reinterpret_cast<hrt_callout>(&PWMIN::freeze_test), this);

	return OK;
}

/*
 * Initialise the timer we are going to use.
 */
void PWMIN::timer_init(void)
{
	// run with interrupts disabled in case the timer is already
	// setup. We don't want it firing while we are doing the setup
	irqstate_t flags = irqsave();
	stm32_configgpio(GPIO_PWM_IN);

	/* claim our interrupt vector */
	irq_attach(PWMIN_TIMER_VECTOR, pwmin_tim_isr);

	/* Clear no bits, set timer enable bit.*/
	modifyreg32(PWMIN_TIMER_POWER_REG, 0, PWMIN_TIMER_POWER_BIT);

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_PWMIN_A;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PWMIN;
	rCCMR2 = CCMR2_PWMIN;
	rSMCR = SMCR_PWMIN_1;	/* Set up mode */
	rSMCR = SMCR_PWMIN_2;	/* Enable slave mode controller */
	rCCER = CCER_PWMIN;
	rDCR = 0;

	// for simplicity scale by the clock in MHz. This gives us
	// readings in microseconds which is typically what is needed
	// for a PWM input driver
	uint32_t prescaler = PWMIN_TIMER_CLOCK/1000000UL;

	/*
	 * define the clock speed. We want the highest possible clock
	 * speed that avoids overflows.
	 */
	rPSC = prescaler - 1;

	/* run the full span of the counter. All timers can handle
	 * uint16 */
	rARR = UINT16_MAX;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	/* enable interrupts */
	up_enable_irq(PWMIN_TIMER_VECTOR);

	irqrestore(flags);

	timer_started = true;
}

void
PWMIN::freeze_test()
{
    /* timeout is true if least read was away back */
    bool timeout = false;
    timeout = (hrt_absolute_time() - last_poll_time > TIMEOUT) ? true : false;
    if (timeout) {
        DOG_PRINT("[pwm_input] Lidar is down, reseting\n");
        hard_reset();
    }
}

void
PWMIN::turn_on() { stm32_gpiowrite(GPIO_VDD_RANGEFINDER_EN, 1); }

void
PWMIN::turn_off() { stm32_gpiowrite(GPIO_VDD_RANGEFINDER_EN, 0); }

void
PWMIN::hard_reset()
{
    turn_off();
    hrt_call_after(&hard_reset_call, 9000, reinterpret_cast<hrt_callout>(&PWMIN::turn_on), this);
}

/*
  hook for open of the driver. We start the timer at this point, then
  leave it running
 */
int
PWMIN::open(struct file *filp)
{
	if (g_dev == nullptr) {
		return -EIO;
	}
	int ret = CDev::open(filp);
	if (ret == OK && !timer_started) {
		g_dev->timer_init();
	}
	return ret;
}


/*
  handle ioctl requests
 */
int
PWMIN::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 500))
				return -EINVAL;

			irqstate_t flags = irqsave();
			if (!reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}
			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return reports->size();

	case SENSORIOCRESET:
		/* user has asked for the timer to be reset. This may
		   be needed if the pin was used for a different
		   purpose (such as PWM output)
		*/
		timer_init();
		return OK;

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}


/*
  read some samples from the device
 */
ssize_t
PWMIN::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct pwm_input_s);
	struct pwm_input_s *buf = reinterpret_cast<struct pwm_input_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	while (count--) {
		if (reports->get(buf)) {
			ret += sizeof(struct pwm_input_s);
			buf++;
		}
	}

	/* if there was no data, warn the caller */
	return ret ? ret : -EAGAIN;
}

/*
  publish some data from the ISR in the ring buffer
 */
void PWMIN::_publish(uint16_t status, uint32_t period, uint32_t pulse_width)
{
	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PWMIN) {
		error_count++;
		return;
	}

    last_poll_time = hrt_absolute_time();

	struct pwm_input_s pwmin_report;
	pwmin_report.timestamp = last_poll_time;
	pwmin_report.error_count = error_count;
	pwmin_report.period = period;
	pwmin_report.pulse_width = pulse_width;

    data.distance = pulse_width * 1e-3f;
    data.timestamp = pwmin_report.timestamp;
    data.error_count = error_count;

    if (pulse_width < MINIMAL_DISTANCE || pulse_width > MAXIMAL_DISTANCE) {
        data.valid = false;
    } else {
        data.valid = true;
    }

    if (range_finder_pub > 0) {
        orb_publish(ORB_ID(sensor_range_finder), range_finder_pub, &data);
    }

	reports->force(&pwmin_report);

	last_period = period;
	last_width = pulse_width;
	pulses_captured++;
}

/*
  print information on the last captured
 */
void PWMIN::_print_info(void)
{
	if (!timer_started) {
		printf("timer not started - try the 'test' command\n");
	} else {
		printf("count=%u period=%u width=%u\n",
		       (unsigned)pulses_captured,
		       (unsigned)last_period,
		       (unsigned)last_width);
	}
}

/*
 * Low pass filter filtering
 * applies LPF with 2 coeffs depending on changes
 */
float pwm_lpf_filtering(float current_value, float low_filtering_coeff, float hight_filtering_coeff, float epsilon) {
    float delta = fabsf(lpf_value - current_value);
    if ( delta > epsilon) {
        lpf_value += hight_filtering_coeff*(current_value - lpf_value);
    } else {
        lpf_value += low_filtering_coeff*(current_value - lpf_value);
    }
    return lpf_value;
}
/*
 * Median filtering
 */
uint32_t pwm_median_filtering(uint32_t *list_width, uint16_t size) {
    uint16_t i = 0, j = 0;
    uint16_t temp = 0;
    for(;i < size-1;i++) {
        bool was_swaped = false;
        for(j=0;j < (size-i-1);j++) {
            if( list_width[j] > list_width[j+1] ) {
                temp = list_width[j];
                list_width[j] = list_width[j+1];
                list_width[j+1] = temp;
                was_swaped = true;
            }
        }
        if (was_swaped = false)
            break;
    }
    for (i=0;i < size;i++)
    return list_width[size/2];
}
/*
  Handle the interupt, gathering pulse data
 */
static int pwmin_tim_isr(int irq, void *context)
{
	uint16_t status = rSR;
	uint32_t period = rCCR_PWMIN_A;
	uint32_t pulse_width = rCCR_PWMIN_B;

	/* ack the interrupts we just read */
	rSR = 0;

	if (g_dev != nullptr) {
		g_dev->_publish(status, period, pulse_width);
	}
	return OK;
}

/*
  start the driver
 */
static void pwmin_start(bool full_start)
{
	if (g_dev != nullptr) {
		printf("driver already started\n");
		exit(1);
	}
	g_dev = new PWMIN();
	if (g_dev == nullptr) {
		errx(1, "driver allocation failed");
	}
	if (g_dev->init() != OK) {
		errx(1, "driver init failed");
	}
    if (full_start) {
        int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
        if (fd == -1) {
            errx(1, "Failed to open device");
        }
        close(fd);
    }
	exit(0);
}

/*
  test the driver
 */
static void pwmin_test(void)
{
	int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
	if (fd == -1) {
		errx(1, "Failed to open device");
	}
	uint64_t start_time = hrt_absolute_time();

	printf("Showing samples for 5 seconds\n");

	while (hrt_absolute_time() < start_time+5U*1000UL*1000UL) {
		struct pwm_input_s buf;
		if (::read(fd, &buf, sizeof(buf)) == sizeof(buf)) {
			printf("period=%u width=%u error_count=%u\n",
			       (unsigned)buf.period,
			       (unsigned)buf.pulse_width,
			       (unsigned)buf.error_count);
		}
	}
	close(fd);
	exit(0);
}


/*
  reset the timer
 */
static void pwmin_reset(void)
{
    g_dev->hard_reset();
	int fd = open(PWMIN0_DEVICE_PATH, O_RDONLY);
	if (fd == -1) {
		errx(1, "Failed to open device");
	}
	if (ioctl(fd, SENSORIOCRESET, 0) != OK) {
		errx(1, "reset failed");
	}
	close(fd);
	exit(0);
}

/*
  show some information on the driver
 */
static void pwmin_info(void)
{
	if (g_dev == nullptr) {
		printf("driver not started\n");
		exit(1);
	}
	g_dev->_print_info();
	exit(0);
}


/*
  driver entry point
 */
int pwm_input_main(int argc, char * argv[])
{
	const char *verb = argv[1];
    /*
     * init driver and start reading
     */
    bool full_start = false;
    if (!strcmp(argv[2], "full")) {
        full_start = true;
    }

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
            pwmin_start(full_start);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		pwmin_info();
	}

	/*
	 * print test results
	 */
	if (!strcmp(verb, "test")) {
		pwmin_test();
	}

	/*
	 * reset the timer
	 */
	if (!strcmp(verb, "reset")) {
		pwmin_reset();
	}

	errx(1, "unrecognized command, try 'start', 'start full', 'info', 'reset' or 'test'");
	return 0;
}
