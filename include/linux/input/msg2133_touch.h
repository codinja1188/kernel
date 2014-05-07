#ifndef __LINUX_MSG2033_H_
#define __LINUX_MSG2033_H_

struct msg2133_platform_data {
	unsigned int irq_active_high;
	unsigned int reset_gpio;
	unsigned int irq_gpio;
	void (*gpio_pin_get)(void);
	void (*gpio_pin_put)(void);
	
};


#endif
