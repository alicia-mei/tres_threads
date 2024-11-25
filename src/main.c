/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>

#define SLEEP_TIME_MS	1

#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#if !DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(LED1_NODE)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

K_MUTEX_DEFINE(timer_mutex);
K_MUTEX_DEFINE(button_mutex);

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback button_cb_data; //callback!!!

static struct gpio_dt_spec alberto = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios,				
						     {0});

struct printk_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	uint32_t led;
	uint32_t cnt;
};

K_FIFO_DEFINE(printk_fifo);

struct led {
	struct gpio_dt_spec spec;
	uint8_t num;
};

static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

static const struct led led1 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
	.num = 1,
};

volatile bool button_event = false;

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    /* Set flag to indicate button press */
    button_event = !button_event;
}

void blink(const struct led *led, uint32_t sleep_ms, uint32_t id)
{
	const struct gpio_dt_spec *spec = &led->spec;
	int cnt = 0;
	int ret;

	if (!device_is_ready(spec->port)) {
		printk("Error: %s device is not ready\n", spec->port->name);
		return;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d (LED '%d')\n",
			ret, spec->pin, led->num);
		return;
	}
	while (1) {
		printk("aaaa\n");
        if(k_mutex_lock(&timer_mutex, K_FOREVER) == 0){
			gpio_pin_set(spec->port, spec->pin, cnt % 2);

			struct printk_data_t tx_data = { .led = id, .cnt = cnt };

			size_t size = sizeof(struct printk_data_t);
			char *mem_ptr = k_malloc(size);
			__ASSERT_NO_MSG(mem_ptr != 0);

			memcpy(mem_ptr, &tx_data, size);

			k_fifo_put(&printk_fifo, mem_ptr);
			printk("A funcao ativada periodicamente foi executada\n");
			k_msleep(sleep_ms);
			k_mutex_unlock(&timer_mutex);
			cnt++;
		}
		else{
			printk("nao peguei timer mutex\n");
		}
	}
}

void timer(void)
{
	printk("bbbbbb\n");
	blink(&led0, 5000, 0);
}

void thread_button(void)
{
    int ret;

    if (!gpio_is_ready_dt(&button)) {
        printk("Error: button device %s is not ready\n", button.port->name);
        return;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure button pin\n", ret);
        return;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        printk("Error %d: failed to configure button interrupt\n", ret);
        return;
    }

    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    printk("Button callback configured\n");

    while (1) {
        if (button_event) {
            /* Lock button_mutex */
            if (k_mutex_lock(&button_mutex, K_MSEC(10)) == 0) {
                printk("Button mutex locked\n");
            } 
		} else {
			k_mutex_unlock(&button_mutex);
			printk("Button mutex unlocked\n");
		}
        k_msleep(SLEEP_TIME_MS);
    }
}

void dupla(void)
{
	while (1) {
        if (k_mutex_lock(&timer_mutex, K_MSEC(1)) == 0){
			k_mutex_unlock(&timer_mutex);
		}else{
			if(k_mutex_lock(&button_mutex, K_MSEC(1)) == 0) {
                k_mutex_unlock(&button_mutex);
            } else {
				printk("As duas funções foram ativadas ao mesmo tempo\n");
            }
		}
		k_msleep(SLEEP_TIME_MS);
    }
}

K_THREAD_DEFINE(timer_id, STACKSIZE, timer, NULL, NULL, NULL,
		PRIORITY, 0, 0);
K_THREAD_DEFINE(button_id, STACKSIZE, thread_button, NULL, NULL, NULL,
 		PRIORITY, 0, 0);
K_THREAD_DEFINE(dupla_id, STACKSIZE, dupla, NULL, NULL, NULL,
 		PRIORITY, 0, 0);