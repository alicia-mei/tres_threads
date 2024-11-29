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

/* tamanho da stack de cada thread*/
#define STACKSIZE 1024

/* definir prioridade - nesse caso, igual para todas */
#define PRIORITY 7

/* definir leds de acordo com o device tree */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

/* confere se o led0 é suportado pela placa */
#if !DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

/* confere se o led1 é suportado pela placa */
#if !DT_NODE_HAS_STATUS_OKAY(LED1_NODE)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

/* define os mutexes usados pela thread periódica e pela thread do botão*/
K_MUTEX_DEFINE(timer_mutex);
K_MUTEX_DEFINE(button_mutex);

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});

static struct gpio_callback button_cb_data; 

static struct gpio_dt_spec alberto = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios,				
						     {0});

struct printk_data_t {
	void *fifo_reserved;
	uint32_t led;
	uint32_t cnt;
};

K_FIFO_DEFINE(printk_fifo);

struct led {
	struct gpio_dt_spec spec;
	uint8_t num;
};

/* cria os objetos led0 e led1 */
static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

static const struct led led1 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
	.num = 1,
};

/* booleano usado para saber o estado do botão */
volatile bool button_event = false;

/* função que muda a variável que indica o estado do botão*/
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    button_event = !button_event;
}

/* função periódica que pisca led */
/* obs.: sabemos atualmente que criar uma função periódica utilizando while(1) e k_msleep não é o ideal, mas não tivemos tempo de refazer.*/
/*POR FAVOR, CONSIDERE NOSSO CONHECIMENTO ATUAL, PROFESSOR S21 ! ! ! ! !*/
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
        if(k_mutex_lock(&timer_mutex, K_FOREVER) == 0){ //se pegou o mutex
			gpio_pin_set(spec->port, spec->pin, cnt % 2); //muda o estado do led

			struct printk_data_t tx_data = { .led = id, .cnt = cnt };

			size_t size = sizeof(struct printk_data_t);
			char *mem_ptr = k_malloc(size);
			__ASSERT_NO_MSG(mem_ptr != 0);

			memcpy(mem_ptr, &tx_data, size);

			k_fifo_put(&printk_fifo, mem_ptr);
			printk("A funcao ativada periodicamente foi executada\n");
			k_msleep(sleep_ms);
			k_mutex_unlock(&timer_mutex); //solta o mutex
			cnt++;
		}
		else{
			printk("nao peguei timer mutex\n");
		}
	}
}

void timer(void)
{
	blink(&led0, 5000, 0); //chama a função blink
}

/* função do botão */
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
        if (button_event) { //se o botão estiver ativado
            if (k_mutex_lock(&button_mutex, K_MSEC(10)) == 0) { //pega o mutex
                printk("Button mutex locked\n");
            } 
		} else {
			k_mutex_unlock(&button_mutex); //senão, solta o mutex
			printk("Button mutex unlocked\n");
		}
        k_msleep(SLEEP_TIME_MS);
    }
}

/* função que printa apenas quando as outras duas estiverem ativadas */
void dupla(void)
{
	while (1) {
        if (k_mutex_lock(&timer_mutex, K_MSEC(1)) == 0){ //se o mutex do timer estiver disponível, significa que a função periódica não está ativada
			k_mutex_unlock(&timer_mutex); //destrava o mutex
		}else{ //se não conseguiu pegar o mutex do timer
			if(k_mutex_lock(&button_mutex, K_MSEC(1)) == 0) {  //tenta pegar o mutex do botão, se conseguir, significa que a função do botão do não está ativada
                k_mutex_unlock(&button_mutex); //destrava o mutex
            } else { //se não conseguir, ambas estão ativadas ao mesmo tempo
				printk("As duas funções foram ativadas ao mesmo tempo\n");
            }
		}
		k_msleep(SLEEP_TIME_MS);
    }
}

/* cria e inicializa as threads */
K_THREAD_DEFINE(timer_id, STACKSIZE, timer, NULL, NULL, NULL,
		PRIORITY, 0, 0);
K_THREAD_DEFINE(button_id, STACKSIZE, thread_button, NULL, NULL, NULL,
 		PRIORITY, 0, 0);
K_THREAD_DEFINE(dupla_id, STACKSIZE, dupla, NULL, NULL, NULL,
 		PRIORITY, 0, 0);