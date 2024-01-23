#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

#define DT_DRV_COMPAT shakti_gpio

#ifdef CONFIG_BOARD_SECURE_IOT

#define CLOCK_FREQUENCY_SIM 1000000000
#define CLOCK_FREQUENCY 40000000
#endif


#define GPIO_START 0x00040200 //GPIO Start Address */
#define GPIO_OFFSET 0x08 /*!Generic offset used to access GPIO registers*/


#define GPIO_DIRECTION_CNTRL_REG  (GPIO_START + (0 * GPIO_OFFSET ))
#define GPIO_DATA_REG             (GPIO_START + (1 * GPIO_OFFSET ))
#define GPIO_SET_REG              (GPIO_START + (2 * GPIO_OFFSET ))
#define GPIO_CLEAR_REG            (GPIO_START + (3 * GPIO_OFFSET ))
#define GPIO_TOGGLE_REG           (GPIO_START + (4 * GPIO_OFFSET ))
#define GPIO_QUAL_REG             (GPIO_START + (5 * GPIO_OFFSET ))
#define GPIO_INTERRUPT_CONFIG_REG (GPIO_START + (6 * GPIO_OFFSET ))

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1
#define GPIO_QUAL_MAX_CYCLES 15
#define ALL_GPIO_PINS -1


#define GPIO0  (1 <<  0)
#define GPIO1  (1 <<  1)
#define GPIO2  (1 <<  2)
#define GPIO3  (1 <<  3)
#define GPIO4  (1 <<  4)
#define GPIO5  (1 <<  5)
#define GPIO6  (1 <<  6)
#define GPIO7  (1 <<  7)
#define GPIO8  (1 <<  8)
#define GPIO9  (1 <<  9)
#define GPIO10 (1 << 10)
#define GPIO11 (1 << 11)
#define GPIO12 (1 << 12)
#define GPIO13 (1 << 13)
#define GPIO14 (1 << 14)
#define GPIO15 (1 << 15)
#define GPIO16 (1 << 16)
#define GPIO17 (1 << 17)
#define GPIO18 (1 << 18)
#define GPIO19 (1 << 19)
#define GPIO20 (1 << 20)
#define GPIO21 (1 << 21)
#define GPIO22 (1 << 22)
#define GPIO23 (1 << 23)
#define GPIO24 (1 << 24)
#define GPIO25 (1 << 25)
#define GPIO26 (1 << 26)
#define GPIO27 (1 << 27)
#define GPIO28 (1 << 28)
#define GPIO29 (1 << 29)
#define GPIO30 (1 << 30)
#define GPIO31 (1 << 31)
#define GPIO_COUNT  0x20

typedef void (*gpio_shakti_cfg_func_t)(void);
typedef uint8_t gpio_pin_t;



/* Struct to access GPIO registers as 16 bit registers */
struct gpio_shakti_regs_t
{
    uint32_t  direction;               /*! direction register */
    uint32_t  reserved0;                /*! reserved for future use */
    uint32_t  data;                    /*! data register */
    uint32_t  reserved1;                /*! reserved for future use */
    uint32_t  set;                 /*! set register */
    uint32_t  reserved2;                /*! reserved for future use */
    uint32_t  clear;                   /*! clear register */
    uint32_t  reserved3;                /*! reserved for future use */
    uint32_t  toggle;               /*! toggle register */
    uint32_t  reserved4;                /*! reserved for future use */
    uint8_t  qualification;    /*! qualification register */
    uint8_t  reserved5;                /*! reserved for future use */
    uint16_t  reserved6;              /*! reserved for future use */
    uint32_t  reserved12;              /*! reserved for future use */
    uint32_t  intr_config;     /*! interrupt configuration register */
//  uint16_t  reserved13;              /*! reserved for future use */
    uint32_t  reserved7;              /*! reserved for future use */
};





struct gpio_shakti_config{
/* gpio_driver_config needs to be first */
    struct gpio_driver_config common;
    uintptr_t gpio_base_addr;
    uint32_t gpio_irq_base;
    gpio_shakti_cfg_func_t gpio_cfg_func;

};

struct gpio_shakti_data {

    struct gpio_driver_data common;
    sys_slist_t cb;

};

/* Helper Macros for GPIO */
#define DEV_GPIO_CFG(dev)						\
	((const struct gpio_shakti_config * const)(dev)->config)
#define DEV_GPIO(dev)							\
	((volatile struct gpio_shakti_regs_t *)(DEV_GPIO_CFG(dev))->gpio_base_addr)
#define DEV_GPIO_DATA(dev)				\
	((struct gpio_shakti_data *)(dev)->data)

// struct gpio_shakti_regs_t *gpio;

/* Initialize's the gpio's*/

int gpio_shakti_init(const struct device *dev){
    
    volatile struct gpio_shakti_regs_t *gpio = DEV_GPIO(dev);
    const struct gpio_shakti_config *cfg = DEV_GPIO_CFG(dev);

    gpio = GPIO_START;
    printk("Initialization Done\n");
}


/*configuration function that configures the whether the gpio pin in input or output */

static int gpio_shakti_pin_configure (const struct device *dev, 
                        gpio_pin_t pin, 
                        gpio_flags_t flags){

    volatile struct gpio_shakti_regs_t *gpio = DEV_GPIO(dev);
    // const struct gpio_shakti_regs_t *cfg = DEV_GPIO_CFG(dev);
    printk("Configuration Done11111111111\n");

    gpio->direction |= (flags << pin);
    printk("Configuration Done2\n");

    return 0;
}

/* Reads the data from the data register */

static int gpio_shakti_pin_get_raw(const struct device *dev,
                    gpio_pin_t pin)
{
    volatile struct gpio_shakti_regs_t *gpio = DEV_GPIO(dev);
    return gpio->data;

}

/*Set the value in th selected pin */

static int gpio_shakti_pin_set_raw(const struct device *dev,
                    gpio_pin_t pin,
                    int value)
{
    volatile struct gpio_shakti_regs_t *gpio = DEV_GPIO(dev);   
    gpio ->set = (value << pin);
    printk("set has been done \n");

    return 0;
}


/*Toggle a pin */

static int gpio_shakti_pin_toggle(const struct device *dev,
                    gpio_pin_t pin)
{
    volatile struct gpio_shakti_regs_t *gpio = DEV_GPIO(dev);
    gpio ->toggle = (1 << pin);

    return 0;
}


/*clear the gpio pins */

static int gpio_shakti_pin_clear_raw(const struct device *dev,
                    gpio_pin_t pin)
{
    volatile struct gpio_shakti_regs_t *gpio = DEV_GPIO(dev);   
    gpio ->clear = (1 << pin);
    printk("Cleared \n");

    return 0;
}

static const struct gpio_driver_api gpio_shakti_driver = {
    .pin_configure              = gpio_shakti_pin_configure,
    .port_get_raw               = gpio_shakti_pin_get_raw,   
    .port_set_bits_raw          = gpio_shakti_pin_set_raw,    
    .port_clear_bits_raw        = gpio_shakti_pin_clear_raw,
    .port_toggle_bits           = gpio_shakti_pin_toggle,   
};

static void gpio_shakti_cfg_0(void);

static const struct gpio_shakti_config gpio_shakti_config0 ={
    // .common = {
    //     .port_pin_mask  = GPIO_PORT_PIN_MASK_FROM_DT_INST(0),
    // },
    .gpio_base_addr     = GPIO_START,
    .gpio_irq_base      = GPIO_INTERRUPT_CONFIG_REG,
    // .gpio_cfg_func      = gpio_shakti_config_0,
};

static struct gpio_shakti_data gpio_shakti_data0;

DEVICE_DT_INST_DEFINE(0,
                gpio_shakti_init,
                NULL,
                &gpio_shakti_data0, &gpio_shakti_config0,
                PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,
                &gpio_shakti_driver);


                






































































// typedef void (*shakti_cfg_func_t)(void);

// // #define GPIO_DIRECTION_CNTRL_REG  (GPIO_START + (0 * GPIO_OFFSET ))
// // #define GPIO_DATA_REG             (GPIO_START + (1 * GPIO_OFFSET ))
// // #define GPIO_SET_REG              (GPIO_START + (2 * GPIO_OFFSET ))
// // #define GPIO_CLEAR_REG            (GPIO_START + (3 * GPIO_OFFSET ))
// // #define GPIO_TOGGLE_REG           (GPIO_START + (4 * GPIO_OFFSET ))
// // #define GPIO_QUAL_REG             (GPIO_START + (5 * GPIO_OFFSET ))
// // #define GPIO_INTERRUPT_CONFIG_REG (GPIO_START + (6 * GPIO_OFFSET ))

// // #define GPIO_IN  0
// // #define GPIO_OUT 1
// // #define GPIO_QUAL_MAX_CYCLES 15
// // #define ALL_GPIO_PINS -1

// struct gpio_shakti_reg_t
// {
//     uint32_t  direction;
//     uint32_t  reserved0;
//     uint32_t  data;
//     uint32_t  reserved1;
//     uint32_t  set;
//     uint32_t  reserved2;
//     uint32_t  clear;
//     uint32_t  reserved3;
//     uint32_t  toggle;
//     uint32_t  reserved4;
//     uint8_t  qualification;
//     uint8_t  reserved5;
//     uint16_t  reserved6;
//     uint32_t  reserved12;
//     uint32_t  intr_config;
//     uint32_t  reserved7;
// };



// struct gpio_shakti_config {
// 	/* gpio_driver_config needs to be first */
// 	struct gpio_driver_config common;
// 	uintptr_t            GPIO_START;
// 	/* multi-level encoded interrupt corresponding to pin 0 */
// 	uint32_t                GPIO_INTERRUPT_CONFIG_REG;
// 	shakti_cfg_func_t    gpio_cfg_func;
// };


// struct gpio_shakti_data {
// 	/* gpio_driver_data needs to be first */
// 	struct gpio_driver_data common;
// 	/* list of callbacks */
// 	sys_slist_t cb;
// };


