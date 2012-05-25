#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <board.h>
#include <linux/gpio_dev.h>


static BOARD_LED_NAME leds[] = {
    kLedAdsl, kLedHpna, kLedWanData, kLedPPP,
    kLedSes, kLedVoip, kLedVoip1, kLedVoip2, 
    kLedPots, kLedSecAdsl, kLedDect, kLedPower, 
    kLedUsbHost, kLedUsbHost2
};

static int led_open(struct inode *inode, struct file *filp) {
    return 0;
}

static int led_release(struct inode *inode, struct file *filp) {
    return 0;
}

static int led_ioctl(struct inode *inode, struct file * filep, unsigned int cmd, unsigned long gpio) {    
    printk("gpio: %d\n", gpio);
    if (gpio >= sizeof(leds) / sizeof(BOARD_LED_NAME))
        return -ENODEV;

    switch (cmd) {
    case GPIO_CLEAR:
        kerSysLedCtrl(leds[gpio], kLedStateOff);
        break;
    case GPIO_SET:
        kerSysLedCtrl(leds[gpio], kLedStateOn);
        break;
    case GPIO_GET:
        kerSysLedCtrl(leds[gpio], kLedStateFail);
        break;
    case GPIO_DIR_IN:
        kerSysLedCtrl(leds[gpio], kLedStateSlowBlinkContinues);
        break;
    case GPIO_DIR_OUT:
        kerSysLedCtrl(leds[gpio], kLedStateFastBlinkContinues);
        break;
    default:
        return -EBADRQC;
    }
    return 0;
}

static struct file_operations gpio_fops = {
	.owner	 = THIS_MODULE,
	.open	 = led_open,
	.release = led_release,
    .ioctl   = led_ioctl,
};

static struct miscdevice gpio_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gpio",
    .fops = &gpio_fops,    
};

static int __init led_init(void) {
    int result;
    result = misc_register(&gpio_dev);

	if(result)
		printk(KERN_INFO "fail to create device\n");
    else
		printk(KERN_INFO "gpio device created successfully, minor=%d\n", gpio_dev.minor);
    
    return result;
}

static void __exit led_cleanup(void) {
    if (gpio_dev.minor != MISC_DYNAMIC_MINOR)
        misc_deregister(&gpio_dev);
}

module_init(led_init);
module_exit(led_cleanup);

MODULE_DESCRIPTION("LED driver for BCM96358VW2");
MODULE_VERSION("0.1.0");
MODULE_AUTHOR("Zhifeng Gu <guzhifeng1979@hotmail.com>");
MODULE_LICENSE("GPL v2");
