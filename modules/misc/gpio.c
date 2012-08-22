#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <bcm_map_part.h>
#include <board.h>
#include <boardparms.h>
#include <linux/gpio_dev.h>

static const int MAX_GPIO_PIN = 39;

static int gpio_open(struct inode *inode, struct file *filp) {
    return 0;
}

static int gpio_release(struct inode *inode, struct file *filp) {
    return 0;
}

static int gpio_ioctl(struct inode *inode, struct file * filep, unsigned int cmd, unsigned long gpio) {    
    volatile unsigned long *gpio_dir_reg = &GPIO->GPIODir;
    volatile unsigned long *gpio_io_reg  = &GPIO->GPIOio;
    unsigned long mask;

    if (gpio > MAX_GPIO_PIN)
        return -ENODEV;
    
    if (gpio < 32) {
        mask = GPIO_NUM_TO_MASK(gpio);
    }
    else {
        mask = GPIO_NUM_TO_MASK_HIGH(gpio);
        gpio_dir_reg = &GPIO->GPIODir_high;
        gpio_io_reg  = &GPIO->GPIOio_high;
    }

    switch (cmd) {
    case GPIO_CLEAR:
        *gpio_io_reg &= ~mask;
        break;
    case GPIO_SET:
        *gpio_io_reg |= mask;
        break;
    case GPIO_GET:
        return (*gpio_io_reg &= mask) ? 1 : 0;
    case GPIO_DIR_IN:
        *gpio_dir_reg &= ~mask;
        break;
    case GPIO_DIR_OUT:
        *gpio_dir_reg |= mask;
        break;
    default:
        return -EBADRQC;
    }
    return 0;
}

static struct file_operations gpio_fops = {
	.owner	 = THIS_MODULE,
	.open	 = gpio_open,
	.release = gpio_release,
    .ioctl   = gpio_ioctl,
};

static struct miscdevice gpio_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gpio",
    .fops = &gpio_fops,    
};

static int __init gpio_init(void) {
    int result;
    result = misc_register(&gpio_dev);

	if(result)
		printk(KERN_INFO "fail to create device\n");
    else
		printk(KERN_INFO "gpio device created successfully, minor=%d\n", gpio_dev.minor);
    
    return result;
}

static void __exit gpio_cleanup(void) {
    if (gpio_dev.minor != MISC_DYNAMIC_MINOR)
        misc_deregister(&gpio_dev);
}

module_init(gpio_init);
module_exit(gpio_cleanup);

MODULE_DESCRIPTION("GPIO driver for BCM96358VW2");
MODULE_VERSION("0.1.0");
MODULE_AUTHOR("Zhifeng Gu <guzhifeng1979@hotmail.com>");
MODULE_LICENSE("GPL v2");
