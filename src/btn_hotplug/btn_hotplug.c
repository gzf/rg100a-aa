#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <bcm_intr.h>
#include "btn_hotplug.h"

#define NUM_OF_EXTIRQ   6

static int ext_irqs[NUM_OF_EXTIRQ] = {
    INTERRUPT_ID_EXTERNAL_0,
    INTERRUPT_ID_EXTERNAL_1,
    INTERRUPT_ID_EXTERNAL_2,
    INTERRUPT_ID_EXTERNAL_3,
    INTERRUPT_ID_EXTERNAL_4,
    INTERRUPT_ID_EXTERNAL_5
};

static int general_isr(int irq, void* dev_id) {
    printk("irq=%x, dev_id=%p\n", irq, dev_id);
    return IRQ_HANDLED;
}

void __init ext_intr_init(void) {
    int i;
    for (i=0; i<NUM_OF_EXTIRQ; i++) {
        BcmHalMapInterrupt(general_isr, 0, ext_irqs[i]);
        BcmHalInterruptEnable(ext_irqs[i]);
    }
}

void __exit ext_intr_cleanup(void) {
    int i;
    for (i=0; i<NUM_OF_EXTIRQ; i++)
        BcmHalInterruptDisable(ext_irqs[i]);
}


module_init(ext_intr_init);
module_exit(ext_intr_cleanup);

MODULE_DESCRIPTION("Button hotplug driver for BCM96358VW2 board");
MODULE_VERSION("0.1.0");
MODULE_AUTHOR("Zhifeng Gu <guzhifeng1979@hotmail.com>");
MODULE_LICENSE("GPL v2");
