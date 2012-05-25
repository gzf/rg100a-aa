#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <bcm_intr.h>
#include "btn_hotplug.h"

static int ext_irqs[NUM_OF_EXTIRQ] = {
    INTERRUPT_ID_EXTERNAL_0,
    INTERRUPT_ID_EXTERNAL_1,
    INTERRUPT_ID_EXTERNAL_2,
    INTERRUPT_ID_EXTERNAL_3
//    INTERRUPT_ID_EXTERNAL_4,
//    INTERRUPT_ID_EXTERNAL_5
};

static char btn_names[NUM_OF_EXTIRQ][7];

static u64 begin[NUM_OF_EXTIRQ];
static u64 last_seen[NUM_OF_EXTIRQ];
static struct timer_list irq_timers[NUM_OF_EXTIRQ];
static struct timer_list rel_timers[NUM_OF_EXTIRQ];

//const unsigned long event_interval = HZ;

int button_hotplug_create_event(char *name, unsigned long seen, int pressed);

static void irq_release(unsigned long index) {
    button_hotplug_create_event(btn_names[index], ((unsigned long) (get_jiffies_64() - begin[index])) / HZ, 0);
    begin[index] = 0;
}

static void enable_ext_irq(unsigned long index) {

    rel_timers[index].expires = jiffies + HZ / 100;
    rel_timers[index].function = irq_release;
    rel_timers[index].data = index;
    add_timer(&rel_timers[index]);

    BcmHalInterruptEnable(ext_irqs[index]);
}

static int general_isr(int irq, void* dev_id) {
    unsigned long i, seen;
    u64 now = get_jiffies_64();

    for (i=0; i<NUM_OF_EXTIRQ; i++)
        if (irq == ext_irqs[i])
            break;
    if (i == NUM_OF_EXTIRQ)
        return IRQ_NONE;

    if (begin[i] == 0) {
        button_hotplug_create_event(btn_names[i], 0, 1);
        begin[i] = now;
        last_seen[i] = 0;
    }
    else {
        seen = ((unsigned long) (now - begin[i])) / HZ;
        if (seen > last_seen[i]) {
            button_hotplug_create_event(btn_names[i], seen, 1);
            last_seen[i] = seen;
        }
        del_timer(&rel_timers[i]); // delete the irq_release handler
    }
    
    irq_timers[i].expires = jiffies + HZ / 100;
    irq_timers[i].function = enable_ext_irq;
    irq_timers[i].data = i;
    add_timer(&irq_timers[i]);

    return IRQ_HANDLED;
}

static int __init ext_intr_init(void) {
    int i;
    for (i=0; i<NUM_OF_EXTIRQ; i++) {
        init_timer(&irq_timers[i]);
        init_timer(&rel_timers[i]);
        begin[i] = 0;
        sprintf(btn_names[i], "BTN_%02d", ext_irqs[i]);
        BcmHalMapInterrupt(general_isr, 0, ext_irqs[i]);
        BcmHalInterruptEnable(ext_irqs[i]);
    }
    printk("BCM96358 button driver loaded\n");
    return 0;
}

static void __exit ext_intr_cleanup(void) {
    int i;
    for (i=0; i<NUM_OF_EXTIRQ; i++) {
        BcmHalInterruptDisable(ext_irqs[i]);
        del_timer_sync(&irq_timers[i]);
        del_timer_sync(&rel_timers[i]);
    }
}

module_init(ext_intr_init);
module_exit(ext_intr_cleanup);

MODULE_DESCRIPTION("Button hotplug driver for BCM96358VW2");
MODULE_VERSION("0.1.0");
MODULE_AUTHOR("Zhifeng Gu <guzhifeng1979@hotmail.com>");
MODULE_LICENSE("GPL v2");
