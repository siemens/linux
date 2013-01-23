/*
 * GPIOs on MPC8349/8572/8610 and compatible
 *
 * Copyright (C) 2008 Peter Korsgaard <jacmet@sunsite.dk>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#define MPC8XXX_GPIO_PINS	32

#define GPIO_DIR		0x00
#define GPIO_ODR		0x04
#define GPIO_DAT		0x08
#define GPIO_IER		0x0c
#define GPIO_IMR		0x10
#define GPIO_ICR		0x14

struct mpc8xxx_gpio_chip {
	struct of_mm_gpio_chip mm_gc;
#ifdef CONFIG_RTX_DOMAIN
	raw_spinlock_t lock;
#else
	spinlock_t lock;
#endif
	int irq_nr;

	/*
	 * shadowed data register to be able to clear/set output pins in
	 * open drain mode safely
	 */
	u32 data;
};

static inline u32 mpc8xxx_gpio2mask(unsigned int gpio)
{
	return 1u << (MPC8XXX_GPIO_PINS - 1 - gpio);
}

static inline struct mpc8xxx_gpio_chip *
to_mpc8xxx_gpio_chip(struct of_mm_gpio_chip *mm)
{
	return container_of(mm, struct mpc8xxx_gpio_chip, mm_gc);
}

static void mpc8xxx_gpio_save_regs(struct of_mm_gpio_chip *mm)
{
	struct mpc8xxx_gpio_chip *mpc8xxx_gc = to_mpc8xxx_gpio_chip(mm);

	mpc8xxx_gc->data = in_be32(mm->regs + GPIO_DAT);
}

static int mpc8xxx_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct of_mm_gpio_chip *mm = to_of_mm_gpio_chip(gc);
    
	return in_be32(mm->regs + GPIO_DAT) & mpc8xxx_gpio2mask(gpio);
}

static void mpc8xxx_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct of_mm_gpio_chip *mm = to_of_mm_gpio_chip(gc);
	struct mpc8xxx_gpio_chip *mpc8xxx_gc = to_mpc8xxx_gpio_chip(mm);
	unsigned long flags;

#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_lock_irqsave(&mpc8xxx_gc->lock, flags);
#else
	spin_lock_irqsave(&mpc8xxx_gc->lock, flags);
#endif
	if (val)
		mpc8xxx_gc->data |= mpc8xxx_gpio2mask(gpio);
	else
		mpc8xxx_gc->data &= ~mpc8xxx_gpio2mask(gpio);

	out_be32(mm->regs + GPIO_DAT, mpc8xxx_gc->data);

#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_unlock_irqrestore(&mpc8xxx_gc->lock, flags);
#else
	spin_unlock_irqrestore(&mpc8xxx_gc->lock, flags);
#endif
}

static int mpc8xxx_gpio_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
	struct of_mm_gpio_chip *mm = to_of_mm_gpio_chip(gc);
	struct mpc8xxx_gpio_chip *mpc8xxx_gc = to_mpc8xxx_gpio_chip(mm);
	unsigned long flags;

#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_lock_irqsave(&mpc8xxx_gc->lock, flags);
#else
	spin_lock_irqsave(&mpc8xxx_gc->lock, flags);
#endif

	clrbits32(mm->regs + GPIO_DIR, mpc8xxx_gpio2mask(gpio));

#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_unlock_irqrestore(&mpc8xxx_gc->lock, flags);
#else
	spin_unlock_irqrestore(&mpc8xxx_gc->lock, flags);
#endif

	return 0;
}

static int mpc8xxx_gpio_dir_out(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct of_mm_gpio_chip *mm = to_of_mm_gpio_chip(gc);
	struct mpc8xxx_gpio_chip *mpc8xxx_gc = to_mpc8xxx_gpio_chip(mm);
	unsigned long flags;

	mpc8xxx_gpio_set(gc, gpio, val);

#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_lock_irqsave(&mpc8xxx_gc->lock, flags);
#else
	spin_lock_irqsave(&mpc8xxx_gc->lock, flags);
#endif

	setbits32(mm->regs + GPIO_DIR, mpc8xxx_gpio2mask(gpio));

#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_unlock_irqrestore(&mpc8xxx_gc->lock, flags);
#else
	spin_unlock_irqrestore(&mpc8xxx_gc->lock, flags);
#endif

	return 0;
}

static int mpc8xxx_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm = to_of_mm_gpio_chip(gc);
	struct mpc8xxx_gpio_chip *mpc8xxx_gc = to_mpc8xxx_gpio_chip(mm);
	unsigned long flags;
        
#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_lock_irqsave(&mpc8xxx_gc->lock, flags);
#else
	spin_lock_irqsave(&mpc8xxx_gc->lock, flags);
#endif

        /* clear possible interrupts, enable mask register */
	out_be32(mm->regs + GPIO_IER, mpc8xxx_gpio2mask(offset));
	setbits32(mm->regs + GPIO_IMR, mpc8xxx_gpio2mask(offset));
        
#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_unlock_irqrestore(&mpc8xxx_gc->lock, flags);
#else
	spin_unlock_irqrestore(&mpc8xxx_gc->lock, flags);
#endif

	return mpc8xxx_gc->irq_nr;
}

/*
 * we introduce a new subroutine to detect whether an
 * interrupt event is set for this gpio bit
 * manfred.neugebauer@siemens.com
 */  
static int mpc8xxx_gpio_get_event(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm = to_of_mm_gpio_chip(gc);
	//struct mpc8xxx_gpio_chip *mpc8xxx_gc = to_mpc8xxx_gpio_chip(mm);
	//unsigned long flags;
        int evtState;
        
	evtState = in_be32(mm->regs + GPIO_IER) & mpc8xxx_gpio2mask(offset);
        if (evtState) /* clear this event bit */
		out_be32(mm->regs + GPIO_IER, mpc8xxx_gpio2mask(offset));
        
	return evtState;
}

static void __init mpc8xxx_add_controller(struct device_node *np)
{
	struct mpc8xxx_gpio_chip *mpc8xxx_gc;
	struct of_mm_gpio_chip *mm_gc;
	struct of_gpio_chip *of_gc;
	struct gpio_chip *gc;
	int ret;

	mpc8xxx_gc = kzalloc(sizeof(*mpc8xxx_gc), GFP_KERNEL);
	if (!mpc8xxx_gc) {
		ret = -ENOMEM;
		goto err;
	}

#ifdef CONFIG_RTX_DOMAIN
	rtx_spin_lock_init(&mpc8xxx_gc->lock);
#else
	spin_lock_init(&mpc8xxx_gc->lock);
#endif

	mm_gc = &mpc8xxx_gc->mm_gc;
	of_gc = &mm_gc->of_gc;
	gc = &of_gc->gc;

	mm_gc->save_regs = mpc8xxx_gpio_save_regs;
	of_gc->gpio_cells = 2;
	gc->ngpio = MPC8XXX_GPIO_PINS;
	gc->direction_input = mpc8xxx_gpio_dir_in;
	gc->direction_output = mpc8xxx_gpio_dir_out;
	gc->get = mpc8xxx_gpio_get;
	gc->set = mpc8xxx_gpio_set;
        gc->to_irq = mpc8xxx_gpio_to_irq;
        gc->get_event =  mpc8xxx_gpio_get_event;

        mpc8xxx_gc->irq_nr = irq_of_parse_and_map(np, 0);

	ret = of_mm_gpiochip_add(np, mm_gc);
	if (ret)
		goto err;

        /* clear mask register to prevent interrupts */
        /* clear all possible interrupts */
	out_be32(mm_gc->regs + GPIO_IMR, 0x0);
	out_be32(mm_gc->regs + GPIO_IER, 0xffffffff);
        
	return;

err:
	pr_err("%s: registration failed with status %d\n",
	       np->full_name, ret);
	kfree(mpc8xxx_gc);

	return;
}

static int __init mpc8xxx_add_gpiochips(void)
{
	struct device_node *np;

	for_each_compatible_node(np, NULL, "fsl,mpc8349-gpio")
		mpc8xxx_add_controller(np);

	for_each_compatible_node(np, NULL, "fsl,mpc8572-gpio")
		mpc8xxx_add_controller(np);

	for_each_compatible_node(np, NULL, "fsl,mpc8610-gpio")
		mpc8xxx_add_controller(np);

	return 0;
}
arch_initcall(mpc8xxx_add_gpiochips);
