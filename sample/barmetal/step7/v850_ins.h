#ifndef _V850_INS_H_
#define _V850_INS_H_


static inline void disable_int_all(void)
{
	asm("	di");
}

static inline void enable_int_all(void)
{
	asm("	ei");
}

static inline void do_halt(void)
{
	asm("	halt");
}

#endif /* _V850_INS_H_ */