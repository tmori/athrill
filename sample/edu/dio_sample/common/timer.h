#ifndef _TIMER_H_
#define _TIMER_H_

extern void timer_init(void (*callack) (void));
extern void timer_start(uint16 cycle_clock);
extern void timer_stop(void);

#endif /* _TIMER_H_ */