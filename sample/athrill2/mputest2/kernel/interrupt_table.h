#ifndef _INTERRUPT_TABLE_H_
#define _INTERRUPT_TABLE_H_

#define INTERRUPT_TABLE_SIZE    30U

extern void register_interrupt_handler(unsigned int intrno, void (*handler) (void));
extern void do_interrupt_handler(unsigned int intrno);

#endif /* _INTERRUPT_TABLE_H_ */