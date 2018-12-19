#ifndef _SERIAL_H_
#define _SERIAL_H_

extern void serial_init(void);
extern int serial_get(char *cp);
extern void serial_put(char c);

#endif /* _SERIAL_H_ */