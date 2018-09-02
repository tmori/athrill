#ifndef _ROSLIB_H_
#define _ROSLIB_H_

extern void roslib_init(void);
extern void roslib_publish(int busid, int elmid, unsigned char *can_datap);
extern int roslib_subscribe(int busid, int elmid, unsigned char *can_datap);

#endif /* _ROSLIB_H_ */