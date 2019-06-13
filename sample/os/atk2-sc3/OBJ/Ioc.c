#include "Ioc.h"


/* IOC_QUE API */
Std_ReturnType
IocSend_IOC_QUE_1(uint8 in1)
{
	Std_ReturnType	ercd;
	IOCMB_IOC_QUE	in;
	in.data1 = in1;
	ercd = ioc_send_generic(IOC_WRAPPER_1, (void *)(&in));
	return(ercd);
}

Std_ReturnType
IocSend_IOC_QUE_0(uint8 in1)
{
	Std_ReturnType	ercd;
	IOCMB_IOC_QUE	in;
	in.data1 = in1;
	ercd = ioc_send_generic(IOC_WRAPPER_2, (void *)(&in));
	return(ercd);
}

Std_ReturnType
IocReceive_IOC_QUE(uint8 *out1)
{
	Std_ReturnType	ercd;
	IOCMB_IOC_QUE	out;
	ercd = ioc_receive_generic(IOC_QUE, (void *)(&out));
	if ((ercd == IOC_E_OK) || (ercd == IOC_E_LOST_DATA)) {
		*out1 = out.data1;
	}
	return(ercd);
}

Std_ReturnType
IocEmptyQueue_IOC_QUE(void)
{
	Std_ReturnType	ercd;
	ercd = ioc_empty_queue_generic(IOC_QUE);
	return(ercd);
}


/* IOC_DEQUE API */
Std_ReturnType
IocWriteGroup_IOC_DEQUE(uint8 in1, uint8 in2, uint8 in3)
{
	Std_ReturnType	ercd;
	IOCMB_IOC_DEQUE	in;
	in.data1 = in1;
	in.data2 = in2;
	in.data3 = in3;
	ercd = ioc_write_generic(IOC_WRAPPER_0, (void *)(&in));
	return(ercd);
}

Std_ReturnType
IocReadGroup_IOC_DEQUE(uint8 *out1, uint8 *out2, uint8 *out3)
{
	Std_ReturnType	ercd;
	IOCMB_IOC_DEQUE	out;
	ercd = ioc_read_generic(IOC_DEQUE, (void *)(&out));
	if (ercd == IOC_E_OK) {
		*out1 = out.data1;
		*out2 = out.data2;
		*out3 = out.data3;
	}
	return(ercd);
}

