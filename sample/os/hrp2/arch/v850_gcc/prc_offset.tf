$ 
$ 		���ե��åȥե����������ѥƥ�ץ졼�ȥե������SH12A�ѡ�
$ 

$ 
$  ɸ��ƥ�ץ졼�ȥե�����Υ��󥯥롼��
$ 
$INCLUDE "kernel/genoffset.tf"$

$ 
$  ���ե��å��ͤΥޥ������������
$ 
$DEFINE("TCB_p_tinib", offsetof_TCB_p_tinib)$
$DEFINE("TCB_texptn", offsetof_TCB_texptn)$
$DEFINE("TCB_usp", offsetof_TCB_usp)$
$DEFINE("TCB_ssp", offsetof_TCB_ssp)$
$DEFINE("TCB_pc", offsetof_TCB_pc)$

$DEFINE("TINIB_exinf", offsetof_TINIB_exinf)$
$DEFINE("TINIB_task", offsetof_TINIB_task)$

$DEFINE("TINIB_TSKINICTXB_stksz", offsetof_TINIB_TSKINICTXB_stksz)$
$DEFINE("TINIB_TSKINICTXB_stk_bottom", offsetof_TINIB_TSKINICTXB_stk_bottom)$

$ 
$  �ӥåȥ��ե��å������Υޥ������������
$ 
$DEFINE_BIT("TCB_enatex", sizeof_TCB, "B")$