#ifndef _INTC_H_
#define _INTC_H_

#include "cpu.h"



#define INTC_FECC_NMI				0x10
#define INTC_FECC_INTWDT2			0x20
#define INTC_NMINO_NMI			0
#define INTC_NMINO_INTWDT2		1
extern int intc_raise_nmi(TargetCoreType *cpu, uint32 nmino);


/*
 * 割込み数
 * パディングが１個あるため，実際には116個であることに注意※．
 * ※割込み番号53までは，連続しているが，
 * ※54番目はパディングされているため，１個ずれている．
 */
#define INTC_NUM			117
#define INTC_INTNO_MAX		116
#define INTC_PINTNO_NOUSE	54

/*
 * 5.3.4 割込み制御レジスタ(xxICn)
 */
#define INTC_REG_ICN_BASE	0xFFFFF110
#define INTC_ICN_ISSET_IF(data)		( (((data) & (1 << 7)) == (1 << 7)) )
#define INTC_ICN_SET_IF(data)		( ((data) |=   (1 << 7)) )
#define INTC_ICN_CLR_IF(data)		( ((data) &=  ~(1 << 7)) )

#define INTC_ICN_ISSET_MK(data)		( (((data) & (1 << 6)) == (1 << 6)) )
#define INTC_ICN_SET_MK(data)		( ((data) |=  (1 << 6)) )
#define INTC_ICN_CLR_MK(data)		( ((data) &=  ~(1 << 6)) )

#define INTC_ICN_PR(data)			( ((data) & 0x07) )

static inline uint32 intc_intno2off(uint32 intno)
{
	uint32 off;
#if 0
	if (intno <= 53) {
		off = intno;
	}
	else {
		off = intno + 1;
	}
#else
	off = intno;
#endif
	return off;
}
static inline int intc_pintno2intno(int pintno)
{
#if 0
	if (pintno <= 53) {
		return pintno;
	}
	else {
		return pintno - 1;
	}
#else
	return pintno;
#endif
}
static inline uint32 intc_regaddr_icn(uint32 intno)
{
	uint32 regaddr;

	regaddr = INTC_REG_ICN_BASE + (intc_intno2off(intno) * 2);

	return regaddr;
}

/*
 * 5.1 ディフォールト・プライオリティ
 */
#define INTC_DEFAULT_PRIORITY(intno)		intc_intno2off((intno))

/*
 * 5.1 例外コード
 */
#define INTC_MASK_ECR_CODE_BASE				0x0080
#define INTC_MASK_ECR_CODE(intno)			( INTC_MASK_ECR_CODE_BASE + (intc_intno2off((intno)) *16) )

/*
 * 5.1 ハンドラ・アドレス
 */
#define INTC_MASK_INTR_ADDR(intno)			INTC_MASK_ECR_CODE((intno))

/*
 * 5.3．5 割込みマスク・レジスタ(IMR0-IMR7)
 */
#define INTC_REG_IMR0	0xFFFFF100
#define INTC_REG_IMR0L	0xFFFFF100
#define INTC_REG_IMR0H	0xFFFFF101

#define INTC_REG_IMR1	0xFFFFF102
#define INTC_REG_IMR1L	0xFFFFF102
#define INTC_REG_IMR1H	0xFFFFF103

#define INTC_REG_IMR2	0xFFFFF104
#define INTC_REG_IMR2L	0xFFFFF104
#define INTC_REG_IMR2H	0xFFFFF105

#define INTC_REG_IMR3	0xFFFFF106
#define INTC_REG_IMR3L	0xFFFFF106
#define INTC_REG_IMR3H	0xFFFFF107

#define INTC_REG_IMR4	0xFFFFF108
#define INTC_REG_IMR4L	0xFFFFF108
#define INTC_REG_IMR4H	0xFFFFF109

#define INTC_REG_IMR5	0xFFFFF10A
#define INTC_REG_IMR5L	0xFFFFF10A
#define INTC_REG_IMR5H	0xFFFFF10B

#define INTC_REG_IMR6	0xFFFFF10C
#define INTC_REG_IMR6L	0xFFFFF10C
#define INTC_REG_IMR6H	0xFFFFF10D

#define INTC_REG_IMR7	0xFFFFF10E

#define INTC_REG_xxICnStr	0xFFFFF110
#define INTC_REG_xxICnEnd	0xFFFFF1F8

#define MASK_INTC_ADDR(addr) ((addr) & 0x03FFFFFF)

/*
 * 5.3．6 インライン・プライオリティ・レジスタ(ISPR)
 */
#define INTC_REG_ISPR	0xFFFFF1FA
/*
 * 割込みコントローラ管理情報
 */
#define INTC_NUM_INTLVL		8
#define INTC_MAX_INTLVL		0
#define INTC_MIN_INTLVL		7

typedef struct {
	bool 	intwdt2_hasreq;
	int		nmi_reqnum;
} NmiIntcStatusType;

typedef struct {
	CpuType					*cpu;
	NmiIntcStatusType 		nmi;
	/*
	 * 現在実行中の割り込み番号
	 */
	int current_intno;

	int saved_intno_off;
	int saved_intno_stack[INTC_NUM_INTLVL];

	/*
	 * 割込み待機中の割り込みがある場合はその優先度が設定されている
	 * INTC_NUM_INTLVLの場合は待機なしと判断する．
	 */
	uint8	is_waiting_lvl[INTC_NUM];
	uint32	waiting_lvl_num[INTC_NUM];
	uint32  waiting_int_num;
} IntcControlType;

extern IntcControlType intc_control;

#endif /* _INTC_H_ */
