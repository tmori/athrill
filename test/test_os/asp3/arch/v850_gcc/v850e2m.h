#ifndef _V850E2M_H_
#define _V850E2M_H_

/*
 *  微少時間待ちのための定義（本来はSILのターゲット依存部）
 */
/*
 * 100MHz
 * 10 nsec per clock
 */
#define SIL_NSEC_PER_CLOCK	10
#define SIL_DLY_TIM1    (9 * SIL_NSEC_PER_CLOCK)
#define SIL_DLY_TIM2    (2 * SIL_NSEC_PER_CLOCK)

#endif /* _V850E2M_H_ */