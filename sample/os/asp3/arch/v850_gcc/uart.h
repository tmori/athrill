#ifndef _UART_H_
#define _UART_H_



/*
 * アシンクロナス・シリアル・インタフェース（ UARTD）
 */
#define UDnChannelNum			UINT_C(8)
#define UDnCH0					UINT_C(0)
#define UDnCH1					UINT_C(1)
#define UDnCH2					UINT_C(2)
#define UDnCH3					UINT_C(3)
#define UDnCH4					UINT_C(4)
#define UDnCH5					UINT_C(5)
#define UDnCH6					UINT_C(6)
#define UDnCH7					UINT_C(7)

/*
 * UARTDn制御レジスタ 0（ UDnCTL0）
 */
#define UDnCTL0_BASE			UINT_C(0xFFFFFA00)
#define UDnCTL0(CH)				(UDnCTL0_BASE + ((CH) * 16U))

/*
 * UARTDn制御レジスタ 1（ UDnCTL1）
 */
#define UDnCTL1_BASE			UINT_C(0xFFFFFA01)
#define UDnCTL1(CH)				(UDnCTL1_BASE + ((CH) * 16U))
/*
 * UARTDn制御レジスタ 2（ UDnCTL2）
 */
#define UDnCTL2_BASE			UINT_C(0xFFFFFA02)
#define UDnCTL2(CH)				(UDnCTL2_BASE + ((CH) * 16U))

/*
 * UARTDn オプション制御レジスタ 0（ UDnOPT0）
 */
#define UDnOPT0_BASE			UINT_C(0xFFFFFA03)
#define UDnOPT0(CH)				(UDnOPT0_BASE + ((CH) * 16U))

/*
 * UARTDn オプション制御レジスタ 1（ UDnOPT1）
 */
#define UDnOPT1_BASE			UINT_C(0xFFFFFA05)
#define UDnOPT1(CH)				(UDnOPT1_BASE + ((CH) * 16U))

/*
 * UARTDn状態レジスタ（ UDnSTR）
 */
#define UDnSTR_BASE				UINT_C(0xFFFFFA04)
#define UDnSTR(CH)				(UDnSTR_BASE + ((CH) * 16U))

/*
 * UARTDn送信データ・レジスタ（ UDnTX）
 */
#define UDnTX_BASE				UINT_C(0xFFFFFA07)
#define UDnTX(CH)				(UDnTX_BASE + ((CH) * 16U))

/*
 * UARTDn受信データ・レジスタ（ UDnRX）
 */
#define UDnRX_BASE				UINT_C(0xFFFFFA06)
#define UDnRX(CH)				(UDnRX_BASE + ((CH) * 16U))

/*
 *  UARTD0受信完了割込み番号
 */
#define INTNO_INTUD0R	UINT_C(36)
/*
 *  UARTD0送信完了割込み番号
 */
#define INTNO_INTUD0T	UINT_C(37)

#endif /* _UART_H_ */