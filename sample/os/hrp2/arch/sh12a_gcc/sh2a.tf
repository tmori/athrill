$ 
$		コア依存テンプレート（SH2A用）
$ 

$ 
$  NMIの割込み番号
$ 
$TINTNO_NMI = 11$

$ 
$  割込み属性中のターゲット依存に用いるビット
$ 
$TARGET_INTATR = TA_POSEDGE | TA_NEGEDGE | TA_LOWLEVEL$

$ 
$  CFG_INTで設定できる割込み属性のリスト
$ 	　・備考
$ 	　　　TA_ENAINTの有無は省略
$ 

$ 
$ IRQ割込みで使用できる割込み属性
$ 		・ポジティブエッジ
$ 		・ネガティブエッジ
$ 		・両ティブエッジ
$ 		・ローレベル
$ 
$valid_irq_intatr_list = {
	0,
	TA_POSEDGE,
	TA_NEGEDGE,
	TA_BOTHEDGE,
	TA_EDGE,
	TA_EDGE | TA_POSEDGE,
	TA_EDGE | TA_NEGEDGE,
	TA_EDGE | TA_BOTHEDGE,
	TA_LOWLEVEL
}$

$ 
$ NMIで使用できる割込み属性
$ 		・ポジティブエッジ
$ 		・ネガティブエッジ
$ 
$valid_nmi_intatr_list = {
	TA_POSEDGE,
	TA_NEGEDGE,
	TA_EDGE,
	TA_EDGE | TA_POSEDGE,
	TA_EDGE | TA_NEGEDGE
}$

$ 
$  プロセッサ依存のテンプレートファイルのインクルード
$ 
$INCLUDE "sh12a_gcc/prc.tf"$
