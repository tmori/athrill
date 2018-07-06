	/*
	 * ラベルロード Lea (Load Effective Address)
	 */
	 
	.macro	Lea name, destination
	movhi	hi(\name), r0, \destination
	movea	lo(\name), \destination, \destination
	.endm

