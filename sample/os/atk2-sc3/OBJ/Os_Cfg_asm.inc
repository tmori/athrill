.macro DISPATCHER_MPU_SETTING reg1, reg2, reg3, reg4, reg5, reg6
	ld.w   OSAPINIB_start_srpw[AMARG(reg1)], AMARG(reg2)
	ld.w   OSAPINIB_limit_srpw[AMARG(reg1)], AMARG(reg3)
	ldsr   AMARG(reg2), 4, 6
	ldsr   AMARG(reg3), 5, 6
	ld.w   OSAPINIB_start_ssrpw[AMARG(reg1)], AMARG(reg2)
	ld.w   OSAPINIB_limit_ssrpw[AMARG(reg1)], AMARG(reg3)
	ldsr   AMARG(reg2), 8, 6
	ldsr   AMARG(reg3), 9, 6
	ld.w   OSAPINIB_start_text[AMARG(reg1)], AMARG(reg2)
	ld.w   OSAPINIB_limit_text[AMARG(reg1)], AMARG(reg3)
	ldsr   AMARG(reg2), 12, 6
	ldsr   AMARG(reg3), 13, 6
	ld.w   OSAPINIB_start_rosdata[AMARG(reg1)], AMARG(reg2)
	ld.w   OSAPINIB_limit_rosdata[AMARG(reg1)], AMARG(reg3)
	ldsr   AMARG(reg2), 16, 6
	ldsr   AMARG(reg3), 17, 6
	ld.w   OSAPINIB_start_ram[AMARG(reg1)], AMARG(reg2)
	ld.w   OSAPINIB_limit_ram[AMARG(reg1)], AMARG(reg3)
	ldsr   AMARG(reg2), 20, 6
	ldsr   AMARG(reg3), 21, 6
	ld.w   OSAPINIB_start_sram[AMARG(reg1)], AMARG(reg2)
	ld.w   OSAPINIB_limit_sram[AMARG(reg1)], AMARG(reg3)
	ldsr   AMARG(reg2), 24, 6
	ldsr   AMARG(reg3), 25, 6
	ld.w   OSAPINIB_mprc[AMARG(reg1)], AMARG(reg2)
	ldsr   AMARG(reg2), 1, 5
.endm

