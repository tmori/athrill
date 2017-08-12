#include "concrete_executor/target/dbg_target_cpu.h"
#include "cpu.h"
#include "symbol_ops.h"

#include <stdio.h>


static void print_register(const char* regname, uint32 addr, char* opt)
{
	 uint32 funcaddr;
	 int funcid;
	 uint32 gladdr;
	 int glid;

	 funcid = symbol_pc2funcid(addr, &funcaddr);
	 if (funcid >= 0) {
		printf("%s		0x%x %s(+0x%x)", regname, addr, symbol_funcid2funcname(funcid), addr - funcaddr);
	 }
	 else {
		glid = symbol_addr2glid(addr, &gladdr);
		if (glid >= 0) {
			printf("%s		0x%x %s(+0x%x)", regname, addr, symbol_glid2glname(glid), addr - gladdr);
		}
		else {
			printf("%s		0x%x", regname, addr);
		}
	 }
	 if (opt != NULL) {
		 printf(" %s\n", opt);
	 }
	 else {
		 printf("\n");
	 }
	return;
}

void dbg_target_print_cpu(void)
{
	int i;
	char buffer[128];
	uint32 pc = virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.pc;

	 print_register("PC", pc, NULL);
	 for (i = 0; i < 32; i++) {
		 char *opt = NULL;
		 sprintf(buffer, "R%d", i);
		 switch (i) {
		 case 3:
			 opt = "Stack Pointer";
			 break;
		 case 10:
			 opt = "Return Value";
			 break;
		 case 6:
			 opt = "Arg1";
			 break;
		 case 7:
			 opt = "Arg2";
			 break;
		 case 8:
			 opt = "Arg3";
			 break;
		 case 9:
			 opt = "Arg4";
			 break;
		 default:
			 break;
		 }
		 print_register(buffer, virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.r[i], opt);
	 }
	 print_register("EIPC", virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.eipc, NULL);
	 printf("EIPSW		0x%x\n", virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.eipsw);
	 printf("ECR		0x%x\n", virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.ecr);
	 printf("PSW		0x%x\n", virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.psw);
	 print_register("EIPC", virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.fepc, NULL);
	 printf("FEPSW 		0x%x\n", virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.fepsw);
	 printf("CTBP		0x%x\n", virtual_cpu.cores[CPU_CONFIG_CORE_ID_0].core.reg.ctbp);

	return;
}
