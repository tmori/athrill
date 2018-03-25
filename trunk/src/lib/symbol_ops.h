#ifndef _SYMBOL_OPS_H_
#define _SYMBOL_OPS_H_

#include "std_types.h"
#include "cpu_config.h"

typedef struct {
	char *name;
	uint32 addr;
	uint32 size;
	uint32 enter_sp[CPU_CONFIG_CORE_NUM];
} DbgSymbolType;

extern uint32 symbol_get_func_num(void);
extern uint32 symbol_get_gl_num(void);

extern int symbol_get_func(char *funcname, uint32 func_len, uint32 *addrp, uint32 *size);
extern char * symbol_pc2func(uint32 pc);
extern int symbol_pc2funcid(uint32 pc, uint32 *funcaddr);
extern char * symbol_funcid2funcname(int id);
extern uint32 symbol_funcid2funcaddr(int id);
extern int symbol_get_gl(char *gl_name, uint32 gl_len, uint32 *addrp, uint32 *size);

extern int symbol_addr2glid(uint32 addr, uint32 *gladdr);
extern char * symbol_glid2glname(int id);
extern uint32 symbol_glid2gladdr(int id);

extern void symbol_print_gl(char *gl_name, uint32 show_num);
extern void symbol_print_func(char *gl_name, uint32 show_num);


extern int symbol_gl_add(DbgSymbolType *sym);
extern int symbol_func_add(DbgSymbolType *sym);
extern uint32 symbol_funcid2funcsize(int id);


extern uint32 symbol_get_entered_sp(int funcid, uint32 core_id);
extern void symbol_set_pc(int funcid, uint32 core_id, uint32 sp);


#endif /* _SYMBOL_OPS_H_ */
