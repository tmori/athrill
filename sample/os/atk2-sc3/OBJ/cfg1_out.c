/* cfg1_out.c */
#define TOPPERS_CFG1_OUT  1
#include "kernel/kernel_int.h"
#include "sample2.h"
#include "sample1.h"
#include "target_serial.h"
#include "target_hw_counter.h"
#include "target_mem.h"


#ifdef INT64_MAX
  typedef sint64 signed_t;
  typedef uint64 unsigned_t;
#else
  typedef sint32 signed_t;
  typedef uint32 unsigned_t;
#endif

#include "target_cfg1_out.h"

const uint32 TOPPERS_cfg_magic_number = 0x12345678;
const uint32 TOPPERS_cfg_sizeof_signed_t = sizeof(signed_t);
const uint32 TOPPERS_cfg_sizeof_pointer = sizeof(const volatile void*);

const unsigned_t TOPPERS_cfg_TOPPERS_SUPPORT_ATT_MOD = 
#if defined(TOPPERS_SUPPORT_ATT_MOD)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_SIL_ENDIAN_BIG = 
#if defined(SIL_ENDIAN_BIG)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_SIL_ENDIAN_LITTLE = 
#if defined(SIL_ENDIAN_LITTLE)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TDOM_KERNEL = ( unsigned_t )TDOM_KERNEL;
const unsigned_t TOPPERS_cfg_TDOM_NONE = ( unsigned_t )TDOM_NONE;
const unsigned_t TOPPERS_cfg_TACP_KERNEL = ( unsigned_t )TACP_KERNEL;
const unsigned_t TOPPERS_cfg_TACP_SHARED = ( unsigned_t )TACP_SHARED;
const unsigned_t TOPPERS_cfg_TOPPERS_ATTSEC = ( unsigned_t )TOPPERS_ATTSEC;
const unsigned_t TOPPERS_cfg_TOPPERS_ATTMEM = ( unsigned_t )TOPPERS_ATTMEM;
const unsigned_t TOPPERS_cfg_TOPPERS_USTACK = ( unsigned_t )TOPPERS_USTACK;
const unsigned_t TOPPERS_cfg_OMIT_STANDARD_MEMINIB = 
#if defined(OMIT_STANDARD_MEMINIB)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_OMIT_IDATA = 
#if defined(OMIT_IDATA)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_OMIT_STANDARD_DATASECINIB = 
#if defined(OMIT_STANDARD_DATASECINIB)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_OMIT_STANDARD_BSSSECINIB = 
#if defined(OMIT_STANDARD_BSSSECINIB)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_REGATR = 
#if defined(TARGET_REGATR)
(TARGET_REGATR);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_MEMATR = 
#if defined(TARGET_MEMATR)
(TARGET_MEMATR);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_MIN_SSTKSZ = 
#if defined(TARGET_MIN_SSTKSZ)
(TARGET_MIN_SSTKSZ);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TARGET_MIN_USTKSZ = 
#if defined(TARGET_MIN_USTKSZ)
(TARGET_MIN_USTKSZ);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_STKSZ_ALIGN = 
#if defined(CHECK_STKSZ_ALIGN)
(CHECK_STKSZ_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_USTKSZ_ALIGN = 
#if defined(CHECK_USTKSZ_ALIGN)
(CHECK_USTKSZ_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_FUNC_ALIGN = 
#if defined(CHECK_FUNC_ALIGN)
(CHECK_FUNC_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_FUNC_NONNULL = 
#if defined(CHECK_FUNC_NONNULL)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_STACK_ALIGN = 
#if defined(CHECK_STACK_ALIGN)
(CHECK_STACK_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_STACK_NONNULL = 
#if defined(CHECK_STACK_NONNULL)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_USTACK_ALIGN = 
#if defined(CHECK_USTACK_ALIGN)
(CHECK_USTACK_ALIGN);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_CHECK_USTACK_NONNULL = 
#if defined(CHECK_USTACK_NONNULL)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_TOPPERS_LABEL_ASM = 
#if defined(TOPPERS_LABEL_ASM)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_sizeof_StackType = ( unsigned_t )sizeof(StackType);
const unsigned_t TOPPERS_cfg_sizeof_MemorySizeType = ( unsigned_t )sizeof(MemorySizeType);
const unsigned_t TOPPERS_cfg_sizeof_void_ptr = ( unsigned_t )sizeof(void*);
const unsigned_t TOPPERS_cfg_sizeof_FunctionRefType = ( unsigned_t )sizeof(FunctionRefType);
const unsigned_t TOPPERS_cfg_sizeof_TINIB = ( unsigned_t )sizeof(TINIB);
const unsigned_t TOPPERS_cfg_sizeof_HWCNTINIB = ( unsigned_t )sizeof(HWCNTINIB);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_task = ( unsigned_t )offsetof(TINIB,task);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_ustk = 
#if !defined(USE_TSKINICTXB)
(offsetof(TINIB,ustk));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_TINIB_sstksz = 
#if !defined(USE_TSKINICTXB)
(offsetof(TINIB,sstksz));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_TINIB_sstk = 
#if !defined(USE_TSKINICTXB)
(offsetof(TINIB,sstk));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_HWCNTINIB_init = ( unsigned_t )offsetof(HWCNTINIB,init);
const unsigned_t TOPPERS_cfg_sizeof_ALMINIB = ( unsigned_t )sizeof(ALMINIB);
const unsigned_t TOPPERS_cfg_offsetof_ALMINIB_action = ( unsigned_t )offsetof(ALMINIB,action);
const unsigned_t TOPPERS_cfg_sizeof_TFINIB = ( unsigned_t )sizeof(TFINIB);
const unsigned_t TOPPERS_cfg_sizeof_TrustedFunctionRefType = ( unsigned_t )sizeof(TrustedFunctionRefType);
const unsigned_t TOPPERS_cfg_TA_NULL = ( unsigned_t )TA_NULL;
const unsigned_t TOPPERS_cfg_TA_NOWRITE = ( unsigned_t )TA_NOWRITE;
const unsigned_t TOPPERS_cfg_TA_NOREAD = ( unsigned_t )TA_NOREAD;
const unsigned_t TOPPERS_cfg_TA_EXEC = ( unsigned_t )TA_EXEC;
const unsigned_t TOPPERS_cfg_TA_MEMINI = ( unsigned_t )TA_MEMINI;
const unsigned_t TOPPERS_cfg_TA_MEMPRSV = ( unsigned_t )TA_MEMPRSV;
const unsigned_t TOPPERS_cfg_TA_SDATA = ( unsigned_t )TA_SDATA;
const unsigned_t TOPPERS_cfg_TA_UNCACHE = ( unsigned_t )TA_UNCACHE;
const unsigned_t TOPPERS_cfg_TA_IODEV = ( unsigned_t )TA_IODEV;
const unsigned_t TOPPERS_cfg_ENABLE = ( unsigned_t )ENABLE;
const unsigned_t TOPPERS_cfg_DISABLE = ( unsigned_t )DISABLE;
const unsigned_t TOPPERS_cfg_AUTO = ( unsigned_t )AUTO;
const unsigned_t TOPPERS_cfg_TMIN_TPRI = ( unsigned_t )TMIN_TPRI;
const unsigned_t TOPPERS_cfg_TMAX_TPRI = ( unsigned_t )TMAX_TPRI;
const unsigned_t TOPPERS_cfg_TPRI_MAXTASK = ( unsigned_t )TPRI_MAXTASK;
const unsigned_t TOPPERS_cfg_TMAX_NTOSAPP = ( unsigned_t )TMAX_NTOSAPP;
const unsigned_t TOPPERS_cfg_OMIT_INITIALIZE_INTERRUPT = 
#if defined(OMIT_INITIALIZE_INTERRUPT)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_TSKINICTXB = 
#if defined(USE_TSKINICTXB)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_OMIT_CHECK_CYCLIC_CHAIN = 
#if defined(OMIT_CHECK_CYCLIC_CHAIN)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_MINIMUM_OSTKSZ = ( unsigned_t )MINIMUM_OSTKSZ;
const unsigned_t TOPPERS_cfg_DEFAULT_TASKSTKSZ = ( unsigned_t )DEFAULT_TASKSTKSZ;
const unsigned_t TOPPERS_cfg_DEFAULT_TASKSYSTEMSTKSZ = ( unsigned_t )DEFAULT_TASKSYSTEMSTKSZ;
const unsigned_t TOPPERS_cfg_DEFAULT_ISRSTKSZ = ( unsigned_t )DEFAULT_ISRSTKSZ;
const unsigned_t TOPPERS_cfg_DEFAULT_ISRSYSTEMSTKSZ = ( unsigned_t )DEFAULT_ISRSYSTEMSTKSZ;
const unsigned_t TOPPERS_cfg_DEFAULT_HOOKSTKSZ = ( unsigned_t )DEFAULT_HOOKSTKSZ;
const unsigned_t TOPPERS_cfg_DEFAULT_TRUSTEDFUNCTIONSTKSZ = ( unsigned_t )DEFAULT_TRUSTEDFUNCTIONSTKSZ;
const unsigned_t TOPPERS_cfg_DEFAULT_OSSTKSZ = ( unsigned_t )DEFAULT_OSSTKSZ;
const unsigned_t TOPPERS_cfg_OMIT_STKMPUINFOB = 
#if defined(OMIT_STKMPUINFOB)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_OMIT_OSAPMPUINFOB = 
#if defined(OMIT_OSAPMPUINFOB)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_TCB_p_tinib = ( unsigned_t )offsetof(TCB,p_tinib);
const unsigned_t TOPPERS_cfg_offsetof_TCB_curpri = ( unsigned_t )offsetof(TCB,curpri);
const unsigned_t TOPPERS_cfg_offsetof_TCB_ssp = ( unsigned_t )offsetof(TCB,tskctxb.ssp);
const unsigned_t TOPPERS_cfg_offsetof_TCB_usp = ( unsigned_t )offsetof(TCB,tskctxb.usp);
const unsigned_t TOPPERS_cfg_offsetof_TCB_priv_mode = ( unsigned_t )offsetof(TCB,tskctxb.priv_mode);
const unsigned_t TOPPERS_cfg_offsetof_TCB_pc = ( unsigned_t )offsetof(TCB,tskctxb.pc);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_exepri = ( unsigned_t )offsetof(TINIB,exepri);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_TSKINICTXB_sstksz = ( unsigned_t )offsetof(TINIB,tskinictxb.sstksz);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_TSKINICTXB_sstk_bottom = ( unsigned_t )offsetof(TINIB,tskinictxb.sstk_bottom);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_TSKINICTXB_stksz = ( unsigned_t )offsetof(TINIB,tskinictxb.stksz);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_TSKINICTXB_stk_bottom = ( unsigned_t )offsetof(TINIB,tskinictxb.stk_bottom);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_STKMPUINFOB_start_ustk = ( unsigned_t )offsetof(TINIB,stkmpu.start_ustk_label);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_STKMPUINFOB_limit_ustk = ( unsigned_t )offsetof(TINIB,stkmpu.limit_ustk_label);
const unsigned_t TOPPERS_cfg_offsetof_OSAPCB_p_osapinib = ( unsigned_t )offsetof(OSAPCB,p_osapinib);
const unsigned_t TOPPERS_cfg_offsetof_TINIB_p_osapcb = ( unsigned_t )offsetof(TINIB,p_osapcb);
const unsigned_t TOPPERS_cfg_offsetof_INTINIB_remain_stksz = ( unsigned_t )offsetof(INTINIB,remain_stksz);
const unsigned_t TOPPERS_cfg_offsetof_ISRINIB_p_intinib = ( unsigned_t )offsetof(ISRINIB,p_intinib);
const unsigned_t TOPPERS_cfg_offsetof_ISRINIB_p_osapcb = ( unsigned_t )offsetof(ISRINIB,p_osapcb);
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_osap_trusted = ( unsigned_t )offsetof(OSAPINIB,osap_trusted);
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_osapmpu = ( unsigned_t )offsetof(OSAPINIB,osap_mpu);
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_start_text = 
#if defined(USE_MPU_PR_TEXT)
(offsetof(OSAPINIB,osap_mpu.start_text_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_limit_text = 
#if defined(USE_MPU_PR_TEXT)
(offsetof(OSAPINIB,osap_mpu.limit_text_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_start_rosdata = 
#if defined(USE_MPU_PR_SDATA)
(offsetof(OSAPINIB,osap_mpu.start_rosdata_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_limit_rosdata = 
#if defined(USE_MPU_PR_SDATA)
(offsetof(OSAPINIB,osap_mpu.limit_rosdata_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_start_ram = 
#if defined(USE_MPU_PRW_DATA)
(offsetof(OSAPINIB,osap_mpu.start_ram_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_limit_ram = 
#if defined(USE_MPU_PRW_DATA)
(offsetof(OSAPINIB,osap_mpu.limit_ram_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_start_sram = 
#if defined(USE_MPU_PRW_SDATA)
(offsetof(OSAPINIB,osap_mpu.start_sram_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_limit_sram = 
#if defined(USE_MPU_PRW_SDATA)
(offsetof(OSAPINIB,osap_mpu.limit_sram_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_start_srpw = 
#if defined(USE_MPU_SRPW_DATA)
(offsetof(OSAPINIB,osap_mpu.start_srpw_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_limit_srpw = 
#if defined(USE_MPU_SRPW_DATA)
(offsetof(OSAPINIB,osap_mpu.limit_srpw_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_start_ssrpw = 
#if defined(USE_MPU_SRPW_SDATA)
(offsetof(OSAPINIB,osap_mpu.start_ssrpw_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_limit_ssrpw = 
#if defined(USE_MPU_SRPW_SDATA)
(offsetof(OSAPINIB,osap_mpu.limit_ssrpw_label));
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_mpu_area_info = ( unsigned_t )offsetof(OSAPINIB,osap_mpu.mpu_area_info);
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_tnum_mpu_area = ( unsigned_t )offsetof(OSAPINIB,osap_mpu.tnum_mpu_area);
const unsigned_t TOPPERS_cfg_offsetof_OSAPINIB_mprc = ( unsigned_t )offsetof(OSAPINIB,osap_mpu.mprc);
const unsigned_t TOPPERS_cfg_sizeof_OSAPINIB = ( unsigned_t )sizeof(OSAPINIB);
const unsigned_t TOPPERS_cfg_offsetof_ISRCB_p_isrinib = ( unsigned_t )offsetof(ISRCB,p_isrinib);
const unsigned_t TOPPERS_cfg_TNUM_INTPRI = ( unsigned_t )TNUM_INTPRI;
const unsigned_t TOPPERS_cfg_TNUM_INT = ( unsigned_t )TNUM_INT;
const unsigned_t TOPPERS_cfg___v850e2v3__ = 
#if defined(__v850e2v3__)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg___v850e3v5__ = 
#if defined(__v850e3v5__) 
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_E_OS_PROTECTION_MEMORY = ( unsigned_t )E_OS_PROTECTION_MEMORY;
const unsigned_t TOPPERS_cfg_E_OS_PROTECTION_EXCEPTION = ( unsigned_t )E_OS_PROTECTION_EXCEPTION;
const unsigned_t TOPPERS_cfg_TNUM_MPU_REG = 
#if defined(TNUM_MPU_REG)
(TNUM_MPU_REG);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_SRPW_DATA = 
#if defined(USE_MPU_SRPW_DATA)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_SRPW_SDATA = 
#if defined(USE_MPU_SRPW_SDATA)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_PR_TEXT = 
#if defined(USE_MPU_PR_TEXT)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_PR_SDATA = 
#if defined(USE_MPU_PR_SDATA)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_PRW_DATA = 
#if defined(USE_MPU_PRW_DATA)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_PRW_SDATA = 
#if defined(USE_MPU_PRW_SDATA)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_SRW_DATA_SDATA = 
#if defined(USE_MPU_SRW_DATA_SDATA)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_SR_TEXT = 
#if defined(USE_MPU_SR_TEXT)
(1);
#else
(0);
#endif
const unsigned_t TOPPERS_cfg_USE_MPU_SR_DATA_SDATA = 
#if defined(USE_MPU_SR_DATA_SDATA)
(1);
#else
(0);
#endif
/* #include "sample2.h" */
/* #include "sample1.h" */
/* #include "target_serial.h" */
/* #include "target_hw_counter.h" */
/* #include "target_mem.h" */
const unsigned_t TOPPERS_cfg_valueof_ISR_INTNO_C2ISR_for_MAIN_HW_COUNTER_Os = ( HWC_DTIM_INTNO ); 
const unsigned_t TOPPERS_cfg_valueof_ISR_INTPRI_C2ISR_for_MAIN_HW_COUNTER_Os = ( HWC_DTIM_INTPRI ); 
const unsigned_t TOPPERS_cfg_valueof_REG_SIZE_INTROM_PE1_Os = ( 4*1024*1024 ); 
const unsigned_t TOPPERS_cfg_valueof_REG_SIZE_INTROM_PE2_Os = ( 2*1024*1024 ); 
const unsigned_t TOPPERS_cfg_valueof_REG_BASE_INTRAM_Os = ( 4273799168 ); 
const unsigned_t TOPPERS_cfg_valueof_REG_SIZE_INTRAM_Os = ( 192*1024 ); 
const unsigned_t TOPPERS_cfg_valueof_REG_SIZE_INTRAM2_Os = ( 192*1024 ); 
const unsigned_t TOPPERS_cfg_valueof_ISR_INTNO_RxHwSerialInt_Os = ( INTNO_SIO ); 
const unsigned_t TOPPERS_cfg_valueof_ISR_INTPRI_RxHwSerialInt_Os = ( INTPRI_SIO ); 

