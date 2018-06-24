/*  This file is generated from svc.def by gensvc. */

#ifndef TOPPERS_SVC_CALL_H
#define TOPPERS_SVC_CALL_H

#ifndef TOPPERS_MACRO_ONLY

extern ER _kernel_act_tsk(ID tskid) throw();
extern ER _kernel_iact_tsk(ID tskid) throw();
extern ER_UINT _kernel_can_act(ID tskid) throw();
extern ER _kernel_ext_tsk(void) throw();
extern ER _kernel_ter_tsk(ID tskid) throw();
extern ER _kernel_chg_pri(ID tskid, PRI tskpri) throw();
extern ER _kernel_get_pri(ID tskid, PRI *p_tskpri) throw();
extern ER _kernel_get_inf(intptr_t *p_exinf) throw();
extern ER _kernel_slp_tsk(void) throw();
extern ER _kernel_tslp_tsk(TMO tmout) throw();
extern ER _kernel_wup_tsk(ID tskid) throw();
extern ER _kernel_iwup_tsk(ID tskid) throw();
extern ER_UINT _kernel_can_wup(ID tskid) throw();
extern ER _kernel_rel_wai(ID tskid) throw();
extern ER _kernel_irel_wai(ID tskid) throw();
extern ER _kernel_dis_wai(ID tskid) throw();
extern ER _kernel_idis_wai(ID tskid) throw();
extern ER _kernel_ena_wai(ID tskid) throw();
extern ER _kernel_iena_wai(ID tskid) throw();
extern ER _kernel_sus_tsk(ID tskid) throw();
extern ER _kernel_rsm_tsk(ID tskid) throw();
extern ER _kernel_dly_tsk(RELTIM dlytim) throw();
extern ER _kernel_ras_tex(ID tskid, TEXPTN rasptn) throw();
extern ER _kernel_iras_tex(ID tskid, TEXPTN rasptn) throw();
extern ER _kernel_dis_tex(void) throw();
extern ER _kernel_ena_tex(void) throw();
extern bool_t _kernel_sns_tex(void) throw();
extern ER _kernel_ref_tex(ID tskid, T_RTEX *pk_rtex) throw();
extern ER _kernel_sig_sem(ID semid) throw();
extern ER _kernel_isig_sem(ID semid) throw();
extern ER _kernel_wai_sem(ID semid) throw();
extern ER _kernel_pol_sem(ID semid) throw();
extern ER _kernel_twai_sem(ID semid, TMO tmout) throw();
extern ER _kernel_set_flg(ID flgid, FLGPTN setptn) throw();
extern ER _kernel_iset_flg(ID flgid, FLGPTN setptn) throw();
extern ER _kernel_clr_flg(ID flgid, FLGPTN clrptn) throw();
extern ER _kernel_wai_flg(ID flgid, FLGPTN waiptn, MODE wfmode, FLGPTN *p_flgptn) throw();
extern ER _kernel_pol_flg(ID flgid, FLGPTN waiptn, MODE wfmode, FLGPTN *p_flgptn) throw();
extern ER _kernel_twai_flg(ID flgid, FLGPTN waiptn, MODE wfmode, FLGPTN *p_flgptn, TMO tmout) throw();
extern ER _kernel_snd_dtq(ID dtqid, intptr_t data) throw();
extern ER _kernel_psnd_dtq(ID dtqid, intptr_t data) throw();
extern ER _kernel_ipsnd_dtq(ID dtqid, intptr_t data) throw();
extern ER _kernel_tsnd_dtq(ID dtqid, intptr_t data, TMO tmout) throw();
extern ER _kernel_fsnd_dtq(ID dtqid, intptr_t data) throw();
extern ER _kernel_ifsnd_dtq(ID dtqid, intptr_t data) throw();
extern ER _kernel_rcv_dtq(ID dtqid, intptr_t *p_data) throw();
extern ER _kernel_prcv_dtq(ID dtqid, intptr_t *p_data) throw();
extern ER _kernel_trcv_dtq(ID dtqid, intptr_t *p_data, TMO tmout) throw();
extern ER _kernel_snd_pdq(ID pdqid, intptr_t data, PRI datapri) throw();
extern ER _kernel_psnd_pdq(ID pdqid, intptr_t data, PRI datapri) throw();
extern ER _kernel_ipsnd_pdq(ID pdqid, intptr_t data, PRI datapri) throw();
extern ER _kernel_tsnd_pdq(ID pdqid, intptr_t data, PRI datapri, TMO tmout) throw();
extern ER _kernel_rcv_pdq(ID pdqid, intptr_t *p_data, PRI *p_datapri) throw();
extern ER _kernel_prcv_pdq(ID pdqid, intptr_t *p_data, PRI *p_datapri) throw();
extern ER _kernel_trcv_pdq(ID pdqid, intptr_t *p_data, PRI *p_datapri, TMO tmout) throw();
extern ER _kernel_loc_mtx(ID mtxid) throw();
extern ER _kernel_ploc_mtx(ID mtxid) throw();
extern ER _kernel_tloc_mtx(ID mtxid, TMO tmout) throw();
extern ER _kernel_unl_mtx(ID mtxid) throw();
extern ER _kernel_get_mpf(ID mpfid, void **p_blk) throw();
extern ER _kernel_pget_mpf(ID mpfid, void **p_blk) throw();
extern ER _kernel_tget_mpf(ID mpfid, void **p_blk, TMO tmout) throw();
extern ER _kernel_rel_mpf(ID mpfid, void *blk) throw();
extern ER _kernel_get_tim(SYSTIM *p_systim) throw();
extern ER _kernel_get_utm(SYSUTM *p_sysutm) throw();
extern ER _kernel_ref_ovr(ID tskid, T_ROVR *pk_rovr) throw();
extern ER _kernel_sta_cyc(ID cycid) throw();
extern ER _kernel_stp_cyc(ID cycid) throw();
extern ER _kernel_sta_alm(ID almid, RELTIM almtim) throw();
extern ER _kernel_ista_alm(ID almid, RELTIM almtim) throw();
extern ER _kernel_stp_alm(ID almid) throw();
extern ER _kernel_istp_alm(ID almid) throw();
extern ER _kernel_sta_ovr(ID tskid, OVRTIM ovrtim) throw();
extern ER _kernel_ista_ovr(ID tskid, OVRTIM ovrtim) throw();
extern ER _kernel_stp_ovr(ID tskid) throw();
extern ER _kernel_istp_ovr(ID tskid) throw();
extern ER _kernel_rot_rdq(PRI tskpri) throw();
extern ER _kernel_irot_rdq(PRI tskpri) throw();
extern ER _kernel_get_did(ID *p_domid) throw();
extern ER _kernel_get_tid(ID *p_tskid) throw();
extern ER _kernel_iget_tid(ID *p_tskid) throw();
extern ER _kernel_loc_cpu(void) throw();
extern ER _kernel_iloc_cpu(void) throw();
extern ER _kernel_unl_cpu(void) throw();
extern ER _kernel_iunl_cpu(void) throw();
extern ER _kernel_dis_dsp(void) throw();
extern ER _kernel_ena_dsp(void) throw();
extern bool_t _kernel_sns_ctx(void) throw();
extern bool_t _kernel_sns_loc(void) throw();
extern bool_t _kernel_sns_dsp(void) throw();
extern bool_t _kernel_sns_dpn(void) throw();
extern bool_t _kernel_sns_ker(void) throw();
extern ER _kernel_ext_ker(void) throw();
extern ER _kernel_prb_mem(const void *base, SIZE size, ID tskid, MODE pmmode) throw();
extern ER _kernel_dis_int(INTNO intno) throw();
extern ER _kernel_ena_int(INTNO intno) throw();
extern ER _kernel_chg_ipm(PRI intpri) throw();
extern ER _kernel_get_ipm(PRI *p_intpri) throw();
extern bool_t _kernel_xsns_dpn(void *p_excinf) throw();
extern bool_t _kernel_xsns_xpn(void *p_excinf) throw();
extern ER _kernel_ini_sem(ID semid) throw();
extern ER _kernel_ini_flg(ID flgid) throw();
extern ER _kernel_ini_dtq(ID dtqid) throw();
extern ER _kernel_ini_pdq(ID pdqid) throw();
extern ER _kernel_ini_mtx(ID mtxid) throw();
extern ER _kernel_ini_mpf(ID mpfid) throw();
extern ER _kernel_ref_tsk(ID tskid, T_RTSK *pk_rtsk) throw();
extern ER _kernel_ref_sem(ID semid, T_RSEM *pk_rsem) throw();
extern ER _kernel_ref_flg(ID flgid, T_RFLG *pk_rflg) throw();
extern ER _kernel_ref_dtq(ID dtqid, T_RDTQ *pk_rdtq) throw();
extern ER _kernel_ref_pdq(ID pdqid, T_RPDQ *pk_rpdq) throw();
extern ER _kernel_ref_mtx(ID mtxid, T_RMTX *pk_rmtx) throw();
extern ER _kernel_ref_mpf(ID mpfid, T_RMPF *pk_rmpf) throw();
extern ER _kernel_ref_cyc(ID cycid, T_RCYC *pk_rcyc) throw();
extern ER _kernel_ref_alm(ID almid, T_RALM *pk_ralm) throw();

#endif /* TOPPERS_MACRO_ONLY */

#ifdef TOPPERS_SVC_CALL

#define act_tsk _kernel_act_tsk
#define iact_tsk _kernel_iact_tsk
#define can_act _kernel_can_act
#define ext_tsk _kernel_ext_tsk
#define ter_tsk _kernel_ter_tsk
#define chg_pri _kernel_chg_pri
#define get_pri _kernel_get_pri
#define get_inf _kernel_get_inf
#define slp_tsk _kernel_slp_tsk
#define tslp_tsk _kernel_tslp_tsk
#define wup_tsk _kernel_wup_tsk
#define iwup_tsk _kernel_iwup_tsk
#define can_wup _kernel_can_wup
#define rel_wai _kernel_rel_wai
#define irel_wai _kernel_irel_wai
#define dis_wai _kernel_dis_wai
#define idis_wai _kernel_idis_wai
#define ena_wai _kernel_ena_wai
#define iena_wai _kernel_iena_wai
#define sus_tsk _kernel_sus_tsk
#define rsm_tsk _kernel_rsm_tsk
#define dly_tsk _kernel_dly_tsk
#define ras_tex _kernel_ras_tex
#define iras_tex _kernel_iras_tex
#define dis_tex _kernel_dis_tex
#define ena_tex _kernel_ena_tex
#define sns_tex _kernel_sns_tex
#define ref_tex _kernel_ref_tex
#define sig_sem _kernel_sig_sem
#define isig_sem _kernel_isig_sem
#define wai_sem _kernel_wai_sem
#define pol_sem _kernel_pol_sem
#define twai_sem _kernel_twai_sem
#define set_flg _kernel_set_flg
#define iset_flg _kernel_iset_flg
#define clr_flg _kernel_clr_flg
#define wai_flg _kernel_wai_flg
#define pol_flg _kernel_pol_flg
#define twai_flg _kernel_twai_flg
#define snd_dtq _kernel_snd_dtq
#define psnd_dtq _kernel_psnd_dtq
#define ipsnd_dtq _kernel_ipsnd_dtq
#define tsnd_dtq _kernel_tsnd_dtq
#define fsnd_dtq _kernel_fsnd_dtq
#define ifsnd_dtq _kernel_ifsnd_dtq
#define rcv_dtq _kernel_rcv_dtq
#define prcv_dtq _kernel_prcv_dtq
#define trcv_dtq _kernel_trcv_dtq
#define snd_pdq _kernel_snd_pdq
#define psnd_pdq _kernel_psnd_pdq
#define ipsnd_pdq _kernel_ipsnd_pdq
#define tsnd_pdq _kernel_tsnd_pdq
#define rcv_pdq _kernel_rcv_pdq
#define prcv_pdq _kernel_prcv_pdq
#define trcv_pdq _kernel_trcv_pdq
#define loc_mtx _kernel_loc_mtx
#define ploc_mtx _kernel_ploc_mtx
#define tloc_mtx _kernel_tloc_mtx
#define unl_mtx _kernel_unl_mtx
#define get_mpf _kernel_get_mpf
#define pget_mpf _kernel_pget_mpf
#define tget_mpf _kernel_tget_mpf
#define rel_mpf _kernel_rel_mpf
#define get_tim _kernel_get_tim
#define get_utm _kernel_get_utm
#define ref_ovr _kernel_ref_ovr
#define sta_cyc _kernel_sta_cyc
#define stp_cyc _kernel_stp_cyc
#define sta_alm _kernel_sta_alm
#define ista_alm _kernel_ista_alm
#define stp_alm _kernel_stp_alm
#define istp_alm _kernel_istp_alm
#define sta_ovr _kernel_sta_ovr
#define ista_ovr _kernel_ista_ovr
#define stp_ovr _kernel_stp_ovr
#define istp_ovr _kernel_istp_ovr
#define rot_rdq _kernel_rot_rdq
#define irot_rdq _kernel_irot_rdq
#define get_did _kernel_get_did
#define get_tid _kernel_get_tid
#define iget_tid _kernel_iget_tid
#define loc_cpu _kernel_loc_cpu
#define iloc_cpu _kernel_iloc_cpu
#define unl_cpu _kernel_unl_cpu
#define iunl_cpu _kernel_iunl_cpu
#define dis_dsp _kernel_dis_dsp
#define ena_dsp _kernel_ena_dsp
#define sns_ctx _kernel_sns_ctx
#define sns_loc _kernel_sns_loc
#define sns_dsp _kernel_sns_dsp
#define sns_dpn _kernel_sns_dpn
#define sns_ker _kernel_sns_ker
#define ext_ker _kernel_ext_ker
#define prb_mem _kernel_prb_mem
#define dis_int _kernel_dis_int
#define ena_int _kernel_ena_int
#define chg_ipm _kernel_chg_ipm
#define get_ipm _kernel_get_ipm
#define xsns_dpn _kernel_xsns_dpn
#define xsns_xpn _kernel_xsns_xpn
#define ini_sem _kernel_ini_sem
#define ini_flg _kernel_ini_flg
#define ini_dtq _kernel_ini_dtq
#define ini_pdq _kernel_ini_pdq
#define ini_mtx _kernel_ini_mtx
#define ini_mpf _kernel_ini_mpf
#define ref_tsk _kernel_ref_tsk
#define ref_sem _kernel_ref_sem
#define ref_flg _kernel_ref_flg
#define ref_dtq _kernel_ref_dtq
#define ref_pdq _kernel_ref_pdq
#define ref_mtx _kernel_ref_mtx
#define ref_mpf _kernel_ref_mpf
#define ref_cyc _kernel_ref_cyc
#define ref_alm _kernel_ref_alm

#ifdef TOPPERS_LABEL_ASM

#define _act_tsk __kernel_act_tsk
#define _iact_tsk __kernel_iact_tsk
#define _can_act __kernel_can_act
#define _ext_tsk __kernel_ext_tsk
#define _ter_tsk __kernel_ter_tsk
#define _chg_pri __kernel_chg_pri
#define _get_pri __kernel_get_pri
#define _get_inf __kernel_get_inf
#define _slp_tsk __kernel_slp_tsk
#define _tslp_tsk __kernel_tslp_tsk
#define _wup_tsk __kernel_wup_tsk
#define _iwup_tsk __kernel_iwup_tsk
#define _can_wup __kernel_can_wup
#define _rel_wai __kernel_rel_wai
#define _irel_wai __kernel_irel_wai
#define _dis_wai __kernel_dis_wai
#define _idis_wai __kernel_idis_wai
#define _ena_wai __kernel_ena_wai
#define _iena_wai __kernel_iena_wai
#define _sus_tsk __kernel_sus_tsk
#define _rsm_tsk __kernel_rsm_tsk
#define _dly_tsk __kernel_dly_tsk
#define _ras_tex __kernel_ras_tex
#define _iras_tex __kernel_iras_tex
#define _dis_tex __kernel_dis_tex
#define _ena_tex __kernel_ena_tex
#define _sns_tex __kernel_sns_tex
#define _ref_tex __kernel_ref_tex
#define _sig_sem __kernel_sig_sem
#define _isig_sem __kernel_isig_sem
#define _wai_sem __kernel_wai_sem
#define _pol_sem __kernel_pol_sem
#define _twai_sem __kernel_twai_sem
#define _set_flg __kernel_set_flg
#define _iset_flg __kernel_iset_flg
#define _clr_flg __kernel_clr_flg
#define _wai_flg __kernel_wai_flg
#define _pol_flg __kernel_pol_flg
#define _twai_flg __kernel_twai_flg
#define _snd_dtq __kernel_snd_dtq
#define _psnd_dtq __kernel_psnd_dtq
#define _ipsnd_dtq __kernel_ipsnd_dtq
#define _tsnd_dtq __kernel_tsnd_dtq
#define _fsnd_dtq __kernel_fsnd_dtq
#define _ifsnd_dtq __kernel_ifsnd_dtq
#define _rcv_dtq __kernel_rcv_dtq
#define _prcv_dtq __kernel_prcv_dtq
#define _trcv_dtq __kernel_trcv_dtq
#define _snd_pdq __kernel_snd_pdq
#define _psnd_pdq __kernel_psnd_pdq
#define _ipsnd_pdq __kernel_ipsnd_pdq
#define _tsnd_pdq __kernel_tsnd_pdq
#define _rcv_pdq __kernel_rcv_pdq
#define _prcv_pdq __kernel_prcv_pdq
#define _trcv_pdq __kernel_trcv_pdq
#define _loc_mtx __kernel_loc_mtx
#define _ploc_mtx __kernel_ploc_mtx
#define _tloc_mtx __kernel_tloc_mtx
#define _unl_mtx __kernel_unl_mtx
#define _get_mpf __kernel_get_mpf
#define _pget_mpf __kernel_pget_mpf
#define _tget_mpf __kernel_tget_mpf
#define _rel_mpf __kernel_rel_mpf
#define _get_tim __kernel_get_tim
#define _get_utm __kernel_get_utm
#define _ref_ovr __kernel_ref_ovr
#define _sta_cyc __kernel_sta_cyc
#define _stp_cyc __kernel_stp_cyc
#define _sta_alm __kernel_sta_alm
#define _ista_alm __kernel_ista_alm
#define _stp_alm __kernel_stp_alm
#define _istp_alm __kernel_istp_alm
#define _sta_ovr __kernel_sta_ovr
#define _ista_ovr __kernel_ista_ovr
#define _stp_ovr __kernel_stp_ovr
#define _istp_ovr __kernel_istp_ovr
#define _rot_rdq __kernel_rot_rdq
#define _irot_rdq __kernel_irot_rdq
#define _get_did __kernel_get_did
#define _get_tid __kernel_get_tid
#define _iget_tid __kernel_iget_tid
#define _loc_cpu __kernel_loc_cpu
#define _iloc_cpu __kernel_iloc_cpu
#define _unl_cpu __kernel_unl_cpu
#define _iunl_cpu __kernel_iunl_cpu
#define _dis_dsp __kernel_dis_dsp
#define _ena_dsp __kernel_ena_dsp
#define _sns_ctx __kernel_sns_ctx
#define _sns_loc __kernel_sns_loc
#define _sns_dsp __kernel_sns_dsp
#define _sns_dpn __kernel_sns_dpn
#define _sns_ker __kernel_sns_ker
#define _ext_ker __kernel_ext_ker
#define _prb_mem __kernel_prb_mem
#define _dis_int __kernel_dis_int
#define _ena_int __kernel_ena_int
#define _chg_ipm __kernel_chg_ipm
#define _get_ipm __kernel_get_ipm
#define _xsns_dpn __kernel_xsns_dpn
#define _xsns_xpn __kernel_xsns_xpn
#define _ini_sem __kernel_ini_sem
#define _ini_flg __kernel_ini_flg
#define _ini_dtq __kernel_ini_dtq
#define _ini_pdq __kernel_ini_pdq
#define _ini_mtx __kernel_ini_mtx
#define _ini_mpf __kernel_ini_mpf
#define _ref_tsk __kernel_ref_tsk
#define _ref_sem __kernel_ref_sem
#define _ref_flg __kernel_ref_flg
#define _ref_dtq __kernel_ref_dtq
#define _ref_pdq __kernel_ref_pdq
#define _ref_mtx __kernel_ref_mtx
#define _ref_mpf __kernel_ref_mpf
#define _ref_cyc __kernel_ref_cyc
#define _ref_alm __kernel_ref_alm

#endif /* TOPPERS_LABEL_ASM */
#endif /* TOPPERS_SVC_CALL */
#endif /* TOPPERS_SVC_CALL_H */
