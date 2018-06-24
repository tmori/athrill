/*  This file is generated from svc.def by gensvc. */

#ifndef TOPPERS_TOOL_SVC_H
#define TOPPERS_TOOL_SVC_H

#ifndef TOPPERS_MACRO_ONLY

Inline ER
act_tsk(ID tskid)
{
	CAL_SVC_1M(ER, TFN_ACT_TSK, ID, tskid);
}

Inline ER
iact_tsk(ID tskid)
{
	CAL_SVC_1M(ER, TFN_IACT_TSK, ID, tskid);
}

Inline ER_UINT
can_act(ID tskid)
{
	CAL_SVC_1M(ER_UINT, TFN_CAN_ACT, ID, tskid);
}

Inline ER
ext_tsk(void)
{
	CAL_SVC_0M(ER, TFN_EXT_TSK);
}

Inline ER
ter_tsk(ID tskid)
{
	CAL_SVC_1M(ER, TFN_TER_TSK, ID, tskid);
}

Inline ER
chg_pri(ID tskid, PRI tskpri)
{
	CAL_SVC_2M(ER, TFN_CHG_PRI, ID, tskid, PRI, tskpri);
}

Inline ER
get_pri(ID tskid, PRI *p_tskpri)
{
	CAL_SVC_2M(ER, TFN_GET_PRI, ID, tskid, PRI *, p_tskpri);
}

Inline ER
get_inf(intptr_t *p_exinf)
{
	CAL_SVC_1M(ER, TFN_GET_INF, intptr_t *, p_exinf);
}

Inline ER
slp_tsk(void)
{
	CAL_SVC_0M(ER, TFN_SLP_TSK);
}

Inline ER
tslp_tsk(TMO tmout)
{
	CAL_SVC_1M(ER, TFN_TSLP_TSK, TMO, tmout);
}

Inline ER
wup_tsk(ID tskid)
{
	CAL_SVC_1M(ER, TFN_WUP_TSK, ID, tskid);
}

Inline ER
iwup_tsk(ID tskid)
{
	CAL_SVC_1M(ER, TFN_IWUP_TSK, ID, tskid);
}

Inline ER_UINT
can_wup(ID tskid)
{
	CAL_SVC_1M(ER_UINT, TFN_CAN_WUP, ID, tskid);
}

Inline ER
rel_wai(ID tskid)
{
	CAL_SVC_1M(ER, TFN_REL_WAI, ID, tskid);
}

Inline ER
irel_wai(ID tskid)
{
	CAL_SVC_1M(ER, TFN_IREL_WAI, ID, tskid);
}

Inline ER
dis_wai(ID tskid)
{
	CAL_SVC_1M(ER, TFN_DIS_WAI, ID, tskid);
}

Inline ER
idis_wai(ID tskid)
{
	CAL_SVC_1M(ER, TFN_IDIS_WAI, ID, tskid);
}

Inline ER
ena_wai(ID tskid)
{
	CAL_SVC_1M(ER, TFN_ENA_WAI, ID, tskid);
}

Inline ER
iena_wai(ID tskid)
{
	CAL_SVC_1M(ER, TFN_IENA_WAI, ID, tskid);
}

Inline ER
sus_tsk(ID tskid)
{
	CAL_SVC_1M(ER, TFN_SUS_TSK, ID, tskid);
}

Inline ER
rsm_tsk(ID tskid)
{
	CAL_SVC_1M(ER, TFN_RSM_TSK, ID, tskid);
}

Inline ER
dly_tsk(RELTIM dlytim)
{
	CAL_SVC_1M(ER, TFN_DLY_TSK, RELTIM, dlytim);
}

Inline ER
ras_tex(ID tskid, TEXPTN rasptn)
{
	CAL_SVC_2M(ER, TFN_RAS_TEX, ID, tskid, TEXPTN, rasptn);
}

Inline ER
iras_tex(ID tskid, TEXPTN rasptn)
{
	CAL_SVC_2M(ER, TFN_IRAS_TEX, ID, tskid, TEXPTN, rasptn);
}

Inline ER
dis_tex(void)
{
	CAL_SVC_0M(ER, TFN_DIS_TEX);
}

Inline ER
ena_tex(void)
{
	CAL_SVC_0M(ER, TFN_ENA_TEX);
}

Inline bool_t
sns_tex(void)
{
	CAL_SVC_0M(bool_t, TFN_SNS_TEX);
}

Inline ER
ref_tex(ID tskid, T_RTEX *pk_rtex)
{
	CAL_SVC_2M(ER, TFN_REF_TEX, ID, tskid, T_RTEX *, pk_rtex);
}

Inline ER
sig_sem(ID semid)
{
	CAL_SVC_1M(ER, TFN_SIG_SEM, ID, semid);
}

Inline ER
isig_sem(ID semid)
{
	CAL_SVC_1M(ER, TFN_ISIG_SEM, ID, semid);
}

Inline ER
wai_sem(ID semid)
{
	CAL_SVC_1M(ER, TFN_WAI_SEM, ID, semid);
}

Inline ER
pol_sem(ID semid)
{
	CAL_SVC_1M(ER, TFN_POL_SEM, ID, semid);
}

Inline ER
twai_sem(ID semid, TMO tmout)
{
	CAL_SVC_2M(ER, TFN_TWAI_SEM, ID, semid, TMO, tmout);
}

Inline ER
set_flg(ID flgid, FLGPTN setptn)
{
	CAL_SVC_2M(ER, TFN_SET_FLG, ID, flgid, FLGPTN, setptn);
}

Inline ER
iset_flg(ID flgid, FLGPTN setptn)
{
	CAL_SVC_2M(ER, TFN_ISET_FLG, ID, flgid, FLGPTN, setptn);
}

Inline ER
clr_flg(ID flgid, FLGPTN clrptn)
{
	CAL_SVC_2M(ER, TFN_CLR_FLG, ID, flgid, FLGPTN, clrptn);
}

Inline ER
wai_flg(ID flgid, FLGPTN waiptn, MODE wfmode, FLGPTN *p_flgptn)
{
	CAL_SVC_4M(ER, TFN_WAI_FLG, ID, flgid, FLGPTN, waiptn, MODE, wfmode, FLGPTN *, p_flgptn);
}

Inline ER
pol_flg(ID flgid, FLGPTN waiptn, MODE wfmode, FLGPTN *p_flgptn)
{
	CAL_SVC_4M(ER, TFN_POL_FLG, ID, flgid, FLGPTN, waiptn, MODE, wfmode, FLGPTN *, p_flgptn);
}

Inline ER
twai_flg(ID flgid, FLGPTN waiptn, MODE wfmode, FLGPTN *p_flgptn, TMO tmout)
{
	CAL_SVC_5M(ER, TFN_TWAI_FLG, ID, flgid, FLGPTN, waiptn, MODE, wfmode, FLGPTN *, p_flgptn, TMO, tmout);
}

Inline ER
snd_dtq(ID dtqid, intptr_t data)
{
	CAL_SVC_2M(ER, TFN_SND_DTQ, ID, dtqid, intptr_t, data);
}

Inline ER
psnd_dtq(ID dtqid, intptr_t data)
{
	CAL_SVC_2M(ER, TFN_PSND_DTQ, ID, dtqid, intptr_t, data);
}

Inline ER
ipsnd_dtq(ID dtqid, intptr_t data)
{
	CAL_SVC_2M(ER, TFN_IPSND_DTQ, ID, dtqid, intptr_t, data);
}

Inline ER
tsnd_dtq(ID dtqid, intptr_t data, TMO tmout)
{
	CAL_SVC_3M(ER, TFN_TSND_DTQ, ID, dtqid, intptr_t, data, TMO, tmout);
}

Inline ER
fsnd_dtq(ID dtqid, intptr_t data)
{
	CAL_SVC_2M(ER, TFN_FSND_DTQ, ID, dtqid, intptr_t, data);
}

Inline ER
ifsnd_dtq(ID dtqid, intptr_t data)
{
	CAL_SVC_2M(ER, TFN_IFSND_DTQ, ID, dtqid, intptr_t, data);
}

Inline ER
rcv_dtq(ID dtqid, intptr_t *p_data)
{
	CAL_SVC_2M(ER, TFN_RCV_DTQ, ID, dtqid, intptr_t *, p_data);
}

Inline ER
prcv_dtq(ID dtqid, intptr_t *p_data)
{
	CAL_SVC_2M(ER, TFN_PRCV_DTQ, ID, dtqid, intptr_t *, p_data);
}

Inline ER
trcv_dtq(ID dtqid, intptr_t *p_data, TMO tmout)
{
	CAL_SVC_3M(ER, TFN_TRCV_DTQ, ID, dtqid, intptr_t *, p_data, TMO, tmout);
}

Inline ER
snd_pdq(ID pdqid, intptr_t data, PRI datapri)
{
	CAL_SVC_3M(ER, TFN_SND_PDQ, ID, pdqid, intptr_t, data, PRI, datapri);
}

Inline ER
psnd_pdq(ID pdqid, intptr_t data, PRI datapri)
{
	CAL_SVC_3M(ER, TFN_PSND_PDQ, ID, pdqid, intptr_t, data, PRI, datapri);
}

Inline ER
ipsnd_pdq(ID pdqid, intptr_t data, PRI datapri)
{
	CAL_SVC_3M(ER, TFN_IPSND_PDQ, ID, pdqid, intptr_t, data, PRI, datapri);
}

Inline ER
tsnd_pdq(ID pdqid, intptr_t data, PRI datapri, TMO tmout)
{
	CAL_SVC_4M(ER, TFN_TSND_PDQ, ID, pdqid, intptr_t, data, PRI, datapri, TMO, tmout);
}

Inline ER
rcv_pdq(ID pdqid, intptr_t *p_data, PRI *p_datapri)
{
	CAL_SVC_3M(ER, TFN_RCV_PDQ, ID, pdqid, intptr_t *, p_data, PRI *, p_datapri);
}

Inline ER
prcv_pdq(ID pdqid, intptr_t *p_data, PRI *p_datapri)
{
	CAL_SVC_3M(ER, TFN_PRCV_PDQ, ID, pdqid, intptr_t *, p_data, PRI *, p_datapri);
}

Inline ER
trcv_pdq(ID pdqid, intptr_t *p_data, PRI *p_datapri, TMO tmout)
{
	CAL_SVC_4M(ER, TFN_TRCV_PDQ, ID, pdqid, intptr_t *, p_data, PRI *, p_datapri, TMO, tmout);
}

Inline ER
loc_mtx(ID mtxid)
{
	CAL_SVC_1M(ER, TFN_LOC_MTX, ID, mtxid);
}

Inline ER
ploc_mtx(ID mtxid)
{
	CAL_SVC_1M(ER, TFN_PLOC_MTX, ID, mtxid);
}

Inline ER
tloc_mtx(ID mtxid, TMO tmout)
{
	CAL_SVC_2M(ER, TFN_TLOC_MTX, ID, mtxid, TMO, tmout);
}

Inline ER
unl_mtx(ID mtxid)
{
	CAL_SVC_1M(ER, TFN_UNL_MTX, ID, mtxid);
}

Inline ER
get_mpf(ID mpfid, void **p_blk)
{
	CAL_SVC_2M(ER, TFN_GET_MPF, ID, mpfid, void **, p_blk);
}

Inline ER
pget_mpf(ID mpfid, void **p_blk)
{
	CAL_SVC_2M(ER, TFN_PGET_MPF, ID, mpfid, void **, p_blk);
}

Inline ER
tget_mpf(ID mpfid, void **p_blk, TMO tmout)
{
	CAL_SVC_3M(ER, TFN_TGET_MPF, ID, mpfid, void **, p_blk, TMO, tmout);
}

Inline ER
rel_mpf(ID mpfid, void *blk)
{
	CAL_SVC_2M(ER, TFN_REL_MPF, ID, mpfid, void *, blk);
}

Inline ER
get_tim(SYSTIM *p_systim)
{
	CAL_SVC_1M(ER, TFN_GET_TIM, SYSTIM *, p_systim);
}

Inline ER
get_utm(SYSUTM *p_sysutm)
{
	CAL_SVC_1M(ER, TFN_GET_UTM, SYSUTM *, p_sysutm);
}

Inline ER
ref_ovr(ID tskid, T_ROVR *pk_rovr)
{
	CAL_SVC_2M(ER, TFN_REF_OVR, ID, tskid, T_ROVR *, pk_rovr);
}

Inline ER
sta_cyc(ID cycid)
{
	CAL_SVC_1M(ER, TFN_STA_CYC, ID, cycid);
}

Inline ER
stp_cyc(ID cycid)
{
	CAL_SVC_1M(ER, TFN_STP_CYC, ID, cycid);
}

Inline ER
sta_alm(ID almid, RELTIM almtim)
{
	CAL_SVC_2M(ER, TFN_STA_ALM, ID, almid, RELTIM, almtim);
}

Inline ER
ista_alm(ID almid, RELTIM almtim)
{
	CAL_SVC_2M(ER, TFN_ISTA_ALM, ID, almid, RELTIM, almtim);
}

Inline ER
stp_alm(ID almid)
{
	CAL_SVC_1M(ER, TFN_STP_ALM, ID, almid);
}

Inline ER
istp_alm(ID almid)
{
	CAL_SVC_1M(ER, TFN_ISTP_ALM, ID, almid);
}

Inline ER
sta_ovr(ID tskid, OVRTIM ovrtim)
{
	CAL_SVC_2M(ER, TFN_STA_OVR, ID, tskid, OVRTIM, ovrtim);
}

Inline ER
ista_ovr(ID tskid, OVRTIM ovrtim)
{
	CAL_SVC_2M(ER, TFN_ISTA_OVR, ID, tskid, OVRTIM, ovrtim);
}

Inline ER
stp_ovr(ID tskid)
{
	CAL_SVC_1M(ER, TFN_STP_OVR, ID, tskid);
}

Inline ER
istp_ovr(ID tskid)
{
	CAL_SVC_1M(ER, TFN_ISTP_OVR, ID, tskid);
}

Inline ER
rot_rdq(PRI tskpri)
{
	CAL_SVC_1M(ER, TFN_ROT_RDQ, PRI, tskpri);
}

Inline ER
irot_rdq(PRI tskpri)
{
	CAL_SVC_1M(ER, TFN_IROT_RDQ, PRI, tskpri);
}

Inline ER
get_did(ID *p_domid)
{
	CAL_SVC_1M(ER, TFN_GET_DID, ID *, p_domid);
}

Inline ER
get_tid(ID *p_tskid)
{
	CAL_SVC_1M(ER, TFN_GET_TID, ID *, p_tskid);
}

Inline ER
iget_tid(ID *p_tskid)
{
	CAL_SVC_1M(ER, TFN_IGET_TID, ID *, p_tskid);
}

Inline ER
loc_cpu(void)
{
	CAL_SVC_0M(ER, TFN_LOC_CPU);
}

Inline ER
iloc_cpu(void)
{
	CAL_SVC_0M(ER, TFN_ILOC_CPU);
}

Inline ER
unl_cpu(void)
{
	CAL_SVC_0M(ER, TFN_UNL_CPU);
}

Inline ER
iunl_cpu(void)
{
	CAL_SVC_0M(ER, TFN_IUNL_CPU);
}

Inline ER
dis_dsp(void)
{
	CAL_SVC_0M(ER, TFN_DIS_DSP);
}

Inline ER
ena_dsp(void)
{
	CAL_SVC_0M(ER, TFN_ENA_DSP);
}

Inline bool_t
sns_ctx(void)
{
	CAL_SVC_0M(bool_t, TFN_SNS_CTX);
}

Inline bool_t
sns_loc(void)
{
	CAL_SVC_0M(bool_t, TFN_SNS_LOC);
}

Inline bool_t
sns_dsp(void)
{
	CAL_SVC_0M(bool_t, TFN_SNS_DSP);
}

Inline bool_t
sns_dpn(void)
{
	CAL_SVC_0M(bool_t, TFN_SNS_DPN);
}

Inline bool_t
sns_ker(void)
{
	CAL_SVC_0M(bool_t, TFN_SNS_KER);
}

Inline ER
ext_ker(void)
{
	CAL_SVC_0M(ER, TFN_EXT_KER);
}

Inline ER
prb_mem(const void *base, SIZE size, ID tskid, MODE pmmode)
{
	CAL_SVC_4M(ER, TFN_PRB_MEM, const void *, base, SIZE, size, ID, tskid, MODE, pmmode);
}

Inline ER
dis_int(INTNO intno)
{
	CAL_SVC_1M(ER, TFN_DIS_INT, INTNO, intno);
}

Inline ER
ena_int(INTNO intno)
{
	CAL_SVC_1M(ER, TFN_ENA_INT, INTNO, intno);
}

Inline ER
chg_ipm(PRI intpri)
{
	CAL_SVC_1M(ER, TFN_CHG_IPM, PRI, intpri);
}

Inline ER
get_ipm(PRI *p_intpri)
{
	CAL_SVC_1M(ER, TFN_GET_IPM, PRI *, p_intpri);
}

Inline bool_t
xsns_dpn(void *p_excinf)
{
	CAL_SVC_1M(bool_t, TFN_XSNS_DPN, void *, p_excinf);
}

Inline bool_t
xsns_xpn(void *p_excinf)
{
	CAL_SVC_1M(bool_t, TFN_XSNS_XPN, void *, p_excinf);
}

Inline ER
ini_sem(ID semid)
{
	CAL_SVC_1M(ER, TFN_INI_SEM, ID, semid);
}

Inline ER
ini_flg(ID flgid)
{
	CAL_SVC_1M(ER, TFN_INI_FLG, ID, flgid);
}

Inline ER
ini_dtq(ID dtqid)
{
	CAL_SVC_1M(ER, TFN_INI_DTQ, ID, dtqid);
}

Inline ER
ini_pdq(ID pdqid)
{
	CAL_SVC_1M(ER, TFN_INI_PDQ, ID, pdqid);
}

Inline ER
ini_mtx(ID mtxid)
{
	CAL_SVC_1M(ER, TFN_INI_MTX, ID, mtxid);
}

Inline ER
ini_mpf(ID mpfid)
{
	CAL_SVC_1M(ER, TFN_INI_MPF, ID, mpfid);
}

Inline ER
ref_tsk(ID tskid, T_RTSK *pk_rtsk)
{
	CAL_SVC_2M(ER, TFN_REF_TSK, ID, tskid, T_RTSK *, pk_rtsk);
}

Inline ER
ref_sem(ID semid, T_RSEM *pk_rsem)
{
	CAL_SVC_2M(ER, TFN_REF_SEM, ID, semid, T_RSEM *, pk_rsem);
}

Inline ER
ref_flg(ID flgid, T_RFLG *pk_rflg)
{
	CAL_SVC_2M(ER, TFN_REF_FLG, ID, flgid, T_RFLG *, pk_rflg);
}

Inline ER
ref_dtq(ID dtqid, T_RDTQ *pk_rdtq)
{
	CAL_SVC_2M(ER, TFN_REF_DTQ, ID, dtqid, T_RDTQ *, pk_rdtq);
}

Inline ER
ref_pdq(ID pdqid, T_RPDQ *pk_rpdq)
{
	CAL_SVC_2M(ER, TFN_REF_PDQ, ID, pdqid, T_RPDQ *, pk_rpdq);
}

Inline ER
ref_mtx(ID mtxid, T_RMTX *pk_rmtx)
{
	CAL_SVC_2M(ER, TFN_REF_MTX, ID, mtxid, T_RMTX *, pk_rmtx);
}

Inline ER
ref_mpf(ID mpfid, T_RMPF *pk_rmpf)
{
	CAL_SVC_2M(ER, TFN_REF_MPF, ID, mpfid, T_RMPF *, pk_rmpf);
}

Inline ER
ref_cyc(ID cycid, T_RCYC *pk_rcyc)
{
	CAL_SVC_2M(ER, TFN_REF_CYC, ID, cycid, T_RCYC *, pk_rcyc);
}

Inline ER
ref_alm(ID almid, T_RALM *pk_ralm)
{
	CAL_SVC_2M(ER, TFN_REF_ALM, ID, almid, T_RALM *, pk_ralm);
}

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_TOOL_SVC_H */
