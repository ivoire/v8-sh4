// Copyright 2011-2012 the V8 project authors. All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//     * Neither the name of Google Inc. nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Low level function that generate sh4 opcodes

void add_imm_(int imm, Register Rd);

void add_(Register Rs, Register Rd);

void addc_(Register Rs, Register Rd);

void addv_(Register Rs, Register Rd);

void and_imm_R0_(int imm);

void and_(Register Rs, Register Rd);

void andb_imm_dispR0GBR_(int imm);

void bra_(int imm);

void bsr_(int imm);

void bt_(int imm);

void bf_(int imm);

void bts_(int imm);

void bfs_(int imm);

void clrmac_();

void clrs_();

void clrt_();

void cmpeq_imm_R0_(int imm);

void cmpeq_(Register Rs, Register Rd);

void cmpge_(Register Rs, Register Rd);

void cmpgt_(Register Rs, Register Rd);

void cmphi_(Register Rs, Register Rd);

void cmphs_(Register Rs, Register Rd);

void cmppl_(Register Rd);

void cmppz_(Register Rd);

void cmpstr_(Register Rs, Register Rd);

void div0s_(Register Rs, Register Rd);

void div0u_();

void div1_(Register Rs, Register Rd);

void extsb_(Register Rs, Register Rd);

void extsw_(Register Rs, Register Rd);

void extub_(Register Rs, Register Rd);

void extuw_(Register Rs, Register Rd);

void icbi_indRd_(Register Rd);

void jmp_indRd_(Register Rd);

void jsr_indRd_(Register Rd);

void ldc_SR_(Register Rd);

void ldc_GBR_(Register Rd);

void ldc_SGR_(Register Rd);

void ldc_VBR_(Register Rd);

void ldc_SSR_(Register Rd);

void ldc_SPC_(Register Rd);

void ldc_DBR_(Register Rd);

void ldc_bank_(Register Rd, int imm);

void ldcl_incRd_SR_(Register Rd);

void ldcl_incRd_GBR_(Register Rd);

void ldcl_incRd_VBR_(Register Rd);

void ldcl_incRd_SGR_(Register Rd);

void ldcl_incRd_SSR_(Register Rd);

void ldcl_incRd_SPC_(Register Rd);

void ldcl_incRd_DBR_(Register Rd);

void ldcl_incRd_bank_(Register Rd, int imm);

void lds_MACH_(Register Rd);

void lds_MACL_(Register Rd);

void lds_PR_(Register Rd);

void lds_FPUL_(Register Rs);

void lds_FPSCR_(Register Rs);

void ldsl_incRd_MACH_(Register Rd);

void ldsl_incRd_MACL_(Register Rd);

void ldsl_incRd_PR_(Register Rd);

void ldsl_incRs_FPUL_(Register Rs);

void ldsl_incRs_FPSCR_(Register Rs);

void ldtlb_();

void macw_incRs_incRd_(Register Rs, Register Rd);

void mov_imm_(int imm, Register Rd);

void mov_(Register Rs, Register Rd);

void movb_dispR0Rd_(Register Rs, Register Rd);

void movb_decRd_(Register Rs, Register Rd);

void movb_indRd_(Register Rs, Register Rd);

void movb_dispRs_R0_(int imm, Register Rs);

void movb_dispGBR_R0_(int imm);

void movb_dispR0Rs_(Register Rs, Register Rd);

void movb_incRs_(Register Rs, Register Rd);

void movb_indRs_(Register Rs, Register Rd);

void movb_R0_dispRd_(int imm, Register Rd);

void movb_R0_dispGBR_(int imm);

void movl_dispRd_(Register Rs, int imm, Register Rd);

void movl_dispR0Rd_(Register Rs, Register Rd);

void movl_decRd_(Register Rs, Register Rd);

void movl_indRd_(Register Rs, Register Rd);

void movl_dispRs_(int imm, Register Rs, Register Rd);

void movl_dispGBR_R0_(int imm);

void movl_dispPC_(int imm, Register Rd);

void movl_dispR0Rs_(Register Rs, Register Rd);

void movl_incRs_(Register Rs, Register Rd);

void movl_indRs_(Register Rs, Register Rd);

void movl_R0_dispGBR_(int imm);

void movw_dispR0Rd_(Register Rs, Register Rd);

void movw_decRd_(Register Rs, Register Rd);

void movw_indRd_(Register Rs, Register Rd);

void movw_dispRs_R0_(int imm, Register Rs);

void movw_dispGBR_R0_(int imm);

void movw_dispPC_(int imm, Register Rd);

void movw_dispR0Rs_(Register Rs, Register Rd);

void movw_incRs_(Register Rs, Register Rd);

void movw_indRs_(Register Rs, Register Rd);

void movw_R0_dispRd_(int imm, Register Rd);

void movw_R0_dispGBR_(int imm);

void mova_dispPC_R0_(int imm);

void movcal_R0_indRd_(Register Rd);

void movcol_R0_indRd_(Register Rd);

void movlil_indRs_R0_(Register Rs);

void movt_(Register Rd);

void movual_indRs_R0_(Register Rs);

void movual_incRs_R0_(Register Rs);

void mulsw_(Register Rs, Register Rd);

void muls_(Register Rs, Register Rd);

void mull_(Register Rs, Register Rd);

void muluw_(Register Rs, Register Rd);

void mulu_(Register Rs, Register Rd);

void neg_(Register Rs, Register Rd);

void negc_(Register Rs, Register Rd);

void nop_();

void not_(Register Rs, Register Rd);

void ocbi_indRd_(Register Rd);

void ocbp_indRd_(Register Rd);

void ocbwb_indRd_(Register Rd);

void or_imm_R0_(int imm);

void or_(Register Rs, Register Rd);

void orb_imm_dispR0GBR_(int imm);

void pref_indRd_(Register Rd);

void prefi_indRd_(Register Rd);

void rotcl_(Register Rd);

void rotcr_(Register Rd);

void rotl_(Register Rd);

void rotr_(Register Rd);

void rte_();

void rts_();

void sets_();

void sett_();

void shad_(Register Rs, Register Rd);

void shld_(Register Rs, Register Rd);

void shal_(Register Rd);

void shar_(Register Rd);

void shll_(Register Rd);

void shll16_(Register Rd);

void shll2_(Register Rd);

void shll8_(Register Rd);

void shlr_(Register Rd);

void shlr16_(Register Rd);

void shlr2_(Register Rd);

void shlr8_(Register Rd);

void sleep_();

void stc_SR_(Register Rd);

void stc_GBR_(Register Rd);

void stc_VBR_(Register Rd);

void stc_SSR_(Register Rd);

void stc_SPC_(Register Rd);

void stc_SGR_(Register Rd);

void stc_DBR_(Register Rd);

void stc_bank_(int imm, Register Rd);

void stcl_SR_decRd_(Register Rd);

void stcl_VBR_decRd_(Register Rd);

void stcl_SSR_decRd_(Register Rd);

void stcl_SPC_decRd_(Register Rd);

void stcl_GBR_decRd_(Register Rd);

void stcl_SGR_decRd_(Register Rd);

void stcl_DBR_decRd_(Register Rd);

void stcl_bank_decRd_(int imm, Register Rd);

void sts_MACH_(Register Rd);

void sts_MACL_(Register Rd);

void sts_PR_(Register Rd);

void sts_FPUL_(Register Rd);

void sts_FPSCR_(Register Rd);

void stsl_MACH_decRd_(Register Rd);

void stsl_MACL_decRd_(Register Rd);

void stsl_PR_decRd_(Register Rd);

void stsl_FPUL_decRd_(Register Rd);

void stsl_FPSCR_decRd_(Register Rd);

void sub_(Register Rs, Register Rd);

void subc_(Register Rs, Register Rd);

void subv_(Register Rs, Register Rd);

void swapb_(Register Rs, Register Rd);

void swapw_(Register Rs, Register Rd);

void synco_();

void tasb_indRd_(Register Rd);

void trapa_imm_(int imm);

void tst_imm_R0_(int imm);

void tst_(Register Rs, Register Rd);

void tstb_imm_dispR0GBR_(int imm);

void xor_imm_R0_(int imm);

void xor_(Register Rs, Register Rd);

void xorb_imm_dispR0GBR_(int imm);

void xtrct_(Register Rs, Register Rd);

void dt_(Register Rd);

void dmulsl_(Register Rs, Register Rd);

void dmulul_(Register Rs, Register Rd);

void macl_incRs_incRd_(Register Rs, Register Rd);

void braf_(Register Rd);

void bsrf_(Register Rd);

void fabs_(SwVfpRegister Rd);

void fabs_double_(DwVfpRegister Rd);

void fadd_(SwVfpRegister Rs, SwVfpRegister Rd);

void fadd_double_(DwVfpRegister Rs, DwVfpRegister Rd);

void fcmpeq_(SwVfpRegister Rs, SwVfpRegister Rd);

void fcmpeq_double_(DwVfpRegister Rs, DwVfpRegister Rd);

void fcmpgt_(SwVfpRegister Rs, SwVfpRegister Rd);

void fcmpgt_double_(DwVfpRegister Rs, DwVfpRegister Rd);

void fcnvds_double_FPUL_(DwVfpRegister Rd);

void fcnvsd_FPUL_double_(DwVfpRegister Rd);

void fdiv_(SwVfpRegister Rs, SwVfpRegister Rd);

void fdiv_double_(DwVfpRegister Rs, DwVfpRegister Rd);

void fipr_(SwVfpRegister Rs, SwVfpRegister Rd);

void fldi0_(SwVfpRegister Rd);

void fldi1_(SwVfpRegister Rd);

void flds_FPUL_(SwVfpRegister Rd);

void float_FPUL_(SwVfpRegister Rd);

void float_FPUL_double_(DwVfpRegister Rd);

void fmac_(SwVfpRegister Rs, SwVfpRegister Rd);

void fmov_(SwVfpRegister Rs, SwVfpRegister Rd);

void fmov_Xdouble_Xdouble_(DwVfpRegister Rs, DwVfpRegister Rd);

void fmov_indRs_(Register Rs, SwVfpRegister Rd);

void fmov_indRs_Xdouble_(Register Rs, DwVfpRegister Rd);

void fmov_indRd_(SwVfpRegister Rs, Register Rd);

void fmov_Xdouble_indRd_(DwVfpRegister Rs, Register Rd);

void fmov_incRs_(Register Rs, SwVfpRegister Rd);

void fmov_decRd_(SwVfpRegister Rs, Register Rd);

void fmov_dispR0Rs_(Register Rs, SwVfpRegister Rd);

void fmov_dispR0Rs_Xdouble_(Register Rs, DwVfpRegister Rd);

void fmov_dispR0Rd_(SwVfpRegister Rs, Register Rd);

void fmov_Xdouble_dispR0Rd_(DwVfpRegister Rs, Register Rd);

void fmovd_indRs_Xdouble_(Register Rs, DwVfpRegister Rd);

void fmovd_Xdouble_indRd_(DwVfpRegister Rs, Register Rd);

void fmovd_incRs_Xdouble_(Register Rs, DwVfpRegister Rd);

void fmovd_Xdouble_decRd_(DwVfpRegister Rs, Register Rd);

void fmovd_dispR0Rs_Xdouble_(Register Rs, DwVfpRegister Rd);

void fmovd_Xdouble_dispR0Rd_(DwVfpRegister Rs, Register Rd);

void fmovs_indRs_(Register Rs, SwVfpRegister Rd);

void fmovs_indRd_(SwVfpRegister Rs, Register Rd);

void fmovs_incRs_(Register Rs, SwVfpRegister Rd);

void fmovs_decRd_(SwVfpRegister Rs, Register Rd);

void fmovs_dispR0Rs_(Register Rs, SwVfpRegister Rd);

void fmovs_dispR0Rd_(SwVfpRegister Rs, Register Rd);

void fmul_(SwVfpRegister Rs, SwVfpRegister Rd);

void fmul_double_(DwVfpRegister Rs, DwVfpRegister Rd);

void fneg_(SwVfpRegister Rd);

void fneg_double_(DwVfpRegister Rd);

void fpchg_();

void frchg_();

void fsca_FPUL_double_(DwVfpRegister Rd);

void fschg_();

void fsqrt_(SwVfpRegister Rd);

void fsqrt_double_(DwVfpRegister Rd);

void fsrra_(SwVfpRegister Rd);

void fsts_FPUL_(SwVfpRegister Rd);

void fsub_(SwVfpRegister Rs, SwVfpRegister Rd);

void fsub_double_(DwVfpRegister Rs, DwVfpRegister Rd);

void ftrc_FPUL_(SwVfpRegister Rd);

void ftrc_double_FPUL_(DwVfpRegister Rd);

void ftrv_(Register Rd);
