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

#ifndef V8_SH4_CHECKS_SH4_H_
#define V8_SH4_CHECKS_SH4_H_

#define SH4_CHECK_RANGE_add_imm(imm) ((imm) >= -128 && (imm) <= 127)
#define FITS_SH4_add_imm(imm) (SH4_CHECK_RANGE_add_imm(imm))

#define SH4_CHECK_RANGE_and_imm_R0(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_and_imm_R0(imm) (SH4_CHECK_RANGE_and_imm_R0(imm))

#define SH4_CHECK_RANGE_andb_imm_dispR0GBR(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_andb_imm_dispR0GBR(imm) \
        (SH4_CHECK_RANGE_andb_imm_dispR0GBR(imm))

#define SH4_CHECK_RANGE_bra(imm) ((imm) >= -4096 && (imm) <= 4094)
#define SH4_CHECK_ALIGN_bra(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_bra(imm) (SH4_CHECK_RANGE_bra(imm) && SH4_CHECK_ALIGN_bra(imm))

#define SH4_CHECK_RANGE_bsr(imm) ((imm) >= -4096 && (imm) <= 4094)
#define SH4_CHECK_ALIGN_bsr(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_bsr(imm) (SH4_CHECK_RANGE_bsr(imm) && SH4_CHECK_ALIGN_bsr(imm))

#define SH4_CHECK_RANGE_bt(imm) ((imm) >= -256 && (imm) <= 254)
#define SH4_CHECK_ALIGN_bt(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_bt(imm) (SH4_CHECK_RANGE_bt(imm) && SH4_CHECK_ALIGN_bt(imm))

#define SH4_CHECK_RANGE_bf(imm) ((imm) >= -256 && (imm) <= 254)
#define SH4_CHECK_ALIGN_bf(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_bf(imm) (SH4_CHECK_RANGE_bf(imm) && SH4_CHECK_ALIGN_bf(imm))

#define SH4_CHECK_RANGE_bts(imm) ((imm) >= -256 && (imm) <= 254)
#define SH4_CHECK_ALIGN_bts(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_bts(imm) (SH4_CHECK_RANGE_bts(imm) && SH4_CHECK_ALIGN_bts(imm))

#define SH4_CHECK_RANGE_bfs(imm) ((imm) >= -256 && (imm) <= 254)
#define SH4_CHECK_ALIGN_bfs(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_bfs(imm) (SH4_CHECK_RANGE_bfs(imm) && SH4_CHECK_ALIGN_bfs(imm))

#define SH4_CHECK_RANGE_cmpeq_imm_R0(imm) ((imm) >= -128 && (imm) <= 127)
#define FITS_SH4_cmpeq_imm_R0(imm) (SH4_CHECK_RANGE_cmpeq_imm_R0(imm))

#define SH4_CHECK_RANGE_ldc_bank(imm) ((imm) >= 0 && (imm) <= 7)
#define FITS_SH4_ldc_bank(imm) (SH4_CHECK_RANGE_ldc_bank(imm))

#define SH4_CHECK_RANGE_ldcl_incRd_bank(imm) ((imm) >= 0 && (imm) <= 7)
#define FITS_SH4_ldcl_incRd_bank(imm) (SH4_CHECK_RANGE_ldcl_incRd_bank(imm))

#define SH4_CHECK_RANGE_mov_imm(imm) ((imm) >= -128 && (imm) <= 127)
#define FITS_SH4_mov_imm(imm) (SH4_CHECK_RANGE_mov_imm(imm))

#define SH4_CHECK_RANGE_movb_dispRs_R0(imm) ((imm) >= 0 && (imm) <= 15)
#define FITS_SH4_movb_dispRs_R0(imm) (SH4_CHECK_RANGE_movb_dispRs_R0(imm))

#define SH4_CHECK_RANGE_movb_dispGBR_R0(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_movb_dispGBR_R0(imm) (SH4_CHECK_RANGE_movb_dispGBR_R0(imm))

#define SH4_CHECK_RANGE_movb_R0_dispRd(imm) ((imm) >= 0 && (imm) <= 15)
#define FITS_SH4_movb_R0_dispRd(imm) (SH4_CHECK_RANGE_movb_R0_dispRd(imm))

#define SH4_CHECK_RANGE_movb_R0_dispGBR(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_movb_R0_dispGBR(imm) (SH4_CHECK_RANGE_movb_R0_dispGBR(imm))

#define SH4_CHECK_RANGE_movl_dispRd(imm) ((imm) >= 0 && (imm) <= 60)
#define SH4_CHECK_ALIGN_movl_dispRd(imm) (((imm) & 0x3) == 0)
#define FITS_SH4_movl_dispRd(imm) \
        (SH4_CHECK_RANGE_movl_dispRd(imm) && SH4_CHECK_ALIGN_movl_dispRd(imm))

#define SH4_CHECK_RANGE_movl_dispRs(imm) ((imm) >= 0 && (imm) <= 60)
#define SH4_CHECK_ALIGN_movl_dispRs(imm) (((imm) & 0x3) == 0)
#define FITS_SH4_movl_dispRs(imm) \
        (SH4_CHECK_RANGE_movl_dispRs(imm) && SH4_CHECK_ALIGN_movl_dispRs(imm))

#define SH4_CHECK_RANGE_movl_dispGBR_R0(imm) ((imm) >= 0 && (imm) <= 1020)
#define SH4_CHECK_ALIGN_movl_dispGBR_R0(imm) (((imm) & 0x3) == 0)
#define FITS_SH4_movl_dispGBR_R0(imm) \
        (SH4_CHECK_RANGE_movl_dispGBR_R0(imm) && \
         SH4_CHECK_ALIGN_movl_dispGBR_R0(imm))

#define SH4_CHECK_RANGE_movl_dispPC(imm) ((imm) >= 0 && (imm) <= 1020)
#define SH4_CHECK_ALIGN_movl_dispPC(imm) (((imm) & 0x3) == 0)
#define FITS_SH4_movl_dispPC(imm) \
        (SH4_CHECK_RANGE_movl_dispPC(imm) && SH4_CHECK_ALIGN_movl_dispPC(imm))

#define SH4_CHECK_RANGE_movl_R0_dispGBR(imm) ((imm) >= 0 && (imm) <= 1020)
#define SH4_CHECK_ALIGN_movl_R0_dispGBR(imm) (((imm) & 0x3) == 0)
#define FITS_SH4_movl_R0_dispGBR(imm) \
        (SH4_CHECK_RANGE_movl_R0_dispGBR(imm) && \
         SH4_CHECK_ALIGN_movl_R0_dispGBR(imm))

#define SH4_CHECK_RANGE_movw_dispRs_R0(imm) ((imm) >= 0 && (imm) <= 30)
#define SH4_CHECK_ALIGN_movw_dispRs_R0(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_movw_dispRs_R0(imm) \
        (SH4_CHECK_RANGE_movw_dispRs_R0(imm) && \
         SH4_CHECK_ALIGN_movw_dispRs_R0(imm))

#define SH4_CHECK_RANGE_movw_dispGBR_R0(imm) ((imm) >= 0 && (imm) <= 510)
#define SH4_CHECK_ALIGN_movw_dispGBR_R0(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_movw_dispGBR_R0(imm) \
        (SH4_CHECK_RANGE_movw_dispGBR_R0(imm) && \
         SH4_CHECK_ALIGN_movw_dispGBR_R0(imm))

#define SH4_CHECK_RANGE_movw_dispPC(imm) ((imm) >= 0 && (imm) <= 510)
#define SH4_CHECK_ALIGN_movw_dispPC(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_movw_dispPC(imm) \
        (SH4_CHECK_RANGE_movw_dispPC(imm) && \
         SH4_CHECK_ALIGN_movw_dispPC(imm))

#define SH4_CHECK_RANGE_movw_R0_dispRd(imm) ((imm) >= 0 && (imm) <= 30)
#define SH4_CHECK_ALIGN_movw_R0_dispRd(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_movw_R0_dispRd(imm) \
        (SH4_CHECK_RANGE_movw_R0_dispRd(imm) && \
         SH4_CHECK_ALIGN_movw_R0_dispRd(imm))

#define SH4_CHECK_RANGE_movw_R0_dispGBR(imm) ((imm) >= 0 && (imm) <= 510)
#define SH4_CHECK_ALIGN_movw_R0_dispGBR(imm) (((imm) & 0x1) == 0)
#define FITS_SH4_movw_R0_dispGBR(imm) \
        (SH4_CHECK_RANGE_movw_R0_dispGBR(imm) && \
         SH4_CHECK_ALIGN_movw_R0_dispGBR(imm))

#define SH4_CHECK_RANGE_mova_dispPC_R0(imm) ((imm) >= 0 && (imm) <= 1020)
#define SH4_CHECK_ALIGN_mova_dispPC_R0(imm) (((imm) & 0x3) == 0)
#define FITS_SH4_mova_dispPC_R0(imm) \
        (SH4_CHECK_RANGE_mova_dispPC_R0(imm) && \
         SH4_CHECK_ALIGN_mova_dispPC_R0(imm))

#define SH4_CHECK_RANGE_or_imm_R0(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_or_imm_R0(imm) (SH4_CHECK_RANGE_or_imm_R0(imm))

#define SH4_CHECK_RANGE_orb_imm_dispR0GBR(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_orb_imm_dispR0GBR(imm) (SH4_CHECK_RANGE_orb_imm_dispR0GBR(imm))

#define SH4_CHECK_RANGE_stc_bank(imm) ((imm) >= 0 && (imm) <= 7)
#define FITS_SH4_stc_bank(imm) (SH4_CHECK_RANGE_stc_bank(imm))

#define SH4_CHECK_RANGE_stcl_bank_decRd(imm) ((imm) >= 0 && (imm) <= 7)
#define FITS_SH4_stcl_bank_decRd(imm) (SH4_CHECK_RANGE_stcl_bank_decRd(imm))

#define SH4_CHECK_RANGE_trapa_imm(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_trapa_imm(imm) (SH4_CHECK_RANGE_trapa_imm(imm))

#define SH4_CHECK_RANGE_tst_imm_R0(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_tst_imm_R0(imm) (SH4_CHECK_RANGE_tst_imm_R0(imm))

#define SH4_CHECK_RANGE_tstb_imm_dispR0GBR(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_tstb_imm_dispR0GBR(imm) \
        (SH4_CHECK_RANGE_tstb_imm_dispR0GBR(imm))

#define SH4_CHECK_RANGE_xor_imm_R0(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_xor_imm_R0(imm) (SH4_CHECK_RANGE_xor_imm_R0(imm))

#define SH4_CHECK_RANGE_xorb_imm_dispR0GBR(imm) ((imm) >= 0 && (imm) <= 255)
#define FITS_SH4_xorb_imm_dispR0GBR(imm) \
        (SH4_CHECK_RANGE_xorb_imm_dispR0GBR(imm))

#endif  // V8_SH4_CHECKS_SH4_H_
