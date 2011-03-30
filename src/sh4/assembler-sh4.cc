// Copyright 2011 the V8 project authors. All rights reserved.
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

#include "v8.h"

#if defined(V8_TARGET_ARCH_SH4)

#include "disassembler.h"
#include "macro-assembler.h"
#include "serialize.h"

namespace v8 {
namespace internal {

void Assembler::Align(int m) {
  UNIMPLEMENTED();
}


Assembler::Assembler(void* buffer, int buffer_size)
    : AssemblerBase(Isolate::Current()),
      positions_recorder_(this) {
  UNIMPLEMENTED();
}


void Assembler::GetCode(CodeDesc* desc) {
  UNIMPLEMENTED();
}


void Assembler::RecordComment(const char* msg, bool force) {
  UNIMPLEMENTED();
}


void Assembler::RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data) {
  UNIMPLEMENTED();
}


void Assembler::bind(Label* L) {
  UNIMPLEMENTED();
}


void Assembler::call(Label* L) {
  UNIMPLEMENTED();
}


void Assembler::db(uint8_t data) {
  UNIMPLEMENTED();
}


void Assembler::dd(uint32_t data) {
  UNIMPLEMENTED();
}


void Assembler::jmp(Label* L) {
  UNIMPLEMENTED();
}


void Assembler::pop(Register dst) {
  UNIMPLEMENTED();
}


void Assembler::push(Register src) {
  UNIMPLEMENTED();
}


Assembler::~Assembler() {
  UNIMPLEMENTED();
}


CpuFeatures::CpuFeatures() {
  UNIMPLEMENTED();
}


const int RelocInfo::kApplyMask = 0;

bool RelocInfo::IsCodedSpecially() {
  UNIMPLEMENTED();
  return false;
}


inline void asm_output(const char *str, int a=0 , int b=0, int c=0) {}
#define REGNUM(reg) (reg).code()

#define SH4_CHECK_RANGE_add_imm(imm) ((imm) >= -128 && (imm) <= 127)

#define FITS_SH4_add_imm(imm) (SH4_CHECK_RANGE_add_imm(imm))

void Assembler::add_imm(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_add_imm(imm));
  emit((0x7 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0xFF) << 0));
  asm_output("add_imm %d, R%d", imm, REGNUM(Rx));
}


void Assembler::add(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("add R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::addc(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("addc R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::addv(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("addv R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_and_imm_R0(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_and_imm_R0(imm) (SH4_CHECK_RANGE_and_imm_R0(imm))

void Assembler::and_imm_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_and_imm_R0(imm));
  emit((0xC << 12) | (0x9 << 8) | ((imm & 0xFF) << 0));
  asm_output("and_imm_R0 %d", imm);
}


void Assembler::and_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("and R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_andb_imm_dispR0GBR(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_andb_imm_dispR0GBR(imm) (SH4_CHECK_RANGE_andb_imm_dispR0GBR(imm))

void Assembler::andb_imm_dispR0GBR(int imm) {
  ASSERT(SH4_CHECK_RANGE_andb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xD << 8) | ((imm & 0xFF) << 0));
  asm_output("andb_imm_dispR0GBR %d", imm);
}


#define SH4_CHECK_RANGE_bra(imm) ((imm) >= -4096 && (imm) <= 4094)

#define SH4_CHECK_ALIGN_bra(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_bra(imm) (SH4_CHECK_RANGE_bra(imm) && SH4_CHECK_ALIGN_bra(imm))

void Assembler::bra(int imm) {
  ASSERT(SH4_CHECK_RANGE_bra(imm) && SH4_CHECK_ALIGN_bra(imm));
  emit((0xA << 12) | (((imm & 0x1FFE) >> 1) << 0));
  asm_output("bra %d", imm);
}


#define SH4_CHECK_RANGE_bsr(imm) ((imm) >= -4096 && (imm) <= 4094)

#define SH4_CHECK_ALIGN_bsr(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_bsr(imm) (SH4_CHECK_RANGE_bsr(imm) && SH4_CHECK_ALIGN_bsr(imm))

void Assembler::bsr(int imm) {
  ASSERT(SH4_CHECK_RANGE_bsr(imm) && SH4_CHECK_ALIGN_bsr(imm));
  emit((0xB << 12) | (((imm & 0x1FFE) >> 1) << 0));
  asm_output("bsr %d", imm);
}


#define SH4_CHECK_RANGE_bt(imm) ((imm) >= -256 && (imm) <= 254)

#define SH4_CHECK_ALIGN_bt(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_bt(imm) (SH4_CHECK_RANGE_bt(imm) && SH4_CHECK_ALIGN_bt(imm))

void Assembler::bt(int imm) {
  ASSERT(SH4_CHECK_RANGE_bt(imm) && SH4_CHECK_ALIGN_bt(imm));
  emit((0x8 << 12) | (0x9 << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("bt %d", imm);
}


#define SH4_CHECK_RANGE_bf(imm) ((imm) >= -256 && (imm) <= 254)

#define SH4_CHECK_ALIGN_bf(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_bf(imm) (SH4_CHECK_RANGE_bf(imm) && SH4_CHECK_ALIGN_bf(imm))

void Assembler::bf(int imm) {
  ASSERT(SH4_CHECK_RANGE_bf(imm) && SH4_CHECK_ALIGN_bf(imm));
  emit((0x8 << 12) | (0xB << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("bf %d", imm);
}


#define SH4_CHECK_RANGE_bts(imm) ((imm) >= -256 && (imm) <= 254)

#define SH4_CHECK_ALIGN_bts(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_bts(imm) (SH4_CHECK_RANGE_bts(imm) && SH4_CHECK_ALIGN_bts(imm))

void Assembler::bts(int imm) {
  ASSERT(SH4_CHECK_RANGE_bts(imm) && SH4_CHECK_ALIGN_bts(imm));
  emit((0x8 << 12) | (0xD << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("bts %d", imm);
}


#define SH4_CHECK_RANGE_bfs(imm) ((imm) >= -256 && (imm) <= 254)

#define SH4_CHECK_ALIGN_bfs(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_bfs(imm) (SH4_CHECK_RANGE_bfs(imm) && SH4_CHECK_ALIGN_bfs(imm))

void Assembler::bfs(int imm) {
  ASSERT(SH4_CHECK_RANGE_bfs(imm) && SH4_CHECK_ALIGN_bfs(imm));
  emit((0x8 << 12) | (0xF << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("bfs %d", imm);
}


void Assembler::clrmac() {
  emit((0x0 << 12) | (0x0 << 8) | (0x2 << 4) | (0x8 << 0));
  asm_output("clrmac");
}


void Assembler::clrs() {
  emit((0x0 << 12) | (0x0 << 8) | (0x4 << 4) | (0x8 << 0));
  asm_output("clrs");
}


void Assembler::clrt() {
  emit((0x0 << 12) | (0x0 << 8) | (0x0 << 4) | (0x8 << 0));
  asm_output("clrt");
}


#define SH4_CHECK_RANGE_cmpeq_imm_R0(imm) ((imm) >= -128 && (imm) <= 127)

#define FITS_SH4_cmpeq_imm_R0(imm) (SH4_CHECK_RANGE_cmpeq_imm_R0(imm))

void Assembler::cmpeq_imm_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_cmpeq_imm_R0(imm));
  emit((0x8 << 12) | (0x8 << 8) | ((imm & 0xFF) << 0));
  asm_output("cmpeq_imm_R0 %d", imm);
}


void Assembler::cmpeq(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x0 << 0));
  asm_output("cmpeq R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmpge(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x3 << 0));
  asm_output("cmpge R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmpgt(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x7 << 0));
  asm_output("cmpgt R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmphi(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("cmphi R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmphs(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x2 << 0));
  asm_output("cmphs R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmppl(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x5 << 0));
  asm_output("cmppl R%d", REGNUM(Rx));
}


void Assembler::cmppz(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x1 << 0));
  asm_output("cmppz R%d", REGNUM(Rx));
}


void Assembler::cmpstr(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("cmpstr R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::div0s(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x7 << 0));
  asm_output("div0s R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::div0u() {
  emit((0x0 << 12) | (0x0 << 8) | (0x1 << 4) | (0x9 << 0));
  asm_output("div0u");
}


void Assembler::div1(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x4 << 0));
  asm_output("div1 R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::extsb(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("extsb R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::extsw(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("extsw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::extub(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("extub R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::extuw(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("extuw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::icbi_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xE << 4) | (0x3 << 0));
  asm_output("icbi_indRx R%d", REGNUM(Rx));
}


void Assembler::jmp_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0xB << 0));
  asm_output("jmp_indRx R%d", REGNUM(Rx));
}


void Assembler::jsr_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0xB << 0));
  asm_output("jsr_indRx R%d", REGNUM(Rx));
}


void Assembler::ldc_SR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0xE << 0));
  asm_output("ldc_SR R%d", REGNUM(Rx));
}


void Assembler::ldc_GBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0xE << 0));
  asm_output("ldc_GBR R%d", REGNUM(Rx));
}


void Assembler::ldc_SGR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0xA << 0));
  asm_output("ldc_SGR R%d", REGNUM(Rx));
}


void Assembler::ldc_VBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0xE << 0));
  asm_output("ldc_VBR R%d", REGNUM(Rx));
}


void Assembler::ldc_SSR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0xE << 0));
  asm_output("ldc_SSR R%d", REGNUM(Rx));
}


void Assembler::ldc_SPC(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x4 << 4) | (0xE << 0));
  asm_output("ldc_SPC R%d", REGNUM(Rx));
}


void Assembler::ldc_DBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xF << 4) | (0xA << 0));
  asm_output("ldc_DBR R%d", REGNUM(Rx));
}


#define SH4_CHECK_RANGE_ldc_bank(imm) ((imm) >= 0 && (imm) <= 7)

#define FITS_SH4_ldc_bank(imm) (SH4_CHECK_RANGE_ldc_bank(imm))

void Assembler::ldc_bank(Register Rx, int imm) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_ldc_bank(imm));
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0x7) << 4) | (0xE << 0));
  asm_output("ldc_bank R%d, %d", REGNUM(Rx), imm);
}


void Assembler::ldcl_incRx_SR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_SR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_GBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_GBR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_VBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_VBR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_SGR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x6 << 0));
  asm_output("ldcl_incRx_SGR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_SSR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_SSR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_SPC(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x4 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_SPC R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_DBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xF << 4) | (0x6 << 0));
  asm_output("ldcl_incRx_DBR R%d", REGNUM(Rx));
}


#define SH4_CHECK_RANGE_ldcl_incRx_bank(imm) ((imm) >= 0 && (imm) <= 7)

#define FITS_SH4_ldcl_incRx_bank(imm) (SH4_CHECK_RANGE_ldcl_incRx_bank(imm))

void Assembler::ldcl_incRx_bank(Register Rx, int imm) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_ldcl_incRx_bank(imm));
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_bank R%d, %d", REGNUM(Rx), imm);
}


void Assembler::lds_MACH(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0xA << 0));
  asm_output("lds_MACH R%d", REGNUM(Rx));
}


void Assembler::lds_MACL(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0xA << 0));
  asm_output("lds_MACL R%d", REGNUM(Rx));
}


void Assembler::lds_PR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0xA << 0));
  asm_output("lds_PR R%d", REGNUM(Rx));
}


void Assembler::lds_FPUL(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x5 << 4) | (0xA << 0));
  asm_output("lds_FPUL R%d", REGNUM(Ry));
}


void Assembler::lds_FPSCR(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x6 << 4) | (0xA << 0));
  asm_output("lds_FPSCR R%d", REGNUM(Ry));
}


void Assembler::ldsl_incRx_MACH(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x6 << 0));
  asm_output("ldsl_incRx_MACH R%d", REGNUM(Rx));
}


void Assembler::ldsl_incRx_MACL(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x6 << 0));
  asm_output("ldsl_incRx_MACL R%d", REGNUM(Rx));
}


void Assembler::ldsl_incRx_PR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x6 << 0));
  asm_output("ldsl_incRx_PR R%d", REGNUM(Rx));
}


void Assembler::ldsl_incRy_FPUL(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x5 << 4) | (0x6 << 0));
  asm_output("ldsl_incRy_FPUL R%d", REGNUM(Ry));
}


void Assembler::ldsl_incRy_FPSCR(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x6 << 4) | (0x6 << 0));
  asm_output("ldsl_incRy_FPSCR R%d", REGNUM(Ry));
}


void Assembler::ldtlb() {
  emit((0x0 << 12) | (0x0 << 8) | (0x3 << 4) | (0x8 << 0));
  asm_output("ldtlb");
}


void Assembler::macw_incRy_incRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("macw_incRy_incRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_mov_imm(imm) ((imm) >= -128 && (imm) <= 127)

#define FITS_SH4_mov_imm(imm) (SH4_CHECK_RANGE_mov_imm(imm))

void Assembler::mov_imm(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_mov_imm(imm));
  emit((0xE << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0xFF) << 0));
  asm_output("mov_imm %d, R%d", imm, REGNUM(Rx));
}


void Assembler::mov(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x3 << 0));
  asm_output("mov R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_dispR0Rx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_decRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_indRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x0 << 0));
  asm_output("movb_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movb_dispRy_R0(imm) ((imm) >= 0 && (imm) <= 15)

#define FITS_SH4_movb_dispRy_R0(imm) (SH4_CHECK_RANGE_movb_dispRy_R0(imm))

void Assembler::movb_dispRy_R0(int imm, Register Ry) {
  ASSERT(REGNUM(Ry) <= 15 && SH4_CHECK_RANGE_movb_dispRy_R0(imm));
  emit((0x8 << 12) | (0x4 << 8) | ((REGNUM(Ry) & 0xF) << 4) | ((imm & 0xF) << 0));
  asm_output("movb_dispRy_R0 %d, R%d", imm, REGNUM(Ry));
}


#define SH4_CHECK_RANGE_movb_dispGBR_R0(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_movb_dispGBR_R0(imm) (SH4_CHECK_RANGE_movb_dispGBR_R0(imm))

void Assembler::movb_dispGBR_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_movb_dispGBR_R0(imm));
  emit((0xC << 12) | (0x4 << 8) | ((imm & 0xFF) << 0));
  asm_output("movb_dispGBR_R0 %d", imm);
}


void Assembler::movb_dispR0Ry(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("movb_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_incRy(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_indRy(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x0 << 0));
  asm_output("movb_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movb_R0_dispRx(imm) ((imm) >= 0 && (imm) <= 15)

#define FITS_SH4_movb_R0_dispRx(imm) (SH4_CHECK_RANGE_movb_R0_dispRx(imm))

void Assembler::movb_R0_dispRx(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_movb_R0_dispRx(imm));
  emit((0x8 << 12) | (0x0 << 8) | ((REGNUM(Rx) & 0xF) << 4) | ((imm & 0xF) << 0));
  asm_output("movb_R0_dispRx %d, R%d", imm, REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movb_R0_dispGBR(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_movb_R0_dispGBR(imm) (SH4_CHECK_RANGE_movb_R0_dispGBR(imm))

void Assembler::movb_R0_dispGBR(int imm) {
  ASSERT(SH4_CHECK_RANGE_movb_R0_dispGBR(imm));
  emit((0xC << 12) | (0x0 << 8) | ((imm & 0xFF) << 0));
  asm_output("movb_R0_dispGBR %d", imm);
}


#define SH4_CHECK_RANGE_movl_dispRx(imm) ((imm) >= 0 && (imm) <= 60)

#define SH4_CHECK_ALIGN_movl_dispRx(imm) (((imm) & 0x3) == 0)

#define FITS_SH4_movl_dispRx(imm) (SH4_CHECK_RANGE_movl_dispRx(imm) && SH4_CHECK_ALIGN_movl_dispRx(imm))

void Assembler::movl_dispRx(Register Ry, int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15 && SH4_CHECK_RANGE_movl_dispRx(imm) && SH4_CHECK_ALIGN_movl_dispRx(imm));
  emit((0x1 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (((imm & 0x3C) >> 2) << 0));
  asm_output("movl_dispRx R%d, %d, R%d", REGNUM(Ry), imm, REGNUM(Rx));
}


void Assembler::movl_dispR0Rx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_decRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_indRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x2 << 0));
  asm_output("movl_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movl_dispRy(imm) ((imm) >= 0 && (imm) <= 60)

#define SH4_CHECK_ALIGN_movl_dispRy(imm) (((imm) & 0x3) == 0)

#define FITS_SH4_movl_dispRy(imm) (SH4_CHECK_RANGE_movl_dispRy(imm) && SH4_CHECK_ALIGN_movl_dispRy(imm))

void Assembler::movl_dispRy(int imm, Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15 && SH4_CHECK_RANGE_movl_dispRy(imm) && SH4_CHECK_ALIGN_movl_dispRy(imm));
  emit((0x5 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (((imm & 0x3C) >> 2) << 0));
  asm_output("movl_dispRy %d, R%d, R%d", imm, REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movl_dispGBR_R0(imm) ((imm) >= 0 && (imm) <= 1020)

#define SH4_CHECK_ALIGN_movl_dispGBR_R0(imm) (((imm) & 0x3) == 0)

#define FITS_SH4_movl_dispGBR_R0(imm) (SH4_CHECK_RANGE_movl_dispGBR_R0(imm) && SH4_CHECK_ALIGN_movl_dispGBR_R0(imm))

void Assembler::movl_dispGBR_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_movl_dispGBR_R0(imm) && SH4_CHECK_ALIGN_movl_dispGBR_R0(imm));
  emit((0xC << 12) | (0x6 << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_dispGBR_R0 %d", imm);
}


#define SH4_CHECK_RANGE_movl_dispPC(imm) ((imm) >= 0 && (imm) <= 1020)

#define SH4_CHECK_ALIGN_movl_dispPC(imm) (((imm) & 0x3) == 0)

#define FITS_SH4_movl_dispPC(imm) (SH4_CHECK_RANGE_movl_dispPC(imm) && SH4_CHECK_ALIGN_movl_dispPC(imm))

void Assembler::movl_dispPC(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_movl_dispPC(imm) && SH4_CHECK_ALIGN_movl_dispPC(imm));
  emit((0xD << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_dispPC %d, R%d", imm, REGNUM(Rx));
}


void Assembler::movl_dispR0Ry(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("movl_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_incRy(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_indRy(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x2 << 0));
  asm_output("movl_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movl_R0_dispGBR(imm) ((imm) >= 0 && (imm) <= 1020)

#define SH4_CHECK_ALIGN_movl_R0_dispGBR(imm) (((imm) & 0x3) == 0)

#define FITS_SH4_movl_R0_dispGBR(imm) (SH4_CHECK_RANGE_movl_R0_dispGBR(imm) && SH4_CHECK_ALIGN_movl_R0_dispGBR(imm))

void Assembler::movl_R0_dispGBR(int imm) {
  ASSERT(SH4_CHECK_RANGE_movl_R0_dispGBR(imm) && SH4_CHECK_ALIGN_movl_R0_dispGBR(imm));
  emit((0xC << 12) | (0x2 << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_R0_dispGBR %d", imm);
}


void Assembler::movw_dispR0Rx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_decRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_indRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x1 << 0));
  asm_output("movw_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movw_dispRy_R0(imm) ((imm) >= 0 && (imm) <= 30)

#define SH4_CHECK_ALIGN_movw_dispRy_R0(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_movw_dispRy_R0(imm) (SH4_CHECK_RANGE_movw_dispRy_R0(imm) && SH4_CHECK_ALIGN_movw_dispRy_R0(imm))

void Assembler::movw_dispRy_R0(int imm, Register Ry) {
  ASSERT(REGNUM(Ry) <= 15 && SH4_CHECK_RANGE_movw_dispRy_R0(imm) && SH4_CHECK_ALIGN_movw_dispRy_R0(imm));
  emit((0x8 << 12) | (0x5 << 8) | ((REGNUM(Ry) & 0xF) << 4) | (((imm & 0x1E) >> 1) << 0));
  asm_output("movw_dispRy_R0 %d, R%d", imm, REGNUM(Ry));
}


#define SH4_CHECK_RANGE_movw_dispGBR_R0(imm) ((imm) >= 0 && (imm) <= 510)

#define SH4_CHECK_ALIGN_movw_dispGBR_R0(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_movw_dispGBR_R0(imm) (SH4_CHECK_RANGE_movw_dispGBR_R0(imm) && SH4_CHECK_ALIGN_movw_dispGBR_R0(imm))

void Assembler::movw_dispGBR_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_movw_dispGBR_R0(imm) && SH4_CHECK_ALIGN_movw_dispGBR_R0(imm));
  emit((0xC << 12) | (0x5 << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("movw_dispGBR_R0 %d", imm);
}


#define SH4_CHECK_RANGE_movw_dispPC(imm) ((imm) >= 0 && (imm) <= 510)

#define SH4_CHECK_ALIGN_movw_dispPC(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_movw_dispPC(imm) (SH4_CHECK_RANGE_movw_dispPC(imm) && SH4_CHECK_ALIGN_movw_dispPC(imm))

void Assembler::movw_dispPC(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_movw_dispPC(imm) && SH4_CHECK_ALIGN_movw_dispPC(imm));
  emit((0x9 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("movw_dispPC %d, R%d", imm, REGNUM(Rx));
}


void Assembler::movw_dispR0Ry(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("movw_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_incRy(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_indRy(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x1 << 0));
  asm_output("movw_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movw_R0_dispRx(imm) ((imm) >= 0 && (imm) <= 30)

#define SH4_CHECK_ALIGN_movw_R0_dispRx(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_movw_R0_dispRx(imm) (SH4_CHECK_RANGE_movw_R0_dispRx(imm) && SH4_CHECK_ALIGN_movw_R0_dispRx(imm))

void Assembler::movw_R0_dispRx(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_movw_R0_dispRx(imm) && SH4_CHECK_ALIGN_movw_R0_dispRx(imm));
  emit((0x8 << 12) | (0x1 << 8) | ((REGNUM(Rx) & 0xF) << 4) | (((imm & 0x1E) >> 1) << 0));
  asm_output("movw_R0_dispRx %d, R%d", imm, REGNUM(Rx));
}


#define SH4_CHECK_RANGE_movw_R0_dispGBR(imm) ((imm) >= 0 && (imm) <= 510)

#define SH4_CHECK_ALIGN_movw_R0_dispGBR(imm) (((imm) & 0x1) == 0)

#define FITS_SH4_movw_R0_dispGBR(imm) (SH4_CHECK_RANGE_movw_R0_dispGBR(imm) && SH4_CHECK_ALIGN_movw_R0_dispGBR(imm))

void Assembler::movw_R0_dispGBR(int imm) {
  ASSERT(SH4_CHECK_RANGE_movw_R0_dispGBR(imm) && SH4_CHECK_ALIGN_movw_R0_dispGBR(imm));
  emit((0xC << 12) | (0x1 << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("movw_R0_dispGBR %d", imm);
}


#define SH4_CHECK_RANGE_mova_dispPC_R0(imm) ((imm) >= 0 && (imm) <= 1020)

#define SH4_CHECK_ALIGN_mova_dispPC_R0(imm) (((imm) & 0x3) == 0)

#define FITS_SH4_mova_dispPC_R0(imm) (SH4_CHECK_RANGE_mova_dispPC_R0(imm) && SH4_CHECK_ALIGN_mova_dispPC_R0(imm))

void Assembler::mova_dispPC_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_mova_dispPC_R0(imm) && SH4_CHECK_ALIGN_mova_dispPC_R0(imm));
  emit((0xC << 12) | (0x7 << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("mova_dispPC_R0 %d", imm);
}


void Assembler::movcal_R0_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xC << 4) | (0x3 << 0));
  asm_output("movcal_R0_indRx R%d", REGNUM(Rx));
}


void Assembler::movcol_R0_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x7 << 4) | (0x3 << 0));
  asm_output("movcol_R0_indRx R%d", REGNUM(Rx));
}


void Assembler::movlil_indRy_R0(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x6 << 4) | (0x3 << 0));
  asm_output("movlil_indRy_R0 R%d", REGNUM(Ry));
}


void Assembler::movt(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x9 << 0));
  asm_output("movt R%d", REGNUM(Rx));
}


void Assembler::movual_indRy_R0(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0xA << 4) | (0x9 << 0));
  asm_output("movual_indRy_R0 R%d", REGNUM(Ry));
}


void Assembler::movual_incRy_R0(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0xE << 4) | (0x9 << 0));
  asm_output("movual_incRy_R0 R%d", REGNUM(Ry));
}


void Assembler::mulsw(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("mulsw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::muls(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("muls R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::mull(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x7 << 0));
  asm_output("mull R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::muluw(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("muluw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::mulu(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("mulu R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::neg(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xB << 0));
  asm_output("neg R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::negc(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xA << 0));
  asm_output("negc R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::nop() {
  emit((0x0 << 12) | (0x0 << 8) | (0x0 << 4) | (0x9 << 0));
  asm_output("nop");
}


void Assembler::not_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x7 << 0));
  asm_output("not R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::ocbi_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x9 << 4) | (0x3 << 0));
  asm_output("ocbi_indRx R%d", REGNUM(Rx));
}


void Assembler::ocbp_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xA << 4) | (0x3 << 0));
  asm_output("ocbp_indRx R%d", REGNUM(Rx));
}


void Assembler::ocbwb_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xB << 4) | (0x3 << 0));
  asm_output("ocbwb_indRx R%d", REGNUM(Rx));
}


#define SH4_CHECK_RANGE_or_imm_R0(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_or_imm_R0(imm) (SH4_CHECK_RANGE_or_imm_R0(imm))

void Assembler::or_imm_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_or_imm_R0(imm));
  emit((0xC << 12) | (0xB << 8) | ((imm & 0xFF) << 0));
  asm_output("or_imm_R0 %d", imm);
}


void Assembler::or_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xB << 0));
  asm_output("or R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_orb_imm_dispR0GBR(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_orb_imm_dispR0GBR(imm) (SH4_CHECK_RANGE_orb_imm_dispR0GBR(imm))

void Assembler::orb_imm_dispR0GBR(int imm) {
  ASSERT(SH4_CHECK_RANGE_orb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xF << 8) | ((imm & 0xFF) << 0));
  asm_output("orb_imm_dispR0GBR %d", imm);
}


void Assembler::pref_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x8 << 4) | (0x3 << 0));
  asm_output("pref_indRx R%d", REGNUM(Rx));
}


void Assembler::prefi_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xD << 4) | (0x3 << 0));
  asm_output("prefi_indRx R%d", REGNUM(Rx));
}


void Assembler::rotcl(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x4 << 0));
  asm_output("rotcl R%d", REGNUM(Rx));
}


void Assembler::rotcr(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x5 << 0));
  asm_output("rotcr R%d", REGNUM(Rx));
}


void Assembler::rotl(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x4 << 0));
  asm_output("rotl R%d", REGNUM(Rx));
}


void Assembler::rotr(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x5 << 0));
  asm_output("rotr R%d", REGNUM(Rx));
}


void Assembler::rte() {
  emit((0x0 << 12) | (0x0 << 8) | (0x2 << 4) | (0xB << 0));
  asm_output("rte");
}


void Assembler::rts() {
  emit((0x0 << 12) | (0x0 << 8) | (0x0 << 4) | (0xB << 0));
  asm_output("rts");
}


void Assembler::sets() {
  emit((0x0 << 12) | (0x0 << 8) | (0x5 << 4) | (0x8 << 0));
  asm_output("sets");
}


void Assembler::sett() {
  emit((0x0 << 12) | (0x0 << 8) | (0x1 << 4) | (0x8 << 0));
  asm_output("sett");
}


void Assembler::shad(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("shad R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::shld(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("shld R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::shal(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x0 << 0));
  asm_output("shal R%d", REGNUM(Rx));
}


void Assembler::shar(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x1 << 0));
  asm_output("shar R%d", REGNUM(Rx));
}


void Assembler::shll(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x0 << 0));
  asm_output("shll R%d", REGNUM(Rx));
}


void Assembler::shll16(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x8 << 0));
  asm_output("shll16 R%d", REGNUM(Rx));
}


void Assembler::shll2(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x8 << 0));
  asm_output("shll2 R%d", REGNUM(Rx));
}


void Assembler::shll8(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x8 << 0));
  asm_output("shll8 R%d", REGNUM(Rx));
}


void Assembler::shlr(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x1 << 0));
  asm_output("shlr R%d", REGNUM(Rx));
}


void Assembler::shlr16(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x9 << 0));
  asm_output("shlr16 R%d", REGNUM(Rx));
}


void Assembler::shlr2(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x9 << 0));
  asm_output("shlr2 R%d", REGNUM(Rx));
}


void Assembler::shlr8(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x9 << 0));
  asm_output("shlr8 R%d", REGNUM(Rx));
}


void Assembler::sleep() {
  emit((0x0 << 12) | (0x0 << 8) | (0x1 << 4) | (0xB << 0));
  asm_output("sleep");
}


void Assembler::stc_SR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x2 << 0));
  asm_output("stc_SR R%d", REGNUM(Rx));
}


void Assembler::stc_GBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x2 << 0));
  asm_output("stc_GBR R%d", REGNUM(Rx));
}


void Assembler::stc_VBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x2 << 0));
  asm_output("stc_VBR R%d", REGNUM(Rx));
}


void Assembler::stc_SSR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x2 << 0));
  asm_output("stc_SSR R%d", REGNUM(Rx));
}


void Assembler::stc_SPC(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x4 << 4) | (0x2 << 0));
  asm_output("stc_SPC R%d", REGNUM(Rx));
}


void Assembler::stc_SGR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0xA << 0));
  asm_output("stc_SGR R%d", REGNUM(Rx));
}


void Assembler::stc_DBR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xF << 4) | (0xA << 0));
  asm_output("stc_DBR R%d", REGNUM(Rx));
}


#define SH4_CHECK_RANGE_stc_bank(imm) ((imm) >= 0 && (imm) <= 7)

#define FITS_SH4_stc_bank(imm) (SH4_CHECK_RANGE_stc_bank(imm))

void Assembler::stc_bank(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_stc_bank(imm));
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x2 << 0));
  asm_output("stc_bank %d, R%d", imm, REGNUM(Rx));
}


void Assembler::stcl_SR_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x3 << 0));
  asm_output("stcl_SR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_VBR_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x3 << 0));
  asm_output("stcl_VBR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_SSR_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x3 << 0));
  asm_output("stcl_SSR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_SPC_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x4 << 4) | (0x3 << 0));
  asm_output("stcl_SPC_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_GBR_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x3 << 0));
  asm_output("stcl_GBR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_SGR_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x2 << 0));
  asm_output("stcl_SGR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_DBR_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xF << 4) | (0x2 << 0));
  asm_output("stcl_DBR_decRx R%d", REGNUM(Rx));
}


#define SH4_CHECK_RANGE_stcl_bank_decRx(imm) ((imm) >= 0 && (imm) <= 7)

#define FITS_SH4_stcl_bank_decRx(imm) (SH4_CHECK_RANGE_stcl_bank_decRx(imm))

void Assembler::stcl_bank_decRx(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_stcl_bank_decRx(imm));
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x3 << 0));
  asm_output("stcl_bank_decRx %d, R%d", imm, REGNUM(Rx));
}


void Assembler::sts_MACH(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0xA << 0));
  asm_output("sts_MACH R%d", REGNUM(Rx));
}


void Assembler::sts_MACL(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0xA << 0));
  asm_output("sts_MACL R%d", REGNUM(Rx));
}


void Assembler::sts_PR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0xA << 0));
  asm_output("sts_PR R%d", REGNUM(Rx));
}


void Assembler::sts_FPUL(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x5 << 4) | (0xA << 0));
  asm_output("sts_FPUL R%d", REGNUM(Rx));
}


void Assembler::sts_FPSCR(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x6 << 4) | (0xA << 0));
  asm_output("sts_FPSCR R%d", REGNUM(Rx));
}


void Assembler::stsl_MACH_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x2 << 0));
  asm_output("stsl_MACH_decRx R%d", REGNUM(Rx));
}


void Assembler::stsl_MACL_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x2 << 0));
  asm_output("stsl_MACL_decRx R%d", REGNUM(Rx));
}


void Assembler::stsl_PR_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x2 << 0));
  asm_output("stsl_PR_decRx R%d", REGNUM(Rx));
}


void Assembler::stsl_FPUL_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x5 << 4) | (0x2 << 0));
  asm_output("stsl_FPUL_decRx R%d", REGNUM(Rx));
}


void Assembler::stsl_FPSCR_decRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x6 << 4) | (0x2 << 0));
  asm_output("stsl_FPSCR_decRx R%d", REGNUM(Rx));
}


void Assembler::sub(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("sub R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::subc(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xA << 0));
  asm_output("subc R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::subv(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xB << 0));
  asm_output("subv R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::swapb(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("swapb R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::swapw(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("swapw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::synco() {
  emit((0x0 << 12) | (0x0 << 8) | (0xA << 4) | (0xB << 0));
  asm_output("synco");
}


void Assembler::tasb_indRx(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0xB << 0));
  asm_output("tasb_indRx R%d", REGNUM(Rx));
}


#define SH4_CHECK_RANGE_trapa_imm(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_trapa_imm(imm) (SH4_CHECK_RANGE_trapa_imm(imm))

void Assembler::trapa_imm(int imm) {
  ASSERT(SH4_CHECK_RANGE_trapa_imm(imm));
  emit((0xC << 12) | (0x3 << 8) | ((imm & 0xFF) << 0));
  asm_output("trapa_imm %d", imm);
}


#define SH4_CHECK_RANGE_tst_imm_R0(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_tst_imm_R0(imm) (SH4_CHECK_RANGE_tst_imm_R0(imm))

void Assembler::tst_imm_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_tst_imm_R0(imm));
  emit((0xC << 12) | (0x8 << 8) | ((imm & 0xFF) << 0));
  asm_output("tst_imm_R0 %d", imm);
}


void Assembler::tst(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("tst R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_tstb_imm_dispR0GBR(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_tstb_imm_dispR0GBR(imm) (SH4_CHECK_RANGE_tstb_imm_dispR0GBR(imm))

void Assembler::tstb_imm_dispR0GBR(int imm) {
  ASSERT(SH4_CHECK_RANGE_tstb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xC << 8) | ((imm & 0xFF) << 0));
  asm_output("tstb_imm_dispR0GBR %d", imm);
}


#define SH4_CHECK_RANGE_xor_imm_R0(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_xor_imm_R0(imm) (SH4_CHECK_RANGE_xor_imm_R0(imm))

void Assembler::xor_imm_R0(int imm) {
  ASSERT(SH4_CHECK_RANGE_xor_imm_R0(imm));
  emit((0xC << 12) | (0xA << 8) | ((imm & 0xFF) << 0));
  asm_output("xor_imm_R0 %d", imm);
}


void Assembler::xor_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xA << 0));
  asm_output("xor R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


#define SH4_CHECK_RANGE_xorb_imm_dispR0GBR(imm) ((imm) >= 0 && (imm) <= 255)

#define FITS_SH4_xorb_imm_dispR0GBR(imm) (SH4_CHECK_RANGE_xorb_imm_dispR0GBR(imm))

void Assembler::xorb_imm_dispR0GBR(int imm) {
  ASSERT(SH4_CHECK_RANGE_xorb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xE << 8) | ((imm & 0xFF) << 0));
  asm_output("xorb_imm_dispR0GBR %d", imm);
}


void Assembler::xtrct(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("xtrct R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::dt(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x0 << 0));
  asm_output("dt R%d", REGNUM(Rx));
}


void Assembler::dmulsl(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("dmulsl R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::dmulul(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x5 << 0));
  asm_output("dmulul R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::macl_incRy_incRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("macl_incRy_incRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::braf(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x3 << 0));
  asm_output("braf R%d", REGNUM(Rx));
}


void Assembler::bsrf(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x3 << 0));
  asm_output("bsrf R%d", REGNUM(Rx));
}


void Assembler::fabs(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x5 << 4) | (0xD << 0));
  asm_output("fabs R%d", REGNUM(Rx));
}


void Assembler::fabs_double(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x5 << 4) | (0xD << 0));
  asm_output("fabs_double R%d", REGNUM(Rx));
}


void Assembler::fadd(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x0 << 0));
  asm_output("fadd R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fadd_double(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x0 << 0));
  asm_output("fadd_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcmpeq(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x4 << 0));
  asm_output("fcmpeq R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcmpeq_double(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x4 << 0));
  asm_output("fcmpeq_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcmpgt(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x5 << 0));
  asm_output("fcmpgt R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcmpgt_double(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x5 << 0));
  asm_output("fcmpgt_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcnvds_double_FPUL(Register Rx) {
  ASSERT(!((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0xB << 4) | (0xD << 0));
  asm_output("fcnvds_double_FPUL R%d", REGNUM(Rx));
}


void Assembler::fcnvsd_FPUL_double(Register Rx) {
  ASSERT(!((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0xA << 4) | (0xD << 0));
  asm_output("fcnvsd_FPUL_double R%d", REGNUM(Rx));
}


void Assembler::fdiv(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x3 << 0));
  asm_output("fdiv R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fdiv_double(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x3 << 0));
  asm_output("fdiv_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fipr(Register Ry, Register Rx) {
  ASSERT(!(((REGNUM(Rx) - 16) & 0x3) || ((REGNUM(Ry) - 16) & 0x3)));
  emit((0xF << 12) | (((((REGNUM(Rx) - 16) & 0xF) << 2) | (((REGNUM(Ry) - 16) & 0xF) >> 2)) << 8) | (0xE << 4) | (0xD << 0));
  asm_output("fipr R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fldi0(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x8 << 4) | (0xD << 0));
  asm_output("fldi0 R%d", REGNUM(Rx));
}


void Assembler::fldi1(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x9 << 4) | (0xD << 0));
  asm_output("fldi1 R%d", REGNUM(Rx));
}


void Assembler::flds_FPUL(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x1 << 4) | (0xD << 0));
  asm_output("flds_FPUL R%d", REGNUM(Rx));
}


void Assembler::float_FPUL(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x2 << 4) | (0xD << 0));
  asm_output("float_FPUL R%d", REGNUM(Rx));
}


void Assembler::float_FPUL_double(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x2 << 4) | (0xD << 0));
  asm_output("float_FPUL_double R%d", REGNUM(Rx));
}


void Assembler::fmac(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xE << 0));
  asm_output("fmac R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xC << 0));
  asm_output("fmov R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_Xdouble_Xdouble(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xC << 0));
  asm_output("fmov_Xdouble_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_indRy(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmov_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_indRy_Xdouble(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmov_indRy_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_indRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xA << 0));
  asm_output("fmov_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_Xdouble_indRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xA << 0));
  asm_output("fmov_Xdouble_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_incRy(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmov_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_incRy_Xdouble(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmov_incRy_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_decRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xB << 0));
  asm_output("fmov_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_Xdouble_decRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xB << 0));
  asm_output("fmov_Xdouble_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_dispR0Ry(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmov_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_dispR0Ry_Xdouble(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmov_dispR0Ry_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_dispR0Rx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmov_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_Xdouble_dispR0Rx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmov_Xdouble_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_indRy_Xdouble(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmovd_indRy_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_Xdouble_indRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xA << 0));
  asm_output("fmovd_Xdouble_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_incRy_Xdouble(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmovd_incRy_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_Xdouble_decRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xB << 0));
  asm_output("fmovd_Xdouble_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_dispR0Ry_Xdouble(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmovd_dispR0Ry_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_Xdouble_dispR0Rx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmovd_Xdouble_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_indRy(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmovs_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_indRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xA << 0));
  asm_output("fmovs_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_incRy(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmovs_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_decRx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xB << 0));
  asm_output("fmovs_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_dispR0Ry(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmovs_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_dispR0Rx(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmovs_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmul(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x2 << 0));
  asm_output("fmul R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmul_double(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x2 << 0));
  asm_output("fmul_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fneg(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x4 << 4) | (0xD << 0));
  asm_output("fneg R%d", REGNUM(Rx));
}


void Assembler::fneg_double(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x4 << 4) | (0xD << 0));
  asm_output("fneg_double R%d", REGNUM(Rx));
}


void Assembler::fpchg() {
  emit((0xF << 12) | (0x7 << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fpchg");
}


void Assembler::frchg() {
  emit((0xF << 12) | (0xB << 8) | (0xF << 4) | (0xD << 0));
  asm_output("frchg");
}


void Assembler::fsca_FPUL_double(Register Rx) {
  ASSERT(!((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fsca_FPUL_double R%d", REGNUM(Rx));
}


void Assembler::fschg() {
  emit((0xF << 12) | (0x3 << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fschg");
}


void Assembler::fsqrt(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x6 << 4) | (0xD << 0));
  asm_output("fsqrt R%d", REGNUM(Rx));
}


void Assembler::fsqrt_double(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x6 << 4) | (0xD << 0));
  asm_output("fsqrt_double R%d", REGNUM(Rx));
}


void Assembler::fsrra(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x7 << 4) | (0xD << 0));
  asm_output("fsrra R%d", REGNUM(Rx));
}


void Assembler::fsts_FPUL(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x0 << 4) | (0xD << 0));
  asm_output("fsts_FPUL R%d", REGNUM(Rx));
}


void Assembler::fsub(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x1 << 0));
  asm_output("fsub R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fsub_double(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x1 << 0));
  asm_output("fsub_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::ftrc_FPUL(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x3 << 4) | (0xD << 0));
  asm_output("ftrc_FPUL R%d", REGNUM(Rx));
}


void Assembler::ftrc_double_FPUL(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x3 << 4) | (0xD << 0));
  asm_output("ftrc_double_FPUL R%d", REGNUM(Rx));
}


void Assembler::ftrv(Register Rx) {
  ASSERT(!((REGNUM(Rx) - 16) & 0x3));
  emit((0xF << 12) | (((((REGNUM(Rx) - 16) & 0xF) << 2) | 0x1) << 8) | (0xF << 4) | (0xD << 0));
  asm_output("ftrv R%d", REGNUM(Rx));
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
