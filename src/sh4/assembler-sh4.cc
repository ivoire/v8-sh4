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

#include "checks-sh4.h"

namespace v8 {
namespace internal {

#ifdef DEBUG
bool CpuFeatures::initialized_ = false;
#endif
unsigned CpuFeatures::supported_ = 0;
unsigned CpuFeatures::found_by_runtime_probing_ = 0;


void CpuFeatures::Probe() {
  ASSERT(!initialized_);
#ifdef DEBUG
  initialized_ = true;
#endif
  // FIXME(STM): define the right features to accept or not
  if (Serializer::enabled()) {
    supported_ |= OS::CpuFeaturesImpliedByPlatform();
    return;  // No features if we might serialize.
  }
}


void Assembler::Align(int m) {
  ASSERT(m >= 4 && IsPowerOf2(m));
  while ((pc_offset() & (m - 1)) != 0) {
    nop_();
  }
}


static const int kMinimalBufferSize = 4*KB;

Assembler::Assembler(Isolate* arg_isolate, void* buffer, int buffer_size)
    : AssemblerBase(arg_isolate),
      positions_recorder_(this),
      emit_debug_code_(FLAG_debug_code) {
  // FIXME(STM): finish this class
  if (buffer == NULL) {
    // Do our own buffer management.
    if (buffer_size <= kMinimalBufferSize) {
      buffer_size = kMinimalBufferSize;

      if (isolate()->assembler_spare_buffer() != NULL) {
        buffer = isolate()->assembler_spare_buffer();
        isolate()->set_assembler_spare_buffer(NULL);
      }
    }
    if (buffer == NULL) {
      buffer_ = NewArray<byte>(buffer_size);
    } else {
      buffer_ = static_cast<byte*>(buffer);
    }
    buffer_size_ = buffer_size;
    own_buffer_ = true;

  } else {
    // Use externally provided buffer instead.
    ASSERT(buffer_size > 0);
    buffer_ = static_cast<byte*>(buffer);
    buffer_size_ = buffer_size;
    own_buffer_ = false;
  }

  // Setup buffer pointers.
  ASSERT(buffer_ != NULL);
  pc_ = buffer_;
  reloc_info_writer.Reposition(buffer_ + buffer_size, pc_);
}


void Assembler::GetCode(CodeDesc* desc) {
  // Emit the constant pool if needed
  // FIXME(STM)

  desc->buffer = buffer_;
  desc->buffer_size = buffer_size_;
  desc->instr_size = pc_offset();
  desc->reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();
  desc->origin = this;
}


void Assembler::RecordComment(const char* msg, bool force) {
  if (FLAG_code_comments) {
    CheckBuffer();
    RecordRelocInfo(RelocInfo::COMMENT, reinterpret_cast<intptr_t>(msg));
  }
}


void Assembler::GrowBuffer() {
  if (!own_buffer_) FATAL("external code buffer is too small");

  // Compute new buffer size.
  CodeDesc desc;  // the new buffer
  if (buffer_size_ < 4*KB) {
    desc.buffer_size = 4*KB;
  } else if (buffer_size_ < 1*MB) {
    desc.buffer_size = 2*buffer_size_;
  } else {
    desc.buffer_size = buffer_size_ + 1*MB;
  }
  CHECK_GT(desc.buffer_size, 0);  // no overflow

  // Setup new buffer.
  desc.buffer = NewArray<byte>(desc.buffer_size);

  desc.instr_size = pc_offset();
  desc.reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();

  // Copy the data.
  int pc_delta = desc.buffer - buffer_;
  int rc_delta = (desc.buffer + desc.buffer_size) - (buffer_ + buffer_size_);
  memmove(desc.buffer, buffer_, desc.instr_size);
  memmove(reloc_info_writer.pos() + rc_delta,
          reloc_info_writer.pos(), desc.reloc_size);

  // Switch buffers.
  if (isolate()->assembler_spare_buffer() == NULL &&
      buffer_size_ == kMinimalBufferSize) {
    isolate()->set_assembler_spare_buffer(buffer_);
  } else {
    DeleteArray(buffer_);
  }
  buffer_ = desc.buffer;
  buffer_size_ = desc.buffer_size;
  pc_ += pc_delta;
  if (last_pc_ != NULL) {
    last_pc_ += pc_delta;
  }
  reloc_info_writer.Reposition(reloc_info_writer.pos() + rc_delta,
                               reloc_info_writer.last_pc() + pc_delta);

  // Relocate runtime entries.
  for (RelocIterator it(desc); !it.done(); it.next()) {
    RelocInfo::Mode rmode = it.rinfo()->rmode();
    if (rmode == RelocInfo::RUNTIME_ENTRY) {
      int32_t* p = reinterpret_cast<int32_t*>(it.rinfo()->pc());
      *p -= pc_delta;  // relocate entry
    } else if (rmode == RelocInfo::INTERNAL_REFERENCE) {
      int32_t* p = reinterpret_cast<int32_t*>(it.rinfo()->pc());
      if (*p != 0) {  // 0 means uninitialized.
        *p += pc_delta;
      }
    }
  }
}


void Assembler::RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data) {
  RelocInfo rinfo(pc_, rmode, data);  // we do not try to reuse pool constants
  if (rmode >= RelocInfo::JS_RETURN && rmode <= RelocInfo::DEBUG_BREAK_SLOT) {
    // Adjust code for new modes.
    ASSERT(RelocInfo::IsDebugBreakSlot(rmode)
           || RelocInfo::IsJSReturn(rmode)
           || RelocInfo::IsComment(rmode)
           || RelocInfo::IsPosition(rmode));
    // These modes do not need an entry in the constant pool.
  } else {
// FIXME(STM): implement the constant pool !
//  ASSERT(num_prinfo_ < kMaxNumPRInfo);
//  prinfo_[num_prinfo_++] = rinfo;
//  // Make sure the constant pool is not emitted in place of the next
//  // instruction for which we just recorded relocation info.
//    BlockConstPoolBefore(pc_offset() + kInstrSize);
  }
  if (rinfo.rmode() != RelocInfo::NONE) {
    // Don't record external references unless the heap will be serialized.
    if (rmode == RelocInfo::EXTERNAL_REFERENCE) {
// FIXME(STM): do something with this ?
// #ifdef DEBUG
//      if (!Serializer::enabled()) {
//        Serializer::TooLateToEnableNow();
//      }
// #endif
      if (!Serializer::enabled() && !emit_debug_code()) {
        return;
      }
    }
    ASSERT(buffer_space() >= kMaxRelocSize);  // too late to grow buffer here
    reloc_info_writer.Write(&rinfo);
  }
}


void Assembler::add(Register Rx, const Immediate& imm) {
  if (imm.is_int8()) {
    add_imm_(imm.x_, Rx);
  } else {
    // Use a super scratch register (r3) and a tiny constant pool
    align();
    movl_dispPC_(4, rtmp);
    nop_();
    bra_(4);
    add_(rtmp, Rx);
    *reinterpret_cast<uint32_t*>(pc_) = imm.x_;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::add(Register Rx, Register Ry, const Immediate& imm) {
  if (Ry.code() != Rx.code())
    mov_(Ry, Rx);
  add(Rx, imm);
}


void Assembler::add(Register Rx, Register Ry, Register Rz) {
  if (Ry.code() == Rx.code())
    add_(Rz, Rx);
  else if (Rz.code() == Rx.code()) {
    add_(Ry, Rx);
  } else {
    mov_(Ry, Rx);
    add_(Rz, Rx);
  }
}


void Assembler::sub(Register Rx, Register Ry, const Immediate& imm) {
  add(Rx, Ry, Immediate(-imm.x_));
}


void Assembler::sub(Register Rx, Register Ry, Register Rz) {
  if (Ry.code() == Rx.code()) {
    sub_(Rz, Rx);
  } else if (Rz.code() == Rx.code()) {
    neg_(Rz, Rx);
    add_(Ry, Rx);
  } else {
    mov_(Ry, Rx);
    sub_(Rz, Rx);
  }
}


void Assembler::lsl(Register Rx, Register Ry, const Immediate& imm) {
  if (Ry.code() != Rx.code())
    mov_(Ry, Rx);
  if (imm.x_ == 1) {
    shll_(Rx);
  } else if (imm.x_ == 2) {
    shll2_(Rx);
  } else {
    mov(rtmp, imm);
    shld_(rtmp, Rx);
  }
}


void Assembler::lsr(Register Rx, Register Ry, const Immediate& imm) {
  if (Ry.code() != Rx.code())
    mov_(Ry, Rx);
  if (imm.x_ == 1) {
    shlr_(Rx);
  } else if (imm.x_ == 2) {
    shlr2_(Rx);
  } else {
    mov(rtmp, Immediate(32 - imm.x_));
    shld_(rtmp, Rx);
  }
}


void Assembler::call(Label* L) {
  UNIMPLEMENTED();
}


void Assembler::db(uint8_t data) {
  CheckBuffer();
  *reinterpret_cast<uint8_t*>(pc_) = data;
  pc_ += sizeof(uint8_t);
}


void Assembler::dd(uint32_t data) {
  CheckBuffer();
  *reinterpret_cast<uint32_t*>(pc_) = data;
  pc_ += sizeof(uint32_t);
}


const int kEndOfChain = 0;

void Assembler::bind_to(Label* L, int pos) {
  while (L->is_linked()) {
    // Compute the current position
    uint16_t* p_pos = reinterpret_cast<uint16_t*>(L->pos());
    // Compute the next before the patch
    next(L);
    // Patch
    *reinterpret_cast<uint32_t*>(p_pos) = pos;
  }
  L->bind_to(pos);

  // Keep track of the last bound label so we don't eliminate any instructions
  // before a bound label.
  if (pos > last_bound_pos_)
    last_bound_pos_ = pos;
}


void Assembler::bind(Label* L) {
  ASSERT(!L->is_bound());  // label can only be bound once
  bind_to(L, pc_offset());
}


void Assembler::next(Label* L) {
  ASSERT(L->is_linked());
  int link = *reinterpret_cast<uint32_t*>(L->pos());
  if (link > 0) {
    L->link_to(link);
  } else {
    ASSERT(link == kEndOfChain);
    L->Unuse();
  }
}


void Assembler::jmp(Label* L) {
  if (L->is_bound()) {
    jmp(L->pos());
  } else {
    if (L->is_linked())
      jmp(L->pos());
    else
      jmp(kEndOfChain);           // Patched later on
    int pos = reinterpret_cast<int>(reinterpret_cast<uint16_t*>(pc_) - 2);
    L->link_to(pos);  // Link to the constant
  }
}


void Assembler::jmp(Handle<Code> code, RelocInfo::Mode rmode) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  if (rmode != RelocInfo::NONE) RecordRelocInfo(rmode);
  intptr_t dst = reinterpret_cast<intptr_t>(code.location()) -
                 reinterpret_cast<intptr_t>(pc_);

  jmp(dst);
}


void Assembler::jmp(int offset) {
  // Do a short jump if possible
  if (offset >= -4096 && offset <= 4094 && offset != 0) {
    bra_(offset);
    nop_();
  } else {
    // Use a super scratch register (r3) and a tiny constant pool
    align();
    movl_dispPC_(4, rtmp);
    nop_();
    braf_(rtmp);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::mov(Register Rx, const Immediate& imm) {
  // FIXME(STM): Internal ref not handled
  ASSERT(imm.rmode_ != RelocInfo::INTERNAL_REFERENCE);

  // FIXME(STM) needed ?
  if (imm.rmode_ != RelocInfo::NONE) RecordRelocInfo(imm.rmode_);

  // Move based on immediates can only be 8 bits long
  if (imm.is_int8()) {
    mov_imm_(imm.x_, Rx);
  } else {
    // Use a tiny constant pool and jump above
    align();
    movl_dispPC_(4, Rx);
    nop_();
    bra_(4);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = imm.x_;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::mov(Register Rx, const Operand& src) {
  if (src.rx_.is_valid())
    mov_(src.rx_, Rx);
  else
    mov(Rx, Immediate(src.imm32_, src.rmode_));
}


void Assembler::mov(Register Rx, const MemOperand& src) {
  if (src.offset_ == 0) {
    movl_indRy_(src.rm_, Rx);
  } else {
    if (FITS_SH4_movl_dispRy(src.offset_)) {
      movl_dispRy_(src.offset_, src.rm_, Rx);
    } else {
      add(rtmp, src.rm_, Immediate(src.offset_));
      movl_indRy_(rtmp, Rx);
    }
  }
}

void Assembler::mov(const MemOperand& dst, Register Rx) {
  if (dst.offset_ == 0) {
    movl_indRx_(Rx, dst.rm_);
  } else {
    if (FITS_SH4_movl_dispRx(dst.offset_)) {
      movl_dispRx_(Rx, dst.offset_, dst.rm_);
    } else {
      add(rtmp, dst.rm_, Immediate(dst.offset_));
      movl_indRx_(Rx, rtmp);
    }
  }
}


void Assembler::pop(Register dst) {
  movl_incRy_(Register(r15), dst);
}


void Assembler::popm(RegList dst) {
  for (int16_t i = Register::kNumRegisters - 1; i >= 0; i--) {
    if ((dst & (1 << i)) != 0) {
      pop(Register::from_code(i));
    }
  }
}


void Assembler::popPR() {
  ldsl_incRx_PR_(r15);
}


void Assembler::push(Register src) {
  movl_decRx_(Register(r15), src);
}


void Assembler::push(const Immediate& imm) {
  mov(rtmp, imm);
  push(rtmp);
}


void Assembler::push(const Operand& op) {
  mov(rtmp, op);
  push(rtmp);
}


void Assembler::pushm(RegList src) {
  for (uint16_t i = 0; i < Register::kNumRegisters; i++) {
    if ((src & (1 << i)) != 0) {
      push(Register::from_code(i));
    }
  }
}


void Assembler::pushPR() {
  stsl_PR_decRx_(r15);
}


Assembler::~Assembler() {
  if (own_buffer_) {
    if (isolate()->assembler_spare_buffer() == NULL &&
        buffer_size_ == kMinimalBufferSize) {
      isolate()->set_assembler_spare_buffer(buffer_);
    } else {
      DeleteArray(buffer_);
    }
  }
}


const int RelocInfo::kApplyMask = 0;

bool RelocInfo::IsCodedSpecially() {
  UNIMPLEMENTED();
  return false;
}


inline void asm_output(const char *str, int a = 0 , int b = 0, int c = 0) {}
#define REGNUM(reg) (reg).code()

void Assembler::add_imm_(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_add_imm(imm));
  emit((0x7 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0xFF) << 0));
  asm_output("add_imm %d, R%d", imm, REGNUM(Rx));
}


void Assembler::add_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("add R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::addc_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("addc R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::addv_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("addv R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::and_imm_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_and_imm_R0(imm));
  emit((0xC << 12) | (0x9 << 8) | ((imm & 0xFF) << 0));
  asm_output("and_imm_R0 %d", imm);
}


void Assembler::and_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("and R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::andb_imm_dispR0GBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_andb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xD << 8) | ((imm & 0xFF) << 0));
  asm_output("andb_imm_dispR0GBR %d", imm);
}


void Assembler::bra_(int imm) {
  ASSERT(SH4_CHECK_RANGE_bra(imm) && SH4_CHECK_ALIGN_bra(imm));
  emit((0xA << 12) | (((imm & 0x1FFE) >> 1) << 0));
  asm_output("bra %d", imm);
}


void Assembler::bsr_(int imm) {
  ASSERT(SH4_CHECK_RANGE_bsr(imm) && SH4_CHECK_ALIGN_bsr(imm));
  emit((0xB << 12) | (((imm & 0x1FFE) >> 1) << 0));
  asm_output("bsr %d", imm);
}


void Assembler::bt_(int imm) {
  ASSERT(SH4_CHECK_RANGE_bt(imm) && SH4_CHECK_ALIGN_bt(imm));
  emit((0x8 << 12) | (0x9 << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("bt %d", imm);
}


void Assembler::bf_(int imm) {
  ASSERT(SH4_CHECK_RANGE_bf(imm) && SH4_CHECK_ALIGN_bf(imm));
  emit((0x8 << 12) | (0xB << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("bf %d", imm);
}


void Assembler::bts_(int imm) {
  ASSERT(SH4_CHECK_RANGE_bts(imm) && SH4_CHECK_ALIGN_bts(imm));
  emit((0x8 << 12) | (0xD << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("bts %d", imm);
}


void Assembler::bfs_(int imm) {
  ASSERT(SH4_CHECK_RANGE_bfs(imm) && SH4_CHECK_ALIGN_bfs(imm));
  emit((0x8 << 12) | (0xF << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("bfs %d", imm);
}


void Assembler::clrmac_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x2 << 4) | (0x8 << 0));
  asm_output("clrmac");
}


void Assembler::clrs_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x4 << 4) | (0x8 << 0));
  asm_output("clrs");
}


void Assembler::clrt_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x0 << 4) | (0x8 << 0));
  asm_output("clrt");
}


void Assembler::cmpeq_imm_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_cmpeq_imm_R0(imm));
  emit((0x8 << 12) | (0x8 << 8) | ((imm & 0xFF) << 0));
  asm_output("cmpeq_imm_R0 %d", imm);
}


void Assembler::cmpeq_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x0 << 0));
  asm_output("cmpeq R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmpge_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x3 << 0));
  asm_output("cmpge R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmpgt_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x7 << 0));
  asm_output("cmpgt R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmphi_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("cmphi R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmphs_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x2 << 0));
  asm_output("cmphs R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::cmppl_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x5 << 0));
  asm_output("cmppl R%d", REGNUM(Rx));
}


void Assembler::cmppz_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x1 << 0));
  asm_output("cmppz R%d", REGNUM(Rx));
}


void Assembler::cmpstr_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("cmpstr R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::div0s_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x7 << 0));
  asm_output("div0s R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::div0u_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x1 << 4) | (0x9 << 0));
  asm_output("div0u");
}


void Assembler::div1_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x4 << 0));
  asm_output("div1 R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::extsb_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("extsb R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::extsw_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("extsw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::extub_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("extub R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::extuw_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("extuw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::icbi_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xE << 4) | (0x3 << 0));
  asm_output("icbi_indRx R%d", REGNUM(Rx));
}


void Assembler::jmp_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0xB << 0));
  asm_output("jmp_indRx R%d", REGNUM(Rx));
}


void Assembler::jsr_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0xB << 0));
  asm_output("jsr_indRx R%d", REGNUM(Rx));
}


void Assembler::ldc_SR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0xE << 0));
  asm_output("ldc_SR R%d", REGNUM(Rx));
}


void Assembler::ldc_GBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0xE << 0));
  asm_output("ldc_GBR R%d", REGNUM(Rx));
}


void Assembler::ldc_SGR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0xA << 0));
  asm_output("ldc_SGR R%d", REGNUM(Rx));
}


void Assembler::ldc_VBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0xE << 0));
  asm_output("ldc_VBR R%d", REGNUM(Rx));
}


void Assembler::ldc_SSR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0xE << 0));
  asm_output("ldc_SSR R%d", REGNUM(Rx));
}


void Assembler::ldc_SPC_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x4 << 4) | (0xE << 0));
  asm_output("ldc_SPC R%d", REGNUM(Rx));
}


void Assembler::ldc_DBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xF << 4) | (0xA << 0));
  asm_output("ldc_DBR R%d", REGNUM(Rx));
}


void Assembler::ldc_bank_(Register Rx, int imm) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_ldc_bank(imm));
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0x7) << 4) | (0xE << 0));
  asm_output("ldc_bank R%d, %d", REGNUM(Rx), imm);
}


void Assembler::ldcl_incRx_SR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_SR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_GBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_GBR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_VBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_VBR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_SGR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x6 << 0));
  asm_output("ldcl_incRx_SGR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_SSR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_SSR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_SPC_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x4 << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_SPC R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_DBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xF << 4) | (0x6 << 0));
  asm_output("ldcl_incRx_DBR R%d", REGNUM(Rx));
}


void Assembler::ldcl_incRx_bank_(Register Rx, int imm) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_ldcl_incRx_bank(imm));
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x7 << 0));
  asm_output("ldcl_incRx_bank R%d, %d", REGNUM(Rx), imm);
}


void Assembler::lds_MACH_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0xA << 0));
  asm_output("lds_MACH R%d", REGNUM(Rx));
}


void Assembler::lds_MACL_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0xA << 0));
  asm_output("lds_MACL R%d", REGNUM(Rx));
}


void Assembler::lds_PR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0xA << 0));
  asm_output("lds_PR R%d", REGNUM(Rx));
}


void Assembler::lds_FPUL_(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x5 << 4) | (0xA << 0));
  asm_output("lds_FPUL R%d", REGNUM(Ry));
}


void Assembler::lds_FPSCR_(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x6 << 4) | (0xA << 0));
  asm_output("lds_FPSCR R%d", REGNUM(Ry));
}


void Assembler::ldsl_incRx_MACH_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x6 << 0));
  asm_output("ldsl_incRx_MACH R%d", REGNUM(Rx));
}


void Assembler::ldsl_incRx_MACL_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x6 << 0));
  asm_output("ldsl_incRx_MACL R%d", REGNUM(Rx));
}


void Assembler::ldsl_incRx_PR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x6 << 0));
  asm_output("ldsl_incRx_PR R%d", REGNUM(Rx));
}


void Assembler::ldsl_incRy_FPUL_(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x5 << 4) | (0x6 << 0));
  asm_output("ldsl_incRy_FPUL R%d", REGNUM(Ry));
}


void Assembler::ldsl_incRy_FPSCR_(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x6 << 4) | (0x6 << 0));
  asm_output("ldsl_incRy_FPSCR R%d", REGNUM(Ry));
}


void Assembler::ldtlb_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x3 << 4) | (0x8 << 0));
  asm_output("ldtlb");
}


void Assembler::macw_incRy_incRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("macw_incRy_incRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::mov_imm_(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_mov_imm(imm));
  emit((0xE << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0xFF) << 0));
  asm_output("mov_imm %d, R%d", imm, REGNUM(Rx));
}


void Assembler::mov_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x3 << 0));
  asm_output("mov R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_dispR0Rx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_decRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_indRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x0 << 0));
  asm_output("movb_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_dispRy_R0_(int imm, Register Ry) {
  ASSERT(REGNUM(Ry) <= 15 && SH4_CHECK_RANGE_movb_dispRy_R0(imm));
  emit((0x8 << 12) | (0x4 << 8) | ((REGNUM(Ry) & 0xF) << 4) | ((imm & 0xF) << 0));
  asm_output("movb_dispRy_R0 %d, R%d", imm, REGNUM(Ry));
}


void Assembler::movb_dispGBR_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movb_dispGBR_R0(imm));
  emit((0xC << 12) | (0x4 << 8) | ((imm & 0xFF) << 0));
  asm_output("movb_dispGBR_R0 %d", imm);
}


void Assembler::movb_dispR0Ry_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("movb_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_incRy_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_indRy_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x0 << 0));
  asm_output("movb_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movb_R0_dispRx_(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_movb_R0_dispRx(imm));
  emit((0x8 << 12) | (0x0 << 8) | ((REGNUM(Rx) & 0xF) << 4) | ((imm & 0xF) << 0));
  asm_output("movb_R0_dispRx %d, R%d", imm, REGNUM(Rx));
}


void Assembler::movb_R0_dispGBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movb_R0_dispGBR(imm));
  emit((0xC << 12) | (0x0 << 8) | ((imm & 0xFF) << 0));
  asm_output("movb_R0_dispGBR %d", imm);
}


void Assembler::movl_dispRx_(Register Ry, int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15 && SH4_CHECK_RANGE_movl_dispRx(imm) && SH4_CHECK_ALIGN_movl_dispRx(imm));
  emit((0x1 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (((imm & 0x3C) >> 2) << 0));
  asm_output("movl_dispRx R%d, %d, R%d", REGNUM(Ry), imm, REGNUM(Rx));
}


void Assembler::movl_dispR0Rx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_decRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_indRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x2 << 0));
  asm_output("movl_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_dispRy_(int imm, Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15 && SH4_CHECK_RANGE_movl_dispRy(imm) && SH4_CHECK_ALIGN_movl_dispRy(imm));
  emit((0x5 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (((imm & 0x3C) >> 2) << 0));
  asm_output("movl_dispRy %d, R%d, R%d", imm, REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_dispGBR_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movl_dispGBR_R0(imm) && SH4_CHECK_ALIGN_movl_dispGBR_R0(imm));
  emit((0xC << 12) | (0x6 << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_dispGBR_R0 %d", imm);
}


void Assembler::movl_dispPC_(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_movl_dispPC(imm) && SH4_CHECK_ALIGN_movl_dispPC(imm));
  emit((0xD << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_dispPC %d, R%d", imm, REGNUM(Rx));
}


void Assembler::movl_dispR0Ry_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("movl_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_incRy_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_indRy_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x2 << 0));
  asm_output("movl_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movl_R0_dispGBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movl_R0_dispGBR(imm) && SH4_CHECK_ALIGN_movl_R0_dispGBR(imm));
  emit((0xC << 12) | (0x2 << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_R0_dispGBR %d", imm);
}


void Assembler::movw_dispR0Rx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_decRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_indRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x1 << 0));
  asm_output("movw_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_dispRy_R0_(int imm, Register Ry) {
  ASSERT(REGNUM(Ry) <= 15 && SH4_CHECK_RANGE_movw_dispRy_R0(imm) && SH4_CHECK_ALIGN_movw_dispRy_R0(imm));
  emit((0x8 << 12) | (0x5 << 8) | ((REGNUM(Ry) & 0xF) << 4) | (((imm & 0x1E) >> 1) << 0));
  asm_output("movw_dispRy_R0 %d, R%d", imm, REGNUM(Ry));
}


void Assembler::movw_dispGBR_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movw_dispGBR_R0(imm) && SH4_CHECK_ALIGN_movw_dispGBR_R0(imm));
  emit((0xC << 12) | (0x5 << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("movw_dispGBR_R0 %d", imm);
}


void Assembler::movw_dispPC_(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_movw_dispPC(imm) && SH4_CHECK_ALIGN_movw_dispPC(imm));
  emit((0x9 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("movw_dispPC %d, R%d", imm, REGNUM(Rx));
}


void Assembler::movw_dispR0Ry_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("movw_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_incRy_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_indRy_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x1 << 0));
  asm_output("movw_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::movw_R0_dispRx_(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_movw_R0_dispRx(imm) && SH4_CHECK_ALIGN_movw_R0_dispRx(imm));
  emit((0x8 << 12) | (0x1 << 8) | ((REGNUM(Rx) & 0xF) << 4) | (((imm & 0x1E) >> 1) << 0));
  asm_output("movw_R0_dispRx %d, R%d", imm, REGNUM(Rx));
}


void Assembler::movw_R0_dispGBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movw_R0_dispGBR(imm) && SH4_CHECK_ALIGN_movw_R0_dispGBR(imm));
  emit((0xC << 12) | (0x1 << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("movw_R0_dispGBR %d", imm);
}


void Assembler::mova_dispPC_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_mova_dispPC_R0(imm) && SH4_CHECK_ALIGN_mova_dispPC_R0(imm));
  emit((0xC << 12) | (0x7 << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("mova_dispPC_R0 %d", imm);
}


void Assembler::movcal_R0_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xC << 4) | (0x3 << 0));
  asm_output("movcal_R0_indRx R%d", REGNUM(Rx));
}


void Assembler::movcol_R0_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x7 << 4) | (0x3 << 0));
  asm_output("movcol_R0_indRx R%d", REGNUM(Rx));
}


void Assembler::movlil_indRy_R0_(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0x6 << 4) | (0x3 << 0));
  asm_output("movlil_indRy_R0 R%d", REGNUM(Ry));
}


void Assembler::movt_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x9 << 0));
  asm_output("movt R%d", REGNUM(Rx));
}


void Assembler::movual_indRy_R0_(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0xA << 4) | (0x9 << 0));
  asm_output("movual_indRy_R0 R%d", REGNUM(Ry));
}


void Assembler::movual_incRy_R0_(Register Ry) {
  ASSERT(REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Ry) & 0xF) << 8) | (0xE << 4) | (0x9 << 0));
  asm_output("movual_incRy_R0 R%d", REGNUM(Ry));
}


void Assembler::mulsw_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("mulsw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::muls_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("muls R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::mull_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x7 << 0));
  asm_output("mull R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::muluw_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("muluw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::mulu_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xE << 0));
  asm_output("mulu R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::neg_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xB << 0));
  asm_output("neg R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::negc_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xA << 0));
  asm_output("negc R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::nop_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x0 << 4) | (0x9 << 0));
  asm_output("nop");
}


void Assembler::not_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x7 << 0));
  asm_output("not R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::ocbi_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x9 << 4) | (0x3 << 0));
  asm_output("ocbi_indRx R%d", REGNUM(Rx));
}


void Assembler::ocbp_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xA << 4) | (0x3 << 0));
  asm_output("ocbp_indRx R%d", REGNUM(Rx));
}


void Assembler::ocbwb_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xB << 4) | (0x3 << 0));
  asm_output("ocbwb_indRx R%d", REGNUM(Rx));
}


void Assembler::or_imm_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_or_imm_R0(imm));
  emit((0xC << 12) | (0xB << 8) | ((imm & 0xFF) << 0));
  asm_output("or_imm_R0 %d", imm);
}


void Assembler::or_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xB << 0));
  asm_output("or R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::orb_imm_dispR0GBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_orb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xF << 8) | ((imm & 0xFF) << 0));
  asm_output("orb_imm_dispR0GBR %d", imm);
}


void Assembler::pref_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x8 << 4) | (0x3 << 0));
  asm_output("pref_indRx R%d", REGNUM(Rx));
}


void Assembler::prefi_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xD << 4) | (0x3 << 0));
  asm_output("prefi_indRx R%d", REGNUM(Rx));
}


void Assembler::rotcl_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x4 << 0));
  asm_output("rotcl R%d", REGNUM(Rx));
}


void Assembler::rotcr_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x5 << 0));
  asm_output("rotcr R%d", REGNUM(Rx));
}


void Assembler::rotl_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x4 << 0));
  asm_output("rotl R%d", REGNUM(Rx));
}


void Assembler::rotr_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x5 << 0));
  asm_output("rotr R%d", REGNUM(Rx));
}


void Assembler::rte_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x2 << 4) | (0xB << 0));
  asm_output("rte");
}


void Assembler::rts_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x0 << 4) | (0xB << 0));
  asm_output("rts");
}


void Assembler::sets_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x5 << 4) | (0x8 << 0));
  asm_output("sets");
}


void Assembler::sett_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x1 << 4) | (0x8 << 0));
  asm_output("sett");
}


void Assembler::shad_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xC << 0));
  asm_output("shad R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::shld_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("shld R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::shal_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x0 << 0));
  asm_output("shal R%d", REGNUM(Rx));
}


void Assembler::shar_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x1 << 0));
  asm_output("shar R%d", REGNUM(Rx));
}


void Assembler::shll_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x0 << 0));
  asm_output("shll R%d", REGNUM(Rx));
}


void Assembler::shll16_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x8 << 0));
  asm_output("shll16 R%d", REGNUM(Rx));
}


void Assembler::shll2_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x8 << 0));
  asm_output("shll2 R%d", REGNUM(Rx));
}


void Assembler::shll8_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x8 << 0));
  asm_output("shll8 R%d", REGNUM(Rx));
}


void Assembler::shlr_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x1 << 0));
  asm_output("shlr R%d", REGNUM(Rx));
}


void Assembler::shlr16_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x9 << 0));
  asm_output("shlr16 R%d", REGNUM(Rx));
}


void Assembler::shlr2_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x9 << 0));
  asm_output("shlr2 R%d", REGNUM(Rx));
}


void Assembler::shlr8_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x9 << 0));
  asm_output("shlr8 R%d", REGNUM(Rx));
}


void Assembler::sleep_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x1 << 4) | (0xB << 0));
  asm_output("sleep");
}


void Assembler::stc_SR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x2 << 0));
  asm_output("stc_SR R%d", REGNUM(Rx));
}


void Assembler::stc_GBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x2 << 0));
  asm_output("stc_GBR R%d", REGNUM(Rx));
}


void Assembler::stc_VBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x2 << 0));
  asm_output("stc_VBR R%d", REGNUM(Rx));
}


void Assembler::stc_SSR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x2 << 0));
  asm_output("stc_SSR R%d", REGNUM(Rx));
}


void Assembler::stc_SPC_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x4 << 4) | (0x2 << 0));
  asm_output("stc_SPC R%d", REGNUM(Rx));
}


void Assembler::stc_SGR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0xA << 0));
  asm_output("stc_SGR R%d", REGNUM(Rx));
}


void Assembler::stc_DBR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xF << 4) | (0xA << 0));
  asm_output("stc_DBR R%d", REGNUM(Rx));
}


void Assembler::stc_bank_(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_stc_bank(imm));
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x2 << 0));
  asm_output("stc_bank %d, R%d", imm, REGNUM(Rx));
}


void Assembler::stcl_SR_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x3 << 0));
  asm_output("stcl_SR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_VBR_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x3 << 0));
  asm_output("stcl_VBR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_SSR_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x3 << 0));
  asm_output("stcl_SSR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_SPC_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x4 << 4) | (0x3 << 0));
  asm_output("stcl_SPC_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_GBR_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x3 << 0));
  asm_output("stcl_GBR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_SGR_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x3 << 4) | (0x2 << 0));
  asm_output("stcl_SGR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_DBR_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0xF << 4) | (0x2 << 0));
  asm_output("stcl_DBR_decRx R%d", REGNUM(Rx));
}


void Assembler::stcl_bank_decRx_(int imm, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && SH4_CHECK_RANGE_stcl_bank_decRx(imm));
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x3 << 0));
  asm_output("stcl_bank_decRx %d, R%d", imm, REGNUM(Rx));
}


void Assembler::sts_MACH_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0xA << 0));
  asm_output("sts_MACH R%d", REGNUM(Rx));
}


void Assembler::sts_MACL_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0xA << 0));
  asm_output("sts_MACL R%d", REGNUM(Rx));
}


void Assembler::sts_PR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0xA << 0));
  asm_output("sts_PR R%d", REGNUM(Rx));
}


void Assembler::sts_FPUL_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x5 << 4) | (0xA << 0));
  asm_output("sts_FPUL R%d", REGNUM(Rx));
}


void Assembler::sts_FPSCR_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x6 << 4) | (0xA << 0));
  asm_output("sts_FPSCR R%d", REGNUM(Rx));
}


void Assembler::stsl_MACH_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x2 << 0));
  asm_output("stsl_MACH_decRx R%d", REGNUM(Rx));
}


void Assembler::stsl_MACL_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x2 << 0));
  asm_output("stsl_MACL_decRx R%d", REGNUM(Rx));
}


void Assembler::stsl_PR_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x2 << 0));
  asm_output("stsl_PR_decRx R%d", REGNUM(Rx));
}


void Assembler::stsl_FPUL_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x5 << 4) | (0x2 << 0));
  asm_output("stsl_FPUL_decRx R%d", REGNUM(Rx));
}


void Assembler::stsl_FPSCR_decRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x6 << 4) | (0x2 << 0));
  asm_output("stsl_FPSCR_decRx R%d", REGNUM(Rx));
}


void Assembler::sub_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("sub R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::subc_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xA << 0));
  asm_output("subc R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::subv_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xB << 0));
  asm_output("subv R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::swapb_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("swapb R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::swapw_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("swapw R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::synco_() {
  emit((0x0 << 12) | (0x0 << 8) | (0xA << 4) | (0xB << 0));
  asm_output("synco");
}


void Assembler::tasb_indRx_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0xB << 0));
  asm_output("tasb_indRx R%d", REGNUM(Rx));
}


void Assembler::trapa_imm_(int imm) {
  ASSERT(SH4_CHECK_RANGE_trapa_imm(imm));
  emit((0xC << 12) | (0x3 << 8) | ((imm & 0xFF) << 0));
  asm_output("trapa_imm %d", imm);
}


void Assembler::tst_imm_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_tst_imm_R0(imm));
  emit((0xC << 12) | (0x8 << 8) | ((imm & 0xFF) << 0));
  asm_output("tst_imm_R0 %d", imm);
}


void Assembler::tst_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("tst R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::tstb_imm_dispR0GBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_tstb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xC << 8) | ((imm & 0xFF) << 0));
  asm_output("tstb_imm_dispR0GBR %d", imm);
}


void Assembler::xor_imm_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_xor_imm_R0(imm));
  emit((0xC << 12) | (0xA << 8) | ((imm & 0xFF) << 0));
  asm_output("xor_imm_R0 %d", imm);
}


void Assembler::xor_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xA << 0));
  asm_output("xor R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::xorb_imm_dispR0GBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_xorb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xE << 8) | ((imm & 0xFF) << 0));
  asm_output("xorb_imm_dispR0GBR %d", imm);
}


void Assembler::xtrct_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("xtrct R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::dt_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x1 << 4) | (0x0 << 0));
  asm_output("dt R%d", REGNUM(Rx));
}


void Assembler::dmulsl_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xD << 0));
  asm_output("dmulsl R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::dmulul_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x5 << 0));
  asm_output("dmulul R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::macl_incRy_incRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && REGNUM(Ry) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0xF << 0));
  asm_output("macl_incRy_incRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::braf_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x2 << 4) | (0x3 << 0));
  asm_output("braf R%d", REGNUM(Rx));
}


void Assembler::bsrf_(Register Rx) {
  ASSERT(REGNUM(Rx) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rx) & 0xF) << 8) | (0x0 << 4) | (0x3 << 0));
  asm_output("bsrf R%d", REGNUM(Rx));
}


void Assembler::fabs_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x5 << 4) | (0xD << 0));
  asm_output("fabs R%d", REGNUM(Rx));
}


void Assembler::fabs_double_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x5 << 4) | (0xD << 0));
  asm_output("fabs_double R%d", REGNUM(Rx));
}


void Assembler::fadd_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x0 << 0));
  asm_output("fadd R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fadd_double_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x0 << 0));
  asm_output("fadd_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcmpeq_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x4 << 0));
  asm_output("fcmpeq R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcmpeq_double_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x4 << 0));
  asm_output("fcmpeq_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcmpgt_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x5 << 0));
  asm_output("fcmpgt R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcmpgt_double_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x5 << 0));
  asm_output("fcmpgt_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fcnvds_double_FPUL_(Register Rx) {
  ASSERT(!((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0xB << 4) | (0xD << 0));
  asm_output("fcnvds_double_FPUL R%d", REGNUM(Rx));
}


void Assembler::fcnvsd_FPUL_double_(Register Rx) {
  ASSERT(!((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0xA << 4) | (0xD << 0));
  asm_output("fcnvsd_FPUL_double R%d", REGNUM(Rx));
}


void Assembler::fdiv_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x3 << 0));
  asm_output("fdiv R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fdiv_double_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x3 << 0));
  asm_output("fdiv_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fipr_(Register Ry, Register Rx) {
  ASSERT(!(((REGNUM(Rx) - 16) & 0x3) || ((REGNUM(Ry) - 16) & 0x3)));
  emit((0xF << 12) | (((((REGNUM(Rx) - 16) & 0xF) << 2) | (((REGNUM(Ry) - 16) & 0xF) >> 2)) << 8) | (0xE << 4) | (0xD << 0));
  asm_output("fipr R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fldi0_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x8 << 4) | (0xD << 0));
  asm_output("fldi0 R%d", REGNUM(Rx));
}


void Assembler::fldi1_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x9 << 4) | (0xD << 0));
  asm_output("fldi1 R%d", REGNUM(Rx));
}


void Assembler::flds_FPUL_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x1 << 4) | (0xD << 0));
  asm_output("flds_FPUL R%d", REGNUM(Rx));
}


void Assembler::float_FPUL_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x2 << 4) | (0xD << 0));
  asm_output("float_FPUL R%d", REGNUM(Rx));
}


void Assembler::float_FPUL_double_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x2 << 4) | (0xD << 0));
  asm_output("float_FPUL_double R%d", REGNUM(Rx));
}


void Assembler::fmac_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xE << 0));
  asm_output("fmac R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xC << 0));
  asm_output("fmov R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_Xdouble_Xdouble_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xC << 0));
  asm_output("fmov_Xdouble_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_indRy_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmov_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_indRy_Xdouble_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmov_indRy_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_indRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xA << 0));
  asm_output("fmov_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_Xdouble_indRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xA << 0));
  asm_output("fmov_Xdouble_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_incRy_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmov_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_incRy_Xdouble_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmov_incRy_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_decRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xB << 0));
  asm_output("fmov_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_Xdouble_decRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xB << 0));
  asm_output("fmov_Xdouble_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_dispR0Ry_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmov_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_dispR0Ry_Xdouble_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmov_dispR0Ry_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_dispR0Rx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmov_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmov_Xdouble_dispR0Rx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmov_Xdouble_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_indRy_Xdouble_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmovd_indRy_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_Xdouble_indRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xA << 0));
  asm_output("fmovd_Xdouble_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_incRy_Xdouble_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmovd_incRy_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_Xdouble_decRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xB << 0));
  asm_output("fmovd_Xdouble_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_dispR0Ry_Xdouble_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmovd_dispR0Ry_Xdouble R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovd_Xdouble_dispR0Rx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmovd_Xdouble_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_indRy_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmovs_indRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_indRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xA << 0));
  asm_output("fmovs_indRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_incRy_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmovs_incRy R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_decRx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0xB << 0));
  asm_output("fmovs_decRx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_dispR0Ry_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && REGNUM(Ry) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | ((REGNUM(Ry) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmovs_dispR0Ry R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmovs_dispR0Rx_(Register Ry, Register Rx) {
  ASSERT(REGNUM(Rx) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | ((REGNUM(Rx) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmovs_dispR0Rx R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmul_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x2 << 0));
  asm_output("fmul R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fmul_double_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x2 << 0));
  asm_output("fmul_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fneg_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x4 << 4) | (0xD << 0));
  asm_output("fneg R%d", REGNUM(Rx));
}


void Assembler::fneg_double_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x4 << 4) | (0xD << 0));
  asm_output("fneg_double R%d", REGNUM(Rx));
}


void Assembler::fpchg_() {
  emit((0xF << 12) | (0x7 << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fpchg");
}


void Assembler::frchg_() {
  emit((0xF << 12) | (0xB << 8) | (0xF << 4) | (0xD << 0));
  asm_output("frchg");
}


void Assembler::fsca_FPUL_double_(Register Rx) {
  ASSERT(!((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fsca_FPUL_double R%d", REGNUM(Rx));
}


void Assembler::fschg_() {
  emit((0xF << 12) | (0x3 << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fschg");
}


void Assembler::fsqrt_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x6 << 4) | (0xD << 0));
  asm_output("fsqrt R%d", REGNUM(Rx));
}


void Assembler::fsqrt_double_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x6 << 4) | (0xD << 0));
  asm_output("fsqrt_double R%d", REGNUM(Rx));
}


void Assembler::fsrra_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x7 << 4) | (0xD << 0));
  asm_output("fsrra R%d", REGNUM(Rx));
}


void Assembler::fsts_FPUL_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x0 << 4) | (0xD << 0));
  asm_output("fsts_FPUL R%d", REGNUM(Rx));
}


void Assembler::fsub_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && (REGNUM(Ry) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x1 << 0));
  asm_output("fsub R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::fsub_double_(Register Ry, Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1) && (REGNUM(Ry) - 16) <= 15 && !((REGNUM(Ry) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (((REGNUM(Ry) - 16) & 0xF) << 4) | (0x1 << 0));
  asm_output("fsub_double R%d, R%d", REGNUM(Ry), REGNUM(Rx));
}


void Assembler::ftrc_FPUL_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15);
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x3 << 4) | (0xD << 0));
  asm_output("ftrc_FPUL R%d", REGNUM(Rx));
}


void Assembler::ftrc_double_FPUL_(Register Rx) {
  ASSERT((REGNUM(Rx) - 16) <= 15 && !((REGNUM(Rx) - 16) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rx) - 16) & 0xF) << 8) | (0x3 << 4) | (0xD << 0));
  asm_output("ftrc_double_FPUL R%d", REGNUM(Rx));
}


void Assembler::ftrv_(Register Rx) {
  ASSERT(!((REGNUM(Rx) - 16) & 0x3));
  emit((0xF << 12) | (((((REGNUM(Rx) - 16) & 0xF) << 2) | 0x1) << 8) | (0xF << 4) | (0xD << 0));
  asm_output("ftrv R%d", REGNUM(Rx));
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
