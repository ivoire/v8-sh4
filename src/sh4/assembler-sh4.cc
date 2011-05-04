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


// -----------------------------------------------------------------------------
// Implementation of Operand and MemOperand
// See assembler-sh4-inl.h for inlined constructors

Operand::Operand(Handle<Object> handle) {
  rx_ = no_reg;
  // Verify all Objects referred by code are NOT in new space.
  Object* obj = *handle;
  ASSERT(!HEAP->InNewSpace(obj));
  if (obj->IsHeapObject()) {
    imm32_ = reinterpret_cast<intptr_t>(handle.location());
    rmode_ = RelocInfo::EMBEDDED_OBJECT;
  } else {
    // no relocation needed
    imm32_ =  reinterpret_cast<intptr_t>(obj);
    rmode_ = RelocInfo::NONE;
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


void Assembler::add(Register Rd, const Immediate& imm) {
  if (imm.is_int8()) {
    add_imm_(imm.x_, Rd);
  } else {
    // Use a super scratch register (r3) and a tiny constant pool
    movl_dispPC_(4, rtmp);
    nop_();
    bra_(4);
    add_(rtmp, Rd);
    *reinterpret_cast<uint32_t*>(pc_) = imm.x_;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::add(Register Rd, Register Rs, const Immediate& imm) {
  if (Rs.code() != Rd.code())
    mov_(Rs, Rd);
  add(Rd, imm);
}


void Assembler::add(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() == Rd.code())
    add_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    add_(Rs, Rd);
  } else {
    mov_(Rs, Rd);
    add_(Rt, Rd);
  }
}


void Assembler::And(Register Rd, const Immediate& imm) {
  mov(r3, imm);
  and_(r3, Rd);
}


void Assembler::sub(Register Rd, Register Rs, const Immediate& imm) {
  add(Rd, Rs, Immediate(-imm.x_));
}


void Assembler::sub(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() == Rd.code()) {
    sub_(Rt, Rd);
  } else if (Rt.code() == Rd.code()) {
    neg_(Rt, Rd);
    add_(Rs, Rd);
  } else {
    mov_(Rs, Rd);
    sub_(Rt, Rd);
  }
}


void Assembler::addv(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() == Rd.code())
    addv_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    addv_(Rs, Rd);
  } else {
    mov_(Rs, Rd);
    addv_(Rt, Rd);
  }
}


void Assembler::lsl(Register Rd, Register Rs, const Immediate& imm) {
  if (Rs.code() != Rd.code())
    mov_(Rs, Rd);
  if (imm.x_ == 1) {
    shll_(Rd);
  } else if (imm.x_ == 2) {
    shll2_(Rd);
  } else {
    mov(rtmp, imm);
    shld_(rtmp, Rd);
  }
}


void Assembler::lsr(Register Rd, Register Rs, const Immediate& imm) {
  if (Rs.code() != Rd.code())
    mov_(Rs, Rd);
  if (imm.x_ == 1) {
    shlr_(Rd);
  } else if (imm.x_ == 2) {
    shlr2_(Rd);
  } else {
    mov(rtmp, Immediate(32 - imm.x_));
    shld_(rtmp, Rd);
  }
}

void Assembler::tst(Register Rd, const Immediate& imm) {
  mov(rtmp, imm);
  tst_(Rd, rtmp);
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

void Assembler::bind(Label* L) {
  // label can only be bound once
  ASSERT(!L->is_bound());

  int pos = pc_offset();

  // List the linked to patch
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


void Assembler::branch(Label* L, branch_type type) {
  if (L->is_bound()) {
    ASSERT(L->pos() != kEndOfChain);
    branch(L->pos(), type);
  } else {
    if (L->is_linked()) {
      ASSERT(L->pos() != kEndOfChain);
      branch(L->pos(), type);
    } else {
      branch(kEndOfChain, type);   // Patched later on
    }
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

void Assembler::jsr(Handle<Code> code, RelocInfo::Mode rmode) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  if (rmode != RelocInfo::NONE) RecordRelocInfo(rmode);
  intptr_t dst = reinterpret_cast<intptr_t>(code.location()) -
                 reinterpret_cast<intptr_t>(pc_);

  jsr(dst);
}

void Assembler::branch(int offset, branch_type type) {
  switch(type) {
  case branch_true:
    bt(offset); break;
  case branch_false:
    bf(offset); break;
  case branch_unconditional:
    jmp(offset); break;
  }
}


void Assembler::bt(int offset) {
  if(FITS_SH4_bt(offset) && offset != kEndOfChain) {
    bt_(offset);
    nop_();
  } else {
    bf_(8);
    nop_();
    movl_dispPC_(4, rtmp);
    nop_();
    braf_(rtmp);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::bf(int offset) {
  if(FITS_SH4_bf(offset) && offset != kEndOfChain) {
    bf_(offset);
    nop_();
  } else {
    bt_(8);
    nop_();
    movl_dispPC_(4, rtmp);
    nop_();
    braf_(rtmp);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::jmp(int offset) {
  // TODO: on other architectures we have:
  // positions_recorder()->WriteRecordedPositions();
  // check if this is necessary

  // Do a short jump if possible
  if (FITS_SH4_bra(offset) && offset != kEndOfChain) {
    bra_(offset);
    nop_();
  } else {
    // Use a super scratch register (r3) and a tiny constant pool
    movl_dispPC_(4, rtmp);
    nop_();
    braf_(rtmp);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);
  }
}

void Assembler::jsr(int offset) {
  // TODO: on other architectures we have:
  // positions_recorder()->WriteRecordedPositions();
  // check if this is necessary

  // Do a short jump if possible
  // TODO: check if we must remove 4 from offset
  // as the sematic of bra is to jump at [pc + 4 + (offset << 1)]
  if (FITS_SH4_bsr(offset) && offset != kEndOfChain) {
    bsr_(offset);
    nop_();
  } else {
    // Use a super scratch register (r3) and a tiny constant pool
    movl_dispPC_(4, rtmp);
    nop_();
    bsrf_(rtmp);
    nop_();
    // TODO: do we need align?
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::mov(Register Rd, const Immediate& imm) {
  // FIXME(STM): Internal ref not handled
  ASSERT(imm.rmode_ != RelocInfo::INTERNAL_REFERENCE);

  // FIXME(STM) needed ?
  if (imm.rmode_ != RelocInfo::NONE) RecordRelocInfo(imm.rmode_);

  // Move based on immediates can only be 8 bits long
  if (imm.is_int8()) {
    mov_imm_(imm.x_, Rd);
  } else {
    // Use a tiny constant pool and jump above
    movl_dispPC_(4, Rd);
    nop_();
    bra_(4);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = imm.x_;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::mov(Register Rd, const Operand& src) {
  if (src.rx_.is_valid())
    mov_(src.rx_, Rd);
  else
    mov(Rd, Immediate(src.imm32_, src.rmode_));
}


void Assembler::mov(Register Rd, const MemOperand& src) {
  if (src.offset_ == 0) {
    movl_indRs_(src.rm_, Rd);
  } else {
    if (FITS_SH4_movl_dispRs(src.offset_)) {
      movl_dispRs_(src.offset_, src.rm_, Rd);
    } else {
      add(rtmp, src.rm_, Immediate(src.offset_));
      movl_indRs_(rtmp, Rd);
    }
  }
}


void Assembler::mov(const MemOperand& dst, Register Rd) {
  if (dst.offset_ == 0) {
    movl_indRd_(Rd, dst.rm_);
  } else {
    if (FITS_SH4_movl_dispRd(dst.offset_)) {
      movl_dispRd_(Rd, dst.offset_, dst.rm_);
    } else {
      add(rtmp, dst.rm_, Immediate(dst.offset_));
      movl_indRd_(Rd, rtmp);
    }
  }
}


void Assembler::pop(Register dst) {
  if (dst.is(pr))
    ldsl_incRd_PR_(r15);
  else
    movl_incRs_(r15, dst);
}


void Assembler::pop(DwVfpRegister dst) {
  fmov_incRs_(r15, dst);
}


void Assembler::popm(RegList dst, bool doubles) {
  if (!doubles) {
    for (int16_t i = Register::kNumRegisters - 1; i >= 0; i--) {
      if ((dst & (1 << i)) != 0) {
        pop(Register::from_code(i));
      }
    }
  } else {
    for (int16_t i = DwVfpRegister::kNumRegisters - 1; i >= 0; i -= 2) {
      if ((dst & (1 << i)) != 0) {
        pop(DwVfpRegister::from_code(i));
      }
    }
  }
}


void Assembler::push(Register src) {
  if (src.is(pr))
    stsl_PR_decRd_(r15);
  else
    movl_decRd_(src, r15);
}


void Assembler::push(DwVfpRegister src) {
  fmov_decRd_(src, r15);
}


void Assembler::push(const Immediate& imm) {
  mov(rtmp, imm);
  push(rtmp);
}


void Assembler::push(const Operand& op) {
  mov(rtmp, op);
  push(rtmp);
}


void Assembler::pushm(RegList src, bool doubles) {
  if (!doubles) {
    for (uint16_t i = 0; i < Register::kNumRegisters; i++) {
      if ((src & (1 << i)) != 0) {
        push(Register::from_code(i));
      }
    }
  } else {
    for (uint16_t i = 0; i < Register::kNumRegisters; i += 2) {
      if ((src & (1 << i)) != 0) {
        push(DwVfpRegister::from_code(i));
      }
    }
  }
}


//#define SH4_DUMP_BUFFER

#ifdef SH4_DUMP_BUFFER
static int buffer_count = 0;
#endif

Assembler::~Assembler() {
#ifdef SH4_DUMP_BUFFER
  // dump the buffer on the disk
  printf("dumping a buffer %i\n", buffer_count++);
  char *psz_filename;
  asprintf(&psz_filename, "buffer-%d.st40", buffer_count);
  FILE *dump = fopen(psz_filename, "w");
  if (dump) {
    fwrite(buffer_, buffer_size_, 1, dump);
    fclose(dump);
  }
  free(psz_filename);
#endif

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

void Assembler::add_imm_(int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_add_imm(imm));
  emit((0x7 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((imm & 0xFF) << 0));
  asm_output("add_imm %d, R%d", imm, REGNUM(Rd));
}


void Assembler::add_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xC << 0));
  asm_output("add R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::addc_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xE << 0));
  asm_output("addc R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::addv_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xF << 0));
  asm_output("addv R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::and_imm_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_and_imm_R0(imm));
  emit((0xC << 12) | (0x9 << 8) | ((imm & 0xFF) << 0));
  asm_output("and_imm_R0 %d", imm);
}


void Assembler::and_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x9 << 0));
  asm_output("and R%d, R%d", REGNUM(Rs), REGNUM(Rd));
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


void Assembler::cmpeq_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x0 << 0));
  asm_output("cmpeq R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::cmpge_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x3 << 0));
  asm_output("cmpge R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::cmpgt_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x7 << 0));
  asm_output("cmpgt R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::cmphi_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x6 << 0));
  asm_output("cmphi R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::cmphs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x2 << 0));
  asm_output("cmphs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::cmppl_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x5 << 0));
  asm_output("cmppl R%d", REGNUM(Rd));
}


void Assembler::cmppz_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x1 << 0));
  asm_output("cmppz R%d", REGNUM(Rd));
}


void Assembler::cmpstr_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xC << 0));
  asm_output("cmpstr R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::div0s_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x7 << 0));
  asm_output("div0s R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::div0u_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x1 << 4) | (0x9 << 0));
  asm_output("div0u");
}


void Assembler::div1_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x4 << 0));
  asm_output("div1 R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::extsb_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xE << 0));
  asm_output("extsb R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::extsw_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xF << 0));
  asm_output("extsw R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::extub_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xC << 0));
  asm_output("extub R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::extuw_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xD << 0));
  asm_output("extuw R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::icbi_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xE << 4) | (0x3 << 0));
  asm_output("icbi_indRd R%d", REGNUM(Rd));
}


void Assembler::jmp_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0xB << 0));
  asm_output("jmp_indRd R%d", REGNUM(Rd));
}


void Assembler::jsr_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0xB << 0));
  asm_output("jsr_indRd R%d", REGNUM(Rd));
}


void Assembler::ldc_SR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0xE << 0));
  asm_output("ldc_SR R%d", REGNUM(Rd));
}


void Assembler::ldc_GBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0xE << 0));
  asm_output("ldc_GBR R%d", REGNUM(Rd));
}


void Assembler::ldc_SGR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0xA << 0));
  asm_output("ldc_SGR R%d", REGNUM(Rd));
}


void Assembler::ldc_VBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0xE << 0));
  asm_output("ldc_VBR R%d", REGNUM(Rd));
}


void Assembler::ldc_SSR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0xE << 0));
  asm_output("ldc_SSR R%d", REGNUM(Rd));
}


void Assembler::ldc_SPC_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x4 << 4) | (0xE << 0));
  asm_output("ldc_SPC R%d", REGNUM(Rd));
}


void Assembler::ldc_DBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xF << 4) | (0xA << 0));
  asm_output("ldc_DBR R%d", REGNUM(Rd));
}


void Assembler::ldc_bank_(Register Rd, int imm) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_ldc_bank(imm));
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((imm & 0x7) << 4) | (0xE << 0));
  asm_output("ldc_bank R%d, %d", REGNUM(Rd), imm);
}


void Assembler::ldcl_incRd_SR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x7 << 0));
  asm_output("ldcl_incRd_SR R%d", REGNUM(Rd));
}


void Assembler::ldcl_incRd_GBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x7 << 0));
  asm_output("ldcl_incRd_GBR R%d", REGNUM(Rd));
}


void Assembler::ldcl_incRd_VBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x7 << 0));
  asm_output("ldcl_incRd_VBR R%d", REGNUM(Rd));
}


void Assembler::ldcl_incRd_SGR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0x6 << 0));
  asm_output("ldcl_incRd_SGR R%d", REGNUM(Rd));
}


void Assembler::ldcl_incRd_SSR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0x7 << 0));
  asm_output("ldcl_incRd_SSR R%d", REGNUM(Rd));
}


void Assembler::ldcl_incRd_SPC_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x4 << 4) | (0x7 << 0));
  asm_output("ldcl_incRd_SPC R%d", REGNUM(Rd));
}


void Assembler::ldcl_incRd_DBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xF << 4) | (0x6 << 0));
  asm_output("ldcl_incRd_DBR R%d", REGNUM(Rd));
}


void Assembler::ldcl_incRd_bank_(Register Rd, int imm) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_ldcl_incRd_bank(imm));
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x7 << 0));
  asm_output("ldcl_incRd_bank R%d, %d", REGNUM(Rd), imm);
}


void Assembler::lds_MACH_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0xA << 0));
  asm_output("lds_MACH R%d", REGNUM(Rd));
}


void Assembler::lds_MACL_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0xA << 0));
  asm_output("lds_MACL R%d", REGNUM(Rd));
}


void Assembler::lds_PR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0xA << 0));
  asm_output("lds_PR R%d", REGNUM(Rd));
}


void Assembler::lds_FPUL_(Register Rs) {
  ASSERT(REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rs) & 0xF) << 8) | (0x5 << 4) | (0xA << 0));
  asm_output("lds_FPUL R%d", REGNUM(Rs));
}


void Assembler::lds_FPSCR_(Register Rs) {
  ASSERT(REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rs) & 0xF) << 8) | (0x6 << 4) | (0xA << 0));
  asm_output("lds_FPSCR R%d", REGNUM(Rs));
}


void Assembler::ldsl_incRd_MACH_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x6 << 0));
  asm_output("ldsl_incRd_MACH R%d", REGNUM(Rd));
}


void Assembler::ldsl_incRd_MACL_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x6 << 0));
  asm_output("ldsl_incRd_MACL R%d", REGNUM(Rd));
}


void Assembler::ldsl_incRd_PR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x6 << 0));
  asm_output("ldsl_incRd_PR R%d", REGNUM(Rd));
}


void Assembler::ldsl_incRs_FPUL_(Register Rs) {
  ASSERT(REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rs) & 0xF) << 8) | (0x5 << 4) | (0x6 << 0));
  asm_output("ldsl_incRs_FPUL R%d", REGNUM(Rs));
}


void Assembler::ldsl_incRs_FPSCR_(Register Rs) {
  ASSERT(REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rs) & 0xF) << 8) | (0x6 << 4) | (0x6 << 0));
  asm_output("ldsl_incRs_FPSCR R%d", REGNUM(Rs));
}


void Assembler::ldtlb_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x3 << 4) | (0x8 << 0));
  asm_output("ldtlb");
}


void Assembler::macw_incRs_incRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xF << 0));
  asm_output("macw_incRs_incRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::mov_imm_(int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_mov_imm(imm));
  emit((0xE << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((imm & 0xFF) << 0));
  asm_output("mov_imm %d, R%d", imm, REGNUM(Rd));
}


void Assembler::mov_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x3 << 0));
  asm_output("mov R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movb_dispR0Rd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_dispR0Rd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movb_decRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_decRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movb_indRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x0 << 0));
  asm_output("movb_indRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movb_dispRs_R0_(int imm, Register Rs) {
  ASSERT(REGNUM(Rs) <= 15 && SH4_CHECK_RANGE_movb_dispRs_R0(imm));
  emit((0x8 << 12) | (0x4 << 8) | ((REGNUM(Rs) & 0xF) << 4) | ((imm & 0xF) << 0));
  asm_output("movb_dispRs_R0 %d, R%d", imm, REGNUM(Rs));
}


void Assembler::movb_dispGBR_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movb_dispGBR_R0(imm));
  emit((0xC << 12) | (0x4 << 8) | ((imm & 0xFF) << 0));
  asm_output("movb_dispGBR_R0 %d", imm);
}


void Assembler::movb_dispR0Rs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xC << 0));
  asm_output("movb_dispR0Rs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movb_incRs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x4 << 0));
  asm_output("movb_incRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movb_indRs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x0 << 0));
  asm_output("movb_indRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movb_R0_dispRd_(int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_movb_R0_dispRd(imm));
  emit((0x8 << 12) | (0x0 << 8) | ((REGNUM(Rd) & 0xF) << 4) | ((imm & 0xF) << 0));
  asm_output("movb_R0_dispRd %d, R%d", imm, REGNUM(Rd));
}


void Assembler::movb_R0_dispGBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movb_R0_dispGBR(imm));
  emit((0xC << 12) | (0x0 << 8) | ((imm & 0xFF) << 0));
  asm_output("movb_R0_dispGBR %d", imm);
}


void Assembler::movl_dispRd_(Register Rs, int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15 && SH4_CHECK_RANGE_movl_dispRd(imm) && SH4_CHECK_ALIGN_movl_dispRd(imm));
  emit((0x1 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (((imm & 0x3C) >> 2) << 0));
  asm_output("movl_dispRd R%d, %d, R%d", REGNUM(Rs), imm, REGNUM(Rd));
}


void Assembler::movl_dispR0Rd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_dispR0Rd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movl_decRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_decRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movl_indRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x2 << 0));
  asm_output("movl_indRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movl_dispRs_(int imm, Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15 && SH4_CHECK_RANGE_movl_dispRs(imm) && SH4_CHECK_ALIGN_movl_dispRs(imm));
  emit((0x5 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (((imm & 0x3C) >> 2) << 0));
  asm_output("movl_dispRs %d, R%d, R%d", imm, REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movl_dispGBR_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movl_dispGBR_R0(imm) && SH4_CHECK_ALIGN_movl_dispGBR_R0(imm));
  emit((0xC << 12) | (0x6 << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_dispGBR_R0 %d", imm);
}


void Assembler::movl_dispPC_(int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_movl_dispPC(imm) && SH4_CHECK_ALIGN_movl_dispPC(imm));
  emit((0xD << 12) | ((REGNUM(Rd) & 0xF) << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_dispPC %d, R%d", imm, REGNUM(Rd));
}


void Assembler::movl_dispR0Rs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xE << 0));
  asm_output("movl_dispR0Rs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movl_incRs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x6 << 0));
  asm_output("movl_incRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movl_indRs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x2 << 0));
  asm_output("movl_indRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movl_R0_dispGBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movl_R0_dispGBR(imm) && SH4_CHECK_ALIGN_movl_R0_dispGBR(imm));
  emit((0xC << 12) | (0x2 << 8) | (((imm & 0x3FC) >> 2) << 0));
  asm_output("movl_R0_dispGBR %d", imm);
}


void Assembler::movw_dispR0Rd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_dispR0Rd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movw_decRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_decRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movw_indRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x1 << 0));
  asm_output("movw_indRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movw_dispRs_R0_(int imm, Register Rs) {
  ASSERT(REGNUM(Rs) <= 15 && SH4_CHECK_RANGE_movw_dispRs_R0(imm) && SH4_CHECK_ALIGN_movw_dispRs_R0(imm));
  emit((0x8 << 12) | (0x5 << 8) | ((REGNUM(Rs) & 0xF) << 4) | (((imm & 0x1E) >> 1) << 0));
  asm_output("movw_dispRs_R0 %d, R%d", imm, REGNUM(Rs));
}


void Assembler::movw_dispGBR_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_movw_dispGBR_R0(imm) && SH4_CHECK_ALIGN_movw_dispGBR_R0(imm));
  emit((0xC << 12) | (0x5 << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("movw_dispGBR_R0 %d", imm);
}


void Assembler::movw_dispPC_(int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_movw_dispPC(imm) && SH4_CHECK_ALIGN_movw_dispPC(imm));
  emit((0x9 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (((imm & 0x1FE) >> 1) << 0));
  asm_output("movw_dispPC %d, R%d", imm, REGNUM(Rd));
}


void Assembler::movw_dispR0Rs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xD << 0));
  asm_output("movw_dispR0Rs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movw_incRs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x5 << 0));
  asm_output("movw_incRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movw_indRs_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x1 << 0));
  asm_output("movw_indRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::movw_R0_dispRd_(int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_movw_R0_dispRd(imm) && SH4_CHECK_ALIGN_movw_R0_dispRd(imm));
  emit((0x8 << 12) | (0x1 << 8) | ((REGNUM(Rd) & 0xF) << 4) | (((imm & 0x1E) >> 1) << 0));
  asm_output("movw_R0_dispRd %d, R%d", imm, REGNUM(Rd));
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


void Assembler::movcal_R0_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xC << 4) | (0x3 << 0));
  asm_output("movcal_R0_indRd R%d", REGNUM(Rd));
}


void Assembler::movcol_R0_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x7 << 4) | (0x3 << 0));
  asm_output("movcol_R0_indRd R%d", REGNUM(Rd));
}


void Assembler::movlil_indRs_R0_(Register Rs) {
  ASSERT(REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rs) & 0xF) << 8) | (0x6 << 4) | (0x3 << 0));
  asm_output("movlil_indRs_R0 R%d", REGNUM(Rs));
}


void Assembler::movt_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x9 << 0));
  asm_output("movt R%d", REGNUM(Rd));
}


void Assembler::movual_indRs_R0_(Register Rs) {
  ASSERT(REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rs) & 0xF) << 8) | (0xA << 4) | (0x9 << 0));
  asm_output("movual_indRs_R0 R%d", REGNUM(Rs));
}


void Assembler::movual_incRs_R0_(Register Rs) {
  ASSERT(REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rs) & 0xF) << 8) | (0xE << 4) | (0x9 << 0));
  asm_output("movual_incRs_R0 R%d", REGNUM(Rs));
}


void Assembler::mulsw_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xF << 0));
  asm_output("mulsw R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::muls_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xF << 0));
  asm_output("muls R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::mull_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x7 << 0));
  asm_output("mull R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::muluw_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xE << 0));
  asm_output("muluw R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::mulu_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xE << 0));
  asm_output("mulu R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::neg_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xB << 0));
  asm_output("neg R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::negc_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xA << 0));
  asm_output("negc R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::nop_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x0 << 4) | (0x9 << 0));
  asm_output("nop");
}


void Assembler::not_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x7 << 0));
  asm_output("not R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::ocbi_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x9 << 4) | (0x3 << 0));
  asm_output("ocbi_indRd R%d", REGNUM(Rd));
}


void Assembler::ocbp_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xA << 4) | (0x3 << 0));
  asm_output("ocbp_indRd R%d", REGNUM(Rd));
}


void Assembler::ocbwb_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xB << 4) | (0x3 << 0));
  asm_output("ocbwb_indRd R%d", REGNUM(Rd));
}


void Assembler::or_imm_R0_(int imm) {
  ASSERT(SH4_CHECK_RANGE_or_imm_R0(imm));
  emit((0xC << 12) | (0xB << 8) | ((imm & 0xFF) << 0));
  asm_output("or_imm_R0 %d", imm);
}


void Assembler::or_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xB << 0));
  asm_output("or R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::orb_imm_dispR0GBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_orb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xF << 8) | ((imm & 0xFF) << 0));
  asm_output("orb_imm_dispR0GBR %d", imm);
}


void Assembler::pref_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x8 << 4) | (0x3 << 0));
  asm_output("pref_indRd R%d", REGNUM(Rd));
}


void Assembler::prefi_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xD << 4) | (0x3 << 0));
  asm_output("prefi_indRd R%d", REGNUM(Rd));
}


void Assembler::rotcl_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x4 << 0));
  asm_output("rotcl R%d", REGNUM(Rd));
}


void Assembler::rotcr_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x5 << 0));
  asm_output("rotcr R%d", REGNUM(Rd));
}


void Assembler::rotl_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x4 << 0));
  asm_output("rotl R%d", REGNUM(Rd));
}


void Assembler::rotr_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x5 << 0));
  asm_output("rotr R%d", REGNUM(Rd));
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


void Assembler::shad_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xC << 0));
  asm_output("shad R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::shld_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xD << 0));
  asm_output("shld R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::shal_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x0 << 0));
  asm_output("shal R%d", REGNUM(Rd));
}


void Assembler::shar_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x1 << 0));
  asm_output("shar R%d", REGNUM(Rd));
}


void Assembler::shll_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x0 << 0));
  asm_output("shll R%d", REGNUM(Rd));
}


void Assembler::shll16_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x8 << 0));
  asm_output("shll16 R%d", REGNUM(Rd));
}


void Assembler::shll2_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x8 << 0));
  asm_output("shll2 R%d", REGNUM(Rd));
}


void Assembler::shll8_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x8 << 0));
  asm_output("shll8 R%d", REGNUM(Rd));
}


void Assembler::shlr_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x1 << 0));
  asm_output("shlr R%d", REGNUM(Rd));
}


void Assembler::shlr16_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x9 << 0));
  asm_output("shlr16 R%d", REGNUM(Rd));
}


void Assembler::shlr2_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x9 << 0));
  asm_output("shlr2 R%d", REGNUM(Rd));
}


void Assembler::shlr8_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x9 << 0));
  asm_output("shlr8 R%d", REGNUM(Rd));
}


void Assembler::sleep_() {
  emit((0x0 << 12) | (0x0 << 8) | (0x1 << 4) | (0xB << 0));
  asm_output("sleep");
}


void Assembler::stc_SR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x2 << 0));
  asm_output("stc_SR R%d", REGNUM(Rd));
}


void Assembler::stc_GBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x2 << 0));
  asm_output("stc_GBR R%d", REGNUM(Rd));
}


void Assembler::stc_VBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x2 << 0));
  asm_output("stc_VBR R%d", REGNUM(Rd));
}


void Assembler::stc_SSR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0x2 << 0));
  asm_output("stc_SSR R%d", REGNUM(Rd));
}


void Assembler::stc_SPC_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x4 << 4) | (0x2 << 0));
  asm_output("stc_SPC R%d", REGNUM(Rd));
}


void Assembler::stc_SGR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0xA << 0));
  asm_output("stc_SGR R%d", REGNUM(Rd));
}


void Assembler::stc_DBR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xF << 4) | (0xA << 0));
  asm_output("stc_DBR R%d", REGNUM(Rd));
}


void Assembler::stc_bank_(int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_stc_bank(imm));
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x2 << 0));
  asm_output("stc_bank %d, R%d", imm, REGNUM(Rd));
}


void Assembler::stcl_SR_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x3 << 0));
  asm_output("stcl_SR_decRd R%d", REGNUM(Rd));
}


void Assembler::stcl_VBR_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x3 << 0));
  asm_output("stcl_VBR_decRd R%d", REGNUM(Rd));
}


void Assembler::stcl_SSR_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0x3 << 0));
  asm_output("stcl_SSR_decRd R%d", REGNUM(Rd));
}


void Assembler::stcl_SPC_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x4 << 4) | (0x3 << 0));
  asm_output("stcl_SPC_decRd R%d", REGNUM(Rd));
}


void Assembler::stcl_GBR_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x3 << 0));
  asm_output("stcl_GBR_decRd R%d", REGNUM(Rd));
}


void Assembler::stcl_SGR_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0x2 << 0));
  asm_output("stcl_SGR_decRd R%d", REGNUM(Rd));
}


void Assembler::stcl_DBR_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0xF << 4) | (0x2 << 0));
  asm_output("stcl_DBR_decRd R%d", REGNUM(Rd));
}


void Assembler::stcl_bank_decRd_(int imm, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && SH4_CHECK_RANGE_stcl_bank_decRd(imm));
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((imm & 0x7) << 4) | (0x3 << 0));
  asm_output("stcl_bank_decRd %d, R%d", imm, REGNUM(Rd));
}


void Assembler::sts_MACH_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0xA << 0));
  asm_output("sts_MACH R%d", REGNUM(Rd));
}


void Assembler::sts_MACL_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0xA << 0));
  asm_output("sts_MACL R%d", REGNUM(Rd));
}


void Assembler::sts_PR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0xA << 0));
  asm_output("sts_PR R%d", REGNUM(Rd));
}


void Assembler::sts_FPUL_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x5 << 4) | (0xA << 0));
  asm_output("sts_FPUL R%d", REGNUM(Rd));
}


void Assembler::sts_FPSCR_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x6 << 4) | (0xA << 0));
  asm_output("sts_FPSCR R%d", REGNUM(Rd));
}


void Assembler::stsl_MACH_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x2 << 0));
  asm_output("stsl_MACH_decRd R%d", REGNUM(Rd));
}


void Assembler::stsl_MACL_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x2 << 0));
  asm_output("stsl_MACL_decRd R%d", REGNUM(Rd));
}


void Assembler::stsl_PR_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x2 << 0));
  asm_output("stsl_PR_decRd R%d", REGNUM(Rd));
}


void Assembler::stsl_FPUL_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x5 << 4) | (0x2 << 0));
  asm_output("stsl_FPUL_decRd R%d", REGNUM(Rd));
}


void Assembler::stsl_FPSCR_decRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x6 << 4) | (0x2 << 0));
  asm_output("stsl_FPSCR_decRd R%d", REGNUM(Rd));
}


void Assembler::sub_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x8 << 0));
  asm_output("sub R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::subc_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xA << 0));
  asm_output("subc R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::subv_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xB << 0));
  asm_output("subv R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::swapb_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x8 << 0));
  asm_output("swapb R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::swapw_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x6 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x9 << 0));
  asm_output("swapw R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::synco_() {
  emit((0x0 << 12) | (0x0 << 8) | (0xA << 4) | (0xB << 0));
  asm_output("synco");
}


void Assembler::tasb_indRd_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0xB << 0));
  asm_output("tasb_indRd R%d", REGNUM(Rd));
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


void Assembler::tst_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x8 << 0));
  asm_output("tst R%d, R%d", REGNUM(Rs), REGNUM(Rd));
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


void Assembler::xor_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xA << 0));
  asm_output("xor R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::xorb_imm_dispR0GBR_(int imm) {
  ASSERT(SH4_CHECK_RANGE_xorb_imm_dispR0GBR(imm));
  emit((0xC << 12) | (0xE << 8) | ((imm & 0xFF) << 0));
  asm_output("xorb_imm_dispR0GBR %d", imm);
}


void Assembler::xtrct_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x2 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xD << 0));
  asm_output("xtrct R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::dt_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x4 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0x0 << 0));
  asm_output("dt R%d", REGNUM(Rd));
}


void Assembler::dmulsl_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xD << 0));
  asm_output("dmulsl R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::dmulul_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x3 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x5 << 0));
  asm_output("dmulul R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::macl_incRs_incRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xF << 0));
  asm_output("macl_incRs_incRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::braf_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0x3 << 0));
  asm_output("braf R%d", REGNUM(Rd));
}


void Assembler::bsrf_(Register Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0x0 << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0x3 << 0));
  asm_output("bsrf R%d", REGNUM(Rd));
}


void Assembler::fabs_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x5 << 4) | (0xD << 0));
  asm_output("fabs R%d", REGNUM(Rd));
}


void Assembler::fabs_double_(DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (0x5 << 4) | (0xD << 0));
  asm_output("fabs_double R%d", REGNUM(Rd));
}


void Assembler::fadd_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x0 << 0));
  asm_output("fadd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fadd_double_(DwVfpRegister Rs, DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1) && (REGNUM(Rs) * 2) <= 15 && !((REGNUM(Rs) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (((REGNUM(Rs) * 2) & 0xF) << 4) | (0x0 << 0));
  asm_output("fadd_double R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fcmpeq_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x4 << 0));
  asm_output("fcmpeq R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fcmpeq_double_(DwVfpRegister Rs, DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1) && (REGNUM(Rs) * 2) <= 15 && !((REGNUM(Rs) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (((REGNUM(Rs) * 2) & 0xF) << 4) | (0x4 << 0));
  asm_output("fcmpeq_double R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fcmpgt_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x5 << 0));
  asm_output("fcmpgt R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fcmpgt_double_(DwVfpRegister Rs, DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1) && (REGNUM(Rs) * 2) <= 15 && !((REGNUM(Rs) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (((REGNUM(Rs) * 2) & 0xF) << 4) | (0x5 << 0));
  asm_output("fcmpgt_double R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fcnvds_double_FPUL_(DwVfpRegister Rd) {
  ASSERT(!((REGNUM(Rd) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (0xB << 4) | (0xD << 0));
  asm_output("fcnvds_double_FPUL R%d", REGNUM(Rd));
}


void Assembler::fcnvsd_FPUL_double_(DwVfpRegister Rd) {
  ASSERT(!((REGNUM(Rd) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (0xA << 4) | (0xD << 0));
  asm_output("fcnvsd_FPUL_double R%d", REGNUM(Rd));
}


void Assembler::fdiv_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x3 << 0));
  asm_output("fdiv R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fdiv_double_(DwVfpRegister Rs, DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1) && (REGNUM(Rs) * 2) <= 15 && !((REGNUM(Rs) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (((REGNUM(Rs) * 2) & 0xF) << 4) | (0x3 << 0));
  asm_output("fdiv_double R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fipr_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(!((REGNUM(Rd) & 0x3) || (REGNUM(Rs) & 0x3)));
  emit((0xF << 12) | ((((REGNUM(Rd) & 0xF) << 2) | ((REGNUM(Rs) & 0xF) >> 2)) << 8) | (0xE << 4) | (0xD << 0));
  asm_output("fipr R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fldi0_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x8 << 4) | (0xD << 0));
  asm_output("fldi0 R%d", REGNUM(Rd));
}


void Assembler::fldi1_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x9 << 4) | (0xD << 0));
  asm_output("fldi1 R%d", REGNUM(Rd));
}


void Assembler::flds_FPUL_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x1 << 4) | (0xD << 0));
  asm_output("flds_FPUL R%d", REGNUM(Rd));
}


void Assembler::float_FPUL_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x2 << 4) | (0xD << 0));
  asm_output("float_FPUL R%d", REGNUM(Rd));
}


void Assembler::float_FPUL_double_(DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (0x2 << 4) | (0xD << 0));
  asm_output("float_FPUL_double R%d", REGNUM(Rd));
}


void Assembler::fmac_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xE << 0));
  asm_output("fmac R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xC << 0));
  asm_output("fmov R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_Xdouble_Xdouble_(DwVfpRegister Rs, DwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xC << 0));
  asm_output("fmov_Xdouble_Xdouble R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_indRs_(Register Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmov_indRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_indRs_Xdouble_(Register Rs, DwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmov_indRs_Xdouble R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_indRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xA << 0));
  asm_output("fmov_indRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_Xdouble_indRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xA << 0));
  asm_output("fmov_Xdouble_indRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_incRs_(Register Rs, DwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmov_incRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_decRd_(DwVfpRegister Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xB << 0));
  asm_output("fmov_decRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_dispR0Rs_(Register Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmov_dispR0Rs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_dispR0Rs_Xdouble_(Register Rs, DwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmov_dispR0Rs_Xdouble R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_dispR0Rd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmov_dispR0Rd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmov_Xdouble_dispR0Rd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmov_Xdouble_dispR0Rd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovd_indRs_Xdouble_(Register Rs, DwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmovd_indRs_Xdouble R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovd_Xdouble_indRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xA << 0));
  asm_output("fmovd_Xdouble_indRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovd_incRs_Xdouble_(Register Rs, DwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmovd_incRs_Xdouble R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovd_Xdouble_decRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xB << 0));
  asm_output("fmovd_Xdouble_decRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovd_dispR0Rs_Xdouble_(Register Rs, DwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmovd_dispR0Rs_Xdouble R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovd_Xdouble_dispR0Rd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmovd_Xdouble_dispR0Rd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovs_indRs_(Register Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x8 << 0));
  asm_output("fmovs_indRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovs_indRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xA << 0));
  asm_output("fmovs_indRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovs_incRs_(Register Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x9 << 0));
  asm_output("fmovs_incRs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovs_decRd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0xB << 0));
  asm_output("fmovs_decRd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovs_dispR0Rs_(Register Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x6 << 0));
  asm_output("fmovs_dispR0Rs R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmovs_dispR0Rd_(Register Rs, Register Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x7 << 0));
  asm_output("fmovs_dispR0Rd R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmul_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x2 << 0));
  asm_output("fmul R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fmul_double_(DwVfpRegister Rs, DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1) && (REGNUM(Rs) * 2) <= 15 && !((REGNUM(Rs) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (((REGNUM(Rs) * 2) & 0xF) << 4) | (0x2 << 0));
  asm_output("fmul_double R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fneg_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x4 << 4) | (0xD << 0));
  asm_output("fneg R%d", REGNUM(Rd));
}


void Assembler::fneg_double_(DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (0x4 << 4) | (0xD << 0));
  asm_output("fneg_double R%d", REGNUM(Rd));
}


void Assembler::fpchg_() {
  emit((0xF << 12) | (0x7 << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fpchg");
}


void Assembler::frchg_() {
  emit((0xF << 12) | (0xB << 8) | (0xF << 4) | (0xD << 0));
  asm_output("frchg");
}


void Assembler::fsca_FPUL_double_(DwVfpRegister Rd) {
  ASSERT(!((REGNUM(Rd) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fsca_FPUL_double R%d", REGNUM(Rd));
}


void Assembler::fschg_() {
  emit((0xF << 12) | (0x3 << 8) | (0xF << 4) | (0xD << 0));
  asm_output("fschg");
}


void Assembler::fsqrt_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x6 << 4) | (0xD << 0));
  asm_output("fsqrt R%d", REGNUM(Rd));
}


void Assembler::fsqrt_double_(DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (0x6 << 4) | (0xD << 0));
  asm_output("fsqrt_double R%d", REGNUM(Rd));
}


void Assembler::fsrra_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x7 << 4) | (0xD << 0));
  asm_output("fsrra R%d", REGNUM(Rd));
}


void Assembler::fsts_FPUL_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x0 << 4) | (0xD << 0));
  asm_output("fsts_FPUL R%d", REGNUM(Rd));
}


void Assembler::fsub_(SwVfpRegister Rs, SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15 && REGNUM(Rs) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | ((REGNUM(Rs) & 0xF) << 4) | (0x1 << 0));
  asm_output("fsub R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::fsub_double_(DwVfpRegister Rs, DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1) && (REGNUM(Rs) * 2) <= 15 && !((REGNUM(Rs) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (((REGNUM(Rs) * 2) & 0xF) << 4) | (0x1 << 0));
  asm_output("fsub_double R%d, R%d", REGNUM(Rs), REGNUM(Rd));
}


void Assembler::ftrc_FPUL_(SwVfpRegister Rd) {
  ASSERT(REGNUM(Rd) <= 15);
  emit((0xF << 12) | ((REGNUM(Rd) & 0xF) << 8) | (0x3 << 4) | (0xD << 0));
  asm_output("ftrc_FPUL R%d", REGNUM(Rd));
}


void Assembler::ftrc_double_FPUL_(DwVfpRegister Rd) {
  ASSERT((REGNUM(Rd) * 2) <= 15 && !((REGNUM(Rd) * 2) & 0x1));
  emit((0xF << 12) | (((REGNUM(Rd) * 2) & 0xF) << 8) | (0x3 << 4) | (0xD << 0));
  asm_output("ftrc_double_FPUL R%d", REGNUM(Rd));
}


void Assembler::ftrv_(Register Rd) {
  ASSERT(!(REGNUM(Rd) & 0x3));
  emit((0xF << 12) | ((((REGNUM(Rd) & 0xF) << 2) | 0x1) << 8) | (0xF << 4) | (0xD << 0));
  asm_output("ftrv R%d", REGNUM(Rd));
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
