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

  // Fill the buffer with 0 so it will normally crash if we jump into it
  if (own_buffer_) {
    memset(buffer_, 0x00, buffer_size);
  }

  // Setup buffer pointers.
  ASSERT(buffer_ != NULL);
  pc_ = buffer_;
  reloc_info_writer.Reposition(buffer_ + buffer_size, pc_);

  last_pc_ = NULL;
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
  ASSERT(rmode != RelocInfo::NONE);
  // Don't record external references unless the heap will be serialized.
  if (rmode == RelocInfo::EXTERNAL_REFERENCE) {
#ifdef DEBUG
    if (!Serializer::enabled()) {
      Serializer::TooLateToEnableNow();
    }
#endif
    if (!Serializer::enabled() && !emit_debug_code()) {
      return;
    }
  }
  RelocInfo rinfo(pc_, rmode, data);  // we do not try to reuse pool constants
  reloc_info_writer.Write(&rinfo);
}


void Assembler::add(Register Rd, const Immediate& imm) {
  if (imm.is_int8()) {
    add_imm_(imm.x_, Rd);
  } else {
    ASSERT(!Rd.is(rtmp));
    // Use a super scratch register (r3) and a tiny constant pool
    align();
    movl_dispPC_(4, rtmp);
    nop_();
    bra_(4);
    add_(rtmp, Rd);
    *reinterpret_cast<uint32_t*>(pc_) = imm.x_;
    pc_ += sizeof(uint32_t);
  }
}


void Assembler::add(Register Rd, Register Rs, const Immediate& imm) {
  if (Rs.code() != Rd.code()) {
    mov(Rd, imm);
    add_(Rs, Rd);
  } else {
    add(Rd, imm);
  }
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


void Assembler::asl(Register Rd, Register Rs, const Immediate& imm) {
  if (Rs.code() != Rd.code())
    mov(Rd, Rs);
  if (imm.x_ == 1) {
    shal_(Rd);
  } else {
    ASSERT(!Rd.is(rtmp));
    mov(rtmp, imm);
    shad_(rtmp, Rd);
  }
}


void Assembler::asr(Register Rd, Register Rs, const Immediate& imm) {
  if (Rs.code() != Rd.code())
    mov(Rd, Rs);
  if (imm.x_ == 1) {
    shal_(Rd);
  } else {
    mov(rtmp, Immediate(32 - imm.x_));
    shad_(rtmp, Rd);
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
    ASSERT(!Rd.is(rtmp));
    mov(rtmp, imm);
    shld_(rtmp, Rd);
  }
}


void Assembler::lsl(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() != Rd.code())
    mov_(Rs, Rd);
  shld_(Rt, Rd);
}


void Assembler::lsr(Register Rd, Register Rs, const Immediate& imm) {
  if (Rs.code() != Rd.code())
    mov_(Rs, Rd);
  if (imm.x_ == 1) {
    shlr_(Rd);
  } else if (imm.x_ == 2) {
    shlr2_(Rd);
  } else {
    ASSERT(!Rd.is(rtmp));
    mov(rtmp, Immediate(32 - imm.x_));
    shld_(rtmp, Rd);
  }
}

void Assembler::land(Register Rd, Register Rs, const Immediate& imm) {
  if (Rd.is(r0) && Rd.is(Rs) && FITS_SH4_and_imm_R0(imm.x_)) {
    and_imm_R0_(imm.x_);
  } else {
    ASSERT(!Rs.is(rtmp));
    mov(rtmp, imm);
    land(Rd, Rs, rtmp);
  }
}

void Assembler::land(Register Rd, Register Rs, Register Rt) {
  if (!Rd.is(Rs))
    mov_(Rs, Rd);
  and_(Rt, Rd);
}

void Assembler::lor(Register Rd, Register Rs, const Immediate& imm) {
  if (Rd.is(r0) && Rd.is(Rs) && FITS_SH4_or_imm_R0(imm.x_)) {
    or_imm_R0_(imm.x_);
  } else {
    ASSERT(!Rs.is(rtmp));
    mov(rtmp, imm);
    lor(Rd, Rs, rtmp);
  }
}

void Assembler::lor(Register Rd, Register Rs, Register Rt) {
  if (!Rd.is(Rs))
    mov_(Rs, Rd);
  or_(Rt, Rd);
}

void Assembler::lxor(Register Rd, Register Rs, const Immediate& imm) {
  if (Rd.is(r0) && Rd.is(Rs) && FITS_SH4_xor_imm_R0(imm.x_)) {
    xor_imm_R0_(imm.x_);
  } else {
    ASSERT(!Rs.is(rtmp));
    mov(rtmp, imm);
    lxor(Rd, Rs, rtmp);
  }
}

void Assembler::lxor(Register Rd, Register Rs, Register Rt) {
  if (!Rd.is(Rs))
    mov_(Rs, Rd);
  xor_(Rt, Rd);
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


void Assembler::dw(uint16_t data) {
  CheckBuffer();
  *reinterpret_cast<uint16_t*>(pc_) = data;
  pc_ += sizeof(uint16_t);
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

  int target_pos = (int)pc_;

  // List the linked to patch
  while (L->is_linked()) {
    // Compute the current position
    uint16_t* p_pos = reinterpret_cast<uint16_t*>(L->pos());

    // Compute the next before the patch
    next(L);

    // Patching
    patchBranchOffset(target_pos, p_pos);
  }
  L->bind_to(target_pos);

  // Keep track of the last bound label so we don't eliminate any instructions
  // before a bound label.
  if (target_pos > last_bound_pos_)
    last_bound_pos_ = target_pos;
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
    branch(L->pos() - (int)pc_, type, false);
  } else {
    if (L->is_linked()) {
      ASSERT(L->pos() != kEndOfChain);
      branch(L->pos(), type, true);
    } else {
      branch(kEndOfChain, type, true);   // Patched later on
    }
    // Compensate the place of the constant (sizeof(uint32_t))
    // Constant pool is always emited last in the sequence
    int pos = reinterpret_cast<int>(pc_) - sizeof(uint32_t);
    L->link_to(pos);  // Link to the constant
  }
}


void Assembler::jmp(Handle<Code> code, RelocInfo::Mode rmode) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  // TODO: make a faster sequence where the constant pool is
  // after the branch
  mov(rtmp, Immediate(reinterpret_cast<intptr_t>(code.location()), rmode));
  jmp_indRd_(rtmp);
  nop_();
}

void Assembler::jsr(Handle<Code> code, RelocInfo::Mode rmode) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  // TODO: make a faster sequence where the constant pool is
  // after the branch
  mov(rtmp, Immediate(reinterpret_cast<intptr_t>(code.location()), rmode));
  jsr_indRd_(rtmp);
  nop_();
}

void Assembler::branch(int offset, branch_type type, bool patched_later) {
  switch(type) {
  case branch_true:
    bt(offset, patched_later); break;
  case branch_false:
    bf(offset, patched_later); break;
  case branch_unconditional:
    jmp(offset, patched_later); break;
  case branch_subroutine:
    jsr(offset, patched_later); break;
  }
}


void Assembler::patchBranchOffset(int target_pos, uint16_t *p_constant) {
  // Patch the constant
  ASSERT(*(p_constant - 1) == 0x09);
  *reinterpret_cast<uint32_t*>(p_constant) = target_pos - (unsigned)p_constant;
}


void Assembler::bt(int offset, bool patched_later) {
  if (patched_later) {
    align();
    bf_(8);
    nop_();
    movl_dispPC_(4, r3);
    nop_();
    braf_(r3);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);
  } else {
    if(FITS_SH4_bt(offset - 4)) {
      bt_(offset - 4);
      nop_();
    } else {
      int nop_count = align();
      bf_(8);
      nop_();
      movl_dispPC_(4, r3);
      nop_();
      braf_(r3);
      nop_();
      *reinterpret_cast<uint32_t*>(pc_) = offset - 4 - 8 - 2*nop_count;
      pc_ += sizeof(uint32_t);
    }
  }
}


void Assembler::bf(int offset, bool patched_later) {
  //TODO: rename offset to pos and keep this meaning
  if (patched_later) {
    align();
    bt_(8);
    nop_();
    movl_dispPC_(4, r3);
    nop_();
    braf_(r3);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);
  } else {
    if (FITS_SH4_bf(offset - 4)) {
      bf_(offset - 4);
      nop_();
    } else {
      int nop_count = align();
      bt_(8);
      nop_();
      movl_dispPC_(4, r3);
      nop_();
      braf_(r3);
      nop_();
      *reinterpret_cast<uint32_t*>(pc_) = offset - 4 - 8 - 2*nop_count;
      pc_ += sizeof(uint32_t);
    }
  }
}


void Assembler::jmp(int offset, bool patched_later) {
  //TODO: rename offset to pos and keep this meaning

  // TODO: on other architectures we have:
  // positions_recorder()->WriteRecordedPositions();
  // check if this is necessary

  // Is it going to be pacthed later on
  if (patched_later) {
    // There is no way to know the size of the offset: take the worst case
    align();
    movl_dispPC_(4, r3);
    nop();
    braf_(r3);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);

  } else {
    // Does it fits in a bra offset
    if (FITS_SH4_bra(offset - 4)) {
      bra_(offset - 4);
      nop_();
    } else {
      int nop_count = align();
      movl_dispPC_(4, r3);
      nop();
      braf_(r3);
      nop_();
      *reinterpret_cast<uint32_t*>(pc_) = offset - 4 - 4 - 2*nop_count;
      pc_ += sizeof(uint32_t);
    }
  }
}

void Assembler::jsr(int offset, bool patched_later) {
  //TODO: rename offset to pos and keep this meaning

  // TODO: on other architectures we have:
  // positions_recorder()->WriteRecordedPositions();
  // check if this is necessary

  // Is it going to be pacthed later on
  if (patched_later) {
    // There is no way to know the size of the offset: take the worst case
    align();
    movl_dispPC_(4, r3);
    nop();
    bsrf_(r3);
    nop_();
    *reinterpret_cast<uint32_t*>(pc_) = offset;
    pc_ += sizeof(uint32_t);

  } else {
    // Does it fits in a bsr offset
    if (FITS_SH4_bsr(offset - 4)) {
      bsr_(offset - 4);
      nop_();
    } else {
      int nop_count = align();
      movl_dispPC_(4, r3);
      nop();
      bsrf_(r3);
      nop_();
      *reinterpret_cast<uint32_t*>(pc_) = offset - 4 - 4 - 2*nop_count;
      pc_ += sizeof(uint32_t);
    }
  }
}


void Assembler::mov(Register Rd, const Immediate& imm) {
  // FIXME(STM): Internal ref not handled
  ASSERT(imm.rmode_ != RelocInfo::INTERNAL_REFERENCE);

#ifdef DEBUG
  Address instr_address = pc_;
#endif

  // Record the relocation location.
  // Actually we record the PC of the instruction,
  // though the target address is encoded in the constant pool below.
  // If the code sequence changes, one must update
  // Assembler::target_address_address_at().
  if (imm.rmode_ != RelocInfo::NONE) RecordRelocInfo(imm.rmode_);

  // Move based on immediates can only be 8 bits long
  if (imm.is_int8() && imm.rmode_ == RelocInfo::NONE) {
    mov_imm_(imm.x_, Rd);
  } else {
    // Use a tiny constant pool and jump above
    align();
    movl_dispPC_(4, Rd);
    nop_();
    bra_(4);
    nop_();
#ifdef DEBUG
    if (imm.rmode_ != RelocInfo::NONE) {
      Address target_address = pc_;
      // Verify that target_address_address_at() is actually returning
      // the address where the target address for the instruction is stored.
      ASSERT(target_address == target_address_address_at(instr_address));
    }
#endif
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
  } else if(!src.rn_.is_valid()) {
    if (FITS_SH4_movl_dispRs(src.offset_)) {
      movl_dispRs_(src.offset_, src.rm_, Rd);
    } else {
      add(rtmp, src.rm_, Immediate(src.offset_));
      movl_indRs_(rtmp, Rd);
    }
  } else {
    add(rtmp, src.rm_, src.rn_);
    movl_indRs_(rtmp, Rd);
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


void Assembler::mul(Register Rd, Register Rs, Register Rt) {
  mull_(Rs, Rt);
  sts_MACL_(Rd);
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


// Exception-generating instructions and debugging support.
// Stops with a non-negative code less than kNumOfWatchedStops support
// enabling/disabling and a counter feature. See simulator-arm.h .
void Assembler::stop(const char* msg) {
  // TODO handle simulator based stuff (ref to ARM code)
  bkpt();
}


void Assembler::bkpt() {
  // Encode directly a BRK
  //dw(0x003b);

  // Use a privileged instruction
  ldtlb_();
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


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
