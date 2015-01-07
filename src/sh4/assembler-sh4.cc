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

#include "v8.h"

#if V8_TARGET_ARCH_SH4

#include "sh4/assembler-sh4-inl.h"
#include "disassembler.h"
#include "macro-assembler.h"
#include "serialize.h"
#include "disasm.h"

#include "checks-sh4.h"

namespace v8 {
namespace internal {

#ifdef DEBUG
bool CpuFeatures::initialized_ = false;
#endif
unsigned CpuFeatures::supported_ = 0;
unsigned CpuFeatures::found_by_runtime_probing_only_ = 0;
unsigned CpuFeatures::cross_compile_ = 0;


ExternalReference ExternalReference::cpu_features() {
  ASSERT(CpuFeatures::initialized_);
  return ExternalReference(&CpuFeatures::supported_);
}


// Get the CPU features enabled by the build. For cross compilation the
// preprocessor symbols CAN_USE_FPU_INSTRUCTIONS
// can be defined to enable FPU instructions when building the
// snapshot.
static unsigned CpuFeaturesImpliedByCompiler() {
  unsigned answer = 0;

  // SH4: Assume SH4-300 with FPU
  answer |= 1u << FPU;

  return answer;
}


void CpuFeatures::Probe(bool serializer_enabled) {
  uint64_t standard_features = static_cast<unsigned>(
      OS::CpuFeaturesImpliedByPlatform()) | CpuFeaturesImpliedByCompiler();
  ASSERT(supported_ == 0 ||
         (supported_ & standard_features) == standard_features);
#ifdef DEBUG
  initialized_ = true;
#endif

  // Get the features implied by the OS and the compiler settings. This is the
  // minimal set of features which is also alowed for generated code in the
  // snapshot.
  supported_ |= standard_features;

  if (serializer_enabled) {
    // No probing for features if we might serialize (generate snapshot).
    return;
  }
}


void CpuFeatures::PrintTarget() {
  const char* sh4_arch = "";
  const char* sh4_fpu = "";

#ifdef __sh__
  sh4_arch = " simulator";
#else  // __sh__
#endif  // __sh__

  printf("target%s %s\n", sh4_arch, sh4_fpu);
}


void CpuFeatures::PrintFeatures() {
  printf("FPU=%d\n", CpuFeatures::IsSupported(FPU));
}


const char* DwVfpRegister::AllocationIndexToString(int index) {
  ASSERT(index >= 0 && index < NumAllocatableRegisters());
  const char* const names[] = {
    "dr0",
    "dr2",
    "dr4",
    "dr6",
    "dr8",
    "dr10",
    "dr12",
    "dr14",
  };
  ASSERT(index >= 0 && (unsigned)index < sizeof(names)/sizeof(names[0]));
  return names[index];
}


// -----------------------------------------------------------------------------
// Implementation of RelocInfo

const int RelocInfo::kApplyMask = 0;


bool RelocInfo::IsCodedSpecially() {
  // The deserializer needs to know whether a pointer is specially coded.  Being
  // specially coded on ARM means that it is a movw/movt instruction.  We don't
  // generate those yet.
  // SH4: always use load constant, thus false.
  return false;
}


bool RelocInfo::IsInConstantPool() {
  return false;
}


void RelocInfo::PatchCode(byte* instructions, int instruction_count) {
  // Patch the code at the current address with the supplied instructions.
  Instr* pc = reinterpret_cast<Instr*>(pc_);
  Instr* instr = reinterpret_cast<Instr*>(instructions);
  for (int i = 0; i < instruction_count; i++) {
    *(pc + i) = *(instr + i);
  }

  // Indicate that code has changed.
  CPU::FlushICache(pc_, instruction_count * Assembler::kInstrSize);
}


// Patch the code at the current PC with a call to the target address.
// Additional guard instructions can be added if required.
void RelocInfo::PatchCodeWithCall(Address target, int guard_bytes) {
  // Patch the code at the current address with a call to the target.
  UNIMPLEMENTED();
}


// -----------------------------------------------------------------------------
// Implementation of Operand and MemOperand
// See assembler-sh4-inl.h for inlined constructors

Operand::Operand(Handle<Object> handle) {
  // Verify all Objects referred by code are NOT in new space.
  Object* obj = *handle;
  rm_ = no_reg;
  if (obj->IsHeapObject()) {
    ASSERT(!HeapObject::cast(obj)->GetHeap()->InNewSpace(obj));
    imm32_ = reinterpret_cast<intptr_t>(handle.location());
    rmode_ = RelocInfo::EMBEDDED_OBJECT;
  } else {
    // no relocation needed
    imm32_ = reinterpret_cast<intptr_t>(obj);
    rmode_ = RelocInfo::NONE32;
  }
}

// We have to use the temporary register for things that can be relocated even
// if they can be encoded in the ARM's 12 bits of immediate-offset instruction
// space.  There is no guarantee that the relocated location can be similarly
// encoded.
bool Operand::must_output_reloc_info(Isolate* isolate,
                                     const Assembler* assembler) const {
  if (rmode_ == RelocInfo::EXTERNAL_REFERENCE) {
    if (assembler != NULL && assembler->predictable_code_size()) return true;
    return Serializer::enabled(isolate);
  } else if (RelocInfo::IsNone(rmode_)) {
    return false;
  }
  return true;
}


void Assembler::memcpy(Register dst, Register src, Register count, Register scratch1, Register scratch2,
                       Register scratch3, Register scratch4) {
  align();
  mov_(r0, scratch4);

  mov_(dst, r0);
  add_(count, r0);

  sub_(dst, src);
  add_imm_(-1, src);

  shlr_(count);
  mov_(src, scratch3);

  movb_dispR0Rs_(src, scratch1);
  bfs_(8);  // odd

    add_imm_(-1, scratch3);
  tst_(count, count);

  bts_(12);  // end
    movb_decRd_(scratch1, r0);

  // even:
  movb_dispR0Rs_(src, scratch1);
  // odd:
  movb_dispR0Rs_(scratch3, scratch2);
  dt_(count);

  movb_decRd_(scratch1, r0);
  bfs_(-12);  // even
    movb_decRd_(scratch2, r0);

  mov_(scratch4, r0);
}

void Assembler::memcmp(Register left, Register right, Register length,
                       Register scratch1, Register scratch2, Label *not_equal) {
  Label loop;
  bind(&loop);
   movb_incRs_(left, scratch1);
   movb_incRs_(right, scratch2);
   cmpeq(scratch1, scratch2);
   bf(not_equal);
   dt_(length);
  bf(&loop);
}


Assembler::Assembler(Isolate* isolate, void* buffer, int buffer_size)
    : AssemblerBase(isolate, buffer, buffer_size),
      recorded_ast_id_(TypeFeedbackId::None()),
      positions_recorder_(this) {
  reloc_info_writer.Reposition(buffer_ + buffer_size_, pc_);
  num_pending_reloc_info_ = 0;
  next_buffer_check_ = 0;
  const_pool_blocked_nesting_ = 0;
  no_const_pool_before_ = 0;
  emiting_const_pool_ = false;
  first_const_pool_use_ = -1;
  last_const_pool_use_ = -1;
  last_bound_pos_ = 0;
  ClearRecordedAstId();
}


Assembler::~Assembler() {
  ASSERT(const_pool_blocked_nesting_ == 0);
}


void Assembler::GetCode(CodeDesc* desc) {
  // Emit constant pool if necessary.
  CheckConstPool(true, false);
  ASSERT(num_pending_reloc_info_ == 0);

  // Set up code descriptor.
  desc->buffer = buffer_;
  desc->buffer_size = buffer_size_;
  desc->instr_size = pc_offset();
  desc->reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();
  desc->origin = this;
}


void Assembler::Align(int m) {
  ASSERT(m >= 4 && IsPowerOf2(m));
  while ((pc_offset() & (m - 1)) != 0) {
    nop_();
  }
}


Condition Assembler::GetCondition(Instr instr) {
  ASSERT(IsCondBranch(instr));
  return (instr & 0x200) == 0x200 ?
    ne :        // bf|bf/s
    eq;         // bt|bt/s
}

bool Assembler::IsCondBranch(Instr instr) {
  // bt|bf|bt/s|bf/s instrs.
  return (instr & 0xF900) == 0x8900;
}

int Assembler::GetBranchOffset(Instr instr) {
  ASSERT(IsCondBranch(instr) || IsBra(instr));
  uint8_t disp = 0;
  if (IsCondBranch(instr))
    disp = instr & 0xFF;
  else if (IsBra(instr))
    disp = instr & 0xFFF;
  else
    UNREACHABLE();
  return (disp * 2) + 4;
}

bool Assembler::IsBt(Instr instr) {
  // bt instr (but not bt/s)
  return (instr & 0xFF00) == 0x8900;
}

bool Assembler::IsBf(Instr instr) {
  // bf instr (but not bf/s)
  return (instr & 0xFF00) == 0x8B00;
}

Register Assembler::GetRn(Instr instr) {
  ASSERT(IsCmpRegister(instr) || IsTstRegister(instr) || IsMovImmediate(instr));
  Register reg;
  // extract Rn from cmp/xx Rm, Rn
  reg.code_ = (instr & 0x0F00) >> 8;
  return reg;
}


Register Assembler::GetRm(Instr instr) {
  ASSERT(IsCmpRegister(instr) || IsTstRegister(instr));
  Register reg;
  // extract Rm from cmp/xx Rm, Rn
  reg.code_ = (instr & 0x00F0) >> 4;
  return reg;
}


bool Assembler::IsMovImmediate(Instr instr) {
  // mov #ii, Rn
  return (instr & 0xF000) == 0xE000;
}


bool Assembler::IsJsr(Instr instr) {
  // jsr @rx
  return (instr & 0xF0FF) == 0x400B;
}


bool Assembler::IsNop(Instr instr, int type) {
  ASSERT(type >= 0 && type <= 14); // Ref to ::nop()
  if (type == NON_MARKING_NOP) {
    // nop
    return instr == 0x0009;
  } else {
    // mov Rm, Rn
    return (instr & 0xF00F) == 0x6003 /*mov Rm, Rn*/ &&
      ((instr & 0x0F00) >> 8)/*Rn*/ == ((instr & 0x00F0) >> 4)/*Rm*/;
  }
}


bool Assembler::IsBra(Instr instr) {
  // bra #offset
  return (instr & 0xF000) == 0xA000;
}


bool Assembler::IsCmpRegister(Instr instr) {
  // cmp/eq Rm, Rn
  return (instr & 0xF00F) == 0x3000;
}


bool Assembler::IsCmpImmediate(Instr instr) {
  // cmp/eq #ii, R0
  return (instr & 0xFF00) == 0x8800;
}


bool Assembler::IsTstRegister(Instr instr) {
  // tst Rm, Rn
  return (instr & 0xF00F) == 0x2008;
}


Register Assembler::GetCmpImmediateRegister(Instr instr) {
  ASSERT(IsCmpImmediate(instr));
  // The instruction is cmpeq #ii, r0, return r0
  return r0;
}


int Assembler::GetCmpImmediateAsUnsigned(Instr instr) {
  ASSERT(IsCmpImmediate(instr));
  // The instruction is cmpeq #ii, r0, return 8-bit #ii as unsigned
  return (instr & 0xFF);
}


void Assembler::add(Register Rd, const Operand& src, Register rtmp) {
  if (src.is_reg()) {
    add_(src.rm(), Rd);
    return;
  }
  ASSERT(!src.is_reg());
  if (src.is_int8()) {
    add_imm_(src.imm32_, Rd);
  } else {
    ASSERT(!Rd.is(rtmp));
    mov(rtmp, src);
    add_(rtmp, Rd);
  }
}


void Assembler::add(Register Rd, Register Rs, const Operand& src,
                    Register rtmp) {
  if (src.is_reg()) {
    add(Rd, Rs, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  if (Rs.code() != Rd.code()) {
    mov(Rd, src);
    add_(Rs, Rd);
  } else {
    add(Rd, src, rtmp);
  }
}


void Assembler::add(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() == Rd.code())
    add_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    add_(Rs, Rd);
  } else {
    ASSERT(!Rs.is(Rd) && !Rt.is(Rd));
    mov_(Rs, Rd);
    add_(Rt, Rd);
  }
}


void Assembler::sub(Register Rd, Register Rs, const Operand& src,
                    Register rtmp) {
  if (src.is_reg()) {
    sub(Rd, Rs, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  mov(rtmp, src);
  if (Rs.code() == Rd.code()) {
    sub_(rtmp, Rd);
  } else {
    mov_(Rs, Rd);
    sub_(rtmp, Rd);
  }
}


void Assembler::sub(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() == Rd.code()) {
    sub_(Rt, Rd);
  } else if (Rt.code() == Rd.code()) {
    ASSERT(!Rs.is(Rd));
    neg_(Rt, Rd);
    add_(Rs, Rd);
  } else {
    ASSERT(!Rs.is(Rd) && !Rt.is(Rd));
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
    ASSERT(!Rs.is(Rd) && !Rt.is(Rd));
    mov_(Rs, Rd);
    addv_(Rt, Rd);
  }
}


void Assembler::addv(Register Rd, Register Rs, const Operand& src,
                     Register rtmp) {
  if (src.is_reg()) {
    addv(Rd, Rs, src.rm());
  } else {
    mov(rtmp, src);
    addv(Rd, Rs, rtmp);
  }
}

void Assembler::addc(Register Rd, Register Rs, Register Rt) {
  // Clear T bit before using addc
  clrt_();
  if (Rs.code() == Rd.code())
    addc_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    addc_(Rs, Rd);
  } else {
    ASSERT(!Rs.is(Rd) && !Rt.is(Rd));
    mov_(Rs, Rd);
    addc_(Rt, Rd);
  }
}


void Assembler::subv(Register Rd, Register Rs, Register Rt, Register rtmp) {
  if (Rs.code() == Rd.code())
    subv_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    ASSERT(!Rs.is(rtmp) && !Rt.is(rtmp));
    mov_(Rs, rtmp);
    subv_(Rt, rtmp);
    mov_(rtmp, Rd);
  } else {
    ASSERT(!Rs.is(Rd) && !Rt.is(Rd));
    mov_(Rs, Rd);
    subv_(Rt, Rd);
  }
}


void Assembler::subv(Register Rd, Register Rs, const Operand& src,
                     Register rtmp) {
  if (src.is_reg()) {
    subv(Rd, Rs, src.rm(), rtmp);
  } else {
    mov(rtmp, src);
    subv(Rd, Rs, rtmp, no_reg/*not used*/);
  }
}


void Assembler::subc(Register Rd, Register Rs, Register Rt, Register rtmp) {
  // Clear T bit before using subc
  clrt_();
  if (Rs.code() == Rd.code())
    subc_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    ASSERT(!Rs.is(rtmp) && !Rt.is(rtmp));
    mov_(Rs, rtmp);
    subc_(Rt, rtmp);
    mov_(rtmp, Rd);
  } else {
    ASSERT(!Rs.is(Rd) && !Rt.is(Rd));
    mov_(Rs, Rd);
    subc_(Rt, Rd);
  }
}


void Assembler::rsbv(Register Rd, Register Rs, const Operand& src,
    Register rtmp) {
  if (src.is_reg()) {
    rsbv(Rd, Rs, src.rm());
    return;
  }
  mov(rtmp, src);
  subv(Rd, rtmp, Rs);
}


void Assembler::rsbv(Register Rd, Register Rs, Register Rt) {
  subv(Rd, Rt, Rs);
}


void Assembler::asr(Register Rd, Register Rs, Register Rt, bool in_range, Register rtmp) {
  ASSERT(!Rs.is(rtmp) && !Rd.is(rtmp) && !Rt.is(rtmp));
  // If !in_range, we must clamp shift value to 31 max
  if (!in_range) {
    movt_(rtmp);
    push(rtmp);
    cmphi(Rt, Operand(31), rtmp); // Does modify the T bit
  }
  neg_(Rt, rtmp);
  if (!in_range) {
    bf_(0);
    mov_imm_(-31, rtmp);
  }
  if (Rs.code() != Rd.code()) {
    mov_(Rs, Rd);
  }
  shad_(rtmp, Rd); // Does not modify the T bit
  if (!in_range) {
    pop(rtmp);
    cmppl_(rtmp); // gives back t bit
  }
}


void Assembler::asr(Register Rd, Register Rs, const Operand& imm,
                    Register rtmp) {
  if (imm.is_reg())
    UNIMPLEMENTED();
  ASSERT(imm.imm32_ >= 0 && imm.imm32_ < 32);
  if (Rs.code() != Rd.code())
    mov_(Rs, Rd);
  // Note that we do not want to modify the T bit, hence shar is prohibited for >> 1
  ASSERT(!Rs.is(rtmp) && !Rd.is(rtmp));
  mov_imm_(-imm.imm32_, rtmp);
  shad_(rtmp, Rd);
}


void Assembler::lsl(Register Rd, Register Rs, const Operand& imm,
                    Register rtmp) {
  if (imm.is_reg())
    UNIMPLEMENTED();
  ASSERT(imm.imm32_ >= 0 && imm.imm32_ < 32);
  if (Rs.code() != Rd.code())
    mov_(Rs, Rd);
  if (imm.imm32_ == 1) {
    // SH4: shll must not be used here as it sets the T bit
    add(Rd, Rd, Rd); // x + x == x << 1
  } else if (imm.imm32_ == 2) {
    shll2_(Rd);
  } else {
    ASSERT(!Rs.is(rtmp) && !Rd.is(rtmp));
    mov_imm_(imm.imm32_, rtmp);
    shld_(rtmp, Rd);
  }
}

void Assembler::lsl(Register Rd, Register Rs, Register Rt, bool wrap, Register rtmp) {
  ASSERT(!Rs.is(rtmp) && !Rd.is(rtmp) && !Rt.is(rtmp));
  Register rshift = Rt;
  // If !in_range, we must flush in case of shift value >= 32
  if (!wrap) {
    movt_(rtmp);
    push(rtmp);
    cmphi(Rt, Operand(31), rtmp);
  }
  if (Rs.code() != Rd.code()) {
    if (Rt.is(Rd)) {
      rshift = rtmp;
      mov_(Rt, rtmp);
    }
    mov_(Rs, Rd);
  }
  shld_(rshift, Rd);
  if (!wrap) {
    bf_(0);
    // Nullify result for shift amount >= 32
    mov_imm_(0, Rd);
    pop(rtmp);
    cmppl_(rtmp); // gives back t bit
  }
}


void Assembler::lsr(Register Rd, Register Rs, const Operand& imm,
                    Register rtmp) {
  if (imm.is_reg())
    UNIMPLEMENTED();
  ASSERT(imm.imm32_ >= 0 && imm.imm32_ < 32);
  if (Rs.code() != Rd.code())
    mov_(Rs, Rd);
  // Note that we do not want to modify the T bit, hence shlr is prohibited for >> 1
  if (imm.imm32_ == 2) {
    shlr2_(Rd);
  } else {
    ASSERT(!Rs.is(rtmp) && !Rd.is(rtmp));
    mov_imm_(-imm.imm32_, rtmp);
    shld_(rtmp, Rd);
  }
}

void Assembler::lsr(Register Rd, Register Rs, Register Rt, bool in_range, Register rtmp) {
  ASSERT(!Rs.is(rtmp) && !Rd.is(rtmp) && !Rt.is(rtmp));
  // If !in_range, we must flush in case of shift value >= 32
  if (!in_range) {
    movt_(rtmp);
    push(rtmp);
    cmphi(Rt, Operand(31), rtmp);
  }
  neg_(Rt, rtmp);
  if (Rs.code() != Rd.code()) {
    mov_(Rs, Rd);
  }
  shld_(rtmp, Rd);
  if (!in_range) {
    bf_(0);
    // Nullify result for shift amount >= 32
    mov_imm_(0, Rd);
    pop(rtmp);
    cmppl_(rtmp); // gives back t bit
  }
}


void Assembler::land(Register Rd, Register Rs, const Operand& src,
                     Register rtmp) {
  if (src.is_reg()) {
    land(Rd, Rs, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  if (Rd.is(r0) && Rd.is(Rs) && FITS_SH4_and_imm_R0(src.imm32_)) {
    and_imm_R0_(src.imm32_);
  } else {
    ASSERT(!Rs.is(rtmp));
    if (!Rs.is(Rd)) {
      mov(Rd, src);
      land(Rd, Rs, Rd);
    } else {
      mov(rtmp, src);
      land(Rd, Rs, rtmp);
    }
  }
}


void Assembler::land(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() == Rd.code())
    and_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    and_(Rs, Rd);
  } else {
    ASSERT(!Rt.is(Rd) && !Rs.is(Rd));
    mov_(Rs, Rd);
    and_(Rt, Rd);
  }
}


void Assembler::lor(Register Rd, Register Rs, const Operand& src,
                    Register rtmp) {
  if (src.is_reg()) {
    lor(Rd, Rs, src.rm());
    return;
  }
  if (Rd.is(r0) && Rd.is(Rs) && FITS_SH4_or_imm_R0(src.imm32_)) {
    or_imm_R0_(src.imm32_);
  } else {
    ASSERT(!Rs.is(rtmp));
    if (!Rs.is(Rd)) {
      mov(Rd, src);
      lor(Rd, Rs, Rd);
    } else {
      mov(rtmp, src);
      lor(Rd, Rs, rtmp);
    }
  }
}

void Assembler::lor(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() == Rd.code())
    or_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    or_(Rs, Rd);
  } else {
    ASSERT(!Rt.is(Rd) && !Rs.is(Rd));
    mov_(Rs, Rd);
    or_(Rt, Rd);
  }
}

void Assembler::lor(Register Rd, Register Rs, Register Rt, Condition cond) {
  ASSERT(cond == ne || cond == eq);
  Label end;
  if (cond == eq)
    bf_near(&end);   // Jump after sequence if T bit is false
  else
    bt_near(&end);   // Jump after sequence if T bit is true
  lor(Rd, Rs, Rt);
  bind(&end);
}


void Assembler::lor(Register Rd, Register Rs, const Operand& src,
                    Condition cond, Register rtmp) {
  ASSERT(cond == ne || cond == eq);
  if (src.is_reg()) {
    lor(Rd, Rs, src.rm(), cond);
    return;
  }
  ASSERT(!src.is_reg());
  Label end;
  if (cond == eq)
    bf_near(&end);   // Jump after sequence if T bit is false
  else
    bt_near(&end);   // Jump after sequence if T bit is true
  lor(Rd, Rs, src, rtmp);
  bind(&end);
}


void Assembler::lxor(Register Rd, Register Rs, const Operand& src,
                     Register rtmp) {
  if (src.is_reg()) {
    lxor(Rd, Rs, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  if (Rd.is(r0) && Rd.is(Rs) && FITS_SH4_xor_imm_R0(src.imm32_)) {
    xor_imm_R0_(src.imm32_);
  } else {
    ASSERT(!Rs.is(rtmp));
    if (!Rs.is(Rd)) {
      mov(Rd, src);
      lxor(Rd, Rs, Rd);
    } else {
      mov(rtmp, src);
      lxor(Rd, Rs, rtmp);
    }
  }
}

void Assembler::lxor(Register Rd, Register Rs, Register Rt) {
  if (Rs.code() == Rd.code())
    xor_(Rt, Rd);
  else if (Rt.code() == Rd.code()) {
    xor_(Rs, Rd);
  } else {
    ASSERT(!Rt.is(Rd) && !Rs.is(Rd));
    mov_(Rs, Rd);
    xor_(Rt, Rd);
  }
}


void Assembler::tst(Register Rd, const Operand& src, Register rtmp) {
  if (src.is_reg()) {
    tst_(src.rm(), Rd);
  } else {
    ASSERT(!Rd.is(rtmp));
    mov(rtmp, src);
    tst_(rtmp, Rd);
  }

}

void Assembler::call(Label* L) {
  jsr(L);
}

void Assembler::emitPatchableNearBranch(uint16_t data) {
  // must not emit the constant pool here. caller has to BlockConstPoolFor()
  ASSERT(pc_offset() < next_buffer_check_);
  CheckBuffer();
  *reinterpret_cast<uint16_t*>(pc_) = data;
  pc_ += sizeof(uint16_t);
}

// The branch offsets emitted through this should move to the constant pool
void Assembler::ddLegacyBranchConst(uint32_t data) {
  // must not emit the constant pool here. caller has to BlockConstPoolFor()
  ASSERT(pc_offset() < next_buffer_check_);
  CheckBuffer();
  *reinterpret_cast<uint32_t*>(pc_) = data;
  pc_ += sizeof(uint32_t);
}


void Assembler::emitConstPool(uint32_t data) {
  CheckBuffer();
  *reinterpret_cast<uint32_t*>(pc_) = data;
  pc_ += sizeof(uint32_t);
}
void Assembler::bind(Label* L) {
  // label can only be bound once
  ASSERT(!L->is_bound());

  // Jump directly to the current PC
  int target_pos = reinterpret_cast<int>(pc_);
  int is_near_linked = L->is_near_linked();
  Label::Distance distance = is_near_linked ? Label::kNear : Label::kFar;

  // List the links to patch
  while (L->is_linked()) {
    // Compute the current position
    // L->pos() is the offset from the begining of the buffer
    uint16_t* p_pos = reinterpret_cast<uint16_t*>(L->pos() + buffer_);
    ASSERT((is_near_linked && (int)(*p_pos) % 4 != 0) || !is_near_linked);

    // Compute the next before the patch
    next(L, distance);

    // Patching
    // Is it a backtrack label or a classical one ?
    // In this case the LSB is set to 1
    if (!is_near_linked && (((signed)*p_pos) & 0x3) == 1) {
      ASSERT(((*reinterpret_cast<int16_t*>(p_pos)) & ~ 0x3) == kEndOfChain);
      *reinterpret_cast<uint32_t*>(p_pos) = target_pos - (unsigned)buffer_ + (Code::kHeaderSize - kHeapObjectTag);
    }
    else
      patchBranchOffset(target_pos, p_pos, is_near_linked);
  }
  L->bind_to(pc_offset());
  L->UnuseNear();

  // Keep track of the last bound label so we don't eliminate any instructions
  // before a bound label.
  if (pc_offset() > last_bound_pos_)
    last_bound_pos_ = pc_offset();

  // Unblock only for near branches
  if (is_near_linked) {
    EndBlockConstPool();
  }
}


void Assembler::next(Label* L, Label::Distance distance) {
  if (distance == Label::kNear) {
    ASSERT(L->is_near_linked());
    int16_t link = (*reinterpret_cast<uint16_t*>(L->pos() + buffer_)) & ~ 0x3;
    if (link > 0) {
      L->link_to(link & ~0x3);
    } else {
      ASSERT(link == kEndOfChain);
      L->Unuse();
    }
  } else {
    ASSERT(L->is_linked());
    int link = *reinterpret_cast<uint32_t*>(L->pos() + buffer_);
    if (link > 0) {
      L->link_to(link);
    } else {
      ASSERT((link & ~0x3) == kEndOfChain);
      L->Unuse();
    }
  }
}


void Assembler::load_label(Label* L) {
  BlockConstPoolFor(1);
  ASSERT(!L->is_bound());
  int offset = L->is_linked() ? L->pos() : kEndOfChain;
  ASSERT((offset % 4) == 0);
  mov(r0, Operand(offset + 0x1), true);
  L->link_to(pc_offset() - sizeof(uint32_t));
}


void Assembler::branch_local(Label* L, Register rtmp, branch_type type,
                             Label::Distance distance) {
  // SH4: Local branch, must not repcord position
  // Block the constant pool at the first near branch
  if (distance == Label::kNear && !L->is_near_linked() && !L->is_bound()) {
    StartBlockConstPool();
  }

  // when bound both near and far labels are represented the same way
  if (L->is_bound()) {
    ASSERT(L->pos() != kEndOfChain);
    BlockConstPoolScope scope(this);
    branch_local(L->pos() - pc_offset(), rtmp, type, distance, false);
  } else {
    // The only difference between Near and far label is in the
    // is_near_linked function.
    // For this reason, we set the near_link_pos to 1 and then we use the
    // generic link_to to know the position
    // Moreover the position of the linked branch will be altered when dumped
    // on memory to carry the type of branch to write when patching back
    if (distance == Label::kNear)
      L->link_to(1, Label::kNear);

    if (L->is_linked()) {
      ASSERT(L->pos() != kEndOfChain);
      branch_local(L->pos(), rtmp, type, distance, true);
    } else {
      branch_local(kEndOfChain, rtmp, type, distance, true);   // Patched later on
    }

    int pos;
    if (distance == Label::kFar) {
      // Compensate the place of the constant (sizeof(uint32_t))
      // Constant pool is always emited last in the sequence
      // the position is defined as an offset from the begining of the buffer
      pos = pc_offset() - sizeof(uint32_t);
    } else {
      pos = pc_offset() - sizeof(uint16_t);
    }
    L->link_to(pos);  // Link to the constant
  }
}


void Assembler::jmp(Register Rd) {
  positions_recorder()->WriteRecordedPositions(); // Record position of a jmp to code
  jmp_indRd_(Rd);
  nop_();
}

void Assembler::jsr(Register Rd, Condition cond) {
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  positions_recorder()->WriteRecordedPositions(); // Record position of a jmp to code
  jsr_indRd_(Rd);
  nop_();
  if (cond != al)
    bind(&skip);

}

void Assembler::jmp(Handle<Code> code, RelocInfo::Mode rmode, Register rtmp) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  positions_recorder()->WriteRecordedPositions(); // Record position of a jmp to code
  mov(rtmp, Operand(reinterpret_cast<intptr_t>(code.location()), rmode));
  jmp_indRd_(rtmp);
  nop_();
}


void Assembler::jsr_at_following_address(Register rtmp) {
  // Used by Code::PatchPlatformCodeAge().
  // This is a special sequence of jsr that must be used as in:
  //   jsr_at_following_address()
  //   align();
  //   dd(reinterpret_cast<uint32_t>(stub->instruction_start()))
  // It must be used in the following conditions:
  // - no pending constant pools,
  // - the following address must be aligned.
  // It generates a jsr to the address stored just after by
  // emit_code_stub_at_address().
  // This sequence length is constant, but an alignment must
  // follow.
  // The generate code will be:
  // start:
  // mov.l rtmp, @(pc+4 + alignment) // where alignment is 2 if end is misaligned
  // nop
  // jmp rtmp
  // nop
  // end:
  // align()
  // .word @stub // generated by dd(reinterpret_cast<uint32_t>(stub->instruction_start()))
  int start_offset = pc_offset();
  int alignment = (uintptr_t)pc_ % 4 == 0 ? 0: 4;
  movl_dispPC_(4 + alignment, rtmp);
  nop_();
  positions_recorder()->WriteRecordedPositions(); // Record position of a jmp to code
  jsr_indRd_(rtmp);
  nop_();
  // Must fix Code::PatchPlatformCodeAge() if this sequence length changes.
  USE(start_offset);
  ASSERT(pc_offset() - start_offset == 4 * kInstrSize);
}


void Assembler::jsr_at_breakpoint(uint32_t patch_location, uint32_t call_target, Register rtmp)
{
  // Used by BreakLocationIterator::SetDebugBreakAtReturn/Slot() functions.
  // This special sequence enforces:
  // - a constant length whatever the alignment, actually 8 instructions
  // - no change in the initial alignment, i.e. multiple of 4 bytes in length
  // - a standard mov target sequence at start such that target_address_at() works
  // - a standrd jsr sequence at end such that the return address is the address
  //   just after the sequence.
  // The sequence may be generated through a CodePatcher, hence we can't rely
  // on the pc_offset() for the alignment, thus we use the patch_location
  // value given instead.
  // The sequence must not generate additionl relocations, hence we use a raw
  // integer value for the call_target.
  bool aligned = (patch_location % 4 == 0);
  int start_offset = pc_offset();

  movl_dispPC_(4, rtmp);
  if (aligned) nop_();
  bra_(aligned? 4: 6);
  nop_();
  dd(call_target);
  if (!aligned) nop_();
  jsr_indRd_(rtmp);
  nop_();

  // Must change kDebugBreakSlotInstructions, kJSReturnSequenceInstructions and
  // kPatchDebugBreakSlotReturnOffset if the length changes.
  USE(start_offset);
  ASSERT(pc_offset() - start_offset == 8 * kInstrSize);
  ASSERT((pc_offset() - start_offset) % 4 == 0);

}


void Assembler::jsr(Handle<Code> code, RelocInfo::Mode rmode, Register rtmp) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  positions_recorder()->WriteRecordedPositions(); // Record position of a jsr to code
  mov(rtmp, Operand(reinterpret_cast<intptr_t>(code.location()), rmode));
  jsr_indRd_(rtmp);
  nop_();
}

void Assembler::branch_local(int offset, Register rtmp, branch_type type,
                             Label::Distance distance, bool patched_later) {
  // SH4: local branch, must not record position
  switch (type) {
  case branch_true:
    conditional_branch_local(offset, rtmp, distance, patched_later, true);
    break;
  case branch_false:
    conditional_branch_local(offset, rtmp, distance, patched_later, false);
    break;
  case branch_unconditional:
    jmp_local(offset, rtmp, distance, patched_later);
    break;
  case branch_subroutine:
    jsr_local(offset, rtmp, patched_later);
    break;
  }
}


void Assembler::patchBranchOffset(int target_pos, uint16_t *p_constant, int is_near_linked) {
  if (is_near_linked) {
    // The two least significant bits represent the type of branch (1, 2 or 3)
    ASSERT((*p_constant & 0x3) == 1 ||  // bt
           (*p_constant & 0x3) == 2 ||  // bf
           (*p_constant & 0x3) == 3);   // jmp
    int disp = target_pos - (unsigned)p_constant - 4;

    // select the right branch type
    switch ((*p_constant & 0x3)) {
      case 1:
        ASSERT(FITS_SH4_bt(disp));
        *p_constant = (0x8 << 12) | (0x9 << 8) | (((disp & 0x1FE) >> 1) << 0);
        break;
      case 2:
        ASSERT(FITS_SH4_bf(disp));
        *p_constant = (0x8 << 12) | (0xB << 8) | (((disp & 0x1FE) >> 1) << 0);
        break;
      case 3:
        ASSERT(FITS_SH4_bra(disp + 2));
        ASSERT(*(p_constant - 1) == 0x9);
        *(p_constant - 1) = (0xA << 12) | ((((disp + 2) & 0x1FFE) >> 1) << 0);
        *(p_constant) = 0x9;
        break;
      default:
        UNREACHABLE();
    }

  } else {
    // Patch the constant
    ASSERT(*(p_constant - 1) == 0x09);
    // Is it a jsr or any other branch ?
    if (*(p_constant - 2) == 0xa002)
      *reinterpret_cast<uint32_t*>(p_constant) = target_pos -
                                                 (unsigned)p_constant + 4;
    else
      *reinterpret_cast<uint32_t*>(p_constant) = target_pos -
                                                 (unsigned)p_constant;
  }
}


void Assembler::conditional_branch_local(int offset, Register rtmp,
                                         Label::Distance distance, bool patched_later,
                                         bool type) {
  // SH4: local branch. Must not record position.
  if (patched_later) {
    if (distance == Label::kNear) {
      align();
      // Use the 2 least significant bits to store the type of branch
      // We assume (and assert) that they always are null
      ASSERT((offset % 4) == 0);
      dw(offset + (type ? 0x1 : 0x2));
    } else {
      align();
      type ? bf_(12) : bt_(12);
      nop_();
      movl_dispPC_(4, rtmp);
      nop_();
      braf_(rtmp);
      nop_();
      dd(offset);
    }
  } else {
    if (FITS_SH4_bt(offset - 4)) {
      type ? bt_(offset - 4) : bf_(offset - 4);
      nop_();
    } else {
      int nop_count = align();
      type ? bf_(12) : bt_(12);
      nop_();
      movl_dispPC_(4, rtmp);
      nop_();
      braf_(rtmp);
      nop_();
      dd(offset - 4 - 8 - 2 * nop_count);
    }
  }

}


void Assembler::bkpt() {
  // Use a privileged instruction.
  // Will generate an illegal instruction exception code: 0x180.
  ldtlb_();
}


void Assembler::jmp_local(int offset, Register rtmp, Label::Distance distance, bool patched_later) {
  // SH4: local branch. Must not record position
  // Is it going to be pacthed later on
  if (patched_later) {
    if (distance == Label::kNear) {
      misalign();
      nop();
      ASSERT((offset % 4) == 0);
      dw(offset + 0x3);
    } else {
      // There is no way to know the size of the offset: take the worst case
      align();
      movl_dispPC_(4, rtmp);
      nop();
      braf_(rtmp);
      nop_();
      dd(offset);
    }
  } else {
    // Does it fits in a bra offset
    if (FITS_SH4_bra(offset - 4)) {
      bra_(offset - 4);
      nop_();
    } else {
      int nop_count = align();
      movl_dispPC_(4, rtmp);
      nop();
      braf_(rtmp);
      nop_();
      dd(offset - 4 - 4 - 2 * nop_count);
    }
  }
}


void Assembler::jsr_local(int offset, Register rtmp, bool patched_later) {
  // SH4: local branch. Must not record position
  // Is it going to be patched later on ?
  if (patched_later) {
    // There is no way to know the size of the offset: take the worst case
    align();
    movl_dispPC_(8, rtmp);
    nop();
    bsrf_(rtmp);
    nop_();
    bra_(4);
    nop_();
    dd(offset);
  } else {
    // Does it fits in a bsr offset
    if (FITS_SH4_bsr(offset - 4)) {
      bsr_(offset - 4);
      nop_();
    } else {
      int nop_count = align();
      movl_dispPC_(8, rtmp);
      nop();
      bsrf_(rtmp);
      nop_();
      bra_(4);
      nop_();
      dd(offset - 4 - 4 - 2 * nop_count);
    }
  }
}


void Assembler::mov(Register Rd, const Operand& src, bool force) {
  if (src.is_reg()) {
    ASSERT(force == false);
    mov(Rd, src.rm());
    return;
  }
  ASSERT(!Rd.is(pr));
  ASSERT(!src.is_reg());
  // FIXME(STM): Internal ref not handled
  ASSERT(src.rmode_ != RelocInfo::INTERNAL_REFERENCE);

  // Move based on immediates can only be 8 bits long
  if (force == false && (src.is_int8() && src.rmode_ == RelocInfo::NONE32)) {
    mov_imm_(src.imm32_, Rd);
  } else {
    // Use a tiny constant pool and jump above
    align();
#ifdef DEBUG
    int instr_address = pc_offset();
#endif
    // Record the relocation location (after the align).
    // Actually we record the PC of the instruction,
    // though the target address is encoded in the constant pool below.
    // If the code sequence changes, one must update
    // Assembler::target_address_address_at().
    if (src.rmode_ != RelocInfo::NONE32) RecordRelocInfo(src.rmode_);
    movl_dispPC_(4, Rd);
    nop_();
    bra_(4);
    nop_();
#ifdef DEBUG
    if (src.rmode_ != RelocInfo::NONE32) {
      Address target_address = pc_;
      // Verify that target_pointer_address_at() is actually returning
      // the address where the pointer for the instruction is stored.
      ASSERT(target_address ==
             target_pointer_address_at(
                reinterpret_cast<byte*>(buffer_ + instr_address)));
    }
#endif
    dd(src.imm32_);
  }
}

void Assembler::addpc(Register Rd, int offset, Register Pr) {
  // We compute a pc+offset value where the pc
  // is the pc after this code sequence.
  // In order to do this, we do a bsr and get the link register.
  ASSERT(Pr.is(pr));
  bsr_(0);
  nop_();
  sts_PR_(Rd);
  add_imm_(4+offset, Rd);
}

void Assembler::movpc(Register Pr) {
  // We move the pc after this code sequence into the
  // link register pr.
  ASSERT(Pr.is(pr));
  bsr_(0);
  nop_();
}


void Assembler::mov(Register Rd, Register Rs, Condition cond) {
  ASSERT(cond == ne || cond == eq || cond == al);
  // If cond is eq, we move Rs into Rd, otherwise, nop
  if (cond == eq)
    bf_(0);     // Jump after sequence if T bit is false
  else if (cond == ne)
    bt_(0);     // Jump after sequence if T bit is true
  if (Rs.is(pr)) {
    ASSERT(Rd.is_valid());
    sts_PR_(Rd);
  } else if (Rd.is(pr)) {
    ASSERT(Rs.is_valid());
    lds_PR_(Rs);
  } else {
    ASSERT(Rs.is_valid());
    ASSERT(Rd.is_valid());
    mov_(Rs, Rd);
  }

}


void Assembler::mov(Register Rd, const Operand& src, Condition cond) {
  ASSERT(cond == ne || cond == eq);
  if (src.is_reg()) {
    mov(Rd, src.rm(), cond);
    return;
  }
  ASSERT(!src.is_reg());
  if (FITS_SH4_mov_imm(src.imm32_)) {
    // If cond is eq, we move Rs into Rd, otherwise, nop
    if (cond == eq)
      bf_(0);           // Jump after sequence if T bit is false
    else
      bt_(0);           // Jump after sequence if T bit is true
    mov_imm_(src.imm32_, Rd);
  } else {
    Label skip;
    if (cond == eq)
      bf_near(&skip);
    else
      bt_near(&skip);
    mov(Rd, src);
    bind(&skip);
  }
}


void Assembler::mov(Register Rd, const MemOperand& src, Register rtmp) {
  ASSERT(src.mode_ == Offset);
  ASSERT(!Rd.is(pr));
  if (src.roffset_.is_valid()) {
    ASSERT(rtmp.is_valid());
    add(rtmp, src.rn_, src.roffset_);
    movl_indRs_(rtmp, Rd);
  } else {
    if (src.offset_ == 0) {
      movl_indRs_(src.rn_, Rd);
    } else if (FITS_SH4_movl_dispRs(src.offset_)) {
      movl_dispRs_(src.offset_, src.rn_, Rd);
    } else {
      ASSERT(rtmp.is_valid());
      add(rtmp, src.rn_, Operand(src.offset_));
      movl_indRs_(rtmp, Rd);
    }
  }
}


void Assembler::movsb(Register Rd, const MemOperand& src, Register rtmp) {
  ASSERT(src.mode_ == Offset);
  ASSERT(!Rd.is(pr));
  if (src.roffset_.is_valid()) {
    add(rtmp, src.rn_, src.roffset_);
    movb_indRs_(rtmp, Rd);
  } else {
    if (src.offset_ == 0) {
      movb_indRs_(src.rn_, Rd);
    } else {
      add(rtmp, src.rn_, Operand(src.offset_));
      movb_indRs_(rtmp, Rd);
    }
  }
}


void Assembler::movb(Register Rd, const MemOperand& src, Register rtmp) {
  ASSERT(src.mode_ == Offset);
  ASSERT(!Rd.is(pr));
  movsb(Rd, src, rtmp);
  extub_(Rd, Rd);       // zero extension
}


void Assembler::movsw(Register Rd, const MemOperand& src, Register rtmp) {
  ASSERT(src.mode_ == Offset);
  ASSERT(!Rd.is(pr));
  if (src.roffset_.is_valid()) {
    add(rtmp, src.rn_, src.roffset_);
    movw_indRs_(rtmp, Rd);
  } else {
    if (src.offset_ == 0) {
      movw_indRs_(src.rn_, Rd);
    } else {
      add(rtmp, src.rn_, Operand(src.offset_));
      movw_indRs_(rtmp, Rd);
    }
  }
}

void Assembler::movw(Register Rd, const MemOperand& src, Register rtmp) {
  ASSERT(src.mode_ == Offset);
  ASSERT(!Rd.is(pr));
  movsw(Rd, src, rtmp);
  extuw_(Rd, Rd);       // zero extension
}


void Assembler::movd(DwVfpRegister Dd, Register Rs1, Register Rs2) {
  // SH4: use the stack, enforce a correct data layout.
  // Ref to: pop(double) implementation
  push(Rs2); // high bits
  push(Rs1); // low bits
  pop(Dd);
}


void Assembler::movd(Register Rd1, Register Rd2, DwVfpRegister Ds) {
  // SH4: use the stack, enforce a correct data layout.
  // Ref to: push(double) implementation
  push(Ds);
  pop(Rd1); // low bits
  pop(Rd2); // high bits
}


void Assembler::movf(SwVfpRegister Fd, Register Rs) {
  // SH4: use the stack
  push(Rs);
  fmov_incRs_(sp, Fd);
}


void Assembler::movf(Register Rd, SwVfpRegister Fs) {
  // SH4: use the stack
  fmov_decRd_(Fs, sp);
  pop(Rd);
}


void Assembler::fldr(SwVfpRegister dst, const MemOperand& src, Register rtmp) { 
  ASSERT(src.mode_ == Offset);
  if (src.roffset_.is_valid()) {
    add(rtmp, src.rn_, src.roffset_);
    fmov_indRs_(rtmp, dst);
  } else {
    if (src.offset_ == 0) {
      fmov_indRs_(src.rn_, dst);
    } else {
      ASSERT(src.roffset_.is(no_reg));
      add(rtmp, src.rn_, Operand(src.offset_));
      fmov_indRs_(rtmp, dst);
    }
  }
}


void Assembler::dldr(DwVfpRegister dst, const MemOperand& src, Register rtmp) {
  ASSERT(src.mode_ == Offset);
  if (src.roffset_.is_valid()) {
    UNIMPLEMENTED();
  } else {
    fldr(dst.low(), src, rtmp);
    fldr(dst.high(), MemOperand(src.rn_, src.offset_ + 4), rtmp);
  }
}


void Assembler::fstr(SwVfpRegister src, const MemOperand& dst, Register rtmp) {
  ASSERT(dst.mode_ == Offset);
  if (dst.roffset_.is_valid()) {
    add(rtmp, dst.rn_ ,dst.roffset_);
    fmov_indRd_(src, rtmp);
  } else {
    if (dst.offset_ == 0) {
      fmov_indRd_(src, dst.rn_);
    } else {
      ASSERT(dst.roffset_.is(no_reg));
      add(rtmp, dst.rn_, Operand(dst.offset_));
      fmov_indRd_(src, rtmp);
    }
  }
}


void Assembler::dstr(DwVfpRegister src, const MemOperand& dst, Register rtmp) {
  ASSERT(dst.mode_ == Offset);
  if (dst.roffset_.is_valid()) {
    UNIMPLEMENTED();
  } else {
    fstr(src.low(), dst, rtmp);
    fstr(src.high(), MemOperand(dst.rn_, dst.offset_ + 4), rtmp);
  }
}


void Assembler::dfloat(DwVfpRegister Dd, const Operand &src, Register rtmp)
{
  Register r_src = rtmp;
  if (src.is_reg()) {
    r_src = src.rm();
  } else {
    mov(r_src, src);
  }
  dfloat(Dd, r_src);
}


void Assembler::dfloat(DwVfpRegister Dd, Register Rs)
{
  lds_FPUL_(Rs);
  float_FPUL_double_(Dd);
}


void Assembler::dufloat(DwVfpRegister Dd, Register Rs, Register rtmp) {
  ASSERT(!Dd.is(kDoubleRegZero));

  Label too_large, end;

  // Performs a signed conversion first (no unsigned conversion SH4)
  dfloat(Dd, Rs);

  // Test if the unsigned integer is small enough for a signed conversion
  tst(Rs, Operand(0x80000000), rtmp);
  bt_near(&end);

  // Otherwise it was interpreted as signed, thus we need to add 2^32.
  push(kDoubleRegZero); // Use it as scratch
  DwVfpRegister dbl_scratch = kDoubleRegZero;
  vmov(dbl_scratch, 4294967296.0/*2^32*/, no_reg/*not used*/, rtmp);
  fadd(Dd, dbl_scratch);
  pop(kDoubleRegZero); // Restore double used as scratch
  bind(&end);
}


void Assembler::idouble(Register Rd, DwVfpRegister Ds, Register fpscr)
{
  ftrc_double_FPUL_(Ds);
  if(!fpscr.is(no_reg))
    sts_FPSCR_(fpscr);
  sts_FPUL_(Rd);
}


void Assembler::mov(const MemOperand& dst, Register Rd, Register rtmp) {
  ASSERT(dst.mode_ == Offset);
  ASSERT(!Rd.is(pr));
  if (dst.roffset_.is_valid()) {
    add(rtmp, dst.rn_, dst.roffset_);
    movl_indRd_(Rd, rtmp);
  } else {
    if (dst.offset_ == 0) {
      movl_indRd_(Rd, dst.rn_);
    } else {
      if (FITS_SH4_movl_dispRd(dst.offset_)) {
        movl_dispRd_(Rd, dst.offset_, dst.rn_);
      } else {
        ASSERT(!Rd.is(rtmp));
        add(rtmp, dst.rn_, Operand(dst.offset_));
        movl_indRd_(Rd, rtmp);
      }
    }
  }
}


void Assembler::movb(const MemOperand& dst, Register Rd, Register rtmp) {
  ASSERT(dst.mode_ == Offset);
  ASSERT(!Rd.is(pr));
  if (dst.roffset_.is_valid()) {
    add(rtmp, dst.rn_, dst.roffset_);
    movb_indRd_(Rd, rtmp);
  } else {
    if (dst.offset_ == 0) {
      movb_indRd_(Rd, dst.rn_);
    } else {
      ASSERT(!Rd.is(rtmp));
      add(rtmp, dst.rn_, Operand(dst.offset_));
      movb_indRd_(Rd, rtmp);
    }
  }
}


void Assembler::movw(const MemOperand& dst, Register Rd, Register rtmp) {
  ASSERT(dst.mode_ == Offset);
  ASSERT(!Rd.is(pr));
  if (dst.roffset_.is_valid()) {
    add(rtmp, dst.rn_, dst.roffset_);
    movw_indRd_(Rd, rtmp);
  } else {
    if (dst.offset_ == 0) {
      movw_indRd_(Rd, dst.rn_);
    } else {
      ASSERT(!Rd.is(rtmp));
      add(rtmp, dst.rn_, Operand(dst.offset_));
      movw_indRd_(Rd, rtmp);
    }
  }
}


void Assembler::mul(Register Rd, Register Rs, Register Rt) {
  mull_(Rs, Rt);
  sts_MACL_(Rd);
}


void Assembler::dmuls(Register dstL, Register dstH, Register src1,
                      Register src2) {
  dmulsl_(src1, src2);
  sts_MACL_(dstL);
  sts_MACH_(dstH);
}


// Pseudo instructions.
void Assembler::nop(int type) {
  // SH4: generate a SH4 nop for type 0
  // Otherwize a mov rx, rx
  ASSERT(0 <= type && type <= 14);  // Avoid writing sp even if harmless
  if (type == 0) {
    nop_();
  } else {
    Register reg = Register::from_code(type);
    mov_(reg, reg);
  }
}

void Assembler::pop(Register dst) {
  if (dst.is(pr))
    ldsl_incRd_PR_(sp);
  else
    movl_incRs_(sp, dst);
}


void Assembler::pop(DwVfpRegister dst) {
  // SH4: when poping a double from stack, assumes
  // a correct data layout:
  // old_sp -> [low bits of double]
  //           [high bits of double]
  // new_sp -> ...
  fmov_incRs_(sp, dst.low());
  fmov_incRs_(sp, dst.high());
}


void Assembler::push(Register src) {
  if (src.is(pr))
    stsl_PR_decRd_(sp);
  else
    movl_decRd_(src, sp);
}


void Assembler::push(DwVfpRegister src) {
  // SH4: when pushing on stack a double, enforce
  // a correct data layout:
  // new_sp -> [low bits of double]
  //           [high bits of double]
  // old_sp -> ...
  fmov_decRd_(src.high(), sp);
  fmov_decRd_(src.low(), sp);
}


void Assembler::push(const Operand& src, Register rtmp) {
  Register r_src = rtmp;
  if (src.is_reg()) {
    r_src = src.rm();
  } else {
    mov(r_src, src);
  }
  push(r_src);
}


void Assembler::pushm(RegList dst) {
  for (int i = Register::kNumRegisters - 1; i >= 0; i--) {
    if ((dst & (1 << i)) != 0) {
      push(Register::from_code(i));
    }
  }
}


void Assembler::vpushm(RegList dst) {
  for (int i = DwVfpRegister::kMaxNumRegisters - 1; i >= 0; i -= 2) {
    if ((dst & (1 << i)) != 0) {
      push(DwVfpRegister::from_code(i));
    }
  }
}


void Assembler::vpushm(DwVfpRegister first, DwVfpRegister last) {
  ASSERT_LE(first.code(), last.code());
  for (int i = last.code(); i >= first.code(); i -= 2) {
    push(DwVfpRegister::from_code(i));
  }
}


void Assembler::popm(RegList src) {
  for (int i = 0; i < Register::kNumRegisters; i++) {
    if ((src & (1 << i)) != 0) {
      pop(Register::from_code(i));
    }
  }
}


void Assembler::vpopm(RegList src) {
  for (int i = 0; i < DwVfpRegister::kMaxNumRegisters; i += 2) {
    if ((src & (1 << i)) != 0) {
      pop(DwVfpRegister::from_code(i));
    }
  }
}


void Assembler::vpopm(DwVfpRegister first, DwVfpRegister last) {
  ASSERT_LE(first.code(), last.code());
  for (int i = first.code(); i <= last.code(); i += 2) {
    pop(DwVfpRegister::from_code(i));
  }
}


// Exception-generating instructions and debugging support.
// Stops with a non-negative code less than kNumOfWatchedStops support
// enabling/disabling and a counter feature. See simulator-arm.h .
void Assembler::stop(const char* msg, Condition cond) {
  ASSERT(cond == ne || cond == eq || cond == al);
  Label skip;
  if (cond == eq)
    bf_near(&skip);   // Jump after sequence if T bit is false
  else
    bt_near(&skip);   // Jump after sequence if T bit is true
#ifndef __sh__
  {
    BlockConstPoolScope block_const_pool(this);
    emit(kStoppoint);
    CheckBuffer();
    *reinterpret_cast<uint32_t*>(pc_) = reinterpret_cast<uint32_t>(msg);
    pc_ += sizeof(uint32_t);
  }
#else
  // Generate a privileged instruction
  bkpt();
#endif // __sh__
  bind(&skip);
}


// Debugging.
void Assembler::RecordJSReturn() {
  positions_recorder()->WriteRecordedPositions();
  CheckBuffer();
  RecordRelocInfo(RelocInfo::JS_RETURN);
}


void Assembler::RecordDebugBreakSlot() {
  positions_recorder()->WriteRecordedPositions();
  CheckBuffer();
  RecordRelocInfo(RelocInfo::DEBUG_BREAK_SLOT);
}


void Assembler::RecordComment(const char* msg) {
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

  // Set up new buffer.
  desc.buffer = NewArray<byte>(desc.buffer_size);

  desc.instr_size = pc_offset();
  desc.reloc_size = (buffer_ + buffer_size_) - reloc_info_writer.pos();

  // Copy the data.
  int pc_delta = desc.buffer - buffer_;
  int rc_delta = (desc.buffer + desc.buffer_size) - (buffer_ + buffer_size_);
  OS::MemMove(desc.buffer, buffer_, desc.instr_size);
  OS::MemMove(reloc_info_writer.pos() + rc_delta,
              reloc_info_writer.pos(), desc.reloc_size);

  // Switch buffers.
  DeleteArray(buffer_);
  buffer_ = desc.buffer;
  buffer_size_ = desc.buffer_size;
  pc_ += pc_delta;
  reloc_info_writer.Reposition(reloc_info_writer.pos() + rc_delta,
                               reloc_info_writer.last_pc() + pc_delta);

  // None of our relocation types are pc relative pointing outside the code
  // buffer nor pc absolute pointing inside the code buffer, so there is no need
  // to relocate any emitted relocation entries.

  // Relocate pending relocation entries.
  for (int i = 0; i < num_pending_reloc_info_; i++) {
    RelocInfo& rinfo = pending_reloc_info_[i];
    ASSERT(rinfo.rmode() != RelocInfo::COMMENT &&
           rinfo.rmode() != RelocInfo::POSITION);
    if (rinfo.rmode() != RelocInfo::JS_RETURN) {
      rinfo.set_pc(rinfo.pc() + pc_delta);
    }
  }
}


void Assembler::db(uint8_t data) {
  // No relocation info should be pending while using db. db is used
  // to write pure data with no pointers and the constant pool should
  // be emitted before using db.
  ASSERT(num_pending_reloc_info_ == 0);
  CheckBuffer();
  *reinterpret_cast<uint8_t*>(pc_) = data;
  pc_ += sizeof(uint8_t);
}


void Assembler::dw(uint16_t data) {
  // No relocation info should be pending while using dd. dd is used
  // to write pure data with no pointers and the constant pool should
  // be emitted before using dd.
  ASSERT(num_pending_reloc_info_ == 0);
  CheckBuffer();
  *reinterpret_cast<uint16_t*>(pc_) = data;
  pc_ += sizeof(uint16_t);
}

void Assembler::dd(uint32_t data) {
  // No relocation info should be pending while using dd. dd is used
  // to write pure data with no pointers and the constant pool should
  // be emitted before using dd.
  ASSERT(num_pending_reloc_info_ == 0);
  CheckBuffer();
  *reinterpret_cast<uint32_t*>(pc_) = data;
  pc_ += sizeof(uint32_t);
}


void Assembler::RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data) {
  // Don't record external references unless the heap will be serialized.
  if (rmode == RelocInfo::EXTERNAL_REFERENCE) {
    if (!Serializer::enabled(isolate()) && !emit_debug_code()) {
      return;
    }
  }
  ASSERT(buffer_space() >= kMaxRelocSize);  // too late to grow buffer here
  if (rmode == RelocInfo::CODE_TARGET_WITH_ID) {
      RelocInfo reloc_info_with_ast_id(pc_,
                                       rmode,
                                       RecordedAstId().ToInt(),
                                       NULL);
    ClearRecordedAstId();
    reloc_info_writer.Write(&reloc_info_with_ast_id);
  } else {
    RelocInfo rinfo(pc_, rmode, data, NULL);  // we do not try to reuse pool constants
    reloc_info_writer.Write(&rinfo);
  }
}


void Assembler::BlockConstPoolFor(unsigned instructions) {
  // no-op without constant pool
}


void Assembler::CheckConstPool(bool force_emit, bool require_jump) {
  // no-op without constant pool
}


// FPU ARM interface emulation (ref to src/arm/assembler-arm.h)
void Assembler::vldr(DwVfpRegister dst, Register base, int offset,
                     Condition cond, Register rtmp)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  dldr(dst, MemOperand(base, offset), rtmp);
  if (cond != al)
    bind(&skip);
}

void Assembler::vldr(DwVfpRegister dst, const MemOperand& src,
                     Condition cond, Register rtmp)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  dldr(dst, src, rtmp);
  if (cond != al)
    bind(&skip);
}

void Assembler::vldr(SwVfpRegister dst, Register base, int offset,
                     Condition cond, Register rtmp)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  fldr(dst, MemOperand(base, offset), rtmp);
  if (cond != al)
    bind(&skip);
}

void Assembler::vldr(SwVfpRegister dst, const MemOperand& src,
                     Condition cond, Register rtmp)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  fldr(dst, src, rtmp);
  if (cond != al)
    bind(&skip);
}

void Assembler::vstr(DwVfpRegister src, Register base, int offset,
                     Condition cond, Register rtmp)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  dstr(src, MemOperand(base, offset), rtmp);
  if (cond != al)
    bind(&skip);
}

void Assembler::vstr(DwVfpRegister src, const MemOperand& dst,
            Condition cond, Register rtmp)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  dstr(src, dst, rtmp);
  if (cond != al)
    bind(&skip);
}

void Assembler::vstr(SwVfpRegister src, Register base, int offset,
                     Condition cond, Register rtmp)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  fstr(src, MemOperand(base, offset), rtmp);
  if (cond != al)
    bind(&skip);
}

void Assembler::vstr(SwVfpRegister src, const MemOperand& dst,
            Condition cond, Register rtmp)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  fstr(src, dst, rtmp);
  if (cond != al)
    bind(&skip);
}

void Assembler::vcvt_f64_s32(DwVfpRegister dst,
                             Register src,
                             VFPConversionMode mode,
                             Condition cond)
{
  // SH4: the conversion from int is always exact, ignore the conversion mode
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  dfloat(dst, src);
  if (cond != al)
    bind(&skip);
}

void Assembler::vcvt_f64_u32(DwVfpRegister dst,
                             Register src,
                             VFPConversionMode mode,
                             Condition cond,
                             Register rtmp)
{
  // SH4: the conversion from unsigned is always exact, ignore the conversion mode
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  dufloat(dst, src, rtmp);
  if (cond != al)
    bind(&skip);
}


void Assembler::vcvt_s32_f64(Register dst,
                             DwVfpRegister src,
                             VFPConversionMode mode,
                             Condition cond)
{
  // SH4: support truncation (hardware) or round to nearest (emulated)
  ASSERT(mode == kDefaultRoundToZero || mode == kFPSCRRounding);
  ASSERT(cond == al || cond == ne || cond ==  eq);
  ASSERT(!src.is(kDoubleRegZero));

  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  if (mode == kDefaultRoundToZero) {
    idouble(dst, src);
  } else if (mode == kFPSCRRounding) {
    /* Round to nearest integer not implemented in hardware,
       simulate it with trunc((x+2^52)-2^52) (for positives)
       or trunc((x-2^52)+2^52) (for negatives). */
    dcmpgt(kDoubleRegZero, src);
    DwVfpRegister dscratch1 = kDoubleRegZero;
    DwVfpRegister dscratch2 = src.is(sh4_dr0) ? sh4_dr2: sh4_dr0;
    push(dscratch1);
    push(dscratch2);
    vmov(dscratch1, 4.50359962737049600000e+15/*2^52*/);
    vneg(dscratch1, dscratch1, t); /* if 0.0 > src */
    vadd(dscratch2, dscratch1, src);
    vsub(dscratch2, dscratch2, dscratch1);
    idouble(dst, dscratch2);
    pop(dscratch2);
    pop(dscratch1);
  } else {
    UNREACHABLE();
  }
  if (cond != al)
    bind(&skip);
}


void Assembler::vcvt_f64_f32(DwVfpRegister dst,
                             SwVfpRegister src,
                             VFPConversionMode mode,
                             Condition cond)
{
  // SH4: the conversion from 32 bits float is always exact, ignore the conversion mode
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  fcnvsd(dst, src);
  if (cond != al)
    bind(&skip);
}


void Assembler::vcvt_f32_f64(SwVfpRegister dst,
                             DwVfpRegister src,
                             VFPConversionMode mode,
                             Condition cond)
{
  // SH4: TODO: sh4 default conversion is round to nearest even
  // though the default in this interface is round to zero.
  // May have to change the FPSCR flags setting.
  // Keep mode check as ARM code always use round to zero.
  ASSERT(mode == kDefaultRoundToZero);
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  fcnvds(dst, src);
  if (cond != al)
    bind(&skip);
}


static void DoubleAsTwoUInt32(double d, uint32_t* lo, uint32_t* hi) {
  uint64_t i;
  OS::MemCopy(&i, &d, 8);

  *lo = i & 0xffffffff;
  *hi = i >> 32;
}

void Assembler::vmov(DwVfpRegister dst,
                     double imm,
                     Register scratch,
                     Register rtmp)
{
  // SH4: scratch is not used, we use rtmp instead.
  uint32_t lo, hi;
  DoubleAsTwoUInt32(imm, &lo, &hi);
  // SH4: push high bits first, then low bits, such that data layout is correct
  // when pointed by new SP. Ref to pop(double) implementation.
  mov(rtmp, Operand((int32_t)hi));
  push(rtmp);
  mov(rtmp, Operand((int32_t)lo));
  push(rtmp);
  // Now pop double from stack
  pop(dst);
}


void Assembler::vmov(DwVfpRegister dst, DwVfpRegister src, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  fmov_(src.low(), dst.low());
  fmov_(src.high(), dst.high());
  if (cond != al)
    bind(&skip);
}


void Assembler::vmov(DwVfpRegister dst, Register src1, Register src2, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  movd(dst, src1, src2);
  if (cond != al)
    bind(&skip);
}

void Assembler::vmov(Register dst1, Register dst2, DwVfpRegister src, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  movd(dst1, dst2, src);
  if (cond != al)
    bind(&skip);
}


void Assembler::vmov(SwVfpRegister dst, Register src, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  movf(dst, src);
  if (cond != al)
    bind(&skip);
}


void Assembler::vmov(Register dst, SwVfpRegister src, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  movf(dst, src);
  if (cond != al)
    bind(&skip);
}


void Assembler::vneg(DwVfpRegister dst, DwVfpRegister src, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  if (src.code() != dst.code())
    vmov(dst, src);
  fneg(dst);
  if (cond != al)
    bind(&skip);
}

void Assembler::vabs(DwVfpRegister dst, DwVfpRegister src, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  if (src.code() != dst.code())
    vmov(dst, src);
  fabs(dst);
  if (cond != al)
    bind(&skip);
}

void Assembler::vadd(DwVfpRegister dst, DwVfpRegister src1, DwVfpRegister src2, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  if (src1.code() == dst.code())
    fadd(dst, src2);
  else if (src2.code() == dst.code())
    fadd(dst, src1);
  else {
    vmov(dst, src1);
    fadd(dst, src2);
  }
  if (cond != al)
    bind(&skip);
}

void Assembler::vsub(DwVfpRegister dst, DwVfpRegister src1, DwVfpRegister src2, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  if (src1.code() == dst.code())
    fsub(dst, src2);
  else if (src2.code() == dst.code()) {
    // Need to preserve src1 and use it as temporary
    push(src1);
    fsub(src1, src2);
    vmov(dst, src1);
    pop(src1);
  } else {
    vmov(dst, src1);
    fsub(dst, src2);
  }
  if (cond != al)
    bind(&skip);
}

void Assembler::vmul(DwVfpRegister dst, DwVfpRegister src1, DwVfpRegister src2, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  if (src1.code() == dst.code())
    fmul(dst, src2);
  else if (src2.code() == dst.code())
    fmul(dst, src1);
  else {
    vmov(dst, src1);
    fmul(dst, src2);
  }
  if (cond != al)
    bind(&skip);
}

void Assembler::vdiv(DwVfpRegister dst, DwVfpRegister src1, DwVfpRegister src2, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  if (src1.code() == dst.code())
    fdiv(dst, src2);
  else if (src2.code() == dst.code()) {
    // Need to preserve src1 and use it as temporary
    push(src1);
    fdiv(src1, src2);
    vmov(dst, src1);
    pop(src1);
  } else {
    vmov(dst, src1);
    fdiv(dst, src2);
  }
  if (cond != al)
    bind(&skip);
}

void Assembler::vsqrt(DwVfpRegister dst, DwVfpRegister src, Condition cond)
{
  Label skip;
  ASSERT(cond == al || cond == ne || cond ==  eq);
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  if (src.code() != dst.code())
    vmov(dst, src);
  fsqrt(dst);
  if (cond != al)
    bind(&skip);
}


#ifdef DEBUG
void Assembler::emit(Instr x) {
  CheckBuffer();
  // INSERT: instruction checks
  // if (x == 0xXXXX) ASSERT(0);
  *reinterpret_cast<Instr*>(pc_) = x;
  if (FLAG_print_emit) {
    disasm::Disassembler::Disassemble(stdout, pc_, pc_ + kInstrSize);
    fflush(stdout);
  }
  pc_ += kInstrSize;
}
#endif


Handle<ConstantPoolArray> Assembler::NewConstantPool(Isolate* isolate) {
  // No out-of-line constant pool support.
  ASSERT(!FLAG_enable_ool_constant_pool);
  return isolate->factory()->empty_constant_pool_array();
}


void Assembler::PopulateConstantPool(ConstantPoolArray* constant_pool) {
  // No out-of-line constant pool support.
  ASSERT(!FLAG_enable_ool_constant_pool);
  return;
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
