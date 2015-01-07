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

#ifndef V8_SH4_ASSEMBLER_SH4_INL_H_
#define V8_SH4_ASSEMBLER_SH4_INL_H_

#include "sh4/assembler-sh4.h"
#include "sh4/checks-sh4.h"
#include "sh4/constants-sh4.h"
#include "cpu.h"
#include "debug.h"
#include "disasm.h"

namespace v8 {
namespace internal {


int Register::NumAllocatableRegisters() { // REVIEWEDBY: CG
  return kMaxNumAllocatableRegisters;
}


int DwVfpRegister::NumRegisters() { // REVIEWEDBY: CG
  return kMaxNumRegisters;
}


int DwVfpRegister::NumReservedRegisters() { // REVIEWEDBY: CG
  return kNumReservedRegisters;
}


int DwVfpRegister::NumAllocatableRegisters() { // REVIEWEDBY: CG
  return NumRegisters() - kNumReservedRegisters;
}


int DwVfpRegister::ToAllocationIndex(DwVfpRegister reg) { // REVIEWEDBY: CG
  ASSERT(!reg.is(kDoubleRegZero));
  ASSERT(!reg.is(kScratchDoubleReg));
  ASSERT(reg.is_valid());
  return reg.code() / 2;
}


DwVfpRegister DwVfpRegister::FromAllocationIndex(int index) { // REVIEWEDBY: CG
  ASSERT(index >= 0 && index < NumAllocatableRegisters());
  return from_code(index * 2);
}


void RelocInfo::apply(intptr_t delta) { // REVIEWEDBY: CG
  if (RelocInfo::IsInternalReference(rmode_)) {
    // absolute code pointer inside code object moves with the code object.
    int32_t* p = reinterpret_cast<int32_t*>(pc_);
    *p += delta;  // relocate entry
  }
  // We do not use pc relative addressing on SH4, so there is
  // nothing else to do.
}


Address RelocInfo::target_address() { // REVIEWEDBY: CG
  ASSERT(IsCodeTarget(rmode_) || IsRuntimeEntry(rmode_));
  return Assembler::target_address_at(pc_, host_);
}


Address RelocInfo::target_address_address() { // REVIEWEDBY: CG
  ASSERT(IsCodeTarget(rmode_) || IsRuntimeEntry(rmode_)
                              || rmode_ == EMBEDDED_OBJECT
                              || rmode_ == EXTERNAL_REFERENCE);
  if (FLAG_enable_ool_constant_pool) {
    // We return the PC for ool constant pool since this function is used by the
    // serializerer and expects the address to reside within the code object.
    return reinterpret_cast<Address>(pc_);
  } else {
    return Assembler::target_pointer_address_at(pc_);
  }
}


Address RelocInfo::constant_pool_entry_address() { // REVIEWEDBY: CG
  UNREACHABLE();
  return NULL;
}


int RelocInfo::target_address_size() { // REVIEWEDBY: CG
  return kPointerSize;
}


void RelocInfo::set_target_address(Address target, WriteBarrierMode mode) { // REVIEWEDBY: CG
  ASSERT(IsCodeTarget(rmode_) || IsRuntimeEntry(rmode_));
  Assembler::set_target_address_at(pc_, host_, target);
  if (mode == UPDATE_WRITE_BARRIER && host() != NULL && IsCodeTarget(rmode_)) {
    Object* target_code = Code::GetCodeFromTargetAddress(target);
    host()->GetHeap()->incremental_marking()->RecordWriteIntoCode(
        host(), this, HeapObject::cast(target_code));
  }
}


Object* RelocInfo::target_object() { // REVIEWEDBY: CG
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  return reinterpret_cast<Object*>(Assembler::target_address_at(pc_, host_));
}


Handle<Object> RelocInfo::target_object_handle(Assembler* origin) { // REVIEWEDBY: CG
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  return Handle<Object>(reinterpret_cast<Object**>(
      Assembler::target_address_at(pc_, host_)));
}


void RelocInfo::set_target_object(Object* target, WriteBarrierMode mode) { // REVIEWEDBY: CG
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  ASSERT(!target->IsConsString());
  Assembler::set_target_address_at(pc_, host_,
                                   reinterpret_cast<Address>(target));
  if (mode == UPDATE_WRITE_BARRIER &&
      host() != NULL &&
      target->IsHeapObject()) {
    host()->GetHeap()->incremental_marking()->RecordWrite(
        host(), &Memory::Object_at(pc_), HeapObject::cast(target));
  }
}


Address RelocInfo::target_reference() { // REVIEWEDBY: CG
  ASSERT(rmode_ == EXTERNAL_REFERENCE);
  return Assembler::target_address_at(pc_, host_);
}


Address RelocInfo::target_runtime_entry(Assembler* origin) { // REVIEWEDBY: CG
  ASSERT(IsRuntimeEntry(rmode_));
  return target_address();
}


void RelocInfo::set_target_runtime_entry(Address target, // REVIEWEDBY: CG
                                         WriteBarrierMode mode) {
  ASSERT(IsRuntimeEntry(rmode_));
  if (target_address() != target) set_target_address(target, mode);
}


Handle<Cell> RelocInfo::target_cell_handle() { // REVIEWEDBY: CG
  ASSERT(rmode_ == RelocInfo::CELL);
  Address address = Memory::Address_at(pc_);
  return Handle<Cell>(reinterpret_cast<Cell**>(address));
}


Cell* RelocInfo::target_cell() { // REVIEWEDBY: CG
  ASSERT(rmode_ == RelocInfo::CELL);
  return Cell::FromValueAddress(Memory::Address_at(pc_));
}


void RelocInfo::set_target_cell(Cell* cell, WriteBarrierMode mode) { // REVIEWEDBY: CG
  ASSERT(rmode_ == RelocInfo::CELL);
  Address address = cell->address() + Cell::kValueOffset;
  Memory::Address_at(pc_) = address;
  if (mode == UPDATE_WRITE_BARRIER && host() != NULL) {
    // TODO(1550) We are passing NULL as a slot because cell can never be on
    // evacuation candidate.
    host()->GetHeap()->incremental_marking()->RecordWrite(
        host(), NULL, cell);
  }
}


// Ref to Code::PatchPlatformCodeAge(): 8 instructions + 1 padding nop for alignment * kInstrSize
static const int kNoCodeAgeSequenceLength = (8 + 1) * Assembler::kInstrSize;


Handle<Object> RelocInfo::code_age_stub_handle(Assembler* origin) { // REVIEWEDBY: CG
  // SH4: assume same as ARM
  UNREACHABLE(); // This should never be reached on Arm.
  return Handle<Object>();
}


Code* RelocInfo::code_age_stub() { // REVIEWEDBY: CG
  ASSERT(rmode_ == RelocInfo::CODE_AGE_SEQUENCE);
  // SH4: must be the same computation as in Code::GetCodeAgeAndParity
  // and RelocInfo::set_code_age_stub().
  byte *target_address_pointer =
    pc_ + kNoCodeAgeSequenceLength - 4;
  if ((uintptr_t)target_address_pointer % 4 == 2)
    target_address_pointer -= 2;
  ASSERT((uintptr_t)target_address_pointer % 4 == 0);
  return Code::GetCodeFromTargetAddress(Memory::Address_at(target_address_pointer));
}


void RelocInfo::set_code_age_stub(Code* stub) { // REVIEWEDBY: CG
  ASSERT(rmode_ == RelocInfo::CODE_AGE_SEQUENCE);
  // SH4: must be the same computation as in Code::GetCodeAgeAndParity
  // and RelocInfo::code_age_stub().
  byte *target_address_pointer =
    pc_ + kNoCodeAgeSequenceLength - 4;
  if ((uintptr_t)target_address_pointer % 4 == 2)
      target_address_pointer -= 2;
  ASSERT((uintptr_t)target_address_pointer % 4 == 0);
  Memory::Address_at(target_address_pointer) = stub->instruction_start();
}


Address RelocInfo::call_address() { // REVIEWEDBY: CG
  ASSERT((IsJSReturn(rmode()) && IsPatchedReturnSequence()) ||
         (IsDebugBreakSlot(rmode()) && IsPatchedDebugBreakSlotSequence()));
  // SH4: both sequences are identical, thus can be treated the same way
  ASSERT(Assembler::kJSReturnSequenceInstructions ==
         Assembler::kDebugBreakSlotInstructions);
  // SH4: Standard call sequence with inline address, use Assembler functions
  return Assembler::target_address_at(pc_, (ConstantPoolArray*)NULL);
}


void RelocInfo::set_call_address(Address target) { // REVIEWEDBY: CG
  ASSERT((IsJSReturn(rmode()) && IsPatchedReturnSequence()) ||
         (IsDebugBreakSlot(rmode()) && IsPatchedDebugBreakSlotSequence()));
  // SH4: both sequences are identical, thus can be treated the same way
  ASSERT(Assembler::kJSReturnSequenceInstructions ==
         Assembler::kDebugBreakSlotInstructions);
  // SH4: Standard call sequence with inline address, use Assembler functions
  Assembler::set_target_address_at(pc_, (ConstantPoolArray*)NULL, target);
  if (host() != NULL) {
    Object* target_code = Code::GetCodeFromTargetAddress(target);
    host()->GetHeap()->incremental_marking()->RecordWriteIntoCode(
        host(), this, HeapObject::cast(target_code));
  }
}


void RelocInfo::WipeOut() { // REVIEWEDBY: CG
  ASSERT(IsEmbeddedObject(rmode_) ||
         IsCodeTarget(rmode_) ||
         IsRuntimeEntry(rmode_) ||
         IsExternalReference(rmode_));
  Assembler::set_target_address_at(pc_, host_, NULL);
}


bool RelocInfo::IsPatchedReturnSequence() { // REVIEWEDBY: CG
  Instr current_instr = Assembler::instr_at(pc_);
  // A patched return sequence is a call to the debug stub (ref SetDebugBreakAtSlot()):
  //  mov.l ip, @(pc+...)
  //  ...
  //  address (4 bytes)
  //  ...
  //  jsr
  //  ...
  return Assembler::IsMovlPcRelative(current_instr);
}


bool RelocInfo::IsPatchedDebugBreakSlotSequence() { // REVIEWEDBY: CG
  Instr current_instr = Assembler::instr_at(pc_);
  return !Assembler::IsNop(current_instr, Assembler::DEBUG_BREAK_NOP);
}


void RelocInfo::Visit(Isolate* isolate, ObjectVisitor* visitor) { // REVIEWEDBY: CG
  RelocInfo::Mode mode = rmode();
  if (mode == RelocInfo::EMBEDDED_OBJECT) {
    visitor->VisitEmbeddedPointer(this);
  } else if (RelocInfo::IsCodeTarget(mode)) {
    visitor->VisitCodeTarget(this);
  } else if (mode == RelocInfo::CELL) {
    visitor->VisitCell(this);
  } else if (mode == RelocInfo::EXTERNAL_REFERENCE) {
    visitor->VisitExternalReference(this);
  } else if (RelocInfo::IsCodeAgeSequence(mode)) {
    visitor->VisitCodeAgeSequence(this);
  } else if (((RelocInfo::IsJSReturn(mode) &&
              IsPatchedReturnSequence()) ||
             (RelocInfo::IsDebugBreakSlot(mode) &&
              IsPatchedDebugBreakSlotSequence())) &&
             isolate->debug()->has_break_points()) {
    visitor->VisitDebugTarget(this);
  } else if (RelocInfo::IsRuntimeEntry(mode)) {
    visitor->VisitRuntimeEntry(this);
  }
}


template<typename StaticVisitor>
void RelocInfo::Visit(Heap* heap) { // REVIEWEDBY: CG
  RelocInfo::Mode mode = rmode();
  if (mode == RelocInfo::EMBEDDED_OBJECT) {
    StaticVisitor::VisitEmbeddedPointer(heap, this);
  } else if (RelocInfo::IsCodeTarget(mode)) {
    StaticVisitor::VisitCodeTarget(heap, this);
  } else if (mode == RelocInfo::CELL) {
    StaticVisitor::VisitCell(heap, this);
  } else if (mode == RelocInfo::EXTERNAL_REFERENCE) {
    StaticVisitor::VisitExternalReference(this);
  } else if (RelocInfo::IsCodeAgeSequence(mode)) {
    StaticVisitor::VisitCodeAgeSequence(heap, this);
  } else if (heap->isolate()->debug()->has_break_points() &&
             ((RelocInfo::IsJSReturn(mode) &&
              IsPatchedReturnSequence()) ||
             (RelocInfo::IsDebugBreakSlot(mode) &&
              IsPatchedDebugBreakSlotSequence()))) {
    StaticVisitor::VisitDebugTarget(heap, this);
  } else if (RelocInfo::IsRuntimeEntry(mode)) {
    StaticVisitor::VisitRuntimeEntry(this);
  }
}


Operand::Operand(int32_t immediate, RelocInfo::Mode rmode)  { // REVIEWEDBY: CG
  rm_ = no_reg;
  imm32_ = immediate;
  rmode_ = rmode;
}


Operand::Operand(const ExternalReference& f)  { // REVIEWEDBY: CG
  rm_ = no_reg;
  imm32_ = reinterpret_cast<int32_t>(f.address());
  rmode_ = RelocInfo::EXTERNAL_REFERENCE;
}


Operand::Operand(Smi* value) { // REVIEWEDBY: CG
  rm_ = no_reg;
  imm32_ =  reinterpret_cast<intptr_t>(value);
  rmode_ = RelocInfo::NONE32;
}


Operand::Operand(Register rm) { // REVIEWEDBY: CG
  rm_ = rm;
  imm32_ = 0;
  rmode_ = RelocInfo::NONE32;
}


bool Operand::is_reg() const { // REVIEWEDBY: CG
  return rm_.is_valid();
}


MemOperand::MemOperand(Register Rx, int32_t offset, AddrMode mode) { // REVIEWEDBY: CG
  rn_ = Rx;
  roffset_ = no_reg;
  offset_ = offset;
  mode_ = mode;
}


MemOperand::MemOperand(Register Rd, Register offset) { // REVIEWEDBY: CG
  rn_ = Rd;
  roffset_ = offset;
  offset_ = 0;
  mode_ = Offset;
}


void Assembler::CheckBuffer() { // REVIEWEDBY: CG
  if (buffer_space() <= kGap) {
    GrowBuffer();
  }
  if (pc_offset() >= next_buffer_check_) {
    CheckConstPool(false, true);
  }
}

#ifdef DEBUG
/* Defined in assembler-sh4.cc in DEBUG MODE. */
#else
void Assembler::emit(Instr x) { // REVIEWEDBY: CG
  CheckBuffer();
  *reinterpret_cast<Instr*>(pc_) = x;
  pc_ += kInstrSize;
}
#endif

Address Assembler::target_pointer_address_at(Address pc) { // REVIEWEDBY: CG
  // Compute the actual address in the code where the address of the
  // jump/call/mov instruction is stored given the instruction pc.
  // Ref to functions that call Assembler::RecordRelocInfo()
  // such as Assembler::mov(), Assembler::jmp(), Assembler::jsr().

  // With inline addresses, all sequences for jmp/jsr/mov uses the same
  // sequence as mov(), i.e.:
  // align 4;
  // @ pc argument is there
  // movl pc+x => R; [nop;] bra pc+4; nop; pool[0..32]
  //
  Instr instr = instr_at(pc);
  ASSERT(IsMovlPcRelative(instr)); // check if 'movl disp, pc'
  Address pool_address = Address(reinterpret_cast<uint32_t>(pc + 4) & ~0x3);
  pool_address += (instr & kOff8Mask) << 2;
  ASSERT(reinterpret_cast<uint32_t>(pool_address) % 4 == 0); // pool is aligned
  return pool_address;
}


Address Assembler::target_constant_pool_address_at(
    Address pc, ConstantPoolArray* constant_pool) {
  UNIMPLEMENTED();
  return NULL;
}


Address Assembler::target_address_at(Address pc, // REVIEWEDBY: CG
                                     ConstantPoolArray* constant_pool) {
  if (FLAG_enable_ool_constant_pool) {
    return Memory::Address_at(
        target_constant_pool_address_at(pc, constant_pool));
  } else {
   ASSERT(IsMovlPcRelative(instr_at(pc)));
   return Memory::Address_at(target_pointer_address_at(pc));
  }
}


Address Assembler::target_address_from_return_address(Address pc) { // REVIEWEDBY: CG
  // This is used only in the case of a call with immediate
  // target (ref ARM port).
  // No need to account for a direct register based call.
  Instr possible_mov;
  Address candidate;

  // As we go backward from the return address we do not need
  // to account for the alignment at the start of a sequence.
  // Hence use the kOldStyleCallTargetAddressOffsetWithoutAlignment
  // value.
  candidate = pc - kOldStyleCallTargetAddressOffsetWithoutAlignment;
  possible_mov = instr_at(candidate);
  USE(possible_mov);
  ASSERT(IsMovlPcRelative(possible_mov));
  return candidate;
}


Address Assembler::return_address_from_call_start(Address pc) { // REVIEWEDBY: CG
  // This is used only in the case of a call with immediate
  // target (ref ARM port).
  // No need to account for a direct register based call.
  // There are two cases:
  // - delayed constant pool: movl, jsr, nop
  // - immediate constant pool: movl, nop, bra, ....
  // SH4: actuallyremoved the case of delayed constant pool, unimplemented.
  // Ref to assembler-sh4.h
  // We assert that we are actually pointing to a PC relative
  // mov instruction which always starts an immediate call sequence.
  // NOTE: this function is not equivalent to
  // GetCallTargetAddressOffset() which is independent of the
  // call location but dependent on the call alignment.
  // Actually this must return the offset after the alignment
  // while GetCallTargetAddressOffset() must add an instruction
  // word in the misaligned case.
  ASSERT(IsMovlPcRelative(instr_at(pc)));

  // Do not need to account for the alignment at the start of a sequence.
  // Hence use the kOldStyleCallTargetAddressOffsetWithoutAlignment
  // value.
  return pc + kOldStyleCallTargetAddressOffsetWithoutAlignment;
}


void Assembler::deserialization_set_special_target_at( // REVIEWEDBY: CG
    Address constant_pool_entry, Code* code, Address target) {
  if (FLAG_enable_ool_constant_pool) {
    set_target_address_at(constant_pool_entry, code, target);
  } else {
    Memory::Address_at(constant_pool_entry) = target;
  }
}


void Assembler::set_target_address_at(Address pc, // REVIEWEDBY: CG
                                      ConstantPoolArray* constant_pool,
                                      Address target) {
  if (FLAG_enable_ool_constant_pool) {
    Memory::Address_at(
      target_constant_pool_address_at(pc, constant_pool)) = target;
  } else {
    ASSERT(IsMovlPcRelative(Assembler::instr_at(pc)));
    Memory::Address_at(target_pointer_address_at(pc)) = target;
    // Intuitively, we would think it is necessary to flush the instruction cache
    // after patching a target address in the code as follows:
    //   CPU::FlushICache(pc, sizeof(target));
    // However, on SH4, no instruction was actually patched by the assignment
    // above; the target address is not part of an instruction, it is patched in
    // the constant pool and is read via a data access; the instruction accessing
    // this address in the constant pool remains unchanged.
  }
}


int Assembler::align() { // REVIEWEDBY: CG
  int count = 0;
  while (((uintptr_t)pc_ & 0x3) != 0) {
    nop_();
    count++;
  }
  return count;
}


int Assembler::misalign() { // REVIEWEDBY: CG
  int count = 0;
  while (((uintptr_t)pc_ & 0x3) != 2) {
    nop_();
    count++;
  }
  return count;
}


void Assembler::cmp(Condition *cond, Register Rd, Register Rs) { // REVIEWEDBY: CG
  Condition cond_to_test = t;
  switch (*cond) {
  case ne:
    cond_to_test = f;
  case eq:
    cmpeq(Rd, Rs);
    break;
  case lt:
    cond_to_test = f;
  case ge:
    cmpge(Rd, Rs);
    break;
  case le:
    cond_to_test = f;
  case gt:
    cmpgt(Rd, Rs);
    break;
  case lo:
    cond_to_test = f;
  case hs:
    cmphs(Rd, Rs);
    break;
  case ls:
    cond_to_test = f;
  case hi:
    cmphi(Rd, Rs);
    break;
  default:
    UNREACHABLE();
  }
  *cond = cond_to_test;
}


void Assembler::cmpeq(Register Rd, const Operand& src, Register rtmp) { // REVIEWEDBY: CG
  if (src.is_reg()) {
    cmpeq(Rd, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  if (Rd.is(r0) && FITS_SH4_cmpeq_imm_R0(src.imm32_)) {
    cmpeq_imm_R0_(src.imm32_);
  } else {
    mov(rtmp, src);
    cmpeq_(rtmp, Rd);
  }
}


void Assembler::cmpgt(Register Rd, const Operand& src, Register rtmp) { // REVIEWEDBY: CG
  if (src.is_reg()) {
    cmpgt(Rd, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  mov(rtmp, src);
  cmpgt_(rtmp, Rd);
}


void Assembler::cmpge(Register Rd, const Operand& src, Register rtmp) { // REVIEWEDBY: CG
  if (src.is_reg()) {
    cmpge(Rd, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  mov(rtmp, src);
  cmpge_(rtmp, Rd);
}


void Assembler::cmphi(Register Rd, const Operand& src, Register rtmp) { // REVIEWEDBY: CG
  if (src.is_reg()) {
    cmphi(Rd, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  mov(rtmp, src);
  cmphi_(rtmp, Rd);
}


void Assembler::cmphs(Register Rd, const Operand& src, Register rtmp) { // REVIEWEDBY: CG
  if (src.is_reg()) {
    cmphs(Rd, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  mov(rtmp, src);
  cmphs_(rtmp, Rd);
}


void Assembler::rsb(Register Rd, Register Rs, const Operand& src, // REVIEWEDBY: CG
                    Register rtmp) {
  if (src.is_reg()) {
    rsb(Rd, Rs, src.rm());
    return;
  }
  ASSERT(!src.is_reg());
  if (src.imm32_ == 0 && src.rmode_ == RelocInfo::NONE32) {
    neg_(Rs, Rd);
  } else {
    mov(rtmp, src);
    sub(Rd, rtmp, Rs);
  }
}

void Assembler::rsb(Register Rd, Register Rs, Register Rt) { // REVIEWEDBY: CG
  sub(Rd, Rt, Rs);
}


void Assembler::rsb(Register Rd, Register Rs, const Operand& src, // REVIEWEDBY: CG
                    Condition cond, Register rtmp) {
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  ASSERT(!src.is_reg());
  rsb(Rd, Rs, src, rtmp);
  if (cond != al)
    bind(&skip);
}


void Assembler::rts() { // REVIEWEDBY: CG
  rts_();
  nop_();
}


void Assembler::ldr(Register Rd, const MemOperand& src, Condition cond, Register rtmp) { // REVIEWEDBY: CG
  ASSERT(src.mode_ == Offset || src.roffset().is(no_reg));
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  switch(src.mode_) {
  case PreIndex:
    add(src.rn_ , src.rn_, Operand(src.offset()), rtmp);
    mov(Rd, MemOperand(src.rn_, 0), rtmp);
    break;
  case PostIndex:
    mov(Rd, MemOperand(src.rn_, 0), rtmp);
    add(src.rn_, src.rn_, Operand(src.offset()), rtmp);
    break;
  case Offset:
    mov(Rd, src, rtmp);
    break;
  }
  if (cond != al)
    bind(&skip);
}


void Assembler::ldrh(Register Rd, const MemOperand& src, Condition cond, Register rtmp) { // REVIEWEDBY: CG
  ASSERT(src.mode_ == Offset || src.roffset().is(no_reg));
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  switch(src.mode_) {
  case PreIndex:
    add(src.rn_ , src.rn_, Operand(src.offset()), rtmp);
    movw(Rd, MemOperand(src.rn_, 0), rtmp);
    break;
  case PostIndex:
    movw(Rd, MemOperand(src.rn_, 0), rtmp);
    add(src.rn_, src.rn_, Operand(src.offset()), rtmp);
    break;
  case Offset:
    movw(Rd, src, rtmp);
    break;
  }
  if (cond != al)
    bind(&skip);
}

void Assembler::ldrsh(Register Rd, const MemOperand& src, Condition cond, Register rtmp) { // REVIEWEDBY: CG
  ASSERT(src.mode_ == Offset || src.roffset().is(no_reg));
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  switch(src.mode_) {
  case PreIndex:
    add(src.rn_ , src.rn_, Operand(src.offset()), rtmp);
    movsw(Rd, MemOperand(src.rn_, 0), rtmp);
    break;
  case PostIndex:
    movsw(Rd, MemOperand(src.rn_, 0), rtmp);
    add(src.rn_, src.rn_, Operand(src.offset()), rtmp);
    break;
  case Offset:
    movsw(Rd, src, rtmp);
    break;
  }
  if (cond != al)
    bind(&skip);
}


void Assembler::ldrb(Register Rd, const MemOperand& src, Condition cond, Register rtmp) { // REVIEWEDBY: CG
  ASSERT(src.mode_ == Offset || src.roffset().is(no_reg));
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  switch(src.mode_) {
  case PreIndex:
    add(src.rn_ , src.rn_, Operand(src.offset()), rtmp);
    movb(Rd, MemOperand(src.rn_, 0), rtmp);
    break;
  case PostIndex:
    movb(Rd, MemOperand(src.rn_, 0), rtmp);
    add(src.rn_, src.rn_, Operand(src.offset()), rtmp);
    break;
  case Offset:
    movb(Rd, src, rtmp);
    break;
  }
  if (cond != al)
    bind(&skip);
}

void Assembler::ldrsb(Register Rd, const MemOperand& src, Condition cond, Register rtmp) { // REVIEWEDBY: CG
  ASSERT(src.mode_ == Offset || src.roffset().is(no_reg));
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  switch(src.mode_) {
  case PreIndex:
    add(src.rn_ , src.rn_, Operand(src.offset()), rtmp);
    movsb(Rd, MemOperand(src.rn_, 0), rtmp);
    break;
  case PostIndex:
    movsb(Rd, MemOperand(src.rn_, 0), rtmp);
    add(src.rn_, src.rn_, Operand(src.offset()), rtmp);
    break;
  case Offset:
    movsb(Rd, src, rtmp);
    break;
  }
  if (cond != al)
    bind(&skip);
}


void Assembler::str(Register Rs, const MemOperand& dst, Condition cond, Register rtmp) { // REVIEWEDBY: CG
  ASSERT(dst.mode_ == Offset || dst.roffset().is(no_reg));
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  switch(dst.mode_) {
  case PreIndex:
    add(dst.rn_ , dst.rn_, Operand(dst.offset()), rtmp);
    mov(MemOperand(dst.rn_, 0), Rs, rtmp);
    break;
  case PostIndex:
    mov(MemOperand(dst.rn_, 0), Rs, rtmp);
    add(dst.rn_, dst.rn_, Operand(dst.offset()), rtmp);
    break;
  case Offset:
    mov(dst, Rs, rtmp);
    break;
  }
  if (cond != al)
    bind(&skip);
}


void Assembler::strh(Register Rs, const MemOperand& dst, Condition cond, Register rtmp) { // REVIEWEDBY: CG
  ASSERT(dst.mode_ == Offset || dst.roffset().is(no_reg));
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  switch(dst.mode_) {
  case PreIndex:
    add(dst.rn_ , dst.rn_, Operand(dst.offset()), rtmp);
    movw(MemOperand(dst.rn_, 0), Rs, rtmp);
    break;
  case PostIndex:
    movw(MemOperand(dst.rn_, 0), Rs, rtmp);
    add(dst.rn_, dst.rn_, Operand(dst.offset()), rtmp);
    break;
  case Offset:
    movw(dst, Rs, rtmp);
    break;
  }
  if (cond != al)
    bind(&skip);
}


void Assembler::strb(Register Rs, const MemOperand& dst, Condition cond, Register rtmp) { // REVIEWEDBY: CG
  ASSERT(dst.mode_ == Offset || dst.roffset().is(no_reg));
  ASSERT(cond == al || cond == ne || cond ==  eq);
  Label skip;
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  switch(dst.mode_) {
  case PreIndex:
    add(dst.rn_ , dst.rn_, Operand(dst.offset()), rtmp);
    movb(MemOperand(dst.rn_, 0), Rs, rtmp);
    break;
  case PostIndex:
    movb(MemOperand(dst.rn_, 0), Rs, rtmp);
    add(dst.rn_, dst.rn_, Operand(dst.offset()), rtmp);
    break;
  case Offset:
    movb(dst, Rs, rtmp);
    break;
  }
  if (cond != al)
    bind(&skip);
}


} }  // namespace v8::internal

#endif  // V8_SH4_ASSEMBLER_SH4_INL_H_
