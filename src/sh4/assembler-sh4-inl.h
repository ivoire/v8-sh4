// Copyright (c) 1994-2006 Sun Microsystems Inc.
// All Rights Reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// - Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// - Redistribution in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// - Neither the name of Sun Microsystems or the names of contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The original source code covered by the above license above has been
// modified significantly by Google Inc.
// Copyright 2006-2008 the V8 project authors. All rights reserved.

// A light-weight SH4 Assembler.

#ifndef V8_SH4_ASSEMBLER_SH4_INL_H_
#define V8_SH4_ASSEMBLER_SH4_INL_H_

#include "sh4/assembler-sh4.h"
#include "cpu.h"
#include "debug.h"

namespace v8 {
namespace internal {

void Assembler::CheckBuffer() {
  if (buffer_space() <= kGap) {
    GrowBuffer();
  }
  // FIXME(STM): check if we must emit the constant pool
}


// The modes possibly affected by apply must be in kApplyMask.
void RelocInfo::apply(intptr_t delta) {
  if (RelocInfo::IsInternalReference(rmode_)) {
    // absolute code pointer inside code object moves with the code object.
    int32_t* p = reinterpret_cast<int32_t*>(pc_);
    *p += delta;  // relocate entry
  }
  // We do not use pc relative addressing on ARM, so there is
  // nothing else to do.
}


Address RelocInfo::target_address() {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == RUNTIME_ENTRY);
  return Assembler::target_address_at(pc_);
}


Address RelocInfo::target_address_address() {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == RUNTIME_ENTRY);
  return reinterpret_cast<Address>(Assembler::target_address_address_at(pc_));
}


int RelocInfo::target_address_size() {
  return Assembler::kExternalTargetSize;
}


void RelocInfo::set_target_address(Address target) {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == RUNTIME_ENTRY);
  Assembler::set_target_address_at(pc_, target);
}


Object* RelocInfo::target_object() {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  return Memory::Object_at(Assembler::target_address_address_at(pc_));
}


Handle<Object> RelocInfo::target_object_handle(Assembler* origin) {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  return Memory::Object_Handle_at(Assembler::target_address_address_at(pc_));
}


Object** RelocInfo::target_object_address() {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  return reinterpret_cast<Object**>(Assembler::target_address_address_at(pc_));
}


void RelocInfo::set_target_object(Object* target) {
  ASSERT(IsCodeTarget(rmode_) || rmode_ == EMBEDDED_OBJECT);
  Assembler::set_target_address_at(pc_, reinterpret_cast<Address>(target));
}


Address* RelocInfo::target_reference_address() {
  ASSERT(rmode_ == EXTERNAL_REFERENCE);
  return reinterpret_cast<Address*>(Assembler::target_address_address_at(pc_));
}


Handle<JSGlobalPropertyCell> RelocInfo::target_cell_handle() {
  ASSERT(rmode_ == RelocInfo::GLOBAL_PROPERTY_CELL);
  Address address = Memory::Address_at(pc_);
  return Handle<JSGlobalPropertyCell>(
      reinterpret_cast<JSGlobalPropertyCell**>(address));
}


JSGlobalPropertyCell* RelocInfo::target_cell() {
  ASSERT(rmode_ == RelocInfo::GLOBAL_PROPERTY_CELL);
  Address address = Memory::Address_at(pc_);
  Object* object = HeapObject::FromAddress(
      address - JSGlobalPropertyCell::kValueOffset);
  return reinterpret_cast<JSGlobalPropertyCell*>(object);
}


void RelocInfo::set_target_cell(JSGlobalPropertyCell* cell) {
  ASSERT(rmode_ == RelocInfo::GLOBAL_PROPERTY_CELL);
  Address address = cell->address() + JSGlobalPropertyCell::kValueOffset;
  Memory::Address_at(pc_) = address;
  CPU::FlushICache(pc_, sizeof(Address));
}


Address RelocInfo::call_address() {
  UNIMPLEMENTED();
}


void RelocInfo::set_call_address(Address target) {
  UNIMPLEMENTED();
}


Object* RelocInfo::call_object() {
  return *call_object_address();
}


void RelocInfo::set_call_object(Object* target) {
  *call_object_address() = target;
}


Object** RelocInfo::call_object_address() {
  UNIMPLEMENTED();
}


bool RelocInfo::IsPatchedReturnSequence() {
  UNIMPLEMENTED();
}


bool RelocInfo::IsPatchedDebugBreakSlotSequence() {
  UNIMPLEMENTED();
}


void RelocInfo::Visit(ObjectVisitor* visitor) {
  RelocInfo::Mode mode = rmode();
  if (mode == RelocInfo::EMBEDDED_OBJECT) {
    visitor->VisitPointer(target_object_address());
    CPU::FlushICache(pc_, sizeof(Address));
  } else if (RelocInfo::IsCodeTarget(mode)) {
    visitor->VisitCodeTarget(this);
  } else if (mode == RelocInfo::GLOBAL_PROPERTY_CELL) {
    visitor->VisitGlobalPropertyCell(this);
  } else if (mode == RelocInfo::EXTERNAL_REFERENCE) {
    visitor->VisitExternalReference(target_reference_address());
    CPU::FlushICache(pc_, sizeof(Address));
#ifdef ENABLE_DEBUGGER_SUPPORT
  // TODO(isolates): Get a cached isolate below.
  } else if (((RelocInfo::IsJSReturn(mode) &&
              IsPatchedReturnSequence()) ||
             (RelocInfo::IsDebugBreakSlot(mode) &&
              IsPatchedDebugBreakSlotSequence())) &&
             Isolate::Current()->debug()->has_break_points()) {
    visitor->VisitDebugTarget(this);
#endif
  } else if (mode == RelocInfo::RUNTIME_ENTRY) {
    visitor->VisitRuntimeEntry(this);
  }
}


template<typename StaticVisitor>
void RelocInfo::Visit(Heap* heap) {
  RelocInfo::Mode mode = rmode();
  if (mode == RelocInfo::EMBEDDED_OBJECT) {
    StaticVisitor::VisitPointer(heap, target_object_address());
    CPU::FlushICache(pc_, sizeof(Address));
  } else if (RelocInfo::IsCodeTarget(mode)) {
    StaticVisitor::VisitCodeTarget(heap, this);
  } else if (mode == RelocInfo::GLOBAL_PROPERTY_CELL) {
    StaticVisitor::VisitGlobalPropertyCell(heap, this);
  } else if (mode == RelocInfo::EXTERNAL_REFERENCE) {
    StaticVisitor::VisitExternalReference(target_reference_address());
    CPU::FlushICache(pc_, sizeof(Address));
#ifdef ENABLE_DEBUGGER_SUPPORT
  } else if (heap->isolate()->debug()->has_break_points() &&
             ((RelocInfo::IsJSReturn(mode) &&
              IsPatchedReturnSequence()) ||
             (RelocInfo::IsDebugBreakSlot(mode) &&
              IsPatchedDebugBreakSlotSequence()))) {
    StaticVisitor::VisitDebugTarget(heap, this);
#endif
  } else if (mode == RelocInfo::RUNTIME_ENTRY) {
    StaticVisitor::VisitRuntimeEntry(this);
  }
}


Immediate::Immediate(int x, RelocInfo::Mode rmode)  {
  x_ = x;
  rmode_ = rmode;
}


Immediate::Immediate(const ExternalReference& ext) {
  x_ = reinterpret_cast<int32_t>(ext.address());
  rmode_ = RelocInfo::EXTERNAL_REFERENCE;
}


Immediate::Immediate(Label* internal_offset) {
  x_ = reinterpret_cast<int32_t>(internal_offset);
  rmode_ = RelocInfo::INTERNAL_REFERENCE;
}


Immediate::Immediate(Handle<Object> handle) {
  // Verify all Objects referred by code are NOT in new space.
  Object* obj = *handle;
  ASSERT(!HEAP->InNewSpace(obj));
  if (obj->IsHeapObject()) {
    x_ = reinterpret_cast<intptr_t>(handle.location());
    rmode_ = RelocInfo::EMBEDDED_OBJECT;
  } else {
    // no relocation needed
    x_ =  reinterpret_cast<intptr_t>(obj);
    rmode_ = RelocInfo::NONE;
  }
}


Immediate::Immediate(Smi* value) {
  x_ = reinterpret_cast<intptr_t>(value);
  rmode_ = RelocInfo::NONE;
}


Immediate::Immediate(Address addr) {
  x_ = reinterpret_cast<int32_t>(addr);
  rmode_ = RelocInfo::NONE;
}


Address Assembler::target_address_address_at(Address pc) {
  // Compute the actual address in the code where the address of the
  // jump/call/mov instruction is stored given the instruction pc.
  // Ref to functions that call Assembler::RecordRelocInfo() 
  // such as Assembler::mov(), Assembler::jmp(), such as Assembler::jsr().

  // All sequences for jmp/jsr/mov uses the same sequence as mov(), i.e.:
  // align 4; 
  // movl pc+4 => R; nop; bra pc+4; nop; pool[0..32]
  // We compute the address of pool[0] given the pc address after the align
  Address pool_address = pc;
  ASSERT(reinterpret_cast<uint32_t>(pc) % 4 == 0); // check after align 
  pool_address += 4 * kInstrSize;
  return pool_address;
}


Address Assembler::target_address_at(Address pc) {
  return Memory::Address_at(target_address_address_at(pc));
}


void Assembler::set_target_address_at(Address pc, Address target) {
  Memory::Address_at(target_address_address_at(pc)) = target;
  // Intuitively, we would think it is necessary to flush the instruction cache
  // after patching a target address in the code as follows:
  //   CPU::FlushICache(pc, sizeof(target));
  // However, on ARM, no instruction was actually patched by the assignment
  // above; the target address is not part of an instruction, it is patched in
  // the constant pool and is read via a data access; the instruction accessing
  // this address in the constant pool remains unchanged.
}


void Assembler::cmp(Condition &cond, Register Rd, Register Rs) {
  Condition cond_to_test = eq;
  switch (cond) {
  case ne:
    cond_to_test = ne;
  case eq:
    cmpeq(Rd, Rs);
    break;
  case lt:
    cond_to_test = ne;
  case ge:
    cmpge(Rd, Rs);
    break;
  default:
    UNIMPLEMENTED();
  }
  cond = cond_to_test;
}


Operand::Operand(int32_t immediate, RelocInfo::Mode rmode) {
  rx_ = no_reg;
  imm32_ = immediate;
  rmode_ = rmode;
}


Operand::Operand(const ExternalReference& f) {
  rx_ = no_reg;
  imm32_ = reinterpret_cast<int32_t>(f.address());
  rmode_ = RelocInfo::EXTERNAL_REFERENCE;
}


MemOperand::MemOperand(Register Rx, int32_t offset) {
  rm_ = Rx;
  rn_ = no_reg;
  offset_ = offset;
}

MemOperand::MemOperand(Register Rd, Register offset) {
  rm_ = Rd;
  rn_ = offset;
  offset_ = 0;
}

} }  // namespace v8::internal

#endif  // V8_SH4_ASSEMBLER_SH4_INL_H_
