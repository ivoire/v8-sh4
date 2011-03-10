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

#include "cpu.h"
#include "debug.h"

namespace v8 {
namespace internal {


// The modes possibly affected by apply must be in kApplyMask.
void RelocInfo::apply(intptr_t delta) {
  UNIMPLEMENTED();
}


Address RelocInfo::target_address() {
  UNIMPLEMENTED();
}


Address RelocInfo::target_address_address() {
  UNIMPLEMENTED();
}


int RelocInfo::target_address_size() {
  UNIMPLEMENTED();
}


void RelocInfo::set_target_address(Address target) {
  UNIMPLEMENTED();
}


Object* RelocInfo::target_object() {
  UNIMPLEMENTED();
}


Handle<Object> RelocInfo::target_object_handle(Assembler* origin) {
  UNIMPLEMENTED();
}


Object** RelocInfo::target_object_address() {
  UNIMPLEMENTED();
}


void RelocInfo::set_target_object(Object* target) {
  UNIMPLEMENTED();
}


Address* RelocInfo::target_reference_address() {
  UNIMPLEMENTED();
}


Handle<JSGlobalPropertyCell> RelocInfo::target_cell_handle() {
  UNIMPLEMENTED();
}


JSGlobalPropertyCell* RelocInfo::target_cell() {
  UNIMPLEMENTED();
}


void RelocInfo::set_target_cell(JSGlobalPropertyCell* cell) {
  UNIMPLEMENTED();
}


Address RelocInfo::call_address() {
  UNIMPLEMENTED();
}


void RelocInfo::set_call_address(Address target) {
  UNIMPLEMENTED();
}


Object* RelocInfo::call_object() {
  UNIMPLEMENTED();
}


void RelocInfo::set_call_object(Object* target) {
  UNIMPLEMENTED();
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
  UNIMPLEMENTED();
}


template<typename StaticVisitor>
void RelocInfo::Visit(Heap* heap) {
  UNIMPLEMENTED();
}


Immediate::Immediate(int x)  {
  UNIMPLEMENTED();
}


Immediate::Immediate(const ExternalReference& ext) {
  UNIMPLEMENTED();
}


Immediate::Immediate(Label* internal_offset) {
  UNIMPLEMENTED();
}


Immediate::Immediate(Handle<Object> handle) {
  UNIMPLEMENTED();
}


Immediate::Immediate(Smi* value) {
  UNIMPLEMENTED();
}


Immediate::Immediate(Address addr) {
  UNIMPLEMENTED();
}


Address Assembler::target_address_at(Address pc) {
  UNIMPLEMENTED();
}


void Assembler::set_target_address_at(Address pc, Address target) {
  UNIMPLEMENTED();
}


void Operand::set_modrm(int mod, Register rm) {
  UNIMPLEMENTED();
}


void Operand::set_sib(ScaleFactor scale, Register index, Register base) {
  UNIMPLEMENTED();
}


void Operand::set_disp8(int8_t disp) {
  UNIMPLEMENTED();
}


void Operand::set_dispr(int32_t disp, RelocInfo::Mode rmode) {
  UNIMPLEMENTED();
}


Operand::Operand(Register reg) {
  UNIMPLEMENTED();
}


Operand::Operand(XMMRegister xmm_reg) {
  UNIMPLEMENTED();
}


Operand::Operand(int32_t disp, RelocInfo::Mode rmode) {
  UNIMPLEMENTED();
}

} }  // namespace v8::internal

#endif  // V8_SH4_ASSEMBLER_SH4_INL_H_
