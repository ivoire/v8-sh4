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


void Assembler::nop() {
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


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
