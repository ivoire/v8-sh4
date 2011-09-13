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

#include "code-stubs.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm)


void ArgumentsAccessStub::GenerateNewObject(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void ArgumentsAccessStub::GenerateReadElement(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


Handle<Code> GetTypeRecordingBinaryOpStub(int key,
                        TRBinaryOpIC::TypeInfo type_info,
                        TRBinaryOpIC::TypeInfo result_type_info) {
  UNIMPLEMENTED();
}


Register InstanceofStub::left() {
  UNIMPLEMENTED();
}


Register InstanceofStub::right() {
  UNIMPLEMENTED();
}


void CEntryStub::Generate(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


bool CEntryStub::NeedsImmovableCode() {
  return true;
}


void JSEntryStub::GenerateBody(MacroAssembler* masm, bool is_construct) {
  //FIXME(STM): really generate this body
  __ rts();
}


void StackCheckStub::Generate(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
