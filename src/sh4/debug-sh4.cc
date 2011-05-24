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

#include "codegen.h"
#include "debug.h"

namespace v8 {
namespace internal {

#ifdef ENABLE_DEBUGGER_SUPPORT

#ifdef DEBUG
#define RECORD_LINE() ACCESS_MASM(masm) RecordFunctionLine(__FUNCTION__, __LINE__)
#define __ RECORD_LINE(); ACCESS_MASM(masm)
#else
#define RECORD_LINE() ((void)0)
#define __ ACCESS_MASM(masm)
#endif


void BreakLocationIterator::ClearDebugBreakAtReturn() {
  UNIMPLEMENTED();
}


void BreakLocationIterator::ClearDebugBreakAtSlot() {
  UNIMPLEMENTED();
}


// A debug break in the frame exit code is identified by the JS frame exit code
// having been patched with a call instruction.
bool Debug::IsDebugBreakAtReturn(RelocInfo* rinfo) {
  UNIMPLEMENTED();
  return false;
}


bool BreakLocationIterator::IsDebugBreakAtReturn() {
  return Debug::IsDebugBreakAtReturn(rinfo());
}


bool BreakLocationIterator::IsDebugBreakAtSlot() {
  UNIMPLEMENTED();
  return false;
}


void BreakLocationIterator::SetDebugBreakAtReturn() {
  UNIMPLEMENTED();
}


void BreakLocationIterator::SetDebugBreakAtSlot() {
  UNIMPLEMENTED();
}


void Debug::GenerateCallICDebugBreak(MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateConstructCallDebugBreak(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateFrameDropperLiveEdit(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateKeyedLoadICDebugBreak(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateKeyedStoreICDebugBreak(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateLoadICDebugBreak(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GeneratePlainReturnLiveEdit(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateReturnDebugBreak(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateSlot(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateSlotDebugBreak(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateStoreICDebugBreak(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Debug::GenerateStubNoRegistersDebugBreak(v8::internal::MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}

const bool Debug::kFrameDropperSupported = true;

#undef __

#endif  // ENABLE_DEBUGGER_SUPPORT

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
