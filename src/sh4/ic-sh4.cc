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

#include "ic-inl.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm)

void LoadIC::GenerateArrayLength(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void LoadIC::GenerateFunctionPrototype(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void LoadIC::GenerateMegamorphic(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void LoadIC::GenerateMiss(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r4    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  //  -- sp[0] : receiver
  // -----------------------------------
  Isolate* isolate = masm->isolate();

  __ IncrementCounter(isolate->counters()->load_miss(), 1, r1, r2);

  __ mov(r7, r4);
  __ Push(r7, r4);

  // Perform tail call to the entry.
  ExternalReference ref =
      ExternalReference(IC_Utility(kLoadIC_Miss), isolate);
  __ TailCallExternalReference(ref, 2, 1);
}


void LoadIC::GenerateNormal(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void LoadIC::GenerateStringLength(MacroAssembler* masm, bool support_wrappers) {
  UNIMPLEMENTED();
}


void StoreIC::GenerateArrayLength(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void StoreIC::GenerateGlobalProxy(MacroAssembler* masm,
                                  StrictModeFlag strict_mode) {
  UNIMPLEMENTED();
}


void StoreIC::GenerateMegamorphic(MacroAssembler* masm,
                                  StrictModeFlag strict_mode) {
  UNIMPLEMENTED();
}


void StoreIC::GenerateMiss(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- r4     : value
  //  -- r5     : key
  //  -- r6     : receiver
  //  -- pr     : return address
  // -----------------------------------

  // Push receiver, key and value for runtime call.
  __ Push(r6, r5, r4);

  ExternalReference ref =
      ExternalReference(IC_Utility(kKeyedStoreIC_Miss), masm->isolate());
  __ TailCallExternalReference(ref, 3, 1);
}


void StoreIC::GenerateNormal(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void CallIC::GenerateMegamorphic(MacroAssembler* masm, int argc) {
  UNIMPLEMENTED();
}


void CallIC::GenerateMiss(MacroAssembler* masm, int argc) {
  UNIMPLEMENTED();
}


void CallIC::GenerateNormal(MacroAssembler* masm, int argc) {
  UNIMPLEMENTED();
}


void CompareIC::UpdateCaches(Handle<Object> x, Handle<Object> y) {
  UNIMPLEMENTED();
}


void KeyedCallIC::GenerateMegamorphic(MacroAssembler* masm, int argc) {
  UNIMPLEMENTED();
}


void KeyedCallIC::GenerateMiss(MacroAssembler* masm, int argc) {
  UNIMPLEMENTED();
}


void KeyedCallIC::GenerateNormal(v8::internal::MacroAssembler*, int) {
  UNIMPLEMENTED();
}


void KeyedLoadIC::GenerateGeneric(v8::internal::MacroAssembler*) {
  UNIMPLEMENTED();
}


void KeyedLoadIC::GenerateIndexedInterceptor(v8::internal::MacroAssembler*) {
  UNIMPLEMENTED();
}


void KeyedLoadIC::GenerateMiss(v8::internal::MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- r4     : key
  //  -- r5     : receiver
  //  -- pr     : return address
  // -----------------------------------
  Isolate* isolate = masm->isolate();

  __ IncrementCounter(isolate->counters()->keyed_load_miss(), 1, r1, r2);

  __ Push(r5, r4);

  ExternalReference ref =
      ExternalReference(IC_Utility(kKeyedLoadIC_Miss), isolate);
  __ TailCallExternalReference(ref, 2, 1);
}


void KeyedLoadIC::GenerateString(v8::internal::MacroAssembler*) {
  UNIMPLEMENTED();
}


void KeyedStoreIC::GenerateGeneric(MacroAssembler* masm,
                                   StrictModeFlag strict_mode) {
  UNIMPLEMENTED();
}


void KeyedStoreIC::GenerateMiss(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- r4     : value
  //  -- r5     : key
  //  -- r6     : receiver
  //  -- pr     : return address
  // -----------------------------------

  // Push receiver, key and value for runtime call.
  __ Push(r6, r5, r4);

  ExternalReference ref =
      ExternalReference(IC_Utility(kKeyedStoreIC_Miss), masm->isolate());
  __ TailCallExternalReference(ref, 3, 1);
}


void PatchInlinedSmiCode(Address address) {
  UNIMPLEMENTED();
}

#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_ARM
