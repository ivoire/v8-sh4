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
#include "ic-inl.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm)

void LoadIC::GenerateArrayLength(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r4    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  //  -- sp[0] : receiver
  // -----------------------------------
  Label miss;

  StubCompiler::GenerateLoadArrayLength(masm, r4, r7, &miss);
  __ bind(&miss);
  StubCompiler::GenerateLoadMiss(masm, Code::LOAD_IC);
}


void LoadIC::GenerateFunctionPrototype(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r4    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  //  -- sp[0] : receiver
  // -----------------------------------
  Label miss;

  StubCompiler::GenerateLoadFunctionPrototype(masm, r4, r5, r7, &miss);
  __ bind(&miss);
  StubCompiler::GenerateLoadMiss(masm, Code::LOAD_IC);
}


void LoadIC::GenerateMegamorphic(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r4    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  //  -- sp[0] : receiver
  // -----------------------------------

  // Probe the stub cache.
  Code::Flags flags = Code::ComputeFlags(Code::LOAD_IC,
                                         NOT_IN_LOOP,
                                         MONOMORPHIC);
  Isolate::Current()->stub_cache()->GenerateProbe(
      masm, flags, r4, r6, r7, r1, r2);

  // Cache miss: Jump to runtime.
  GenerateMiss(masm);
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


static void GenerateGlobalInstanceTypeCheck(MacroAssembler* masm,
                                            Register type,
                                            Label* global_object) {
  // Register usage:
  //   type: holds the receiver instance type on entry.
  ASSERT(!type.is(r3));
  __ mov(r3, Immediate(JS_GLOBAL_OBJECT_TYPE));
  __ cmpeq(type, r3);
  __ bt(global_object);
  __ mov(r3, Immediate(JS_BUILTINS_OBJECT_TYPE));
  __ cmpeq(type, r3);
  __ bt(global_object);
  __ mov(r3, Immediate(JS_GLOBAL_PROXY_TYPE));
  __ cmpeq(type, r3);
  __ bt(global_object);
}


// Generated code falls through if the receiver is a regular non-global
// JS object with slow properties and no interceptors.
static void GenerateStringDictionaryReceiverCheck(MacroAssembler* masm,
                                                  Register receiver,
                                                  Register elements,
                                                  Register t0,
                                                  Register t1,
                                                  Label* miss) {
  // Register usage:
  //   receiver: holds the receiver on entry and is unchanged.
  //   elements: holds the property dictionary on fall through.
  // Scratch registers:
  //   t0: used to holds the receiver map.
  //   t1: used to holds the receiver instance type, receiver bit mask and
  //       elements map.

  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ bt(miss);

  // Check that the receiver is a valid JS object.
  __ CompareObjectType(receiver, t0, t1, FIRST_JS_OBJECT_TYPE, ge);
  __ bf(miss);

  // If this assert fails, we have to check upper bound too.
  ASSERT(LAST_TYPE == JS_FUNCTION_TYPE);

  GenerateGlobalInstanceTypeCheck(masm, t1, miss);

  // Check that the global object does not require access checks.
  __ mov(t1, FieldMemOperand(t0, Map::kBitFieldOffset));        // FIXME mov.b ??
  __ tst(t1, Immediate((1 << Map::kIsAccessCheckNeeded) |
                     (1 << Map::kHasNamedInterceptor)));
  __ bf(miss);

  __ mov(elements, FieldMemOperand(receiver, JSObject::kPropertiesOffset));
  __ mov(t1, FieldMemOperand(elements, HeapObject::kMapOffset));
  __ LoadRoot(r3, Heap::kHashTableMapRootIndex);
  __ cmpeq(t1, r3);
  __ bf(miss);
}

// Probe the string dictionary in the |elements| register. Jump to the
// |done| label if a property with the given name is found. Jump to
// the |miss| label otherwise.
static void GenerateStringDictionaryProbes(MacroAssembler* masm,
                                           Label* miss,
                                           Label* done,
                                           Register elements,
                                           Register name,
                                           Register scratch1,
                                           Register scratch2) {
  ASSERT(!elements.is(r3) && !name.is(r3) && !scratch1.is(r3) && !scratch2.is(r3));
  // Assert that name contains a string.
  if (FLAG_debug_code) __ AbortIfNotString(name);

  // Compute the capacity mask.
  const int kCapacityOffset = StringDictionary::kHeaderSize +
      StringDictionary::kCapacityIndex * kPointerSize;
  __ mov(scratch1, FieldMemOperand(elements, kCapacityOffset));
  __ asr(scratch1, scratch1, Immediate(kSmiTagSize));  // convert smi to int
  __ sub(scratch1, scratch1, Immediate(1));

  const int kElementsStartOffset = StringDictionary::kHeaderSize +
      StringDictionary::kElementsStartIndex * kPointerSize;

  // Generate an unrolled loop that performs a few probes before
  // giving up. Measurements done on Gmail indicate that 2 probes
  // cover ~93% of loads from dictionaries.
  static const int kProbes = 4;
  for (int i = 0; i < kProbes; i++) {
    // Compute the masked index: (hash + i + i * i) & mask.
    __ mov(scratch2, FieldMemOperand(name, String::kHashFieldOffset));
    if (i > 0) {
      // Add the probe offset (i + i * i) left shifted to avoid right shifting
      // the hash in a separate instruction. The value hash + i + i * i is right
      // shifted in the following and instruction.
      ASSERT(StringDictionary::GetProbeOffset(i) <
             1 << (32 - String::kHashFieldOffset));
      __ add(scratch2, scratch2, Immediate(
          StringDictionary::GetProbeOffset(i) << String::kHashShift));
    }
    __ lsr(scratch2, scratch2, Immediate(String::kHashShift));
    __ land(scratch2, scratch1, scratch2);

    // Scale the index by multiplying by the element size.
    ASSERT(StringDictionary::kEntrySize == 3);
    // scratch2 = scratch2 * 3.
    __ add(r3, scratch2, scratch2);
    __ add(scratch2, r3, scratch2);

    // Check if the key is identical to the name.
    __ lsl(scratch2, scratch2, Immediate(2));
    __ add(scratch2, scratch2, elements);
    __ mov(r3, FieldMemOperand(scratch2, kElementsStartOffset));
    __ cmpeq(name, r3);
    if (i != kProbes - 1) {
      __ bt(done);
    } else {
      __ bf(miss);
    }
  }
}


// Helper function used from LoadIC/CallIC GenerateNormal.
//
// elements: Property dictionary. It is not clobbered if a jump to the miss
//           label is done.
// name:     Property name. It is not clobbered if a jump to the miss label is
//           done
// result:   Register for the result. It is only updated if a jump to the miss
//           label is not done. Can be the same as elements or name clobbering
//           one of these in the case of not jumping to the miss label.
// The two scratch registers need to be different from elements, name and
// result.
// The generated code assumes that the receiver has slow properties,
// is not a global object and does not have interceptors.
static void GenerateDictionaryLoad(MacroAssembler* masm,
                                   Label* miss,
                                   Register elements,
                                   Register name,
                                   Register result,
                                   Register scratch1,
                                   Register scratch2) {
  ASSERT(!scratch1.is(r3) && !scratch2.is(r3));

  // Main use of the scratch registers.
  // scratch1: Used as temporary and to hold the capacity of the property
  //           dictionary.
  // scratch2: Used as temporary.
  Label done;

  // Probe the dictionary.
  GenerateStringDictionaryProbes(masm,
                                 miss,
                                 &done,
                                 elements,
                                 name,
                                 scratch1,
                                 scratch2);

  // If probing finds an entry check that the value is a normal
  // property.
  __ bind(&done);  // scratch2 == elements + 4 * index
  const int kElementsStartOffset = StringDictionary::kHeaderSize +
      StringDictionary::kElementsStartIndex * kPointerSize;
  const int kDetailsOffset = kElementsStartOffset + 2 * kPointerSize;
  __ mov(scratch1, FieldMemOperand(scratch2, kDetailsOffset));
  __ mov(r3, Immediate(PropertyDetails::TypeField::mask() << kSmiTagSize));
  __ tst(scratch1, r3);
  __ bf(miss);

  // Get the value at the masked, scaled index and return.
  __ mov(result,
         FieldMemOperand(scratch2, kElementsStartOffset + 1 * kPointerSize));
}


void LoadIC::GenerateNormal(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r4    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  //  -- sp[0] : receiver
  // -----------------------------------
  Label miss;

  GenerateStringDictionaryReceiverCheck(masm, r4, r5, r6, r7, &miss);

  // r1: elements
  GenerateDictionaryLoad(masm, &miss, r5, r6, r4, r7, r1);
  __ rts();

  // Cache miss: Jump to runtime.
  __ bind(&miss);
  GenerateMiss(masm);
}


void LoadIC::GenerateStringLength(MacroAssembler* masm, bool support_wrappers) {
  // ----------- S t a t e -------------
  //  -- r4    : receiver
  //  -- r6    : name
  //  -- lr    : return address
  //  -- sp[0] : receiver
  // -----------------------------------
  Label miss;

  StubCompiler::GenerateLoadStringLength(masm, r4, r5, r7, &miss,
                                         support_wrappers);
  // Cache miss: Jump to runtime.
  __ bind(&miss);
  StubCompiler::GenerateLoadMiss(masm, Code::LOAD_IC);
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


void KeyedLoadIC::GenerateString(v8::internal::MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- pr     : return address
  //  -- r4     : key (index)
  //  -- r5     : receiver
  // -----------------------------------
  Label miss;

  Register receiver = r1;
  Register index = r0;
  Register scratch1 = r2;
  Register scratch2 = r3;
  Register result = r0;

  StringCharAtGenerator char_at_generator(receiver,
                                          index,
                                          scratch1,
                                          scratch2,
                                          result,
                                          &miss,  // When not a string.
                                          &miss,  // When not a number.
                                          &miss,  // When index out of range.
                                          STRING_INDEX_IS_ARRAY_INDEX);
  char_at_generator.GenerateFast(masm);
  __ rts();

  StubRuntimeCallHelper call_helper;
  char_at_generator.GenerateSlow(masm, call_helper);

  __ bind(&miss);
  GenerateMiss(masm);
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
