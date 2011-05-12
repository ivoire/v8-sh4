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


// Helper function used from StoreIC::GenerateNormal.
//
// elements: Property dictionary. It is not clobbered if a jump to the miss
//           label is done.
// name:     Property name. It is not clobbered if a jump to the miss label is
//           done
// value:    The value to store.
// The two scratch registers need to be different from elements, name and
// result.
// The generated code assumes that the receiver has slow properties,
// is not a global object and does not have interceptors.
static void GenerateDictionaryStore(MacroAssembler* masm,
                                    Label* miss,
                                    Register elements,
                                    Register name,
                                    Register value,
                                    Register scratch1,
                                    Register scratch2) {
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

  // If probing finds an entry in the dictionary check that the value
  // is a normal property that is not read only.
  __ bind(&done);  // scratch2 == elements + 4 * index
  const int kElementsStartOffset = StringDictionary::kHeaderSize +
      StringDictionary::kElementsStartIndex * kPointerSize;
  const int kDetailsOffset = kElementsStartOffset + 2 * kPointerSize;
  const int kTypeAndReadOnlyMask
      = (PropertyDetails::TypeField::mask() |
         PropertyDetails::AttributesField::encode(READ_ONLY)) << kSmiTagSize;
  __ mov(scratch1, FieldMemOperand(scratch2, kDetailsOffset));
  __ tst(scratch1, Immediate(kTypeAndReadOnlyMask));
  __ bf(miss);

  // Store the value at the masked, scaled index and return.
  const int kValueOffset = kElementsStartOffset + kPointerSize;
  __ add(scratch2, scratch2, Immediate(kValueOffset - kHeapObjectTag));
  __ mov(MemOperand(scratch2), value);

  // Update the write barrier. Make sure not to clobber the value.
  __ mov(scratch1, value);
  __ RecordWrite(elements, scratch2, scratch1);
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
  // ----------- S t a t e -------------
  //  -- r4    : value
  //  -- r5    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  // -----------------------------------
  //
  // This accepts as a receiver anything JSObject::SetElementsLength accepts
  // (currently anything except for external and pixel arrays which means
  // anything with elements of FixedArray type.), but currently is restricted
  // to JSArray.
  // Value must be a number, but only smis are accepted as the most common case.

  Label miss;

  Register receiver = r5;
  Register value = r4;
  Register scratch = r7;

  // Check that the receiver isn't a smi.
  __ JumpIfSmi(receiver, &miss);

  // Check that the object is a JS array.
  __ CompareObjectType(receiver, scratch, scratch, JS_ARRAY_TYPE);
  __ bf(&miss);

  // Check that elements are FixedArray.
  // We rely on StoreIC_ArrayLength below to deal with all types of
  // fast elements (including COW).
  __ mov(scratch, FieldMemOperand(receiver, JSArray::kElementsOffset));
  __ CompareObjectType(scratch, scratch, scratch, FIXED_ARRAY_TYPE);
  __ bf(&miss);

  // Check that value is a smi.
  __ JumpIfNotSmi(value, &miss);

  // Prepare tail call to StoreIC_ArrayLength.
  __ Push(receiver, value);

  ExternalReference ref =
      ExternalReference(IC_Utility(kStoreIC_ArrayLength), masm->isolate());
  __ TailCallExternalReference(ref, 2, 1);

  __ bind(&miss);

  GenerateMiss(masm);
}


void StoreIC::GenerateGlobalProxy(MacroAssembler* masm,
                                  StrictModeFlag strict_mode) {
  // ----------- S t a t e -------------
  //  -- r4    : value
  //  -- r5    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  // -----------------------------------

  __ Push(r5, r6, r4);

  __ mov(r5, Immediate(Smi::FromInt(NONE)));  // PropertyAttributes
  __ mov(r4, Immediate(Smi::FromInt(strict_mode)));
  __ Push(r5, r4);

  // Do tail-call to runtime routine.
  __ TailCallRuntime(Runtime::kSetProperty, 5, 1);
}


void StoreIC::GenerateMegamorphic(MacroAssembler* masm,
                                  StrictModeFlag strict_mode) {
  // ----------- S t a t e -------------
  //  -- r4    : value
  //  -- r5    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  // -----------------------------------

  // Get the receiver from the stack and probe the stub cache.
  Code::Flags flags = Code::ComputeFlags(Code::STORE_IC,
                                         NOT_IN_LOOP,
                                         MONOMORPHIC,
                                         strict_mode);

  Isolate::Current()->stub_cache()->GenerateProbe(
      masm, flags, r5, r6, r7, r1, r2);

  // Cache miss: Jump to runtime.
  GenerateMiss(masm);
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
  // ----------- S t a t e -------------
  //  -- r4    : value
  //  -- r5    : receiver
  //  -- r6    : name
  //  -- pr    : return address
  // -----------------------------------
  Label miss;

  GenerateStringDictionaryReceiverCheck(masm, r5, r7, r1, r2, &miss);

  GenerateDictionaryStore(masm, &miss, r7, r6, r4, r1, r2);
  Counters* counters = masm->isolate()->counters();
  __ IncrementCounter(counters->store_normal_hit(),
                      1, r1, r2);
  __ rts();

  __ bind(&miss);
  __ IncrementCounter(counters->store_normal_miss(), 1, r1, r2);
  GenerateMiss(masm);
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


void KeyedLoadIC::GenerateIndexedInterceptor(v8::internal::MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- pr     : return address
  //  -- r4     : key
  //  -- r5     : receiver
  // -----------------------------------
  Label slow;

  // Check that the receiver isn't a smi.
  __ JumpIfSmi(r5, &slow);

  // Check that the key is an array index, that is Uint32.
  __ tst(r4, Immediate(kSmiTagMask | kSmiSignMask));
  __ bf(&slow);

  // Get the map of the receiver.
  __ mov(r6, FieldMemOperand(r5, HeapObject::kMapOffset));

  // Check that it has indexed interceptor and access checks
  // are not enabled for this object.
  __ mov(r7, FieldMemOperand(r6, Map::kBitFieldOffset));        // FIXME: mov.b ??
  __ land(r7, r7, Immediate(kSlowCaseBitFieldMask));
  __ mov(r3, Immediate(1 << Map::kHasIndexedInterceptor));
  __ cmpeq(r7, r3);
  __ bf(&slow);

  // Everything is fine, call runtime.
  __ Push(r5, r4);  // Receiver, key.

  // Perform tail call to the entry.
  __ TailCallExternalReference(
      ExternalReference(IC_Utility(kKeyedLoadPropertyWithInterceptor),
                        masm->isolate()),
      2,
      1);

  __ bind(&slow);
  GenerateMiss(masm);
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


void KeyedStoreIC::GenerateRuntimeSetProperty(MacroAssembler* masm,
                                              StrictModeFlag strict_mode) {
  // ---------- S t a t e --------------
  //  -- r4     : value
  //  -- r5     : key
  //  -- r6     : receiver
  //  -- pr     : return address
  // -----------------------------------

  // Push receiver, key and value for runtime call.
  __ Push(r6, r5, r4);

  __ mov(r5, Immediate(Smi::FromInt(NONE)));          // PropertyAttributes
  __ mov(r4, Immediate(Smi::FromInt(strict_mode)));   // Strict mode.
  __ Push(r5, r4);

  __ TailCallRuntime(Runtime::kSetProperty, 5, 1);
}


void KeyedStoreIC::GenerateGeneric(MacroAssembler* masm,
                                   StrictModeFlag strict_mode) {
  // ---------- S t a t e --------------
  //  -- r4     : value
  //  -- r5     : key
  //  -- r6     : receiver
  //  -- pr     : return address
  // -----------------------------------
  Label slow, fast, array, extra, end;

  // Register usage.
  Register value = r4;
  Register key = r5;
  Register receiver = r6;
  Register elements = r7;  // Elements array of the receiver.
  // r1 and r2 are used as general scratch registers.

  // Check that the key is a smi.
  __ tst(key, Immediate(kSmiTagMask));
  __ bf(&slow);
  // Check that the object isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ bt(&slow);
  // Get the map of the object.
  __ mov(r1, FieldMemOperand(receiver, HeapObject::kMapOffset));
  // Check that the receiver does not require access checks.  We need
  // to do this because this generic stub does not perform map checks.
  __ mov(r3, FieldMemOperand(r1, Map::kBitFieldOffset));        // FIXME: mov.b ??
  __ tst(r3, Immediate(1 << Map::kIsAccessCheckNeeded));
  __ bf(&slow);
  // Check if the object is a JS array or not.
  __ mov(r1, FieldMemOperand(r1, Map::kInstanceTypeOffset));    // FIXME: mov.b ??
  __ mov(r3, Immediate(JS_ARRAY_TYPE));
  __ cmpeq(r1, r3);
  __ bt(&array);
  // Check that the object is some kind of JS object.
  __ mov(r3, Immediate(FIRST_JS_OBJECT_TYPE));
  __ cmpgt(r3, r4);
  __ bt(&slow);

  // Object case: Check key against length in the elements array.
  __ mov(elements, FieldMemOperand(receiver, JSObject::kElementsOffset));
  // Check that the object is in fast mode and writable.
  __ mov(r1, FieldMemOperand(elements, HeapObject::kMapOffset));
  __ LoadRoot(r3, Heap::kFixedArrayMapRootIndex);
  __ cmpeq(r1, r3);
  __ bf(&slow);
  // Check array bounds. Both the key and the length of FixedArray are smis.
  __ mov(r3, FieldMemOperand(elements, FixedArray::kLengthOffset));
  __ cmpgtu(r3, key);
  __ bt(&fast);

  // Slow case, handle jump to runtime.
  __ bind(&slow);
  // Entry registers are intact.
  // r4: value.
  // r5: key.
  // r6: receiver.
  GenerateRuntimeSetProperty(masm, strict_mode);

  // Extra capacity case: Check if there is extra capacity to
  // perform the store and update the length. Used for adding one
  // element to the array by writing to array[array.length].
  __ bind(&extra);
  // Condition code from comparing key and array length is still available.
  __ bf(&slow);  // Only support writing to writing to array[array.length].
  // Check for room in the elements backing store.
  // Both the key and the length of FixedArray are smis.
  __ mov(r3, FieldMemOperand(elements, FixedArray::kLengthOffset));
  __ cmpgeu(key, r3);
  __ bt(&slow);
  // Calculate key + 1 as smi.
  ASSERT_EQ(0, kSmiTag);
  __ add(r1, key, Immediate(Smi::FromInt(1)));
  __ mov(FieldMemOperand(receiver, JSArray::kLengthOffset), r1);
  __ jmp(&fast);

  // Array case: Get the length and the elements array from the JS
  // array. Check that the array is in fast mode (and writable); if it
  // is the length is always a smi.
  __ bind(&array);
  __ mov(elements, FieldMemOperand(receiver, JSObject::kElementsOffset));
  __ mov(r1, FieldMemOperand(elements, HeapObject::kMapOffset));
  __ LoadRoot(r3, Heap::kFixedArrayMapRootIndex);
  __ cmpeq(r4, r3);
  __ bf(&slow);

  // Check the key against the length in the array.
  __ mov(r3, FieldMemOperand(receiver, JSArray::kLengthOffset));
  __ cmpgeu(key, r3);
  __ bt(&extra);
  // Fall through to fast case.

  __ bind(&fast);
  // Fast case, store the value to the elements backing store.
  __ add(r2, elements, Immediate(FixedArray::kHeaderSize - kHeapObjectTag));
  __ lsl(r3, key, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(r2, r2, r3);
  __ mov(MemOperand(r2), value);
  // Skip write barrier if the written value is a smi.
  __ tst(value, Immediate(kSmiTagMask));
  __ bt(&end);
  __ rts();
  // Update write barrier for the elements array address.
  __ sub(r1, r2, elements);
  __ RecordWrite(elements, r1, r2);     // FIXME: not exactly like arm one

  __ bind(&end);
  __ rts();
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
