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
  ASSERT(result.is(r0));

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


static void GenerateNumberDictionaryLoad(MacroAssembler* masm,
                                         Label* miss,
                                         Register elements,
                                         Register key,
                                         Register result,
                                         Register t0,
                                         Register t1,
                                         Register t2) {
  ASSERT(!elements.is(r3) && !key.is(r3) && !result.is(r3) &&
         !t0.is(r3) && !t1.is(r3) && !t2.is(r3));
  ASSERT(result.is(r0));
  // Register use:
  //
  // elements - holds the slow-case elements of the receiver on entry.
  //            Unchanged unless 'result' is the same register.
  //
  // key      - holds the smi key on entry.
  //            Unchanged unless 'result' is the same register.
  //
  // result   - holds the result on exit if the load succeeded.
  //            Allowed to be the same as 'key' or 'result'.
  //            Unchanged on bailout so 'key' or 'result' can be used
  //            in further computation.
  //
  // Scratch registers:
  //
  // t0 - holds the untagged key on entry and holds the hash once computed.
  //
  // t1 - used to hold the capacity mask of the dictionary
  //
  // t2 - used for the index into the dictionary.
  Label done;

  // Compute the hash code from the untagged key.  This must be kept in sync
  // with ComputeIntegerHash in utils.h.
  //
  // hash = ~hash + (hash << 15);
  __ lnot(t1, t0);
  __ lsl(r3, t0, Immediate(15));
  __ add(t0, t1, r3);
  // hash = hash ^ (hash >> 12);
  __ lsr(r3, t0, Immediate(12));
  __ lor(t0, t0, r3);
  // hash = hash + (hash << 2);
  __ lsl(r3, t0, Immediate(2));
  __ add(t0, t0, r3);
  // hash = hash ^ (hash >> 4);
  __ lsr(r3, t0, Immediate(4));
  __ lor(t0, t0, r3);
  // hash = hash * 2057;
  __ mov(t1, Immediate(2057));
  __ mul(t0, t0, t1);
  // hash = hash ^ (hash >> 16);
  __ lsr(r3, t0, Immediate(16));
  __ lor(t0, t0, r3);

  // Compute the capacity mask.
  __ mov(t1, FieldMemOperand(elements, NumberDictionary::kCapacityOffset));
  __ asr(t1, t1, Immediate(kSmiTagSize));
  __ sub(t1, t1, Immediate(1));

  // Generate an unrolled loop that performs a few probes before giving up.
  static const int kProbes = 4;
  for (int i = 0; i < kProbes; i++) {
    // Use t2 for index calculations and keep the hash intact in t0.
    __ mov(t2, t0);
    // Compute the masked index: (hash + i + i * i) & mask.
    if (i > 0) {
      __ add(t2, t2, Immediate(NumberDictionary::GetProbeOffset(i)));
    }
    __ land(t2, t2, t1);

    // Scale the index by multiplying by the element size.
    ASSERT(NumberDictionary::kEntrySize == 3);
    __ lsl(r3, t2, Immediate(1));
    __ add(t2, t2, r3);  // t2 = t2 * 3

    // Check if the key is identical to the name.
    __ lsl(r3, t2, Immediate(kPointerSizeLog2));
    __ add(t2, elements, r3);
    __ mov(r3, FieldMemOperand(t2, NumberDictionary::kElementsStartOffset));
    __ cmpeq(key, r3);
    if (i != kProbes - 1) {
      __ bt(&done);
    } else {
      __ bf(miss);
    }
  }

  __ bind(&done);
  // Check that the value is a normal property.
  // t2: elements + (index * kPointerSize)
  const int kDetailsOffset =
      NumberDictionary::kElementsStartOffset + 2 * kPointerSize;
  __ mov(t1, FieldMemOperand(t2, kDetailsOffset));
  __ tst(t1, Immediate(Smi::FromInt(PropertyDetails::TypeField::mask())));
  __ bf(miss);

  // Get the value at the masked, scaled index and return.
  const int kValueOffset =
      NumberDictionary::kElementsStartOffset + kPointerSize;
  __ mov(result, FieldMemOperand(t2, kValueOffset));
}


// Checks whether a key is an array index string or a symbol string.
// Falls through if a key is a symbol.
static void GenerateKeyStringCheck(MacroAssembler* masm,
                                   Register key,
                                   Register map,
                                   Register hash,
                                   Label* index_string,
                                   Label* not_symbol) {
  // The key is not a smi.
  // Is it a string?
  __ CompareObjectType(key, map, hash, FIRST_NONSTRING_TYPE, ge);
  __ bt(not_symbol);

  // Is the string an array index, with cached numeric value?
  __ mov(hash, FieldMemOperand(key, String::kHashFieldOffset));
  __ tst(hash, Immediate(String::kContainsCachedArrayIndexMask));
  __ bt(index_string);

  // Is the string a symbol?
  // map: key map
  __ mov(hash, FieldMemOperand(map, Map::kInstanceTypeOffset)); // FIXME: mov.b ??
  ASSERT(kSymbolTag != 0);
  __ tst(hash, Immediate(kIsSymbolMask));
  __ bt(not_symbol);
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
  GenerateDictionaryLoad(masm, &miss, r5, r6, r0, r7, r1);
  __ rts();

  // Cache miss: Jump to runtime.
  __ bind(&miss);
  GenerateMiss(masm);
}


void LoadIC::GenerateStringLength(MacroAssembler* masm, bool support_wrappers) {
  // ----------- S t a t e -------------
  //  -- r4    : receiver
  //  -- r6    : name
  //  -- pr    : return address
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


// Checks the receiver for special cases (value type, slow case bits).
// Falls through for regular JS object.
static void GenerateKeyedLoadReceiverCheck(MacroAssembler* masm,
                                           Register receiver,
                                           Register map,
                                           Register scratch,
                                           int interceptor_bit,
                                           Label* slow) {
  ASSERT(!receiver.is(r3) && !map.is(r3) && !scratch.is(r3));
  // Check that the object isn't a smi.
  __ JumpIfSmi(receiver, slow);
  // Get the map of the receiver.
  __ mov(map, FieldMemOperand(receiver, HeapObject::kMapOffset));
  // Check bit field.
  __ mov(scratch, FieldMemOperand(map, Map::kBitFieldOffset));  // FIXME: mov.b ??
  __ tst(scratch,
         Immediate((1 << Map::kIsAccessCheckNeeded) | (1 << interceptor_bit)));
  __ bf(slow);
  // Check that the object is some kind of JS object EXCEPT JS Value type.
  // In the case that the object is a value-wrapper object,
  // we enter the runtime system to make sure that indexing into string
  // objects work as intended.
  ASSERT(JS_OBJECT_TYPE > JS_VALUE_TYPE);
  __ mov(scratch, FieldMemOperand(map, Map::kInstanceTypeOffset));      // FIXME: mov.b ??
  __ mov(r3, Immediate(JS_OBJECT_TYPE));
  __ cmpgt(r3, scratch);
  __ bt(slow);
}


// Loads an indexed element from a fast case array.
// If not_fast_array is NULL, doesn't perform the elements map check.
static void GenerateFastArrayLoad(MacroAssembler* masm,
                                  Register receiver,
                                  Register key,
                                  Register elements,
                                  Register scratch1,
                                  Register scratch2,
                                  Register result,
                                  Label* not_fast_array,
                                  Label* out_of_range) {
 ASSERT(!receiver.is(r3) && !key.is(r3) && !elements.is(r3) &&
        !scratch1.is(r3) && !scratch2.is(r3) && !result.is(r3));
  ASSERT(result.is(r0));

  // Register use:
  //
  // receiver - holds the receiver on entry.
  //            Unchanged unless 'result' is the same register.
  //
  // key      - holds the smi key on entry.
  //            Unchanged unless 'result' is the same register.
  //
  // elements - holds the elements of the receiver on exit.
  //
  // result   - holds the result on exit if the load succeeded.
  //            Allowed to be the the same as 'receiver' or 'key'.
  //            Unchanged on bailout so 'receiver' and 'key' can be safely
  //            used by further computation.
  //
  // Scratch registers:
  //
  // scratch1 - used to hold elements map and elements length.
  //            Holds the elements map if not_fast_array branch is taken.
  //
  // scratch2 - used to hold the loaded value.

  __ mov(elements, FieldMemOperand(receiver, JSObject::kElementsOffset));
  if (not_fast_array != NULL) {
    // Check that the object is in fast mode and writable.
    __ mov(scratch1, FieldMemOperand(elements, HeapObject::kMapOffset));
    __ LoadRoot(r3, Heap::kFixedArrayMapRootIndex);
    __ cmpeq(scratch1, r3);
    __ bf(not_fast_array);
  } else {
    __ AssertFastElements(elements);
  }

  // Check that the key (index) is within bounds.
  __ mov(scratch1, FieldMemOperand(elements, FixedArray::kLengthOffset));
  __ cmpgeu(key, scratch1);
  __ bt(out_of_range);
  // Fast case: Do the load.
  __ add(scratch1, elements, Immediate(FixedArray::kHeaderSize - kHeapObjectTag));
  // The key is a smi.
  ASSERT(kSmiTag == 0 && kSmiTagSize < kPointerSizeLog2);
  __ lsl(r3, key, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ mov(scratch2,
         MemOperand(scratch1, r3));
  __ LoadRoot(r3, Heap::kTheHoleValueRootIndex);
  __ cmpeq(scratch2, r3);
  // In case the loaded value is the_hole we have to consult GetProperty
  // to ensure the prototype chain is searched.
  __ bt(out_of_range);
  __ mov(result, scratch2);
}


void KeyedLoadIC::GenerateRuntimeGetProperty(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- pr     : return address
  //  -- r4     : key
  //  -- r5     : receiver
  // -----------------------------------

  __ Push(r5, r4);

  __ TailCallRuntime(Runtime::kKeyedGetProperty, 2, 1);
}


void KeyedLoadIC::GenerateGeneric(v8::internal::MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- pr     : return address
  //  -- r4     : key
  //  -- r5     : receiver
  // -----------------------------------
  Label slow, check_string, index_smi, index_string, property_array_property;
  Label probe_dictionary, check_number_dictionary;

  Register key = r4;
  Register receiver = r5;

  Isolate* isolate = masm->isolate();

  // Check that the key is a smi.
  __ JumpIfNotSmi(key, &check_string);
  __ bind(&index_smi);
  // Now the key is known to be a smi. This place is also jumped to from below
  // where a numeric string is converted to a smi.

  GenerateKeyedLoadReceiverCheck(
      masm, receiver, r6, r7, Map::kHasIndexedInterceptor, &slow);

  // Check the "has fast elements" bit in the receiver's map which is
  // now in r6.
  __ mov(r7, FieldMemOperand(r6, Map::kBitField2Offset));       // FIXME: mov.b ??
  __ tst(r7, Immediate(1 << Map::kHasFastElements));
  __ bt(&check_number_dictionary);

  GenerateFastArrayLoad(
      masm, receiver, key, r1, r7, r6, r0, NULL, &slow);
  __ IncrementCounter(isolate->counters()->keyed_load_generic_smi(), 1, r2, r3);
  __ rts();

  __ bind(&check_number_dictionary);
  __ mov(r1, FieldMemOperand(receiver, JSObject::kElementsOffset));
  __ mov(r7, FieldMemOperand(r1, JSObject::kMapOffset));

  // Check whether the elements is a number dictionary.
  // r0: key
  // r7: elements map
  // r1: elements
  __ LoadRoot(r3, Heap::kHashTableMapRootIndex);
  __ cmpeq(r7, r3);
  __ bf(&slow);
  __ asr(r6, r4, Immediate(kSmiTagSize));
  GenerateNumberDictionaryLoad(masm, &slow, r1, r4, r0, r6, r7, r2);
  __ rts();

  // Slow case, key and receiver still in r0 and r1.
  __ bind(&slow);
  __ IncrementCounter(isolate->counters()->keyed_load_generic_slow(),
                      1, r6, r7);
  GenerateRuntimeGetProperty(masm);

  __ bind(&check_string);
  GenerateKeyStringCheck(masm, key, r6, r7, &index_string, &slow);

  GenerateKeyedLoadReceiverCheck(
      masm, receiver, r6, r7, Map::kHasNamedInterceptor, &slow);

  // If the receiver is a fast-case object, check the keyed lookup
  // cache. Otherwise probe the dictionary.
  __ mov(r7, FieldMemOperand(r5, JSObject::kPropertiesOffset));
  __ mov(r1, FieldMemOperand(r7, HeapObject::kMapOffset));
  __ LoadRoot(r3, Heap::kHashTableMapRootIndex);
  __ cmpeq(r1, r3);
  __ bt(&probe_dictionary);

  // Load the map of the receiver, compute the keyed lookup cache hash
  // based on 32 bits of the map pointer and the string hash.
  __ mov(r6, FieldMemOperand(r5, HeapObject::kMapOffset));
  __ asr(r7, r6, Immediate(KeyedLookupCache::kMapHashShift));
  __ mov(r1, FieldMemOperand(r4, String::kHashFieldOffset));
  __ asr(r7, r1, Immediate(String::kHashShift));
  __ lor(r7, r7, r7);
  __ land(r7, r7, Immediate(KeyedLookupCache::kCapacityMask));

  // Load the key (consisting of map and symbol) from the cache and
  // check for match.
  ExternalReference cache_keys =
      ExternalReference::keyed_lookup_cache_keys(isolate);
  __ mov(r7, Operand(cache_keys));
  __ lsl(r3, r7, Immediate(kPointerSizeLog2 + 1));
  __ add(r1, r1, r3);
  __ mov(r2, MemOperand(r1, kPointerSize));  // Move r1 to symbol.
  __ add(r1, r1, Immediate(kPointerSize));

  __ cmpeq(r6, r2);
  __ bf(&slow);
  __ mov(r2, MemOperand(r1));
  __ cmpeq(r4, r2);
  __ bf(&slow);

  // Get field offset.
  // r4     : key
  // r5     : receiver
  // r6     : receiver's map
  // r7     : lookup cache index
  ExternalReference cache_field_offsets =
      ExternalReference::keyed_lookup_cache_field_offsets(isolate);
  __ mov(r1, Operand(cache_field_offsets));
  __ lsl(r3, r7, Immediate(kPointerSizeLog2));
  __ mov(r2, MemOperand(r1, r3));
  __ mov(r3, FieldMemOperand(r6, Map::kInObjectPropertiesOffset));      // FIXME: mov.b ??
  __ sub(r2, r2, r3);
  __ mov(r3, Immediate(0));
  __ cmpge(r2, r3);
  __ bt(&property_array_property);

  // Load in-object property.
  __ mov(r3, FieldMemOperand(r6, Map::kInstanceSizeOffset));            // FIXME: mov.b ??
  __ add(r3, r3, r2);  // Index from start of object.
  __ sub(r5, r5, Immediate(kHeapObjectTag));  // Remove the heap tag.
  __ lsl(r3, r5, Immediate(kPointerSizeLog2));
  __ mov(r4, MemOperand(r5, r3));                                       // FIXME: return value ?
  __ IncrementCounter(isolate->counters()->keyed_load_generic_lookup_cache(),
                      1, r6, r7);
  __ rts();

  // Load property array property.
  __ bind(&property_array_property);
  __ mov(r5, FieldMemOperand(r5, JSObject::kPropertiesOffset));
  __ add(r5, r5, Immediate(FixedArray::kHeaderSize - kHeapObjectTag));
  __ lsl(r3, r2, Immediate(kPointerSizeLog2));
  __ mov(r4, MemOperand(r5, r3));                                       // FIXME: return value ?
  __ IncrementCounter(isolate->counters()->keyed_load_generic_lookup_cache(),
                      1, r6, r7);
  __ rts();

  // Do a quick inline probe of the receiver's dictionary, if it
  // exists.
  __ bind(&probe_dictionary);
  // r5: receiver
  // r4: key
  // r7: elements
  __ mov(r6, FieldMemOperand(r5, HeapObject::kMapOffset));
  __ mov(r6, FieldMemOperand(r6, Map::kInstanceTypeOffset));    // FIXME: mov.b ??
  GenerateGlobalInstanceTypeCheck(masm, r6, &slow);
  // Load the property to r0.
  GenerateDictionaryLoad(masm, &slow, r7, r4, r0, r6, r1);
  __ IncrementCounter(isolate->counters()->keyed_load_generic_symbol(),
                      1, r6, r7);
  __ rts();

  __ bind(&index_string);
  __ IndexFromHash(r7, key);
  // Now jump to the place where smi keys are handled.
  __ jmp(&index_smi);
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
