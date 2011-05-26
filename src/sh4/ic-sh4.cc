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

#include "assembler-sh4.h"
#include "code-stubs.h"
#include "codegen.h"
#include "disasm.h"
#include "ic-inl.h"
#include "runtime.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {


//
// ************************************************
// *** WARNING: ARM to SH4 mapping. READ CAREFULLY.
// ************************************************
// Starting from this point, we use a systematic mapping for converting
// ARM code to SH4 code.
// Some registers must be checked, see defines below.
// if needing an additional register, use sh4_r8
// all other registers unchanged
//
// For this we define a fixed mapping based on #define.
// All functions should come from the ARM implementation
// in ic-arm.cc
// If this latter file is updated, please also update this one.
//
#define r0 sh4_r0
#define r1 sh4_r1
#define r2 sh4_r2
#define r3 sh4_r10
#define r4 sh4_r4
#define r5 sh4_r5
#define r6 sh4_r6
#define r7 sh4_r7
#define r8 "should be cp"
#define r9 sh4_r9
#define ip sh4_r11
#define lr pr
#define pc "to be checked"
#define r10 "should be roots"
#define r11 "Unexpected"
#define r12 "Unexpected"
#define r13 "Unexpected"
#define r14 "Unexpected"
#define r15 "Unexpected"


// ----------------------------------------------------------------------------
// Static IC stub generators.
//

#define __ ACCESS_MASM(masm)


static void GenerateGlobalInstanceTypeCheck(MacroAssembler* masm,
                                            Register type,
                                            Label* global_object) {
  // Register usage:
  //   type: holds the receiver instance type on entry.
  __ cmpeq(type, Immediate(JS_GLOBAL_OBJECT_TYPE));
  __ bt(global_object);
  __ cmpeq(type, Immediate(JS_BUILTINS_OBJECT_TYPE));
  __ bt(global_object);
  __ cmpeq(type, Immediate(JS_GLOBAL_PROXY_TYPE));
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
  __ ldrb(t1, FieldMemOperand(t0, Map::kBitFieldOffset));
  __ tst(t1, Immediate((1 << Map::kIsAccessCheckNeeded) |
                     (1 << Map::kHasNamedInterceptor)));
  __ bf(miss);

  __ ldr(elements, FieldMemOperand(receiver, JSObject::kPropertiesOffset));
  __ ldr(t1, FieldMemOperand(elements, HeapObject::kMapOffset));
  __ LoadRoot(ip, Heap::kHashTableMapRootIndex);
  __ cmpeq(t1, ip);
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
  // Assert that name contains a string.
  if (FLAG_debug_code) __ AbortIfNotString(name);

  // Compute the capacity mask.
  const int kCapacityOffset = StringDictionary::kHeaderSize +
      StringDictionary::kCapacityIndex * kPointerSize;
  __ ldr(scratch1, FieldMemOperand(elements, kCapacityOffset));
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
    __ ldr(scratch2, FieldMemOperand(name, String::kHashFieldOffset));
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
    __ lsl(ip, scratch2, Immediate(1));
    __ add(scratch2, scratch2, ip);

    // Check if the key is identical to the name.
    __ lsl(scratch2, scratch2, Immediate(2));
    __ add(scratch2, elements, scratch2);
    __ ldr(ip, FieldMemOperand(scratch2, kElementsStartOffset));
    __ cmpeq(name, /*TODO:Operand()*/ip);
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
  __ ldr(scratch1, FieldMemOperand(scratch2, kDetailsOffset));
  __ tst(scratch1, Immediate(PropertyDetails::TypeField::mask() << kSmiTagSize));
  __ bf(miss);

  // Get the value at the masked, scaled index and return.
  __ ldr(result,
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
  __ ldr(scratch1, FieldMemOperand(scratch2, kDetailsOffset));
  __ tst(scratch1, Immediate(kTypeAndReadOnlyMask));
  __ bf(miss);

  // Store the value at the masked, scaled index and return.
  const int kValueOffset = kElementsStartOffset + kPointerSize;
  __ add(scratch2, scratch2, Immediate(kValueOffset - kHeapObjectTag));
  __ str(value, MemOperand(scratch2));

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
  // t2 - used for the index into the dictionary and as scratch for the first part
  Label done;

  // Compute the hash code from the untagged key.  This must be kept in sync
  // with ComputeIntegerHash in utils.h.
  //
  // hash = ~hash + (hash << 15);
  __ mvn(t1, /*TODO:Operand()*/t0);
  __ lsl(t2, t0, Immediate(15));
  __ add(t0, t1, t2);
  // hash = hash ^ (hash >> 12);
  __ lsr(t2, t0, Immediate(12));
  __ eor(t0, t0, t2);
  // hash = hash + (hash << 2);
  __ lsl(t2, t0, Immediate(2));
  __ add(t0, t0, t2);
  // hash = hash ^ (hash >> 4);
  __ lsr(t2, t0, Immediate(4));
  __ eor(t0, t0, t2);
  // hash = hash * 2057;
  __ mov(t1, Immediate(2057));
  __ mul(t0, t0, t1);
  // hash = hash ^ (hash >> 16);
  __ lsr(t2, t0, Immediate(16));
  __ eor(t0, t0, t2);

  // Compute the capacity mask.
  __ ldr(t1, FieldMemOperand(elements, NumberDictionary::kCapacityOffset));
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
    __ land(t2, t2, /*TODO:Operand()*/t1);

    // Scale the index by multiplying by the element size.
    ASSERT(NumberDictionary::kEntrySize == 3);
    __ lsl(ip, t2, Immediate(1));
    __ add(t2, t2, ip);  // t2 = t2 * 3

    // Check if the key is identical to the name.
    __ lsl(ip, t2, Immediate(kPointerSizeLog2));
    __ add(t2, elements, ip);
    __ ldr(ip, FieldMemOperand(t2, NumberDictionary::kElementsStartOffset));
    __ cmpeq(key, /*TODO:Operand()*/ip);
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
  __ ldr(t1, FieldMemOperand(t2, kDetailsOffset));
  __ tst(t1, Immediate(Smi::FromInt(PropertyDetails::TypeField::mask())));
  __ bf(miss);

  // Get the value at the masked, scaled index and return.
  const int kValueOffset =
      NumberDictionary::kElementsStartOffset + kPointerSize;
  __ ldr(result, FieldMemOperand(t2, kValueOffset));
}


void LoadIC::GenerateArrayLength(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  //  -- r0    : receiver
  //  -- sp[0] : receiver
  // -----------------------------------
  Label miss;

  StubCompiler::GenerateLoadArrayLength(masm, r0, r3, &miss);
  __ bind(&miss);
  StubCompiler::GenerateLoadMiss(masm, Code::LOAD_IC);
}


void LoadIC::GenerateStringLength(MacroAssembler* masm, bool support_wrappers) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  //  -- r0    : receiver
  //  -- sp[0] : receiver
  // -----------------------------------
  Label miss;

  StubCompiler::GenerateLoadStringLength(masm, r0, r1, r3, &miss,
                                         support_wrappers);
  // Cache miss: Jump to runtime.
  __ bind(&miss);
  StubCompiler::GenerateLoadMiss(masm, Code::LOAD_IC);
}


void LoadIC::GenerateFunctionPrototype(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  //  -- r0    : receiver
  //  -- sp[0] : receiver
  // -----------------------------------
  Label miss;

  StubCompiler::GenerateLoadFunctionPrototype(masm, r0, r1, r3, &miss);
  __ bind(&miss);
  StubCompiler::GenerateLoadMiss(masm, Code::LOAD_IC);
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
  __ ldr(hash, FieldMemOperand(key, String::kHashFieldOffset));
  __ tst(hash, Immediate(String::kContainsCachedArrayIndexMask));
  __ bt(index_string);

  // Is the string a symbol?
  // map: key map
  __ ldrb(hash, FieldMemOperand(map, Map::kInstanceTypeOffset));
  ASSERT(kSymbolTag != 0);
  __ tst(hash, Immediate(kIsSymbolMask));
  __ bt(not_symbol);
}


// Defined in ic.cc.
Object* CallIC_Miss(Arguments args);

// The generated code does not accept smi keys.
// The generated code falls through if both probes miss.
static void GenerateMonomorphicCacheProbe(MacroAssembler* masm,
                                          int argc,
                                          Code::Kind kind) {
  // ----------- S t a t e -------------
  //  -- r1    : receiver
  //  -- r2    : name
  // -----------------------------------
  Label number, non_number, non_string, boolean, probe, miss;

  // Probe the stub cache.
  Code::Flags flags = Code::ComputeFlags(kind,
                                         NOT_IN_LOOP,
                                         MONOMORPHIC,
                                         Code::kNoExtraICState,
                                         NORMAL,
                                         argc);
  Isolate::Current()->stub_cache()->GenerateProbe(
      masm, flags, r1, r2, r3, r4, r5);

  // If the stub cache probing failed, the receiver might be a value.
  // For value objects, we use the map of the prototype objects for
  // the corresponding JSValue for the cache and that is what we need
  // to probe.
  //
  // Check for number.
  __ tst(r1, Immediate/*TODO:Operand*/(kSmiTagMask));
  __ bt(&number);
  __ CompareObjectType(r1, r3, r3, HEAP_NUMBER_TYPE, eq);
  __ bf(&non_number);
  __ bind(&number);
  StubCompiler::GenerateLoadGlobalFunctionPrototype(
      masm, Context::NUMBER_FUNCTION_INDEX, r1);
  __ b(&probe);

  // Check for string.
  __ bind(&non_number);
  __ cmphs(r3, Immediate/*TODO:Operand*/(FIRST_NONSTRING_TYPE));
  __ bt(&non_string);
  StubCompiler::GenerateLoadGlobalFunctionPrototype(
      masm, Context::STRING_FUNCTION_INDEX, r1);
  __ b(&probe);

  // Check for boolean.
  __ bind(&non_string);
  __ LoadRoot(ip, Heap::kTrueValueRootIndex);
  __ cmpeq(r1, ip);
  __ bt(&boolean);
  __ LoadRoot(ip, Heap::kFalseValueRootIndex);
  __ cmpeq(r1, ip);
  __ bf(&miss);
  __ bind(&boolean);
  StubCompiler::GenerateLoadGlobalFunctionPrototype(
      masm, Context::BOOLEAN_FUNCTION_INDEX, r1);

  // Probe the stub cache for the value object.
  __ bind(&probe);
  Isolate::Current()->stub_cache()->GenerateProbe(
      masm, flags, r1, r2, r3, r4, r5);

  __ bind(&miss);
}


static void GenerateFunctionTailCall(MacroAssembler* masm,
                                     int argc,
                                     Label* miss,
                                     Register scratch) {
  // r1: function

  // Check that the value isn't a smi.
  __ tst(r1, Immediate/*TODO:Operand*/(kSmiTagMask));
  __ bt(miss);

  // Check that the value is a JSFunction.
  __ CompareObjectType(r1, scratch, scratch, JS_FUNCTION_TYPE, eq);
  __ bf(miss);

  // Invoke the function.
  ParameterCount actual(argc);
  __ InvokeFunction(r1, actual, JUMP_FUNCTION);
}


static void GenerateCallNormal(MacroAssembler* masm, int argc) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  Label miss;

  // Get the receiver of the function from the stack into r1.
  __ ldr(r1, MemOperand(sp, argc * kPointerSize));

  GenerateStringDictionaryReceiverCheck(masm, r1, r0, r3, r4, &miss);

  // r0: elements
  // Search the dictionary - put result in register r1.
  GenerateDictionaryLoad(masm, &miss, r0, r2, r1, r3, r4);

  GenerateFunctionTailCall(masm, argc, &miss, r4);

  __ bind(&miss);
}


static void GenerateCallMiss(MacroAssembler* masm, int argc, IC::UtilityId id) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  Isolate* isolate = masm->isolate();

  if (id == IC::kCallIC_Miss) {
    __ IncrementCounter(isolate->counters()->call_miss(), 1, r3, r4);
  } else {
    __ IncrementCounter(isolate->counters()->keyed_call_miss(), 1, r3, r4);
  }

  // Get the receiver of the function from the stack.
  __ ldr(r3, MemOperand(sp, argc * kPointerSize));

  __ EnterInternalFrame();

  // Push the receiver and the name of the function.
  __ Push(r3, r2);

  // Call the entry.
  __ mov(r0, Immediate/*TODO:Operand*/(2));
  __ mov(r1, Operand(ExternalReference(IC_Utility(id), isolate)));

  CEntryStub stub(1);
  __ CallStub(&stub);

  // Move result to r1 and leave the internal frame.
  __ mov(r1, /*TODO:Operand()*/r0);
  __ LeaveInternalFrame();

  // Check if the receiver is a global object of some sort.
  // This can happen only for regular CallIC but not KeyedCallIC.
  if (id == IC::kCallIC_Miss) {
    Label invoke, global;
    __ ldr(r2, MemOperand(sp, argc * kPointerSize));  // receiver
    __ tst(r2, Immediate/*TODO:Operand*/(kSmiTagMask));
    __ bt(&invoke);
    __ CompareObjectType(r2, r3, r3, JS_GLOBAL_OBJECT_TYPE, eq);
    __ bt(&global);
    __ cmpeq(r3, Immediate/*TODO:Operand*/(JS_BUILTINS_OBJECT_TYPE));
    __ bf(&invoke);

    // Patch the receiver on the stack.
    __ bind(&global);
    __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalReceiverOffset));
    __ str(r2, MemOperand(sp, argc * kPointerSize));
    __ bind(&invoke);
  }

  // Invoke the function.
  ParameterCount actual(argc);
  __ InvokeFunction(r1, actual, JUMP_FUNCTION);
}


void CallIC::GenerateMiss(MacroAssembler* masm, int argc) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  GenerateCallMiss(masm, argc, IC::kCallIC_Miss);
}


void CallIC::GenerateMegamorphic(MacroAssembler* masm, int argc) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  // Get the receiver of the function from the stack into r1.
  __ ldr(r1, MemOperand(sp, argc * kPointerSize));
  GenerateMonomorphicCacheProbe(masm, argc, Code::CALL_IC);
  GenerateMiss(masm, argc);
}


void CallIC::GenerateNormal(MacroAssembler* masm, int argc) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  GenerateCallNormal(masm, argc);
  GenerateMiss(masm, argc);
}


void KeyedCallIC::GenerateMiss(MacroAssembler* masm, int argc) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  GenerateCallMiss(masm, argc, IC::kKeyedCallIC_Miss);
}


void KeyedCallIC::GenerateMegamorphic(MacroAssembler* masm, int argc) {
  __ UNIMPLEMENTED_BREAK();
}


void KeyedCallIC::GenerateNormal(MacroAssembler* masm, int argc) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  // Check if the name is a string.
  Label miss;
  __ tst(r2, Immediate(kSmiTagMask));
  __ bt(&miss);
  __ IsObjectJSStringType(r2, r0, &miss);

  GenerateCallNormal(masm, argc);
  __ bind(&miss);
  GenerateMiss(masm, argc);
}


// Defined in ic.cc.
Object* LoadIC_Miss(Arguments args);

void LoadIC::GenerateMegamorphic(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  //  -- r0    : receiver
  //  -- sp[0] : receiver
  // -----------------------------------

  // Probe the stub cache.
  Code::Flags flags = Code::ComputeFlags(Code::LOAD_IC,
                                         NOT_IN_LOOP,
                                         MONOMORPHIC);
  Isolate::Current()->stub_cache()->GenerateProbe(
      masm, flags, r0, r2, r3, r4, r5);

  // Cache miss: Jump to runtime.
  GenerateMiss(masm);
}


void LoadIC::GenerateNormal(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  //  -- r0    : receiver
  //  -- sp[0] : receiver
  // -----------------------------------
  Label miss;

  GenerateStringDictionaryReceiverCheck(masm, r0, r1, r3, r4, &miss);

  // r1: elements
  GenerateDictionaryLoad(masm, &miss, r1, r2, r0, r3, r4);
  __ Ret();

  // Cache miss: Jump to runtime.
  __ bind(&miss);
  GenerateMiss(masm);
}


void LoadIC::GenerateMiss(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  //  -- r0    : receiver
  //  -- sp[0] : receiver
  // -----------------------------------
  Isolate* isolate = masm->isolate();

  __ IncrementCounter(isolate->counters()->load_miss(), 1, r3, r4);

  __ mov(r3, r0);
  __ Push(r3, r2);

  // Perform tail call to the entry.
  ExternalReference ref =
      ExternalReference(IC_Utility(kLoadIC_Miss), isolate);
  __ TailCallExternalReference(ref, 2, 1);
}


Object* KeyedLoadIC_Miss(Arguments args);


void KeyedLoadIC::GenerateMiss(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- lr     : return address
  //  -- r0     : key
  //  -- r1     : receiver
  // -----------------------------------
  Isolate* isolate = masm->isolate();

  __ IncrementCounter(isolate->counters()->keyed_load_miss(), 1, r3, r4);

  __ Push(r1, r0);

  ExternalReference ref =
      ExternalReference(IC_Utility(kKeyedLoadIC_Miss), isolate);
  __ TailCallExternalReference(ref, 2, 1);
}


void KeyedLoadIC::GenerateRuntimeGetProperty(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- lr     : return address
  //  -- r0     : key
  //  -- r1     : receiver
  // -----------------------------------

  __ Push(r1, r0);

  __ TailCallRuntime(Runtime::kKeyedGetProperty, 2, 1);
}


void KeyedLoadIC::GenerateGeneric(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- lr     : return address
  //  -- r0     : key
  //  -- r1     : receiver
  // -----------------------------------
  Label slow, check_string, index_smi, index_string, property_array_property;
  Label probe_dictionary, check_number_dictionary;

  Register key = r0;
  Register receiver = r1;

  Isolate* isolate = masm->isolate();

  // Check that the key is a smi.
  __ JumpIfNotSmi(key, &check_string);
  __ bind(&index_smi);
  // Now the key is known to be a smi. This place is also jumped to from below
  // where a numeric string is converted to a smi.

  //TODO
  __ UNIMPLEMENTED_BREAK();
  //GenerateKeyedLoadReceiverCheck(
  //    masm, receiver, r2, r3, Map::kHasIndexedInterceptor, &slow);

  // Check the "has fast elements" bit in the receiver's map which is
  // now in r2.
  __ ldrb(r3, FieldMemOperand(r2, Map::kBitField2Offset));
  __ tst(r3, Immediate(1 << Map::kHasFastElements));
  __ bt(&check_number_dictionary);

  //TODO
  __ UNIMPLEMENTED_BREAK();
  //GenerateFastArrayLoad(
  //  masm, receiver, key, r4, r3, r2, r0, NULL, &slow);
  __ IncrementCounter(isolate->counters()->keyed_load_generic_smi(), 1, r2, r3);
  __ Ret();

  __ bind(&check_number_dictionary);
  __ ldr(r4, FieldMemOperand(receiver, JSObject::kElementsOffset));
  __ ldr(r3, FieldMemOperand(r4, JSObject::kMapOffset));

  // Check whether the elements is a number dictionary.
  // r0: key
  // r3: elements map
  // r4: elements
  __ LoadRoot(ip, Heap::kHashTableMapRootIndex);
  __ cmpeq(r3, ip);
  __ bf(&slow);
  __ asr(r2, r0, Immediate(kSmiTagSize));
  GenerateNumberDictionaryLoad(masm, &slow, r4, r0, r0, r2, r3, r5);
  __ Ret();

  // Slow case, key and receiver still in r0 and r1.
  __ bind(&slow);
  __ IncrementCounter(isolate->counters()->keyed_load_generic_slow(),
                      1, r2, r3);
  GenerateRuntimeGetProperty(masm);

  __ bind(&check_string);
  GenerateKeyStringCheck(masm, key, r2, r3, &index_string, &slow);

  //TODO
  __ UNIMPLEMENTED_BREAK();
  //GenerateKeyedLoadReceiverCheck(
  //  masm, receiver, r2, r3, Map::kHasNamedInterceptor, &slow);

  // If the receiver is a fast-case object, check the keyed lookup
  // cache. Otherwise probe the dictionary.
  __ ldr(r3, FieldMemOperand(r1, JSObject::kPropertiesOffset));
  __ ldr(r4, FieldMemOperand(r3, HeapObject::kMapOffset));
  __ LoadRoot(ip, Heap::kHashTableMapRootIndex);
  __ cmpeq(r4, ip);
  __ bt(&probe_dictionary);

  // Load the map of the receiver, compute the keyed lookup cache hash
  // based on 32 bits of the map pointer and the string hash.
  __ ldr(r2, FieldMemOperand(r1, HeapObject::kMapOffset));
  __ asr(r3, r2, Immediate(KeyedLookupCache::kMapHashShift));
  __ ldr(r4, FieldMemOperand(r0, String::kHashFieldOffset));
  __ asr(r4, r4, Immediate(String::kHashShift));
  __ eor(r3, r3, r4);
  __ land(r3, r3, Immediate(KeyedLookupCache::kCapacityMask));

  // Load the key (consisting of map and symbol) from the cache and
  // check for match.
  ExternalReference cache_keys =
      ExternalReference::keyed_lookup_cache_keys(isolate);
  __ mov(r4, Operand(cache_keys));
  __ lsl(r5, r3, Immediate(kPointerSizeLog2 + 1));
  __ add(r4, r4, r5);
  __ ldr(r5, MemOperand(r4));  // Move r4 to symbol.
  __ add(r4, r4, Immediate(kPointerSize));
  __ cmpeq(r2, r5);
  __ bf(&slow);
  __ ldr(r5, MemOperand(r4));
  __ cmpeq(r0, r5);
  __ bf(&slow);

  // Get field offset.
  // r0     : key
  // r1     : receiver
  // r2     : receiver's map
  // r3     : lookup cache index
  ExternalReference cache_field_offsets =
      ExternalReference::keyed_lookup_cache_field_offsets(isolate);
  __ mov(r4, Operand(cache_field_offsets));
  __ lsl(r6, r3, Immediate(kPointerSizeLog2));
  __ ldr(r5, MemOperand(r4, r6));
  __ ldrb(r6, FieldMemOperand(r2, Map::kInObjectPropertiesOffset));
  __ UNIMPLEMENTED_BREAK();
  //TODO __ sub(r5, r5, r6, SetCC);
  //TODO __ b(ge, &property_array_property);//TODO

  // Load in-object property.
  __ ldrb(r6, FieldMemOperand(r2, Map::kInstanceSizeOffset));
  __ add(r6, r6, r5);  // Index from start of object.
  __ sub(r1, r1, Immediate(kHeapObjectTag));  // Remove the heap tag.
  //TODO: CHECK
  __ UNIMPLEMENTED_BREAK();
  //__ ldr(r0, MemOperand(r1, r6, LSL, kPointerSizeLog2));
  __ lsl(r6, r6, Immediate(kPointerSizeLog2));
  __ ldr(r0, MemOperand(r1, r6));
  __ IncrementCounter(isolate->counters()->keyed_load_generic_lookup_cache(),
                      1, r2, r3);
  __ Ret();

  // Load property array property.
  __ bind(&property_array_property);
  __ ldr(r1, FieldMemOperand(r1, JSObject::kPropertiesOffset));
  __ add(r1, r1, Immediate(FixedArray::kHeaderSize - kHeapObjectTag));
  __ lsl(r5, r5, Immediate(kPointerSizeLog2));
  __ ldr(r0, MemOperand(r1, r5));
  __ IncrementCounter(isolate->counters()->keyed_load_generic_lookup_cache(),
                      1, r2, r3);
  __ Ret();

  // Do a quick inline probe of the receiver's dictionary, if it
  // exists.
  __ bind(&probe_dictionary);
  // r1: receiver
  // r0: key
  // r3: elements
  __ ldr(r2, FieldMemOperand(r1, HeapObject::kMapOffset));
  __ ldrb(r2, FieldMemOperand(r2, Map::kInstanceTypeOffset));
  GenerateGlobalInstanceTypeCheck(masm, r2, &slow);
  // Load the property to r0.
  GenerateDictionaryLoad(masm, &slow, r3, r0, r0, r2, r4);
  __ IncrementCounter(isolate->counters()->keyed_load_generic_symbol(),
                      1, r2, r3);
  __ Ret();

  __ bind(&index_string);
  __ IndexFromHash(r3, key);
  // Now jump to the place where smi keys are handled.
  __ jmp(&index_smi);
}


void KeyedLoadIC::GenerateString(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- lr     : return address
  //  -- r0     : key (index)
  //  -- r1     : receiver
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
  __ Ret();

  StubRuntimeCallHelper call_helper;
  char_at_generator.GenerateSlow(masm, call_helper);

  __ bind(&miss);
  GenerateMiss(masm);
}


void KeyedLoadIC::GenerateIndexedInterceptor(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- lr     : return address
  //  -- r0     : key
  //  -- r1     : receiver
  // -----------------------------------
  Label slow;

  // Check that the receiver isn't a smi.
  __ JumpIfSmi(r1, &slow);

  // Check that the key is an array index, that is Uint32.
  __ tst(r0, Immediate(kSmiTagMask | kSmiSignMask));
  __ bf(&slow);

  // Get the map of the receiver.
  __ ldr(r2, FieldMemOperand(r1, HeapObject::kMapOffset));

  // Check that it has indexed interceptor and access checks
  // are not enabled for this object.
  __ ldrb(r3, FieldMemOperand(r2, Map::kBitFieldOffset));
  __ land(r3, r3, Immediate(kSlowCaseBitFieldMask));
  __ cmpeq(r3, Immediate(1 << Map::kHasIndexedInterceptor));
  __ bf(&slow);

  // Everything is fine, call runtime.
  __ Push(r1, r0);  // Receiver, key.

  // Perform tail call to the entry.
  __ TailCallExternalReference(
      ExternalReference(IC_Utility(kKeyedLoadPropertyWithInterceptor),
                        masm->isolate()),
      2,
      1);

  __ bind(&slow);
  GenerateMiss(masm);
}


void KeyedStoreIC::GenerateMiss(MacroAssembler* masm) {
  // ---------- S t a t e --------------
  //  -- r0     : value
  //  -- r1     : key
  //  -- r2     : receiver
  //  -- lr     : return address
  // -----------------------------------

  // Push receiver, key and value for runtime call.
  __ Push(r2, r1, r0);

  ExternalReference ref =
      ExternalReference(IC_Utility(kKeyedStoreIC_Miss), masm->isolate());
  __ TailCallExternalReference(ref, 3, 1);
}


void KeyedStoreIC::GenerateRuntimeSetProperty(MacroAssembler* masm,
                                              StrictModeFlag strict_mode) {
  // ---------- S t a t e --------------
  //  -- r0     : value
  //  -- r1     : key
  //  -- r2     : receiver
  //  -- lr     : return address
  // -----------------------------------

  // Push receiver, key and value for runtime call.
  __ Push(r2, r1, r0);

  __ mov(r1, Immediate(Smi::FromInt(NONE)));          // PropertyAttributes
  __ mov(r0, Immediate(Smi::FromInt(strict_mode)));   // Strict mode.
  __ Push(r1, r0);

  __ TailCallRuntime(Runtime::kSetProperty, 5, 1);
}


void KeyedStoreIC::GenerateGeneric(MacroAssembler* masm,
                                   StrictModeFlag strict_mode) {
  // ---------- S t a t e --------------
  //  -- r0     : value
  //  -- r1     : key
  //  -- r2     : receiver
  //  -- lr     : return address
  // -----------------------------------
  Label slow, fast, array, extra, noret;

  // Register usage.
  Register value = r0;
  Register key = r1;
  Register receiver = r2;
  Register elements = r3;  // Elements array of the receiver.
  // r4 and r5 are used as general scratch registers.

  // Check that the key is a smi.
  __ tst(key, Immediate(kSmiTagMask));
  __ bf(&slow);
  // Check that the object isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ bt(&slow);
  // Get the map of the object.
  __ ldr(r4, FieldMemOperand(receiver, HeapObject::kMapOffset));
  // Check that the receiver does not require access checks.  We need
  // to do this because this generic stub does not perform map checks.
  __ ldrb(ip, FieldMemOperand(r4, Map::kBitFieldOffset));
  __ tst(ip, Immediate(1 << Map::kIsAccessCheckNeeded));
  __ bf(&slow);
  // Check if the object is a JS array or not.
  __ ldrb(r4, FieldMemOperand(r4, Map::kInstanceTypeOffset));
  __ cmpeq(r4, Immediate(JS_ARRAY_TYPE));
  __ bt(&array);
  // Check that the object is some kind of JS object.
  __ cmpge(r4, Immediate(FIRST_JS_OBJECT_TYPE));
  __ bf(&slow);

  // Object case: Check key against length in the elements array.
  __ ldr(elements, FieldMemOperand(receiver, JSObject::kElementsOffset));
  // Check that the object is in fast mode and writable.
  __ ldr(r4, FieldMemOperand(elements, HeapObject::kMapOffset));
  __ LoadRoot(ip, Heap::kFixedArrayMapRootIndex);
  __ cmpeq(r4, ip);
  __ bf(&slow);
  // Check array bounds. Both the key and the length of FixedArray are smis.
  __ ldr(ip, FieldMemOperand(elements, FixedArray::kLengthOffset));
  __ cmphs(key, /*TODO:Operand()*/ip);
  __ bf(&fast);

  // Slow case, handle jump to runtime.
  __ bind(&slow);
  // Entry registers are intact.
  // r0: value.
  // r1: key.
  // r2: receiver.
  GenerateRuntimeSetProperty(masm, strict_mode);

  // Extra capacity case: Check if there is extra capacity to
  // perform the store and update the length. Used for adding one
  // element to the array by writing to array[array.length].
  __ bind(&extra);
  // Condition code from comparing key and array length is still available.
  __ bf(&slow);  // Only support writing to writing to array[array.length].
  // Check for room in the elements backing store.
  // Both the key and the length of FixedArray are smis.
  __ ldr(ip, FieldMemOperand(elements, FixedArray::kLengthOffset));
  __ cmphs(key, /*TODO:Operand()*/ip);
  __ bt(&slow);
  // Calculate key + 1 as smi.
  ASSERT_EQ(0, kSmiTag);
  __ add(r4, key, Immediate(Smi::FromInt(1)));
  __ str(r4, FieldMemOperand(receiver, JSArray::kLengthOffset));
  __ b(&fast);

  // Array case: Get the length and the elements array from the JS
  // array. Check that the array is in fast mode (and writable); if it
  // is the length is always a smi.
  __ bind(&array);
  __ ldr(elements, FieldMemOperand(receiver, JSObject::kElementsOffset));
  __ ldr(r4, FieldMemOperand(elements, HeapObject::kMapOffset));
  __ LoadRoot(ip, Heap::kFixedArrayMapRootIndex);
  __ cmpeq(r4, ip);
  __ bf(&slow);

  // Check the key against the length in the array.
  __ ldr(ip, FieldMemOperand(receiver, JSArray::kLengthOffset));
  __ cmphs(key, /*TODO:Operand()*/ip);
  __ bt(&extra);
  // Fall through to fast case.

  __ bind(&fast);
  // Fast case, store the value to the elements backing store.
  __ add(r5, elements, Immediate(FixedArray::kHeaderSize - kHeapObjectTag));
  __ lsl(r4, key, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(r5, r5, r4);
  __ str(value, MemOperand(r5));
  // Skip write barrier if the written value is a smi.
  __ tst(value, Immediate(kSmiTagMask));
  __ bf(&noret);
  __ Ret();
  __ bind(&noret);
  // Update write barrier for the elements array address.
  __ sub(r4, r5, /*TODO:Operand()*/elements);
  //TODO
  __ UNIMPLEMENTED_BREAK();
  //__ RecordWrite(elements, Operand(r4), r5, r6);

  __ Ret();
}


void StoreIC::GenerateMegamorphic(MacroAssembler* masm,
                                  StrictModeFlag strict_mode) {
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  // Get the receiver from the stack and probe the stub cache.
  Code::Flags flags = Code::ComputeFlags(Code::STORE_IC,
                                         NOT_IN_LOOP,
                                         MONOMORPHIC,
                                         strict_mode);

  Isolate::Current()->stub_cache()->GenerateProbe(
      masm, flags, r1, r2, r3, r4, r5);

  // Cache miss: Jump to runtime.
  GenerateMiss(masm);
}


void StoreIC::GenerateMiss(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  __ Push(r1, r2, r0);

  // Perform tail call to the entry.
  ExternalReference ref =
      ExternalReference(IC_Utility(kStoreIC_Miss), masm->isolate());
  __ TailCallExternalReference(ref, 3, 1);
}


void StoreIC::GenerateArrayLength(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  //
  // This accepts as a receiver anything JSObject::SetElementsLength accepts
  // (currently anything except for external and pixel arrays which means
  // anything with elements of FixedArray type.), but currently is restricted
  // to JSArray.
  // Value must be a number, but only smis are accepted as the most common case.

  Label miss;

  Register receiver = r1;
  Register value = r0;
  Register scratch = r3;

  // Check that the receiver isn't a smi.
  __ JumpIfSmi(receiver, &miss);

  // Check that the object is a JS array.
  __ CompareObjectType(receiver, scratch, scratch, JS_ARRAY_TYPE, eq);
  __ bf(&miss);

  // Check that elements are FixedArray.
  // We rely on StoreIC_ArrayLength below to deal with all types of
  // fast elements (including COW).
  __ ldr(scratch, FieldMemOperand(receiver, JSArray::kElementsOffset));
  __ CompareObjectType(scratch, scratch, scratch, FIXED_ARRAY_TYPE, eq);
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


void StoreIC::GenerateNormal(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  Label miss;

  GenerateStringDictionaryReceiverCheck(masm, r1, r3, r4, r5, &miss);

  GenerateDictionaryStore(masm, &miss, r3, r2, r0, r4, r5);
  Counters* counters = masm->isolate()->counters();
  __ IncrementCounter(counters->store_normal_hit(),
                      1, r4, r5);
  __ Ret();

  __ bind(&miss);
  __ IncrementCounter(counters->store_normal_miss(), 1, r4, r5);
  GenerateMiss(masm);
}


void StoreIC::GenerateGlobalProxy(MacroAssembler* masm,
                                  StrictModeFlag strict_mode) {
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  __ Push(r1, r2, r0);

  __ mov(r1, Immediate(Smi::FromInt(NONE)));  // PropertyAttributes
  __ mov(r0, Immediate(Smi::FromInt(strict_mode)));
  __ Push(r1, r0);

  // Do tail-call to runtime routine.
  __ TailCallRuntime(Runtime::kSetProperty, 5, 1);
}


#undef __

void PatchInlinedSmiCode(Address address) {
  UNIMPLEMENTED();
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_ARM
