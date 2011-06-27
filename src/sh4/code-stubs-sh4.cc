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

#include "bootstrapper.h"
#include "code-stubs.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm)

static void EmitIdenticalObjectComparison(MacroAssembler* masm,
                                          Label* slow,
                                          Condition cond,
                                          bool never_nan_nan);
static void EmitSmiNonsmiComparison(MacroAssembler* masm,
                                    Register lhs,
                                    Register rhs,
                                    Label* lhs_not_nan,
                                    Label* slow,
                                    bool strict);
static void EmitTwoNonNanDoubleComparison(MacroAssembler* masm, Condition cond);
static void EmitStrictTwoHeapObjectCompare(MacroAssembler* masm,
                                           Register lhs,
                                           Register rhs);
static void EmitCheckForTwoHeapNumbers(MacroAssembler* masm,
                                       Register lhs,
                                       Register rhs,
                                       Label* both_loaded_as_doubles,
                                       Label* not_heap_numbers,
                                       Label* slow);
static void EmitCheckForSymbolsOrObjects(MacroAssembler* masm,
                                         Register lhs,
                                         Register rhs,
                                         Label* possible_strings,
                                         Label* not_both_strings);
void EmitNanCheck(MacroAssembler* masm, Label* lhs_not_nan, Condition cond);


// Copy from ARM
#include "map-sh4.h" // Define register map

void ToNumberStub::Generate(MacroAssembler* masm) {
  // Entry argument: r0
  // Exit in: r0
#ifdef DEBUG
  // Clobber other parameter registers on entry.
  __ Dead(r1, r2, r3);
  __ Dead(r4, r5, r6, r7);
#endif
  Label check_heap_number, call_builtin;
  __ tst(r0, Immediate(kSmiTagMask));
  __ b(ne, &check_heap_number);
  __ Ret();

  __ bind(&check_heap_number);
  __ ldr(r1, FieldMemOperand(r0, HeapObject::kMapOffset));
  __ LoadRoot(ip, Heap::kHeapNumberMapRootIndex);
  __ cmp(r1, ip);
  __ b(ne, &call_builtin);
  __ Ret();

  __ bind(&call_builtin);
  __ push(r0);
  __ InvokeBuiltin(Builtins::TO_NUMBER, JUMP_JS);
}


void FastNewClosureStub::Generate(MacroAssembler* masm) {
  // Create a new closure from the given function info in new
  // space. Set the context to the current context in cp.
  Label gc;

  // Pop the function info from the stack.
  __ pop(r3);

  // Attempt to allocate new JSFunction in new space.
  __ AllocateInNewSpace(JSFunction::kSize,
                        r0,
                        r1,
                        r2,
                        &gc,
                        TAG_OBJECT);

  int map_index = strict_mode_ == kStrictMode
      ? Context::STRICT_MODE_FUNCTION_MAP_INDEX
      : Context::FUNCTION_MAP_INDEX;

  // Compute the function map in the current global context and set that
  // as the map of the allocated object.
  __ ldr(r2, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalContextOffset));
  __ ldr(r2, MemOperand(r2, Context::SlotOffset(map_index)));
  __ str(r2, FieldMemOperand(r0, HeapObject::kMapOffset));

  // Initialize the rest of the function. We don't have to update the
  // write barrier because the allocated object is in new space.
  __ LoadRoot(r1, Heap::kEmptyFixedArrayRootIndex);
  __ LoadRoot(r2, Heap::kTheHoleValueRootIndex);
  __ LoadRoot(r4, Heap::kUndefinedValueRootIndex);
  __ str(r1, FieldMemOperand(r0, JSObject::kPropertiesOffset));
  __ str(r1, FieldMemOperand(r0, JSObject::kElementsOffset));
  __ str(r2, FieldMemOperand(r0, JSFunction::kPrototypeOrInitialMapOffset));
  __ str(r3, FieldMemOperand(r0, JSFunction::kSharedFunctionInfoOffset));
  __ str(cp, FieldMemOperand(r0, JSFunction::kContextOffset));
  __ str(r1, FieldMemOperand(r0, JSFunction::kLiteralsOffset));
  __ str(r4, FieldMemOperand(r0, JSFunction::kNextFunctionLinkOffset));


  // Initialize the code pointer in the function to be the one
  // found in the shared function info object.
  __ ldr(r3, FieldMemOperand(r3, SharedFunctionInfo::kCodeOffset));
  __ add(r3, r3, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ str(r3, FieldMemOperand(r0, JSFunction::kCodeEntryOffset));

  // Return result. The argument function info has been popped already.
  __ Ret();

  // Create a new closure through the slower runtime call.
  __ bind(&gc);
  __ LoadRoot(r4, Heap::kFalseValueRootIndex);
  __ Push(cp, r3, r4);
  __ TailCallRuntime(Runtime::kNewClosure, 3, 1);
}


void FastNewContextStub::Generate(MacroAssembler* masm) {
  // Try to allocate the context in new space.
  Label gc;
  int length = slots_ + Context::MIN_CONTEXT_SLOTS;

  // Attempt to allocate the context in new space.
  __ AllocateInNewSpace(FixedArray::SizeFor(length),
                        r0,
                        r1,
                        r2,
                        &gc,
                        TAG_OBJECT);

  // Load the function from the stack.
  __ ldr(r3, MemOperand(sp, 0));

  // Setup the object header.
  __ LoadRoot(r2, Heap::kContextMapRootIndex);
  __ str(r2, FieldMemOperand(r0, HeapObject::kMapOffset));
  __ mov(r2, Immediate(Smi::FromInt(length)));
  __ str(r2, FieldMemOperand(r0, FixedArray::kLengthOffset));

  // Setup the fixed slots.
  __ mov(r1, Immediate(Smi::FromInt(0)));
  __ str(r3, MemOperand(r0, Context::SlotOffset(Context::CLOSURE_INDEX)));
  __ str(r0, MemOperand(r0, Context::SlotOffset(Context::FCONTEXT_INDEX)));
  __ str(r1, MemOperand(r0, Context::SlotOffset(Context::PREVIOUS_INDEX)));
  __ str(r1, MemOperand(r0, Context::SlotOffset(Context::EXTENSION_INDEX)));

  // Copy the global object from the surrounding context.
  __ ldr(r1, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  __ str(r1, MemOperand(r0, Context::SlotOffset(Context::GLOBAL_INDEX)));

  // Initialize the rest of the slots to undefined.
  __ LoadRoot(r1, Heap::kUndefinedValueRootIndex);
  for (int i = Context::MIN_CONTEXT_SLOTS; i < length; i++) {
    __ str(r1, MemOperand(r0, Context::SlotOffset(i)));
  }

  // Remove the on-stack argument and return.
  __ mov(cp, r0);
  __ pop();
  __ Ret();

  // Need to collect. Call into runtime system.
  __ bind(&gc);
  __ TailCallRuntime(Runtime::kNewContext, 1, 1);
}


void FastCloneShallowArrayStub::Generate(MacroAssembler* masm) {
  // Stack layout on entry:
  //
  // [sp]: constant elements.
  // [sp + kPointerSize]: literal index.
  // [sp + (2 * kPointerSize)]: literals array.

  // All sizes here are multiples of kPointerSize.
  int elements_size = (length_ > 0) ? FixedArray::SizeFor(length_) : 0;
  int size = JSArray::kSize + elements_size;

  // Load boilerplate object into r3 and check if we need to create a
  // boilerplate.
  Label slow_case;
  __ ldr(r3, MemOperand(sp, 2 * kPointerSize));
  __ ldr(r0, MemOperand(sp, 1 * kPointerSize));
  __ add(r3, r3, Immediate(FixedArray::kHeaderSize - kHeapObjectTag));
  __ lsl(ip, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ ldr(r3, MemOperand(r3, ip));
  __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
  __ cmp(r3, ip);
  __ b(eq, &slow_case);

  if (FLAG_debug_code) {
    const char* message;
    Heap::RootListIndex expected_map_index;
    if (mode_ == CLONE_ELEMENTS) {
      message = "Expected (writable) fixed array";
      expected_map_index = Heap::kFixedArrayMapRootIndex;
    } else {
      ASSERT(mode_ == COPY_ON_WRITE_ELEMENTS);
      message = "Expected copy-on-write fixed array";
      expected_map_index = Heap::kFixedCOWArrayMapRootIndex;
    }
    __ push(r3);
    __ ldr(r3, FieldMemOperand(r3, JSArray::kElementsOffset));
    __ ldr(r3, FieldMemOperand(r3, HeapObject::kMapOffset));
    __ LoadRoot(ip, expected_map_index);
    __ cmp(r3, ip);
    __ Assert(eq, message);
    __ pop(r3);
  }

  // Allocate both the JS array and the elements array in one big
  // allocation. This avoids multiple limit checks.
  __ AllocateInNewSpace(size,
                        r0,
                        r1,
                        r2,
                        &slow_case,
                        TAG_OBJECT);

  // Copy the JS array part.
  for (int i = 0; i < JSArray::kSize; i += kPointerSize) {
    if ((i != JSArray::kElementsOffset) || (length_ == 0)) {
      __ ldr(r1, FieldMemOperand(r3, i));
      __ str(r1, FieldMemOperand(r0, i));
    }
  }

  if (length_ > 0) {
    // Get hold of the elements array of the boilerplate and setup the
    // elements pointer in the resulting object.
    __ ldr(r3, FieldMemOperand(r3, JSArray::kElementsOffset));
    __ add(r2, r0, Immediate(JSArray::kSize));
    __ str(r2, FieldMemOperand(r0, JSArray::kElementsOffset));

    // Copy the elements array.
    __ CopyFields(r2, r3, r1.bit(), elements_size / kPointerSize);
  }

  // Return and remove the on-stack parameters.
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ Ret();

  __ bind(&slow_case);
  __ TailCallRuntime(Runtime::kCreateArrayLiteralShallow, 3, 1);
}


// Takes a Smi and converts to an IEEE 64 bit floating point value in two
// registers.  The format is 1 sign bit, 11 exponent bits (biased 1023) and
// 52 fraction bits (20 in the first word, 32 in the second).  Zeros is a
// scratch register.  Destroys the source register.  No GC occurs during this
// stub so you don't have to set up the frame.
class ConvertToDoubleStub : public CodeStub {
 public:
  ConvertToDoubleStub(Register result_reg_1,
                      Register result_reg_2,
                      Register source_reg,
                      Register scratch_reg)
      : result1_(result_reg_1),
        result2_(result_reg_2),
        source_(source_reg),
        zeros_(scratch_reg) { }

 private:
  Register result1_;
  Register result2_;
  Register source_;
  Register zeros_;

  // Minor key encoding in 16 bits.
  class ModeBits: public BitField<OverwriteMode, 0, 2> {};
  class OpBits: public BitField<Token::Value, 2, 14> {};

  Major MajorKey() { return ConvertToDouble; }
  int MinorKey() {
    // Encode the parameters in a unique 16 bit value.
    return  result1_.code() +
           (result2_.code() << 4) +
           (source_.code() << 8) +
           (zeros_.code() << 12);
  }

  void Generate(MacroAssembler* masm);

  const char* GetName() { return "ConvertToDoubleStub"; }

#ifdef DEBUG
  void Print() { PrintF("ConvertToDoubleStub\n"); }
#endif
};


void ConvertToDoubleStub::Generate(MacroAssembler* masm) {
  Register exponent = result1_;
  Register mantissa = result2_;

  Label not_special;
  // Convert from Smi to integer.
  __ asr(source_, source_, Immediate(kSmiTagSize));
  // Move sign bit from source to destination.  This works because the sign bit
  // in the exponent word of the double has the same position and polarity as
  // the 2's complement sign bit in a Smi.
  STATIC_ASSERT(HeapNumber::kSignMask == 0x80000000u);
  __ land(exponent, source_, Immediate(HeapNumber::kSignMask));
  __ tst(exponent, exponent);
  // Subtract from 0 if source was negative.
  __ rsb(source_, source_, Immediate(0), ne);

  // We have -1, 0 or 1, which we treat specially. Register source_ contains
  // absolute value: it is either equal to 1 (special case of -1 and 1),
  // greater than 1 (not a special case) or less than 1 (special case of 0).
  __ cmpgt(source_, Immediate(1));
  __ b(&not_special);

  // For 1 or -1 we need to or in the 0 exponent (biased to 1023).
  static const uint32_t exponent_word_for_1 =
      HeapNumber::kExponentBias << HeapNumber::kExponentShift;
  __ orr(exponent, exponent, Immediate(exponent_word_for_1), eq);
  // 1, 0 and -1 all have 0 for the second word.
  __ mov(mantissa, Immediate(0));
  __ Ret();

  __ bind(&not_special);
  // Count leading zeros.  Uses mantissa for a scratch register on pre-ARM5.
  // Gets the wrong answer for 0, but we already checked for that case above.
  __ CountLeadingZeros(zeros_, source_, mantissa);
  // Compute exponent and or it into the exponent register.
  // We use mantissa as a scratch register here.  Use a fudge factor to
  // divide the constant 31 + HeapNumber::kExponentBias, 0x41d, into two parts
  // that fit in the ARM's constant field.
  int fudge = 0x400;
  __ rsb(mantissa, zeros_, Immediate(31 + HeapNumber::kExponentBias - fudge));
  __ add(mantissa, mantissa, Immediate(fudge));
  __ lsl(ip, mantissa, Immediate(HeapNumber::kExponentShift));
  __ orr(exponent, exponent, ip);
  // Shift up the source chopping the top bit off.
  __ add(zeros_, zeros_, Immediate(1));
  // This wouldn't work for 1.0 or -1.0 as the shift would be 32 which means 0.
  __ lsl(source_, source_, zeros_);
  // Compute lower part of fraction (last 12 bits).
  __ lsl(mantissa, source_, Immediate(HeapNumber::kMantissaBitsInTopWord));
  // And the top (top 20 bits).
  __ lsr(ip, source_, Immediate(32 - HeapNumber::kMantissaBitsInTopWord));
  __ orr(exponent, exponent, ip);
  __ Ret();
}


void FloatingPointHelper::LoadSmis(MacroAssembler* masm,
                                   FloatingPointHelper::Destination destination,
                                   Register scratch1,
                                   Register scratch2) {
//TODO: VFP
//  if (CpuFeatures::IsSupported(VFP3)) {
//    CpuFeatures::Scope scope(VFP3);
//    __ mov(scratch1, Operand(r0, ASR, kSmiTagSize));
//    __ vmov(d7.high(), scratch1);
//    __ vcvt_f64_s32(d7, d7.high());
//    __ mov(scratch1, Operand(r1, ASR, kSmiTagSize));
//    __ vmov(d6.high(), scratch1);
//    __ vcvt_f64_s32(d6, d6.high());
//    if (destination == kCoreRegisters) {
//      __ vmov(r2, r3, d7);
//      __ vmov(r0, r1, d6);
//    }
//  } else
  {
    ASSERT(destination == kCoreRegisters);
    // Write Smi from r0 to r3 and r2 in double format.
    __ mov(scratch1, r0);
    ConvertToDoubleStub stub1(r3, r2, scratch1, scratch2);
    __ push(lr);
    __ Call(stub1.GetCode(), RelocInfo::CODE_TARGET);
    // Write Smi from r1 to r1 and r0 in double format.
    __ mov(scratch1, r1);
    ConvertToDoubleStub stub2(r1, r0, scratch1, scratch2);
    __ Call(stub2.GetCode(), RelocInfo::CODE_TARGET);
    __ pop(lr);
  }
}


void FloatingPointHelper::LoadOperands(
    MacroAssembler* masm,
    FloatingPointHelper::Destination destination,
    Register heap_number_map,
    Register scratch1,
    Register scratch2,
    Label* slow) {

  // Load right operand (r0) to d6 or r2/r3.
  LoadNumber(masm, destination,
             r0, dr8, r2, r3, heap_number_map, scratch1, scratch2, slow);

  // Load left operand (r1) to d7 or r0/r1.
  LoadNumber(masm, destination,
             r1, dr6, r0, r1, heap_number_map, scratch1, scratch2, slow);
}


void FloatingPointHelper::LoadNumber(MacroAssembler* masm,
                                     Destination destination,
                                     Register object,
                                     DwVfpRegister dst,
                                     Register dst1,
                                     Register dst2,
                                     Register heap_number_map,
                                     Register scratch1,
                                     Register scratch2,
                                     Label* not_number) {
  if (FLAG_debug_code) {
    __ AbortIfNotRootValue(heap_number_map,
                           Heap::kHeapNumberMapRootIndex,
                           "HeapNumberMap register clobbered.");
  }

  Label is_smi, done;

  __ JumpIfSmi(object, &is_smi);
  __ JumpIfNotHeapNumber(object, heap_number_map, scratch1, not_number);

  // Handle loading a double from a heap number.
//TODO: VFP
//  if (CpuFeatures::IsSupported(VFP3) &&
//      destination == kVFPRegisters) {
//    CpuFeatures::Scope scope(VFP3);
//    // Load the double from tagged HeapNumber to double register.
//    __ sub(scratch1, object, Operand(kHeapObjectTag));
//    __ vldr(dst, scratch1, HeapNumber::kValueOffset);
//  } else
  {
    ASSERT(destination == kCoreRegisters);
    // Load the double from heap number to dst1 and dst2 in double format.
    __ Ldrd(dst1, dst2, FieldMemOperand(object, HeapNumber::kValueOffset));
  }
  __ jmp(&done);

  // Handle loading a double from a smi.
  __ bind(&is_smi);
//TODO: VFP
//  if (CpuFeatures::IsSupported(VFP3)) {
//    CpuFeatures::Scope scope(VFP3);
//    // Convert smi to double using VFP instructions.
//    __ SmiUntag(scratch1, object);
//    __ vmov(dst.high(), scratch1);
//    __ vcvt_f64_s32(dst, dst.high());
//    if (destination == kCoreRegisters) {
//      // Load the converted smi to dst1 and dst2 in double format.
//      __ vmov(dst1, dst2, dst);
//    }
//  } else
  {
    ASSERT(destination == kCoreRegisters);
    // Write smi to dst1 and dst2 double format.
    __ mov(scratch1, object);
    ConvertToDoubleStub stub(dst2, dst1, scratch1, scratch2);
    __ push(lr);
    __ Call(stub.GetCode(), RelocInfo::CODE_TARGET);
    __ pop(lr);
  }

  __ bind(&done);
}


void FloatingPointHelper::CallCCodeForDoubleOperation(
    MacroAssembler* masm,
    Token::Value op,
    Register heap_number_result,
    Register scratch) {
  // Using core registers:
  // r0: Left value (least significant part of mantissa).
  // r1: Left value (sign, exponent, top of mantissa).
  // r2: Right value (least significant part of mantissa).
  // r3: Right value (sign, exponent, top of mantissa).

  // Assert that heap_number_result is callee-saved.
  // We currently always use r5 to pass it.
  ASSERT(heap_number_result.is(r5));

  // Push the current return address before the C call. Return will be
  // through pop(pc) below.
  __ push(lr);
  __ PrepareCallCFunction(4, scratch);  // Two doubles are 4 arguments.
  // Call C routine that may not cause GC or other trouble.
  __ CallCFunction(ExternalReference::double_fp_operation(op, masm->isolate()),
                   4);
  // Store answer in the overwritable heap number. Double returned in
  // registers r0 and r1.
  __ Strd(r0, r1, FieldMemOperand(heap_number_result,
                                  HeapNumber::kValueOffset));
  // Place heap_number_result in r0 and return to the pushed return address.
  __ mov(r0, heap_number_result);
  __ pop(lr);
  __ rts();
}


// See comment for class.
void WriteInt32ToHeapNumberStub::Generate(MacroAssembler* masm) {
  Label max_negative_int;
  // the_int_ has the answer which is a signed int32 but not a Smi.
  // We test for the special value that has a different exponent.  This test
  // has the neat side effect of setting the flags according to the sign.
  STATIC_ASSERT(HeapNumber::kSignMask == 0x80000000u);
  __ cmp(the_int_, Immediate(0x80000000u));
  __ b(eq, &max_negative_int);
  // Set up the correct exponent in scratch_.  All non-Smi int32s have the same.
  // A non-Smi integer is 1.xxx * 2^30 so the exponent is 30 (biased).
  uint32_t non_smi_exponent =
      (HeapNumber::kExponentBias + 30) << HeapNumber::kExponentShift;
  __ mov(scratch_, Immediate(non_smi_exponent));
  __ cmpge(the_int_, Immediate(0));
  Label skip;
  __ bt(&skip);
  // Set the sign bit in scratch_ if the value was negative.
  __ lor(scratch_, scratch_, Immediate(HeapNumber::kSignMask));
  // Subtract from 0 if the value was negative.
  __ rsb(the_int_, the_int_, Immediate(0));
  __ bind(&skip);
  // We should be masking the implict first digit of the mantissa away here,
  // but it just ends up combining harmlessly with the last digit of the
  // exponent that happens to be 1.  The sign bit is 0 so we shift 10 to get
  // the most significant 1 to hit the last bit of the 12 bit sign and exponent.
  ASSERT(((1 << HeapNumber::kExponentShift) & non_smi_exponent) != 0);
  const int shift_distance = HeapNumber::kNonMantissaBitsInTopWord - 2;
  __ lsr(ip, the_int_, Immediate(shift_distance));
  __ lor(scratch_, scratch_, ip);
  __ str(scratch_, FieldMemOperand(the_heap_number_,
                                   HeapNumber::kExponentOffset));
  __ lsl(scratch_, the_int_, Immediate(32 - shift_distance));
  __ str(scratch_, FieldMemOperand(the_heap_number_,
                                   HeapNumber::kMantissaOffset));
  __ Ret();

  __ bind(&max_negative_int);
  // The max negative int32 is stored as a positive number in the mantissa of
  // a double because it uses a sign bit instead of using two's complement.
  // The actual mantissa bits stored are all 0 because the implicit most
  // significant 1 bit is not stored.
  non_smi_exponent += 1 << HeapNumber::kExponentShift;
  __ mov(ip, Immediate(HeapNumber::kSignMask | non_smi_exponent));
  __ str(ip, FieldMemOperand(the_heap_number_, HeapNumber::kExponentOffset));
  __ mov(ip, Immediate(0));
  __ str(ip, FieldMemOperand(the_heap_number_, HeapNumber::kMantissaOffset));
  __ Ret();
}


// Handle the case where the lhs and rhs are the same object.
// Equality is almost reflexive (everything but NaN), so this is a test
// for "identity and not NaN".
static void EmitIdenticalObjectComparison(MacroAssembler* masm,
                                          Label* slow,
                                          Condition cond,
                                          bool never_nan_nan) {
  Label not_identical;
  Label heap_number, return_equal;
  __ cmp(r0, r1);
  __ b(ne, &not_identical);

  // The two objects are identical.  If we know that one of them isn't NaN then
  // we now know they test equal.
  if (cond != eq || !never_nan_nan) {
    // Test for NaN. Sadly, we can't just compare to FACTORY->nan_value(),
    // so we do the second best thing - test it ourselves.
    // They are both equal and they are not both Smis so both of them are not
    // Smis.  If it's not a heap number, then return equal.
    if (cond == lt || cond == gt) {
      __ CompareObjectType(r0, r4, r4, FIRST_JS_OBJECT_TYPE, ge);
      __ b(t, slow);
    } else {
      __ CompareObjectType(r0, r4, r4, HEAP_NUMBER_TYPE, eq);
      __ b(eq, &heap_number);
      // Comparing JS objects with <=, >= is complicated.
      if (cond != eq) {
        __ cmpge(r4, Immediate(FIRST_JS_OBJECT_TYPE));
        __ b(t, slow);
        // Normally here we fall through to return_equal, but undefined is
        // special: (undefined == undefined) == true, but
        // (undefined <= undefined) == false!  See ECMAScript 11.8.5.
        if (cond == le || cond == ge) {
          __ cmp(r4, Immediate(ODDBALL_TYPE));
          __ b(ne, &return_equal);
          __ LoadRoot(r2, Heap::kUndefinedValueRootIndex);
          __ cmp(r0, r2);
          __ b(ne, &return_equal);
          if (cond == le) {
            // undefined <= undefined should fail.
            __ mov(r0, Immediate(GREATER));
          } else  {
            // undefined >= undefined should fail.
            __ mov(r0, Immediate(LESS));
          }
          __ Ret();
        }
      }
    }
  }

  __ bind(&return_equal);
  if (cond == lt) {
    __ mov(r0, Operand(GREATER));  // Things aren't less than themselves.
  } else if (cond == gt) {
    __ mov(r0, Operand(LESS));     // Things aren't greater than themselves.
  } else {
    __ mov(r0, Operand(EQUAL));    // Things are <=, >=, ==, === themselves.
  }
  __ Ret();

  if (cond != eq || !never_nan_nan) {
    // For less and greater we don't have to check for NaN since the result of
    // x < x is false regardless.  For the others here is some code to check
    // for NaN.
    if (cond != lt && cond != gt) {
      __ bind(&heap_number);
      // It is a heap number, so return non-equal if it's NaN and equal if it's
      // not NaN.

      // The representation of NaN values has all exponent bits (52..62) set,
      // and not all mantissa bits (0..51) clear.
      // Read top bits of double representation (second word of value).
      __ ldr(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
      // Test that exponent bits are all set.
      __ Sbfx(r3, r2, HeapNumber::kExponentShift, HeapNumber::kExponentBits);
      // NaNs have all-one exponents so they sign extend to -1.
      __ cmp(r3, Immediate(-1));
      __ b(ne, &return_equal);

      // Shift out flag and all exponent bits, retaining only mantissa.
      __ lsl(r2, r2, Immediate(HeapNumber::kNonMantissaBitsInTopWord));
      // Or with all low-bits of mantissa.
      __ ldr(r3, FieldMemOperand(r0, HeapNumber::kMantissaOffset));
      __ orr(r0, r3, r2);
      __ tst(r0, r0);
      // For equal we already have the right value in r0:  Return zero (equal)
      // if all bits in mantissa are zero (it's an Infinity) and non-zero if
      // not (it's a NaN).  For <= and >= we need to load r0 with the failing
      // value if it's a NaN.
      if (cond != eq) {
        // All-zero means Infinity means equal.
        __ Ret(eq);
        if (cond == le) {
          __ mov(r0, Operand(GREATER));  // NaN <= NaN should fail.
        } else {
          __ mov(r0, Operand(LESS));     // NaN >= NaN should fail.
        }
      }
      __ Ret();
    }
    // No fall through here.
  }

  __ bind(&not_identical);
}


// See comment at call site.
static void EmitSmiNonsmiComparison(MacroAssembler* masm,
                                    Register lhs,
                                    Register rhs,
                                    Label* lhs_not_nan,
                                    Label* slow,
                                    bool strict) {
  ASSERT((lhs.is(r0) && rhs.is(r1)) ||
         (lhs.is(r1) && rhs.is(r0)));

  Label rhs_is_smi;
  __ tst(rhs, Immediate(kSmiTagMask));
  __ b(eq, &rhs_is_smi);

  // Lhs is a Smi.  Check whether the rhs is a heap number.
  __ CompareObjectType(rhs, r4, r4, HEAP_NUMBER_TYPE, eq);
  if (strict) {
    // If rhs is not a number and lhs is a Smi then strict equality cannot
    // succeed.  Return non-equal
    // If rhs is r0 then there is already a non zero value in it.
    if (!rhs.is(r0)) {
      __ mov(r0, Immediate(NOT_EQUAL), ne);
    }
    __ Ret(ne);
  } else {
    // Smi compared non-strictly with a non-Smi non-heap-number.  Call
    // the runtime.
    __ b(ne, slow);
  }

  // Lhs is a smi, rhs is a number.
  //TODO: floating point
  __ push(lr);
  // Convert lhs to a double in r2, r3.
  __ mov(r7, lhs);
  ConvertToDoubleStub stub1(r3, r2, r7, r6);
  __ Call(stub1.GetCode(), RelocInfo::CODE_TARGET);
  // Load rhs to a double in r0, r1.
  __ Ldrd(r0, r1, FieldMemOperand(rhs, HeapNumber::kValueOffset));
  __ pop(lr);

  // We now have both loaded as doubles but we can skip the lhs nan check
  // since it's a smi.
  __ jmp(lhs_not_nan);

  __ bind(&rhs_is_smi);
  // Rhs is a smi.  Check whether the non-smi lhs is a heap number.
  __ CompareObjectType(lhs, r4, r4, HEAP_NUMBER_TYPE, eq);
  if (strict) {
    // If lhs is not a number and rhs is a smi then strict equality cannot
    // succeed.  Return non-equal.
    // If lhs is r0 then there is already a non zero value in it.
    if (!lhs.is(r0)) {
      __ mov(r0, Immediate(NOT_EQUAL), ne);
    }
    __ Ret(ne);
  } else {
    // Smi compared non-strictly with a non-smi non-heap-number.  Call
    // the runtime.
    __ b(ne, slow);
  }

  // Rhs is a smi, lhs is a heap number.
  //TODO: floating point
  __ push(lr);
  // Load lhs to a double in r2, r3.
  __ Ldrd(r2, r3, FieldMemOperand(lhs, HeapNumber::kValueOffset));
  // Convert rhs to a double in r0, r1.
  __ mov(r7, rhs);
  ConvertToDoubleStub stub2(r1, r0, r7, r6);
  __ Call(stub2.GetCode(), RelocInfo::CODE_TARGET);
  __ pop(lr);
  // Fall through to both_loaded_as_doubles.
}


void EmitNanCheck(MacroAssembler* masm, Label* lhs_not_nan, Condition cond) {
  bool exp_first = (HeapNumber::kExponentOffset == HeapNumber::kValueOffset);
  Register rhs_exponent = exp_first ? r0 : r1;
  Register lhs_exponent = exp_first ? r2 : r3;
  Register rhs_mantissa = exp_first ? r1 : r0;
  Register lhs_mantissa = exp_first ? r3 : r2;
  Label one_is_nan, neither_is_nan;

  __ Sbfx(r4,
          lhs_exponent,
          HeapNumber::kExponentShift,
          HeapNumber::kExponentBits);
  // NaNs have all-one exponents so they sign extend to -1.
  __ cmp(r4, Immediate(-1));
  __ b(ne, lhs_not_nan);
  __ lsl(r4, lhs_exponent, Immediate(HeapNumber::kNonMantissaBitsInTopWord));
  __ cmpeq(r4, Immediate(0));
  __ b(ne, &one_is_nan);
  __ cmp(lhs_mantissa, Immediate(0));
  __ b(ne, &one_is_nan);

  __ bind(lhs_not_nan);
  __ Sbfx(r4,
          rhs_exponent,
          HeapNumber::kExponentShift,
          HeapNumber::kExponentBits);
  // NaNs have all-one exponents so they sign extend to -1.
  __ cmp(r4, Immediate(-1));
  __ b(ne, &neither_is_nan);
  __ lsl(r4, rhs_exponent, Immediate(HeapNumber::kNonMantissaBitsInTopWord));
  __ cmpeq(r4, Immediate(0));
  __ b(ne, &one_is_nan);
  __ cmp(rhs_mantissa, Immediate(0));
  __ b(eq, &neither_is_nan);

  __ bind(&one_is_nan);
  // NaN comparisons always fail.
  // Load whatever we need in r0 to make the comparison fail.
  if (cond == lt || cond == le) {
    __ mov(r0, Operand(GREATER));
  } else {
    __ mov(r0, Operand(LESS));
  }
  __ Ret();

  __ bind(&neither_is_nan);
}


// See comment at call site.
static void EmitTwoNonNanDoubleComparison(MacroAssembler* masm,
                                          Condition cond) {
  bool exp_first = (HeapNumber::kExponentOffset == HeapNumber::kValueOffset);
  Register rhs_exponent = exp_first ? r0 : r1;
  Register lhs_exponent = exp_first ? r2 : r3;
  Register rhs_mantissa = exp_first ? r1 : r0;
  Register lhs_mantissa = exp_first ? r3 : r2;

  // r0, r1, r2, r3 have the two doubles.  Neither is a NaN.
  if (cond == eq) {
    // Doubles are not equal unless they have the same bit pattern.
    // Exception: 0 and -0.
    __ cmp(rhs_mantissa, lhs_mantissa);
    __ lor(r0, rhs_mantissa, lhs_mantissa, ne);
    // Return non-zero if the numbers are unequal.
    __ Ret(ne);

    __ sub(r0, rhs_exponent, lhs_exponent);
    __ tst(r0, r0);
    // If exponents are equal then return 0.
    __ Ret(eq);

    // Exponents are unequal.  The only way we can return that the numbers
    // are equal is if one is -0 and the other is 0.  We already dealt
    // with the case where both are -0 or both are 0.
    // We start by seeing if the mantissas (that are equal) or the bottom
    // 31 bits of the rhs exponent are non-zero.  If so we return not
    // equal.
    __ lsl(r4, lhs_exponent, Immediate(kSmiTagSize));
    __ orr(r4, lhs_mantissa, r4);
    __ tst(r4, r4);
    __ mov(r0, r4, ne);
    __ Ret(ne);
    // Now they are equal if and only if the lhs exponent is zero in its
    // low 31 bits.
    __ lsl(r0, rhs_exponent, Immediate(kSmiTagSize));
    __ Ret();
  } else {
    // Call a native function to do a comparison between two non-NaNs.
    // Call C routine that may not cause GC or other trouble.
    __ push(lr);
    __ PrepareCallCFunction(4, r5);  // Two doubles count as 4 arguments.
    __ CallCFunction(ExternalReference::compare_doubles(masm->isolate()), 4);
    __ pop(lr);
    __ Ret();
  }
}


// See comment at call site.
static void EmitStrictTwoHeapObjectCompare(MacroAssembler* masm,
                                           Register lhs,
                                           Register rhs) {
    ASSERT((lhs.is(r0) && rhs.is(r1)) ||
           (lhs.is(r1) && rhs.is(r0)));

    // If either operand is a JSObject or an oddball value, then they are
    // not equal since their pointers are different.
    // There is no test for undetectability in strict equality.
    STATIC_ASSERT(LAST_TYPE == JS_FUNCTION_TYPE);
    Label first_non_object;
    // Get the type of the first operand into r2 and compare it with
    // FIRST_JS_OBJECT_TYPE.
    __ CompareObjectType(rhs, r2, r2, FIRST_JS_OBJECT_TYPE, ge);
    __ b(f, &first_non_object);

    // Return non-zero (r0 is not zero)
    Label return_not_equal;
    __ bind(&return_not_equal);
    __ Ret();

    __ bind(&first_non_object);
    // Check for oddballs: true, false, null, undefined.
    __ cmp(r2, Immediate(ODDBALL_TYPE));
    __ b(eq, &return_not_equal);

    __ CompareObjectType(lhs, r3, r3, FIRST_JS_OBJECT_TYPE, ge);
    __ b(t, &return_not_equal);

    // Check for oddballs: true, false, null, undefined.
    __ cmp(r3, Immediate(ODDBALL_TYPE));
    __ b(eq, &return_not_equal);

    // Now that we have the types we might as well check for symbol-symbol.
    // Ensure that no non-strings have the symbol bit set.
    STATIC_ASSERT(LAST_TYPE < kNotStringTag + kIsSymbolMask);
    STATIC_ASSERT(kSymbolTag != 0);
    __ land(r2, r2, r3);
    __ tst(r2, Immediate(kIsSymbolMask));
    __ b(ne, &return_not_equal);
}


// See comment at call site.
static void EmitCheckForTwoHeapNumbers(MacroAssembler* masm,
                                       Register lhs,
                                       Register rhs,
                                       Label* both_loaded_as_doubles,
                                       Label* not_heap_numbers,
                                       Label* slow) {
  ASSERT((lhs.is(r0) && rhs.is(r1)) ||
         (lhs.is(r1) && rhs.is(r0)));

  __ CompareObjectType(rhs, r3, r2, HEAP_NUMBER_TYPE, eq);
  __ b(ne, not_heap_numbers);
  __ ldr(r2, FieldMemOperand(lhs, HeapObject::kMapOffset));
  __ cmp(r2, r3);
  __ b(ne, slow);  // First was a heap number, second wasn't.  Go slow case.

  // Both are heap numbers.  Load them up then jump to the code we have
  // for that.
  //TODO: floating point
  __ Ldrd(r2, r3, FieldMemOperand(lhs, HeapNumber::kValueOffset));
  __ Ldrd(r0, r1, FieldMemOperand(rhs, HeapNumber::kValueOffset));
  __ jmp(both_loaded_as_doubles);
}


// Fast negative check for symbol-to-symbol equality.
static void EmitCheckForSymbolsOrObjects(MacroAssembler* masm,
                                         Register lhs,
                                         Register rhs,
                                         Label* possible_strings,
                                         Label* not_both_strings) {
  ASSERT((lhs.is(r0) && rhs.is(r1)) ||
         (lhs.is(r1) && rhs.is(r0)));

  // r2 is object type of rhs.
  // Ensure that no non-strings have the symbol bit set.
  Label object_test;
  STATIC_ASSERT(kSymbolTag != 0);
  __ tst(r2, Immediate(kIsNotStringMask));
  __ b(ne, &object_test);
  __ tst(r2, Immediate(kIsSymbolMask));
  __ b(eq, possible_strings);
  __ CompareObjectType(lhs, r3, r3, FIRST_NONSTRING_TYPE, ge);
  __ b(not_both_strings);
  __ tst(r3, Immediate(kIsSymbolMask));
  __ b(eq, possible_strings);

  // Both are symbols.  We already checked they weren't the same pointer
  // so they are not equal.
  __ mov(r0, Immediate(NOT_EQUAL));
  __ Ret();

  __ bind(&object_test);
  __ cmpge(r2, Immediate(FIRST_JS_OBJECT_TYPE));
  __ bf(not_both_strings);
  __ CompareObjectType(lhs, r2, r3, FIRST_JS_OBJECT_TYPE, ge);
  __ bf(not_both_strings);
  // If both objects are undetectable, they are equal. Otherwise, they
  // are not equal, since they are different objects and an object is not
  // equal to undefined.
  __ ldr(r3, FieldMemOperand(rhs, HeapObject::kMapOffset));
  __ ldrb(r2, FieldMemOperand(r2, Map::kBitFieldOffset));
  __ ldrb(r3, FieldMemOperand(r3, Map::kBitFieldOffset));
  __ land(r0, r2, r3);
  __ land(r0, r0, Immediate(1 << Map::kIsUndetectable));
  __ eor(r0, r0, Immediate(1 << Map::kIsUndetectable));
  __ Ret();
}


void NumberToStringStub::GenerateLookupNumberStringCache(MacroAssembler* masm,
                                                         Register object,
                                                         Register result,
                                                         Register scratch1,
                                                         Register scratch2,
                                                         Register scratch3,
                                                         bool object_is_smi,
                                                         Label* not_found) {
  // Use of registers. Register result is used as a temporary.
  Register number_string_cache = result;
  Register mask = scratch3;

  // Load the number string cache.
  __ LoadRoot(number_string_cache, Heap::kNumberStringCacheRootIndex);

  // Make the hash mask from the length of the number string cache. It
  // contains two elements (number and string) for each cache entry.
  __ ldr(mask, FieldMemOperand(number_string_cache, FixedArray::kLengthOffset));
  // Divide length by two (length is a smi).
  __ asr(mask, mask, Immediate(kSmiTagSize + 1));
  __ sub(mask, mask, Immediate(1));  // Make mask.

  // Calculate the entry in the number string cache. The hash value in the
  // number string cache for smis is just the smi value, and the hash for
  // doubles is the xor of the upper and lower words. See
  // Heap::GetNumberStringCache.
  Isolate* isolate = masm->isolate();
  Label is_smi;
  Label load_result_from_cache;
  if (!object_is_smi) {
    __ JumpIfSmi(object, &is_smi);
    __ b(not_found);
  }

  __ bind(&is_smi);
  Register scratch = scratch1;
  __ asr(scratch, object, Immediate(1));
  __ land(scratch, mask, scratch);
  // Calculate address of entry in string cache: each entry consists
  // of two pointer sized fields.
  __ lsl(scratch, scratch, Immediate(kPointerSizeLog2 + 1));
  __ add(scratch, number_string_cache, scratch);

  // Check if the entry is the smi we are looking for.
  Register probe = mask;
  __ ldr(probe, FieldMemOperand(scratch, FixedArray::kHeaderSize));
  __ cmp(object, probe);
  __ b(ne, not_found);

  // Get the result from the cache.
  __ bind(&load_result_from_cache);
  __ ldr(result,
         FieldMemOperand(scratch, FixedArray::kHeaderSize + kPointerSize));
  __ IncrementCounter(isolate->counters()->number_to_string_native(),
                      1,
                      scratch1,
                      scratch2);
}


void NumberToStringStub::Generate(MacroAssembler* masm) {
  // Entry argument: on stack
  // Exit in: r0

  Label runtime;

  __ ldr(r1, MemOperand(sp, 0));

  // Generate code to lookup number in the number string cache.
  GenerateLookupNumberStringCache(masm, r1, r0, r2, r3, r4, false, &runtime);
  __ add(sp, sp, Immediate(1 * kPointerSize));
  __ Ret();

  __ bind(&runtime);
  // Handle number to string in the runtime system if not found in the cache.
  __ TailCallRuntime(Runtime::kNumberToStringSkipCache, 1, 1);
}


// On entry lhs_ and rhs_ are the values to be compared.
// On exit r0 is 0, positive or negative to indicate the result of
// the comparison.
void CompareStub::Generate(MacroAssembler* masm) {
  ASSERT((lhs_.is(r0) && rhs_.is(r1)) ||
         (lhs_.is(r1) && rhs_.is(r0)));

  Label slow;  // Call builtin.
  Label not_smis, both_loaded_as_doubles, lhs_not_nan;

  if (include_smi_compare_) {
    Label not_two_smis, smi_done;
    __ orr(r2, r1, r0);
    __ tst(r2, Immediate(kSmiTagMask));
    __ b(ne, &not_two_smis);
    __ asr(r1, r1, Immediate(1));
    __ asr(r0, r0, Immediate(1));
    __ sub(r0, r1, r0);
    __ Ret();
    __ bind(&not_two_smis);
  } else if (FLAG_debug_code) {
    __ orr(r2, r1, r0);
    __ tst(r2, Immediate(kSmiTagMask));
    __ Assert(ne, "CompareStub: unexpected smi operands.");
  }

  // NOTICE! This code is only reached after a smi-fast-case check, so
  // it is certain that at least one operand isn't a smi.

  // Handle the case where the objects are identical.  Either returns the answer
  // or goes to slow.  Only falls through if the objects were not identical.
  EmitIdenticalObjectComparison(masm, &slow, cc_, never_nan_nan_);

  // If either is a Smi (we know that not both are), then they can only
  // be strictly equal if the other is a HeapNumber.
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT_EQ(0, Smi::FromInt(0));
  __ land(r2, lhs_, rhs_);
  __ tst(r2, Immediate(kSmiTagMask));
  __ b(ne, &not_smis);
  // One operand is a smi.  EmitSmiNonsmiComparison generates code that can:
  // 1) Return the answer.
  // 2) Go to slow.
  // 3) Fall through to both_loaded_as_doubles.
  // 4) Jump to lhs_not_nan.
  // In cases 3 and 4 we have found out we were dealing with a number-number
  // comparison.  If VFP3 is supported the double values of the numbers have
  // been loaded into d7 and d6.  Otherwise, the double values have been loaded
  // into r0, r1, r2, and r3.
  EmitSmiNonsmiComparison(masm, lhs_, rhs_, &lhs_not_nan, &slow, strict_);

  __ bind(&both_loaded_as_doubles);
  // The arguments have been converted to doubles and stored in d6 and d7, if
  // VFP3 is supported, or in r0, r1, r2, and r3.
  //TODO: Floating point
  Isolate* isolate = masm->isolate();
  // Checks for NaN in the doubles we have loaded.  Can return the answer or
  // fall through if neither is a NaN.  Also binds lhs_not_nan.
  EmitNanCheck(masm, &lhs_not_nan, cc_);
  // Compares two doubles in r0, r1, r2, r3 that are not NaNs.  Returns the
  // answer.  Never falls through.
  EmitTwoNonNanDoubleComparison(masm, cc_);

  __ bind(&not_smis);
  // At this point we know we are dealing with two different objects,
  // and neither of them is a Smi.  The objects are in rhs_ and lhs_.
  if (strict_) {
    // This returns non-equal for some object types, or falls through if it
    // was not lucky.
    EmitStrictTwoHeapObjectCompare(masm, lhs_, rhs_);
  }

  Label check_for_symbols;
  Label flat_string_check;
  // Check for heap-number-heap-number comparison.  Can jump to slow case,
  // or load both doubles into r0, r1, r2, r3 and jump to the code that handles
  // that case.  If the inputs are not doubles then jumps to check_for_symbols.
  // In this case r2 will contain the type of rhs_.  Never falls through.
  EmitCheckForTwoHeapNumbers(masm,
                             lhs_,
                             rhs_,
                             &both_loaded_as_doubles,
                             &check_for_symbols,
                             &flat_string_check);

  __ bind(&check_for_symbols);
  // In the strict case the EmitStrictTwoHeapObjectCompare already took care of
  // symbols.
  if (cc_ == eq && !strict_) {
    // Returns an answer for two symbols or two detectable objects.
    // Otherwise jumps to string case or not both strings case.
    // Assumes that r2 is the type of rhs_ on entry.
    EmitCheckForSymbolsOrObjects(masm, lhs_, rhs_, &flat_string_check, &slow);
  }

  // Check for both being sequential ASCII strings, and inline if that is the
  // case.
  __ bind(&flat_string_check);

  __ JumpIfNonSmisNotBothSequentialAsciiStrings(lhs_, rhs_, r2, r3, &slow);

  __ IncrementCounter(isolate->counters()->string_compare_native(), 1, r2, r3);
  StringCompareStub::GenerateCompareFlatAsciiStrings(masm,
                                                     lhs_,
                                                     rhs_,
                                                     r2,
                                                     r3,
                                                     r4,
                                                     r5);
  // Never falls through to here.

  __ bind(&slow);

  __ Push(lhs_, rhs_);
  // Figure out which native to call and setup the arguments.
  Builtins::JavaScript native;
  if (cc_ == eq) {
    native = strict_ ? Builtins::STRICT_EQUALS : Builtins::EQUALS;
  } else {
    native = Builtins::COMPARE;
    int ncr;  // NaN compare result
    if (cc_ == lt || cc_ == le) {
      ncr = GREATER;
    } else {
      ASSERT(cc_ == gt || cc_ == ge);  // remaining cases
      ncr = LESS;
    }
    __ mov(r0, Immediate(Smi::FromInt(ncr)));
    __ push(r0);
  }

  // Call the native; it returns -1 (less), 0 (equal), or 1 (greater)
  // tagged as a small integer.
  __ InvokeBuiltin(native, JUMP_JS);
}


Handle<Code> GetTypeRecordingBinaryOpStub(int key,
    TRBinaryOpIC::TypeInfo type_info,
    TRBinaryOpIC::TypeInfo result_type_info) {
  TypeRecordingBinaryOpStub stub(key, type_info, result_type_info);
  return stub.GetCode();
}


void TypeRecordingBinaryOpStub::GenerateTypeTransition(MacroAssembler* masm) {
  Label get_result;

  __ Push(r1, r0);

  __ mov(r2, Immediate(Smi::FromInt(MinorKey())));
  __ mov(r1, Immediate(Smi::FromInt(op_)));
  __ mov(r0, Immediate(Smi::FromInt(operands_type_)));
  __ Push(r2, r1, r0);

  __ TailCallExternalReference(
      ExternalReference(IC_Utility(IC::kTypeRecordingBinaryOp_Patch),
                        masm->isolate()),
      5,
      1);
}


void TypeRecordingBinaryOpStub::GenerateTypeTransitionWithSavedArgs(
    MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void TypeRecordingBinaryOpStub::Generate(MacroAssembler* masm) {
  switch (operands_type_) {
    case TRBinaryOpIC::UNINITIALIZED:
      GenerateTypeTransition(masm);
      break;
    case TRBinaryOpIC::SMI:
      GenerateSmiStub(masm);
      break;
    case TRBinaryOpIC::INT32:
      GenerateInt32Stub(masm);
      break;
    case TRBinaryOpIC::HEAP_NUMBER:
      GenerateHeapNumberStub(masm);
      break;
    case TRBinaryOpIC::ODDBALL:
      GenerateOddballStub(masm);
      break;
    case TRBinaryOpIC::BOTH_STRING:
      GenerateBothStringStub(masm);
      break;
    case TRBinaryOpIC::STRING:
      GenerateStringStub(masm);
      break;
    case TRBinaryOpIC::GENERIC:
      GenerateGeneric(masm);
      break;
    default:
      UNREACHABLE();
  }
}


const char* TypeRecordingBinaryOpStub::GetName() {
  if (name_ != NULL) return name_;
  const int kMaxNameLength = 100;
  name_ = Isolate::Current()->bootstrapper()->AllocateAutoDeletedArray(
      kMaxNameLength);
  if (name_ == NULL) return "OOM";
  const char* op_name = Token::Name(op_);
  const char* overwrite_name;
  switch (mode_) {
    case NO_OVERWRITE: overwrite_name = "Alloc"; break;
    case OVERWRITE_RIGHT: overwrite_name = "OverwriteRight"; break;
    case OVERWRITE_LEFT: overwrite_name = "OverwriteLeft"; break;
    default: overwrite_name = "UnknownOverwrite"; break;
  }

  OS::SNPrintF(Vector<char>(name_, kMaxNameLength),
               "TypeRecordingBinaryOpStub_%s_%s_%s",
               op_name,
               overwrite_name,
               TRBinaryOpIC::GetName(operands_type_));
  return name_;
}


void TypeRecordingBinaryOpStub::GenerateSmiSmiOperation(
    MacroAssembler* masm) {
  // Input register: r0, r1
  Register left = r1;
  Register right = r0;
  Register scratch1 = r7;
  Register scratch2 = r9;

  ASSERT(right.is(r0));
  STATIC_ASSERT(kSmiTag == 0);

  Label not_smi_result;
  switch (op_) {
    case Token::ADD:
      __ addv(right, left, right);	// Add optimistically.
      __ Ret(vc);			// Return if no overflow
      __ sub(right, right, left);  	// Revert optimistic add.
      break;
    case Token::SUB:
      __ subv(right, left, right);	// Subtract optimistically.
      __ Ret(vc);			// Return if no overflow
      __ sub(right, left, right);	// Revert optimistic subtract.
      break;
    case Token::MUL:
      // TODO: implement optimized multiply with overflow check for SH4
      // // Remove tag from one of the operands. This way the multiplication result
      // // will be a smi if it fits the smi range.
      // __ SmiUntag(ip, right);
      // // Do multiplication
      // // scratch1 = lower 32 bits of ip * left.
      // // scratch2 = higher 32 bits of ip * left.
      // __ smull(scratch1, scratch2, left, ip);
      // // Check for overflowing the smi range - no overflow if higher 33 bits of
      // // the result are identical.
      // __ mov(ip, Operand(scratch1, ASR, 31));
      // __ cmp(ip, Operand(scratch2));
      // __ b(ne, &not_smi_result);
      // // Go slow on zero result to handle -0.
      // __ tst(scratch1, Operand(scratch1));
      // __ mov(right, Operand(scratch1), LeaveCC, ne);
      // __ Ret(ne);
      // // We need -0 if we were multiplying a negative number with 0 to get 0.
      // // We know one of them was zero.
      // __ add(scratch2, right, Operand(left), SetCC);
      // __ mov(right, Operand(Smi::FromInt(0)), LeaveCC, pl);
      // __ Ret(pl);  // Return smi 0 if the non-zero one was positive.
      // We fall through here if we multiplied a negative number with 0, because
      // that would mean we should produce -0.
      __ UNIMPLEMENTED_BREAK();
      break;
    case Token::DIV:
      // Check for power of two on the right hand side.
      __ JumpIfNotPowerOfTwoOrZero(right, scratch1, &not_smi_result);
      // Check for positive and no remainder (scratch1 contains right - 1).
      __ orr(scratch2, scratch1, Immediate(0x80000000u));
      __ tst(left, scratch2);
      __ b(ne, &not_smi_result);

      // Perform division by shifting.
      __ CountLeadingZeros(scratch1, scratch1, scratch2);
      __ rsb(scratch1, scratch1, Immediate(31));
      __ lsr(right, left, scratch1);
      __ Ret();
      break;
    case Token::MOD:
      // Check for two positive smis.
      __ orr(scratch1, left, right);
      __ tst(scratch1, Immediate(0x80000000u | kSmiTagMask));
      __ b(ne, &not_smi_result);

      // Check for power of two on the right hand side.
      __ JumpIfNotPowerOfTwoOrZero(right, scratch1, &not_smi_result);

      // Perform modulus by masking.
      __ land(right, left, scratch1);
      __ Ret();
      break;
    case Token::BIT_OR:
      __ orr(right, left, right);
      __ Ret();
      break;
    case Token::BIT_AND:
      __ land(right, left, right);
      __ Ret();
      break;
    case Token::BIT_XOR:
      __ eor(right, left, right);
      __ Ret();
      break;
    case Token::SAR:
      // Remove tags from right operand.
      __ GetLeastBitsFromSmi(scratch1, right, 5);
      __ asr(right, left, scratch1);
      // Smi tag result.
      __ bic(right, right, Immediate(kSmiTagMask));
      __ Ret();
      break;
    case Token::SHR:
      // Remove tags from operands. We can't do this on a 31 bit number
      // because then the 0s get shifted into bit 30 instead of bit 31.
      __ SmiUntag(scratch1, left);
      __ GetLeastBitsFromSmi(scratch2, right, 5);
      __ lsr(scratch1, scratch1, scratch2);
      // Unsigned shift is not allowed to produce a negative number, so
      // check the sign bit and the sign bit after Smi tagging.
      __ tst(scratch1, Immediate(0xc0000000));
      __ b(ne, &not_smi_result);
      // Smi tag result.
      __ SmiTag(right, scratch1);
      __ Ret();
      break;
    case Token::SHL:
      // Remove tags from operands.
      __ SmiUntag(scratch1, left);
      __ GetLeastBitsFromSmi(scratch2, right, 5);
      __ lsl(scratch1, scratch1, scratch2);
      // Check that the signed result fits in a Smi.
      __ add(scratch2, scratch1, Immediate(0x40000000));
      __ cmpge(scratch2, Immediate(0));
      __ bf(&not_smi_result);
      __ SmiTag(right, scratch1);
      __ Ret();
      break;
    default:
      UNREACHABLE();
  }
  __ bind(&not_smi_result);
}


void TypeRecordingBinaryOpStub::GenerateFPOperation(MacroAssembler* masm,
                                                    bool smi_operands,
                                                    Label* not_numbers,
                                                    Label* gc_required) {
  Register left = r1;
  Register right = r0;
  Register scratch1 = r7;
  Register scratch2 = r9;
  Register scratch3 = r4;

  ASSERT(smi_operands || (not_numbers != NULL));
  if (smi_operands && FLAG_debug_code) {
    __ AbortIfNotSmi(left);
    __ AbortIfNotSmi(right);
  }

  Register heap_number_map = r6;
  __ LoadRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);

  switch (op_) {
    case Token::ADD:
    case Token::SUB:
    case Token::MUL:
    case Token::DIV:
    case Token::MOD: {
      // Load left and right operands into d6 and d7 or r0/r1 and r2/r3
      // depending on whether VFP3 is available or not.
      FloatingPointHelper::Destination destination =
//TODO: VFP
//          CpuFeatures::IsSupported(VFP3) &&
//          op_ != Token::MOD ?
//          FloatingPointHelper::kVFPRegisters :
          FloatingPointHelper::kCoreRegisters;

      // Allocate new heap number for result.
      Register result = r5;
      GenerateHeapResultAllocation(
          masm, result, heap_number_map, scratch1, scratch2, gc_required);

      // Load the operands.
      if (smi_operands) {
        FloatingPointHelper::LoadSmis(masm, destination, scratch1, scratch2);
      } else {
        FloatingPointHelper::LoadOperands(masm,
                                          destination,
                                          heap_number_map,
                                          scratch1,
                                          scratch2,
                                          not_numbers);
      }

      // Calculate the result.
//TODO: VFP
//      if (destination == FloatingPointHelper::kVFPRegisters) {
//        // Using VFP registers:
//        // d6: Left value
//        // d7: Right value
//        CpuFeatures::Scope scope(VFP3);
//        switch (op_) {
//          case Token::ADD:
//            __ vadd(d5, d6, d7);
//            break;
//          case Token::SUB:
//            __ vsub(d5, d6, d7);
//            break;
//          case Token::MUL:
//            __ vmul(d5, d6, d7);
//            break;
//          case Token::DIV:
//            __ vdiv(d5, d6, d7);
//            break;
//          default:
//            UNREACHABLE();
//        }
//
//        __ sub(r0, result, Immediate(kHeapObjectTag));
//        __ vstr(d5, r0, HeapNumber::kValueOffset);
//        __ add(r0, r0, Immediate(kHeapObjectTag));
//        __ Ret();
//
//      } else
      {
        // Call the C function to handle the double operation.
        FloatingPointHelper::CallCCodeForDoubleOperation(masm,
                                                         op_,
                                                         result,
                                                         scratch1);
        if (FLAG_debug_code) {
          __ stop("Unreachable code.");
        }
      }
      break;
    }
    case Token::BIT_OR:
    case Token::BIT_XOR:
    case Token::BIT_AND:
    case Token::SAR:
    case Token::SHR:
    case Token::SHL: {
      if (smi_operands) {
        __ SmiUntag(r3, left);
        __ SmiUntag(r2, right);
      } else {
        UNIMPLEMENTED();
/*
        // Convert operands to 32-bit integers. Right in r2 and left in r3.
        FloatingPointHelper::ConvertNumberToInt32(masm,
                                                  left,
                                                  r3,
                                                  heap_number_map,
                                                  scratch1,
                                                  scratch2,
                                                  scratch3,
                                                  d0,
                                                  not_numbers);
        FloatingPointHelper::ConvertNumberToInt32(masm,
                                                  right,
                                                  r2,
                                                  heap_number_map,
                                                  scratch1,
                                                  scratch2,
                                                  scratch3,
                                                  d0,
                                                  not_numbers);
*/
      }

      Label result_not_a_smi;
      switch (op_) {
        case Token::BIT_OR:
          __ orr(r2, r3, r2);
          break;
        case Token::BIT_XOR:
          __ lxor(r2, r3, r2);
          break;
        case Token::BIT_AND:
          __ land(r2, r3, r2);
          break;
        case Token::SAR:
          // Use only the 5 least significant bits of the shift count.
          __ GetLeastBitsFromInt32(r2, r2, 5);
          __ asr(r2, r3, r2);
          break;
        case Token::SHR:
          // Use only the 5 least significant bits of the shift count.
          __ GetLeastBitsFromInt32(r2, r2, 5);
          __ lsr(r2, r3, r2);
          __ cmpge(r2, Immediate(0));
          // SHR is special because it is required to produce a positive answer.
          // The code below for writing into heap numbers isn't capable of
          // writing the register as an unsigned int so we go to slow case if we
          // hit this case.
//TODO: VFP
//          if (CpuFeatures::IsSupported(VFP3)) {
//            __ b(mi, &result_not_a_smi);
//          } else
          {
            __ bf(not_numbers);
          }
          break;
        case Token::SHL:
          // Use only the 5 least significant bits of the shift count.
          __ GetLeastBitsFromInt32(r2, r2, 5);
          __ lsl(r2, r3, r2);
          break;
        default:
          UNREACHABLE();
      }

      // Check that the *signed* result fits in a smi.
      __ add(r3, r2, Immediate(0x40000000));
      __ cmpge(r3, Immediate(0));
      __ bf(&result_not_a_smi);
      __ SmiTag(r0, r2);
      __ Ret();

      // Allocate new heap number for result.
      __ bind(&result_not_a_smi);
      Register result = r5;
      if (smi_operands) {
        __ AllocateHeapNumber(
            result, scratch1, scratch2, heap_number_map, gc_required);
      } else {
        UNIMPLEMENTED();
//TODO: VFP
//        GenerateHeapResultAllocation(
//            masm, result, heap_number_map, scratch1, scratch2, gc_required);
      }

      // r2: Answer as signed int32.
      // r5: Heap number to write answer into.

      // Nothing can go wrong now, so move the heap number to r0, which is the
      // result.
      __ mov(r0, r5);

//TODO: VFP
//      if (CpuFeatures::IsSupported(VFP3)) {
//        // Convert the int32 in r2 to the heap number in r0. r3 is corrupted. As
//        // mentioned above SHR needs to always produce a positive result.
//        CpuFeatures::Scope scope(VFP3);
//        __ vmov(s0, r2);
//        if (op_ == Token::SHR) {
//          __ vcvt_f64_u32(d0, s0);
//        } else {
//          __ vcvt_f64_s32(d0, s0);
//        }
//        __ sub(r3, r0, Immediate(kHeapObjectTag));
//        __ vstr(d0, r3, HeapNumber::kValueOffset);
//        __ Ret();
//      } else
      {
        // Tail call that writes the int32 in r2 to the heap number in r0, using
        // r3 as scratch. r0 is preserved and returned.
        WriteInt32ToHeapNumberStub stub(r2, r0, r3);
        __ TailCallStub(&stub);
      }
      break;
    }
    default:
      UNREACHABLE();
  }
}


// Generate the smi code. If the operation on smis are successful this return is
// generated. If the result is not a smi and heap number allocation is not
// requested the code falls through. If number allocation is requested but a
// heap number cannot be allocated the code jumps to the lable gc_required.
void TypeRecordingBinaryOpStub::GenerateSmiCode(MacroAssembler* masm,
    Label* use_runtime,
    Label* gc_required,
    SmiCodeGenerateHeapNumberResults allow_heapnumber_results) {
  Label not_smis;

  Register left = r1;
  Register right = r0;
  Register scratch1 = r7;
  Register scratch2 = r9;

  // Perform combined smi check on both operands.
  __ lor(scratch1, left, right);
  STATIC_ASSERT(kSmiTag == 0);
  __ tst(scratch1, Immediate(kSmiTagMask));
  __ b(ne, &not_smis);

  // If the smi-smi operation results in a smi return is generated.
  GenerateSmiSmiOperation(masm);

  // If heap number results are possible generate the result in an allocated
  // heap number.
  if (allow_heapnumber_results == ALLOW_HEAPNUMBER_RESULTS) {
    GenerateFPOperation(masm, true, use_runtime, gc_required);
  }
  __ bind(&not_smis);
}


void TypeRecordingBinaryOpStub::GenerateSmiStub(MacroAssembler* masm) {
  Label not_smis, call_runtime;

  if (result_type_ == TRBinaryOpIC::UNINITIALIZED ||
      result_type_ == TRBinaryOpIC::SMI) {
    // Only allow smi results.
    GenerateSmiCode(masm, &call_runtime, NULL, NO_HEAPNUMBER_RESULTS);
  } else {
    // Allow heap number result and don't make a transition if a heap number
    // cannot be allocated.
    GenerateSmiCode(masm,
                    &call_runtime,
                    &call_runtime,
                    ALLOW_HEAPNUMBER_RESULTS);
  }

  // Code falls through if the result is not returned as either a smi or heap
  // number.
  GenerateTypeTransition(masm);

  __ bind(&call_runtime);
  GenerateCallRuntime(masm);
}


void TypeRecordingBinaryOpStub::GenerateStringStub(MacroAssembler* masm) {
  ASSERT(operands_type_ == TRBinaryOpIC::STRING);
  ASSERT(op_ == Token::ADD);
  // Try to add arguments as strings, otherwise, transition to the generic
  // TRBinaryOpIC type.
  GenerateAddStrings(masm);
  GenerateTypeTransition(masm);
}


void TypeRecordingBinaryOpStub::GenerateBothStringStub(MacroAssembler* masm) {
  Label call_runtime;
  ASSERT(operands_type_ == TRBinaryOpIC::BOTH_STRING);
  ASSERT(op_ == Token::ADD);
  // If both arguments are strings, call the string add stub.
  // Otherwise, do a transition.

  // Registers containing left and right operands respectively.
  Register left = r1;
  Register right = r0;

  // Test if left operand is a string.
  __ JumpIfSmi(left, &call_runtime);
  __ CompareObjectType(left, r2, r2, FIRST_NONSTRING_TYPE, ge);
  __ bt(&call_runtime);

  // Test if right operand is a string.
  __ JumpIfSmi(right, &call_runtime);
  __ CompareObjectType(right, r2, r2, FIRST_NONSTRING_TYPE, ge);
  __ bt(&call_runtime);

  StringAddStub string_add_stub(NO_STRING_CHECK_IN_STUB);
  GenerateRegisterArgsPush(masm);
  __ TailCallStub(&string_add_stub);

  __ bind(&call_runtime);
  GenerateTypeTransition(masm);
}


void TypeRecordingBinaryOpStub::GenerateInt32Stub(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void TypeRecordingBinaryOpStub::GenerateOddballStub(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void TypeRecordingBinaryOpStub::GenerateHeapNumberStub(MacroAssembler* masm) {
  Label call_runtime;
  GenerateFPOperation(masm, false, &call_runtime, &call_runtime);

  __ bind(&call_runtime);
  GenerateCallRuntime(masm);
}


void TypeRecordingBinaryOpStub::GenerateGeneric(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void TypeRecordingBinaryOpStub::GenerateAddStrings(MacroAssembler* masm) {
  ASSERT(op_ == Token::ADD);
  Label left_not_string, call_runtime;

  Register left = r1;
  Register right = r0;

  // Check if left argument is a string.
  __ JumpIfSmi(left, &left_not_string);
  __ CompareObjectType(left, r2, r2, FIRST_NONSTRING_TYPE, ge);
  __ bt(&left_not_string);

  StringAddStub string_add_left_stub(NO_STRING_CHECK_LEFT_IN_STUB);
  GenerateRegisterArgsPush(masm);
  __ TailCallStub(&string_add_left_stub);

  // Left operand is not a string, test right.
  __ bind(&left_not_string);
  __ JumpIfSmi(right, &call_runtime);
  __ CompareObjectType(right, r2, r2, FIRST_NONSTRING_TYPE, ge);
  __ bt(&call_runtime);

  StringAddStub string_add_right_stub(NO_STRING_CHECK_RIGHT_IN_STUB);
  GenerateRegisterArgsPush(masm);
  __ TailCallStub(&string_add_right_stub);

  // At least one argument is not a string.
  __ bind(&call_runtime);
}


void TypeRecordingBinaryOpStub::GenerateCallRuntime(MacroAssembler* masm) {
  GenerateRegisterArgsPush(masm);
  switch (op_) {
    case Token::ADD:
      __ InvokeBuiltin(Builtins::ADD, JUMP_JS);
      break;
    case Token::SUB:
      __ InvokeBuiltin(Builtins::SUB, JUMP_JS);
      break;
    case Token::MUL:
      __ InvokeBuiltin(Builtins::MUL, JUMP_JS);
      break;
    case Token::DIV:
      __ InvokeBuiltin(Builtins::DIV, JUMP_JS);
      break;
    case Token::MOD:
      __ InvokeBuiltin(Builtins::MOD, JUMP_JS);
      break;
    case Token::BIT_OR:
      __ InvokeBuiltin(Builtins::BIT_OR, JUMP_JS);
      break;
    case Token::BIT_AND:
      __ InvokeBuiltin(Builtins::BIT_AND, JUMP_JS);
      break;
    case Token::BIT_XOR:
      __ InvokeBuiltin(Builtins::BIT_XOR, JUMP_JS);
      break;
    case Token::SAR:
      __ InvokeBuiltin(Builtins::SAR, JUMP_JS);
      break;
    case Token::SHR:
      __ InvokeBuiltin(Builtins::SHR, JUMP_JS);
      break;
    case Token::SHL:
      __ InvokeBuiltin(Builtins::SHL, JUMP_JS);
      break;
    default:
      UNREACHABLE();
  }
}


void TypeRecordingBinaryOpStub::GenerateHeapResultAllocation(
    MacroAssembler* masm,
    Register result,
    Register heap_number_map,
    Register scratch1,
    Register scratch2,
    Label* gc_required) {

  // Code below will scratch result if allocation fails. To keep both arguments
  // intact for the runtime call result cannot be one of these.
  ASSERT(!result.is(r0) && !result.is(r1));

  if (mode_ == OVERWRITE_LEFT || mode_ == OVERWRITE_RIGHT) {
    Label skip_allocation, allocated;
    Register overwritable_operand = mode_ == OVERWRITE_LEFT ? r1 : r0;
    // If the overwritable operand is already an object, we skip the
    // allocation of a heap number.
    __ JumpIfNotSmi(overwritable_operand, &skip_allocation);
    // Allocate a heap number for the result.
    __ AllocateHeapNumber(
        result, scratch1, scratch2, heap_number_map, gc_required);
    __ b(&allocated);
    __ bind(&skip_allocation);
    // Use object holding the overwritable operand for result.
    __ mov(result, overwritable_operand);
    __ bind(&allocated);
  } else {
    ASSERT(mode_ == NO_OVERWRITE);
    __ AllocateHeapNumber(
        result, scratch1, scratch2, heap_number_map, gc_required);
  }
}


void TypeRecordingBinaryOpStub::GenerateRegisterArgsPush(MacroAssembler* masm) {
  __ Push(r1, r0);
}


void StackCheckStub::Generate(MacroAssembler* masm) {
  __ TailCallRuntime(Runtime::kStackGuard, 0, 1);
}


void GenericUnaryOpStub::Generate(MacroAssembler* masm) {
  Label slow, done;

  // Entry value in r0
  // Exit value in r0

  Register heap_number_map = r6;
  __ LoadRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);

  if (op_ == Token::SUB) {
    if (include_smi_code_) {
      // Check whether the value is a smi.
      Label try_float;
      __ tst(r0, Immediate(kSmiTagMask));
      __ b(ne, &try_float);

      // Go slow case if the value of the expression is zero
      // to make sure that we switch between 0 and -0.
      if (negative_zero_ == kStrictNegativeZero) {
        // If we have to check for zero, then we can check for the max negative
        // smi while we are at it.
        __ bic(ip, r0, Immediate(0x80000000));
	__ tst(ip, ip);
        __ b(eq, &slow);
        __ rsb(r0, r0, Immediate(0));
        __ Ret();
      } else {
        // The value of the expression is a smi and 0 is OK for -0.  Try
        // optimistic subtraction '0 - value'.
	__ mov(ip, Immediate(0));
        __ subv(r0, ip, r0);
        __ Ret(vc);
        // We don't have to reverse the optimistic neg since the only case
        // where we fall through is the minimum negative Smi, which is the case
        // where the neg leaves the register unchanged.
        __ jmp(&slow);  // Go slow on max negative Smi.
      }
      __ bind(&try_float);
    } else if (FLAG_debug_code) {
      __ tst(r0, Immediate(kSmiTagMask));
      __ Assert(ne, "Unexpected smi operand.");
    }

    __ ldr(r1, FieldMemOperand(r0, HeapObject::kMapOffset));
    __ AssertRegisterIsRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);
    __ cmp(r1, heap_number_map);
    __ b(ne, &slow);
    // r0 is a heap number.  Get a new heap number in r1.
    if (overwrite_ == UNARY_OVERWRITE) {
      __ ldr(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
      __ eor(r2, r2, Immediate(HeapNumber::kSignMask));  // Flip sign.
      __ str(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
    } else {
      __ AllocateHeapNumber(r1, r2, r3, r6, &slow);
      __ ldr(r3, FieldMemOperand(r0, HeapNumber::kMantissaOffset));
      __ ldr(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
      __ str(r3, FieldMemOperand(r1, HeapNumber::kMantissaOffset));
      __ eor(r2, r2, Immediate(HeapNumber::kSignMask));  // Flip sign.
      __ str(r2, FieldMemOperand(r1, HeapNumber::kExponentOffset));
      __ mov(r0, r1);
    }
  } else if (op_ == Token::BIT_NOT) {
    if (include_smi_code_) {
      Label non_smi;
      __ JumpIfNotSmi(r0, &non_smi);
      __ mvn(r0, r0);
      // Bit-clear inverted smi-tag.
      __ bic(r0, r0, Immediate(kSmiTagMask));
      __ Ret();
      __ bind(&non_smi);
    } else if (FLAG_debug_code) {
      __ tst(r0, Immediate(kSmiTagMask));
      __ Assert(ne, "Unexpected smi operand.");
    }

    // Check if the operand is a heap number.
    __ ldr(r1, FieldMemOperand(r0, HeapObject::kMapOffset));
    __ AssertRegisterIsRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);
    __ cmp(r1, heap_number_map);
    __ b(ne, &slow);

    // Convert the heap number in r0 to an untagged integer in r1.
    __ ConvertToInt32(r0, r1, r2, r3, no_reg, &slow);

    // Do the bitwise operation (move negated) and check if the result
    // fits in a smi.
    Label try_float;
    __ mvn(r1, r1);
    __ add(r2, r1, Immediate(0x40000000));
    __ cmpge(r2, Immediate(0));
    __ bf(&try_float);
    __ lsl(r0, r1, Immediate(kSmiTagSize));
    __ b(&done);

    __ bind(&try_float);
    if (!overwrite_ == UNARY_OVERWRITE) {
      // Allocate a fresh heap number, but don't overwrite r0 until
      // we're sure we can do it without going through the slow case
      // that needs the value in r0.
      __ AllocateHeapNumber(r2, r3, r4, r6, &slow);
      __ mov(r0, r2);
    }

    // TODO: check fast version with float for SH4
    // if (CpuFeatures::IsSupported(VFP3)) {
    //   // Convert the int32 in r1 to the heap number in r0. r2 is corrupted.
    //   CpuFeatures::Scope scope(VFP3);
    //   __ vmov(s0, r1);
    //   __ vcvt_f64_s32(d0, s0);
    //   __ sub(r2, r0, Operand(kHeapObjectTag));
    //   __ vstr(d0, r2, HeapNumber::kValueOffset);
    // } else 
    {
      // WriteInt32ToHeapNumberStub does not trigger GC, so we do not
      // have to set up a frame.
      WriteInt32ToHeapNumberStub stub(r1, r0, r2);
      __ push(lr);
      __ Call(stub.GetCode(), RelocInfo::CODE_TARGET);
      __ pop(lr);
    }
  } else {
    UNIMPLEMENTED();
  }

  __ bind(&done);
  __ Ret();

  // Handle the slow case by jumping to the JavaScript builtin.
  __ bind(&slow);
  __ push(r0);
  switch (op_) {
    case Token::SUB:
      __ InvokeBuiltin(Builtins::UNARY_MINUS, JUMP_JS);
      break;
    case Token::BIT_NOT:
      __ InvokeBuiltin(Builtins::BIT_NOT, JUMP_JS);
      break;
    default:
      UNREACHABLE();
  }
}


bool CEntryStub::NeedsImmovableCode() {
  return true;
}


void CEntryStub::GenerateCore(MacroAssembler* masm,
                              Label* throw_normal_exception,
                              Label* throw_termination_exception,
                              Label* throw_out_of_memory_exception,
                              bool do_gc,
                              bool always_allocate) {
  // WARNING: this function use the SH4 ABI !!

  // r0: result parameter for PerformGC, if any
  // sh4_r8: number of arguments including receiver  (C callee-saved)
  // sh4_r9: pointer to builtin function  (C callee-saved)
  // sh4_r10: pointer to the first argument (C callee-saved)
  Isolate* isolate = masm->isolate();
  ASSERT(!r0.is(sh4_rtmp) && !sh4_r8.is(sh4_rtmp) && !sh4_r9.is(sh4_rtmp) && !sh4_r10.is(sh4_rtmp));

  if (do_gc) {
    // Passing r0.
    __ PrepareCallCFunction(1, r1);
    __ CallCFunction(ExternalReference::perform_gc_function(isolate), 1);
  }

  ExternalReference scope_depth =
      ExternalReference::heap_always_allocate_scope_depth(isolate);
  if (always_allocate) {
    __ mov(r0, Operand(scope_depth));
    __ ldr(r1, MemOperand(r0));
    __ add(r1, r1, Immediate(1));
    __ str(r1, MemOperand(r0));
  }

  // Call C built-in.
  // r4 = argc, r5 = argv, r6 = isolate
  __ mov(r4, sh4_r8);
  __ mov(r5, sh4_r10);
  __ mov(r6, Operand(ExternalReference::isolate_address()));

  // TODO(1242173): To let the GC traverse the return address of the exit
  // frames, we need to know where the return address is. Right now,
  // we store it on the stack to be able to find it again, but we never
  // restore from it in case of changes, which makes it impossible to
  // support moving the C entry code stub. This should be fixed, but currently
  // this is OK because the CEntryStub gets generated so early in the V8 boot
  // sequence that it is not moving ever.

  // Compute the return address in pr to return to after the jsr below.
  // We use the addpc operation for this with an offset of 6.
  // We add 3 * kInstrSize to the pc after the addpc for the size of
  // the sequence: [str, jsr, nop(delay slot)].
  __ addpc(r3, 3 * Assembler::kInstrSize);
#ifdef DEBUG
  int old_pc = masm->pc_offset();
#endif
  __ str(r3, MemOperand(sp, 0));
  __ jsr(sh4_r9);
#ifdef DEBUG
  ASSERT(masm->pc_offset() - old_pc == 3 * Assembler::kInstrSize);
#endif

  if (always_allocate) {
    // It's okay to clobber r2 and r3 here. Don't mess with r0 and r1
    // though (contain the result).
    __ mov(r2, Operand(scope_depth));
    __ ldr(r3, MemOperand(r2));
    __ sub(r3, r3, Immediate(1));
    __ str(r3, MemOperand(r2));
  }

  // check for failure result
  Label failure_returned;
  STATIC_ASSERT(((kFailureTag + 1) & kFailureTagMask) == 0);
  // Lower 2 bits of r2 are 0 if r0 has failure tag.
  __ add(r2, r0, Immediate(1));
  __ tst(r2, Immediate(kFailureTagMask));
  __ b(eq, &failure_returned);

  // Exit C frame and return.
  // r0:r1: result
  // sp: stack pointer
  // fp: frame pointer
  //  Callee-saved register sh4_r8 still holds argc.
  __ LeaveExitFrame(save_doubles_, sh4_r8);
  __ rts();

  // check if we should retry or throw exception
  Label retry;
  __ bind(&failure_returned);
  STATIC_ASSERT(Failure::RETRY_AFTER_GC == 0);
  __ tst(r0, Immediate(((1 << kFailureTypeTagSize) - 1) << kFailureTagSize));
  __ b(eq, &retry);

  // Special handling of out of memory exceptions.
  Failure* out_of_memory = Failure::OutOfMemoryException();
  __ cmp(r0, Immediate(reinterpret_cast<int32_t>(out_of_memory)));
  __ b(eq, throw_out_of_memory_exception);

  // Retrieve the pending exception and clear the variable.
  __ mov(r3, Operand(ExternalReference::the_hole_value_location(isolate)));
  __ ldr(r2, MemOperand(r3));
  __ mov(r3, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                       isolate)));
  __ ldr(r0, MemOperand(r3));
  __ str(r2, MemOperand(r3));

  // Special handling of termination exceptions which are uncatchable
  // by javascript code.
  __ mov(r3, Operand(isolate->factory()->termination_exception()));
  __ cmpeq(r0, r3);
  __ bt(throw_termination_exception);

  // Handle normal exception.
  __ jmp(throw_normal_exception);

  __ bind(&retry);  // pass last failure (r0) as parameter (r0) when retrying
}


void CEntryStub::Generate(MacroAssembler* masm) {
  // Called from JavaScript; parameters are on stack as if calling JS function
  // r0: number of arguments including receiver
  // r1: pointer to builtin function
  // fp: frame pointer  (restored after C call)
  // sp: stack pointer  (restored as callee's sp after C call)
  // cp: current context  (C callee-saved)

  // NOTE: Invocations of builtins may return failure objects instead
  // of a proper result. The builtin entry handles this by performing
  // a garbage collection and retrying the builtin (twice).

  // Compute the argv pointer in a callee-saved register.
  __ lsl(sh4_r10, r0, Immediate(kPointerSizeLog2));
  __ add(sh4_r10, sp, sh4_r10);
  __ sub(sh4_r10, sh4_r10, Immediate(kPointerSize));

  // Enter the exit frame that transitions from JavaScript to C++.
  __ EnterExitFrame(save_doubles_);

  // Setup argc and the builtin function in callee-saved registers.
  __ mov(sh4_r8, r0);
  __ mov(sh4_r9, r1);

  // sh4_r8: number of arguments (C callee-saved)
  // sh4_r9: pointer to builtin function (C callee-saved)
  // sh4_r10: pointer to first argument (C callee-saved)
  ASSERT(!sh4_r8.is(sh4_rtmp) && !sh4_r9.is(sh4_rtmp) && !sh4_r10.is(sh4_rtmp));

  Label throw_normal_exception;
  Label throw_termination_exception;
  Label throw_out_of_memory_exception;

  // Call into the runtime system.
  GenerateCore(masm,
               &throw_normal_exception,
               &throw_termination_exception,
               &throw_out_of_memory_exception,
               false,
               false);

  // Do space-specific GC and retry runtime call.
  GenerateCore(masm,
               &throw_normal_exception,
               &throw_termination_exception,
               &throw_out_of_memory_exception,
               true,
               false);

  // Do full GC and retry runtime call one final time.
  Failure* failure = Failure::InternalError();
  __ mov(r0, Operand(reinterpret_cast<int32_t>(failure)));
  GenerateCore(masm,
               &throw_normal_exception,
               &throw_termination_exception,
               &throw_out_of_memory_exception,
               true,
               true);

  __ bind(&throw_out_of_memory_exception);
  __ ThrowUncatchable(OUT_OF_MEMORY, r0);

  __ bind(&throw_termination_exception);
  __ ThrowUncatchable(TERMINATION, r0);
  __ bind(&throw_normal_exception);
  __ Throw(r0);
}


void JSEntryStub::GenerateBody(MacroAssembler* masm, bool is_construct) {
  // r4: code entry
  // r5: function
  // r6: receiver
  // r7: argc
  // [sp+0]: argv

  Label invoke, exit;

  // Save callee-saved registers
  __ pushm(kCalleeSaved);
  __ push(pr);

  // Move the registers to use ARM ABI (and JS ABI)
  __ mov(r0, r4);
  __ mov(r1, r5);
  __ mov(r2, r6);
  __ mov(r3, r7);

  // Get address of argv
  // r0: code entry
  // r1: function
  // r2: receiver
  // r3: argc
  // [sp+0]: argv
  __ ldr(r4, MemOperand(sp, (kNumCalleeSaved + 1)* kPointerSize)); // argv

  // Push a frame with special values setup to mark it as an entry frame.
  // r0: code entry
  // r1: function
  // r2: receiver
  // r3: argc
  // r4: argv
  Isolate* isolate = masm->isolate();
  __ mov(ip, Immediate(-1));       // Push a bad frame pointer to fail if it is used.
  __ push(ip);

  int marker = is_construct ? StackFrame::ENTRY_CONSTRUCT : StackFrame::ENTRY;
  __ mov(r7, Immediate(Smi::FromInt(marker)));
  __ mov(r6, Immediate(Smi::FromInt(marker)));

  __ mov(r5, Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate)));
  __ mov(r5, MemOperand(r5));

  __ push(r7);
  __ push(r6);
  __ push(r5);

  // Setup frame pointer for the frame to be pushed.
  __ add(fp, sp, Immediate(-EntryFrameConstants::kCallerFPOffset));

#ifdef ENABLE_LOGGING_AND_PROFILING
//  // If this is the outermost JS call, set js_entry_sp value.
  ExternalReference js_entry_sp(Isolate::k_js_entry_sp_address, isolate);
  Label skip;
  __ mov(r5, Operand(ExternalReference(js_entry_sp)));
  __ ldr(r6, MemOperand(r5));
  __ cmpeq(r6, Immediate(0));
  __ bf(&skip);
  __ str(fp, MemOperand(r5));
  __ bind(&skip);
#endif


  // Call a faked try-block that does the invoke.
  __ jsr(&invoke);

  // Caught exception: Store result (exception) in the pending
  // exception field in the JSEnv and return a failure sentinel.
  // Coming in here the fp will be invalid because the PushTryHandler below
  // sets it to 0 to signal the existence of the JSEntry frame.
  __ mov(ip, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                        isolate)));
  __ str(r0, MemOperand(ip));
  __ mov(r0, Operand(reinterpret_cast<int32_t>(Failure::Exception())));
  __ jmp(&exit);

  // Invoke: Link this frame into the handler chain.
  __ bind(&invoke);
  // Must preserve r0-r4, r5-r7 are available.
  __ PushTryHandler(IN_JS_ENTRY, JS_ENTRY_HANDLER);
  // If an exception not caught by another handler occurs, this handler
  // returns control to the code after the jmp(&invoke) above, which
  // restores all kCalleeSaved registers (including cp and fp) to their
  // saved values before returning a failure to C.

  // Clear any pending exceptions.
  __ mov(ip, Operand(ExternalReference::the_hole_value_location(isolate)));
  __ ldr(r5, MemOperand(ip));
  __ mov(ip, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                       isolate)));
  __ str(r5, MemOperand(ip));

  // Invoke the function by calling through JS entry trampoline builtin.
  // Notice that we cannot store a reference to the trampoline code directly in
  // this stub, because runtime stubs are not traversed when doing GC.

  // Expected registers by Builtins::JSEntryTrampoline
  // r0: code entry
  // r1: function
  // r2: receiver
  // r3: argc
  // r4: argv
  if (is_construct) {
    ExternalReference construct_entry(Builtins::kJSConstructEntryTrampoline,
                                      isolate);
    __ mov(ip, Operand(construct_entry));
  } else {
    ExternalReference entry(Builtins::kJSEntryTrampoline, isolate);
    __ mov(ip, Operand(entry));
  }
  __ ldr(ip, MemOperand(ip));  // deref address

  // JSEntryTrampoline
  __ add(ip, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ jsr(ip);

  // r0 may hold a result here, do not use it

  // Unlink this frame from the handler chain. When reading the
  // address of the next handler, there is no need to use the address
  // displacement since the current stack pointer (sp) points directly
  // to the stack handler.
  __ ldr(r3, MemOperand(sp, StackHandlerConstants::kNextOffset));
  __ mov(ip, Operand(ExternalReference(Isolate::k_handler_address, isolate)));
  __ str(r3, MemOperand(ip));
  // No need to restore registers
  __ add(sp, sp, Immediate(StackHandlerConstants::kSize));

#ifdef ENABLE_LOGGING_AND_PROFILING
  // If current FP value is the same as js_entry_sp value, it means that
  // the current function is the outermost.
  Label skip_out;
  __ mov(r5, Operand(ExternalReference(js_entry_sp)));
  __ ldr(r6, MemOperand(r5));
  __ cmpeq(fp, r6);
  __ bf(&skip_out);
  __ mov(r6, Immediate(0));
  __ str(r6, MemOperand(r5));
  __ bind(&skip_out);
#endif

  __ bind(&exit);  // r0 holds result
  // Restore the top frame descriptors from the stack.
  __ pop(r3);
  __ mov(ip,
         Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate)));
  __ str(r3, MemOperand(ip));

  // Reset the stack to the callee saved registers.
  __ add(sp, sp, Immediate(-EntryFrameConstants::kCallerFPOffset));

  // Pop the linkage register from the stack
  __ pop(pr);

  // Restore callee-saved registers and return.
  __ popm(kCalleeSaved);

  __ rts();
}


void InstanceofStub::Generate(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


Register InstanceofStub::left() {
  UNIMPLEMENTED();
}


Register InstanceofStub::right() {
  UNIMPLEMENTED();
}


void ArgumentsAccessStub::GenerateReadElement(MacroAssembler* masm) {
  // The displacement is the offset of the last parameter (if any)
  // relative to the frame pointer.
  static const int kDisplacement =
      StandardFrameConstants::kCallerSPOffset - kPointerSize;

  // r0 (when parameter) or r0 (when return value)
  // r1 (when parameter)

  // Check that the key is a smi.
  Label slow;
  // SH4 live-in: r0, r1
  // SH4 live-out: r0, r1
  __ JumpIfNotSmi(r1, &slow);

  // Check if the calling frame is an arguments adaptor frame.
  Label adaptor;
  // SH4 live-in: r0, r1
  // SH4 live-out: r0, r1, r2
  __ ldr(r2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ ldr(r3, MemOperand(r2, StandardFrameConstants::kContextOffset));
  __ cmp(r3, Immediate(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ b(eq, &adaptor);

  // Check index against formal parameters count limit passed in
  // through register r0. Use unsigned comparison to get negative
  // check for free.
  // SH4 live-in: r0, r1
  // SH4 live-out: r0, r1
  __ cmpgeu(r1, r0);
  __ bt(&slow);

  // Read the argument from the stack and return it.
  // SH4 live-in: r0, r1
  // SH4 live-out: r0
  __ sub(r3, r0, r1);
  __ lsl(r4, r3, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(r3, fp, r4);
  __ ldr(r0, MemOperand(r3, kDisplacement));
  __ rts();

  // Arguments adaptor case: Check index against actual arguments
  // limit found in the arguments adaptor frame. Use unsigned
  // comparison to get negative check for free.
  __ bind(&adaptor);
  // SH4 live-in: r1, r2
  // SH4 live-out: r0, r1, r2
  __ ldr(r0, MemOperand(r2, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ cmpgeu(r1, r0);
  __ bt(&slow);

  // Read the argument from the adaptor frame and return it.
  // SH4 live-in: r0, r1, r2
  // SH4 live-out: r0
  __ sub(r3, r0, r1);
  __ lsl(r0, r3, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(r3, r2, r0);
  __ ldr(r0, MemOperand(r3, kDisplacement));
  __ rts();

  __ bind(&slow);
  __ push(r1);
  __ TailCallRuntime(Runtime::kGetArgumentsProperty, 1, 1);
}


void ArgumentsAccessStub::GenerateNewObject(MacroAssembler* masm) {
  // sp[0] : number of parameters
  // sp[4] : receiver displacement
  // sp[8] : function

  // live-in registers: none
  // defined registers (live through the whole function): sp, fp, cp

  // Some preconditions, make the code selection simpler
  // If these fail, verify and adapt the code below.
  STATIC_ASSERT(kSmiTagSize == 1);
  STATIC_ASSERT(kPointerSizeLog2 == 2);

  // Check if the calling frame is an arguments adaptor frame.
  Label adaptor_frame, try_allocate, runtime;
  // Pseudo code {
  //   live-in: none
  //   live-out: r2
  //   r2 = load32(add32(fp, kCallerFPOffset))
  //   r3 = load32(add32(r2, kContextOffset))
  //   t0 = r3 == ARGUMENTS_ADAPTOR
  //   if (t0) goto adaptor_frame
  // }
  __ ldr(r2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ ldr(r3, MemOperand(r2, StandardFrameConstants::kContextOffset));
  __ cmpeq(r3, Immediate(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ bt(&adaptor_frame);

  // Get the length from the frame.
  // Pseudo code {
  //   live-in: r2
  //   live-out: r2, r1
  //   r1 = load32(add32(sp, 0))
  //   goto  try_allocate
  // }
  __ ldr(r1, MemOperand(sp, 0));
  __ jmp(&try_allocate);

  // Patch the arguments.length and the parameters pointer.
  __ bind(&adaptor_frame);
  // Pseudo code {
  //   live-in: r2
  //   live-out: r2, r1
  //   r1 = load32(add32(r2, kLengthOffset))
  //   store32(add32(sp, 0), r1)
  //   r3 = add32(r2, shl32(r1, kPointerSizeLog2 - kSmiTagSize))
  //   r3 = add32(r3, kCallerSPOffset)
  //   store32(add32(sp, 1 * kPointerSize), r3)
  // }
  __ ldr(r1, MemOperand(r2, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ str(r1, MemOperand(sp, 0));
  __ lsl(r0, r1, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(r0, r2, r0);
  __ add(r0, r0, Immediate(StandardFrameConstants::kCallerSPOffset));
  __ str(r0, MemOperand(sp, 1 * kPointerSize));

  // Try the new space allocation. Start out with computing the size
  // of the arguments object and the elements array in words.
  Label add_arguments_object;
  __ bind(&try_allocate);
  // Pseudo code {
  //   live-in: r2, r1
  //   live-out: r2, r1
  //   t0 = r1 == 0
  //   if (t0) goto add_arguments_object
  //   r1 = r1 >> kSmiTagSize
  //   r1 = r1 + FixedArray::kHeaderSize / kPointerSize
  //   add_arguments_object:
  //   r1 = r1 + GetArgumentsObjectSize() / kPointerSize
  // }
  __ cmpeq(r1, Immediate(0));
  __ bt(&add_arguments_object);
  __ lsr(r1, r1, Immediate(kSmiTagSize));
  __ add(r1, r1, Immediate(FixedArray::kHeaderSize / kPointerSize));
  __ bind(&add_arguments_object);
  __ add(r1, r1, Immediate(GetArgumentsObjectSize() / kPointerSize));
  
  // Do the allocation of both objects in one go.
  // Pseudo code {
  //   live-in: r1
  //   live-out: r0
  //   AllocateInNewSpace(r1, r0, r2, r3)
  // }
  __ AllocateInNewSpace(
      r1, // object size
      r0, // result
      r2, // scratch1
      r4, // scratch2
      &runtime,
      static_cast<AllocationFlags>(TAG_OBJECT | SIZE_IN_WORDS));

  // Get the arguments boilerplate from the current (global) context.
  // Pseudo code {
  //   live-in: r0
  //   live-out: r0, r4
  //   r4 = load32(add32(cp, Context::SlotOffset(Context::GLOBAL_INDEX)))
  //   r4 = load32(add32(r4, GlobalObject::kGlobalContextOffset))
  //   r4 = load32(add32(r4, Context::SlotOffset(GetArgumentsBoilerplateIndex())))
  // }
  __ ldr(r4, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  __ ldr(r4, FieldMemOperand(r4, GlobalObject::kGlobalContextOffset));
  __ ldr(r4, MemOperand(r4,
			Context::SlotOffset(GetArgumentsBoilerplateIndex())));
  // }

  // Copy the JS object part.
  // Pseudo code {
  //   live-in: r0, r4
  //   live-out: r0
  //   CopyFields(r0 // dst , r4 //src , r3.bit() // scratch list,
  //              JSObject::kHeaderSize / kPointerSize)
  // }
  __ CopyFields(r0, r4, r3.bit(), JSObject::kHeaderSize / kPointerSize);

  if (type_ == NEW_NON_STRICT) {
    // Setup the callee in-object property.
    STATIC_ASSERT(Heap::kArgumentsCalleeIndex == 1);
    const int kCalleeOffset = JSObject::kHeaderSize +
                              Heap::kArgumentsCalleeIndex * kPointerSize;
    // Pseudo code {
    //   live-in: r0
    //   live-out: r0
    //   r3 = load32(add32(sp, 2 * kPointerSize))
    //   store32(add32(r0, kCalleeOffset), r3)
    // }
    __ ldr(r1, MemOperand(sp, 2 * kPointerSize));
    __ str(r1, FieldMemOperand(r0, kCalleeOffset));
  }

  // Get the length (smi tagged) and set that as an in-object property too.
  STATIC_ASSERT(Heap::kArgumentsLengthIndex == 0);
  // Pseudo code {
  //   live-in: r0
  //   live-out: r0, r1
  //   r1 = load32(add32(sp, 0 * kPointerSize))
  //   store32(add32(r0, JSObject::kHeaderSize + Heap::kArgumentsLengthIndex * kPointerSize), r1)
  // }
  __ ldr(r1, MemOperand(sp, 0 * kPointerSize));
  __ str(r1, FieldMemOperand(r0, JSObject::kHeaderSize +
			         Heap::kArgumentsLengthIndex * kPointerSize));

  // If there are no actual arguments, we're done.
  Label done;
  // Pseudo code {
  //   live-in: r1, r0
  //   live-out: r1, r0
  //   t0 = r1 == 0
  //   if (t0) goto done
  // }
  __ cmpeq(r1, Immediate(0));
  __ bt(&done);
  // }

  // Get the parameters pointer from the stack.
  // Pseudo code {
  //   live-in: r1, r0
  //   live-out: r1, r0, r2
  //   r2 = load32(add32(sp, 1 * kPointerSize))
  // }
  __ ldr(r2, MemOperand(sp, 1 * kPointerSize));

  // Setup the elements pointer in the allocated arguments object and
  // initialize the header in the elements fixed array.
  // Pseudo code {
  //   live-in: r1, r0, r2
  //   live-out: r1, r0, r2
  //   r4 = add32(r0, GetArgumentsObjectSize())
  //   store32(add32(r0, JSObject::kElementsOffset), r4)
  //   LoadRoot(r3, Heap::kFixedArrayMapRootIndex);
  //   store32(add32(r4, FixedArray::kMapOffset), r3)
  //   store32(add32(r4, FixedArray::kLengthOffset), r1)
  //   r1 = shr(r1, kSmiTagSize)
  // }
  __ add(r4, r0, Immediate(GetArgumentsObjectSize()));
  __ str(r4, FieldMemOperand(r0, JSObject::kElementsOffset));
  __ LoadRoot(r3, Heap::kFixedArrayMapRootIndex);
  __ str(r3, FieldMemOperand(r4, FixedArray::kMapOffset));
  __ str(r1, FieldMemOperand(r4, FixedArray::kLengthOffset));
  __ lsr(r1, r1, Immediate(kSmiTagSize));

  // Copy the fixed array slots.
  Label loop;
  // Setup r4 to point to the first array slot.
  // Pseudo code {
  //   live-in: r1, r0, r2, r4
  //   live-out: r1, r0, r2, r4
  //   r4 = add32(r4, FixedArray::kHeaderSize - kHeapObjectTag)
  // }
  __ add(r4, r4, Immediate(FixedArray::kHeaderSize - kHeapObjectTag));
  __ bind(&loop);
  // Pre-decrement r2 with kPointerSize on each iteration.
  // Pre-decrement in order to skip receiver.
  // Post-increment r4 with kPointerSize on each iteration.
  // Pseudo code {
  //   live-in: r1, r0, r2, r4
  //   live-out: r1, r0, r2, r4
  //   r2 = sub32(r2, kPointerSize)
  //   r3 = load32(r2)
  //   store32(r4, r3)
  //   r4 = add32(r4, kPointerSize)
  //   r1 = sub32(r1, 1)
  //   t0 = r1 == 0
  //   if (!t0) goto loop
  // }
  __ sub(r2, r2, Immediate(kPointerSize));
  __ ldr(r3, MemOperand(r2, 0));
  __ str(r3, MemOperand(r4, 0));
  __ add(r4, r4, Immediate(kPointerSize));
  __ sub(r1, r1, Immediate(1));
  __ cmpeq(r1, Immediate(0));
  __ bf(&loop);

  // Return and remove the on-stack parameters.
  __ bind(&done);
  // Pseudo code {
  //   live-in: none
  //   live-out: none
  //   sp = sp + 3 * kPointerSize
  //   return
  // }
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ Ret();

  // Do the runtime call to allocate the arguments object.
  __ bind(&runtime);
  // Pseudo code {
  //   live-in: none
  //   live-out: none
  //   TailCallRuntime
  // }
  __ TailCallRuntime(Runtime::kNewArgumentsFast, 3, 1);
}


void CallFunctionStub::Generate(MacroAssembler* masm) {
  Label slow;

  // If the receiver might be a value (string, number or boolean) check for this
  // and box it if it is.
  if (ReceiverMightBeValue()) {
    // Get the receiver from the stack.
    // function, receiver [, arguments]
    Label receiver_is_value, receiver_is_js_object;
    __ ldr(r1, MemOperand(sp, argc_ * kPointerSize));

    // Check if receiver is a smi (which is a number value).
    __ JumpIfSmi(r1, &receiver_is_value);

    // Check if the receiver is a valid JS object.
    __ CompareObjectType(r1, r2, r2, FIRST_JS_OBJECT_TYPE, ge);
    __ bt(&receiver_is_js_object);

    // Call the runtime to box the value.
    __ bind(&receiver_is_value);
    __ EnterInternalFrame();
    __ push(r1);
    __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
    __ LeaveInternalFrame();
    __ str(r0, MemOperand(sp, argc_ * kPointerSize));

    __ bind(&receiver_is_js_object);
  }

  // Get the function to call from the stack.
  // function, receiver [, arguments]
  __ ldr(r1, MemOperand(sp, (argc_ + 1) * kPointerSize));

  // Check that the function is really a JavaScript function.
  // r1: pushed function (to be verified)
  __ JumpIfSmi(r1, &slow);
  // Get the map of the function object.
  __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE, eq);
  __ bf(&slow);

  // Fast-case: Invoke the function now.
  // r1: pushed function
  ParameterCount actual(argc_);
  __ InvokeFunction(r1, actual, JUMP_FUNCTION);

  // Slow-case: Non-function called.
  __ bind(&slow);
  // CALL_NON_FUNCTION expects the non-function callee as receiver (instead
  // of the original receiver from the call site).
  __ str(r1, MemOperand(sp, argc_ * kPointerSize));
  __ mov(r0, Operand(argc_));  // Setup the number of arguments.
  __ mov(r2, Immediate(0));
  __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION);
  // r0, r1, r2, r3: arguments to trampoline
  __ Jump(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
          RelocInfo::CODE_TARGET);
}


const char* CompareStub::GetName() {
  ASSERT((lhs_.is(r0) && rhs_.is(r1)) ||
         (lhs_.is(r1) && rhs_.is(r0)));

  if (name_ != NULL) return name_;
  const int kMaxNameLength = 100;
  name_ = Isolate::Current()->bootstrapper()->AllocateAutoDeletedArray(
      kMaxNameLength);
  if (name_ == NULL) return "OOM";

  const char* cc_name;
  switch (cc_) {
    case lt: cc_name = "LT"; break;
    case gt: cc_name = "GT"; break;
    case le: cc_name = "LE"; break;
    case ge: cc_name = "GE"; break;
    case eq: cc_name = "EQ"; break;
    case ne: cc_name = "NE"; break;
    default: cc_name = "UnknownCondition"; break;
  }

  const char* lhs_name = lhs_.is(r0) ? "_r0" : "_r1";
  const char* rhs_name = rhs_.is(r0) ? "_r0" : "_r1";

  const char* strict_name = "";
  if (strict_ && (cc_ == eq || cc_ == ne)) {
    strict_name = "_STRICT";
  }

  const char* never_nan_nan_name = "";
  if (never_nan_nan_ && (cc_ == eq || cc_ == ne)) {
    never_nan_nan_name = "_NO_NAN";
  }

  const char* include_number_compare_name = "";
  if (!include_number_compare_) {
    include_number_compare_name = "_NO_NUMBER";
  }

  const char* include_smi_compare_name = "";
  if (!include_smi_compare_) {
    include_smi_compare_name = "_NO_SMI";
  }

  OS::SNPrintF(Vector<char>(name_, kMaxNameLength),
               "CompareStub_%s%s%s%s%s%s",
               cc_name,
               lhs_name,
               rhs_name,
               strict_name,
               never_nan_nan_name,
               include_number_compare_name,
               include_smi_compare_name);
  return name_;
}


int CompareStub::MinorKey() {
  // Encode the three parameters in a unique 16 bit value. To avoid duplicate
  // stubs the never NaN NaN condition is only taken into account if the
  // condition is equals.
  ASSERT((static_cast<unsigned>(cc_) >> 28) < (1 << 12));
  ASSERT((lhs_.is(r0) && rhs_.is(r1)) ||
         (lhs_.is(r1) && rhs_.is(r0)));
  return ConditionField::encode(static_cast<unsigned>(cc_) >> 28)
         | RegisterField::encode(lhs_.is(r0))
         | StrictField::encode(strict_)
         | NeverNanNanField::encode(cc_ == eq ? never_nan_nan_ : false)
         | IncludeNumberCompareField::encode(include_number_compare_)
         | IncludeSmiCompareField::encode(include_smi_compare_);
}


// -------------------------------------------------------------------------
// StringCharAtGenerator

void StringCharAtGenerator::GenerateFast(MacroAssembler* masm) {
  char_code_at_generator_.GenerateFast(masm);
  char_from_code_generator_.GenerateFast(masm);
}


void StringCharAtGenerator::GenerateSlow(
    MacroAssembler* masm, const RuntimeCallHelper& call_helper) {
  char_code_at_generator_.GenerateSlow(masm, call_helper);
  char_from_code_generator_.GenerateSlow(masm, call_helper);
}


class StringHelper : public AllStatic {
 public:
  // Generate code for copying characters using a simple loop. This should only
  // be used in places where the number of characters is small and the
  // additional setup and checking in GenerateCopyCharactersLong adds too much
  // overhead. Copying of overlapping regions is not supported.
  // Dest register ends at the position after the last character written.
  static void GenerateCopyCharacters(MacroAssembler* masm,
                                     Register dest,
                                     Register src,
                                     Register count,
                                     Register scratch,
                                     bool ascii);

  // Generate code for copying a large number of characters. This function
  // is allowed to spend extra time setting up conditions to make copying
  // faster. Copying of overlapping regions is not supported.
  // Dest register ends at the position after the last character written.
  static void GenerateCopyCharactersLong(MacroAssembler* masm,
                                         Register dest,
                                         Register src,
                                         Register count,
                                         Register scratch1,
                                         Register scratch2,
                                         Register scratch3,
                                         Register scratch4,
                                         Register scratch5,
                                         int flags);


  // Probe the symbol table for a two character string. If the string is
  // not found by probing a jump to the label not_found is performed. This jump
  // does not guarantee that the string is not in the symbol table. If the
  // string is found the code falls through with the string in register r0.
  // Contents of both c1 and c2 registers are modified. At the exit c1 is
  // guaranteed to contain halfword with low and high bytes equal to
  // initial contents of c1 and c2 respectively.
  static void GenerateTwoCharacterSymbolTableProbe(MacroAssembler* masm,
                                                   Register c1,
                                                   Register c2,
                                                   Register scratch1,
                                                   Register scratch2,
                                                   Register scratch3,
                                                   Register scratch4,
                                                   Register scratch5,
                                                   Label* not_found);

  // Generate string hash.
  // Added a scratch parameter for the SH4 implementation compared to ARM.
  static void GenerateHashInit(MacroAssembler* masm,
                               Register hash,
                               Register character,
			       Register scratch);

  static void GenerateHashAddCharacter(MacroAssembler* masm,
                                       Register hash,
                                       Register character,
                                       Register scratch);

  static void GenerateHashGetHash(MacroAssembler* masm,
                                  Register hash,
                                  Register scratch);

 private:
  DISALLOW_IMPLICIT_CONSTRUCTORS(StringHelper);
};


// -------------------------------------------------------------------------
// StringCharFromCodeGenerator

void StringCharCodeAtGenerator::GenerateFast(MacroAssembler* masm) {
  Label flat_string;
  Label ascii_string;
  Label got_char_code;

  // If the receiver is a smi trigger the non-string case.
  __ JumpIfSmi(object_, receiver_not_string_);

  // Fetch the instance type of the receiver into result register.
  __ ldr(result_, FieldMemOperand(object_, HeapObject::kMapOffset));
  __ ldrb(result_, FieldMemOperand(result_, Map::kInstanceTypeOffset));
  // If the receiver is not a string trigger the non-string case.
  __ tst(result_, Immediate(kIsNotStringMask));
  __ b(ne, receiver_not_string_);

  // If the index is non-smi trigger the non-smi case.
  __ JumpIfNotSmi(index_, &index_not_smi_);

  // Put smi-tagged index into scratch register.
  __ mov(scratch_, index_);
  __ bind(&got_smi_index_);

  // Check for index out of range.
  __ ldr(ip, FieldMemOperand(object_, String::kLengthOffset));
  __ cmpgeu(scratch_, ip);
  __ bf(index_out_of_range_);

  // We need special handling for non-flat strings.
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(result_, Immediate(kStringRepresentationMask));
  __ b(eq, &flat_string);

  // Handle non-flat strings.
  __ tst(result_, Immediate(kIsConsStringMask));
  __ b(eq, &call_runtime_);

  // ConsString.
  // Check whether the right hand side is the empty string (i.e. if
  // this is really a flat string in a cons string). If that is not
  // the case we would rather go to the runtime system now to flatten
  // the string.
  __ ldr(result_, FieldMemOperand(object_, ConsString::kSecondOffset));
  __ LoadRoot(ip, Heap::kEmptyStringRootIndex);
  __ cmpeq(result_, ip);
  __ b(ne, &call_runtime_);
  // Get the first of the two strings and load its instance type.
  __ ldr(object_, FieldMemOperand(object_, ConsString::kFirstOffset));
  __ ldr(result_, FieldMemOperand(object_, HeapObject::kMapOffset));
  __ ldrb(result_, FieldMemOperand(result_, Map::kInstanceTypeOffset));
  // If the first cons component is also non-flat, then go to runtime.
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(result_, Immediate(kStringRepresentationMask));
  __ b(ne, &call_runtime_);

  // Check for 1-byte or 2-byte string.
  __ bind(&flat_string);
  STATIC_ASSERT(kAsciiStringTag != 0);
  __ tst(result_, Immediate(kStringEncodingMask));
  __ b(ne, &ascii_string);

  // 2-byte string.
  // Load the 2-byte character code into the result register. We can
  // add without shifting since the smi tag size is the log2 of the
  // number of bytes in a two-byte character.
  STATIC_ASSERT(kSmiTag == 0 && kSmiTagSize == 1 && kSmiShiftSize == 0);
  __ add(scratch_, object_, scratch_);
  __ bkpt();
  __ mov(result_, FieldMemOperand(scratch_, SeqTwoByteString::kHeaderSize));    // FIXME: mov.w ??
  __ jmp(&got_char_code);

  // ASCII string.
  // Load the byte into the result register.
  __ bind(&ascii_string);
  __ lsr(scratch_, scratch_, Immediate(kSmiTagSize));
  __ add(scratch_, object_, scratch_);
  __ ldrb(result_, FieldMemOperand(scratch_, SeqAsciiString::kHeaderSize));

  __ bind(&got_char_code);
  __ lsl(result_, result_, Immediate(kSmiTagSize));
  __ bind(&exit_);
}


void StringCharCodeAtGenerator::GenerateSlow(
    MacroAssembler* masm, const RuntimeCallHelper& call_helper) {
  __ Abort("Unexpected fallthrough to CharCodeAt slow case");

  // Index is not a smi.
  __ bind(&index_not_smi_);
  // If index is a heap number, try converting it to an integer.
  __ CheckMap(index_,
              scratch_,
              Heap::kHeapNumberMapRootIndex,
              index_not_number_,
              true);
  call_helper.BeforeCall(masm);
  __ Push(object_, index_);
  __ push(index_);  // Consumed by runtime conversion function.
  if (index_flags_ == STRING_INDEX_IS_NUMBER) {
    __ CallRuntime(Runtime::kNumberToIntegerMapMinusZero, 1);
  } else {
    ASSERT(index_flags_ == STRING_INDEX_IS_ARRAY_INDEX);
    // NumberToSmi discards numbers that are not exact integers.
    __ CallRuntime(Runtime::kNumberToSmi, 1);
  }
  // Save the conversion result before the pop instructions below
  // have a chance to overwrite it.
  __ mov(scratch_, r0);
  __ pop(index_);
  __ pop(object_);
  // Reload the instance type.
  __ ldr(result_, FieldMemOperand(object_, HeapObject::kMapOffset));
  __ ldrb(result_, FieldMemOperand(result_, Map::kInstanceTypeOffset));
  call_helper.AfterCall(masm);
  // If index is still not a smi, it must be out of range.
  __ JumpIfNotSmi(scratch_, index_out_of_range_);
  // Otherwise, return to the fast path.
  __ jmp(&got_smi_index_);

  // Call runtime. We get here when the receiver is a string and the
  // index is a number, but the code of getting the actual character
  // is too complex (e.g., when the string needs to be flattened).
  __ bind(&call_runtime_);
  call_helper.BeforeCall(masm);
  __ Push(object_, index_);
  __ CallRuntime(Runtime::kStringCharCodeAt, 2);
  __ mov(result_, r0);
  call_helper.AfterCall(masm);
  __ jmp(&exit_);

  __ Abort("Unexpected fallthrough from CharCodeAt slow case");
}


// -------------------------------------------------------------------------
// StringCharFromCodeGenerator

void StringCharFromCodeGenerator::GenerateFast(MacroAssembler* masm) {
  // Fast case of Heap::LookupSingleCharacterStringFromCode.
  STATIC_ASSERT(kSmiTag == 0);
  STATIC_ASSERT(kSmiShiftSize == 0);
  ASSERT(IsPowerOf2(String::kMaxAsciiCharCode + 1));

  ASSERT(!code_.is(ip) && !result_.is(ip));

  __ tst(code_,
         Immediate(kSmiTagMask |
                 ((~String::kMaxAsciiCharCode) << kSmiTagSize)));
  __ b(ne, &slow_case_);

  __ LoadRoot(result_, Heap::kSingleCharacterStringCacheRootIndex);
  // At this point code register contains smi tagged ASCII char code.
  STATIC_ASSERT(kSmiTag == 0);
  __ lsl(ip, code_, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(result_, result_, ip);
  __ mov(result_, FieldMemOperand(result_, FixedArray::kHeaderSize));
  __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
  __ cmpeq(result_, ip);
  __ b(eq, &slow_case_);
  __ bind(&exit_);
}


void StringCharFromCodeGenerator::GenerateSlow(
    MacroAssembler* masm, const RuntimeCallHelper& call_helper) {
  __ Abort("Unexpected fallthrough to CharFromCode slow case");

  __ bind(&slow_case_);
  call_helper.BeforeCall(masm);
  __ push(code_);
  __ CallRuntime(Runtime::kCharFromCode, 1);
  __ mov(result_, r0);
  call_helper.AfterCall(masm);
  __ jmp(&exit_);

  __ Abort("Unexpected fallthrough from CharFromCode slow case");
}


void StringHelper::GenerateCopyCharacters(MacroAssembler* masm,
                                          Register dest,
                                          Register src,
                                          Register count,
                                          Register scratch,
                                          bool ascii) {
  Label loop;
  Label done;
  // This loop just copies one character at a time, as it is only used for very
  // short strings.
  if (!ascii) {
    __ add(count, count, count);
  } 
  __ cmp(count, Immediate(0));
  __ b(eq, &done);

  __ bind(&loop);
  __ ldrb(scratch, MemOperand(src));
  __ add(src, src, Immediate(1));
  // Perform sub between load and dependent store to get the load time to
  // complete.
  __ sub(count, count, Immediate(1));
  __ cmpgt(count, Immediate(0));
  __ strb(scratch, MemOperand(dest));
  __ add(dest, dest, Immediate(1));
  // last iteration.
  __ bt(&loop);

  __ bind(&done);
}


enum CopyCharactersFlags {
  COPY_ASCII = 1,
  DEST_ALWAYS_ALIGNED = 2
};


void StringHelper::GenerateCopyCharactersLong(MacroAssembler* masm,
                                              Register dest,
                                              Register src,
                                              Register count,
                                              Register scratch1,
                                              Register scratch2,
                                              Register scratch3,
                                              Register scratch4,
                                              Register scratch5,
                                              int flags) {
  bool ascii = (flags & COPY_ASCII) != 0;
  bool dest_always_aligned = (flags & DEST_ALWAYS_ALIGNED) != 0;

  if (dest_always_aligned && FLAG_debug_code) {
    // Check that destination is actually word aligned if the flag says
    // that it is.
    __ tst(dest, Immediate(kPointerAlignmentMask));
    __ Check(eq, "Destination of copy not aligned.");
  }

  const int kReadAlignment = 4;
  // Ensure that reading an entire aligned word containing the last character
  // of a string will not read outside the allocated area (because we pad up
  // to kObjectAlignment).
  STATIC_ASSERT(kObjectAlignment >= kReadAlignment);
  // Assumes word reads and writes are little endian.
  // Nothing to do for zero characters.
  Label done;
  if (!ascii) {
    __ add(count, count, count);
  } 
  __ cmp(count, Immediate(0));
  __ b(eq, &done);

  // Assume that you cannot read (or write) unaligned.
  Label byte_loop;
  __ add(count, dest, count);
  Register limit = count;  
  // TODO: SH4 implementation is not optimized, always copy byte per byte.
  // See ARM version for example of optimized code.

  // Copy bytes from src to dst until dst hits limit.
  __ bind(&byte_loop);
  __ ldrb(scratch1, MemOperand(src));
  __ add(src,src, Immediate(1));
  __ strb(scratch1, MemOperand(dest));
  __ add(dest, dest, Immediate(1));
  __ cmpge(dest, limit);
  __ bf(&byte_loop);

  __ bind(&done);
}

void StringHelper::GenerateTwoCharacterSymbolTableProbe(MacroAssembler* masm,
                                                        Register c1,
                                                        Register c2,
                                                        Register scratch1,
                                                        Register scratch2,
                                                        Register scratch3,
                                                        Register scratch4,
                                                        Register scratch5,
                                                        Label* not_found) {
  // Returns result in r0 if found.

  // Register scratch3 is the general scratch register in this function.
  Register scratch = scratch3;

  // Make sure that both characters are not digits as such strings has a
  // different hash algorithm. Don't try to look for these in the symbol table.
  Label not_array_index;
  __ sub(scratch, c1, Immediate(static_cast<int>('0')));
  __ cmphi(scratch, Immediate(static_cast<int>('9' - '0')));
  __ bt(&not_array_index);
  __ sub(scratch, c2, Immediate(static_cast<int>('0')));
  __ cmpgtu(scratch, Immediate(static_cast<int>('9' - '0')));

  // If check failed combine both characters into single halfword.
  // This is required by the contract of the method: code at the
  // not_found branch expects this combination in c1 register
  __ lsl(scratch1, c2, Immediate(kBitsPerByte));
  __ lor(c1, c1, scratch1, ne);
  __ b(ne, not_found);

  __ bind(&not_array_index);
  // Calculate the two character string hash.
  Register hash = scratch1;
  StringHelper::GenerateHashInit(masm, hash, c1, scratch);
  StringHelper::GenerateHashAddCharacter(masm, hash, c2, scratch);
  StringHelper::GenerateHashGetHash(masm, hash, scratch);

  // Collect the two characters in a register.
  Register chars = c1;
  __ lsl(scratch, c2, Immediate(kBitsPerByte));
  __ lor(chars, chars, scratch);

  // chars: two character string, char 1 in byte 0 and char 2 in byte 1.
  // hash:  hash of two character string.

  // Load symbol table
  // Load address of first element of the symbol table.
  Register symbol_table = c2;
  __ LoadRoot(symbol_table, Heap::kSymbolTableRootIndex);

  Register undefined = scratch4;
  __ LoadRoot(undefined, Heap::kUndefinedValueRootIndex);

  // Calculate capacity mask from the symbol table capacity.
  Register mask = scratch2;
  __ ldr(mask, FieldMemOperand(symbol_table, SymbolTable::kCapacityOffset));
  __ asr(mask, mask, Immediate(1));
  __ sub(mask, mask, Immediate(1));

  // Calculate untagged address of the first element of the symbol table.
  Register first_symbol_table_element = symbol_table;
  __ add(first_symbol_table_element, symbol_table,
         Immediate(SymbolTable::kElementsStartOffset - kHeapObjectTag));

  // Registers
  // chars: two character string, char 1 in byte 0 and char 2 in byte 1.
  // hash:  hash of two character string
  // mask:  capacity mask
  // first_symbol_table_element: address of the first element of
  //                             the symbol table
  // undefined: the undefined object
  // scratch: -

  // Perform a number of probes in the symbol table.
  static const int kProbes = 4;
  Label found_in_symbol_table;
  Label next_probe[kProbes];
  for (int i = 0; i < kProbes; i++) {
    Register candidate = scratch5;  // Scratch register contains candidate.

    // Calculate entry in symbol table.
    if (i > 0) {
      __ add(candidate, hash, Immediate(SymbolTable::GetProbeOffset(i)));
    } else {
      __ mov(candidate, hash);
    }

    __ land(candidate, candidate, mask);

    // Load the entry from the symble table.
    STATIC_ASSERT(SymbolTable::kEntrySize == 1);
    __ lsl(scratch, candidate, Immediate(kPointerSizeLog2));
    __ add(scratch, first_symbol_table_element, scratch);
    __ ldr(candidate, MemOperand(scratch));

    // If entry is undefined no string with this hash can be found.
    Label is_string;
    __ CompareObjectType(candidate, scratch, scratch, ODDBALL_TYPE, eq);
    __ b(ne, &is_string);

    __ cmp(undefined, candidate);
    __ b(eq, not_found);
    // Must be null (deleted entry).
    if (FLAG_debug_code) {
      __ LoadRoot(ip, Heap::kNullValueRootIndex);
      __ cmp(ip, candidate);
      __ Assert(eq, "oddball in symbol table is not undefined or null");
    }
    __ jmp(&next_probe[i]);

    __ bind(&is_string);

    // Check that the candidate is a non-external ASCII string.  The instance
    // type is still in the scratch register from the CompareObjectType
    // operation.
    __ JumpIfInstanceTypeIsNotSequentialAscii(scratch, scratch, &next_probe[i]);

    // If length is not 2 the string is not a candidate.
    __ ldr(scratch, FieldMemOperand(candidate, String::kLengthOffset));
    __ cmp(scratch, Immediate(Smi::FromInt(2)));
    __ b(ne, &next_probe[i]);

    // Check if the two characters match.
    // Assumes that word load is little endian.
    __ ldrh(scratch, FieldMemOperand(candidate, SeqAsciiString::kHeaderSize));
    __ cmp(chars, scratch);
    __ b(eq, &found_in_symbol_table);
    __ bind(&next_probe[i]);
  }

  // No matching 2 character string found by probing.
  __ jmp(not_found);

  // Scratch register contains result when we fall through to here.
  Register result = scratch;
  __ bind(&found_in_symbol_table);
  __ Move(r0, result);
}


void StringHelper::GenerateHashInit(MacroAssembler* masm,
                                    Register hash,
                                    Register character,
                                    Register scratch) {
  // Added a scratch parameter for the SH4 implementation compared to ARM.
  // hash = character + (character << 10);
  __ lsl(hash, character, Immediate(10));
  __ add(hash, character, hash);
  // hash ^= hash >> 6;
  __ asr(scratch, hash, Immediate(6));
  __ eor(hash, hash, scratch);
}


void StringHelper::GenerateHashAddCharacter(MacroAssembler* masm,
                                            Register hash,
                                            Register character,
                                            Register scratch) {

  // Added a scratch parameter for the SH4 implementation compared to ARM.
  // hash += character;
  __ add(hash, hash, character);
  // hash += hash << 10;
  __ lsl(scratch, hash, Immediate(10));
  __ add(hash, hash, scratch);
  // hash ^= hash >> 6;
  __ asr(scratch, hash, Immediate(6));
  __ eor(hash, hash, scratch);
}


void StringHelper::GenerateHashGetHash(MacroAssembler* masm,
                                       Register hash,
				       Register scratch) {
  // Added a scratch parameter for the SH4 implementation compared to ARM.
  // hash += hash << 3;
  __ lsl(scratch, hash, Immediate(3));
  __ add(hash, hash, scratch);
  // hash ^= hash >> 11;
  __ asr(scratch, hash, Immediate(11));
  __ eor(hash, hash, scratch);
  // hash += hash << 15;
  __ lsl(scratch, hash, Immediate(15));
  __ add(hash, hash, scratch);

  // if (hash == 0) hash = 27;
  __ cmp(hash, Immediate(27));
  __ mov(hash, Immediate(27), ne);
}


void SubStringStub::Generate(MacroAssembler* masm) {
  Label runtime;

  // Stack frame on entry.
  //  lr: return address
  //  sp[0]: to
  //  sp[4]: from
  //  sp[8]: string

  // This stub is called from the native-call %_SubString(...), so
  // nothing can be assumed about the arguments. It is tested that:
  //  "string" is a sequential string,
  //  both "from" and "to" are smis, and
  //  0 <= from <= to <= string.length.
  // If any of these assumptions fail, we call the runtime system.

  static const int kToOffset = 0 * kPointerSize;
  static const int kFromOffset = 1 * kPointerSize;
  static const int kStringOffset = 2 * kPointerSize;

  // Check bounds and smi-ness.
  Register to = r6;
  Register from = r7;
  __ Ldrd(to, from, MemOperand(sp, kToOffset));
  STATIC_ASSERT(kFromOffset == kToOffset + 4);
  STATIC_ASSERT(kSmiTag == 0);
  STATIC_ASSERT(kSmiTagSize + kSmiShiftSize == 1);
  __ JumpIfNotBothSmi(to, from, &runtime);      // not in arm code
  // I.e., arithmetic shift right by one un-smi-tags.
  __ asr(r2, to, Immediate(1));
  __ asr(r3, from, Immediate(1));
  __ cmpge(r3, Immediate(0));
  __ bf(&runtime);  // From is negative.

  // Both to and from are smis.

  __ sub(r2, r2, r3);
  __ cmpge(r2, Immediate(0)); 
  __ bf(&runtime);  // Fail if from > to.
  // Special handling of sub-strings of length 1 and 2. One character strings
  // are handled in the runtime system (looked up in the single character
  // cache). Two character strings are looked for in the symbol cache.
  __ cmpge(r2, Immediate(2));
  __ bf(&runtime);

  // r2: length
  // r3: from index (untaged smi)
  // r6 (a.k.a. to): to (smi)
  // r7 (a.k.a. from): from offset (smi)

  // Make sure first argument is a sequential (or flat) string.
  __ ldr(r5, MemOperand(sp, kStringOffset));
  STATIC_ASSERT(kSmiTag == 0);
  __ tst(r5, Immediate(kSmiTagMask));
  __ b(eq, &runtime);
  Condition is_string = masm->IsObjectStringType(r5, r1);
  __ b(NegateCondition(is_string), &runtime);

  // r1: instance type
  // r2: length
  // r3: from index (untagged smi)
  // r5: string
  // r6 (a.k.a. to): to (smi)
  // r7 (a.k.a. from): from offset (smi)
  Label seq_string;
  __ land(r4, r1, Immediate(kStringRepresentationMask));
  STATIC_ASSERT(kSeqStringTag < kConsStringTag);
  STATIC_ASSERT(kConsStringTag < kExternalStringTag);
  __ cmpgt(r4, Immediate(kConsStringTag));
  __ bt(&runtime);  // External strings go to runtime.
  __ cmpge(r4, Immediate(kConsStringTag));
  __ bf(&seq_string);  // Sequential strings are handled directly.

  // Cons string. Try to recurse (once) on the first substring.
  // (This adds a little more generality than necessary to handle flattened
  // cons strings, but not much).
  __ ldr(r5, FieldMemOperand(r5, ConsString::kFirstOffset));
  __ ldr(r4, FieldMemOperand(r5, HeapObject::kMapOffset));
  __ ldrb(r1, FieldMemOperand(r4, Map::kInstanceTypeOffset));
  __ tst(r1, Immediate(kStringRepresentationMask));
  STATIC_ASSERT(kSeqStringTag == 0);
  __ b(ne, &runtime);  // Cons and External strings go to runtime.

  // Definitly a sequential string.
  __ bind(&seq_string);

  // r1: instance type.
  // r2: length
  // r3: from index (untaged smi)
  // r5: string
  // r6 (a.k.a. to): to (smi)
  // r7 (a.k.a. from): from offset (smi)
  __ ldr(r4, FieldMemOperand(r5, String::kLengthOffset));
  __ cmpge(r4, to);
  __ bf(&runtime);  // Fail if to > length.
  to = no_reg;

  // r1: instance type.
  // r2: result string length.
  // r3: from index (untaged smi)
  // r5: string.
  // r7 (a.k.a. from): from offset (smi)
  // Check for flat ASCII string.
  Label non_ascii_flat;
  __ tst(r1, Immediate(kStringEncodingMask));
  STATIC_ASSERT(kTwoByteStringTag == 0);
  __ b(eq, &non_ascii_flat);

  Label result_longer_than_two;
  __ cmpgt(r2, Immediate(2));
  __ bt(&result_longer_than_two);

  // Sub string of length 2 requested.
  // Get the two characters forming the sub string.
  __ add(r5, r5, r3);
  __ ldrb(r3, FieldMemOperand(r5, SeqAsciiString::kHeaderSize));
  __ ldrb(r4, FieldMemOperand(r5, SeqAsciiString::kHeaderSize + 1));

  // Try to lookup two character string in symbol table.
  Label make_two_character_string;
  StringHelper::GenerateTwoCharacterSymbolTableProbe(
      masm, r3, r4, r1, r5, r6, r7, r9, &make_two_character_string);
  Counters* counters = masm->isolate()->counters();
  __ IncrementCounter(counters->sub_string_native(), 1, r3, r4);
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ Ret();

  // r2: result string length.
  // r3: two characters combined into halfword in little endian byte order.
  __ bind(&make_two_character_string);
  __ AllocateAsciiString(r0, r2, r4, r5, r9, &runtime);
  __ strh(r3, FieldMemOperand(r0, SeqAsciiString::kHeaderSize));
  __ IncrementCounter(counters->sub_string_native(), 1, r3, r4);
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ Ret();

  __ bind(&result_longer_than_two);

  // Allocate the result.
  __ AllocateAsciiString(r0, r2, r3, r4, r1, &runtime);

  // r0: result string.
  // r2: result string length.
  // r5: string.
  // r7 (a.k.a. from): from offset (smi)
  // Locate first character of result.
  __ add(r1, r0, Immediate(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  // Locate 'from' character of string.
  __ add(r5, r5, Immediate(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  __ asr(r6, from, Immediate(1));
  __ add(r5, r5, r6);

  // r0: result string.
  // r1: first character of result string.
  // r2: result string length.
  // r5: first character of sub string to copy.
  STATIC_ASSERT((SeqAsciiString::kHeaderSize & kObjectAlignmentMask) == 0);
  StringHelper::GenerateCopyCharactersLong(masm, r1, r5, r2, r3, r4, r6, r7, r9,
                                           COPY_ASCII | DEST_ALWAYS_ALIGNED);
  __ IncrementCounter(counters->sub_string_native(), 1, r3, r4);
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ Ret();

  __ bind(&non_ascii_flat);
  // r2: result string length.
  // r5: string.
  // r7 (a.k.a. from): from offset (smi)
  // Check for flat two byte string.

  // Allocate the result.
  __ AllocateTwoByteString(r0, r2, r1, r3, r4, &runtime);

  // r0: result string.
  // r2: result string length.
  // r5: string.
  // Locate first character of result.
  __ add(r1, r0, Immediate(SeqTwoByteString::kHeaderSize - kHeapObjectTag));
  // Locate 'from' character of string.
  __ add(r5, r5, Immediate(SeqTwoByteString::kHeaderSize - kHeapObjectTag));
  // As "from" is a smi it is 2 times the value which matches the size of a two
  // byte character.
  __ add(r5, r5, from);
  from = no_reg;

  // r0: result string.
  // r1: first character of result.
  // r2: result length.
  // r5: first character of string to copy.
  STATIC_ASSERT((SeqTwoByteString::kHeaderSize & kObjectAlignmentMask) == 0);
  StringHelper::GenerateCopyCharactersLong(
      masm, r1, r5, r2, r3, r4, r6, r7, r9, DEST_ALWAYS_ALIGNED);
  __ IncrementCounter(counters->sub_string_native(), 1, r3, r4);
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ Ret();

  // Just jump to runtime to create the sub string.
  __ bind(&runtime);
  __ TailCallRuntime(Runtime::kSubString, 3, 1);
}


void StringCompareStub::GenerateCompareFlatAsciiStrings(MacroAssembler* masm,
                                                        Register left,
                                                        Register right,
                                                        Register scratch1,
                                                        Register scratch2,
                                                        Register scratch3,
                                                        Register scratch4) {
  Label compare_lengths, skip;
  // Find minimum length and length difference.
  __ ldr(scratch1, FieldMemOperand(left, String::kLengthOffset));
  __ ldr(scratch2, FieldMemOperand(right, String::kLengthOffset));
  __ sub(scratch3, scratch1, scratch2);
  __ cmpge(scratch1, scratch2);
  Register length_delta = scratch3;
  __ mov(scratch1, scratch2, eq);
  Register min_length = scratch1;
  STATIC_ASSERT(kSmiTag == 0);
  __ tst(min_length, min_length);
  __ b(eq, &compare_lengths);

  // Untag smi.
  __ asr(min_length, min_length, Immediate(kSmiTagSize));

  // Setup registers so that we only need to increment one register
  // in the loop.
  __ add(scratch2, min_length,
         Immediate(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  __ add(left, left, scratch2);
  __ add(right, right, scratch2);
  // Registers left and right points to the min_length character of strings.
  __ rsb(min_length, min_length, Immediate(-1));
  Register index = min_length;
  // Index starts at -min_length.

  {
    // Compare loop.
    Label loop, end_loop;
    __ bind(&loop);
    // Skip to compare lengths with eq condition true.
    __ add(index, index, Immediate(1));
    __ tst(index, index);
    __ bt(&compare_lengths);
    // Compare characters.
    __ ldrb(scratch2, MemOperand(left, index));
    __ ldrb(scratch4, MemOperand(right, index));
    __ cmp(scratch2, scratch4);
    __ b(eq, &loop);

    // Fallthrough with last byte differing.
    Label lower;
    __ cmpge(scratch2, scratch4);
    __ bf(&lower);
    __ mov(r0, Immediate(Smi::FromInt(GREATER)));
    __ Ret();
    __ bind(&lower);
    __ mov(r0, Immediate(Smi::FromInt(LESS)));
    __ Ret();
  }
  // Compare lengths -  strings up to min-length are equal.
  __ bind(&compare_lengths);
  ASSERT(Smi::FromInt(EQUAL) == static_cast<Smi*>(0));
  // Use zero length_delta as result.
  __ mov(r0, length_delta);
  __ Ret();
}


void StringCompareStub::Generate(MacroAssembler* masm) {
  Label runtime;

  Counters* counters = masm->isolate()->counters();

  // Stack frame on entry.
  //  sp[0]: right string
  //  sp[4]: left string
  __ Ldrd(r0 , r1, MemOperand(sp));  // Load right in r0, left in r1.

  Label not_same;
  __ cmp(r0, r1);
  __ b(ne, &not_same);
  STATIC_ASSERT(EQUAL == 0);
  STATIC_ASSERT(kSmiTag == 0);
  __ mov(r0, Immediate(Smi::FromInt(EQUAL)));
  __ IncrementCounter(counters->string_compare_native(), 1, r1, r2);
  __ add(sp, sp, Immediate(2 * kPointerSize));
  __ Ret();

  __ bind(&not_same);

  // Check that both objects are sequential ASCII strings.
  __ JumpIfNotBothSequentialAsciiStrings(r1, r0, r2, r3, &runtime);

  // Compare flat ASCII strings natively. Remove arguments from stack first.
  __ IncrementCounter(counters->string_compare_native(), 1, r2, r3);
  __ add(sp, sp, Immediate(2 * kPointerSize));
  GenerateCompareFlatAsciiStrings(masm, r1, r0, r2, r3, r4, r5);

  // Call the runtime; it returns -1 (less), 0 (equal), or 1 (greater)
  // tagged as a small integer.
  __ bind(&runtime);
  __ TailCallRuntime(Runtime::kStringCompare, 2, 1);
}


void StringAddStub::Generate(MacroAssembler* masm) {
  Label string_add_runtime, call_builtin;
  Builtins::JavaScript builtin_id = Builtins::ADD;

  Counters* counters = masm->isolate()->counters();

  // Stack on entry:
  // sp[0]: second argument (right).
  // sp[4]: first argument (left).

  // Load the two arguments.
  __ ldr(r0, MemOperand(sp, 1 * kPointerSize));  // First argument.
  __ ldr(r1, MemOperand(sp, 0 * kPointerSize));  // Second argument.

  // Make sure that both arguments are strings if not known in advance.
  if (flags_ == NO_STRING_ADD_FLAGS) {
    __ JumpIfEitherSmi(r0, r1, &string_add_runtime);
    // Load instance types.
    __ ldr(r4, FieldMemOperand(r0, HeapObject::kMapOffset));
    __ ldr(r5, FieldMemOperand(r1, HeapObject::kMapOffset));
    __ ldrb(r4, FieldMemOperand(r4, Map::kInstanceTypeOffset));
    __ ldrb(r5, FieldMemOperand(r5, Map::kInstanceTypeOffset));
    STATIC_ASSERT(kStringTag == 0);
    // If either is not a string, go to runtime.
    __ tst(r4, Immediate(kIsNotStringMask));
    __ b(ne,&string_add_runtime);
    __ tst(r5, Immediate(kIsNotStringMask));
    __ b(ne,&string_add_runtime);
  } else {
    // Here at least one of the arguments is definitely a string.
    // We convert the one that is not known to be a string.
    if ((flags_ & NO_STRING_CHECK_LEFT_IN_STUB) == 0) {
      ASSERT((flags_ & NO_STRING_CHECK_RIGHT_IN_STUB) != 0);
      GenerateConvertArgument(
          masm, 1 * kPointerSize, r0, r2, r3, r4, r5, &call_builtin);
      builtin_id = Builtins::STRING_ADD_RIGHT;
    } else if ((flags_ & NO_STRING_CHECK_RIGHT_IN_STUB) == 0) {
      ASSERT((flags_ & NO_STRING_CHECK_LEFT_IN_STUB) != 0);
      GenerateConvertArgument(
          masm, 0 * kPointerSize, r1, r2, r3, r4, r5, &call_builtin);
      builtin_id = Builtins::STRING_ADD_LEFT;
    }
  }

  // Both arguments are strings.
  // r0: first string
  // r1: second string
  // r4: first string instance type (if flags_ == NO_STRING_ADD_FLAGS)
  // r5: second string instance type (if flags_ == NO_STRING_ADD_FLAGS)
  {
    Label strings_not_empty, string_return;
    // Check if either of the strings are empty. In that case return the other.
    __ ldr(r2, FieldMemOperand(r0, String::kLengthOffset));
    __ ldr(r3, FieldMemOperand(r1, String::kLengthOffset));
    STATIC_ASSERT(kSmiTag == 0);
    __ cmp(r2, Immediate(Smi::FromInt(0)));  // Test if first string is empty.
    __ mov(r0, r1, eq);  // If first is empty, return second.
    __ b(eq, &string_return);
    STATIC_ASSERT(kSmiTag == 0);
     // Else test if second string is empty.
    __ cmp(r3, Immediate(Smi::FromInt(0)));
    __ b(ne, &strings_not_empty);  // If either string was empty, return r0.

    __ bind(&string_return);
    __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
    __ add(sp, sp, Immediate(2 * kPointerSize));
    __ Ret();

    __ bind(&strings_not_empty);
  }

  __ asr(r2, r2, Immediate(kSmiTagSize));
  __ asr(r3, r3, Immediate(kSmiTagSize));
  // Both strings are non-empty.
  // r0: first string
  // r1: second string
  // r2: length of first string
  // r3: length of second string
  // r4: first string instance type (if flags_ == NO_STRING_ADD_FLAGS)
  // r5: second string instance type (if flags_ == NO_STRING_ADD_FLAGS)
  // Look at the length of the result of adding the two strings.
  Label string_add_flat_result, longer_than_two;
  // Adding two lengths can't overflow.
  STATIC_ASSERT(String::kMaxLength < String::kMaxLength * 2);
  __ add(r6, r2, r3);
  // Use the symbol table when adding two one character strings, as it
  // helps later optimizations to return a symbol here.
  __ cmp(r6, Immediate(2));
  __ b(ne, &longer_than_two);

  // Check that both strings are non-external ASCII strings.
  if (flags_ != NO_STRING_ADD_FLAGS) {
    __ ldr(r4, FieldMemOperand(r0, HeapObject::kMapOffset));
    __ ldr(r5, FieldMemOperand(r1, HeapObject::kMapOffset));
    __ ldrb(r4, FieldMemOperand(r4, Map::kInstanceTypeOffset));
    __ ldrb(r5, FieldMemOperand(r5, Map::kInstanceTypeOffset));
  }
  __ JumpIfBothInstanceTypesAreNotSequentialAscii(r4, r5, r6, r7,
                                                  &string_add_runtime);

  // Get the two characters forming the sub string.
  __ ldrb(r2, FieldMemOperand(r0, SeqAsciiString::kHeaderSize));
  __ ldrb(r3, FieldMemOperand(r1, SeqAsciiString::kHeaderSize));

  // Try to lookup two character string in symbol table. If it is not found
  // just allocate a new one.
  Label make_two_character_string;
  StringHelper::GenerateTwoCharacterSymbolTableProbe(
      masm, r2, r3, r6, r7, r4, r5, r9, &make_two_character_string);
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Immediate(2 * kPointerSize));
  __ Ret();

  __ bind(&make_two_character_string);
  // Resulting string has length 2 and first chars of two strings
  // are combined into single halfword in r2 register.
  // So we can fill resulting string without two loops by a single
  // halfword store instruction (which assumes that processor is
  // in a little endian mode)
  __ mov(r6, Immediate(2));
  __ AllocateAsciiString(r0, r6, r4, r5, r9, &string_add_runtime);
  __ strh(r2, FieldMemOperand(r0, SeqAsciiString::kHeaderSize));
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Immediate(2 * kPointerSize));
  __ Ret();

  __ bind(&longer_than_two);
  // Check if resulting string will be flat.
  __ cmpge(r6, Immediate(String::kMinNonFlatLength));
  __ bf(&string_add_flat_result);
  // Handle exceptionally long strings in the runtime system.
  STATIC_ASSERT((String::kMaxLength & 0x80000000) == 0);
  ASSERT(IsPowerOf2(String::kMaxLength + 1));
  // kMaxLength + 1 is representable as shifted literal, kMaxLength is not.
  __ cmphs(r6, Immediate(String::kMaxLength + 1));
  __ bt(&string_add_runtime);

  // If result is not supposed to be flat, allocate a cons string object.
  // If both strings are ASCII the result is an ASCII cons string.
  if (flags_ != NO_STRING_ADD_FLAGS) {
    __ ldr(r4, FieldMemOperand(r0, HeapObject::kMapOffset));
    __ ldr(r5, FieldMemOperand(r1, HeapObject::kMapOffset));
    __ ldrb(r4, FieldMemOperand(r4, Map::kInstanceTypeOffset));
    __ ldrb(r5, FieldMemOperand(r5, Map::kInstanceTypeOffset));
  }
  Label non_ascii, allocated, ascii_data;
  STATIC_ASSERT(kTwoByteStringTag == 0);
  __ tst(r4, Immediate(kStringEncodingMask));
  __ b(eq, &non_ascii);
  __ tst(r5, Immediate(kStringEncodingMask));
  __ b(eq, &non_ascii);

  // Allocate an ASCII cons string.
  __ bind(&ascii_data);
  __ AllocateAsciiConsString(r7, r6, r4, r5, &string_add_runtime);
  __ bind(&allocated);
  // Fill the fields of the cons string.
  __ str(r0, FieldMemOperand(r7, ConsString::kFirstOffset));
  __ str(r1, FieldMemOperand(r7, ConsString::kSecondOffset));
  __ mov(r0, r7);
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Immediate(2 * kPointerSize));
  __ Ret();

  __ bind(&non_ascii);
  // At least one of the strings is two-byte. Check whether it happens
  // to contain only ASCII characters.
  // r4: first instance type.
  // r5: second instance type.
  __ tst(r4, Immediate(kAsciiDataHintMask));
  __ b(ne, &ascii_data);
  __ tst(r5, Immediate(kAsciiDataHintMask));
  __ b(ne, &ascii_data);
  __ eor(r4, r4, r5);
  STATIC_ASSERT(kAsciiStringTag != 0 && kAsciiDataHintTag != 0);
  __ land(r4, r4, Immediate(kAsciiStringTag | kAsciiDataHintTag));
  __ cmp(r4, Immediate(kAsciiStringTag | kAsciiDataHintTag));
  __ b(eq, &ascii_data);

  // Allocate a two byte cons string.
  __ AllocateTwoByteConsString(r7, r6, r4, r5, &string_add_runtime);
  __ jmp(&allocated);

  // Handle creating a flat result. First check that both strings are
  // sequential and that they have the same encoding.
  // r0: first string
  // r1: second string
  // r2: length of first string
  // r3: length of second string
  // r4: first string instance type (if flags_ == NO_STRING_ADD_FLAGS)
  // r5: second string instance type (if flags_ == NO_STRING_ADD_FLAGS)
  // r6: sum of lengths.
  __ bind(&string_add_flat_result);
  if (flags_ != NO_STRING_ADD_FLAGS) {
    __ ldr(r4, FieldMemOperand(r0, HeapObject::kMapOffset));
    __ ldr(r5, FieldMemOperand(r1, HeapObject::kMapOffset));
    __ ldrb(r4, FieldMemOperand(r4, Map::kInstanceTypeOffset));
    __ ldrb(r5, FieldMemOperand(r5, Map::kInstanceTypeOffset));
  }
  // Check that both strings are sequential.
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(r4, Immediate(kStringRepresentationMask));
  __ b(ne, &string_add_runtime);
  __ tst(r5, Immediate(kStringRepresentationMask));
  __ b(ne, &string_add_runtime);
  // Now check if both strings have the same encoding (ASCII/Two-byte).
  // r0: first string.
  // r1: second string.
  // r2: length of first string.
  // r3: length of second string.
  // r6: sum of lengths..
  Label non_ascii_string_add_flat_result;
  ASSERT(IsPowerOf2(kStringEncodingMask));  // Just one bit to test.
  __ eor(r7, r4, r5);
  __ tst(r7, Immediate(kStringEncodingMask));
  __ b(ne, &string_add_runtime);
  // And see if it's ASCII or two-byte.
  __ tst(r4, Immediate(kStringEncodingMask));
  __ b(eq, &non_ascii_string_add_flat_result);

  // Both strings are sequential ASCII strings. We also know that they are
  // short (since the sum of the lengths is less than kMinNonFlatLength).
  // r6: length of resulting flat string
  __ AllocateAsciiString(r7, r6, r4, r5, r9, &string_add_runtime);
  // Locate first character of result.
  __ add(r6, r7, Immediate(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  // Locate first character of first argument.
  __ add(r0, r0, Immediate(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  // r0: first character of first string.
  // r1: second string.
  // r2: length of first string.
  // r3: length of second string.
  // r6: first character of result.
  // r7: result string.
  StringHelper::GenerateCopyCharacters(masm, r6, r0, r2, r4, true);

  // Load second argument and locate first character.
  __ add(r1, r1, Immediate(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  // r1: first character of second string.
  // r3: length of second string.
  // r6: next character of result.
  // r7: result string.
  StringHelper::GenerateCopyCharacters(masm, r6, r1, r3, r4, true);
  __ mov(r0, r7);
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Immediate(2 * kPointerSize));
  __ Ret();

  __ bind(&non_ascii_string_add_flat_result);
  // Both strings are sequential two byte strings.
  // r0: first string.
  // r1: second string.
  // r2: length of first string.
  // r3: length of second string.
  // r6: sum of length of strings.
  __ AllocateTwoByteString(r7, r6, r4, r5, r9, &string_add_runtime);
  // r0: first string.
  // r1: second string.
  // r2: length of first string.
  // r3: length of second string.
  // r7: result string.

  // Locate first character of result.
  __ add(r6, r7, Immediate(SeqTwoByteString::kHeaderSize - kHeapObjectTag));
  // Locate first character of first argument.
  __ add(r0, r0, Immediate(SeqTwoByteString::kHeaderSize - kHeapObjectTag));

  // r0: first character of first string.
  // r1: second string.
  // r2: length of first string.
  // r3: length of second string.
  // r6: first character of result.
  // r7: result string.
  StringHelper::GenerateCopyCharacters(masm, r6, r0, r2, r4, false);

  // Locate first character of second argument.
  __ add(r1, r1, Immediate(SeqTwoByteString::kHeaderSize - kHeapObjectTag));

  // r1: first character of second string.
  // r3: length of second string.
  // r6: next character of result (after copy of first string).
  // r7: result string.
  StringHelper::GenerateCopyCharacters(masm, r6, r1, r3, r4, false);

  __ mov(r0, r7);
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Immediate(2 * kPointerSize));
  __ Ret();

  // Just jump to runtime to add the two strings.
  __ bind(&string_add_runtime);
  __ TailCallRuntime(Runtime::kStringAdd, 2, 1);

  if (call_builtin.is_linked()) {
    __ bind(&call_builtin);
    __ InvokeBuiltin(builtin_id, JUMP_JS);
  }
}


void StringAddStub::GenerateConvertArgument(MacroAssembler* masm,
                                            int stack_offset,
                                            Register arg,
                                            Register scratch1,
                                            Register scratch2,
                                            Register scratch3,
                                            Register scratch4,
                                            Label* slow) {
  // First check if the argument is already a string.
  Label not_string, done;
  __ JumpIfSmi(arg, &not_string);
  __ CompareObjectType(arg, scratch1, scratch1, FIRST_NONSTRING_TYPE, ge);
  __ bf(&done);

  // Check the number to string cache.
  Label not_cached;
  __ bind(&not_string);
  // Puts the cached result into scratch1.
  NumberToStringStub::GenerateLookupNumberStringCache(masm,
                                                      arg,
                                                      scratch1,
                                                      scratch2,
                                                      scratch3,
                                                      scratch4,
                                                      false,
                                                      &not_cached);
  __ mov(arg, scratch1);
  __ str(arg, MemOperand(sp, stack_offset));
  __ jmp(&done);

  // Check if the argument is a safe string wrapper.
  __ bind(&not_cached);
  __ JumpIfSmi(arg, slow);
  __ CompareObjectType(
	arg, scratch1, scratch2, JS_VALUE_TYPE, eq);  // map -> scratch1.
  __ bf(slow);
  __ ldrb(scratch2, FieldMemOperand(scratch1, Map::kBitField2Offset));
  __ land(scratch2,
          scratch2, Immediate(1 << Map::kStringWrapperSafeForDefaultValueOf));
  __ cmp(scratch2,
         Immediate(1 << Map::kStringWrapperSafeForDefaultValueOf));
  __ b(ne, slow);
  __ ldr(arg, FieldMemOperand(arg, JSValue::kValueOffset));
  __ str(arg, MemOperand(sp, stack_offset));

  __ bind(&done);
}


void ICCompareStub::GenerateSmis(MacroAssembler* masm) {
  ASSERT(state_ == CompareIC::SMIS);
  Label miss;
  __ lor(r2, r1, r0);
  __ tst(r2, Immediate(kSmiTagMask));
  __ b(ne, &miss);

  if (GetCondition() == eq) {
    // For equality we do not care about the sign of the result.
    __ sub(r0, r0, r1);
    __ tst(r0, r0);
  } else {
    // Untag before subtracting to avoid handling overflow.
    __ SmiUntag(r1);
    __ asr(ip, r0, Immediate(kSmiTagSize));
    __ sub(r0, r1, ip);
  }
  __ Ret();

  __ bind(&miss);
  GenerateMiss(masm);
}


void ICCompareStub::GenerateHeapNumbers(MacroAssembler* masm) {
  ASSERT(state_ == CompareIC::HEAP_NUMBERS);

  Label generic_stub;
  Label unordered;
  Label miss;
  __ land(r2, r1, r0);
  __ tst(r2, Immediate(kSmiTagMask));
  __ b(eq, &generic_stub);

  __ CompareObjectType(r0, r2, r2, HEAP_NUMBER_TYPE, eq);
  __ b(ne, &miss);
  __ CompareObjectType(r1, r2, r2, HEAP_NUMBER_TYPE, eq);
  __ b(ne, &miss);

  // Inlining the double comparison and falling back to the general compare
  // stub if NaN is involved or VFP3 is unsupported.
//TODO: VFP
//  if (CpuFeatures::IsSupported(VFP3)) {
//    CpuFeatures::Scope scope(VFP3);
//
//    // Load left and right operand
//    __ sub(r2, r1, Operand(kHeapObjectTag));
//    __ vldr(d0, r2, HeapNumber::kValueOffset);
//    __ sub(r2, r0, Operand(kHeapObjectTag));
//    __ vldr(d1, r2, HeapNumber::kValueOffset);
//
//    // Compare operands
//    __ VFPCompareAndSetFlags(d0, d1);
//
//    // Don't base result on status bits when a NaN is involved.
//    __ b(vs, &unordered);
//
//    // Return a result of -1, 0, or 1, based on status bits.
//    __ mov(r0, Operand(EQUAL), LeaveCC, eq);
//    __ mov(r0, Operand(LESS), LeaveCC, lt);
//    __ mov(r0, Operand(GREATER), LeaveCC, gt);
//    __ Ret();
//
//    __ bind(&unordered);
//  }

  CompareStub stub(GetCondition(), strict(), NO_COMPARE_FLAGS, r1, r0);
  __ bind(&generic_stub);
  __ Jump(stub.GetCode(), RelocInfo::CODE_TARGET);

  __ bind(&miss);
  GenerateMiss(masm);
}


void ICCompareStub::GenerateMiss(MacroAssembler* masm) {
  __ Push(r1, r0);
  __ push(lr);

  // Call the runtime system in a fresh internal frame.
  ExternalReference miss =
      ExternalReference(IC_Utility(IC::kCompareIC_Miss), masm->isolate());
  __ EnterInternalFrame();
  __ Push(r1, r0);
  __ mov(ip, Immediate(Smi::FromInt(op_)));
  __ push(ip);
  __ CallExternalReference(miss, 3);
  __ LeaveInternalFrame();
  // Compute the entry point of the rewritten stub.
  __ add(r2, r0, Immediate(Code::kHeaderSize - kHeapObjectTag));
  // Restore registers.
  __ pop(lr);
  __ pop(r0);
  __ pop(r1);
  __ jmp(r2);
}


#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
