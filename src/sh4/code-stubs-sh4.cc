// Copyright 2011-2012 the V8 project authors. All rights reserved.
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
#include "regexp-macro-assembler.h"

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
#include "map-sh4.h"  // Define register map
// Check if the operand is a heap number.
static void EmitCheckForHeapNumber(MacroAssembler* masm, Register operand,
                                   Register scratch1, Register scratch2,
                                   Label* not_a_heap_number) {
  __ ldr(scratch1, FieldMemOperand(operand, HeapObject::kMapOffset));
  __ LoadRoot(scratch2, Heap::kHeapNumberMapRootIndex);
  __ cmp(scratch1, scratch2);
  __ b(ne, not_a_heap_number);
}


void ToNumberStub::Generate(MacroAssembler* masm) {
  // The ToNumber stub takes one argument in eax.
  // Entry argument: r0
  // Exit in: r0
#ifdef DEBUG
  // Clobber other parameter registers on entry.
  __ Dead(r1, r2, r3);
  __ Dead(r4, r5, r6, r7);
#endif
  Label check_heap_number, call_builtin;
  __ JumpIfNotSmi(r0, &check_heap_number, Label::kNear);
  __ Ret();

  __ bind(&check_heap_number);
  EmitCheckForHeapNumber(masm, r0, r1, ip, &call_builtin);
  __ Ret();

  __ bind(&call_builtin);
  __ push(r0);
  __ InvokeBuiltin(Builtins::TO_NUMBER, JUMP_FUNCTION);
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
  __ add(r3, r3, Operand(Code::kHeaderSize - kHeapObjectTag));
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
  __ LoadRoot(r2, Heap::kFunctionContextMapRootIndex);
  __ str(r2, FieldMemOperand(r0, HeapObject::kMapOffset));
  __ mov(r2, Operand(Smi::FromInt(length)));
  __ str(r2, FieldMemOperand(r0, FixedArray::kLengthOffset));

  // Setup the fixed slots.
  __ mov(r1, Operand(Smi::FromInt(0)));
  __ str(r3, MemOperand(r0, Context::SlotOffset(Context::CLOSURE_INDEX)));
  __ str(cp, MemOperand(r0, Context::SlotOffset(Context::PREVIOUS_INDEX)));
  __ str(r1, MemOperand(r0, Context::SlotOffset(Context::EXTENSION_INDEX)));

  // Copy the global object from the previous context.
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
  __ TailCallRuntime(Runtime::kNewFunctionContext, 1, 1);
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
  __ add(r3, r3, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  __ lsl(ip, r0, Operand(kPointerSizeLog2 - kSmiTagSize));
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
    __ add(r2, r0, Operand(JSArray::kSize));
    __ str(r2, FieldMemOperand(r0, JSArray::kElementsOffset));

    // Copy the elements array.
    __ CopyFields(r2, r3, r1.bit(), elements_size / kPointerSize);
  }

  // Return and remove the on-stack parameters.
  __ add(sp, sp, Operand(3 * kPointerSize));
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
};


void ConvertToDoubleStub::Generate(MacroAssembler* masm) {
  ASSERT(!result1_.is(ip) && !result2_.is(ip) && !zeros_.is(ip));
  Register exponent = result1_;
  Register mantissa = result2_;

  Label not_special;
  // Convert from Smi to integer.
  __ asr(source_, source_, Operand(kSmiTagSize));
  // Move sign bit from source to destination.  This works because the sign bit
  // in the exponent word of the double has the same position and polarity as
  // the 2's complement sign bit in a Smi.
  STATIC_ASSERT(HeapNumber::kSignMask == 0x80000000u);
  __ land(exponent, source_, Operand(HeapNumber::kSignMask));
  __ tst(exponent, exponent);
  // Subtract from 0 if source was negative.
  __ rsb(source_, source_, Operand(0), ne);

  // We have -1, 0 or 1, which we treat specially. Register source_ contains
  // absolute value: it is either equal to 1 (special case of -1 and 1),
  // greater than 1 (not a special case) or less than 1 (special case of 0).
  __ cmpgt(source_, Operand(1));
  __ bt_near(&not_special);

  // For 1 or -1 we need to or in the 0 exponent (biased to 1023).
  static const uint32_t exponent_word_for_1 =
      HeapNumber::kExponentBias << HeapNumber::kExponentShift;
  __ cmpeq(source_, Operand(1));
  __ orr(exponent, exponent, Operand(exponent_word_for_1), eq);
  // 1, 0 and -1 all have 0 for the second word.
  __ mov(mantissa, Operand(0));
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
  __ rsb(mantissa, zeros_, Operand(31 + HeapNumber::kExponentBias - fudge));
  __ add(mantissa, mantissa, Operand(fudge));
  __ lsl(ip, mantissa, Operand(HeapNumber::kExponentShift));
  __ orr(exponent, exponent, ip);
  // Shift up the source chopping the top bit off.
  __ add(zeros_, zeros_, Operand(1));
  // This wouldn't work for 1.0 or -1.0 as the shift would be 32 which means 0.
  __ lsl(source_, source_, zeros_);
  // Compute lower part of fraction (last 12 bits).
  __ lsl(mantissa, source_, Operand(HeapNumber::kMantissaBitsInTopWord));
  // And the top (top 20 bits).
  __ lsr(ip, source_, Operand(32 - HeapNumber::kMantissaBitsInTopWord));
  __ orr(exponent, exponent, ip);
  __ Ret();
}


void FloatingPointHelper::LoadSmis(MacroAssembler* masm,
                                   FloatingPointHelper::Destination destination,
                                   Register scratch1,
                                   Register scratch2) {
  if (CpuFeatures::IsSupported(FPU)) {
    __ asr(scratch1, r0, Operand(kSmiTagSize));
    __ dfloat(dr2, scratch1);
    __ asr(scratch1, r1, Operand(kSmiTagSize));
    __ dfloat(dr0, scratch1);
    if (destination == kCoreRegisters) {
      __ movd(r2, r3, dr2);
      __ movd(r0, r1, dr0);
    }
  } else {
    ASSERT(destination == kCoreRegisters);
    // Write Smi from r0 to r3 and r2 in double format.
    __ mov(scratch1, r0);
    ConvertToDoubleStub stub1(r3, r2, scratch1, scratch2);
    __ push(lr);
    __ Call(stub1.GetCode());
    // Write Smi from r1 to r1 and r0 in double format.
    __ mov(scratch1, r1);
    ConvertToDoubleStub stub2(r1, r0, scratch1, scratch2);
    __ Call(stub2.GetCode());
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

  // Load right operand (r0) to d7 or r2/r3.
  ASSERT(!heap_number_map.is(r0) && !heap_number_map.is(r1) &&
         !heap_number_map.is(r2) && !heap_number_map.is(r3));
  LoadNumber(masm, destination,
             r0, dr2, r2, r3, heap_number_map, scratch1, scratch2, slow);

  // Load left operand (r1) to d6 or r0/r1.
  LoadNumber(masm, destination,
             r1, dr0, r0, r1, heap_number_map, scratch1, scratch2, slow);
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

  __ JumpIfSmi(object, &is_smi, Label::kNear);
  __ JumpIfNotHeapNumber(object, heap_number_map, scratch1, not_number);

  // Handle loading a double from a heap number.
  if (CpuFeatures::IsSupported(FPU) &&
      destination == kVFPRegisters) {
    // Load the double from tagged HeapNumber to double register.
    __ sub(scratch1, object, Operand(kHeapObjectTag));
    __ dldr(dst, MemOperand(scratch1, HeapNumber::kValueOffset));
  } else {
    ASSERT(destination == kCoreRegisters);
    // Load the double from heap number to dst1 and dst2 in double format.
    __ Ldrd(dst1, dst2, FieldMemOperand(object, HeapNumber::kValueOffset));
  }
  __ jmp_near(&done);

  // Handle loading a double from a smi.
  __ bind(&is_smi);
  if (CpuFeatures::IsSupported(FPU)) {
    // Convert smi to double using FPU instructions.
    __ SmiUntag(scratch1, object);
    __ dfloat(dst, scratch1);
    if (destination == kCoreRegisters) {
      // Load the converted smi to dst1 and dst2 in double format.
      __ movd(dst1, dst2, dst);
    }
  } else {
    ASSERT(destination == kCoreRegisters);
    // Write smi to dst1 and dst2 double format.
    __ mov(scratch1, object);
    // TODO(stm): check order of dst2, dst1
    ConvertToDoubleStub stub(dst2, dst1, scratch1, scratch2);
    __ push(lr);
    __ Call(stub.GetCode());
    __ pop(lr);
  }

  __ bind(&done);
}


void FloatingPointHelper::ConvertNumberToInt32(MacroAssembler* masm,
                                               Register object,
                                               Register dst,
                                               Register heap_number_map,
                                               Register scratch1,
                                               Register scratch2,
                                               Register scratch3,
                                               DwVfpRegister double_scratch,
                                               Label* not_number) {
  if (FLAG_debug_code) {
    __ AbortIfNotRootValue(heap_number_map,
                           Heap::kHeapNumberMapRootIndex,
                           "HeapNumberMap register clobbered.");
  }
  Label is_smi;
  Label done;
  Label not_in_int32_range;

  __ JumpIfSmi(object, &is_smi);
  __ ldr(scratch1, FieldMemOperand(object, HeapNumber::kMapOffset));
  __ cmp(scratch1, heap_number_map);
  __ b(ne, not_number);
  __ ConvertToInt32(object,
                    dst,
                    scratch1,
                    scratch2,
                    double_scratch,
                    &not_in_int32_range);
  __ jmp(&done);

  __ bind(&not_in_int32_range);
  __ ldr(scratch1, FieldMemOperand(object, HeapNumber::kExponentOffset));
  __ ldr(scratch2, FieldMemOperand(object, HeapNumber::kMantissaOffset));

  __ EmitOutOfInt32RangeTruncate(dst,
                                 scratch1,
                                 scratch2,
                                 scratch3);
  __ jmp(&done);

  __ bind(&is_smi);
  __ SmiUntag(dst, object);
  __ bind(&done);
}


void FloatingPointHelper::ConvertIntToDouble(MacroAssembler* masm,
                                             Register int_scratch,
                                             Destination destination,
                                             DwVfpRegister double_dst,
                                             Register dst1,
                                             Register dst2,
                                             Register scratch2,
                                             SwVfpRegister single_scratch) {
  ASSERT(single_scratch.is(no_freg));

  ASSERT(!int_scratch.is(scratch2));
  ASSERT(!int_scratch.is(dst1));
  ASSERT(!int_scratch.is(dst2));

  Label done;

  if (CpuFeatures::IsSupported(VFP3)) {
    __ dfloat(double_dst, int_scratch);
    if (destination == kCoreRegisters) {
      __ movd(dst1, dst2, double_dst);
    }
  } else {
    Label fewer_than_20_useful_bits;
    // Expected output:
    // |         dst2            |         dst1            |
    // | s |   exp   |              mantissa               |

    // Check for zero.
    __ cmp(int_scratch, Operand(0));
    __ mov(dst2, int_scratch);
    __ mov(dst1, int_scratch);
    __ b(eq, &done);

    // Preload the sign of the value.
    __ land(dst2, int_scratch, Operand(HeapNumber::kSignMask));
    // Get the absolute value of the object (as an unsigned integer).
    __ cmpge(dst2, Operand(0));
    __ rsb(ip, int_scratch, Operand(0));
    __ mov(int_scratch, ip, f);

    // Get mantisssa[51:20].

    // Get the position of the first set bit.
    __ CountLeadingZeros(dst1, int_scratch, scratch2);
    __ rsb(dst1, dst1, Operand(31));

    // Set the exponent.
    __ add(scratch2, dst1, Operand(HeapNumber::kExponentBias));
    __ Bfi(dst2, scratch2, scratch2,
        HeapNumber::kExponentShift, HeapNumber::kExponentBits);

    // Clear the first non null bit.
    __ mov(scratch2, Operand(1));
    __ lsl(scratch2, scratch2, dst1);
    __ bic(int_scratch, int_scratch, scratch2);

    // __ cmp(dst1, Operand(HeapNumber::kMantissaBitsInTopWord)); // Present on ARM, but dead code.
    // Get the number of bits to set in the lower part of the mantissa.
    __ sub(scratch2, dst1, Operand(HeapNumber::kMantissaBitsInTopWord));
    __ cmpge(scratch2, Operand(0));
    __ b(f, &fewer_than_20_useful_bits, Label::kNear);
    // Set the higher 20 bits of the mantissa.
    __ lsr(ip, int_scratch, scratch2);
    __ orr(dst2, dst2, ip);
    __ rsb(scratch2, scratch2, Operand(32));
    __ lsl(dst1, int_scratch, scratch2);
    __ b(&done);

    __ bind(&fewer_than_20_useful_bits);
    __ rsb(scratch2, dst1, Operand(HeapNumber::kMantissaBitsInTopWord));
    __ lsl(scratch2, int_scratch, scratch2);
    __ orr(dst2, dst2, scratch2);
    // Set dst1 to 0.
    __ mov(dst1, Operand(0));
  }
  __ bind(&done);
}


void FloatingPointHelper::LoadNumberAsInt32Double(MacroAssembler* masm,
                                                  Register object,
                                                  Destination destination,
                                                  DwVfpRegister double_dst,
                                                  Register dst1,
                                                  Register dst2,
                                                  Register heap_number_map,
                                                  Register scratch1,
                                                  Register scratch2,
                                                  SwVfpRegister single_scratch,
                                                  Label* not_int32) {
  // TODO(stm): remove these asserts when the FPU is used
  ASSERT(single_scratch.is(no_freg));

  ASSERT(!scratch1.is(object) && !scratch2.is(object));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!heap_number_map.is(object) &&
         !heap_number_map.is(scratch1) &&
         !heap_number_map.is(scratch2));

  Label done, obj_is_not_smi;

  __ JumpIfNotSmi(object, &obj_is_not_smi);
  __ SmiUntag(scratch1, object);
  ConvertIntToDouble(masm, scratch1, destination, double_dst, dst1, dst2,
                     scratch2, single_scratch);
  __ b(&done);

  __ bind(&obj_is_not_smi);
  if (FLAG_debug_code) {
    __ AbortIfNotRootValue(heap_number_map,
                           Heap::kHeapNumberMapRootIndex,
                           "HeapNumberMap register clobbered.");
  }
  __ JumpIfNotHeapNumber(object, heap_number_map, scratch1, not_int32);

  // Load the number.
  // TODO(stm): FPU
  // if (CpuFeatures::IsSupported(VFP3)) {
  // } else
  {
    ASSERT(!scratch1.is(object) && !scratch2.is(object));
    // Load the double value in the destination registers..
    __ Ldrd(dst1, dst2, FieldMemOperand(object, HeapNumber::kValueOffset));

    // Check for 0 and -0.
    __ bic(scratch1, dst1, Operand(HeapNumber::kSignMask));
    __ orr(scratch1, scratch1, dst2);
    __ cmp(scratch1, Operand(0));
    __ b(eq, &done);

    // Check that the value can be exactly represented by a 32-bit integer.
    // Jump to not_int32 if that's not the case.
    DoubleIs32BitInteger(masm, dst1, dst2, scratch1, scratch2, not_int32);

    // dst1 and dst2 were trashed. Reload the double value.
    __ Ldrd(dst1, dst2, FieldMemOperand(object, HeapNumber::kValueOffset));
  }

  __ bind(&done);
}


void FloatingPointHelper::LoadNumberAsInt32(MacroAssembler* masm,
                                            Register object,
                                            Register dst,
                                            Register heap_number_map,
                                            Register scratch1,
                                            Register scratch2,
                                            Register scratch3,
                                            DwVfpRegister double_scratch,
                                            Label* not_int32) {
  // TODO(stm): remove these asserts when the FPU is used
  ASSERT(double_scratch.is(no_dreg));
  ASSERT(!dst.is(object));
  ASSERT(!scratch1.is(object) && !scratch2.is(object) && !scratch3.is(object));
  ASSERT(!scratch1.is(scratch2) &&
         !scratch1.is(scratch3) &&
         !scratch2.is(scratch3));

  Label done;

  // Untag the object into the destination register.
  __ SmiUntag(dst, object);
  // Just return if the object is a smi.
  __ JumpIfSmi(object, &done);

  if (FLAG_debug_code) {
    __ AbortIfNotRootValue(heap_number_map,
                           Heap::kHeapNumberMapRootIndex,
                           "HeapNumberMap register clobbered.");
  }
  __ JumpIfNotHeapNumber(object, heap_number_map, scratch1, not_int32);

  // Object is a heap number.
  // Convert the floating point value to a 32-bit integer.
  // TODO(stm): FPU
  // if (CpuFeatures::IsSupported(VFP3)) {
  // } else
  {
    // Load the double value in the destination registers.
    __ ldr(scratch1, FieldMemOperand(object, HeapNumber::kExponentOffset));
    __ ldr(scratch2, FieldMemOperand(object, HeapNumber::kMantissaOffset));

    // Check for 0 and -0.
    __ bic(dst, scratch1, Operand(HeapNumber::kSignMask));
    __ orr(dst, scratch2, dst);
    __ cmp(dst, Operand(0));
    __ b(eq, &done);

    DoubleIs32BitInteger(masm, scratch1, scratch2, dst, scratch3, not_int32);

    // Registers state after DoubleIs32BitInteger.
    // dst: mantissa[51:20].
    // scratch2: 1

    // Shift back the higher bits of the mantissa.
    __ lsr(dst, dst, scratch3);
    // Set the implicit first bit.
    __ rsb(scratch3, scratch3, Operand(32));
    __ lsl(scratch2, scratch2, scratch3);
    __ orr(dst, dst, scratch2);
    // Set the sign.
    __ ldr(scratch1, FieldMemOperand(object, HeapNumber::kExponentOffset));
    __ tst(scratch1, Operand(HeapNumber::kSignMask));
    __ rsb(ip, dst, Operand(0));
    __ mov(dst, ip, ne);        // FIXME(stm): strange case !!
  }

  __ bind(&done);
}


void FloatingPointHelper::DoubleIs32BitInteger(MacroAssembler* masm,
                                               Register src1,
                                               Register src2,
                                               Register dst,
                                               Register scratch,
                                               Label* not_int32) {
  // Get exponent alone in scratch.
  __ Ubfx(scratch,
          src1,
          HeapNumber::kExponentShift,
          HeapNumber::kExponentBits);

  // Substract the bias from the exponent.
  __ sub(scratch, scratch, Operand(HeapNumber::kExponentBias));

  // src1: higher (exponent) part of the double value.
  // src2: lower (mantissa) part of the double value.
  // scratch: unbiased exponent.

  // Fast cases. Check for obvious non 32-bit integer values.
  // Negative exponent cannot yield 32-bit integers.
  __ cmpge(scratch, Operand(0));
  __ b(f, not_int32);
  // Exponent greater than 31 cannot yield 32-bit integers.
  // Also, a positive value with an exponent equal to 31 is outside of the
  // signed 32-bit integer range.
  // Another way to put it is that if (exponent - signbit) > 30 then the
  // number cannot be represented as an int32.
  Register tmp = dst;
  __ lsr(tmp, src1, Operand(31));
  __ sub(tmp, scratch, tmp);
  __ cmpgt(tmp, Operand(30));
  __ b(t, not_int32);
  // - Bits [21:0] in the mantissa are not null.
  __ tst(src2, Operand(0x3fffff));
  __ b(ne, not_int32);

  // Otherwise the exponent needs to be big enough to shift left all the
  // non zero bits left. So we need the (30 - exponent) last bits of the
  // 31 higher bits of the mantissa to be null.
  // Because bits [21:0] are null, we can check instead that the
  // (32 - exponent) last bits of the 32 higher bits of the mantisssa are null.

  // Get the 32 higher bits of the mantissa in dst.
  __ Ubfx(dst,
          src2,
          HeapNumber::kMantissaBitsInTopWord,
          32 - HeapNumber::kMantissaBitsInTopWord);
  __ lsl(ip, src1, Operand(HeapNumber::kNonMantissaBitsInTopWord));
  __ orr(dst,
         dst,
         ip);

  // Create the mask and test the lower bits (of the higher bits).
  __ rsb(scratch, scratch, Operand(32));
  __ mov(src2, Operand(1));
  __ lsl(src1, src2, scratch);
  __ sub(src1, src1, Operand(1));
  __ tst(dst, src1);
  __ b(ne, not_int32);
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
  // Note: as r5 is not callee-saved on SH$, we push/pop it below
  ASSERT(heap_number_result.is(r5));

  // Calling C function (using double registers): move r0..r3 to fr4..fr7
  __ movd(dr4, r0, r1);
  __ movd(dr6, r2, r3);

  // Push the current return address before the C call. Return will be
  // through pop(pc) below.
  __ push(lr);
  __ push(heap_number_result); // sh4 specific
  __ PrepareCallCFunction(0, 2, r0); /* Use r0 as scratch: PrepareCallCFunction() disallow use of r4-r7 on sh4. */
  // Call C routine that may not cause GC or other trouble.
  __ CallCFunction(ExternalReference::double_fp_operation(op, masm->isolate()),
                   0, 2);
  __ movd(r0, r1, dr0);
  // Store answer in the overwritable heap number. Double returned in
  // registers r0 and r1 or in d0.
  __ pop(heap_number_result); // sh4 specific
  __ Strd(r0, r1, FieldMemOperand(heap_number_result,
                                  HeapNumber::kValueOffset));
  // Place heap_number_result in r0 and return to the pushed return address.
  __ mov(r0, heap_number_result);
  __ pop(lr);
  __ rts();
}


// See comment for class.
void WriteInt32ToHeapNumberStub::Generate(MacroAssembler* masm) {
  ASSERT(!scratch_.is(ip) && !the_int_.is(ip));
  Label max_negative_int;
  // the_int_ has the answer which is a signed int32 but not a Smi.
  // We test for the special value that has a different exponent.  This test
  // has the neat side effect of setting the flags according to the sign.
  STATIC_ASSERT(HeapNumber::kSignMask == 0x80000000u);
  __ cmp(the_int_, Operand(0x80000000u));
  __ b(eq, &max_negative_int, Label::kNear);
  // Set up the correct exponent in scratch_.  All non-Smi int32s have the same.
  // A non-Smi integer is 1.xxx * 2^30 so the exponent is 30 (biased).
  uint32_t non_smi_exponent =
      (HeapNumber::kExponentBias + 30) << HeapNumber::kExponentShift;
  __ mov(scratch_, Operand(non_smi_exponent));
  __ cmpge(the_int_, Operand(0));
  Label skip;
  __ bt_near(&skip);
  // Set the sign bit in scratch_ if the value was negative.
  __ lor(scratch_, scratch_, Operand(HeapNumber::kSignMask));
  // Subtract from 0 if the value was negative.
  __ rsb(the_int_, the_int_, Operand(0));
  __ bind(&skip);
  // We should be masking the implict first digit of the mantissa away here,
  // but it just ends up combining harmlessly with the last digit of the
  // exponent that happens to be 1.  The sign bit is 0 so we shift 10 to get
  // the most significant 1 to hit the last bit of the 12 bit sign and exponent.
  ASSERT(((1 << HeapNumber::kExponentShift) & non_smi_exponent) != 0);
  const int shift_distance = HeapNumber::kNonMantissaBitsInTopWord - 2;
  __ lsr(ip, the_int_, Operand(shift_distance));
  __ lor(scratch_, scratch_, ip);
  __ str(scratch_, FieldMemOperand(the_heap_number_,
                                   HeapNumber::kExponentOffset));
  __ lsl(scratch_, the_int_, Operand(32 - shift_distance));
  __ str(scratch_, FieldMemOperand(the_heap_number_,
                                   HeapNumber::kMantissaOffset));
  __ Ret();

  __ bind(&max_negative_int);
  // The max negative int32 is stored as a positive number in the mantissa of
  // a double because it uses a sign bit instead of using two's complement.
  // The actual mantissa bits stored are all 0 because the implicit most
  // significant 1 bit is not stored.
  non_smi_exponent += 1 << HeapNumber::kExponentShift;
  __ mov(ip, Operand(HeapNumber::kSignMask | non_smi_exponent));
  __ str(ip, FieldMemOperand(the_heap_number_, HeapNumber::kExponentOffset));
  __ mov(ip, Operand(0));
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
      __ CompareObjectType(r0, r4, r4, FIRST_SPEC_OBJECT_TYPE, ge);
      __ bt(slow);
    } else {
      __ CompareObjectType(r0, r4, r4, HEAP_NUMBER_TYPE, eq);
      __ b(eq, &heap_number, Label::kNear);
      // Comparing JS objects with <=, >= is complicated.
      if (cond != eq) {
        __ cmpge(r4, Operand(FIRST_SPEC_OBJECT_TYPE));
        __ bt(slow);
        // Normally here we fall through to return_equal, but undefined is
        // special: (undefined == undefined) == true, but
        // (undefined <= undefined) == false!  See ECMAScript 11.8.5.
        if (cond == le || cond == ge) {
          __ cmp(r4, Operand(ODDBALL_TYPE));
          __ b(ne, &return_equal, Label::kNear);
          __ LoadRoot(r2, Heap::kUndefinedValueRootIndex);
          __ cmp(r0, r2);
          __ b(ne, &return_equal, Label::kNear);
          if (cond == le) {
            // undefined <= undefined should fail.
            __ mov(r0, Operand(GREATER));
          } else  {
            // undefined >= undefined should fail.
            __ mov(r0, Operand(LESS));
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
      __ cmp(r3, Operand(-1));
      __ b(ne, &return_equal);

      // Shift out flag and all exponent bits, retaining only mantissa.
      __ lsl(r2, r2, Operand(HeapNumber::kNonMantissaBitsInTopWord));
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
  __ JumpIfSmi(rhs, &rhs_is_smi, Label::kNear);

  // Lhs is a Smi.  Check whether the rhs is a heap number.
  __ CompareObjectType(rhs, r4, r4, HEAP_NUMBER_TYPE, eq);
  if (strict) {
    // If rhs is not a number and lhs is a Smi then strict equality cannot
    // succeed.  Return non-equal
    // If rhs is r0 then there is already a non zero value in it.
    if (!rhs.is(r0)) {
      __ mov(r0, Operand(NOT_EQUAL), ne);
    }
    __ Ret(ne);
  } else {
    // Smi compared non-strictly with a non-Smi non-heap-number.  Call
    // the runtime.
    __ b(ne, slow);
  }

  // Lhs is a smi, rhs is a number.
  // TODO(stm): FPU
  // if (CpuFeatures::IsSupported(VFP3)) {
  // } else
  {
    __ push(lr);
    // Convert lhs to a double in r2, r3.
    __ mov(r7, lhs);
    ConvertToDoubleStub stub1(r3, r2, r7, r6);
    __ Call(stub1.GetCode());
    // Load rhs to a double in r0, r1.
    __ Ldrd(r0, r1, FieldMemOperand(rhs, HeapNumber::kValueOffset));
    __ pop(lr);
  }

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
      __ mov(r0, Operand(NOT_EQUAL), ne);
    }
    __ Ret(ne);
  } else {
    // Smi compared non-strictly with a non-smi non-heap-number.  Call
    // the runtime.
    __ b(ne, slow);
  }

  // Rhs is a smi, lhs is a heap number.
  // TODO(stm): FPU
  // if (CpuFeatures::IsSupported(VFP3)) {
  // } else
  {
    __ push(lr);
    // Load lhs to a double in r2, r3.
    __ Ldrd(r2, r3, FieldMemOperand(lhs, HeapNumber::kValueOffset));
    // Convert rhs to a double in r0, r1.
    __ mov(r7, rhs);
    ConvertToDoubleStub stub2(r1, r0, r7, r6);
    __ Call(stub2.GetCode());
    __ pop(lr);
  }
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
  __ cmp(r4, Operand(-1));
  __ b(ne, lhs_not_nan);
  __ lsl(r4, lhs_exponent, Operand(HeapNumber::kNonMantissaBitsInTopWord));
  __ cmpeq(r4, Operand(0));
  __ b(ne, &one_is_nan, Label::kNear);
  __ cmp(lhs_mantissa, Operand(0, RelocInfo::NONE));
  __ b(ne, &one_is_nan, Label::kNear);

  __ bind(lhs_not_nan);
  __ Sbfx(r4,
          rhs_exponent,
          HeapNumber::kExponentShift,
          HeapNumber::kExponentBits);
  // NaNs have all-one exponents so they sign extend to -1.
  __ cmp(r4, Operand(-1));
  __ b(ne, &neither_is_nan, Label::kNear);
  __ lsl(r4, rhs_exponent, Operand(HeapNumber::kNonMantissaBitsInTopWord));
  __ cmpeq(r4, Operand(0, RelocInfo::NONE));
  __ b(ne, &one_is_nan, Label::kNear);
  __ cmp(rhs_mantissa, Operand(0));
  __ b(eq, &neither_is_nan, Label::kNear);

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
    __ orr(r0, rhs_mantissa, lhs_mantissa, ne);
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
    __ lsl(r4, lhs_exponent, Operand(kSmiTagSize));
    __ orr(r4, lhs_mantissa, r4);
    __ tst(r4, r4);
    __ mov(r0, r4, ne);
    __ Ret(ne);
    // Now they are equal if and only if the lhs exponent is zero in its
    // low 31 bits.
    __ lsl(r0, rhs_exponent, Operand(kSmiTagSize));
    __ Ret();
  } else {
    __ Push(r4, r5, r6, r7);
    // Calling C function: move r0..r3 to fr4..fr7
    __ movd(dr4, r0, r1);
    __ movd(dr6, r2, r3);

    // Call a native function to do a comparison between two non-NaNs.
    // Call C routine that may not cause GC or other trouble.
    __ push(lr);
    __ PrepareCallCFunction(0, 2, r0);
    __ CallCFunction(ExternalReference::compare_doubles(masm->isolate()),
                     0, 2);
    __ pop(lr);
    __ Pop(r4, r5, r6, r7);
    __ Ret();
  }
}


// See comment at call site.
static void EmitStrictTwoHeapObjectCompare(MacroAssembler* masm,
                                           Register lhs,
                                           Register rhs) {
    ASSERT((lhs.is(r0) && rhs.is(r1)) ||
           (lhs.is(r1) && rhs.is(r0)));

    // If either operand is a JS object or an oddball value, then they are
    // not equal since their pointers are different.
    // There is no test for undetectability in strict equality.
    STATIC_ASSERT(LAST_TYPE == LAST_CALLABLE_SPEC_OBJECT_TYPE);
    Label first_non_object;
    // Get the type of the first operand into r2 and compare it with
    // FIRST_JS_OBJECT_TYPE.
    __ CompareObjectType(rhs, r2, r2, FIRST_SPEC_OBJECT_TYPE, ge);
    __ bf_near(&first_non_object);

    // Return non-zero (r0 is not zero)
    Label return_not_equal;
    __ bind(&return_not_equal);
    __ Ret();

    __ bind(&first_non_object);
    // Check for oddballs: true, false, null, undefined.
    __ cmp(r2, Operand(ODDBALL_TYPE));
    __ b(eq, &return_not_equal);

    __ CompareObjectType(lhs, r3, r3, FIRST_SPEC_OBJECT_TYPE, ge);
    __ bt(&return_not_equal);

    // Check for oddballs: true, false, null, undefined.
    __ cmp(r3, Operand(ODDBALL_TYPE));
    __ b(eq, &return_not_equal);

    // Now that we have the types we might as well check for symbol-symbol.
    // Ensure that no non-strings have the symbol bit set.
    STATIC_ASSERT(LAST_TYPE < kNotStringTag + kIsSymbolMask);
    STATIC_ASSERT(kSymbolTag != 0);
    __ land(r2, r2, r3);
    __ tst(r2, Operand(kIsSymbolMask));
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
  // TODO(stm): FPU
  // if (CpuFeatures::IsSupported(VFP3)) {
  // } else
  {
    __ Ldrd(r2, r3, FieldMemOperand(lhs, HeapNumber::kValueOffset));
    __ Ldrd(r0, r1, FieldMemOperand(rhs, HeapNumber::kValueOffset));
  }
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
  __ tst(r2, Operand(kIsNotStringMask));
  __ b(ne, &object_test, Label::kNear);
  __ tst(r2, Operand(kIsSymbolMask));
  __ b(eq, possible_strings);
  __ CompareObjectType(lhs, r3, r3, FIRST_NONSTRING_TYPE, ge);
  __ bt(not_both_strings);
  __ tst(r3, Operand(kIsSymbolMask));
  __ b(eq, possible_strings);

  // Both are symbols.  We already checked they weren't the same pointer
  // so they are not equal.
  __ mov(r0, Operand(NOT_EQUAL));
  __ Ret();

  __ bind(&object_test);
  __ cmpge(r2, Operand(FIRST_SPEC_OBJECT_TYPE));
  __ bf(not_both_strings);
  __ CompareObjectType(lhs, r2, r3, FIRST_SPEC_OBJECT_TYPE, ge);
  __ bf(not_both_strings);
  // If both objects are undetectable, they are equal. Otherwise, they
  // are not equal, since they are different objects and an object is not
  // equal to undefined.
  __ ldr(r3, FieldMemOperand(rhs, HeapObject::kMapOffset));
  __ ldrb(r2, FieldMemOperand(r2, Map::kBitFieldOffset));
  __ ldrb(r3, FieldMemOperand(r3, Map::kBitFieldOffset));
  __ land(r0, r2, r3);
  __ land(r0, r0, Operand(1 << Map::kIsUndetectable));
  __ eor(r0, r0, Operand(1 << Map::kIsUndetectable));
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
  __ asr(mask, mask, Operand(kSmiTagSize + 1));
  __ sub(mask, mask, Operand(1));  // Make mask.

  // Calculate the entry in the number string cache. The hash value in the
  // number string cache for smis is just the smi value, and the hash for
  // doubles is the xor of the upper and lower words. See
  // Heap::GetNumberStringCache.
  Isolate* isolate = masm->isolate();
  Label is_smi;
  Label load_result_from_cache;
  if (!object_is_smi) {
    __ JumpIfSmi(object, &is_smi, Label::kNear);

    // TODO(stm): FPU
    // if (CpuFeatures::IsSupported(VFP3)) {
    // } else
    {
      __ b(not_found);
    }
  }

  __ bind(&is_smi);
  Register scratch = scratch1;
  __ asr(scratch, object, Operand(1));
  __ land(scratch, mask, scratch);
  // Calculate address of entry in string cache: each entry consists
  // of two pointer sized fields.
  __ lsl(scratch, scratch, Operand(kPointerSizeLog2 + 1));
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
  __ add(sp, sp, Operand(1 * kPointerSize));
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
    __ JumpIfNotSmi(r2, &not_two_smis, Label::kNear);
    __ asr(r1, r1, Operand(1));
    __ asr(r0, r0, Operand(1));
    __ sub(r0, r1, r0);
    __ Ret();
    __ bind(&not_two_smis);
  } else if (FLAG_debug_code) {
    __ orr(r2, r1, r0);
    __ tst(r2, Operand(kSmiTagMask));
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
  __ JumpIfNotSmi(r2, &not_smis);
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
  // TODO(stm): FPU
  Isolate* isolate = masm->isolate();
  // if (CpuFeatures::IsSupported(VFP3)) {
  // } else
  {
    // Checks for NaN in the doubles we have loaded.  Can return the answer or
    // fall through if neither is a NaN.  Also binds lhs_not_nan.
    EmitNanCheck(masm, &lhs_not_nan, cc_);
    // Compares two doubles in r0, r1, r2, r3 that are not NaNs.  Returns the
    // answer.  Never falls through.
    EmitTwoNonNanDoubleComparison(masm, cc_);
  }

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
  if (cc_ == eq) {
    StringCompareStub::GenerateFlatAsciiStringEquals(masm,
                                                     lhs_,
                                                     rhs_,
                                                     r2,
                                                     r3,
                                                     r4);
  } else {
    StringCompareStub::GenerateCompareFlatAsciiStrings(masm,
                                                       lhs_,
                                                       rhs_,
                                                       r2,
                                                       r3,
                                                       r4,
                                                       r5);
  }
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
    __ mov(r0, Operand(Smi::FromInt(ncr)));
    __ push(r0);
  }

  // Call the native; it returns -1 (less), 0 (equal), or 1 (greater)
  // tagged as a small integer.
  __ InvokeBuiltin(native, JUMP_FUNCTION);
}


// The stub expects its argument in the tos_ register and returns its result in
// it, too: zero for false, and a non-zero value for true.
void ToBooleanStub::Generate(MacroAssembler* masm) {
  Label patch;
  const Register map = r9.is(tos_) ? r7 : r9;

  // undefined -> false.
  CheckOddball(masm, UNDEFINED, Heap::kUndefinedValueRootIndex, false);

  // Boolean -> its value.
  CheckOddball(masm, BOOLEAN, Heap::kFalseValueRootIndex, false);
  CheckOddball(masm, BOOLEAN, Heap::kTrueValueRootIndex, true);

  // 'null' -> false.
  CheckOddball(masm, NULL_TYPE, Heap::kNullValueRootIndex, false);

  if (types_.Contains(SMI)) {
    // Smis: 0 -> false, all other -> true
    __ tst(tos_, Operand(kSmiTagMask));
    // tos_ contains the correct return value already
    __ Ret(eq);
  } else if (types_.NeedsMap()) {
    // If we need a map later and have a Smi -> patch.
    __ JumpIfSmi(tos_, &patch);
  }

  if (types_.NeedsMap()) {
    __ ldr(map, FieldMemOperand(tos_, HeapObject::kMapOffset));

    if (types_.CanBeUndetectable()) {
      __ ldrb(ip, FieldMemOperand(map, Map::kBitFieldOffset));
      __ tst(ip, Operand(1 << Map::kIsUndetectable));
      // Undetectable -> false.
      Label skip;
      __ bt_near(&skip);
      __ mov(tos_, Operand(0, RelocInfo::NONE));
      __ rts();
      __ bind(&skip);
    }
  }

  if (types_.Contains(SPEC_OBJECT)) {
    // Spec object -> true.
    __ CompareInstanceType(map, ip, FIRST_SPEC_OBJECT_TYPE, ge);
    // tos_ contains the correct non-zero return value already.
    __ Ret(eq);
  }

  if (types_.Contains(STRING)) {
    // String value -> false iff empty.
  __ CompareInstanceType(map, ip, FIRST_NONSTRING_TYPE, ge);
  Label skip;
  __ bt_near(&skip);
  __ ldr(tos_, FieldMemOperand(tos_, String::kLengthOffset));
  __ rts();  // the string length is OK as the return value
  __ bind(&skip);
  }

  if (types_.Contains(HEAP_NUMBER)) {
    // Heap number -> false iff +0, -0, or NaN.
    Label not_heap_number;
    __ CompareRoot(map, Heap::kHeapNumberMapRootIndex);
    __ bf_near(&not_heap_number);
    __ dldr(dr0, FieldMemOperand(tos_, HeapNumber::kValueOffset));
    // "tos_" is a register, and contains a non zero value by default.
    // Hence we only need to overwrite "tos_" with zero to return false for
    // FP_ZERO or FP_NAN cases. Otherwise, by default it returns true.
    __ dfloat(dr2, Operand(0));
    __ dcmpeq(dr0, dr2);
    __ mov(tos_, Operand(0, RelocInfo::NONE), eq);  // for FP_ZERO
    __ dcmpeq(dr0, dr0);
    __ mov(tos_, Operand(0, RelocInfo::NONE), ne);  // for FP_NAN (dr0 != dr0 iff isnan(dr0))
    __ Ret();
    __ bind(&not_heap_number);
  }

  __ bind(&patch);
  GenerateTypeTransition(masm);
}


void ToBooleanStub::CheckOddball(MacroAssembler* masm,
                                 Type type,
                                 Heap::RootListIndex value,
                                 bool result) {
  if (types_.Contains(type)) {
    // If we see an expected oddball, return its ToBoolean value tos_.
    __ LoadRoot(ip, value);
    __ cmp(tos_, ip);
    // The value of a root is never NULL, so we can avoid loading a non-null
    // value into tos_ when we want to return 'true'.
    if (!result) {
      __ mov(tos_, Operand(0, RelocInfo::NONE), eq);
    }
    __ Ret(eq);
  }
}


void ToBooleanStub::GenerateTypeTransition(MacroAssembler* masm) {
  if (!tos_.is(r3)) {
    __ mov(r3, tos_);
  }
  __ mov(r2, Operand(Smi::FromInt(tos_.code())));
  __ mov(r1, Operand(Smi::FromInt(types_.ToByte())));
  __ Push(r3, r2, r1);
  // Patch the caller to an appropriate specialized stub and return the
  // operation result to the caller of the stub.
  __ TailCallExternalReference(
      ExternalReference(IC_Utility(IC::kToBoolean_Patch), masm->isolate()),
      3,
      1);
}


void UnaryOpStub::PrintName(StringStream* stream) {
  const char* op_name = Token::Name(op_);
  const char* overwrite_name = NULL;  // Make g++ happy.
  switch (mode_) {
    case UNARY_NO_OVERWRITE: overwrite_name = "Alloc"; break;
    case UNARY_OVERWRITE: overwrite_name = "Overwrite"; break;
  }
  stream->Add("UnaryOpStub_%s_%s_%s",
              op_name,
              overwrite_name,
              UnaryOpIC::GetName(operand_type_));
}


// TODO(svenpanne): Use virtual functions instead of switch.
void UnaryOpStub::Generate(MacroAssembler* masm) {
  switch (operand_type_) {
    case UnaryOpIC::UNINITIALIZED:
      GenerateTypeTransition(masm);
      break;
    case UnaryOpIC::SMI:
      GenerateSmiStub(masm);
      break;
    case UnaryOpIC::HEAP_NUMBER:
      GenerateHeapNumberStub(masm);
      break;
    case UnaryOpIC::GENERIC:
      GenerateGenericStub(masm);
      break;
  }
}


void UnaryOpStub::GenerateTypeTransition(MacroAssembler* masm) {
  __ mov(r3, r0);  // the operand
  __ mov(r2, Operand(Smi::FromInt(op_)));
  __ mov(r1, Operand(Smi::FromInt(mode_)));
  __ mov(r0, Operand(Smi::FromInt(operand_type_)));
  __ Push(r3, r2, r1, r0);

  __ TailCallExternalReference(
      ExternalReference(IC_Utility(IC::kUnaryOp_Patch), masm->isolate()), 4, 1);
}


// TODO(svenpanne): Use virtual functions instead of switch.
void UnaryOpStub::GenerateSmiStub(MacroAssembler* masm) {
  switch (op_) {
    case Token::SUB:
      GenerateSmiStubSub(masm);
      break;
    case Token::BIT_NOT:
      GenerateSmiStubBitNot(masm);
      break;
    default:
      UNREACHABLE();
  }
}


void UnaryOpStub::GenerateSmiStubSub(MacroAssembler* masm) {
  Label non_smi, slow;
  GenerateSmiCodeSub(masm, &non_smi, &slow);
  __ bind(&non_smi);
  __ bind(&slow);
  GenerateTypeTransition(masm);
}


void UnaryOpStub::GenerateSmiStubBitNot(MacroAssembler* masm) {
  Label non_smi;
  GenerateSmiCodeBitNot(masm, &non_smi);
  __ bind(&non_smi);
  GenerateTypeTransition(masm);
}


void UnaryOpStub::GenerateSmiCodeSub(MacroAssembler* masm,
                                     Label* non_smi,
                                     Label* slow) {
  __ JumpIfNotSmi(r0, non_smi);

  // The result of negating zero or the smallest negative smi is not a smi.
  __ bic(ip, r0, Operand(0x80000000));
  __ tst(ip, ip);
  __ b(eq, slow);

  // Return '0 - value'.
  __ rsb(r0, r0, Operand(0, RelocInfo::NONE));
  __ Ret();
}


void UnaryOpStub::GenerateSmiCodeBitNot(MacroAssembler* masm,
                                        Label* non_smi) {
  __ JumpIfNotSmi(r0, non_smi);

  // Flip bits and revert inverted smi-tag.
  __ mvn(r0, r0);
  __ bic(r0, r0, Operand(kSmiTagMask));
  __ Ret();
}


// TODO(svenpanne): Use virtual functions instead of switch.
void UnaryOpStub::GenerateHeapNumberStub(MacroAssembler* masm) {
  switch (op_) {
    case Token::SUB:
      GenerateHeapNumberStubSub(masm);
      break;
    case Token::BIT_NOT:
      GenerateHeapNumberStubBitNot(masm);
      break;
    default:
      UNREACHABLE();
  }
}


void UnaryOpStub::GenerateHeapNumberStubSub(MacroAssembler* masm) {
  Label non_smi, slow, call_builtin;
  GenerateSmiCodeSub(masm, &non_smi, &call_builtin);
  __ bind(&non_smi);
  GenerateHeapNumberCodeSub(masm, &slow);
  __ bind(&slow);
  GenerateTypeTransition(masm);
  __ bind(&call_builtin);
  GenerateGenericCodeFallback(masm);
}


void UnaryOpStub::GenerateHeapNumberStubBitNot(MacroAssembler* masm) {
  Label non_smi, slow;
  GenerateSmiCodeBitNot(masm, &non_smi);
  __ bind(&non_smi);
  GenerateHeapNumberCodeBitNot(masm, &slow);
  __ bind(&slow);
  GenerateTypeTransition(masm);
}

void UnaryOpStub::GenerateHeapNumberCodeSub(MacroAssembler* masm,
                                            Label* slow) {
  EmitCheckForHeapNumber(masm, r0, r1, r6, slow);
  // r0 is a heap number.  Get a new heap number in r1.
  if (mode_ == UNARY_OVERWRITE) {
    __ ldr(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
    __ eor(r2, r2, Operand(HeapNumber::kSignMask));  // Flip sign.
    __ str(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
  } else {
    Label slow_allocate_heapnumber, heapnumber_allocated;
    __ AllocateHeapNumber(r1, r2, r3, r6, &slow_allocate_heapnumber);
    __ jmp_near(&heapnumber_allocated);

    __ bind(&slow_allocate_heapnumber);
    __ EnterInternalFrame();
    __ push(r0);
    __ CallRuntime(Runtime::kNumberAlloc, 0);
    __ mov(r1, r0);
    __ pop(r0);
    __ LeaveInternalFrame();

    __ bind(&heapnumber_allocated);
    __ ldr(r3, FieldMemOperand(r0, HeapNumber::kMantissaOffset));
    __ ldr(r2, FieldMemOperand(r0, HeapNumber::kExponentOffset));
    __ str(r3, FieldMemOperand(r1, HeapNumber::kMantissaOffset));
    __ eor(r2, r2, Operand(HeapNumber::kSignMask));  // Flip sign.
    __ str(r2, FieldMemOperand(r1, HeapNumber::kExponentOffset));
    __ mov(r0, r1);
  }
  __ Ret();
}


void UnaryOpStub::GenerateHeapNumberCodeBitNot(
    MacroAssembler* masm, Label* slow) {
  Label impossible;

  EmitCheckForHeapNumber(masm, r0, r1, r6, slow);
  // Convert the heap number is r0 to an untagged integer in r1.
  __ ConvertToInt32(r0, r1, r2, r3, /*d0*/no_dreg, slow);

  // Do the bitwise operation and check if the result fits in a smi.
  Label try_float;
  __ mvn(r1, r1);
  __ add(r2, r1, Operand(0x40000000));
  __ cmpge(r2, Operand(0));
  __ bf(&try_float);

  // Tag the result as a smi and we're done.
  __ lsl(r0, r1, Operand(kSmiTagSize));
  __ Ret();

  // Try to store the result in a heap number.
  __ bind(&try_float);
  if (mode_ == UNARY_NO_OVERWRITE) {
    Label slow_allocate_heapnumber, heapnumber_allocated;
    // Allocate a new heap number without zapping r0, which we need if it fails.
    __ AllocateHeapNumber(r2, r3, r4, r6, &slow_allocate_heapnumber);
    __ jmp(&heapnumber_allocated);

    __ bind(&slow_allocate_heapnumber);
    __ EnterInternalFrame();
    __ push(r0);  // Push the heap number, not the untagged int32.
    __ CallRuntime(Runtime::kNumberAlloc, 0);
    __ mov(r2, r0);  // Move the new heap number into r2.
    // Get the heap number into r0, now that the new heap number is in r2.
    __ pop(r0);
    __ LeaveInternalFrame();

    // Convert the heap number in r0 to an untagged integer in r1.
    // This can't go slow-case because it's the same number we already
    // converted once again.
    __ ConvertToInt32(r0, r1, r3, r4, /*d0*/no_dreg, &impossible);
    __ mvn(r1, r1);

    __ bind(&heapnumber_allocated);
    __ mov(r0, r2);  // Move newly allocated heap number to r0.
  }

//  if (CpuFeatures::IsSupported(VFP3)) {
//  } else
  {
    // WriteInt32ToHeapNumberStub does not trigger GC, so we do not
    // have to set up a frame.
    WriteInt32ToHeapNumberStub stub(r1, r0, r2);
    __ Jump(stub.GetCode(), RelocInfo::CODE_TARGET);
  }

  __ bind(&impossible);
  if (FLAG_debug_code) {
    __ stop("Incorrect assumption in bit-not stub");
  }
}


// TODO(svenpanne): Use virtual functions instead of switch.
void UnaryOpStub::GenerateGenericStub(MacroAssembler* masm) {
  switch (op_) {
    case Token::SUB:
      GenerateGenericStubSub(masm);
      break;
    case Token::BIT_NOT:
      GenerateGenericStubBitNot(masm);
      break;
    default:
      UNREACHABLE();
  }
}


void UnaryOpStub::GenerateGenericStubSub(MacroAssembler* masm) {
  Label non_smi, slow;
  GenerateSmiCodeSub(masm, &non_smi, &slow);
  __ bind(&non_smi);
  GenerateHeapNumberCodeSub(masm, &slow);
  __ bind(&slow);
  GenerateGenericCodeFallback(masm);
}


void UnaryOpStub::GenerateGenericStubBitNot(MacroAssembler* masm) {
  Label non_smi, slow;
  GenerateSmiCodeBitNot(masm, &non_smi);
  __ bind(&non_smi);
  GenerateHeapNumberCodeBitNot(masm, &slow);
  __ bind(&slow);
  GenerateGenericCodeFallback(masm);
}


void UnaryOpStub::GenerateGenericCodeFallback(MacroAssembler* masm) {
  // Handle the slow case by jumping to the JavaScript builtin.
  __ push(r0);
  switch (op_) {
    case Token::SUB:
      __ InvokeBuiltin(Builtins::UNARY_MINUS, JUMP_FUNCTION);
      break;
    case Token::BIT_NOT:
      __ InvokeBuiltin(Builtins::BIT_NOT, JUMP_FUNCTION);
      break;
    default:
      UNREACHABLE();
  }
}


void BinaryOpStub::GenerateTypeTransition(MacroAssembler* masm) {
  Label get_result;

  __ Push(r1, r0);

  __ mov(r2, Operand(Smi::FromInt(MinorKey())));
  __ mov(r1, Operand(Smi::FromInt(op_)));
  __ mov(r0, Operand(Smi::FromInt(operands_type_)));
  __ Push(r2, r1, r0);

  __ TailCallExternalReference(
      ExternalReference(IC_Utility(IC::kBinaryOp_Patch),
                        masm->isolate()),
      5,
      1);
}


void BinaryOpStub::GenerateTypeTransitionWithSavedArgs(
    MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void BinaryOpStub::Generate(MacroAssembler* masm) {
  switch (operands_type_) {
    case BinaryOpIC::UNINITIALIZED:
      GenerateTypeTransition(masm);
      break;
    case BinaryOpIC::SMI:
      GenerateSmiStub(masm);
      break;
    case BinaryOpIC::INT32:
      GenerateInt32Stub(masm);
      break;
    case BinaryOpIC::HEAP_NUMBER:
      GenerateHeapNumberStub(masm);
      break;
    case BinaryOpIC::ODDBALL:
      GenerateOddballStub(masm);
      break;
    case BinaryOpIC::BOTH_STRING:
      GenerateBothStringStub(masm);
      break;
    case BinaryOpIC::STRING:
      GenerateStringStub(masm);
      break;
    case BinaryOpIC::GENERIC:
      GenerateGeneric(masm);
      break;
    default:
      UNREACHABLE();
  }
}


void BinaryOpStub::PrintName(StringStream* stream) {
  const char* op_name = Token::Name(op_);
  const char* overwrite_name;
  switch (mode_) {
    case NO_OVERWRITE: overwrite_name = "Alloc"; break;
    case OVERWRITE_RIGHT: overwrite_name = "OverwriteRight"; break;
    case OVERWRITE_LEFT: overwrite_name = "OverwriteLeft"; break;
    default: overwrite_name = "UnknownOverwrite"; break;
  }
  stream->Add("BinaryOpStub_%s_%s_%s",
              op_name,
              overwrite_name,
              BinaryOpIC::GetName(operands_type_));
}


void BinaryOpStub::GenerateSmiSmiOperation(MacroAssembler* masm) {
  Register left = r1;
  Register right = r0;
  Register scratch1 = r7;
  Register scratch2 = r9;

  ASSERT(right.is(r0));
  STATIC_ASSERT(kSmiTag == 0);

  Label not_smi_result;
  switch (op_) {
    case Token::ADD:
      __ addv(right, left, right);        // Add optimistically.
      __ Ret(f);                        // Return if no overflow
      __ sub(right, right, left);          // Revert optimistic add.
      break;
    case Token::SUB:
      __ subv(right, left, right);        // Subtract optimistically.
      __ Ret(f);                        // Return if no overflow
      __ sub(right, left, right);        // Revert optimistic subtract.
      break;
    case Token::MUL:
      // TODO(stm): implement optimized multiply with overflow check for SH4
      // Remove tag from one of the operands. This way the multiplication result
      // will be a smi if it fits the smi range.
      __ SmiUntag(ip, right);
      // Do multiplication
      // scratch1 = lower 32 bits of ip * left.
      // scratch2 = higher 32 bits of ip * left.
      __ dmuls(scratch1, scratch2, left, ip);
      // Check for overflowing the smi range - no overflow if higher 33 bits of
      // the result are identical.
      __ asr(ip, scratch1, Operand(31));
      __ cmp(ip, scratch2);
      __ b(ne, &not_smi_result);
      // Go slow on zero result to handle -0.
      // TODO(stm): merge cond mov and cond Ret or use fast branch
      __ tst(scratch1, scratch1);
      __ mov(right, scratch1, ne);
      __ Ret(ne);
      // We need -0 if we were multiplying a negative number with 0 to get 0.
      // We know one of them was zero.
      __ add(scratch2, right, left);
      __ cmpge(scratch2, Operand(0));
      __ mov(right, Operand(Smi::FromInt(0)), t);
      __ Ret(t);  // Return smi 0 if the non-zero one was positive.
      // We fall through here if we multiplied a negative number with 0, because
      // that would mean we should produce -0.
      break;
    case Token::DIV:
      // Check for power of two on the right hand side.
      __ JumpIfNotPowerOfTwoOrZero(right, scratch1, &not_smi_result);
      // Check for positive and no remainder (scratch1 contains right - 1).
      __ orr(scratch2, scratch1, Operand(0x80000000u));
      __ tst(left, scratch2);
      __ b(ne, &not_smi_result);

      // Perform division by shifting.
      __ CountLeadingZeros(scratch1, scratch1, scratch2);
      __ rsb(scratch1, scratch1, Operand(31));
      __ lsr(right, left, scratch1);
      __ Ret();
      break;
    case Token::MOD:
      // Check for two positive smis.
      __ orr(scratch1, left, right);
      __ tst(scratch1, Operand(0x80000000u | kSmiTagMask));
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
      __ bic(right, right, Operand(kSmiTagMask));
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
      __ tst(scratch1, Operand(0xc0000000));
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
      __ add(scratch2, scratch1, Operand(0x40000000));
      __ cmpge(scratch2, Operand(0));
      __ bf(&not_smi_result);
      __ SmiTag(right, scratch1);
      __ Ret();
      break;
    default:
      UNREACHABLE();
  }
  __ bind(&not_smi_result);
}


void BinaryOpStub::GenerateFPOperation(MacroAssembler* masm,
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
          CpuFeatures::IsSupported(FPU) &&
          op_ != Token::MOD ?
          FloatingPointHelper::kVFPRegisters :
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
      if (destination == FloatingPointHelper::kVFPRegisters) {
        // Using VFP registers:
        // dr0: Left value   d6
        // dr2: Right value  d7
        switch (op_) {
          case Token::ADD:
            __ fadd(dr0, dr2);
            break;
          case Token::SUB:
            __ fsub(dr0, dr2);
            break;
          case Token::MUL:
            __ fmul(dr0, dr2);
            break;
          case Token::DIV:
            __ fdiv(dr0, dr2);
            break;
          default:
            UNREACHABLE();
        }

        __ sub(r0, result, Operand(kHeapObjectTag));
        __ dstr(dr0, MemOperand(r0, HeapNumber::kValueOffset));
        __ add(r0, r0, Operand(kHeapObjectTag));
        __ Ret();
      } else {
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
        // Convert operands to 32-bit integers. Right in r2 and left in r3.
        FloatingPointHelper::ConvertNumberToInt32(masm,
                                                  left,
                                                  r3,
                                                  heap_number_map,
                                                  scratch1,
                                                  scratch2,
                                                  scratch3,
                                                  dr0,
                                                  not_numbers);
        FloatingPointHelper::ConvertNumberToInt32(masm,
                                                  right,
                                                  r2,
                                                  heap_number_map,
                                                  scratch1,
                                                  scratch2,
                                                  scratch3,
                                                  dr0,
                                                  not_numbers);
      }

      Label result_not_a_smi;
      switch (op_) {
        case Token::BIT_OR:
          __ orr(r2, r3, r2);
          break;
        case Token::BIT_XOR:
          __ eor(r2, r3, r2);
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
          __ cmpge(r2, Operand(0)); // Check non-negative (see comment below).
          // SHR is special because it is required to produce a positive answer.
          // The code below for writing into heap numbers isn't capable of
          // writing the register as an unsigned int so we go to slow case if we
          // hit this case.
          // TODO(stm): FPU
          // if (CpuFeatures::IsSupported(VFP3)) {
          // } else
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
      __ add(r3, r2, Operand(0x40000000));
      __ cmpge(r3, Operand(0));
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
        GenerateHeapResultAllocation(
            masm, result, heap_number_map, scratch1, scratch2, gc_required);
      }

      // r2: Answer as signed int32.
      // r5: Heap number to write answer into.

      // Nothing can go wrong now, so move the heap number to r0, which is the
      // result.
      __ mov(r0, r5);       //TODO(stm): look at this: it should be better

      // TODO(stm): FPU
      // if (CpuFeatures::IsSupported(VFP3)) {
      // } else
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
void BinaryOpStub::GenerateSmiCode(
    MacroAssembler* masm,
    Label* use_runtime,
    Label* gc_required,
    SmiCodeGenerateHeapNumberResults allow_heapnumber_results) {
  Label not_smis;

  Register left = r1;
  Register right = r0;
  Register scratch1 = r7;

  // Perform combined smi check on both operands.
  __ orr(scratch1, left, right);
  STATIC_ASSERT(kSmiTag == 0);
  __ JumpIfNotSmi(scratch1, &not_smis);

  // If the smi-smi operation results in a smi return is generated.
  GenerateSmiSmiOperation(masm);

  // If heap number results are possible generate the result in an allocated
  // heap number.
  if (allow_heapnumber_results == ALLOW_HEAPNUMBER_RESULTS) {
    GenerateFPOperation(masm, true, use_runtime, gc_required);
  }
  __ bind(&not_smis);
}


void BinaryOpStub::GenerateSmiStub(MacroAssembler* masm) {
  Label not_smis, call_runtime;

  if (result_type_ == BinaryOpIC::UNINITIALIZED ||
      result_type_ == BinaryOpIC::SMI) {
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


void BinaryOpStub::GenerateStringStub(MacroAssembler* masm) {
  ASSERT(operands_type_ == BinaryOpIC::STRING);
  ASSERT(op_ == Token::ADD);
  // Try to add arguments as strings, otherwise, transition to the generic
  // BinaryOpIC type.
  GenerateAddStrings(masm);
  GenerateTypeTransition(masm);
}


void BinaryOpStub::GenerateBothStringStub(MacroAssembler* masm) {
  Label call_runtime;
  ASSERT(operands_type_ == BinaryOpIC::BOTH_STRING);
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


void BinaryOpStub::GenerateInt32Stub(MacroAssembler* masm) {
  ASSERT(operands_type_ == BinaryOpIC::INT32);

  Register left = r1;
  Register right = r0;
  Register scratch1 = r7;
  Register scratch2 = r9;
  // DwVfpRegister double_scratch = no_dreg/*d0*/;         // TODO(stm): FPU
  // SwVfpRegister single_scratch = no_freg/*s3*/;         // TODO(stm): FPU

  Register heap_number_result = no_reg;
  Register heap_number_map = r6;
  __ LoadRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);

  Label call_runtime;
  // Labels for type transition, used for wrong input or output types.
  // Both label are currently actually bound to the same position. We use two
  // different label to differentiate the cause leading to type transition.
  Label transition;

  // Smi-smi fast case.
  Label skip;
  __ orr(scratch1, left, right);
  __ JumpIfNotSmi(scratch1, &skip);
  GenerateSmiSmiOperation(masm);
  // Fall through if the result is not a smi.
  __ bind(&skip);

  switch (op_) {
    case Token::ADD:
    case Token::SUB:
    case Token::MUL:
    case Token::DIV:
    case Token::MOD: {
      // Load both operands and check that they are 32-bit integer.
      // Jump to type transition if they are not. The registers r0 and r1 (right
      // and left) are preserved for the runtime call.
      // TODO(STM): for the moment we only use core registers
      FloatingPointHelper::Destination destination =
/*          (CpuFeatures::IsSupported(FPU) && op_ != Token::MOD)
              ? FloatingPointHelper::kVFPRegisters
              : */FloatingPointHelper::kCoreRegisters;

      FloatingPointHelper::LoadNumberAsInt32Double(masm,
                                                   right,
                                                   destination,
                                                   dr2,
                                                   r2,
                                                   r3,
                                                   heap_number_map,
                                                   scratch1,
                                                   scratch2,
                                                   no_freg,
                                                   &transition);
      FloatingPointHelper::LoadNumberAsInt32Double(masm,
                                                   left,
                                                   destination,
                                                   dr0,
                                                   r4,
                                                   r5,
                                                   heap_number_map,
                                                   scratch1,
                                                   scratch2,
                                                   no_freg,
                                                   &transition);

    // TODO(stm): FPU
    // if (destination == FloatingPointHelper::kVFPRegisters) {
    // } else
    {
        // We preserved r0 and r1 to be able to call runtime.
        // Save the left value on the stack.
        __ Push(r5, r4);

        Label pop_and_call_runtime;

        // Allocate a heap number to store the result.
        heap_number_result = r5;
        GenerateHeapResultAllocation(masm,
                                     heap_number_result,
                                     heap_number_map,
                                     scratch1,
                                     scratch2,
                                     &pop_and_call_runtime);

        // Load the left value from the value saved on the stack.
        __ Pop(r1, r0);

        // Call the C function to handle the double operation.
        FloatingPointHelper::CallCCodeForDoubleOperation(
            masm, op_, heap_number_result, scratch2);
        if (FLAG_debug_code) {
          __ stop("Unreachable code.");
        }

        __ bind(&pop_and_call_runtime);
        __ Drop(2);
        __ b(&call_runtime);
      }

      break;
    }

    case Token::BIT_OR:
    case Token::BIT_XOR:
    case Token::BIT_AND:
    case Token::SAR:
    case Token::SHR:
    case Token::SHL: {
      Label return_heap_number;
      Register scratch3 = r5;
      // Convert operands to 32-bit integers. Right in r2 and left in r3. The
      // registers r0 and r1 (right and left) are preserved for the runtime
      // call.
      FloatingPointHelper::LoadNumberAsInt32(masm,
                                             left,
                                             r3,
                                             heap_number_map,
                                             scratch1,
                                             scratch2,
                                             scratch3,
                                             no_dreg/*TODO:d0*/,
                                             &transition);
      FloatingPointHelper::LoadNumberAsInt32(masm,
                                             right,
                                             r2,
                                             heap_number_map,
                                             scratch1,
                                             scratch2,
                                             scratch3,
                                             no_dreg/*TODO:d0*/,
                                             &transition);

      // The ECMA-262 standard specifies that, for shift operations, only the
      // 5 least significant bits of the shift value should be used.
      switch (op_) {
        case Token::BIT_OR:
          __ orr(r2, r3, r2);
          break;
        case Token::BIT_XOR:
          __ eor(r2, r3, r2);
          break;
        case Token::BIT_AND:
          __ land(r2, r3, r2);
          break;
        case Token::SAR:
          __ land(r2, r2, Operand(0x1f));
          __ asr(r2, r3, r2);
          break;
        case Token::SHR:
          __ land(r2, r2, Operand(0x1f));
          __ lsr(r2, r3, r2);
          // SHR is special because it is required to produce a positive answer.
          // We only get a negative result if the shift value (r2) is 0.
          // This result cannot be respresented as a signed 32-bit integer, try
          // to return a heap number if we can.
          // The non vfp3 code does not support this special case, so jump to
          // runtime if we don't support it.
          __ cmpge(r2, Operand(0));
          // TODO(stm): FPU
          // if (CpuFeatures::IsSupported(VFP3)) {
          // } else
          {
            __ b(f, (result_type_ <= BinaryOpIC::INT32)
                      ? &transition
                      : &call_runtime);
          }
          break;
        case Token::SHL:
          __ land(r2, r2, Operand(0x1f));
          __ lsl(r2, r3, r2);
          break;
        default:
          UNREACHABLE();
      }

      // Check if the result fits in a smi.
      __ add(scratch1, r2, Operand(0x40000000));
      // If not try to return a heap number. (We know the result is an int32.)
      __ cmpge(scratch1, Operand(0));
      __ b(f, &return_heap_number);
      // Tag the result and return.
      __ SmiTag(r0, r2);
      __ Ret();

      __ bind(&return_heap_number);
      heap_number_result = r5;
      GenerateHeapResultAllocation(masm,
                                   heap_number_result,
                                   heap_number_map,
                                   scratch1,
                                   scratch2,
                                   &call_runtime);
      // TODO(stm): FPU
      // if (CpuFeatures::IsSupported(VFP3)) {
      // } else
      {
        // Tail call that writes the int32 in r2 to the heap number in r0, using
        // r3 as scratch. r0 is preserved and returned.
        __ mov(r0, r5);
        WriteInt32ToHeapNumberStub stub(r2, r0, r3);
        __ TailCallStub(&stub);
      }

      break;
    }

    default:
      UNREACHABLE();
  }

  // We never expect DIV to yield an integer result, so we always generate
  // type transition code for DIV operations expecting an integer result: the
  // code will fall through to this type transition.
  if (transition.is_linked() ||
      ((op_ == Token::DIV) && (result_type_ <= BinaryOpIC::INT32))) {
    __ bind(&transition);
    GenerateTypeTransition(masm);
  }

  __ bind(&call_runtime);
  GenerateCallRuntime(masm);
}


void BinaryOpStub::GenerateOddballStub(MacroAssembler* masm) {
  Label call_runtime;

  if (op_ == Token::ADD) {
    // Handle string addition here, because it is the only operation
    // that does not do a ToNumber conversion on the operands.
    GenerateAddStrings(masm);
  }

  // Convert oddball arguments to numbers.
  Label check, done;
  __ CompareRoot(r1, Heap::kUndefinedValueRootIndex);
  __ b(ne, &check);
  if (Token::IsBitOp(op_)) {
    __ mov(r1, Operand(Smi::FromInt(0)));
  } else {
    __ LoadRoot(r1, Heap::kNanValueRootIndex);
  }
  __ jmp_near(&done);
  __ bind(&check);
  __ CompareRoot(r0, Heap::kUndefinedValueRootIndex);
  __ b(ne, &done, Label::kNear);
  if (Token::IsBitOp(op_)) {
    __ mov(r0, Operand(Smi::FromInt(0)));
  } else {
    __ LoadRoot(r0, Heap::kNanValueRootIndex);
  }
  __ bind(&done);

  GenerateHeapNumberStub(masm);
}


void BinaryOpStub::GenerateHeapNumberStub(MacroAssembler* masm) {
  Label call_runtime;
  GenerateFPOperation(masm, false, &call_runtime, &call_runtime);

  __ bind(&call_runtime);
  GenerateCallRuntime(masm);
}


void BinaryOpStub::GenerateGeneric(MacroAssembler* masm) {
  Label call_runtime, call_string_add_or_runtime;

  GenerateSmiCode(masm, &call_runtime, &call_runtime, ALLOW_HEAPNUMBER_RESULTS);

  GenerateFPOperation(masm, false, &call_string_add_or_runtime, &call_runtime);

  __ bind(&call_string_add_or_runtime);
  if (op_ == Token::ADD) {
    GenerateAddStrings(masm);
  }

  __ bind(&call_runtime);
  GenerateCallRuntime(masm);
}


void BinaryOpStub::GenerateAddStrings(MacroAssembler* masm) {
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


void BinaryOpStub::GenerateCallRuntime(MacroAssembler* masm) {
  GenerateRegisterArgsPush(masm);
  switch (op_) {
    case Token::ADD:
      __ InvokeBuiltin(Builtins::ADD, JUMP_FUNCTION);
      break;
    case Token::SUB:
      __ InvokeBuiltin(Builtins::SUB, JUMP_FUNCTION);
      break;
    case Token::MUL:
      __ InvokeBuiltin(Builtins::MUL, JUMP_FUNCTION);
      break;
    case Token::DIV:
      __ InvokeBuiltin(Builtins::DIV, JUMP_FUNCTION);
      break;
    case Token::MOD:
      __ InvokeBuiltin(Builtins::MOD, JUMP_FUNCTION);
      break;
    case Token::BIT_OR:
      __ InvokeBuiltin(Builtins::BIT_OR, JUMP_FUNCTION);
      break;
    case Token::BIT_AND:
      __ InvokeBuiltin(Builtins::BIT_AND, JUMP_FUNCTION);
      break;
    case Token::BIT_XOR:
      __ InvokeBuiltin(Builtins::BIT_XOR, JUMP_FUNCTION);
      break;
    case Token::SAR:
      __ InvokeBuiltin(Builtins::SAR, JUMP_FUNCTION);
      break;
    case Token::SHR:
      __ InvokeBuiltin(Builtins::SHR, JUMP_FUNCTION);
      break;
    case Token::SHL:
      __ InvokeBuiltin(Builtins::SHL, JUMP_FUNCTION);
      break;
    default:
      UNREACHABLE();
  }
}


void BinaryOpStub::GenerateHeapResultAllocation(MacroAssembler* masm,
                                                Register result,
                                                Register heap_number_map,
                                                Register scratch1,
                                                Register scratch2,
                                                Label* gc_required) {
  // Code below will scratch result if allocation fails. To keep both arguments
  // intact for the runtime call result cannot be one of these.
  ASSERT(!result.is(r0) && !result.is(r1));

  if (mode_ == OVERWRITE_LEFT || mode_ == OVERWRITE_RIGHT) {
    Label skip_allocation;
    Label allocated;
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


void BinaryOpStub::GenerateRegisterArgsPush(MacroAssembler* masm) {
  __ Push(r1, r0);
}


void TranscendentalCacheStub::Generate(MacroAssembler* masm) {
  // Untagged case: double input in d2, double result goes
  //   into d2.
  // Tagged case: tagged input on top of stack and in r0,
  //   tagged result (heap number) goes into r0.

  // Label input_not_smi;
  // Label loaded;
  // Label calculate;
  // Label invalid_cache;
  // const Register scratch0 = r9;
  // const Register scratch1 = r7;
  // const Register cache_entry = r0;
  const bool tagged = (argument_type_ == TAGGED);

  // FIXME(stm): FPU
  // if (CpuFeatures::IsSupported(VFP3)) {
  // }  // if (CpuFeatures::IsSupported(VFP3))

  // __ bind(&calculate);
  if (tagged) {
    // __ bind(&invalid_cache);
    ExternalReference runtime_function =
        ExternalReference(RuntimeFunction(), masm->isolate());
    __ TailCallExternalReference(runtime_function, 1, 1);
  } else {
    // FIXME(stm): FPU
    UNREACHABLE();
  }
}


void TranscendentalCacheStub::GenerateCallCFunction(MacroAssembler* masm,
                                                    Register scratch) {
  Isolate* isolate = masm->isolate();

  __ push(lr);
  __ PrepareCallCFunction(0, 1, scratch);
  __ movd(dr4, r0, r1);
  switch (type_) {
    case TranscendentalCache::SIN:
      __ CallCFunction(ExternalReference::math_sin_double_function(isolate),
          0, 1);
      break;
    case TranscendentalCache::COS:
      __ CallCFunction(ExternalReference::math_cos_double_function(isolate),
          0, 1);
      break;
    case TranscendentalCache::LOG:
      __ CallCFunction(ExternalReference::math_log_double_function(isolate),
          0, 1);
      break;
    default:
      UNIMPLEMENTED();
      break;
  }
  __ pop(lr);
}


Runtime::FunctionId TranscendentalCacheStub::RuntimeFunction() {
  switch (type_) {
    // Add more cases when necessary.
    case TranscendentalCache::SIN: return Runtime::kMath_sin;
    case TranscendentalCache::COS: return Runtime::kMath_cos;
    case TranscendentalCache::LOG: return Runtime::kMath_log;
    default:
      UNIMPLEMENTED();
      return Runtime::kAbort;
  }
}


void StackCheckStub::Generate(MacroAssembler* masm) {
  __ TailCallRuntime(Runtime::kStackGuard, 0, 1);
}


void MathPowStub::Generate(MacroAssembler* masm) {
  // TODO(stm): FPU
  // if (CpuFeatures::IsSupported(VFP3)) {
  // } else
  {
    __ TailCallRuntime(Runtime::kMath_pow_cfunction, 2, 1);
  }
}


bool CEntryStub::NeedsImmovableCode() {
  return true;
}


void CEntryStub::GenerateThrowTOS(MacroAssembler* masm) {
  __ Throw(r0);
}


void CEntryStub::GenerateThrowUncatchable(MacroAssembler* masm,
                                          UncatchableExceptionType type) {
  __ ThrowUncatchable(type, r0);
}


void CEntryStub::GenerateCore(MacroAssembler* masm,
                              Label* throw_normal_exception,
                              Label* throw_termination_exception,
                              Label* throw_out_of_memory_exception,
                              bool do_gc,
                              bool always_allocate) {
  // WARNING: this function use the SH4 ABI !!

  // Input
  // r0: result parameter for PerformGC, if any
  // sh4_r8: number of arguments including receiver  (C callee-saved)
  //         Used later by LeaveExitFrame()
  // sh4_r9: pointer to builtin function  (C callee-saved)
  // sh4_r10: pointer to the first argument (C callee-saved)
  // TODO(stm): use of r10 is dangerous (ip)
  // sh4: moved callee-saved to stack localtion (see ::Generate())
  Isolate* isolate = masm->isolate();
  ASSERT(!r0.is(sh4_rtmp));
  ASSERT(!r0.is(sh4_ip));
  //ASSERT(!sh4_r8.is(sh4_rtmp) && !sh4_r9.is(sh4_rtmp) && !sh4_r10.is(sh4_rtmp));

  if (do_gc) {
    // Passing r0.
    __ PrepareCallCFunction(1, 0, r1);
    __ mov(r4, r0);
    __ CallCFunction(ExternalReference::perform_gc_function(isolate),
        1, 0);
  }

  ExternalReference scope_depth =
      ExternalReference::heap_always_allocate_scope_depth(isolate);
  if (always_allocate) {
    __ mov(r0, Operand(scope_depth));
    __ ldr(r1, MemOperand(r0));
    __ add(r1, r1, Operand(1));
    __ str(r1, MemOperand(r0));
  }

  // Call C built-in.
  // r4 = argc, r5 = argv, r6 = isolate
  //__ mov(r4, sh4_r8);
  //__ mov(r5, sh4_r10);
  // sh4: ref to ::Generate that stored into the stack
  __ ldr(r4, MemOperand(sp, (1+0)*kPointerSize));
  __ ldr(r5, MemOperand(sp, (1+2)*kPointerSize));

#if defined(V8_HOST_ARCH_SH4)
  int frame_alignment = OS::ActivationFrameAlignment();
  int frame_alignment_mask = frame_alignment - 1;
  if (FLAG_debug_code) {
    if (frame_alignment > kPointerSize) {
      Label alignment_as_expected;
      ASSERT(IsPowerOf2(frame_alignment));
      __ tst(sp, Operand(frame_alignment_mask));
      __ b(eq, &alignment_as_expected);
      // Don't use Check here, as it will call Runtime_Abort re-entering here.
      __ stop("Unexpected alignment");
      __ bind(&alignment_as_expected);
    }
  }
#endif

  __ mov(r6, Operand(ExternalReference::isolate_address()));

  // sh4: ref to ::Generate() that stored the builtin into the stack
  __ ldr(r2, MemOperand(sp, (1+1)*kPointerSize));

  // To let the GC traverse the return address of the exit frames, we need to
  // know where the return address is. The CEntryStub is unmovable, so
  // we can store the address on the stack to be able to find it again and
  // we never have to restore it, because it will not change.
  // Compute the return address in lr to return to after the jump below. Pc is
  // already at '+ 8' from the current instruction but return is after three
  // instructions so add another 4 to pc to get the return address.

  // Compute the return address in pr to return to after the jsr below.
  // We use the addpc operation for this with an offset of 6.
  // We add 3 * kInstrSize to the pc after the addpc for the size of
  // the sequence: [str, jsr, nop(delay slot)].
  __ addpc(r3, 3 * Assembler::kInstrSize, pr);
#ifdef DEBUG
  int old_pc = masm->pc_offset();
#endif
  __ str(r3, MemOperand(sp, 0));
  __ jsr(r2);
  //  __ jsr(sh4_r9);
#ifdef DEBUG
  ASSERT(masm->pc_offset() - old_pc == 3 * Assembler::kInstrSize);
#endif

  if (always_allocate) {
    // It's okay to clobber r2 and r3 here. Don't mess with r0 and r1
    // though (contain the result).
    __ mov(r2, Operand(scope_depth));
    __ ldr(r3, MemOperand(r2));
    __ sub(r3, r3, Operand(1));
    __ str(r3, MemOperand(r2));
  }

  // check for failure result
  Label failure_returned;
  STATIC_ASSERT(((kFailureTag + 1) & kFailureTagMask) == 0);
  // Lower 2 bits of r2 are 0 if r0 has failure tag.
  __ add(r2, r0, Operand(1));
  __ tst(r2, Operand(kFailureTagMask));
  __ b(eq, &failure_returned);

  // Exit C frame and return.
  // r0:r1: result
  // sp: stack pointer
  // fp: frame pointer
  //  Callee-saved register sh4_r8 still holds argc.
  // sh4: stored on stack into ::Generate()
  __ ldr(r2, MemOperand(sp, (1+0)*kPointerSize));
  __ LeaveExitFrame(save_doubles_, r2);
  //  __ LeaveExitFrame(save_doubles_, sh4_r8);
  __ rts();

  // check if we should retry or throw exception
  Label retry;
  __ bind(&failure_returned);
  STATIC_ASSERT(Failure::RETRY_AFTER_GC == 0);
  __ tst(r0, Operand(((1 << kFailureTypeTagSize) - 1) << kFailureTagSize));
  __ b(eq, &retry, Label::kNear);

  // Special handling of out of memory exceptions.
  Failure* out_of_memory = Failure::OutOfMemoryException();
  __ cmp(r0, Operand(reinterpret_cast<int32_t>(out_of_memory)));
  __ b(eq, throw_out_of_memory_exception);

  // Retrieve the pending exception and clear the variable.
  __ mov(r3, Operand(ExternalReference::the_hole_value_location(isolate)));
  __ ldr(r2, MemOperand(r3));
  __ mov(r3, Operand(ExternalReference(Isolate::kPendingExceptionAddress,
                                       isolate)));
  __ ldr(r0, MemOperand(r3));
  __ str(r2, MemOperand(r3));

  // Special handling of termination exceptions which are uncatchable
  // by javascript code.
  __ mov(r3, Operand(isolate->factory()->termination_exception()));
  __ cmpeq(r0, r3);
  __ b(eq, throw_termination_exception);

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

  // sh4: clobbers r3
  // Result returned in r0 or r0+r1 by default.

  // NOTE: Invocations of builtins may return failure objects
  // instead of a proper result. The builtin entry handles
  // this by performing a garbage collection and retrying the
  // builtin once.

  // Compute the argv pointer in a callee-saved register.
  // sh4: will be saved on stack
  // __ lsl(sh4_r10, r0, Operand(kPointerSizeLog2));
  // __ add(sh4_r10, sp, sh4_r10);
  // __ sub(sh4_r10, sh4_r10, Operand(kPointerSize));
  __ lsl(r3, r0, Operand(kPointerSizeLog2));
  __ add(r3, sp, r3);
  __ sub(r3, r3, Operand(kPointerSize));

  // Enter the exit frame that transitions from JavaScript to C++.
  __ EnterExitFrame(save_doubles_, 3); // SH4: Reserve space for 3 stack locations

  // Setup argc and the builtin function in callee-saved registers.
  // sh4: save on stack instead of keep in callee-saved
  // sh4: sp contains: sp[0] == lr; sp[1] == argc; sp[2] == builtin; sp[3] = argv
  // __ mov(sh4_r8, r0);
  // __ mov(sh4_r9, r1);
  __ str(r0, MemOperand(sp, (1+0)*kPointerSize)); // skip lr location at sp[1]
  __ str(r1, MemOperand(sp, (1+1)*kPointerSize));
  __ str(r3, MemOperand(sp, (1+2)*kPointerSize));

  // sh4_r8: number of arguments (C callee-saved)
  // sh4_r9: pointer to builtin function (C callee-saved)
  // sh4_r10: pointer to first argument (C callee-saved)
  //ASSERT(!sh4_r8.is(sh4_rtmp) && !sh4_r9.is(sh4_rtmp) && !sh4_r10.is(sh4_rtmp));

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
  GenerateThrowUncatchable(masm, OUT_OF_MEMORY);

  __ bind(&throw_termination_exception);
  GenerateThrowUncatchable(masm, TERMINATION);

  __ bind(&throw_normal_exception);
  GenerateThrowTOS(masm);
}


void JSEntryStub::GenerateBody(MacroAssembler* masm, bool is_construct) {
  // r4: code entry
  // r5: function
  // r6: receiver
  // r7: argc
  // [sp+0]: argv

  Label invoke;
  Label exit;

  // Save callee-saved registers
  __ push(pr);
  __ pushm(kCalleeSaved);

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

  // Setup argv in r4.
  int offset_to_argv = (kNumCalleeSaved + 1) * kPointerSize;
  __ ldr(r4, MemOperand(sp, offset_to_argv));

  // Push a frame with special values setup to mark it as an entry frame.
  // r0: code entry
  // r1: function
  // r2: receiver
  // r3: argc
  // r4: argv
  Isolate* isolate = masm->isolate();
  // Push a bad frame pointer to fail if it is used.
  __ mov(ip, Operand(-1));
  __ push(ip);

  int marker = is_construct ? StackFrame::ENTRY_CONSTRUCT : StackFrame::ENTRY;
  __ mov(r7, Operand(Smi::FromInt(marker)));
  __ mov(r6, Operand(Smi::FromInt(marker)));
  __ mov(r5,
         Operand(ExternalReference(Isolate::kCEntryFPAddress, isolate)));
  __ ldr(r5, MemOperand(r5));
  __ push(r7);
  __ push(r6);
  __ push(r5);

  // Setup frame pointer for the frame to be pushed.
  __ add(fp, sp, Operand(-EntryFrameConstants::kCallerFPOffset));

  // If this is the outermost JS call, set js_entry_sp value.
  Label non_outermost_js;
  ExternalReference js_entry_sp(Isolate::kJSEntrySPAddress, isolate);
  __ mov(r5, Operand(ExternalReference(js_entry_sp)));
  __ ldr(r6, MemOperand(r5));
  __ cmp(r6, Operand(0));
  __ b(ne, &non_outermost_js, Label::kNear);
  __ str(fp, MemOperand(r5));
  __ mov(ip, Operand(Smi::FromInt(StackFrame::OUTERMOST_JSENTRY_FRAME)));
  Label cont;
  __ b_near(&cont);
  __ bind(&non_outermost_js);
  __ mov(ip, Operand(Smi::FromInt(StackFrame::INNER_JSENTRY_FRAME)));
  __ bind(&cont);
  __ push(ip);

  // Call a faked try-block that does the invoke.
  __ jsr(&invoke);

  // Caught exception: Store result (exception) in the pending
  // exception field in the JSEnv and return a failure sentinel.
  // Coming in here the fp will be invalid because the PushTryHandler below
  // sets it to 0 to signal the existence of the JSEntry frame.
  __ mov(ip, Operand(ExternalReference(Isolate::kPendingExceptionAddress,
                                       isolate)));
  __ str(r0, MemOperand(ip));
  __ mov(r0, Operand(reinterpret_cast<int32_t>(Failure::Exception())));
  __ b(&exit);

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
  __ mov(ip, Operand(ExternalReference(Isolate::kPendingExceptionAddress,
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
  __ add(ip, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ jsr(ip);

  // Unlink this frame from the handler chain.
  __ PopTryHandler();

  __ bind(&exit);  // r0 holds result
  // Check if the current stack frame is marked as the outermost JS frame.
  Label non_outermost_js_2;
  __ pop(r5);
  __ cmp(r5, Operand(Smi::FromInt(StackFrame::OUTERMOST_JSENTRY_FRAME)));
  __ b(ne, &non_outermost_js_2, Label::kNear);
  __ mov(r6, Operand(0));
  __ mov(r5, Operand(ExternalReference(js_entry_sp)));
  __ str(r6, MemOperand(r5));
  __ bind(&non_outermost_js_2);

  // Restore the top frame descriptors from the stack.
  __ pop(r3);
  __ mov(ip,
         Operand(ExternalReference(Isolate::kCEntryFPAddress, isolate)));
  __ str(r3, MemOperand(ip));

  // Reset the stack to the callee saved registers.
  __ add(sp, sp, Operand(-EntryFrameConstants::kCallerFPOffset));

  // Restore callee-saved registers and return.
  __ popm(kCalleeSaved);
  __ pop(pr);

  __ rts();
}


// Uses registers r0 to r4.
// Expected input (depending on whether args are in registers or on the stack):
// * object: r0 or at sp + 1 * kPointerSize.
// * function: r1 or at sp.
//
// An inlined call site may have been generated before calling this stub.
// In this case the offset to the inline site to patch is passed on the stack,
// in the safepoint slot for register r4.
// (See LCodeGen::DoInstanceOfKnownGlobal)
void InstanceofStub::Generate(MacroAssembler* masm) {
  // Call site inlining and patching implies arguments in registers.
  ASSERT(HasArgsInRegisters() || !HasCallSiteInlineCheck());
  // ReturnTrueFalse is only implemented for inlined call sites.
  ASSERT(!ReturnTrueFalseObject() || HasCallSiteInlineCheck());

  // Fixed register usage throughout the stub:
  const Register object = r0;  // Object (lhs).
  Register map = r3;  // Map of the object.
  const Register function = r1;  // Function (rhs).
  const Register prototype = r4;  // Prototype of the function.
  const Register inline_site = r9;
  const Register scratch = r2;

  const int32_t kDeltaToLoadBoolResult = 3 * kPointerSize;

  Label slow, loop, is_instance, is_not_instance, not_js_object;

  if (!HasArgsInRegisters()) {
    __ ldr(object, MemOperand(sp, 1 * kPointerSize));
    __ ldr(function, MemOperand(sp, 0));
  }

  // Check that the left hand is a JS object and load map.
  __ JumpIfSmi(object, &not_js_object);
  __ IsObjectJSObjectType(object, map, scratch, &not_js_object);

  // If there is a call site cache don't look in the global cache, but do the
  // real lookup and update the call site cache.
  if (!HasCallSiteInlineCheck()) {
    Label miss;
    __ LoadRoot(ip, Heap::kInstanceofCacheFunctionRootIndex);
    __ cmp(function, ip);
    __ b(ne, &miss, Label::kNear);
    __ LoadRoot(ip, Heap::kInstanceofCacheMapRootIndex);
    __ cmp(map, ip);
    __ b(ne, &miss, Label::kNear);
    __ LoadRoot(r0, Heap::kInstanceofCacheAnswerRootIndex);
    __ Ret(HasArgsInRegisters() ? 0 : 2);

    __ bind(&miss);
  }

  // Get the prototype of the function.
  __ TryGetFunctionPrototype(function, prototype, scratch, &slow);

  // Check that the function prototype is a JS object.
  __ JumpIfSmi(prototype, &slow);
  __ IsObjectJSObjectType(prototype, scratch, scratch, &slow);

  // Update the global instanceof or call site inlined cache with the current
  // map and function. The cached answer will be set when it is known below.
  if (!HasCallSiteInlineCheck()) {
    __ StoreRoot(function, Heap::kInstanceofCacheFunctionRootIndex);
    __ StoreRoot(map, Heap::kInstanceofCacheMapRootIndex);
  } else {
    ASSERT(HasArgsInRegisters());
    // Patch the (relocated) inlined map check.

    // The offset was stored in r4 safepoint slot.
    // (See LCodeGen::DoDeferredLInstanceOfKnownGlobal)
    __ LoadFromSafepointRegisterSlot(scratch, r4);
    __ sub(inline_site, lr, scratch);
    // Get the map location in scratch and patch it.
    __ GetRelocatedValueLocation(inline_site, scratch);
    __ str(map, MemOperand(scratch));
  }

  // Register mapping: r3 is object map and r4 is function prototype.
  // Get prototype of object into r2.
  __ ldr(scratch, FieldMemOperand(map, Map::kPrototypeOffset));

  // We don't need map any more. Use it as a scratch register.
  Register scratch2 = map;
  map = no_reg;

  // Loop through the prototype chain looking for the function prototype.
  __ LoadRoot(scratch2, Heap::kNullValueRootIndex);
  __ bind(&loop);
  __ cmp(scratch, prototype);
  __ b(eq, &is_instance, Label::kNear);
  __ cmp(scratch, scratch2);
  __ b(eq, &is_not_instance);
  __ ldr(scratch, FieldMemOperand(scratch, HeapObject::kMapOffset));
  __ ldr(scratch, FieldMemOperand(scratch, Map::kPrototypeOffset));
  __ jmp(&loop);

  __ bind(&is_instance);
  if (!HasCallSiteInlineCheck()) {
    __ mov(r0, Operand(Smi::FromInt(0)));
    __ StoreRoot(r0, Heap::kInstanceofCacheAnswerRootIndex);
  } else {
    // Patch the call site to return true.
    __ LoadRoot(r0, Heap::kTrueValueRootIndex);
    __ add(inline_site, inline_site, Operand(kDeltaToLoadBoolResult));
    // Get the boolean result location in scratch and patch it.
    __ GetRelocatedValueLocation(inline_site, scratch);
    __ str(r0, MemOperand(scratch));

    if (!ReturnTrueFalseObject()) {
      __ mov(r0, Operand(Smi::FromInt(0)));
    }
  }
  __ Ret(HasArgsInRegisters() ? 0 : 2);

  __ bind(&is_not_instance);
  if (!HasCallSiteInlineCheck()) {
    __ mov(r0, Operand(Smi::FromInt(1)));
    __ StoreRoot(r0, Heap::kInstanceofCacheAnswerRootIndex);
  } else {
    // Patch the call site to return false.
    __ LoadRoot(r0, Heap::kFalseValueRootIndex);
    __ add(inline_site, inline_site, Operand(kDeltaToLoadBoolResult));
    // Get the boolean result location in scratch and patch it.
    __ GetRelocatedValueLocation(inline_site, scratch);
    __ str(r0, MemOperand(scratch));

    if (!ReturnTrueFalseObject()) {
      __ mov(r0, Operand(Smi::FromInt(1)));
    }
  }
  __ Ret(HasArgsInRegisters() ? 0 : 2);

  Label object_not_null;
  Label object_not_null_or_smi;
  __ bind(&not_js_object);
  // Before null, smi and string value checks, check that the rhs is a function
  // as for a non-function rhs an exception needs to be thrown.
  __ JumpIfSmi(function, &slow);
  __ CompareObjectType(function, scratch2, scratch, JS_FUNCTION_TYPE, eq);
  __ b(ne, &slow);

  // Null is not instance of anything.
  __ cmp(scratch, Operand(masm->isolate()->factory()->null_value()));
  __ b(ne, &object_not_null, Label::kNear);
  __ mov(r0, Operand(Smi::FromInt(1)));
  __ Ret(HasArgsInRegisters() ? 0 : 2);

  __ bind(&object_not_null);
  // Smi values are not instances of anything.
  __ JumpIfNotSmi(object, &object_not_null_or_smi);
  __ mov(r0, Operand(Smi::FromInt(1)));
  __ Ret(HasArgsInRegisters() ? 0 : 2);

  __ bind(&object_not_null_or_smi);
  // String values are not instances of anything.
  __ IsObjectJSStringType(object, scratch, &slow);
  __ mov(r0, Operand(Smi::FromInt(1)));
  __ Ret(HasArgsInRegisters() ? 0 : 2);

  // Slow-case.  Tail call builtin.
  __ bind(&slow);
  if (!ReturnTrueFalseObject()) {
    if (HasArgsInRegisters()) {
      __ Push(r0, r1);
    }
  __ InvokeBuiltin(Builtins::INSTANCE_OF, JUMP_FUNCTION);
  } else {
    Label ltrue, lfalse;
    __ EnterInternalFrame();
    __ Push(r0, r1);
    __ InvokeBuiltin(Builtins::INSTANCE_OF, CALL_FUNCTION);
    __ LeaveInternalFrame();
    __ cmp(r0, Operand(0));
    __ bf_near(&lfalse);
    __ LoadRoot(r0, Heap::kTrueValueRootIndex);
    __ jmp_near(&ltrue);
    __ bind(&lfalse);
    __ LoadRoot(r0, Heap::kFalseValueRootIndex);
    __ bind(&ltrue);
    __ Ret(HasArgsInRegisters() ? 0 : 2);
  }
}


Register InstanceofStub::left() { return r0; }


Register InstanceofStub::right() { return r1; }


void ArgumentsAccessStub::GenerateReadElement(MacroAssembler* masm) {
  // The displacement is the offset of the last parameter (if any)
  // relative to the frame pointer.
  static const int kDisplacement =
      StandardFrameConstants::kCallerSPOffset - kPointerSize;

  // Check that the key is a smi.
  Label slow;
  __ JumpIfNotSmi(r1, &slow, Label::kNear);

  // Check if the calling frame is an arguments adaptor frame.
  Label adaptor;
  __ ldr(r2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ ldr(r3, MemOperand(r2, StandardFrameConstants::kContextOffset));
  __ cmp(r3, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ b(eq, &adaptor, Label::kNear);

  // Check index against formal parameters count limit passed in
  // through register r0. Use unsigned comparison to get negative
  // check for free.
  __ cmphs(r1, r0);
  __ bt_near(&slow);

  // Read the argument from the stack and return it.
  __ sub(r3, r0, r1);
  __ lsl(r0, r3, Operand(kPointerSizeLog2 - kSmiTagSize));
  __ add(r3, fp, r0);
  __ ldr(r0, MemOperand(r3, kDisplacement));
  __ rts();

  // Arguments adaptor case: Check index against actual arguments
  // limit found in the arguments adaptor frame. Use unsigned
  // comparison to get negative check for free.
  __ bind(&adaptor);
  __ ldr(r0, MemOperand(r2, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ cmphs(r1, r0);
  __ bt_near(&slow);

  // Read the argument from the adaptor frame and return it.
  __ sub(r3, r0, r1);
  __ lsl(r0, r3, Operand(kPointerSizeLog2 - kSmiTagSize));
  __ add(r3, r2, r0);
  __ ldr(r0, MemOperand(r3, kDisplacement));
  __ rts();

  // Slow-case: Handle non-smi or out-of-bounds access to arguments
  // by calling the runtime system.
  __ bind(&slow);
  __ push(r1);
  __ TailCallRuntime(Runtime::kGetArgumentsProperty, 1, 1);
}


void ArgumentsAccessStub::GenerateNewNonStrictSlow(MacroAssembler* masm) {
  // sp[0] : number of parameters
  // sp[4] : receiver displacement
  // sp[8] : function

  // Check if the calling frame is an arguments adaptor frame.
  Label runtime;
  __ ldr(r3, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ ldr(r2, MemOperand(r3, StandardFrameConstants::kContextOffset));
  __ cmp(r2, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ b(ne, &runtime, Label::kNear);

  // Patch the arguments.length and the parameters pointer in the current frame.
  __ ldr(r2, MemOperand(r3, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ str(r2, MemOperand(sp, 0 * kPointerSize));
  __ lsl(ip, r2, Operand(1));
  __ add(r3, r3, ip);
  __ add(r3, r3, Operand(StandardFrameConstants::kCallerSPOffset));
  __ str(r3, MemOperand(sp, 1 * kPointerSize));

  __ bind(&runtime);
  __ TailCallRuntime(Runtime::kNewArgumentsFast, 3, 1);
}


void ArgumentsAccessStub::GenerateNewNonStrictFast(MacroAssembler* masm) {
  // Stack layout:
  //  sp[0] : number of parameters (tagged)
  //  sp[4] : address of receiver argument
  //  sp[8] : function
  // Registers used over whole function:
  //  r6 : allocated object (tagged)
  //  r9 : mapped parameter count (tagged)

  __ ldr(r1, MemOperand(sp, 0 * kPointerSize));
  // r1 = parameter count (tagged)

  // Check if the calling frame is an arguments adaptor frame.
  Label runtime;
  Label adaptor_frame, try_allocate;
  __ ldr(r3, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ ldr(r2, MemOperand(r3, StandardFrameConstants::kContextOffset));
  __ cmp(r2, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ b(eq, &adaptor_frame, Label::kNear);

  // No adaptor, parameter count = argument count.
  __ mov(r2, r1);
  __ b_near(&try_allocate);

  // We have an adaptor frame. Patch the parameters pointer.
  __ bind(&adaptor_frame);
  __ ldr(r2, MemOperand(r3, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ lsl(ip, r2, Operand(1));
  __ add(r3, r3, ip);
  __ add(r3, r3, Operand(StandardFrameConstants::kCallerSPOffset));
  __ str(r3, MemOperand(sp, 1 * kPointerSize));

  // r1 = parameter count (tagged)
  // r2 = argument count (tagged)
  // Compute the mapped parameter count = min(r1, r2) in r1.
  __ cmpgt(r1, r2);
  __ mov(r1, r2, t);

  __ bind(&try_allocate);

  // Compute the sizes of backing store, parameter map, and arguments object.
  // 1. Parameter map, has 2 extra words containing context and backing store.
  const int kParameterMapHeaderSize =
      FixedArray::kHeaderSize + 2 * kPointerSize;
  // If there are no mapped parameters, we do not need the parameter_map.
  __ cmpeq(r1, Operand(Smi::FromInt(0)));
  __ mov(r9, Operand(0), t);
  Label skip;
  __ bt_near(&skip);
  __ lsl(r9, r1, Operand(1));
  __ add(r9, r9, Operand(kParameterMapHeaderSize));
  __ bind(&skip);

  // 2. Backing store.
  __ lsl(ip, r2, Operand(1));
  __ add(r9, r9, ip);
  __ add(r9, r9, Operand(FixedArray::kHeaderSize));

  // 3. Arguments object.
  __ add(r9, r9, Operand(Heap::kArgumentsObjectSize));

  // Do the allocation of all three objects in one go.
  __ AllocateInNewSpace(r9, r0, r3, r4, &runtime, TAG_OBJECT);

  // r0 = address of new object(s) (tagged)
  // r2 = argument count (tagged)
  // Get the arguments boilerplate from the current (global) context into r4.
  const int kNormalOffset =
      Context::SlotOffset(Context::ARGUMENTS_BOILERPLATE_INDEX);
  const int kAliasedOffset =
      Context::SlotOffset(Context::ALIASED_ARGUMENTS_BOILERPLATE_INDEX);

  __ ldr(r4, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  __ ldr(r4, FieldMemOperand(r4, GlobalObject::kGlobalContextOffset));
  __ cmp(r1, Operand(0));
  Label lf, end;
  __ bf_near(&lf);
  __ ldr(r4, MemOperand(r4, kNormalOffset));
  __ b_near(&end);
  __ bind(&lf);
  __ ldr(r4, MemOperand(r4, kAliasedOffset));
  __ bind(&end);

  // r0 = address of new object (tagged)
  // r1 = mapped parameter count (tagged)
  // r2 = argument count (tagged)
  // r4 = address of boilerplate object (tagged)
  // Copy the JS object part.
  for (int i = 0; i < JSObject::kHeaderSize; i += kPointerSize) {
    __ ldr(r3, FieldMemOperand(r4, i));
    __ str(r3, FieldMemOperand(r0, i));
  }

  // Setup the callee in-object property.
  STATIC_ASSERT(Heap::kArgumentsCalleeIndex == 1);
  __ ldr(r3, MemOperand(sp, 2 * kPointerSize));
  const int kCalleeOffset = JSObject::kHeaderSize +
      Heap::kArgumentsCalleeIndex * kPointerSize;
  __ str(r3, FieldMemOperand(r0, kCalleeOffset));

  // Use the length (smi tagged) and set that as an in-object property too.
  STATIC_ASSERT(Heap::kArgumentsLengthIndex == 0);
  const int kLengthOffset = JSObject::kHeaderSize +
      Heap::kArgumentsLengthIndex * kPointerSize;
  __ str(r2, FieldMemOperand(r0, kLengthOffset));

  // Setup the elements pointer in the allocated arguments object.
  // If we allocated a parameter map, r4 will point there, otherwise
  // it will point to the backing store.
  __ add(r4, r0, Operand(Heap::kArgumentsObjectSize));
  __ str(r4, FieldMemOperand(r0, JSObject::kElementsOffset));

  // r0 = address of new object (tagged)
  // r1 = mapped parameter count (tagged)
  // r2 = argument count (tagged)
  // r4 = address of parameter map or backing store (tagged)
  // Initialize parameter map. If there are no mapped arguments, we're done.
  Label skip_parameter_map;
  __ cmp(r1, Operand(Smi::FromInt(0)));
  // Move backing store address to r3, because it is
  // expected there when filling in the unmapped arguments.
  __ mov(r3, r4, eq);
  __ b(eq, &skip_parameter_map);

  __ LoadRoot(r6, Heap::kNonStrictArgumentsElementsMapRootIndex);
  __ str(r6, FieldMemOperand(r4, FixedArray::kMapOffset));
  __ add(r6, r1, Operand(Smi::FromInt(2)));
  __ str(r6, FieldMemOperand(r4, FixedArray::kLengthOffset));
  __ str(cp, FieldMemOperand(r4, FixedArray::kHeaderSize + 0 * kPointerSize));
  __ lsl(r6, r1, Operand(1));
  __ add(r6, r4, r6);
  __ add(r6, r6, Operand(kParameterMapHeaderSize));
  __ str(r6, FieldMemOperand(r4, FixedArray::kHeaderSize + 1 * kPointerSize));

  // Copy the parameter slots and the holes in the arguments.
  // We need to fill in mapped_parameter_count slots. They index the context,
  // where parameters are stored in reverse order, at
  //   MIN_CONTEXT_SLOTS .. MIN_CONTEXT_SLOTS+parameter_count-1
  // The mapped parameter thus need to get indices
  //   MIN_CONTEXT_SLOTS+parameter_count-1 ..
  //       MIN_CONTEXT_SLOTS+parameter_count-mapped_parameter_count
  // We loop from right to left.
  Label parameters_loop, parameters_test;
  __ mov(r6, r1);
  __ ldr(r9, MemOperand(sp, 0 * kPointerSize));
  __ add(r9, r9, Operand(Smi::FromInt(Context::MIN_CONTEXT_SLOTS)));
  __ sub(r9, r9, r1);
  __ LoadRoot(r7, Heap::kTheHoleValueRootIndex);
  __ lsl(r3, r6, Operand(1));
  __ add(r3, r4, r3);
  __ add(r3, r3, Operand(kParameterMapHeaderSize));

  // r6 = loop variable (tagged)
  // r1 = mapping index (tagged)
  // r3 = address of backing store (tagged)
  // r4 = address of parameter map (tagged)
  // r5 = temporary scratch (a.o., for address calculation)
  // r7 = the hole value
  __ jmp_near(&parameters_test);

  __ bind(&parameters_loop);
  __ sub(r6, r6, Operand(Smi::FromInt(1)));
  __ lsl(r5, r6, Operand(1));
  __ add(r5, r5, Operand(kParameterMapHeaderSize - kHeapObjectTag));
  __ str(r9, MemOperand(r4, r5));
  __ sub(r5, r5, Operand(kParameterMapHeaderSize - FixedArray::kHeaderSize));
  __ str(r7, MemOperand(r3, r5));
  __ add(r9, r9, Operand(Smi::FromInt(1)));
  __ bind(&parameters_test);
  __ cmp(r6, Operand(Smi::FromInt(0)));
  __ b(ne, &parameters_loop);

  __ bind(&skip_parameter_map);
  // r2 = argument count (tagged)
  // r3 = address of backing store (tagged)
  // r5 = scratch
  // Copy arguments header and remaining slots (if there are any).
  __ LoadRoot(r5, Heap::kFixedArrayMapRootIndex);
  __ str(r5, FieldMemOperand(r3, FixedArray::kMapOffset));
  __ str(r2, FieldMemOperand(r3, FixedArray::kLengthOffset));

  Label arguments_loop, arguments_test;
  __ mov(r9, r1);
  __ ldr(r4, MemOperand(sp, 1 * kPointerSize));
  __ lsl(r5, r9, Operand(1));
  __ sub(r4, r4, r5);
  __ jmp_near(&arguments_test);

  __ bind(&arguments_loop);
  __ sub(r4, r4, Operand(kPointerSize));
  __ ldr(r6, MemOperand(r4, 0));
  __ lsl(r5, r9, Operand(1));
  __ add(r5, r3, r5);
  __ str(r6, FieldMemOperand(r5, FixedArray::kHeaderSize));
  __ add(r9, r9, Operand(Smi::FromInt(1)));

  __ bind(&arguments_test);
  __ cmpge(r9, r2);
  __ bf(&arguments_loop);

  // Return and remove the on-stack parameters.
  __ add(sp, sp, Operand(3 * kPointerSize));
  __ Ret();

  // Do the runtime call to allocate the arguments object.
  // r2 = argument count (taggged)
  __ bind(&runtime);
  __ str(r2, MemOperand(sp, 0 * kPointerSize));  // Patch argument count.
  __ TailCallRuntime(Runtime::kNewArgumentsFast, 3, 1);
}


void ArgumentsAccessStub::GenerateNewStrict(MacroAssembler* masm) {
  // sp[0] : number of parameters
  // sp[4] : receiver displacement
  // sp[8] : function
  // Check if the calling frame is an arguments adaptor frame.
  Label adaptor_frame, try_allocate, runtime;
  __ ldr(r2, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ ldr(r3, MemOperand(r2, StandardFrameConstants::kContextOffset));
  __ cmp(r3, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ b(eq, &adaptor_frame, Label::kNear);

  // Get the length from the frame.
  __ ldr(r1, MemOperand(sp, 0));
  __ b_near(&try_allocate);

  // Patch the arguments.length and the parameters pointer.
  __ bind(&adaptor_frame);
  __ ldr(r1, MemOperand(r2, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ str(r1, MemOperand(sp, 0));
  __ lsl(r3, r1, Operand(kPointerSizeLog2 - kSmiTagSize));
  __ add(r3, r2, r3);
  __ add(r3, r3, Operand(StandardFrameConstants::kCallerSPOffset));
  __ str(r3, MemOperand(sp, 1 * kPointerSize));

  // Try the new space allocation. Start out with computing the size
  // of the arguments object and the elements array in words.
  Label add_arguments_object;
  __ bind(&try_allocate);
  __ cmp(r1, Operand(0, RelocInfo::NONE));
  __ b(eq, &add_arguments_object, Label::kNear);
  __ lsr(r1, r1, Operand(kSmiTagSize));
  __ add(r1, r1, Operand(FixedArray::kHeaderSize / kPointerSize));
  __ bind(&add_arguments_object);
  __ add(r1, r1, Operand(Heap::kArgumentsObjectSizeStrict / kPointerSize));

  // Do the allocation of both objects in one go.
  __ AllocateInNewSpace(r1,  // object size
                        r0,  // result
                        r2,  // scratch1
                        r3,  // scratch2
                        &runtime,
                        static_cast<AllocationFlags>(TAG_OBJECT |
                                                     SIZE_IN_WORDS));

  // Get the arguments boilerplate from the current (global) context.
  __ ldr(r4, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  __ ldr(r4, FieldMemOperand(r4, GlobalObject::kGlobalContextOffset));
  __ ldr(r4, MemOperand(r4, Context::SlotOffset(
      Context::STRICT_MODE_ARGUMENTS_BOILERPLATE_INDEX)));

  // Copy the JS object part.
  __ CopyFields(r0, r4, r3.bit(), JSObject::kHeaderSize / kPointerSize);

  // Get the length (smi tagged) and set that as an in-object property too.
  STATIC_ASSERT(Heap::kArgumentsLengthIndex == 0);
  __ ldr(r1, MemOperand(sp, 0 * kPointerSize));
  __ str(r1, FieldMemOperand(r0, JSObject::kHeaderSize +
      Heap::kArgumentsLengthIndex * kPointerSize));

  // If there are no actual arguments, we're done.
  Label done;
  __ cmp(r1, Operand(0, RelocInfo::NONE));
  __ b(eq, &done, Label::kNear);

  // Get the parameters pointer from the stack.
  __ ldr(r2, MemOperand(sp, 1 * kPointerSize));

  // Setup the elements pointer in the allocated arguments object and
  // initialize the header in the elements fixed array.
  __ add(r4, r0, Operand(Heap::kArgumentsObjectSizeStrict));
  __ str(r4, FieldMemOperand(r0, JSObject::kElementsOffset));
  __ LoadRoot(r3, Heap::kFixedArrayMapRootIndex);
  __ str(r3, FieldMemOperand(r4, FixedArray::kMapOffset));
  __ str(r1, FieldMemOperand(r4, FixedArray::kLengthOffset));
  // Untag the length for the loop.
  __ lsr(r1, r1, Operand(kSmiTagSize));

  // Copy the fixed array slots.
  Label loop;
  // Setup r4 to point to the first array slot.
  __ add(r4, r4, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  __ bind(&loop);
  // Pre-decrement r2 with kPointerSize on each iteration.
  // Pre-decrement in order to skip receiver.
  __ sub(r2, r2, Operand(kPointerSize));
  __ ldr(r3, MemOperand(r2));
  // Post-increment r4 with kPointerSize on each iteration.
  __ str(r3, MemOperand(r4, kPointerSize, PostIndex));
  __ sub(r1, r1, Operand(1));
  __ cmp(r1, Operand(0, RelocInfo::NONE));
  __ b(ne, &loop);

  // Return and remove the on-stack parameters.
  __ bind(&done);
  __ add(sp, sp, Operand(3 * kPointerSize));
  __ Ret();

  // Do the runtime call to allocate the arguments object.
  __ bind(&runtime);
  __ TailCallRuntime(Runtime::kNewStrictArgumentsFast, 3, 1);
}


void RegExpExecStub::Generate(MacroAssembler* masm) {
  // Just jump directly to runtime if native RegExp is not selected at compile
  // time or if regexp entry in generated code is turned off runtime switch or
  // at compilation.
#ifdef V8_INTERPRETED_REGEXP
  __ TailCallRuntime(Runtime::kRegExpExec, 4, 1);
#else  // V8_INTERPRETED_REGEXP
  if (!FLAG_regexp_entry_native) {
    __ TailCallRuntime(Runtime::kRegExpExec, 4, 1);
    return;
  }

  // Stack frame on entry.
  //  sp[0]: last_match_info (expected JSArray)
  //  sp[4]: previous index
  //  sp[8]: subject string
  //  sp[12]: JSRegExp object

  static const int kLastMatchInfoOffset = 0 * kPointerSize;
  static const int kPreviousIndexOffset = 1 * kPointerSize;
  static const int kSubjectOffset = 2 * kPointerSize;
  static const int kJSRegExpOffset = 3 * kPointerSize;

  Label runtime;

  // Allocation of registers for this function. These are in callee save
  // registers and will be preserved by the call to the native RegExp code, as
  // this code is called using the normal C calling convention. When calling
  // directly from generated code the native RegExp code will not do a GC and
  // therefore the content of these registers are safe to use after the call.
  Register subject = r4;
  Register regexp_data = r5;
  Register last_match_info_elements = r6;

  // Ensure that a RegExp stack is allocated.
  Isolate* isolate = masm->isolate();
  ExternalReference address_of_regexp_stack_memory_address =
      ExternalReference::address_of_regexp_stack_memory_address(isolate);
  ExternalReference address_of_regexp_stack_memory_size =
      ExternalReference::address_of_regexp_stack_memory_size(isolate);
  __ mov(r0, Operand(address_of_regexp_stack_memory_size));
  __ ldr(r0, MemOperand(r0, 0));
  __ tst(r0, r0);
  __ b(eq, &runtime);

  // Check that the first argument is a JSRegExp object.
  __ ldr(r0, MemOperand(sp, kJSRegExpOffset));
  STATIC_ASSERT(kSmiTag == 0);
  __ JumpIfSmi(r0, &runtime);
  __ CompareObjectType(r0, r1, r1, JS_REGEXP_TYPE, eq);
  __ b(ne, &runtime);

  // Check that the RegExp has been compiled (data contains a fixed array).
  __ ldr(regexp_data, FieldMemOperand(r0, JSRegExp::kDataOffset));
  if (FLAG_debug_code) {
    __ tst(regexp_data, Operand(kSmiTagMask));
    __ Check(ne, "Unexpected type for RegExp data, FixedArray expected");
    __ CompareObjectType(regexp_data, r0, r0, FIXED_ARRAY_TYPE, eq);
    __ Check(eq, "Unexpected type for RegExp data, FixedArray expected");
  }

  // regexp_data: RegExp data (FixedArray)
  // Check the type of the RegExp. Only continue if type is JSRegExp::IRREGEXP.
  __ ldr(r0, FieldMemOperand(regexp_data, JSRegExp::kDataTagOffset));
  __ cmp(r0, Operand(Smi::FromInt(JSRegExp::IRREGEXP)));
  __ b(ne, &runtime);

  // regexp_data: RegExp data (FixedArray)
  // Check that the number of captures fit in the static offsets vector buffer.
  __ ldr(r2,
         FieldMemOperand(regexp_data, JSRegExp::kIrregexpCaptureCountOffset));
  // Calculate number of capture registers (number_of_captures + 1) * 2. This
  // uses the asumption that smis are 2 * their untagged value.
  STATIC_ASSERT(kSmiTag == 0);
  STATIC_ASSERT(kSmiTagSize + kSmiShiftSize == 1);
  __ add(r2, r2, Operand(2));  // r2 was a smi.
  // Check that the static offsets vector buffer is large enough.
  __ cmphi(r2, Operand(OffsetsVector::kStaticOffsetsVectorSize));
  __ b(eq, &runtime);

  // r2: Number of capture registers
  // regexp_data: RegExp data (FixedArray)
  // Check that the second argument is a string.
  __ ldr(subject, MemOperand(sp, kSubjectOffset));
  __ JumpIfSmi(subject, &runtime);
  Condition is_string = masm->IsObjectStringType(subject, r0);
  __ b(NegateCondition(is_string), &runtime);
  // Get the length of the string to r3.
  __ ldr(r3, FieldMemOperand(subject, String::kLengthOffset));

  // r2: Number of capture registers
  // r3: Length of subject string as a smi
  // subject: Subject string
  // regexp_data: RegExp data (FixedArray)
  // Check that the third argument is a positive smi less than the subject
  // string length. A negative value will be greater (unsigned comparison).
  __ ldr(r0, MemOperand(sp, kPreviousIndexOffset));
  __ JumpIfNotSmi(r0, &runtime);
  __ cmphi(r3, r0);
  __ bf(&runtime);

  // r2: Number of capture registers
  // subject: Subject string
  // regexp_data: RegExp data (FixedArray)
  // Check that the fourth object is a JSArray object.
  __ ldr(r0, MemOperand(sp, kLastMatchInfoOffset));
  __ JumpIfSmi(r0, &runtime);
  __ CompareObjectType(r0, r1, r1, JS_ARRAY_TYPE, eq);
  __ b(ne, &runtime);
  // Check that the JSArray is in fast case.
  __ ldr(last_match_info_elements,
         FieldMemOperand(r0, JSArray::kElementsOffset));
  __ ldr(r0, FieldMemOperand(last_match_info_elements, HeapObject::kMapOffset));
  __ LoadRoot(ip, Heap::kFixedArrayMapRootIndex);
  __ cmp(r0, ip);
  __ b(ne, &runtime);
  // Check that the last match info has space for the capture registers and the
  // additional information.
  __ ldr(r0,
         FieldMemOperand(last_match_info_elements, FixedArray::kLengthOffset));
  __ add(r2, r2, Operand(RegExpImpl::kLastMatchOverhead));
  __ asr(ip, r0, Operand(kSmiTagSize));
  __ cmpgt(r2, ip);
  __ bt(&runtime);

  // Reset offset for possibly sliced string.
  __ mov(r9, Operand(0));
  // subject: Subject string
  // regexp_data: RegExp data (FixedArray)
  // Check the representation and encoding of the subject string.
  Label seq_string;
  __ ldr(r0, FieldMemOperand(subject, HeapObject::kMapOffset));
  __ ldrb(r0, FieldMemOperand(r0, Map::kInstanceTypeOffset));
  // First check for flat string.
  __ land(r1, r0, Operand(kIsNotStringMask | kStringRepresentationMask));
  __ tst(r1, r1);
  STATIC_ASSERT((kStringTag | kSeqStringTag) == 0);
  __ b(eq, &seq_string);

  // subject: Subject string
  // regexp_data: RegExp data (FixedArray)
  // Check for flat cons string or sliced string.
  // A flat cons string is a cons string where the second part is the empty
  // string. In that case the subject string is just the first part of the cons
  // string. Also in this case the first part of the cons string is known to be
  // a sequential string or an external string.
  // In the case of a sliced string its offset has to be taken into account.
  Label cons_string, check_encoding;
  STATIC_ASSERT(kConsStringTag < kExternalStringTag);
  STATIC_ASSERT(kSlicedStringTag > kExternalStringTag);
  __ cmpge(r1, Operand(kExternalStringTag));
  __ bf(&cons_string);
  __ cmpeq(r1, Operand(kExternalStringTag));
  __ b(eq, &runtime);

  // String is sliced.
  __ ldr(r9, FieldMemOperand(subject, SlicedString::kOffsetOffset));
  __ asr(r9, r9, Operand(kSmiTagSize));
  __ ldr(subject, FieldMemOperand(subject, SlicedString::kParentOffset));
  // r9: offset of sliced string, smi-tagged.
  __ jmp(&check_encoding);
  // String is a cons string, check whether it is flat.
  __ bind(&cons_string);
  __ ldr(r0, FieldMemOperand(subject, ConsString::kSecondOffset));
  __ LoadRoot(r1, Heap::kEmptyStringRootIndex);
  __ cmp(r0, r1);
  __ b(ne, &runtime);
  __ ldr(subject, FieldMemOperand(subject, ConsString::kFirstOffset));
  // Is first part of cons or parent of slice a flat string?
  __ bind(&check_encoding);
  __ ldr(r0, FieldMemOperand(subject, HeapObject::kMapOffset));
  __ ldrb(r0, FieldMemOperand(r0, Map::kInstanceTypeOffset));
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(r0, Operand(kStringRepresentationMask));
  __ b(ne, &runtime);
  __ bind(&seq_string);
  // subject: Subject string
  // regexp_data: RegExp data (FixedArray)
  // r0: Instance type of subject string
  STATIC_ASSERT(4 == kAsciiStringTag);
  STATIC_ASSERT(kTwoByteStringTag == 0);
  // Find the code object based on the assumptions above.
  __ land(r0, r0, Operand(kStringEncodingMask));
  __ asr(r3, r0, Operand(2));
  __ tst(r3, r3);
  Label skip_true, skip_end;
  __ bt_near(&skip_true);
  __ ldr(r7, FieldMemOperand(regexp_data, JSRegExp::kDataAsciiCodeOffset));
  __ b_near(&skip_end);
  __ bind(&skip_true);
  __ ldr(r7, FieldMemOperand(regexp_data, JSRegExp::kDataUC16CodeOffset));
  __ bind(&skip_end);

  // Check that the irregexp code has been generated for the actual string
  // encoding. If it has, the field contains a code object otherwise it contains
  // a smi (code flushing support).
  __ JumpIfSmi(r7, &runtime);

  // r3: encoding of subject string (1 if ASCII, 0 if two_byte);
  // r7: code
  // subject: Subject string
  // regexp_data: RegExp data (FixedArray)
  // Load used arguments before starting to push arguments for call to native
  // RegExp code to avoid handling changing stack height.
  __ ldr(r1, MemOperand(sp, kPreviousIndexOffset));
  __ asr(r1, r1, Operand(kSmiTagSize));

  // r1: previous index
  // r3: encoding of subject string (1 if ASCII, 0 if two_byte);
  // r7: code
  // subject: Subject string
  // regexp_data: RegExp data (FixedArray)
  // All checks done. Now push arguments for native regexp code.
  __ IncrementCounter(isolate->counters()->regexp_entry_native(), 1, r0, r2);

  // Save r4-r7 as they are going to be used afterward in the C code
  __ Push(r4, r5, r6, r7);

  // Isolates: note we add an additional parameter here (isolate pointer).
  static const int kRegExpExecuteArguments = 8;
  static const int kParameterRegisters = 4;
  __ EnterExitFrame(false, kRegExpExecuteArguments - kParameterRegisters);

  // Stack pointer now points to cell where return address is to be written.
  // Arguments are before that on the stack or in registers.

  // Argument 8 (sp[16]): Pass current isolate address.
  __ mov(r0, Operand(ExternalReference::isolate_address()));
  __ str(r0, MemOperand(sp, 4 * kPointerSize));

  // Argument 7 (sp[12]): Indicate that this is a direct call from JavaScript.
  __ mov(r0, Operand(1));
  __ str(r0, MemOperand(sp, 3 * kPointerSize));

  // Argument 6 (sp[8]): Start (high end) of backtracking stack memory area.
  __ mov(r0, Operand(address_of_regexp_stack_memory_address));
  __ ldr(r0, MemOperand(r0, 0));
  __ mov(r2, Operand(address_of_regexp_stack_memory_size));
  __ ldr(r2, MemOperand(r2, 0));
  __ add(r0, r0, r2);
  __ str(r0, MemOperand(sp, 2 * kPointerSize));

  // Argument 5 (sp[4]): static offsets vector buffer.
  __ mov(r0,
         Operand(ExternalReference::address_of_static_offsets_vector(isolate)));
  __ str(r0, MemOperand(sp, 1 * kPointerSize));

  // For arguments 4 and 3 get string length, calculate start of string data and
  // calculate the shift of the index (0 for ASCII and 1 for two byte).
  STATIC_ASSERT(SeqAsciiString::kHeaderSize == SeqTwoByteString::kHeaderSize);
  __ add(sh4_r8, subject, Operand(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  __ eor(r3, r3, Operand(1));
  // Load the length from the original subject string from the previous stack
  // frame. Therefore we have to use fp, which points exactly to two pointer
  // sizes below the previous sp. (Because creating a new stack frame pushes
  // the previous fp onto the stack and moves up sp by 2 * kPointerSize.)
  // We also have to take into account the 4 registers pushed on the stack [r4, r7]
  __ ldr(subject, MemOperand(fp, kSubjectOffset + 2 * kPointerSize + 4 * kPointerSize));
  // If slice offset is not 0, load the length from the original sliced string.
  // Argument 4, r3: End of string data
  // Argument 3, r2: Start of string data
  // Prepare start and end index of the input.
  __ lsl(r9, r9, r3);
  __ add(r9, sh4_r8, r9);
  __ lsl(ip, r1, r3);
  __ add(r2, r9, ip);

  __ ldr(sh4_r8, FieldMemOperand(subject, String::kLengthOffset));
  __ asr(sh4_r8, sh4_r8, Operand(kSmiTagSize));
  __ lsl(r3, sh4_r8, r3);
  __ add(r3, r9, r3);

  // Argument 2 (r1): Previous index.
  // Already there

  // Argument 1 (r0): Subject string.
  __ mov(r0, subject);

  __ mov(r4, r0);
  __ mov(r5, r1);
  __ mov(r6, r2);

  // Locate the code entry and call it.
  __ add(r0, r7, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ mov(r7, r3);

  DirectCEntryStub stub(r2);
  stub.GenerateCall(masm, r0, r3);

  __ LeaveExitFrame(false, no_reg);
  __ Pop(r4, r5, r6, r7);

  // r0: result
  // subject: subject string (callee saved)
  // regexp_data: RegExp data (callee saved)
  // last_match_info_elements: Last match info elements (callee saved)

  // Check the result.
  Label success;

  __ cmp(r0, Operand(NativeRegExpMacroAssembler::SUCCESS));
  __ b(eq, &success);
  Label failure;
  __ cmp(r0, Operand(NativeRegExpMacroAssembler::FAILURE));
  __ b(eq, &failure);
  __ cmp(r0, Operand(NativeRegExpMacroAssembler::EXCEPTION));
  // If not exception it can only be retry. Handle that in the runtime system.
  __ b(ne, &runtime);
  // Result must now be exception. If there is no pending exception already a
  // stack overflow (on the backtrack stack) was detected in RegExp code but
  // haven't created the exception yet. Handle that in the runtime system.
  // TODO(592): Rerunning the RegExp to get the stack overflow exception.
  __ mov(r1, Operand(ExternalReference::the_hole_value_location(isolate)));
  __ ldr(r1, MemOperand(r1, 0));
  __ mov(r2, Operand(ExternalReference(Isolate::kPendingExceptionAddress,
                                       isolate)));
  __ ldr(r0, MemOperand(r2, 0));
  __ cmp(r0, r1);
  __ b(eq, &runtime);

  __ str(r1, MemOperand(r2, 0));  // Clear pending exception.

  // Check if the exception is a termination. If so, throw as uncatchable.
  __ CompareRoot(r0, Heap::kTerminationExceptionRootIndex);

  Label termination_exception;
  __ b(eq, &termination_exception);

  __ Throw(r0);  // Expects thrown value in r0.

  __ bind(&termination_exception);
  __ ThrowUncatchable(TERMINATION, r0);  // Expects thrown value in r0.

  __ bind(&failure);
  // For failure and exception return null.
  __ mov(r0, Operand(masm->isolate()->factory()->null_value()));
  __ add(sp, sp, Operand(4 * kPointerSize));
  __ Ret();

  // Process the result from the native regexp code.
  __ bind(&success);
  __ ldr(r1,
         FieldMemOperand(regexp_data, JSRegExp::kIrregexpCaptureCountOffset));
  // Calculate number of capture registers (number_of_captures + 1) * 2.
  STATIC_ASSERT(kSmiTag == 0);
  STATIC_ASSERT(kSmiTagSize + kSmiShiftSize == 1);
  __ add(r1, r1, Operand(2));  // r1 was a smi.

  // r1: number of capture registers
  // r4: subject string
  // Store the capture count.
  __ lsl(r2, r1, Operand(kSmiTagSize + kSmiShiftSize));  // To smi.
  __ str(r2, FieldMemOperand(last_match_info_elements,
                             RegExpImpl::kLastCaptureCountOffset));
  // Store last subject and last input.
  __ mov(r3, last_match_info_elements);  // Moved up to reduce latency.
  __ str(subject,
         FieldMemOperand(last_match_info_elements,
                         RegExpImpl::kLastSubjectOffset));
  __ RecordWrite(r3, Operand(RegExpImpl::kLastSubjectOffset), r2, r7);
  __ str(subject,
         FieldMemOperand(last_match_info_elements,
                         RegExpImpl::kLastInputOffset));
  __ mov(r3, last_match_info_elements);
  __ RecordWrite(r3, Operand(RegExpImpl::kLastInputOffset), r2, r7);

  // Get the static offsets vector filled by the native regexp code.
  ExternalReference address_of_static_offsets_vector =
      ExternalReference::address_of_static_offsets_vector(isolate);
  __ mov(r2, Operand(address_of_static_offsets_vector));

  // r1: number of capture registers
  // r2: offsets vector
  Label next_capture, done;
  // Capture register counter starts from number of capture registers and
  // counts down until wraping after zero.
  __ add(r0,
         last_match_info_elements,
         Operand(RegExpImpl::kFirstCaptureOffset - kHeapObjectTag));
  __ bind(&next_capture);
  __ sub(r1, r1, Operand(1));
  __ cmpge(r1, Operand(0));
  __ bf(&done);
  // Read the value from the static offsets vector buffer.
  __ ldr(r3, MemOperand(r2, kPointerSize, PostIndex));
  // Store the smi value in the last match info.
  __ lsl(r3, r3, Operand(kSmiTagSize));
  __ str(r3, MemOperand(r0, kPointerSize, PostIndex));
  __ jmp(&next_capture);
  __ bind(&done);

  // Return last match info.
  __ ldr(r0, MemOperand(sp, kLastMatchInfoOffset));
  __ add(sp, sp, Operand(4 * kPointerSize));
  __ Ret();

  // Do the runtime call to execute the regexp.
  __ bind(&runtime);
  __ TailCallRuntime(Runtime::kRegExpExec, 4, 1);

#endif  // V8_INTERPRETED_REGEXP
}


void RegExpConstructResultStub::Generate(MacroAssembler* masm) {
  const int kMaxInlineLength = 100;
  Label slowcase;
  Label done;
  Factory* factory = masm->isolate()->factory();

  __ ldr(r1, MemOperand(sp, kPointerSize * 2));
  STATIC_ASSERT(kSmiTag == 0);
  STATIC_ASSERT(kSmiTagSize == 1);
  __ JumpIfNotSmi(r1, &slowcase);
  __ cmphi(r1, Operand(Smi::FromInt(kMaxInlineLength)));
  __ b(t, &slowcase);
  // Smi-tagging is equivalent to multiplying by 2.
  // Allocate RegExpResult followed by FixedArray with size in ebx.
  // JSArray:   [Map][empty properties][Elements][Length-smi][index][input]
  // Elements:  [Map][Length][..elements..]
  // Size of JSArray with two in-object properties and the header of a
  // FixedArray.
  int objects_size =
      (JSRegExpResult::kSize + FixedArray::kHeaderSize) / kPointerSize;
  __ lsr(r5, r1, Operand(kSmiTagSize + kSmiShiftSize));
  __ add(r2, r5, Operand(objects_size));
  __ AllocateInNewSpace(
      r2,  // In: Size, in words.
      r0,  // Out: Start of allocation (tagged).
      r3,  // Scratch register.
      r4,  // Scratch register.
      &slowcase,
      static_cast<AllocationFlags>(TAG_OBJECT | SIZE_IN_WORDS));
  // r0: Start of allocated area, object-tagged.
  // r1: Number of elements in array, as smi.
  // r5: Number of elements, untagged.

  // Set JSArray map to global.regexp_result_map().
  // Set empty properties FixedArray.
  // Set elements to point to FixedArray allocated right after the JSArray.
  // Interleave operations for better latency.
  __ ldr(r2, ContextOperand(cp, Context::GLOBAL_INDEX));
  __ add(r3, r0, Operand(JSRegExpResult::kSize));
  __ mov(r4, Operand(factory->empty_fixed_array()));
  __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalContextOffset));
  __ str(r3, FieldMemOperand(r0, JSObject::kElementsOffset));
  __ ldr(r2, ContextOperand(r2, Context::REGEXP_RESULT_MAP_INDEX));
  __ str(r4, FieldMemOperand(r0, JSObject::kPropertiesOffset));
  __ str(r2, FieldMemOperand(r0, HeapObject::kMapOffset));

  // Set input, index and length fields from arguments.
  __ ldr(r1, MemOperand(sp, kPointerSize * 0));
  __ str(r1, FieldMemOperand(r0, JSRegExpResult::kInputOffset));
  __ ldr(r1, MemOperand(sp, kPointerSize * 1));
  __ str(r1, FieldMemOperand(r0, JSRegExpResult::kIndexOffset));
  __ ldr(r1, MemOperand(sp, kPointerSize * 2));
  __ str(r1, FieldMemOperand(r0, JSArray::kLengthOffset));

  // Fill out the elements FixedArray.
  // r0: JSArray, tagged.
  // r3: FixedArray, tagged.
  // r5: Number of elements in array, untagged.

  // Set map.
  __ mov(r2, Operand(factory->fixed_array_map()));
  __ str(r2, FieldMemOperand(r3, HeapObject::kMapOffset));
  // Set FixedArray length.
  __ lsl(r6, r5, Operand(kSmiTagSize));
  __ str(r6, FieldMemOperand(r3, FixedArray::kLengthOffset));
  // Fill contents of fixed-array with the-hole.
  __ mov(r2, Operand(factory->the_hole_value()));
  __ add(r3, r3, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  // Fill fixed array elements with hole.
  // r0: JSArray, tagged.
  // r2: the hole.
  // r3: Start of elements in FixedArray.
  // r5: Number of elements to fill.
  Label loop;
  __ cmpgt(r5, Operand(0));
  __ bind(&loop);
  __ b(f, &done, Label::kNear);  // Jump if r1 is negative or zero.
  __ sub(r5, r5, Operand(1));
  __ lsl(ip, r5, Operand(kPointerSizeLog2));
  __ str(r2, MemOperand(r3, ip));
  __ cmpgt(r5, Operand(0));
  __ jmp(&loop);

  __ bind(&done);
  __ add(sp, sp, Operand(3 * kPointerSize));
  __ Ret();

  __ bind(&slowcase);
  __ TailCallRuntime(Runtime::kRegExpConstructResult, 3, 1);
}


void CallFunctionStub::Generate(MacroAssembler* masm) {
  Label slow;

  // The receiver might implicitly be the global object. This is
  // indicated by passing the hole as the receiver to the call
  // function stub.
  if (ReceiverMightBeImplicit()) {
    Label call;
    // Get the receiver from the stack.
    // function, receiver [, arguments]
    __ ldr(r4, MemOperand(sp, argc_ * kPointerSize));
    // Call as function is indicated with the hole.
    __ CompareRoot(r4, Heap::kTheHoleValueRootIndex);
    __ b(ne, &call, Label::kNear);
    // Patch the receiver on the stack with the global receiver object.
    __ ldr(r1, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
    __ ldr(r1, FieldMemOperand(r1, GlobalObject::kGlobalReceiverOffset));
    __ str(r1, MemOperand(sp, argc_ * kPointerSize));
    __ bind(&call);
  }

  // Get the function to call from the stack.
  // function, receiver [, arguments]
  __ ldr(r1, MemOperand(sp, (argc_ + 1) * kPointerSize));

  // Check that the function is really a JavaScript function.
  // r1: pushed function (to be verified)
  __ JumpIfSmi(r1, &slow);
  // Get the map of the function object.
  __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE, eq);
  __ b(ne, &slow);

  // Fast-case: Invoke the function now.
  // r1: pushed function
  ParameterCount actual(argc_);

  if (ReceiverMightBeImplicit()) {
    Label call_as_function;
    __ CompareRoot(r4, Heap::kTheHoleValueRootIndex);
    __ b(eq, &call_as_function, Label::kNear);
    __ InvokeFunction(r1,
                      actual,
                      JUMP_FUNCTION,
                      NullCallWrapper(),
                      CALL_AS_METHOD);
    __ bind(&call_as_function);
  }
  __ InvokeFunction(r1,
                    actual,
                    JUMP_FUNCTION,
                    NullCallWrapper(),
                    CALL_AS_FUNCTION);

  // Slow-case: Non-function called.
  __ bind(&slow);
  // CALL_NON_FUNCTION expects the non-function callee as receiver (instead
  // of the original receiver from the call site).
  __ str(r1, MemOperand(sp, argc_ * kPointerSize));
  __ mov(r0, Operand(argc_));  // Setup the number of arguments.
  __ mov(r2, Operand(0, RelocInfo::NONE));
  __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION);
  __ SetCallKind(r5, CALL_AS_METHOD);
  __ Jump(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
          RelocInfo::CODE_TARGET);
}


// Unfortunately you have to run without snapshots to see most of these
// names in the profile since most compare stubs end up in the snapshot.
void CompareStub::PrintName(StringStream* stream) {
  ASSERT((lhs_.is(r0) && rhs_.is(r1)) ||
         (lhs_.is(r1) && rhs_.is(r0)));
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
  bool is_equality = cc_ == eq || cc_ == ne;
  stream->Add("CompareStub_%s", cc_name);
  stream->Add(lhs_.is(r0) ? "_r0" : "_r1");
  stream->Add(rhs_.is(r0) ? "_r0" : "_r1");
  if (strict_ && is_equality) stream->Add("_STRICT");
  if (never_nan_nan_ && is_equality) stream->Add("_NO_NAN");
  if (!include_number_compare_) stream->Add("_NO_NUMBER");
  if (!include_smi_compare_) stream->Add("_NO_SMI");
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


// StringCharCodeAtGenerator
void StringCharCodeAtGenerator::GenerateFast(MacroAssembler* masm) {
  Label flat_string;
  Label ascii_string;
  Label got_char_code;
  Label sliced_string;

  // If the receiver is a smi trigger the non-string case.
  __ JumpIfSmi(object_, receiver_not_string_);

  // Fetch the instance type of the receiver into result register.
  __ ldr(result_, FieldMemOperand(object_, HeapObject::kMapOffset));
  __ ldrb(result_, FieldMemOperand(result_, Map::kInstanceTypeOffset));
  // If the receiver is not a string trigger the non-string case.
  __ tst(result_, Operand(kIsNotStringMask));
  __ b(ne, receiver_not_string_);

  // If the index is non-smi trigger the non-smi case.
  __ JumpIfNotSmi(index_, &index_not_smi_);

  // Put smi-tagged index into scratch register.
  __ mov(scratch_, index_);
  __ bind(&got_smi_index_);

  // Check for index out of range.
  __ ldr(ip, FieldMemOperand(object_, String::kLengthOffset));
  __ cmphi(ip, scratch_);
  __ bf(index_out_of_range_);

  // We need special handling for non-flat strings.
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(result_, Operand(kStringRepresentationMask));
  __ b(eq, &flat_string);

  // Handle non-flat strings.
  __ land(result_, result_, Operand(kStringRepresentationMask));
  STATIC_ASSERT(kConsStringTag < kExternalStringTag);
  STATIC_ASSERT(kSlicedStringTag > kExternalStringTag);
  __ cmpgt(result_, Operand(kExternalStringTag));
  __ bt_near(&sliced_string);
  __ cmpeq(result_, Operand(kExternalStringTag));
  __ bt(&call_runtime_);

  // ConsString.
  // Check whether the right hand side is the empty string (i.e. if
  // this is really a flat string in a cons string). If that is not
  // the case we would rather go to the runtime system now to flatten
  // the string.
  Label assure_seq_string;
  __ ldr(result_, FieldMemOperand(object_, ConsString::kSecondOffset));
  __ LoadRoot(ip, Heap::kEmptyStringRootIndex);
  __ cmp(result_, ip);
  __ b(ne, &call_runtime_);
  // Get the first of the two strings and load its instance type.
  __ ldr(object_, FieldMemOperand(object_, ConsString::kFirstOffset));
  __ jmp_near(&assure_seq_string);

  // SlicedString, unpack and add offset.
  __ bind(&sliced_string);
  __ ldr(result_, FieldMemOperand(object_, SlicedString::kOffsetOffset));
  __ add(scratch_, scratch_, result_);
  __ ldr(object_, FieldMemOperand(object_, SlicedString::kParentOffset));

  // Assure that we are dealing with a sequential string. Go to runtime if not.
  __ bind(&assure_seq_string);
  __ ldr(result_, FieldMemOperand(object_, HeapObject::kMapOffset));
  __ ldrb(result_, FieldMemOperand(result_, Map::kInstanceTypeOffset));
  // Check that parent is not an external string. Go to runtime otherwise.
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(result_, Operand(kStringRepresentationMask));
  __ b(ne, &call_runtime_);

  // Check for 1-byte or 2-byte string.
  __ bind(&flat_string);
  STATIC_ASSERT((kStringEncodingMask & kAsciiStringTag) != 0);
  STATIC_ASSERT((kStringEncodingMask & kTwoByteStringTag) == 0);
  __ tst(result_, Operand(kStringEncodingMask));
  __ b(ne, &ascii_string, Label::kNear);

  // 2-byte string.
  // Load the 2-byte character code into the result register. We can
  // add without shifting since the smi tag size is the log2 of the
  // number of bytes in a two-byte character.
  STATIC_ASSERT(kSmiTag == 0 && kSmiTagSize == 1 && kSmiShiftSize == 0);
  __ add(scratch_, object_, scratch_);
  __ ldrh(result_, FieldMemOperand(scratch_, SeqTwoByteString::kHeaderSize));
  __ jmp_near(&got_char_code);

  // ASCII string.
  // Load the byte into the result register.
  __ bind(&ascii_string);
  __ lsr(scratch_, scratch_, Operand(kSmiTagSize));
  __ add(scratch_, object_, scratch_);
  __ ldrb(result_, FieldMemOperand(scratch_, SeqAsciiString::kHeaderSize));

  __ bind(&got_char_code);
  __ lsl(result_, result_, Operand(kSmiTagSize));
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
              DONT_DO_SMI_CHECK);
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
         Operand(kSmiTagMask |
                 ((~String::kMaxAsciiCharCode) << kSmiTagSize)));
  __ b(ne, &slow_case_);

  __ LoadRoot(result_, Heap::kSingleCharacterStringCacheRootIndex);
  // At this point code register contains smi tagged ASCII char code.
  STATIC_ASSERT(kSmiTag == 0);
  __ lsl(ip, code_, Operand(kPointerSizeLog2 - kSmiTagSize));
  __ add(result_, result_, ip);
  __ ldr(result_, FieldMemOperand(result_, FixedArray::kHeaderSize));
  __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
  __ cmp(result_, ip);
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


void StringHelper::GenerateCopyCharacters(MacroAssembler* masm,
                                          Register dest,
                                          Register src,
                                          Register count,
                                          Register scratch,
                                          bool ascii) {
  Label loop, done;
  // This loop just copies one character at a time, as it is only used for very
  // short strings.
  if (!ascii) {
    __ add(count, count, count);
  }
  __ cmp(count, Operand(0));
  __ b(eq, &done, Label::kNear);

  __ bind(&loop);
  __ ldrb(scratch, MemOperand(src));
  __ add(src, src, Operand(1));
  // Perform sub between load and dependent store to get the load time to
  // complete.
  __ cmpgt(count, Operand(1));
  __ sub(count, count, Operand(1));
  __ strb(scratch, MemOperand(dest));
  __ add(dest, dest, Operand(1));
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
    __ tst(dest, Operand(kPointerAlignmentMask));
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
  __ cmpeq(count, Operand(0));
  __ bt_near(&done);

  // Assume that you cannot read (or write) unaligned.
  Label byte_loop;
  __ add(count, dest, count);
  Register limit = count;
  // TODO(stm): SH4 implementation is not optimized, always copy byte per byte.
  // See ARM version for example of optimized code.

  // Copy bytes from src to dst until dst hits limit.
  __ bind(&byte_loop);
  __ ldrb(scratch1, MemOperand(src));
  __ add(src, src, Operand(1));
  __ strb(scratch1, MemOperand(dest));
  __ add(dest, dest, Operand(1));
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
  // Register scratch3 is the general scratch register in this function.
  Register scratch = scratch3;

  // Make sure that both characters are not digits as such strings has a
  // different hash algorithm. Don't try to look for these in the symbol table.
  Label not_array_index;
  __ sub(scratch, c1, Operand(static_cast<int>('0')));
  __ cmphi(scratch, Operand(static_cast<int>('9' - '0')));
  __ bt_near(&not_array_index);
  __ sub(scratch, c2, Operand(static_cast<int>('0')));
  __ cmphi(scratch, Operand(static_cast<int>('9' - '0')));

  // If check failed combine both characters into single halfword.
  // This is required by the contract of the method: code at the
  // not_found branch expects this combination in c1 register
  __ lsl(scratch1, c2, Operand(kBitsPerByte));
  __ lor(c1, c1, scratch1, f);
  __ b(f, not_found);

  __ bind(&not_array_index);
  // Calculate the two character string hash.
  Register hash = scratch1;
  StringHelper::GenerateHashInit(masm, hash, c1, scratch);
  StringHelper::GenerateHashAddCharacter(masm, hash, c2, scratch);
  StringHelper::GenerateHashGetHash(masm, hash, scratch);

  // Collect the two characters in a register.
  Register chars = c1;
  __ lsl(scratch, c2, Operand(kBitsPerByte));
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
  __ asr(mask, mask, Operand(1));
  __ sub(mask, mask, Operand(1));

  // Calculate untagged address of the first element of the symbol table.
  Register first_symbol_table_element = symbol_table;
  __ add(first_symbol_table_element, symbol_table,
         Operand(SymbolTable::kElementsStartOffset - kHeapObjectTag));

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
  Register candidate = scratch5;  // Scratch register contains candidate.
  for (int i = 0; i < kProbes; i++) {

    // Calculate entry in symbol table.
    if (i > 0) {
      __ add(candidate, hash, Operand(SymbolTable::GetProbeOffset(i)));
    } else {
      __ mov(candidate, hash);
    }

    __ land(candidate, candidate, mask);

    // Load the entry from the symble table.
    STATIC_ASSERT(SymbolTable::kEntrySize == 1);
    __ lsl(scratch, candidate, Operand(kPointerSizeLog2));
    __ add(scratch, first_symbol_table_element, scratch);
    __ ldr(candidate, MemOperand(scratch));

    // If entry is undefined no string with this hash can be found.
    Label is_string;
    __ CompareObjectType(candidate, scratch, scratch, ODDBALL_TYPE, eq);
    __ b(ne, &is_string, Label::kNear);

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
    __ cmp(scratch, Operand(Smi::FromInt(2)));
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
  Register result = candidate;
  __ bind(&found_in_symbol_table);
  __ Move(r0, result);
}


void StringHelper::GenerateHashInit(MacroAssembler* masm,
                                    Register hash,
                                    Register character,
                                    Register scratch) {
  // Added a scratch parameter for the SH4 implementation compared to ARM.
  // hash = character + (character << 10);
  __ lsl(hash, character, Operand(10));
  __ add(hash, character, hash);
  // hash ^= hash >> 6;
  __ asr(scratch, hash, Operand(6));
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
  __ lsl(scratch, hash, Operand(10));
  __ add(hash, hash, scratch);
  // hash ^= hash >> 6;
  __ asr(scratch, hash, Operand(6));
  __ eor(hash, hash, scratch);
}


void StringHelper::GenerateHashGetHash(MacroAssembler* masm,
                                       Register hash,
                                       Register scratch) {
  // Added a scratch parameter for the SH4 implementation compared to ARM.
  // hash += hash << 3;
  __ lsl(scratch, hash, Operand(3));
  __ add(hash, hash, scratch);
  // hash ^= hash >> 11;
  __ asr(scratch, hash, Operand(11));
  __ eor(hash, hash, scratch);
  // hash += hash << 15;
  __ lsl(scratch, hash, Operand(15));
  __ add(hash, hash, scratch);

  // if (hash == 0) hash = 27;
  // This code is in sync with StringHasher::GetHash() in objects-inl.h
  __ cmpeq(hash, Operand(0));
  __ mov(hash, Operand(27), t);
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
  __ asr(r2, to, Operand(1));
  __ asr(r3, from, Operand(1));
  __ cmpge(r3, Operand(0));
  __ bf(&runtime);  // From is negative.

  // Both to and from are smis.
  __ sub(r2, r2, r3);
  __ cmpge(r2, Operand(0));
  __ bf(&runtime);  // Fail if from > to.
  // Special handling of sub-strings of length 1 and 2. One character strings
  // are handled in the runtime system (looked up in the single character
  // cache). Two character strings are looked for in the symbol cache in
  // generated code.
  __ cmpge(r2, Operand(2));
  __ bf(&runtime);

  // r2: result string length
  // r3: from index (untagged smi)
  // r6 (a.k.a. to): to (smi)
  // r7 (a.k.a. from): from offset (smi)
  // Make sure first argument is a sequential (or flat) string.
  __ ldr(r0, MemOperand(sp, kStringOffset));
  STATIC_ASSERT(kSmiTag == 0);
  __ JumpIfSmi(r0, &runtime);
  Condition is_string = masm->IsObjectStringType(r0, r1);
  __ b(NegateCondition(is_string), &runtime);

  // Short-cut for the case of trivial substring.
  Label return_r0;
  // r0: original string
  // r2: result string length
  __ ldr(r4, FieldMemOperand(r0, String::kLengthOffset));
  __ asr(r4, r4, Operand(1));
  __ cmp(r2, r4);
  __ b(eq, &return_r0);

  Label create_slice;
  if (FLAG_string_slices) {
    __ cmpge(r2, Operand(SlicedString::kMinLength));
    __ bt(&create_slice);
  }

  // r0: original string
  // r1: instance type
  // r2: result string length
  // r3: from index (untagged smi)
  // r6 (a.k.a. to): to (smi)
  // r7 (a.k.a. from): from offset (smi)
  Label seq_string;
  __ land(r4, r1, Operand(kStringRepresentationMask));
  STATIC_ASSERT(kSeqStringTag < kConsStringTag);
  STATIC_ASSERT(kConsStringTag < kExternalStringTag);
  STATIC_ASSERT(kConsStringTag < kSlicedStringTag);
  __ cmpgt(r4, Operand(kConsStringTag));
  __ bt(&runtime);  // Slices and external strings go to runtime.
  __ cmpge(r4, Operand(kConsStringTag));
  __ bf_near(&seq_string);  // Sequential strings are handled directly.

  // Cons string. Try to recurse (once) on the first substring.
  // (This adds a little more generality than necessary to handle flattened
  // cons strings, but not much).
  __ ldr(r0, FieldMemOperand(r0, ConsString::kFirstOffset));
  __ ldr(r4, FieldMemOperand(r0, HeapObject::kMapOffset));
  __ ldrb(r1, FieldMemOperand(r4, Map::kInstanceTypeOffset));
  __ tst(r1, Operand(kStringRepresentationMask));
  STATIC_ASSERT(kSeqStringTag == 0);
  __ b(ne, &runtime);  // Cons, slices and external strings go to runtime.

  // Definitly a sequential string.
  __ bind(&seq_string);

  // r0: original string
  // r1: instance type
  // r2: result string length
  // r3: from index (untagged smi)
  // r6 (a.k.a. to): to (smi)
  // r7 (a.k.a. from): from offset (smi)
  __ ldr(r4, FieldMemOperand(r0, String::kLengthOffset));
  __ cmpge(r4, to);
  __ bf(&runtime);  // Fail if to > length.
  to = no_reg;

  // r0: original string or left hand side of the original cons string.
  // r1: instance type
  // r2: result string length
  // r3: from index (untagged smi)
  // r7 (a.k.a. from): from offset (smi)
  // Check for flat ASCII string.
  Label non_ascii_flat;
  __ tst(r1, Operand(kStringEncodingMask));
  STATIC_ASSERT(kTwoByteStringTag == 0);
  __ b(eq, &non_ascii_flat);

  Label result_longer_than_two;
  __ cmpgt(r2, Operand(2));
  __ bt(&result_longer_than_two);

  // Sub string of length 2 requested.
  // Get the two characters forming the sub string.
  __ add(r0, r0, r3);
  __ ldrb(r3, FieldMemOperand(r0, SeqAsciiString::kHeaderSize));
  __ ldrb(r4, FieldMemOperand(r0, SeqAsciiString::kHeaderSize + 1));

  // Try to lookup two character string in symbol table.
  Label make_two_character_string;
  StringHelper::GenerateTwoCharacterSymbolTableProbe(
      masm, r3, r4, r1, r5, r6, r7, r9, &make_two_character_string);
  Counters* counters = masm->isolate()->counters();
  __ jmp(&return_r0);

  // r2: result string length.
  // r3: two characters combined into halfword in little endian byte order.
  __ bind(&make_two_character_string);
  __ AllocateAsciiString(r0, r2, r4, r5, r9, &runtime);
  __ strh(r3, FieldMemOperand(r0, SeqAsciiString::kHeaderSize));
  __ jmp(&return_r0);

  __ bind(&result_longer_than_two);

  // Locate 'from' character of string.
  __ add(r5, r0, Operand(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  __ asr(ip, from, Operand(1));
  __ add(r5, r5, ip);

  // Allocate the result.
  __ AllocateAsciiString(r0, r2, r3, r4, r1, &runtime);

  // r0: result string
  // r2: result string length
  // r5: first character of substring to copy
  // r7 (a.k.a. from): from offset (smi)
  // Locate first character of result.
  __ add(r1, r0, Operand(SeqAsciiString::kHeaderSize - kHeapObjectTag));

  // r0: result string
  // r1: first character of result string
  // r2: result string length
  // r5: first character of substring to copy
  STATIC_ASSERT((SeqAsciiString::kHeaderSize & kObjectAlignmentMask) == 0);
  StringHelper::GenerateCopyCharactersLong(masm, r1, r5, r2, r3, r4, r6, r7, r9,
                                           COPY_ASCII | DEST_ALWAYS_ALIGNED);
  __ jmp(&return_r0);

  __ bind(&non_ascii_flat);
  // r0: original string
  // r2: result string length
  // r7 (a.k.a. from): from offset (smi)
  // Check for flat two byte string.

  // Locate 'from' character of string.
  __ add(r5, r0, Operand(SeqTwoByteString::kHeaderSize - kHeapObjectTag));
  // As "from" is a smi it is 2 times the value which matches the size of a two
  // byte character.
  STATIC_ASSERT(kSmiTagSize == 1 && kSmiTag == 0);
  __ add(r5, r5, from);

  // Allocate the result.
  __ AllocateTwoByteString(r0, r2, r1, r3, r4, &runtime);

  // r0: result string
  // r2: result string length
  // r5: first character of substring to copy
  // Locate first character of result.
  __ add(r1, r0, Operand(SeqTwoByteString::kHeaderSize - kHeapObjectTag));
  from = no_reg;

  // r0: result string.
  // r1: first character of result.
  // r2: result length.
  // r5: first character of substring to copy.
  STATIC_ASSERT((SeqTwoByteString::kHeaderSize & kObjectAlignmentMask) == 0);
  StringHelper::GenerateCopyCharactersLong(
      masm, r1, r5, r2, r3, r4, r6, r7, r9, DEST_ALWAYS_ALIGNED);
  __ jmp(&return_r0);

  if (FLAG_string_slices) {
    __ bind(&create_slice);
    // r0: original string
    // r1: instance type
    // r2: length
    // r3: from index (untagged smi)
    // r6 (a.k.a. to): to (smi)
    // r7 (a.k.a. from): from offset (smi)
    Label allocate_slice, sliced_string, seq_string;
    STATIC_ASSERT(kSeqStringTag == 0);
    __ tst(r1, Operand(kStringRepresentationMask));
    __ b(eq, &seq_string, Label::kNear);
    STATIC_ASSERT(kIsIndirectStringMask == (kSlicedStringTag & kConsStringTag));
    STATIC_ASSERT(kIsIndirectStringMask != 0);
    __ tst(r1, Operand(kIsIndirectStringMask));
    // External string.  Jump to runtime.
    __ b(eq, &runtime);

    __ tst(r1, Operand(kSlicedNotConsMask));
    __ b(ne, &sliced_string, Label::kNear);
    // Cons string.  Check whether it is flat, then fetch first part.
    __ ldr(r5, FieldMemOperand(r0, ConsString::kSecondOffset));
    __ LoadRoot(r9, Heap::kEmptyStringRootIndex);
    __ cmp(r5, r9);
    __ b(ne, &runtime);
    __ ldr(r5, FieldMemOperand(r0, ConsString::kFirstOffset));
    __ jmp_near(&allocate_slice);

    __ bind(&sliced_string);
    // Sliced string.  Fetch parent and correct start index by offset.
    __ ldr(r5, FieldMemOperand(r0, SlicedString::kOffsetOffset));
    __ add(r7, r7, r5);
    __ ldr(r5, FieldMemOperand(r0, SlicedString::kParentOffset));
    __ jmp_near(&allocate_slice);

    __ bind(&seq_string);
    // Sequential string.  Just move string to the right register.
    __ mov(r5, r0);

    __ bind(&allocate_slice);
    // r1: instance type of original string
    // r2: length
    // r5: underlying subject string
    // r7 (a.k.a. from): from offset (smi)
    // Allocate new sliced string.  At this point we do not reload the instance
    // type including the string encoding because we simply rely on the info
    // provided by the original string.  It does not matter if the original
    // string's encoding is wrong because we always have to recheck encoding of
    // the newly created string's parent anyways due to externalized strings.
    Label two_byte_slice, set_slice_header;
    STATIC_ASSERT((kStringEncodingMask & kAsciiStringTag) != 0);
    STATIC_ASSERT((kStringEncodingMask & kTwoByteStringTag) == 0);
    __ tst(r1, Operand(kStringEncodingMask));
    __ b(eq, &two_byte_slice);
    __ AllocateAsciiSlicedString(r0, r2, r3, r4, &runtime);
    __ jmp(&set_slice_header);
    __ bind(&two_byte_slice);
    __ AllocateTwoByteSlicedString(r0, r2, r3, r4, &runtime);
    __ bind(&set_slice_header);
    __ str(r7, FieldMemOperand(r0, SlicedString::kOffsetOffset));
    __ str(r5, FieldMemOperand(r0, SlicedString::kParentOffset));
  }

  __ bind(&return_r0);
  __ IncrementCounter(counters->sub_string_native(), 1, r3, r4);
  __ add(sp, sp, Operand(3 * kPointerSize));
  __ Ret();

  // Just jump to runtime to create the sub string.
  __ bind(&runtime);
  __ TailCallRuntime(Runtime::kSubString, 3, 1);
}


void StringCompareStub::GenerateFlatAsciiStringEquals(MacroAssembler* masm,
                                                      Register left,
                                                      Register right,
                                                      Register scratch1,
                                                      Register scratch2,
                                                      Register scratch3) {
  Register length = scratch1;

  // Compare lengths.
  Label strings_not_equal, check_zero_length;
  __ ldr(length, FieldMemOperand(left, String::kLengthOffset));
  __ ldr(scratch2, FieldMemOperand(right, String::kLengthOffset));
  __ cmp(length, scratch2);
  __ b(eq, &check_zero_length, Label::kNear);
  __ bind(&strings_not_equal);
  __ mov(r0, Operand(Smi::FromInt(NOT_EQUAL)));
  __ Ret();

  // Check if the length is zero.
  Label compare_chars;
  __ bind(&check_zero_length);
  STATIC_ASSERT(kSmiTag == 0);
  __ tst(length, length);
  __ b(ne, &compare_chars, Label::kNear);
  __ mov(r0, Operand(Smi::FromInt(EQUAL)));
  __ Ret();

  // Compare characters.
  __ bind(&compare_chars);
  GenerateAsciiCharsCompareLoop(masm,
                                left, right, length, scratch2, scratch3,
                                &strings_not_equal);

  // Characters are equal.
  __ mov(r0, Operand(Smi::FromInt(EQUAL)));
  __ Ret();
}


void StringCompareStub::GenerateCompareFlatAsciiStrings(MacroAssembler* masm,
                                                        Register left,
                                                        Register right,
                                                        Register scratch1,
                                                        Register scratch2,
                                                        Register scratch3,
                                                        Register scratch4) {
  ASSERT(!scratch2.is(r0) && !scratch4.is(r0));
  Label result_not_equal;
  Label compare_lengths, skip;
  // Find minimum length and length difference.
  __ ldr(scratch1, FieldMemOperand(left, String::kLengthOffset));
  __ ldr(scratch2, FieldMemOperand(right, String::kLengthOffset));
  __ cmpgt(scratch1, scratch2); // for cond mov below
  __ sub(scratch3, scratch1, scratch2);
  Register length_delta = scratch3;
  __ mov(scratch1, scratch2, t);
  Register min_length = scratch1;
  STATIC_ASSERT(kSmiTag == 0);
  __ tst(min_length, min_length);
  __ b(eq, &compare_lengths);

  // Compare loop.
  GenerateAsciiCharsCompareLoop(masm,
                                left, right, min_length, scratch2, scratch4,
                                &result_not_equal);

  // Compare lengths - strings up to min-length are equal.
  __ bind(&compare_lengths);
  ASSERT(Smi::FromInt(EQUAL) == static_cast<Smi*>(0));
  // Use length_delta as result if it's zero.
  __ mov(r0, length_delta);
  __ cmpgt(r0, Operand(0));
  __ mov(r0, Operand(Smi::FromInt(GREATER)), t);
  __ cmpge(r0, Operand(0));
  __ mov(r0, Operand(Smi::FromInt(LESS)), f);
  __ Ret();

  __ bind(&result_not_equal);
  // Conditionally update the result based either on length_delta or
  // the last comparion performed in the loop above.
  __ cmpgt(scratch2, scratch4);
  __ mov(r0, Operand(Smi::FromInt(GREATER)), t);
  __ cmpge(scratch2, scratch4);
  __ mov(r0, Operand(Smi::FromInt(LESS)), f);
  __ Ret();
}


void StringCompareStub::GenerateAsciiCharsCompareLoop(
    MacroAssembler* masm,
    Register left,
    Register right,
    Register length,
    Register scratch1,
    Register scratch2,
    Label* chars_not_equal) {
  // Change index to run from -length to -1 by adding length to string
  // start. This means that loop ends when index reaches zero, which
  // doesn't need an additional compare.
  __ SmiUntag(length);
  __ add(scratch1, length,
         Operand(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  __ add(left, left, scratch1);
  __ add(right, right, scratch1);
  __ rsb(length, length, Operand(0));
  Register index = length;  // index = -length;

  // Compare loop.
  Label loop;
  __ bind(&loop);
  __ ldrb(scratch1, MemOperand(left, index));
  __ ldrb(scratch2, MemOperand(right, index));
  __ cmp(scratch1, scratch2);
  __ b(ne, chars_not_equal);
  __ add(index, index, Operand(1));
  __ tst(index, index);
  __ b(ne, &loop);
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
  __ b(ne, &not_same, Label::kNear);
  STATIC_ASSERT(EQUAL == 0);
  STATIC_ASSERT(kSmiTag == 0);
  __ mov(r0, Operand(Smi::FromInt(EQUAL)));
  __ IncrementCounter(counters->string_compare_native(), 1, r1, r2);
  __ add(sp, sp, Operand(2 * kPointerSize));
  __ Ret();

  __ bind(&not_same);

  // Check that both objects are sequential ASCII strings.
  __ JumpIfNotBothSequentialAsciiStrings(r1, r0, r2, r3, &runtime);

  // Compare flat ASCII strings natively. Remove arguments from stack first.
  __ IncrementCounter(counters->string_compare_native(), 1, r2, r3);
  __ add(sp, sp, Operand(2 * kPointerSize));
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
    __ tst(r4, Operand(kIsNotStringMask));
    __ b(ne, &string_add_runtime);
    __ tst(r5, Operand(kIsNotStringMask));
    __ b(ne, &string_add_runtime);
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
    __ cmp(r2, Operand(Smi::FromInt(0)));  // Test if first string is empty.
    __ mov(r0, r1, eq);  // If first is empty, return second.
    __ bt_near(&string_return);
    STATIC_ASSERT(kSmiTag == 0);
     // Else test if second string is empty.
    __ cmp(r3, Operand(Smi::FromInt(0)));
    __ b(ne, &strings_not_empty, Label::kNear);  // If either string was empty, return r0.

    __ bind(&string_return);
    __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
    __ add(sp, sp, Operand(2 * kPointerSize));
    __ Ret();

    __ bind(&strings_not_empty);
  }

  __ asr(r2, r2, Operand(kSmiTagSize));
  __ asr(r3, r3, Operand(kSmiTagSize));
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
  __ cmp(r6, Operand(2));
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
  __ add(sp, sp, Operand(2 * kPointerSize));
  __ Ret();

  __ bind(&make_two_character_string);
  // Resulting string has length 2 and first chars of two strings
  // are combined into single halfword in r2 register.
  // So we can fill resulting string without two loops by a single
  // halfword store instruction (which assumes that processor is
  // in a little endian mode)
  __ mov(r6, Operand(2));
  __ AllocateAsciiString(r0, r6, r4, r5, r9, &string_add_runtime);
  __ strh(r2, FieldMemOperand(r0, SeqAsciiString::kHeaderSize));
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Operand(2 * kPointerSize));
  __ Ret();

  __ bind(&longer_than_two);
  // Check if resulting string will be flat.
  __ cmpge(r6, Operand(String::kMinNonFlatLength));
  __ bf(&string_add_flat_result);
  // Handle exceptionally long strings in the runtime system.
  STATIC_ASSERT((String::kMaxLength & 0x80000000) == 0);
  ASSERT(IsPowerOf2(String::kMaxLength + 1));
  // kMaxLength + 1 is representable as shifted literal, kMaxLength is not.
  __ cmphs(r6, Operand(String::kMaxLength + 1));
  __ bt(&string_add_runtime);

  // If result is not supposed to be flat, allocate a cons string object.
  // If both strings are ASCII the result is an ASCII cons string.
  if (flags_ != NO_STRING_ADD_FLAGS) {
    __ ldr(r4, FieldMemOperand(r0, HeapObject::kMapOffset));
    __ ldr(r5, FieldMemOperand(r1, HeapObject::kMapOffset));
    __ ldrb(r4, FieldMemOperand(r4, Map::kInstanceTypeOffset));
    __ ldrb(r5, FieldMemOperand(r5, Map::kInstanceTypeOffset));
  }
  Label ascii_data, allocated;
  Label non_ascii;
  STATIC_ASSERT(kTwoByteStringTag == 0);
  __ tst(r4, Operand(kStringEncodingMask));
  __ bt_near(&non_ascii);
  __ tst(r5, Operand(kStringEncodingMask));
  __ b(eq, &non_ascii, Label::kNear);

  // Allocate an ASCII cons string.
  __ bind(&ascii_data);
  __ AllocateAsciiConsString(r7, r6, r4, r5, &string_add_runtime);
  __ bind(&allocated);
  // Fill the fields of the cons string.
  __ str(r0, FieldMemOperand(r7, ConsString::kFirstOffset));
  __ str(r1, FieldMemOperand(r7, ConsString::kSecondOffset));
  __ mov(r0, r7);
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Operand(2 * kPointerSize));
  __ Ret();

  __ bind(&non_ascii);
  // At least one of the strings is two-byte. Check whether it happens
  // to contain only ASCII characters.
  // r4: first instance type.
  // r5: second instance type.
  __ tst(r4, Operand(kAsciiDataHintMask));
  __ b(ne, &ascii_data);
  __ tst(r5, Operand(kAsciiDataHintMask));
  __ b(ne, &ascii_data);
  __ eor(r4, r4, r5);
  STATIC_ASSERT(kAsciiStringTag != 0 && kAsciiDataHintTag != 0);
  __ land(r4, r4, Operand(kAsciiStringTag | kAsciiDataHintTag));
  __ cmp(r4, Operand(kAsciiStringTag | kAsciiDataHintTag));
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
  __ tst(r4, Operand(kStringRepresentationMask));
  __ b(ne, &string_add_runtime);
  __ tst(r5, Operand(kStringRepresentationMask));
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
  __ tst(r7, Operand(kStringEncodingMask));
  __ b(ne, &string_add_runtime);
  // And see if it's ASCII or two-byte.
  __ tst(r4, Operand(kStringEncodingMask));
  __ b(eq, &non_ascii_string_add_flat_result);

  // Both strings are sequential ASCII strings. We also know that they are
  // short (since the sum of the lengths is less than kMinNonFlatLength).
  // r6: length of resulting flat string
  __ AllocateAsciiString(r7, r6, r4, r5, r9, &string_add_runtime);
  // Locate first character of result.
  __ add(r6, r7, Operand(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  // Locate first character of first argument.
  __ add(r0, r0, Operand(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  // r0: first character of first string.
  // r1: second string.
  // r2: length of first string.
  // r3: length of second string.
  // r6: first character of result.
  // r7: result string.
  StringHelper::GenerateCopyCharacters(masm, r6, r0, r2, r4, true);

  // Load second argument and locate first character.
  __ add(r1, r1, Operand(SeqAsciiString::kHeaderSize - kHeapObjectTag));
  // r1: first character of second string.
  // r3: length of second string.
  // r6: next character of result.
  // r7: result string.
  StringHelper::GenerateCopyCharacters(masm, r6, r1, r3, r4, true);
  __ mov(r0, r7);
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Operand(2 * kPointerSize));
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
  __ add(r6, r7, Operand(SeqTwoByteString::kHeaderSize - kHeapObjectTag));
  // Locate first character of first argument.
  __ add(r0, r0, Operand(SeqTwoByteString::kHeaderSize - kHeapObjectTag));

  // r0: first character of first string.
  // r1: second string.
  // r2: length of first string.
  // r3: length of second string.
  // r6: first character of result.
  // r7: result string.
  StringHelper::GenerateCopyCharacters(masm, r6, r0, r2, r4, false);

  // Locate first character of second argument.
  __ add(r1, r1, Operand(SeqTwoByteString::kHeaderSize - kHeapObjectTag));

  // r1: first character of second string.
  // r3: length of second string.
  // r6: next character of result (after copy of first string).
  // r7: result string.
  StringHelper::GenerateCopyCharacters(masm, r6, r1, r3, r4, false);

  __ mov(r0, r7);
  __ IncrementCounter(counters->string_add_native(), 1, r2, r3);
  __ add(sp, sp, Operand(2 * kPointerSize));
  __ Ret();

  // Just jump to runtime to add the two strings.
  __ bind(&string_add_runtime);
  __ TailCallRuntime(Runtime::kStringAdd, 2, 1);

  if (call_builtin.is_linked()) {
    __ bind(&call_builtin);
    __ InvokeBuiltin(builtin_id, JUMP_FUNCTION);
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
  __ b(ne, slow);
  __ ldrb(scratch2, FieldMemOperand(scratch1, Map::kBitField2Offset));
  __ land(scratch2,
          scratch2, Operand(1 << Map::kStringWrapperSafeForDefaultValueOf));
  __ cmp(scratch2,
         Operand(1 << Map::kStringWrapperSafeForDefaultValueOf));
  __ b(ne, slow);
  __ ldr(arg, FieldMemOperand(arg, JSValue::kValueOffset));
  __ str(arg, MemOperand(sp, stack_offset));

  __ bind(&done);
}


void ICCompareStub::GenerateSmis(MacroAssembler* masm) {
  ASSERT(state_ == CompareIC::SMIS);
  Label miss;
  __ orr(r2, r1, r0);
  __ JumpIfNotSmi(r2, &miss);

  if (GetCondition() == eq) {
    // For equality we do not care about the sign of the result.
    __ sub(r0, r0, r1);
    __ tst(r0, r0);     // TODO(stm): why setting CC? is it used?
  } else {
    // Untag before subtracting to avoid handling overflow.
    __ SmiUntag(r1);
    __ SmiUntag(r0);
    __ sub(r0, r1, r0);
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
  __ JumpIfSmi(r2, &generic_stub, Label::kNear);

  __ CompareObjectType(r0, r2, r2, HEAP_NUMBER_TYPE, eq);
  __ b(ne, &miss, Label::kNear);
  __ CompareObjectType(r1, r2, r2, HEAP_NUMBER_TYPE, eq);
  __ b(ne, &miss, Label::kNear);

  // Inlining the double comparison and falling back to the general compare
  // stub if NaN is involved or VFP3 is unsupported.
  // TODO(stm): FPU
  // if (CpuFeatures::IsSupported(VFP3)) {
  // }

  CompareStub stub(GetCondition(), strict(), NO_COMPARE_FLAGS, r1, r0);
  __ bind(&generic_stub);
  __ Jump(stub.GetCode(), RelocInfo::CODE_TARGET);

  __ bind(&miss);
  GenerateMiss(masm);
}


void ICCompareStub::GenerateSymbols(MacroAssembler* masm) {
  ASSERT(state_ == CompareIC::SYMBOLS);
  Label miss;

  // Registers containing left and right operands respectively.
  Register left = r1;
  Register right = r0;
  Register tmp1 = r2;
  Register tmp2 = r3;

  // Check that both operands are heap objects.
  __ JumpIfEitherSmi(left, right, &miss, Label::kNear);

  // Check that both operands are symbols.
  __ ldr(tmp1, FieldMemOperand(left, HeapObject::kMapOffset));
  __ ldr(tmp2, FieldMemOperand(right, HeapObject::kMapOffset));
  __ ldrb(tmp1, FieldMemOperand(tmp1, Map::kInstanceTypeOffset));
  __ ldrb(tmp2, FieldMemOperand(tmp2, Map::kInstanceTypeOffset));
  STATIC_ASSERT(kSymbolTag != 0);
  __ land(tmp1, tmp1, tmp2);
  __ tst(tmp1, Operand(kIsSymbolMask));
  __ b(eq, &miss, Label::kNear);

  // Symbols are compared by identity.
  __ cmp(left, right);
  // Make sure r0 is non-zero. At this point input operands are
  // guaranteed to be non-zero.
  ASSERT(right.is(r0));
  STATIC_ASSERT(EQUAL == 0);
  STATIC_ASSERT(kSmiTag == 0);
  __ mov(r0, Operand(Smi::FromInt(EQUAL)), eq);
  __ Ret();

  __ bind(&miss);
  GenerateMiss(masm);
}


void ICCompareStub::GenerateStrings(MacroAssembler* masm) {
  ASSERT(state_ == CompareIC::STRINGS);
  Label miss;

  // Registers containing left and right operands respectively.
  Register left = r1;
  Register right = r0;
  Register tmp1 = r2;
  Register tmp2 = r3;
  Register tmp3 = r4;
  Register tmp4 = r5;

  // Check that both operands are heap objects.
  __ JumpIfEitherSmi(left, right, &miss);

  // Check that both operands are strings. This leaves the instance
  // types loaded in tmp1 and tmp2.
  __ ldr(tmp1, FieldMemOperand(left, HeapObject::kMapOffset));
  __ ldr(tmp2, FieldMemOperand(right, HeapObject::kMapOffset));
  __ ldrb(tmp1, FieldMemOperand(tmp1, Map::kInstanceTypeOffset));
  __ ldrb(tmp2, FieldMemOperand(tmp2, Map::kInstanceTypeOffset));
  STATIC_ASSERT(kNotStringTag != 0);
  __ orr(tmp3, tmp1, tmp2);
  __ tst(tmp3, Operand(kIsNotStringMask));
  __ b(ne, &miss);

  // Fast check for identical strings.
  __ cmp(left, right);
  STATIC_ASSERT(EQUAL == 0);
  STATIC_ASSERT(kSmiTag == 0);
  __ mov(r0, Operand(Smi::FromInt(EQUAL)), eq);
  __ Ret(eq);

  // Handle not identical strings.

  // Check that both strings are symbols. If they are, we're done
  // because we already know they are not identical.
  ASSERT(GetCondition() == eq);
  STATIC_ASSERT(kSymbolTag != 0);
  __ land(tmp3, tmp1, tmp2);
  __ tst(tmp3, Operand(kIsSymbolMask));
  // Make sure r0 is non-zero. At this point input operands are
  // guaranteed to be non-zero.
  ASSERT(right.is(r0));
  __ Ret(ne);

  // Check that both strings are sequential ASCII.
  Label runtime;
  __ JumpIfBothInstanceTypesAreNotSequentialAscii(tmp1, tmp2, tmp3, tmp4,
                                                  &runtime);

  // Compare flat ASCII strings. Returns when done.
  StringCompareStub::GenerateFlatAsciiStringEquals(
      masm, left, right, tmp1, tmp2, tmp3);

  // Handle more complex cases in runtime.
  __ bind(&runtime);
  __ Push(left, right);
  __ TailCallRuntime(Runtime::kStringEquals, 2, 1);

  __ bind(&miss);
  GenerateMiss(masm);
}


void ICCompareStub::GenerateObjects(MacroAssembler* masm) {
  ASSERT(state_ == CompareIC::OBJECTS);
  Label miss;
  __ land(r2, r1, r0);
  __ JumpIfSmi(r2, &miss, Label::kNear);

  __ CompareObjectType(r0, r2, r2, JS_OBJECT_TYPE, eq);
  __ b(ne, &miss, Label::kNear);
  __ CompareObjectType(r1, r2, r2, JS_OBJECT_TYPE, eq);
  __ b(ne, &miss, Label::kNear);

  ASSERT(GetCondition() == eq);
  __ sub(r0, r0, r1);
  __ Ret();

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
  __ mov(ip, Operand(Smi::FromInt(op_)));
  __ push(ip);
  __ CallExternalReference(miss, 3);
  __ LeaveInternalFrame();
  // Compute the entry point of the rewritten stub.
  __ add(r2, r0, Operand(Code::kHeaderSize - kHeapObjectTag));
  // Restore registers.
  __ pop(lr);
  __ pop(r0);
  __ pop(r1);
  __ jmp(r2);
}


void DirectCEntryStub::Generate(MacroAssembler* masm) {
  __ ldr(scratch_, MemOperand(sp, 0), scratch_);
  __ jmp(scratch_);
}


void DirectCEntryStub::GenerateCall(MacroAssembler* masm,
                                    ExternalReference function,
				    Register scratch1,
				    Register scratch2) {
  __ mov(scratch1, Operand(function));
  GenerateCall(masm, scratch1, scratch2);
}


void DirectCEntryStub::GenerateCall(MacroAssembler* masm,
				    Register target,
				    Register scratch1) {

  ASSERT(!target.is(scratch_));
  ASSERT(!target.is(scratch1));
  ASSERT(!scratch1.is(scratch_));
  // Get pr (pointing to DirectCEntryStub::Generate) into scratch1
  // pr can't be used directly as it is clobbered by addpc later
  __ mov(scratch1, Operand(reinterpret_cast<intptr_t>(GetCode().location()),
                   RelocInfo::CODE_TARGET));
  int return_address_offset = 4 * Assembler::kInstrSize;
  Label start;
  // Push return address (accessible to GC through exit frame pc).
  __ addpc(scratch_, return_address_offset, pr);
  __ bind(&start);
  // restore the right pr (pointing to DirectCEntryStub::Generate)
  __ mov(pr,scratch1);
  __ str(scratch_, MemOperand(sp, 0), no_reg);
  __ jmp(target);   // Call the api function.
  ASSERT_EQ(return_address_offset,
            masm->SizeOfCodeGeneratedSince(&start));
}


MaybeObject* StringDictionaryLookupStub::GenerateNegativeLookup(
    MacroAssembler* masm,
    Label* miss,
    Label* done,
    Register receiver,
    Register properties,
    String* name,
    Register scratch0) {
  ASSERT(!scratch0.is(ip));
  // If names of slots in range from 1 to kProbes - 1 for the hash value are
  // not equal to the name and kProbes-th slot is not used (its name is the
  // undefined value), it guarantees the hash table doesn't contain the
  // property. It's true even if some slots represent deleted properties
  // (their names are the null value).
  for (int i = 0; i < kInlinedProbes; i++) {
    // scratch0 points to properties hash.
    // Compute the masked index: (hash + i + i * i) & mask.
    Register index = scratch0;
    // Capacity is smi 2^n.
    __ ldr(index, FieldMemOperand(properties, kCapacityOffset));
    __ sub(index, index, Operand(1));
    __ land(index, index, Operand(
        Smi::FromInt(name->Hash() + StringDictionary::GetProbeOffset(i))));

    // Scale the index by multiplying by the entry size.
    ASSERT(StringDictionary::kEntrySize == 3);
    __ lsl(ip, index, Operand(1));
    __ add(index, index, ip);  // index *= 3.

    Register entity_name = scratch0;
    // Having undefined at this place means the name is not contained.
    ASSERT_EQ(kSmiTagSize, 1);
    Register tmp = properties;
    __ lsl(entity_name, index, Operand(1)); /* use entity_name as scratch (defined just after) */
    __ add(tmp, properties, entity_name);
    __ ldr(entity_name, FieldMemOperand(tmp, kElementsStartOffset));

    ASSERT(!tmp.is(entity_name));
    __ LoadRoot(tmp, Heap::kUndefinedValueRootIndex);
    __ cmp(entity_name, tmp);
    __ b(eq, done);

    if (i != kInlinedProbes - 1) {
      // Stop if found the property.
      __ cmp(entity_name, Operand(Handle<String>(name)));
      __ b(eq, miss);

      // Check if the entry name is not a symbol.
      __ ldr(entity_name, FieldMemOperand(entity_name, HeapObject::kMapOffset));
      __ ldrb(entity_name,
              FieldMemOperand(entity_name, Map::kInstanceTypeOffset));
      __ tst(entity_name, Operand(kIsSymbolMask));
      __ b(eq, miss);

      // Restore the properties.
      __ ldr(properties,
             FieldMemOperand(receiver, JSObject::kPropertiesOffset));
    }
  }

  const int spill_mask =
      (r6.bit() | r5.bit() | r4.bit() | r3.bit() |
       r2.bit() | r1.bit() | r0.bit());

  __ push(pr);
  __ pushm(spill_mask);
  __ ldr(r0, FieldMemOperand(receiver, JSObject::kPropertiesOffset));
  __ mov(r1, Operand(Handle<String>(name)));
  StringDictionaryLookupStub stub(NEGATIVE_LOOKUP);
  MaybeObject* result = masm->TryCallStub(&stub);
  if (result->IsFailure()) return result;
  __ tst(r0, r0);
  __ popm(spill_mask);
  __ pop(pr);

  __ b(eq, done);
  __ b(ne, miss);
  return result;
}


// Probe the string dictionary in the |elements| register. Jump to the
// |done| label if a property with the given name is found. Jump to
// the |miss| label otherwise.
// If lookup was successful |scratch2| will be equal to elements + 4 * index.
void StringDictionaryLookupStub::GeneratePositiveLookup(MacroAssembler* masm,
                                                        Label* miss,
                                                        Label* done,
                                                        Register elements,
                                                        Register name,
                                                        Register scratch1,
                                                        Register scratch2) {
  // Assert that name contains a string.
  if (FLAG_debug_code) __ AbortIfNotString(name);

  // Compute the capacity mask.
  __ ldr(scratch1, FieldMemOperand(elements, kCapacityOffset));
  __ asr(scratch1, scratch1, Operand(kSmiTagSize));  // convert smi to int
  __ sub(scratch1, scratch1, Operand(1));

  // Generate an unrolled loop that performs a few probes before
  // giving up. Measurements done on Gmail indicate that 2 probes
  // cover ~93% of loads from dictionaries.
  for (int i = 0; i < kInlinedProbes; i++) {
    // Compute the masked index: (hash + i + i * i) & mask.
    __ ldr(scratch2, FieldMemOperand(name, String::kHashFieldOffset));
    if (i > 0) {
      // Add the probe offset (i + i * i) left shifted to avoid right shifting
      // the hash in a separate instruction. The value hash + i + i * i is right
      // shifted in the following and instruction.
      ASSERT(StringDictionary::GetProbeOffset(i) <
             1 << (32 - String::kHashFieldOffset));
      __ add(scratch2, scratch2, Operand(
          StringDictionary::GetProbeOffset(i) << String::kHashShift));
    }
    __ lsr(scratch2, scratch2, Operand(String::kHashShift));
    __ land(scratch2, scratch1, scratch2);

    // Scale the index by multiplying by the element size.
    ASSERT(StringDictionary::kEntrySize == 3);
    // scratch2 = scratch2 * 3.
    __ lsl(ip, scratch2, Operand(1));
    __ add(scratch2, scratch2, ip);

    // Check if the key is identical to the name.
    __ lsl(scratch2, scratch2, Operand(2));
    __ add(scratch2, elements, scratch2);
    __ ldr(ip, FieldMemOperand(scratch2, kElementsStartOffset));
    __ cmp(name, ip);
    __ b(eq, done);
  }

  const int spill_mask =
      ( r6.bit() | r5.bit() | r4.bit() |
       r3.bit() | r2.bit() | r1.bit() | r0.bit()) &
      ~(scratch1.bit() | scratch2.bit());

  __ push(pr);
  __ pushm(spill_mask);
  __ Move(r0, elements);
  __ Move(r1, name);
  StringDictionaryLookupStub stub(POSITIVE_LOOKUP);
  __ CallStub(&stub);
  __ tst(r0, r0);
  __ mov(scratch2, r2);
  __ popm(spill_mask);
  __ pop(pr);

  __ b(ne, done);
  __ b(eq, miss);
}


void StringDictionaryLookupStub::Generate(MacroAssembler* masm) {
  // Registers:
  //  result: StringDictionary to probe
  //  r1: key
  //  : StringDictionary to probe.
  //  index_: will hold an index of entry if lookup is successful.
  //          might alias with result_.
  // Returns:
  //  result_ is zero if lookup failed, non zero otherwise.

  Register result = r0;
  Register dictionary = r0;
  Register key = r1;
  Register index = r2;
  Register mask = r3;
  Register hash = r4;
  Register undefined = r5;
  Register entry_key = r6;

  Label in_dictionary, maybe_in_dictionary, not_in_dictionary;

  __ ldr(mask, FieldMemOperand(dictionary, kCapacityOffset));
  __ asr(mask, mask, Operand(kSmiTagSize));
  __ sub(mask, mask, Operand(1));

  __ ldr(hash, FieldMemOperand(key, String::kHashFieldOffset));

  __ LoadRoot(undefined, Heap::kUndefinedValueRootIndex);

  for (int i = kInlinedProbes; i < kTotalProbes; i++) {
    // Compute the masked index: (hash + i + i * i) & mask.
    // Capacity is smi 2^n.
    if (i > 0) {
      // Add the probe offset (i + i * i) left shifted to avoid right shifting
      // the hash in a separate instruction. The value hash + i + i * i is right
      // shifted in the following and instruction.
      ASSERT(StringDictionary::GetProbeOffset(i) <
             1 << (32 - String::kHashFieldOffset));
      __ add(index, hash, Operand(
          StringDictionary::GetProbeOffset(i) << String::kHashShift));
    } else {
      __ mov(index, hash);
    }
    __ lsr(index, index, Operand(String::kHashShift));
    __ land(index, mask, index);

    // Scale the index by multiplying by the entry size.
    ASSERT(StringDictionary::kEntrySize == 3);
    __ lsl(ip, index, Operand(1));
    __ add(index, index, ip);  // index *= 3.

    ASSERT_EQ(kSmiTagSize, 1);
    __ lsl(index, index, Operand(2));
    __ add(index, dictionary, index);
    __ ldr(entry_key, FieldMemOperand(index, kElementsStartOffset));

    // Having undefined at this place means the name is not contained.
    __ cmp(entry_key, undefined);
    __ b(eq, &not_in_dictionary);

    // Stop if found the property.
    __ cmp(entry_key, key);
    __ b(eq, &in_dictionary);

    if (i != kTotalProbes - 1 && mode_ == NEGATIVE_LOOKUP) {
      // Check if the entry name is not a symbol.
      __ ldr(entry_key, FieldMemOperand(entry_key, HeapObject::kMapOffset));
      __ ldrb(entry_key,
              FieldMemOperand(entry_key, Map::kInstanceTypeOffset));
      __ tst(entry_key, Operand(kIsSymbolMask));
      __ b(eq, &maybe_in_dictionary);
    }
  }

  __ bind(&maybe_in_dictionary);
  // If we are doing negative lookup then probing failure should be
  // treated as a lookup success. For positive lookup probing failure
  // should be treated as lookup failure.
  if (mode_ == POSITIVE_LOOKUP) {
    __ mov(result, Operand(0));
    __ Ret();
  }

  __ bind(&in_dictionary);
  __ mov(result, Operand(1));
  __ Ret();

  __ bind(&not_in_dictionary);
  __ mov(result, Operand(0));
  __ Ret();
}


#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
