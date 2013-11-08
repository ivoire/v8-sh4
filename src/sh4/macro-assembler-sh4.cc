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
#include "codegen.h"
#include "debug.h"
#include "runtime.h"

namespace v8 {
namespace internal {

#ifdef DEBUG
#define RECORD_LINE() RecordFunctionLine(__FUNCTION__, __LINE__)
#else
#define RECORD_LINE() ((void)0)
#endif


void MacroAssembler::TryGetFunctionPrototype(Register function,
                                             Register result,
                                             Register scratch,
                                             Label* miss) {
  ASSERT(!function.is(sh4_ip) && !result.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!function.is(sh4_rtmp) && !result.is(sh4_rtmp) && !scratch.is(sh4_rtmp));

  RECORD_LINE();
  // Check that the receiver isn't a smi.
  JumpIfSmi(function, miss);

  // Check that the function really is a function.  Load map into result reg.
  CompareObjectType(function, result, scratch, JS_FUNCTION_TYPE, eq);
  bf(miss);

  RECORD_LINE();
  // Make sure that the function has an instance prototype.
  Label non_instance;
  ldrb(scratch, FieldMemOperand(result, Map::kBitFieldOffset));
  tst(scratch, Operand(1 << Map::kHasNonInstancePrototype));
  bf_near(&non_instance);

  RECORD_LINE();
  // Get the prototype or initial map from the function.
  mov(result,
      FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));

  // If the prototype or initial map is the hole, don't return it and
  // simply miss the cache instead. This will allow us to allocate a
  // prototype object on-demand in the runtime system.
  LoadRoot(sh4_ip, Heap::kTheHoleValueRootIndex);
  cmpeq(result, sh4_ip);
  bt(miss);

  RECORD_LINE();
  // If the function does not have an initial map, we're done.
  Label done;
  CompareObjectType(result, scratch, scratch, MAP_TYPE, eq);
  bf_near(&done);

  RECORD_LINE();
  // Get the prototype from the initial map.
  mov(result, FieldMemOperand(result, Map::kPrototypeOffset));
  jmp_near(&done);

  RECORD_LINE();
  // Non-instance prototype: Fetch prototype from constructor field
  // in initial map.
  bind(&non_instance);
  mov(result, FieldMemOperand(result, Map::kConstructorOffset));

  // All done.
  bind(&done);
}


void MacroAssembler::CallStub(CodeStub* stub) {
  ASSERT(allow_stub_calls());  // Stub calls are not allowed in some stubs.
  RECORD_LINE();
  jsr(stub->GetCode(), RelocInfo::CODE_TARGET);
}


MaybeObject* MacroAssembler::TryCallStub(CodeStub* stub) {
  ASSERT(allow_stub_calls());  // Stub calls are not allowed in some stubs.
  Object* result;
  { MaybeObject* maybe_result = stub->TryGetCode();
    if (!maybe_result->ToObject(&result)) return maybe_result;
  }
  Handle<Code> code(Code::cast(result));
  Call(code, RelocInfo::CODE_TARGET, kNoASTId);
  return result;
}


void MacroAssembler::TailCallStub(CodeStub* stub) {
  ASSERT(allow_stub_calls());  // Stub calls are not allowed in some stubs.
  jmp(stub->GetCode(), RelocInfo::CODE_TARGET);
}


MaybeObject* MacroAssembler::TryTailCallStub(CodeStub* stub) {
  ASSERT(allow_stub_calls());  // Stub calls are not allowed in some stubs.
  Object* result;
  { MaybeObject* maybe_result = stub->TryGetCode();
    if (!maybe_result->ToObject(&result)) return maybe_result;
  }
  Jump(stub->GetCode(), RelocInfo::CODE_TARGET);
  return result;
}


static int AddressOffset(ExternalReference ref0, ExternalReference ref1) {
  return ref0.address() - ref1.address();
}


MaybeObject* MacroAssembler::TryCallApiFunctionAndReturn(
    ExternalReference function, int stack_space) {
  ExternalReference next_address =
      ExternalReference::handle_scope_next_address();
  const int kNextOffset = 0;
  const int kLimitOffset = AddressOffset(
      ExternalReference::handle_scope_limit_address(),
      next_address);
  const int kLevelOffset = AddressOffset(
      ExternalReference::handle_scope_level_address(),
      next_address);

  mov(r4, r0);
  mov(r5, r1);

  // Allocate HandleScope in callee-save registers.
  // TODO(stm): use of r10 and r11 is dangerous here (ip and rtmp)
  // We must be sure to not have them clobbered until the actual call.
  mov(r11, Operand(next_address));
  ldr(r8, MemOperand(r11, kNextOffset), r0);
  ldr(r9, MemOperand(r11, kLimitOffset), r0);
  ldr(r10, MemOperand(r11, kLevelOffset), r0);
  add(r10, r10, Operand(1), r0);
  str(r10, MemOperand(r11, kLevelOffset), r0);

  // Native call returns to the DirectCEntry stub which redirects to the
  // return address pushed on stack (could have moved after GC).
  // DirectCEntry stub itself is generated early and never moves.
  DirectCEntryStub stub(r2); // This scratch register must not be the return value
  stub.GenerateCall(this, function, r0, r1);

  // Move back the registers [r8, r11] => [r4, r7]
  mov(r4, r8);
  mov(r5, r9);
  mov(r6, r10);
  mov(r7, r11);

  Label promote_scheduled_exception;
  Label delete_allocated_handles;
  Label leave_exit_frame;

  // If result is non-zero, dereference to get the result value
  // otherwise set it to undefined.
  Label ltrue, lfalse;
  cmp(r0, Operand(0));
  bf_near(&lfalse);
  LoadRoot(r0, Heap::kUndefinedValueRootIndex);
  jmp_near(&ltrue);
  bind(&lfalse);
  ldr(r0, MemOperand(r0));
  bind(&ltrue);

  // No more valid handles (the result handle was the last one). Restore
  // previous handle scope.
  str(r4, MemOperand(r7, kNextOffset));
  if (emit_debug_code()) {
    ldr(r1, MemOperand(r7, kLevelOffset));
    cmp(r1, r6);
    Check(eq, "Unexpected level after return from api call");
  }
  sub(r6, r6, Operand(1));
  str(r6, MemOperand(r7, kLevelOffset));
  ldr(sh4_ip, MemOperand(r7, kLimitOffset));
  cmp(r5, sh4_ip);
  b(ne, &delete_allocated_handles);

  // Check if the function scheduled an exception.
  bind(&leave_exit_frame);
  LoadRoot(r4, Heap::kTheHoleValueRootIndex);
  mov(sh4_ip,
      Operand(ExternalReference::scheduled_exception_address(isolate())));
  ldr(r5, MemOperand(sh4_ip));
  cmp(r4, r5);
  b(ne, &promote_scheduled_exception);

  // LeaveExitFrame expects unwind space to be in a register.
  mov(r4, Operand(stack_space));
  LeaveExitFrame(false, r4);
  rts();

  bind(&promote_scheduled_exception);
  MaybeObject* result
      = TryTailCallExternalReference(
          ExternalReference(Runtime::kPromoteScheduledException, isolate()),
          0,
          1);
  if (result->IsFailure()) {
    return result;
  }

  // HandleScope limit has changed. Delete allocated extensions.
  bind(&delete_allocated_handles);
  str(r9, MemOperand(r7, kLimitOffset)); // use r9 instead of r5 for making PrepareCallCFunction() happy
  mov(r8, r0); // preserve in calle-saved the result (r0)
  PrepareCallCFunction(1, r9);
  mov(r4, Operand(ExternalReference::isolate_address())); // C-ABI paramater
  CallCFunction(
      ExternalReference::delete_handle_scope_extensions(isolate()), 1);
  mov(r0, r8);  // restore result (r0)
  jmp(&leave_exit_frame);

  return result;
}


void MacroAssembler::IllegalOperation(int num_arguments) {
  RECORD_LINE();
  if (num_arguments > 0) {
    add(sp, sp, Operand(num_arguments * kPointerSize));
  }
  LoadRoot(r0, Heap::kUndefinedValueRootIndex);
}

void MacroAssembler::SmiToDoubleFPURegister(Register smi,
                                            DwVfpRegister value,
                                            Register scratch) {
  asr(scratch, smi, Operand(kSmiTagSize));
  dfloat(value, scratch);
}

// Tries to get a signed int32 out of a double precision floating point heap
// number. Rounds towards 0. Branch to 'not_int32' if the double is out of the
// 32bits signed integer range.
void MacroAssembler::ConvertToInt32(Register source,
                                    Register dest,
                                    Register scratch,
                                    Register scratch2,
                                    DwVfpRegister double_scratch,
                                    Label *not_int32) {
  ASSERT(!source.is(sh4_ip) && !dest.is(sh4_ip) && !scratch.is(sh4_ip) &&
         !scratch2.is(sh4_ip));
  ASSERT(!source.is(sh4_rtmp) && !dest.is(sh4_rtmp) && !scratch.is(sh4_rtmp) &&
         !scratch2.is(sh4_rtmp));
  ASSERT(!source.is(dest) && !source.is(scratch) && !source.is(scratch2) &&
         !dest.is(scratch) && !dest.is(scratch2) && !scratch.is(scratch2));

  // TODO(stm): FPU
  if (CpuFeatures::IsSupported(FPU)) {
    sub(scratch, source, Operand(kHeapObjectTag));
    dldr(double_scratch, MemOperand(scratch, HeapNumber::kValueOffset));
    idouble(dest, double_scratch);
    // Signed vcvt instruction will saturate to the minimum (0x80000000) or
    // maximun (0x7fffffff) signed 32bits integer when the double is out of
    // range. When substracting one, the minimum signed integer becomes the
    // maximun signed integer.
    sub(scratch, dest, Operand(1));
    cmpge(scratch, Operand(LONG_MAX - 1));
    // If equal then dest was LONG_MAX, if greater dest was LONG_MIN.
    bt(not_int32);
  } else {
    // This code is faster for doubles that are in the ranges -0x7fffffff to
    // -0x40000000 or 0x40000000 to 0x7fffffff. This corresponds almost to
    // the range of signed int32 values that are not Smis.  Jumps to the label
    // 'not_int32' if the double isn't in the range -0x80000000.0 to
    // 0x80000000.0 (excluding the endpoints).
    Label right_exponent, done;
    // Get exponent word.
    ldr(scratch, FieldMemOperand(source, HeapNumber::kExponentOffset));
    // Get exponent alone in scratch2.
    Ubfx(scratch2,
         scratch,
         HeapNumber::kExponentShift,
         HeapNumber::kExponentBits);
    // Load dest with zero.  We use this either for the final shift or
    // for the answer.
    mov(dest, Operand(0));
    // Check whether the exponent matches a 32 bit signed int that is not a Smi.
    // A non-Smi integer is 1.xxx * 2^30 so the exponent is 30 (biased). This is
    // the exponent that we are fastest at and also the highest exponent we can
    // handle here.
    const uint32_t non_smi_exponent = HeapNumber::kExponentBias + 30;
    // The non_smi_exponent, 0x41d, is too big for ARM's immediate field so we
    // split it up to avoid a constant pool entry.  You can't do that in general
    // for cmp because of the overflow flag, but we know the exponent is in the
    // range 0-2047 so there is no overflow.
    int fudge_factor = 0x400;
    sub(scratch2, scratch2, Operand(fudge_factor));
    cmp(scratch2, Operand(non_smi_exponent - fudge_factor));
    // If we have a match of the int32-but-not-Smi exponent then skip some
    // logic.
    b(eq, &right_exponent);
    // If the exponent is higher than that then go to slow case.  This catches
    // numbers that don't fit in a signed int32, infinities and NaNs.
    cmpgt(scratch2, Operand(non_smi_exponent - fudge_factor));
    bt(not_int32);

    // We know the exponent is smaller than 30 (biased).  If it is less than
    // 0 (biased) then the number is smaller in magnitude than 1.0 * 2^0, ie
    // it rounds to zero.
    const uint32_t zero_exponent = HeapNumber::kExponentBias + 0;
    cmpge(scratch2, Operand(zero_exponent - fudge_factor));  // for branch below
    sub(scratch2, scratch2, Operand(zero_exponent - fudge_factor));
    // Dest already has a Smi zero.
    bf(&done);

    // We have an exponent between 0 and 30 in scratch2.  Subtract from 30 to
    // get how much to shift down.
    rsb(dest, scratch2, Operand(30));

    bind(&right_exponent);
    // Get the top bits of the mantissa.
    land(scratch2, scratch, Operand(HeapNumber::kMantissaMask));
    // Put back the implicit 1.
    lor(scratch2, scratch2, Operand(1 << HeapNumber::kExponentShift));
    // Shift up the mantissa bits to take up the space the exponent used to
    // take. We just orred in the implicit bit so that took care of one and
    // we want to leave the sign bit 0 so we subtract 2 bits from the shift
    // distance.
    const int shift_distance = HeapNumber::kNonMantissaBitsInTopWord - 2;
    lsl(scratch2, scratch2, Operand(shift_distance));
    // Put sign in zero flag.
    tst(scratch, Operand(HeapNumber::kSignMask));
    // Get the second half of the double. For some exponents we don't
    // actually need this because the bits get shifted out again, but
    // it's probably slower to test than just to do it.
    ldr(scratch, FieldMemOperand(source, HeapNumber::kMantissaOffset));
    // Shift down 22 bits to get the last 10 bits.
    lsr(scratch, scratch, Operand(32 - shift_distance));
    lor(scratch, scratch2, scratch);
    // Move down according to the exponent.
    lsr(dest, scratch, dest);
    // Fix sign if sign bit was set.
    rsb(dest, dest, Operand(0), ne);
    bind(&done);
  }
}


void MacroAssembler::EmitFPUTruncate(FPURoundingMode rounding_mode,
                                     Register result,
                                     DwVfpRegister double_input,
                                     Register scratch,
                                     CheckForInexactConversion check_inexact) {
  ASSERT(rounding_mode == kRoundToZero);
  int32_t check_inexact_conversion =
    (check_inexact == kCheckForInexactConversion) ? kFPUInexactExceptionBit : 0;

  idouble(result, double_input, scratch);

  // Check for FPU exceptions
  tst(scratch, Operand(kFPUExceptionMask | check_inexact_conversion));
}


void MacroAssembler::EmitOutOfInt32RangeTruncate(Register result,
                                                 Register input_high,
                                                 Register input_low,
                                                 Register scratch) {
  Label done, normal_exponent, restore_sign;

  // Extract the biased exponent in result.
  Ubfx(result,
       input_high,
       HeapNumber::kExponentShift,
       HeapNumber::kExponentBits);

  // Check for Infinity and NaNs, which should return 0.
  cmp(result, Operand(HeapNumber::kExponentMask));
  mov(result, Operand(0), eq);
  b(eq, &done);

  // Express exponent as delta to (number of mantissa bits + 31).
  sub(result,
      result,
      Operand(HeapNumber::kExponentBias + HeapNumber::kMantissaBits + 31));
  cmpgt(result, Operand(0));

  // If the delta is strictly positive, all bits would be shifted away,
  // which means that we can return 0.
  b(f, &normal_exponent);
  mov(result, Operand(0));
  b(&done);

  bind(&normal_exponent);
  const int kShiftBase = HeapNumber::kNonMantissaBitsInTopWord - 1;
  // Calculate shift.
  add(scratch, result, Operand(kShiftBase + HeapNumber::kMantissaBits));

  // Save the sign.
  Register sign = result;
  result = no_reg;
  land(sign, input_high, Operand(HeapNumber::kSignMask));

  // Set the implicit 1 before the mantissa part in input_high.
  orr(input_high,
      input_high,
      Operand(1 << HeapNumber::kMantissaBitsInTopWord));
  // Shift the mantissa bits to the correct position.
  // We don't need to clear non-mantissa bits as they will be shifted away.
  // If they weren't, it would mean that the answer is in the 32bit range.
  lsl(input_high, input_high, scratch);

  // Replace the shifted bits with bits from the lower mantissa word.
  Label pos_shift, shift_done;
  rsb(scratch, scratch, Operand(32));
  cmpge(scratch, Operand(0));
  bt(&pos_shift);

  // Negate scratch.
  rsb(scratch, scratch, Operand(0));
  lsl(input_low, input_low, scratch);
  b(&shift_done);

  bind(&pos_shift);
  lsr(input_low, input_low, scratch);

  bind(&shift_done);
  orr(input_high, input_high, input_low);
  // Restore sign if necessary.
  cmp(sign, Operand(0));
  result = sign;
  sign = no_reg;
  rsb(result, input_high, Operand(0));
  mov(result, input_high, eq);
  bind(&done);
}


void MacroAssembler::IndexFromHash(Register hash, Register index) {
  // If the hash field contains an array index pick it out. The assert checks
  // that the constants for the maximum number of digits for an array index
  // cached in the hash field and the number of bits reserved for it does not
  // conflict.
  ASSERT(TenToThe(String::kMaxCachedArrayIndexLength) <
         (1 << String::kArrayIndexValueBits));
  // We want the smi-tagged index in key.  kArrayIndexValueMask has zeros in
  // the low kHashShift bits.
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT(!hash.is(sh4_ip) && !index.is(sh4_ip));
  ASSERT(!hash.is(sh4_rtmp) && !index.is(sh4_rtmp));
  RECORD_LINE();
  Ubfx(hash, hash, String::kHashShift, String::kArrayIndexValueBits);
  lsl(index, hash, Operand(kSmiTagSize));
}


void MacroAssembler::Jump(intptr_t target, RelocInfo::Mode rmode) {
  RECORD_LINE();
  mov(sh4_ip, Operand(target, rmode));
  jmp(sh4_ip);
}

void MacroAssembler::Jump(Handle<Code> code, RelocInfo::Mode rmode) {
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  RECORD_LINE();
  Jump(reinterpret_cast<intptr_t>(code.location()), rmode);
}


void MacroAssembler::Call(Handle<Code> code,
                          RelocInfo::Mode rmode,
                          unsigned ast_id) {
  // Block constant pool when emitting call (might be redundant)
  BlockConstPoolScope block_const_pool(this);

  // TODO(stm): check whether this is necessary
  // Label start;
  // bind(&start);

  RECORD_LINE();
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  if (rmode == RelocInfo::CODE_TARGET && ast_id != kNoASTId) {
    SetRecordedAstId(ast_id);
    rmode = RelocInfo::CODE_TARGET_WITH_ID;
  }
  jsr(code, rmode, sh4_ip);

  // TODO(stm): check whether this is necessary
  // ASSERT_EQ(CallSize(code, rmode, ast_id, cond),
  //           SizeOfCodeGeneratedSince(&start));
}


void MacroAssembler::EmitECMATruncate(Register result,
                                      DwVfpRegister double_input,
                                      SwVfpRegister single_scratch,
                                      Register scratch,
                                      Register input_high,
                                      Register input_low) {
  ASSERT(CpuFeatures::IsSupported(FPU));
  ASSERT(!input_high.is(result));
  ASSERT(!input_low.is(result));
  ASSERT(!input_low.is(input_high));
  ASSERT(!scratch.is(result) &&
         !scratch.is(input_high) &&
         !scratch.is(input_low));
  ASSERT(!single_scratch.is(double_input.low()) &&
         !single_scratch.is(double_input.high()));

  Label done;

  // Do the conversion
  idouble(result, double_input);
  // Retrieve the FPSCR.
  str_fpscr(scratch);
  // Check for overflow and NaNs.
  tst(scratch, Operand(kFPUOverflowExceptionBit |
                       kFPUUnderflowExceptionBit |
                       kFPUInvalidExceptionBit |
                       kFPUInexactExceptionBit));
  // If we had no exceptions we are done.
  b(eq, &done);

  // Load the double value and perform a manual truncation.
  movd(input_low, input_high, double_input);
  EmitOutOfInt32RangeTruncate(result,
                              input_high,
                              input_low,
                              scratch);
  bind(&done);
}


void MacroAssembler::GetLeastBitsFromSmi(Register dst,
                                         Register src,
                                         int num_least_bits) {
  Ubfx(dst, src, kSmiTagSize, num_least_bits);
}


void MacroAssembler::GetLeastBitsFromInt32(Register dst,
                                           Register src,
                                           int num_least_bits) {
  ASSERT(!dst.is(sh4_rtmp) && !src.is(sh4_rtmp));
  land(dst, src, Operand((1 << num_least_bits) - 1));
}


void MacroAssembler::CallRuntime(const Runtime::Function* f,
                                 int num_arguments) {
  // No register conventions on entry.
  // All parameters are on stack.
  // Return value in r0 after call.
#ifdef DEBUG
  // Clobber parameter registers on entry.
  Dead(r0, r1, r2, r3);
  Dead(r4, r5, r6, r7);
#endif

  // If the expected number of arguments of the runtime function is
  // constant, we check that the actual number of arguments match the
  // expectation.
  RECORD_LINE();
  if (f->nargs >= 0 && f->nargs != num_arguments) {
    IllegalOperation(num_arguments);
    return;
  }

  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.
  mov(r0, Operand(num_arguments));
  mov(r1, Operand(ExternalReference(f, isolate())));
  CEntryStub stub(1);
  CallStub(&stub);
}

void MacroAssembler::CallRuntime(Runtime::FunctionId fid, int num_arguments) {
  RECORD_LINE();
  CallRuntime(Runtime::FunctionForId(fid), num_arguments);
}


void MacroAssembler::IsObjectJSStringType(Register object,
                                          Register scratch,
                                          Label* fail) {
  ASSERT(kNotStringTag != 0);
  ASSERT(!object.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!object.is(sh4_rtmp) && !scratch.is(sh4_rtmp));

  ldr(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
  ldrb(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));
  tst(scratch, Operand(kIsNotStringMask));
  bf(fail);
}


#ifdef ENABLE_DEBUGGER_SUPPORT
void MacroAssembler::DebugBreak() {
  RECORD_LINE();
  UNIMPLEMENTED_BREAK();
}
#endif


void MacroAssembler::Drop(int stack_elements) {
  RECORD_LINE();
  if (stack_elements > 0) {
    RECORD_LINE();
    add(sp, sp, Operand(stack_elements * kPointerSize));
  }
}


void MacroAssembler::Ret(int drop) {
  Drop(drop);
  Ret();
}


void MacroAssembler::UnimplementedBreak(const char *file, int line) {
  uint32_t file_id = 0;
  const char *base = strrchr(file, '/');
  if (base == NULL)
    base = file;
  while (*base) {
    file_id += *base;
    base++;
  }
  RECORD_LINE();
  mov(r0, Operand(file_id));
  mov(r1, Operand(line));
  bkpt();
}

void MacroAssembler::EnterFrame(StackFrame::Type type) {
  // r0-r3: must be preserved
  RECORD_LINE();
  Push(pr, fp, cp);
  mov(sh4_ip, Operand(Smi::FromInt(type)));
  push(sh4_ip);
  mov(sh4_ip, Operand(CodeObject()));
  push(sh4_ip);
  add(fp, sp, Operand(3 * kPointerSize));  // Adjust FP to point to saved FP.
}


void MacroAssembler::LeaveFrame(StackFrame::Type type) {
  // r0, r1, r2: must be preserved

  // Drop the execution stack down to the frame pointer and restore
  // the caller frame pointer and return address.
  RECORD_LINE();
  mov(sp, fp);
  Pop(pr, fp);
}


void MacroAssembler::EnterExitFrame(bool save_doubles, int stack_space, Register scratch) {
  // Parameters are on stack as if calling JS function
  // ARM -> ST40 mapping: ip -> scratch (defaults sh4_ip)

  // r0, r1, cp: must be preserved
  // sp, fp: input/output
  // Actual clobbers: scratch (r2 by default)

  // Setup the frame structure on the stack
  ASSERT_EQ(2 * kPointerSize, ExitFrameConstants::kCallerSPDisplacement);
  ASSERT_EQ(1 * kPointerSize, ExitFrameConstants::kCallerPCOffset);
  ASSERT_EQ(0 * kPointerSize, ExitFrameConstants::kCallerFPOffset);

  RECORD_LINE();
  // Save PR and FP
  Push(pr, fp);
  // Setup a new frame pointer
  mov(fp, sp);

  // Reserve room for saved entry sp and code object
  sub(sp, sp, Operand(2*kPointerSize));
  if (emit_debug_code()) {
    mov(scratch, Operand(0));
    str(scratch, MemOperand(fp, ExitFrameConstants::kSPOffset));
  }

  mov(scratch, Operand(CodeObject()));
  str(scratch, MemOperand(fp, ExitFrameConstants::kCodeOffset));

  // Save the frame pointer and the context in top.
  mov(scratch, Operand(ExternalReference(Isolate::kCEntryFPAddress, isolate())));
  str(fp, MemOperand(scratch));
  mov(scratch, Operand(ExternalReference(Isolate::kContextAddress, isolate())));
  str(cp, MemOperand(scratch));

  // Optionally save all double registers.
  if (save_doubles) {
    RECORD_LINE();
    UNIMPLEMENTED_BREAK();
  }

  // Reserve place for the return address and stack space and align the frame
  // preparing for calling the runtime function.
  const int frame_alignment = OS::ActivationFrameAlignment();
  sub(sp, sp, Operand((stack_space + 1) * kPointerSize));
  if (frame_alignment > 0) {
    ASSERT(IsPowerOf2(frame_alignment));
    land(sp, sp, Operand(-frame_alignment));
  }

  // Set the exit frame sp value to point just before the return address
  // location.
  add(scratch, sp, Operand(kPointerSize));
  str(scratch, MemOperand(fp, ExitFrameConstants::kSPOffset));
}


void MacroAssembler::LeaveExitFrame(bool save_doubles,
                                    Register argument_count) {
  ASSERT(!argument_count.is(sh4_ip));
  ASSERT(!argument_count.is(sh4_rtmp));
  // input: argument_count
  // r0, r1: results must be preserved
  // sp: stack pointer
  // fp: frame pointer

  // Actual clobbers: r3 and ip
  // r4 should be preserved: see the end of RegExpExecStub::Generate

  RECORD_LINE();
  if (save_doubles) {
    RECORD_LINE();
    UNIMPLEMENTED_BREAK();
  }

  // Clear top frame.
  mov(r3, Operand(0, RelocInfo::NONE32));
  mov(sh4_ip, Operand(ExternalReference(Isolate::kCEntryFPAddress, isolate())));
  str(r3, MemOperand(sh4_ip));

  // Restore current context from top and clear it in debug mode.
  mov(sh4_ip, Operand(ExternalReference(Isolate::kContextAddress, isolate())));
  ldr(cp, MemOperand(sh4_ip));
#ifdef DEBUG
  str(r3, MemOperand(sh4_ip));
#endif

  // Tear down the exit frame, pop the arguments, and return.
  mov(sp, fp);

  Pop(pr, fp);
  if (argument_count.is_valid()) {
    ASSERT(!argument_count.is(r3));
    lsl(r3, argument_count, Operand(kPointerSizeLog2));
    add(sp, sp, r3);
  }
}


void MacroAssembler::SetCallKind(Register dst, CallKind call_kind) {
  // This macro takes the dst register to make the code more readable
  // at the call sites. However, the dst register has to be r5 to
  // follow the calling convention which requires the call type to be
  // in r5.
  ASSERT(dst.is(r5));
  if (call_kind == CALL_AS_FUNCTION) {
    mov(dst, Operand(Smi::FromInt(1)));
  } else {
    mov(dst, Operand(Smi::FromInt(0)));
  }
}


void MacroAssembler::InvokePrologue(const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    Handle<Code> code_constant,
                                    Register code_reg,
                                    Label* done,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper,
                                    CallKind call_kind) {
  ASSERT(!code_reg.is(sh4_ip));
  ASSERT(!code_reg.is(sh4_rtmp));
  bool definitely_matches = false;
  Label regular_invoke;

  // Check whether the expected and actual arguments count match. If not,
  // setup registers according to contract with ArgumentsAdaptorTrampoline:
  // ARM -> SH4
  //  r0: actual arguments count
  //  r1: function (passed through to callee)
  //  r2: expected arguments count
  //  r3: callee code entry

  // The code below is made a lot easier because the calling code already sets
  // up actual and expected registers according to the contract if values are
  // passed in registers.
  ASSERT(actual.is_immediate() || actual.reg().is(r0));
  ASSERT(expected.is_immediate() || expected.reg().is(r2));
  ASSERT((!code_constant.is_null() && code_reg.is(no_reg)) || code_reg.is(r3));

  RECORD_LINE();
  if (expected.is_immediate()) {
    ASSERT(actual.is_immediate());
    if (expected.immediate() == actual.immediate()) {
      definitely_matches = true;
    } else {
      mov(r0, Operand(actual.immediate()));
      const int sentinel = SharedFunctionInfo::kDontAdaptArgumentsSentinel;
      if (expected.immediate() == sentinel) {
        // Don't worry about adapting arguments for builtins that
        // don't want that done. Skip adaption code by making it look
        // like we have a match between expected and actual number of
        // arguments.
        definitely_matches = true;
      } else {
        mov(r2, Operand(expected.immediate()));
      }
    }
  } else {
    if (actual.is_immediate()) {
      cmpeq(expected.reg(), Operand((actual.immediate())));
      bt(&regular_invoke);
      mov(r0, Operand(actual.immediate()));
    } else {
      cmpeq(expected.reg(), actual.reg());
      bt(&regular_invoke);
    }
  }

  RECORD_LINE();
  if (!definitely_matches) {
    if (!code_constant.is_null()) {
      mov(r3, Operand(code_constant));
      add(r3, r3, Operand(Code::kHeaderSize - kHeapObjectTag));
    }
    Handle<Code> adaptor =
        isolate()->builtins()->ArgumentsAdaptorTrampoline();
    if (flag == CALL_FUNCTION) {
      call_wrapper.BeforeCall(2 * kInstrSize);
      SetCallKind(r5, call_kind);
      Call(adaptor);
      call_wrapper.AfterCall();
      b(done);
    } else {
      SetCallKind(r5, call_kind);
      Jump(adaptor, RelocInfo::CODE_TARGET);
    }
    bind(&regular_invoke);
  }
}


void MacroAssembler::InvokeCode(Register code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                InvokeFlag flag,
                                const CallWrapper& call_wrapper,
                                CallKind call_kind) {
  Label done;
  // r1: must hold function pointer
  // actual: must be r0 if register
  ASSERT(actual.is_immediate() || actual.reg().is(r0));
  ASSERT(!code.is(sh4_ip) && !code.is(sh4_rtmp) && !code.is(r5));

  RECORD_LINE();
  InvokePrologue(expected, actual, Handle<Code>::null(), code, &done, flag,
                 call_wrapper, call_kind);
  RECORD_LINE();
  if (flag == CALL_FUNCTION) {
    call_wrapper.BeforeCall(2 * kInstrSize);
    SetCallKind(r5, call_kind);
    jsr(code);
    call_wrapper.AfterCall();
  } else {
    ASSERT(flag == JUMP_FUNCTION);
    SetCallKind(r5, call_kind);
    jmp(code);
  }

  // Continue here if InvokePrologue does handle the invocation due to
  // mismatched parameter counts.
  bind(&done);
}


void MacroAssembler::InvokeCode(Handle<Code> code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                RelocInfo::Mode rmode,
                                InvokeFlag flag,
                                CallKind call_kind) {
  Label done;

  InvokePrologue(expected, actual, code, no_reg, &done, flag,
                 NullCallWrapper(), call_kind);
  if (flag == CALL_FUNCTION) {
    SetCallKind(r5, call_kind);
    Call(code, rmode);
  } else {
    SetCallKind(r5, call_kind);
    Jump(code, rmode);
  }

  // Continue here if InvokePrologue does handle the invocation due to
  // mismatched parameter counts.
  bind(&done);
}


void MacroAssembler::InvokeFunction(Register fun,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper,
                                    CallKind call_kind) {
  // Contract with called JS functions requires that function is passed in r1.
  // Also enforce that actual is passed in r0 if not immediate
  ASSERT(fun.is(r1));
  ASSERT(actual.is_immediate() || actual.reg().is(r0));

  Register expected_reg = r2;
  Register code_reg = r3;

  RECORD_LINE();
  ldr(code_reg, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
  ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));
  ldr(expected_reg,
      FieldMemOperand(code_reg,
                      SharedFunctionInfo::kFormalParameterCountOffset));
  asr(expected_reg, expected_reg, Operand(kSmiTagSize));
  ldr(code_reg,
      FieldMemOperand(r1, JSFunction::kCodeEntryOffset));

  ParameterCount expected(expected_reg);
  InvokeCode(code_reg, expected, actual, flag, call_wrapper, call_kind);
}


void MacroAssembler::InvokeFunction(JSFunction* function,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    CallKind call_kind) {
  ASSERT(function->is_compiled());

  // Get the function and setup the context.
  mov(r1, Operand(Handle<JSFunction>(function)));
  ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));

  // Invoke the cached code.
  Handle<Code> code(function->code());
  ParameterCount expected(function->shared()->formal_parameter_count());
  if (V8::UseCrankshaft()) {
    // TODO(kasperl): For now, we always call indirectly through the
    // code field in the function to allow recompilation to take effect
    // without changing any of the call sites.
    ldr(r3, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
    InvokeCode(r3, expected, actual, flag, NullCallWrapper(), call_kind);
  } else {
    InvokeCode(code, expected, actual, RelocInfo::CODE_TARGET, flag, call_kind);
  }
}


void MacroAssembler::IsObjectJSObjectType(Register heap_object,
                                          Register map,
                                          Register scratch,
                                          Label* fail) {
  ldr(map, FieldMemOperand(heap_object, HeapObject::kMapOffset));
  IsInstanceJSObjectType(map, scratch, fail);
}


void MacroAssembler::IsInstanceJSObjectType(Register map,
                                            Register scratch,
                                            Label* fail) {
  ldrb(scratch, FieldMemOperand(map, Map::kInstanceTypeOffset));
  cmpge(scratch, Operand(FIRST_NONCALLABLE_SPEC_OBJECT_TYPE));
  bf(fail);
  cmpgt(scratch, Operand(LAST_NONCALLABLE_SPEC_OBJECT_TYPE));
  bt(fail);
}


MacroAssembler::MacroAssembler(Isolate* arg_isolate, void* buffer, int size)
    : Assembler(arg_isolate, buffer, size),
      generating_stub_(false),
      allow_stub_calls_(true) {
  if (isolate() != NULL) {
    code_object_ = Handle<Object>(isolate()->heap()->undefined_value(),
                                  isolate());
  }
}


void MacroAssembler::Move(Register dst, Handle<Object> value) {
  RECORD_LINE();
  mov(dst, Operand(value));
}


void MacroAssembler::Move(Register dst, Register src) {
  if (!dst.is(src)) {
    RECORD_LINE();
    mov(dst, src);
  }
}


void MacroAssembler::Sbfx(Register dst, Register src1, int lsb, int width) {
  ASSERT(!dst.is(sh4_rtmp) && !src1.is(sh4_rtmp));
  ASSERT(lsb >= 0 && lsb < 32);
  ASSERT(width > 0 && width <= 32);
  ASSERT(width + lsb <= 32);
  int32_t mask1 = width < 32 ? (1<<width)-1 : -1;
  int32_t mask = mask1 << lsb;
  land(dst, src1, Operand(mask));
  int shift_up = 32 - lsb - width;
  int shift_down = lsb + shift_up;
  if (shift_up != 0) {
    lsl(dst, dst, Operand(shift_up));
  }
  if (shift_down != 0) {
    asr(dst, dst, Operand(shift_down));
  }
}


void MacroAssembler::Bfi(Register dst,
                         Register src,
                         Register scratch,
                         int lsb,
                         int width) {
  ASSERT(0 <= lsb && lsb < 32);
  ASSERT(0 <= width && width <= 32);
  ASSERT(lsb + width <= 32);
  ASSERT(!dst.is(src) && !dst.is(scratch));
  if (width == 0) return;
  if (width == 32) {
    mov(dst, src);
    return;
  }
  int mask = (1 << (width + lsb)) - 1 - ((1 << lsb) - 1);
  bic(dst, dst, Operand(mask));
  land(scratch, src, Operand((1 << width) - 1));
  lsl(scratch, scratch, Operand(lsb));
  orr(dst, dst, scratch);
}


static const int kRegisterPassedArguments = 4;

int MacroAssembler::CalculateStackPassedWords(int num_reg_arguments,
                                              int num_double_arguments) {
  int stack_passed_words = 0;
  // Up to 4 simple arguments are passed in [r4, r7]
  if (num_reg_arguments > kRegisterPassedArguments) {
    stack_passed_words += num_reg_arguments - kRegisterPassedArguments;
  }

  // Up to 4 double arguments are passed in [dr4, dr10]
  if (num_double_arguments > kRegisterPassedArguments) {
    stack_passed_words += 2 * (num_double_arguments - kRegisterPassedArguments);
  }

  return stack_passed_words;
}

void MacroAssembler::PrepareCallCFunction(int num_reg_arguments,
                                          int num_double_arguments,
                                          Register scratch) {
  ASSERT(!scratch.is(sh4_ip));
  ASSERT(!scratch.is(sh4_rtmp));
  // Depending on the number of registers used, assert on the right scratch registers.
  ASSERT((num_reg_arguments < 1 || !scratch.is(r4)) &&
         (num_reg_arguments < 2 || !scratch.is(r5)) &&
         (num_reg_arguments < 3 || !scratch.is(r6)) &&
         (num_reg_arguments < 4 || !scratch.is(r7)));
  int frame_alignment = OS::ActivationFrameAlignment();
  int stack_passed_arguments = CalculateStackPassedWords(num_reg_arguments,
                                                         num_double_arguments);
  if (frame_alignment > kPointerSize) {
    RECORD_LINE();
    // Make stack end at alignment and make room for num_arguments - 4 words
    // and the original value of sp.
    mov(scratch, sp);
    sub(sp, sp, Operand((stack_passed_arguments + 1) * kPointerSize));
    ASSERT(IsPowerOf2(frame_alignment));
    land(sp, sp, Operand(-frame_alignment));
    str(scratch, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    RECORD_LINE();
    sub(sp, sp, Operand(stack_passed_arguments * kPointerSize));
  }
}


void MacroAssembler::CallCFunction(ExternalReference function,
                                   int num_reg_arguments,
                                   int num_double_arguments) {
  RECORD_LINE();
  CallCFunctionHelper(no_reg, function, r3, num_reg_arguments, num_double_arguments);
}


void MacroAssembler::CallCFunctionHelper(Register function,
                                         ExternalReference function_reference,
                                         Register scratch,
                                         int num_reg_arguments,
                                         int num_double_arguments) {
  ASSERT(!function.is(sh4_ip));
  ASSERT(!function.is(sh4_rtmp));
#if defined(V8_HOST_ARCH_SH4)
  if (emit_debug_code()) {
    int frame_alignment = OS::ActivationFrameAlignment();
    int frame_alignment_mask = frame_alignment - 1;
    if (frame_alignment > kPointerSize) {
      ASSERT(IsPowerOf2(frame_alignment));
      Label alignment_as_expected;
      tst(sp, Operand(frame_alignment_mask));
      b(eq, &alignment_as_expected);
      // Don't use Check here, as it will call Runtime_Abort possibly
      // re-entering here.
      stop("Unexpected alignment");
      bind(&alignment_as_expected);
    }
  }
#endif

  {
  // Block constant pool when emitting call (might be redundant)
  BlockConstPoolFor(3);
#ifdef DEBUG
  Label begin;
  bind(&begin);
#endif
  // Just call directly. The function called cannot cause a GC, or
  // allow preemption, so the return address in the link register
  // stays correct.
  if (function.is(no_reg)) {
    RECORD_LINE();
    mov(scratch, Operand(function_reference));
    function = scratch;
  }
  RECORD_LINE();
  jsr(function);
  ASSERT(!constant_pool_pool_ ||
         (pc_offset() - begin.pos() == 2 * kInstrSize ||
          pc_offset() - begin.pos() == 3 * kInstrSize));
  }

  int stack_passed_arguments = CalculateStackPassedWords(num_reg_arguments,
                                                         num_double_arguments);
  if (OS::ActivationFrameAlignment() > kPointerSize) {
    ldr(sp, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    add(sp, sp, Operand(stack_passed_arguments * sizeof(kPointerSize)));
  }
}


void MacroAssembler::PushTryHandler(CodeLocation try_location,
                                    HandlerType type) {
  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 5 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kStateOffset == 1 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kContextOffset == 2 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 3 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kPCOffset == 4 * kPointerSize);

  // The pc (return address) is passed in register pr.
  if (try_location == IN_JAVASCRIPT) {
    RECORD_LINE();
    if (type == TRY_CATCH_HANDLER) {
      mov(r3, Operand(StackHandler::TRY_CATCH));
    } else {
      mov(r3, Operand(StackHandler::TRY_FINALLY));
    }
    Push(pr, fp, cp, r3);
    // Save the current handler as the next handler.
    mov(r3, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
    ldr(r1, MemOperand(r3));
    ASSERT(StackHandlerConstants::kNextOffset == 0);
    push(r1);
    // Link this handler as the new current one.
    str(sp, MemOperand(r3));
  } else {
    // Must preserve r0-r4, r5-r7 are available.
    ASSERT(try_location == IN_JS_ENTRY);
    RECORD_LINE();
    // The frame pointer does not point to a JS frame so we save NULL
    // for ebp. We expect the code throwing an exception to check ebp
    // before dereferencing it to restore the context.
    push(pr);
    push(Operand(0));  // Null Frame pointer
    push(Operand(Smi::FromInt(0)));  // Indicates no context
    push(Operand(StackHandler::ENTRY));  // State
    // Save the current handler as the next handler.
    mov(r7, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
    mov(r6, MemOperand(r7));
    push(r6);
    // Link this handler as the new current one.
    str(sp, MemOperand(r7));
  }
}


void MacroAssembler::PopTryHandler() {
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  RECORD_LINE();
  pop(r1);
  mov(sh4_ip, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  add(sp, sp, Operand(StackHandlerConstants::kSize - kPointerSize));
  str(r1, MemOperand(sh4_ip));
}


void MacroAssembler::Throw(Register value) {
  ASSERT(!value.is(sh4_ip));
  ASSERT(!value.is(sh4_rtmp));

  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 5 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kStateOffset == 1 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kContextOffset == 2 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 3 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kPCOffset == 4 * kPointerSize);

  // r0 is expected to hold the exception.
  if (!value.is(r0)) {
    RECORD_LINE();
    mov(r0, value);
  }

  RECORD_LINE();
  // Drop the sp to the top of the handler.
  mov(r3, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  ldr(sp, MemOperand(r3));

  // Restore the next handler.
  RECORD_LINE();
  pop(r2);
  str(r2, MemOperand(r3));
  Pop(fp, cp, r3);

  // If the handler is a JS frame, restore the context to the frame.
  // (r3 == ENTRY) == (fp == 0) == (cp == 0), so we could test any
  // of them.
  RECORD_LINE();
  Label skip;
  cmp(r3, Operand(StackHandler::ENTRY));
  bt(&skip);
  str(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  bind(&skip);

#ifdef DEBUG
  if (emit_debug_code()) {
//    mov(lr, Operand(pc));
  }
#endif
  pop(pr);
  rts();
}


void MacroAssembler::ThrowUncatchable(UncatchableExceptionType type,
                                      Register value) {
  ASSERT(!value.is(sh4_ip));
  ASSERT(!value.is(sh4_rtmp));
  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 5 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kStateOffset == 1 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kContextOffset == 2 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 3 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kPCOffset == 4 * kPointerSize);
  // r0 is expected to hold the exception.
  if (!value.is(r0)) {
    RECORD_LINE();
    mov(r0, value);
  }

  RECORD_LINE();
  // Drop sp to the top stack handler.
  mov(r3, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  ldr(sp, MemOperand(r3));

  // Unwind the handlers until the ENTRY handler is found.
  Label loop, done;
  bind(&loop);
  RECORD_LINE();
  // Load the type of the current stack handler.
  const int kStateOffset = StackHandlerConstants::kStateOffset;
  ldr(r2, MemOperand(sp, kStateOffset));
  cmpeq(r2, Operand(StackHandler::ENTRY), r3);
  bt(&done);
  RECORD_LINE();
  // Fetch the next handler in the list.
  const int kNextOffset = StackHandlerConstants::kNextOffset;
  ldr(sp, MemOperand(sp, kNextOffset));
  jmp(&loop);
  bind(&done);
  RECORD_LINE();

  // Set the top handler address to next handler past the current ENTRY handler.
  pop(r2);
  mov(r3, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  str(r2, MemOperand(r3));

  if (type == OUT_OF_MEMORY) {
    // Set external caught exception to false.
    ExternalReference external_caught(
        Isolate::kExternalCaughtExceptionAddress, isolate());
    RECORD_LINE();
    mov(r0, Operand(false, RelocInfo::NONE32));
    mov(r2, Operand(external_caught));
    str(r0, MemOperand(r2));

    // Set pending exception and r0 to out of memory exception.
    Failure* out_of_memory = Failure::OutOfMemoryException();
    mov(r0, Operand(reinterpret_cast<int32_t>(out_of_memory)));
    mov(r2, Operand(ExternalReference(Isolate::kPendingExceptionAddress,
                                      isolate())));
    str(r0, MemOperand(r2));
  }

  // Stack layout at this point. See also StackHandlerConstants.
  // sp ->   state (ENTRY)
  //         cp
  //         fp
  //         pr

  RECORD_LINE();
  // Restore context and frame pointer, discard state (r2).
  Pop(fp, cp, r2);
#ifdef DEBUG
  if (emit_debug_code()) {
//    mov(lr, Operand(pc));
  }
#endif
  pop(pr);
  rts();
}


#include "map-sh4.h"    // Define register map
void MacroAssembler::CheckAccessGlobalProxy(Register holder_reg,
                                            Register scratch,
                                            Label* miss) {
  Label same_contexts;

  ASSERT(!holder_reg.is(scratch));
  ASSERT(!holder_reg.is(ip));
  ASSERT(!scratch.is(ip));

  // Load current lexical context from the stack frame.
  ldr(scratch, MemOperand(fp, StandardFrameConstants::kContextOffset));
  // In debug mode, make sure the lexical context is set.
#ifdef DEBUG
  cmp(scratch, Operand(0, RelocInfo::NONE32));
  Check(ne, "we should not have an empty lexical context");
#endif

  // Load the global context of the current context.
  int offset = Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
  ldr(scratch, FieldMemOperand(scratch, offset));
  ldr(scratch, FieldMemOperand(scratch, GlobalObject::kGlobalContextOffset));

  // Check the context is a global context.
  if (emit_debug_code()) {
    // TODO(119): avoid push(holder_reg)/pop(holder_reg)
    // Cannot use ip as a temporary in this verification code. Due to the fact
    // that ip is clobbered as part of cmp with an object Operand.
    push(holder_reg);  // Temporarily save holder on the stack.
    // Read the first word and compare to the global_context_map.
    ldr(holder_reg, FieldMemOperand(scratch, HeapObject::kMapOffset));
    LoadRoot(ip, Heap::kGlobalContextMapRootIndex);
    cmp(holder_reg, ip);
    Check(eq, "JSGlobalObject::global_context should be a global context.");
    pop(holder_reg);  // Restore holder.
  }

  // Check if both contexts are the same.
  ldr(ip, FieldMemOperand(holder_reg, JSGlobalProxy::kContextOffset));
  cmp(scratch, ip);
  b(eq, &same_contexts);

  // Check the context is a global context.
  if (emit_debug_code()) {
    // TODO(119): avoid push(holder_reg)/pop(holder_reg)
    // Cannot use ip as a temporary in this verification code. Due to the fact
    // that ip is clobbered as part of cmp with an object Operand.
    push(holder_reg);  // Temporarily save holder on the stack.
    mov(holder_reg, ip);  // Move ip to its holding place.
    LoadRoot(ip, Heap::kNullValueRootIndex);
    cmp(holder_reg, ip);
    Check(ne, "JSGlobalProxy::context() should not be null.");

    ldr(holder_reg, FieldMemOperand(holder_reg, HeapObject::kMapOffset));
    LoadRoot(ip, Heap::kGlobalContextMapRootIndex);
    cmp(holder_reg, ip);
    Check(eq, "JSGlobalObject::global_context should be a global context.");
    // Restore ip is not needed. ip is reloaded below.
    pop(holder_reg);  // Restore holder.
    // Restore ip to holder's context.
    ldr(ip, FieldMemOperand(holder_reg, JSGlobalProxy::kContextOffset));
  }

  // Check that the security token in the calling global object is
  // compatible with the security token in the receiving global
  // object.
  int token_offset = Context::kHeaderSize +
                     Context::SECURITY_TOKEN_INDEX * kPointerSize;

  ldr(scratch, FieldMemOperand(scratch, token_offset));
  ldr(ip, FieldMemOperand(ip, token_offset));
  cmp(scratch, ip);
  b(ne, miss);

  bind(&same_contexts);
}


void MacroAssembler::LoadFromNumberDictionary(Label* miss,
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
  // t2 - used for the index into the dictionary.
  Label done;

  // Compute the hash code from the untagged key.  This must be kept in sync
  // with ComputeIntegerHash in utils.h.
  //
  // hash = ~hash + (hash << 15);
  mvn(t1, t0);
  lsl(t2, t0, Operand(15));
  add(t0, t1, t2);
  // hash = hash ^ (hash >> 12);
  lsr(t2, t0, Operand(12));
  eor(t0, t0, t2);
  // hash = hash + (hash << 2);
  lsl(t2, t0, Operand(2));
  add(t0, t0, t2);
  // hash = hash ^ (hash >> 4);
  lsr(t2, t0, Operand(4));
  eor(t0, t0, t2);
  // hash = hash * 2057;
  mov(t1, Operand(2057));
  mul(t0, t0, t1);
  // hash = hash ^ (hash >> 16);
  lsr(t2, t0, Operand(16));
  eor(t0, t0, t2);

  // Compute the capacity mask.
  ldr(t1, FieldMemOperand(elements, NumberDictionary::kCapacityOffset));
  asr(t1, t1, Operand(kSmiTagSize));  // convert smi to int
  sub(t1, t1, Operand(1));

  // Generate an unrolled loop that performs a few probes before giving up.
  static const int kProbes = 4;
  for (int i = 0; i < kProbes; i++) {
    // Use t2 for index calculations and keep the hash intact in t0.
    mov(t2, t0);
    // Compute the masked index: (hash + i + i * i) & mask.
    if (i > 0) {
      add(t2, t2, Operand(NumberDictionary::GetProbeOffset(i)));
    }
    land(t2, t2, t1);

    // Scale the index by multiplying by the element size.
    ASSERT(NumberDictionary::kEntrySize == 3);
    lsl(ip, t2, Operand(1));
    add(t2, t2, ip);  // t2 = t2 * 3

    // Check if the key is identical to the name.
    lsl(ip, t2, Operand(kPointerSizeLog2));
    add(t2, elements, ip);
    ldr(ip, FieldMemOperand(t2, NumberDictionary::kElementsStartOffset));
    cmp(key, ip);
    if (i != kProbes - 1) {
      b(eq, &done);
    } else {
      b(ne, miss);
    }
  }

  bind(&done);
  // Check that the value is a normal property.
  // t2: elements + (index * kPointerSize)
  const int kDetailsOffset =
      NumberDictionary::kElementsStartOffset + 2 * kPointerSize;
  ldr(t1, FieldMemOperand(t2, kDetailsOffset));
  tst(t1, Operand(Smi::FromInt(PropertyDetails::TypeField::kMask)));
  b(ne, miss);

  // Get the value at the masked, scaled index and return.
  const int kValueOffset =
      NumberDictionary::kElementsStartOffset + kPointerSize;
  ldr(result, FieldMemOperand(t2, kValueOffset));
}

#include "map-sh4.h"    // Undefine register map



void MacroAssembler::LoadFromSafepointRegisterSlot(Register dst, Register src) {
  ldr(dst, SafepointRegisterSlot(src));
}


int MacroAssembler::SafepointRegisterStackIndex(int reg_code) {
  UNIMPLEMENTED();
  return 0;
}


MemOperand MacroAssembler::SafepointRegisterSlot(Register reg) {
  return MemOperand(sp, SafepointRegisterStackIndex(reg.code()) * kPointerSize);
}


void MacroAssembler::Ldrd(Register dst1, Register dst2,
                          const MemOperand& src) {
  ASSERT(src.rn().is(no_reg));
  ASSERT_EQ(0, dst1.code() % 2);
  ASSERT_EQ(dst1.code() + 1, dst2.code());
  ASSERT(!dst1.is(sh4_ip) && !dst2.is(sh4_ip));
  ASSERT(!dst1.is(sh4_rtmp) && !dst2.is(sh4_rtmp));

  // Generate two ldr instructions if ldrd is not available.
  //  if (CpuFeatures::IsSupported(ARMv7)) {
  //    CpuFeatures::Scope scope(ARMv7);
  //    ldrd(dst1, dst2, src, cond);
  //  } else
  {
    MemOperand src2(src);
    src2.set_offset(src2.offset() + 4);
    if (dst1.is(src.rm())) {
      ldr(dst2, src2);
      ldr(dst1, src);
    } else {
      ldr(dst1, src);
      ldr(dst2, src2);
    }
  }
}


void MacroAssembler::Strd(Register src1, Register src2,
                          const MemOperand& dst) {
  ASSERT(dst.rn().is(no_reg));
  ASSERT_EQ(0, src1.code() % 2);
  ASSERT_EQ(src1.code() + 1, src2.code());
  ASSERT(!src1.is(sh4_ip) && !src2.is(sh4_ip));
  ASSERT(!src1.is(sh4_rtmp) && !src2.is(sh4_rtmp));

  // Generate two str instructions if strd is not available.
  //  if (CpuFeatures::IsSupported(ARMv7)) {
  //    CpuFeatures::Scope scope(ARMv7);
  //    strd(src1, src2, dst, cond);
  //  } else
  {
    MemOperand dst2(dst);
    dst2.set_offset(dst2.offset() + 4);
    str(src1, dst);
    str(src2, dst2);
  }
}


void MacroAssembler::InvokeBuiltin(Builtins::JavaScript id,
                                   InvokeFlag flags,
                                   const CallWrapper& call_wrapper) {
  // No register conventions on entry.
  // All parameters are on stack.
  // Return value in r0 after call.
#ifdef DEBUG
  // Clobber parameter registers on entry.
  Dead(r0, r1, r2, r3);
  Dead(r4, r5, r6, r7);
#endif

  RECORD_LINE();
  GetBuiltinEntry(r2, id);
  if (flags == CALL_FUNCTION) {
    RECORD_LINE();
    call_wrapper.BeforeCall(2 * kInstrSize);
    SetCallKind(r5, CALL_AS_METHOD);
    jsr(r2);
    call_wrapper.AfterCall();
  } else {
    ASSERT(flags == JUMP_FUNCTION);
    RECORD_LINE();
    SetCallKind(r5, CALL_AS_METHOD);
    jmp(r2);
  }
}


void MacroAssembler::GetBuiltinFunction(Register target,
                                        Builtins::JavaScript id) {
  ASSERT(!target.is(sh4_ip));
  ASSERT(!target.is(sh4_rtmp));
  RECORD_LINE();
  // Load the builtins object into target register.
  ldr(target, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  ldr(target, FieldMemOperand(target, GlobalObject::kBuiltinsOffset));
  // Load the JavaScript builtin function from the builtins object.
  ldr(target, FieldMemOperand(target,
                          JSBuiltinsObject::OffsetOfFunctionWithId(id)));
}


void MacroAssembler::GetBuiltinEntry(Register target, Builtins::JavaScript id) {
  // FIXME(stm): why r1 ??
  ASSERT(!target.is(r1));
  ASSERT(!target.is(sh4_rtmp));
  ASSERT(!target.is(sh4_ip));
  RECORD_LINE();
  GetBuiltinFunction(r1, id);
  RECORD_LINE();
  // Load the code entry point from the builtins object.
  ldr(target, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
}


void MacroAssembler::SetCounter(StatsCounter* counter, int value,
                                Register scratch1, Register scratch2) {
  RECORD_LINE();
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  ASSERT(!scratch1.is(sh4_ip) && !scratch2.is(sh4_ip));
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch1, Operand(value));
    mov(scratch2, Operand(ExternalReference(counter)));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::IncrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  ASSERT(!scratch1.is(sh4_ip) && !scratch2.is(sh4_ip));
  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch2, Operand(ExternalReference(counter)));
    ldr(scratch1, MemOperand(scratch2));
    add(scratch1, scratch1, Operand(value));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::DecrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  ASSERT(!scratch1.is(sh4_ip) && !scratch2.is(sh4_ip));

  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch2, Operand(ExternalReference(counter)));
    ldr(scratch1, MemOperand(scratch2));
    sub(scratch1, scratch1, Operand(value));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::Assert(Condition cond, const char* msg) {
  if (emit_debug_code())
    Check(cond, msg);
}


void MacroAssembler::AssertRegisterIsRoot(Register reg, Register scratch,
                                          Heap::RootListIndex index) {
  ASSERT(!reg.is(scratch));
  if (emit_debug_code()) {
    LoadRoot(scratch, index);
    cmp(reg, scratch);
    Check(eq, "Register did not match expected root");
  }
}


void MacroAssembler::AssertFastElements(Register elements) {
  RECORD_LINE();
  if (emit_debug_code()) {
    ASSERT(!elements.is(sh4_rtmp));
    ASSERT(!elements.is(sh4_ip));
    Label ok;
    RECORD_LINE();
    push(elements);
    ldr(elements, FieldMemOperand(elements, HeapObject::kMapOffset));
    LoadRoot(sh4_ip, Heap::kFixedArrayMapRootIndex);
    cmp(elements, sh4_ip);
    b(eq, &ok);
    RECORD_LINE();
    LoadRoot(sh4_ip, Heap::kFixedCOWArrayMapRootIndex);
    cmp(elements, sh4_ip);
    b(eq, &ok);
    RECORD_LINE();
    Abort("JSObject with fast elements map has slow elements");
    bind(&ok);
    RECORD_LINE();
    pop(elements);
  }
}


void MacroAssembler::Check(Condition cond, const char* msg) {
  Label L;
  RECORD_LINE();
  b(cond, &L);
  Abort(msg);
  // will not return here
  bind(&L);
}

void MacroAssembler::DebugPrint(Register obj) {
  RECORD_LINE();
  push(obj);
  CallRuntime(Runtime::kDebugPrint, 1);
}

void MacroAssembler::Abort(const char* msg) {
  Label abort_start;
  bind(&abort_start);
  RECORD_LINE();
  // We want to pass the msg string like a smi to avoid GC
  // problems, however msg is not guaranteed to be aligned
  // properly. Instead, we pass an aligned pointer that is
  // a proper v8 smi, but also pass the alignment difference
  // from the real pointer as a smi.
  intptr_t p1 = reinterpret_cast<intptr_t>(msg);
  intptr_t p0 = (p1 & ~kSmiTagMask) + kSmiTag;
  ASSERT(reinterpret_cast<Object*>(p0)->IsSmi());
#ifdef DEBUG
  if (msg != NULL) {
    RecordComment("Abort message: ");
    RecordComment(msg);
  }
#endif
  // Disable stub call restrictions to always allow calls to abort.
  AllowStubCallsScope allow_scope(this, true);

  RECORD_LINE();
  mov(r0, Operand(p0));
  push(r0);
  mov(r0, Operand(Smi::FromInt(p1 - p0)));
  push(r0);
  CallRuntime(Runtime::kAbort, 2);
  // will not return here
  if (is_const_pool_blocked()) {
    // ARM and MIPS pad the number of instructions in the abort block to
    // 10 and 14 respectively. The reason for this and how it relates to the
    // constant pool (being blocked) is not given.
  }
}


void MacroAssembler::AllocateInNewSpace(int object_size,
                                        Register result,
                                        Register scratch1,
                                        Register scratch2,
                                        Label* gc_required,
                                        AllocationFlags flags) {
  RECORD_LINE();
  if (!FLAG_inline_new) {
    if (emit_debug_code()) {
      // Trash the registers to simulate an allocation failure.
      RECORD_LINE();
      mov(result, Operand(0x7091));
      mov(scratch1, Operand(0x7191));
      mov(scratch2, Operand(0x7291));
    }
    RECORD_LINE();
    jmp(gc_required);
    return;
  }

  // Assert that the register arguments are different and that none of
  // them are ip. ip is used explicitly in the code generated below.
  ASSERT(!result.is(scratch1) && !result.is(scratch2) &&
         !scratch1.is(scratch2));
  ASSERT(!result.is(sh4_ip) && !scratch1.is(sh4_ip) &&
         !scratch2.is(sh4_ip));
  ASSERT(!result.is(sh4_rtmp) && !scratch1.is(sh4_rtmp) &&
         !scratch2.is(sh4_rtmp));

  // Make object size into bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    object_size *= kPointerSize;
  }
  ASSERT_EQ(0, object_size & kObjectAlignmentMask);

  // Check relative positions of allocation top and limit addresses.
  // The values must be adjacent in memory to allow the use of LDM.
  // Also, assert that the registers are numbered such that the values
  // are loaded in the correct order.
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address(isolate());
  ExternalReference new_space_allocation_limit =
      ExternalReference::new_space_allocation_limit_address(isolate());
  intptr_t top =
      reinterpret_cast<intptr_t>(new_space_allocation_top.address());
  intptr_t limit =
      reinterpret_cast<intptr_t>(new_space_allocation_limit.address());
  ASSERT((limit - top) == kPointerSize);

  // Set up allocation top address and object size registers.
  Register topaddr = scratch1;
  Register obj_size_reg = scratch2;
  RECORD_LINE();
  mov(topaddr, Operand(new_space_allocation_top));
  mov(obj_size_reg, Operand(object_size));

  // This code stores a temporary value in sh4_ip (ARM:ip).
  // This is OK, as the code below
  // does not need sh4_ip (ARM:ip) for implicit literal generation.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    RECORD_LINE();
    // Load allocation top into result and allocation limit into sh4_ip (ARM:ip)
    ldr(result, MemOperand(topaddr));
    ldr(sh4_ip, MemOperand(topaddr, 4));
  } else {
    if (emit_debug_code()) {
      RECORD_LINE();
      // Assert that result actually contains top on entry.
      // sh4_ip (ARM:ip) is used
      // immediately below so this use of ip does not cause difference with
      // respect to register content between debug and release mode.
      ldr(sh4_ip, MemOperand(topaddr));
      cmp(result, sh4_ip);
      Check(eq, "Unexpected allocation top");
    }
    RECORD_LINE();
    // Load allocation limit into sh4_ip (ARM: ip).
    // Result already contains allocation top.
    ldr(sh4_ip, MemOperand(topaddr, limit - top));
  }

  RECORD_LINE();
  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top.
  addc(scratch2, result, obj_size_reg);
  b(t, gc_required);

  RECORD_LINE();
  cmphi(scratch2, sh4_ip);
  bt(gc_required);

  RECORD_LINE();
  str(scratch2, MemOperand(topaddr));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    RECORD_LINE();
    add(result, result, Operand(kHeapObjectTag));
  }
}


void MacroAssembler::AllocateInNewSpace(Register object_size,
                                        Register result,
                                        Register scratch1,
                                        Register scratch2,
                                        Label* gc_required,
                                        AllocationFlags flags) {
  RECORD_LINE();
  if (!FLAG_inline_new) {
    if (emit_debug_code()) {
      // Trash the registers to simulate an allocation failure.
      RECORD_LINE();
      mov(result, Operand(0x7091));
      mov(scratch1, Operand(0x7191));
      mov(scratch2, Operand(0x7291));
    }
    RECORD_LINE();
    jmp(gc_required);
    return;
  }

  // Assert that the register arguments are different and that none of
  // them are ip. ip is used explicitly in the code generated below.
  // Also assert that rtmp is not used as it is used in assembler-sh4.cc.
  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!result.is(sh4_ip));
  ASSERT(!scratch1.is(sh4_ip));
  ASSERT(!scratch2.is(sh4_ip));
  ASSERT(!result.is(sh4_rtmp));
  ASSERT(!scratch1.is(sh4_rtmp));
  ASSERT(!scratch2.is(sh4_rtmp));

  // Check relative positions of allocation top and limit addresses.
  // The values must be adjacent in memory to allow the use of LDM.
  // Also, assert that the registers are numbered such that the values
  // are loaded in the correct order.
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address(isolate());
  ExternalReference new_space_allocation_limit =
      ExternalReference::new_space_allocation_limit_address(isolate());
  intptr_t top =
      reinterpret_cast<intptr_t>(new_space_allocation_top.address());
  intptr_t limit =
      reinterpret_cast<intptr_t>(new_space_allocation_limit.address());
  ASSERT((limit - top) == kPointerSize);
  ASSERT(result.code() < sh4_ip.code());

  // Set up allocation top address.
  Register topaddr = scratch1;

  RECORD_LINE();
  mov(topaddr, Operand(new_space_allocation_top));

  // This code stores a temporary value in sh4_ip (ARM:ip).
  // This is OK, as the code below
  // does not need sh4_ip (ARM:ip) for implicit literal generation.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    RECORD_LINE();
    // Load allocation top into result and allocation limit into sh4_ip (ARM:ip)
    ldr(result, MemOperand(topaddr));
    ldr(sh4_ip, MemOperand(topaddr, 4));
  } else {
    if (emit_debug_code()) {
      RECORD_LINE();
      // Assert that result actually contains top on entry.
      // sh4_ip (ARM:ip) is used
      // immediately below so this use of ip does not cause difference with
      // respect to register content between debug and release mode.
      ldr(sh4_ip, MemOperand(topaddr));
      cmp(result, sh4_ip);
      Check(eq, "Unexpected allocation top");
    }
    RECORD_LINE();
    // Load allocation limit into sh4_ip (ARM: ip).
    // Result already contains allocation top.
    ldr(sh4_ip, MemOperand(topaddr, limit - top));
  }

  RECORD_LINE();
  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top. Object size may be in words so a shift is
  // required to get the number of bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    RECORD_LINE();
    lsl(scratch2, object_size, Operand(kPointerSizeLog2));
    addc(scratch2, result, scratch2);
  } else {
    RECORD_LINE();
    addc(scratch2, result, object_size);
  }
  RECORD_LINE();
  b(t, gc_required);
  RECORD_LINE();
  cmphi(scratch2, sh4_ip);
  bt(gc_required);
  RECORD_LINE();

  // Update allocation top. result temporarily holds the new top.
  if (emit_debug_code()) {
    RECORD_LINE();
    tst(scratch2, Operand(kObjectAlignmentMask));
    Check(eq, "Unaligned allocation in new space");
  }
  RECORD_LINE();
  str(scratch2, MemOperand(topaddr));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    RECORD_LINE();
    add(result, result, Operand(kHeapObjectTag));
  }
}


void MacroAssembler::InitializeNewString(Register string,
                                         Register length,
                                         Heap::RootListIndex map_index,
                                         Register scratch1,
                                         Register scratch2) {
  RECORD_LINE();
  lsl(scratch1, length, Operand(kSmiTagSize));
  LoadRoot(scratch2, map_index);
  str(scratch1, FieldMemOperand(string, String::kLengthOffset));
  mov(scratch1, Operand(String::kEmptyHashField));
  str(scratch2, FieldMemOperand(string, HeapObject::kMapOffset));
  str(scratch1, FieldMemOperand(string, String::kHashFieldOffset));
}


void MacroAssembler::UndoAllocationInNewSpace(Register object,
                                              Register scratch) {
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address(isolate());

  // Make sure the object has no tag before resetting top.
  land(object, object, Operand(~kHeapObjectTagMask));
#ifdef DEBUG
  // Check that the object un-allocated is below the current top.
  mov(scratch, Operand(new_space_allocation_top));
  ldr(scratch, MemOperand(scratch));
  cmpge(object, scratch);
  Check(ne, "Undo allocation of non allocated memory");
#endif
  // Write the address of the object to un-allocate as the current top.
  mov(scratch, Operand(new_space_allocation_top));
  str(object, MemOperand(scratch));
}


// Allocate a sequential string. All the header fields of the string object
// are initialized.
void MacroAssembler::AllocateTwoByteString(Register result,
                                           Register length,
                                           Register scratch1,
                                           Register scratch2,
                                           Register scratch3,
                                           Label* gc_required) {
  // Calculate the number of bytes needed for the characters in the string while
  // observing object alignment.
  ASSERT((SeqTwoByteString::kHeaderSize & kObjectAlignmentMask) == 0);
  RECORD_LINE();
  lsl(scratch1, length, Operand(1));  // Length in bytes, not chars.
  add(scratch1, scratch1,
      Operand(kObjectAlignmentMask + SeqTwoByteString::kHeaderSize));
  land(scratch1, scratch1, Operand(~kObjectAlignmentMask));

  // Allocate two-byte string in new space.
  AllocateInNewSpace(scratch1,
                     result,
                     scratch2,
                     scratch3,
                     gc_required,
                     TAG_OBJECT);

  // Set the map, length and hash field.
  RECORD_LINE();
  InitializeNewString(result,
                      length,
                      Heap::kStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateAsciiString(Register result,
                                         Register length,
                                         Register scratch1,
                                         Register scratch2,
                                         Register scratch3,
                                         Label* gc_required) {
  // Calculate the number of bytes needed for the characters in the string while
  // observing object alignment.
  ASSERT((SeqAsciiString::kHeaderSize & kObjectAlignmentMask) == 0);
  ASSERT(kCharSize == 1);
  RECORD_LINE();
  add(scratch1, length,
      Operand(kObjectAlignmentMask + SeqAsciiString::kHeaderSize));
  land(scratch1, scratch1, Operand(~kObjectAlignmentMask));

  // Allocate ASCII string in new space.
  AllocateInNewSpace(scratch1,
                     result,
                     scratch2,
                     scratch3,
                     gc_required,
                     TAG_OBJECT);

  RECORD_LINE();
  // Set the map, length and hash field.
  InitializeNewString(result,
                      length,
                      Heap::kAsciiStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateAsciiConsString(Register result,
                                             Register length,
                                             Register scratch1,
                                             Register scratch2,
                                             Label* gc_required) {
  RECORD_LINE();
  AllocateInNewSpace(ConsString::kSize,
                     result,
                     scratch1,
                     scratch2,
                     gc_required,
                     TAG_OBJECT);

  RECORD_LINE();
  InitializeNewString(result,
                      length,
                      Heap::kConsAsciiStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateTwoByteConsString(Register result,
                                               Register length,
                                               Register scratch1,
                                               Register scratch2,
                                               Label* gc_required) {
  RECORD_LINE();
  AllocateInNewSpace(ConsString::kSize,
                     result,
                     scratch1,
                     scratch2,
                     gc_required,
                     TAG_OBJECT);

  RECORD_LINE();
  InitializeNewString(result,
                      length,
                      Heap::kConsStringMapRootIndex,
                      scratch1,
                      scratch2);
}


// Allocates a heap number or jumps to the need_gc label if the young space
// is full and a scavenge is needed.
void MacroAssembler::AllocateHeapNumber(Register result,
                                        Register scratch1,
                                        Register scratch2,
                                        Register heap_number_map,
                                        Label* gc_required) {
  // Allocate an object in the heap for the heap number and tag it as a heap
  // object.
  RECORD_LINE();
  AllocateInNewSpace(HeapNumber::kSize,
                     result,
                     scratch1,
                     scratch2,
                     gc_required,
                     TAG_OBJECT);

  // Store heap number map in the allocated object.
  RECORD_LINE();
  AssertRegisterIsRoot(heap_number_map, scratch1, Heap::kHeapNumberMapRootIndex);
  RECORD_LINE();
  str(heap_number_map, FieldMemOperand(result, HeapObject::kMapOffset));
}


void MacroAssembler::CopyBytes(Register src,
                               Register dst,
                               Register length,
                               Register scratch) {
  Label align_loop, align_loop_1, word_loop, byte_loop, byte_loop_1, done;

  // Align src before copying in word size chunks.
  bind(&align_loop);
  cmp(length, Operand(0));
  b(eq, &done, Label::kNear);
  bind(&align_loop_1);
  tst(src, Operand(kPointerSize - 1));
  b(eq, &word_loop, Label::kNear);
  ldrb(scratch, MemOperand(src));
  add(src, src, Operand(1));
  strb(scratch, MemOperand(dst));
  add(dst, dst, Operand(1));
  dt(length);
  b(ne, &byte_loop_1, Label::kNear);

  // Copy bytes in word size chunks.
  bind(&word_loop);
  if (emit_debug_code()) {
    tst(src, Operand(kPointerSize - 1));
    Assert(eq, "Expecting alignment for CopyBytes");
  }
  cmpge(length, Operand(kPointerSize));
  bf_near(&byte_loop);
  ldr(scratch, MemOperand(src));
  add(src, src, Operand(kPointerSize));
#if CAN_USE_UNALIGNED_ACCESSES
  str(scratch, MemOperand(dst));
  add(dst, dst, Operand(kPointerSize));
#else
  strb(scratch, MemOperand(dst));
  add(dst, dst, Operand(1));
  lsr(scratch, scratch, Operand(8));
  strb(scratch, MemOperand(dst));
  add(dst, dst, Operand(1));
  lsr(scratch, scratch, Operand(8));
  strb(scratch, MemOperand(dst));
  add(dst, dst, Operand(1));
  lsr(scratch, scratch, Operand(8));
  strb(scratch, MemOperand(dst));
  add(dst, dst, Operand(1));
#endif
  sub(length, length, Operand(kPointerSize));
  b_near(&word_loop);

  // Copy the last bytes if any left.
  bind(&byte_loop);
  cmp(length, Operand(0));
  b(eq, &done, Label::kNear);
  bind(&byte_loop_1);
  ldrb(scratch, MemOperand(src));
  add(src, src, Operand(1));
  strb(scratch, MemOperand(dst));
  add(dst, dst, Operand(1));
  dt(length);
  b(ne, &byte_loop_1, Label::kNear);
  bind(&done);
}


void MacroAssembler::CountLeadingZeros(Register zeros,   // Answer.
                                       Register source,  // Input.
                                       Register scratch) {
  ASSERT(!zeros.is(source) || !source.is(scratch));
  ASSERT(!zeros.is(scratch));
  ASSERT(!scratch.is(sh4_rtmp));
  ASSERT(!source.is(sh4_rtmp));
  ASSERT(!zeros.is(sh4_rtmp));
  ASSERT(!scratch.is(sh4_ip));
  ASSERT(!source.is(sh4_ip));
  ASSERT(!zeros.is(sh4_ip));
  RECORD_LINE();

  Label l0, l1, l2, l3, l4, l5;
  cmpeq(source, Operand(0));
  bf_near(&l0);
  mov(zeros, Operand(32));
  jmp_near(&l5);

  bind(&l0);
  // Be carefull to save source in scratch, source and zeros may be the same register
  mov(scratch, source);
  mov(zeros, Operand(0));
  // Top 16.
  tst(scratch, Operand(0xffff0000));
  bf_near(&l1);
  add(zeros, zeros, Operand(16));
  lsl(scratch, scratch, Operand(16));
  // Top 8.
  bind(&l1);
  tst(scratch, Operand(0xff000000));
  bf_near(&l2);
  add(zeros, zeros, Operand(8));
  lsl(scratch, scratch, Operand(8));
  // Top 4.
  bind(&l2);
  tst(scratch, Operand(0xf0000000));
  bf_near(&l3);
  add(zeros, zeros, Operand(4));
  lsl(scratch, scratch, Operand(4));
  // Top 2.
  bind(&l3);
  tst(scratch, Operand(0xc0000000));
  bf_near(&l4);
  add(zeros, zeros, Operand(2));
  lsl(scratch, scratch, Operand(2));
  // Top bit.
  bind(&l4);
  tst(scratch, Operand(0x80000000u));
  bf_near(&l5);
  add(zeros, zeros, Operand(1));
  bind(&l5);
}


// Copies a fixed number of fields of heap objects from src to dst.
void MacroAssembler::CopyFields(Register dst,
                                Register src,
                                RegList temps,
                                int field_count) {
  // At least one bit set in the first 15 registers.
  ASSERT((temps & ((1 << 15) - 1)) != 0);
  ASSERT((temps & dst.bit()) == 0);
  ASSERT((temps & src.bit()) == 0);

  // Primitive implementation using only one temporary register.
  Register tmp = no_reg;
  // Find a temp register in temps list.
  for (int i = 0; i < 15; i++) {
    if ((temps & (1 << i)) != 0) {
      tmp.set_code(i);
      break;
    }
  }
  ASSERT(!tmp.is(no_reg));
  RECORD_LINE();
  for (int i = 0; i < field_count; i++) {
    RECORD_LINE();
    mov(tmp, FieldMemOperand(src, i * kPointerSize));
    mov(FieldMemOperand(dst, i * kPointerSize), tmp);
  }
}


void MacroAssembler::Usat(Register dst, int satpos, Register src) {
    ASSERT((satpos > 0) && (satpos <= 31));

    int satval = (1 << satpos) - 1;

    if (!src.is(dst)) {
      mov(dst, src);
    }
    cmpge(dst, Operand(0));
    mov(dst, Operand(0), f);  // 0 if negative.
    cmpgt(dst, Operand(satval));
    mov(dst, Operand(satval), t);  // satval if > satval
}


void MacroAssembler::LoadRoot(Register destination,
                              Heap::RootListIndex index) {
  RECORD_LINE();
  mov(destination, MemOperand(roots, index << kPointerSizeLog2));
}

void MacroAssembler::StoreRoot(Register source,
                               Heap::RootListIndex index) {
  RECORD_LINE();
  mov(MemOperand(roots, index << kPointerSizeLog2), source);
}

void MacroAssembler::Ret(Condition cond) {
  ASSERT(cond == al || cond == eq || cond == ne);
  if (cond == al) {
    RECORD_LINE();
    rts();
  } else {
    // TODO: block constant pool
    RECORD_LINE();
    if (cond == eq) {
      bf_(2);
    } else {
      bt_(2);
    }
    rts();
    // bt/bf destination
  }
}


void MacroAssembler::JumpToExternalReference(const ExternalReference& builtin) {
  RECORD_LINE();
  mov(r1, Operand(builtin));
  CEntryStub stub(1);
  RECORD_LINE();
  jmp(stub.GetCode(), RelocInfo::CODE_TARGET);
}


MaybeObject* MacroAssembler::TryJumpToExternalReference(
    const ExternalReference& builtin) {
  mov(r1, Operand(builtin));
  CEntryStub stub(1);
  return TryTailCallStub(&stub);
}


void MacroAssembler::CallExternalReference(const ExternalReference& ext,
                                           int num_arguments) {
  mov(r0, Operand(num_arguments));
  mov(r1, Operand(ext));

  CEntryStub stub(1);
  CallStub(&stub);
}


void MacroAssembler::TailCallExternalReference(const ExternalReference& ext,
                                               int num_arguments,
                                               int result_size) {
  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.

  // Block constant pool when emitting call (might be redundant)
  BlockConstPoolScope block_const_pool(this);

  RECORD_LINE();
  mov(r0, Operand(num_arguments));
  JumpToExternalReference(ext);
}


MaybeObject* MacroAssembler::TryTailCallExternalReference(
    const ExternalReference& ext, int num_arguments, int result_size) {
  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.

  // Block constant pool when emitting call (might be redundant)
  BlockConstPoolScope block_const_pool(this);

  RECORD_LINE();
  mov(r0, Operand(num_arguments));
  return TryJumpToExternalReference(ext);
}


void MacroAssembler::TailCallRuntime(Runtime::FunctionId fid,
                                     int num_arguments,
                                     int result_size) {
  RECORD_LINE();
  TailCallExternalReference(ExternalReference(fid, isolate()),
                            num_arguments,
                            result_size);
}


void MacroAssembler::Ubfx(Register dst, Register src, int lsb, int width) {
  ASSERT(!dst.is(sh4_rtmp) && !src.is(sh4_rtmp));
  ASSERT(lsb >= 0 && lsb < 32);
  ASSERT(width > 0 && width <= 32);
  ASSERT(width + lsb <= 32);
  // Extract unsigned value from bits src1[lsb..lsb+width-1] into dst
  int32_t mask1 = width < 32 ? (1<<width)-1 : -1;
  int32_t mask = mask1 << lsb;
  RECORD_LINE();
  land(dst, src, Operand(mask));
  if (lsb != 0) {
    RECORD_LINE();
    lsr(dst, dst, Operand(lsb));
  }
}


void MacroAssembler::Bfc(Register dst, int lsb, int width) {
  ASSERT(!dst.is(sh4_rtmp));
  ASSERT(lsb >= 0 && lsb < 32);
  ASSERT(width > 0 && width <= 32);
  ASSERT(width + lsb <= 32);
  // Clear bits [lsb..lsb+width-1] of dst
  int32_t mask1 = width < 32 ? (1<<width)-1 : -1;
  int32_t mask = mask1 << lsb;
  RECORD_LINE();
  land(dst, dst, Operand(~mask));
}


MacroAssembler* MacroAssembler::RecordFunctionLine(const char* function,
                                                   int line) {
  if (FLAG_code_comments) {
    /* 10(strlen of MAXINT) + 1(separator) +1(nul). */
    int size = strlen("/line/")+strlen(function) + 10 + 1 + 1;
    char *buffer = new char[size];
    snprintf(buffer, size, "/line/%s/%d", function, line);
    buffer[size-1] = '\0';
    RecordComment(buffer);
  }
  return this;
}


void MacroAssembler::InNewSpace(Register object,
                                Register scratch,
                                Condition cond,
                                Label* branch) {
  ASSERT(!object.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!object.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  ASSERT(cond == eq || cond == ne);
  RECORD_LINE();
  land(scratch, object,
       Operand(ExternalReference::new_space_mask(isolate())));
  mov(sh4_ip, Operand(ExternalReference::new_space_start(isolate())));
  cmpeq(scratch, sh4_ip);
  b(cond, branch);
}


void MacroAssembler::RecordWriteHelper(Register object,
                                       Register address,
                                       Register scratch) {
  ASSERT(!object.is(sh4_ip) && !address.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!object.is(sh4_rtmp) && !address.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  if (emit_debug_code()) {
    // Check that the object is not in new space.
    Label not_in_new_space;
    RECORD_LINE();
    InNewSpace(object, scratch, ne, &not_in_new_space);
    Abort("new-space object passed to RecordWriteHelper");
    bind(&not_in_new_space);
  }

  RECORD_LINE();
  // Calculate page address.
  Bfc(object, 0, kPageSizeBits);

  // Calculate region number.
  Ubfx(address, address, Page::kRegionSizeLog2,
       kPageSizeBits - Page::kRegionSizeLog2);

  // Mark region dirty.
  mov(scratch, MemOperand(object, Page::kDirtyFlagOffset));
  mov(sh4_ip, Operand(1));
  // The region number is in [0,31], hence we can use the fast case (wrap == true)
  lsl(sh4_ip, sh4_ip, address, true);
  lor(scratch, scratch, sh4_ip);
  mov(MemOperand(object, Page::kDirtyFlagOffset), scratch);
}


// Will clobber 4 registers: object, scratch0/1, sh4_ip (ARM:ip).  The
// register 'object' contains a heap object pointer.  The heap object
// tag is shifted away.
void MacroAssembler::RecordWrite(Register object,
                                 Operand offset,
                                 Register scratch0,
                                 Register scratch1) {
  // The compiled code assumes that record write doesn't change the
  // context register, so we check that none of the clobbered
  // registers are cp.
  ASSERT(!object.is(cp) && !scratch0.is(cp) && !scratch1.is(cp));
  // Also check that the scratch are not sh4_rtmp (super scratch).
  ASSERT(!object.is(sh4_rtmp) && !scratch0.is(sh4_rtmp) && !scratch1.is(sh4_rtmp));
  // Also check that the scratch are not sh4_ip (used in macro-assembler-sh4.cc).
  ASSERT(!object.is(sh4_ip) && !scratch0.is(sh4_ip) && !scratch1.is(sh4_ip));

  Label done;

  RECORD_LINE();
  // First, test that the object is not in the new space.  We cannot set
  // region marks for new space pages.
  InNewSpace(object, scratch0, eq, &done);

  // Add offset into the object.
  add(scratch0, object, offset);

  // Record the actual write.
  RecordWriteHelper(object, scratch0, scratch1);

  bind(&done);
  RECORD_LINE();

  // Clobber all input registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    RECORD_LINE();
    mov(object, Operand(BitCast<int32_t>(kSH4ZapValue)));
    mov(scratch0, Operand(BitCast<int32_t>(kSH4ZapValue)));
    mov(scratch1, Operand(BitCast<int32_t>(kSH4ZapValue)));
  }
}


// Will clobber 4 registers: object, scratch0/1, sh4_ip (ARM:ip). The
// register 'object' contains a heap object pointer. The heap object
// tag is shifted away.
// Note that offset is preserved.
void MacroAssembler::RecordWrite(Register object,
                                 Register offset,
                                 Register scratch0,
                                 Register scratch1) {
  // The compiled code assumes that record write doesn't change the
  // context register, so we check that none of the clobbered
  // registers are cp.
  ASSERT(!object.is(cp) && !offset.is(cp) && !scratch0.is(cp) && !scratch1.is(cp));
  // Also check that the scratch are not sh4_rtmp (super scratch).
  ASSERT(!object.is(sh4_rtmp) && !offset.is(sh4_rtmp) && !scratch0.is(sh4_rtmp) && !scratch1.is(sh4_rtmp));
  // Also check that the scratch are not sh4_ip (used in macro-assembler-sh4.cc).
  ASSERT(!object.is(sh4_ip) && !offset.is(sh4_ip) && !scratch0.is(sh4_ip) && !scratch1.is(sh4_ip));

  Label done;

  // First, test that the object is not in the new space.  We cannot set
  // region marks for new space pages.
  InNewSpace(object, scratch0, eq, &done);

  // Add offset into the object.
  add(scratch0, object, offset);

  // Record the actual write.
  RecordWriteHelper(object, scratch0, scratch1);

  bind(&done);

  // Clobber all input registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    mov(object, Operand(BitCast<int32_t>(kSH4ZapValue)));
    mov(scratch0, Operand(BitCast<int32_t>(kSH4ZapValue)));
    mov(scratch1, Operand(BitCast<int32_t>(kSH4ZapValue)));
  }
}


// Will clobber 4 registers: object, address, scratch, sh4_ip (ARM:ip).  The
// register 'object' contains a heap object pointer.  The heap object
// tag is shifted away.
void MacroAssembler::RecordWrite(Register object,
                                 Register address,
                                 Register scratch) {
  // The compiled code assumes that record write doesn't change the
  // context register, so we check that none of the clobbered
  // registers are cp.
  ASSERT(!object.is(cp) && !address.is(cp) && !scratch.is(cp));
  // Also check that the scratch are not sh4_rtmp (super scratch).
  ASSERT(!object.is(sh4_rtmp) && !address.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  // Also check that the scratch are not sh4_ip (used in macro-assembler-sh4.cc).
  ASSERT(!object.is(sh4_ip) && !address.is(sh4_ip) && !scratch.is(sh4_ip));

  Label done;
  RECORD_LINE();

  // First, test that the object is not in the new space.  We cannot set
  // region marks for new space pages.
  InNewSpace(object, scratch, eq, &done);

  // Record the actual write.
  RecordWriteHelper(object, address, scratch);

  bind(&done);
  RECORD_LINE();

  // Clobber all input registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    RECORD_LINE();
    mov(object, Operand(BitCast<int32_t>(kSH4ZapValue)));
    mov(address, Operand(BitCast<int32_t>(kSH4ZapValue)));
    mov(scratch, Operand(BitCast<int32_t>(kSH4ZapValue)));
  }
}


// Clobbers: sh4_rtmp, dst
// live-in: cp
// live-out: cp, dst
void MacroAssembler::LoadContext(Register dst, int context_chain_length) {
  ASSERT(!dst.is(sh4_rtmp));
  RECORD_LINE();
  if (context_chain_length > 0) {
    RECORD_LINE();
    // Move up the chain of contexts to the context containing the slot.
    ldr(dst, MemOperand(cp, Context::SlotOffset(Context::PREVIOUS_INDEX)));
    for (int i = 1; i < context_chain_length; i++) {
      RECORD_LINE();
      ldr(dst, MemOperand(dst, Context::SlotOffset(Context::PREVIOUS_INDEX)));
    }
  } else {
    RECORD_LINE();
    // Slot is in the current function context.  Move it into the
    // destination register in case we store into it (the write barrier
    // cannot be allowed to destroy the context in esi).
    mov(dst, cp);
  }
}


void MacroAssembler::LoadGlobalFunction(int index, Register function) {
  // Load the global or builtins object from the current context.
  ldr(function, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  // Load the global context from the global or builtins object.
  ldr(function, FieldMemOperand(function,
                                GlobalObject::kGlobalContextOffset));
  // Load the function from the global context.
  ldr(function, MemOperand(function, Context::SlotOffset(index)));
}


void MacroAssembler::LoadGlobalFunctionInitialMap(Register function,
                                                  Register map,
                                                  Register scratch) {
  ASSERT(!scratch.is(sh4_ip));
  ASSERT(!scratch.is(sh4_rtmp));
  // Load the initial map. The global functions all have initial maps.
  ldr(map, FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));
  if (emit_debug_code()) {
    Label ok;
    Label fail;
    CheckMap(map, scratch, Heap::kMetaMapRootIndex, &fail, false);
    b_near(&ok);
    bind(&fail);
    Abort("Global functions must have initial map");
    bind(&ok);
  }
}


void MacroAssembler::JumpIfNotPowerOfTwoOrZero(
    Register reg,
    Register scratch,
    Label* not_power_of_two_or_zero) {
  ASSERT(!reg.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  // Note: actually the case 0x80000000 is considered a power of two
  // (not a neg value)
  sub(scratch, reg, Operand(1));
  cmpge(scratch, Operand(0));
  bf(not_power_of_two_or_zero);
  tst(scratch, reg);
  b(ne, not_power_of_two_or_zero);
}


void MacroAssembler::JumpIfNotPowerOfTwoOrZeroAndNeg(
    Register reg,
    Register scratch,
    Label* zero_and_neg,
    Label* not_power_of_two) {
  ASSERT(!reg.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  // Note: actually the case 0x80000000 is considered a pozer of two
  // (not a neg value)
  sub(scratch, reg, Operand(1));
  cmpge(scratch, Operand(0));
  bf(zero_and_neg);
  tst(scratch, reg);
  b(ne, not_power_of_two);
}


void MacroAssembler::JumpIfNotBothSmi(Register reg1,
                                      Register reg2,
                                      Label* on_not_both_smi,
                                      Label::Distance distance) {
  ASSERT(!reg1.is(sh4_rtmp) && !reg2.is(sh4_rtmp));
  STATIC_ASSERT(kSmiTag == 0);
  RECORD_LINE();
  tst(reg1, Operand(kSmiTagMask));
  b(ne, on_not_both_smi, distance);
  tst(reg2, Operand(kSmiTagMask));
  b(ne, on_not_both_smi, distance);
}


void MacroAssembler::JumpIfEitherSmi(Register reg1,
                                     Register reg2,
                                     Label* on_either_smi,
                                     Label::Distance distance) {
  ASSERT(!reg1.is(sh4_rtmp) && !reg2.is(sh4_rtmp));
  STATIC_ASSERT(kSmiTag == 0);
  RECORD_LINE();
  tst(reg1, Operand(kSmiTagMask));
  b(eq, on_either_smi, distance);
  tst(reg2, Operand(kSmiTagMask));
  b(eq, on_either_smi, distance);
}


void MacroAssembler::AbortIfSmi(Register object) {
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT(!object.is(sh4_rtmp));

  tst(object, Operand(kSmiTagMask));
  Assert(ne, "Operand is a smi");
}


void MacroAssembler::AbortIfNotSmi(Register object) {
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT(!object.is(sh4_rtmp));

  tst(object, Operand(kSmiTagMask));
  Assert(eq, "Operand is not smi");
}


void MacroAssembler::AbortIfNotString(Register object) {
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT(!object.is(sh4_ip));
  ASSERT(!object.is(sh4_rtmp));

  tst(object, Operand(kSmiTagMask));
  Assert(ne, "Operand is not a string");
  RECORD_LINE();
  push(object);
  ldr(object, FieldMemOperand(object, HeapObject::kMapOffset));
  CompareInstanceType(object, object, FIRST_NONSTRING_TYPE, hs);
  pop(object);
  Assert(ne, "Operand is not a string");
}


void MacroAssembler::AbortIfNotRootValue(Register src,
                                         Heap::RootListIndex root_value_index,
                                         const char* message) {
  ASSERT(!src.is(sh4_ip));
  ASSERT(!src.is(sh4_rtmp));
  CompareRoot(src, root_value_index);
  Assert(eq, message);
}


void MacroAssembler::PrintRegisterValue(Register reg) {
  ASSERT(!reg.is(r4) && !reg.is(r5) && !reg.is(r6) && !reg.is(r7));
  ASSERT(!reg.is(sh4_rtmp));
  Label gc_required, skip, not_smi;
  RECORD_LINE();
  EnterInternalFrame();
  // Save reg as it is scratched by WriteInt32ToHeapNumberStub()
  push(reg);
  pushm(kJSCallerSaved);
  TrySmiTag(reg, &not_smi, r5/*scratch*/);
  mov(r4, reg);
  jmp(&skip);
  bind(&not_smi);
  RECORD_LINE();
  LoadRoot(r7, Heap::kHeapNumberMapRootIndex);
  AllocateHeapNumber(r4/*result heap number*/, r5/*scratch*/, r6/*scratch*/,
                     r7/*heap_number_map*/, &gc_required);
  WriteInt32ToHeapNumberStub stub(reg, r4, r5/*scratch*/);
  CallStub(&stub);
  jmp(&skip);
  bind(&gc_required);
  RECORD_LINE();
  Abort("GC required while dumping number");
  bind(&skip);
  RECORD_LINE();
  push(r4);
  CallRuntime(Runtime::kNumberToString, 1);
  push(r0);
  CallRuntime(Runtime::kGlobalPrint, 1);
  popm(kJSCallerSaved);
  pop(reg);
  LeaveInternalFrame();
}


void MacroAssembler::JumpIfNotHeapNumber(Register object,
                                         Register heap_number_map,
                                         Register scratch,
                                         Label* on_not_heap_number) {
  RECORD_LINE();
  ASSERT(!scratch.is(sh4_ip));
  ASSERT(!scratch.is(sh4_rtmp));
  AssertRegisterIsRoot(heap_number_map, scratch, Heap::kHeapNumberMapRootIndex);
  ldr(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
  cmp(scratch, heap_number_map);
  b(ne, on_not_heap_number);
}


void MacroAssembler::JumpIfNonSmisNotBothSequentialAsciiStrings(
    Register first,
    Register second,
    Register scratch1,
    Register scratch2,
    Label* failure) {

  ASSERT(!first.is(sh4_ip) && !second.is(sh4_ip) && !scratch1.is(sh4_ip) &&
         !scratch2.is(sh4_ip));
  ASSERT(!first.is(sh4_rtmp) && !second.is(sh4_rtmp) && !scratch1.is(sh4_rtmp) &&
         !scratch2.is(sh4_rtmp));
  RECORD_LINE();
  // Test that both first and second are sequential ASCII strings.
  // Assume that they are non-smis.
  ldr(scratch1, FieldMemOperand(first, HeapObject::kMapOffset));
  ldr(scratch2, FieldMemOperand(second, HeapObject::kMapOffset));
  ldrb(scratch1, FieldMemOperand(scratch1, Map::kInstanceTypeOffset));
  ldrb(scratch2, FieldMemOperand(scratch2, Map::kInstanceTypeOffset));

  JumpIfBothInstanceTypesAreNotSequentialAscii(scratch1,
                                               scratch2,
                                               scratch1,
                                               scratch2,
                                               failure);
}


void MacroAssembler::JumpIfNotBothSequentialAsciiStrings(Register first,
                                                         Register second,
                                                         Register scratch1,
                                                         Register scratch2,
                                                         Label* failure) {
  ASSERT(!first.is(sh4_ip) && !second.is(sh4_ip) && !scratch1.is(sh4_ip) &&
         !scratch2.is(sh4_ip));
  ASSERT(!first.is(sh4_rtmp) && !second.is(sh4_rtmp) && !scratch1.is(sh4_rtmp) &&
         !scratch2.is(sh4_rtmp));
  RECORD_LINE();
  // Check that neither is a smi.
  STATIC_ASSERT(kSmiTag == 0);
  land(scratch1, first, second);
  tst(scratch1, Operand(kSmiTagMask));
  b(eq, failure);
  JumpIfNonSmisNotBothSequentialAsciiStrings(first,
                                             second,
                                             scratch1,
                                             scratch2,
                                             failure);
}


void MacroAssembler::JumpIfBothInstanceTypesAreNotSequentialAscii(
    Register first,
    Register second,
    Register scratch1,
    Register scratch2,
    Label* failure) {
  ASSERT(!first.is(sh4_ip) && !second.is(sh4_ip) && !scratch1.is(sh4_ip) &&
         !scratch2.is(sh4_ip));
  ASSERT(!first.is(sh4_rtmp) && !second.is(sh4_rtmp) && !scratch1.is(sh4_rtmp) &&
         !scratch2.is(sh4_rtmp));

  int kFlatAsciiStringMask =
      kIsNotStringMask | kStringEncodingMask | kStringRepresentationMask;
  int kFlatAsciiStringTag = ASCII_STRING_TYPE;
  RECORD_LINE();
  land(scratch1, first, Operand(kFlatAsciiStringMask));
  land(scratch2, second, Operand(kFlatAsciiStringMask));
  cmp(scratch1, Operand(kFlatAsciiStringTag));
  b(ne, failure);
  RECORD_LINE();
  cmp(scratch2, Operand(kFlatAsciiStringTag));
  b(ne, failure);
}


void MacroAssembler::JumpIfInstanceTypeIsNotSequentialAscii(Register type,
                                                            Register scratch,
                                                            Label* failure) {
  ASSERT(!type.is(sh4_rtmp) && !scratch.is(sh4_rtmp));

  int kFlatAsciiStringMask =
      kIsNotStringMask | kStringEncodingMask | kStringRepresentationMask;
  int kFlatAsciiStringTag = ASCII_STRING_TYPE;
  RECORD_LINE();
  land(scratch, type, Operand(kFlatAsciiStringMask));
  cmp(scratch, Operand(kFlatAsciiStringTag));
  b(ne, failure);
}


void MacroAssembler::AllocateTwoByteSlicedString(Register result,
                                                 Register length,
                                                 Register scratch1,
                                                 Register scratch2,
                                                 Label* gc_required) {
  AllocateInNewSpace(SlicedString::kSize,
                     result,
                     scratch1,
                     scratch2,
                     gc_required,
                     TAG_OBJECT);

  InitializeNewString(result,
                      length,
                      Heap::kSlicedStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateAsciiSlicedString(Register result,
                                               Register length,
                                               Register scratch1,
                                               Register scratch2,
                                               Label* gc_required) {
  AllocateInNewSpace(SlicedString::kSize,
                     result,
                     scratch1,
                     scratch2,
                     gc_required,
                     TAG_OBJECT);

  InitializeNewString(result,
                      length,
                      Heap::kSlicedAsciiStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::CompareObjectType(Register object,
                                       Register map,
                                       Register type_reg,
                                       InstanceType type,
                                       Condition cond) {
  ASSERT(!object.is(sh4_ip) && !map.is(sh4_ip) && !type_reg.is(sh4_ip));
  ASSERT(!object.is(sh4_rtmp) && !map.is(sh4_rtmp) && !type_reg.is(sh4_rtmp));

  RECORD_LINE();
  ldr(map, FieldMemOperand(object, HeapObject::kMapOffset));
  CompareInstanceType(map, type_reg, type, cond);
}


void MacroAssembler::CompareInstanceType(Register map,
                                         Register type_reg,
                                         InstanceType type,
                                         Condition cond) {
  ASSERT(!map.is(sh4_rtmp) && !type_reg.is(sh4_rtmp));

  RECORD_LINE();
  ldrb(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset));
  switch (cond) {
  case eq:
    RECORD_LINE();
    cmpeq(type_reg, Operand(type));
    break;
  case ge:
    RECORD_LINE();
    cmpge(type_reg, Operand(type));
    break;
  case hs:
    RECORD_LINE();
    cmphs(type_reg, Operand(type));
    break;
  case gt:
    RECORD_LINE();
    cmpgt(type_reg, Operand(type));
    break;
  default:
    UNIMPLEMENTED();
  }
}


void MacroAssembler::CompareRoot(Register obj,
                                 Heap::RootListIndex index) {
  ASSERT(!obj.is(sh4_ip));
  ASSERT(!obj.is(sh4_rtmp));
  RECORD_LINE();
  LoadRoot(sh4_ip, index);
  cmpeq(obj, sh4_ip);
}


void MacroAssembler::CheckFastElements(Register map,
                                       Register scratch,
                                       Label* fail) {
  STATIC_ASSERT(FAST_ELEMENTS == 0);
  ldrb(scratch, FieldMemOperand(map, Map::kBitField2Offset));
  cmphi(scratch, Operand(Map::kMaximumBitField2FastElementValue));
  bt(fail);
}


void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Handle<Map> map,
                              Label* fail,
                              bool is_heap_object) {
  ASSERT(!obj.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!obj.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  if (!is_heap_object) {
    RECORD_LINE();
    JumpIfSmi(obj, fail);
  }
  RECORD_LINE();
  ldr(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  mov(sh4_ip, Operand(map));
  cmp(scratch, sh4_ip);
  b(ne, fail);
}


void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Heap::RootListIndex index,
                              Label* fail,
                              bool is_heap_object) {
  ASSERT(!obj.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!obj.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  if (!is_heap_object) {
    RECORD_LINE();
    JumpIfSmi(obj, fail);
  }
  RECORD_LINE();
  ldr(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  LoadRoot(sh4_ip, index);
  cmp(scratch, sh4_ip);
  b(ne, fail);
}


void MacroAssembler::DispatchMap(Register obj,
                                 Register scratch,
                                 Handle<Map> map,
                                 Handle<Code> success,
                                 SmiCheckType smi_check_type) {
  Label fail;
  if (smi_check_type == DO_SMI_CHECK) {
    JumpIfSmi(obj, &fail, Label::kNear);
  }
  ldr(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  cmp(scratch, Operand(map));
  bf_near(&fail);
  jmp(success, RelocInfo::CODE_TARGET);
  bind(&fail);
}


void MacroAssembler::GetRelocatedValueLocation(Register ldr_location,
                               Register result) {
  UNIMPLEMENTED();
}


void MacroAssembler::LoadInstanceDescriptors(Register map,
                                             Register descriptors) {
  ldr(descriptors,
      FieldMemOperand(map, Map::kInstanceDescriptorsOrBitField3Offset));
  Label not_smi;
  JumpIfNotSmi(descriptors, &not_smi, Label::kNear);
  mov(descriptors, Operand(FACTORY->empty_descriptor_array()));
  bind(&not_smi);
}


CodePatcher::CodePatcher(byte* address, int instructions)
    : address_(address),
      instructions_(instructions),
      size_(instructions * Assembler::kInstrSize),
      masm_(Isolate::Current(), address, size_ + Assembler::kGap) {
  // Create a new macro assembler pointing to the address of the code to patch.
  // The size is adjusted with kGap on order for the assembler to generate size
  // bytes of instructions without failing with buffer size constraints.
  ASSERT(masm_.reloc_info_writer.pos() == address_ + size_ + Assembler::kGap);
}


CodePatcher::~CodePatcher() {
  // Indicate that code has changed.
  CPU::FlushICache(address_, size_);

  // Check that the code was patched as expected.
  ASSERT(masm_.pc_ == address_ + size_);
  ASSERT(masm_.reloc_info_writer.pos() == address_ + size_ + Assembler::kGap);
}


void CodePatcher::EmitCondition(Condition cond) {
  Instr instr = Assembler::instr_at(masm_.pc_);
  ASSERT(cond == eq || cond == ne);
  ASSERT(Assembler::IsBranch(instr));
  instr = (instr & ~0x200);     // Changed to bt
  if (cond == ne)
    instr |= 0x200;             // Changed to bf
  masm_.emit(instr);
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
