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
  tst(scratch, Immediate(1 << Map::kHasNonInstancePrototype));
  bf(&non_instance);

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
  bf(&done);

  RECORD_LINE();
  // Get the prototype from the initial map.
  mov(result, FieldMemOperand(result, Map::kPrototypeOffset));
  jmp(&done);

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

  // Allocate HandleScope in callee-save registers.
  mov(r7, Immediate(next_address));
  ldr(r4, MemOperand(r7, kNextOffset));
  ldr(r5, MemOperand(r7, kLimitOffset));
  ldr(r6, MemOperand(r7, kLevelOffset));
  add(r6, r6, Immediate(1));
  str(r6, MemOperand(r7, kLevelOffset));

  // Native call returns to the DirectCEntry stub which redirects to the
  // return address pushed on stack (could have moved after GC).
  // DirectCEntry stub itself is generated early and never moves.
  DirectCEntryStub stub;
  stub.GenerateCall(this, function);

  Label promote_scheduled_exception;
  Label delete_allocated_handles;
  Label leave_exit_frame;

  // If result is non-zero, dereference to get the result value
  // otherwise set it to undefined.
  Label ltrue, lfalse;
  cmp(r0, Immediate(0));
  bf(&lfalse);
  LoadRoot(r0, Heap::kUndefinedValueRootIndex);
  jmp(&ltrue);
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
  sub(r6, r6, Immediate(1));
  str(r6, MemOperand(r7, kLevelOffset));
  ldr(sh4_ip, MemOperand(r7, kLimitOffset));
  cmp(r5, sh4_ip);
  b(ne, &delete_allocated_handles);

  // Check if the function scheduled an exception.
  bind(&leave_exit_frame);
  LoadRoot(r4, Heap::kTheHoleValueRootIndex);
  mov(sh4_ip, Immediate(ExternalReference::scheduled_exception_address(isolate())));
  ldr(r5, MemOperand(sh4_ip));
  cmp(r4, r5);
  b(ne, &promote_scheduled_exception);

  // LeaveExitFrame expects unwind space to be in a register.
  mov(r4, Immediate(stack_space));
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
  str(r5, MemOperand(r7, kLimitOffset));
  mov(r4, r0);
  PrepareCallCFunction(1, r5);
  mov(r0, Operand(ExternalReference::isolate_address()));
  CallCFunction(
      ExternalReference::delete_handle_scope_extensions(isolate()), 1);
  mov(r0, r4);
  jmp(&leave_exit_frame);

  return result;
}


void MacroAssembler::IllegalOperation(int num_arguments) {
  RECORD_LINE();
  if (num_arguments > 0) {
    add(sp, sp, Immediate(num_arguments * kPointerSize));
  }
  LoadRoot(r0, Heap::kUndefinedValueRootIndex);
}


// Tries to get a signed int32 out of a double precision floating point heap
// number. Rounds towards 0. Branch to 'not_int32' if the double is out of the
// 32bits signed integer range.
void MacroAssembler::ConvertToInt32(Register source,
                                    Register dest,
                                    Register scratch,
                                    Register scratch2,
                                    Register double_scratch,
                                    Label *not_int32) {

  ASSERT(!source.is(sh4_ip) && !dest.is(sh4_ip) && !scratch.is(sh4_ip) &&
         !scratch2.is(sh4_ip) && !double_scratch.is(sh4_ip));
  ASSERT(!source.is(dest));

  // TODO: SH4, check VFP code
  // if (CpuFeatures::IsSupported(VFP3)) {
  //   CpuFeatures::Scope scope(VFP3);
  //   sub(scratch, source, Operand(kHeapObjectTag));
  //   vldr(double_scratch, scratch, HeapNumber::kValueOffset);
  //   vcvt_s32_f64(double_scratch.low(), double_scratch);
  //   vmov(dest, double_scratch.low());
  //   // Signed vcvt instruction will saturate to the minimum (0x80000000) or
  //   // maximun (0x7fffffff) signed 32bits integer when the double is out of
  //   // range. When substracting one, the minimum signed integer becomes the
  //   // maximun signed integer.
  //   sub(scratch, dest, Operand(1));
  //   cmp(scratch, Operand(LONG_MAX - 1));
  //   // If equal then dest was LONG_MAX, if greater dest was LONG_MIN.
  //   b(ge, not_int32);
  // } else 
  {
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
    mov(dest, Immediate(0));
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
    sub(scratch2, scratch2, Immediate(fudge_factor));
    cmp(scratch2, Immediate(non_smi_exponent - fudge_factor));
    // If we have a match of the int32-but-not-Smi exponent then skip some
    // logic.
    b(eq, &right_exponent);
    // If the exponent is higher than that then go to slow case.  This catches
    // numbers that don't fit in a signed int32, infinities and NaNs.
    cmpgt(scratch2, Immediate(non_smi_exponent - fudge_factor));
    bt(not_int32);

    // We know the exponent is smaller than 30 (biased).  If it is less than
    // 0 (biased) then the number is smaller in magnitude than 1.0 * 2^0, ie
    // it rounds to zero.
    const uint32_t zero_exponent = HeapNumber::kExponentBias + 0;
    sub(scratch2, scratch2, Immediate(zero_exponent - fudge_factor));
    cmpge(scratch2, Immediate(0));
    // Dest already has a Smi zero.
    bf(&done);

    // We have an exponent between 0 and 30 in scratch2.  Subtract from 30 to
    // get how much to shift down.
    rsb(dest, scratch2, Immediate(30));

    bind(&right_exponent);
    // Get the top bits of the mantissa.
    land(scratch2, scratch, Immediate(HeapNumber::kMantissaMask));
    // Put back the implicit 1.
    lor(scratch2, scratch2, Immediate(1 << HeapNumber::kExponentShift));
    // Shift up the mantissa bits to take up the space the exponent used to
    // take. We just orred in the implicit bit so that took care of one and
    // we want to leave the sign bit 0 so we subtract 2 bits from the shift
    // distance.
    const int shift_distance = HeapNumber::kNonMantissaBitsInTopWord - 2;
    lsl(scratch2, scratch2, Immediate(shift_distance));
    // Put sign in zero flag.
    tst(scratch, Immediate(HeapNumber::kSignMask));
    // Get the second half of the double. For some exponents we don't
    // actually need this because the bits get shifted out again, but
    // it's probably slower to test than just to do it.
    ldr(scratch, FieldMemOperand(source, HeapNumber::kMantissaOffset));
    // Shift down 22 bits to get the last 10 bits.
    lsr(scratch, scratch, Immediate(32 - shift_distance));
    lor(scratch, scratch2, scratch);
    // Move down according to the exponent.
    lsr(dest, scratch, dest);
    // Fix sign if sign bit was set.
    b(eq, &done);
    rsb(dest, dest, Immediate(0));
    bind(&done);
  }
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
  RECORD_LINE();
  Ubfx(hash, hash, String::kHashShift, String::kArrayIndexValueBits);
  lsl(index, hash, Immediate(kSmiTagSize));
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

void MacroAssembler::Call(
    intptr_t target, RelocInfo::Mode rmode) {
  //TODO: check whether this is necessaery
  //   // Block constant pool for the call instruction sequence.
  //   BlockConstPoolScope block_const_pool(this);
  // #ifdef DEBUG
  //   int pre_position = pc_offset();
  // #endif

  RECORD_LINE();

  align(); // Force alignment such that we can check kCallTargetAddressOffset
#ifdef DEBUG
  int start_position = pc_offset();
#endif

  // TODO: check whether this is necessary
  // Statement positions are expected to be recorded when the target
  // address is loaded. The mov method will automatically record
  // positions when pc is the target, since this is not the case here
  // we have to do it explicitly.
  positions_recorder()->WriteRecordedPositions();

  mov(sh4_ip, Operand(target, rmode));
  jsr(sh4_ip);

  // The offset from the start of the call sequence to the address after
  // the jsr/nop. Ref to assembler-sh4.h for kCallTargetAddressOffset.
  ASSERT(kCallTargetAddressOffset == pc_offset() - start_position);


  //TODO: check whether this is necessaery
  // #ifdef DEBUG
  //   int post_position = pc_offset();
  //   CHECK_EQ(pre_position + CallSize(target, rmode, cond), post_position);
  // #endif
}


void MacroAssembler::Call(
    Handle<Code> code, RelocInfo::Mode rmode) {
  //TODO: check whether this is necessaery
  // #ifdef DEBUG
  //   int pre_position = pc_offset();
  // #endif

  ASSERT(RelocInfo::IsCodeTarget(rmode));
  RECORD_LINE();
  Call(reinterpret_cast<intptr_t>(code.location()), rmode);

  //TODO: check whether this is necessaery
  // #ifdef DEBUG
  //   int post_position = pc_offset();
  //   CHECK_EQ(pre_position + CallSize(code, rmode, cond), post_position);
  // #endif
}


void MacroAssembler::GetLeastBitsFromSmi(Register dst,
                                         Register src,
                                         int num_least_bits) {
  ASSERT(!dst.is(sh4_ip) && !src.is(sh4_ip));
  Ubfx(dst, src, kSmiTagSize, num_least_bits);
}


void MacroAssembler::GetLeastBitsFromInt32(Register dst,
                                           Register src,
                                           int num_least_bits) {
  ASSERT(!dst.is(sh4_ip) && !src.is(sh4_ip));
  land(dst, src, Immediate((1 << num_least_bits) - 1));
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
  mov(r0, Immediate(num_arguments));
  mov(r1, Immediate(ExternalReference(f, isolate())));
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

  ldr(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
  ldrb(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));
  tst(scratch, Immediate(kIsNotStringMask));
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
    add(sp, sp, Immediate(stack_elements * kPointerSize));
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
  while(*base) {
    file_id += *base;
    base++;
  }
  RECORD_LINE();
  mov(r0, Immediate(file_id));
  mov(r1, Immediate(line));
  bkpt();
}

void MacroAssembler::EnterFrame(StackFrame::Type type) {
  // r0-r3: must be preserved
  RECORD_LINE();
  Push(pr, fp, cp);
  mov(sh4_ip, Immediate(Smi::FromInt(type)));
  push(sh4_ip);
  mov(sh4_ip, Operand(CodeObject()));
  push(sh4_ip);
  add(fp, sp, Immediate(3 * kPointerSize));  // Adjust FP to point to saved FP.
}


void MacroAssembler::LeaveFrame(StackFrame::Type type) {
  // r0, r1, r2: must be preserved

  // Drop the execution stack down to the frame pointer and restore
  // the caller frame pointer and return address.
  RECORD_LINE();
  mov(sp, fp);
  Pop(pr, fp);
}


void MacroAssembler::EnterExitFrame(bool save_doubles, int stack_space) {
  // Parameters are on stack as if calling JS function
  // ARM -> ST40 mapping: ip -> r2

  // r0, r1, cp: must be preserved
  // sp, fp: input/output
  // Actual clobbers: r2

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
  sub(sp, sp, Immediate(2*kPointerSize));
  if (emit_debug_code()) {
    mov(r2, Immediate(0));
    mov(MemOperand(fp, ExitFrameConstants::kSPOffset), r2);
  }

  mov(r2, Operand(CodeObject()));
  mov(MemOperand(fp, ExitFrameConstants::kCodeOffset), r2);

  // Save the frame pointer and the context in top.
  mov(r2, Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate())));
  mov(MemOperand(r2), fp);
  mov(r2, Operand(ExternalReference(Isolate::k_context_address, isolate())));
  mov(MemOperand(r2), cp);

  // Optionally save all double registers.
  if (save_doubles) {
    RECORD_LINE();
    UNIMPLEMENTED_BREAK();
  }

  // Reserve place for the return address and stack space and align the frame
  // preparing for calling the runtime function.
  const int frame_alignment = OS::ActivationFrameAlignment();
  sub(sp, sp, Immediate((stack_space + 1) * kPointerSize));
  if (frame_alignment) {
    ASSERT(IsPowerOf2(frame_alignment));
    land(sp, sp, Immediate(-frame_alignment));
  }

  // Set the exit frame sp value to point just before the return address
  // location.
  add(r2, sp, Immediate(kPointerSize));
  mov(MemOperand(fp, ExitFrameConstants::kSPOffset), r2);
}


void MacroAssembler::LeaveExitFrame(bool save_doubles,
                                    Register argument_count) {
  ASSERT(!argument_count.is(sh4_ip));
  // input: argument_count
  // r0, r1: results must be preserved
  // sp: stack pointer
  // fp: frame pointer

  // Actual clobbers: r2, r3

  RECORD_LINE();
  if (save_doubles) {
    RECORD_LINE();
    UNIMPLEMENTED_BREAK();
  }

  // Clear top frame.
  mov(r2, Immediate(0));
  mov(r3, Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate())));
  str(r2, MemOperand(r3));
  
  // Restore current context from top and clear it in debug mode.
  mov(r2, Operand(ExternalReference(Isolate::k_context_address, isolate())));
  ldr(cp, MemOperand(r2));
  
  // Tear down the exit frame, pop the arguments, and return.
  mov(sp, fp);
      
  Pop(pr, fp);
  if (argument_count.is_valid()) {
    ASSERT(!argument_count.is(r2));
    ASSERT(!argument_count.is(r3)); 
    lsl(r2, argument_count, Immediate(kPointerSizeLog2));
    add(sp, sp, r2);
  }
}


void MacroAssembler::InvokePrologue(const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    Handle<Code> code_constant,
                                    Register code_reg,
                                    Label* done,
                                    InvokeFlag flag,
                                    CallWrapper* call_wrapper) {
  ASSERT(!code_reg.is(sh4_ip));
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
      mov(r0, Immediate(actual.immediate()));
      const int sentinel = SharedFunctionInfo::kDontAdaptArgumentsSentinel;
      if (expected.immediate() == sentinel) {
        // Don't worry about adapting arguments for builtins that
        // don't want that done. Skip adaption code by making it look
        // like we have a match between expected and actual number of
        // arguments.
        definitely_matches = true;
      } else {
        mov(r2, Immediate(expected.immediate()));
      }
    }
  }
 else {
    if (actual.is_immediate()) {
      cmpeq(expected.reg(), Immediate((actual.immediate())));
      bt(&regular_invoke);
      mov(r0, Immediate(actual.immediate()));
    } else {
      cmpeq(expected.reg(), actual.reg());
      bt(&regular_invoke);
    }
  }

  RECORD_LINE();
  if (!definitely_matches) {
    if (!code_constant.is_null()) {
      mov(r3, Operand(code_constant));
      add(r3, r3, Immediate(Code::kHeaderSize - kHeapObjectTag));
    }
    Handle<Code> adaptor =
        isolate()->builtins()->ArgumentsAdaptorTrampoline();
    if (flag == CALL_FUNCTION) {
      if (call_wrapper != NULL) call_wrapper->BeforeCall(2 * kInstrSize);
      Call(adaptor, RelocInfo::CODE_TARGET);
      if (call_wrapper != NULL) call_wrapper->AfterCall();
      b(done);
    } else {
      Jump(adaptor, RelocInfo::CODE_TARGET);
    }
    bind(&regular_invoke);
  }
}


void MacroAssembler::InvokeCode(Register code,
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                InvokeFlag flag,
                                CallWrapper* call_wrapper) {
  Label done;
  // r1: must hold function pointer
  // actual: must be r0 if register
  ASSERT(actual.is_immediate() || actual.reg().is(r0));
  ASSERT(!code.is(sh4_ip));

  RECORD_LINE();
  InvokePrologue(expected, actual, Handle<Code>::null(), code, &done, flag,
                 call_wrapper);
  RECORD_LINE();
  if (flag == CALL_FUNCTION) {
    if (call_wrapper != NULL) call_wrapper->BeforeCall(2 * kInstrSize);
    jsr(code);
    if (call_wrapper != NULL) call_wrapper->AfterCall();
  } else {
    ASSERT(flag == JUMP_FUNCTION);
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
                                InvokeFlag flag) {
  Label done;

  InvokePrologue(expected, actual, code, no_reg, &done, flag);
  if (flag == CALL_FUNCTION) {
    Call(code, rmode);
  } else {
    Jump(code, rmode);
  }

  // Continue here if InvokePrologue does handle the invocation due to
  // mismatched parameter counts.
  bind(&done);
}


void MacroAssembler::InvokeFunction(Register fun,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    CallWrapper* call_wrapper) {
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
  asr(expected_reg, expected_reg, Immediate(kSmiTagSize));
  ldr(code_reg,
      FieldMemOperand(r1, JSFunction::kCodeEntryOffset));

  ParameterCount expected(expected_reg);
  InvokeCode(code_reg, expected, actual, flag, call_wrapper);
}


void MacroAssembler::InvokeFunction(JSFunction* function,
                                    const ParameterCount& actual,
                                    InvokeFlag flag) {
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
    InvokeCode(r3, expected, actual, flag);
  } else {
    InvokeCode(code, expected, actual, RelocInfo::CODE_TARGET, flag);
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
  cmpge(scratch, Immediate(FIRST_JS_OBJECT_TYPE));
  bf(fail);
  cmpgt(scratch, Immediate(LAST_JS_OBJECT_TYPE));
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
  mov(dst, Immediate(value));
}


void MacroAssembler::Move(Register dst, Register src) {
  if (!dst.is(src)) {
    RECORD_LINE();
    mov(dst, src);
  }
}


void MacroAssembler::Sbfx(Register dst, Register src1, int lsb, int width) {
  ASSERT(lsb >= 0 && lsb < 32);
  ASSERT(width > 0 && width <= 32);
  ASSERT(width + lsb <= 32);
  int32_t mask1 = width < 32 ? (1<<width)-1 : -1;
  int32_t mask = mask1 << lsb;
  land(dst, src1, Immediate(mask));
  int shift_up = 32 - lsb - width;
  int shift_down = lsb + shift_up;
  if (shift_up != 0) {
    lsl(dst, dst, Immediate(shift_up));
  }
  if (shift_down != 0) {
    asr(dst, dst, Immediate(shift_down));
  }
}


void MacroAssembler::PopTryHandler() {
  RECORD_LINE();
  ASSERT_EQ(0, StackHandlerConstants::kNextOffset);
  pop(r1);
  mov(sh4_ip, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  add(sp, sp, Immediate(StackHandlerConstants::kSize - kPointerSize));
  str(r1, MemOperand(sh4_ip));
}


static const int kRegisterPassedArguments = 4;

void MacroAssembler::PrepareCallCFunction(int num_arguments, Register scratch) {
  ASSERT(!scratch.is(sh4_ip));
  int frame_alignment = OS::ActivationFrameAlignment();

  // Up to four simple arguments are passed in registers r4..r7.
  int stack_passed_arguments = (num_arguments <= kRegisterPassedArguments) ?
                               0 : num_arguments - kRegisterPassedArguments;

  if (frame_alignment > kPointerSize) {
    RECORD_LINE();
    mov(scratch, sp);
    sub(sp, sp, Immediate((stack_passed_arguments + 1) * kPointerSize));
    ASSERT(IsPowerOf2(frame_alignment));
    land(sp, sp, Immediate(-frame_alignment));
    mov(MemOperand(sp, stack_passed_arguments * kPointerSize), scratch);
  } else {
    RECORD_LINE();
    sub(sp, sp, Immediate(stack_passed_arguments * kPointerSize));
  }
}


void MacroAssembler::CallCFunction(ExternalReference function,
                                   int num_arguments) {
  RECORD_LINE();
  CallCFunctionHelper(no_reg, function, r3, num_arguments);
}


void MacroAssembler::CallCFunctionHelper(Register function,
                                         ExternalReference function_reference,
                                         Register scratch,
                                         int num_arguments) {
  ASSERT(!function.is(sh4_ip));
  // Make sure that the stack is aligned before calling a C function unless
  // running in the simulator. The simulator has its own alignment check which
  // provides more information.
#if defined(V8_HOST_ARCH_SH4)
  if (emit_debug_code()) {
    RECORD_LINE();
    int frame_alignment = OS::ActivationFrameAlignment();
    int frame_alignment_mask = frame_alignment - 1;
    if (frame_alignment > kPointerSize) {
      ASSERT(IsPowerOf2(frame_alignment));
      Label alignment_as_expected;
      tst(sp, Immediate(frame_alignment_mask));
      b(eq, &alignment_as_expected);
      // Don't use Check here, as it will call Runtime_Abort possibly
      // re-entering here.
      stop("Unexpected alignment");
      bind(&alignment_as_expected);
    }
  }
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

  int stack_passed_arguments = (num_arguments <= kRegisterPassedArguments) ?
                               0 : num_arguments - kRegisterPassedArguments;
  if (OS::ActivationFrameAlignment() > kPointerSize) {
    mov(sp, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    add(sp, sp, Immediate(stack_passed_arguments * sizeof(kPointerSize)));
  }
}


void MacroAssembler::PushTryHandler(CodeLocation try_location,
                                    HandlerType type) {
  if (try_location == IN_JAVASCRIPT) {
    RECORD_LINE();
    if (type == TRY_CATCH_HANDLER) {
      mov(r3, Immediate(StackHandler::TRY_CATCH));
    } else {
      mov(r3, Immediate(StackHandler::TRY_FINALLY));
    }
    ASSERT(StackHandlerConstants::kStateOffset == 1 * kPointerSize
           && StackHandlerConstants::kFPOffset == 2 * kPointerSize
           && StackHandlerConstants::kPCOffset == 3 * kPointerSize);
    Push(pr, fp, r3);
    // Save the current handler as the next handler.
    mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
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
    ASSERT(StackHandlerConstants::kStateOffset == 1 * kPointerSize
           && StackHandlerConstants::kFPOffset == 2 * kPointerSize
           && StackHandlerConstants::kPCOffset == 3 * kPointerSize);
    push(pr);
    push(Immediate(0)); // Null FP
    push(Immediate(StackHandler::ENTRY));
    // Save the current handler as the next handler.
    mov(r7, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
    mov(r6, MemOperand(r7));
    ASSERT(StackHandlerConstants::kNextOffset == 0);
    push(r6);
    // Link this handler as the new current one.
    str(sp, MemOperand(r7));
  }
}


void MacroAssembler::Throw(Register value) {
  ASSERT(!value.is(sh4_ip));
  // r0 is expected to hold the exception.
  if (!value.is(r0)) {
    RECORD_LINE();
    mov(r0, value);
  }

  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 4 * kPointerSize);

  RECORD_LINE();
  // Drop the sp to the top of the handler.
  mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  ldr(sp, MemOperand(r3));

  // Restore the next handler and frame pointer, discard handler state.
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  pop(r2);
  str(r2, MemOperand(r3));
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 2 * kPointerSize);
  pop(r3);
  pop(fp);

  // Before returning we restore the context from the frame pointer if
  // not NULL.  The frame pointer is NULL in the exception handler of a
  // JS entry frame.
  Label restore, restore_end;
  cmpeq(fp, Immediate(0));
  bf(&restore);
  RECORD_LINE();
  // Set cp to NULL if fp is NULL.
  mov(cp, Immediate(0));
  jmp(&restore_end);
  bind(&restore);
  RECORD_LINE();
  // Restore cp otherwise.
  ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  bind(&restore_end);
  RECORD_LINE();
  STATIC_ASSERT(StackHandlerConstants::kPCOffset == 3 * kPointerSize);
  pop(pr);
  rts();
}


void MacroAssembler::ThrowUncatchable(UncatchableExceptionType type,
                                      Register value) {
  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 4 * kPointerSize);
  ASSERT(!value.is(sh4_ip));

  // r0 is expected to hold the exception.
  if (!value.is(r0)) {
    RECORD_LINE();
    mov(r0, value);
  }

  RECORD_LINE();
  // Drop sp to the top stack handler.
  mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  ldr(sp, MemOperand(r3));

  // Unwind the handlers until the ENTRY handler is found.
  Label loop, done;
  bind(&loop);
  RECORD_LINE();
  // Load the type of the current stack handler.
  const int kStateOffset = StackHandlerConstants::kStateOffset;
  ldr(r2, MemOperand(sp, kStateOffset));
  mov(r3, Operand(StackHandler::ENTRY));
  cmpeq(r2, r3);
  bt(&done);
  RECORD_LINE();
  // Fetch the next handler in the list.
  const int kNextOffset = StackHandlerConstants::kNextOffset;
  mov(sp, MemOperand(sp, kNextOffset));
  jmp(&loop);
  bind(&done);
  RECORD_LINE();

  // Set the top handler address to next handler past the current ENTRY handler.
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  pop(r2);
  mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  str(r2, MemOperand(r3));

  if (type == OUT_OF_MEMORY) {
    // Set external caught exception to false.
    ExternalReference external_caught(
        Isolate::k_external_caught_exception_address, isolate());
    RECORD_LINE();
    mov(r0, Immediate(false));
    mov(r2, Operand(external_caught));
    str(r0, MemOperand(r2));

    // Set pending exception and r0 to out of memory exception.
    Failure* out_of_memory = Failure::OutOfMemoryException();
    mov(r0, Immediate(reinterpret_cast<int32_t>(out_of_memory)));
    mov(r2, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                      isolate())));
    str(r0, MemOperand(r2));
  }

  // Stack layout at this point. See also StackHandlerConstants.
  // sp ->   state (ENTRY)
  //         fp
  //         pr

  RECORD_LINE();
  // Discard handler state (r3 is not used) and restore frame pointer.
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 2 * kPointerSize);
  pop(r2);
  pop(fp);
  // Before returning we restore the context from the frame pointer if
  // not NULL.  The frame pointer is NULL in the exception handler of a
  // JS entry frame.
  Label restore, restore_end;
  cmpeq(fp, Immediate(0));
  bf(&restore);
  RECORD_LINE();
  // Set cp to NULL if fp is NULL.
  mov(cp, Immediate(0));
  jmp(&restore_end);
  bind(&restore);
  RECORD_LINE();
  // Restore cp otherwise.
  ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  bind(&restore_end);
  RECORD_LINE();
  STATIC_ASSERT(StackHandlerConstants::kPCOffset == 3 * kPointerSize);
  pop(pr);
  rts();
}

#include "map-sh4.h" // Define register map
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
  cmp(scratch, Immediate(0));
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
#include "map-sh4.h" // Undefine register map


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
                                   InvokeJSFlags flags,
                                   CallWrapper* call_wrapper) {
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
  if (flags == CALL_JS) {
    RECORD_LINE();
    if (call_wrapper != NULL) call_wrapper->BeforeCall(2 * kInstrSize);
    jsr(r2);
    if (call_wrapper != NULL) call_wrapper->AfterCall();
  } else {
    ASSERT(flags == JUMP_JS);
    RECORD_LINE();
    jmp(r2);
  }
}


void MacroAssembler::GetBuiltinFunction(Register target,
                                        Builtins::JavaScript id) {
  ASSERT(!target.is(sh4_ip));
  RECORD_LINE();
  // Load the builtins object into target register.
  ldr(target, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  ldr(target, FieldMemOperand(target, GlobalObject::kBuiltinsOffset));
  // Load the JavaScript builtin function from the builtins object.
  ldr(target, FieldMemOperand(target,
                          JSBuiltinsObject::OffsetOfFunctionWithId(id)));
}


void MacroAssembler::GetBuiltinEntry(Register target, Builtins::JavaScript id) {
  //FIXME: why r1 ??
  ASSERT(!target.is(r1) && !target.is(sh4_ip));
  RECORD_LINE();
  GetBuiltinFunction(r1, id);
  RECORD_LINE();
  // Load the code entry point from the builtins object.
  ldr(target, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
}


void MacroAssembler::SetCounter(StatsCounter* counter, int value,
                                Register scratch1, Register scratch2) {
  RECORD_LINE();
  ASSERT(!scratch1.is(scratch2) && !scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch1, Immediate(value));
    mov(scratch2, Operand(ExternalReference(counter)));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::IncrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  ASSERT(!scratch1.is(scratch2) && !scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch2, Operand(ExternalReference(counter)));
    ldr(scratch1, MemOperand(scratch2));
    add(scratch1, scratch1, Immediate(value));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::DecrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  ASSERT(!scratch1.is(scratch2) && !scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) { 
    RECORD_LINE();
    mov(scratch2, Operand(ExternalReference(counter)));
    ldr(scratch1, MemOperand(scratch2));
    sub(scratch1, scratch1, Immediate(value));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::Assert(Condition cond, const char* msg) {
  if (emit_debug_code())
    Check(cond, msg);
}


void MacroAssembler::AssertRegisterIsRoot(Register reg,
                                          Heap::RootListIndex index) {
  ASSERT(!reg.is(r3));
  if (emit_debug_code()) {
    LoadRoot(r3, index);
    cmp(reg, r3);
    Check(eq, "Register did not match expected root");
  }
}


void MacroAssembler::AssertFastElements(Register elements) {
  RECORD_LINE();
  if (emit_debug_code()) {
    ASSERT(!elements.is(sh4_ip));
    Label ok;
    RECORD_LINE();
    push(elements);
    ldr(elements, FieldMemOperand(elements, HeapObject::kMapOffset));
    LoadRoot(sh4_ip, Heap::kFixedArrayMapRootIndex);
    cmp(elements, sh4_ip);
    b(eq,&ok);
    RECORD_LINE();
    LoadRoot(sh4_ip, Heap::kFixedCOWArrayMapRootIndex);
    cmp(elements, sh4_ip);
    b(eq,&ok);
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
  mov(r0, Immediate(p0));
  push(r0);
  mov(r0, Immediate(Smi::FromInt(p1 - p0)));
  push(r0);
  CallRuntime(Runtime::kAbort, 2);
  // will not return here
  // TODO: implement this when const pool manager is active
  //if (is_const_pool_blocked()) {
  //  // If the calling code cares about the exact number of
  //  // instructions generated, we insert padding here to keep the size
  //  // of the Abort macro constant.
  //  static const int kExpectedAbortInstructions = 10;
  //  int abort_instructions = InstructionsGeneratedSince(&abort_start);
  //  ASSERT(abort_instructions <= kExpectedAbortInstructions);
  //  while (abort_instructions++ < kExpectedAbortInstructions) {
  //    nop();
  //  }
  //}
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
      mov(result, Immediate(0x7091));
      mov(scratch1, Immediate(0x7191));
      mov(scratch2, Immediate(0x7291));
    }
    RECORD_LINE();
    jmp(gc_required);
    return;
  }

  // Assert that the register arguments are different and that none of
  // them are ip. ip is used explicitly in the code generated below.
  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!result.is(sh4_ip));
  ASSERT(!scratch1.is(sh4_ip));
  ASSERT(!scratch2.is(sh4_ip));

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
  mov(topaddr, Immediate(new_space_allocation_top));
  mov(obj_size_reg, Immediate(object_size));

  // This code stores a temporary value in sh4_ip (ARM:ip). This is OK, as the code below
  // does not need sh4_ip (ARM:ip) for implicit literal generation.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    RECORD_LINE();
    // Load allocation top into result and allocation limit into sh4_ip (ARM:ip).
    ldr(result, MemOperand(topaddr));
    ldr(sh4_ip, MemOperand(topaddr, 4));
  } else {
    if (emit_debug_code()) {
      RECORD_LINE();
      // Assert that result actually contains top on entry. sh4_ip (ARM:ip) is used
      // immediately below so this use of ip does not cause difference with
      // respect to register content between debug and release mode.
      ldr(sh4_ip, MemOperand(topaddr));
      cmp(result, sh4_ip);
      Check(eq, "Unexpected allocation top");
    }
    RECORD_LINE();
    // Load allocation limit into sh4_ip (ARM: ip). Result already contains allocation top.
    ldr(sh4_ip, MemOperand(topaddr, limit - top));
  }

  RECORD_LINE();
  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top.
  addc(scratch2, result, obj_size_reg);
  b(cs, gc_required);

  RECORD_LINE();
  cmpgtu(scratch2, sh4_ip);
  bt(gc_required);

  RECORD_LINE();
  str(scratch2, MemOperand(topaddr));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    RECORD_LINE();
    add(result, result, Immediate(kHeapObjectTag));
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
      mov(result, Immediate(0x7091));
      mov(scratch1, Immediate(0x7191));
      mov(scratch2, Immediate(0x7291));
    }
    RECORD_LINE();
    jmp(gc_required);
    return;
  }

  // Assert that the register arguments are different and that none of
  // them are ip. ip is used explicitly in the code generated below.
  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!result.is(sh4_ip));
  ASSERT(!scratch1.is(sh4_ip));
  ASSERT(!scratch2.is(sh4_ip));

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

  // This code stores a temporary value in sh4_ip (ARM:ip). This is OK, as the code below
  // does not need sh4_ip (ARM:ip) for implicit literal generation.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    RECORD_LINE();
    // Load allocation top into result and allocation limit into sh4_ip (ARM:ip).
    ldr(result, MemOperand(topaddr));
    ldr(sh4_ip, MemOperand(topaddr, 4));
  } else {
    if (emit_debug_code()) {
      RECORD_LINE();
      // Assert that result actually contains top on entry. sh4_ip (ARM:ip) is used
      // immediately below so this use of ip does not cause difference with
      // respect to register content between debug and release mode.
      ldr(sh4_ip, MemOperand(topaddr));
      cmp(result, sh4_ip);
      Check(eq, "Unexpected allocation top");
    }
    RECORD_LINE();
    // Load allocation limit into sh4_ip (ARM: ip). Result already contains allocation top.
    ldr(sh4_ip, MemOperand(topaddr, limit - top));
  }

  RECORD_LINE();
  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top. Object size may be in words so a shift is
  // required to get the number of bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    RECORD_LINE();
    lsl(scratch2, object_size, Immediate(kPointerSizeLog2));
    addc(scratch2, result, scratch2);
  } else {
    RECORD_LINE();
    addc(scratch2, result, object_size);
  }
  RECORD_LINE();
  b(cs,gc_required);
  RECORD_LINE();
  cmpgtu(scratch2, sh4_ip);
  bt(gc_required);
  RECORD_LINE();

  // Update allocation top. result temporarily holds the new top.
  if (emit_debug_code()) {
    RECORD_LINE();
    tst(scratch2, Immediate(kObjectAlignmentMask));
    Check(eq, "Unaligned allocation in new space");
  }
  RECORD_LINE();
  str(scratch2, MemOperand(topaddr));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    RECORD_LINE();
    add(result, result, Immediate(kHeapObjectTag));
  }
}

 
void MacroAssembler::InitializeNewString(Register string,
                                         Register length,
                                         Heap::RootListIndex map_index,
                                         Register scratch1,
                                         Register scratch2) {
  RECORD_LINE();
  lsl(scratch1, length, Immediate(kSmiTagSize));
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
  land(object, object, Immediate(~kHeapObjectTagMask));
#ifdef DEBUG
  // Check that the object un-allocated is below the current top.
  mov(scratch, Immediate(new_space_allocation_top));
  ldr(scratch, MemOperand(scratch));
  cmpge(object, scratch);
  Check(ne, "Undo allocation of non allocated memory");
#endif
  // Write the address of the object to un-allocate as the current top.
  mov(scratch, Immediate(new_space_allocation_top));
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
  lsl(scratch1, length, Immediate(1));  // Length in bytes, not chars.
  add(scratch1, scratch1,
      Immediate(kObjectAlignmentMask + SeqTwoByteString::kHeaderSize));
  land(scratch1, scratch1, Immediate(~kObjectAlignmentMask));

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
      Immediate(kObjectAlignmentMask + SeqAsciiString::kHeaderSize));
  land(scratch1, scratch1, Immediate(~kObjectAlignmentMask));

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
  AssertRegisterIsRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);
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
  cmp(length, Immediate(0));
  b(eq, &done);
  bind(&align_loop_1);
  tst(src, Immediate(kPointerSize - 1));
  b(eq, &word_loop);
  ldrb(scratch, MemOperand(src)); add(src, src, Immediate(1));
  strb(scratch, MemOperand(dst)); add(dst, dst, Immediate(1));
  sub(length, length, Immediate(1));
  tst(length, length);
  b(ne, &byte_loop_1);

  // Copy bytes in word size chunks.
  bind(&word_loop);
  if (emit_debug_code()) {
    tst(src, Immediate(kPointerSize - 1));
    Assert(eq, "Expecting alignment for CopyBytes");
  }
  cmpge(length, Immediate(kPointerSize));
  bf(&byte_loop);
  ldr(scratch, MemOperand(src)); add(src, src, Immediate(kPointerSize));
#if CAN_USE_UNALIGNED_ACCESSES
  str(scratch, MemOperand(dst)); add(dst, dst, Immediate(kPointerSize));
#else
  strb(scratch, MemOperand(dst)); add(dst, dst, Immediate(1));
  lsr(scratch, scratch, Immediate(8));
  strb(scratch, MemOperand(dst)); add(dst, dst, Immediate(1));
  lsr(scratch, scratch, Immediate(8));
  strb(scratch, MemOperand(dst)); add(dst, dst, Immediate(1));
  lsr(scratch, scratch, Immediate(8));
  strb(scratch, MemOperand(dst)); add(dst, dst, Immediate(1));
#endif
  sub(length, length, Immediate(kPointerSize));
  b(&word_loop);

  // Copy the last bytes if any left.
  bind(&byte_loop);
  cmp(length, Immediate(0));
  b(eq, &done);
  bind(&byte_loop_1);
  ldrb(scratch, MemOperand(src)); add(src, src, Immediate(1));
  strb(scratch, MemOperand(dst)); add(dst, dst, Immediate(1));
  sub(length, length, Immediate(1));
  tst(length, length);
  b(ne, &byte_loop_1);
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
  RECORD_LINE();

  Label l0, l1, l2, l3, l4, l5;
  cmpeq(source, Immediate(0));
  bf(&l0);
  mov(zeros, Immediate(32));
  jmp(&l5);

  bind(&l0);
  mov(zeros, Immediate(0));
  mov(scratch, source);
  // Top 16.
  tst(scratch, Immediate(0xffff0000));
  bf(&l1);
  add(zeros, zeros, Immediate(16));
  lsl(scratch, scratch, Immediate(16));
  // Top 8.
  bind(&l1);
  tst(scratch, Immediate(0xff000000));
  bf(&l2);
  add(zeros, zeros, Immediate(8));
  lsl(scratch, scratch, Immediate(8));
  // Top 4.
  bind(&l2);
  tst(scratch, Immediate(0xf0000000));
  bf(&l3);
  add(zeros, zeros, Immediate(4));
  lsl(scratch, scratch, Immediate(4));
  // Top 2.
  bind(&l3);
  tst(scratch, Immediate(0xc0000000));
  bf(&l4);
  add(zeros, zeros, Immediate(2));
  lsl(scratch, scratch, Immediate(2));
  // Top bit.
  bind(&l4);
  tst(scratch, Immediate(0x80000000u));
  bf(&l5);
  add(zeros, zeros, Immediate(1));
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
    RECORD_LINE();
    Label skip;
    if (cond == eq) bf(&skip);
    else bt(&skip);
    rts();
    bind(&skip);
  }
}


void MacroAssembler::JumpToExternalReference(const ExternalReference& builtin) {
  RECORD_LINE();
  mov(r1, Immediate(builtin));
  CEntryStub stub(1);
  RECORD_LINE();
  jmp(stub.GetCode(), RelocInfo::CODE_TARGET);
}


MaybeObject* MacroAssembler::TryJumpToExternalReference(
    const ExternalReference& builtin) {
  mov(r1, Immediate(builtin));
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
  RECORD_LINE();
  mov(r0, Immediate(num_arguments));
  JumpToExternalReference(ext);
}


MaybeObject* MacroAssembler::TryTailCallExternalReference(
    const ExternalReference& ext, int num_arguments, int result_size) {
  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.
  RECORD_LINE();
  mov(r0, Immediate(num_arguments));
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
  ASSERT(!dst.is(sh4_ip) && !src.is(sh4_ip));
  ASSERT(lsb >= 0 && lsb < 32);
  ASSERT(width > 0 && width <= 32);
  ASSERT(width + lsb <= 32);
  // Extract unsigned value from bits src1[lsb..lsb+width-1] into dst
  int32_t mask1 = width < 32 ? (1<<width)-1 : -1;
  int32_t mask = mask1 << lsb;
  RECORD_LINE();
  land(dst, src, Immediate(mask));
  if (lsb != 0) {
    RECORD_LINE();
    lsr(dst, dst, Immediate(lsb));
  }
}


void MacroAssembler::Bfc(Register dst, int lsb, int width) {
  ASSERT(!dst.is(sh4_ip));
  ASSERT(lsb >= 0 && lsb < 32);
  ASSERT(width > 0 && width <= 32);
  ASSERT(width + lsb <= 32);
  // Clear bits [lsb..lsb+width-1] of dst
  int32_t mask1 = width < 32 ? (1<<width)-1 : -1;
  int32_t mask = mask1 << lsb;
  RECORD_LINE();
  land(dst, dst, Immediate(~mask));
}


MacroAssembler* MacroAssembler::RecordFunctionLine(const char* function, int line) {
  if (FLAG_code_comments) {
    int size = strlen("/line/")+strlen(function) + 10 + 1 + 1; /* 10(strlen of MAXINT) + 1(separator) +1(nul). */
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
  ASSERT(cond == eq || cond == ne);
  RECORD_LINE();
  land(scratch, object, Immediate(ExternalReference::new_space_mask(isolate())));
  mov(sh4_ip, Immediate(ExternalReference::new_space_start(isolate())));
  cmpeq(scratch, sh4_ip);
  b(cond, branch);
}


void MacroAssembler::RecordWriteHelper(Register object,
                                       Register address,
                                       Register scratch) {
  ASSERT(!object.is(sh4_ip) && !address.is(sh4_ip) && !scratch.is(sh4_ip));
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
  mov(sh4_ip, Immediate(1));
  lsl(sh4_ip, sh4_ip, address);
  lor(scratch, scratch, sh4_ip);
  mov(MemOperand(object, Page::kDirtyFlagOffset), scratch);
}


// Will clobber 4 registers: object, scratch0/1, sh4_ip (ARM:ip).  The
// register 'object' contains a heap object pointer.  The heap object
// tag is shifted away.
void MacroAssembler::RecordWrite(Register object,
                                 int offset,
                                 Register scratch0,
                                 Register scratch1) {
  // The compiled code assumes that record write doesn't change the
  // context register, so we check that none of the clobbered
  // registers are cp.
  ASSERT(!object.is(cp) && !scratch0.is(cp) && !scratch1.is(cp));
  // Also check that the scratch are not sh4_ip (super scratch).
  ASSERT(!object.is(sh4_ip) && !scratch0.is(sh4_ip) && !scratch1.is(sh4_ip));

  Label done;

  RECORD_LINE();
  // First, test that the object is not in the new space.  We cannot set
  // region marks for new space pages.
  InNewSpace(object, scratch0, eq, &done);

  // Add offset into the object.
  add(scratch0, object, Immediate(offset));

  // Record the actual write.
  RecordWriteHelper(object, scratch0, scratch1);

  bind(&done);
  RECORD_LINE();

  // Clobber all input registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    RECORD_LINE();
    mov(object, Immediate(BitCast<int32_t>(kSH4ZapValue)));
    mov(scratch0, Immediate(BitCast<int32_t>(kSH4ZapValue)));
    mov(scratch1, Immediate(BitCast<int32_t>(kSH4ZapValue)));
  }
}


// Will clobber 4 registers: object, offset, scratch, sh4_ip (ARM:ip). The
// register 'object' contains a heap object pointer. The heap object
// tag is shifted away.
void MacroAssembler::RecordWrite(Register object,
                                 Register offset,
                                 Register scratch0,
                                 Register scratch1) {
  // The compiled code assumes that record write doesn't change the
  // context register, so we check that none of the clobbered
  // registers are cp.
  ASSERT(!object.is(cp) && !scratch0.is(cp) && !scratch1.is(cp));
  // Also check that the scratch are not sh4_ip (super scratch).
  ASSERT(!object.is(sh4_ip) && !scratch0.is(sh4_ip) && !scratch1.is(sh4_ip));

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
  // Also check that the scratch are not sh4_ip (super scratch).
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
    mov(object, Immediate(BitCast<int32_t>(kSH4ZapValue)));
    mov(address, Immediate(BitCast<int32_t>(kSH4ZapValue)));
    mov(scratch, Immediate(BitCast<int32_t>(kSH4ZapValue)));
  }
}


// Clobbers: sh4_ip, dst
// live-in: cp
// live-out: cp, dst
void MacroAssembler::LoadContext(Register dst, int context_chain_length) {
  ASSERT(!dst.is(sh4_ip));
  RECORD_LINE();
  if (context_chain_length > 0) {
    RECORD_LINE();
    // Move up the chain of contexts to the context containing the slot.
    mov(dst, MemOperand(cp, Context::SlotOffset(Context::CLOSURE_INDEX)));
    // Load the function context (which is the incoming, outer context).
    mov(dst, FieldMemOperand(dst, JSFunction::kContextOffset));
    for (int i = 1; i < context_chain_length; i++) {
      RECORD_LINE();
      mov(dst, MemOperand(dst, Context::SlotOffset(Context::CLOSURE_INDEX)));
      mov(dst, FieldMemOperand(dst, JSFunction::kContextOffset));
    }
  } else {
    RECORD_LINE();
    // Slot is in the current function context.  Move it into the
    // destination register in case we store into it (the write barrier
    // cannot be allowed to destroy the context in esi).
    mov(dst, cp);
  }

  // We should not have found a 'with' context by walking the context chain
  // (i.e., the static scope chain and runtime context chain do not agree).
  // A variable occurring in such a scope should have slot type LOOKUP and
  // not CONTEXT.
  if (emit_debug_code()) {
    RECORD_LINE();
    mov(sh4_ip, MemOperand(dst, Context::SlotOffset(Context::FCONTEXT_INDEX)));
    cmpeq(dst, sh4_ip);
    Check(eq, "Yo dawg, I heard you liked function contexts "
	  "so I put function contexts in all your contexts");
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
  // Load the initial map. The global functions all have initial maps.
  ldr(map, FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));
  if (emit_debug_code()) {
    Label ok, fail;
    CheckMap(map, scratch, Heap::kMetaMapRootIndex, &fail, false);
    b(&ok);
    bind(&fail);
    Abort("Global functions must have initial map");
    bind(&ok);
  }
}


void MacroAssembler::JumpIfNotPowerOfTwoOrZero(
    Register reg,
    Register scratch,
    Label* not_power_of_two_or_zero) {
  ASSERT(!reg.is(sh4_ip) && !scratch.is(sh4_ip));
  RECORD_LINE();
  // Note: actually the case 0x80000000 is considered a power of two (not a neg value)
  sub(scratch, reg, Immediate(1));
  cmpge(scratch, Immediate(0));
  bf(not_power_of_two_or_zero);
  tst(scratch, reg);
  b(ne, not_power_of_two_or_zero);
}


void MacroAssembler::JumpIfNotPowerOfTwoOrZeroAndNeg(
    Register reg,
    Register scratch,
    Label* zero_and_neg,
    Label* not_power_of_two) {
  ASSERT(!reg.is(sh4_ip) && !scratch.is(sh4_ip));
  RECORD_LINE();
  // Note: actually the case 0x80000000 is considered a pozer of two (not a neg value)
  sub(scratch, reg, Immediate(1));
  cmpge(scratch, Immediate(0));
  bf(zero_and_neg);
  tst(scratch, reg);
  b(ne, not_power_of_two);
}


void MacroAssembler::JumpIfNotBothSmi(Register reg1,
                                      Register reg2,
                                      Label* on_not_both_smi) {
  ASSERT(!reg1.is(sh4_ip) && !reg2.is(sh4_ip));
  STATIC_ASSERT(kSmiTag == 0);
  RECORD_LINE();
  tst(reg1, Immediate(kSmiTagMask));
  b(ne, on_not_both_smi);
  tst(reg2, Immediate(kSmiTagMask));
  b(ne, on_not_both_smi);
}


void MacroAssembler::JumpIfEitherSmi(Register reg1,
                                     Register reg2,
                                     Label* on_either_smi) {
  ASSERT(!reg1.is(sh4_ip) && !reg2.is(sh4_ip));
  STATIC_ASSERT(kSmiTag == 0);
  RECORD_LINE();
  tst(reg1, Immediate(kSmiTagMask));
  b(eq, on_either_smi);
  tst(reg2, Immediate(kSmiTagMask));
  b(eq, on_either_smi);
}


void MacroAssembler::AbortIfSmi(Register object) {
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT(!object.is(sh4_ip));

  tst(object, Immediate(kSmiTagMask));
  Assert(ne, "Operand is a smi");
}


void MacroAssembler::AbortIfNotSmi(Register object) {
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT(!object.is(sh4_ip));

  tst(object, Immediate(kSmiTagMask));
  Assert(eq, "Operand is not smi");
}


void MacroAssembler::AbortIfNotString(Register object) {
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT(!object.is(sh4_ip));

  tst(object, Immediate(kSmiTagMask));
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
  CompareRoot(src, root_value_index);
  Assert(eq, message);
}


void MacroAssembler::PrintRegisterValue(Register reg) {
  ASSERT(!reg.is(r4) && !reg.is(r5) && !reg.is(r6) && !reg.is(r7));
  Label gc_required, skip, not_smi;
  RECORD_LINE();
  EnterInternalFrame();
  push(reg); // Save reg as it is scratched by WriteInt32ToHeapNumberStub()
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
  ldr(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
  AssertRegisterIsRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);
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
  RECORD_LINE();
  // Check that neither is a smi.
  STATIC_ASSERT(kSmiTag == 0);
  land(scratch1, first, second);
  tst(scratch1, Immediate(kSmiTagMask));
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

  int kFlatAsciiStringMask =
      kIsNotStringMask | kStringEncodingMask | kStringRepresentationMask;
  int kFlatAsciiStringTag = ASCII_STRING_TYPE;
  RECORD_LINE();
  land(scratch1, first, Immediate(kFlatAsciiStringMask));
  land(scratch2, second, Immediate(kFlatAsciiStringMask));
  cmp(scratch1, Immediate(kFlatAsciiStringTag));
  b(ne, failure);
  RECORD_LINE();
  cmp(scratch2, Immediate(kFlatAsciiStringTag));
  b(ne, failure);
}


void MacroAssembler::JumpIfInstanceTypeIsNotSequentialAscii(Register type,
                                                            Register scratch,
                                                            Label* failure) {

  ASSERT(!type.is(sh4_ip) && ! scratch.is(sh4_ip));

  int kFlatAsciiStringMask =
      kIsNotStringMask | kStringEncodingMask | kStringRepresentationMask;
  int kFlatAsciiStringTag = ASCII_STRING_TYPE;
  RECORD_LINE();
  land(scratch, type, Immediate(kFlatAsciiStringMask));
  cmp(scratch, Immediate(kFlatAsciiStringTag));
  b(ne, failure);
}


void MacroAssembler::CompareObjectType(Register object,
                                       Register map,
                                       Register type_reg,
                                       InstanceType type,
                                       Condition cond) {
  ASSERT(!object.is(sh4_ip) && !map.is(sh4_ip) && !type_reg.is(sh4_ip));

  RECORD_LINE();
  ldr(map, FieldMemOperand(object, HeapObject::kMapOffset));
  CompareInstanceType(map, type_reg, type, cond);
}


void MacroAssembler::CompareInstanceType(Register map,
                                         Register type_reg,
                                         InstanceType type,
                                         Condition cond) {
  ASSERT(!map.is(sh4_ip) && !type_reg.is(sh4_ip));

  RECORD_LINE();
  ldrb(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset));
  switch(cond) {
  case eq:
    RECORD_LINE();
    cmpeq(type_reg, Immediate(type)); break;
  case ge:
    RECORD_LINE();
    cmpge(type_reg, Immediate(type)); break;
  case hs:
    RECORD_LINE();
    cmphs(type_reg, Immediate(type)); break;
  default:
    UNIMPLEMENTED();
  }
}


void MacroAssembler::CompareRoot(Register obj,
                                 Heap::RootListIndex index) {
  ASSERT(!obj.is(sh4_ip));
  RECORD_LINE();
  LoadRoot(sh4_ip, index);
  cmpeq(obj, sh4_ip);
}


void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Handle<Map> map,
                              Label* fail,
                              bool is_heap_object) {
  ASSERT(!obj.is(sh4_ip) && !scratch.is(sh4_ip));
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


void MacroAssembler::GetRelocatedValueLocation(Register ldr_location,
                               Register result) {
  UNIMPLEMENTED();
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
  instr = (instr & ~0x200); // Changed to bt
  if (cond == ne) 
    instr |= 0x200; // Changed to bf
  masm_.emit(instr);
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
