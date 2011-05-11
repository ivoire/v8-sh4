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
#include "serialize.h"

namespace v8 {
namespace internal {


void MacroAssembler::CallStub(CodeStub* stub) {
  ASSERT(allow_stub_calls());  // Stub calls are not allowed in some stubs.
  jsr(stub->GetCode(), RelocInfo::CODE_TARGET);
}


void MacroAssembler::IllegalOperation(int num_arguments) {
  if (num_arguments > 0) {
    add(sp, sp, Immediate(num_arguments * kPointerSize));
  }
  LoadRoot(r0, Heap::kUndefinedValueRootIndex);
}


void MacroAssembler::Call(
    intptr_t target, RelocInfo::Mode rmode) {
  //TODO: check whether this is necessaery
  //   // Block constant pool for the call instruction sequence.
  //   BlockConstPoolScope block_const_pool(this);
  // #ifdef DEBUG
  //   int pre_position = pc_offset();
  // #endif
  
  // TODO: check whether this is necessary
  // Statement positions are expected to be recorded when the target
  // address is loaded. The mov method will automatically record
  // positions when pc is the target, since this is not the case here
  // we have to do it explicitly.
  //positions_recorder()->WriteRecordedPositions();

  mov(rtmp, Operand(target, rmode));
  jsr(rtmp);
  nop();

  ASSERT(kCallTargetAddressOffset == 2 * kInstrSize);


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
  Call(reinterpret_cast<intptr_t>(code.location()), rmode);

  //TODO: check whether this is necessaery
  // #ifdef DEBUG
  //   int post_position = pc_offset();
  //   CHECK_EQ(pre_position + CallSize(code, rmode, cond), post_position);
  // #endif
}


void MacroAssembler::CallRuntime(const Runtime::Function* f,
                                 int num_arguments) {
  // All parameters are on the stack.  r0 has the return value after call.

  // If the expected number of arguments of the runtime function is
  // constant, we check that the actual number of arguments match the
  // expectation.
  if (f->nargs >= 0 && f->nargs != num_arguments) {
    IllegalOperation(num_arguments);
    return;
  }

  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.
  mov(r4, Immediate(num_arguments));
  mov(r5, Immediate(ExternalReference(f, isolate())));
  CEntryStub stub(1);
  CallStub(&stub);
}

void MacroAssembler::CallRuntime(Runtime::FunctionId fid, int num_arguments) {
  CallRuntime(Runtime::FunctionForId(fid), num_arguments);
}


#ifdef ENABLE_DEBUGGER_SUPPORT
void MacroAssembler::DebugBreak() {
  UNIMPLEMENTED();
}
#endif


void MacroAssembler::Drop(int stack_elements) {
  UNIMPLEMENTED();
}


void MacroAssembler::EnterFrame(StackFrame::Type type) {
  // r4-r7: preserved
  Push(cp, fp, pr);
  mov(r3, Immediate(Smi::FromInt(type)));
  push(r3);
  mov(r3, Operand(CodeObject()));
  push(r3);
  add(fp, sp, Immediate(3 * kPointerSize));  // Adjust FP to point to saved FP.
}


void MacroAssembler::LeaveFrame(StackFrame::Type type) {
  // r4: preserved
  // r5: preserved
  // r6: preserved

  // Drop the execution stack down to the frame pointer and restore
  // the caller frame pointer and return address.
  mov(sp, fp);
  Pop(fp, pr);
}


void MacroAssembler::EnterExitFrame(bool save_doubles, int stack_space) {
  // Setup the frame structure on the stack
  ASSERT_EQ(2 * kPointerSize, ExitFrameConstants::kCallerSPDisplacement);
  ASSERT_EQ(1 * kPointerSize, ExitFrameConstants::kCallerPCOffset);
  ASSERT_EQ(0 * kPointerSize, ExitFrameConstants::kCallerFPOffset);

  // Save PR and FP
  push(pr);
  push(fp);
  // Setup a new frame pointer
  mov(fp, sp);

  // Reserve room for saved entry sp and code object
  sub(sp, sp, Immediate(2*kPointerSize));
  if (emit_debug_code()) {
    mov(r3, Immediate(0));
    mov(MemOperand(fp, ExitFrameConstants::kSPOffset), r3);
  }
//  mov(r3, Operand(CodeObject()));     //TODO: relocation !!
  mov(MemOperand(fp, ExitFrameConstants::kCodeOffset), r3);

  // Save the frame pointer and the context in top.
  mov(r3, Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate())));
  mov(MemOperand(r3), fp);
  mov(r3, Operand(ExternalReference(Isolate::k_context_address, isolate())));
  mov(MemOperand(r3), cp);

  // Optionally save all double registers.
  if (save_doubles)
    pushm(kAllRegisters, true);

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
  add(r3, sp, Immediate(kPointerSize));
  mov(MemOperand(fp, ExitFrameConstants::kSPOffset), r3);
}


void MacroAssembler::LeaveExitFrame(bool save_doubles,
                                    Register argument_count) {
  // Clear top frame.
  mov(r3, Immediate(0));
  mov(r2, Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate())));
  mov(MemOperand(r2), r3);

  // Restore current context from top and clear it in debug mode.
  mov(r3, Operand(ExternalReference(Isolate::k_context_address, isolate())));
  mov(cp, MemOperand(r3));

  // Pop the doubles if needed
  if (save_doubles) {
    // Calculate the stack location of the saved doubles and restore them.
    const int offset = 2 * kPointerSize;
    sub(sp, fp, Immediate(offset + DwVfpRegister::kNumRegisters * kDoubleSize));
    popm(kAllRegisters, true);
  }

  // Tear down the exit frame, pop the arguments, and return.
  mov(sp, fp);
  pop(fp);
  pop(pr);

  if (argument_count.is_valid()) {
    // sp = sp + argument_count << kPointerSizeLog2
    ASSERT(!argument_count.is(r3));
    lsl(r3, argument_count, Immediate(kPointerSizeLog2));
    add(sp, sp, r3);
  }
}


void MacroAssembler::InvokePrologue(const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    Handle<Code> code_constant,
                                    Register code_reg,
                                    Label* done,
                                    InvokeFlag flag,
                                    CallWrapper* call_wrapper) {
  bool definitely_matches = false;
  Label regular_invoke;

  // Check whether the expected and actual arguments count match. If not,
  // setup registers according to contract with ArgumentsAdaptorTrampoline:
  //  r0 -> r4: actual arguments count
  //  r1 -> r5: function (passed through to callee)
  //  r2 -> r6: expected arguments count
  //  r3 -> r7: callee code entry

  // The code below is made a lot easier because the calling code already sets
  // up actual and expected registers according to the contract if values are
  // passed in registers.
  ASSERT(actual.is_immediate() || actual.reg().is(r4));
  ASSERT(expected.is_immediate() || expected.reg().is(r6));
  ASSERT((!code_constant.is_null() && code_reg.is(no_reg)) || code_reg.is(r7));

  if (expected.is_immediate()) {
    ASSERT(actual.is_immediate());
    if (expected.immediate() == actual.immediate()) {
      definitely_matches = true;
    } else {
      mov(r4, Immediate(actual.immediate()));
      const int sentinel = SharedFunctionInfo::kDontAdaptArgumentsSentinel;
      if (expected.immediate() == sentinel) {
        // Don't worry about adapting arguments for builtins that
        // don't want that done. Skip adaption code by making it look
        // like we have a match between expected and actual number of
        // arguments.
        definitely_matches = true;
      } else {
        mov(r6, Immediate(expected.immediate()));
      }
    }
  }
 else {
    if (actual.is_immediate()) {
      mov(r3, Immediate((actual.immediate())));
      cmpeq(expected.reg(), r3);
      bt(&regular_invoke);
      mov(r0, Immediate(actual.immediate()));
    } else {
      cmpeq(expected.reg(), actual.reg());
      bt(&regular_invoke);
    }
  }

  if (!definitely_matches) {
    if (!code_constant.is_null()) {
      mov(r7, Operand(code_constant));
      add(r7, r7, Immediate(Code::kHeaderSize - kHeapObjectTag));
    }
    Handle<Code> adaptor =
        isolate()->builtins()->ArgumentsAdaptorTrampoline();
    if (flag == CALL_FUNCTION) {
      if (call_wrapper != NULL) call_wrapper->BeforeCall(2 * kInstrSize);
      Call(adaptor, RelocInfo::CODE_TARGET);
      if (call_wrapper != NULL) call_wrapper->AfterCall();
      jmp(done);
    } else {
      jmp(adaptor, RelocInfo::CODE_TARGET);
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

  InvokePrologue(expected, actual, Handle<Code>::null(), code, &done, flag,
                 call_wrapper);
  if (flag == CALL_FUNCTION) {
    if (call_wrapper != NULL) call_wrapper->BeforeCall(2 * kInstrSize);
    jsr(code);
    if (call_wrapper != NULL) call_wrapper->AfterCall();
  } else {
    ASSERT(flag == JUMP_FUNCTION);
    jsr(code);
  }

  // Continue here if InvokePrologue does handle the invocation due to
  // mismatched parameter counts.
  bind(&done);
}


void MacroAssembler::InvokeFunction(Register fun,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    CallWrapper* call_wrapper) {
  // Contract with called JS functions requires that function is passed in r5.
  ASSERT(fun.is(r5));

  Register expected_reg = r6;
  Register code_reg = r7;

  mov(code_reg, FieldMemOperand(r5, JSFunction::kSharedFunctionInfoOffset));
  mov(cp, FieldMemOperand(r5, JSFunction::kContextOffset));
  mov(expected_reg,
      FieldMemOperand(code_reg,
                      SharedFunctionInfo::kFormalParameterCountOffset));
  asr(expected_reg, expected_reg, Immediate(kSmiTagSize));
  mov(code_reg,
      FieldMemOperand(r5, JSFunction::kCodeEntryOffset));

  ParameterCount expected(expected_reg);
  InvokeCode(code_reg, expected, actual, flag, call_wrapper);
}


MacroAssembler::MacroAssembler(Isolate* arg_isolate, void* buffer, int size)
    : Assembler(arg_isolate, buffer, size),
      generating_stub_(false),
      allow_stub_calls_(true) {
  if (isolate() != NULL)
    code_object_ = Handle<Object>(isolate()->heap()->undefined_value(),
                                  isolate());
}


void MacroAssembler::Move(Register dst, Register src) {
  if (!dst.is(src)) {
    mov(dst, src);
  }
}


void MacroAssembler::PopTryHandler() {
  UNIMPLEMENTED();
}


static const int kRegisterPassedArguments = 4;

void MacroAssembler::PrepareCallCFunction(int num_arguments, Register scratch) {
  int frame_alignment = OS::ActivationFrameAlignment();

  // Up to four simple arguments are passed in registers r4..r7.
  int stack_passed_arguments = (num_arguments <= kRegisterPassedArguments) ?
                               0 : num_arguments - kRegisterPassedArguments;

  if (frame_alignment > 0) {
    mov(scratch, sp);
    sub(sp, sp, Immediate((stack_passed_arguments + 1) * kPointerSize));
    ASSERT(IsPowerOf2(frame_alignment));
    land(sp, sp, Immediate(-frame_alignment));
    mov(MemOperand(sp, stack_passed_arguments * kPointerSize), scratch);
  } else {
    sub(sp, sp, Immediate(stack_passed_arguments * kPointerSize));
  }
}


void MacroAssembler::CallCFunction(ExternalReference function,
                                   int num_arguments) {
  CallCFunctionHelper(no_reg, function, r3, num_arguments);
}


void MacroAssembler::CallCFunctionHelper(Register function,
                                         ExternalReference function_reference,
                                         Register scratch,
                                         int num_arguments) {
  // Make sure that the stack is aligned before calling a C function unless
  // running in the simulator. The simulator has its own alignment check which
  // provides more information.
#if defined(V8_HOST_ARCH_SH4)
  if (emit_debug_code()) {
    UNIMPLEMENTED();
  }
#endif

  // Just call directly. The function called cannot cause a GC, or
  // allow preemption, so the return address in the link register
  // stays correct.
  if (function.is(no_reg)) {
    mov(scratch, Operand(function_reference));
    function = scratch;
  }
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
    UNIMPLEMENTED();
  } else {
    ASSERT(try_location == IN_JS_ENTRY);
    // The frame pointer does not point to a JS frame so we save NULL
    // for ebp. We expect the code throwing an exception to check ebp
    // before dereferencing it to restore the context.
    push(Immediate(StackHandler::ENTRY));
    push(Immediate(0));

    // Save the current handler as the next handler.
    push(Operand(ExternalReference(Isolate::k_handler_address, isolate())));

    // Link this handler as the new current one.
    mov(sp, Operand(ExternalReference(Isolate::k_handler_address,
                                      isolate())));
  }
}


void MacroAssembler::Throw(Register value) {
  // r0 is expected to hold the exception.
  if (!value.is(r0)) {
    mov(r0, value);
  }

  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 4 * kPointerSize);

  // Drop the sp to the top of the handler.
  mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  mov(sp, MemOperand(r3));

  // Restore the next handler and frame pointer, discard handler state.
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  pop(r2);
  mov(MemOperand(r3), r2);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 2 * kPointerSize);
  pop(r3);
  pop(fp);

  // Before returning we restore the context from the frame pointer if
  // not NULL.  The frame pointer is NULL in the exception handler of a
  // JS entry frame.
  Label restore, restore_end;
  mov(r3, Immediate(0));
  cmpeq(fp, r3);
  bf(&restore);
  // Set cp to NULL if fp is NULL.
  mov(cp, Immediate(0));
  jmp(&restore_end);
  bind(&restore);
  // Restore cp otherwise.
  mov(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  bind(&restore_end);
  STATIC_ASSERT(StackHandlerConstants::kPCOffset == 3 * kPointerSize);
  pop(pr);
  rts();
}


void MacroAssembler::ThrowUncatchable(UncatchableExceptionType type,
                                      Register value) {
  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 4 * kPointerSize);

  // r0 is expected to hold the exception.
  if (!value.is(r0)) {
    mov(r0, value);
  }

  // Drop sp to the top stack handler.
  mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  mov(sp, MemOperand(r3));

  // Unwind the handlers until the ENTRY handler is found.
  Label loop, done;
  bind(&loop);
  // Load the type of the current stack handler.
  const int kStateOffset = StackHandlerConstants::kStateOffset;
  mov(r3, MemOperand(sp, kStateOffset));
  mov(r2, Operand(StackHandler::ENTRY));
  cmpeq(r2, r3);
  bt(&done);
  // Fetch the next handler in the list.
  const int kNextOffset = StackHandlerConstants::kNextOffset;
  mov(sp, MemOperand(sp, kNextOffset));
  jmp(&loop);
  bind(&done);

  // Set the top handler address to next handler past the current ENTRY handler.
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  pop(r2);
  mov(MemOperand(r3), r2);

  if (type == OUT_OF_MEMORY) {
    // Set external caught exception to false.
    ExternalReference external_caught(
        Isolate::k_external_caught_exception_address, isolate());
    mov(r0, Immediate(false));
    mov(r3, Operand(external_caught));
    mov(MemOperand(r0), r3);

    // Set pending exception and r0 to out of memory exception.
    Failure* out_of_memory = Failure::OutOfMemoryException();
    mov(r0, Immediate(reinterpret_cast<int32_t>(out_of_memory)));
    mov(r3, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                      isolate())));
    mov(MemOperand(r3), r0);
  }

  // Stack layout at this point. See also StackHandlerConstants.
  // sp ->   state (ENTRY)
  //         fp
  //         pr

  // Discard handler state (r3 is not used) and restore frame pointer.
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 2 * kPointerSize);
  pop(r3);
  pop(fp);
  // Before returning we restore the context from the frame pointer if
  // not NULL.  The frame pointer is NULL in the exception handler of a
  // JS entry frame.
  Label restore, restore_end;
  mov(r3, Immediate(0));
  cmpeq(fp, r3);
  bf(&restore);
  // Set cp to NULL if fp is NULL.
  mov(cp, Immediate(0));
  jmp(&restore_end);
  bind(&restore);
  // Restore cp otherwise.
  mov(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  bind(&restore_end);
  STATIC_ASSERT(StackHandlerConstants::kPCOffset == 3 * kPointerSize);
  pop(pr);
  rts();
}

int MacroAssembler::SafepointRegisterStackIndex(int reg_code) {
  UNIMPLEMENTED();
  return 0;
}


void MacroAssembler::GetBuiltinFunction(Register target,
                                        Builtins::JavaScript id) {
  // Load the builtins object into target register.
  mov(target, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  mov(target, FieldMemOperand(target, GlobalObject::kBuiltinsOffset));
  // Load the JavaScript builtin function from the builtins object.
  mov(target, FieldMemOperand(target,
                          JSBuiltinsObject::OffsetOfFunctionWithId(id)));
}


void MacroAssembler::GetBuiltinEntry(Register target, Builtins::JavaScript id) {
  ASSERT(!target.is(r3));
  GetBuiltinFunction(r3, id);
  // Load the code entry point from the builtins object.
  mov(target, MemOperand(r3, JSFunction::kCodeEntryOffset));
}


void MacroAssembler::SetCounter(StatsCounter* counter, int value,
                                Register scratch1, Register scratch2) {
  if (FLAG_native_code_counters && counter->Enabled()) {
    mov(scratch1, Immediate(value));
    mov(scratch2, Operand(ExternalReference(counter)));
    mov(MemOperand(scratch2), scratch1);
  }
}


void MacroAssembler::IncrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  if (FLAG_native_code_counters && counter->Enabled()) {
    mov(scratch2, Operand(ExternalReference(counter)));
    mov(scratch1, MemOperand(scratch2));
    add(scratch1, scratch1, Immediate(value));
    mov(MemOperand(scratch2), scratch1);
  }
}


void MacroAssembler::DecrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  if (FLAG_native_code_counters && counter->Enabled()) {
    mov(scratch2, Operand(ExternalReference(counter)));
    mov(scratch1, MemOperand(scratch2));
    sub(scratch1, scratch1, Immediate(value));
    mov(MemOperand(scratch2), scratch1);
  }
}


void MacroAssembler::Assert(const char* msg, bool value) {
  if (emit_debug_code())
  Check(msg, value);
}


void MacroAssembler::Check(const char* msg, bool value) {
  Label L;
  // ARM code {
  // b(cond, &L)
  // Abort(msg);
  // }
  // SH4 code {
  if (value)
    bt(&L);
  else
    bf(&L);
  Abort(msg);
  // }
  // will not return here
  bind(&L);
}

void MacroAssembler::Abort(const char* msg) {
  Label abort_start;
  bind(&abort_start);
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

void MacroAssembler::AllocateInNewSpace(Register object_size,
                                        Register result,
                                        Register scratch1,
                                        Register scratch2,
                                        Label* gc_required,
                                        AllocationFlags flags) {
  if (!FLAG_inline_new) {
    if (emit_debug_code()) {
      // Trash the registers to simulate an allocation failure.
      // ARM code {
      // mov(result, Immediate(0x7091));
      // mov(scratch1, Immediate(0x7191));
      // mov(scratch2, Immediate(0x7291));
      // }
      // SH4 code {
      mov(result, Immediate(0x7091));
      mov(scratch1, Immediate(0x7191));
      mov(scratch2, Immediate(0x7291));
      // }
    }
    // ARM code: jmp(gc_required);
    // SH4 code:
    jmp(gc_required);
    return;
  }

  // Assert that the register arguments are different and that none of
  // them are ip. ip is used explicitly in the code generated below.
  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!result.is(rtmp));
  ASSERT(!scratch1.is(rtmp));
  ASSERT(!scratch2.is(rtmp));

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

  // Set up allocation top address.
  Register topaddr = scratch1;
  // ARM code: mov(topaddr, Operand(new_space_allocation_top));
  // SH4 code:
  mov(topaddr, Immediate(new_space_allocation_top));

  // This code stores a temporary value in rtmp (ARM:ip). This is OK, as the code below
  // does not need rtmp (ARM:ip) for implicit literal generation.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    // Load allocation top into result and allocation limit into rtmp (ARM:ip).
    // ARM code: ldm(ia, topaddr, result.bit() | ip.bit());
    // SH4 code {
    mov(result, MemOperand(topaddr));
    mov(rtmp, MemOperand(topaddr, 4));
    // }
  } else {
    if (emit_debug_code()) {
      // Assert that result actually contains top on entry. rtmp (ARM:ip) is used
      // immediately below so this use of ip does not cause difference with
      // respect to register content between debug and release mode.
      // ARM code {
      // ldr(ip, MemOperand(topaddr));
      // cmp(result, rtmp);
      // Check(eq, "Unexpected allocation top");
      // }
      // SH4 code {
      mov(rtmp, MemOperand(topaddr));
      cmpeq(result, rtmp);
      Check("Unexpected allocation top");
      // }
    }
    // Load allocation limit into rtmp (ARM: ip). Result already contains allocation top.
    // ARM code: ldr(ip, MemOperand(topaddr, limit - top));
    // SH4 code:
    mov(rtmp, MemOperand(topaddr, limit - top));
  }

  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top. Object size may be in words so a shift is
  // required to get the number of bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    // ARM code: add(scratch2, result, Operand(object_size, LSL, kPointerSizeLog2), SetCC);
    // SH4 code {
    lsl(object_size, object_size, Immediate(kPointerSizeLog2));
    addv(scratch2, result, object_size);
    // }
  } else {
    // ARM code: add(scratch2, result, Operand(object_size), SetCC);
    // SH4 code:
    addv(scratch2, result, object_size);
  }
  // ARM code {
  // b(cs, gc_required);
  // cmp(scratch2, Operand(ip));
  // b(hi, gc_required);
  // }
  // SH4 code {
  bt(gc_required);
  cmpgtu(scratch2, rtmp);
  bt(gc_required);
  // }

  // Update allocation top. result temporarily holds the new top.
  if (emit_debug_code()) {
    // ARM code {
    // tst(scratch2, Operand(kObjectAlignmentMask));
    // Check(eq, "Unaligned allocation in new space");
    // }
    tst(scratch2, Immediate(kObjectAlignmentMask));
    Check("Unaligned allocation in new space");
  }
  // ARM code: str(scratch2, MemOperand(topaddr));
  // SH4 code:
  mov(MemOperand(topaddr), scratch2);

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    add(result, result, Immediate(kHeapObjectTag));
  }
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
  for (int i = 0; i < field_count; i++) {
    mov(tmp, FieldMemOperand(src, i * kPointerSize));
    mov(FieldMemOperand(dst, i * kPointerSize), tmp);
  }
}

void MacroAssembler::LoadRoot(Register destination,
                              Heap::RootListIndex index) {
  mov(destination, MemOperand(roots, index << kPointerSizeLog2));
}

void MacroAssembler::StoreRoot(Register source,
			       Heap::RootListIndex index) {
  mov(MemOperand(roots, index << kPointerSizeLog2), source);
}

void MacroAssembler::Ret() {
  rts();
  nop();
}


void MacroAssembler::JumpToExternalReference(const ExternalReference& builtin) {
  mov(r5, Immediate(builtin));
  CEntryStub stub(1);
  jmp(stub.GetCode(), RelocInfo::CODE_TARGET);
}

void MacroAssembler::TailCallExternalReference(const ExternalReference& ext,
                                               int num_arguments,
                                               int result_size) {
  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.
  mov(r4, Immediate(num_arguments));
  JumpToExternalReference(ext);
}

void MacroAssembler::TailCallRuntime(Runtime::FunctionId fid,
                                     int num_arguments,
                                     int result_size) {
  TailCallExternalReference(ExternalReference(fid, isolate()),
                            num_arguments,
                            result_size);
}


void MacroAssembler::Ubfx(Register dst, Register src, int lsb, int width) {
  ASSERT(lsb < 32);
  ASSERT(lsb + width < 32);
  // Extract unsigned value from bits src1[lsb..lsb+width-1] into dst
  uint32_t mask = ((uint32_t)1 << (width + lsb)) - 1 - (((uint32_t)1 << lsb) - 1);
  land(dst, src, Immediate(mask));
  if (lsb != 0) {
    lsr(dst, dst, Immediate(lsb));
  }
}


void MacroAssembler::Bfc(Register dst, int lsb, int width) {
  ASSERT(lsb < 32);
  ASSERT(lsb + width < 32);
  // Clear bits [lsb..lsb+width-1] of dst
  uint32_t mask = ~(((uint32_t)1 << (width + lsb)) - 1 - (((uint32_t)1 << lsb) - 1));
  land(dst, dst, Immediate(mask));
}


void MacroAssembler::InNewSpace(Register object,
                                Register scratch,
                                int eq_0_ne_1,
                                Label* branch) {
  ASSERT(!scratch.is(rtmp));
  ASSERT(eq_0_ne_1 == 0 || eq_0_ne_1 == 1);
  land(scratch, object, Immediate(ExternalReference::new_space_mask(isolate())));
  mov(rtmp, Immediate(ExternalReference::new_space_start(isolate())));
  cmpeq(scratch, rtmp);
  if (eq_0_ne_1 == 0)
    bt(branch);
  else
    bf(branch);
}


void MacroAssembler::RecordWriteHelper(Register object,
                                       Register address,
                                       Register scratch) {
  ASSERT(!scratch.is(rtmp));
  if (emit_debug_code()) {
    // Check that the object is not in new space.
    Label not_in_new_space;
    InNewSpace(object, scratch, 1/*ne*/, &not_in_new_space);
    Abort("new-space object passed to RecordWriteHelper");
    bind(&not_in_new_space);
  }

  // Calculate page address.
  Bfc(object, 0, kPageSizeBits);

  // Calculate region number.
  Ubfx(address, address, Page::kRegionSizeLog2,
       kPageSizeBits - Page::kRegionSizeLog2);

  // Mark region dirty.
  mov(scratch, MemOperand(object, Page::kDirtyFlagOffset));
  mov(rtmp, Immediate(1));
  lsl(rtmp, rtmp, address);
  lor(scratch, scratch, rtmp);
  mov(MemOperand(object, Page::kDirtyFlagOffset), scratch);
}


// Will clobber 4 registers: object, scratch0/1, rtmp (ARM:ip).  The
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

  Label done;

  // First, test that the object is not in the new space.  We cannot set
  // region marks for new space pages.
  InNewSpace(object, scratch0, 0/*eq*/, &done);

  // Add offset into the object.
  add(scratch0, object, Immediate(offset));

  // Record the actual write.
  RecordWriteHelper(object, scratch0, scratch1);

  bind(&done);

  // Clobber all input registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    mov(object, Immediate(BitCast<int32_t>(kZapValue)));
    mov(scratch0, Immediate(BitCast<int32_t>(kZapValue)));
    mov(scratch1, Immediate(BitCast<int32_t>(kZapValue)));
  }
}


// Will clobber 4 registers: object, address, scratch, rtmp (ARM:ip).  The
// register 'object' contains a heap object pointer.  The heap object
// tag is shifted away.
void MacroAssembler::RecordWrite(Register object,
                                 Register address,
                                 Register scratch) {
  // The compiled code assumes that record write doesn't change the
  // context register, so we check that none of the clobbered
  // registers are cp.
  ASSERT(!object.is(cp) && !address.is(cp) && !scratch.is(cp));

  Label done;

  // First, test that the object is not in the new space.  We cannot set
  // region marks for new space pages.
  InNewSpace(object, scratch, 0/*eq*/, &done);

  // Record the actual write.
  RecordWriteHelper(object, address, scratch);

  bind(&done);

  // Clobber all input registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    mov(object, Immediate(BitCast<int32_t>(kZapValue)));
    mov(address, Immediate(BitCast<int32_t>(kZapValue)));
    mov(scratch, Immediate(BitCast<int32_t>(kZapValue)));
  }
}


// Clobbers: rtmp, dst
// live-in: cp
// live-out: cp, dst
void MacroAssembler::LoadContext(Register dst, int context_chain_length) {
  if (context_chain_length > 0) {
    // Move up the chain of contexts to the context containing the slot.
    mov(dst, MemOperand(cp, Context::SlotOffset(Context::CLOSURE_INDEX)));
    // Load the function context (which is the incoming, outer context).
    mov(dst, FieldMemOperand(dst, JSFunction::kContextOffset));
    for (int i = 1; i < context_chain_length; i++) {
      mov(dst, MemOperand(dst, Context::SlotOffset(Context::CLOSURE_INDEX)));
      mov(dst, FieldMemOperand(dst, JSFunction::kContextOffset));
    }
  } else {
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
    mov(rtmp, MemOperand(dst, Context::SlotOffset(Context::FCONTEXT_INDEX)));
    cmpeq(dst, rtmp);
    Check("Yo dawg, I heard you liked function contexts "
	  "so I put function contexts in all your contexts");
  }
}


void MacroAssembler::AbortIfNotString(Register object) {
  STATIC_ASSERT(kSmiTag == 0);
  tst(object, Immediate(kSmiTagMask));
  Assert("Operand is not a string");
  push(object);
  mov(object, FieldMemOperand(object, HeapObject::kMapOffset));
  CompareInstanceType(object, object, FIRST_NONSTRING_TYPE, hs);
  pop(object);
  Assert("Operand is not a string", false);
}


void MacroAssembler::CompareObjectType(Register object,
                                       Register map,
                                       Register type_reg,
                                       InstanceType type,
                                       Condition cond) {
  mov(map, FieldMemOperand(object, HeapObject::kMapOffset));
  CompareInstanceType(map, type_reg, type, cond);
}


void MacroAssembler::CompareInstanceType(Register map,
                                         Register type_reg,
                                         InstanceType type,
                                         Condition cond) {
  ASSERT(!map.is(r3));
  mov(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset)); //FIXME: mov.b ??
  mov(r3, Immediate(type));
  switch(cond) {
  case eq:
    cmpeq(type_reg, r3); break;
  case ge:
    cmpge(type_reg, r3); break;
  default:
    UNIMPLEMENTED();
  }
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
