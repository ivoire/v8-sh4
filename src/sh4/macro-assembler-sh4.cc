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
  ASSERT(!result.is(r3) && !scratch.is(r3));

  RECORD_LINE();
  // Check that the receiver isn't a smi.
  JumpIfSmi(function, miss);

  // Check that the function really is a function.  Load map into result reg.
  CompareObjectType(function, result, scratch, JS_FUNCTION_TYPE);
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
  LoadRoot(r3, Heap::kTheHoleValueRootIndex);
  cmpeq(result, r3);
  bt(miss);
  
  RECORD_LINE();
  // If the function does not have an initial map, we're done.
  Label done;
  CompareObjectType(result, scratch, scratch, MAP_TYPE);
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


void MacroAssembler::IllegalOperation(int num_arguments) {
  RECORD_LINE();
  if (num_arguments > 0) {
    add(sp, sp, Immediate(num_arguments * kPointerSize));
  }
  LoadRoot(r0, Heap::kUndefinedValueRootIndex);
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
  RECORD_LINE();
  Ubfx(hash, hash, String::kHashShift, String::kArrayIndexValueBits);
  lsl(r3, hash, Immediate(kSmiTagSize));
  mov(index, r3);
}


void MacroAssembler::Jump(intptr_t target, RelocInfo::Mode rmode) {
  RECORD_LINE();
  mov(r3, Operand(target, rmode));
  jmp(r3);
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
  
  // TODO: check whether this is necessary
  // Statement positions are expected to be recorded when the target
  // address is loaded. The mov method will automatically record
  // positions when pc is the target, since this is not the case here
  // we have to do it explicitly.
  //positions_recorder()->WriteRecordedPositions();

  RECORD_LINE();
  mov(r3, Operand(target, rmode));
  jsr(r3);

  ASSERT(kCallTargetAddressOffset == 8 * kInstrSize);


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


void MacroAssembler::CallRuntime(const Runtime::Function* f,
                                 int num_arguments) {
  // All parameters are on the stack.  r0 has the return value after call.

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
  // r4-r7: must be preserved
  RECORD_LINE();
  Push(pr, fp, cp);
  mov(r3, Immediate(Smi::FromInt(type)));
  push(r3);
  mov(r3, Operand(CodeObject()));
  push(r3);
  add(fp, sp, Immediate(3 * kPointerSize));  // Adjust FP to point to saved FP.
}


void MacroAssembler::LeaveFrame(StackFrame::Type type) {
  // r4, r5, r6: must be preserved

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
  bool definitely_matches = false;
  Label regular_invoke;

  // Check whether the expected and actual arguments count match. If not,
  // setup registers according to contract with ArgumentsAdaptorTrampoline:
  // ARM -> SH4
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

  RECORD_LINE();
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
      mov(r7, Operand(code_constant));
      add(r7, r7, Immediate(Code::kHeaderSize - kHeapObjectTag));
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
  // r5: must hold function pointer
  // actual: must be r4 if register
  ASSERT(actual.is_immediate() || actual.reg().is(r4));

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


void MacroAssembler::InvokeFunction(Register fun,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    CallWrapper* call_wrapper) {
  // Contract with called JS functions requires that function is passed in r5.
  // Also enforce that actual is passed in r4 if not immediate
  ASSERT(fun.is(r5));
  ASSERT(actual.is_immediate() || actual.reg().is(r4));

  Register expected_reg = r6;
  Register code_reg = r7;

  RECORD_LINE();
  ldr(code_reg, FieldMemOperand(r5, JSFunction::kSharedFunctionInfoOffset));
  ldr(cp, FieldMemOperand(r5, JSFunction::kContextOffset));
  ldr(expected_reg,
      FieldMemOperand(code_reg,
                      SharedFunctionInfo::kFormalParameterCountOffset));
  asr(expected_reg, expected_reg, Immediate(kSmiTagSize));
  ldr(code_reg,
      FieldMemOperand(r5, JSFunction::kCodeEntryOffset));

  ParameterCount expected(expected_reg);
  InvokeCode(code_reg, expected, actual, flag, call_wrapper);
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


void MacroAssembler::Move(Register dst, Register src) {
  if (!dst.is(src)) {
    RECORD_LINE();
    mov(dst, src);
  }
}


void MacroAssembler::PopTryHandler() {
  RECORD_LINE();
  UNIMPLEMENTED_BREAK();
}


static const int kRegisterPassedArguments = 4;

void MacroAssembler::PrepareCallCFunction(int num_arguments, Register scratch) {
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
  // Make sure that the stack is aligned before calling a C function unless
  // running in the simulator. The simulator has its own alignment check which
  // provides more information.
#if defined(V8_HOST_ARCH_SH4)
  if (emit_debug_code()) {
    RECORD_LINE();
    UNIMPLEMENTED_BREAK();
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
    UNIMPLEMENTED_BREAK();
  } else {
    // Must preserve r4-r8, r0-r2 are available.
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
    mov(r0, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
    mov(r1, MemOperand(r0));
    ASSERT(StackHandlerConstants::kNextOffset == 0);
    push(r1);

    // Link this handler as the new current one.
    mov(MemOperand(r0), sp);
  }
}


void MacroAssembler::Throw(Register value) {
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
  RECORD_LINE();
  // Set cp to NULL if fp is NULL.
  mov(cp, Immediate(0));
  jmp(&restore_end);
  bind(&restore);
  RECORD_LINE();
  // Restore cp otherwise.
  mov(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
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

  // r0 is expected to hold the exception.
  if (!value.is(r0)) {
    RECORD_LINE();
    mov(r0, value);
  }

  RECORD_LINE();
  // Drop sp to the top stack handler.
  mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  mov(sp, MemOperand(r3));

  // Unwind the handlers until the ENTRY handler is found.
  Label loop, done;
  bind(&loop);
  RECORD_LINE();
  // Load the type of the current stack handler.
  const int kStateOffset = StackHandlerConstants::kStateOffset;
  mov(r3, MemOperand(sp, kStateOffset));
  mov(r2, Operand(StackHandler::ENTRY));
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
  mov(r3, Operand(ExternalReference(Isolate::k_handler_address, isolate())));
  pop(r2);
  mov(MemOperand(r3), r2);

  if (type == OUT_OF_MEMORY) {
    // Set external caught exception to false.
    ExternalReference external_caught(
        Isolate::k_external_caught_exception_address, isolate());
    RECORD_LINE();
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

  RECORD_LINE();
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
  RECORD_LINE();
  // Set cp to NULL if fp is NULL.
  mov(cp, Immediate(0));
  jmp(&restore_end);
  bind(&restore);
  RECORD_LINE();
  // Restore cp otherwise.
  mov(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  bind(&restore_end);
  RECORD_LINE();
  STATIC_ASSERT(StackHandlerConstants::kPCOffset == 3 * kPointerSize);
  pop(pr);
  rts();
}

int MacroAssembler::SafepointRegisterStackIndex(int reg_code) {
  UNIMPLEMENTED();
  return 0;
}


void MacroAssembler::InvokeBuiltin(Builtins::JavaScript id,
                                   InvokeJSFlags flags,
                                   CallWrapper* call_wrapper) {
  RECORD_LINE();
  GetBuiltinEntry(r6, id);
  if (flags == CALL_JS) {
    RECORD_LINE();
    if (call_wrapper != NULL) call_wrapper->BeforeCall(2 * kInstrSize);
    jsr(r6);
    if (call_wrapper != NULL) call_wrapper->AfterCall();
  } else {
    ASSERT(flags == JUMP_JS);
    RECORD_LINE();
    jmp(r6);
  }
}


void MacroAssembler::GetBuiltinFunction(Register target,
                                        Builtins::JavaScript id) {
  ASSERT(!target.is(r3));
  RECORD_LINE();
  // Load the builtins object into target register.
  mov(target, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  mov(target, FieldMemOperand(target, GlobalObject::kBuiltinsOffset));
  // Load the JavaScript builtin function from the builtins object.
  mov(target, FieldMemOperand(target,
                          JSBuiltinsObject::OffsetOfFunctionWithId(id)));
}


void MacroAssembler::GetBuiltinEntry(Register target, Builtins::JavaScript id) {
  ASSERT(!target.is(r3));
  RECORD_LINE();
  GetBuiltinFunction(target, id);
  RECORD_LINE();
  // Load the code entry point from the builtins object.
  mov(target, MemOperand(target, JSFunction::kCodeEntryOffset));
}


void MacroAssembler::SetCounter(StatsCounter* counter, int value,
                                Register scratch1, Register scratch2) {
  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch1, Immediate(value));
    mov(scratch2, Operand(ExternalReference(counter)));
    mov(MemOperand(scratch2), scratch1);
  }
}


void MacroAssembler::IncrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch2, Operand(ExternalReference(counter)));
    mov(scratch1, MemOperand(scratch2));
    add(scratch1, scratch1, Immediate(value));
    mov(MemOperand(scratch2), scratch1);
  }
}


void MacroAssembler::DecrementCounter(StatsCounter* counter, int value,
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) { 
    RECORD_LINE();
    mov(scratch2, Operand(ExternalReference(counter)));
    mov(scratch1, MemOperand(scratch2));
    sub(scratch1, scratch1, Immediate(value));
    mov(MemOperand(scratch2), scratch1);
  }
}


void MacroAssembler::Assert(const char* msg, bool value) {
  RECORD_LINE();
  if (emit_debug_code()) {
    RECORD_LINE();
    Check(msg, value);
  }
}


void MacroAssembler::AssertFastElements(Register elements) {
  ASSERT(!elements.is(r3));
  RECORD_LINE();
  if (emit_debug_code()) {
    ASSERT(!elements.is(r3));
    Label ok;
    RECORD_LINE();
    push(elements);
    mov(elements, FieldMemOperand(elements, HeapObject::kMapOffset));
    LoadRoot(r3, Heap::kFixedArrayMapRootIndex);
    cmpeq(elements, r3);
    bt(&ok);
    RECORD_LINE();
    LoadRoot(r3, Heap::kFixedCOWArrayMapRootIndex);
    cmpeq(elements, r3);
    bt(&ok);
    RECORD_LINE();
    Abort("JSObject with fast elements map has slow elements");
    bind(&ok);
    RECORD_LINE();
    pop(elements);
  }
}


void MacroAssembler::Check(const char* msg, bool value) {
  Label L;
  // ARM code {
  // b(cond, &L)
  // Abort(msg);
  // }
  // SH4 code {
  RECORD_LINE();
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
      // ARM code {
      // mov(result, Immediate(0x7091));
      // mov(scratch1, Immediate(0x7191));
      // mov(scratch2, Immediate(0x7291));
      // }
      // SH4 code {
      RECORD_LINE();
      mov(result, Immediate(0x7091));
      mov(scratch1, Immediate(0x7191));
      mov(scratch2, Immediate(0x7291));
      // }
    }
    // ARM code: jmp(gc_required);
    // SH4 code:
    RECORD_LINE();
    jmp(gc_required);
    return;
  }

  // Assert that the register arguments are different and that none of
  // them are ip. ip is used explicitly in the code generated below.
  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!result.is(r3));
  ASSERT(!scratch1.is(r3));
  ASSERT(!scratch2.is(r3));

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

  RECORD_LINE();
  // ARM code: mov(topaddr, Operand(new_space_allocation_top));
  // SH4 code:
  mov(topaddr, Immediate(new_space_allocation_top));

  // This code stores a temporary value in r3 (ARM:ip). This is OK, as the code below
  // does not need r3 (ARM:ip) for implicit literal generation.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    RECORD_LINE();
    // Load allocation top into result and allocation limit into r3 (ARM:ip).
    // ARM code: ldm(ia, topaddr, result.bit() | ip.bit());
    // SH4 code {
    mov(result, MemOperand(topaddr));
    mov(r3, MemOperand(topaddr, 4));
    // }
  } else {
    if (emit_debug_code()) {
      RECORD_LINE();
      // Assert that result actually contains top on entry. r3 (ARM:ip) is used
      // immediately below so this use of ip does not cause difference with
      // respect to register content between debug and release mode.
      // ARM code {
      // ldr(ip, MemOperand(topaddr));
      // cmp(result, r3);
      // Check(eq, "Unexpected allocation top");
      // }
      // SH4 code {
      mov(r3, MemOperand(topaddr));
      cmpeq(result, r3);
      Check("Unexpected allocation top");
      // }
    }
    RECORD_LINE();
    // Load allocation limit into r3 (ARM: ip). Result already contains allocation top.
    // ARM code: ldr(ip, MemOperand(topaddr, limit - top));
    // SH4 code:
    mov(r3, MemOperand(topaddr, limit - top));
  }

  RECORD_LINE();
  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top. Object size may be in words so a shift is
  // required to get the number of bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    RECORD_LINE();
    // ARM code: add(scratch2, result, Operand(object_size, LSL, kPointerSizeLog2), SetCC);
    // SH4 code {
    lsl(object_size, object_size, Immediate(kPointerSizeLog2));
    addv(scratch2, result, object_size);
    // }
  } else {
    RECORD_LINE();
    // ARM code: add(scratch2, result, Operand(object_size), SetCC);
    // SH4 code:
    addv(scratch2, result, object_size);
  }
  RECORD_LINE();
  // ARM code {
  // b(cs, gc_required);
  // cmp(scratch2, Operand(ip));
  // b(hi, gc_required);
  // }
  // SH4 code {
  bt(gc_required);
  RECORD_LINE();
  cmpgtu(scratch2, r3);
  bt(gc_required);
  RECORD_LINE();
  // }

  // Update allocation top. result temporarily holds the new top.
  if (emit_debug_code()) {
    // ARM code {
    // tst(scratch2, Operand(kObjectAlignmentMask));
    // Check(eq, "Unaligned allocation in new space");
    // }
    RECORD_LINE();
    tst(scratch2, Immediate(kObjectAlignmentMask));
    Check("Unaligned allocation in new space");
  }
  // ARM code: str(scratch2, MemOperand(topaddr));
  // SH4 code:
  RECORD_LINE();
  mov(MemOperand(topaddr), scratch2);

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    RECORD_LINE();
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

void MacroAssembler::Ret() {
  RECORD_LINE();
  rts();
}


void MacroAssembler::JumpToExternalReference(const ExternalReference& builtin) {
  RECORD_LINE();
  mov(r1, Immediate(builtin));
  CEntryStub stub(1);
  RECORD_LINE();
  jmp(stub.GetCode(), RelocInfo::CODE_TARGET);
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

void MacroAssembler::TailCallRuntime(Runtime::FunctionId fid,
                                     int num_arguments,
                                     int result_size) {
  RECORD_LINE();
  TailCallExternalReference(ExternalReference(fid, isolate()),
                            num_arguments,
                            result_size);
}


void MacroAssembler::Ubfx(Register dst, Register src, int lsb, int width) {
  ASSERT(lsb < 32);
  ASSERT(lsb + width < 32);
  // Extract unsigned value from bits src1[lsb..lsb+width-1] into dst
  uint32_t mask = ((uint32_t)1 << (width + lsb)) - 1 - (((uint32_t)1 << lsb) - 1);
  RECORD_LINE();
  land(dst, src, Immediate(mask));
  if (lsb != 0) {
    RECORD_LINE();
    lsr(dst, dst, Immediate(lsb));
  }
}


void MacroAssembler::Bfc(Register dst, int lsb, int width) {
  ASSERT(lsb < 32);
  ASSERT(lsb + width < 32);
  // Clear bits [lsb..lsb+width-1] of dst
  uint32_t mask = ~(((uint32_t)1 << (width + lsb)) - 1 - (((uint32_t)1 << lsb) - 1));
  RECORD_LINE();
  land(dst, dst, Immediate(mask));
}


void MacroAssembler::InNewSpace(Register object,
                                Register scratch,
                                int eq_0_ne_1,
                                Label* branch) {
  ASSERT(!scratch.is(r3));
  ASSERT(eq_0_ne_1 == 0 || eq_0_ne_1 == 1);
  RECORD_LINE();
  land(scratch, object, Immediate(ExternalReference::new_space_mask(isolate())));
  mov(r3, Immediate(ExternalReference::new_space_start(isolate())));
  cmpeq(scratch, r3);
  if (eq_0_ne_1 == 0) {
    RECORD_LINE();
    bt(branch);
  } else {
    RECORD_LINE();
    bf(branch);
  }
}


void MacroAssembler::RecordWriteHelper(Register object,
                                       Register address,
                                       Register scratch) {
  ASSERT(!scratch.is(r3));
  RECORD_LINE();
  if (emit_debug_code()) {
    // Check that the object is not in new space.
    Label not_in_new_space;
    RECORD_LINE();
    InNewSpace(object, scratch, 1/*ne*/, &not_in_new_space);
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
  mov(r3, Immediate(1));
  lsl(r3, r3, address);
  lor(scratch, scratch, r3);
  mov(MemOperand(object, Page::kDirtyFlagOffset), scratch);
}


// Will clobber 4 registers: object, scratch0/1, r3 (ARM:ip).  The
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

  RECORD_LINE();
  // First, test that the object is not in the new space.  We cannot set
  // region marks for new space pages.
  InNewSpace(object, scratch0, 0/*eq*/, &done);

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
    mov(object, Immediate(BitCast<int32_t>(kZapValue)));
    mov(scratch0, Immediate(BitCast<int32_t>(kZapValue)));
    mov(scratch1, Immediate(BitCast<int32_t>(kZapValue)));
  }
}


// Will clobber 4 registers: object, offset, scratch, r3 (ARM:ip). The
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
  // Also check that the scratch are not r3 (super scratch).
  ASSERT(!object.is(r3) && !scratch0.is(r3) && !scratch1.is(r3));

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
    mov(object, Operand(BitCast<int32_t>(kZapValue)));
    mov(scratch0, Operand(BitCast<int32_t>(kZapValue)));
    mov(scratch1, Operand(BitCast<int32_t>(kZapValue)));
  }
}



// Will clobber 4 registers: object, address, scratch, r3 (ARM:ip).  The
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
  RECORD_LINE();

  // First, test that the object is not in the new space.  We cannot set
  // region marks for new space pages.
  InNewSpace(object, scratch, 0/*eq*/, &done);

  // Record the actual write.
  RecordWriteHelper(object, address, scratch);

  bind(&done);
  RECORD_LINE();

  // Clobber all input registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    RECORD_LINE();
    mov(object, Immediate(BitCast<int32_t>(kZapValue)));
    mov(address, Immediate(BitCast<int32_t>(kZapValue)));
    mov(scratch, Immediate(BitCast<int32_t>(kZapValue)));
  }
}


// Clobbers: r3, dst
// live-in: cp
// live-out: cp, dst
void MacroAssembler::LoadContext(Register dst, int context_chain_length) {
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
    mov(r3, MemOperand(dst, Context::SlotOffset(Context::FCONTEXT_INDEX)));
    cmpeq(dst, r3);
    Check("Yo dawg, I heard you liked function contexts "
	  "so I put function contexts in all your contexts");
  }
}


void MacroAssembler::AbortIfNotString(Register object) {
  STATIC_ASSERT(kSmiTag == 0);
  RECORD_LINE();
  tst(object, Immediate(kSmiTagMask));
  Assert("Operand is not a string");
  RECORD_LINE();
  push(object);
  mov(object, FieldMemOperand(object, HeapObject::kMapOffset));
  CompareInstanceType(object, object, FIRST_NONSTRING_TYPE, hs);
  pop(object);
  Assert("Operand is not a string", false);
  RECORD_LINE();
}


void MacroAssembler::CompareObjectType(Register object,
                                       Register map,
                                       Register type_reg,
                                       InstanceType type,
                                       Condition cond) {
  RECORD_LINE();
  ldr(map, FieldMemOperand(object, HeapObject::kMapOffset));
  CompareInstanceType(map, type_reg, type, cond);
}


void MacroAssembler::CompareInstanceType(Register map,
                                         Register type_reg,
                                         InstanceType type,
                                         Condition cond) {
  ASSERT(!map.is(r3));
  ASSERT(!type_reg.is(r3));
  RECORD_LINE();
  ldrb(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset));
  mov(r3, Immediate(type));
  switch(cond) {
  case eq:
    RECORD_LINE();
    cmpeq(type_reg, r3); break;
  case ge:
    RECORD_LINE();
    cmpge(type_reg, r3); break;
  default:
    UNIMPLEMENTED_BREAK();
  }
}


void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Handle<Map> map,
                              Label* fail,
                              bool is_heap_object) {
  ASSERT(!obj.is(r3));
  RECORD_LINE();
  if (!is_heap_object) {
    RECORD_LINE();
    JumpIfSmi(obj, fail);
  }
  RECORD_LINE();
  mov(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  mov(r3, Operand(map));
  cmpeq(scratch, r3);
  bf(fail);
  RECORD_LINE();

}


void MacroAssembler::CheckMap(Register obj,
                              Register scratch,
                              Heap::RootListIndex index,
                              Label* fail,
                              bool is_heap_object) {
  ASSERT(!obj.is(r3));
  RECORD_LINE();
  if (!is_heap_object) {
    RECORD_LINE();
    JumpIfSmi(obj, fail);
  }
  RECORD_LINE();
  mov(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  LoadRoot(r3, index);
  cmpeq(scratch, r3);
  bf(fail);
  RECORD_LINE();
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
