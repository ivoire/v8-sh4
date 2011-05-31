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

#include "codegen.h"
#include "debug.h"
#include "deoptimizer.h"
#include "full-codegen.h"
#include "runtime.h"

namespace v8 {
namespace internal {


#define __ ACCESS_MASM(masm)


void Builtins::Generate_Adaptor(MacroAssembler* masm,
                                CFunctionId id,
                                BuiltinExtraArguments extra_args) {
  // ----------- S t a t e -------------
  //  -- r0                 : number of arguments excluding receiver
  //  -- r1                 : called function (only guaranteed when
  //                          extra_args requires it)
  //  -- cp                 : context
  //  -- sp[0]              : last argument
  //  -- ...
  //  -- sp[4 * (argc - 1)] : first argument (argc == r0)
  //  -- sp[4 * argc]       : receiver
  // -----------------------------------

  // Insert extra arguments.
  int num_extra_args = 0;
  if (extra_args == NEEDS_CALLED_FUNCTION) {
    num_extra_args = 1;
    __ push(r1);
  } else {
    ASSERT(extra_args == NO_EXTRA_ARGUMENTS);
  }

  // JumpToExternalReference expects r0 to contain the number of arguments
  // including the receiver and the extra arguments.
  __ add(r0, Immediate(num_extra_args + 1));
  __ JumpToExternalReference(ExternalReference(id, masm->isolate()));
}


void Builtins::Generate_ArrayCode(MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}

void Builtins::Generate_ArrayConstructCode(MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Builtins::Generate_StringConstructCode(MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Builtins::Generate_JSConstructCall(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0     : number of arguments
  //  -- r1     : constructor function
  //  -- pr     : return address
  //  -- sp[...]: constructor arguments
  // -----------------------------------
  Label non_function_call;

  // Check that the function is not a smi.
  __ tst(r1, Immediate(kSmiTagMask));
  __ bt(&non_function_call);
  // Check that the function is a JSFunction.
  __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE);
  __ bf(&non_function_call);

  // Jump to the function-specific construct stub.
  __ ldr(r2, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
  __ ldr(r2, FieldMemOperand(r2, SharedFunctionInfo::kConstructStubOffset));
  __ add(r3, r2, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ jsr(r3);   // FIXME: jsr or jmp ???

  // r0: number of arguments
  // r1: called object
  __ bind(&non_function_call);
  // r2: expected number of arguments to zero (not changing r0).
  __ mov(r2, Immediate(0));
  // r3: builtin entry
  __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION_AS_CONSTRUCTOR);
  __ jmp(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
          RelocInfo::CODE_TARGET);
}

static void Generate_JSConstructStubHelper(MacroAssembler* masm,
                                           bool is_api_function,
                                           bool count_constructions) {
  __ UNIMPLEMENTED_BREAK();
}

void Builtins::Generate_JSConstructStubCountdown(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, false, true);
}


void Builtins::Generate_JSConstructStubGeneric(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, false, false);
}


void Builtins::Generate_JSConstructStubApi(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, true, false);
}


static void Generate_JSEntryTrampolineHelper(MacroAssembler* masm,
                                             bool is_construct) {
  // Called from Generate_JS_Entry
  // r0: code entry
  // r1: function
  // r2: receiver
  // r3: argc
  // r4: argv (JSEntryStub does set it)
  // r5-r7, cp may be clobbered

  // Clear the context before we push it when entering the JS frame.
  __ mov(cp, Immediate(0));

  // Enter an internal frame.
  __ EnterInternalFrame();

  // Set up the context from the function argument.
  __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));

  // Set up the roots register.
  ExternalReference roots_address =
      ExternalReference::roots_address(masm->isolate());
  __ mov(roots, Operand(roots_address));

  // Push the function and the receiver onto the stack.
  __ push(r1);
  __ push(r2);

  // Copy arguments to the stack in a loop.
  // r1: function
  // r3: argc
  // r4: argv, i.e. points to first arg
  Label loop, entry;
  __ lsl(r5, r3, Immediate(kPointerSizeLog2));
  __ add(r2, r4, r5);
  // r2 points past last arg.
  __ jmp(&entry);
  __ bind(&loop);
    __ ldr(r0, MemOperand(r4, kPointerSize));     // read next parameter
    __ add(r4, r4, Immediate(kPointerSize));      // mov the r4 pointer onto the next parameter
    __ ldr(r0, MemOperand(r0));  // dereference handle
    __ push(r0);  // push parameter
  __ bind(&entry);
    __ cmpeq(r4, r2);
  __ bf(&loop);

  // Initialize all JavaScript callee-saved registers, since they will be seen
  // by the garbage collector as part of handlers.
  __ LoadRoot(r4, Heap::kUndefinedValueRootIndex);
  __ mov(r5, r4);
  __ mov(r6, r4);
  __ mov(r7, r4);
  __ mov(r8, r4);
  __ mov(r9, r4);

  // Invoke the code and pass argc as r0.
  __ mov(r0, r3);

  // r0: argc
  // r1: function
  if (is_construct) {
    __ Call(masm->isolate()->builtins()->JSConstructCall(),
            RelocInfo::CODE_TARGET);
  } else {
    ParameterCount actual(r0);
    __ InvokeFunction(r1, actual, CALL_FUNCTION);
  }

  // Exit the JS frame and remove the parameters (except function), and return.
  // Respect ABI stack constraint.
  __ LeaveInternalFrame();
  __ rts();

  // r0: result
}


void Builtins::Generate_JSEntryTrampoline(MacroAssembler* masm) {
  Generate_JSEntryTrampolineHelper(masm, false);
}


void Builtins::Generate_JSConstructEntryTrampoline(MacroAssembler* masm) {
  Generate_JSEntryTrampolineHelper(masm, true);
}


static void Generate_LazyCompileHelper(MacroAssembler* masm, Runtime::FunctionId fid) {
  // Enter an internal frame.
  __ EnterInternalFrame();

  // Preserve the function.
  __ push(r1);

  // Push the function on the stack as the argument to the runtime function.
  __ push(r1);
  __ CallRuntime(fid, 1);
  // Calculate the entry point.
  __ add(r2, r0, Immediate(Code::kHeaderSize - kHeapObjectTag));
  // Restore saved function.
  __ pop(r1);

  // Tear down temporary frame.
  __ LeaveInternalFrame();

  // Do a tail-call of the compiled function.
  __ jmp(r2);
}


void Builtins::Generate_LazyCompile(MacroAssembler* masm) {
  Generate_LazyCompileHelper(masm, Runtime::kLazyCompile);
}


void Builtins::Generate_LazyRecompile(MacroAssembler* masm) {
  Generate_LazyCompileHelper(masm, Runtime::kLazyRecompile);
}


static void Generate_NotifyDeoptimizedHelper(MacroAssembler* masm,
                                             Deoptimizer::BailoutType type) {
  __ EnterInternalFrame();
  // Pass the function and deoptimization type to the runtime system.
  __ mov(r0, Immediate(Smi::FromInt(static_cast<int>(type))));
  __ push(r0);
  __ CallRuntime(Runtime::kNotifyDeoptimized, 1);
  __ LeaveInternalFrame();

  // Get the full codegen state from the stack and untag it -> r6.
  __ ldr(r6, MemOperand(sp, 0 * kPointerSize));
  __ SmiUntag(r6);
  // Switch on the state.
  Label with_tos_register, unknown_state;
  __ cmpeq(r6, Immediate(FullCodeGenerator::NO_REGISTERS));
  __ bf(&with_tos_register);
  __ add(sp, sp, Immediate(1 * kPointerSize));  // Remove state.
  __ rts();

  __ bind(&with_tos_register);
  __ ldr(r0, MemOperand(sp, 1 * kPointerSize));
  __ cmpeq(r6, Immediate(FullCodeGenerator::TOS_REG));
  __ bf(&unknown_state);
  __ add(sp, sp, Immediate(2 * kPointerSize));  // Remove state.
  __ rts();

  __ bind(&unknown_state);
  __ stop("no cases left");
}


void Builtins::Generate_NotifyDeoptimized(MacroAssembler* masm) {
  Generate_NotifyDeoptimizedHelper(masm, Deoptimizer::EAGER);
}


void Builtins::Generate_NotifyLazyDeoptimized(MacroAssembler* masm) {
  Generate_NotifyDeoptimizedHelper(masm, Deoptimizer::LAZY);
}


void Builtins::Generate_NotifyOSR(MacroAssembler* masm) {
  // For now, we are relying on the fact that Runtime::NotifyOSR
  // doesn't do any garbage collection which allows us to save/restore
  // the registers without worrying about which of them contain
  // pointers. This seems a bit fragile.
  __ pushm(kJSCallerSaved | kCalleeSaved);
  __ Push(pr, fp);
  __ EnterInternalFrame();
  __ CallRuntime(Runtime::kNotifyOSR, 0);
  __ LeaveInternalFrame();
  __ Pop(pr, fp);
  __ popm(kJSCallerSaved | kCalleeSaved);
  __ rts();
}


void Builtins::Generate_OnStackReplacement(MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Builtins::Generate_FunctionCall(MacroAssembler* masm) {
  // 1. Make sure we have at least one argument.
  // r0: actual number of arguments
  { Label done;
    __ tst(r0, r0);
    __ bf(&done);
    __ LoadRoot(r2, Heap::kUndefinedValueRootIndex);
    __ push(r2);
    __ add(r0, r0, Immediate(1));
    __ bind(&done);
  }

  // 2. Get the function to call (passed as receiver) from the stack, check
  //    if it is a function.
  // r0: actual number of arguments
  Label non_function;
  __ lsl(r11, r0, Immediate(kPointerSizeLog2));
  __ mov(r1, MemOperand(sp, r11));
  __ tst(r1, Immediate(kSmiTagMask));
  __ bt(&non_function);
  __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE);
  __ bf(&non_function);

  // 3a. Patch the first argument if necessary when calling a function.
  // r0: actual number of arguments
  // r1: function
  Label shift_arguments;
  { Label convert_to_object, use_global_receiver, patch_receiver;
    // Change context eagerly in case we need the global receiver.
    __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));

    // Do not transform the receiver for strict mode functions.
    __ ldr(r2, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
    __ ldr(r2, FieldMemOperand(r2, SharedFunctionInfo::kCompilerHintsOffset));
    __ tst(r2, Immediate(1 << (SharedFunctionInfo::kStrictModeFunction +
                               kSmiTagSize)));
    __ bf(&shift_arguments);

    // Compute the receiver in non-strict mode.
    __ lsl(r11, r0, Immediate(kPointerSizeLog2));
    __ add(r2, sp, r11);
    __ ldr(r2, MemOperand(r2, -kPointerSize));
    // r0: actual number of arguments
    // r1: function
    // r2: first argument
    __ tst(r2, Immediate(kSmiTagMask));
    __ bt(&convert_to_object);

    __ LoadRoot(r3, Heap::kNullValueRootIndex);
    __ cmpeq(r2, r3);
    __ bt(&use_global_receiver);
    __ LoadRoot(r3, Heap::kUndefinedValueRootIndex);
    __ cmpeq(r2, r3);
    __ bt(&use_global_receiver);

    __ CompareObjectType(r2, r3, r3, FIRST_JS_OBJECT_TYPE, ge);
    __ bf(&convert_to_object);
    __ cmpgt(r3, Immediate(LAST_JS_OBJECT_TYPE));
    __ bf(&shift_arguments);

    __ bind(&convert_to_object);
    __ EnterInternalFrame();  // In order to preserve argument count.
    __ lsl(r0, r0, Immediate(kSmiTagSize)); // Smi-tagged.
    __ push(r0);

    __ push(r2);
    __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
    __ mov(r2, r0);

    __ pop(r0);
    __ asr(r0, r0, Immediate(kSmiTagSize));
    __ LeaveInternalFrame();
    // Restore the function to r1.
    __ lsl(r1, r0, Immediate(kPointerSizeLog2));
    __ ldr(r1, MemOperand(sp, r1));
    __ jmp(&patch_receiver);

    // Use the global receiver object from the called function as the
    // receiver.
    __ bind(&use_global_receiver);
    const int kGlobalIndex =
        Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
    __ ldr(r2, FieldMemOperand(cp, kGlobalIndex));
    __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalContextOffset));
    __ ldr(r2, FieldMemOperand(r2, kGlobalIndex));
    __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalReceiverOffset));

    __ bind(&patch_receiver);
    __ lsl(r3, r0, Immediate(kPointerSizeLog2));
    __ add(r3, sp, r3);
    __ str(r2, MemOperand(r3, -kPointerSize));

    __ jmp(&shift_arguments);
  }

  // 3b. Patch the first argument when calling a non-function.  The
  //     CALL_NON_FUNCTION builtin expects the non-function callee as
  //     receiver, so overwrite the first argument which will ultimately
  //     become the receiver.
  // r0: actual number of arguments
  // r1: function
  __ bind(&non_function);
  __ lsl(r2, r0, Immediate(kPointerSizeLog2));
  __ add(r2, sp, r2);
  __ str(r1, MemOperand(r2, -kPointerSize));
  // Clear r1 to indicate a non-function being called.
  __ mov(r1, Immediate(0));

  // 4. Shift arguments and return address one slot down on the stack
  //    (overwriting the original receiver).  Adjust argument count to make
  //    the original first argument the new receiver.
  // r0: actual number of arguments
  // r1: function
  __ bind(&shift_arguments);
  { Label loop;
    // Calculate the copy start address (destination). Copy end address is sp.
    __ lsl(r2, r0, Immediate(kPointerSizeLog2));
    __ add(r2, sp, r2);

    __ bind(&loop);
    __ ldr(r11, MemOperand(r2, -kPointerSize));
    __ str(r11, MemOperand(r2));
    __ sub(r2, r2, Immediate(kPointerSize));
    __ cmpeq(r2, sp);
    __ bf(&loop);
    // Adjust the actual number of arguments and remove the top element
    // (which is a copy of the last argument).
    __ sub(r0, r0, Immediate(1));
    __ add(sp, sp, Immediate(kPointerSize));    // Pop one pointer without saving it
  }

  // 5a. Call non-function via tail call to CALL_NON_FUNCTION builtin.
  // r0: actual number of arguments
  // r1: function
  { Label function;
    __ tst(r1, r1);
    __ bf(&function);
    // Expected number of arguments is 0 for CALL_NON_FUNCTION.
    __ mov(r2, Immediate(0));
    __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION);
    __ Call(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
            RelocInfo::CODE_TARGET);
    __ bind(&function);
  }

  // 5b. Get the code to call from the function and check that the number of
  //     expected arguments matches what we're providing.  If so, jump
  //     (tail-call) to the code in register edx without checking arguments.
  // r0: actual number of arguments
  // r1: function
 Label end;
  __ ldr(r3, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
  __ ldr(r2,
         FieldMemOperand(r3, SharedFunctionInfo::kFormalParameterCountOffset));
  __ asr(r2, r2, Immediate(kSmiTagSize));
  __ ldr(r3, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
  __ cmpeq(r2, r0);  // Check formal and actual parameter counts.
  __ bt(&end);
  __ Call(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
          RelocInfo::CODE_TARGET);
  __ bind(&end);
  ParameterCount expected(0);
  __ InvokeCode(r3, expected, expected, JUMP_FUNCTION);
}


void Builtins::Generate_FunctionApply(MacroAssembler* masm) {
  const int kIndexOffset    = -5 * kPointerSize;
  const int kLimitOffset    = -4 * kPointerSize;
  const int kArgsOffset     =  2 * kPointerSize;
  const int kRecvOffset     =  3 * kPointerSize;
  const int kFunctionOffset =  4 * kPointerSize;

  __ EnterInternalFrame();

  __ ldr(r0, MemOperand(fp, kFunctionOffset));  // get the function
  __ push(r0);
  __ ldr(r0, MemOperand(fp, kArgsOffset));  // get the args array
  __ push(r0);
  __ InvokeBuiltin(Builtins::APPLY_PREPARE, CALL_JS);

  // Check the stack for overflow. We are not trying need to catch
  // interruptions (e.g. debug break and preemption) here, so the "real stack
  // limit" is checked.
  Label okay;
  __ LoadRoot(r2, Heap::kRealStackLimitRootIndex);
  // Make r2 the space we have left. The stack might already be overflowed
  // here which will cause r2 to become negative.
  __ sub(r2, sp, r2);
  // Check if the arguments will overflow the stack.
  __ lsl(r11, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ cmpgt(r2, r11);
  __ bt(&okay);  // Signed comparison.

  // Out of stack space.
  __ ldr(r1, MemOperand(fp, kFunctionOffset));
  __ push(r1);
  __ push(r0);
  __ InvokeBuiltin(Builtins::APPLY_OVERFLOW, CALL_JS);
  // End of stack check.

  // Push current limit and index.
  __ bind(&okay);
  __ push(r0);  // limit
  __ mov(r1, Immediate(0));  // initial index
  __ push(r1);

  // Change context eagerly to get the right global object if necessary.
  __ ldr(r0, MemOperand(fp, kFunctionOffset));
  __ ldr(cp, FieldMemOperand(r0, JSFunction::kContextOffset));
  // Load the shared function info while the function is still in r0.
  __ ldr(r1, FieldMemOperand(r0, JSFunction::kSharedFunctionInfoOffset));

  // Compute the receiver.
  Label call_to_object, use_global_receiver, push_receiver;
  __ ldr(r0, MemOperand(fp, kRecvOffset));

  // Do not transform the receiver for strict mode functions.
  __ ldr(r1, FieldMemOperand(r1, SharedFunctionInfo::kCompilerHintsOffset));
  __ tst(r1, Immediate(1 << (SharedFunctionInfo::kStrictModeFunction +
                             kSmiTagSize)));
  __ bf(&push_receiver);

  // Compute the receiver in non-strict mode.
  __ tst(r0, Immediate(kSmiTagMask));
  __ bt(&call_to_object);
  __ LoadRoot(r1, Heap::kNullValueRootIndex);
  __ cmpeq(r0, r1);
  __ bt(&use_global_receiver);
  __ LoadRoot(r1, Heap::kUndefinedValueRootIndex);
  __ cmpeq(r0, r1);
  __ bt(&use_global_receiver);

  // Check if the receiver is already a JavaScript object.
  // r0: receiver
  __ CompareObjectType(r0, r1, r1, FIRST_JS_OBJECT_TYPE, ge);
  __ bf(&call_to_object);
  __ cmpgt(r1, Immediate(LAST_JS_OBJECT_TYPE));
  __ bf(&push_receiver);

  // Convert the receiver to a regular object.
  // r0: receiver
  __ bind(&call_to_object);
  __ push(r0);
  __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
  __ jmp(&push_receiver);

  // Use the current global receiver object as the receiver.
  __ bind(&use_global_receiver);
  const int kGlobalOffset =
      Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
  __ ldr(r0, FieldMemOperand(cp, kGlobalOffset));
  __ ldr(r0, FieldMemOperand(r0, GlobalObject::kGlobalContextOffset));
  __ ldr(r0, FieldMemOperand(r0, kGlobalOffset));
  __ ldr(r0, FieldMemOperand(r0, GlobalObject::kGlobalReceiverOffset));

  // Push the receiver.
  // r0: receiver
  __ bind(&push_receiver);
  __ push(r0);

  // Copy all arguments from the array to the stack.
  Label entry, loop;
  __ ldr(r0, MemOperand(fp, kIndexOffset));
  __ jmp(&entry);

  // Load the current argument from the arguments array and push it to the
  // stack.
  // r0: current argument index
  __ bind(&loop);
  __ ldr(r1, MemOperand(fp, kArgsOffset));
  __ push(r1);
  __ push(r0);

  // Call the runtime to access the property in the arguments array.
  __ CallRuntime(Runtime::kGetProperty, 2);
  __ push(r0);

  // Use inline caching to access the arguments.
  __ ldr(r0, MemOperand(fp, kIndexOffset));
  __ add(r0, r0, Immediate(1 << kSmiTagSize));
  __ str(r0, MemOperand(fp, kIndexOffset));

  // Test if the copy loop has finished copying all the elements from the
  // arguments object.
  __ bind(&entry);
  __ ldr(r1, MemOperand(fp, kLimitOffset));
  __ cmpeq(r0, r1);
  __ bf(&loop);

  // Invoke the function.
  ParameterCount actual(r0);
  __ asr(r0, r0, Immediate(kSmiTagSize));
  __ ldr(r1, MemOperand(fp, kFunctionOffset));
  __ InvokeFunction(r1, actual, CALL_FUNCTION);

  // Tear down the internal frame and remove function, receiver and args.
  __ LeaveInternalFrame();
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ rts();
}


static void EnterArgumentsAdaptorFrame(MacroAssembler* masm) {
  __ lsl(r0, r0, Immediate(kSmiTagSize));
  __ mov(r4, Immediate(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ Push(r0, r1, r4, fp);
  __ push(pr);
  __ add(fp, sp, Immediate(3 * kPointerSize));
}


static void LeaveArgumentsAdaptorFrame(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0 : result being passed through
  // -----------------------------------
  // Get the number of arguments passed (as a smi), tear down the frame and
  // then tear down the parameters.
  __ ldr(r1, MemOperand(fp, -3 * kPointerSize));
  __ mov(sp, fp);
  __ Pop(pr, fp);
  __ lsl(r11, r1, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(sp, sp, r11);
  __ add(sp, sp, Immediate(kPointerSize));  // adjust for receiver
}


void Builtins::Generate_ArgumentsAdaptorTrampoline(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0 : actual number of arguments
  //  -- r1 : function (passed through to callee)
  //  -- r2 : expected number of arguments
  //  -- r3 : code entry to call
  // -----------------------------------

  Label invoke, dont_adapt_arguments;

  Label enough, too_few;
  __ cmpge(r0, r2);     // actual number of argument lower than expected number
  __ bf(&too_few);
  __ cmpeq(r2, Immediate(SharedFunctionInfo::kDontAdaptArgumentsSentinel));     // FIXME: Immediate or Operand ??
  __ bt(&dont_adapt_arguments);

  {  // Enough parameters: actual >= expected
    __ bind(&enough);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into r0 and copy end address into r2.
    // r0: actual number of arguments as a smi
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    __ lsl(r0, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
    __ add(r0, fp, r0);
    // adjust for return address and receiver
    __ add(r0, r0, Immediate(2 * kPointerSize));
    __ lsl(r2, r2, Immediate(kPointerSizeLog2));
    __ sub(r2, r0, r2);

    // Copy the arguments (including the receiver) to the new stack frame.
    // r0: copy start address
    // r1: function
    // r2: copy end address
    // r3: code entry to call

    Label copy;
    __ bind(&copy);
    __ ldr(r11, MemOperand(r4, 0));
    __ push(r11);
    __ cmpeq(r0, r2);  // Compare before moving to next argument.
    __ sub(r0, r0, Immediate(kPointerSize));
    __ bf(&copy);

    __ jmp(&invoke);
  }

  {  // Too few parameters: Actual < expected
    __ bind(&too_few);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into r0 and copy end address is fp.
    // r0: actual number of arguments as a smi
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    __ lsl(r0, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
    __ add(r0, fp, r0);

    // Copy the arguments (including the receiver) to the new stack frame.
    // r0: copy start address
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    Label copy;
    __ bind(&copy);
    // Adjust load for return address and receiver.
    __ mov(r11, MemOperand(r0, 2 * kPointerSize));
    __ push(r11);
    __ cmpeq(r0, fp);  // Compare before moving to next argument.
    __ sub(r0, r0, Immediate(kPointerSize));
    __ bf(&copy);

    // Fill the remaining expected arguments with undefined.
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    __ LoadRoot(r4, Heap::kUndefinedValueRootIndex);
    __ lsl(r2, r2, Immediate(kPointerSizeLog2));
    __ sub(r2, fp, r2);
    __ sub(r2, r2, Immediate(4 * kPointerSize));  // Adjust for frame.

    Label fill;
    __ bind(&fill);
    __ push(r4);        // kept from the previous load
    __ cmpeq(sp, r2);
    __ bf(&fill);
  }

  // Call the entry point.
  __ bind(&invoke);
  __ jsr(r3);
  // Exit frame and return.
  LeaveArgumentsAdaptorFrame(masm);
  __ rts();


  // -------------------------------------------
  // Dont adapt arguments.
  // -------------------------------------------
  __ bind(&dont_adapt_arguments);
  __ jmp(r3);
}

#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
