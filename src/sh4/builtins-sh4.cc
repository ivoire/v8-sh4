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


#ifdef DEBUG
#define RECORD_LINE() ACCESS_MASM(masm) RecordFunctionLine(__FUNCTION__, __LINE__)
#define __ RECORD_LINE(); ACCESS_MASM(masm)
#else
#define RECORD_LINE() ((void)0)
#define __ ACCESS_MASM(masm)
#endif



void Builtins::Generate_Adaptor(MacroAssembler* masm,
                                CFunctionId id,
                                BuiltinExtraArguments extra_args) {
  // ----------- S t a t e -------------
  //  -- r0                 : number of arguments excluding receiver
  //  -- r1                 : called function (only guaranteed when
  //                          extra_args requires it)
  //  -- cp                 : context FIXME(STM) does CP exist and what does it mean here ?
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
  //  -- r4     : number of arguments
  //  -- r5     : constructor function
  //  -- pr     : return address
  //  -- sp[...]: constructor arguments
  // -----------------------------------
  Label non_function_call;

  // Check that the function is not a smi.
  __ tst(r5, Immediate(kSmiTagMask));
  __ bt(&non_function_call);
  // Check that the function is a JSFunction.
  __ CompareObjectType(r5, r2, r2, JS_FUNCTION_TYPE);   // r2 is a temp register
  __ bf(&non_function_call);

  // Jump to the function-specific construct stub.
  __ mov(r3, FieldMemOperand(r5, JSFunction::kSharedFunctionInfoOffset));
  __ mov(r2, FieldMemOperand(r2, SharedFunctionInfo::kConstructStubOffset));
  __ add(r3, r2, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ jsr(r3);

  // r4: number of arguments
  // r5: called object
  __ bind(&non_function_call);
  // Set expected number of arguments to zero (not changing r0).
  __ mov(r4, Immediate(0));
  __ GetBuiltinEntry(r6, Builtins::CALL_NON_FUNCTION_AS_CONSTRUCTOR);
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
  // r0 -> r4: code entry
  // r1 -> r5: function
  // r2 -> r6: receiver
  // r3 -> r7: argc
  // r4 -> r8: argv (JSEntryStub does set it)
  // r0-r2, cp may be clobbered

  // Clear the context before we push it when entering the JS frame.
  __ mov(cp, Immediate(0));

  // Enter an internal frame.
  __ EnterInternalFrame();

  // Set up the context from the function argument.
  __ mov(cp, FieldMemOperand(r5, JSFunction::kContextOffset));

  // Set up the roots register.
  ExternalReference roots_address =
      ExternalReference::roots_address(masm->isolate());
  __ mov(roots, Operand(roots_address));

  // Push the function and the receiver onto the stack.
  __ push(r5);
  __ push(r6);

  // Copy arguments to the stack in a loop.
  // r1 -> r5: function
  // r3 -> r7: argc
  // r4 -> r8: argv, i.e. points to first arg
  Label loop, entry;
  __ lsl(r3, r7, Immediate(kPointerSizeLog2));
  __ add(r6, r8, r3);
  // r6 points past last arg.
  __ jmp(&entry);
  __ bind(&loop);
    __ mov(r1, MemOperand(r8, kPointerSize));     // read next parameter
    __ add(r8, r8, Immediate(kPointerSize));      // mov the r8 pointer onto the next parameter
    __ mov(r1, MemOperand(r1));  // dereference handle
    __ push(r1);  // push parameter
  __ bind(&entry);
    __ cmpeq(r8, r6);
  __ bf(&loop);

  // Initialize all JavaScript callee-saved registers, since they will be seen
  // by the garbage collector as part of handlers.
  __ LoadRoot(r8, Heap::kUndefinedValueRootIndex);
  __ mov(r9, r8);
  __ mov(r10, r8);

  // Invoke the code and pass argc as r4.
  __ mov(r4, Operand(r7));
  if (is_construct) {
    __ Call(masm->isolate()->builtins()->JSConstructCall(),
            RelocInfo::CODE_TARGET);
  } else {
    ParameterCount actual(r4);
    __ InvokeFunction(r5, actual, CALL_FUNCTION);
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
  __ push(r5);

  // Push the function on the stack as the argument to the runtime function.
  __ push(r5);
  __ CallRuntime(fid, 1);
  // Calculate the entry point.
  __ add(r6, r4, Immediate(Code::kHeaderSize - kHeapObjectTag));
  // Restore saved function.
  __ pop(r5);

  // Tear down temporary frame.
  __ LeaveInternalFrame();

  // Do a tail-call of the compiled function.
  __ jsr(r6);
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

  // Get the full codegen state from the stack and untag it -> r1.
  __ mov(r1, MemOperand(sp, 0 * kPointerSize));
  __ SmiUntag(r1);
  // Switch on the state.
  Label with_tos_register, unknown_state;
  __ mov(r3, Immediate(FullCodeGenerator::NO_REGISTERS));
  __ cmpeq(r1, r3);
  __ bf(&with_tos_register);
  __ add(sp, sp, Immediate(1 * kPointerSize));  // Remove state.
  __ rts();

  __ bind(&with_tos_register);
  __ mov(r0, MemOperand(sp, 1 * kPointerSize));
  __ mov(r3, Immediate(FullCodeGenerator::TOS_REG));
  __ cmpeq(r1, r3);
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
  // r4: actual number of arguments
  { Label done;
    __ tst(r4, r4);     // FIXME ????
    __ bf(&done);
    __ LoadRoot(r6, Heap::kUndefinedValueRootIndex);
    __ push(r6);
    __ add(r4, r4, Immediate(1));
    __ bind(&done);
  }

  // 2. Get the function to call (passed as receiver) from the stack, check
  //    if it is a function.
  // r4: actual number of arguments
  Label non_function;
  __ lsl(r3, r4, Immediate(kPointerSizeLog2));
  __ mov(r5, MemOperand(sp, r3));
  __ tst(r5, Immediate(kSmiTagMask));
  __ bt(&non_function);
  __ CompareObjectType(r5, r6, r6, JS_FUNCTION_TYPE);
  __ bf(&non_function);

  // 3a. Patch the first argument if necessary when calling a function.
  // r4: actual number of arguments
  // r5: function
  Label shift_arguments;
  { Label convert_to_object, use_global_receiver, patch_receiver;
    // Change context eagerly in case we need the global receiver.
    __ mov(cp, FieldMemOperand(r5, JSFunction::kContextOffset));

    // Do not transform the receiver for strict mode functions.
    __ mov(r6, FieldMemOperand(r5, JSFunction::kSharedFunctionInfoOffset));
    __ mov(r6, FieldMemOperand(r6, SharedFunctionInfo::kCompilerHintsOffset));
    __ tst(r6, Immediate(1 << (SharedFunctionInfo::kStrictModeFunction +
                               kSmiTagSize)));
    __ bf(&shift_arguments);

    // Compute the receiver in non-strict mode.
    __ lsl(r3, r4, Immediate(kPointerSizeLog2));
    __ add(r6, sp, r3);
    __ mov(r6, MemOperand(r6, -kPointerSize));
    // r4: actual number of arguments
    // r5: function
    // r6: first argument
    __ tst(r6, Immediate(kSmiTagMask));
    __ bt(&convert_to_object);

    __ LoadRoot(r7, Heap::kNullValueRootIndex);
    __ cmpeq(r6, r7);
    __ bt(&use_global_receiver);
    __ LoadRoot(r7, Heap::kUndefinedValueRootIndex);
    __ cmpeq(r6, r7);
    __ bt(&use_global_receiver);

    __ CompareObjectType(r6, r7, r7, FIRST_JS_OBJECT_TYPE, ge);
    __ bf(&convert_to_object);
    __ mov(r3, Immediate(LAST_JS_OBJECT_TYPE));
    __ cmpge(r3, r7);
    __ bt(&shift_arguments);

    __ bind(&convert_to_object);
    __ EnterInternalFrame();  // In order to preserve argument count.
    __ lsl(r4, r4, Immediate(kSmiTagSize)); // Smi-tagged.
    __ push(r4);

    __ push(r6);
    __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
    __ mov(r6, r4);

    __ pop(r4);
    __ asr(r4, r4, Immediate(kSmiTagSize));
    __ LeaveInternalFrame();
    // Restore the function to r5.
    __ lsl(r3, r0, Immediate(kPointerSizeLog2));
    __ mov(r5, MemOperand(sp, r3));
    __ jmp(&patch_receiver);

    // Use the global receiver object from the called function as the
    // receiver.
    __ bind(&use_global_receiver);
    const int kGlobalIndex =
        Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
    __ mov(r6, FieldMemOperand(cp, kGlobalIndex));
    __ mov(r6, FieldMemOperand(r6, GlobalObject::kGlobalContextOffset));
    __ mov(r6, FieldMemOperand(r6, kGlobalIndex));
    __ mov(r6, FieldMemOperand(r6, GlobalObject::kGlobalReceiverOffset));

    __ bind(&patch_receiver);
    __ lsl(r3, r0, Immediate(kPointerSizeLog2));
    __ add(r7, sp, r3);
    __ mov(MemOperand(r7, -kPointerSize), r6);

    __ jmp(&shift_arguments);
  }

  // 3b. Patch the first argument when calling a non-function.  The
  //     CALL_NON_FUNCTION builtin expects the non-function callee as
  //     receiver, so overwrite the first argument which will ultimately
  //     become the receiver.
  // r4: actual number of arguments
  // r5: function
  __ bind(&non_function);
  __ lsl(r3, r0, Immediate(kPointerSizeLog2));
  __ add(r6, sp, r3);
  __ mov(MemOperand(r6, -kPointerSize), r5);
  // Clear r5 to indicate a non-function being called.
  __ mov(r5, Immediate(0));

  // 4. Shift arguments and return address one slot down on the stack
  //    (overwriting the original receiver).  Adjust argument count to make
  //    the original first argument the new receiver.
  // r4: actual number of arguments
  // r5: function
  __ bind(&shift_arguments);
  { Label loop;
    // Calculate the copy start address (destination). Copy end address is sp.
    __ lsl(r3, r0, Immediate(kPointerSizeLog2));
    __ add(r6, sp, r3);

    __ bind(&loop);
    __ mov(r3, MemOperand(r6, -kPointerSize));
    __ mov(MemOperand(r6), r3);
    __ sub(r6, r6, Immediate(kPointerSize));
    __ cmpeq(r6, sp);
    __ bf(&loop);
    // Adjust the actual number of arguments and remove the top element
    // (which is a copy of the last argument).
    __ sub(r0, r0, Immediate(1));
    __ add(sp, sp, Immediate(kPointerSize));    // Pop one pointer without saving it
  }

  // 5a. Call non-function via tail call to CALL_NON_FUNCTION builtin.
  // r4: actual number of arguments
  // r5: function
  { Label function;
    __ tst(r5, r5);
    __ bf(&function);
    // Expected number of arguments is 0 for CALL_NON_FUNCTION.
    __ mov(r6, Immediate(0));
    __ GetBuiltinEntry(r7, Builtins::CALL_NON_FUNCTION);
    __ Call(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
            RelocInfo::CODE_TARGET);
    __ bind(&function);
  }

  // 5b. Get the code to call from the function and check that the number of
  //     expected arguments matches what we're providing.  If so, jump
  //     (tail-call) to the code in register edx without checking arguments.
  // r4: actual number of arguments
  // r5: function
 Label end;
  __ mov(r7, FieldMemOperand(r5, JSFunction::kSharedFunctionInfoOffset));
  __ mov(r6,
         FieldMemOperand(r7, SharedFunctionInfo::kFormalParameterCountOffset));
  __ asr(r6, r6, Immediate(kSmiTagSize));
  __ mov(r7, FieldMemOperand(r5, JSFunction::kCodeEntryOffset));
  __ cmpeq(r6, r4);  // Check formal and actual parameter counts.
  __ bt(&end);
  __ Call(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
          RelocInfo::CODE_TARGET);
  __ bind(&end);
  ParameterCount expected(0);
  __ InvokeCode(r7, expected, expected, JUMP_FUNCTION);
}


void Builtins::Generate_FunctionApply(MacroAssembler* masm) {
  const int kIndexOffset    = -5 * kPointerSize;
  const int kLimitOffset    = -4 * kPointerSize;
  const int kArgsOffset     =  2 * kPointerSize;
  const int kRecvOffset     =  3 * kPointerSize;
  const int kFunctionOffset =  4 * kPointerSize;

  __ EnterInternalFrame();

  __ mov(r4, MemOperand(fp, kFunctionOffset));  // get the function
  __ push(r4);
  __ mov(r4, MemOperand(fp, kArgsOffset));  // get the args array
  __ push(r4);
  __ InvokeBuiltin(Builtins::APPLY_PREPARE, CALL_JS);

  // Check the stack for overflow. We are not trying need to catch
  // interruptions (e.g. debug break and preemption) here, so the "real stack
  // limit" is checked.
  Label okay;
  __ LoadRoot(r6, Heap::kRealStackLimitRootIndex);
  // Make r6 the space we have left. The stack might already be overflowed
  // here which will cause r6 to become negative.
  __ sub(r6, sp, r6);
  // Check if the arguments will overflow the stack.
  __ lsl(r3, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ cmpgt(r6, r3);
  __ bt(&okay);  // Signed comparison.

  // Out of stack space.
  __ mov(r5, MemOperand(fp, kFunctionOffset));
  __ push(r5);
  __ push(r4);
  __ InvokeBuiltin(Builtins::APPLY_OVERFLOW, CALL_JS);
  // End of stack check.

  // Push current limit and index.
  __ bind(&okay);
  __ push(r4);  // limit
  __ mov(r5, Immediate(0));  // initial index
  __ push(r4);

  // Change context eagerly to get the right global object if necessary.
  __ mov(r4, MemOperand(fp, kFunctionOffset));
  __ mov(cp, FieldMemOperand(r4, JSFunction::kContextOffset));
  // Load the shared function info while the function is still in r4.
  __ mov(r5, FieldMemOperand(r4, JSFunction::kSharedFunctionInfoOffset));

  // Compute the receiver.
  Label call_to_object, use_global_receiver, push_receiver;
  __ mov(r4, MemOperand(fp, kRecvOffset));

  // Do not transform the receiver for strict mode functions.
  __ mov(r5, FieldMemOperand(r5, SharedFunctionInfo::kCompilerHintsOffset));
  __ tst(r5, Immediate(1 << (SharedFunctionInfo::kStrictModeFunction +
                             kSmiTagSize)));
  __ bf(&push_receiver);

  // Compute the receiver in non-strict mode.
  __ tst(r4, Immediate(kSmiTagMask));
  __ bt(&call_to_object);
  __ LoadRoot(r5, Heap::kNullValueRootIndex);
  __ cmpeq(r4, r5);
  __ bt(&use_global_receiver);
  __ LoadRoot(r5, Heap::kUndefinedValueRootIndex);
  __ cmpeq(r4, r5);
  __ bt(&use_global_receiver);

  // Check if the receiver is already a JavaScript object.
  // r4: receiver
  __ CompareObjectType(r4, r5, r5, FIRST_JS_OBJECT_TYPE, ge);
  __ bf(&call_to_object);
  __ mov(r3, Immediate(LAST_JS_OBJECT_TYPE));
  __ cmpgt(r4, r3);
  __ bf(&push_receiver);

  // Convert the receiver to a regular object.
  // r4: receiver
  __ bind(&call_to_object);
  __ push(r4);
  __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
  __ jmp(&push_receiver);

  // Use the current global receiver object as the receiver.
  __ bind(&use_global_receiver);
  const int kGlobalOffset =
      Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
  __ mov(r4, FieldMemOperand(cp, kGlobalOffset));
  __ mov(r4, FieldMemOperand(r4, GlobalObject::kGlobalContextOffset));
  __ mov(r4, FieldMemOperand(r4, kGlobalOffset));
  __ mov(r4, FieldMemOperand(r4, GlobalObject::kGlobalReceiverOffset));

  // Push the receiver.
  // r4: receiver
  __ bind(&push_receiver);
  __ push(r4);

  // Copy all arguments from the array to the stack.
  Label entry, loop;
  __ mov(r4, MemOperand(fp, kIndexOffset));
  __ jmp(&entry);

  // Load the current argument from the arguments array and push it to the
  // stack.
  // r4: current argument index
  __ bind(&loop);
  __ mov(r5, MemOperand(fp, kArgsOffset));
  __ push(r5);
  __ push(r4);

  // Call the runtime to access the property in the arguments array.
  __ CallRuntime(Runtime::kGetProperty, 2);
  __ push(r4);

  // Use inline caching to access the arguments.
  __ mov(r4, MemOperand(fp, kIndexOffset));
  __ add(r4, r4, Immediate(1 << kSmiTagSize));
  __ mov(MemOperand(fp, kIndexOffset), r4);

  // Test if the copy loop has finished copying all the elements from the
  // arguments object.
  __ bind(&entry);
  __ mov(r5, MemOperand(fp, kLimitOffset));
  __ cmpeq(r4, r5);
  __ bf(&loop);

  // Invoke the function.
  ParameterCount actual(r4);
  __ asr(r4, r4, Immediate(kSmiTagSize));
  __ mov(r5, MemOperand(fp, kFunctionOffset));
  __ InvokeFunction(r5, actual, CALL_FUNCTION);

  // Tear down the internal frame and remove function, receiver and args.
  __ LeaveInternalFrame();
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ rts();
}


static void EnterArgumentsAdaptorFrame(MacroAssembler* masm) {
  __ lsl(r4, r4, Immediate(kSmiTagSize));
  __ mov(r3, Immediate(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ Push(r4, r5, r3, fp);
  __ push(pr);
  __ add(fp, sp, Immediate(3 * kPointerSize));
}


static void LeaveArgumentsAdaptorFrame(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0 : result being passed through
  // -----------------------------------
  // Get the number of arguments passed (as a smi), tear down the frame and
  // then tear down the parameters.
  __ mov(r1, MemOperand(fp, -3 * kPointerSize));
  __ mov(sp, fp);
  __ Pop(pr, fp);
  __ lsl(r1, r1, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(sp, sp, r1);
  __ add(sp, sp, Immediate(kPointerSize));  // adjust for receiver
}


void Builtins::Generate_ArgumentsAdaptorTrampoline(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r4 : actual number of arguments
  //  -- r5 : function (passed through to callee)
  //  -- r6 : expected number of arguments
  //  -- r7 : code entry to call
  // -----------------------------------
  // arm -> sh4
  // r0 -> r4
  // r1 -> r5
  // r2 -> r6
  // r3 -> r7

  Label invoke, dont_adapt_arguments;

  Label enough, too_few;
  __ cmpge(r4, r6);     // actual number of argument lower than expected number
  __ bf(&too_few);
  __ mov(r3, Immediate(SharedFunctionInfo::kDontAdaptArgumentsSentinel));
  __ cmpeq(r6, r3);
  __ bt(&dont_adapt_arguments);

  {  // Enough parameters: actual >= expected
    __ bind(&enough);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into r0 and copy end address into r2.
    // r4: actual number of arguments as a smi
    // r5: function
    // r6: expected number of arguments
    // r7: code entry to call
    __ lsl(r4, r4, Immediate(kPointerSizeLog2 - kSmiTagSize));
    __ add(r4, fp, r4);
    // adjust for return address and receiver
    __ add(r4, r4, Immediate(2 * kPointerSize));
    __ lsl(r6, r6, Immediate(kPointerSizeLog2));
    __ sub(r6, r4, r6);

    // Copy the arguments (including the receiver) to the new stack frame.
    // r4: copy start address
    // r5: function
    // r6: copy end address
    // r7: code entry to call

    Label copy;
    __ bind(&copy);
    __ mov(r3, MemOperand(r4, 0));
    __ push(r3);
    __ cmpeq(r4, r6);  // Compare before moving to next argument.
    __ sub(r4, r4, Immediate(kPointerSize));
    __ bf(&copy);

    __ jmp(&invoke);
  }

  {  // Too few parameters: Actual < expected
    __ bind(&too_few);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into r0 and copy end address is fp.
    // r4: actual number of arguments as a smi
    // r5: function
    // r6: expected number of arguments
    // r7: code entry to call
    __ lsl(r4, r4, Immediate(kPointerSizeLog2 - kSmiTagSize));
    __ add(r4, fp, r4);

    // Copy the arguments (including the receiver) to the new stack frame.
    // r4: copy start address
    // r5: function
    // r6: expected number of arguments
    // r7: code entry to call
    Label copy;
    __ bind(&copy);
    // Adjust load for return address and receiver.
    __ mov(r2, MemOperand(r4, 2 * kPointerSize));
    __ push(r2);
    __ cmpeq(r4, fp);  // Compare before moving to next argument.
    __ sub(r4, r4, Immediate(kPointerSize));
    __ bf(&copy);

    // Fill the remaining expected arguments with undefined.
    // r5: function
    // r6: expected number of arguments
    // r7: code entry to call
    __ LoadRoot(r3, Heap::kUndefinedValueRootIndex);
    __ lsl(r6, r6, Immediate(kPointerSizeLog2));
    __ sub(r6, fp, r6);
    __ sub(r6, r6, Immediate(4 * kPointerSize));  // Adjust for frame.

    Label fill;
    __ bind(&fill);
    __ push(r2);        // kept from the previous load
    __ cmpeq(sp, r6);
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
  __ mov(r0, r7);
  __ rts();
}

#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_ARM
