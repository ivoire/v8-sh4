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
  UNIMPLEMENTED();
}

void Builtins::Generate_ArrayConstructCode(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_StringConstructCode(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_JSConstructCall(MacroAssembler* masm) {
  UNIMPLEMENTED();
}

void Builtins::Generate_JSConstructStubCountdown(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_JSConstructStubGeneric(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_JSConstructStubApi(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_JSEntryTrampoline(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_JSConstructEntryTrampoline(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_LazyCompile(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_LazyRecompile(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_NotifyDeoptimized(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_NotifyLazyDeoptimized(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_NotifyOSR(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_OnStackReplacement(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_FunctionCall(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void Builtins::Generate_FunctionApply(MacroAssembler* masm) {
  UNIMPLEMENTED();
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


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_ARM
