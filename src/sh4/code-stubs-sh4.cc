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

#include "code-stubs.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm)


void ArgumentsAccessStub::GenerateReadElement(MacroAssembler* masm) {
  // The displacement is the offset of the last parameter (if any)
  // relative to the frame pointer.
  static const int kDisplacement =
      StandardFrameConstants::kCallerSPOffset - kPointerSize;

  // r0 (when parameter) or r0 (when return value)
  // r1 (when parameter)
  // r2
  // r3

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
  __ cmpeq(r3, Immediate(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ bt(&adaptor);

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

  // Slow-case: Handle non-smi or out-of-bounds access to arguments
  // by calling the runtime system.
  __ bind(&slow);
  // SH4 live-in: r1
  // SH4 live-out: empty
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
  __ mov(MemOperand(sp, 1 * kPointerSize), r0);

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


Handle<Code> GetTypeRecordingBinaryOpStub(int key,
                        TRBinaryOpIC::TypeInfo type_info,
                        TRBinaryOpIC::TypeInfo result_type_info) {
  UNIMPLEMENTED();
}


Register InstanceofStub::left() {
  UNIMPLEMENTED();
}


Register InstanceofStub::right() {
  UNIMPLEMENTED();
}


void CEntryStub::GenerateCore(MacroAssembler* masm,
                              Label* throw_normal_exception,
                              Label* throw_termination_exception,
                              Label* throw_out_of_memory_exception,
                              bool do_gc,
                              bool always_allocate) {
  // WARNING: this function use the SH4 ABI !!

  // r0: result parameter for PerformGC, if any
  // r8: number of arguments including receiver  (C callee-saved)
  // r9: pointer to builtin function  (C callee-saved)
  // r10: pointer to the first argument (C callee-saved)
  Isolate* isolate = masm->isolate();

  if (do_gc) {
    // Passing r0.
    __ PrepareCallCFunction(1, r1);
    __ CallCFunction(ExternalReference::perform_gc_function(isolate), 1);
  }

  ExternalReference scope_depth =
      ExternalReference::heap_always_allocate_scope_depth(isolate);
  if (always_allocate) {
    __ mov(r0, Operand(scope_depth));
    __ mov(r1, MemOperand(r0));
    __ add(r1, Immediate(1));
    __ mov(MemOperand(r0), r1);
  }

  // Call C built-in.
  // r4 = argc, r5 = argv, r6 = isolate
  __ mov(r4, r8);
  __ mov(r5, r10);
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
  __ jsr(r9);
#ifdef DEBUG
  ASSERT(masm->pc_offset() - old_pc == 3 * Assembler::kInstrSize);
#endif

  if (always_allocate) {
    // It's okay to clobber r2 and r3 here. Don't mess with r0 and r1
    // though (contain the result).
    __ mov(r2, Operand(scope_depth));
    __ mov(r3, MemOperand(r2));
    __ sub(r3, r3, Immediate(1));
    __ mov(MemOperand(r2), r3);
  }

  // check for failure result
  Label failure_returned;
  STATIC_ASSERT(((kFailureTag + 1) & kFailureTagMask) == 0);
  // Lower 2 bits of r2 are 0 if r0 has failure tag.
  __ add(r2, r0, Immediate(1));
  __ mov(r3, Immediate(kFailureTagMask));
  __ tst(r2, r3);
  __ bt(&failure_returned);

  // Exit C frame and return.
  // r0:r1: result
  // sp: stack pointer
  // fp: frame pointer
  //  Callee-saved register r8 still holds argc.
  __ LeaveExitFrame(save_doubles_, r8);
  __ rts();

  // check if we should retry or throw exception
  Label retry;
  __ bind(&failure_returned);
  STATIC_ASSERT(Failure::RETRY_AFTER_GC == 0);
  __ tst(r0, Immediate(((1 << kFailureTypeTagSize) - 1) << kFailureTagSize));
  __ bt(&retry);

  // Special handling of out of memory exceptions.
  Failure* out_of_memory = Failure::OutOfMemoryException();
  __ mov(r3, Immediate(reinterpret_cast<int32_t>(out_of_memory)));
  __ cmpeq(r0, r3);
  __ bt(throw_out_of_memory_exception);

  // Retrieve the pending exception and clear the variable.
  __ mov(r3, Operand(ExternalReference::the_hole_value_location(isolate)));
  __ mov(r2, MemOperand(r3));
  __ mov(r3, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                       isolate)));
  __ mov(r0, MemOperand(r3));
  __ mov(MemOperand(r3), r2);

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
  __ lsl(r10, r0, Immediate(kPointerSizeLog2));
  __ add(r10, sp, r10);
  __ sub(r10, r10, Immediate(kPointerSize));

  // Enter the exit frame that transitions from JavaScript to C++.
  __ EnterExitFrame(save_doubles_);

  // Setup argc and the builtin function in callee-saved registers.
  __ mov(r8, r0);
  __ mov(r9, r1);

  // r8: number of arguments (C callee-saved)
  // r9: pointer to builtin function (C callee-saved)
  // r10: pointer to first argument (C callee-saved)

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


bool CEntryStub::NeedsImmovableCode() {
  return true;
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
  __ mov(r8, Immediate(-1));       // Push a bad frame pointer to fail if it is used.
  __ push(r8);

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
  __ mov(r11, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                        isolate)));
  __ str(r0, MemOperand(r11));
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
  __ mov(r11, Operand(ExternalReference::the_hole_value_location(isolate)));
  __ ldr(r5, MemOperand(r11));
  __ mov(r11, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                       isolate)));
  __ str(r5, MemOperand(r11));

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
    __ mov(r11, Operand(construct_entry));
  } else {
    ExternalReference entry(Builtins::kJSEntryTrampoline, isolate);
    __ mov(r11, Operand(entry));
  }
  __ ldr(r11, MemOperand(r11));  // deref address

  // JSEntryTrampoline
  __ add(r11, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ jsr(r11);

  // r0 may hold a result here, do not use it

  // Unlink this frame from the handler chain. When reading the
  // address of the next handler, there is no need to use the address
  // displacement since the current stack pointer (sp) points directly
  // to the stack handler.
  __ ldr(r3, MemOperand(sp, StackHandlerConstants::kNextOffset));
  __ mov(r11, Operand(ExternalReference(Isolate::k_handler_address, isolate)));
  __ str(r3, MemOperand(r11));
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
  __ mov(r11,
         Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate)));
  __ str(r3, MemOperand(r11));

  // Reset the stack to the callee saved registers.
  __ add(sp, sp, Immediate(-EntryFrameConstants::kCallerFPOffset));

  // Pop the linkage register from the stack
  __ pop(pr);

  // Restore callee-saved registers and return.
  __ popm(kCalleeSaved);

  __ rts();
}


void StackCheckStub::Generate(MacroAssembler* masm) {
  __ TailCallRuntime(Runtime::kStackGuard, 0, 1);
}


void FastNewContextStub::Generate(MacroAssembler* masm) {
  UNIMPLEMENTED();
  // // Try to allocate the context in new space.
  // Label gc;
  // int length = slots_ + Context::MIN_CONTEXT_SLOTS;

  // // Attempt to allocate the context in new space.
  // __ AllocateInNewSpace(FixedArray::SizeFor(length),
  //                       r0,
  //                       r1,
  //                       r2,
  //                       &gc,
  //                       TAG_OBJECT);

  // // Load the function from the stack.
  // __ ldr(r3, MemOperand(sp, 0));

  // // Setup the object header.
  // __ LoadRoot(r2, Heap::kContextMapRootIndex);
  // __ str(r2, FieldMemOperand(r0, HeapObject::kMapOffset));
  // __ mov(r2, Operand(Smi::FromInt(length)));
  // __ str(r2, FieldMemOperand(r0, FixedArray::kLengthOffset));

  // // Setup the fixed slots.
  // __ mov(r1, Operand(Smi::FromInt(0)));
  // __ str(r3, MemOperand(r0, Context::SlotOffset(Context::CLOSURE_INDEX)));
  // __ str(r0, MemOperand(r0, Context::SlotOffset(Context::FCONTEXT_INDEX)));
  // __ str(r1, MemOperand(r0, Context::SlotOffset(Context::PREVIOUS_INDEX)));
  // __ str(r1, MemOperand(r0, Context::SlotOffset(Context::EXTENSION_INDEX)));

  // // Copy the global object from the surrounding context.
  // __ ldr(r1, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  // __ str(r1, MemOperand(r0, Context::SlotOffset(Context::GLOBAL_INDEX)));

  // // Initialize the rest of the slots to undefined.
  // __ LoadRoot(r1, Heap::kUndefinedValueRootIndex);
  // for (int i = Context::MIN_CONTEXT_SLOTS; i < length; i++) {
  //   __ str(r1, MemOperand(r0, Context::SlotOffset(i)));
  // }

  // // Remove the on-stack argument and return.
  // __ mov(cp, r0);
  // __ pop();
  // __ Ret();

  // // Need to collect. Call into runtime system.
  // __ bind(&gc);
  // __ TailCallRuntime(Runtime::kNewContext, 1, 1);
}


void CompareStub::Generate(MacroAssembler* masm) {
  UNIMPLEMENTED();
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
  __ mov(r2, Immediate/*TODO:Operand*/(0));
  __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION);
  // r0, r1, r2, r3: arguments to trampoline
  __ Jump(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
          RelocInfo::CODE_TARGET);
}


const char* CompareStub::GetName() {
  UNIMPLEMENTED();
}


int CompareStub::MinorKey() {
  UNIMPLEMENTED();
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


// -------------------------------------------------------------------------
// StringCharFromCodeGenerator

void StringCharCodeAtGenerator::GenerateFast(MacroAssembler* masm) {
  Label flat_string;
  Label ascii_string;
  Label got_char_code;

  // If the receiver is a smi trigger the non-string case.
  __ JumpIfSmi(object_, receiver_not_string_);

  // Fetch the instance type of the receiver into result register.
  __ mov(result_, FieldMemOperand(object_, HeapObject::kMapOffset));
  __ ldrb(result_, FieldMemOperand(result_, Map::kInstanceTypeOffset));
  // If the receiver is not a string trigger the non-string case.
  __ tst(result_, Immediate(kIsNotStringMask));
  __ bf(receiver_not_string_);

  // If the index is non-smi trigger the non-smi case.
  __ JumpIfNotSmi(index_, &index_not_smi_);

  // Put smi-tagged index into scratch register.
  __ mov(scratch_, index_);
  __ bind(&got_smi_index_);

  // Check for index out of range.
  __ ldr(r11, FieldMemOperand(object_, String::kLengthOffset));
  __ cmpgeu(scratch_, r11);
  __ bf(index_out_of_range_);

  // We need special handling for non-flat strings.
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(result_, Immediate(kStringRepresentationMask));
  __ bt(&flat_string);

  // Handle non-flat strings.
  __ tst(result_, Immediate(kIsConsStringMask));
  __ bt(&call_runtime_);

  // ConsString.
  // Check whether the right hand side is the empty string (i.e. if
  // this is really a flat string in a cons string). If that is not
  // the case we would rather go to the runtime system now to flatten
  // the string.
  __ ldr(result_, FieldMemOperand(object_, ConsString::kSecondOffset));
  __ LoadRoot(r11, Heap::kEmptyStringRootIndex);
  __ cmpeq(result_, r11);
  __ bf(&call_runtime_);
  // Get the first of the two strings and load its instance type.
  __ ldr(object_, FieldMemOperand(object_, ConsString::kFirstOffset));
  __ ldr(result_, FieldMemOperand(object_, HeapObject::kMapOffset));
  __ ldrb(result_, FieldMemOperand(result_, Map::kInstanceTypeOffset));
  // If the first cons component is also non-flat, then go to runtime.
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(result_, Immediate(kStringRepresentationMask));
  __ bf(&call_runtime_);

  // Check for 1-byte or 2-byte string.
  __ bind(&flat_string);
  STATIC_ASSERT(kAsciiStringTag != 0);
  __ tst(result_, Immediate(kStringEncodingMask));
  __ bf(&ascii_string);

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

  ASSERT(!code_.is(r11) && !result_.is(r11));

  __ tst(code_,
         Immediate(kSmiTagMask |
                 ((~String::kMaxAsciiCharCode) << kSmiTagSize)));
  __ bf(&slow_case_);

  __ LoadRoot(result_, Heap::kSingleCharacterStringCacheRootIndex);
  // At this point code register contains smi tagged ASCII char code.
  STATIC_ASSERT(kSmiTag == 0);
  __ lsl(r11, code_, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(result_, result_, r11);
  __ mov(result_, FieldMemOperand(result_, FixedArray::kHeaderSize));
  __ LoadRoot(r11, Heap::kUndefinedValueRootIndex);
  __ cmpeq(result_, r11);
  __ bt(&slow_case_);
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




#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
