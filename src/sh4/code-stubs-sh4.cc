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


void ArgumentsAccessStub::GenerateNewObject(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void ArgumentsAccessStub::GenerateReadElement(MacroAssembler* masm) {
  UNIMPLEMENTED();
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


void CEntryStub::Generate(MacroAssembler* masm) {
  UNIMPLEMENTED();
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

  // Get address of argv
  // r4: code entry
  // r5: function
  // r6: receiver
  // r7: argc
  __ mov(r8, MemOperand(sp, kNumCalleeSaved * kPointerSize)); // r8: argv

  // Push the linkage register on the stack
  __ pushPR();

  // Push a frame with special values setup to mark it as an entry frame.
  // r4: code entry
  // r5: function
  // r6: receiver
  // r7: argc
  // r8: argv
  Isolate* isolate = masm->isolate();
  __ mov(r0, Immediate(-1));       // Push a bad frame pointer to fail if it is used.
  int marker = is_construct ? StackFrame::ENTRY_CONSTRUCT : StackFrame::ENTRY;
  __ mov(r1, Immediate(Smi::FromInt(marker)));
  __ mov(r2, Immediate(Smi::FromInt(marker)));

  __ mov(r3, Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate)));
  __ mov(r3, MemOperand(r3));

  __ push(r3);
  __ push(r2);
  __ push(r1);
  __ push(r0);

  // Setup frame pointer for the frame to be pushed.
  __ add(fp, sp, Immediate(-EntryFrameConstants::kCallerFPOffset));

  // Call a faked try-block that does the invoke.
  __ jmp(&invoke);

  // Caught exception: Store result (exception) in the pending
  // exception field in the JSEnv and return a failure sentinel.
  // Coming in here the fp will be invalid because the PushTryHandler below
  // sets it to 0 to signal the existence of the JSEntry frame.
  __ mov(r1, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                       isolate)));
  __ mov(MemOperand(r1), r0);
  __ mov(r0, Operand(reinterpret_cast<int32_t>(Failure::Exception())));
  __ jmp(&exit);

  // Invoke: Link this frame into the handler chain.
  __ bind(&invoke);
  // Must preserve r4-r8, r0-r3 are available.
  __ PushTryHandler(IN_JS_ENTRY, JS_ENTRY_HANDLER);
  // If an exception not caught by another handler occurs, this handler
  // returns control to the code after the jmp(&invoke) above, which
  // restores all kCalleeSaved registers (including cp and fp) to their
  // saved values before returning a failure to C.

  // Clear any pending exceptions.
  __ mov(r1, Operand(ExternalReference::the_hole_value_location(isolate)));
  __ mov(r2, MemOperand(r1));
  __ mov(r1, Operand(ExternalReference(Isolate::k_pending_exception_address,
                                       isolate)));
  __ mov(MemOperand(r1), r2);

  // Invoke the function by calling through JS entry trampoline builtin.
  // Notice that we cannot store a reference to the trampoline code directly in
  // this stub, because runtime stubs are not traversed when doing GC.

  // Expected registers by Builtins::JSEntryTrampoline
  // r4: code entry
  // r5: function
  // r6: receiver
  // r7: argc
  // r8: argv
  if (is_construct) {
    ExternalReference construct_entry(Builtins::kJSConstructEntryTrampoline,
                                      isolate);
    __ mov(r1, Operand(construct_entry));
  } else {
    ExternalReference entry(Builtins::kJSEntryTrampoline, isolate);
    __ mov(r1, Operand(entry));
  }
  __ mov(r1, MemOperand(r1));  // deref address

  // JSEntryTrampoline
  __ add(r1, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ jsr(r3);

  // Unlink this frame from the handler chain. When reading the
  // address of the next handler, there is no need to use the address
  // displacement since the current stack pointer (sp) points directly
  // to the stack handler.
  __ mov(r3, sp);       // __ mov(r1, MemOperand(sp, StackHandlerConstants::kNextOffset));
  __ add(r3, Immediate(StackHandlerConstants::kNextOffset));
  __ mov(r3, MemOperand(r3));

  __ mov(r2, Operand(ExternalReference(Isolate::k_handler_address, isolate)));
  __ mov(MemOperand(r2), r1);
  // No need to restore registers
  __ add(sp, Immediate(StackHandlerConstants::kSize));

  __ bind(&exit);  // r0 holds result
  // Restore the top frame descriptors from the stack.
  __ pop(r1);
  __ mov(r2,
         Operand(ExternalReference(Isolate::k_c_entry_fp_address, isolate)));
  __ mov(MemOperand(r2), r1);

  // Pop the linkage register from the stack
  __ popPR();

  // Restore callee-saved registers and return.
  __ popm(kCalleeSaved);

  __ rts(); // FIXME(STM): What to put in r0 ?
}


void StackCheckStub::Generate(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
