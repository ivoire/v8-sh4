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
    And(sp, Immediate(-frame_alignment));
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
    And(sp, Immediate(-frame_alignment));
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


void MacroAssembler::Check(const char* msg) {
  Label L;
  // ARM code {
  // b(cond, &L)
  // Abort(msg);
  // }
  // SH4 code {
  bt(&L);
  // TODO: implement Abort()
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
    // TODO: check implementation
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
  // TODO: understand if the following is needed
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

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
