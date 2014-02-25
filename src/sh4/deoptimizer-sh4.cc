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

#include "codegen.h"
#include "deoptimizer.h"
#include "full-codegen.h"
#include "safepoint-table.h"

namespace v8 {
namespace internal {

#include "map-sh4.h"

//  Bailout table entry size: __ mov + __ push + __ b(...,kFar)
const int Deoptimizer::table_entry_size_ = 16; // DIFF: codegen: ref to ::GeneratePrologue

int Deoptimizer::patch_size() {
  //TODO(ivoire): find the right value !
  const int kCallInstructionSizeInWords = 3;
  return kCallInstructionSizeInWords * Assembler::kInstrSize;
}


void Deoptimizer::PatchCodeForDeoptimization(Isolate* isolate, Code* code) {
  UNIMPLEMENTED();
}


void Deoptimizer::FillInputFrame(Address tos, JavaScriptFrame* frame) {
  UNIMPLEMENTED();
}


void Deoptimizer::SetPlatformCompiledStubRegisters( // SAMEAS: arm
    FrameDescription* output_frame, CodeStubInterfaceDescriptor* descriptor) {
  ApiFunction function(descriptor->deoptimization_handler_);
  ExternalReference xref(&function, ExternalReference::BUILTIN_CALL, isolate_);
  intptr_t handler = reinterpret_cast<intptr_t>(xref.address());
  int params = descriptor->environment_length();
  output_frame->SetRegister(r0.code(), params);
  output_frame->SetRegister(r1.code(), handler);
}


void Deoptimizer::CopyDoubleRegisters(FrameDescription* output_frame) {
  // TODO(stm): copy doubles
  // for (int i = 0; i < DwVfpRegister::kMaxNumRegisters; ++i) {
  //   double double_value = input_->GetDoubleRegister(i);
  //   output_frame->SetDoubleRegister(i, double_value);
  // }
}


bool Deoptimizer::HasAlignmentPadding(JSFunction* function) {
  UNIMPLEMENTED();
  return false;
}


#define __ masm()->

// This code tries to be close to ia32 code so that any changes can be
// easily ported.
void Deoptimizer::EntryGenerator::Generate() { // SAMEAS: arm
  GeneratePrologue();

  // Save all general purpose registers before messing with them.
  const int kNumberOfRegisters = Register::kNumRegisters;

  // Everything but pc, lr and sp which will be saved but not restored.
  // SH4: actually all registers but sp (pr and pc are not GP registers on SH4)
  RegList restored_regs = kJSCallerSaved | kCalleeSaved | ip.bit();

  ASSERT(NumRegs(restored_regs) == kNumberOfRegisters - 1/* sp */);

  const int kDoubleRegsSize = 0;
  //    kDoubleSize * DwVfpRegister::kMaxNumAllocatableRegisters;  TODO(stm) : FP regs

  // Save all allocatable VFP registers before messing with them.
  //ASSERT(kDoubleRegZero.code() == 14);  TODO(stm) : FP regs
  //ASSERT(kScratchDoubleReg.code() == 15);  TODO(stm) : FP regs

  // Check CPU flags for number of registers, setting the Z condition flag.
  //__ CheckFor32DRegs(ip); TODO(stm) : FP regs

  // Push registers d0-d13, and possibly d16-d31, on the stack.
  // If d16-d31 are not pushed, decrease the stack pointer instead.
  //__ vstm(db_w, sp, d16, d31, ne);  TODO(stm) : FP regs
  //__ sub(sp, sp, Operand(16 * kDoubleSize), LeaveCC, eq);  TODO(stm) : FP regs
  //__ vstm(db_w, sp, d0, d13);  TODO(stm) : FP regs

  // Push all 16 registers (needed to populate FrameDescription::registers_).
  // TODO(1588) Note that using pc with stm is deprecated, so we should perhaps
  // handle this a bit differently.
  __ push(pr/*pc*/); // fake pc push on SH4 (push pr instead): DIFF: codegen 
  __ push(pr); // push pr: DIFF: codegen
  __ push(sp); // push sp: DIFF: codegen

  __ pushm(restored_regs); // push other regs // DIFF: codegen

  const int kSavedRegistersAreaSize =
    ((kNumberOfRegisters + 2 /* pc and pr */) * kPointerSize) + kDoubleRegsSize;

  // Get the bailout id from the stack.
  __ ldr(sh4_r6/*r2*/, MemOperand(sp, kSavedRegistersAreaSize));  // DIFF: abi: r2 -> sh4_r6

  // Get the address of the location in the code object (r3) (return
  // address for lazy deoptimization) and compute the fp-to-sp delta in
  // register r4. SH4: compute in register sh4_r0 instead of r4.
  __ mov(sh4_r7/*r3*/, lr);  // DIFF: abi: r3 -> sh4_r7
  // Correct one word for bailout id.
  __ add(sh4_r0/*r4*/, sp, Operand(kSavedRegistersAreaSize + (1 * kPointerSize)));  // DIFF: abi: r4 -> sh4_r0
  __ sub(sh4_r0/*r4*/, fp, sh4_r0/*r4*/); // DIFF: abi: r4 -> sh4_r0

  // Allocate a new deoptimizer object.
  // Pass four arguments in r0 to r3 and fifth argument on stack.
  __ PrepareCallCFunction(6, sh4_r1/*r5*/); // DIFF: codegen: use r1 on SH4 as scratch
  __ ldr(sh4_r4/*r0*/, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset)); // DIFF: abi: r0 -> sh4_r4
  __ mov(sh4_r5/*r1*/, Operand(type()));  // bailout type,  // DIFF: abi: r1 -> sh4_r5
  // sh4_r6/*r2*/: bailout id already loaded.  // DIFF: abi: r2 -> sh4_r6
  // sh4_r7/*r3*/: code address or 0 already loaded.  // DIFF: abi: r3 -> sh4_r7
  __ str(sh4_r0/*r4*/, MemOperand(sp, 0 * kPointerSize));  // Fp-to-sp delta. // DIFF: abi: r4 -> sh4_r0
  __ mov(ip, Operand(ExternalReference::isolate_address(isolate())));  // DIFF: abi: use scratch ip
  __ str(ip, MemOperand(sp, 1 * kPointerSize));  // Isolate. // DIFF: abi: use scratch ip
  // Call Deoptimizer::New().
  {
    AllowExternalCallThatCantCauseGC scope(masm());
    __ CallCFunction(ExternalReference::new_deoptimizer_function(isolate()), 6);
  }

  // Preserve "deoptimizer" object in register r0 and get the input
  // frame descriptor pointer to r1 (deoptimizer->input_);
  __ ldr(r1, MemOperand(r0, Deoptimizer::input_offset()));

  // Copy core registers into FrameDescription::registers_[kNumRegisters].
  ASSERT(Register::kNumRegisters == kNumberOfRegisters);
  for (int i = 0; i < kNumberOfRegisters; i++) {
    int offset = (i * kPointerSize) + FrameDescription::registers_offset();
    __ ldr(r2, MemOperand(sp, i * kPointerSize));
    __ str(r2, MemOperand(r1, offset)); // TODO(stm): do we need an ordered list?
  }

  // Copy VFP registers to
  // double_registers_[DoubleRegister::kMaxNumAllocatableRegisters]
  // int double_regs_offset = FrameDescription::double_registers_offset();
  // for (int i = 0; i < DwVfpRegister::kMaxNumAllocatableRegisters; ++i) {
  //   int dst_offset = i * kDoubleSize + double_regs_offset;
  //   int src_offset = i * kDoubleSize + kNumberOfRegisters * kPointerSize;
  //   __ vldr(d0, sp, src_offset);
  //   __ vstr(d0, r1, dst_offset);
  // }

  ASSERT(kSavedRegistersAreaSize ==
         ((kNumberOfRegisters + 2 /* pc and pr */) * kPointerSize)
         + kDoubleRegsSize);

  // Remove the bailout id and the saved registers from the stack.
  __ add(sp, sp, Operand(kSavedRegistersAreaSize + (1 * kPointerSize)));

  // Compute a pointer to the unwinding limit in register r2; that is
  // the first stack slot not part of the input frame.
  __ ldr(r2, MemOperand(r1, FrameDescription::frame_size_offset()));
  __ add(r2, r2, sp);

  // Unwind the stack down to - but not including - the unwinding
  // limit and copy the contents of the activation frame to the input
  // frame description.
  __ add(r3,  r1, Operand(FrameDescription::frame_content_offset()));
  Label pop_loop;
  Label pop_loop_header;
  __ b(&pop_loop_header);
  __ bind(&pop_loop);
  __ pop(r4);
  __ str(r4, MemOperand(r3, 0));
  __ add(r3, r3, Operand(sizeof(uint32_t)));
  __ bind(&pop_loop_header);
  __ cmp(r2, sp);
  __ b(ne, &pop_loop);

  // Compute the output frame in the deoptimizer.
  __ push(r0);  // Preserve deoptimizer object across call.
  // r0: deoptimizer object; r1: scratch.
  __ mov(sh4_r4, r0); // DIFF: codegen: move R0 into SH4 first parameter
  __ PrepareCallCFunction(1, r1);
  // Call Deoptimizer::ComputeOutputFrames().
  {
    AllowExternalCallThatCantCauseGC scope(masm());
    __ CallCFunction(
        ExternalReference::compute_output_frames_function(isolate()), 1);
  }
  __ pop(r0);  // Restore deoptimizer object (class Deoptimizer).

  // Replace the current (input) frame with the output frames.
  Label outer_push_loop, inner_push_loop,
      outer_loop_header, inner_loop_header;
  // Outer loop state: r4 = current "FrameDescription** output_",
  // r1 = one past the last FrameDescription**.
  __ ldr(r1, MemOperand(r0, Deoptimizer::output_count_offset()));
  __ ldr(r4, MemOperand(r0, Deoptimizer::output_offset()));  // r4 is output_.
  __ lsl(r1, r1, Operand(2)); // DIFF: codegen
  __ add(r1, r4, Operand(r1)); // DIFF: codegen
  __ jmp(&outer_loop_header);
  __ bind(&outer_push_loop);
  // Inner loop state: r2 = current FrameDescription*, r3 = loop index.
  __ ldr(r2, MemOperand(r4, 0));  // output_[ix]
  __ ldr(r3, MemOperand(r2, FrameDescription::frame_size_offset()));
  __ jmp(&inner_loop_header);
  __ bind(&inner_push_loop);
  __ sub(r3, r3, Operand(sizeof(uint32_t)));
  __ add(r6, r2, Operand(r3));
  __ ldr(r6, MemOperand(r6, FrameDescription::frame_content_offset()));
  __ push(r6);
  __ bind(&inner_loop_header);
  __ cmp(r3, Operand::Zero());
  __ b(ne, &inner_push_loop);  // test for gt?
  __ add(r4, r4, Operand(kPointerSize));
  __ bind(&outer_loop_header);
  __ cmpge(r4, r1); // DIFF: codegen
  __ b(f, &outer_push_loop); // DIFF: codegen

  // TODO(stm): FP registers
  // // Check CPU flags for number of registers, setting the Z condition flag.
  // __ CheckFor32DRegs(ip);

  // __ ldr(r1, MemOperand(r0, Deoptimizer::input_offset()));
  // int src_offset = FrameDescription::double_registers_offset();
  // for (int i = 0; i < DwVfpRegister::kMaxNumRegisters; ++i) {
  //   if (i == kDoubleRegZero.code()) continue;
  //   if (i == kScratchDoubleReg.code()) continue;

  //   const DwVfpRegister reg = DwVfpRegister::from_code(i);
  //   __ vldr(reg, r1, src_offset, i < 16 ? al : ne);
  //   src_offset += kDoubleSize;
  // }

  // Push state, pc, and continuation from the last output frame.
  __ ldr(r6, MemOperand(r2, FrameDescription::state_offset()));
  __ push(r6);
  __ ldr(r6, MemOperand(r2, FrameDescription::pc_offset()));
  __ push(r6);
  __ ldr(r6, MemOperand(r2, FrameDescription::continuation_offset()));
  __ push(r6);

  // Push the registers from the last output frame.
  for (int i = kNumberOfRegisters - 1; i >= 0; i--) {
    int offset = (i * kPointerSize) + FrameDescription::registers_offset();
    __ ldr(r6, MemOperand(r2, offset));
    __ push(r6);
  }

  // Restore the registers from the stack.
  // SH4: assert that the just pushed registers are restored
  ASSERT(kNumberOfRegisters == NumRegs(restored_regs) + 1 /* sp*/);
  __ popm(restored_regs);  // all but pc registers. // DIFF: codegen
  __ pop(ip);  // remove sp
  //__ pop(ip);  // remove lr // DIFF: codegen: not needed on SH4 (lr not in GPRs)
  //__ pop(ip);  // remove pc // DIFF: codegen: not needed on SH4 (pc not in GPRs)

  __ InitializeRootRegister();

  __ pop(ip);  // get continuation, leave pc on stack
  __ pop(lr);  // return pc put in lr
  __ Jump(ip); // Jump to continuation
  __ stop("Unreachable.");
}


void Deoptimizer::TableEntryGenerator::GeneratePrologue() { // SAMEAS: arm
  // Create a sequence of deoptimization entries.
  // Note that registers are still live when jumping to an entry.
  Label done;
  int table_start = masm()->pc_offset(); // DIFF: add global table size check
  for (int i = 0; i < count(); i++) {
    int start = masm()->pc_offset();
    USE(start);
    __ mov(ip, Operand(i));
    __ push(ip);
    __ b(&done);
    ASSERT(masm()->pc_offset() - start == table_entry_size_);
  }
  ASSERT(masm()->pc_offset() - table_start == table_entry_size_ * count());  // DIFF: add global table size check
  __ bind(&done);
}


void FrameDescription::SetCallerPc(unsigned offset, intptr_t value) { // SAMEAS: arm
  SetFrameSlot(offset, value);
}


void FrameDescription::SetCallerFp(unsigned offset, intptr_t value) { // SAMEAS: arm
  SetFrameSlot(offset, value);
}


#undef __

} }  // namespace v8::internal
