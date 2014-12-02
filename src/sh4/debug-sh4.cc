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

#if V8_TARGET_ARCH_SH4

#include "codegen.h"
#include "debug.h"

namespace v8 {
namespace internal {

// TODO(stm): remove SH4 check in cctest/test-debug.cc when debug support is implemented there
bool BreakLocationIterator::IsDebugBreakAtReturn() { // SAMEAS: arm
  return Debug::IsDebugBreakAtReturn(rinfo());
}

void BreakLocationIterator::SetDebugBreakAtReturn() { // SAMEAS: arm
  // Patch the code changing the return from JS function sequence from
  // Ref to FullCodeGenerator::EmitReturnSequence():
  //   mov fp, sp
  //   pop fp
  //   pop pr
  //   add #4, sp
  //   rts
  //   nop
  //   ...
  //   nop (padding nops)
  // to a call to the debug break at return code through a standard call sequence:
  // mov(ip, address)
  // jsr(ip)
  //
  // Note that actually this sequence is the same than the ::SetDebugBreakAtSlot()
  // and the FullGenerator::CallIC() such that the offsets from the return address
  // is the same for the three case.
  CodePatcher patcher(rinfo()->pc(), Assembler::kJSReturnSequenceInstructions);
  ASSERT((uintptr_t)rinfo()->pc() % 4 == 0); // Must be aligned: ref RecordJSReturn()
  patcher.masm()->mov(sh4_ip,
                      Operand(reinterpret_cast<int32_t>(debug_info_->GetIsolate()->builtins()->Return_DebugBreak()->entry()))); // Pass as int32_t to avoid emittion of relocation
  patcher.masm()->jsr(sh4_ip);
  ASSERT(patcher.masm()->pc_offset() - Assembler::kPatchDebugBreakSlotReturnOffset == 0);
  ASSERT(Assembler::kJSReturnSequenceInstructions * Assembler::kInstrSize ==
         patcher.masm()->pc_offset());
  ASSERT(Assembler::target_address_at(reinterpret_cast<byte*>(rinfo()->pc() + Assembler::kPatchReturnSequenceAddressOffset), (ConstantPoolArray* )NULL) == debug_info_->GetIsolate()->builtins()->Return_DebugBreak()->entry());
  ASSERT(patcher.masm()->CallSize(debug_info_->GetIsolate()->builtins()->Return_DebugBreak()->entry(), 0, RelocInfo::NONE32) == patcher.masm()->pc_offset());
}


// Restore the JS frame exit code.
void BreakLocationIterator::ClearDebugBreakAtReturn() { // SAMEAS: arm
  rinfo()->PatchCode(original_rinfo()->pc(),
                     Assembler::kJSReturnSequenceInstructions);
}


// A debug break in the frame exit code is identified by the JS frame exit code
// having been patched with a call instruction.
bool Debug::IsDebugBreakAtReturn(RelocInfo* rinfo) { // SAMEAS: arm
  ASSERT(RelocInfo::IsJSReturn(rinfo->rmode()));
  return rinfo->IsPatchedReturnSequence();
}


bool BreakLocationIterator::IsDebugBreakAtSlot() { // SAMEAS: arm
  ASSERT(IsDebugBreakSlot());
  // Check whether the debug break slot instructions have been patched.
  return rinfo()->IsPatchedDebugBreakSlotSequence();
}


void BreakLocationIterator::SetDebugBreakAtSlot() { // SAMEAS: arm
  ASSERT(IsDebugBreakSlot());
  // Patch the code changing the debug break slot code from
  //   mov r2, r2
  //   .... Assembler::kDebugBreakSlotInstructions times
  // to a call to the debug break slot code through a standard call sequence:
  // mov(ip, address)
  // jsr(ip)
  //
  // Note that actually this sequence is the same than the ::SetDebugBreakAtReturn()
  // and the FullGenerator::CallIC() such that the offsets from the return address
  // is the same for the three case.
  CodePatcher patcher(rinfo()->pc(), Assembler::kDebugBreakSlotInstructions);
  ASSERT((uintptr_t)rinfo()->pc() % 4 == 0); // Must be aligned: ref RecordJSReturn()
  patcher.masm()->mov(sh4_ip,
                      Operand(reinterpret_cast<int32_t>(debug_info_->GetIsolate()->builtins()->Slot_DebugBreak()->entry()))); // Pass as int32_t to avoid emittion of relocation
  patcher.masm()->jsr(sh4_ip);
  ASSERT(patcher.masm()->pc_offset() - Assembler::kPatchDebugBreakSlotReturnOffset == 0);
  ASSERT(Assembler::kDebugBreakSlotInstructions * Assembler::kInstrSize ==
         patcher.masm()->pc_offset());
  ASSERT(Assembler::target_address_at(reinterpret_cast<byte*>(rinfo()->pc() + Assembler::kPatchDebugBreakSlotAddressOffset), (ConstantPoolArray*)NULL) == debug_info_->GetIsolate()->builtins()->Slot_DebugBreak()->entry());
  ASSERT(patcher.masm()->CallSize(debug_info_->GetIsolate()->builtins()->Slot_DebugBreak()->entry(), 0, RelocInfo::NONE32) == patcher.masm()->pc_offset());
}



void BreakLocationIterator::ClearDebugBreakAtSlot() { // SAMEAS: arm
  ASSERT(IsDebugBreakSlot());
  rinfo()->PatchCode(original_rinfo()->pc(),
                     Assembler::kDebugBreakSlotInstructions);
}

const bool Debug::FramePaddingLayout::kIsSupported = false;


#define __ ACCESS_MASM(masm)


static void Generate_DebugBreakCallHelper(MacroAssembler* masm, // SAMEAS: arm
                                          RegList object_regs,
                                          RegList non_object_regs) {
  {
    FrameScope scope(masm, StackFrame::INTERNAL);

    // Store the registers containing live values on the expression stack to
    // make sure that these are correctly updated during GC. Non object values
    // are stored as a smi causing it to be untouched by GC.
    ASSERT((object_regs & ~kJSCallerSaved) == 0);
    ASSERT((non_object_regs & ~kJSCallerSaved) == 0);
    ASSERT((object_regs & non_object_regs) == 0);
    if ((object_regs | non_object_regs) != 0) {
      for (int i = 0; i < kNumJSCallerSaved; i++) {
        int r = JSCallerSavedCode(i);
        Register reg = { r };
        if ((non_object_regs & (1 << r)) != 0) {
          if (FLAG_debug_code) {
            __ tst(reg, Operand(0xc0000000));
            __ Assert(eq, kUnableToEncodeValueAsSmi);
          }
          __ SmiTag(reg);
        }
      }
      __ pushm(object_regs | non_object_regs); // DIFF: codegen
    }

#ifdef DEBUG
    __ RecordComment("// Calling from debug break to runtime - come in - over");
#endif
    __ mov(r0, Operand::Zero());  // no arguments
    __ mov(r1, Operand(ExternalReference::debug_break(masm->isolate())));

    CEntryStub ceb(masm->isolate(),1);
    __ CallStub(&ceb);

    // Restore the register values from the expression stack.
    if ((object_regs | non_object_regs) != 0) {
      __ popm(object_regs | non_object_regs); // DIFF: codegen
      for (int i = 0; i < kNumJSCallerSaved; i++) {
        int r = JSCallerSavedCode(i);
        Register reg = { r };
        if ((non_object_regs & (1 << r)) != 0) {
          __ SmiUntag(reg);
        }
        if (FLAG_debug_code &&
            (((object_regs |non_object_regs) & (1 << r)) == 0)) {
          __ mov(reg, Operand(kDebugZapValue));
        }
      }
    }

    // Leave the internal frame.
  }

  // Now that the break point has been handled, resume normal execution by
  // jumping to the target address intended by the caller and that was
  // overwritten by the address of DebugBreakXXX.
  ExternalReference after_break_target =
      ExternalReference(Debug_Address::AfterBreakTarget(), masm->isolate());
  __ mov(sh4_ip, Operand(after_break_target));
  __ ldr(sh4_ip, MemOperand(sh4_ip));
  __ Jump(sh4_ip);
}

void Debug::GenerateCallICStubDebugBreak(MacroAssembler* masm) {
  // Register state for CallICStub
  // ----------- S t a t e -------------
  //  -- r1 : function
  //  -- r3 : slot in feedback array (smi)
  // -----------------------------------
  Generate_DebugBreakCallHelper(masm, r1.bit() | r3.bit(), 0);
}

void Debug::GenerateLoadICDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // Calling convention for IC load (from ic-arm.cc).
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  //  -- r0    : receiver
  //  -- [sp]  : receiver
  // -----------------------------------
  // Registers r0 and r2 contain objects that need to be pushed on the
  // expression stack of the fake JS frame.
  Generate_DebugBreakCallHelper(masm, r0.bit() | r2.bit(), 0);
}


void Debug::GenerateStoreICDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // Calling convention for IC store (from ic-arm.cc).
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  // Registers r0, r1, and r2 contain objects that need to be pushed on the
  // expression stack of the fake JS frame.
  Generate_DebugBreakCallHelper(masm, r0.bit() | r1.bit() | r2.bit(), 0);
}


void Debug::GenerateKeyedLoadICDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // ---------- S t a t e --------------
  //  -- lr     : return address
  //  -- r0     : key
  //  -- r1     : receiver
  Generate_DebugBreakCallHelper(masm, r0.bit() | r1.bit(), 0);
}


void Debug::GenerateKeyedStoreICDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // ---------- S t a t e --------------
  //  -- r0     : value
  //  -- r1     : key
  //  -- r2     : receiver
  //  -- lr     : return address
  Generate_DebugBreakCallHelper(masm, r0.bit() | r1.bit() | r2.bit(), 0);
}


void Debug::GenerateCompareNilICDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // Register state for CompareNil IC
  // ----------- S t a t e -------------
  //  -- r0    : value
  // -----------------------------------
  Generate_DebugBreakCallHelper(masm, r0.bit(), 0);
}

void Debug::GenerateReturnDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // In places other than IC call sites it is expected that r0 is TOS which
  // is an object - this is not generally the case so this should be used with
  // care.
  Generate_DebugBreakCallHelper(masm, r0.bit(), 0);
}


void Debug::GenerateCallFunctionStubDebugBreak(MacroAssembler* masm) {
  // Register state for CallFunctionStub (from code-stubs-arm.cc).
  // ----------- S t a t e -------------
  //  -- r1 : function
  // -----------------------------------
  Generate_DebugBreakCallHelper(masm, r1.bit(), 0);
}

void Debug::GenerateCallConstructStubDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // Calling convention for CallConstructStub (from code-stubs-arm.cc)
  // ----------- S t a t e -------------
  //  -- r0     : number of arguments (not smi)
  //  -- r1     : constructor function
  // -----------------------------------
  Generate_DebugBreakCallHelper(masm, r1.bit(), r0.bit());
}


void Debug::GenerateCallConstructStubRecordDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // Calling convention for CallConstructStub (from code-stubs-arm.cc)
  // ----------- S t a t e -------------
  //  -- r0     : number of arguments (not smi)
  //  -- r1     : constructor function
  //  -- r2     : feedback array
  //  -- r3     : feedback slot (smi)
  // -----------------------------------
  Generate_DebugBreakCallHelper(masm, r1.bit() | r2.bit() | r3.bit(), r0.bit());
}


void Debug::GenerateSlot(MacroAssembler* masm) { // SAMEAS: arm
  // Generate enough nop's to make space for a call instruction. Avoid emitting
  // the constant pool in the debug break slot code.
  Assembler::BlockConstPoolScope block_const_pool(masm);
  Label check_codesize;
  __ align(); // Force alignment: Required for SH4 before RecordDebugBreakSlot()
  __ bind(&check_codesize);
  __ RecordDebugBreakSlot();
  for (int i = 0; i < Assembler::kDebugBreakSlotInstructions; i++) {
    __ nop(MacroAssembler::DEBUG_BREAK_NOP);
  }
  ASSERT_EQ(Assembler::kDebugBreakSlotInstructions,
            masm->InstructionsGeneratedSince(&check_codesize));
}


void Debug::GenerateSlotDebugBreak(MacroAssembler* masm) { // SAMEAS: arm
  // In the places where a debug break slot is inserted no registers can contain
  // object pointers.
  Generate_DebugBreakCallHelper(masm, 0, 0);
}

void Debug::GeneratePlainReturnLiveEdit(MacroAssembler* masm) { // SAMEAS: arm
  masm->Abort(kLiveEditFrameDroppingIsNotSupportedOnSh4);
}

void Debug::GenerateFrameDropperLiveEdit(MacroAssembler* masm) { // SAMEAS: arm
  masm->Abort(kLiveEditFrameDroppingIsNotSupportedOnSh4);
}

#undef __



const bool Debug::kFrameDropperSupported = false;

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
