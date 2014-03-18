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
#include "macro-assembler.h"
#include "simulator-sh4.h"

namespace v8 {
namespace internal {


UnaryMathFunction CreateTranscendentalFunction(TranscendentalCache::Type type) {
  switch (type) {
    case TranscendentalCache::SIN: return &sin;
    case TranscendentalCache::COS: return &cos;
    case TranscendentalCache::TAN: return &tan;
    case TranscendentalCache::LOG: return &log;
    default: UNIMPLEMENTED();
  }
  return NULL;
}


#define __ masm.


#if defined(USE_SIMULATOR)
byte* fast_exp_arm_machine_code = NULL;
double fast_exp_simulator(double x) {
  UNIMPLEMENTED();
  return 0.0;
}
#endif


UnaryMathFunction CreateExpFunction() {
  UNIMPLEMENTED();
  return NULL;
}

#undef __


UnaryMathFunction CreateSqrtFunction() {
  return &sqrt;
}


// -------------------------------------------------------------------------
// Platform-specific RuntimeCallHelper functions.

void StubRuntimeCallHelper::BeforeCall(MacroAssembler* masm) const {
  masm->EnterFrame(StackFrame::INTERNAL);
  ASSERT(!masm->has_frame());
  masm->set_has_frame(true);
}


void StubRuntimeCallHelper::AfterCall(MacroAssembler* masm) const {
  masm->LeaveFrame(StackFrame::INTERNAL);
  ASSERT(masm->has_frame());
  masm->set_has_frame(false);
}


// -------------------------------------------------------------------------
// Code generators

#define __ ACCESS_MASM(masm)

void ElementsTransitionGenerator::GenerateMapChangeElementsTransition(
    MacroAssembler* masm, AllocationSiteMode mode,
    Label* allocation_memento_found) {
  __ UNIMPLEMENTED_BREAK();
}


void ElementsTransitionGenerator::GenerateSmiToDouble(
    MacroAssembler* masm, AllocationSiteMode mode, Label* fail) {
  __ UNIMPLEMENTED_BREAK();
}


void ElementsTransitionGenerator::GenerateDoubleToObject(
    MacroAssembler* masm, AllocationSiteMode mode, Label* fail) {
  __ UNIMPLEMENTED_BREAK();
}


void StringCharLoadGenerator::Generate(MacroAssembler* masm, // SAMEAS: arm
                                       Register string,
                                       Register index,
                                       Register result,
                                       Label* call_runtime) {
  // Fetch the instance type of the receiver into result register.
  __ ldr(result, FieldMemOperand(string, HeapObject::kMapOffset));
  __ ldrb(result, FieldMemOperand(result, Map::kInstanceTypeOffset));

  // We need special handling for indirect strings.
  Label check_sequential;
  __ tst(result, Operand(kIsIndirectStringMask));
  __ b(eq, &check_sequential);

  // Dispatch on the indirect string shape: slice or cons.
  Label cons_string;
  __ tst(result, Operand(kSlicedNotConsMask));
  __ b(eq, &cons_string);

  // Handle slices.
  Label indirect_string_loaded;
  __ ldr(result, FieldMemOperand(string, SlicedString::kOffsetOffset));
  __ ldr(string, FieldMemOperand(string, SlicedString::kParentOffset));
  __ SmiUntag(result); // DIFF: codegen
  __ add(index, index, result); // DIFF: codegen
  __ jmp(&indirect_string_loaded);

  // Handle cons strings.
  // Check whether the right hand side is the empty string (i.e. if
  // this is really a flat string in a cons string). If that is not
  // the case we would rather go to the runtime system now to flatten
  // the string.
  __ bind(&cons_string);
  __ ldr(result, FieldMemOperand(string, ConsString::kSecondOffset));
  __ CompareRoot(result, Heap::kempty_stringRootIndex);
  __ b(ne, call_runtime);
  // Get the first of the two strings and load its instance type.
  __ ldr(string, FieldMemOperand(string, ConsString::kFirstOffset));

  __ bind(&indirect_string_loaded);
  __ ldr(result, FieldMemOperand(string, HeapObject::kMapOffset));
  __ ldrb(result, FieldMemOperand(result, Map::kInstanceTypeOffset));

  // Distinguish sequential and external strings. Only these two string
  // representations can reach here (slices and flat cons strings have been
  // reduced to the underlying sequential or external string).
  Label external_string, check_encoding;
  __ bind(&check_sequential);
  STATIC_ASSERT(kSeqStringTag == 0);
  __ tst(result, Operand(kStringRepresentationMask));
  __ b(ne, &external_string);

  // Prepare sequential strings
  STATIC_ASSERT(SeqTwoByteString::kHeaderSize == SeqOneByteString::kHeaderSize);
  __ add(string,
         string,
         Operand(SeqTwoByteString::kHeaderSize - kHeapObjectTag));
  __ jmp(&check_encoding);

  // Handle external strings.
  __ bind(&external_string);
  if (FLAG_debug_code) {
    // Assert that we do not have a cons or slice (indirect strings) here.
    // Sequential strings have already been ruled out.
    __ tst(result, Operand(kIsIndirectStringMask));
    __ Assert(eq, kExternalStringExpectedButNotFound);
  }
  // Rule out short external strings.
  STATIC_CHECK(kShortExternalStringTag != 0);
  __ tst(result, Operand(kShortExternalStringMask));
  __ b(ne, call_runtime);
  __ ldr(string, FieldMemOperand(string, ExternalString::kResourceDataOffset));

  Label ascii, done;
  __ bind(&check_encoding);
  STATIC_ASSERT(kTwoByteStringTag == 0);
  __ tst(result, Operand(kStringEncodingMask));
  __ b(ne, &ascii);
  // Two-byte string.
  __ lsl(result, index, Operand(1)); // DIFF: codegen
  __ ldrh(result, MemOperand(string, result)); // DIFF: codegen
  __ jmp(&done);
  __ bind(&ascii);
  // Ascii string.
  __ ldrb(result, MemOperand(string, index));
  __ bind(&done);
}


void MathExpGenerator::EmitMathExp(MacroAssembler* masm,
                                   DwVfpRegister input,
                                   DwVfpRegister result,
                                   DwVfpRegister double_scratch1,
                                   DwVfpRegister double_scratch2,
                                   Register temp1,
                                   Register temp2,
                                   Register temp3) {
  __ UNIMPLEMENTED_BREAK();
}

#undef __

// The code age marker is a nop at the start of sequence (ref to PatchPlatformCodeAge())
static const uint16_t kCodeAgePatchFirstInstruction = 0x0009;

static byte* GetNoCodeAgeSequence(uint32_t* length) {
  // The sequence of instructions that is patched out for aging code is the
  // following boilerplate stack-building prologue that is found in FUNCTIONS
  static bool initialized = false;
  static uint16_t sequence[kNoCodeAgeSequenceLength];
  byte* byte_sequence = reinterpret_cast<byte*>(sequence);
  *length = kNoCodeAgeSequenceLength * Assembler::kInstrSize;
  if (!initialized) {
    CodePatcher patcher(byte_sequence, kNoCodeAgeSequenceLength);
    PredictableCodeSizeScope scope(patcher.masm(), *length);
    // This code must be the same as in MacroAssembler::Prologue
    // Otherwise the ASSERT(result) in ::IsYoungSequence() will fail
    // for young code.
    // We add nops at the end of the sequence to match kNoCodeAgeSequenceLength.
    // kNoCodeAgeSequenceLength == (4 + 1 + 2) instrs (Push, nop, add) + 2 (2 padding nops)
    int start_offset = patcher.masm()->pc_offset();
    patcher.masm()->Push(pr, fp, cp, r1); // 4 instrs
    patcher.masm()->nop(); // 1 instr
    patcher.masm()->add(fp, sp, Operand(2 * kPointerSize)); // 2 instr
    patcher.masm()->nop(); // padding
    patcher.masm()->nop(); // padding
    ASSERT(patcher.masm()->pc_offset() - start_offset ==
           (4 + 1 + 2 + 1 + 1) * Assembler::kInstrSize);
    ASSERT(patcher.masm()->pc_offset() - start_offset ==
           kNoCodeAgeSequenceLength * Assembler::kInstrSize);
    initialized = true;
  }
  return byte_sequence;
}


bool Code::IsYoungSequence(byte* sequence) {
  uint32_t young_length;
  byte* young_sequence = GetNoCodeAgeSequence(&young_length);
  bool result = !memcmp(sequence, young_sequence, young_length);
  ASSERT(result ||
         Memory::uint16_at(sequence) == kCodeAgePatchFirstInstruction);
  return result;
}


void Code::GetCodeAgeAndParity(byte* sequence, Age* age,
                               MarkingParity* parity) {
  if (IsYoungSequence(sequence)) {
    *age = kNoAgeCodeAge;
    *parity = NO_MARKING_PARITY;
  } else {
    // Must be the same computation as in RelocInfo::code_age_stub()
    // and RelocInfo::set_code_age_stub().
    // For SH4 the address may be misaligned, in this case it is one
    // instruction before.
    byte *target_address_pointer =
      sequence + Assembler::kInstrSize * kNoCodeAgeSequenceLength - 4;
    if ((uintptr_t)target_address_pointer % 4 == 2)
      target_address_pointer -= 2;
    ASSERT((uintptr_t)target_address_pointer % 4 == 0);
    Address target_address = Memory::Address_at(target_address_pointer);
    Code* stub = GetCodeFromTargetAddress(target_address);
    GetCodeAgeAndParity(stub, age, parity);
  }
}


void Code::PatchPlatformCodeAge(Isolate* isolate,
                                byte* sequence,
                                Code::Age age,
                                MarkingParity parity) {
  uint32_t young_length;
  byte* young_sequence = GetNoCodeAgeSequence(&young_length);
  if (age == kNoAgeCodeAge) {
    CopyBytes(sequence, young_sequence, young_length);
    CPU::FlushICache(sequence, young_length);
  } else {
    // This code must be the same as in MacroAssembler::Prologue().
    // r1 contains isolate and must not be killed there
    // Ref to called stubs called there:
    // Builtins::Generate_MarkCodeAsExecutedOnce()
    // Builtins::GenerateMakeCodeYoungAgainCommon()
    // Use r2 as temporary register to preserve return address,
    // it will be restored in the builtins stubs.
    // Use the special method jsr_at_code_stub_address()
    // that will link and jump to the address emitted just after by
    // dd(reinterpret_cast<uint32_t>(stub->instruction_start()))
    // The generated pr will be 6 bytes before the end of the sequence
    // whatever the sequence alignment.
    // The head of the sequence will be rematerialized from it in the
    // builtins.
    // This method uses the same trick as Mips but for SH4 there is
    // the misalignment issue to handle.
    Code* stub = GetCodeAgeStub(isolate, age, parity);
    CodePatcher patcher(sequence, young_length / Assembler::kInstrSize);
    int start_offset = patcher.masm()->pc_offset();
    patcher.masm()->nop(); // This nop() is the code age marker
    patcher.masm()->mov(r2, pr);
    patcher.masm()->jsr_at_following_address();
    int padding = ((uintptr_t)sequence + patcher.masm()->pc_offset()) % 4 == 0 ?  1: 0;
    patcher.masm()->align(); // align for address below
    patcher.masm()->dd(reinterpret_cast<uint32_t>(stub->instruction_start()));
    // if not previously aligned, add the nop at the end
    if (padding) patcher.masm()->nop();
    // The sequence length is supposed to be:
    // (1 + 1 + 4) instructions (nop, mov, jsr_at_code_stub_address) + 4 bytes
    // plus the padding nop which is either at the end or before the jsr (align)
    ASSERT(patcher.masm()->pc_offset() - start_offset ==
           (1 + 1 + 4 + 1) * Assembler::kInstrSize + 4);
    // The sequence size must be identical to the young sequence size.
    ASSERT((uint32_t)patcher.masm()->pc_offset() - start_offset == young_length);
    // Actually is must be kNoCodeAgeSequenceLength * kInstrSize.
    ASSERT(patcher.masm()->pc_offset() - start_offset ==
           kNoCodeAgeSequenceLength * Assembler::kInstrSize);
  }
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
