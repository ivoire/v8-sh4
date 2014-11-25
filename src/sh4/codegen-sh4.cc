// Copyright 2012 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "v8.h"

#if V8_TARGET_ARCH_SH4 // FILE: SAMEAS: arm, REVIEWEDBY: CG

#include "codegen.h"
#include "macro-assembler.h"
#include "simulator-sh4.h"
#include "map-sh4.h" // DIFF: codegen

namespace v8 {
namespace internal {


#define __ masm.


UnaryMathFunction CreateExpFunction() { // REVIEWEDBY: CG
  // SH4: TODO: optionally implement fast version (ref to arm)
  return &exp;
}



UnaryMathFunction CreateSqrtFunction() { // REVIEWEDBY: CG
  // SH4: TODO: optionally implement fast version (ref to arm)
  return &sqrt;
}

#undef __


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

void ElementsTransitionGenerator::GenerateMapChangeElementsTransition( // REVIEWEDBY: CG
    MacroAssembler* masm, AllocationSiteMode mode,
    Label* allocation_memento_found) {
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : key
  //  -- r2    : receiver
  //  -- lr    : return address
  //  -- r3    : target map, scratch for subsequent call
  //  -- r4    : scratch (elements)
  // -----------------------------------
  if (mode == TRACK_ALLOCATION_SITE) {
    ASSERT(allocation_memento_found != NULL);
    __ JumpIfJSArrayHasAllocationMemento(r2, r4, allocation_memento_found);
  }

  // Set transitioned map.
  __ str(r3, FieldMemOperand(r2, HeapObject::kMapOffset));
  __ RecordWriteField(r2,
                      HeapObject::kMapOffset,
                      r3,
                      r9,
                      kLRHasNotBeenSaved,
                      kDontSaveFPRegs,
                      EMIT_REMEMBERED_SET,
                      OMIT_SMI_CHECK);
}


void ElementsTransitionGenerator::GenerateSmiToDouble( // REVIEWEDBY: CG
    MacroAssembler* masm, AllocationSiteMode mode, Label* fail) {
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : key
  //  -- r2    : receiver
  //  -- lr    : return address
  //  -- r3    : target map, scratch for subsequent call
  //  -- r4    : scratch (elements)
  // -----------------------------------
  Label loop, entry, convert_hole, gc_required, only_change_map, done;

  if (mode == TRACK_ALLOCATION_SITE) {
    __ JumpIfJSArrayHasAllocationMemento(r2, r4, fail);
  }

  // Check for empty arrays, which only require a map transition and no changes
  // to the backing store.
  __ ldr(r4, FieldMemOperand(r2, JSObject::kElementsOffset));
  __ CompareRoot(r4, Heap::kEmptyFixedArrayRootIndex);
  __ b(eq, &only_change_map);

  __ push(r7/*lr*/); // SH4: use r7 instead of lr. // DIFF: codegen
  __ ldr(r5, FieldMemOperand(r4, FixedArray::kLengthOffset));
  // r5: number of elements (smi-tagged)

  // Allocate new FixedDoubleArray.
  // Use lr as a temporary register. // SH4: use r7 instead of lr
  __ lsl(r7/*lr*/, r5, Operand(2)); // DIFF: codegen
  __ add(r7/*lr*/, r7/*lr*/, Operand(FixedDoubleArray::kHeaderSize)); // DIFF: codegen
  __ Allocate(r7/*lr*/, r6, r4, r9, &gc_required, DOUBLE_ALIGNMENT); // DIFF: codegen
  // r6: destination FixedDoubleArray, not tagged as heap object.
  __ ldr(r4, FieldMemOperand(r2, JSObject::kElementsOffset));
  // r4: source FixedArray.

  // Set destination FixedDoubleArray's length and map.
  __ LoadRoot(r9, Heap::kFixedDoubleArrayMapRootIndex);
  __ str(r5, MemOperand(r6, FixedDoubleArray::kLengthOffset));
  // Update receiver's map.
  __ str(r9, MemOperand(r6, HeapObject::kMapOffset));

  __ str(r3, FieldMemOperand(r2, HeapObject::kMapOffset));
  __ RecordWriteField(r2,
                      HeapObject::kMapOffset,
                      r3,
                      r9,
                      kLRHasBeenSaved,
                      kDontSaveFPRegs,
                      OMIT_REMEMBERED_SET,
                      OMIT_SMI_CHECK);
  // Replace receiver's backing store with newly created FixedDoubleArray.
  __ add(r3, r6, Operand(kHeapObjectTag));
  __ str(r3, FieldMemOperand(r2, JSObject::kElementsOffset));
  __ RecordWriteField(r2,
                      JSObject::kElementsOffset,
                      r3,
                      r9,
                      kLRHasBeenSaved,
                      kDontSaveFPRegs,
                      EMIT_REMEMBERED_SET,
                      OMIT_SMI_CHECK);

  // Prepare for conversion loop.
  __ add(r3, r4, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  __ add(r9, r6, Operand(FixedDoubleArray::kHeaderSize));
  __ lsl(r6, r5, Operand(2)); // DIFF: codegen
  __ add(r6, r9, r6); // DIFF: codegen
  __ mov(r4, Operand(kHoleNanLower32));
  __ mov(r5, Operand(kHoleNanUpper32));
  // r3: begin of source FixedArray element fields, not tagged
  // r4: kHoleNanLower32
  // r5: kHoleNanUpper32
  // r6: end of destination FixedDoubleArray, not tagged
  // r9: begin of FixedDoubleArray element fields, not tagged

  __ b(&entry);

  __ bind(&only_change_map);
  __ str(r3, FieldMemOperand(r2, HeapObject::kMapOffset));
  __ RecordWriteField(r2,
                      HeapObject::kMapOffset,
                      r3,
                      r9,
                      kLRHasNotBeenSaved,
                      kDontSaveFPRegs,
                      OMIT_REMEMBERED_SET,
                      OMIT_SMI_CHECK);
  __ b(&done);

  // Call into runtime if GC is required.
  __ bind(&gc_required);
  __ pop(r7/*lr*/); // DIFF: codegen
  __ b(fail);

  // Convert and copy elements.
  __ bind(&loop);
  __ ldr(r7/*lr*/, MemOperand(r3, 4, PostIndex)); // DIFF: codegen
  // r7/*lr*/: current element
  __ UntagAndJumpIfNotSmi(r7/*lr*/, r7/*lr*/, &convert_hole); // DIFF: codegen

  // Normal smi, convert to double and store.
  __ vcvt_f64_s32(d0, r7/*lr*/); // DIFF: codegen
  __ vstr(d0, r9, 0);
  __ add(r9, r9, Operand(8));
  __ b(&entry);

  // Hole found, store the-hole NaN.
  __ bind(&convert_hole);
  if (FLAG_debug_code) {
    // Restore a "smi-untagged" heap object.
    __ SmiTag(r7/*lr*/); // DIFF: codegen
    __ orr(r7/*lr*/, r7/*lr*/, Operand(1)); // DIFF: codegen
    __ CompareRoot(r7/*lr*/, Heap::kTheHoleValueRootIndex); // DIFF: codegen
    __ Assert(eq, kObjectFoundInSmiOnlyArray);
  }
  __ Strd(r4, r5, MemOperand(r9)); // DIFF: codegen
  __ add(r9, r9, Operand(8)); // DIFF: codegen

  __ bind(&entry);
  __ cmpge(r9, r6); // DIFF: codegen
  __ b(f, &loop); // DIFF: codegen

  __ pop(r7/*lr*/); // DIFF: codegen
  __ bind(&done);
}


void ElementsTransitionGenerator::GenerateDoubleToObject( // REVIEWEDBY: CG
    MacroAssembler* masm, AllocationSiteMode mode, Label* fail) {
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : key
  //  -- r2    : receiver
  //  -- lr    : return address
  //  -- r3    : target map, scratch for subsequent call
  //  -- r4    : scratch (elements)
  // -----------------------------------
  Label entry, loop, convert_hole, gc_required, only_change_map;

  if (mode == TRACK_ALLOCATION_SITE) {
    __ JumpIfJSArrayHasAllocationMemento(r2, r4, fail);
  }

  // Check for empty arrays, which only require a map transition and no changes
  // to the backing store.
  __ ldr(r4, FieldMemOperand(r2, JSObject::kElementsOffset));
  __ CompareRoot(r4, Heap::kEmptyFixedArrayRootIndex);
  __ b(eq, &only_change_map);

  __ push(r7/*lr*/); // SH4: use r7 instead of lr. // DIFF: codegen
  __ Push(r3, r2, r1, r0);
  __ ldr(r5, FieldMemOperand(r4, FixedArray::kLengthOffset));
  // r4: source FixedDoubleArray
  // r5: number of elements (smi-tagged)

  // Allocate new FixedArray.
  __ mov(r0, Operand(FixedDoubleArray::kHeaderSize));
  __ lsl(r3, r5, Operand(1)); // r3 available as scratch // DIFF: codegen
  __ add(r0, r0, r3); // DIFF: codegen
  __ Allocate(r0, r6, r3, r9, &gc_required, NO_ALLOCATION_FLAGS);
  // r6: destination FixedArray, not tagged as heap object
  // Set destination FixedDoubleArray's length and map.
  __ LoadRoot(r9, Heap::kFixedArrayMapRootIndex);
  __ str(r5, MemOperand(r6, FixedDoubleArray::kLengthOffset));
  __ str(r9, MemOperand(r6, HeapObject::kMapOffset));

  // Prepare for conversion loop.
  __ add(r4, r4, Operand(FixedDoubleArray::kHeaderSize - kHeapObjectTag + 4));
  __ add(r3, r6, Operand(FixedArray::kHeaderSize));
  __ add(r6, r6, Operand(kHeapObjectTag));
  __ lsl(r5, r5, Operand(1)); // r5 available as scratch // DIFF: codegen
  __ add(r5, r3, r5); // DIFF: codegen
  __ LoadRoot(r9, Heap::kHeapNumberMapRootIndex);
  // Using offsetted addresses in r4 to fully take advantage of post-indexing.
  // r3: begin of destination FixedArray element fields, not tagged
  // r4: begin of source FixedDoubleArray element fields, not tagged, +4
  // r5: end of destination FixedArray, not tagged
  // r6: destination FixedArray
  // r9: heap number map
  __ b(&entry);

  // Call into runtime if GC is required.
  __ bind(&gc_required);
  __ Pop(r3, r2, r1, r0);
  __ pop(r7/*lr*/); // DIFF: codegen
  __ b(fail);

  __ bind(&loop);
  __ ldr(r1, MemOperand(r4, 8, PostIndex));
  // r1: current element's upper 32 bit
  // r4: address of next element's upper 32 bit
  __ cmp(r1, Operand(kHoleNanUpper32));
  __ b(eq, &convert_hole);

  // Non-hole double, copy value into a heap number.
  __ AllocateHeapNumber(r2, r0, r7/*lr*/, r9, &gc_required); // DIFF: codegen
  // r2: new heap number
  __ ldr(r0, MemOperand(r4, -12)); // DIFF: codegen
  __ Strd(r0, r1, FieldMemOperand(r2, HeapNumber::kValueOffset));
  __ mov(r0, r3);
  __ str(r2, MemOperand(r3, 4, PostIndex));
  __ RecordWrite(r6,
                 r0,
                 r2,
                 kLRHasBeenSaved,
                 kDontSaveFPRegs,
                 EMIT_REMEMBERED_SET,
                 OMIT_SMI_CHECK);
  __ b(&entry);

  // Replace the-hole NaN with the-hole pointer.
  __ bind(&convert_hole);
  __ LoadRoot(r0, Heap::kTheHoleValueRootIndex);
  __ str(r0, MemOperand(r3, 4, PostIndex));

  __ bind(&entry);
  __ cmpge(r3, r5); // DIFF: codegen
  __ b(f, &loop); // DIFF: codegen

  __ Pop(r3, r2, r1, r0);
  // Replace receiver's backing store with newly created and filled FixedArray.
  __ str(r6, FieldMemOperand(r2, JSObject::kElementsOffset));
  __ RecordWriteField(r2,
                      JSObject::kElementsOffset,
                      r6,
                      r9,
                      kLRHasBeenSaved,
                      kDontSaveFPRegs,
                      EMIT_REMEMBERED_SET,
                      OMIT_SMI_CHECK);
  __ pop(r7/*lr*/); // DIFF: codegen

  __ bind(&only_change_map);
  // Update receiver's map.
  __ str(r3, FieldMemOperand(r2, HeapObject::kMapOffset));
  __ RecordWriteField(r2,
                      HeapObject::kMapOffset,
                      r3,
                      r9,
                      kLRHasNotBeenSaved,
                      kDontSaveFPRegs,
                      OMIT_REMEMBERED_SET,
                      OMIT_SMI_CHECK);
}


void StringCharLoadGenerator::Generate(MacroAssembler* masm, // REVIEWEDBY: CG
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
  __ SmiUntag(result); // result available as scratch here // DIFF: codegen
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


// SH4: unused
//static MemOperand ExpConstant(int index, Register base) {
//  return MemOperand(base, index * kDoubleSize);
//}


void MathExpGenerator::EmitMathExp(MacroAssembler* masm,
                                   DwVfpRegister input,
                                   DwVfpRegister result,
                                   DwVfpRegister double_scratch1,
                                   DwVfpRegister double_scratch2,
                                   Register temp1,
                                   Register temp2,
                                   Register temp3) {
  __ UNIMPLEMENTED_BREAK(); // SH4: not used (ref to CreateExpFunction())
}

#undef __

#ifdef DEBUG
// SH4: The code age marker is a nop at the start of sequence (ref to PatchPlatformCodeAge())
static const uint16_t kCodeAgePatchFirstInstruction = 0x0009;
#endif

CodeAgingHelper::CodeAgingHelper() { // REVIEWEDBY: CG
  // SH4: the code sequence here are SH4 specific, change with care
  ASSERT(young_sequence_.length() == kNoCodeAgeSequenceLength);
  // Since patcher is a large object, allocate it dynamically when needed,
  // to avoid overloading the stack in stress conditions.
  // DONT_FLUSH is used because the CodeAgingHelper is initialized early in
  // the process, before ARM simulator ICache is setup.
  SmartPointer<CodePatcher> patcher(
      new CodePatcher(young_sequence_.start(),
                      young_sequence_.length() / Assembler::kInstrSize,
                      CodePatcher::DONT_FLUSH));
  PredictableCodeSizeScope scope(patcher->masm(), young_sequence_.length());
  // This code must be the same as in MacroAssembler::Prologue
  // Otherwise the ASSERT(result) in ::IsYoungSequence() will fail
  // for young code.
  // We add nops at the end of the sequence to match kNoCodeAgeSequenceLength.
  // kNoCodeAgeSequenceLength == (4 + 1 + 2) instrs (Push, nop, add) + 2 (2 padding nops)
  int start_offset = patcher->masm()->pc_offset();
  patcher->masm()->Push(pr, fp, cp, r1); // 4 instrs
  patcher->masm()->nop(); // 1 instr
  patcher->masm()->add(fp, sp, Operand(2 * kPointerSize)); // 2 instr
  patcher->masm()->nop(); // padding
  patcher->masm()->nop(); // padding
  USE(start_offset);
  ASSERT(patcher->masm()->pc_offset() - start_offset ==
	 (4 + 1 + 2 + 1 + 1) * Assembler::kInstrSize);
  ASSERT(patcher->masm()->pc_offset() - start_offset ==
	 kNoCodeAgeSequenceLength);
}


#ifdef DEBUG
bool CodeAgingHelper::IsOld(byte* candidate) const { // REVIEWEDBY: CG
  return Memory::uint16_at(candidate) == kCodeAgePatchFirstInstruction;
}
#endif


bool Code::IsYoungSequence(Isolate* isolate, byte* sequence) { // REVIEWEDBY: CG
  bool result = isolate->code_aging_helper()->IsYoung(sequence);
  ASSERT(result || isolate->code_aging_helper()->IsOld(sequence));
  return result;
}


void Code::GetCodeAgeAndParity(Isolate* isolate, byte* sequence, Age* age, // REVIEWEDBY: CG
                               MarkingParity* parity) {
  if (IsYoungSequence(isolate, sequence)) {
    *age = kNoAgeCodeAge;
    *parity = NO_MARKING_PARITY;
  } else {
    // Must be the same computation as in RelocInfo::code_age_stub()
    // and RelocInfo::set_code_age_stub().
    // SH4: for sh4 the address may be misaligned, in this case it is one
    // instruction before.
    byte *target_address_pointer =
      sequence + kNoCodeAgeSequenceLength - 4;
    if ((uintptr_t)target_address_pointer % 4 == 2)
      target_address_pointer -= 2;
    ASSERT((uintptr_t)target_address_pointer % 4 == 0);
    Address target_address = Memory::Address_at(target_address_pointer);
    Code* stub = GetCodeFromTargetAddress(target_address);
    GetCodeAgeAndParity(stub, age, parity);
  }
}


void Code::PatchPlatformCodeAge(Isolate* isolate, // REVIEWEDBY: CG
                                byte* sequence,
                                Code::Age age,
                                MarkingParity parity) {
  uint32_t young_length = isolate->code_aging_helper()->young_sequence_length();
  if (age == kNoAgeCodeAge) {
    isolate->code_aging_helper()->CopyYoungSequenceTo(sequence);
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
    USE(start_offset);
    ASSERT(patcher.masm()->pc_offset() - start_offset ==
           (1 + 1 + 4 + 1) * Assembler::kInstrSize + 4);
    // The sequence size must be identical to the young sequence size.
    ASSERT((uint32_t)patcher.masm()->pc_offset() - start_offset == young_length);
    // Actually is must be kNoCodeAgeSequenceLength.
    ASSERT(patcher.masm()->pc_offset() - start_offset ==
           kNoCodeAgeSequenceLength);
  }
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
