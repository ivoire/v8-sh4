// Copyright 2012 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <limits.h>  // For LONG_MIN, LONG_MAX.

#include "v8.h"

#if V8_TARGET_ARCH_SH4 // FILE, SAMEAS: arm, REVIEWEDBY: RD, CG

#include "bootstrapper.h"
#include "codegen.h"
#include "cpu-profiler.h"
#include "debug.h"
#include "isolate-inl.h"
#include "runtime.h"

#include "map-sh4.h"  // SH4: define arm->sh4 register map

namespace v8 {
namespace internal {

// SH4: Use for recording line info in code comments
#ifdef DEBUG
#define RECORD_LINE() RecordFunctionLine(__FUNCTION__, __LINE__)
#else
#define RECORD_LINE() ((void)0)
#endif


MacroAssembler::MacroAssembler(Isolate* arg_isolate, void* buffer, int size)
    : Assembler(arg_isolate, buffer, size),
      generating_stub_(false),
      has_frame_(false) {
  if (isolate() != NULL) {
    code_object_ = Handle<Object>(isolate()->heap()->undefined_value(),
                                  isolate());
  }
}


void MacroAssembler::Jump(Register target, Condition cond) { // REVIEWEDBY: RD
  Label skip;
  ASSERT(cond == eq || cond == ne || cond == al);
  RECORD_LINE();
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  jmp(target);
  if (cond != al)
      bind(&skip);
}


void MacroAssembler::Jump(intptr_t target, RelocInfo::Mode rmode, // REVIEWEDBY: RD
                          Condition cond) {

  ASSERT(RelocInfo::IsCodeTarget(rmode));
  Label skip;
  ASSERT(cond == eq || cond == ne || cond == al);
  RECORD_LINE();
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  mov(ip, Operand(target, rmode));
  jmp(ip);
  if (cond != al)
    bind(&skip);
}


void MacroAssembler::Jump(Address target, RelocInfo::Mode rmode, // REVIEWEDBY: RD
                          Condition cond) {

  ASSERT(!RelocInfo::IsCodeTarget(rmode));
  UNIMPLEMENTED(); // SH4: obsolete. Seems obsolete also on ARM
  // Jump(reinterpret_cast<intptr_t>(target), rmode, cond);
}


void MacroAssembler::Jump(Handle<Code> code, RelocInfo::Mode rmode, // REVIEWEDBY: RD
                          Condition cond) {

  ASSERT(RelocInfo::IsCodeTarget(rmode));
  ASSERT(cond == eq || cond == ne || cond == al);
  RECORD_LINE();
  Jump(reinterpret_cast<intptr_t>(code.location()), rmode, cond);
}


int MacroAssembler::CallSize(Register target, int call_offset, Condition cond) { // REVIEWEDBY: RD
  ASSERT_EQ(cond, al);
  // Register based call: jsr @target; nop;
  return 2 * kInstrSize;
}


void MacroAssembler::Call(Register target, Condition cond) { // REVIEWEDBY: CG
  ASSERT_EQ(cond, al);
  BlockConstPoolScope block_const_pool(this);
  Label start;
  int call_offset = pc_offset();
  bind(&start);
  jsr(target);
  USE(call_offset);
  ASSERT_EQ(CallSize(target, call_offset, cond), SizeOfCodeGeneratedSince(&start));
}


int MacroAssembler::CallSize(Address target, // REVIEWEDBY: RD
                             int call_offset,
                             RelocInfo::Mode rmode) {
  // Depends upon constant pool management:
  // same as GetCallTargetAddressOffset()
  return GetCallTargetAddressOffset(call_offset);
}


void MacroAssembler::Call(Address target, // REVIEWEDBY: CG
                          RelocInfo::Mode rmode,
                          TargetAddressStorageMode mode) {
  // Block constant pool for the call instruction sequence.
  BlockConstPoolScope block_const_pool(this);
  Label start;
  int call_offset = pc_offset();
  bind(&start);

  bool old_predictable_code_size = predictable_code_size();
  if (mode == NEVER_INLINE_TARGET_ADDRESS) {
    set_predictable_code_size(true);
  }

  // Call sequence may be:
  // @ start of ::Call()
  //  mov  ip, @ct_pool
  //  jsr  ip
  //  nop
  //                      @ return address
  // Or without the constant pool optimization
  // @ start of ::Call()
  //  align(4) // a variable sequence (nop)
  //  mov  ip, @slot
  //  nop
  //  jmp 4
  //  nop
  //  ..const..
  //  ..const..
  //  jsr  ip
  //  nop

  // Statement positions are expected to be recorded when the target
  // address is loaded. The mov method will automatically record
  // positions when pc is the target, since this is not the case here
  // we have to do it explicitly.
  positions_recorder()->WriteRecordedPositions();

  mov(ip, Operand(reinterpret_cast<int32_t>(target), rmode));
  jsr(ip);

  USE(call_offset);
  ASSERT_EQ(CallSize(target, call_offset, rmode), SizeOfCodeGeneratedSince(&start));
  if (mode == NEVER_INLINE_TARGET_ADDRESS) {
    set_predictable_code_size(old_predictable_code_size);
  }
}


int MacroAssembler::CallSize(Handle<Code> code, // REVIEWEDBY: RD
                             int call_offset,
                             RelocInfo::Mode rmode,
                             TypeFeedbackId ast_id) {
  AllowDeferredHandleDereference using_raw_address;
  return CallSize(reinterpret_cast<Address>(code.location()), call_offset, rmode);
}


void MacroAssembler::Call(Handle<Code> code, // REVIEWEDBY: RD
                          RelocInfo::Mode rmode,
                          TypeFeedbackId ast_id,
                          TargetAddressStorageMode mode) {
  // Block constant pool when emitting call (might be redundant)
  BlockConstPoolScope block_const_pool(this);

  RECORD_LINE();
  ASSERT(RelocInfo::IsCodeTarget(rmode));
  if (rmode == RelocInfo::CODE_TARGET && !ast_id.IsNone()) {
    SetRecordedAstId(ast_id);
    rmode = RelocInfo::CODE_TARGET_WITH_ID;
  }
  AllowDeferredHandleDereference embedding_raw_address;
  Call(reinterpret_cast<Address>(code.location()), rmode, mode);
}


void MacroAssembler::Ret(Condition cond) { // REVIEWEDBY: CG
  Label skip;
  ASSERT(cond == eq || cond == ne || cond == al);
  RECORD_LINE();
  if (cond != al)
    b(cond == ne ? eq: ne, &skip, Label::kNear);
  rts();
  if (cond != al)
    bind(&skip);
}


void MacroAssembler::Drop(int count) { // REVIEWEDBY: CG
  RECORD_LINE();
  if (count > 0) {
    add(sp, sp, Operand(count * kPointerSize));
  }
}


void MacroAssembler::Ret(int drop) { // REVIEWEDBY: CG
  Drop(drop);
  Ret();
}


void MacroAssembler::Call(Label* target) { // REVIEWEDBY: CG
  jsr(target);
}

void MacroAssembler::UnimplementedBreak(const char *file, int line) { // REVIEWEDBY: CG
  // SH4: generate an unimplemented breakpoint
  // Stores in r0 a file id and in r1 the line number.
  // Used by UNIMPLEMENTED_BREAK() macro.
  uint32_t file_id = 0;
  const char *base = strrchr(file, '/');
  if (base == NULL)
    base = file;
  else
    base++;
  while (*base) {
    file_id += *base;
    base++;
  }
  RECORD_LINE();
  mov(r0, Operand(file_id));
  mov(r1, Operand(line));
  bkpt();
}


void MacroAssembler::Move(Register dst, Handle<Object> value) { // REVIEWEDBY: CG
  AllowDeferredHandleDereference smi_check;
  RECORD_LINE();
  if (value->IsSmi()) {
    mov(dst, Operand(value));
  } else {
    ASSERT(value->IsHeapObject());
    if (isolate()->heap()->InNewSpace(*value)) {
      Handle<Cell> cell = isolate()->factory()->NewCell(value);
      mov(dst, Operand(cell));
      ldr(dst, FieldMemOperand(dst, Cell::kValueOffset));
    } else {
      mov(dst, Operand(value));
    }
  }
}


void MacroAssembler::Move(Register dst, Register src, Condition cond) { // REVIEWEDBY: CG
  ASSERT(cond == eq || cond == ne || cond == al);
  if (!dst.is(src)) {
    RECORD_LINE();
    Label skip;
    if (cond != al)
      b(cond == ne ? eq: ne, &skip, Label::kNear);
    mov(dst, src);
    if (cond != al)
      bind(&skip);
  }
}


void MacroAssembler::And(Register dst, Register src1, const Operand& src2) { // REVIEWEDBY: CG
  RECORD_LINE();
  if (!src2.is_reg() &&
      !src2.must_output_reloc_info(isolate(), this) &&
      src2.immediate() == 0) {
    mov(dst, Operand::Zero());
  } else {
    and_(dst, src1, src2);
  }
}


void MacroAssembler::Ubfx(Register dst, Register src, int lsb, int width) { // REVIEWEDBY: CG
  ASSERT(0 <= lsb && lsb < 32);
  ASSERT(0 < width && width <= 32);
  ASSERT(lsb + width <= 32);

  if (width == 32) {
    mov(dst, src);
    return;
  }
  uint32_t mask = (~0U >> (32 - (width + lsb))) -
    (lsb == 0 ? 0: (~0U >> (32 - lsb)));
  RECORD_LINE();
  land(dst, src, Operand((int)mask));
  if (lsb != 0) {
    RECORD_LINE();
    lsr(dst, dst, Operand(lsb));
  }
}


void MacroAssembler::Sbfx(Register dst, Register src, int lsb, int width) { // REVIEWEDBY: CG
  ASSERT(0 <= lsb && lsb < 32);
  ASSERT(0 < width && width <= 32);
  ASSERT(lsb + width <= 32);
  ASSERT(!dst.is(sh4_rtmp) && !src.is(sh4_rtmp));

  if (width == 32) {
    mov(dst, src);
    return;
  }
  uint32_t mask = (~0U >> (32 - (width + lsb))) -
    (lsb == 0 ? 0: (~0U >> (32 - lsb)));
  land(dst, src, Operand(mask));
  int shift_up = 32 - lsb - width;
  int shift_down = lsb + shift_up;
  if (shift_up != 0) {
    lsl(dst, dst, Operand(shift_up));
  }
  if (shift_down != 0) {
    asr(dst, dst, Operand(shift_down));
  }
}


void MacroAssembler::Bfi(Register dst, // REVIEWEDBY: CG
                         Register src,
                         Register scratch,
                         int lsb,
                         int width) {
  ASSERT(0 <= lsb && lsb < 32);
  ASSERT(0 <= width && width <= 32);
  ASSERT(lsb + width <= 32);
  ASSERT(!dst.is(src) && !dst.is(scratch));

  if (width == 0) return;
  if (width == 32) {
    mov(dst, src);
    return;
  }
  uint32_t dst_mask = (~0U >> (32 - (width + lsb))) -
    (lsb == 0 ? 0: (~0U >> (32 - lsb)));
  uint32_t src_mask = (~0U >> (32 - width));
  bic(dst, dst, Operand(dst_mask));
  land(scratch, src, Operand(src_mask));
  lsl(scratch, scratch, Operand(lsb));
  orr(dst, dst, scratch);
}


void MacroAssembler::Bfc(Register dst, Register src, int lsb, int width) { // REVIEWEDBY: CG
  ASSERT(0 <= lsb && lsb < 32);
  ASSERT(0 <= width && width <= 32);
  ASSERT(lsb + width <= 32);

  if (width == 0) {
    mov(dst, src);
    return;
  }
  if (width == 32) {
    mov(dst, Operand(0));
    return;
  }
  uint32_t mask = (~0U >> (32 - (width + lsb))) -
    (lsb == 0 ? 0: (~0U >> (32 - lsb)));
  RECORD_LINE();
  land(dst, src, Operand(~mask));
}


void MacroAssembler::Usat(Register dst, int satpos, Register src) { // REVIEWEDBY: CG
    ASSERT((satpos >= 0) && (satpos <= 31));

    int satval = (1 << satpos) - 1;

    if (!src.is(dst)) {
      mov(dst, src);
    }
    cmpge(dst, Operand(0));
    mov(dst, Operand(0), f);  // 0 if negative.
    cmpgt(dst, Operand(satval));
    mov(dst, Operand(satval), t);  // satval if > satval
}


void MacroAssembler::Load(Register dst, // REVIEWEDBY: CG
                          const MemOperand& src,
                          Representation r) {
  ASSERT(!r.IsDouble());
  if (r.IsInteger8()) {
    ldrsb(dst, src);
  } else if (r.IsUInteger8()) {
    ldrb(dst, src);
  } else if (r.IsInteger16()) {
    ldrsh(dst, src);
  } else if (r.IsUInteger16()) {
    ldrh(dst, src);
  } else {
    ldr(dst, src);
  }
}

void MacroAssembler::Store(Register src, // REVIEWEDBY: CG
                           const MemOperand& dst,
                           Representation r) {
  ASSERT(!r.IsDouble());
  if (r.IsInteger8() || r.IsUInteger8()) {
    strb(src, dst);
  } else if (r.IsInteger16() || r.IsUInteger16()) {
    strh(src, dst);
  } else {
    str(src, dst);
  }
}

void MacroAssembler::LoadRoot(Register destination, // REVIEWEDBY: CG
                              Heap::RootListIndex index,
                              Condition cond) {
  RECORD_LINE();
  ldr(destination, MemOperand(kRootRegister, index << kPointerSizeLog2), cond);
}


void MacroAssembler::StoreRoot(Register source, // REVIEWEDBY: CG
                               Heap::RootListIndex index,
                               Condition cond) {
  RECORD_LINE();
  str(source, MemOperand(kRootRegister, index << kPointerSizeLog2), cond);
}


MacroAssembler* MacroAssembler::RecordFunctionLine(const char* function, // REVIEWEDBY: CG
                                                   int line) {
  // SH4: on sh4 target we emit function/line in code comments
  // Ref to RECORD_LINE() macro
  if (FLAG_code_comments) {
    /* 10(strlen of MAXINT) + 1(separator) +1(nul). */
    int size = strlen("/line/")+strlen(function) + 10 + 1 + 1;
    char *buffer = new char[size];
    snprintf(buffer, size, "/line/%s/%d", function, line);
    buffer[size-1] = '\0';
    RecordComment(buffer);
  }
  return this;
}


void MacroAssembler::InNewSpace(Register object, // REVIEWEDBY: CG
                                Register scratch,
                                Condition cond,
                                Label* branch) {
  ASSERT(!object.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!object.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  ASSERT(cond == eq || cond == ne);
  and_(scratch, object, Operand(ExternalReference::new_space_mask(isolate())));
  cmp(scratch, Operand(ExternalReference::new_space_start(isolate())));
  b(cond, branch);
}


void MacroAssembler::RecordWriteField( // REVIEWEDBY: CG
    Register object,
    int offset,
    Register value,
    Register dst,
    LinkRegisterStatus lr_status,
    SaveFPRegsMode save_fp,
    RememberedSetAction remembered_set_action,
    SmiCheck smi_check) {
  // First, check if a write barrier is even needed. The tests below
  // catch stores of Smis.
  Label done;

  // Skip barrier if writing a smi.
  if (smi_check == INLINE_SMI_CHECK) {
    JumpIfSmi(value, &done);
  }

  // Although the object register is tagged, the offset is relative to the start
  // of the object, so so offset must be a multiple of kPointerSize.
  ASSERT(IsAligned(offset, kPointerSize));

  add(dst, object, Operand(offset - kHeapObjectTag));
  if (emit_debug_code()) {
    Label ok;
    tst(dst, Operand((1 << kPointerSizeLog2) - 1));
    b(eq, &ok);
    stop("Unaligned cell in write barrier");
    bind(&ok);
  }

  RecordWrite(object,
              dst,
              value,
              lr_status,
              save_fp,
              remembered_set_action,
              OMIT_SMI_CHECK);

  bind(&done);

  // Clobber clobbered input registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    mov(value, Operand(BitCast<int32_t>(kZapValue + 4)));
    mov(dst, Operand(BitCast<int32_t>(kZapValue + 8)));
  }
}


// Will clobber 4 registers: object, address, scratch, ip.  The
// register 'object' contains a heap object pointer.  The heap object
// tag is shifted away.
void MacroAssembler::RecordWrite(Register object, // REVIEWEDBY: CG
                                 Register address,
                                 Register value,
                                 LinkRegisterStatus lr_status,
                                 SaveFPRegsMode fp_mode,
                                 RememberedSetAction remembered_set_action,
                                 SmiCheck smi_check) {
  ASSERT(!object.is(value));
  if (emit_debug_code()) {
    ldr(ip, MemOperand(address));
    cmp(ip, value);
    Check(eq, kWrongAddressOrValuePassedToRecordWrite);
  }

  // Count number of write barriers in generated code.
  isolate()->counters()->write_barriers_static()->Increment();
  // TODO(mstarzinger): Dynamic counter missing.

  // First, check if a write barrier is even needed. The tests below
  // catch stores of smis and stores into the young generation.
  Label done;

  if (smi_check == INLINE_SMI_CHECK) {
    JumpIfSmi(value, &done);
  }

  CheckPageFlag(value,
                value,  // Used as scratch.
                MemoryChunk::kPointersToHereAreInterestingMask,
                eq,
                &done);
  CheckPageFlag(object,
                value,  // Used as scratch.
                MemoryChunk::kPointersFromHereAreInterestingMask,
                eq,
                &done);

  // Record the actual write.
  if (lr_status == kLRHasNotBeenSaved) {
    push(lr);
  }
  RecordWriteStub stub(isolate(), object, value, address, remembered_set_action,
                       fp_mode);
  CallStub(&stub);
  if (lr_status == kLRHasNotBeenSaved) {
    pop(lr);
  }

  bind(&done);

  // Clobber clobbered registers when running with the debug-code flag
  // turned on to provoke errors.
  if (emit_debug_code()) {
    mov(address, Operand(BitCast<int32_t>(kZapValue + 12)));
    mov(value, Operand(BitCast<int32_t>(kZapValue + 16)));
  }
}


void MacroAssembler::RememberedSetHelper(Register object,  // For debug tests. // REVIEWEDBY: CG
                                         Register address,
                                         Register scratch,
                                         SaveFPRegsMode fp_mode,
                                         RememberedSetFinalAction and_then) {
  Label done;
  if (emit_debug_code()) {
    Label ok;
    JumpIfNotInNewSpace(object, scratch, &ok);
    stop("Remembered set pointer is in new space");
    bind(&ok);
  }
  // Load store buffer top.
  ExternalReference store_buffer =
      ExternalReference::store_buffer_top(isolate());
  mov(ip, Operand(store_buffer));
  ldr(scratch, MemOperand(ip));
  // Store pointer to buffer and increment buffer top.
  str(address, MemOperand(scratch, kPointerSize, PostIndex));
  // Write back new top of buffer.
  str(scratch, MemOperand(ip));
  // Call stub on end of buffer.
  // Check for end of buffer.
  tst(scratch, Operand(StoreBuffer::kStoreBufferOverflowBit));
  if (and_then == kFallThroughAtEnd) {
    b(eq, &done);
  } else {
    ASSERT(and_then == kReturnAtEnd);
    Ret(eq);
  }
  push(pr);
  StoreBufferOverflowStub store_buffer_overflow =
      StoreBufferOverflowStub(isolate(), fp_mode);
  CallStub(&store_buffer_overflow);
  pop(lr);
  bind(&done);
  if (and_then == kReturnAtEnd) {
    Ret();
  }
}

void MacroAssembler::PushFixedFrame(Register marker_reg) { // REVIEWEDBY: CG
  ASSERT(!marker_reg.is_valid() || marker_reg.code() < cp.code());
  if (marker_reg.is_valid())
    Push(pr, fp, cp, marker_reg);
  else
    Push(pr, fp, cp);
}


void MacroAssembler::PopFixedFrame(Register marker_reg) { // REVIEWEDBY: CG
  ASSERT(!marker_reg.is_valid() || marker_reg.code() < cp.code());
  if (marker_reg.is_valid())
    Pop(pr, fp, cp, marker_reg);
  else
    Pop(pr, fp, cp);
}


// Push and pop all registers that can hold pointers.
void MacroAssembler::PushSafepointRegisters() { // REVIEWEDBY: CG
  // Safepoints expect a block of contiguous register values starting with r0:
  ASSERT(((1 << kNumSafepointSavedRegisters) - 1) == kSafepointSavedRegisters);
  // Safepoints expect a block of kNumSafepointRegisters values on the
  // stack, so adjust the stack for unsaved registers.
  const int num_unsaved = kNumSafepointRegisters - kNumSafepointSavedRegisters;
  ASSERT(num_unsaved >= 0);
  sub(sp, sp, Operand(num_unsaved * kPointerSize));
  pushm(kSafepointSavedRegisters); // DIFF: codegen
}


void MacroAssembler::PopSafepointRegisters() { // REVIEWEDBY: CG
  const int num_unsaved = kNumSafepointRegisters - kNumSafepointSavedRegisters;
  popm(kSafepointSavedRegisters); // DIFF: codegen
  add(sp, sp, Operand(num_unsaved * kPointerSize));
}


void MacroAssembler::PushSafepointRegistersAndDoubles() {
  UNIMPLEMENTED();
}


void MacroAssembler::PopSafepointRegistersAndDoubles() {
  UNIMPLEMENTED();
}

void MacroAssembler::StoreToSafepointRegistersAndDoublesSlot(Register src, // REVIEWEDBY: CG
                                                             Register dst) {
  str(src, SafepointRegistersAndDoublesSlot(dst));
}


void MacroAssembler::StoreToSafepointRegisterSlot(Register src, Register dst) { // REVIEWEDBY: CG
  str(src, SafepointRegisterSlot(dst));
}


void MacroAssembler::LoadFromSafepointRegisterSlot(Register dst, Register src) { // REVIEWEDBY: CG
  ldr(dst, SafepointRegisterSlot(src));
}


int MacroAssembler::SafepointRegisterStackIndex(int reg_code) { // REVIEWEDBY: CG
  // The registers are pushed starting with the highest encoding,
  // which means that lowest encodings are closest to the stack pointer.
  ASSERT(reg_code >= 0 && reg_code < kNumSafepointRegisters);
  return reg_code;
}


MemOperand MacroAssembler::SafepointRegisterSlot(Register reg) {  // REVIEWEDBY: CG
  return MemOperand(sp, SafepointRegisterStackIndex(reg.code()) * kPointerSize);
}


MemOperand MacroAssembler::SafepointRegistersAndDoublesSlot(Register reg) { // SH4: TODO
  // Number of d-regs not known at snapshot time.
  ASSERT(!Serializer::enabled(isolate())); // SH4: known actually, but leave as ARM
  // General purpose registers are pushed last on the stack.
  UNIMPLEMENTED();
  int doubles_size = 0; // DwVfpRegister::NumAllocatableRegisters() * kDoubleSize;
  int register_offset = SafepointRegisterStackIndex(reg.code()) * kPointerSize;
  return MemOperand(sp, doubles_size + register_offset);
}


void MacroAssembler::Ldrd(Register dst1, Register dst2, // REVIEWEDBY: CG
                          const MemOperand& src) {
  ASSERT(src.am() == Offset);
  ASSERT(src.roffset().is(no_reg));
  ASSERT_EQ(0, dst1.code() % 2);
  ASSERT_EQ(dst1.code() + 1, dst2.code());
  ASSERT(!dst1.is(sh4_ip) && !dst2.is(sh4_ip));
  ASSERT(!dst1.is(sh4_rtmp) && !dst2.is(sh4_rtmp));

  {
    MemOperand src2(src);
    src2.set_offset(src2.offset() + 4);
    if (dst1.is(src.rn())) {
      ldr(dst2, src2);
      ldr(dst1, src);
    } else {
      ldr(dst1, src);
      ldr(dst2, src2);
    }
  }
}


void MacroAssembler::Strd(Register src1, Register src2, // REVIEWEDBY: CG
                          const MemOperand& dst) {
  ASSERT(dst.am() == Offset);
  ASSERT(dst.roffset().is(no_reg));
  ASSERT_EQ(0, src1.code() % 2);
  ASSERT_EQ(src1.code() + 1, src2.code());
  ASSERT(!src1.is(sh4_ip) && !src2.is(sh4_ip));
  ASSERT(!src1.is(sh4_rtmp) && !src2.is(sh4_rtmp));

  {
    MemOperand dst2(dst);
    dst2.set_offset(dst2.offset() + 4);
    str(src1, dst);
    str(src2, dst2);
  }
}


void MacroAssembler::VFPEnsureFPSCRState(Register scratch) { // REVIEWEDBY: CG
  // SH4: on SH4 the Hole NaN is never generated by hardware float operations
  // and in VFPCanonicalizeNaN we force a unique canonical NaN.
  RECORD_LINE();
  nop();  // SH4: mark with a nop anyway
}


void MacroAssembler::VFPCanonicalizeNaN(const DwVfpRegister dst, // REVIEWEDBY: CG
                                        const DwVfpRegister src,
                                        const Condition cond) {
  // TODO: SH4: may use a faster sequence: vsub(dst, src, kDoubleRegZero, cond)
  // as on ARM.,
  // Though it should be tested with care due the the SH4 defaults with
  // signaling/not signaling NaN which may be selected through the FPSCR.
  ASSERT(cond == al);
  RECORD_LINE();
  Label not_a_nan;
  dcmpeq(src, src);
  if (src.code() != dst.code())
    vmov(dst, src);
  bt_near(&not_a_nan);
  // We have a NaN, replace by the canonical NaN value
  vmov(dst, FixedDoubleArray::canonical_not_the_hole_nan_as_double());
  bind(&not_a_nan);
}


void MacroAssembler::Vmov(const DwVfpRegister dst, // REVIEWEDBY: CG
                          const double imm,
                          const Register scratch) {
  // SH4: scratch is not used
  static const DoubleRepresentation minus_zero(-0.0);
  static const DoubleRepresentation zero(0.0);
  DoubleRepresentation value(imm);
  // Handle special values first.
  if (value.bits == zero.bits) {
    vmov(dst, kDoubleRegZero);
  } else if (value.bits == minus_zero.bits) {
    vneg(dst, kDoubleRegZero);
  } else {
    vmov(dst, imm, scratch);
  }
}


void MacroAssembler::VmovHigh(Register dst, DwVfpRegister src) { // REVIEWEDBY: CG
  vmov(dst, src.high());
}


void MacroAssembler::VmovHigh(DwVfpRegister dst, Register src) { // REVIEWEDBY: CG
  vmov(dst.high(), src);
}


void MacroAssembler::VmovLow(Register dst, DwVfpRegister src) { // REVIEWEDBY: CG
  vmov(dst, src.low());
}


void MacroAssembler::VmovLow(DwVfpRegister dst, Register src) { // REVIEWEDBY: CG
  vmov(dst.low(), src);
}


void MacroAssembler::LoadConstantPoolPointerRegister() { // REVIEWEDBY: CG (check TODO)
  if (FLAG_enable_ool_constant_pool) {
    UNIMPLEMENTED(); // SH4: TODO: constant pool
  }
}


void MacroAssembler::Prologue(PrologueFrameMode frame_mode) { // REVIEWEDBY: RD, CG

  if (frame_mode == BUILD_STUB_FRAME) {
    PushFixedFrame();
    Push(Smi::FromInt(StackFrame::STUB));
    // Adjust FP to point to saved FP.
    add(fp, sp, Operand(StandardFrameConstants::kFixedFrameSizeFromFp));
  } else {
    PredictableCodeSizeScope predictible_code_size_scope(
        this, kNoCodeAgeSequenceLength);
    // The following instructions must remain together and unmodified
    // for code aging to work properly.
    if (isolate()->IsCodePreAgingActive()) {
      // Pre-age the code.
      // This code must be the same as in Code::PatchPlatformCodeAge().
      // Ref to codegen-sh4.cc.
      Code* stub = Code::GetPreAgedCodeAgeStub(isolate());
      nop(); // This nop() is the code age marker
      mov(r2, pr);
      jsr_at_following_address();
      int padding = (uintptr_t)pc_ % 4 == 0 ?  1: 0;
      align(); // align
      dd(reinterpret_cast<uint32_t>(stub->instruction_start()));
      if (padding) nop();
    } else {
      // This code must be the same as in CodeAgingHelper()
      // Ref to codegen-sh4.cc.
      Push(pr, fp, cp, r1);
      nop();
      // Adjust FP to point to saved FP.
      add(fp, sp, Operand(StandardFrameConstants::kFixedFrameSizeFromFp));
      nop();
      nop();
    }
  }
}


void MacroAssembler::EnterFrame(StackFrame::Type type,
                                bool load_constant_pool) { // REVIEWEDBY: RD, CG
  // r0-r3: must be preserved
  RECORD_LINE();
  PushFixedFrame();
  if (FLAG_enable_ool_constant_pool && load_constant_pool) {
    LoadConstantPoolPointerRegister();
  }
  mov(ip, Operand(Smi::FromInt(type)));
  push(ip);
  mov(ip, Operand(CodeObject()));
  push(ip);
  // Adjust FP to point to saved FP.
  add(fp, sp,
      Operand(StandardFrameConstants::kFixedFrameSizeFromFp + kPointerSize));
}


int MacroAssembler::LeaveFrame(StackFrame::Type type) { // REVIEWEDBY: RD, CG
  // r0: preserved
  // r1: preserved
  // r2: preserved

  // Drop the execution stack down to the frame pointer and restore
  // the caller frame pointer and return address.
  int frame_ends;
  RECORD_LINE();
  mov(sp, fp);
  frame_ends = pc_offset();
  Pop(pr, fp);
  return frame_ends;
}


void MacroAssembler::EnterExitFrame(bool save_doubles, int stack_space) { // REVIEWEDBY: RD, CG
  // Set up the frame structure on the stack.
  // Parameters are on stack as if calling JS function
  // r0, r1, cp: must be preserved
  // sp, fp: input/output
  // Actual clobbers: ip
  ASSERT_EQ(2 * kPointerSize, ExitFrameConstants::kCallerSPDisplacement);
  ASSERT_EQ(1 * kPointerSize, ExitFrameConstants::kCallerPCOffset);
  ASSERT_EQ(0 * kPointerSize, ExitFrameConstants::kCallerFPOffset);

  RECORD_LINE();
  Push(pr, fp);
  mov(fp, sp);  // Set up new frame pointer.
  // Reserve room for saved entry sp and code object.
  sub(sp, sp, Operand(ExitFrameConstants::kFrameSize));
  if (emit_debug_code()) {
    mov(ip, Operand::Zero());
    str(ip, MemOperand(fp, ExitFrameConstants::kSPOffset));
  }
  mov(ip, Operand(CodeObject()));
  str(ip, MemOperand(fp, ExitFrameConstants::kCodeOffset));

  // Save the frame pointer and the context in top.
  mov(ip, Operand(ExternalReference(Isolate::kCEntryFPAddress, isolate())));
  str(fp, MemOperand(ip));
  mov(ip, Operand(ExternalReference(Isolate::kContextAddress, isolate())));
  str(cp, MemOperand(ip));

  // Optionally save all double registers.
  if (save_doubles) {
    RECORD_LINE();
    SaveFPRegs(sp, ip); // Note: this modifies sp
    // Note that d0 will be accessible at
    //   fp - 2 * kPointerSize - DwVfpRegister::kMaxNumRegisters * kDoubleSize,
    // since the sp slot and code slot were pushed after the fp.
  }

  // Reserve place for the return address and stack space and align the frame
  // preparing for calling the runtime function.
  const int frame_alignment = MacroAssembler::ActivationFrameAlignment();
  sub(sp, sp, Operand((stack_space + 1) * kPointerSize));
  if (frame_alignment > 0) {
    ASSERT(IsPowerOf2(frame_alignment));
    land(sp, sp, Operand(-frame_alignment));
  }

  // Set the exit frame sp value to point just before the return address
  // location.
  add(ip, sp, Operand(kPointerSize));
  str(ip, MemOperand(fp, ExitFrameConstants::kSPOffset));
}


void MacroAssembler::InitializeNewString(Register string, // REVIEWEDBY: CG
                                         Register length,
                                         Heap::RootListIndex map_index,
                                         Register scratch1,
                                         Register scratch2) {
  RECORD_LINE();
  SmiTag(scratch1, length);
  LoadRoot(scratch2, map_index);
  str(scratch1, FieldMemOperand(string, String::kLengthOffset));
  mov(scratch1, Operand(String::kEmptyHashField));
  str(scratch2, FieldMemOperand(string, HeapObject::kMapOffset));
  str(scratch1, FieldMemOperand(string, String::kHashFieldOffset));
}


int MacroAssembler::ActivationFrameAlignment() { // REVIEWEDBY: CG
#if V8_HOST_ARCH_SH4
  // Running on the real platform. Use the alignment as mandated by the local
  // environment.
  return OS::ActivationFrameAlignment();
#else  // V8_HOST_ARCH_SH4
  // If we are using the simulator then we should always align to the expected
  // alignment. As the simulator is used to generate snapshots we do not know
  // if the target platform will need alignment, so this is controlled from a
  // flag.
  return FLAG_sim_stack_alignment;
#endif  // V8_HOST_ARCH_SH4
}


void MacroAssembler::LeaveExitFrame(bool save_doubles, // REVIEWEDBY: RD, CG
                                    Register argument_count,
                                    bool restore_context) {
  ASSERT(!argument_count.is(sh4_ip));
  ASSERT(!argument_count.is(sh4_rtmp));
  // input: argument_count
  // r0, r1: results must be preserved
  // sp: stack pointer
  // fp: frame pointer

  // Actual clobbers: r3 and ip
  // r4 should be preserved: see the end of RegExpExecStub::Generate

  RECORD_LINE();
  // Optionally restore all double registers.
  if (save_doubles) {
    // Calculate the stack location of the saved doubles and restore them.
    const int offset = ExitFrameConstants::kFrameSize;
    sub(r3, fp,
        Operand(offset + DwVfpRegister::kMaxNumRegisters * kDoubleSize));
    RestoreFPRegs(r3, ip); // Note: this modifies r3, don't use fp/sp directly
  }

  // Clear top frame.
  mov(r3, Operand::Zero());
  mov(ip, Operand(ExternalReference(Isolate::kCEntryFPAddress, isolate())));
  str(r3, MemOperand(ip));

  // Restore current context from top and clear it in debug mode.
  if (restore_context) {
    mov(ip, Operand(ExternalReference(Isolate::kContextAddress, isolate())));
    ldr(cp, MemOperand(ip));
  }
#ifdef DEBUG
  mov(ip, Operand(ExternalReference(Isolate::kContextAddress, isolate())));
  str(r3, MemOperand(ip));
#endif

  // Tear down the exit frame, pop the arguments, and return.
  mov(sp, fp);
  Pop(pr, fp);
  if (argument_count.is_valid()) {
    ASSERT(!argument_count.is(r3));
    lsl(r3, argument_count, Operand(kPointerSizeLog2));
    add(sp, sp, r3);
  }
}


void MacroAssembler::MovFromFloatResult(const DwVfpRegister dst) { // REVIEWEDBY: CG
  // SH4: return value is in dr0
  vmov(dst, sh4_dr0);
}

void MacroAssembler::MovFromFloatParameter(DwVfpRegister dst) { // REVIEWEDBY: CG
  // SH4: first parameter is in dr4
  vmov(dst, sh4_dr4);
}


void MacroAssembler::InvokePrologue(const ParameterCount& expected, // REVIEWEDBY: CG
                                    const ParameterCount& actual,
                                    Handle<Code> code_constant,
                                    Register code_reg,
                                    Label* done,
                                    bool* definitely_mismatches,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper) {
  ASSERT(!code_reg.is(sh4_ip));
  ASSERT(!code_reg.is(sh4_rtmp));
  bool definitely_matches = false;
  *definitely_mismatches = false;
  Label regular_invoke;

  // Check whether the expected and actual arguments count match. If not,
  // setup registers according to contract with ArgumentsAdaptorTrampoline:
  // ARM -> SH4
  //  r0: actual arguments count
  //  r1: function (passed through to callee)
  //  r2: expected arguments count

  // The code below is made a lot easier because the calling code already sets
  // up actual and expected registers according to the contract if values are
  // passed in registers.
  ASSERT(actual.is_immediate() || actual.reg().is(r0));
  ASSERT(expected.is_immediate() || expected.reg().is(r2));
  ASSERT((!code_constant.is_null() && code_reg.is(no_reg)) || code_reg.is(r3));

  RECORD_LINE();
  if (expected.is_immediate()) {
    ASSERT(actual.is_immediate());
    if (expected.immediate() == actual.immediate()) {
      definitely_matches = true;
    } else {
      mov(r0, Operand(actual.immediate()));
      const int sentinel = SharedFunctionInfo::kDontAdaptArgumentsSentinel;
      if (expected.immediate() == sentinel) {
        // Don't worry about adapting arguments for builtins that
        // don't want that done. Skip adaption code by making it look
        // like we have a match between expected and actual number of
        // arguments.
        definitely_matches = true;
      } else {
        *definitely_mismatches = true;
        mov(r2, Operand(expected.immediate()));
      }
    }
  } else {
    if (actual.is_immediate()) {
      cmpeq(expected.reg(), Operand((actual.immediate())));
      bt(&regular_invoke);
      mov(r0, Operand(actual.immediate()));
    } else {
      cmpeq(expected.reg(), actual.reg());
      bt(&regular_invoke);
    }
  }

  RECORD_LINE();
  if (!definitely_matches) {
    if (!code_constant.is_null()) {
      mov(r3, Operand(code_constant));
      add(r3, r3, Operand(Code::kHeaderSize - kHeapObjectTag));
    }

    Handle<Code> adaptor =
        isolate()->builtins()->ArgumentsAdaptorTrampoline();
    if (flag == CALL_FUNCTION) {
      call_wrapper.BeforeCall(CallSize(adaptor, pc_offset()));
      Call(adaptor);
      call_wrapper.AfterCall();
      if (!*definitely_mismatches) {
        b(done);
      }
    } else {
      Jump(adaptor, RelocInfo::CODE_TARGET);
    }
    bind(&regular_invoke);
  }
}


void MacroAssembler::InvokeCode(Register code, // REVIEWEDBY: CG
                                const ParameterCount& expected,
                                const ParameterCount& actual,
                                InvokeFlag flag,
                                const CallWrapper& call_wrapper) {
  // You can't call a function without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());

  Label done;
  bool definitely_mismatches = false;
  ASSERT(!code.is(sh4_ip) && !code.is(sh4_rtmp) && !code.is(r5));

  RECORD_LINE();
  InvokePrologue(expected, actual, Handle<Code>::null(), code,
                 &done, &definitely_mismatches, flag,
                 call_wrapper);
  RECORD_LINE();
  if (!definitely_mismatches) {
    if (flag == CALL_FUNCTION) {
      call_wrapper.BeforeCall(CallSize(code, pc_offset()));
      Call(code);
      call_wrapper.AfterCall();
    } else {
      ASSERT(flag == JUMP_FUNCTION);
      Jump(code);
    }

    // Continue here if InvokePrologue does handle the invocation due to
    // mismatched parameter counts.
    bind(&done);
  }
}


void MacroAssembler::InvokeFunction(Register fun, // REVIEWEDBY: CG
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper) {
  // You can't call a function without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());

  // Contract with called JS functions requires that function is passed in r1.
  ASSERT(fun.is(r1));
  ASSERT(actual.is_immediate() || actual.reg().is(r0));

  Register expected_reg = r2;
  Register code_reg = r3;

  RECORD_LINE();
  ldr(code_reg, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
  ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));
  ldr(expected_reg,
      FieldMemOperand(code_reg,
                      SharedFunctionInfo::kFormalParameterCountOffset));
  SmiUntag(expected_reg);
  ldr(code_reg,
      FieldMemOperand(r1, JSFunction::kCodeEntryOffset));

  ParameterCount expected(expected_reg);
  InvokeCode(code_reg, expected, actual, flag, call_wrapper);
}


void MacroAssembler::InvokeFunction(Register function, // REVIEWEDBY: CG
                                    const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper) {
  // You can't call a function without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());

  // Contract with called JS functions requires that function is passed in r1.
  ASSERT(function.is(r1));

  // Get the function and setup the context.
  ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));

  // We call indirectly through the code field in the function to
  // allow recompilation to take effect without changing any of the
  // call sites.
  ldr(r3, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
  InvokeCode(r3, expected, actual, flag, call_wrapper);
}


void MacroAssembler::InvokeFunction(Handle<JSFunction> function, // REVIEWEDBY: CG
                                    const ParameterCount& expected,
                                    const ParameterCount& actual,
                                    InvokeFlag flag,
                                    const CallWrapper& call_wrapper) {
  Move(r1, function);
  InvokeFunction(r1, expected, actual, flag, call_wrapper);
}


void MacroAssembler::IsObjectJSObjectType(Register heap_object, // REVIEWEDBY: CG
                                          Register map,
                                          Register scratch,
                                          Label* fail) {
  ldr(map, FieldMemOperand(heap_object, HeapObject::kMapOffset));
  IsInstanceJSObjectType(map, scratch, fail);
}


void MacroAssembler::IsInstanceJSObjectType(Register map, // REVIEWEDBY: CG
                                            Register scratch,
                                            Label* fail) {
  ldrb(scratch, FieldMemOperand(map, Map::kInstanceTypeOffset));
  cmpge(scratch, Operand(FIRST_NONCALLABLE_SPEC_OBJECT_TYPE));
  bf(fail);
  cmpgt(scratch, Operand(LAST_NONCALLABLE_SPEC_OBJECT_TYPE));
  bt(fail);
}


void MacroAssembler::IsObjectJSStringType(Register object, // REVIEWEDBY: CG
                                          Register scratch,
                                          Label* fail) {
  ASSERT(kNotStringTag != 0);
  ASSERT(!object.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!object.is(sh4_rtmp) && !scratch.is(sh4_rtmp));

  ldr(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
  ldrb(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));
  tst(scratch, Operand(kIsNotStringMask));
  bf(fail);
}


void MacroAssembler::IsObjectNameType(Register object, // REVIEWEDBY: CG
                                      Register scratch,
                                      Label* fail) {
  ldr(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
  ldrb(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));
  cmphi(scratch, Operand(LAST_NAME_TYPE));
  bt(fail);
}


void MacroAssembler::DebugBreak() { // REVIEWEDBY: CG
  RECORD_LINE();
  mov(r0, Operand::Zero());
  mov(r1, Operand(ExternalReference(Runtime::kDebugBreak, isolate())));
  CEntryStub ces(isolate(), 1);
  ASSERT(AllowThisStubCall(&ces));
  Call(ces.GetCode(), RelocInfo::DEBUG_BREAK);
}


void MacroAssembler::PushTryHandler(StackHandler::Kind kind, // REVIEWEDBY: CG
                                    int handler_index) {
  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 5 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kCodeOffset == 1 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kStateOffset == 2 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kContextOffset == 3 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 4 * kPointerSize);

  // For the JSEntry handler, we must preserve r0-r4, r5-r6 are available.
  // We will build up the handler from the bottom by pushing on the stack.
  // Set up the code object (r5) and the state (r6) for pushing.
  unsigned state =
      StackHandler::IndexField::encode(handler_index) |
      StackHandler::KindField::encode(kind);
  mov(r5, Operand(CodeObject()));
  mov(r6, Operand(state));

  // Push the frame pointer, context, state, and code object.
  if (kind == StackHandler::JS_ENTRY) {
    mov(cp, Operand(Smi::FromInt(0)));  // Indicates no context.
    mov(sh4_ip, Operand::Zero());  // NULL frame pointer.
    Push(sh4_ip, cp, r6, r5);
  } else {
    Push(fp, cp, r6, r5);
  }

  // Link the current handler as the next handler.
  mov(r6, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  ldr(r5, MemOperand(r6));
  push(r5);
  // Set this new handler as the current one.
  str(sp, MemOperand(r6));
}


void MacroAssembler::PopTryHandler() { // REVIEWEDBY: CG
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  RECORD_LINE();
  pop(r1);
  mov(ip, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  add(sp, sp, Operand(StackHandlerConstants::kSize - kPointerSize));
  str(r1, MemOperand(ip));
}


void MacroAssembler::JumpToHandlerEntry() { // REVIEWEDBY: CG
  // Compute the handler entry address and jump to it.  The handler table is
  // a fixed array of (smi-tagged) code offsets.
  // r0 = exception, r1 = code object, r2 = state.
  ldr(r3, FieldMemOperand(r1, Code::kHandlerTableOffset));  // Handler table.
  add(r3, r3, Operand(FixedArray::kHeaderSize - kHeapObjectTag));
  lsr(r2, r2, Operand(StackHandler::kKindWidth)); // Handler index. // DIFF: codegen
  lsl(r2, r2, Operand(kPointerSizeLog2)); // DIFF: codegen
  ldr(r2, MemOperand(r3, r2));  // Smi-tagged offset. // DIFF: codegen
  add(r1, r1, Operand(Code::kHeaderSize - kHeapObjectTag));  // Code start.
  SmiUntag(r2); // DIFF: codegen
  add(r1, r1, r2); // DIFF: codegen
  jmp(r1); // DIFF: codegen
}


void MacroAssembler::ClampUint8(Register output_reg, Register input_reg) { // REVIEWEDBY: CG
  Usat(output_reg, 8, input_reg);
}


void MacroAssembler::ClampDoubleToUint8(Register result_reg, // REVIEWEDBY: CG
                                        DwVfpRegister input_reg,
                                        DwVfpRegister double_scratch) {
  Label done;

  // SH4: clamp first if > 255 or <= 0, else convert (truncation on SH4)

  // Handle inputs <= 0.0 (including -infinity).
  // Handle also NaN that will be converted to 0 as compare will always fail
  // if input is NaN.
  Vmov(double_scratch, 0.0, result_reg);
  mov(result_reg, Operand(0));
  dcmpgt(input_reg, double_scratch); // DIFF: codegen
  bf_near(&done); // DIFF: codegen

  // Handle inputs >= 255.0 (including +infinity).
  Vmov(double_scratch, 255.0, result_reg);
  mov(result_reg, Operand(255));
  dcmpgt(double_scratch, input_reg); // DIFF: codegen
  bf_near(&done); // DIFF: codegen

  // For inputs > 0.0 and < 255.0, round to nearest int.
  // Add 0.5 and truncate.
  vcvt_s32_f64(result_reg, input_reg, kFPSCRRounding);

  bind(&done);
}


void MacroAssembler::Throw(Register value) { // REVIEWEDBY: CG
  ASSERT(!value.is(sh4_ip));
  ASSERT(!value.is(sh4_rtmp));

  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 5 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0);
  STATIC_ASSERT(StackHandlerConstants::kCodeOffset == 1 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kStateOffset == 2 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kContextOffset == 3 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 4 * kPointerSize);

  // The exception is expected in r0.
  if (!value.is(r0)) {
    RECORD_LINE();
    mov(r0, value);
  }

  RECORD_LINE();
  // Drop the stack pointer to the top of the top handler.
  mov(r3, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  ldr(sp, MemOperand(r3));
  // Restore the next handler.
  RECORD_LINE();
  pop(r2);
  str(r2, MemOperand(r3));

  // Get the code object (r1) and state (r2).  Restore the context and frame
  // pointer.
  Pop(fp, cp, r2, r1); // DIFF: codegen

  // If the handler is a JS frame, restore the context to the frame.
  // (kind == ENTRY) == (fp == 0) == (cp == 0), so we could test either fp
  // or cp.
  RECORD_LINE();
  tst(cp, cp);
  str(cp, MemOperand(fp, StandardFrameConstants::kContextOffset), ne);

  JumpToHandlerEntry();
}


void MacroAssembler::ThrowUncatchable(Register value) { // REVIEWEDBY: CG
  ASSERT(!value.is(sh4_ip));
  ASSERT(!value.is(sh4_rtmp));
  // Adjust this code if not the case.
  STATIC_ASSERT(StackHandlerConstants::kSize == 5 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kNextOffset == 0 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kCodeOffset == 1 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kStateOffset == 2 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kContextOffset == 3 * kPointerSize);
  STATIC_ASSERT(StackHandlerConstants::kFPOffset == 4 * kPointerSize);

  // The exception is expected in r0.
  if (!value.is(r0)) {
    RECORD_LINE();
    mov(r0, value);
  }
  RECORD_LINE();
  // Drop the stack pointer to the top of the top stack handler.
  mov(r3, Operand(ExternalReference(Isolate::kHandlerAddress, isolate())));
  ldr(sp, MemOperand(r3));

  // Unwind the handlers until the ENTRY handler is found.
  Label fetch_next, check_kind;
  jmp(&check_kind);
  bind(&fetch_next);
  ldr(sp, MemOperand(sp, StackHandlerConstants::kNextOffset));

  bind(&check_kind);
  STATIC_ASSERT(StackHandler::JS_ENTRY == 0);
  ldr(r2, MemOperand(sp, StackHandlerConstants::kStateOffset));
  tst(r2, Operand(StackHandler::KindField::kMask));
  b(ne, &fetch_next);

  // Set the top handler address to next handler past the top ENTRY handler.
  pop(r2);
  str(r2, MemOperand(r3));
  // Get the code object (r1) and state (r2).  Clear the context and frame
  // pointer (0 was saved in the handler).
  Pop(fp, cp, r2, r1);

  JumpToHandlerEntry();
}


void MacroAssembler::CheckAccessGlobalProxy(Register holder_reg, // REVIEWEDBY: CG
                                            Register scratch,
                                            Label* miss) {
  Label same_contexts;

  ASSERT(!holder_reg.is(scratch));
  ASSERT(!holder_reg.is(ip));
  ASSERT(!scratch.is(ip));

  // Load current lexical context from the stack frame.
  ldr(scratch, MemOperand(fp, StandardFrameConstants::kContextOffset));
  // In debug mode, make sure the lexical context is set.
#ifdef DEBUG
  cmp(scratch, Operand::Zero());
  Check(ne, kWeShouldNotHaveAnEmptyLexicalContext);
#endif

  // Load the native context of the current context.
  int offset =
      Context::kHeaderSize + Context::GLOBAL_OBJECT_INDEX * kPointerSize;
  ldr(scratch, FieldMemOperand(scratch, offset));
  ldr(scratch, FieldMemOperand(scratch, GlobalObject::kNativeContextOffset));

  // Check the context is a native context.
  if (emit_debug_code()) {
    // Cannot use ip as a temporary in this verification code. Due to the fact
    // that ip is clobbered as part of cmp with an object Operand.
    push(holder_reg);  // Temporarily save holder on the stack.
    // Read the first word and compare to the native_context_map.
    ldr(holder_reg, FieldMemOperand(scratch, HeapObject::kMapOffset));
    LoadRoot(ip, Heap::kNativeContextMapRootIndex);
    cmp(holder_reg, ip);
    Check(eq, kJSGlobalObjectNativeContextShouldBeANativeContext);
    pop(holder_reg);  // Restore holder.
  }

  // Check if both contexts are the same.
  ldr(ip, FieldMemOperand(holder_reg, JSGlobalProxy::kNativeContextOffset));
  cmp(scratch, ip);
  b(eq, &same_contexts);

  // Check the context is a native context.
  if (emit_debug_code()) {
    // Cannot use ip as a temporary in this verification code. Due to the fact
    // that ip is clobbered as part of cmp with an object Operand.
    push(holder_reg);  // Temporarily save holder on the stack.
    mov(holder_reg, ip);  // Move ip to its holding place.
    LoadRoot(ip, Heap::kNullValueRootIndex);
    cmp(holder_reg, ip);
    Check(ne, kJSGlobalProxyContextShouldNotBeNull);

    ldr(holder_reg, FieldMemOperand(holder_reg, HeapObject::kMapOffset));
    LoadRoot(ip, Heap::kNativeContextMapRootIndex);
    cmp(holder_reg, ip);
    Check(eq, kJSGlobalObjectNativeContextShouldBeANativeContext);
    // Restore ip is not needed. ip is reloaded below.
    pop(holder_reg);  // Restore holder.
    // Restore ip to holder's context.
    ldr(ip, FieldMemOperand(holder_reg, JSGlobalProxy::kNativeContextOffset));
  }

  // Check that the security token in the calling global object is
  // compatible with the security token in the receiving global
  // object.
  int token_offset = Context::kHeaderSize +
                     Context::SECURITY_TOKEN_INDEX * kPointerSize;

  ldr(scratch, FieldMemOperand(scratch, token_offset));
  ldr(ip, FieldMemOperand(ip, token_offset));
  cmp(scratch, ip);
  b(ne, miss);

  bind(&same_contexts);
}


// Compute the hash code from the untagged key.  This must be kept in sync with
// ComputeIntegerHash in utils.h and KeyedLoadGenericElementStub in
// code-stub-hydrogen.cc
void MacroAssembler::GetNumberHash(Register t0, Register scratch) { // REVIEWEDBY: CG
  // First of all we assign the hash seed to scratch.
  LoadRoot(scratch, Heap::kHashSeedRootIndex);
  SmiUntag(scratch);

  // Xor original key with a seed.
  eor(t0, t0, scratch);

  // Compute the hash code from the untagged key.  This must be kept in sync
  // with ComputeIntegerHash in utils.h.
  //
  // hash = ~hash + (hash << 15);
  mvn(scratch, t0);
  lsl(t0, t0, Operand(15)); // DIFF: codegen
  add(t0, scratch, t0);
  // hash = hash ^ (hash >> 12);
  lsr(scratch, t0, Operand(12)); // DIFF: codegen
  eor(t0, t0, scratch);
  // hash = hash + (hash << 2);
  lsl(scratch, t0, Operand(2)); // DIFF: codegen
  add(t0, t0, scratch);
  // hash = hash ^ (hash >> 4);
  lsr(scratch, t0, Operand(4)); // DIFF: codegen
  eor(t0, t0, scratch);
  // hash = hash * 2057; (2057 == 2048 + 8 + 1)
  // SH4: order sequence for using a single scratch
  // Compute scratch = t0 + t0 * 8
  // Then t0 = t0 * 2048 + scratch (hence t0 = t0 * 2057)
  lsl(scratch, t0, Operand(3)); // DIFF: codegen
  add(scratch, t0, scratch); // DIFF: codegen
  lsl(t0, t0, Operand(11)); // DIFF: codegen
  add(t0, t0, scratch); // DIFF: codegen
  // hash = hash ^ (hash >> 16);
  lsr(scratch, t0, Operand(16));
  eor(t0, t0, scratch);
}


void MacroAssembler::LoadFromNumberDictionary(Label* miss, // REVIEWEDBY: CG
                                              Register elements,
                                              Register key,
                                              Register result,
                                              Register t0,
                                              Register t1,
                                              Register t2) {
  // Register use:
  //
  // elements - holds the slow-case elements of the receiver on entry.
  //            Unchanged unless 'result' is the same register.
  //
  // key      - holds the smi key on entry.
  //            Unchanged unless 'result' is the same register.
  //
  // result   - holds the result on exit if the load succeeded.
  //            Allowed to be the same as 'key' or 'result'.
  //            Unchanged on bailout so 'key' or 'result' can be used
  //            in further computation.
  //
  // Scratch registers:
  //
  // t0 - holds the untagged key on entry and holds the hash once computed.
  //
  // t1 - used to hold the capacity mask of the dictionary
  //
  // t2 - used for the index into the dictionary.
  Label done;

  GetNumberHash(t0, t1);

  // Compute the capacity mask.
  ldr(t1, FieldMemOperand(elements, SeededNumberDictionary::kCapacityOffset));
  SmiUntag(t1);
  sub(t1, t1, Operand(1));

  // Generate an unrolled loop that performs a few probes before giving up.
  for (int i = 0; i < kNumberDictionaryProbes; i++) {
    // Use t2 for index calculations and keep the hash intact in t0.
    mov(t2, t0);
    // Compute the masked index: (hash + i + i * i) & mask.
    if (i > 0) {
      add(t2, t2, Operand(SeededNumberDictionary::GetProbeOffset(i)));
    }
    and_(t2, t2, t1);

    // Scale the index by multiplying by the element size.
    ASSERT(SeededNumberDictionary::kEntrySize == 3);
    lsl(ip, t2, Operand(1));
    add(t2, t2, ip);  // t2 = t2 * 3

    // Check if the key is identical to the name.
    lsl(ip, t2, Operand(kPointerSizeLog2));
    add(t2, elements, ip);
    ldr(ip, FieldMemOperand(t2, SeededNumberDictionary::kElementsStartOffset));
    cmp(key, ip);
    if (i != kNumberDictionaryProbes - 1) {
      b(eq, &done);
    } else {
      b(ne, miss);
    }
  }

  bind(&done);
  // Check that the value is a normal property.
  // t2: elements + (index * kPointerSize)
  const int kDetailsOffset =
      SeededNumberDictionary::kElementsStartOffset + 2 * kPointerSize;
  ldr(t1, FieldMemOperand(t2, kDetailsOffset));
  tst(t1, Operand(Smi::FromInt(PropertyDetails::TypeField::kMask)));
  b(ne, miss);

  // Get the value at the masked, scaled index and return.
  const int kValueOffset =
      SeededNumberDictionary::kElementsStartOffset + kPointerSize;
  ldr(result, FieldMemOperand(t2, kValueOffset));
}


void MacroAssembler::Allocate(int object_size, // REVIEWEDBY: CG
                              Register result,
                              Register scratch1,
                              Register scratch2,
                              Label* gc_required,
                              AllocationFlags flags) {
  ASSERT(object_size <= Page::kMaxRegularHeapObjectSize);
  RECORD_LINE();
  if (!FLAG_inline_new) {
    if (emit_debug_code()) {
      // Trash the registers to simulate an allocation failure.
      RECORD_LINE();
      mov(result, Operand(0x7091));
      mov(scratch1, Operand(0x7191));
      mov(scratch2, Operand(0x7291));
    }
    RECORD_LINE();
    jmp(gc_required);
    return;
  }

  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!scratch1.is(ip));
  ASSERT(!scratch2.is(ip));
  ASSERT(!scratch1.is(sh4_rtmp));
  ASSERT(!scratch2.is(sh4_rtmp));
  ASSERT(ip.is(sh4_ip));

  // Make object size into bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    object_size *= kPointerSize;
  }
  ASSERT_EQ(0, object_size & kObjectAlignmentMask);

  // Check relative positions of allocation top and limit addresses.
  // The values must be adjacent in memory to allow the use of LDM.
  // Also, assert that the registers are numbered such that the values
  // are loaded in the correct order.
  ExternalReference allocation_top =
      AllocationUtils::GetAllocationTopReference(isolate(), flags);
  ExternalReference allocation_limit =
      AllocationUtils::GetAllocationLimitReference(isolate(), flags);

  intptr_t top   =
      reinterpret_cast<intptr_t>(allocation_top.address());
  intptr_t limit =
      reinterpret_cast<intptr_t>(allocation_limit.address());
  ASSERT((limit - top) == kPointerSize);


  // Set up allocation top address register.
  Register topaddr = scratch1;
  RECORD_LINE();
  mov(topaddr, Operand(allocation_top));

  // This code stores a temporary value in ip. This is OK, as the code below
  // does not need ip for implicit literal generation.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    RECORD_LINE();
    // Load allocation top into result and allocation limit into ip.
    ldr(result, MemOperand(topaddr)); // DIFF: codegen
    ldr(ip, MemOperand(topaddr, 4)); // DIFF: codegen
  } else {
    if (emit_debug_code()) {
      RECORD_LINE();
      // Assert that result actually contains top on entry. ip is used
      // immediately below so this use of ip does not cause difference with
      // respect to register content between debug and release mode.
      ldr(ip, MemOperand(topaddr));
      cmp(result, ip);
      Check(eq, kUnexpectedAllocationTop);
    }
    RECORD_LINE();
    // Load allocation limit into ip. Result already contains allocation top.
    ldr(ip, MemOperand(topaddr, limit - top));
  }

  if ((flags & DOUBLE_ALIGNMENT) != 0) {
    // Align the next allocation. Storing the filler map without checking top is
    // safe in new-space because the limit of the heap is aligned there.
    ASSERT((flags & PRETENURE_OLD_POINTER_SPACE) == 0);
    STATIC_ASSERT(kPointerAlignment * 2 == kDoubleAlignment);
    Label aligned;
    tst(result, Operand(kDoubleAlignmentMask)); // DIFF: codegen
    b(t, &aligned); // DIFF: codegen
    if ((flags & PRETENURE_OLD_DATA_SPACE) != 0) {
      cmphs(result, Operand(ip)); // DIFF: codegen
      b(t, gc_required); // DIFF: codegen
    }
    mov(scratch2, Operand(isolate()->factory()->one_pointer_filler_map()));
    str(scratch2, MemOperand(result, kDoubleSize / 2, PostIndex));
    bind(&aligned);
  }

  RECORD_LINE();
  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top.
  // ARM: We must preserve the ip register at this
  // point, so we cannot just use add().
  // SH4: simplified there compared to ARM, as ip is not scratched
  // by addv.
  ASSERT(object_size > 0);
  mov(scratch2, Operand(object_size)); // DIFF: codegen
  addc(scratch2, result, scratch2); // DIFF: codegen
  b(t, gc_required); // DIFF: codegen

  RECORD_LINE();
  cmphi(scratch2, ip); // DIFF: codegen
  bt(gc_required); // DIFF: codegen

  RECORD_LINE();
  str(scratch2, MemOperand(topaddr));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    RECORD_LINE();
    add(result, result, Operand(kHeapObjectTag));
  }
}


void MacroAssembler::Allocate(Register object_size, // REVIEWEDBY: CG
                              Register result,
                              Register scratch1,
                              Register scratch2,
                              Label* gc_required,
                              AllocationFlags flags) {
  RECORD_LINE();
  if (!FLAG_inline_new) {
    if (emit_debug_code()) {
      // Trash the registers to simulate an allocation failure.
      RECORD_LINE();
      mov(result, Operand(0x7091));
      mov(scratch1, Operand(0x7191));
      mov(scratch2, Operand(0x7291));
    }
    RECORD_LINE();
    jmp(gc_required);
    return;
  }

  // Assert that the register arguments are different and that none of
  // them are ip. ip is used explicitly in the code generated below.
  // Also assert that rtmp is not used as it is used in assembler-sh4.cc.
  ASSERT(!result.is(scratch1));
  ASSERT(!result.is(scratch2));
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!object_size.is(ip));
  ASSERT(!result.is(ip));
  ASSERT(!scratch1.is(ip));
  ASSERT(!scratch2.is(ip));
  ASSERT(!object_size.is(sh4_rtmp));
  ASSERT(!result.is(sh4_rtmp));
  ASSERT(!scratch1.is(sh4_rtmp));
  ASSERT(!scratch2.is(sh4_rtmp));
  ASSERT(ip.is(sh4_ip));

  // Check relative positions of allocation top and limit addresses.
  // The values must be adjacent in memory to allow the use of LDM.
  // Also, assert that the registers are numbered such that the values
  // are loaded in the correct order.
  ExternalReference allocation_top =
      AllocationUtils::GetAllocationTopReference(isolate(), flags);
  ExternalReference allocation_limit =
      AllocationUtils::GetAllocationLimitReference(isolate(), flags);
  intptr_t top =
      reinterpret_cast<intptr_t>(allocation_top.address());
  intptr_t limit =
      reinterpret_cast<intptr_t>(allocation_limit.address());
  ASSERT((limit - top) == kPointerSize);

  // Set up allocation top address.
  RECORD_LINE();
  Register topaddr = scratch1;
  mov(topaddr, Operand(allocation_top));

  // This code stores a temporary value in ip. This is OK, as the code below
  // does not need ip for implicit literal generation.
  if ((flags & RESULT_CONTAINS_TOP) == 0) {
    RECORD_LINE();
    // Load allocation top into result and allocation limit into ip.
    ldr(result, MemOperand(topaddr)); // DIFF: codegen
    ldr(sh4_ip, MemOperand(topaddr, 4)); // DIFF: codegen
  } else {
    if (emit_debug_code()) {
      RECORD_LINE();
      // Assert that result actually contains top on entry. ip is used
      // immediately below so this use of ip does not cause difference with
      // respect to register content between debug and release mode.
      ldr(ip, MemOperand(topaddr));
      cmp(result, ip);
      Check(eq, kUnexpectedAllocationTop);
    }
    RECORD_LINE();
    // Load allocation limit into ip. Result already contains allocation top.
    ldr(ip, MemOperand(topaddr, limit - top));
  }

  if ((flags & DOUBLE_ALIGNMENT) != 0) {
    // Align the next allocation. Storing the filler map without checking top is
    // safe in new-space because the limit of the heap is aligned there.
    ASSERT((flags & PRETENURE_OLD_POINTER_SPACE) == 0);
    ASSERT(kPointerAlignment * 2 == kDoubleAlignment);
    Label aligned;
    tst(result, Operand(kDoubleAlignmentMask)); // DIFF: codegen
    b(eq, &aligned);
    if ((flags & PRETENURE_OLD_DATA_SPACE) != 0) {
      cmphs(result, Operand(ip)); // DIFF: codegen
      b(t, gc_required); // DIFF/ codegen
    }
    mov(scratch2, Operand(isolate()->factory()->one_pointer_filler_map()));
    str(scratch2, MemOperand(result, kDoubleSize / 2, PostIndex));
    bind(&aligned);
  }

  // Calculate new top and bail out if new space is exhausted. Use result
  // to calculate the new top. Object size may be in words so a shift is
  // required to get the number of bytes.
  if ((flags & SIZE_IN_WORDS) != 0) {
    RECORD_LINE();
    lsl(scratch2, object_size, Operand(kPointerSizeLog2)); // DIFF: codegen
    addc(scratch2, result, scratch2); // DIFF: codegen
  } else {
    RECORD_LINE();
    addc(scratch2, result, object_size); // DIFF: codegen
  }
  RECORD_LINE();
  b(t, gc_required); // DIFF: codegen
  RECORD_LINE();
  cmphi(scratch2, ip); // DIFF: codegen
  b(t, gc_required); // DIFF: codegen
  RECORD_LINE();

  // Update allocation top. result temporarily holds the new top.
  if (emit_debug_code()) {
    RECORD_LINE();
    tst(scratch2, Operand(kObjectAlignmentMask));
    Check(eq, kUnalignedAllocationInNewSpace);
  }
  RECORD_LINE();
  str(scratch2, MemOperand(topaddr));

  // Tag object if requested.
  if ((flags & TAG_OBJECT) != 0) {
    RECORD_LINE();
    add(result, result, Operand(kHeapObjectTag));
  }
}


void MacroAssembler::UndoAllocationInNewSpace(Register object, // REVIEWEDBY: CG
                                              Register scratch) {
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address(isolate());

  // Make sure the object has no tag before resetting top.
  and_(object, object, Operand(~kHeapObjectTagMask));
#ifdef DEBUG
  // Check that the object un-allocated is below the current top.
  mov(scratch, Operand(new_space_allocation_top));
  ldr(scratch, MemOperand(scratch));
  cmpge(object, scratch);
  Check(f, kUndoAllocationOfNonAllocatedMemory);
#endif
  // Write the address of the object to un-allocate as the current top.
  mov(scratch, Operand(new_space_allocation_top));
  str(object, MemOperand(scratch));
}


void MacroAssembler::AllocateTwoByteString(Register result, // REVIEWEDBY: CG
                                           Register length,
                                           Register scratch1,
                                           Register scratch2,
                                           Register scratch3,
                                           Label* gc_required) {
  // Calculate the number of bytes needed for the characters in the string while
  // observing object alignment.
  ASSERT((SeqTwoByteString::kHeaderSize & kObjectAlignmentMask) == 0);
  RECORD_LINE();
  lsl(scratch1, length, Operand(1));  // Length in bytes, not chars. // DIFF: codegen
  add(scratch1, scratch1,
      Operand(kObjectAlignmentMask + SeqTwoByteString::kHeaderSize));
  and_(scratch1, scratch1, Operand(~kObjectAlignmentMask));

  // Allocate two-byte string in new space.
  Allocate(scratch1,
           result,
           scratch2,
           scratch3,
           gc_required,
           TAG_OBJECT);

  // Set the map, length and hash field.
  RECORD_LINE();
  InitializeNewString(result,
                      length,
                      Heap::kStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateAsciiString(Register result, // REVIEWEDBY: CG
                                         Register length,
                                         Register scratch1,
                                         Register scratch2,
                                         Register scratch3,
                                         Label* gc_required) {
  // Calculate the number of bytes needed for the characters in the string while
  // observing object alignment.
  ASSERT((SeqOneByteString::kHeaderSize & kObjectAlignmentMask) == 0);
  ASSERT(kCharSize == 1);
  RECORD_LINE();
  add(scratch1, length,
      Operand(kObjectAlignmentMask + SeqOneByteString::kHeaderSize));
  and_(scratch1, scratch1, Operand(~kObjectAlignmentMask));

  // Allocate ASCII string in new space.
  Allocate(scratch1,
           result,
           scratch2,
           scratch3,
           gc_required,
           TAG_OBJECT);

  RECORD_LINE();
  // Set the map, length and hash field.
  InitializeNewString(result,
                      length,
                      Heap::kAsciiStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateTwoByteConsString(Register result, // REVIEWEDBY: CG
                                               Register length,
                                               Register scratch1,
                                               Register scratch2,
                                               Label* gc_required) {
  RECORD_LINE();
  Allocate(ConsString::kSize, result, scratch1, scratch2, gc_required,
           TAG_OBJECT);

  RECORD_LINE();
  InitializeNewString(result,
                      length,
                      Heap::kConsStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateAsciiConsString(Register result, // REVIEWEDBY: CG
                                             Register length,
                                             Register scratch1,
                                             Register scratch2,
                                             Label* gc_required) {
  Label allocate_new_space, install_map;
  AllocationFlags flags = TAG_OBJECT;

  ExternalReference high_promotion_mode = ExternalReference::
      new_space_high_promotion_mode_active_address(isolate());
  mov(scratch1, Operand(high_promotion_mode));
  ldr(scratch1, MemOperand(scratch1, 0));
  cmp(scratch1, Operand::Zero());
  b(eq, &allocate_new_space);

  Allocate(ConsString::kSize,
           result,
           scratch1,
           scratch2,
           gc_required,
           static_cast<AllocationFlags>(flags | PRETENURE_OLD_POINTER_SPACE));

  jmp(&install_map);

  bind(&allocate_new_space);
  Allocate(ConsString::kSize,
           result,
           scratch1,
           scratch2,
           gc_required,
           flags);

  bind(&install_map);

  InitializeNewString(result,
                      length,
                      Heap::kConsAsciiStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateTwoByteSlicedString(Register result, // REVIEWEDBY: CG
                                                 Register length,
                                                 Register scratch1,
                                                 Register scratch2,
                                                 Label* gc_required) {
  Allocate(SlicedString::kSize, result, scratch1, scratch2, gc_required,
           TAG_OBJECT);

  InitializeNewString(result,
                      length,
                      Heap::kSlicedStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::AllocateAsciiSlicedString(Register result, // REVIEWEDBY: CG
                                               Register length,
                                               Register scratch1,
                                               Register scratch2,
                                               Label* gc_required) {
  Allocate(SlicedString::kSize, result, scratch1, scratch2, gc_required,
           TAG_OBJECT);

  InitializeNewString(result,
                      length,
                      Heap::kSlicedAsciiStringMapRootIndex,
                      scratch1,
                      scratch2);
}


void MacroAssembler::CompareObjectType(Register object, // REVIEWEDBY: CG
                                       Register map,
                                       Register type_reg,
                                       InstanceType type,
                                       Condition cond) {
  ASSERT(!object.is(sh4_ip) && !map.is(sh4_ip) && !type_reg.is(sh4_ip));
  ASSERT(!object.is(sh4_rtmp) && !map.is(sh4_rtmp) && !type_reg.is(sh4_rtmp));

  RECORD_LINE();
  ldr(map, FieldMemOperand(object, HeapObject::kMapOffset));
  CompareInstanceType(map, type_reg, type, cond);
}


void MacroAssembler::CompareInstanceType(Register map, // REVIEWEDBY: CG
                                         Register type_reg,
                                         InstanceType type,
                                         Condition cond) {
  Condition cond_to_test = cond;
  // Registers map and type_reg can be ip. These two lines assert
  // that ip can be used with the two instructions (the constants
  // will never need ip).
  STATIC_ASSERT(Map::kInstanceTypeOffset < 4096);
  STATIC_ASSERT(LAST_TYPE < 256);
  ASSERT(!map.is(sh4_rtmp) && !type_reg.is(sh4_rtmp));

  RECORD_LINE();
  ldrb(type_reg, FieldMemOperand(map, Map::kInstanceTypeOffset));
  cmp(&cond_to_test, type_reg, Operand(type));
  ASSERT(cond_to_test == t); // SH4: accept only conditions that can test againt true
}


void MacroAssembler::CompareRoot(Register obj, // REVIEWEDBY: CG
                                 Heap::RootListIndex index) {
  ASSERT(!obj.is(sh4_ip));
  ASSERT(!obj.is(sh4_rtmp));
  RECORD_LINE();
  LoadRoot(ip, index);
  cmpeq(obj, ip);
}


void MacroAssembler::CheckFastElements(Register map, // REVIEWEDBY: CG
                                       Register scratch,
                                       Label* fail) {
  STATIC_ASSERT(FAST_SMI_ELEMENTS == 0);
  STATIC_ASSERT(FAST_HOLEY_SMI_ELEMENTS == 1);
  STATIC_ASSERT(FAST_ELEMENTS == 2);
  STATIC_ASSERT(FAST_HOLEY_ELEMENTS == 3);
  ldrb(scratch, FieldMemOperand(map, Map::kBitField2Offset));
  cmphi(scratch, Operand(Map::kMaximumBitField2FastHoleyElementValue)); // DIFF: codegen
  bt(fail); // DIFF: codegen
}


void MacroAssembler::CheckFastObjectElements(Register map, // REVIEWEDBY: CG
                                             Register scratch,
                                             Label* fail) {
  STATIC_ASSERT(FAST_SMI_ELEMENTS == 0);
  STATIC_ASSERT(FAST_HOLEY_SMI_ELEMENTS == 1);
  STATIC_ASSERT(FAST_ELEMENTS == 2);
  STATIC_ASSERT(FAST_HOLEY_ELEMENTS == 3);
  ldrb(scratch, FieldMemOperand(map, Map::kBitField2Offset));
  cmphi(scratch, Operand(Map::kMaximumBitField2FastHoleySmiElementValue));
  bf(fail);
  cmphi(scratch, Operand(Map::kMaximumBitField2FastHoleyElementValue));
  bt(fail);
}


void MacroAssembler::CheckFastSmiElements(Register map, // REVIEWEDBY: CG
                                          Register scratch,
                                          Label* fail) {
  STATIC_ASSERT(FAST_SMI_ELEMENTS == 0);
  STATIC_ASSERT(FAST_HOLEY_SMI_ELEMENTS == 1);
  ldrb(scratch, FieldMemOperand(map, Map::kBitField2Offset));
  cmphi(scratch, Operand(Map::kMaximumBitField2FastHoleySmiElementValue));
  bt(fail);
}


void MacroAssembler::StoreNumberToDoubleElements( // REVIEWEDBY: CG
                                      Register value_reg,
                                      Register key_reg,
                                      Register elements_reg,
                                      Register scratch1,
                                      DwVfpRegister double_scratch,
                                      Label* fail,
                                      int elements_offset) {
  Label smi_value, store;

  // Handle smi values specially.
  JumpIfSmi(value_reg, &smi_value);

  // Ensure that the object is a heap number
  CheckMap(value_reg,
           scratch1,
           isolate()->factory()->heap_number_map(),
           fail,
           DONT_DO_SMI_CHECK);

  vldr(double_scratch, FieldMemOperand(value_reg, HeapNumber::kValueOffset));
  // Force a canonical NaN.
  // SH4: no need for NaN mode check
  // if (emit_debug_code()) {
  //   vmrs(ip);
  //   tst(ip, Operand(kVFPDefaultNaNModeControlBit));
  //   Assert(ne, kDefaultNaNModeNotSet);
  // }
  VFPCanonicalizeNaN(double_scratch);
  b(&store);

  bind(&smi_value);
  SmiToDouble(double_scratch, value_reg);

  bind(&store);
  GetDoubleOffsetFromSmiKey(scratch1, key_reg);
  add(scratch1, elements_reg, scratch1);
  vstr(double_scratch,
       FieldMemOperand(scratch1,
                       FixedDoubleArray::kHeaderSize - elements_offset));
}


void MacroAssembler::CompareMap(Register obj, // REVIEWEDBY: CG
                                Register scratch,
                                Handle<Map> map,
                                Label* early_success) {
  ldr(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  CompareMap(scratch, map, early_success);
}


void MacroAssembler::CompareMap(Register obj_map, // REVIEWEDBY: CG
                                Handle<Map> map,
                                Label* early_success) {
  cmp(obj_map, Operand(map));
}


void MacroAssembler::CheckMap(Register obj, // REVIEWEDBY: CG
                              Register scratch,
                              Handle<Map> map,
                              Label* fail,
                              SmiCheckType smi_check_type) {
  ASSERT(!obj.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!obj.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  if (smi_check_type == DO_SMI_CHECK) {
    RECORD_LINE();
    JumpIfSmi(obj, fail);
  }
  RECORD_LINE();
  Label success;
  CompareMap(obj, scratch, map, &success);
  b(ne, fail);
  bind(&success);
}


void MacroAssembler::CheckMap(Register obj, // REVIEWEDBY: CG
                              Register scratch,
                              Heap::RootListIndex index,
                              Label* fail,
                              SmiCheckType smi_check_type) {
  ASSERT(!obj.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!obj.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  if (smi_check_type == DO_SMI_CHECK) {
    RECORD_LINE();
    JumpIfSmi(obj, fail);
  }
  RECORD_LINE();
  ldr(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  LoadRoot(ip, index);
  cmp(scratch, ip);
  b(ne, fail);
}


void MacroAssembler::DispatchMap(Register obj, // REVIEWEDBY: CG
                                 Register scratch,
                                 Handle<Map> map,
                                 Handle<Code> success,
                                 SmiCheckType smi_check_type) {
  Label fail;
  if (smi_check_type == DO_SMI_CHECK) {
    JumpIfSmi(obj, &fail, Label::kNear);
  }
  ldr(scratch, FieldMemOperand(obj, HeapObject::kMapOffset));
  mov(ip, Operand(map));
  cmp(scratch, ip);
  Jump(success, RelocInfo::CODE_TARGET, eq);
  bind(&fail);
}


void MacroAssembler::TryGetFunctionPrototype(Register function, // REVIEWEDBY: CG
                                             Register result,
                                             Register scratch,
                                             Label* miss,
                                             bool miss_on_bound_function) {
  ASSERT(!function.is(sh4_ip) && !result.is(sh4_ip) && !scratch.is(sh4_ip));
  ASSERT(!function.is(sh4_rtmp) && !result.is(sh4_rtmp) && !scratch.is(sh4_rtmp));

  RECORD_LINE();
  // Check that the receiver isn't a smi.
  JumpIfSmi(function, miss);

  // Check that the function really is a function.  Load map into result reg.
  CompareObjectType(function, result, scratch, JS_FUNCTION_TYPE, eq);
  bf(miss);

  if (miss_on_bound_function) {
    RECORD_LINE();
    ldr(scratch,
        FieldMemOperand(function, JSFunction::kSharedFunctionInfoOffset));
    ldr(scratch,
        FieldMemOperand(scratch, SharedFunctionInfo::kCompilerHintsOffset));
    tst(scratch,
        Operand(Smi::FromInt(1 << SharedFunctionInfo::kBoundFunction)));
    b(ne, miss);
  }

  RECORD_LINE();
  // Make sure that the function has an instance prototype.
  Label non_instance;
  ldrb(scratch, FieldMemOperand(result, Map::kBitFieldOffset));
  tst(scratch, Operand(1 << Map::kHasNonInstancePrototype));
  bf_near(&non_instance);

  RECORD_LINE();
  // Get the prototype or initial map from the function.
  ldr(result,
      FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));

  // If the prototype or initial map is the hole, don't return it and
  // simply miss the cache instead. This will allow us to allocate a
  // prototype object on-demand in the runtime system.
  LoadRoot(sh4_ip, Heap::kTheHoleValueRootIndex);
  cmpeq(result, sh4_ip);
  bt(miss);

  RECORD_LINE();
  // If the function does not have an initial map, we're done.
  Label done;
  CompareObjectType(result, scratch, scratch, MAP_TYPE, eq);
  bf_near(&done);

  RECORD_LINE();
  // Get the prototype from the initial map.
  ldr(result, FieldMemOperand(result, Map::kPrototypeOffset));
  jmp_near(&done);

  RECORD_LINE();
  // Non-instance prototype: Fetch prototype from constructor field
  // in initial map.
  bind(&non_instance);
  ldr(result, FieldMemOperand(result, Map::kConstructorOffset));

  // All done.
  bind(&done);
}


void MacroAssembler::CallStub(CodeStub* stub, // REVIEWEDBY: CG
                              TypeFeedbackId ast_id) {
  ASSERT(AllowThisStubCall(stub));  // Stub calls are not allowed in some stubs.
  RECORD_LINE();
  Call(stub->GetCode(), RelocInfo::CODE_TARGET, ast_id);
}


void MacroAssembler::TailCallStub(CodeStub* stub, Condition cond) { // REVIEWEDBY: CG
  Jump(stub->GetCode(), RelocInfo::CODE_TARGET, cond);
}


static int AddressOffset(ExternalReference ref0, ExternalReference ref1) { // REVIEWEDBY: CG
  return ref0.address() - ref1.address();
}


void MacroAssembler::CallApiFunctionAndReturn( // REVIEWEDBY: CG
    Register function_address,
    ExternalReference thunk_ref,
    int stack_space,
    MemOperand return_value_operand,
    MemOperand* context_restore_operand) {
  ExternalReference next_address =
      ExternalReference::handle_scope_next_address(isolate());
  const int kNextOffset = 0;
  const int kLimitOffset = AddressOffset(
      ExternalReference::handle_scope_limit_address(isolate()),
      next_address);
  const int kLevelOffset = AddressOffset(
      ExternalReference::handle_scope_level_address(isolate()),
      next_address);

  // ARM code does:
  // - Pass arguments registers in r0,r1,r2 (no more than 3)
  // - Compute and call function address through r3
  // - Preserve some registers through: r4,r5,r6,r9
  // - get result register r0
  // SH4 code does:
  // - Pass arguments registers in r4,r5,r6 (moved from r0,r1,r2)
  // - Compute and call function address through r3
  // - Preserve ARM callee-saved registers into SH4 calle-saved
  //    ARM(r4,r5,r6,r9) <=> SH4(r8, r10, r11, r9)
  // - get result register r0

  ASSERT(function_address.is(r1) || function_address.is(r2));

  Label profiler_disabled;
  Label end_profiler_check;
  mov(r9, Operand(ExternalReference::is_profiling_address(isolate())));
  ldrb(r9, MemOperand(r9, 0));
  cmp(r9, Operand(0));
  b(eq, &profiler_disabled);

  // Additional parameter is the address of the actual callback.
  mov(r3, Operand(thunk_ref));
  jmp(&end_profiler_check);

  bind(&profiler_disabled);
  Move(r3, function_address);
  bind(&end_profiler_check);

  // Allocate HandleScope in callee-save registers.
  mov(r9, Operand(next_address));
  ldr(r4, MemOperand(r9, kNextOffset));
  ldr(r5, MemOperand(r9, kLimitOffset));
  ldr(r6, MemOperand(r9, kLevelOffset));
  add(r6, r6, Operand(1));
  str(r6, MemOperand(r9, kLevelOffset));

  if (FLAG_log_timer_events) {
    FrameScope frame(this, StackFrame::MANUAL);
    PushSafepointRegisters();
    PrepareCallCFunction(1, r0);
    mov(sh4_r4/*r0*/, Operand(ExternalReference::isolate_address(isolate())));
    CallCFunction(ExternalReference::log_enter_external_function(isolate()), 1);
    PopSafepointRegisters();
  }

  // SH4: Adapt calling convention ARM -> SH4 (entry)
  //  Move ARM callee-saved prepared above into SH4 callee saved
  //  ARM: r4, r5, r6, r9 -> SH4: r8, r10, r11, r9
  // Note: r10 (ip) and r11 (rtmp) must not be cloberred from now on
  // up to the return of DirectCEntryStub.GenerateCall() below.
  mov(sh4_r8, r4);
  mov(sh4_r10, r5);
  mov(sh4_r11, r6);
  mov(sh4_r9, r9);

  //  Move ARM arguments to SH4 arguments (3 first only)
  mov(sh4_r4, r0);
  mov(sh4_r5, r1);
  mov(sh4_r6, r2);

  // Native call returns to the DirectCEntry stub which redirects to the
  // return address pushed on stack (could have moved after GC).
  // DirectCEntry stub itself is generated early and never moves.
  DirectCEntryStub stub(isolate());
  stub.GenerateCall(this, r3);

  // SH4: Adapt calling convention SH4 -> ARM (exit)
  //  Move back SH4 callee-saved into ARM calle saved
  mov(r4, sh4_r8);
  mov(r5, sh4_r10);
  mov(r6, sh4_r11);
  mov(r9, sh4_r9);

  if (FLAG_log_timer_events) {
    FrameScope frame(this, StackFrame::MANUAL);
    PushSafepointRegisters();
    PrepareCallCFunction(1, r0);
    mov(sh4_r4/*r0*/, Operand(ExternalReference::isolate_address(isolate())));
    CallCFunction(ExternalReference::log_leave_external_function(isolate()), 1);
    PopSafepointRegisters();
  }

  Label promote_scheduled_exception;
  Label exception_handled;
  Label delete_allocated_handles;
  Label leave_exit_frame;
  Label return_value_loaded;

  // load value from ReturnValue
  ldr(r0, return_value_operand);
  bind(&return_value_loaded);
  // No more valid handles (the result handle was the last one). Restore
  // previous handle scope.
  str(r4, MemOperand(r9, kNextOffset));
  if (emit_debug_code()) {
    ldr(r1, MemOperand(r9, kLevelOffset));
    cmp(r1, r6);
    Check(eq, kUnexpectedLevelAfterReturnFromApiCall);
  }
  sub(r6, r6, Operand(1));
  str(r6, MemOperand(r9, kLevelOffset));
  ldr(ip, MemOperand(r9, kLimitOffset));
  cmp(r5, ip);
  b(ne, &delete_allocated_handles);

  // Check if the function scheduled an exception.
  bind(&leave_exit_frame);
  LoadRoot(r4, Heap::kTheHoleValueRootIndex);
  mov(ip, Operand(ExternalReference::scheduled_exception_address(isolate())));
  ldr(r5, MemOperand(ip));
  cmp(r4, r5);
  b(ne, &promote_scheduled_exception);
  bind(&exception_handled);

  bool restore_context = context_restore_operand != NULL;
  if (restore_context) {
    ldr(cp, *context_restore_operand);
  }
  // LeaveExitFrame expects unwind space to be in a register.
  mov(r4, Operand(stack_space));
  LeaveExitFrame(false, r4, !restore_context);
  rts(); // DIFF: codegen

  bind(&promote_scheduled_exception);
  {
    FrameScope frame(this, StackFrame::INTERNAL);
    CallExternalReference(
        ExternalReference(Runtime::kHiddenPromoteScheduledException, isolate()),
        0);
  }
  jmp(&exception_handled);

  // HandleScope limit has changed. Delete allocated extensions.
  bind(&delete_allocated_handles);
  str(r5, MemOperand(r9, kLimitOffset));
  mov(sh4_r8/*r4*/, r0); // SH4: preserve return value in callee-saved r8
  PrepareCallCFunction(1, r5);
  mov(sh4_r4/*r0*/, Operand(ExternalReference::isolate_address(isolate())));
  CallCFunction(
      ExternalReference::delete_handle_scope_extensions(isolate()), 1);
  mov(r0, sh4_r8/*r4*/); // SH4: restore return value
  jmp(&leave_exit_frame);
}


bool MacroAssembler::AllowThisStubCall(CodeStub* stub) { // REVIEWEDBY: CG
  return has_frame_ || !stub->SometimesSetsUpAFrame();
}

void MacroAssembler::IndexFromHash(Register hash, Register index) { // REVIEWEDBY: CG
  // If the hash field contains an array index pick it out. The assert checks
  // that the constants for the maximum number of digits for an array index
  // cached in the hash field and the number of bits reserved for it does not
  // conflict.
  ASSERT(TenToThe(String::kMaxCachedArrayIndexLength) <
         (1 << String::kArrayIndexValueBits));
  // We want the smi-tagged index in key.  kArrayIndexValueMask has zeros in
  // the low kHashShift bits.
  STATIC_ASSERT(kSmiTag == 0);
  ASSERT(!hash.is(sh4_ip) && !index.is(sh4_ip));
  ASSERT(!hash.is(sh4_rtmp) && !index.is(sh4_rtmp));
  RECORD_LINE();
  Ubfx(hash, hash, String::kHashShift, String::kArrayIndexValueBits);
  SmiTag(index, hash);
}


void MacroAssembler::SmiToDouble(DwVfpRegister value, Register smi) { // REVIEWEDBY: CG
  ASSERT(!smi.is(ip));
  // ip is used as scratch here, be careful at call site
  SmiUntag(ip, smi);
  vcvt_f64_s32(value, ip);
}

void MacroAssembler::TestDoubleIsInt32(DwVfpRegister double_input, // REVIEWEDBY: CG
                                       DwVfpRegister double_scratch) {
  ASSERT(!double_input.is(double_scratch));
  // SH4: needs an additional scratch register, save/restore ip
  push(ip);
  vcvt_s32_f64(ip, double_input);
  vcvt_f64_s32(double_scratch, ip);
  pop(ip);
  dcmpeq(double_input, double_scratch);
  // The T bit is cleared if not an int32 (ref to TryDoubleToInt32Exact)
}


void MacroAssembler::TryDoubleToInt32Exact(Register result, // REVIEWEDBY: CG
                                           DwVfpRegister double_input,
                                           DwVfpRegister double_scratch) {
  ASSERT(!double_input.is(double_scratch));
  vcvt_s32_f64(result, double_input);
  vcvt_f64_s32(double_scratch, result);
  dcmpeq(double_input, double_scratch);
  // The T bit will be cleared if not exact (i.e. test with ne or f)
  // as in:
  // TryDoubleToInt32Exact(...)
  // b(ne, &not_exact);
}


void MacroAssembler::TryInt32Floor(Register result, // SH4: TODO
                                   DwVfpRegister double_input,
                                   Register input_high,
                                   DwVfpRegister double_scratch,
                                   Label* done,
                                   Label* exact) {
  UNIMPLEMENTED();
}


void MacroAssembler::TryInlineTruncateDoubleToI(Register result, // REVIEWEDBY: CG
                                                DwVfpRegister double_input,
                                                Label* done) {
  vcvt_s32_f64(result, double_input); // DIFF: codegen

  // If result is not saturated (0x7fffffff or 0x80000000), we are done.
  sub(ip, result, Operand(1));
  cmpge(ip, Operand(0x7ffffffe)); // DIFF; codegen
  b(f, done); // DIFF; codegen
}


void MacroAssembler::TruncateDoubleToI(Register result, // REVIEWEDBY: CG
                                       DwVfpRegister double_input) {
  Label done;

  TryInlineTruncateDoubleToI(result, double_input, &done);

  // If we fell through then inline version didn't succeed - call stub instead.
  push(lr);
  sub(sp, sp, Operand(kDoubleSize));  // Put input on stack.
  vstr(double_input, MemOperand(sp, 0));

  DoubleToIStub stub(isolate(), sp, result, 0, true, true);
  CallStub(&stub);

  add(sp, sp, Operand(kDoubleSize));
  pop(lr);

  bind(&done);
}


void MacroAssembler::TruncateHeapNumberToI(Register result, // REVIEWEDBY: CG
                                           Register object) {
  Label done;
  DwVfpRegister double_scratch = kScratchDoubleReg;
  ASSERT(!result.is(object));

  vldr(double_scratch,
       MemOperand(object, HeapNumber::kValueOffset - kHeapObjectTag));
  TryInlineTruncateDoubleToI(result, double_scratch, &done);

  // If we fell through then inline version didn't succeed - call stub instead.
  push(lr);
  DoubleToIStub stub(isolate(),
                     object,
                     result,
                     HeapNumber::kValueOffset - kHeapObjectTag,
                     true,
                     true);
  CallStub(&stub);
  pop(lr);

  bind(&done);
}


void MacroAssembler::TruncateNumberToI(Register object, // REVIEWEDBY: CG
                                       Register result,
                                       Register heap_number_map,
                                       Register scratch1,
                                       Label* not_number) {
  Label done;
  ASSERT(!result.is(object));

  UntagAndJumpIfSmi(result, object, &done);
  JumpIfNotHeapNumber(object, heap_number_map, scratch1, not_number);
  TruncateHeapNumberToI(result, object);

  bind(&done);
}


void MacroAssembler::GetLeastBitsFromSmi(Register dst, // REVIEWEDBY: CG
                                         Register src,
                                         int num_least_bits) {
  Ubfx(dst, src, kSmiTagSize, num_least_bits);
}


void MacroAssembler::GetLeastBitsFromInt32(Register dst, // REVIEWEDBY: CG
                                           Register src,
                                           int num_least_bits) {
  ASSERT(!dst.is(sh4_rtmp) && !src.is(sh4_rtmp));
  and_(dst, src, Operand((1 << num_least_bits) - 1));
}


void MacroAssembler::CallRuntime(const Runtime::Function* f, // REVIEWEDBY: CG
                                 int num_arguments,
                                 SaveFPRegsMode save_doubles) {
  // All parameters are on the stack.  r0 has the return value after call.
#ifdef DEBUG
  // Clobber parameter registers on entry (except r0, r1 defined below).
  Dead(r2, r3);
  Dead(r4, r5, r6, r7);
#endif

  // If the expected number of arguments of the runtime function is
  // constant, we check that the actual number of arguments match the
  // expectation.
  RECORD_LINE();
  CHECK(f->nargs < 0 || f->nargs == num_arguments);

  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.
  mov(r0, Operand(num_arguments));
  mov(r1, Operand(ExternalReference(f, isolate())));
  CEntryStub stub(isolate(), 1, save_doubles);
  CallStub(&stub);
}


void MacroAssembler::CallExternalReference(const ExternalReference& ext, // REVIEWEDBY: CG
                                           int num_arguments) {
  mov(r0, Operand(num_arguments));
  mov(r1, Operand(ext));

  CEntryStub stub(isolate(), 1);
  CallStub(&stub);
}


void MacroAssembler::TailCallExternalReference(const ExternalReference& ext, // REVIEWEDBY: CG
                                               int num_arguments,
                                               int result_size) {
  // TODO(1236192): Most runtime routines don't need the number of
  // arguments passed in because it is constant. At some point we
  // should remove this need and make the runtime routine entry code
  // smarter.

  // Block constant pool when emitting call (might be redundant)
  BlockConstPoolScope block_const_pool(this);

  RECORD_LINE();
  mov(r0, Operand(num_arguments));
  JumpToExternalReference(ext);
}


void MacroAssembler::TailCallRuntime(Runtime::FunctionId fid, // REVIEWEDBY: CG
                                     int num_arguments,
                                     int result_size) {
  RECORD_LINE();
  TailCallExternalReference(ExternalReference(fid, isolate()),
                            num_arguments,
                            result_size);
}


void MacroAssembler::JumpToExternalReference(const ExternalReference& builtin) { // REVIEWEDBY: CG
  RECORD_LINE();
  mov(r1, Operand(builtin));
  CEntryStub stub(isolate(), 1);
  RECORD_LINE();
  Jump(stub.GetCode(), RelocInfo::CODE_TARGET);
}


void MacroAssembler::InvokeBuiltin(Builtins::JavaScript id, // REVIEWEDBY: CG
                                   InvokeFlag flag,
                                   const CallWrapper& call_wrapper) {
  // You can't call a builtin without a valid frame.
  ASSERT(flag == JUMP_FUNCTION || has_frame());
  // No register conventions on entry.
  // All parameters are on stack.
  // Return value in r0 after call.
#ifdef DEBUG
  // Clobber parameter registers on entry (except r2 defined below).
  Dead(r0, r1, r3);
  Dead(r4, r5, r6, r7);
#endif

  RECORD_LINE();
  GetBuiltinEntry(r2, id);
  if (flag == CALL_FUNCTION) {
    RECORD_LINE();
    call_wrapper.BeforeCall(CallSize(r2, pc_offset()));
    Call(r2);
    call_wrapper.AfterCall();
  } else {
    ASSERT(flag == JUMP_FUNCTION);
    RECORD_LINE();
    Jump(r2);
  }
}


void MacroAssembler::GetBuiltinFunction(Register target, // REVIEWEDBY: CG
                                        Builtins::JavaScript id) {
  ASSERT(!target.is(sh4_ip));
  ASSERT(!target.is(sh4_rtmp));
  RECORD_LINE();
  // Load the builtins object into target register.
  ldr(target,
      MemOperand(cp, Context::SlotOffset(Context::GLOBAL_OBJECT_INDEX)));
  ldr(target, FieldMemOperand(target, GlobalObject::kBuiltinsOffset));
  // Load the JavaScript builtin function from the builtins object.
  ldr(target, FieldMemOperand(target,
                          JSBuiltinsObject::OffsetOfFunctionWithId(id)));
}


void MacroAssembler::GetBuiltinEntry(Register target, Builtins::JavaScript id) { // REVIEWEDBY: CG
  // SH4: r1 is used as scratch here, same as for ARM.
  ASSERT(!target.is(r1));
  ASSERT(!target.is(sh4_rtmp));
  ASSERT(!target.is(sh4_ip));
  RECORD_LINE();
  GetBuiltinFunction(r1, id);
  RECORD_LINE();
  // Load the code entry point from the builtins object.
  ldr(target, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
}


void MacroAssembler::SetCounter(StatsCounter* counter, int value, // REVIEWEDBY: CG
                                Register scratch1, Register scratch2) {
  RECORD_LINE();
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  ASSERT(!scratch1.is(sh4_ip) && !scratch2.is(sh4_ip));
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch1, Operand(value));
    mov(scratch2, Operand(ExternalReference(counter)));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::IncrementCounter(StatsCounter* counter, int value, // REVIEWEDBY: CG
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  ASSERT(!scratch1.is(sh4_ip) && !scratch2.is(sh4_ip));
  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch2, Operand(ExternalReference(counter)));
    ldr(scratch1, MemOperand(scratch2));
    add(scratch1, scratch1, Operand(value));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::DecrementCounter(StatsCounter* counter, int value, // REVIEWEDBY: CG
                                      Register scratch1, Register scratch2) {
  ASSERT(value > 0);
  ASSERT(!scratch1.is(scratch2));
  ASSERT(!scratch1.is(sh4_rtmp) && !scratch2.is(sh4_rtmp));
  ASSERT(!scratch1.is(sh4_ip) && !scratch2.is(sh4_ip));

  RECORD_LINE();
  if (FLAG_native_code_counters && counter->Enabled()) {
    RECORD_LINE();
    mov(scratch2, Operand(ExternalReference(counter)));
    ldr(scratch1, MemOperand(scratch2));
    sub(scratch1, scratch1, Operand(value));
    str(scratch1, MemOperand(scratch2));
  }
}


void MacroAssembler::Assert(Condition cond, BailoutReason reason) { // REVIEWEDBY: CG
  ASSERT(cond == ne || cond == eq); // Limitation of sh4 Check()
  if (emit_debug_code())
    Check(cond, reason);
}


void MacroAssembler::AssertFastElements(Register elements) { // REVIEWEDBY: CG
  RECORD_LINE();
  if (emit_debug_code()) {
    ASSERT(!elements.is(sh4_rtmp));
    ASSERT(!elements.is(sh4_ip));
    Label ok;
    RECORD_LINE();
    push(elements);
    ldr(elements, FieldMemOperand(elements, HeapObject::kMapOffset));
    LoadRoot(ip, Heap::kFixedArrayMapRootIndex);
    cmp(elements, ip);
    b(eq, &ok);
    RECORD_LINE();
    LoadRoot(ip, Heap::kFixedDoubleArrayMapRootIndex);
    cmp(elements, ip);
    b(eq, &ok);
    RECORD_LINE();
    LoadRoot(ip, Heap::kFixedCOWArrayMapRootIndex);
    cmp(elements, ip);
    b(eq, &ok);
    Abort(kJSObjectWithFastElementsMapHasSlowElements);
    bind(&ok);
    RECORD_LINE();
    pop(elements);
  }
}


void MacroAssembler::Check(Condition cond, BailoutReason reason) { // REVIEWEDBY: CG
  Label L;
  RECORD_LINE();
  ASSERT(cond == ne || cond == eq); // Limitation of sh4 b(cond,...)
  b(cond, &L);
  Abort(reason);
  // will not return here
  bind(&L);
}


void MacroAssembler::DebugPrint(Register obj) { // REVIEWEDBY: CG
  RECORD_LINE();
  push(obj);
  CallRuntime(Runtime::kDebugPrint, 1);
}


void MacroAssembler::Abort(BailoutReason reason) { // REVIEWEDBY: CG
  Label abort_start;
  bind(&abort_start);
#ifdef DEBUG
  const char* msg = GetBailoutReason(reason);
  if (msg != NULL) {
    RecordComment("Abort message: ");
    RecordComment(msg);
  }

  if (FLAG_trap_on_abort) {
    stop(msg);
    return;
  }
#endif
  RECORD_LINE();
  mov(r0, Operand(Smi::FromInt(reason)));
  push(r0);

  // Disable stub call restrictions to always allow calls to abort.
  if (!has_frame_) {
    // We don't actually want to generate a pile of code for this, so just
    // claim there is a stack frame, without generating one.
    FrameScope scope(this, StackFrame::NONE);
    CallRuntime(Runtime::kAbort, 1);
  } else {
    CallRuntime(Runtime::kAbort, 1);
  }
  // will not return here
  if (is_const_pool_blocked()) {
    // If the calling code cares about the exact number of
    // instructions generated, we insert padding here to keep the size
    // of the Abort macro constant.
    // SH4: use a larger count than on arm due to longer call sequences (31 here instead of 7)
    static const int kExpectedAbortInstructions = 31; // DIFF: codegen
    int abort_instructions = InstructionsGeneratedSince(&abort_start);
    ASSERT(abort_instructions <= kExpectedAbortInstructions);
    while (abort_instructions++ < kExpectedAbortInstructions) {
      nop();
    }
  }
}


void MacroAssembler::LoadContext(Register dst, int context_chain_length) { // REVIEWEDBY: CG
  ASSERT(!dst.is(sh4_rtmp));
  RECORD_LINE();
  if (context_chain_length > 0) {
    RECORD_LINE();
    // Move up the chain of contexts to the context containing the slot.
    ldr(dst, MemOperand(cp, Context::SlotOffset(Context::PREVIOUS_INDEX)));
    for (int i = 1; i < context_chain_length; i++) {
      RECORD_LINE();
      ldr(dst, MemOperand(dst, Context::SlotOffset(Context::PREVIOUS_INDEX)));
    }
  } else {
    RECORD_LINE();
    // Slot is in the current function context.  Move it into the
    // destination register in case we store into it (the write barrier
    // cannot be allowed to destroy the context in esi).
    mov(dst, cp);
  }
}


void MacroAssembler::LoadTransitionedArrayMapConditional( // REVIEWEDBY: CG
    ElementsKind expected_kind,
    ElementsKind transitioned_kind,
    Register map_in_out,
    Register scratch,
    Label* no_map_match) {
  // Load the global or builtins object from the current context.
  ldr(scratch,
      MemOperand(cp, Context::SlotOffset(Context::GLOBAL_OBJECT_INDEX)));
  ldr(scratch, FieldMemOperand(scratch, GlobalObject::kNativeContextOffset));

  // Check that the function's map is the same as the expected cached map.
  ldr(scratch,
      MemOperand(scratch,
                 Context::SlotOffset(Context::JS_ARRAY_MAPS_INDEX)));
  size_t offset = expected_kind * kPointerSize +
      FixedArrayBase::kHeaderSize;
  ldr(ip, FieldMemOperand(scratch, offset));
  cmp(map_in_out, ip);
  b(ne, no_map_match);

  // Use the transitioned cached map.
  offset = transitioned_kind * kPointerSize +
      FixedArrayBase::kHeaderSize;
  ldr(map_in_out, FieldMemOperand(scratch, offset));
}

void MacroAssembler::LoadGlobalFunction(int index, Register function) { // REVIEWEDBY: CG
  // Load the global or builtins object from the current context.
  ldr(function,
      MemOperand(cp, Context::SlotOffset(Context::GLOBAL_OBJECT_INDEX)));
  // Load the native context from the global or builtins object.
  ldr(function, FieldMemOperand(function,
                                GlobalObject::kNativeContextOffset));
  // Load the function from the native context.
  ldr(function, MemOperand(function, Context::SlotOffset(index)));
}

void MacroAssembler::LoadGlobalFunctionInitialMap(Register function, // REVIEWEDBY: CG
                                                  Register map,
                                                  Register scratch) {
  ASSERT(!scratch.is(sh4_ip));
  ASSERT(!scratch.is(sh4_rtmp));
  // Load the initial map. The global functions all have initial maps.
  ldr(map, FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));
  if (emit_debug_code()) {
    Label ok, fail;
    CheckMap(map, scratch, Heap::kMetaMapRootIndex, &fail, DO_SMI_CHECK);
    b(&ok);
    bind(&fail);
    Abort(kGlobalFunctionsMustHaveInitialMap);
    bind(&ok);
  }
}


void MacroAssembler::JumpIfNotPowerOfTwoOrZero( // REVIEWEDBY: CG
    Register reg,
    Register scratch,
    Label* not_power_of_two_or_zero) {
  ASSERT(!reg.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  sub(scratch, reg, Operand(1));
  cmpge(scratch, Operand(0));
  bf(not_power_of_two_or_zero);
  tst(scratch, reg);
  b(ne, not_power_of_two_or_zero);
}


void MacroAssembler::JumpIfNotPowerOfTwoOrZeroAndNeg( // REVIEWEDBY: CG
    Register reg,
    Register scratch,
    Label* zero_and_neg,
    Label* not_power_of_two) {
  ASSERT(!reg.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  RECORD_LINE();
  sub(scratch, reg, Operand(1));
  cmpge(scratch, Operand(0));
  bf(zero_and_neg);
  tst(scratch, reg);
  b(ne, not_power_of_two);
}


void MacroAssembler::JumpIfNotBothSmi(Register reg1, // REVIEWEDBY: CG
                                      Register reg2,
                                      Label* on_not_both_smi,
                                      Label::Distance distance) {
  ASSERT(!reg1.is(sh4_rtmp) && !reg2.is(sh4_rtmp));
  STATIC_ASSERT(kSmiTag == 0);
  RECORD_LINE();
  tst(reg1, Operand(kSmiTagMask));
  b(ne, on_not_both_smi, distance);
  tst(reg2, Operand(kSmiTagMask));
  b(ne, on_not_both_smi, distance);
}


void MacroAssembler::UntagAndJumpIfSmi( // REVIEWEDBY: CG
    Register dst, Register src, Label* smi_case) {
  STATIC_ASSERT(kSmiTag == 0);
  SmiUntag(dst, src, SetT);
  bt(smi_case);  // T bit is set for a smi
}


void MacroAssembler::UntagAndJumpIfNotSmi( // REVIEWEDBY: CG
    Register dst, Register src, Label* non_smi_case) {
  STATIC_ASSERT(kSmiTag == 0);
  SmiUntag(dst, src, SetT);
  bf(non_smi_case);  // T bit is not set if not a smi
}


void MacroAssembler::JumpIfEitherSmi(Register reg1, // REVIEWEDBY: CG
                                     Register reg2,
                                     Label* on_either_smi,
                                     Label::Distance distance) {
  ASSERT(!reg1.is(sh4_rtmp) && !reg2.is(sh4_rtmp));
  STATIC_ASSERT(kSmiTag == 0);
  RECORD_LINE();
  tst(reg1, Operand(kSmiTagMask));
  b(eq, on_either_smi, distance);
  tst(reg2, Operand(kSmiTagMask));
  b(eq, on_either_smi, distance);
}


void MacroAssembler::AssertNotSmi(Register object) { // REVIEWEDBY: CG
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    tst(object, Operand(kSmiTagMask));
    Check(ne, kOperandIsASmi);
  }
}


void MacroAssembler::AssertSmi(Register object) { // REVIEWEDBY: CG
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    tst(object, Operand(kSmiTagMask));
    Check(eq, kOperandIsNotSmi);
  }
}


void MacroAssembler::AssertString(Register object) { // REVIEWEDBY: CG
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    tst(object, Operand(kSmiTagMask));
    Check(ne, kOperandIsASmiAndNotAString);
    push(object);
    ldr(object, FieldMemOperand(object, HeapObject::kMapOffset));
    CompareInstanceType(object, object, FIRST_NONSTRING_TYPE, hs);
    pop(object);
    Check(f, kOperandIsNotAString);
  }
}


void MacroAssembler::AssertName(Register object) { // REVIEWEDBY: CG
  if (emit_debug_code()) {
    STATIC_ASSERT(kSmiTag == 0);
    tst(object, Operand(kSmiTagMask));
    Check(ne, kOperandIsASmiAndNotAName);
    push(object);
    ldr(object, FieldMemOperand(object, HeapObject::kMapOffset));
    CompareInstanceType(object, object, LAST_NAME_TYPE, gt);
    pop(object);
    Check(f, kOperandIsNotAName);
  }
}

void MacroAssembler::AssertUndefinedOrAllocationSite(Register object, // REVIEWEDBY: CG
                                                     Register scratch) {
  if (emit_debug_code()) {
    Label done_checking;
    AssertNotSmi(object);
    CompareRoot(object, Heap::kUndefinedValueRootIndex);
    b(eq, &done_checking);
    ldr(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
    CompareRoot(scratch, Heap::kAllocationSiteMapRootIndex);
    Assert(eq, kExpectedUndefinedOrCell);
    bind(&done_checking);
  }
}

void MacroAssembler::AssertIsRoot(Register reg, Heap::RootListIndex index) { // REVIEWEDBY: CG
  if (emit_debug_code()) {
    CompareRoot(reg, index);
    Check(eq, kHeapNumberMapRegisterClobbered);
  }
}


void MacroAssembler::JumpIfNotHeapNumber(Register object, // REVIEWEDBY: CG
                                         Register heap_number_map,
                                         Register scratch,
                                         Label* on_not_heap_number) {
  RECORD_LINE();
  ASSERT(!scratch.is(ip));
  ASSERT(!scratch.is(sh4_rtmp));
  ldr(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
  AssertIsRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);
  cmp(scratch, heap_number_map);
  b(ne, on_not_heap_number);
}


void MacroAssembler::LookupNumberStringCache(Register object, // REVIEWEDBY: CG
                                             Register result,
                                             Register scratch1,
                                             Register scratch2,
                                             Register scratch3,
                                             Label* not_found) {
  // Use of registers. Register result is used as a temporary.
  Register number_string_cache = result;
  Register mask = scratch3;

  // Load the number string cache.
  LoadRoot(number_string_cache, Heap::kNumberStringCacheRootIndex);

  // Make the hash mask from the length of the number string cache. It
  // contains two elements (number and string) for each cache entry.
  ldr(mask, FieldMemOperand(number_string_cache, FixedArray::kLengthOffset));
  // Divide length by two (length is a smi).
  asr(mask, mask, Operand(kSmiTagSize + 1)); // DIFF: codegen
  sub(mask, mask, Operand(1));  // Make mask.

  // Calculate the entry in the number string cache. The hash value in the
  // number string cache for smis is just the smi value, and the hash for
  // doubles is the xor of the upper and lower words. See
  // Heap::GetNumberStringCache.
  Label is_smi;
  Label load_result_from_cache;
  JumpIfSmi(object, &is_smi);
  CheckMap(object,
           scratch1,
           Heap::kHeapNumberMapRootIndex,
           not_found,
           DONT_DO_SMI_CHECK);

  STATIC_ASSERT(8 == kDoubleSize);
  add(scratch1,
      object,
      Operand(HeapNumber::kValueOffset - kHeapObjectTag));
  ldr(scratch2, MemOperand(scratch1, 4)); // DIFF: codegen
  ldr(scratch1, MemOperand(scratch1, 0)); // DIFF: codegen
  eor(scratch1, scratch1, Operand(scratch2));
  and_(scratch1, scratch1, Operand(mask));

  // Calculate address of entry in string cache: each entry consists
  // of two pointer sized fields.
  lsl(scratch1, scratch1, Operand(kPointerSizeLog2 + 1)); // DIFF: codegen
  add(scratch1,
      number_string_cache,
      scratch1); // DIFF: codegen

  Register probe = mask;
  ldr(probe, FieldMemOperand(scratch1, FixedArray::kHeaderSize));
  JumpIfSmi(probe, not_found);
  sub(scratch2, object, Operand(kHeapObjectTag));
  vldr(sh4_dr0, scratch2, HeapNumber::kValueOffset); // DIFF: codegen
  sub(probe, probe, Operand(kHeapObjectTag));
  vldr(sh4_dr2, probe, HeapNumber::kValueOffset); // DIFF: codegen
  dcmpeq(sh4_dr0, sh4_dr2); // DIFF: codegen
  b(ne, not_found);  // The cache did not contain this value.
  b(&load_result_from_cache);

  bind(&is_smi);
  Register scratch = scratch1;
  asr(scratch, object, Operand(1)); // DIFF: codegen
  and_(scratch, mask, scratch); // DIFF: codegen
  // Calculate address of entry in string cache: each entry consists
  // of two pointer sized fields.
  lsl(scratch, scratch, Operand(kPointerSizeLog2 + 1)); // DIFF: codegen
  add(scratch,
      number_string_cache,
      scratch); // DIFF: codegen

  // Check if the entry is the smi we are looking for.
  ldr(probe, FieldMemOperand(scratch, FixedArray::kHeaderSize));
  cmp(object, probe);
  b(ne, not_found);

  // Get the result from the cache.
  bind(&load_result_from_cache);
  ldr(result, FieldMemOperand(scratch, FixedArray::kHeaderSize + kPointerSize));
  IncrementCounter(isolate()->counters()->number_to_string_native(),
                   1,
                   scratch1,
                   scratch2);
}


void MacroAssembler::JumpIfNonSmisNotBothSequentialAsciiStrings( // REVIEWEDBY: CG
    Register first,
    Register second,
    Register scratch1,
    Register scratch2,
    Label* failure) {

  ASSERT(!first.is(sh4_ip) && !second.is(sh4_ip) && !scratch1.is(sh4_ip) &&
         !scratch2.is(sh4_ip));
  ASSERT(!first.is(sh4_rtmp) && !second.is(sh4_rtmp) && !scratch1.is(sh4_rtmp) &&
         !scratch2.is(sh4_rtmp));
  RECORD_LINE();
  // Test that both first and second are sequential ASCII strings.
  // Assume that they are non-smis.
  ldr(scratch1, FieldMemOperand(first, HeapObject::kMapOffset));
  ldr(scratch2, FieldMemOperand(second, HeapObject::kMapOffset));
  ldrb(scratch1, FieldMemOperand(scratch1, Map::kInstanceTypeOffset));
  ldrb(scratch2, FieldMemOperand(scratch2, Map::kInstanceTypeOffset));

  JumpIfBothInstanceTypesAreNotSequentialAscii(scratch1,
                                               scratch2,
                                               scratch1,
                                               scratch2,
                                               failure);
}


void MacroAssembler::JumpIfNotBothSequentialAsciiStrings(Register first, // REVIEWEDBY: CG
                                                         Register second,
                                                         Register scratch1,
                                                         Register scratch2,
                                                         Label* failure) {
  ASSERT(!first.is(sh4_ip) && !second.is(sh4_ip) && !scratch1.is(sh4_ip) &&
         !scratch2.is(sh4_ip));
  ASSERT(!first.is(sh4_rtmp) && !second.is(sh4_rtmp) && !scratch1.is(sh4_rtmp) &&
         !scratch2.is(sh4_rtmp));
  RECORD_LINE();
  // Check that neither is a smi.
  and_(scratch1, first, second);
  JumpIfSmi(scratch1, failure);
  JumpIfNonSmisNotBothSequentialAsciiStrings(first,
                                             second,
                                             scratch1,
                                             scratch2,
                                             failure);
}


void MacroAssembler::JumpIfNotUniqueName(Register reg, // REVIEWEDBY: CG
                                         Label* not_unique_name) {
  STATIC_ASSERT(kInternalizedTag == 0 && kStringTag == 0);
  Label succeed;
  tst(reg, Operand(kIsNotStringMask | kIsNotInternalizedMask));
  b(eq, &succeed);
  cmp(reg, Operand(SYMBOL_TYPE));
  b(ne, not_unique_name);

  bind(&succeed);
}


// Allocates a heap number or jumps to the need_gc label if the young space
// is full and a scavenge is needed.
void MacroAssembler::AllocateHeapNumber(Register result, // REVIEWEDBY: CG
                                        Register scratch1,
                                        Register scratch2,
                                        Register heap_number_map,
                                        Label* gc_required,
                                        TaggingMode tagging_mode) {
  // Allocate an object in the heap for the heap number and tag it as a heap
  // object.
  RECORD_LINE();
  Allocate(HeapNumber::kSize, result, scratch1, scratch2, gc_required,
           tagging_mode == TAG_RESULT ? TAG_OBJECT : NO_ALLOCATION_FLAGS);

  // Store heap number map in the allocated object.
  RECORD_LINE();
  AssertIsRoot(heap_number_map, Heap::kHeapNumberMapRootIndex);
  if (tagging_mode == TAG_RESULT) {
    RECORD_LINE();
    str(heap_number_map, FieldMemOperand(result, HeapObject::kMapOffset));
  } else {
    str(heap_number_map, MemOperand(result, HeapObject::kMapOffset));
  }
}


void MacroAssembler::AllocateHeapNumberWithValue(Register result, // SH4: TODO
                                                 DwVfpRegister value,
                                                 Register scratch1,
                                                 Register scratch2,
                                                 Register heap_number_map,
                                                 Label* gc_required) {
  UNIMPLEMENTED();
}


// Copies a fixed number of fields of heap objects from src to dst.
void MacroAssembler::CopyFields(Register dst, // REVIEWEDBY: CG
                                Register src,
                                DwVfpRegister double_scratch,
                                int field_count) {
  int double_count = field_count / (DwVfpRegister::kSizeInBytes / kPointerSize);
  for (int i = 0; i < double_count; i++) {
    vldr(double_scratch, FieldMemOperand(src, i * DwVfpRegister::kSizeInBytes));
    vstr(double_scratch, FieldMemOperand(dst, i * DwVfpRegister::kSizeInBytes));
  }

  STATIC_ASSERT(SwVfpRegister::kSizeInBytes == kPointerSize);
  STATIC_ASSERT(2 * SwVfpRegister::kSizeInBytes == DwVfpRegister::kSizeInBytes);

  int remain = field_count % (DwVfpRegister::kSizeInBytes / kPointerSize);
  if (remain != 0) {
    vldr(double_scratch.low(),
         FieldMemOperand(src, (field_count - 1) * kPointerSize));
    vstr(double_scratch.low(),
         FieldMemOperand(dst, (field_count - 1) * kPointerSize));
  }
}


void MacroAssembler::CopyBytes(Register src, // REVIEWEDBY: CG
                               Register dst,
                               Register length,
                               Register scratch) {
  Label align_loop, align_loop_1, word_loop, byte_loop, byte_loop_1, done;

  // Align src before copying in word size chunks.
  cmpgt(length, Operand(kPointerSize)); // DIFF: codegen
  bf_near(&byte_loop); // DIFF: codegen

  bind(&align_loop_1);
  tst(src, Operand(kPointerSize - 1));
  bt_near(&word_loop); // DIFF: codegen
  ldrb(scratch, MemOperand(src, 1, PostIndex));
  strb(scratch, MemOperand(dst, 1, PostIndex));
  dt(length); // DIFF: codegen
  b_near(&align_loop_1);
  // Copy bytes in word size chunks.
  bind(&word_loop);
  if (emit_debug_code()) {
    tst(src, Operand(kPointerSize - 1));
    Assert(eq, kExpectingAlignmentForCopyBytes);
  }
  cmpge(length, Operand(kPointerSize));
  bf_near(&byte_loop);
  ldr(scratch, MemOperand(src, kPointerSize, PostIndex));
  if (CpuFeatures::IsSupported(UNALIGNED_ACCESSES)) {
    UNREACHABLE(); // SH4: no UNALIGNED_ACCESS // DIFF: codegen
  } else {
    strb(scratch, MemOperand(dst, 1, PostIndex));
    lsr(scratch, scratch, Operand(8)); // DIFF: codegen
    strb(scratch, MemOperand(dst, 1, PostIndex));
    lsr(scratch, scratch, Operand(8)); // DIFF: codegen
    strb(scratch, MemOperand(dst, 1, PostIndex));
    lsr(scratch, scratch, Operand(8)); // DIFF: codegen
    strb(scratch, MemOperand(dst, 1, PostIndex));
  }
  sub(length, length, Operand(kPointerSize));
  b_near(&word_loop);

  // Copy the last bytes if any left.
  bind(&byte_loop);
  cmpeq(length, Operand(0)); // DIFF: codegen
  bt_near(&done); // DIFF: codegen
  bind(&byte_loop_1);
  ldrb(scratch, MemOperand(src, 1, PostIndex));
  strb(scratch, MemOperand(dst, 1, PostIndex));
  dt(length); // DIFF: codegen
  bf_near(&byte_loop_1); // DIFF: codegen
  bind(&done);
}


void MacroAssembler::InitializeFieldsWithFiller(Register start_offset, // REVIEWEDBY: CG
                                                Register end_offset,
                                                Register filler) {
  Label loop, entry;
  b(&entry);
  bind(&loop);
  str(filler, MemOperand(start_offset, kPointerSize, PostIndex));
  bind(&entry);
  cmpge(start_offset, end_offset);
  bf(&loop);
}


void MacroAssembler::SaveFPRegs(Register location, Register scratch) { // REVIEWEDBY: CG
  // SH4: scratch is not used
  sub(location, location, Operand(DwVfpRegister::kMaxNumRegisters * kDoubleSize));
  for (int i = 0; i < DwVfpRegister::kMaxNumRegisters; i++) {
    vstr(DwVfpRegister::from_code(i * 2), MemOperand(location, i * kDoubleSize));
  }
}


void MacroAssembler::RestoreFPRegs(Register location, Register scratch) { // REVIEWEDBY: CG
  // SH4: scratch is not used
  for (int i = DwVfpRegister::kMaxNumRegisters - 1; i >= 0; i--) {
    vldr(DwVfpRegister::from_code(i * 2), MemOperand(location, i * kDoubleSize));
  }
  add(location, location, Operand(DwVfpRegister::kMaxNumRegisters * kDoubleSize));
}


void MacroAssembler::JumpIfBothInstanceTypesAreNotSequentialAscii( // REVIEWEDBY: CG
    Register first,
    Register second,
    Register scratch1,
    Register scratch2,
    Label* failure) {
  ASSERT(!first.is(sh4_ip) && !second.is(sh4_ip) && !scratch1.is(sh4_ip) &&
         !scratch2.is(sh4_ip));
  ASSERT(!first.is(sh4_rtmp) && !second.is(sh4_rtmp) && !scratch1.is(sh4_rtmp) &&
         !scratch2.is(sh4_rtmp));

  const int kFlatAsciiStringMask =
      kIsNotStringMask | kStringEncodingMask | kStringRepresentationMask;
  const int kFlatAsciiStringTag =
      kStringTag | kOneByteStringTag | kSeqStringTag;
  and_(scratch1, first, Operand(kFlatAsciiStringMask));
  and_(scratch2, second, Operand(kFlatAsciiStringMask));
  cmp(scratch1, Operand(kFlatAsciiStringTag));
  b(ne, failure); // DIFF: codegen
  cmp(scratch2, Operand(kFlatAsciiStringTag)); // DIFF: codegen
  b(ne, failure);
}


void MacroAssembler::JumpIfInstanceTypeIsNotSequentialAscii(Register type, // REVIEWEDBY: CG
                                                            Register scratch,
                                                            Label* failure) {
  ASSERT(!type.is(sh4_rtmp) && !scratch.is(sh4_rtmp));
  const int kFlatAsciiStringMask =
      kIsNotStringMask | kStringEncodingMask | kStringRepresentationMask;
  const int kFlatAsciiStringTag =
      kStringTag | kOneByteStringTag | kSeqStringTag;
  and_(scratch, type, Operand(kFlatAsciiStringMask));
  cmp(scratch, Operand(kFlatAsciiStringTag));
  b(ne, failure);
}

static const int kRegisterPassedArguments = 4;


int MacroAssembler::CalculateStackPassedWords(int num_reg_arguments, // REVIEWEDBY: CG
                                              int num_double_arguments) {
  int stack_passed_words = 0;
  // Up to 4 simple arguments are passed in [r4, r7]
  if (num_reg_arguments > kRegisterPassedArguments) {
    stack_passed_words += num_reg_arguments - kRegisterPassedArguments;
  }

  // Up to 4 double arguments are passed in [dr4, dr10]
  if (num_double_arguments > kRegisterPassedArguments) {
    stack_passed_words += 2 * (num_double_arguments - kRegisterPassedArguments);
  }

  return stack_passed_words;
}

void MacroAssembler::EmitSeqStringSetCharCheck(Register string, // REVIEWEDBY: CG
                                               Register index,
                                               Register value,
                                               uint32_t encoding_mask) {
  Label is_object;
  SmiTst(string);
  Check(ne, kNonObject);

  ldr(ip, FieldMemOperand(string, HeapObject::kMapOffset));
  ldrb(ip, FieldMemOperand(ip, Map::kInstanceTypeOffset));

  and_(ip, ip, Operand(kStringRepresentationMask | kStringEncodingMask));
  cmp(ip, Operand(encoding_mask));
  Check(eq, kUnexpectedStringType);

  // The index is assumed to be untagged coming in, tag it to compare with the
  // string length without using a temp register, it is restored at the end of
  // this function.
  Label index_tag_ok, index_tag_bad;
  TrySmiTag(index, index, &index_tag_bad);
  b(&index_tag_ok);
  bind(&index_tag_bad);
  Abort(kIndexIsTooLarge);
  bind(&index_tag_ok);

  ldr(ip, FieldMemOperand(string, String::kLengthOffset));
  cmpge(index, ip); // DIFF: codegen
  Check(ne, kIndexIsTooLarge); // DIFF: codegen

  cmpge(index, Operand(Smi::FromInt(0))); // DIFF: codegen
  Check(eq, kIndexIsNegative); // DIFF: codegen

  SmiUntag(index, index);
}

void MacroAssembler::PrepareCallCFunction(int num_reg_arguments, // REVIEWEDBY: CG
                                          int num_double_arguments,
                                          Register scratch) {
  ASSERT(!scratch.is(sh4_ip));
  ASSERT(!scratch.is(sh4_rtmp));
  int frame_alignment = ActivationFrameAlignment();
  int stack_passed_arguments = CalculateStackPassedWords(
      num_reg_arguments, num_double_arguments);
  if (frame_alignment > kPointerSize) {
    RECORD_LINE();
    // Make stack end at alignment and make room for num_arguments - 4 words
    // and the original value of sp.
    mov(scratch, sp);
    sub(sp, sp, Operand((stack_passed_arguments + 1) * kPointerSize));
    ASSERT(IsPowerOf2(frame_alignment));
    and_(sp, sp, Operand(-frame_alignment));
    str(scratch, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    RECORD_LINE();
    sub(sp, sp, Operand(stack_passed_arguments * kPointerSize));
  }
}


void MacroAssembler::PrepareCallCFunction(int num_reg_arguments, // REVIEWEDBY: CG
                                          Register scratch) {
  PrepareCallCFunction(num_reg_arguments, 0, scratch);
}


void MacroAssembler::MovToFloatParameter(DwVfpRegister src) { // REVIEWEDBY: CG
  // SH4: first parameter is passed in dr4
  vmov(sh4_dr4, src);
}

void MacroAssembler::MovToFloatParameters(DwVfpRegister src1, // REVIEWEDBY: CG
                                          DwVfpRegister src2) {
  // SH4: first two parameters are passed in dr4, dr6
  // Do not treat the case of a swap (dr6,dr4) -> (dr4,dr6)
  // Then order the move such that no register is clobbered
  ASSERT(!(src1.is(sh4_dr6) && src2.is(sh4_dr4)));
  if (!src2.is(sh4_dr4)) {
    vmov(sh4_dr4, src1);
    vmov(sh4_dr6, src2);
  } else {
    vmov(sh4_dr6, src2);
    vmov(sh4_dr4, src1);
  }
}

void MacroAssembler::CallCFunction(ExternalReference function, // REVIEWEDBY: CG
                                   int num_reg_arguments,
                                   int num_double_arguments) {
  RECORD_LINE();
  mov(ip, Operand(function));
  CallCFunctionHelper(ip, num_reg_arguments, num_double_arguments);
}


void MacroAssembler::CallCFunction(Register function, // REVIEWEDBY: CG
                                   int num_reg_arguments,
                                   int num_double_arguments) {
  CallCFunctionHelper(function, num_reg_arguments, num_double_arguments);
}


void MacroAssembler::CallCFunction(ExternalReference function, // REVIEWEDBY: CG
                                   int num_arguments) {
  CallCFunction(function, num_arguments, 0);
}


void MacroAssembler::CallCFunction(Register function, // REVIEWEDBY: CG
                                   int num_arguments) {
  CallCFunction(function, num_arguments, 0);
}


void MacroAssembler::CallCFunctionHelper(Register function, // REVIEWEDBY: CG
                                         int num_reg_arguments,
                                         int num_double_arguments) {
  ASSERT(has_frame());
  ASSERT(function.is(ip));
  // Make sure that the stack is aligned before calling a C function unless
  // running in the simulator. The simulator has its own alignment check which
  // provides more information.
#if V8_HOST_ARCH_SH4
  if (emit_debug_code()) {
    int frame_alignment = OS::ActivationFrameAlignment();
    int frame_alignment_mask = frame_alignment - 1;
    if (frame_alignment > kPointerSize) {
      ASSERT(IsPowerOf2(frame_alignment));
      Label alignment_as_expected;
      tst(sp, Operand(frame_alignment_mask));
      b(eq, &alignment_as_expected);
      // Don't use Check here, as it will call Runtime_Abort possibly
      // re-entering here.
      stop("Unexpected alignment");
      bind(&alignment_as_expected);
    }
  }
#endif

  // Just call directly. The function called cannot cause a GC, or
  // allow preemption, so the return address in the link register
  // stays correct.
  Call(function);
  int stack_passed_arguments = CalculateStackPassedWords(
      num_reg_arguments, num_double_arguments);
  if (ActivationFrameAlignment() > kPointerSize) {
    ldr(sp, MemOperand(sp, stack_passed_arguments * kPointerSize));
  } else {
    add(sp, sp, Operand(stack_passed_arguments * sizeof(kPointerSize)));
  }
}


void MacroAssembler::GetRelocatedValueLocation(Register ldr_location, // SH4: TODO
                               Register result) {
  UNIMPLEMENTED();
}


void MacroAssembler::CheckPageFlag( // REVIEWEDBY: CG
    Register object,
    Register scratch,
    int mask,
    Condition cc,
    Label* condition_met) {
  ASSERT(cc == eq || cc == ne);
  Bfc(scratch, object, 0, kPageSizeBits);
  ldr(scratch, MemOperand(scratch, MemoryChunk::kFlagsOffset));
  tst(scratch, Operand(mask));
  b(cc,condition_met);
}


void MacroAssembler::CheckMapDeprecated(Handle<Map> map, // REVIEWEDBY: CG
                                        Register scratch,
                                        Label* if_deprecated) {
  if (map->CanBeDeprecated()) {
    mov(scratch, Operand(map));
    ldr(scratch, FieldMemOperand(scratch, Map::kBitField3Offset));
    tst(scratch, Operand(Smi::FromInt(Map::Deprecated::kMask)));
    b(ne, if_deprecated);
  }
}


void MacroAssembler::JumpIfBlack(Register object, // REVIEWEDBY: CG
                                 Register scratch0,
                                 Register scratch1,
                                 Label* on_black) {
  HasColor(object, scratch0, scratch1, on_black, 1, 0);  // kBlackBitPattern.
  ASSERT(strcmp(Marking::kBlackBitPattern, "10") == 0);
}


void MacroAssembler::HasColor(Register object, // REVIEWEDBY: CG
                              Register bitmap_scratch,
                              Register mask_scratch,
                              Label* has_color,
                              int first_bit,
                              int second_bit) {
  ASSERT(!AreAliased(object, bitmap_scratch, mask_scratch, no_reg));

  GetMarkBits(object, bitmap_scratch, mask_scratch);

  Label other_color, word_boundary;
  ldr(ip, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize));
  tst(ip, mask_scratch);
  b(first_bit == 1 ? eq : ne, &other_color);
  // Shift left 1 by adding.
  add(mask_scratch, mask_scratch, Operand(mask_scratch)); // DIFF: codegen
  cmpeq(mask_scratch, Operand::Zero()); // DIFF: codegen
  b(eq, &word_boundary);
  tst(ip, mask_scratch);
  b(second_bit == 1 ? ne : eq, has_color);
  jmp(&other_color);

  bind(&word_boundary);
  ldr(ip, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize + kPointerSize));
  tst(ip, Operand(1));
  b(second_bit == 1 ? ne : eq, has_color);
  bind(&other_color);
}


// Detect some, but not all, common pointer-free objects.  This is used by the
// incremental write barrier which doesn't care about oddballs (they are always
// marked black immediately so this code is not hit).
void MacroAssembler::JumpIfDataObject(Register value, // REVIEWEDBY: CG
                                      Register scratch,
                                      Label* not_data_object) {
  Label is_data_object;
  ldr(scratch, FieldMemOperand(value, HeapObject::kMapOffset));
  CompareRoot(scratch, Heap::kHeapNumberMapRootIndex);
  b(eq, &is_data_object);
  ASSERT(kIsIndirectStringTag == 1 && kIsIndirectStringMask == 1);
  ASSERT(kNotStringTag == 0x80 && kIsNotStringMask == 0x80);
  // If it's a string and it's not a cons string then it's an object containing
  // no GC pointers.
  ldrb(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));
  tst(scratch, Operand(kIsIndirectStringMask | kIsNotStringMask));
  b(ne, not_data_object);
  bind(&is_data_object);
}


void MacroAssembler::GetMarkBits(Register addr_reg, // REVIEWEDBY: CG
                                 Register bitmap_reg,
                                 Register mask_reg) {
  ASSERT(!AreAliased(addr_reg, bitmap_reg, mask_reg, no_reg));
  and_(bitmap_reg, addr_reg, Operand(~Page::kPageAlignmentMask));
  Ubfx(mask_reg, addr_reg, kPointerSizeLog2, Bitmap::kBitsPerCellLog2);
  const int kLowBits = kPointerSizeLog2 + Bitmap::kBitsPerCellLog2;
  Ubfx(ip, addr_reg, kLowBits, kPageSizeBits - kLowBits);
  lsl(ip, ip, Operand(kPointerSizeLog2)); // DIFF: codegen
  add(bitmap_reg, bitmap_reg, ip); // DIFF: codegen
  mov(ip, Operand(1));
  lsl(mask_reg, ip, mask_reg); // DIFF: codegen
}


void MacroAssembler::EnsureNotWhite( // REVIEWEDBY: CG
    Register value,
    Register bitmap_scratch,
    Register mask_scratch,
    Register load_scratch,
    Label* value_is_white_and_not_data) {
  ASSERT(!AreAliased(value, bitmap_scratch, mask_scratch, ip));
  GetMarkBits(value, bitmap_scratch, mask_scratch);

  // If the value is black or grey we don't need to do anything.
  ASSERT(strcmp(Marking::kWhiteBitPattern, "00") == 0);
  ASSERT(strcmp(Marking::kBlackBitPattern, "10") == 0);
  ASSERT(strcmp(Marking::kGreyBitPattern, "11") == 0);
  ASSERT(strcmp(Marking::kImpossibleBitPattern, "01") == 0);

  Label done;

  // Since both black and grey have a 1 in the first position and white does
  // not have a 1 there we only need to check one bit.
  ldr(load_scratch, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize));
  tst(mask_scratch, load_scratch);
  b(ne, &done);

  if (emit_debug_code()) {
    // Check for impossible bit pattern.
    Label ok;
    // LSL may overflow, making the check conservative.
    lsl(ip, mask_scratch, Operand(1)); // DIFF: codegen
    tst(load_scratch, ip); // DIFF: codegen
    b(eq, &ok);
    stop("Impossible marking bit pattern");
    bind(&ok);
  }

  // Value is white.  We check whether it is data that doesn't need scanning.
  // Currently only checks for HeapNumber and non-cons strings.
  Register map = load_scratch;  // Holds map while checking type.
  Register length = load_scratch;  // Holds length of object after testing type.
  Label is_data_object;

  // Check for heap-number
  ldr(map, FieldMemOperand(value, HeapObject::kMapOffset));
  CompareRoot(map, Heap::kHeapNumberMapRootIndex);
  mov(length, Operand(HeapNumber::kSize), eq); // DIFF: codegen
  b(eq, &is_data_object);

  // Check for strings.
  ASSERT(kIsIndirectStringTag == 1 && kIsIndirectStringMask == 1);
  ASSERT(kNotStringTag == 0x80 && kIsNotStringMask == 0x80);
  // If it's a string and it's not a cons string then it's an object containing
  // no GC pointers.
  Register instance_type = load_scratch;
  ldrb(instance_type, FieldMemOperand(map, Map::kInstanceTypeOffset));
  tst(instance_type, Operand(kIsIndirectStringMask | kIsNotStringMask));
  b(ne, value_is_white_and_not_data);
  // It's a non-indirect (non-cons and non-slice) string.
  // If it's external, the length is just ExternalString::kSize.
  // Otherwise it's String::kHeaderSize + string->length() * (1 or 2).
  // External strings are the only ones with the kExternalStringTag bit
  // set.
  ASSERT_EQ(0, kSeqStringTag & kExternalStringTag);
  ASSERT_EQ(0, kConsStringTag & kExternalStringTag);
  tst(instance_type, Operand(kExternalStringTag));
  mov(length, Operand(ExternalString::kSize), ne); // DIFF: codegen
  b(ne, &is_data_object);

  // Sequential string, either ASCII or UC16.
  // For ASCII (char-size of 1) we shift the smi tag away to get the length.
  // For UC16 (char-size of 2) we just leave the smi tag in place, thereby
  // getting the length multiplied by 2.
  ASSERT(kOneByteStringTag == 4 && kStringEncodingMask == 4);
  ASSERT(kSmiTag == 0 && kSmiTagSize == 1);
  ldr(ip, FieldMemOperand(value, String::kLengthOffset));
  tst(instance_type, Operand(kStringEncodingMask));
  Label skip1;
  bt_near(&skip1); // DIFF: codegen
  lsr(ip, ip, Operand(1)); // DIFF: codegen
  bind(&skip1);
  add(length, ip, Operand(SeqString::kHeaderSize + kObjectAlignmentMask));
  and_(length, length, Operand(~kObjectAlignmentMask));

  bind(&is_data_object);
  // Value is a data object, and it is white.  Mark it black.  Since we know
  // that the object is white we can make it black by flipping one bit.
  ldr(ip, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize));
  orr(ip, ip, Operand(mask_scratch));
  str(ip, MemOperand(bitmap_scratch, MemoryChunk::kHeaderSize));

  and_(bitmap_scratch, bitmap_scratch, Operand(~Page::kPageAlignmentMask));
  ldr(ip, MemOperand(bitmap_scratch, MemoryChunk::kLiveBytesOffset));
  add(ip, ip, Operand(length));
  str(ip, MemOperand(bitmap_scratch, MemoryChunk::kLiveBytesOffset));

  bind(&done);
}


void MacroAssembler::CountLeadingZeros(Register zeros,   // Answer. // REVIEWEDBY: CG
                                       Register source,  // Input.
                                       Register scratch) {
  ASSERT(!zeros.is(source) || !source.is(scratch));
  ASSERT(!zeros.is(scratch));
  ASSERT(!scratch.is(sh4_rtmp));
  ASSERT(!source.is(sh4_rtmp));
  ASSERT(!zeros.is(sh4_rtmp));
  ASSERT(!scratch.is(sh4_ip));
  ASSERT(!source.is(sh4_ip));
  ASSERT(!zeros.is(sh4_ip));
  RECORD_LINE();

  Label l0, l1, l2, l3, l4, l5;
  cmpeq(source, Operand(0));
  bf_near(&l0);
  mov(zeros, Operand(32));
  jmp_near(&l5);

  bind(&l0);
  // Be carefull to save source in scratch, source and zeros may be the same register
  mov(scratch, source);
  mov(zeros, Operand(0));
  // Top 16.
  tst(scratch, Operand(0xffff0000));
  bf_near(&l1);
  add(zeros, zeros, Operand(16));
  lsl(scratch, scratch, Operand(16));
  // Top 8.
  bind(&l1);
  tst(scratch, Operand(0xff000000));
  bf_near(&l2);
  add(zeros, zeros, Operand(8));
  lsl(scratch, scratch, Operand(8));
  // Top 4.
  bind(&l2);
  tst(scratch, Operand(0xf0000000));
  bf_near(&l3);
  add(zeros, zeros, Operand(4));
  lsl(scratch, scratch, Operand(4));
  // Top 2.
  bind(&l3);
  tst(scratch, Operand(0xc0000000));
  bf_near(&l4);
  add(zeros, zeros, Operand(2));
  lsl(scratch, scratch, Operand(2));
  // Top bit.
  bind(&l4);
  tst(scratch, Operand(0x80000000u));
  bf_near(&l5);
  add(zeros, zeros, Operand(1));
  bind(&l5);
}


void MacroAssembler::LoadInstanceDescriptors(Register map, // REVIEWEDBY: CG
                                             Register descriptors) {
  ldr(descriptors, FieldMemOperand(map, Map::kDescriptorsOffset));
}


void MacroAssembler::NumberOfOwnDescriptors(Register dst, Register map) { // REVIEWEDBY: CG
  ldr(dst, FieldMemOperand(map, Map::kBitField3Offset));
  DecodeField<Map::NumberOfOwnDescriptorsBits>(dst);
}


void MacroAssembler::EnumLength(Register dst, Register map) { // REVIEWEDBY: CG
  STATIC_ASSERT(Map::EnumLengthBits::kShift == 0);
  ldr(dst, FieldMemOperand(map, Map::kBitField3Offset));
  and_(dst, dst, Operand(Smi::FromInt(Map::EnumLengthBits::kMask)));
}


void MacroAssembler::CheckEnumCache(Register null_value, Label* call_runtime) { // REVIEWEDBY: CG
  Register  empty_fixed_array_value = r6;
  LoadRoot(empty_fixed_array_value, Heap::kEmptyFixedArrayRootIndex);
  Label next, start;
  mov(r2, r0);

  // Check if the enum length field is properly initialized, indicating that
  // there is an enum cache.
  ldr(r1, FieldMemOperand(r2, HeapObject::kMapOffset));

  EnumLength(r3, r1);
  cmp(r3, Operand(Smi::FromInt(kInvalidEnumCacheSentinel)));
  b(eq, call_runtime);

  jmp(&start);

  bind(&next);
  ldr(r1, FieldMemOperand(r2, HeapObject::kMapOffset));

  // For all objects but the receiver, check that the cache is empty.
  EnumLength(r3, r1);
  cmp(r3, Operand(Smi::FromInt(0)));
  b(ne, call_runtime);

  bind(&start);

  // Check that there are no elements. Register r2 contains the current JS
  // object we've reached through the prototype chain.
  Label no_elements;
  ldr(r2, FieldMemOperand(r2, JSObject::kElementsOffset));
  cmp(r2, empty_fixed_array_value);
  b(eq, &no_elements);

  // Second chance, the object may be using the empty slow element dictionary.
  CompareRoot(r2, Heap::kEmptySlowElementDictionaryRootIndex);
  b(ne, call_runtime);

  bind(&no_elements);
  ldr(r2, FieldMemOperand(r1, Map::kPrototypeOffset));
  cmp(r2, null_value);
  b(ne, &next);
}


void MacroAssembler::TestJSArrayForAllocationMemento( // REVIEWEDBY: CG
    Register receiver_reg,
    Register scratch_reg,
    Label* no_memento_found,
    Condition cond) {
  ExternalReference new_space_start =
      ExternalReference::new_space_start(isolate());
  ExternalReference new_space_allocation_top =
      ExternalReference::new_space_allocation_top_address(isolate());
  ASSERT_EQ(cond, eq); // SH4: Must be used only for test against eq as per interface
  add(scratch_reg, receiver_reg,
      Operand(JSArray::kSize + AllocationMemento::kSize - kHeapObjectTag));
  cmpge(scratch_reg, Operand(new_space_start)); // DIFF: codegen
  b(f, no_memento_found); // DIFF: codegen
  mov(ip, Operand(new_space_allocation_top));
  ldr(ip, MemOperand(ip));
  cmpgt(scratch_reg, ip); // DIFF: codegen
  b(t, no_memento_found); // DIFF: codegen
  ldr(scratch_reg, MemOperand(scratch_reg, -AllocationMemento::kSize));
  cmpeq(scratch_reg, // SH4: test for eq only // DIFF: codegen
        Operand(isolate()->factory()->allocation_memento_map()));
}


Register GetRegisterThatIsNotOneOf(Register reg1, // REVIEWEDBY: CG
                                   Register reg2,
                                   Register reg3,
                                   Register reg4,
                                   Register reg5,
                                   Register reg6) {
  RegList regs = 0;
  if (reg1.is_valid()) regs |= reg1.bit();
  if (reg2.is_valid()) regs |= reg2.bit();
  if (reg3.is_valid()) regs |= reg3.bit();
  if (reg4.is_valid()) regs |= reg4.bit();
  if (reg5.is_valid()) regs |= reg5.bit();
  if (reg6.is_valid()) regs |= reg6.bit();

  for (int i = 0; i < Register::NumAllocatableRegisters(); i++) {
    Register candidate = Register::FromAllocationIndex(i);
    if (regs & candidate.bit()) continue;
    return candidate;
  }
  UNREACHABLE();
  return no_reg;
}

void MacroAssembler::JumpIfDictionaryInPrototypeChain( // REVIEWEDBY: CG
    Register object,
    Register scratch0,
    Register scratch1,
    Label* found) {
  ASSERT(!scratch1.is(scratch0));
  Factory* factory = isolate()->factory();
  Register current = scratch0;
  Label loop_again;

  // scratch contained elements pointer.
  mov(current, object);

  // Loop based on the map going up the prototype chain.
  bind(&loop_again);
  ldr(current, FieldMemOperand(current, HeapObject::kMapOffset));
  // SH4: we can't generate unaligned accesses, use ldrb
  // see code-stubs-sh4.cc InternalArrayConstructorStub::Generate
  ldrb(scratch1, FieldMemOperand(current, Map::kBitField2Offset)); // DIFF: codegen
  Ubfx(scratch1, scratch1, Map::kElementsKindShift, Map::kElementsKindBitCount);
  cmp(scratch1, Operand(DICTIONARY_ELEMENTS));
  b(eq, found);
  ldr(current, FieldMemOperand(current, Map::kPrototypeOffset));
  cmp(current, Operand(factory->null_value()));
  b(ne, &loop_again);
}

#ifdef DEBUG
bool AreAliased(Register reg1, // REVIEWEDBY: CG
                Register reg2,
                Register reg3,
                Register reg4,
                Register reg5,
                Register reg6) {
  int n_of_valid_regs = reg1.is_valid() + reg2.is_valid() +
    reg3.is_valid() + reg4.is_valid() + reg5.is_valid() + reg6.is_valid();

  RegList regs = 0;
  if (reg1.is_valid()) regs |= reg1.bit();
  if (reg2.is_valid()) regs |= reg2.bit();
  if (reg3.is_valid()) regs |= reg3.bit();
  if (reg4.is_valid()) regs |= reg4.bit();
  if (reg5.is_valid()) regs |= reg5.bit();
  if (reg6.is_valid()) regs |= reg6.bit();
  int n_of_non_aliasing_regs = NumRegs(regs);

  return n_of_valid_regs != n_of_non_aliasing_regs;
}
#endif


CodePatcher::CodePatcher(byte* address, // REVIEWEDBY: CG
                         int instructions,
                         FlushICache flush_cache)
    : address_(address),
      size_(instructions * Assembler::kInstrSize),
      masm_(NULL, address, size_ + Assembler::kGap),
      flush_cache_(flush_cache) {
  // Create a new macro assembler pointing to the address of the code to patch.
  // The size is adjusted with kGap on order for the assembler to generate size
  // bytes of instructions without failing with buffer size constraints.
  ASSERT(masm_.reloc_info_writer.pos() == address_ + size_ + Assembler::kGap);
}


CodePatcher::~CodePatcher() { // REVIEWEDBY: CG
  // Indicate that code has changed.
  if (flush_cache_ == FLUSH) {
    CPU::FlushICache(address_, size_);
  }

  // Check that the code was patched as expected.
  ASSERT(masm_.pc_ == address_ + size_);
  ASSERT(masm_.reloc_info_writer.pos() == address_ + size_ + Assembler::kGap);
}


void CodePatcher::Emit(Instr instr) { // REVIEWEDBY: CG
  masm()->emit(instr);
}


void CodePatcher::Emit(Address addr) { // REVIEWEDBY: CG
  // SH4: must have been aligned before call
  ASSERT(((uintptr_t)address_ + masm()->pc_offset()) % 4 == 0);
  masm()->dd(reinterpret_cast<uint32_t>(addr));
}


void CodePatcher::EmitCondition(Condition cond) { // REVIEWEDBY: CG
  Instr instr = Assembler::instr_at(masm_.pc_);
  ASSERT(cond == eq || cond == ne);
  ASSERT(Assembler::IsCondBranch(instr));
  instr = (instr & ~0x200);     // Changed to bt
  if (cond == ne)
    instr |= 0x200;             // Changed to bf
  masm_.emit(instr);
}

void MacroAssembler::TruncatingDiv(Register result, // REVIEWEDBY: CG
                                   Register dividend,
                                   int32_t divisor) {
  ASSERT(!dividend.is(result));
  ASSERT(!dividend.is(ip));
  ASSERT(!result.is(ip));
  MultiplierAndShift ms(divisor);
  mov(ip, Operand(ms.multiplier()));
  smull(ip, result, dividend, ip);
  if (divisor > 0 && ms.multiplier() < 0) {
    add(result, result, Operand(dividend));
  }
  if (divisor < 0 && ms.multiplier() > 0) {
    sub(result, result, Operand(dividend));
  }
  if (ms.shift() > 0) asr(result, result, Operand(ms.shift())); // DIFF: codegen
  lsr(ip, dividend, Operand(31)); // DIFF: codegen
  add(result, result, ip); // DIFF: codegen
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
