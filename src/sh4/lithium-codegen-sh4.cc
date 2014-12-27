// Copyright 2012 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// FILE: SAMEAS: arm, REVIEWEDBY: CG, not fully implemented

#include "v8.h"

#include "sh4/lithium-codegen-sh4.h"
#include "sh4/lithium-gap-resolver-sh4.h"
#include "code-stubs.h"
#include "stub-cache.h"
#include "hydrogen-osr.h"

namespace v8 {
namespace internal {

#include "map-sh4.h"  // SH4: define arm->sh4 register map

class SafepointGenerator V8_FINAL : public CallWrapper {
 public:
  SafepointGenerator(LCodeGen* codegen,
                     LPointerMap* pointers,
                     Safepoint::DeoptMode mode)
      : codegen_(codegen),
        pointers_(pointers),
        deopt_mode_(mode) { }
  virtual ~SafepointGenerator() {}

  virtual void BeforeCall(int call_size) const V8_OVERRIDE {}

  virtual void AfterCall() const V8_OVERRIDE {
    codegen_->RecordSafepoint(pointers_, deopt_mode_);
  }

 private:
  LCodeGen* codegen_;
  LPointerMap* pointers_;
  Safepoint::DeoptMode deopt_mode_;
};


#define __ masm()->

bool LCodeGen::GenerateCode() { // REVIEWEDBY: CG
  LPhase phase("Z_Code generation", chunk());
  ASSERT(is_unused());
  status_ = GENERATING;

  // Open a frame scope to indicate that there is a frame on the stack.  The
  // NONE indicates that the scope shouldn't actually generate code to set up
  // the frame (that is done in GeneratePrologue).
  FrameScope frame_scope(masm_, StackFrame::NONE);

  return GeneratePrologue() &&
      GenerateBody() &&
      GenerateDeferredCode() &&
      GenerateDeoptJumpTable() &&
      GenerateSafepointTable();
}


void LCodeGen::FinishCode(Handle<Code> code) { // REVIEWEDBY: CG
  ASSERT(is_done());
  code->set_stack_slots(GetStackSlotCount());
  code->set_safepoint_table_offset(safepoints_.GetCodeOffset());
  if (code->is_optimized_code()) RegisterWeakObjectsInOptimizedCode(code);
  PopulateDeoptimizationData(code);
}


void LCodeGen::SaveCallerDoubles() { // REVIEWEDBY: CG
  ASSERT(info()->saves_caller_doubles());
  ASSERT(NeedsEagerFrame());
  Comment(";;; Save clobbered callee double registers");
  int count = 0;
  BitVector* doubles = chunk()->allocated_double_registers();
  BitVector::Iterator save_iterator(doubles);
  while (!save_iterator.Done()) {
    __ vstr(DwVfpRegister::FromAllocationIndex(save_iterator.Current()),
            MemOperand(sp, count * kDoubleSize));
    save_iterator.Advance();
    count++;
  }
}


void LCodeGen::RestoreCallerDoubles() { // REVIEWEDBY: CG
  ASSERT(info()->saves_caller_doubles());
  ASSERT(NeedsEagerFrame());
  Comment(";;; Restore clobbered callee double registers");
  BitVector* doubles = chunk()->allocated_double_registers();
  BitVector::Iterator save_iterator(doubles);
  int count = 0;
  while (!save_iterator.Done()) {
    __ vldr(DwVfpRegister::FromAllocationIndex(save_iterator.Current()),
             MemOperand(sp, count * kDoubleSize));
    save_iterator.Advance();
    count++;
  }
}


bool LCodeGen::GeneratePrologue() { // REVIEWEDBY: CG
  ASSERT(is_generating());

  if (info()->IsOptimizing()) {
    ProfileEntryHookStub::MaybeCallEntryHook(masm_);

#ifdef DEBUG
    if (strlen(FLAG_stop_at) > 0 &&
        info_->function()->name()->IsUtf8EqualTo(CStrVector(FLAG_stop_at))) {
      __ stop("stop_at");
    }
#endif

    // r1: Callee's JS function.
    // cp: Callee's context.
    // pp: Callee's constant pool pointer (if FLAG_enable_ool_constant_pool)
    // fp: Caller's frame pointer.
    // lr: Caller's pc.

    // Sloppy mode functions and builtins need to replace the receiver with the
    // global proxy when called as functions (without an explicit receiver
    // object).
    if (info_->this_has_uses() &&
        info_->strict_mode() == SLOPPY &&
        !info_->is_native()) {
      Label ok;
      int receiver_offset = info_->scope()->num_parameters() * kPointerSize;
      __ ldr(r2, MemOperand(sp, receiver_offset));
      __ CompareRoot(r2, Heap::kUndefinedValueRootIndex);
      __ bf_near(&ok); // DIFF: codegen

      __ ldr(r2, GlobalObjectOperand());
      __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalReceiverOffset));

      __ str(r2, MemOperand(sp, receiver_offset));

      __ bind(&ok);
    }
  }

  info()->set_prologue_offset(masm_->pc_offset());
  if (NeedsEagerFrame()) {
    __ Prologue(info()->IsStub() ? BUILD_STUB_FRAME : BUILD_FUNCTION_FRAME);
    frame_is_built_ = true;
    info_->AddNoFrameRange(0, masm_->pc_offset());
  }

  // Reserve space for the stack slots needed by the code.
  int slots = GetStackSlotCount();
  if (slots > 0) {
    if (FLAG_debug_code) {
      __ sub(sp,  sp, Operand(slots * kPointerSize));
      __ push(r0);
      __ push(r1);
      __ add(r0, sp, Operand(slots *  kPointerSize));
      __ mov(r1, Operand(kSlotsZapValue));
      Label loop;
      __ bind(&loop);
      __ sub(r0, r0, Operand(kPointerSize));
      __ str(r1, MemOperand(r0, 2 * kPointerSize));
      __ cmp(r0, sp);
      __ b(ne, &loop);
      __ pop(r1);
      __ pop(r0);
    } else {
      __ sub(sp,  sp, Operand(slots * kPointerSize));
    }
  }

  if (info()->saves_caller_doubles()) {
    SaveCallerDoubles();
  }

  // Possibly allocate a local context.
  int heap_slots = info()->num_heap_slots() - Context::MIN_CONTEXT_SLOTS;
  if (heap_slots > 0) {
    Comment(";;; Allocate local context");
    // Argument to NewContext is the function, which is in r1.
    if (heap_slots <= FastNewContextStub::kMaximumSlots) {
      FastNewContextStub stub(isolate(), heap_slots);
      __ CallStub(&stub);
    } else {
      __ push(r1);
      __ CallRuntime(Runtime::kHiddenNewFunctionContext, 1);
    }
    RecordSafepoint(Safepoint::kNoLazyDeopt);
    // Context is returned in both r0 and cp.  It replaces the context
    // passed to us.  It's saved in the stack and kept live in cp.
    __ mov(cp, r0);
    __ str(r0, MemOperand(fp, StandardFrameConstants::kContextOffset));
    // Copy any necessary parameters into the context.
    int num_parameters = scope()->num_parameters();
    for (int i = 0; i < num_parameters; i++) {
      Variable* var = scope()->parameter(i);
      if (var->IsContextSlot()) {
        int parameter_offset = StandardFrameConstants::kCallerSPOffset +
            (num_parameters - 1 - i) * kPointerSize;
        // Load parameter from stack.
        __ ldr(r0, MemOperand(fp, parameter_offset));
        // Store it in the context.
        MemOperand target = ContextOperand(cp, var->index());
        __ str(r0, target);
        // Update the write barrier. This clobbers r3 and r0.
        __ RecordWriteContextSlot(
            cp,
            target.offset(),
            r0,
            r3,
            GetLinkRegisterState(),
            kSaveFPRegs);
      }
    }
    Comment(";;; End allocate local context");
  }

  // Trace the call.
  if (FLAG_trace && info()->IsOptimizing()) {
    // We have not executed any compiled code yet, so cp still holds the
    // incoming context.
    __ CallRuntime(Runtime::kTraceEnter, 0);
  }
  return !is_aborted();
}


void LCodeGen::GenerateOsrPrologue() { // REVIEWEDBY: CG
  // Generate the OSR entry prologue at the first unknown OSR value, or if there
  // are none, at the OSR entrypoint instruction.
  if (osr_pc_offset_ >= 0) return;

  osr_pc_offset_ = masm()->pc_offset();

  // Adjust the frame size, subsuming the unoptimized frame into the
  // optimized frame.
  int slots = GetStackSlotCount() - graph()->osr()->UnoptimizedFrameSlots();
  ASSERT(slots >= 0);
  __ sub(sp, sp, Operand(slots * kPointerSize));
}


void LCodeGen::GenerateBodyInstructionPre(LInstruction* instr) { // REVIEWEDBY: CG
  if (instr->IsCall()) {
    EnsureSpaceForLazyDeopt(Deoptimizer::patch_size());
  }
  if (!instr->IsLazyBailout() && !instr->IsGap()) {
    safepoints_.BumpLastLazySafepointIndex();
  }
}


bool LCodeGen::GenerateDeferredCode() { // REVIEWEDBY: CG
  ASSERT(is_generating());
  if (deferred_.length() > 0) {
    for (int i = 0; !is_aborted() && i < deferred_.length(); i++) {
      LDeferredCode* code = deferred_[i];

      HValue* value =
          instructions_->at(code->instruction_index())->hydrogen_value();
      RecordAndWritePosition(
          chunk()->graph()->SourcePositionToScriptPosition(value->position()));

      Comment(";;; <@%d,#%d> "
              "-------------------- Deferred %s --------------------",
              code->instruction_index(),
              code->instr()->hydrogen_value()->id(),
              code->instr()->Mnemonic());
      __ bind(code->entry());
      if (NeedsDeferredFrame()) {
        Comment(";;; Build frame");
        ASSERT(!frame_is_built_);
        ASSERT(info()->IsStub());
        frame_is_built_ = true;
        __ PushFixedFrame();
        __ mov(scratch0(), Operand(Smi::FromInt(StackFrame::STUB)));
        __ push(scratch0());
        // Adjust FP to point to saved FP.
        __ add(fp, sp, Operand(StandardFrameConstants::kFixedFrameSizeFromFp));
        Comment(";;; Deferred code");
      }
      code->Generate();
      if (NeedsDeferredFrame()) {
        Comment(";;; Destroy frame");
        ASSERT(frame_is_built_);
        __ pop(ip);
        __ PopFixedFrame();
        frame_is_built_ = false;
      }
      __ jmp(code->exit());
    }
  }

  // Force constant pool emission at the end of the deferred code to make
  // sure that no constant pools are emitted after.
  masm()->CheckConstPool(true, false);

  return !is_aborted();
}


bool LCodeGen::GenerateDeoptJumpTable() { // REVIEWEDBY: CG
  // Check that the jump table is accessible from everywhere in the function
  // code, i.e. that offsets to the table can be encoded in the 24bit signed
  // immediate of a branch instruction.
  // To simplify we consider the code size from the first instruction to the
  // end of the jump table. We also don't consider the pc load delta.
  // Each entry in the jump table generates one instruction and inlines one
  // 32bit data after it.
  // SH4: branches to Deopt jump table are not limited in range as they
  // use far branch. No need to check for jump table length as on ARM.

  if (deopt_jump_table_.length() > 0) {
    Comment(";;; -------------------- Jump table --------------------");
  }
  Label table_start;
  __ bind(&table_start);
  Label needs_frame;
  for (int i = 0; i < deopt_jump_table_.length(); i++) {
    __ bind(&deopt_jump_table_[i].label);
    Address entry = deopt_jump_table_[i].address;
    Deoptimizer::BailoutType type = deopt_jump_table_[i].bailout_type;
    int id = Deoptimizer::GetDeoptimizationId(isolate(), entry, type);
    if (id == Deoptimizer::kNotDeoptimizationEntry) {
      Comment(";;; jump table entry %d.", i);
    } else {
      Comment(";;; jump table entry %d: deoptimization bailout %d.", i, id);
    }
    if (deopt_jump_table_[i].needs_frame) {
      ASSERT(!info()->saves_caller_doubles());
      __ mov(ip, Operand(ExternalReference::ForDeoptEntry(entry)));
      if (needs_frame.is_bound()) {
        __ b(&needs_frame);
      } else {
        __ bind(&needs_frame);
        __ PushFixedFrame();
        // This variant of deopt can only be used with stubs. Since we don't
        // have a function pointer to install in the stack frame that we're
        // building, install a special marker there instead.
        ASSERT(info()->IsStub());
        __ mov(scratch0(), Operand(Smi::FromInt(StackFrame::STUB)));
        __ push(scratch0());
        __ add(fp, sp, Operand(StandardFrameConstants::kFixedFrameSizeFromFp));
        __ jsr(ip); // DIFF: codegen
      }
    } else {
      if (info()->saves_caller_doubles()) {
        ASSERT(info()->IsStub());
        RestoreCallerDoubles();
      }
      __ mov(ip, Operand(ExternalReference::ForDeoptEntry(entry))); // DIFF: codegen
      __ jsr(ip); // DIFF: codegen
    }
    masm()->CheckConstPool(false, false);
  }

  // Force constant pool emission at the end of the deopt jump table to make
  // sure that no constant pools are emitted after.
  masm()->CheckConstPool(true, false);

  // The deoptimization jump table is the last part of the instruction
  // sequence. Mark the generated code as done unless we bailed out.
  if (!is_aborted()) status_ = DONE;
  return !is_aborted();
}


bool LCodeGen::GenerateSafepointTable() {
  ASSERT(is_done());
  safepoints_.Emit(masm(), GetStackSlotCount());
  return !is_aborted();
}


Register LCodeGen::ToRegister(int index) const {
  return Register::FromAllocationIndex(index);
}


DwVfpRegister LCodeGen::ToDoubleRegister(int index) const {
  return DwVfpRegister::FromAllocationIndex(index);
}


Register LCodeGen::ToRegister(LOperand* op) const {
  ASSERT(op->IsRegister());
  return ToRegister(op->index());
}


Register LCodeGen::EmitLoadRegister(LOperand* op, Register scratch) { // REVIEWEDBY: CG
  if (op->IsRegister()) {
    return ToRegister(op->index());
  } else if (op->IsConstantOperand()) {
    LConstantOperand* const_op = LConstantOperand::cast(op);
    HConstant* constant = chunk_->LookupConstant(const_op);
    Handle<Object> literal = constant->handle(isolate());
    Representation r = chunk_->LookupLiteralRepresentation(const_op);
    if (r.IsInteger32()) {
      ASSERT(literal->IsNumber());
      __ mov(scratch, Operand(static_cast<int32_t>(literal->Number())));
    } else if (r.IsDouble()) {
      Abort(kEmitLoadRegisterUnsupportedDoubleImmediate);
    } else {
      ASSERT(r.IsSmiOrTagged());
      __ Move(scratch, literal);
    }
    return scratch;
  } else if (op->IsStackSlot()) {
    __ ldr(scratch, ToMemOperand(op));
    return scratch;
  }
  UNREACHABLE();
  return scratch;
}


DwVfpRegister LCodeGen::ToDoubleRegister(LOperand* op) const { // REVIEWEDBY: CG
  ASSERT(op->IsDoubleRegister());
  return ToDoubleRegister(op->index());
}


DwVfpRegister LCodeGen::EmitLoadDoubleRegister(LOperand* op, // REVIEWEDBY: CG
                                               SwVfpRegister flt_scratch,
                                               DwVfpRegister dbl_scratch) {
  if (op->IsDoubleRegister()) {
    return ToDoubleRegister(op->index());
  } else if (op->IsConstantOperand()) {
    LConstantOperand* const_op = LConstantOperand::cast(op);
    HConstant* constant = chunk_->LookupConstant(const_op);
    Handle<Object> literal = constant->handle(isolate());
    Representation r = chunk_->LookupLiteralRepresentation(const_op);
    if (r.IsInteger32()) {
      ASSERT(literal->IsNumber());
      // SH4: note: flt_scratch is not used
      __ mov(ip, Operand(static_cast<int32_t>(literal->Number())));
      __ vcvt_f64_s32(dbl_scratch, ip); // DIFF: codegen
      return dbl_scratch;
    } else if (r.IsDouble()) {
      Abort(kUnsupportedDoubleImmediate);
    } else if (r.IsTagged()) {
      Abort(kUnsupportedTaggedImmediate);
    }
  } else if (op->IsStackSlot()) {
    // TODO(regis): Why is vldr not taking a MemOperand?
    // __ vldr(dbl_scratch, ToMemOperand(op));
    MemOperand mem_op = ToMemOperand(op);
    ASSERT(mem_op.am() == Offset);
    ASSERT(mem_op.roffset().is(no_reg));
    __ vldr(dbl_scratch, mem_op.rn(), mem_op.offset());
    return dbl_scratch;
  }
  UNREACHABLE();
  return dbl_scratch;
}


Handle<Object> LCodeGen::ToHandle(LConstantOperand* op) const { // REVIEWEDBY: CG
  HConstant* constant = chunk_->LookupConstant(op);
  ASSERT(chunk_->LookupLiteralRepresentation(op).IsSmiOrTagged());
  return constant->handle(isolate());
}


bool LCodeGen::IsInteger32(LConstantOperand* op) const { // REVIEWEDBY: CG
  return chunk_->LookupLiteralRepresentation(op).IsSmiOrInteger32();
}


bool LCodeGen::IsSmi(LConstantOperand* op) const { // REVIEWEDBY: CG
  return chunk_->LookupLiteralRepresentation(op).IsSmi();
}


int32_t LCodeGen::ToInteger32(LConstantOperand* op) const { // REVIEWEDBY: CG
  return ToRepresentation(op, Representation::Integer32());
}


int32_t LCodeGen::ToRepresentation(LConstantOperand* op, // REVIEWEDBY: CG
                                   const Representation& r) const {
  HConstant* constant = chunk_->LookupConstant(op);
  int32_t value = constant->Integer32Value();
  if (r.IsInteger32()) return value;
  ASSERT(r.IsSmiOrTagged());
  return reinterpret_cast<int32_t>(Smi::FromInt(value));
}


Smi* LCodeGen::ToSmi(LConstantOperand* op) const { // REVIEWEDBY: CG
  HConstant* constant = chunk_->LookupConstant(op);
  return Smi::FromInt(constant->Integer32Value());
}


double LCodeGen::ToDouble(LConstantOperand* op) const { // REVIEWEDBY: CG
  HConstant* constant = chunk_->LookupConstant(op);
  ASSERT(constant->HasDoubleValue());
  return constant->DoubleValue();
}


Operand LCodeGen::ToOperand(LOperand* op) { // REVIEWEDBY: CG
  if (op->IsConstantOperand()) {
    LConstantOperand* const_op = LConstantOperand::cast(op);
    HConstant* constant = chunk()->LookupConstant(const_op);
    Representation r = chunk_->LookupLiteralRepresentation(const_op);
    if (r.IsSmi()) {
      ASSERT(constant->HasSmiValue());
      return Operand(Smi::FromInt(constant->Integer32Value()));
    } else if (r.IsInteger32()) {
      ASSERT(constant->HasInteger32Value());
      return Operand(constant->Integer32Value());
    } else if (r.IsDouble()) {
      Abort(kToOperandUnsupportedDoubleImmediate);
    }
    ASSERT(r.IsTagged());
    return Operand(constant->handle(isolate()));
  } else if (op->IsRegister()) {
    return Operand(ToRegister(op));
  } else if (op->IsDoubleRegister()) {
    Abort(kToOperandIsDoubleRegisterUnimplemented);
    return Operand::Zero();
  }
  // Stack slots not implemented, use ToMemOperand instead.
  UNREACHABLE();
  return Operand::Zero();
}


static int ArgumentsOffsetWithoutFrame(int index) {
  ASSERT(index < 0);
  return -(index + 1) * kPointerSize;
}


MemOperand LCodeGen::ToMemOperand(LOperand* op) const { // REVIEWEDBY: CG
  ASSERT(!op->IsRegister());
  ASSERT(!op->IsDoubleRegister());
  ASSERT(op->IsStackSlot() || op->IsDoubleStackSlot());
  if (NeedsEagerFrame()) {
    return MemOperand(fp, StackSlotOffset(op->index()));
  } else {
    // Retrieve parameter without eager stack-frame relative to the
    // stack-pointer.
    return MemOperand(sp, ArgumentsOffsetWithoutFrame(op->index()));
  }
}


MemOperand LCodeGen::ToHighMemOperand(LOperand* op) const { // REVIEWEDBY: CG
  ASSERT(op->IsDoubleStackSlot());
  if (NeedsEagerFrame()) {
    return MemOperand(fp, StackSlotOffset(op->index()) + kPointerSize);
  } else {
    // Retrieve parameter without eager stack-frame relative to the
    // stack-pointer.
    return MemOperand(
        sp, ArgumentsOffsetWithoutFrame(op->index()) + kPointerSize);
  }
}


void LCodeGen::WriteTranslation(LEnvironment* environment, // REVIEWEDBY: CG
                                Translation* translation) {
  if (environment == NULL) return;

  // The translation includes one command per value in the environment.
  int translation_size = environment->translation_size();
  // The output frame height does not include the parameters.
  int height = translation_size - environment->parameter_count();

  WriteTranslation(environment->outer(), translation);
  bool has_closure_id = !info()->closure().is_null() &&
      !info()->closure().is_identical_to(environment->closure());
  int closure_id = has_closure_id
      ? DefineDeoptimizationLiteral(environment->closure())
      : Translation::kSelfLiteralId;

  switch (environment->frame_type()) {
    case JS_FUNCTION:
      translation->BeginJSFrame(environment->ast_id(), closure_id, height);
      break;
    case JS_CONSTRUCT:
      translation->BeginConstructStubFrame(closure_id, translation_size);
      break;
    case JS_GETTER:
      ASSERT(translation_size == 1);
      ASSERT(height == 0);
      translation->BeginGetterStubFrame(closure_id);
      break;
    case JS_SETTER:
      ASSERT(translation_size == 2);
      ASSERT(height == 0);
      translation->BeginSetterStubFrame(closure_id);
      break;
    case STUB:
      translation->BeginCompiledStubFrame();
      break;
    case ARGUMENTS_ADAPTOR:
      translation->BeginArgumentsAdaptorFrame(closure_id, translation_size);
      break;
  }

  int object_index = 0;
  int dematerialized_index = 0;
  for (int i = 0; i < translation_size; ++i) {
    LOperand* value = environment->values()->at(i);
    AddToTranslation(environment,
                     translation,
                     value,
                     environment->HasTaggedValueAt(i),
                     environment->HasUint32ValueAt(i),
                     &object_index,
                     &dematerialized_index);
  }
}


void LCodeGen::AddToTranslation(LEnvironment* environment, // REVIEWEDBY: CG
                                Translation* translation,
                                LOperand* op,
                                bool is_tagged,
                                bool is_uint32,
                                int* object_index_pointer,
                                int* dematerialized_index_pointer) {
  if (op == LEnvironment::materialization_marker()) {
    int object_index = (*object_index_pointer)++;
    if (environment->ObjectIsDuplicateAt(object_index)) {
      int dupe_of = environment->ObjectDuplicateOfAt(object_index);
      translation->DuplicateObject(dupe_of);
      return;
    }
    int object_length = environment->ObjectLengthAt(object_index);
    if (environment->ObjectIsArgumentsAt(object_index)) {
      translation->BeginArgumentsObject(object_length);
    } else {
      translation->BeginCapturedObject(object_length);
    }
    int dematerialized_index = *dematerialized_index_pointer;
    int env_offset = environment->translation_size() + dematerialized_index;
    *dematerialized_index_pointer += object_length;
    for (int i = 0; i < object_length; ++i) {
      LOperand* value = environment->values()->at(env_offset + i);
      AddToTranslation(environment,
                       translation,
                       value,
                       environment->HasTaggedValueAt(env_offset + i),
                       environment->HasUint32ValueAt(env_offset + i),
                       object_index_pointer,
                       dematerialized_index_pointer);
    }
    return;
  }

  if (op->IsStackSlot()) {
    if (is_tagged) {
      translation->StoreStackSlot(op->index());
    } else if (is_uint32) {
      translation->StoreUint32StackSlot(op->index());
    } else {
      translation->StoreInt32StackSlot(op->index());
    }
  } else if (op->IsDoubleStackSlot()) {
    translation->StoreDoubleStackSlot(op->index());
  } else if (op->IsRegister()) {
    Register reg = ToRegister(op);
    if (is_tagged) {
      translation->StoreRegister(reg);
    } else if (is_uint32) {
      translation->StoreUint32Register(reg);
    } else {
      translation->StoreInt32Register(reg);
    }
  } else if (op->IsDoubleRegister()) {
    DoubleRegister reg = ToDoubleRegister(op);
    translation->StoreDoubleRegister(reg);
  } else if (op->IsConstantOperand()) {
    HConstant* constant = chunk()->LookupConstant(LConstantOperand::cast(op));
    int src_index = DefineDeoptimizationLiteral(constant->handle(isolate()));
    translation->StoreLiteral(src_index);
  } else {
    UNREACHABLE();
  }
}


int LCodeGen::CallCodeSize(Handle<Code> code, int call_offset, RelocInfo::Mode mode) { // REVIEWEDBY: CG
  int size = masm()->CallSize(code, call_offset, mode);
  if (code->kind() == Code::BINARY_OP_IC ||
      code->kind() == Code::COMPARE_IC) {
    size += Assembler::kInstrSize;  // extra nop() added in CallCodeGeneric.
  }
  return size;
}


void LCodeGen::CallCode(Handle<Code> code, // REVIEWEDBY: CG
                        RelocInfo::Mode mode,
                        LInstruction* instr,
                        TargetAddressStorageMode storage_mode) {
  CallCodeGeneric(code, mode, instr, RECORD_SIMPLE_SAFEPOINT, storage_mode);
}


void LCodeGen::CallCodeGeneric(Handle<Code> code, // REVIEWEDBY: CG
                               RelocInfo::Mode mode,
                               LInstruction* instr,
                               SafepointMode safepoint_mode,
                               TargetAddressStorageMode storage_mode) {
  ASSERT(instr != NULL);
  // Block literal pool emission to ensure nop indicating no inlined smi code
  // is in the correct position.
  Assembler::BlockConstPoolScope block_const_pool(masm());
  __ Call(code, mode, TypeFeedbackId::None(), storage_mode); // DIFF: codegen
  RecordSafepointWithLazyDeopt(instr, safepoint_mode);

  // Signal that we don't inline smi code before these stubs in the
  // optimizing code generator.
  if (code->kind() == Code::BINARY_OP_IC ||
      code->kind() == Code::COMPARE_IC) {
    __ nop(); // SH4: ref to EmitPatchInfo()
  }
}


void LCodeGen::CallRuntime(const Runtime::Function* function, // REVIEWEDBY: CG
                           int num_arguments,
                           LInstruction* instr,
                           SaveFPRegsMode save_doubles) {
  ASSERT(instr != NULL);

  __ CallRuntime(function, num_arguments, save_doubles);

  RecordSafepointWithLazyDeopt(instr, RECORD_SIMPLE_SAFEPOINT);
}


void LCodeGen::LoadContextFromDeferred(LOperand* context) { // REVIEWEDBY: CG
  if (context->IsRegister()) {
    __ Move(cp, ToRegister(context));
  } else if (context->IsStackSlot()) {
    __ ldr(cp, ToMemOperand(context));
  } else if (context->IsConstantOperand()) {
    HConstant* constant =
        chunk_->LookupConstant(LConstantOperand::cast(context));
    __ Move(cp, Handle<Object>::cast(constant->handle(isolate())));
  } else {
    UNREACHABLE();
  }
}


void LCodeGen::CallRuntimeFromDeferred(Runtime::FunctionId id, // REVIEWEDBY: CG
                                       int argc,
                                       LInstruction* instr,
                                       LOperand* context) {
  LoadContextFromDeferred(context);
  __ CallRuntimeSaveDoubles(id);
  RecordSafepointWithRegisters(
      instr->pointer_map(), argc, Safepoint::kNoLazyDeopt);
}


void LCodeGen::RegisterEnvironmentForDeoptimization(LEnvironment* environment, // REVIEWEDBY: CG
                                                    Safepoint::DeoptMode mode) {
  environment->set_has_been_used();
  if (!environment->HasBeenRegistered()) {
    // Physical stack frame layout:
    // -x ............. -4  0 ..................................... y
    // [incoming arguments] [spill slots] [pushed outgoing arguments]

    // Layout of the environment:
    // 0 ..................................................... size-1
    // [parameters] [locals] [expression stack including arguments]

    // Layout of the translation:
    // 0 ........................................................ size - 1 + 4
    // [expression stack including arguments] [locals] [4 words] [parameters]
    // |>------------  translation_size ------------<|

    int frame_count = 0;
    int jsframe_count = 0;
    for (LEnvironment* e = environment; e != NULL; e = e->outer()) {
      ++frame_count;
      if (e->frame_type() == JS_FUNCTION) {
        ++jsframe_count;
      }
    }
    Translation translation(&translations_, frame_count, jsframe_count, zone());
    WriteTranslation(environment, &translation);
    int deoptimization_index = deoptimizations_.length();
    int pc_offset = masm()->pc_offset();
    environment->Register(deoptimization_index,
                          translation.index(),
                          (mode == Safepoint::kLazyDeopt) ? pc_offset : -1);
    deoptimizations_.Add(environment, zone());
  }
}


void LCodeGen::DeoptimizeIf(Condition condition, // REVIEWEDBY: CG
                            LEnvironment* environment,
                            Deoptimizer::BailoutType bailout_type) {
  ASSERT(condition == ne || condition == eq || condition == al); // DIFF: codegen
  RegisterEnvironmentForDeoptimization(environment, Safepoint::kNoLazyDeopt);
  ASSERT(environment->HasBeenRegistered());
  int id = environment->deoptimization_index();
  ASSERT(info()->IsOptimizing() || info()->IsStub());
  Address entry =
      Deoptimizer::GetDeoptimizationEntry(isolate(), id, bailout_type);
  if (entry == NULL) {
    Abort(kBailoutWasNotPrepared);
    return;
  }


  if (FLAG_deopt_every_n_times != 0 && !info()->IsStub()) {
    Label skip;
    Register scratch = scratch0();
    ExternalReference count = ExternalReference::stress_deopt_count(isolate());

    // Store the condition on the stack if necessary
    if (condition != al) {
      __ mov(scratch, Operand::Zero(), NegateCondition(condition)); // DIFF: codegen
      __ mov(scratch, Operand(1), condition); // DIFF: codegen
      __ push(scratch);
    }

    __ push(r1);
    __ mov(scratch, Operand(count));
    __ ldr(r1, MemOperand(scratch));
    __ sub(r1, r1, Operand(1)); // DIFF: codegen
    __ cmpeq(r1, Operand(0)); // DIFF: codegen // T bit set also for Call() below
    __ mov(r1, Operand(FLAG_deopt_every_n_times), t); // DIFF: codegen
    __ str(r1, MemOperand(scratch));
    __ pop(r1);

    if (condition != al) {
      // Clean up the stack before the deoptimizer call
      __ pop(scratch);
    }

    __ bf_near(&skip); // DIFF: codegen // Skip call if T bit is not set
    __ Call(entry, RelocInfo::RUNTIME_ENTRY); // DIFF: codegen
    __ bind(&skip);

    // 'Restore' the condition in a slightly hacky way. (It would be better
    // to use 'msr' and 'mrs' instructions here, but they are not supported by
    // our ARM simulator).
    if (condition != al) {
      condition = ne;
      __ cmp(scratch, Operand::Zero());
    }
  }

  if (info()->ShouldTrapOnDeopt()) {
    __ stop("trap_on_deopt", condition);
  }

  ASSERT(info()->IsStub() || frame_is_built_);
  // Go through jump table if we need to handle condition, build frame, or
  // restore caller doubles.
  if (condition == al && frame_is_built_ &&
      !info()->saves_caller_doubles()) {
    __ Call(entry, RelocInfo::RUNTIME_ENTRY);
  } else {
    // We often have several deopts to the same entry, reuse the last
    // jump entry if this is the case.
    if (deopt_jump_table_.is_empty() ||
        (deopt_jump_table_.last().address != entry) ||
        (deopt_jump_table_.last().bailout_type != bailout_type) ||
        (deopt_jump_table_.last().needs_frame != !frame_is_built_)) {
      Deoptimizer::JumpTableEntry table_entry(entry,
                                              bailout_type,
                                              !frame_is_built_);
      deopt_jump_table_.Add(table_entry, zone());
    }
    // SH4: this branch will be resolved in
    // in LCodeGen::GenerateDeoptJumpTable()
    // Force kFar to enforce a large branch offset.
    __ b(condition, &deopt_jump_table_.last().label, Label::kFar); // DIFF: codegen
  }
}


void LCodeGen::DeoptimizeIf(Condition condition, // REVIEWEDBY: CG
                            LEnvironment* environment) {
  ASSERT(condition == ne || condition == eq || condition == al); // DIFF: codegen
  Deoptimizer::BailoutType bailout_type = info()->IsStub()
      ? Deoptimizer::LAZY
      : Deoptimizer::EAGER;
  DeoptimizeIf(condition, environment, bailout_type);
}


void LCodeGen::PopulateDeoptimizationData(Handle<Code> code) { // REVIEWEDBY: CG
  int length = deoptimizations_.length();
  if (length == 0) return;
  Handle<DeoptimizationInputData> data =
      DeoptimizationInputData::New(isolate(), length, TENURED);

  Handle<ByteArray> translations =
      translations_.CreateByteArray(isolate()->factory());
  data->SetTranslationByteArray(*translations);
  data->SetInlinedFunctionCount(Smi::FromInt(inlined_function_count_));
  data->SetOptimizationId(Smi::FromInt(info_->optimization_id()));
  if (info_->IsOptimizing()) {
    // Reference to shared function info does not change between phases.
    AllowDeferredHandleDereference allow_handle_dereference;
    data->SetSharedFunctionInfo(*info_->shared_info());
  } else {
    data->SetSharedFunctionInfo(Smi::FromInt(0));
  }

  Handle<FixedArray> literals =
      factory()->NewFixedArray(deoptimization_literals_.length(), TENURED);
  { AllowDeferredHandleDereference copy_handles;
    for (int i = 0; i < deoptimization_literals_.length(); i++) {
      literals->set(i, *deoptimization_literals_[i]);
    }
    data->SetLiteralArray(*literals);
  }

  data->SetOsrAstId(Smi::FromInt(info_->osr_ast_id().ToInt()));
  data->SetOsrPcOffset(Smi::FromInt(osr_pc_offset_));

  // Populate the deoptimization entries.
  for (int i = 0; i < length; i++) {
    LEnvironment* env = deoptimizations_[i];
    data->SetAstId(i, env->ast_id());
    data->SetTranslationIndex(i, Smi::FromInt(env->translation_index()));
    data->SetArgumentsStackHeight(i,
                                  Smi::FromInt(env->arguments_stack_height()));
    data->SetPc(i, Smi::FromInt(env->pc_offset()));
  }
  code->set_deoptimization_data(*data);
}


int LCodeGen::DefineDeoptimizationLiteral(Handle<Object> literal) { // REVIEWEDBY: CG
  int result = deoptimization_literals_.length();
  for (int i = 0; i < deoptimization_literals_.length(); ++i) {
    if (deoptimization_literals_[i].is_identical_to(literal)) return i;
  }
  deoptimization_literals_.Add(literal, zone());
  return result;
}


void LCodeGen::PopulateDeoptimizationLiteralsWithInlinedFunctions() { // REVIEWEDBY: CG
  ASSERT(deoptimization_literals_.length() == 0);

  const ZoneList<Handle<JSFunction> >* inlined_closures =
      chunk()->inlined_closures();

  for (int i = 0, length = inlined_closures->length();
       i < length;
       i++) {
    DefineDeoptimizationLiteral(inlined_closures->at(i));
  }

  inlined_function_count_ = deoptimization_literals_.length();
}


void LCodeGen::RecordSafepointWithLazyDeopt( // REVIEWEDBY: CG
    LInstruction* instr, SafepointMode safepoint_mode) {
  if (safepoint_mode == RECORD_SIMPLE_SAFEPOINT) {
    RecordSafepoint(instr->pointer_map(), Safepoint::kLazyDeopt);
  } else {
    ASSERT(safepoint_mode == RECORD_SAFEPOINT_WITH_REGISTERS_AND_NO_ARGUMENTS);
    RecordSafepointWithRegisters(
        instr->pointer_map(), 0, Safepoint::kLazyDeopt);
  }
}


void LCodeGen::RecordSafepoint( // REVIEWEDBY: CG
    LPointerMap* pointers,
    Safepoint::Kind kind,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {
  ASSERT(expected_safepoint_kind_ == kind);

  const ZoneList<LOperand*>* operands = pointers->GetNormalizedOperands();
  Safepoint safepoint = safepoints_.DefineSafepoint(masm(),
      kind, arguments, deopt_mode);
  for (int i = 0; i < operands->length(); i++) {
    LOperand* pointer = operands->at(i);
    if (pointer->IsStackSlot()) {
      safepoint.DefinePointerSlot(pointer->index(), zone());
    } else if (pointer->IsRegister() && (kind & Safepoint::kWithRegisters)) {
      safepoint.DefinePointerRegister(ToRegister(pointer), zone());
    }
  }
  // SH4: no ool constant pool
  // if (FLAG_enable_ool_constant_pool && (kind & Safepoint::kWithRegisters)) {
  //  // Register pp always contains a pointer to the constant pool.
  //  safepoint.DefinePointerRegister(pp, zone());
  //}
}


void LCodeGen::RecordSafepoint(LPointerMap* pointers, // REVIEWEDBY: CG
                               Safepoint::DeoptMode deopt_mode) {
  RecordSafepoint(pointers, Safepoint::kSimple, 0, deopt_mode);
}


void LCodeGen::RecordSafepoint(Safepoint::DeoptMode deopt_mode) { // REVIEWEDBY: CG
  LPointerMap empty_pointers(zone());
  RecordSafepoint(&empty_pointers, deopt_mode);
}


void LCodeGen::RecordSafepointWithRegisters(LPointerMap* pointers, // REVIEWEDBY: CG
                                            int arguments,
                                            Safepoint::DeoptMode deopt_mode) {
  RecordSafepoint(
      pointers, Safepoint::kWithRegisters, arguments, deopt_mode);
}


void LCodeGen::RecordSafepointWithRegistersAndDoubles( // REVIEWEDBY: CG
    LPointerMap* pointers,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {
  RecordSafepoint(
      pointers, Safepoint::kWithRegistersAndDoubles, arguments, deopt_mode);
}


void LCodeGen::RecordAndWritePosition(int position) { // REVIEWEDBY: CG
  if (position == RelocInfo::kNoPosition) return;
  masm()->positions_recorder()->RecordPosition(position);
  masm()->positions_recorder()->WriteRecordedPositions();
}


static const char* LabelType(LLabel* label) { // REVIEWEDBY: CG
  if (label->is_loop_header()) return " (loop header)";
  if (label->is_osr_entry()) return " (OSR entry)";
  return "";
}


void LCodeGen::DoLabel(LLabel* label) { // REVIEWEDBY: CG
  Comment(";;; <@%d,#%d> -------------------- B%d%s --------------------",
          current_instruction_,
          label->hydrogen_value()->id(),
          label->block_id(),
          LabelType(label));
  __ bind(label->label());
  current_block_ = label->block_id();
  DoGap(label);
}


void LCodeGen::DoParallelMove(LParallelMove* move) { // REVIEWEDBY: CG
  resolver_.Resolve(move);
}


void LCodeGen::DoGap(LGap* gap) { // REVIEWEDBY: CG
  for (int i = LGap::FIRST_INNER_POSITION;
       i <= LGap::LAST_INNER_POSITION;
       i++) {
    LGap::InnerPosition inner_pos = static_cast<LGap::InnerPosition>(i);
    LParallelMove* move = gap->GetParallelMove(inner_pos);
    if (move != NULL) DoParallelMove(move);
  }
}


void LCodeGen::DoInstructionGap(LInstructionGap* instr) { // REVIEWEDBY: CG
  DoGap(instr);
}


void LCodeGen::DoParameter(LParameter* instr) { // REVIEWEDBY: CG
  // Nothing to do.
}


void LCodeGen::DoCallStub(LCallStub* instr) { // REVIEWEDBY: CG
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->result()).is(r0));
  switch (instr->hydrogen()->major_key()) {
    case CodeStub::RegExpExec: {
      RegExpExecStub stub(isolate());
      CallCode(stub.GetCode(), RelocInfo::CODE_TARGET, instr);
      break;
    }
    case CodeStub::SubString: {
      SubStringStub stub(isolate());
      CallCode(stub.GetCode(), RelocInfo::CODE_TARGET, instr);
      break;
    }
    case CodeStub::StringCompare: {
      StringCompareStub stub(isolate());
      CallCode(stub.GetCode(), RelocInfo::CODE_TARGET, instr);
      break;
    }
    default:
      UNREACHABLE();
  }
}


void LCodeGen::DoUnknownOSRValue(LUnknownOSRValue* instr) { // REVIEWEDBY: CG
  GenerateOsrPrologue();
}


void LCodeGen::DoModByPowerOf2I(LModByPowerOf2I* instr) { // REVIEWEDBY: CG
  Register dividend = ToRegister(instr->dividend());
  int32_t divisor = instr->divisor();
  ASSERT(dividend.is(ToRegister(instr->result())));

  // Theoretically, a variation of the branch-free code for integer division by
  // a power of 2 (calculating the remainder via an additional multiplication
  // (which gets simplified to an 'and') and subtraction) should be faster, and
  // this is exactly what GCC and clang emit. Nevertheless, benchmarks seem to
  // indicate that positive dividends are heavily favored, so the branching
  // version performs better.
  HMod* hmod = instr->hydrogen();
  int32_t mask = divisor < 0 ? -(divisor + 1) : (divisor - 1);
  Label dividend_is_not_negative, done;
  if (hmod->CheckFlag(HValue::kLeftCanBeNegative)) {
    __ cmpge(dividend, Operand::Zero()); // DIFF: codegen
    __ b(t, &dividend_is_not_negative); // DIFF: codegen
    // Note that this is correct even for kMinInt operands.
    __ rsb(dividend, dividend, Operand::Zero());
    __ and_(dividend, dividend, Operand(mask));
    __ rsb(dividend, dividend, Operand::Zero()); // DIFF: codegen
    if (hmod->CheckFlag(HValue::kBailoutOnMinusZero)) {
      __ cmp(dividend, Operand(0)); // DIFF: codegen
      DeoptimizeIf(eq, instr->environment());
    }
    __ b(&done);
  }

  __ bind(&dividend_is_not_negative);
  __ and_(dividend, dividend, Operand(mask));
  __ bind(&done);
}


void LCodeGen::DoModByConstI(LModByConstI* instr) { // REVIEWEDBY: CG
  Register dividend = ToRegister(instr->dividend());
  int32_t divisor = instr->divisor();
  Register result = ToRegister(instr->result());
  ASSERT(!dividend.is(result));

  if (divisor == 0) {
    DeoptimizeIf(al, instr->environment());
    return;
  }

  __ TruncatingDiv(result, dividend, Abs(divisor));
  __ mov(ip, Operand(Abs(divisor)));
  __ smull(result, ip, result, ip);
  __ sub(result, dividend, result); // DIFF: codegen

  // Check for negative zero.
  HMod* hmod = instr->hydrogen();
  if (hmod->CheckFlag(HValue::kBailoutOnMinusZero)) {
    Label remainder_not_zero;
    __ cmp(result, Operand(0)); // DIFF: codegen
    __ b(ne, &remainder_not_zero);
    __ cmpge(dividend, Operand::Zero()); // DIFF: codegen
    DeoptimizeIf(f, instr->environment()); // DIFF: codegen
    __ bind(&remainder_not_zero);
  }
}


void LCodeGen::DoModI(LModI* instr) { // REVIEWEDBY: CG
  HMod* hmod = instr->hydrogen();
  if (CpuFeatures::IsSupported(SUDIV)) {
    UNREACHABLE(); // SH4: no SUDIV // DIFF: codegen
  } else {
    // General case, without any SDIV support.
    Register left_reg = ToRegister(instr->left());
    Register right_reg = ToRegister(instr->right());
    Register result_reg = ToRegister(instr->result());
    Register scratch = scratch0();
    ASSERT(!scratch.is(left_reg));
    ASSERT(!scratch.is(right_reg));
    ASSERT(!scratch.is(result_reg));
    DwVfpRegister dividend = ToDoubleRegister(instr->temp());
    DwVfpRegister divisor = ToDoubleRegister(instr->temp2());
    ASSERT(!divisor.is(dividend));
    DwVfpRegister quotient = double_scratch0();
    ASSERT(!quotient.is(dividend));
    ASSERT(!quotient.is(divisor));

    Label done;
    // Check for x % 0, we have to deopt in this case because we can't return a
    // NaN.
    if (hmod->CheckFlag(HValue::kCanBeDivByZero)) {
      __ cmp(right_reg, Operand::Zero());
      DeoptimizeIf(eq, instr->environment());
    }

    __ Move(result_reg, left_reg);
    // Load the arguments in VFP registers. The divisor value is preloaded
    // before. Be careful that 'right_reg' is only live on entry.
    // TODO(svenpanne) The last comments seems to be wrong nowadays.
    __ vcvt_f64_s32(dividend, left_reg); // DIFF: codegen
    __ vcvt_f64_s32(divisor, right_reg); // DIFF: codegen

    // We do not care about the sign of the divisor. Note that we still handle
    // the kMinInt % -1 case correctly, though.
    __ vabs(divisor, divisor);
    // Compute the quotient and round it to a 32bit integer.
    __ vdiv(quotient, dividend, divisor);
    __ vcvt_s32_f64(scratch, quotient); // DIFF: codegen
    __ vcvt_f64_s32(quotient, scratch); // DIFF: codegen

    // Compute the remainder in result.
    __ vmul(double_scratch0(), divisor, quotient);
    __ vcvt_s32_f64(scratch, double_scratch0()); // DIFF: codegen
    __ sub(result_reg, left_reg, scratch); // DIFF: codegen

    // If we care about -0, test if the dividend is <0 and the result is 0.
    if (hmod->CheckFlag(HValue::kBailoutOnMinusZero)) {
      __ cmp(result_reg, Operand(0)); // DIFF: codegen
      __ b(ne, &done);
      __ cmpge(left_reg, Operand::Zero()); // DIFF: codegen
      DeoptimizeIf(f, instr->environment()); // DIFF: codegen
    }
    __ bind(&done);
  }
}


void LCodeGen::DoDivByPowerOf2I(LDivByPowerOf2I* instr) { // REVIEWEDBY: CG
  Register dividend = ToRegister(instr->dividend());
  int32_t divisor = instr->divisor();
  Register result = ToRegister(instr->result());
  ASSERT(divisor == kMinInt || IsPowerOf2(Abs(divisor)));
  ASSERT(!result.is(dividend));

  // Check for (0 / -x) that will produce negative zero.
  HDiv* hdiv = instr->hydrogen();
  if (hdiv->CheckFlag(HValue::kBailoutOnMinusZero) && divisor < 0) {
    __ cmp(dividend, Operand::Zero());
    DeoptimizeIf(eq, instr->environment());
  }
  // Check for (kMinInt / -1).
  if (hdiv->CheckFlag(HValue::kCanOverflow) && divisor == -1) {
    __ cmp(dividend, Operand(kMinInt));
    DeoptimizeIf(eq, instr->environment());
  }
  // Deoptimize if remainder will not be 0.
  if (!hdiv->CheckFlag(HInstruction::kAllUsesTruncatingToInt32) &&
      divisor != 1 && divisor != -1) {
    int32_t mask = divisor < 0 ? -(divisor + 1) : (divisor - 1);
    __ tst(dividend, Operand(mask));
    DeoptimizeIf(ne, instr->environment());
  }

  if (divisor == -1) {  // Nice shortcut, not needed for correctness.
    __ rsb(result, dividend, Operand(0));
    return;
  }
  int32_t shift = WhichPowerOf2Abs(divisor);
  if (shift == 0) {
    __ mov(result, dividend);
  } else if (shift == 1) {
    __ lsr(result, dividend, Operand(31)); // DIFF: codegen
    __ add(result, dividend, result); // DIFF: codegen
  } else {
    __ asr(result, dividend, Operand(31)); // DIFF: codegen
    __ lsr(result, result, Operand(32 - shift)); // DIFF: codegen
    __ add(result, dividend, result); // DIFF: codegen
  }
  if (shift > 0) __ asr(result, result, Operand(shift)); // DIFF: codegen
  if (divisor < 0) __ rsb(result, result, Operand(0));
}


void LCodeGen::DoDivByConstI(LDivByConstI* instr) { // REVIEWEDBY: CG
  Register dividend = ToRegister(instr->dividend());
  int32_t divisor = instr->divisor();
  Register result = ToRegister(instr->result());
  ASSERT(!dividend.is(result));

  if (divisor == 0) {
    DeoptimizeIf(al, instr->environment());
    return;
  }

  // Check for (0 / -x) that will produce negative zero.
  HDiv* hdiv = instr->hydrogen();
  if (hdiv->CheckFlag(HValue::kBailoutOnMinusZero) && divisor < 0) {
    __ cmp(dividend, Operand::Zero());
    DeoptimizeIf(eq, instr->environment());
  }

  __ TruncatingDiv(result, dividend, Abs(divisor));
  if (divisor < 0) __ rsb(result, result, Operand::Zero());

  if (!hdiv->CheckFlag(HInstruction::kAllUsesTruncatingToInt32)) {
    __ mov(ip, Operand(divisor));
    __ smull(scratch0(), ip, result, ip);
    __ sub(scratch0(), scratch0(), dividend); // DIFF: codegen
    __ cmp(scratch0(), Operand(0)); // DIFF: codegen
    DeoptimizeIf(ne, instr->environment());
  }
}


// TODO(svenpanne) Refactor this to avoid code duplication with DoFlooringDivI.
void LCodeGen::DoDivI(LDivI* instr) { // REVIEWEDBY: CG
  HBinaryOperation* hdiv = instr->hydrogen();
  Register dividend = ToRegister(instr->dividend());
  Register divisor = ToRegister(instr->divisor());
  Register result = ToRegister(instr->result());

  // Check for x / 0.
  if (hdiv->CheckFlag(HValue::kCanBeDivByZero)) {
    __ cmp(divisor, Operand::Zero());
    DeoptimizeIf(eq, instr->environment());
  }

  // Check for (0 / -x) that will produce negative zero.
  if (hdiv->CheckFlag(HValue::kBailoutOnMinusZero)) {
    Label positive;
    // SH4: redo the test in all cases
    if (!instr->hydrogen_value()->CheckFlag(HValue::kCanBeDivByZero)) {
      (void)0; // DIFF: codegen
    }
    __ cmpge(divisor, Operand::Zero()); // SH4: redo the test // DIFF: codegen
    __ b(t, &positive); // right >= 0 // DIFF: codegen
    __ cmp(dividend, Operand::Zero());
    DeoptimizeIf(eq, instr->environment());
    __ bind(&positive);
  }

  // Check for (kMinInt / -1).
  if (hdiv->CheckFlag(HValue::kCanOverflow) &&
      (!CpuFeatures::IsSupported(SUDIV) ||
       !hdiv->CheckFlag(HValue::kAllUsesTruncatingToInt32))) {
    Label skip;
    // We don't need to check for overflow when truncating with sdiv
    // support because, on ARM, sdiv kMinInt, -1 -> kMinInt.
    __ cmp(dividend, Operand(kMinInt));
    __ bf_near(&skip);
    __ cmp(divisor, Operand(-1));
    DeoptimizeIf(eq, instr->environment());
    __ bind(&skip);
  }

  if (CpuFeatures::IsSupported(SUDIV)) {
    UNREACHABLE(); // SH4: no SUDIV // DIFF: codegen
  } else {
    DoubleRegister vleft = ToDoubleRegister(instr->temp());
    DoubleRegister vright = double_scratch0();
    __ vcvt_f64_s32(vleft, dividend); // DIFF:codegen
    __ vcvt_f64_s32(vright, divisor);  // DIFF:codegen
    __ vdiv(vleft, vleft, vright);  // vleft now contains the result.
    __ vcvt_s32_f64(result, vleft); // DIFF: codegen
  }

  if (!hdiv->CheckFlag(HValue::kAllUsesTruncatingToInt32)) {
    // Compute remainder and deopt if it's not zero.
    Register remainder = scratch0();
    __ mul(remainder, divisor, dividend); // DIFF: codegen
    __ sub(remainder, result, remainder); // DIFF: codegen
    __ cmp(remainder, Operand::Zero());
    DeoptimizeIf(ne, instr->environment());
  }
}


void LCodeGen::DoMultiplyAddD(LMultiplyAddD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMultiplySubD(LMultiplySubD* instr) {
  __ UNIMPLEMENTED_BREAK();
}

void LCodeGen::DoFlooringDivByPowerOf2I(LFlooringDivByPowerOf2I* instr) {
  __ UNIMPLEMENTED_BREAK();
}

void LCodeGen::DoFlooringDivByConstI(LFlooringDivByConstI* instr) {
  __ UNIMPLEMENTED_BREAK();
}

// TODO(svenpanne) Refactor this to avoid code duplication with DoDivI.
void LCodeGen::DoFlooringDivI(LFlooringDivI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMulI(LMulI* instr) { // REVIEWEDBY: CG
  Register result = ToRegister(instr->result());
  // Note that result may alias left.
  Register left = ToRegister(instr->left());
  LOperand* right_op = instr->right();

  bool bailout_on_minus_zero =
    instr->hydrogen()->CheckFlag(HValue::kBailoutOnMinusZero);
  bool overflow = instr->hydrogen()->CheckFlag(HValue::kCanOverflow);

  if (right_op->IsConstantOperand()) {
    int32_t constant = ToInteger32(LConstantOperand::cast(right_op));

    if (bailout_on_minus_zero && (constant < 0)) {
      // The case of a null constant will be handled separately.
      // If constant is negative and left is null, the result should be -0.
      __ cmp(left, Operand::Zero());
      DeoptimizeIf(eq, instr->environment());
    }

    switch (constant) {
      case -1:
        if (overflow) {
          __ rsbv(result, left, Operand::Zero()); // DIFF: codegen
          DeoptimizeIf(t, instr->environment()); // DIFF: codegen
        } else {
          __ rsb(result, left, Operand::Zero());
        }
        break;
      case 0:
        if (bailout_on_minus_zero) {
          // If left is strictly negative and the constant is null, the
          // result is -0. Deoptimize if required, otherwise return 0.
          __ cmpge(left, Operand::Zero()); // DIFF: codegen
          DeoptimizeIf(f, instr->environment()); // DIFF: codegen
        }
        __ mov(result, Operand::Zero());
        break;
      case 1:
        __ Move(result, left);
        break;
      default:
        // Multiplying by powers of two and powers of two plus or minus
        // one can be done faster with shifted operands.
        // For other constants we emit standard code.
        int32_t mask = constant >> 31;
        uint32_t constant_abs = (constant + mask) ^ mask;

        if (IsPowerOf2(constant_abs)) {
          int32_t shift = WhichPowerOf2(constant_abs);
          __ lsl(result, left, Operand(shift)); // DIFF: codegen
          // Correct the sign of the result is the constant is negative.
          if (constant < 0)  __ rsb(result, result, Operand::Zero());
        } else if (IsPowerOf2(constant_abs - 1)) {
          int32_t shift = WhichPowerOf2(constant_abs - 1);
          __ lsl(result, left, Operand(shift)); // DIFF: codegen
          __ add(result, left, result); // DIFF: codegen
          // Correct the sign of the result is the constant is negative.
          if (constant < 0)  __ rsb(result, result, Operand::Zero());
        } else if (IsPowerOf2(constant_abs + 1)) {
          int32_t shift = WhichPowerOf2(constant_abs + 1);
          __ lsl(result, left, Operand(shift)); // DIFF: codegen
          __ rsb(result, left, result); // DIFF: codegen
          // Correct the sign of the result is the constant is negative.
          if (constant < 0)  __ rsb(result, result, Operand::Zero());
        } else {
          // Generate standard code.
          __ mov(ip, Operand(constant));
          __ mul(result, left, ip);
        }
    }

  } else {
    ASSERT(right_op->IsRegister());
    Register right = ToRegister(right_op);

    if (overflow) {
      Register scratch = scratch0();
      // scratch:result = left * right.
      if (instr->hydrogen()->representation().IsSmi()) {
        __ SmiUntag(result, left);
        __ smull(result, scratch, result, right);
      } else {
        __ smull(result, scratch, left, right);
      }
      __ asr(ip, result, Operand(31)); // DIFF: codegen
      __ cmp(scratch, ip); // DIFF: codegen
      DeoptimizeIf(ne, instr->environment());
    } else {
      if (instr->hydrogen()->representation().IsSmi()) {
        __ SmiUntag(result, left);
        __ mul(result, result, right);
      } else {
        __ mul(result, left, right);
      }
    }

    if (bailout_on_minus_zero) {
      Label done;
      Register scratch = scratch0(); // DIFF: codegen
      // Done if none or both negative (test high bits)
      __ lxor(scratch, left, right); // DIFF: codegen
      __ cmpge(scratch, Operand(0)); // DIFF: codegen
      __ bt(&done);
      // Bail out if the result is minus zero.
      __ cmp(result, Operand::Zero());
      DeoptimizeIf(eq, instr->environment());
      __ bind(&done);
    }
  }
}


void LCodeGen::DoBitI(LBitI* instr) { // REVIEWEDBY: CG
  LOperand* left_op = instr->left();
  LOperand* right_op = instr->right();
  ASSERT(left_op->IsRegister());
  Register left = ToRegister(left_op);
  Register result = ToRegister(instr->result());
  Operand right(no_reg);

  if (right_op->IsStackSlot()) {
    right = Operand(EmitLoadRegister(right_op, ip));
  } else {
    ASSERT(right_op->IsRegister() || right_op->IsConstantOperand());
    right = ToOperand(right_op);
  }

  switch (instr->op()) {
    case Token::BIT_AND:
      __ and_(result, left, right);
      break;
    case Token::BIT_OR:
      __ orr(result, left, right);
      break;
    case Token::BIT_XOR:
      if (right_op->IsConstantOperand() && right.immediate() == int32_t(~0)) {
        __ mvn(result, left); // DIFF: codegen
      } else {
        __ eor(result, left, right);
      }
      break;
    default:
      UNREACHABLE();
      break;
  }
}


void LCodeGen::DoShiftI(LShiftI* instr) { // REVIEWEDBY: CG
  // Both 'left' and 'right' are "used at start" (see LCodeGen::DoShift), so
  // result may alias either of them.
  LOperand* right_op = instr->right();
  Register left = ToRegister(instr->left());
  Register result = ToRegister(instr->result());
  Register scratch = scratch0();
  Register scratch2 = ToRegister(instr->temp1()); // DIFF: codegen
  Label skip;

  if (right_op->IsRegister()) {
    // Mask the right_op operand.
    __ and_(scratch, ToRegister(right_op), Operand(0x1F));
    switch (instr->op()) {
      case Token::ROR:
        // SH4: ROR not available, use lsr/lsl and an additional scratch
        // SH4: if 0, skip to avoid the undefined: left << (32-0) case
        __ mov(result, left); // DIFF: codegen
        __ cmp(scratch, Operand(0)); // DIFF: codegen
        __ bt_near(&skip);
        __ lsr(scratch2, left, scratch, true/*in_range*/); // DIFF: codegen
        __ rsb(scratch, scratch, Operand(32)); // DIFF: codegen
        __ lsl(scratch, left, scratch, true/*in_range*/); // DIFF: codegen
        __ lor(result, scratch, scratch2); // Diff: codegen
        __ bind(&skip);
        break;
      case Token::SAR:
        __ asr(result, left, scratch, true/*in_range*/); // DIFF: codegen
        break;
      case Token::SHR:
        if (instr->can_deopt()) {
          __ lsr(result, left, scratch, true/*in_range*/);
          __ cmpge(result, Operand(0)); // DIFF: codegen
          DeoptimizeIf(f, instr->environment()); // DIFF: codegen
        } else {
          __ lsr(result, left, scratch); // DIFF: codegen
        }
        break;
      case Token::SHL:
        __ lsl(result, left, scratch, true/*in_range*/); // DIFF: codegen
        break;
      default:
        UNREACHABLE();
        break;
    }
  } else {
    // Mask the right_op operand.
    int value = ToInteger32(LConstantOperand::cast(right_op));
    uint8_t shift_count = static_cast<uint8_t>(value & 0x1F);
    switch (instr->op()) {
      case Token::ROR:
        if (shift_count != 0) {
          // SH4: ROR not available, use lsr/lsl
          __ lsr(scratch, left, Operand(shift_count)); // DIFF: codegen
          __ lsl(result, left, Operand(32 - shift_count)); // DIFF: codegen
          __ lor(result, result, scratch); // Diff: codegen
        } else {
          __ Move(result, left);
        }
        break;
      case Token::SAR:
        if (shift_count != 0) {
          __ asr(result, left, Operand(shift_count)); // DIFF: codegen
        } else {
          __ Move(result, left);
        }
        break;
    case Token::SHR:
        if (shift_count != 0) {
          __ lsr(result, left, Operand(shift_count)); // DIFF: codegen
        } else {
          if (instr->can_deopt()) {
            __ cmpge(left, Operand(0)); // DIFF: codegen
            DeoptimizeIf(f, instr->environment()); // DIFF: codegen
          }
          __ Move(result, left);
        }
        break;
      case Token::SHL:
        if (shift_count != 0) {
          if (instr->hydrogen_value()->representation().IsSmi() &&
              instr->can_deopt()) {
            if (shift_count != 1) {
              __ lsl(result, left, Operand(shift_count - 1)); // DIFF: codegen
              __ SmiTag(result, result, SetT); // DIFF: codegen
            } else {
              __ SmiTag(result, left, SetT); // DIFF: codegen
            }
            DeoptimizeIf(t, instr->environment()); // DIFF: codegen
          } else {
            __ lsl(result, left, Operand(shift_count)); // DIFF: codegen
          }
        } else {
          __ Move(result, left);
        }
        break;
      default:
        UNREACHABLE();
        break;
    }
  }
}


void LCodeGen::DoSubI(LSubI* instr) { // REVIEWEDBY: CG
  LOperand* left = instr->left();
  LOperand* right = instr->right();
  LOperand* result = instr->result();
  bool can_overflow = instr->hydrogen()->CheckFlag(HValue::kCanOverflow);
  SBit set_cond = can_overflow ? SetT : LeaveT; // DIFF: codegen

  if (right->IsStackSlot()) {
    Register right_reg = EmitLoadRegister(right, ip);
    if (set_cond == SetT)
      __ subv(ToRegister(result), ToRegister(left), Operand(right_reg)); // DIFF: codegen
    else
      __ sub(ToRegister(result), ToRegister(left), Operand(right_reg)); // DIFF: codegen
  } else {
    ASSERT(right->IsRegister() || right->IsConstantOperand());
    if (set_cond == SetT)
      __ subv(ToRegister(result), ToRegister(left), ToOperand(right)); // DIFF: codegen
    else
      __ sub(ToRegister(result), ToRegister(left), ToOperand(right)); // DIFF: codegen
  }

  if (can_overflow) {
    DeoptimizeIf(t, instr->environment()); // DIFF: codegen
  }
}


void LCodeGen::DoRSubI(LRSubI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoConstantI(LConstantI* instr) { // REVIEWEDBY: CG
  __ mov(ToRegister(instr->result()), Operand(instr->value()));
}


void LCodeGen::DoConstantS(LConstantS* instr) { // REVIEWEDBY: CG
  __ mov(ToRegister(instr->result()), Operand(instr->value()));
}


void LCodeGen::DoConstantD(LConstantD* instr) { // REVIEWEDBY: CG
  ASSERT(instr->result()->IsDoubleRegister());
  DwVfpRegister result = ToDoubleRegister(instr->result());
  double v = instr->value();
  __ Vmov(result, v, scratch0());
}


void LCodeGen::DoConstantE(LConstantE* instr) { // REVIEWEDBY: CG
  __ mov(ToRegister(instr->result()), Operand(instr->value()));
}


void LCodeGen::DoConstantT(LConstantT* instr) { // REVIEWEDBY: CG
  Handle<Object> object = instr->value(isolate());
  AllowDeferredHandleDereference smi_check;
  if (instr->hydrogen()->HasObjectMap()) {
    Handle<Map> object_map = instr->hydrogen()->ObjectMap().handle();
    ASSERT(object->IsHeapObject());
    ASSERT(!object_map->is_stable() ||
           *object_map == Handle<HeapObject>::cast(object)->map());
    USE(object_map);
  }
  __ Move(ToRegister(instr->result()), object);
}


void LCodeGen::DoMapEnumLength(LMapEnumLength* instr) { // REVIEWEDBY: CG
  Register result = ToRegister(instr->result());
  Register map = ToRegister(instr->value());
  __ EnumLength(result, map);
}


void LCodeGen::DoDateField(LDateField* instr) {
  __ UNIMPLEMENTED_BREAK();
}


MemOperand LCodeGen::BuildSeqStringOperand(Register string, // REVIEWEDBY: CG
                                           LOperand* index,
                                           String::Encoding encoding) {
  if (index->IsConstantOperand()) {
    int offset = ToInteger32(LConstantOperand::cast(index));
    if (encoding == String::TWO_BYTE_ENCODING) {
      offset *= kUC16Size;
    }
    STATIC_ASSERT(kCharSize == 1);
    return FieldMemOperand(string, SeqString::kHeaderSize + offset);
  }
  Register scratch = scratch0();
  ASSERT(!scratch.is(string));
  ASSERT(!scratch.is(ToRegister(index)));
  if (encoding == String::ONE_BYTE_ENCODING) {
    __ add(scratch, string, Operand(ToRegister(index)));
  } else {
    STATIC_ASSERT(kUC16Size == 2);
    __ lsl(scratch, ToRegister(index), Operand(1)); // DIFF: codegen
    __ add(scratch, string, scratch);  // DIFF: codegen
  }
  return FieldMemOperand(scratch, SeqString::kHeaderSize);
}


void LCodeGen::DoSeqStringGetChar(LSeqStringGetChar* instr) { // REVIEWEDBY: CG
  String::Encoding encoding = instr->hydrogen()->encoding();
  Register string = ToRegister(instr->string());
  Register result = ToRegister(instr->result());

  if (FLAG_debug_code) {
    Register scratch = scratch0();
    __ ldr(scratch, FieldMemOperand(string, HeapObject::kMapOffset));
    __ ldrb(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));

    __ and_(scratch, scratch,
            Operand(kStringRepresentationMask | kStringEncodingMask));
    static const uint32_t one_byte_seq_type = kSeqStringTag | kOneByteStringTag;
    static const uint32_t two_byte_seq_type = kSeqStringTag | kTwoByteStringTag;
    __ cmp(scratch, Operand(encoding == String::ONE_BYTE_ENCODING
                            ? one_byte_seq_type : two_byte_seq_type));
    __ Check(eq, kUnexpectedStringType);
  }

  MemOperand operand = BuildSeqStringOperand(string, instr->index(), encoding);
  if (encoding == String::ONE_BYTE_ENCODING) {
    __ ldrb(result, operand);
  } else {
    __ ldrh(result, operand);
  }
}


void LCodeGen::DoSeqStringSetChar(LSeqStringSetChar* instr) { // REVIEWEDBY: CG
  String::Encoding encoding = instr->hydrogen()->encoding();
  Register string = ToRegister(instr->string());
  Register value = ToRegister(instr->value());

  if (FLAG_debug_code) {
    Register index = ToRegister(instr->index());
    static const uint32_t one_byte_seq_type = kSeqStringTag | kOneByteStringTag;
    static const uint32_t two_byte_seq_type = kSeqStringTag | kTwoByteStringTag;
    int encoding_mask =
        instr->hydrogen()->encoding() == String::ONE_BYTE_ENCODING
        ? one_byte_seq_type : two_byte_seq_type;
    __ EmitSeqStringSetCharCheck(string, index, value, encoding_mask);
  }

  MemOperand operand = BuildSeqStringOperand(string, instr->index(), encoding);
  if (encoding == String::ONE_BYTE_ENCODING) {
    __ strb(value, operand);
  } else {
    __ strh(value, operand);
  }
}


void LCodeGen::DoAddI(LAddI* instr) { // REVIEWEDBY: CG
  LOperand* left = instr->left();
  LOperand* right = instr->right();
  LOperand* result = instr->result();
  bool can_overflow = instr->hydrogen()->CheckFlag(HValue::kCanOverflow);
  SBit set_cond = can_overflow ? SetT : LeaveT; // DIFF: codegen

  if (right->IsStackSlot()) {
    Register right_reg = EmitLoadRegister(right, ip);
    if (set_cond == SetT)
      __ addv(ToRegister(result), ToRegister(left), Operand(right_reg)); // DIFF: codegen
    else
      __ add(ToRegister(result), ToRegister(left), Operand(right_reg));
  } else {
    ASSERT(right->IsRegister() || right->IsConstantOperand());
    if (set_cond == SetT)
      __ addv(ToRegister(result), ToRegister(left), ToOperand(right));
    else
      __ add(ToRegister(result), ToRegister(left), ToOperand(right));
  }

  if (can_overflow) {
    DeoptimizeIf(t, instr->environment()); // DIFF: codegen
  }
}


void LCodeGen::DoMathMinMax(LMathMinMax* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoArithmeticD(LArithmeticD* instr) { // REVIEWEDBY: CG
  DwVfpRegister left = ToDoubleRegister(instr->left());
  DwVfpRegister right = ToDoubleRegister(instr->right());
  DwVfpRegister result = ToDoubleRegister(instr->result());
  switch (instr->op()) {
    case Token::ADD:
      __ vadd(result, left, right);
      break;
    case Token::SUB:
      __ vsub(result, left, right);
      break;
    case Token::MUL:
      __ vmul(result, left, right);
      break;
    case Token::DIV:
      __ vdiv(result, left, right);
      break;
    case Token::MOD: {
      __ PrepareCallCFunction(0, 2, scratch0());
      __ MovToFloatParameters(left, right);
      __ CallCFunction(
          ExternalReference::mod_two_doubles_operation(isolate()),
          0, 2);
      // Move the result in the double result register.
      __ MovFromFloatResult(result);
      break;
    }
    default:
      UNREACHABLE();
      break;
  }
}


void LCodeGen::DoArithmeticT(LArithmeticT* instr) { // REVIEWEDBY: CG
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->left()).is(r1));
  ASSERT(ToRegister(instr->right()).is(r0));
  ASSERT(ToRegister(instr->result()).is(r0));

  BinaryOpICStub stub(isolate(), instr->op(), NO_OVERWRITE);
  // Block literal pool emission to ensure nop indicating no inlined smi code
  // is in the correct position.
  Assembler::BlockConstPoolScope block_const_pool(masm());
  CallCode(stub.GetCode(), RelocInfo::CODE_TARGET, instr);
}


template<class InstrType>
void LCodeGen::EmitBranch(InstrType instr, Condition condition) { // REVIEWEDBY: CG
  int left_block = instr->TrueDestination(chunk_);
  int right_block = instr->FalseDestination(chunk_);

  int next_block = GetNextEmittedBlock();
  ASSERT(condition == eq || condition == ne || condition == al); // DIFF: codegen
  if (right_block == left_block || condition == al) {
    EmitGoto(left_block);
  } else if (left_block == next_block) {
    __ b(NegateCondition(condition), chunk_->GetAssemblyLabel(right_block));
  } else if (right_block == next_block) {
    __ b(condition, chunk_->GetAssemblyLabel(left_block));
  } else {
    __ b(condition, chunk_->GetAssemblyLabel(left_block));
    __ b(chunk_->GetAssemblyLabel(right_block));
  }
}


template<class InstrType>
void LCodeGen::EmitFalseBranch(InstrType instr, Condition condition) { // REVIEWEDBY: CG
  int false_block = instr->FalseDestination(chunk_);
  ASSERT(condition == eq || condition == ne || condition == al); // DIFF: codegen
  __ b(condition, chunk_->GetAssemblyLabel(false_block));
}


void LCodeGen::DoDebugBreak(LDebugBreak* instr) { // REVIEWEDBY: CG
  __ stop("LBreak");
}


void LCodeGen::DoBranch(LBranch* instr) { // REVIEWEDBY: CG
  Representation r = instr->hydrogen()->value()->representation();
  if (r.IsInteger32() || r.IsSmi()) {
    ASSERT(!info()->IsStub());
    Register reg = ToRegister(instr->value());
    __ cmp(reg, Operand::Zero());
    EmitBranch(instr, ne);
  } else if (r.IsDouble()) {
    ASSERT(!info()->IsStub());
    Label is_false;
    DwVfpRegister reg = ToDoubleRegister(instr->value());
    // Test the double value. Zero and NaN are false.
    __ dcmpeq(reg, reg); // DIFF: codegen
    EmitFalseBranch(instr, ne); // NaN
    __ dcmpeq(reg, kDoubleRegZero); // DIFF: codegen
    EmitBranch(instr, ne);
  } else {
    ASSERT(r.IsTagged());
    Register reg = ToRegister(instr->value());
    HType type = instr->hydrogen()->value()->type();
    if (type.IsBoolean()) {
      ASSERT(!info()->IsStub());
      __ CompareRoot(reg, Heap::kTrueValueRootIndex);
      EmitBranch(instr, eq);
    } else if (type.IsSmi()) {
      ASSERT(!info()->IsStub());
      __ cmp(reg, Operand::Zero());
      EmitBranch(instr, ne);
    } else if (type.IsJSArray()) {
      ASSERT(!info()->IsStub());
      EmitBranch(instr, al);
    } else if (type.IsHeapNumber()) {
      ASSERT(!info()->IsStub());
      DwVfpRegister dbl_scratch = double_scratch0();
      __ vldr(dbl_scratch, FieldMemOperand(reg, HeapNumber::kValueOffset));
      // Test the double value. Zero and NaN are false.
      __ dcmpeq(dbl_scratch, dbl_scratch); // DIFF: codegen
      EmitFalseBranch(instr, ne); // NaN
      __ dcmpeq(dbl_scratch, kDoubleRegZero); // DIFF: codegen
      EmitBranch(instr, ne);
    } else if (type.IsString()) {
      ASSERT(!info()->IsStub());
      __ ldr(ip, FieldMemOperand(reg, String::kLengthOffset));
      __ cmp(ip, Operand::Zero());
      EmitBranch(instr, ne);
    } else {
      ToBooleanStub::Types expected = instr->hydrogen()->expected_input_types();
      // Avoid deopts in the case where we've never executed this path before.
      if (expected.IsEmpty()) expected = ToBooleanStub::Types::Generic();

      if (expected.Contains(ToBooleanStub::UNDEFINED)) {
        // undefined -> false.
        __ CompareRoot(reg, Heap::kUndefinedValueRootIndex);
        __ b(eq, instr->FalseLabel(chunk_));
      }
      if (expected.Contains(ToBooleanStub::BOOLEAN)) {
        // Boolean -> its value.
        __ CompareRoot(reg, Heap::kTrueValueRootIndex);
        __ b(eq, instr->TrueLabel(chunk_));
        __ CompareRoot(reg, Heap::kFalseValueRootIndex);
        __ b(eq, instr->FalseLabel(chunk_));
      }
      if (expected.Contains(ToBooleanStub::NULL_TYPE)) {
        // 'null' -> false.
        __ CompareRoot(reg, Heap::kNullValueRootIndex);
        __ b(eq, instr->FalseLabel(chunk_));
      }

      if (expected.Contains(ToBooleanStub::SMI)) {
        // Smis: 0 -> false, all other -> true.
        __ cmp(reg, Operand::Zero());
        __ b(eq, instr->FalseLabel(chunk_));
        __ JumpIfSmi(reg, instr->TrueLabel(chunk_));
      } else if (expected.NeedsMap()) {
        // If we need a map later and have a Smi -> deopt.
        __ SmiTst(reg);
        DeoptimizeIf(eq, instr->environment());
      }

      const Register map = scratch0();
      if (expected.NeedsMap()) {
        __ ldr(map, FieldMemOperand(reg, HeapObject::kMapOffset));

        if (expected.CanBeUndetectable()) {
          // Undetectable -> false.
          __ ldrb(ip, FieldMemOperand(map, Map::kBitFieldOffset));
          __ tst(ip, Operand(1 << Map::kIsUndetectable));
          __ b(ne, instr->FalseLabel(chunk_));
        }
      }

      if (expected.Contains(ToBooleanStub::SPEC_OBJECT)) {
        // spec object -> true.
        __ CompareInstanceType(map, ip, FIRST_SPEC_OBJECT_TYPE, ge); // DIFF: codegen
        __ b(t, instr->TrueLabel(chunk_)); // DIFF: codegen
      }

      if (expected.Contains(ToBooleanStub::STRING)) {
        // String value -> false iff empty.
        Label not_string;
        __ CompareInstanceType(map, ip, FIRST_NONSTRING_TYPE, ge); // DIFF: codegen
        __ b(t, &not_string); // DIFF: codegen
        __ ldr(ip, FieldMemOperand(reg, String::kLengthOffset));
        __ cmp(ip, Operand::Zero());
        __ b(ne, instr->TrueLabel(chunk_));
        __ b(instr->FalseLabel(chunk_));
        __ bind(&not_string);
      }

      if (expected.Contains(ToBooleanStub::SYMBOL)) {
        // Symbol value -> true.
        __ CompareInstanceType(map, ip, SYMBOL_TYPE, eq); // DIFF: codegen
        __ b(t, instr->TrueLabel(chunk_)); // DIFF: codegen
      }

      if (expected.Contains(ToBooleanStub::HEAP_NUMBER)) {
        // heap number -> false iff +0, -0, or NaN.
        DwVfpRegister dbl_scratch = double_scratch0();
        Label not_heap_number;
        __ CompareRoot(map, Heap::kHeapNumberMapRootIndex);
        __ bf_near(&not_heap_number);
        __ vldr(dbl_scratch, FieldMemOperand(reg, HeapNumber::kValueOffset));
        // Test the double value. Zero and NaN are false.
        __ dcmpeq(dbl_scratch, dbl_scratch); // DIFF: codegen
        __ b(ne, instr->FalseLabel(chunk_));
        __ dcmpeq(dbl_scratch, kDoubleRegZero); // DIFF: codegen
        __ b(eq, instr->FalseLabel(chunk_));
        __ b(instr->TrueLabel(chunk_));
        __ bind(&not_heap_number);
      }

      if (!expected.IsGeneric()) {
        // We've seen something for the first time -> deopt.
        // This can only happen if we are not generic already.
        DeoptimizeIf(al, instr->environment());
      }
    }
  }
}


void LCodeGen::EmitGoto(int block) { // REVIEWEDBY: CG
  if (!IsNextEmittedBlock(block)) {
    __ jmp(chunk_->GetAssemblyLabel(LookupDestination(block)));
  }
}


void LCodeGen::DoGoto(LGoto* instr) { // REVIEWEDBY: CG
  EmitGoto(instr->block_id());
}


Condition LCodeGen::TokenToCondition(Token::Value op, bool is_unsigned) { // REVIEWEDBY: CG
  Condition cond = kNoCondition;
  switch (op) {
    case Token::EQ:
    case Token::EQ_STRICT:
      cond = eq;
      break;
    case Token::NE:
    case Token::NE_STRICT:
      cond = ne;
      break;
    case Token::LT:
      cond = is_unsigned ? lo : lt;
      break;
    case Token::GT:
      cond = is_unsigned ? hi : gt;
      break;
    case Token::LTE:
      cond = is_unsigned ? ls : le;
      break;
    case Token::GTE:
      cond = is_unsigned ? hs : ge;
      break;
    case Token::IN:
    case Token::INSTANCEOF:
    default:
      UNREACHABLE();
  }
  return cond;
}


// SH4 Specific:
// Emits a modified cond when comparing double or jump to the nan label.
static void EmitVFPCompare(LCodeGen* codegen,
                           Condition *cond,
                           DwVfpRegister double1,
                           DwVfpRegister double2,
                           Label *nan)
{
  // TODO: SH4: may move this to macro-assembler-sh4.cc
  // Test for NaN
  codegen-> __ dcmpeq(double1, double1);
  codegen-> __ bf(nan);
  codegen-> __ dcmpeq(double2, double2);
  codegen-> __ bf(nan);

  // Generate code and modify condition
  switch (*cond) {
  case eq:
    codegen-> __ dcmpeq(double1, double2);
    *cond = t;
    break;
  case ne:
    codegen-> __ dcmpeq(double1, double2);
    *cond = f;
    break;
  case gt:
    codegen-> __ dcmpgt(double1, double2);
    *cond = t;
    break;
  case lt:
    codegen-> __ dcmpgt(double2, double1);
    *cond = t;
    break;
  default:
    UNIMPLEMENTED();
  }
}

void LCodeGen::DoCompareNumericAndBranch(LCompareNumericAndBranch* instr) { // REVIEWEDBY: CG
  LOperand* left = instr->left();
  LOperand* right = instr->right();
  Condition cond = TokenToCondition(instr->op(), false);

  if (left->IsConstantOperand() && right->IsConstantOperand()) {
    // We can statically evaluate the comparison.
    double left_val = ToDouble(LConstantOperand::cast(left));
    double right_val = ToDouble(LConstantOperand::cast(right));
    int next_block = EvalComparison(instr->op(), left_val, right_val) ?
        instr->TrueDestination(chunk_) : instr->FalseDestination(chunk_);
    EmitGoto(next_block);
  } else {
    if (instr->is_double()) {
      // Compare left and right operands as doubles and load the
      // If a NaN is involved, i.e. the result is unordered (V set),
      // // resulting flags into the normal status register.
      EmitVFPCompare(this, &cond, ToDoubleRegister(left), ToDoubleRegister(right), instr->FalseLabel(chunk_)); // DIFF: codegen
    } else {
      if (right->IsConstantOperand()) {
        int32_t value = ToInteger32(LConstantOperand::cast(right));
        if (instr->hydrogen_value()->representation().IsSmi()) {
          __ cmp(&cond, ToRegister(left), Operand(Smi::FromInt(value))); // DIFF: codegen
        } else {
          __ cmp(&cond, ToRegister(left), Operand(value)); // DIFF: codegen
        }
      } else if (left->IsConstantOperand()) {
        int32_t value = ToInteger32(LConstantOperand::cast(left));
        if (instr->hydrogen_value()->representation().IsSmi()) {
          __ cmp(&cond, ToRegister(right), Operand(Smi::FromInt(value))); // DIFF: codegen
        } else {
          __ cmp(&cond, ToRegister(right), Operand(value)); // DIFF: codegen
        }
        // We transposed the operands. Reverse the condition.
        cond = ReverseCondition(cond);
      } else {
        __ cmp(&cond, ToRegister(left), ToRegister(right)); // DIFF: codegen
      }
    }
    EmitBranch(instr, cond);
  }
}


void LCodeGen::DoCmpObjectEqAndBranch(LCmpObjectEqAndBranch* instr) { // REVIEWEDBY: CG
  Register left = ToRegister(instr->left());
  Register right = ToRegister(instr->right());

  __ cmp(left, Operand(right));
  EmitBranch(instr, eq);
}


void LCodeGen::DoCmpHoleAndBranch(LCmpHoleAndBranch* instr) { // REVIEWEDBY: CG
  if (instr->hydrogen()->representation().IsTagged()) {
    Register input_reg = ToRegister(instr->object());
    __ mov(ip, Operand(factory()->the_hole_value()));
    __ cmp(input_reg, ip);
    EmitBranch(instr, eq);
    return;
  }

  DwVfpRegister input_reg = ToDoubleRegister(instr->object());
  __ dcmpeq(input_reg, input_reg); // DIFF: codegen
  EmitFalseBranch(instr, eq); // not a NaN // DIFF: codegen

  Register scratch = scratch0();
  __ VmovHigh(scratch, input_reg);
  __ cmp(scratch, Operand(kHoleNanUpper32));
  EmitBranch(instr, eq);
}


void LCodeGen::DoCompareMinusZeroAndBranch(LCompareMinusZeroAndBranch* instr) { // REVIEWEDBY: CG
  Representation rep = instr->hydrogen()->value()->representation();
  ASSERT(!rep.IsInteger32());
  Register scratch = ToRegister(instr->temp());

  if (rep.IsDouble()) {
    DwVfpRegister value = ToDoubleRegister(instr->value());
    __ dcmpeq(value, ToDoubleRegister(0.0)); // DIFF: codegen
    EmitFalseBranch(instr, ne); // not a NaN // DIFF: codegen
    __ VmovHigh(scratch, value);
    __ cmp(scratch, Operand(0x80000000));
  } else {
    Label skip;
    Register value = ToRegister(instr->value());
    __ CheckMap(value,
                scratch,
                Heap::kHeapNumberMapRootIndex,
                instr->FalseLabel(chunk()),
                DO_SMI_CHECK);
    __ ldr(scratch, FieldMemOperand(value, HeapNumber::kExponentOffset));
    __ ldr(ip, FieldMemOperand(value, HeapNumber::kMantissaOffset));
    __ cmp(scratch, Operand(0x80000000));
    __ bf_near(&skip);
    __ cmp(ip, Operand(0x00000000)); // DIFF: codegen
    __ bind(&skip);
  }
  EmitBranch(instr, eq);
}


Condition LCodeGen::EmitIsObject(Register input, // REVIEWEDBY: CG
                                 Register temp1,
                                 Label* is_not_object,
                                 Label* is_object) {
  Register temp2 = scratch0();
  __ JumpIfSmi(input, is_not_object);

  __ LoadRoot(temp2, Heap::kNullValueRootIndex);
  __ cmp(input, temp2);
  __ b(eq, is_object);

  // Load map.
  __ ldr(temp1, FieldMemOperand(input, HeapObject::kMapOffset));
  // Undetectable objects behave like undefined.
  __ ldrb(temp2, FieldMemOperand(temp1, Map::kBitFieldOffset));
  __ tst(temp2, Operand(1 << Map::kIsUndetectable));
  __ b(ne, is_not_object);

  // Load instance type and check that it is in object type range.
  __ ldrb(temp2, FieldMemOperand(temp1, Map::kInstanceTypeOffset));
  __ cmpge(temp2, Operand(FIRST_NONCALLABLE_SPEC_OBJECT_TYPE)); // DIFF: codegen
  __ b(f, is_not_object); // DIFF: codegen
  __ cmpgt(temp2, Operand(LAST_NONCALLABLE_SPEC_OBJECT_TYPE)); // DIFF: codegen
  return f; // DIFF: codegen
}


void LCodeGen::DoIsObjectAndBranch(LIsObjectAndBranch* instr) { // REVIEWEDBY: CG
  Register reg = ToRegister(instr->value());
  Register temp1 = ToRegister(instr->temp());

  Condition true_cond =
      EmitIsObject(reg, temp1,
          instr->FalseLabel(chunk_), instr->TrueLabel(chunk_));

  EmitBranch(instr, true_cond);
}


Condition LCodeGen::EmitIsString(Register input, // REVIEWEDBY: CG
                                 Register temp1,
                                 Label* is_not_string,
                                 SmiCheck check_needed = INLINE_SMI_CHECK) {
  if (check_needed == INLINE_SMI_CHECK) {
    __ JumpIfSmi(input, is_not_string);
  }
  __ CompareObjectType(input, temp1, temp1, FIRST_NONSTRING_TYPE, ge); // DIFF: codegen

  return f; // DIFF: codegen
}


void LCodeGen::DoIsStringAndBranch(LIsStringAndBranch* instr) { // REVIEWEDBY: CG
  Register reg = ToRegister(instr->value());
  Register temp1 = ToRegister(instr->temp());

  SmiCheck check_needed =
      instr->hydrogen()->value()->IsHeapObject()
          ? OMIT_SMI_CHECK : INLINE_SMI_CHECK;
  Condition true_cond =
      EmitIsString(reg, temp1, instr->FalseLabel(chunk_), check_needed);

  EmitBranch(instr, true_cond);
}


void LCodeGen::DoIsSmiAndBranch(LIsSmiAndBranch* instr) { // REVIEWEDBY: CG
  Register input_reg = EmitLoadRegister(instr->value(), ip);
  __ SmiTst(input_reg);
  EmitBranch(instr, eq);
}


void LCodeGen::DoIsUndetectableAndBranch(LIsUndetectableAndBranch* instr) { // REVIEWEDBY: CG
  Register input = ToRegister(instr->value());
  Register temp = ToRegister(instr->temp());

  if (!instr->hydrogen()->value()->IsHeapObject()) {
    __ JumpIfSmi(input, instr->FalseLabel(chunk_));
  }
  __ ldr(temp, FieldMemOperand(input, HeapObject::kMapOffset));
  __ ldrb(temp, FieldMemOperand(temp, Map::kBitFieldOffset));
  __ tst(temp, Operand(1 << Map::kIsUndetectable));
  EmitBranch(instr, ne);
}


void LCodeGen::DoStringCompareAndBranch(LStringCompareAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


static InstanceType TestType(HHasInstanceTypeAndBranch* instr) { // REVIEWEDBY: CG
  InstanceType from = instr->from();
  InstanceType to = instr->to();
  if (from == FIRST_TYPE) return to;
  ASSERT(from == to || to == LAST_TYPE);
  return from;
}


static Condition BranchCondition(HHasInstanceTypeAndBranch* instr, Condition &cmp_cond) { // REVIEWEDBY: CG
  InstanceType from = instr->from();
  InstanceType to = instr->to();
  // SH4: must generate both compare and branch conditions
  if (from == to) { cmp_cond = eq; return t; }
  if (to == LAST_TYPE) { cmp_cond = hs; return t; }
  if (from == FIRST_TYPE) { cmp_cond = hi; return f; }
  UNREACHABLE();
  cmp_cond = eq;
  return t;
}


void LCodeGen::DoHasInstanceTypeAndBranch(LHasInstanceTypeAndBranch* instr) { // REVIEWEDBY: CG
  Register scratch = scratch0();
  Register input = ToRegister(instr->value());
  // SH4: must generate both compare and branch conditions
  Condition br_cond, cmp_cond;

  if (!instr->hydrogen()->value()->IsHeapObject()) {
    __ JumpIfSmi(input, instr->FalseLabel(chunk_));
  }

  br_cond = BranchCondition(instr->hydrogen(), cmp_cond);
  __ CompareObjectType(input, scratch, scratch, TestType(instr->hydrogen()), cmp_cond);
  EmitBranch(instr, br_cond);
}


void LCodeGen::DoGetCachedArrayIndex(LGetCachedArrayIndex* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoHasCachedArrayIndexAndBranch(
    LHasCachedArrayIndexAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


// Branches to a label or falls through with the answer in flags.  Trashes
// the temp registers, but not the input.
void LCodeGen::EmitClassOfTest(Label* is_true,
                               Label* is_false,
                               Handle<String>class_name,
                               Register input,
                               Register temp,
                               Register temp2) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoClassOfTestAndBranch(LClassOfTestAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCmpMapAndBranch(LCmpMapAndBranch* instr) { // REVIEWEDBY: CG
  Register reg = ToRegister(instr->value());
  Register temp = ToRegister(instr->temp());

  __ ldr(temp, FieldMemOperand(reg, HeapObject::kMapOffset));
  __ cmp(temp, Operand(instr->map()));
  EmitBranch(instr, eq);
}


void LCodeGen::DoInstanceOf(LInstanceOf* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr,
                                               Label* map_check) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCmpT(LCmpT* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoReturn(LReturn* instr) { // REVIEWEDBY: CG
  if (FLAG_trace && info()->IsOptimizing()) {
    // Push the return value on the stack as the parameter.
    // Runtime::TraceExit returns its parameter in r0.  We're leaving the code
    // managed by the register allocator and tearing down the frame, it's
    // safe to write to the context register.
    __ push(r0);
    __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
    __ CallRuntime(Runtime::kTraceExit, 1);
  }
  if (info()->saves_caller_doubles()) {
    RestoreCallerDoubles();
  }
  int no_frame_start = -1;
  if (NeedsEagerFrame()) {
    no_frame_start = masm_->LeaveFrame(StackFrame::JAVA_SCRIPT);
  }
  if (instr->has_constant_parameter_count()) {
    int parameter_count = ToInteger32(instr->constant_parameter_count());
    int32_t sp_delta = (parameter_count + 1) * kPointerSize;
    if (sp_delta != 0) {
      __ add(sp, sp, Operand(sp_delta));
    }
  } else {
    Register reg = ToRegister(instr->parameter_count());
    // The argument count parameter is a smi
    __ SmiUntag(reg);
    __ lsl(reg, reg, Operand(kPointerSizeLog2)); // DIFF: codegen
    __ add(sp, sp, reg);
  }

  __ rts(); // DIFF: codegen

  if (no_frame_start != -1) {
    info_->AddNoFrameRange(no_frame_start, masm_->pc_offset());
  }
}


void LCodeGen::DoLoadGlobalCell(LLoadGlobalCell* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadGlobalGeneric(LLoadGlobalGeneric* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreGlobalCell(LStoreGlobalCell* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadContextSlot(LLoadContextSlot* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreContextSlot(LStoreContextSlot* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadNamedField(LLoadNamedField* instr) { // REVIEWEDBY: CG
  HObjectAccess access = instr->hydrogen()->access();
  int offset = access.offset();
  Register object = ToRegister(instr->object());

  if (access.IsExternalMemory()) {
    Register result = ToRegister(instr->result());
    MemOperand operand = MemOperand(object, offset);
    __ Load(result, operand, access.representation());
    return;
  }

  if (instr->hydrogen()->representation().IsDouble()) {
    DwVfpRegister result = ToDoubleRegister(instr->result());
    __ vldr(result, FieldMemOperand(object, offset));
    return;
  }

  Register result = ToRegister(instr->result());
  if (!access.IsInobject()) {
    __ ldr(result, FieldMemOperand(object, JSObject::kPropertiesOffset));
    object = result;
  }
  MemOperand operand = FieldMemOperand(object, offset);
  __ Load(result, operand, access.representation());
}


void LCodeGen::DoLoadNamedGeneric(LLoadNamedGeneric* instr) { // REVIEWEDBY: CG
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->object()).is(r0));
  ASSERT(ToRegister(instr->result()).is(r0));

  // Name is always in r2.
  __ mov(r2, Operand(instr->name()));
  Handle<Code> ic = LoadIC::initialize_stub(isolate(), NOT_CONTEXTUAL);
  CallCode(ic, RelocInfo::CODE_TARGET, instr, NEVER_INLINE_TARGET_ADDRESS);
}


void LCodeGen::DoLoadFunctionPrototype(LLoadFunctionPrototype* instr) { // REVIEWEDBY: CG
  Register scratch = scratch0();
  Register function = ToRegister(instr->function());
  Register result = ToRegister(instr->result());

  // Check that the function really is a function. Load map into the
  // result register.
  __ CompareObjectType(function, result, scratch, JS_FUNCTION_TYPE, eq); // DIFF: codegen
  DeoptimizeIf(f, instr->environment()); // DIFF: codegen

  // Make sure that the function has an instance prototype.
  Label non_instance;
  __ ldrb(scratch, FieldMemOperand(result, Map::kBitFieldOffset));
  __ tst(scratch, Operand(1 << Map::kHasNonInstancePrototype));
  __ b(ne, &non_instance);

  // Get the prototype or initial map from the function.
  __ ldr(result,
         FieldMemOperand(function, JSFunction::kPrototypeOrInitialMapOffset));

  // Check that the function has a prototype or an initial map.
  __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
  __ cmp(result, ip);
  DeoptimizeIf(eq, instr->environment());

  // If the function does not have an initial map, we're done.
  Label done;
  __ CompareObjectType(result, scratch, scratch, MAP_TYPE, eq); // DIFF: codegen
  __ b(f, &done); // DIFF: codegen

  // Get the prototype from the initial map.
  __ ldr(result, FieldMemOperand(result, Map::kPrototypeOffset));
  __ jmp(&done);

  // Non-instance prototype: Fetch prototype from constructor field
  // in initial map.
  __ bind(&non_instance);
  __ ldr(result, FieldMemOperand(result, Map::kConstructorOffset));

  // All done.
  __ bind(&done);
}


void LCodeGen::DoLoadRoot(LLoadRoot* instr) { // REVIEWEDBY: CG
  Register result = ToRegister(instr->result());
  __ LoadRoot(result, instr->index());
}


void LCodeGen::DoAccessArgumentsAt(LAccessArgumentsAt* instr) { // REVIEWEDBY: CG
  Register arguments = ToRegister(instr->arguments());
  Register result = ToRegister(instr->result());
  // There are two words between the frame pointer and the last argument.
  // Subtracting from length accounts for one of them add one more.
  if (instr->length()->IsConstantOperand()) {
    int const_length = ToInteger32(LConstantOperand::cast(instr->length()));
    if (instr->index()->IsConstantOperand()) {
      int const_index = ToInteger32(LConstantOperand::cast(instr->index()));
      int index = (const_length - const_index) + 1;
      __ ldr(result, MemOperand(arguments, index * kPointerSize));
    } else {
      Register index = ToRegister(instr->index());
      __ rsb(result, index, Operand(const_length + 1));
      __ lsl(result, result, Operand(kPointerSizeLog2)); // DIFF: codegen
      __ add(result, arguments, result); // DIFF: codegen
      __ ldr(result, MemOperand(result)); // DIFF: codegen
    }
  } else if (instr->index()->IsConstantOperand()) {
      Register length = ToRegister(instr->length());
      int const_index = ToInteger32(LConstantOperand::cast(instr->index()));
      int loc = const_index - 1;
      if (loc != 0) {
        __ sub(result, length, Operand(loc));
	__ lsl(result, result, Operand(kPointerSizeLog2)); // DIFF: codegen
	__ add(result, arguments, result); // DIFF: codegen
	__ ldr(result, MemOperand(result)); // DIFF: codegen
      } else {
	  __ lsl(length, length, Operand(kPointerSizeLog2)); // DIFF: codegen
	  __ add(result, arguments, length); // DIFF: codegen
	  __ ldr(result, MemOperand(result)); // DIFF: codegen
      }
    } else {
    Register length = ToRegister(instr->length());
    Register index = ToRegister(instr->index());
    __ sub(result, length, index);
    __ add(result, result, Operand(1));
    __ lsl(result, result, Operand(kPointerSizeLog2)); // DIFF: codegen
    __ add(result, arguments, result); // DIFF: codegen
    __ ldr(result, MemOperand(result)); // DIFF: codegen
  }
}


void LCodeGen::DoLoadKeyedExternalArray(LLoadKeyed* instr) { // REVIEWEDBY: CG
  Register external_pointer = ToRegister(instr->elements());
  Register key = no_reg;
  ElementsKind elements_kind = instr->elements_kind();
  bool key_is_constant = instr->key()->IsConstantOperand();
  int constant_key = 0;
  if (key_is_constant) {
    constant_key = ToInteger32(LConstantOperand::cast(instr->key()));
    if (constant_key & 0xF0000000) {
      Abort(kArrayIndexConstantValueTooBig);
    }
  } else {
    key = ToRegister(instr->key());
  }
  int element_size_shift = ElementsKindToShiftSize(elements_kind);
  int shift_size = (instr->hydrogen()->key()->representation().IsSmi())
      ? (element_size_shift - kSmiTagSize) : element_size_shift;
  int additional_offset = IsFixedTypedArrayElementsKind(elements_kind)
      ? FixedTypedArrayBase::kDataOffset - kHeapObjectTag
      : 0;


  if (elements_kind == EXTERNAL_FLOAT32_ELEMENTS ||
      elements_kind == FLOAT32_ELEMENTS ||
      elements_kind == EXTERNAL_FLOAT64_ELEMENTS ||
      elements_kind == FLOAT64_ELEMENTS) {
    int base_offset =
      (instr->additional_index() << element_size_shift) + additional_offset;
    DwVfpRegister result = ToDoubleRegister(instr->result());
    if (key_is_constant) {
      __ add(scratch0(), external_pointer, Operand(constant_key << element_size_shift)); // DIFF: codegen
    } else {
      __ lsl(scratch0(), key, Operand(shift_size)); // DIFF: codegen
      __ add(scratch0(), external_pointer, scratch0()); // DIFF: codegen
    }
    if (elements_kind == EXTERNAL_FLOAT32_ELEMENTS ||
        elements_kind == FLOAT32_ELEMENTS) {
      __ vldr(double_scratch0().low(), scratch0(), base_offset);
      __ vcvt_f64_f32(result, double_scratch0().low());
    } else  {  // loading doubles, not floats.
      __ vldr(result, scratch0(), base_offset);
    }
  } else {
    Register result = ToRegister(instr->result());
    MemOperand mem_operand = PrepareKeyedOperand(
        key, external_pointer, key_is_constant, constant_key,
        element_size_shift, shift_size,
        instr->additional_index(), additional_offset);
    switch (elements_kind) {
      case EXTERNAL_INT8_ELEMENTS:
      case INT8_ELEMENTS:
        __ ldrsb(result, mem_operand);
        break;
      case EXTERNAL_UINT8_CLAMPED_ELEMENTS:
      case EXTERNAL_UINT8_ELEMENTS:
      case UINT8_ELEMENTS:
      case UINT8_CLAMPED_ELEMENTS:
        __ ldrb(result, mem_operand);
        break;
      case EXTERNAL_INT16_ELEMENTS:
      case INT16_ELEMENTS:
        __ ldrsh(result, mem_operand);
        break;
      case EXTERNAL_UINT16_ELEMENTS:
      case UINT16_ELEMENTS:
        __ ldrh(result, mem_operand);
        break;
      case EXTERNAL_INT32_ELEMENTS:
      case INT32_ELEMENTS:
        __ ldr(result, mem_operand);
        break;
      case EXTERNAL_UINT32_ELEMENTS:
      case UINT32_ELEMENTS:
        __ ldr(result, mem_operand);
        if (!instr->hydrogen()->CheckFlag(HInstruction::kUint32)) {
          __ cmphs(result, Operand(0x80000000)); // DIFF: codegen
          DeoptimizeIf(eq, instr->environment()); // DIFF: codegen
        }
        break;
      case FLOAT32_ELEMENTS:
      case FLOAT64_ELEMENTS:
      case EXTERNAL_FLOAT32_ELEMENTS:
      case EXTERNAL_FLOAT64_ELEMENTS:
      case FAST_HOLEY_DOUBLE_ELEMENTS:
      case FAST_HOLEY_ELEMENTS:
      case FAST_HOLEY_SMI_ELEMENTS:
      case FAST_DOUBLE_ELEMENTS:
      case FAST_ELEMENTS:
      case FAST_SMI_ELEMENTS:
      case DICTIONARY_ELEMENTS:
      case SLOPPY_ARGUMENTS_ELEMENTS:
        UNREACHABLE();
        break;
    }
  }
}


void LCodeGen::DoLoadKeyedFixedDoubleArray(LLoadKeyed* instr) { // REVIEWEDBY: CG
  Register elements = ToRegister(instr->elements());
  bool key_is_constant = instr->key()->IsConstantOperand();
  Register key = no_reg;
  DwVfpRegister result = ToDoubleRegister(instr->result());
  Register scratch = scratch0();

  int element_size_shift = ElementsKindToShiftSize(FAST_DOUBLE_ELEMENTS);

  int base_offset =
      FixedDoubleArray::kHeaderSize - kHeapObjectTag +
      (instr->additional_index() << element_size_shift);
  if (key_is_constant) {
    int constant_key = ToInteger32(LConstantOperand::cast(instr->key()));
    if (constant_key & 0xF0000000) {
      Abort(kArrayIndexConstantValueTooBig);
    }
    base_offset += constant_key << element_size_shift;
  }
  __ add(scratch, elements, Operand(base_offset));

  if (!key_is_constant) {
    key = ToRegister(instr->key());
    int shift_size = (instr->hydrogen()->key()->representation().IsSmi())
        ? (element_size_shift - kSmiTagSize) : element_size_shift;
    __ lsl(ip, key, Operand(shift_size)); // DIFF: codegen
    __ add(scratch, scratch, ip); // DIFF: codegen
  }

  __ vldr(result, scratch, 0);

  if (instr->hydrogen()->RequiresHoleCheck()) {
    __ ldr(scratch, MemOperand(scratch, sizeof(kHoleNanLower32)));
    __ cmp(scratch, Operand(kHoleNanUpper32));
    DeoptimizeIf(eq, instr->environment());
  }
}


void LCodeGen::DoLoadKeyedFixedArray(LLoadKeyed* instr) { // REVIEWEDBY: CG
  Register elements = ToRegister(instr->elements());
  Register result = ToRegister(instr->result());
  Register scratch = scratch0();
  Register store_base = scratch;
  int offset = 0;

  if (instr->key()->IsConstantOperand()) {
    LConstantOperand* const_operand = LConstantOperand::cast(instr->key());
    offset = FixedArray::OffsetOfElementAt(ToInteger32(const_operand) +
                                           instr->additional_index());
    store_base = elements;
  } else {
    Register key = ToRegister(instr->key());
    // Even though the HLoadKeyed instruction forces the input
    // representation for the key to be an integer, the input gets replaced
    // during bound check elimination with the index argument to the bounds
    // check, which can be tagged, so that case must be handled here, too.
    if (instr->hydrogen()->key()->representation().IsSmi()) {
      __ GetPointerOffsetFromSmiKey(scratch, key); // DIFF: codegen
      __ add(scratch, elements, scratch); // DIFF: codegen
    } else {
      __ lsl(scratch, key, Operand(kPointerSizeLog2)); // DIFF: codegen
      __ add(scratch, elements, scratch); // DIFF: codegen
    }
    offset = FixedArray::OffsetOfElementAt(instr->additional_index());
  }
  __ ldr(result, FieldMemOperand(store_base, offset));

  // Check for the hole value.
  if (instr->hydrogen()->RequiresHoleCheck()) {
    if (IsFastSmiElementsKind(instr->hydrogen()->elements_kind())) {
      __ SmiTst(result);
      DeoptimizeIf(ne, instr->environment());
    } else {
      __ LoadRoot(scratch, Heap::kTheHoleValueRootIndex);
      __ cmpeq(result, scratch); // DIFF: codegen
      DeoptimizeIf(eq, instr->environment());
    }
  }
}


void LCodeGen::DoLoadKeyed(LLoadKeyed* instr) { // REVIEWEDBY: CG
  if (instr->is_typed_elements()) {
    DoLoadKeyedExternalArray(instr);
  } else if (instr->hydrogen()->representation().IsDouble()) {
    DoLoadKeyedFixedDoubleArray(instr);
  } else {
    DoLoadKeyedFixedArray(instr);
  }
}


MemOperand LCodeGen::PrepareKeyedOperand(Register key, // REVIEWEDBY: CG
                                         Register base,
                                         bool key_is_constant,
                                         int constant_key,
                                         int element_size,
                                         int shift_size,
                                         int additional_index,
                                         int additional_offset) {
  // SH4: for SH4 mem operands are limited to (base + offset) or (base + reg)
  int base_offset = (additional_index << element_size) + additional_offset;
  if (key_is_constant) {
    return MemOperand(base,
                      base_offset + (constant_key << element_size));
  }

  if (additional_offset != 0) {
    if (shift_size >= 0) {
      __ lsl(scratch0(), key, Operand(shift_size)); // DIFF: codegen
      __ add(scratch0(), scratch0(), Operand(base_offset)); // DIFF: codegen
    } else {
      ASSERT_EQ(-1, shift_size);
      // key can be negative, so using ASR here.
      __ asr(scratch0(), key, Operand(1)); // DIFF: codegen
      __ add(scratch0(), scratch0(), Operand(base_offset)); // DIFF: codegen
    }
    return MemOperand(base, scratch0());
  }

  if (additional_index != 0) {
    additional_index *= 1 << (element_size - shift_size);
    __ add(scratch0(), key, Operand(additional_index));
  }

  if (additional_index == 0) {
    if (shift_size >= 0) {
      __ lsl(scratch0(), key, Operand(shift_size)); // DIFF: codegen
      return MemOperand(base, scratch0()); // DIFF: codegen
    } else {
      ASSERT_EQ(-1, shift_size);
      __ lsr(scratch0(), key, Operand(1)); // DIFF: codegen
      return MemOperand(base, scratch0()); // DIFF: codegen
    }
  }

  if (shift_size >= 0) {
    __ lsl(scratch0(), scratch0(), Operand(shift_size)); // DIFF: codegen
    return MemOperand(base, scratch0()); // DIFF: codegen
  } else {
    ASSERT_EQ(-1, shift_size);
    __ lsr(scratch0(), scratch0(), Operand(1)); // DIFF: codegen
    return MemOperand(base, scratch0()); // DIFF: codegen
  }
}


void LCodeGen::DoLoadKeyedGeneric(LLoadKeyedGeneric* instr) { // REVIEWEDBY: CG
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->object()).is(r1));
  ASSERT(ToRegister(instr->key()).is(r0));

  Handle<Code> ic = isolate()->builtins()->KeyedLoadIC_Initialize();
  CallCode(ic, RelocInfo::CODE_TARGET, instr, NEVER_INLINE_TARGET_ADDRESS);
}


void LCodeGen::DoArgumentsElements(LArgumentsElements* instr) { // REVIEWEDBY: CG
  Register scratch = scratch0();
  Register result = ToRegister(instr->result());

  if (instr->hydrogen()->from_inlined()) {
    __ sub(result, sp, Operand(2 * kPointerSize));
  } else {
    // Check if the calling frame is an arguments adaptor frame.
    Label done, adapted;
    __ ldr(scratch, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
    __ ldr(result, MemOperand(scratch, StandardFrameConstants::kContextOffset));
    __ cmp(result, Operand(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));

    // Result is the frame pointer for the frame if not adapted and for the real
    // frame below the adaptor frame if adapted.
    __ mov(result, fp, ne); // DIFF: codegen
    __ mov(result, scratch, eq); // DIFF: codegen
  }
}


void LCodeGen::DoArgumentsLength(LArgumentsLength* instr) { // REVIEWEDBY: CG
  Register elem = ToRegister(instr->elements());
  Register result = ToRegister(instr->result());

  Label done;

  // If no arguments adaptor frame the number of arguments is fixed.
  __ cmp(fp, elem);
  __ mov(result, Operand(scope()->num_parameters()));
  __ b(eq, &done);

  // Arguments adaptor frame present. Get argument length from there.
  __ ldr(result, MemOperand(fp, StandardFrameConstants::kCallerFPOffset));
  __ ldr(result,
         MemOperand(result, ArgumentsAdaptorFrameConstants::kLengthOffset));
  __ SmiUntag(result);

  // Argument length is in result register.
  __ bind(&done);
}


void LCodeGen::DoWrapReceiver(LWrapReceiver* instr) { // REVIEWEDBY: CG
  Register receiver = ToRegister(instr->receiver());
  Register function = ToRegister(instr->function());
  Register result = ToRegister(instr->result());
  Register scratch = scratch0();

  // If the receiver is null or undefined, we have to pass the global
  // object as a receiver to normal functions. Values have to be
  // passed unchanged to builtins and strict-mode functions.
  Label global_object, result_in_receiver;

  if (!instr->hydrogen()->known_function()) {
    // Do not transform the receiver to object for strict mode
    // functions.
    __ ldr(scratch,
           FieldMemOperand(function, JSFunction::kSharedFunctionInfoOffset));
    __ ldr(scratch,
           FieldMemOperand(scratch, SharedFunctionInfo::kCompilerHintsOffset));
    int mask = 1 << (SharedFunctionInfo::kStrictModeFunction + kSmiTagSize);
    __ tst(scratch, Operand(mask));
    __ b(ne, &result_in_receiver);

    // Do not transform the receiver to object for builtins.
    __ tst(scratch, Operand(1 << (SharedFunctionInfo::kNative + kSmiTagSize)));
    __ b(ne, &result_in_receiver);
  }

  // Normal function. Replace undefined or null with global receiver.
  __ LoadRoot(scratch, Heap::kNullValueRootIndex);
  __ cmp(receiver, scratch);
  __ b(eq, &global_object);
  __ LoadRoot(scratch, Heap::kUndefinedValueRootIndex);
  __ cmp(receiver, scratch);
  __ b(eq, &global_object);

  // Deoptimize if the receiver is not a JS object.
  __ SmiTst(receiver);
  DeoptimizeIf(eq, instr->environment());
  __ CompareObjectType(receiver, scratch, scratch, FIRST_SPEC_OBJECT_TYPE, ge); // DIFF: codegen
  DeoptimizeIf(f, instr->environment()); // DIFF: codegen

  __ b(&result_in_receiver);
  __ bind(&global_object);
  __ ldr(result, FieldMemOperand(function, JSFunction::kContextOffset));
  __ ldr(result,
         ContextOperand(result, Context::GLOBAL_OBJECT_INDEX));
  __ ldr(result,
         FieldMemOperand(result, GlobalObject::kGlobalReceiverOffset));

  if (result.is(receiver)) {
    __ bind(&result_in_receiver);
  } else {
    Label result_ok;
    __ b(&result_ok);
    __ bind(&result_in_receiver);
    __ mov(result, receiver);
    __ bind(&result_ok);
  }
}


void LCodeGen::DoApplyArguments(LApplyArguments* instr) { // REVIEWEDBY: CG
  Register receiver = ToRegister(instr->receiver());
  Register function = ToRegister(instr->function());
  Register length = ToRegister(instr->length());
  Register elements = ToRegister(instr->elements());
  Register scratch = scratch0();
  ASSERT(receiver.is(r0));  // Used for parameter count.
  ASSERT(function.is(r1));  // Required by InvokeFunction.
  ASSERT(ToRegister(instr->result()).is(r0));

  // Copy the arguments to this function possibly from the
  // adaptor frame below it.
  const uint32_t kArgumentsLimit = 1 * KB;
  __ cmphi(length, Operand(kArgumentsLimit)); // DIFF: codegen
  DeoptimizeIf(t, instr->environment()); // DIFF: codegen

  // Push the receiver and use the register to keep the original
  // number of arguments.
  __ push(receiver);
  __ mov(receiver, length);
  // The arguments are at a one pointer size offset from elements.
  __ add(elements, elements, Operand(1 * kPointerSize));

  // Loop through the arguments pushing them onto the execution
  // stack.
  Label invoke, loop;
  // length is a small non-negative integer, due to the test above.
  __ cmp(length, Operand::Zero());
  __ b(eq, &invoke);
  __ bind(&loop);
  __ lsl(scratch, length, Operand(2)); // DIFF: codegen
  __ ldr(scratch, MemOperand(elements, scratch)); // DIFF: codegen
  __ push(scratch);
  __ sub(length, length, Operand(1)); // DIFF: codegen
  __ cmp(length, Operand(0)); // DIFF: codegen
  __ b(ne, &loop);

  __ bind(&invoke);
  ASSERT(instr->HasPointerMap());
  LPointerMap* pointers = instr->pointer_map();
  SafepointGenerator safepoint_generator(
      this, pointers, Safepoint::kLazyDeopt);
  // The number of arguments is stored in receiver which is r0, as expected
  // by InvokeFunction.
  ParameterCount actual(receiver);
  __ InvokeFunction(function, actual, CALL_FUNCTION, safepoint_generator);
}


void LCodeGen::DoPushArgument(LPushArgument* instr) { // REVIEWEDBY: CG
  LOperand* argument = instr->value();
  if (argument->IsDoubleRegister() || argument->IsDoubleStackSlot()) {
    Abort(kDoPushArgumentNotImplementedForDoubleType);
  } else {
    Register argument_reg = EmitLoadRegister(argument, ip);
    __ push(argument_reg);
  }
}


void LCodeGen::DoDrop(LDrop* instr) { // REVIEWEDBY: CG
  __ Drop(instr->count());
}


void LCodeGen::DoThisFunction(LThisFunction* instr) { // REVIEWEDBY: CG
  Register result = ToRegister(instr->result());
  __ ldr(result, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
}


void LCodeGen::DoContext(LContext* instr) { // REVIEWEDBY: CG
  // If there is a non-return use, the context must be moved to a register.
  Register result = ToRegister(instr->result());
  if (info()->IsOptimizing()) {
    __ ldr(result, MemOperand(fp, StandardFrameConstants::kContextOffset));
  } else {
    // If there is no frame, the context must be in cp.
    ASSERT(result.is(cp));
  }
}


void LCodeGen::DoDeclareGlobals(LDeclareGlobals* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::CallKnownFunction(Handle<JSFunction> function, // REVIEWEDBY: CG
                                 int formal_parameter_count,
                                 int arity,
                                 LInstruction* instr,
                                 R1State r1_state) {
  bool dont_adapt_arguments =
      formal_parameter_count == SharedFunctionInfo::kDontAdaptArgumentsSentinel;
  bool can_invoke_directly =
      dont_adapt_arguments || formal_parameter_count == arity;

  LPointerMap* pointers = instr->pointer_map();

  if (can_invoke_directly) {
    if (r1_state == R1_UNINITIALIZED) {
      __ Move(r1, function);
    }

    // Change context.
    __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));

    // Set r0 to arguments count if adaption is not needed. Assumes that r0
    // is available to write to at this point.
    if (dont_adapt_arguments) {
      __ mov(r0, Operand(arity));
    }

    // Invoke function.
    __ ldr(ip, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
    __ Call(ip);

    // Set up deoptimization.
    RecordSafepointWithLazyDeopt(instr, RECORD_SIMPLE_SAFEPOINT);
  } else {
    SafepointGenerator generator(this, pointers, Safepoint::kLazyDeopt);
    ParameterCount count(arity);
    ParameterCount expected(formal_parameter_count);
    __ InvokeFunction(function, expected, count, CALL_FUNCTION, generator);
  }
}


void LCodeGen::DoDeferredMathAbsTaggedHeapNumber(LMathAbs* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::EmitIntegerMathAbs(LMathAbs* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathAbs(LMathAbs* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathFloor(LMathFloor* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathRound(LMathRound* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathSqrt(LMathSqrt* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathPowHalf(LMathPowHalf* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoPower(LPower* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathExp(LMathExp* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathLog(LMathLog* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathClz32(LMathClz32* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoInvokeFunction(LInvokeFunction* instr) { // REVIEWEDBY: CG
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->function()).is(r1));
  ASSERT(instr->HasPointerMap());

  Handle<JSFunction> known_function = instr->hydrogen()->known_function();
  if (known_function.is_null()) {
    LPointerMap* pointers = instr->pointer_map();
    SafepointGenerator generator(this, pointers, Safepoint::kLazyDeopt);
    ParameterCount count(instr->arity());
    __ InvokeFunction(r1, count, CALL_FUNCTION, generator);
  } else {
    CallKnownFunction(known_function,
                      instr->hydrogen()->formal_parameter_count(),
                      instr->arity(),
                      instr,
                      R1_CONTAINS_TARGET);
  }
}


void LCodeGen::DoCallWithDescriptor(LCallWithDescriptor* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallJSFunction(LCallJSFunction* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallFunction(LCallFunction* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallNew(LCallNew* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallNewArray(LCallNewArray* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallRuntime(LCallRuntime* instr) { // REVIEWEDBY: CG
  CallRuntime(instr->function(), instr->arity(), instr);
}


void LCodeGen::DoStoreCodeEntry(LStoreCodeEntry* instr) { // REVIEWEDBY: CG
  Register function = ToRegister(instr->function());
  Register code_object = ToRegister(instr->code_object());
  __ add(code_object, code_object, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ str(code_object,
         FieldMemOperand(function, JSFunction::kCodeEntryOffset));
}


void LCodeGen::DoInnerAllocatedObject(LInnerAllocatedObject* instr) { // REVIEWEDBY: CG
  Register result = ToRegister(instr->result());
  Register base = ToRegister(instr->base_object());
  if (instr->offset()->IsConstantOperand()) {
    LConstantOperand* offset = LConstantOperand::cast(instr->offset());
    __ add(result, base, Operand(ToInteger32(offset)));
  } else {
    Register offset = ToRegister(instr->offset());
    __ add(result, base, offset);
  }
}


void LCodeGen::DoStoreNamedField(LStoreNamedField* instr) { // REVIEWEDBY: CG
  Representation representation = instr->representation();

  Register object = ToRegister(instr->object());
  Register scratch = scratch0();
  HObjectAccess access = instr->hydrogen()->access();
  int offset = access.offset();

  if (access.IsExternalMemory()) {
    Register value = ToRegister(instr->value());
    MemOperand operand = MemOperand(object, offset);
    __ Store(value, operand, representation);
    return;
  }

  SmiCheck check_needed =
      instr->hydrogen()->value()->IsHeapObject()
          ? OMIT_SMI_CHECK : INLINE_SMI_CHECK;

  ASSERT(!(representation.IsSmi() &&
           instr->value()->IsConstantOperand() &&
           !IsSmi(LConstantOperand::cast(instr->value()))));
  if (representation.IsHeapObject()) {
    Register value = ToRegister(instr->value());
    if (!instr->hydrogen()->value()->type().IsHeapObject()) {
      __ SmiTst(value);
      DeoptimizeIf(eq, instr->environment());

      // We know now that value is not a smi, so we can omit the check below.
      check_needed = OMIT_SMI_CHECK;
    }
  } else if (representation.IsDouble()) {
    ASSERT(access.IsInobject());
    ASSERT(!instr->hydrogen()->has_transition());
    ASSERT(!instr->hydrogen()->NeedsWriteBarrier());
    DwVfpRegister value = ToDoubleRegister(instr->value());
    __ vstr(value, FieldMemOperand(object, offset));
    return;
  }

  if (instr->hydrogen()->has_transition()) {
    Handle<Map> transition = instr->hydrogen()->transition_map();
    AddDeprecationDependency(transition);
    __ mov(scratch, Operand(transition));
    __ str(scratch, FieldMemOperand(object, HeapObject::kMapOffset));
    if (instr->hydrogen()->NeedsWriteBarrierForMap()) {
      Register temp = ToRegister(instr->temp());
      // Update the write barrier for the map field.
      __ RecordWriteField(object,
                          HeapObject::kMapOffset,
                          scratch,
                          temp,
                          GetLinkRegisterState(),
                          kSaveFPRegs,
                          OMIT_REMEMBERED_SET,
                          OMIT_SMI_CHECK);
    }
  }

  // Do the store.
  Register value = ToRegister(instr->value());
  if (access.IsInobject()) {
    MemOperand operand = FieldMemOperand(object, offset);
    __ Store(value, operand, representation);
    if (instr->hydrogen()->NeedsWriteBarrier()) {
      // Update the write barrier for the object for in-object properties.
      __ RecordWriteField(object,
                          offset,
                          value,
                          scratch,
                          GetLinkRegisterState(),
                          kSaveFPRegs,
                          EMIT_REMEMBERED_SET,
                          check_needed);
    }
  } else {
    __ ldr(scratch, FieldMemOperand(object, JSObject::kPropertiesOffset));
    MemOperand operand = FieldMemOperand(scratch, offset);
    __ Store(value, operand, representation);
    if (instr->hydrogen()->NeedsWriteBarrier()) {
      // Update the write barrier for the properties array.
      // object is used as a scratch register.
      __ RecordWriteField(scratch,
                          offset,
                          value,
                          object,
                          GetLinkRegisterState(),
                          kSaveFPRegs,
                          EMIT_REMEMBERED_SET,
                          check_needed);
    }
  }
}


  void LCodeGen::DoStoreNamedGeneric(LStoreNamedGeneric* instr) { // REVIEWEDBY: CG
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->object()).is(r1));
  ASSERT(ToRegister(instr->value()).is(r0));

  // Name is always in r2.
  __ mov(r2, Operand(instr->name()));
  Handle<Code> ic = StoreIC::initialize_stub(isolate(), instr->strict_mode());
  CallCode(ic, RelocInfo::CODE_TARGET, instr, NEVER_INLINE_TARGET_ADDRESS);
}


void LCodeGen::DoBoundsCheck(LBoundsCheck* instr) { // REVIEWEDBY: CG
  Condition cc = instr->hydrogen()->allow_equality() ? hi : hs;
  if (instr->index()->IsConstantOperand()) {
    Operand index = ToOperand(instr->index());
    Register length = ToRegister(instr->length());
    __ cmp(&cc, length, index); // DIFF: codegen
    cc = ReverseCondition(cc);
  } else {
    Register index = ToRegister(instr->index());
    Operand length = ToOperand(instr->length());
    __ cmp(&cc, index, length); // DIFF: codegen
  }
  if (FLAG_debug_code && instr->hydrogen()->skip_check()) {
    Label done;
    __ b(NegateCondition(cc), &done);
    __ stop("eliminated bounds check failed");
    __ bind(&done);
  } else {
    DeoptimizeIf(cc, instr->environment());
  }
}


void LCodeGen::DoStoreKeyedExternalArray(LStoreKeyed* instr) { // REVIEWEDBY: CG
  Register external_pointer = ToRegister(instr->elements());
  Register key = no_reg;
  ElementsKind elements_kind = instr->elements_kind();
  bool key_is_constant = instr->key()->IsConstantOperand();
  int constant_key = 0;
  if (key_is_constant) {
    constant_key = ToInteger32(LConstantOperand::cast(instr->key()));
    if (constant_key & 0xF0000000) {
      Abort(kArrayIndexConstantValueTooBig);
    }
  } else {
    key = ToRegister(instr->key());
  }
  int element_size_shift = ElementsKindToShiftSize(elements_kind);
  int shift_size = (instr->hydrogen()->key()->representation().IsSmi())
      ? (element_size_shift - kSmiTagSize) : element_size_shift;
  int additional_offset = IsFixedTypedArrayElementsKind(elements_kind)
      ? FixedTypedArrayBase::kDataOffset - kHeapObjectTag
      : 0;

  if (elements_kind == EXTERNAL_FLOAT32_ELEMENTS ||
      elements_kind == FLOAT32_ELEMENTS ||
      elements_kind == EXTERNAL_FLOAT64_ELEMENTS ||
      elements_kind == FLOAT64_ELEMENTS) {
    int base_offset =
      (instr->additional_index() << element_size_shift) + additional_offset;
    Register address = scratch0();
    DwVfpRegister value(ToDoubleRegister(instr->value()));
    if (key_is_constant) {
      if (constant_key != 0) {
        __ add(address, external_pointer,
               Operand(constant_key << element_size_shift));
      } else {
        address = external_pointer;
      }
    } else {
      __ lsl(address, key, Operand(shift_size)); // DIFF: codegen
      __ add(address, external_pointer, address);
    }
    if (elements_kind == EXTERNAL_FLOAT32_ELEMENTS ||
        elements_kind == FLOAT32_ELEMENTS) {
      __ vcvt_f32_f64(double_scratch0().low(), value);
      __ vstr(double_scratch0().low(), address, base_offset);
    } else {  // Storing doubles, not floats.
      __ vstr(value, address, base_offset);
    }
  } else {
    Register value(ToRegister(instr->value()));
    MemOperand mem_operand = PrepareKeyedOperand(
        key, external_pointer, key_is_constant, constant_key,
        element_size_shift, shift_size,
        instr->additional_index(), additional_offset);
    switch (elements_kind) {
      case EXTERNAL_UINT8_CLAMPED_ELEMENTS:
      case EXTERNAL_INT8_ELEMENTS:
      case EXTERNAL_UINT8_ELEMENTS:
      case UINT8_ELEMENTS:
      case UINT8_CLAMPED_ELEMENTS:
      case INT8_ELEMENTS:
        __ strb(value, mem_operand);
        break;
      case EXTERNAL_INT16_ELEMENTS:
      case EXTERNAL_UINT16_ELEMENTS:
      case INT16_ELEMENTS:
      case UINT16_ELEMENTS:
        __ strh(value, mem_operand);
        break;
      case EXTERNAL_INT32_ELEMENTS:
      case EXTERNAL_UINT32_ELEMENTS:
      case INT32_ELEMENTS:
      case UINT32_ELEMENTS:
        __ str(value, mem_operand);
        break;
      case FLOAT32_ELEMENTS:
      case FLOAT64_ELEMENTS:
      case EXTERNAL_FLOAT32_ELEMENTS:
      case EXTERNAL_FLOAT64_ELEMENTS:
      case FAST_DOUBLE_ELEMENTS:
      case FAST_ELEMENTS:
      case FAST_SMI_ELEMENTS:
      case FAST_HOLEY_DOUBLE_ELEMENTS:
      case FAST_HOLEY_ELEMENTS:
      case FAST_HOLEY_SMI_ELEMENTS:
      case DICTIONARY_ELEMENTS:
      case SLOPPY_ARGUMENTS_ELEMENTS:
        UNREACHABLE();
        break;
    }
  }
}


void LCodeGen::DoStoreKeyedFixedDoubleArray(LStoreKeyed* instr) { // REVIEWEDBY: CG
  DwVfpRegister value = ToDoubleRegister(instr->value());
  Register elements = ToRegister(instr->elements());
  Register scratch = scratch0();
  DwVfpRegister double_scratch = double_scratch0();
  bool key_is_constant = instr->key()->IsConstantOperand();

  // Calculate the effective address of the slot in the array to store the
  // double value.
  int element_size_shift = ElementsKindToShiftSize(FAST_DOUBLE_ELEMENTS);
  if (key_is_constant) {
    int constant_key = ToInteger32(LConstantOperand::cast(instr->key()));
    if (constant_key & 0xF0000000) {
      Abort(kArrayIndexConstantValueTooBig);
    }
    __ add(scratch, elements,
           Operand((constant_key << element_size_shift) +
                   FixedDoubleArray::kHeaderSize - kHeapObjectTag));
  } else {
    int shift_size = (instr->hydrogen()->key()->representation().IsSmi())
        ? (element_size_shift - kSmiTagSize) : element_size_shift;
    __ lsl(scratch, ToRegister(instr->key()), Operand(shift_size)); // DIFF: codegen
    __ add(scratch, scratch, Operand(FixedDoubleArray::kHeaderSize - kHeapObjectTag)); // DIFF: codegen
    __ add(scratch, elements, scratch); // DIFF: codegen
  }

  if (instr->NeedsCanonicalization()) {
    // Force a canonical NaN.
    // SH4: no need for NaN mode check
    // if (masm()->emit_debug_code()) {
    //   __ vmrs(ip);
    //   __ tst(ip, Operand(kVFPDefaultNaNModeControlBit));
    //   __ Assert(ne, kDefaultNaNModeNotSet);
    // }
    __ VFPCanonicalizeNaN(double_scratch, value);
    __ vstr(double_scratch, scratch,
            instr->additional_index() << element_size_shift);
  } else {
    __ vstr(value, scratch, instr->additional_index() << element_size_shift);
  }
}


void LCodeGen::DoStoreKeyedFixedArray(LStoreKeyed* instr) { // REVIEWEDBY: CG
  Register value = ToRegister(instr->value());
  Register elements = ToRegister(instr->elements());
  Register key = instr->key()->IsRegister() ? ToRegister(instr->key())
      : no_reg;
  Register scratch = scratch0();
  Register store_base = scratch;
  int offset = 0;

  // Do the store.
  if (instr->key()->IsConstantOperand()) {
    ASSERT(!instr->hydrogen()->NeedsWriteBarrier());
    LConstantOperand* const_operand = LConstantOperand::cast(instr->key());
    offset = FixedArray::OffsetOfElementAt(ToInteger32(const_operand) +
                                           instr->additional_index());
    store_base = elements;
  } else {
    // Even though the HLoadKeyed instruction forces the input
    // representation for the key to be an integer, the input gets replaced
    // during bound check elimination with the index argument to the bounds
    // check, which can be tagged, so that case must be handled here, too.
    if (instr->hydrogen()->key()->representation().IsSmi()) {
      __ GetPointerOffsetFromSmiKey(scratch, key); // DIFF: codegen
      __ add(scratch, elements, scratch); // DIFF: codegen
    } else {
      __ lsl(scratch, key, Operand(kPointerSizeLog2)); // DIFF: codegen
      __ add(scratch, elements, scratch); // DIFF: codegen
    }
    offset = FixedArray::OffsetOfElementAt(instr->additional_index());
  }
  __ str(value, FieldMemOperand(store_base, offset));

  if (instr->hydrogen()->NeedsWriteBarrier()) {
    SmiCheck check_needed =
        instr->hydrogen()->value()->IsHeapObject()
            ? OMIT_SMI_CHECK : INLINE_SMI_CHECK;
    // Compute address of modified element and store it into key register.
    __ add(key, store_base, Operand(offset - kHeapObjectTag));
    __ RecordWrite(elements,
                   key,
                   value,
                   GetLinkRegisterState(),
                   kSaveFPRegs,
                   EMIT_REMEMBERED_SET,
                   check_needed);
  }
}


void LCodeGen::DoStoreKeyed(LStoreKeyed* instr) { // REVIEWEDBY: CG
  // By cases: external, fast double
  if (instr->is_typed_elements()) {
    DoStoreKeyedExternalArray(instr);
  } else if (instr->hydrogen()->value()->representation().IsDouble()) {
    DoStoreKeyedFixedDoubleArray(instr);
  } else {
    DoStoreKeyedFixedArray(instr);
  }
}


void LCodeGen::DoStoreKeyedGeneric(LStoreKeyedGeneric* instr) { // REVIEWEDBY: CG
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->object()).is(r2));
  ASSERT(ToRegister(instr->key()).is(r1));
  ASSERT(ToRegister(instr->value()).is(r0));

  Handle<Code> ic = instr->strict_mode() == STRICT
      ? isolate()->builtins()->KeyedStoreIC_Initialize_Strict()
      : isolate()->builtins()->KeyedStoreIC_Initialize();
  CallCode(ic, RelocInfo::CODE_TARGET, instr, NEVER_INLINE_TARGET_ADDRESS);
}


void LCodeGen::DoTransitionElementsKind(LTransitionElementsKind* instr) { // REVIEWEDBY: CG
  Register object_reg = ToRegister(instr->object());
  Register scratch = scratch0();

  Handle<Map> from_map = instr->original_map();
  Handle<Map> to_map = instr->transitioned_map();
  ElementsKind from_kind = instr->from_kind();
  ElementsKind to_kind = instr->to_kind();

  Label not_applicable;
  __ ldr(scratch, FieldMemOperand(object_reg, HeapObject::kMapOffset));
  __ cmp(scratch, Operand(from_map));
  __ b(ne, &not_applicable);

  if (IsSimpleMapChangeTransition(from_kind, to_kind)) {
    Register new_map_reg = ToRegister(instr->new_map_temp());
    __ mov(new_map_reg, Operand(to_map));
    __ str(new_map_reg, FieldMemOperand(object_reg, HeapObject::kMapOffset));
    // Write barrier.
    __ RecordWriteField(object_reg, HeapObject::kMapOffset, new_map_reg,
                        scratch, GetLinkRegisterState(), kDontSaveFPRegs);
  } else {
    ASSERT(ToRegister(instr->context()).is(cp));
    ASSERT(object_reg.is(r0));
    PushSafepointRegistersScope scope(
        this, Safepoint::kWithRegistersAndDoubles);
    __ Move(r1, to_map);
    bool is_js_array = from_map->instance_type() == JS_ARRAY_TYPE;
    TransitionElementsKindStub stub(isolate(), from_kind, to_kind, is_js_array);
    __ CallStub(&stub);
    RecordSafepointWithRegistersAndDoubles(
        instr->pointer_map(), 0, Safepoint::kLazyDeopt);
  }
  __ bind(&not_applicable);
}


void LCodeGen::DoTrapAllocationMemento(LTrapAllocationMemento* instr) { // REVIEWEDBY: CG
  Register object = ToRegister(instr->object());
  Register temp = ToRegister(instr->temp());
  Label no_memento_found;
  __ TestJSArrayForAllocationMemento(object, temp, &no_memento_found);
  DeoptimizeIf(eq, instr->environment());
  __ bind(&no_memento_found);
}


void LCodeGen::DoStringAdd(LStringAdd* instr) { // REVIEWEDBY: CG
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->left()).is(r1));
  ASSERT(ToRegister(instr->right()).is(r0));
  StringAddStub stub(isolate(),
                     instr->hydrogen()->flags(),
                     instr->hydrogen()->pretenure_flag());
  CallCode(stub.GetCode(), RelocInfo::CODE_TARGET, instr);
}


void LCodeGen::DoStringCharCodeAt(LStringCharCodeAt* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredStringCharCodeAt(LStringCharCodeAt* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStringCharFromCode(LStringCharFromCode* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredStringCharFromCode(LStringCharFromCode* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoInteger32ToDouble(LInteger32ToDouble* instr) { // REVIEWEDBY: CG
  LOperand* input = instr->value();
  ASSERT(input->IsRegister() || input->IsStackSlot());
  LOperand* output = instr->result();
  ASSERT(output->IsDoubleRegister());
  Register input_reg; // DIFF: codegen
  if (input->IsStackSlot()) {
    Register scratch = scratch0();
    __ ldr(scratch, ToMemOperand(input));
    input_reg = scratch; // DIFF: codegen
  } else {
    input_reg = ToRegister(input); // DIFF: codegen
  }
  __ vcvt_f64_s32(ToDoubleRegister(output), input_reg); // DIFF: codegen
}


void LCodeGen::DoUint32ToDouble(LUint32ToDouble* instr) { // REVIEWEDBY: CG
  LOperand* input = instr->value();
  LOperand* output = instr->result();

  __ vcvt_f64_u32(ToDoubleRegister(output), ToRegister(input)); // DIFF: codegen
}


void LCodeGen::DoNumberTagI(LNumberTagI* instr) { // REVIEWEDBY: CG
  class DeferredNumberTagI V8_FINAL : public LDeferredCode {
   public:
    DeferredNumberTagI(LCodeGen* codegen, LNumberTagI* instr)
        : LDeferredCode(codegen), instr_(instr) { }
    virtual void Generate() V8_OVERRIDE {
      codegen()->DoDeferredNumberTagIU(instr_,
                                       instr_->value(),
                                       instr_->temp1(),
                                       instr_->temp2(),
                                       SIGNED_INT32);
    }
    virtual LInstruction* instr() V8_OVERRIDE { return instr_; }
   private:
    LNumberTagI* instr_;
  };

  Register src = ToRegister(instr->value());
  Register dst = ToRegister(instr->result());

  DeferredNumberTagI* deferred = new(zone()) DeferredNumberTagI(this, instr);
  __ SmiTag(dst, src, SetT); // DIFF: codegen
  __ b(t, deferred->entry()); // DIFF: codegen
  __ bind(deferred->exit());
}


void LCodeGen::DoNumberTagU(LNumberTagU* instr) { // REVIEWEDBY: CG
  class DeferredNumberTagU V8_FINAL : public LDeferredCode {
   public:
    DeferredNumberTagU(LCodeGen* codegen, LNumberTagU* instr)
        : LDeferredCode(codegen), instr_(instr) { }
    virtual void Generate() V8_OVERRIDE {
      codegen()->DoDeferredNumberTagIU(instr_,
                                       instr_->value(),
                                       instr_->temp1(),
                                       instr_->temp2(),
                                       UNSIGNED_INT32);
    }
    virtual LInstruction* instr() V8_OVERRIDE { return instr_; }
   private:
    LNumberTagU* instr_;
  };

  Register input = ToRegister(instr->value());
  Register result = ToRegister(instr->result());

  DeferredNumberTagU* deferred = new(zone()) DeferredNumberTagU(this, instr);
  __ cmphi(input, Operand(Smi::kMaxValue)); // DIFF: codegen
  __ b(t, deferred->entry()); // DIFF: codegen
  __ SmiTag(result, input);
  __ bind(deferred->exit());
}


void LCodeGen::DoDeferredNumberTagIU(LInstruction* instr, // REVIEWEDBY: CG
                                     LOperand* value,
                                     LOperand* temp1,
                                     LOperand* temp2,
                                     IntegerSignedness signedness) {
  Label done, slow;
  Register src = ToRegister(value);
  Register dst = ToRegister(instr->result());
  Register tmp1 = scratch0();
  Register tmp2 = ToRegister(temp1);
  Register tmp3 = ToRegister(temp2);
  DwVfpRegister dbl_scratch = double_scratch0();

  if (signedness == SIGNED_INT32) {
    // There was overflow, so bits 30 and 31 of the original integer
    // disagree. Try to allocate a heap number in new space and store
    // the value in there. If that fails, call the runtime system.
    if (dst.is(src)) {
      __ SmiUntag(src, dst);
      __ eor(src, src, Operand(0x80000000));
    }
    __ vcvt_f64_s32(dbl_scratch, src); // DIFF: codegen
  } else {
    __ vcvt_f64_u32(dbl_scratch, src); // DIFF: codegen
  }

  if (FLAG_inline_new) {
    __ LoadRoot(tmp3, Heap::kHeapNumberMapRootIndex);
    __ AllocateHeapNumber(dst, tmp1, tmp2, tmp3, &slow, DONT_TAG_RESULT);
    __ b(&done);
  }

  // Slow case: Call the runtime system to do the number allocation.
  __ bind(&slow);
  {
    // TODO(3095996): Put a valid pointer value in the stack slot where the
    // result register is stored, as this register is in the pointer map, but
    // contains an integer value.
    __ mov(dst, Operand::Zero());

    // Preserve the value of all registers.
    PushSafepointRegistersScope scope(this, Safepoint::kWithRegisters);

    // NumberTagI and NumberTagD use the context from the frame, rather than
    // the environment's HContext or HInlinedContext value.
    // They only call Runtime::kHiddenAllocateHeapNumber.
    // The corresponding HChange instructions are added in a phase that does
    // not have easy access to the local context.
    __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
    __ CallRuntimeSaveDoubles(Runtime::kHiddenAllocateHeapNumber);
    RecordSafepointWithRegisters(
        instr->pointer_map(), 0, Safepoint::kNoLazyDeopt);
    __ sub(r0, r0, Operand(kHeapObjectTag));
    __ StoreToSafepointRegisterSlot(r0, dst);
  }

  // Done. Put the value in dbl_scratch into the value of the allocated heap
  // number.
  __ bind(&done);
  __ vstr(dbl_scratch, dst, HeapNumber::kValueOffset);
  __ add(dst, dst, Operand(kHeapObjectTag));
}


void LCodeGen::DoNumberTagD(LNumberTagD* instr) { // REVIEWEDBY: CG
  class DeferredNumberTagD V8_FINAL : public LDeferredCode {
   public:
    DeferredNumberTagD(LCodeGen* codegen, LNumberTagD* instr)
        : LDeferredCode(codegen), instr_(instr) { }
    virtual void Generate() V8_OVERRIDE {
      codegen()->DoDeferredNumberTagD(instr_);
    }
    virtual LInstruction* instr() V8_OVERRIDE { return instr_; }
   private:
    LNumberTagD* instr_;
  };

  DwVfpRegister input_reg = ToDoubleRegister(instr->value());
  Register scratch = scratch0();
  Register reg = ToRegister(instr->result());
  Register temp1 = ToRegister(instr->temp());
  Register temp2 = ToRegister(instr->temp2());

  DeferredNumberTagD* deferred = new(zone()) DeferredNumberTagD(this, instr);
  if (FLAG_inline_new) {
    __ LoadRoot(scratch, Heap::kHeapNumberMapRootIndex);
    // We want the untagged address first for performance
    __ AllocateHeapNumber(reg, temp1, temp2, scratch, deferred->entry(),
                          DONT_TAG_RESULT);
  } else {
    __ jmp(deferred->entry());
  }
  __ bind(deferred->exit());
  __ vstr(input_reg, reg, HeapNumber::kValueOffset);
  // Now that we have finished with the object's real address tag it
  __ add(reg, reg, Operand(kHeapObjectTag));
}


void LCodeGen::DoDeferredNumberTagD(LNumberTagD* instr) { // REVIEWEDBY: CG
  // TODO(3095996): Get rid of this. For now, we need to make the
  // result register contain a valid pointer because it is already
  // contained in the register pointer map.
  Register reg = ToRegister(instr->result());
  __ mov(reg, Operand::Zero());

  PushSafepointRegistersScope scope(this, Safepoint::kWithRegisters);
  // NumberTagI and NumberTagD use the context from the frame, rather than
  // the environment's HContext or HInlinedContext value.
  // They only call Runtime::kHiddenAllocateHeapNumber.
  // The corresponding HChange instructions are added in a phase that does
  // not have easy access to the local context.
  __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  __ CallRuntimeSaveDoubles(Runtime::kHiddenAllocateHeapNumber);
  RecordSafepointWithRegisters(
      instr->pointer_map(), 0, Safepoint::kNoLazyDeopt);
  __ sub(r0, r0, Operand(kHeapObjectTag));
  __ StoreToSafepointRegisterSlot(r0, reg);
}


void LCodeGen::DoSmiTag(LSmiTag* instr) { // REVIEWEDBY: CG
  HChange* hchange = instr->hydrogen();
  Register input = ToRegister(instr->value());
  Register output = ToRegister(instr->result());
  if (hchange->CheckFlag(HValue::kCanOverflow) &&
      hchange->value()->CheckFlag(HValue::kUint32)) {
    __ tst(input, Operand(0xc0000000));
    DeoptimizeIf(ne, instr->environment());
  }
  if (hchange->CheckFlag(HValue::kCanOverflow) &&
      !hchange->value()->CheckFlag(HValue::kUint32)) {
    __ SmiTag(output, input, SetT); // DIFF: codegen
    DeoptimizeIf(t, instr->environment());
  } else {
    __ SmiTag(output, input);
  }
}


void LCodeGen::DoSmiUntag(LSmiUntag* instr) { // REVIEWEDBY: CG
  Register input = ToRegister(instr->value());
  Register result = ToRegister(instr->result());
  if (instr->needs_check()) {
    STATIC_ASSERT(kHeapObjectTag == 1);
    // If the input is a HeapObject, SmiUntag will clear the T bit
    __ SmiUntag(result, input, SetT); // DIFF: codegen
    DeoptimizeIf(f, instr->environment()); // DIFF: codegen
  } else {
    __ SmiUntag(result, input);
  }
}


void LCodeGen::EmitNumberUntagD(Register input_reg, // REVIEWEDBY: CG
                                DwVfpRegister result_reg,
                                bool can_convert_undefined_to_nan,
                                bool deoptimize_on_minus_zero,
                                LEnvironment* env,
                                NumberUntagDMode mode) {
  Register scratch = scratch0();
  ASSERT(!result_reg.is(double_scratch0()));
  Label convert, load_smi, done;
  if (mode == NUMBER_CANDIDATE_IS_ANY_TAGGED) {
    // Smi check.
    __ UntagAndJumpIfSmi(scratch, input_reg, &load_smi);
    // Heap number map check.
    __ ldr(scratch, FieldMemOperand(input_reg, HeapObject::kMapOffset));
    __ LoadRoot(ip, Heap::kHeapNumberMapRootIndex);
    __ cmp(scratch, Operand(ip));
    if (can_convert_undefined_to_nan) {
      __ b(ne, &convert);
    } else {
      DeoptimizeIf(ne, env);
    }
    // load heap number
    __ vldr(result_reg, input_reg, HeapNumber::kValueOffset - kHeapObjectTag);
    if (deoptimize_on_minus_zero) {
      __ VmovLow(scratch, result_reg);
      __ cmp(scratch, Operand::Zero());
      __ b(ne, &done);
      __ VmovHigh(scratch, result_reg);
      __ cmp(scratch, Operand(HeapNumber::kSignMask));
      DeoptimizeIf(eq, env);
    }
    __ jmp(&done);
    if (can_convert_undefined_to_nan) {
      __ bind(&convert);
      // Convert undefined (and hole) to NaN.
      __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
      __ cmp(input_reg, Operand(ip));
      DeoptimizeIf(ne, env);
      __ LoadRoot(scratch, Heap::kNanValueRootIndex);
      __ vldr(result_reg, scratch, HeapNumber::kValueOffset - kHeapObjectTag);
      __ jmp(&done);
    }
  } else {
    __ SmiUntag(scratch, input_reg);
    ASSERT(mode == NUMBER_CANDIDATE_IS_SMI);
  }
  // Smi to double register conversion
  __ bind(&load_smi);
  // scratch: untagged value of input_reg
  __ vcvt_f64_s32(result_reg, scratch); // DIFF: codegen
  __ bind(&done);
}


void LCodeGen::DoDeferredTaggedToI(LTaggedToI* instr) { // REVIEWEDBY: CG
  Register input_reg = ToRegister(instr->value());
  Register scratch1 = scratch0();
  Register scratch2 = ToRegister(instr->temp());
  DwVfpRegister double_scratch = double_scratch0();
  DwVfpRegister double_scratch2 = ToDoubleRegister(instr->temp2());

  ASSERT(!scratch1.is(input_reg) && !scratch1.is(scratch2));
  ASSERT(!scratch2.is(input_reg) && !scratch2.is(scratch1));

  Label done;

  // The input was optimistically untagged; revert it.
  // The carry flag is set when we reach this deferred code as we just executed
  // SmiUntag(heap_object, SetCC).
  STATIC_ASSERT(kHeapObjectTag == 1);
  // SH4: it was a tagged object (ref LCodeGen::DoTaggedToI()), recreate the tag
  __ add(scratch2, input_reg, Operand(input_reg)); // DIFF: codegen
  __ lor(scratch2, scratch2, Operand(kHeapObjectTag)); // DIFF: codegen

  // Heap number map check.
  __ ldr(scratch1, FieldMemOperand(scratch2, HeapObject::kMapOffset));
  __ LoadRoot(ip, Heap::kHeapNumberMapRootIndex);
  __ cmp(scratch1, Operand(ip));

  if (instr->truncating()) {
    // Performs a truncating conversion of a floating point number as used by
    // the JS bitwise operations.
    Label no_heap_number, check_bools, check_false;
    __ b(ne, &no_heap_number);
    __ TruncateHeapNumberToI(input_reg, scratch2);
    __ b(&done);

    // Check for Oddballs. Undefined/False is converted to zero and True to one
    // for truncating conversions.
    __ bind(&no_heap_number);
    __ LoadRoot(ip, Heap::kUndefinedValueRootIndex);
    __ cmp(scratch2, Operand(ip));
    __ b(ne, &check_bools);
    __ mov(input_reg, Operand::Zero());
    __ b(&done);

    __ bind(&check_bools);
    __ LoadRoot(ip, Heap::kTrueValueRootIndex);
    __ cmp(scratch2, Operand(ip));
    __ b(ne, &check_false);
    __ mov(input_reg, Operand(1));
    __ b(&done);

    __ bind(&check_false);
    __ LoadRoot(ip, Heap::kFalseValueRootIndex);
    __ cmp(scratch2, Operand(ip));
    DeoptimizeIf(ne, instr->environment());
    __ mov(input_reg, Operand::Zero());
    __ b(&done);
  } else {
    // Deoptimize if we don't have a heap number.
    DeoptimizeIf(ne, instr->environment());

    __ sub(ip, scratch2, Operand(kHeapObjectTag));
    __ vldr(double_scratch2, ip, HeapNumber::kValueOffset);
    __ TryDoubleToInt32Exact(input_reg, double_scratch2, double_scratch);
    DeoptimizeIf(ne, instr->environment());

    if (instr->hydrogen()->CheckFlag(HValue::kBailoutOnMinusZero)) {
      __ cmp(input_reg, Operand::Zero());
      __ b(ne, &done);
      __ VmovHigh(scratch1, double_scratch2);
      __ tst(scratch1, Operand(HeapNumber::kSignMask));
      DeoptimizeIf(ne, instr->environment());
    }
  }
  __ bind(&done);
}


void LCodeGen::DoTaggedToI(LTaggedToI* instr) { // REVIEWEDBY: CG
  class DeferredTaggedToI V8_FINAL : public LDeferredCode {
   public:
    DeferredTaggedToI(LCodeGen* codegen, LTaggedToI* instr)
        : LDeferredCode(codegen), instr_(instr) { }
    virtual void Generate() V8_OVERRIDE {
      codegen()->DoDeferredTaggedToI(instr_);
    }
    virtual LInstruction* instr() V8_OVERRIDE { return instr_; }
   private:
    LTaggedToI* instr_;
  };

  LOperand* input = instr->value();
  ASSERT(input->IsRegister());
  ASSERT(input->Equals(instr->result()));

  Register input_reg = ToRegister(input);

  if (instr->hydrogen()->value()->representation().IsSmi()) {
    __ SmiUntag(input_reg);
  } else {
    DeferredTaggedToI* deferred = new(zone()) DeferredTaggedToI(this, instr);

    // Optimistically untag the input.
    // If the input is a HeapObject, SmiUntag will clear the T bit
    __ SmiUntag(input_reg, SetT); // DIFF: codegen
    // Branch to deferred code if the input was tagged.
    // The deferred code will take care of restoring the tag.
    __ bf(deferred->entry()); // DIFF: codegen
    __ bind(deferred->exit());
  }
}


void LCodeGen::DoNumberUntagD(LNumberUntagD* instr) { // REVIEWEDBY: CG
  LOperand* input = instr->value();
  ASSERT(input->IsRegister());
  LOperand* result = instr->result();
  ASSERT(result->IsDoubleRegister());

  Register input_reg = ToRegister(input);
  DwVfpRegister result_reg = ToDoubleRegister(result);

  HValue* value = instr->hydrogen()->value();
  NumberUntagDMode mode = value->representation().IsSmi()
      ? NUMBER_CANDIDATE_IS_SMI : NUMBER_CANDIDATE_IS_ANY_TAGGED;

  EmitNumberUntagD(input_reg, result_reg,
                   instr->hydrogen()->can_convert_undefined_to_nan(),
                   instr->hydrogen()->deoptimize_on_minus_zero(),
                   instr->environment(),
                   mode);
}


void LCodeGen::DoDoubleToI(LDoubleToI* instr) { // REVIEWEDBY: CG
  Register result_reg = ToRegister(instr->result());
  Register scratch1 = scratch0();
  DwVfpRegister double_input = ToDoubleRegister(instr->value());
  DwVfpRegister double_scratch = double_scratch0();

  if (instr->truncating()) {
    __ TruncateDoubleToI(result_reg, double_input);
  } else {
    __ TryDoubleToInt32Exact(result_reg, double_input, double_scratch);
    // Deoptimize if the input wasn't a int32 (inside a double).
    DeoptimizeIf(ne, instr->environment());
    if (instr->hydrogen()->CheckFlag(HValue::kBailoutOnMinusZero)) {
      Label done;
      __ cmp(result_reg, Operand::Zero());
      __ b(ne, &done);
      __ VmovHigh(scratch1, double_input);
      __ tst(scratch1, Operand(HeapNumber::kSignMask));
      DeoptimizeIf(ne, instr->environment());
      __ bind(&done);
    }
  }
}


void LCodeGen::DoDoubleToSmi(LDoubleToSmi* instr) { // REVIEWEDBY: CG
  Register result_reg = ToRegister(instr->result());
  Register scratch1 = scratch0();
  DwVfpRegister double_input = ToDoubleRegister(instr->value());
  DwVfpRegister double_scratch = double_scratch0();

  if (instr->truncating()) {
    __ TruncateDoubleToI(result_reg, double_input);
  } else {
    __ TryDoubleToInt32Exact(result_reg, double_input, double_scratch);
    // Deoptimize if the input wasn't a int32 (inside a double).
    DeoptimizeIf(ne, instr->environment());
    if (instr->hydrogen()->CheckFlag(HValue::kBailoutOnMinusZero)) {
      Label done;
      __ cmp(result_reg, Operand::Zero());
      __ b(ne, &done);
      __ VmovHigh(scratch1, double_input);
      __ tst(scratch1, Operand(HeapNumber::kSignMask));
      DeoptimizeIf(ne, instr->environment());
      __ bind(&done);
    }
  }
  __ SmiTag(result_reg, SetT); // DIFF: codegen
  DeoptimizeIf(t, instr->environment());
}


void LCodeGen::DoCheckSmi(LCheckSmi* instr) { // REVIEWEDBY: CG
  LOperand* input = instr->value();
  __ SmiTst(ToRegister(input));
  DeoptimizeIf(ne, instr->environment());
}


void LCodeGen::DoCheckNonSmi(LCheckNonSmi* instr) { // REVIEWEDBY: CG
  if (!instr->hydrogen()->value()->IsHeapObject()) {
    LOperand* input = instr->value();
    __ SmiTst(ToRegister(input));
    DeoptimizeIf(eq, instr->environment());
  }
}


void LCodeGen::DoCheckInstanceType(LCheckInstanceType* instr) { // REVIEWEDBY: CG
  Register input = ToRegister(instr->value());
  Register scratch = scratch0();

  __ ldr(scratch, FieldMemOperand(input, HeapObject::kMapOffset));
  __ ldrb(scratch, FieldMemOperand(scratch, Map::kInstanceTypeOffset));

  if (instr->hydrogen()->is_interval_check()) {
    InstanceType first;
    InstanceType last;
    instr->hydrogen()->GetCheckInterval(&first, &last);


    // If there is only one type in the interval check for equality.
    if (first == last) {
      __ cmpeq(scratch, Operand(first)); // DIFF: codegen
      DeoptimizeIf(ne, instr->environment());
    } else {
      __ cmphs(scratch, Operand(first)); // DIFF: codegen
      DeoptimizeIf(f, instr->environment()); // DIFF: codegen
      // Omit check for the last type.
      if (last != LAST_TYPE) {
        __ cmphi(scratch, Operand(last)); // DIFF: codegen
        DeoptimizeIf(t, instr->environment()); // DIFF: codegen
      }
    }
  } else {
    uint8_t mask;
    uint8_t tag;
    instr->hydrogen()->GetCheckMaskAndTag(&mask, &tag);

    if (IsPowerOf2(mask)) {
      ASSERT(tag == 0 || IsPowerOf2(tag));
      __ tst(scratch, Operand(mask));
      DeoptimizeIf(tag == 0 ? ne : eq, instr->environment());
    } else {
      __ and_(scratch, scratch, Operand(mask));
      __ cmp(scratch, Operand(tag));
      DeoptimizeIf(ne, instr->environment());
    }
  }
}


void LCodeGen::DoCheckValue(LCheckValue* instr) { // REVIEWEDBY: CG
  Register reg = ToRegister(instr->value());
  Handle<HeapObject> object = instr->hydrogen()->object().handle();
  AllowDeferredHandleDereference smi_check;
  if (isolate()->heap()->InNewSpace(*object)) {
    Register reg = ToRegister(instr->value());
    Handle<Cell> cell = isolate()->factory()->NewCell(object);
    __ mov(ip, Operand(Handle<Object>(cell)));
    __ ldr(ip, FieldMemOperand(ip, Cell::kValueOffset));
    __ cmp(reg, ip);
  } else {
    __ cmp(reg, Operand(object));
  }
  DeoptimizeIf(ne, instr->environment());
}


void LCodeGen::DoDeferredInstanceMigration(LCheckMaps* instr, Register object) { // REVIEWEDBY: CG
  {
    PushSafepointRegistersScope scope(this, Safepoint::kWithRegisters);
    __ push(object);
    __ mov(cp, Operand::Zero());
    __ CallRuntimeSaveDoubles(Runtime::kTryMigrateInstance);
    RecordSafepointWithRegisters(
        instr->pointer_map(), 1, Safepoint::kNoLazyDeopt);
    __ StoreToSafepointRegisterSlot(r0, scratch0());
  }
  __ tst(scratch0(), Operand(kSmiTagMask));
  DeoptimizeIf(eq, instr->environment());
}


void LCodeGen::DoCheckMaps(LCheckMaps* instr) { // REVIEWEDBY: CG
  class DeferredCheckMaps V8_FINAL : public LDeferredCode {
   public:
    DeferredCheckMaps(LCodeGen* codegen, LCheckMaps* instr, Register object)
        : LDeferredCode(codegen), instr_(instr), object_(object) {
      SetExit(check_maps());
    }
    virtual void Generate() V8_OVERRIDE {
      codegen()->DoDeferredInstanceMigration(instr_, object_);
    }
    Label* check_maps() { return &check_maps_; }
    virtual LInstruction* instr() V8_OVERRIDE { return instr_; }
   private:
    LCheckMaps* instr_;
    Label check_maps_;
    Register object_;
  };

  if (instr->hydrogen()->IsStabilityCheck()) {
    const UniqueSet<Map>* maps = instr->hydrogen()->maps();
    for (int i = 0; i < maps->size(); ++i) {
      AddStabilityDependency(maps->at(i).handle());
    }
    return;
  }

  Register map_reg = scratch0();

  LOperand* input = instr->value();
  ASSERT(input->IsRegister());
  Register reg = ToRegister(input);

  __ ldr(map_reg, FieldMemOperand(reg, HeapObject::kMapOffset));

  DeferredCheckMaps* deferred = NULL;
  if (instr->hydrogen()->HasMigrationTarget()) {
    deferred = new(zone()) DeferredCheckMaps(this, instr, reg);
    __ bind(deferred->check_maps());
  }

  const UniqueSet<Map>* maps = instr->hydrogen()->maps();
  Label success;
  for (int i = 0; i < maps->size() - 1; i++) {
    Handle<Map> map = maps->at(i).handle();
    __ CompareMap(map_reg, map, &success);
    __ b(eq, &success);
  }

  Handle<Map> map = maps->at(maps->size() - 1).handle();
  __ CompareMap(map_reg, map, &success);
  if (instr->hydrogen()->HasMigrationTarget()) {
    __ b(ne, deferred->entry());
  } else {
    DeoptimizeIf(ne, instr->environment());
  }

  __ bind(&success);
}


void LCodeGen::DoClampDToUint8(LClampDToUint8* instr) { // REVIEWEDBY: CG
  DwVfpRegister value_reg = ToDoubleRegister(instr->unclamped());
  Register result_reg = ToRegister(instr->result());
  __ ClampDoubleToUint8(result_reg, value_reg, double_scratch0());
}


void LCodeGen::DoClampIToUint8(LClampIToUint8* instr) { // REVIEWEDBY: CG
  Register unclamped_reg = ToRegister(instr->unclamped());
  Register result_reg = ToRegister(instr->result());
  __ ClampUint8(result_reg, unclamped_reg);
}


void LCodeGen::DoClampTToUint8(LClampTToUint8* instr) { // REVIEWEDBY: CG
  Register scratch = scratch0();
  Register input_reg = ToRegister(instr->unclamped());
  Register result_reg = ToRegister(instr->result());
  DwVfpRegister temp_reg = ToDoubleRegister(instr->temp());
  Label is_smi, done, heap_number;

  // Both smi and heap number cases are handled.
  __ UntagAndJumpIfSmi(result_reg, input_reg, &is_smi);

  // Check for heap number
  __ ldr(scratch, FieldMemOperand(input_reg, HeapObject::kMapOffset));
  __ cmp(scratch, Operand(factory()->heap_number_map()));
  __ b(eq, &heap_number);

  // Check for undefined. Undefined is converted to zero for clamping
  // conversions.
  __ cmp(input_reg, Operand(factory()->undefined_value()));
  DeoptimizeIf(ne, instr->environment());
  __ mov(result_reg, Operand::Zero());
  __ jmp(&done);

  // Heap number
  __ bind(&heap_number);
  __ vldr(temp_reg, FieldMemOperand(input_reg, HeapNumber::kValueOffset));
  __ ClampDoubleToUint8(result_reg, temp_reg, double_scratch0());
  __ jmp(&done);

  // smi
  __ bind(&is_smi);
  __ ClampUint8(result_reg, result_reg);

  __ bind(&done);
}


void LCodeGen::DoDoubleBits(LDoubleBits* instr) { // REVIEWEDBY: CG
  DwVfpRegister value_reg = ToDoubleRegister(instr->value());
  Register result_reg = ToRegister(instr->result());
  if (instr->hydrogen()->bits() == HDoubleBits::HIGH) {
    __ VmovHigh(result_reg, value_reg);
  } else {
    __ VmovLow(result_reg, value_reg);
  }
}


void LCodeGen::DoConstructDouble(LConstructDouble* instr) { // REVIEWEDBY: CG
  Register hi_reg = ToRegister(instr->hi());
  Register lo_reg = ToRegister(instr->lo());
  DwVfpRegister result_reg = ToDoubleRegister(instr->result());
  __ VmovHigh(result_reg, hi_reg);
  __ VmovLow(result_reg, lo_reg);
}


void LCodeGen::DoAllocate(LAllocate* instr) { // REVIEWEDBY: CG
  class DeferredAllocate V8_FINAL : public LDeferredCode {
   public:
    DeferredAllocate(LCodeGen* codegen, LAllocate* instr)
        : LDeferredCode(codegen), instr_(instr) { }
    virtual void Generate() V8_OVERRIDE {
      codegen()->DoDeferredAllocate(instr_);
    }
    virtual LInstruction* instr() V8_OVERRIDE { return instr_; }
   private:
    LAllocate* instr_;
  };

  DeferredAllocate* deferred =
      new(zone()) DeferredAllocate(this, instr);

  Register result = ToRegister(instr->result());
  Register scratch = ToRegister(instr->temp1());
  Register scratch2 = ToRegister(instr->temp2());

  // Allocate memory for the object.
  AllocationFlags flags = TAG_OBJECT;
  if (instr->hydrogen()->MustAllocateDoubleAligned()) {
    flags = static_cast<AllocationFlags>(flags | DOUBLE_ALIGNMENT);
  }
  if (instr->hydrogen()->IsOldPointerSpaceAllocation()) {
    ASSERT(!instr->hydrogen()->IsOldDataSpaceAllocation());
    ASSERT(!instr->hydrogen()->IsNewSpaceAllocation());
    flags = static_cast<AllocationFlags>(flags | PRETENURE_OLD_POINTER_SPACE);
  } else if (instr->hydrogen()->IsOldDataSpaceAllocation()) {
    ASSERT(!instr->hydrogen()->IsNewSpaceAllocation());
    flags = static_cast<AllocationFlags>(flags | PRETENURE_OLD_DATA_SPACE);
  }

  if (instr->size()->IsConstantOperand()) {
    int32_t size = ToInteger32(LConstantOperand::cast(instr->size()));
    if (size <= Page::kMaxRegularHeapObjectSize) {
      __ Allocate(size, result, scratch, scratch2, deferred->entry(), flags);
    } else {
      __ jmp(deferred->entry());
    }
  } else {
    Register size = ToRegister(instr->size());
    __ Allocate(size,
                result,
                scratch,
                scratch2,
                deferred->entry(),
                flags);
  }

  __ bind(deferred->exit());

  if (instr->hydrogen()->MustPrefillWithFiller()) {
    if (instr->size()->IsConstantOperand()) {
      int32_t size = ToInteger32(LConstantOperand::cast(instr->size()));
      __ mov(scratch, Operand(size));
    } else {
      scratch = ToRegister(instr->size());
    }
    __ sub(scratch, scratch, Operand(kPointerSize));
    __ sub(result, result, Operand(kHeapObjectTag));
    Label loop;
    __ bind(&loop);
    __ mov(scratch2, Operand(isolate()->factory()->one_pointer_filler_map()));
    __ str(scratch2, MemOperand(result, scratch));
    __ sub(scratch, scratch, Operand(kPointerSize));
    __ cmpge(scratch, Operand(0)); // DIFF: codegen
    __ b(t, &loop); // DIFF: codegen
    __ add(result, result, Operand(kHeapObjectTag));
  }
}


void LCodeGen::DoDeferredAllocate(LAllocate* instr) { // REVIEWEDBY: CG
  Register result = ToRegister(instr->result());

  // TODO(3095996): Get rid of this. For now, we need to make the
  // result register contain a valid pointer because it is already
  // contained in the register pointer map.
  __ mov(result, Operand(Smi::FromInt(0)));

  PushSafepointRegistersScope scope(this, Safepoint::kWithRegisters);
  if (instr->size()->IsRegister()) {
    Register size = ToRegister(instr->size());
    ASSERT(!size.is(result));
    __ SmiTag(size);
    __ push(size);
  } else {
    int32_t size = ToInteger32(LConstantOperand::cast(instr->size()));
    if (size >= 0 && size <= Smi::kMaxValue) {
      __ Push(Smi::FromInt(size));
    } else {
      // We should never get here at runtime => abort
      __ stop("invalid allocation size");
      return;
    }
  }

  int flags = AllocateDoubleAlignFlag::encode(
      instr->hydrogen()->MustAllocateDoubleAligned());
  if (instr->hydrogen()->IsOldPointerSpaceAllocation()) {
    ASSERT(!instr->hydrogen()->IsOldDataSpaceAllocation());
    ASSERT(!instr->hydrogen()->IsNewSpaceAllocation());
    flags = AllocateTargetSpace::update(flags, OLD_POINTER_SPACE);
  } else if (instr->hydrogen()->IsOldDataSpaceAllocation()) {
    ASSERT(!instr->hydrogen()->IsNewSpaceAllocation());
    flags = AllocateTargetSpace::update(flags, OLD_DATA_SPACE);
  } else {
    flags = AllocateTargetSpace::update(flags, NEW_SPACE);
  }
  __ Push(Smi::FromInt(flags));

  CallRuntimeFromDeferred(
      Runtime::kHiddenAllocateInTargetSpace, 2, instr, instr->context());
  __ StoreToSafepointRegisterSlot(r0, result);
}


void LCodeGen::DoToFastProperties(LToFastProperties* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoRegExpLiteral(LRegExpLiteral* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoFunctionLiteral(LFunctionLiteral* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoTypeof(LTypeof* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoTypeofIsAndBranch(LTypeofIsAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


Condition LCodeGen::EmitTypeofIs(Label* true_label,
                                 Label* false_label,
                                 Register input,
                                 Handle<String> type_name) {
  __ UNIMPLEMENTED_BREAK();
  return ne;
}


void LCodeGen::DoIsConstructCallAndBranch(LIsConstructCallAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::EmitIsConstructCall(Register temp1, Register temp2) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::EnsureSpaceForLazyDeopt(int space_needed) { // REVIEWEDBY: CG
  if (!info()->IsStub()) {
    // Ensure that we have enough space after the previous lazy-bailout
    // instruction for patching the code here.
    int current_pc = masm()->pc_offset();
    if (current_pc < last_lazy_deopt_pc_ + space_needed) {
      // Block literal pool emission for duration of padding.
      Assembler::BlockConstPoolScope block_const_pool(masm());
      int padding_size = last_lazy_deopt_pc_ + space_needed - current_pc;
      ASSERT_EQ(0, padding_size % Assembler::kInstrSize);
      while (padding_size > 0) {
        __ nop();
        padding_size -= Assembler::kInstrSize;
      }
    }
  }
  last_lazy_deopt_pc_ = masm()->pc_offset();
}


void LCodeGen::DoLazyBailout(LLazyBailout* instr) { // REVIEWEDBY: CG
  last_lazy_deopt_pc_ = masm()->pc_offset();
  ASSERT(instr->HasEnvironment());
  LEnvironment* env = instr->environment();
  RegisterEnvironmentForDeoptimization(env, Safepoint::kLazyDeopt);
  safepoints_.RecordLazyDeoptimizationIndex(env->deoptimization_index());
}


void LCodeGen::DoDeoptimize(LDeoptimize* instr) { // REVIEWEDBY: CG
  Deoptimizer::BailoutType type = instr->hydrogen()->type();
  // TODO(danno): Stubs expect all deopts to be lazy for historical reasons (the
  // needed return address), even though the implementation of LAZY and EAGER is
  // now identical. When LAZY is eventually completely folded into EAGER, remove
  // the special case below.
  if (info()->IsStub() && type == Deoptimizer::EAGER) {
    type = Deoptimizer::LAZY;
  }

  Comment(";;; deoptimize: %s", instr->hydrogen()->reason());
  DeoptimizeIf(al, instr->environment(), type);
}


void LCodeGen::DoDummy(LDummy* instr) { // REVIEWEDBY: CG
  // Nothing to see here, move on!
}


void LCodeGen::DoDummyUse(LDummyUse* instr) { // REVIEWEDBY: CG
  // Nothing to see here, move on!
}


void LCodeGen::DoDeferredStackCheck(LStackCheck* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStackCheck(LStackCheck* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoOsrEntry(LOsrEntry* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoForInPrepareMap(LForInPrepareMap* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoForInCacheArray(LForInCacheArray* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCheckMapValue(LCheckMapValue* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredLoadMutableDouble(LLoadFieldByIndex* instr,
                                           Register result,
                                           Register object,
                                           Register index) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadFieldByIndex(LLoadFieldByIndex* instr) {
  __ UNIMPLEMENTED_BREAK();
}


#undef __

} }  // namespace v8::internal
