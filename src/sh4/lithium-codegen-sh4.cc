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

#include "sh4/lithium-codegen-sh4.h"
#include "sh4/lithium-gap-resolver-sh4.h"
#include "code-stubs.h"
#include "stub-cache.h"
#include "hydrogen-osr.h"

namespace v8 {
namespace internal {

#include "map-sh4.h"  // Define register map

class SafepointGenerator : public CallWrapper {
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

bool LCodeGen::GenerateCode() {
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


void LCodeGen::FinishCode(Handle<Code> code) { // SAMEAS: arm
  ASSERT(is_done());
  code->set_stack_slots(GetStackSlotCount());
  code->set_safepoint_table_offset(safepoints_.GetCodeOffset());
  if (FLAG_weak_embedded_maps_in_optimized_code) {
    RegisterDependentCodeForEmbeddedMaps(code);
  }
  PopulateDeoptimizationData(code);
  info()->CommitDependencies(code);
}


void LCodeGen::Abort(BailoutReason reason) { // SAMEAS: arm
  info()->set_bailout_reason(reason);
  status_ = ABORTED;
}


bool LCodeGen::GeneratePrologue() { // SAMEAS: arm
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
    // fp: Caller's frame pointer.
    // lr: Caller's pc.

    // Strict mode functions and builtins need to replace the receiver
    // with undefined when called as functions (without an explicit
    // receiver object). r5 is zero for method calls and non-zero for
    // function calls.
    if (!info_->is_classic_mode() || info_->is_native()) {
      Label skip;
      __ cmp(r5, Operand::Zero());
      int receiver_offset = scope()->num_parameters() * kPointerSize;
      __ LoadRoot(r2, Heap::kUndefinedValueRootIndex);
      __ bt_near(&skip);
      __ str(r2, MemOperand(sp, receiver_offset)); // DIFF: codegen
      __ bind(&skip);
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

  // Possibly allocate a local context.
  int heap_slots = info()->num_heap_slots() - Context::MIN_CONTEXT_SLOTS;
  if (heap_slots > 0) {
    Comment(";;; Allocate local context");
    // Argument to NewContext is the function, which is in r1.
    __ push(r1);
    if (heap_slots <= FastNewContextStub::kMaximumSlots) {
      FastNewContextStub stub(heap_slots);
      __ CallStub(&stub);
    } else {
      __ CallRuntime(Runtime::kNewFunctionContext, 1);
    }
    RecordSafepoint(Safepoint::kNoLazyDeopt);
    // Context is returned in both r0 and cp.  It replaces the context
    // passed to us.  It's saved in the stack and kept live in cp.
    __ str(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
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


void LCodeGen::GenerateOsrPrologue() { // SAMEAS: arm
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


bool LCodeGen::GenerateDeferredCode() { // SAEMAS: arm
  ASSERT(is_generating());
  if (deferred_.length() > 0) {
    for (int i = 0; !is_aborted() && i < deferred_.length(); i++) {
      LDeferredCode* code = deferred_[i];

      HValue* value =
          instructions_->at(code->instruction_index())->hydrogen_value();
      RecordAndWritePosition(value->position());

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
        // SH4: sameas MacroAssembler::Prologue for BUILD_STUB_FRAME
        __ Push(pr, fp, cp); // DIFF: codegen
        __ Push(Smi::FromInt(StackFrame::STUB)); // DIFF: codegen
        // Adjust FP to point to saved FP.
        __ add(fp, sp, Operand(2 * kPointerSize));
        Comment(";;; Deferred code");
      }
      code->Generate();
      if (NeedsDeferredFrame()) {
        Comment(";;; Destroy frame");
        ASSERT(frame_is_built_);
        __ Pop(ip); // Drop STUB kind // DIFF: codegen
        __ Pop(pr, fp, cp); // DIFF: codegen
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


bool LCodeGen::GenerateDeoptJumpTable() { // SAMEAS: arm
  // Check that the jump table is accessible from everywhere in the function
  // code, i.e. that offsets to the table can be encoded in the 24bit signed
  // immediate of a branch instruction.
  // To simplify we consider the code size from the first instruction to the
  // end of the jump table. We also don't consider the pc load delta.
  // Each entry in the jump table generates one instruction and inlines one
  // 32bit data after it.
  // TODO(stm): SH4: branches can encode only 8 bits instruction words offsets
  // This is certainly insufficient in order to encode the jump table
  // offsets from everywhere in the code.
  // Need to understand which branch are potentially jumping to a Deopt
  // Jump Table.
  // TODO(stm): should patch a 32 bit indirect branch as branch
  // immediates are too short on SH4: ref to LCodeGen::DeoptimizeIf()
  //if (!FITS_SH4_bt(masm()->pc_offset() + // DIFF: encoding
  //                 deopt_jump_table_.length() *
  //                 7/*max instructions*/ * Assembler::kInstrSize)) {
  //  Abort(kGeneratedCodeIsTooLarge);
  //}

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
      __ mov(ip, Operand(ExternalReference::ForDeoptEntry(entry)));
      if (needs_frame.is_bound()) {
        __ b(&needs_frame);
      } else {
        __ bind(&needs_frame);
        __ Push(pr, fp, cp); // DIFF: codegen
        // This variant of deopt can only be used with stubs. Since we don't
        // have a function pointer to install in the stack frame that we're
        // building, install a special marker there instead.
        ASSERT(info()->IsStub());
        __ mov(scratch0(), Operand(Smi::FromInt(StackFrame::STUB)));
        __ push(scratch0());
        __ add(fp, sp, Operand(2 * kPointerSize));
        __ jsr(ip); // DIFF: codegen
      }
    } else {
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


bool LCodeGen::GenerateSafepointTable() { // SAMEAS: arm
  ASSERT(is_done());
  safepoints_.Emit(masm(), GetStackSlotCount());
  return !is_aborted();
}


Register LCodeGen::ToRegister(int index) const { // SAMEAS: arm
  return Register::FromAllocationIndex(index);
}


DwVfpRegister LCodeGen::ToDoubleRegister(int index) const { // SAMEAS: arm
  return DwVfpRegister::FromAllocationIndex(index);
}


Register LCodeGen::ToRegister(LOperand* op) const { // SAMEAS: arm
  ASSERT(op->IsRegister());
  return ToRegister(op->index());
}


Register LCodeGen::EmitLoadRegister(LOperand* op, Register scratch) { // SAMEAS: arm
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
  } else if (op->IsStackSlot() || op->IsArgument()) {
    __ ldr(scratch, ToMemOperand(op));
    return scratch;
  }
  UNREACHABLE();
  return scratch;
}


DwVfpRegister LCodeGen::ToDoubleRegister(LOperand* op) const { // SAMEAS: arm
  ASSERT(op->IsDoubleRegister());
  return ToDoubleRegister(op->index());
}


DwVfpRegister LCodeGen::EmitLoadDoubleRegister(LOperand* op,
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
      // SH4: note flt_scratch is not used
      __ mov(ip, Operand(static_cast<int32_t>(literal->Number())));
      __ vcvt_f64_s32(dbl_scratch, ip); // DIFF: codegen
      return dbl_scratch;
    } else if (r.IsDouble()) {
      Abort(kUnsupportedDoubleImmediate);
    } else if (r.IsTagged()) {
      Abort(kUnsupportedTaggedImmediate);
    }
  } else if (op->IsStackSlot() || op->IsArgument()) {
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


Handle<Object> LCodeGen::ToHandle(LConstantOperand* op) const { // SAMEAS: arm
  HConstant* constant = chunk_->LookupConstant(op);
  ASSERT(chunk_->LookupLiteralRepresentation(op).IsSmiOrTagged());
  return constant->handle(isolate());
}


bool LCodeGen::IsInteger32(LConstantOperand* op) const { // SAMEAS: arm
  return chunk_->LookupLiteralRepresentation(op).IsSmiOrInteger32();
}


bool LCodeGen::IsSmi(LConstantOperand* op) const { // SAMEAS: arm
  return chunk_->LookupLiteralRepresentation(op).IsSmi();
}


int32_t LCodeGen::ToInteger32(LConstantOperand* op) const { // SAMEAS: arm
  return ToRepresentation(op, Representation::Integer32());
}


int32_t LCodeGen::ToRepresentation(LConstantOperand* op, // SAMEAS: arm
                                   const Representation& r) const {
  HConstant* constant = chunk_->LookupConstant(op);
  int32_t value = constant->Integer32Value();
  if (r.IsInteger32()) return value;
  ASSERT(r.IsSmiOrTagged());
  return reinterpret_cast<int32_t>(Smi::FromInt(value));
}


Smi* LCodeGen::ToSmi(LConstantOperand* op) const { // SAMEAS: arm
  HConstant* constant = chunk_->LookupConstant(op);
  return Smi::FromInt(constant->Integer32Value());
}


double LCodeGen::ToDouble(LConstantOperand* op) const {
  UNIMPLEMENTED();
  return 0.0;
}


Operand LCodeGen::ToOperand(LOperand* op) { // SAMEAS: arm
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


MemOperand LCodeGen::ToMemOperand(LOperand* op) const { // SAMEAS: arm
  ASSERT(!op->IsRegister());
  ASSERT(!op->IsDoubleRegister());
  ASSERT(op->IsStackSlot() || op->IsDoubleStackSlot());
  return MemOperand(fp, StackSlotOffset(op->index()));
}


MemOperand LCodeGen::ToHighMemOperand(LOperand* op) const { // SAMEAS: arm
  ASSERT(op->IsDoubleStackSlot());
  return MemOperand(fp, StackSlotOffset(op->index()) + kPointerSize);
}


void LCodeGen::WriteTranslation(LEnvironment* environment, // SAMEAS: arm
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


void LCodeGen::AddToTranslation(LEnvironment* environment, // SAMEAS: arm
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
  } else if (op->IsArgument()) {
    ASSERT(is_tagged);
    int src_index = GetStackSlotCount() + op->index();
    translation->StoreStackSlot(src_index);
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


void LCodeGen::CallCode(Handle<Code> code, // SAMEAS: arm
                        RelocInfo::Mode mode,
                        LInstruction* instr,
                        TargetAddressStorageMode storage_mode) {
  CallCodeGeneric(code, mode, instr, RECORD_SIMPLE_SAFEPOINT, storage_mode);
}


void LCodeGen::CallCodeGeneric(Handle<Code> code, // SAMEAS: arm
                               RelocInfo::Mode mode,
                               LInstruction* instr,
                               SafepointMode safepoint_mode,
                               TargetAddressStorageMode storage_mode) {
  EnsureSpaceForLazyDeopt(Deoptimizer::patch_size());
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
    __ nop();
  }
}


void LCodeGen::CallRuntime(const Runtime::Function* function, // SAMEAS: arm
                           int num_arguments,
                           LInstruction* instr,
                           SaveFPRegsMode save_doubles) {
  ASSERT(instr != NULL);

  __ CallRuntime(function, num_arguments, save_doubles);

  RecordSafepointWithLazyDeopt(instr, RECORD_SIMPLE_SAFEPOINT);
}


void LCodeGen::LoadContextFromDeferred(LOperand* context) { // SAMEAS: arm
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


void LCodeGen::CallRuntimeFromDeferred(Runtime::FunctionId id, // SAMEAS: arm
                                       int argc,
                                       LInstruction* instr,
                                       LOperand* context) {
  LoadContextFromDeferred(context);
  __ CallRuntimeSaveDoubles(id);
  RecordSafepointWithRegisters(
      instr->pointer_map(), argc, Safepoint::kNoLazyDeopt);
}


void LCodeGen::RegisterEnvironmentForDeoptimization(LEnvironment* environment, // SAMEAS: arm
                                                    Safepoint::DeoptMode mode) {
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


void LCodeGen::DeoptimizeIf(Condition condition, // SAMEAS: arm
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

  ASSERT(FLAG_deopt_every_n_times < 2);  // Other values not supported on ARM.
  if (FLAG_deopt_every_n_times == 1 &&
      !info()->IsStub() &&
      info()->opt_count() == id) {
    ASSERT(frame_is_built_);
    __ Call(entry, RelocInfo::RUNTIME_ENTRY);
    return;
  }

  if (info()->ShouldTrapOnDeopt()) {
    __ stop("trap_on_deopt", condition);
  }

  ASSERT(info()->IsStub() || frame_is_built_);
  if (condition == al && frame_is_built_) {
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


void LCodeGen::DeoptimizeIf(Condition condition, // SAMEAS: arm
                            LEnvironment* environment) {
  ASSERT(condition == ne || condition == eq || condition == al); // DIFF: codegen
  Deoptimizer::BailoutType bailout_type = info()->IsStub()
      ? Deoptimizer::LAZY
      : Deoptimizer::EAGER;
  DeoptimizeIf(condition, environment, bailout_type);
}

void LCodeGen::RegisterDependentCodeForEmbeddedMaps(Handle<Code> code) { // SAMEAS: arm
  ZoneList<Handle<Map> > maps(1, zone());
  ZoneList<Handle<JSObject> > objects(1, zone());
  int mode_mask = RelocInfo::ModeMask(RelocInfo::EMBEDDED_OBJECT);
  for (RelocIterator it(*code, mode_mask); !it.done(); it.next()) {
    if (Code::IsWeakEmbeddedObject(code->kind(), it.rinfo()->target_object())) {
      if (it.rinfo()->target_object()->IsMap()) {
        Handle<Map> map(Map::cast(it.rinfo()->target_object()));
        maps.Add(map, zone());
      } else if (it.rinfo()->target_object()->IsJSObject()) {
        Handle<JSObject> object(JSObject::cast(it.rinfo()->target_object()));
        objects.Add(object, zone());
      }
    }
  }
#ifdef VERIFY_HEAP
  // This disables verification of weak embedded objects after full GC.
  // AddDependentCode can cause a GC, which would observe the state where
  // this code is not yet in the depended code lists of the embedded maps.
  NoWeakObjectVerificationScope disable_verification_of_embedded_objects;
#endif
  for (int i = 0; i < maps.length(); i++) {
    maps.at(i)->AddDependentCode(DependentCode::kWeaklyEmbeddedGroup, code);
  }
  for (int i = 0; i < objects.length(); i++) {
    AddWeakObjectToCodeDependency(isolate()->heap(), objects.at(i), code);
  }
}

void LCodeGen::PopulateDeoptimizationData(Handle<Code> code) { // SAMEAS: arm
  int length = deoptimizations_.length();
  if (length == 0) return;
  Handle<DeoptimizationInputData> data =
      factory()->NewDeoptimizationInputData(length, TENURED);

  Handle<ByteArray> translations =
      translations_.CreateByteArray(isolate()->factory());
  data->SetTranslationByteArray(*translations);
  data->SetInlinedFunctionCount(Smi::FromInt(inlined_function_count_));

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


int LCodeGen::DefineDeoptimizationLiteral(Handle<Object> literal) {
  UNIMPLEMENTED();
  return -1;
}


void LCodeGen::PopulateDeoptimizationLiteralsWithInlinedFunctions() {
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


void LCodeGen::RecordSafepointWithLazyDeopt( // SAMEAS: arm
    LInstruction* instr, SafepointMode safepoint_mode) {
  if (safepoint_mode == RECORD_SIMPLE_SAFEPOINT) {
    RecordSafepoint(instr->pointer_map(), Safepoint::kLazyDeopt);
  } else {
    ASSERT(safepoint_mode == RECORD_SAFEPOINT_WITH_REGISTERS_AND_NO_ARGUMENTS);
    RecordSafepointWithRegisters(
        instr->pointer_map(), 0, Safepoint::kLazyDeopt);
  }
}


void LCodeGen::RecordSafepoint( // SAMEAS: arm
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
}


void LCodeGen::RecordSafepoint(LPointerMap* pointers, // SAMEAS: arm
                               Safepoint::DeoptMode deopt_mode) {
  RecordSafepoint(pointers, Safepoint::kSimple, 0, deopt_mode);
}


void LCodeGen::RecordSafepoint(Safepoint::DeoptMode deopt_mode) { // SAMEAS: arm
  LPointerMap empty_pointers(zone());
  RecordSafepoint(&empty_pointers, deopt_mode);
}


void LCodeGen::RecordSafepointWithRegisters(LPointerMap* pointers, // SAMEAS: arm
                                            int arguments,
                                            Safepoint::DeoptMode deopt_mode) {
  RecordSafepoint(
      pointers, Safepoint::kWithRegisters, arguments, deopt_mode);
}


void LCodeGen::RecordSafepointWithRegistersAndDoubles( // SAMEAS: arm
    LPointerMap* pointers,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {
  RecordSafepoint(
      pointers, Safepoint::kWithRegistersAndDoubles, arguments, deopt_mode);
}


void LCodeGen::RecordAndWritePosition(int position) {
  if (position == RelocInfo::kNoPosition) return;
  masm()->positions_recorder()->RecordPosition(position);
  masm()->positions_recorder()->WriteRecordedPositions();
}


static const char* LabelType(LLabel* label) { // SAMEAS: arm
  if (label->is_loop_header()) return " (loop header)";
  if (label->is_osr_entry()) return " (OSR entry)";
  return "";
}


void LCodeGen::DoLabel(LLabel* label) { // SAMEAS: arm
  Comment(";;; <@%d,#%d> -------------------- B%d%s --------------------",
          current_instruction_,
          label->hydrogen_value()->id(),
          label->block_id(),
          LabelType(label));
  __ bind(label->label());
  current_block_ = label->block_id();
  DoGap(label);
}


void LCodeGen::DoParallelMove(LParallelMove* move) { // SAMEAS: arm
  resolver_.Resolve(move);
}


void LCodeGen::DoGap(LGap* gap) { // SAMEAS: arm
  for (int i = LGap::FIRST_INNER_POSITION;
       i <= LGap::LAST_INNER_POSITION;
       i++) {
    LGap::InnerPosition inner_pos = static_cast<LGap::InnerPosition>(i);
    LParallelMove* move = gap->GetParallelMove(inner_pos);
    if (move != NULL) DoParallelMove(move);
  }
}


void LCodeGen::DoInstructionGap(LInstructionGap* instr) { // SAMEAS: arm
  DoGap(instr);
}


void LCodeGen::DoParameter(LParameter* instr) { // SAMEAS: arm
  // Nothing to do.
}


void LCodeGen::DoCallStub(LCallStub* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoUnknownOSRValue(LUnknownOSRValue* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoModI(LModI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::EmitSignedIntegerDivisionByConstant(
    Register result,
    Register dividend,
    int32_t divisor,
    Register remainder,
    Register scratch,
    LEnvironment* environment) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDivI(LDivI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMultiplyAddD(LMultiplyAddD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMultiplySubD(LMultiplySubD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathFloorOfDiv(LMathFloorOfDiv* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMulI(LMulI* instr) { // SAMEAS: arm
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
          __ UNIMPLEMENTED_BREAK();
          // __ rsb(result, left, Operand::Zero(), SetCC);
          // DeoptimizeIf(vs, instr->environment());
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
          __ add(result, result, left); // DIFF: codegen
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
      __ UNIMPLEMENTED_BREAK();
      //Register scratch = scratch0();
      // // scratch:result = left * right.
      // if (instr->hydrogen()->representation().IsSmi()) {
      //   __ SmiUntag(result, left);
      //   __ smull(result, scratch, result, right);
      // } else {
      //   __ smull(result, scratch, left, right);
      // }
      // __ cmp(scratch, Operand(result, ASR, 31));
      // DeoptimizeIf(ne, instr->environment());
    } else {
      if (instr->hydrogen()->representation().IsSmi()) {
        __ SmiUntag(result, left);
        __ mul(result, result, right);
      } else {
        __ mul(result, left, right);
      }
    }

    if (bailout_on_minus_zero) {
      __ UNIMPLEMENTED_BREAK();
      //Label done;
      // __ teq(left, Operand(right));
      // __ b(pl, &done);
      // // Bail out if the result is minus zero.
      // __ cmp(result, Operand::Zero());
      // DeoptimizeIf(eq, instr->environment());
      // __ bind(&done);
    }
  }
}


void LCodeGen::DoBitI(LBitI* instr) { // SAMEAS: arm
  LOperand* left_op = instr->left();
  LOperand* right_op = instr->right();
  ASSERT(left_op->IsRegister());
  Register left = ToRegister(left_op);
  Register result = ToRegister(instr->result());
  Operand right(no_reg);

  if (right_op->IsStackSlot() || right_op->IsArgument()) {
    right = Operand(EmitLoadRegister(right_op, ip));
  } else {
    ASSERT(right_op->IsRegister() || right_op->IsConstantOperand());
    right = ToOperand(right_op);
  }

  switch (instr->op()) {
    case Token::BIT_AND:
      __ land(result, left, right); // DIFF: codegen
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


void LCodeGen::DoShiftI(LShiftI* instr) { // SAMEAS: arm
  // Both 'left' and 'right' are "used at start" (see LCodeGen::DoShift), so
  // result may alias either of them.
  LOperand* right_op = instr->right();
  Register left = ToRegister(instr->left());
  Register result = ToRegister(instr->result());
  Register scratch = scratch0();
  Register scratch2 = ToRegister(instr->temp1()); // DIFF: codegen

  if (right_op->IsRegister()) {
    // Mask the right_op operand.
    __ land(scratch, ToRegister(right_op), Operand(0x1F)); // DIFF: codegen
    switch (instr->op()) {
      case Token::ROR:
        // SH4: ROR not available, use lsr/lsl and an additional scratch
        __ lsr(scratch2, left, scratch, true/*in_range*/); // DIFF: codegen
        __ lsl(scratch, left, scratch, true/*in_range*/); // DIFF: codegen
        __ lor(result, scratch, scratch2); // Diff: codegen
        break;
      case Token::SAR:
        __ asr(result, left, scratch, true/*in_range*/); // DIFF: codegen
        break;
      case Token::SHR:
        if (instr->can_deopt()) {
          __ lsr(result, left, scratch, true/*in_range*/);
          __ tst(result, Operand(0x80000000)); // DIFF: codegen
          DeoptimizeIf(ne, instr->environment()); // DIFF: codegen
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
          __ lsl(result, left, Operand(shift_count)); // DIFF: codegen
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
            __ tst(left, Operand(0x80000000));
            DeoptimizeIf(ne, instr->environment());
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


void LCodeGen::DoSubI(LSubI* instr) { // SAMEAS: arm
  LOperand* left = instr->left();
  LOperand* right = instr->right();
  LOperand* result = instr->result();
  bool can_overflow = instr->hydrogen()->CheckFlag(HValue::kCanOverflow);
  //SBit set_cond = can_overflow ? SetCC : LeaveCC;

  if (right->IsStackSlot() || right->IsArgument()) {
    Register right_reg = EmitLoadRegister(right, ip);
    if (can_overflow)
      __ subv(ToRegister(result), ToRegister(left), Operand(right_reg)); // DIFF: codegen
    else
      __ sub(ToRegister(result), ToRegister(left), Operand(right_reg)); // DIFF: codegen
  } else {
    ASSERT(right->IsRegister() || right->IsConstantOperand());
    if (can_overflow)
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


void LCodeGen::DoConstantI(LConstantI* instr) { // SAMEAS: arm
  __ mov(ToRegister(instr->result()), Operand(instr->value()));
}


void LCodeGen::DoConstantS(LConstantS* instr) { // SAMEAS: arm
  __ mov(ToRegister(instr->result()), Operand(instr->value()));
}


void LCodeGen::DoConstantD(LConstantD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoConstantE(LConstantE* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoConstantT(LConstantT* instr) {
  Handle<Object> value = instr->value(isolate());
  AllowDeferredHandleDereference smi_check;
  __ Move(ToRegister(instr->result()), value);
}


void LCodeGen::DoMapEnumLength(LMapEnumLength* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoElementsKind(LElementsKind* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoValueOf(LValueOf* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDateField(LDateField* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoSeqStringSetChar(LSeqStringSetChar* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoThrow(LThrow* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoAddI(LAddI* instr) { // SAMEAS: arm
  LOperand* left = instr->left();
  LOperand* right = instr->right();
  LOperand* result = instr->result();
  bool can_overflow = instr->hydrogen()->CheckFlag(HValue::kCanOverflow);
  SBit set_cond = can_overflow ? SetT : LeaveT; // DIFF: codegen

  if (right->IsStackSlot() || right->IsArgument()) {
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


void LCodeGen::DoArithmeticD(LArithmeticD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoArithmeticT(LArithmeticT* instr) {
  __ UNIMPLEMENTED_BREAK();
}


template<class InstrType>
void LCodeGen::EmitBranch(InstrType instr, Condition condition) { // SAMEAS: arm
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
void LCodeGen::EmitFalseBranch(InstrType instr, Condition condition) {
  int false_block = instr->FalseDestination(chunk_);
  ASSERT(condition == eq || condition == ne || condition == al); // DIFF: codegen
  __ b(condition, chunk_->GetAssemblyLabel(false_block));
}


void LCodeGen::DoDebugBreak(LDebugBreak* instr) { // SAMEAS: arm
  __ stop("LBreak");
}


void LCodeGen::DoBranch(LBranch* instr) { // SAMEAS: arm
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


void LCodeGen::EmitGoto(int block) { // SAMEAS: arm
  if (!IsNextEmittedBlock(block)) {
    __ jmp(chunk_->GetAssemblyLabel(LookupDestination(block)));
  }
}


void LCodeGen::DoGoto(LGoto* instr) { // SAMEAS: arm
  EmitGoto(instr->block_id());
}


Condition LCodeGen::TokenToCondition(Token::Value op, bool is_unsigned) { // SAMEAS: arm
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
// Emits a modified cond when comparing doible or jump to
// the nan label.
static void EmitVFPCompare(LCodeGen* codegen,
                           Condition *cond,
                           DwVfpRegister double1,
                           DwVfpRegister double2,
                           Label *nan)
{
  // TODO: may move this to macro-assembler-sh4.cc
  // Test for NaN
  codegen-> __ dcmpeq(double1, double1);
  codegen-> __ bf(nan);
  codegen-> __ dcmpeq(double2, double2);
  codegen-> __ bf(nan);

  // Generate code eand modify condition
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

void LCodeGen::DoCompareNumericAndBranch(LCompareNumericAndBranch* instr) { // SAMEAS: arm
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


void LCodeGen::DoCmpObjectEqAndBranch(LCmpObjectEqAndBranch* instr) { // SAMEAS: arm
  Register left = ToRegister(instr->left());
  Register right = ToRegister(instr->right());

  __ cmp(left, Operand(right));
  EmitBranch(instr, eq);
}


void LCodeGen::DoCmpHoleAndBranch(LCmpHoleAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


Condition LCodeGen::EmitIsObject(Register input,
                                 Register temp1,
                                 Label* is_not_object,
                                 Label* is_object) {
  __ UNIMPLEMENTED_BREAK();
  return ne;
}


void LCodeGen::DoIsObjectAndBranch(LIsObjectAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


Condition LCodeGen::EmitIsString(Register input, // SAMEAS: arm
                                 Register temp1,
                                 Label* is_not_string,
                                 SmiCheck check_needed = INLINE_SMI_CHECK) {
  if (check_needed == INLINE_SMI_CHECK) {
    __ JumpIfSmi(input, is_not_string);
  }
  __ CompareObjectType(input, temp1, temp1, FIRST_NONSTRING_TYPE, ge); // DIFF: codegen

  return f; // DIFF: codegen
}


void LCodeGen::DoIsStringAndBranch(LIsStringAndBranch* instr) { // SAMEAS: arm
  Register reg = ToRegister(instr->value());
  Register temp1 = ToRegister(instr->temp());

  SmiCheck check_needed =
      instr->hydrogen()->value()->IsHeapObject()
          ? OMIT_SMI_CHECK : INLINE_SMI_CHECK;
  Condition true_cond =
      EmitIsString(reg, temp1, instr->FalseLabel(chunk_), check_needed);

  EmitBranch(instr, true_cond);
}


void LCodeGen::DoIsSmiAndBranch(LIsSmiAndBranch* instr) { // SAMEAS: arm
  Register input_reg = EmitLoadRegister(instr->value(), ip);
  __ SmiTst(input_reg);
  EmitBranch(instr, eq);
}


void LCodeGen::DoIsUndetectableAndBranch(LIsUndetectableAndBranch* instr) { // SAMEAS: arm
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


void LCodeGen::DoHasInstanceTypeAndBranch(LHasInstanceTypeAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoCmpMapAndBranch(LCmpMapAndBranch* instr) { // SAMEAS: arm
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


void LCodeGen::DoReturn(LReturn* instr) { // SAMEAS: arm
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
    ASSERT(NeedsEagerFrame());
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
  int no_frame_start = -1;
  if (NeedsEagerFrame()) {
    __ mov(sp, fp);
    no_frame_start = masm_->pc_offset();
    __ Pop(lr, fp); // DIFF: codegen
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


void LCodeGen::DoStoreGlobalGeneric(LStoreGlobalGeneric* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadContextSlot(LLoadContextSlot* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreContextSlot(LStoreContextSlot* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadNamedField(LLoadNamedField* instr) { // SAMEAS: arm
  HObjectAccess access = instr->hydrogen()->access();
  int offset = access.offset();
  Register object = ToRegister(instr->object());

  if (access.IsExternalMemory()) {
    Register result = ToRegister(instr->result());
    MemOperand operand = MemOperand(object, offset);
    if (access.representation().IsByte()) {
      __ ldrb(result, operand);
    } else {
      __ ldr(result, operand);
    }
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
  if (access.representation().IsByte()) {
    __ ldrb(result, operand);
  } else {
    __ ldr(result, operand);
  }
}


void LCodeGen::DoLoadNamedGeneric(LLoadNamedGeneric* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadFunctionPrototype(LLoadFunctionPrototype* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadRoot(LLoadRoot* instr) { // SAMEAS: arm
  Register result = ToRegister(instr->result());
  __ LoadRoot(result, instr->index());
}


void LCodeGen::DoLoadExternalArrayPointer(
    LLoadExternalArrayPointer* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoAccessArgumentsAt(LAccessArgumentsAt* instr) { // SAMEAS: arm
  Register arguments = ToRegister(instr->arguments());
  Register result = ToRegister(instr->result());
  if (instr->length()->IsConstantOperand() &&
      instr->index()->IsConstantOperand()) {
    int const_index = ToInteger32(LConstantOperand::cast(instr->index()));
    int const_length = ToInteger32(LConstantOperand::cast(instr->length()));
    int index = (const_length - const_index) + 1;
    __ ldr(result, MemOperand(arguments, index * kPointerSize));
  } else {
    Register length = ToRegister(instr->length());
    Register index = ToRegister(instr->index());
    // There are two words between the frame pointer and the last argument.
    // Subtracting from length accounts for one of them add one more.
    __ sub(length, length, index);
    __ add(length, length, Operand(1));
    __ lsl(length, length, Operand(kPointerSizeLog2)); // DIFF: codegen
    __ add(length, arguments, length); // DIFF: codegen
    __ ldr(result, MemOperand(length, 0)); // DIFF: codegen
  }
}


void LCodeGen::DoLoadKeyedExternalArray(LLoadKeyed* instr) { // SAMEAS: arm
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
  int additional_offset = instr->additional_index() << element_size_shift;

  if (elements_kind == EXTERNAL_FLOAT_ELEMENTS ||
      elements_kind == EXTERNAL_DOUBLE_ELEMENTS) {
    __ UNIMPLEMENTED_BREAK(); // TODO: FPU
    //DwVfpRegister result = ToDoubleRegister(instr->result());
    //Operand operand = key_is_constant
    //    ? Operand(constant_key << element_size_shift)
    //    : Operand(key, LSL, shift_size);
    //__ add(scratch0(), external_pointer, operand);
    //if (elements_kind == EXTERNAL_FLOAT_ELEMENTS) {
    //  __ vldr(double_scratch0().low(), scratch0(), additional_offset);
    //  __ vcvt_f64_f32(result, double_scratch0().low());
    //} else  {  // i.e. elements_kind == EXTERNAL_DOUBLE_ELEMENTS
    //  __ vldr(result, scratch0(), additional_offset);
    //}
  } else {
    Register result = ToRegister(instr->result());
    MemOperand mem_operand = PrepareKeyedOperand(
        key, external_pointer, key_is_constant, constant_key,
        element_size_shift, shift_size,
        instr->additional_index(), additional_offset);
    switch (elements_kind) {
      case EXTERNAL_BYTE_ELEMENTS:
        __ ldrsb(result, mem_operand);
        break;
      case EXTERNAL_PIXEL_ELEMENTS:
      case EXTERNAL_UNSIGNED_BYTE_ELEMENTS:
        __ ldrb(result, mem_operand);
        break;
      case EXTERNAL_SHORT_ELEMENTS:
        __ ldrsh(result, mem_operand);
        break;
      case EXTERNAL_UNSIGNED_SHORT_ELEMENTS:
        __ ldrh(result, mem_operand);
        break;
      case EXTERNAL_INT_ELEMENTS:
        __ ldr(result, mem_operand);
        break;
      case EXTERNAL_UNSIGNED_INT_ELEMENTS:
        __ ldr(result, mem_operand);
        if (!instr->hydrogen()->CheckFlag(HInstruction::kUint32)) {
          __ cmphs(result, Operand(0x80000000)); // DIFF: codegen
          DeoptimizeIf(eq, instr->environment()); // DIFF: codegen
        }
        break;
      case EXTERNAL_FLOAT_ELEMENTS:
      case EXTERNAL_DOUBLE_ELEMENTS:
      case FAST_HOLEY_DOUBLE_ELEMENTS:
      case FAST_HOLEY_ELEMENTS:
      case FAST_HOLEY_SMI_ELEMENTS:
      case FAST_DOUBLE_ELEMENTS:
      case FAST_ELEMENTS:
      case FAST_SMI_ELEMENTS:
      case DICTIONARY_ELEMENTS:
      case NON_STRICT_ARGUMENTS_ELEMENTS:
        UNREACHABLE();
        break;
    }
  }
}


void LCodeGen::DoLoadKeyedFixedDoubleArray(LLoadKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadKeyedFixedArray(LLoadKeyed* instr) { // SAMEAS: arm
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


void LCodeGen::DoLoadKeyed(LLoadKeyed* instr) {
  if (instr->is_external()) {
    DoLoadKeyedExternalArray(instr);
  } else if (instr->hydrogen()->representation().IsDouble()) {
    DoLoadKeyedFixedDoubleArray(instr);
  } else {
    DoLoadKeyedFixedArray(instr);
  }
}


MemOperand LCodeGen::PrepareKeyedOperand(Register key,
                                         Register base,
                                         bool key_is_constant,
                                         int constant_key,
                                         int element_size,
                                         int shift_size,
                                         int additional_index,
                                         int additional_offset) {
  __ UNIMPLEMENTED_BREAK();
  return MemOperand(no_reg, 0);
}


void LCodeGen::DoLoadKeyedGeneric(LLoadKeyedGeneric* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoArgumentsElements(LArgumentsElements* instr) { // SAMEAS: arm
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


void LCodeGen::DoArgumentsLength(LArgumentsLength* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoWrapReceiver(LWrapReceiver* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoApplyArguments(LApplyArguments* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoPushArgument(LPushArgument* instr) { // SAMEAS: arm
  LOperand* argument = instr->value();
  if (argument->IsDoubleRegister() || argument->IsDoubleStackSlot()) {
    Abort(kDoPushArgumentNotImplementedForDoubleType);
  } else {
    Register argument_reg = EmitLoadRegister(argument, ip);
    __ push(argument_reg);
  }
}


void LCodeGen::DoDrop(LDrop* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoThisFunction(LThisFunction* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoContext(LContext* instr) { // SAMEAS: arm
  // If there is a non-return use, the context must be moved to a register.
  Register result = ToRegister(instr->result());
  if (info()->IsOptimizing()) {
    __ ldr(result, MemOperand(fp, StandardFrameConstants::kContextOffset));
  } else {
    // If there is no frame, the context must be in cp.
    ASSERT(result.is(cp));
  }
}


void LCodeGen::DoOuterContext(LOuterContext* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeclareGlobals(LDeclareGlobals* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoGlobalObject(LGlobalObject* instr) { // SAMEAS: arm
  Register context = ToRegister(instr->context());
  Register result = ToRegister(instr->result());
  __ ldr(result, ContextOperand(context, Context::GLOBAL_OBJECT_INDEX));
}


void LCodeGen::DoGlobalReceiver(LGlobalReceiver* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::CallKnownFunction(Handle<JSFunction> function, // SAMEAS: arm
                                 int formal_parameter_count,
                                 int arity,
                                 LInstruction* instr,
                                 CallKind call_kind,
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
    __ SetCallKind(r5, call_kind);
    __ ldr(ip, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
    __ Call(ip);

    // Set up deoptimization.
    RecordSafepointWithLazyDeopt(instr, RECORD_SIMPLE_SAFEPOINT);
  } else {
    SafepointGenerator generator(this, pointers, Safepoint::kLazyDeopt);
    ParameterCount count(arity);
    ParameterCount expected(formal_parameter_count);
    __ InvokeFunction(
        function, expected, count, CALL_FUNCTION, generator, call_kind);
  }
}


void LCodeGen::DoCallConstantFunction(LCallConstantFunction* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoRandom(LRandom* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathExp(LMathExp* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathLog(LMathLog* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathTan(LMathTan* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathCos(LMathCos* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoMathSin(LMathSin* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoInvokeFunction(LInvokeFunction* instr) { // SAMEAS: arm
  ASSERT(ToRegister(instr->context()).is(cp));
  ASSERT(ToRegister(instr->function()).is(r1));
  ASSERT(instr->HasPointerMap());

  Handle<JSFunction> known_function = instr->hydrogen()->known_function();
  if (known_function.is_null()) {
    LPointerMap* pointers = instr->pointer_map();
    SafepointGenerator generator(this, pointers, Safepoint::kLazyDeopt);
    ParameterCount count(instr->arity());
    __ InvokeFunction(r1, count, CALL_FUNCTION, generator, CALL_AS_METHOD);
  } else {
    CallKnownFunction(known_function,
                      instr->hydrogen()->formal_parameter_count(),
                      instr->arity(),
                      instr,
                      CALL_AS_METHOD,
                      R1_CONTAINS_TARGET);
  }
}


void LCodeGen::DoCallKeyed(LCallKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallNamed(LCallNamed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallFunction(LCallFunction* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallGlobal(LCallGlobal* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallKnownGlobal(LCallKnownGlobal* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallNew(LCallNew* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallNewArray(LCallNewArray* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCallRuntime(LCallRuntime* instr) { // SAMEAS: arm
  CallRuntime(instr->function(), instr->arity(), instr);
}


void LCodeGen::DoStoreCodeEntry(LStoreCodeEntry* instr) { // SAMEAS: arm
  Register function = ToRegister(instr->function());
  Register code_object = ToRegister(instr->code_object());
  __ add(code_object, code_object, Operand(Code::kHeaderSize - kHeapObjectTag));
  __ str(code_object,
         FieldMemOperand(function, JSFunction::kCodeEntryOffset));
}


void LCodeGen::DoInnerAllocatedObject(LInnerAllocatedObject* instr) { // SAMEAS: arm
  Register result = ToRegister(instr->result());
  Register base = ToRegister(instr->base_object());
  __ add(result, base, Operand(instr->offset()));
}


void LCodeGen::DoStoreNamedField(LStoreNamedField* instr) { // SAMEAS: arm
  Representation representation = instr->representation();

  Register object = ToRegister(instr->object());
  Register scratch = scratch0();
  HObjectAccess access = instr->hydrogen()->access();
  int offset = access.offset();

  if (access.IsExternalMemory()) {
    Register value = ToRegister(instr->value());
    MemOperand operand = MemOperand(object, offset);
    if (representation.IsByte()) {
      __ strb(value, operand);
    } else {
      __ str(value, operand);
    }
    return;
  }

  Handle<Map> transition = instr->transition();

  if (FLAG_track_heap_object_fields && representation.IsHeapObject()) {
    Register value = ToRegister(instr->value());
    if (!instr->hydrogen()->value()->type().IsHeapObject()) {
      __ SmiTst(value);
      DeoptimizeIf(eq, instr->environment());
    }
  } else if (FLAG_track_double_fields && representation.IsDouble()) {
    ASSERT(transition.is_null());
    ASSERT(access.IsInobject());
    ASSERT(!instr->hydrogen()->NeedsWriteBarrier());
    __ UNIMPLEMENTED_BREAK(); // TODO: FPU
    //DwVfpRegister value = ToDoubleRegister(instr->value());
    //__ vstr(value, FieldMemOperand(object, offset));
    return;
  }

  if (!transition.is_null()) {
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
  ASSERT(!object.is(value));
  SmiCheck check_needed =
      instr->hydrogen()->value()->IsHeapObject()
          ? OMIT_SMI_CHECK : INLINE_SMI_CHECK;
  if (access.IsInobject()) {
    MemOperand operand = FieldMemOperand(object, offset);
    if (representation.IsByte()) {
      __ strb(value, operand);
    } else {
      __ str(value, operand);
    }
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
    if (representation.IsByte()) {
      __ strb(value, operand);
    } else {
      __ str(value, operand);
    }
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


void LCodeGen::DoStoreNamedGeneric(LStoreNamedGeneric* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::ApplyCheckIf(Condition condition, LBoundsCheck* check) { // SAMEAS: arm
  if (FLAG_debug_code && check->hydrogen()->skip_check()) {
    Label done;
    __ b(NegateCondition(condition), &done);
    __ stop("eliminated bounds check failed");
    __ bind(&done);
  } else {
    DeoptimizeIf(condition, check->environment());
  }
}


void LCodeGen::DoBoundsCheck(LBoundsCheck* instr) { // SAMEAS: arm
  if (instr->hydrogen()->skip_check()) return;

  Condition condition = instr->hydrogen()->allow_equality() ? hi : hs; // DIFF: codegen
  if (instr->index()->IsConstantOperand()) {
    int constant_index =
        ToInteger32(LConstantOperand::cast(instr->index()));
    if (instr->hydrogen()->length()->representation().IsSmi()) {
      __ mov(ip, Operand(Smi::FromInt(constant_index)));
    } else {
      __ mov(ip, Operand(constant_index));
    }
    __ cmp(&condition, ip, ToRegister(instr->length())); // DIFF: codegen
  } else {
    __ cmp(&condition, ToRegister(instr->index()), ToRegister(instr->length())); // DIFF: codegen
  }
  ApplyCheckIf(condition, instr); // DIFF: codegen
}


void LCodeGen::DoStoreKeyedExternalArray(LStoreKeyed* instr) { // SAMEAS: arm
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
  int additional_offset = instr->additional_index() << element_size_shift;

  if (elements_kind == EXTERNAL_FLOAT_ELEMENTS ||
      elements_kind == EXTERNAL_DOUBLE_ELEMENTS) {
    __ UNIMPLEMENTED_BREAK(); // TODO: FPU
    // Register address = scratch0();
    // DwVfpRegister value(ToDoubleRegister(instr->value()));
    // if (key_is_constant) {
    //   if (constant_key != 0) {
    //     __ add(address, external_pointer,
    //            Operand(constant_key << element_size_shift));
    //   } else {
    //     address = external_pointer;
    //   }
    // } else {
    //   __ add(address, external_pointer, Operand(key, LSL, shift_size));
    // }
    // if (elements_kind == EXTERNAL_FLOAT_ELEMENTS) {
    //    __ vcvt_f32_f64(double_scratch0().low(), value);
    //    __ vstr(double_scratch0().low(), address, additional_offset);
    // } else {  // i.e. elements_kind == EXTERNAL_DOUBLE_ELEMENTS
    //   __ vstr(value, address, additional_offset);
    // }
  } else {
    Register value(ToRegister(instr->value()));
    MemOperand mem_operand = PrepareKeyedOperand(
        key, external_pointer, key_is_constant, constant_key,
        element_size_shift, shift_size,
        instr->additional_index(), additional_offset);
    switch (elements_kind) {
      case EXTERNAL_PIXEL_ELEMENTS:
      case EXTERNAL_BYTE_ELEMENTS:
      case EXTERNAL_UNSIGNED_BYTE_ELEMENTS:
        __ strb(value, mem_operand);
        break;
      case EXTERNAL_SHORT_ELEMENTS:
      case EXTERNAL_UNSIGNED_SHORT_ELEMENTS:
        __ strh(value, mem_operand);
        break;
      case EXTERNAL_INT_ELEMENTS:
      case EXTERNAL_UNSIGNED_INT_ELEMENTS:
        __ str(value, mem_operand);
        break;
      case EXTERNAL_FLOAT_ELEMENTS:
      case EXTERNAL_DOUBLE_ELEMENTS:
      case FAST_DOUBLE_ELEMENTS:
      case FAST_ELEMENTS:
      case FAST_SMI_ELEMENTS:
      case FAST_HOLEY_DOUBLE_ELEMENTS:
      case FAST_HOLEY_ELEMENTS:
      case FAST_HOLEY_SMI_ELEMENTS:
      case DICTIONARY_ELEMENTS:
      case NON_STRICT_ARGUMENTS_ELEMENTS:
        UNREACHABLE();
        break;
    }
  }
}


void LCodeGen::DoStoreKeyedFixedDoubleArray(LStoreKeyed* instr) { // SAMEAS: arm
  //DwVfpRegister value = ToDoubleRegister(instr->value());
  Register elements = ToRegister(instr->elements());
  Register scratch = scratch0();
  //DwVfpRegister double_scratch = double_scratch0();
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
      __ UNIMPLEMENTED_BREAK(); // TODO: FPU
    // // Force a canonical NaN.
    // if (masm()->emit_debug_code()) {
    //   __ vmrs(ip);
    //   __ tst(ip, Operand(kVFPDefaultNaNModeControlBit));
    //   __ Assert(ne, kDefaultNaNModeNotSet);
    // }
    // __ VFPCanonicalizeNaN(double_scratch, value);
    // __ vstr(double_scratch, scratch,
    //         instr->additional_index() << element_size_shift);
  } else {
    __ UNIMPLEMENTED_BREAK();
    //__ vstr(value, scratch, instr->additional_index() << element_size_shift);
  }
}


void LCodeGen::DoStoreKeyedFixedArray(LStoreKeyed* instr) { // SAMEAS: arm
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


void LCodeGen::DoStoreKeyed(LStoreKeyed* instr) { // SAMEAS: arm
  // By cases: external, fast double
  if (instr->is_external()) {
    DoStoreKeyedExternalArray(instr);
  } else if (instr->hydrogen()->value()->representation().IsDouble()) {
    DoStoreKeyedFixedDoubleArray(instr);
  } else {
    DoStoreKeyedFixedArray(instr);
  }
}


void LCodeGen::DoStoreKeyedGeneric(LStoreKeyedGeneric* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoTransitionElementsKind(LTransitionElementsKind* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoTrapAllocationMemento(LTrapAllocationMemento* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStringAdd(LStringAdd* instr) { // SAMEAS: arm
  ASSERT(ToRegister(instr->context()).is(cp));
  __ push(ToRegister(instr->left()));
  __ push(ToRegister(instr->right()));
  StringAddStub stub(instr->hydrogen()->flags());
  CallCode(stub.GetCode(isolate()), RelocInfo::CODE_TARGET, instr);
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


void LCodeGen::DoInteger32ToDouble(LInteger32ToDouble* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoInteger32ToSmi(LInteger32ToSmi* instr) { // SAMEAS: arm
  LOperand* input = instr->value();
  LOperand* output = instr->result();
  __ SmiTag(ToRegister(output), ToRegister(input), SetT); // DIFF: codegen
  if (!instr->hydrogen()->value()->HasRange() ||
      !instr->hydrogen()->value()->range()->IsInSmiRange()) {
    DeoptimizeIf(t, instr->environment()); // DIFF: codegen
  }
}


void LCodeGen::DoUint32ToDouble(LUint32ToDouble* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoUint32ToSmi(LUint32ToSmi* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoNumberTagI(LNumberTagI* instr) {
  class DeferredNumberTagI V8_FINAL : public LDeferredCode {
   public:
    DeferredNumberTagI(LCodeGen* codegen, LNumberTagI* instr)
        : LDeferredCode(codegen), instr_(instr) { }
    virtual void Generate() V8_OVERRIDE {
      codegen()->DoDeferredNumberTagI(instr_,
                                      instr_->value(),
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


void LCodeGen::DoNumberTagU(LNumberTagU* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredNumberTagI(LInstruction* instr, // SAMEAS: arm
                                    LOperand* value,
                                    IntegerSignedness signedness) {
  Label slow;
  Register src = ToRegister(value);
  Register dst = ToRegister(instr->result());
  DwVfpRegister dbl_scratch = double_scratch0();

  // Preserve the value of all registers.
  PushSafepointRegistersScope scope(this, Safepoint::kWithRegisters);

  Label done;
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
    __ LoadRoot(scratch0(), Heap::kHeapNumberMapRootIndex);
    __ AllocateHeapNumber(r5, r3, r4, scratch0(), &slow, DONT_TAG_RESULT);
    __ Move(dst, r5);
    __ b(&done);
  }

  // Slow case: Call the runtime system to do the number allocation.
  __ bind(&slow);

  // TODO(3095996): Put a valid pointer value in the stack slot where the result
  // register is stored, as this register is in the pointer map, but contains an
  // integer value.
  __ mov(ip, Operand::Zero());
  __ StoreToSafepointRegisterSlot(ip, dst);
  // NumberTagI and NumberTagD use the context from the frame, rather than
  // the environment's HContext or HInlinedContext value.
  // They only call Runtime::kAllocateHeapNumber.
  // The corresponding HChange instructions are added in a phase that does
  // not have easy access to the local context.
  __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  __ CallRuntimeSaveDoubles(Runtime::kAllocateHeapNumber);
  RecordSafepointWithRegisters(
      instr->pointer_map(), 0, Safepoint::kNoLazyDeopt);
  __ Move(dst, r0);
  __ sub(dst, dst, Operand(kHeapObjectTag));

  // Done. Put the value in dbl_scratch into the value of the allocated heap
  // number.
  __ bind(&done);
  __ vstr(dbl_scratch, dst, HeapNumber::kValueOffset);
  __ add(dst, dst, Operand(kHeapObjectTag));
  __ StoreToSafepointRegisterSlot(dst, dst);
}


void LCodeGen::DoNumberTagD(LNumberTagD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredNumberTagD(LNumberTagD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoSmiTag(LSmiTag* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoSmiUntag(LSmiUntag* instr) { // SAMEAS: arm
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


void LCodeGen::EmitNumberUntagD(Register input_reg, // SAMEAS: arm
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


void LCodeGen::DoDeferredTaggedToI(LTaggedToI* instr) { // SAMEAS: arm
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


void LCodeGen::DoTaggedToI(LTaggedToI* instr) { // SAMEAS: arm
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


void LCodeGen::DoNumberUntagD(LNumberUntagD* instr) { // SAMEAS: arm
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


void LCodeGen::DoDoubleToI(LDoubleToI* instr) {
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


void LCodeGen::DoDoubleToSmi(LDoubleToSmi* instr) { // SAMEAS: arm
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


void LCodeGen::DoCheckSmi(LCheckSmi* instr) { // SAMEAS: arm
  LOperand* input = instr->value();
  __ SmiTst(ToRegister(input));
  DeoptimizeIf(ne, instr->environment());
}


void LCodeGen::DoCheckNonSmi(LCheckNonSmi* instr) { // SAMEAS: arm
  if (!instr->hydrogen()->value()->IsHeapObject()) {
    LOperand* input = instr->value();
    __ SmiTst(ToRegister(input));
    DeoptimizeIf(eq, instr->environment());
  }
}


void LCodeGen::DoCheckInstanceType(LCheckInstanceType* instr) { // SAMEAS: arm
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


void LCodeGen::DoCheckValue(LCheckValue* instr) { // SAMEAS: arm
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


void LCodeGen::DoDeferredInstanceMigration(LCheckMaps* instr, Register object) { // SAMEAS: arm
  {
    PushSafepointRegistersScope scope(this, Safepoint::kWithRegisters);
    __ push(object);
    __ mov(cp, Operand::Zero());
    __ CallRuntimeSaveDoubles(Runtime::kMigrateInstance);
    RecordSafepointWithRegisters(
        instr->pointer_map(), 1, Safepoint::kNoLazyDeopt);
    __ StoreToSafepointRegisterSlot(r0, scratch0());
  }
  __ tst(scratch0(), Operand(kSmiTagMask));
  DeoptimizeIf(eq, instr->environment());
}


void LCodeGen::DoCheckMaps(LCheckMaps* instr) { // SAMEAS: arm
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

  if (instr->hydrogen()->CanOmitMapChecks()) return;
  Register map_reg = scratch0();

  LOperand* input = instr->value();
  ASSERT(input->IsRegister());
  Register reg = ToRegister(input);

  __ ldr(map_reg, FieldMemOperand(reg, HeapObject::kMapOffset));

  DeferredCheckMaps* deferred = NULL;
  if (instr->hydrogen()->has_migration_target()) {
    deferred = new(zone()) DeferredCheckMaps(this, instr, reg);
    __ bind(deferred->check_maps());
  }

  UniqueSet<Map> map_set = instr->hydrogen()->map_set();
  Label success;
  for (int i = 0; i < map_set.size() - 1; i++) {
    Handle<Map> map = map_set.at(i).handle();
    __ CompareMap(map_reg, map, &success);
    __ b(eq, &success);
  }

  Handle<Map> map = map_set.at(map_set.size() - 1).handle();
  __ CompareMap(map_reg, map, &success);
  if (instr->hydrogen()->has_migration_target()) {
    __ b(ne, deferred->entry());
  } else {
    DeoptimizeIf(ne, instr->environment());
  }

  __ bind(&success);
}


void LCodeGen::DoClampDToUint8(LClampDToUint8* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoClampIToUint8(LClampIToUint8* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoClampTToUint8(LClampTToUint8* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoAllocate(LAllocate* instr) { // SAMEAS: arm
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
    __ Allocate(size, result, scratch, scratch2, deferred->entry(), flags);
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


void LCodeGen::DoDeferredAllocate(LAllocate* instr) { // SAMEAS: arm
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
    __ Push(Smi::FromInt(size));
  }

  if (instr->hydrogen()->IsOldPointerSpaceAllocation()) {
    ASSERT(!instr->hydrogen()->IsOldDataSpaceAllocation());
    ASSERT(!instr->hydrogen()->IsNewSpaceAllocation());
    CallRuntimeFromDeferred(Runtime::kAllocateInOldPointerSpace, 1, instr,
                            instr->context());
  } else if (instr->hydrogen()->IsOldDataSpaceAllocation()) {
    ASSERT(!instr->hydrogen()->IsNewSpaceAllocation());
    CallRuntimeFromDeferred(Runtime::kAllocateInOldDataSpace, 1, instr,
                            instr->context());
  } else {
    CallRuntimeFromDeferred(Runtime::kAllocateInNewSpace, 1, instr,
                            instr->context());
  }
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


void LCodeGen::EnsureSpaceForLazyDeopt(int space_needed) { // SAMEAS: arm
  if (info()->IsStub()) return;
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


void LCodeGen::DoLazyBailout(LLazyBailout* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeoptimize(LDeoptimize* instr) {
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


void LCodeGen::DoDummy(LDummy* instr) {  // SAMEAS: arm
  // Nothing to see here, move on!
}


void LCodeGen::DoDummyUse(LDummyUse* instr) { // SAMEAS: arm
  // Nothing to see here, move on!
}


void LCodeGen::DoDeferredStackCheck(LStackCheck* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStackCheck(LStackCheck* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoOsrEntry(LOsrEntry* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoLoadFieldByIndex(LLoadFieldByIndex* instr) {
  __ UNIMPLEMENTED_BREAK();
}


#undef __

} }  // namespace v8::internal
