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


bool LCodeGen::GeneratePrologue() { // SAMEAS: arm, DIFF: codegen
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
      __ bt_near(&skip);
      __ LoadRoot(r2, Heap::kUndefinedValueRootIndex);
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
      __ cmpeq(r0, sp); // DIFF: codegen
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
      __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::GenerateOsrPrologue() {
  __ UNIMPLEMENTED_BREAK();
}


bool LCodeGen::GenerateDeferredCode() {
  __ UNIMPLEMENTED_BREAK(); // Fake succesful generation

  // Force constant pool emission at the end of the deferred code to make
  // sure that no constant pools are emitted after.
  masm()->CheckConstPool(true, false);

  return !is_aborted();
}


bool LCodeGen::GenerateDeoptJumpTable() {
  __ UNIMPLEMENTED_BREAK(); // Fake succesful generation

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


Register LCodeGen::ToRegister(int index) const {
  __ UNIMPLEMENTED_BREAK();
  return no_reg;
}


DwVfpRegister LCodeGen::ToDoubleRegister(int index) const {
  __ UNIMPLEMENTED_BREAK();
  return no_dreg;
}


Register LCodeGen::ToRegister(LOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return no_reg;
}


Register LCodeGen::EmitLoadRegister(LOperand* op, Register scratch) {
  __ UNIMPLEMENTED_BREAK();
  return no_reg;
}


DwVfpRegister LCodeGen::ToDoubleRegister(LOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return no_dreg;
}


DwVfpRegister LCodeGen::EmitLoadDoubleRegister(LOperand* op,
                                               SwVfpRegister flt_scratch,
                                               DwVfpRegister dbl_scratch) {
  __ UNIMPLEMENTED_BREAK();
  return no_dreg;
}


Handle<Object> LCodeGen::ToHandle(LConstantOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return Handle<Object>();
}


bool LCodeGen::IsInteger32(LConstantOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return false;
}


bool LCodeGen::IsSmi(LConstantOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return false;
}


int32_t LCodeGen::ToInteger32(LConstantOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return -1;
}


int32_t LCodeGen::ToRepresentation(LConstantOperand* op,
                                   const Representation& r) const {
  __ UNIMPLEMENTED_BREAK();
  return -1;
}


Smi* LCodeGen::ToSmi(LConstantOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return NULL;
}


double LCodeGen::ToDouble(LConstantOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return 0.0;
}


Operand LCodeGen::ToOperand(LOperand* op) {
  __ UNIMPLEMENTED_BREAK();
  return Operand::Zero();
}


MemOperand LCodeGen::ToMemOperand(LOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return MemOperand(no_reg, 0);
}


MemOperand LCodeGen::ToHighMemOperand(LOperand* op) const {
  __ UNIMPLEMENTED_BREAK();
  return MemOperand(no_reg, 0);
}


void LCodeGen::WriteTranslation(LEnvironment* environment,
                                Translation* translation) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::AddToTranslation(LEnvironment* environment,
                                Translation* translation,
                                LOperand* op,
                                bool is_tagged,
                                bool is_uint32,
                                int* object_index_pointer,
                                int* dematerialized_index_pointer) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::CallCode(Handle<Code> code,
                        RelocInfo::Mode mode,
                        LInstruction* instr,
                        TargetAddressStorageMode storage_mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::CallCodeGeneric(Handle<Code> code,
                               RelocInfo::Mode mode,
                               LInstruction* instr,
                               SafepointMode safepoint_mode,
                               TargetAddressStorageMode storage_mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::CallRuntime(const Runtime::Function* function,
                           int num_arguments,
                           LInstruction* instr,
                           SaveFPRegsMode save_doubles) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::CallRuntimeFromDeferred(Runtime::FunctionId id,
                                       int argc,
                                       LInstruction* instr,
                                       LOperand* context) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::RegisterEnvironmentForDeoptimization(LEnvironment* environment,
                                                    Safepoint::DeoptMode mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DeoptimizeIf(Condition condition,
                            LEnvironment* environment,
                            Deoptimizer::BailoutType bailout_type) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DeoptimizeIf(Condition condition,
                            LEnvironment* environment) {
  __ UNIMPLEMENTED_BREAK();
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

void LCodeGen::PopulateDeoptimizationData(Handle<Code> code) {
  __ UNIMPLEMENTED_BREAK();
}


int LCodeGen::DefineDeoptimizationLiteral(Handle<Object> literal) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::RecordSafepointWithLazyDeopt(
    LInstruction* instr, SafepointMode safepoint_mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::RecordSafepoint(
    LPointerMap* pointers,
    Safepoint::Kind kind,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::RecordSafepoint(LPointerMap* pointers,
                               Safepoint::DeoptMode deopt_mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::RecordSafepoint(Safepoint::DeoptMode deopt_mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::RecordSafepointWithRegisters(LPointerMap* pointers,
                                            int arguments,
                                            Safepoint::DeoptMode deopt_mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::RecordSafepointWithRegistersAndDoubles(
    LPointerMap* pointers,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::RecordAndWritePosition(int position) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLabel(LLabel* label) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoParallelMove(LParallelMove* move) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoGap(LGap* gap) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoInstructionGap(LInstructionGap* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoParameter(LParameter* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoMulI(LMulI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoBitI(LBitI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoShiftI(LShiftI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoSubI(LSubI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoRSubI(LRSubI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoConstantI(LConstantI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoConstantS(LConstantS* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoConstantD(LConstantD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoConstantE(LConstantE* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoConstantT(LConstantT* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoAddI(LAddI* instr) {
  __ UNIMPLEMENTED_BREAK();
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
void LCodeGen::EmitBranch(InstrType instr, Condition condition) {
  __ UNIMPLEMENTED_BREAK();
}


template<class InstrType>
void LCodeGen::EmitFalseBranch(InstrType instr, Condition condition) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDebugBreak(LDebugBreak* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoBranch(LBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::EmitGoto(int block) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoGoto(LGoto* instr) {
  __ UNIMPLEMENTED_BREAK();
}


Condition LCodeGen::TokenToCondition(Token::Value op, bool is_unsigned) {
  UNIMPLEMENTED();
  return ne;
}


void LCodeGen::DoCompareNumericAndBranch(LCompareNumericAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCmpObjectEqAndBranch(LCmpObjectEqAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
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


Condition LCodeGen::EmitIsString(Register input,
                                 Register temp1,
                                 Label* is_not_string,
                                 SmiCheck check_needed = INLINE_SMI_CHECK) {
  __ UNIMPLEMENTED_BREAK();
  return ne;
}


void LCodeGen::DoIsStringAndBranch(LIsStringAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoIsSmiAndBranch(LIsSmiAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoIsUndetectableAndBranch(LIsUndetectableAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoCmpMapAndBranch(LCmpMapAndBranch* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoReturn(LReturn* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoLoadNamedField(LLoadNamedField* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadNamedGeneric(LLoadNamedGeneric* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadFunctionPrototype(LLoadFunctionPrototype* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadRoot(LLoadRoot* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadExternalArrayPointer(
    LLoadExternalArrayPointer* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoAccessArgumentsAt(LAccessArgumentsAt* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadKeyedExternalArray(LLoadKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadKeyedFixedDoubleArray(LLoadKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadKeyedFixedArray(LLoadKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLoadKeyed(LLoadKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoArgumentsElements(LArgumentsElements* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoPushArgument(LPushArgument* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDrop(LDrop* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoThisFunction(LThisFunction* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoContext(LContext* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoOuterContext(LOuterContext* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeclareGlobals(LDeclareGlobals* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoGlobalObject(LGlobalObject* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoGlobalReceiver(LGlobalReceiver* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::CallKnownFunction(Handle<JSFunction> function,
                                 int formal_parameter_count,
                                 int arity,
                                 LInstruction* instr,
                                 CallKind call_kind,
                                 R1State r1_state) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoInvokeFunction(LInvokeFunction* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoCallRuntime(LCallRuntime* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreCodeEntry(LStoreCodeEntry* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoInnerAllocatedObject(LInnerAllocatedObject* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreNamedField(LStoreNamedField* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreNamedGeneric(LStoreNamedGeneric* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::ApplyCheckIf(Condition condition, LBoundsCheck* check) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoBoundsCheck(LBoundsCheck* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreKeyedExternalArray(LStoreKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreKeyedFixedDoubleArray(LStoreKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreKeyedFixedArray(LStoreKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoStoreKeyed(LStoreKeyed* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoStringAdd(LStringAdd* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoInteger32ToSmi(LInteger32ToSmi* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoUint32ToDouble(LUint32ToDouble* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoUint32ToSmi(LUint32ToSmi* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoNumberTagI(LNumberTagI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoNumberTagU(LNumberTagU* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredNumberTagI(LInstruction* instr,
                                    LOperand* value,
                                    IntegerSignedness signedness) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoSmiUntag(LSmiUntag* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::EmitNumberUntagD(Register input_reg,
                                DwVfpRegister result_reg,
                                bool can_convert_undefined_to_nan,
                                bool deoptimize_on_minus_zero,
                                LEnvironment* env,
                                NumberUntagDMode mode) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredTaggedToI(LTaggedToI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoTaggedToI(LTaggedToI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoNumberUntagD(LNumberUntagD* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDoubleToI(LDoubleToI* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDoubleToSmi(LDoubleToSmi* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCheckSmi(LCheckSmi* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCheckNonSmi(LCheckNonSmi* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCheckInstanceType(LCheckInstanceType* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCheckValue(LCheckValue* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredInstanceMigration(LCheckMaps* instr, Register object) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoCheckMaps(LCheckMaps* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::DoAllocate(LAllocate* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeferredAllocate(LAllocate* instr) {
  __ UNIMPLEMENTED_BREAK();
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


void LCodeGen::EnsureSpaceForLazyDeopt(int space_needed) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoLazyBailout(LLazyBailout* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDeoptimize(LDeoptimize* instr) {
  __ UNIMPLEMENTED_BREAK();
}


void LCodeGen::DoDummyUse(LDummyUse* instr) {
  __ UNIMPLEMENTED_BREAK();
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
