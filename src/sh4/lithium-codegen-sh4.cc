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
  UNIMPLEMENTED();
  return true;
}


void LCodeGen::FinishCode(Handle<Code> code) {
  UNIMPLEMENTED();
}


void LCodeGen::Abort(BailoutReason reason) {
  UNIMPLEMENTED();
}


bool LCodeGen::GeneratePrologue() {
  UNIMPLEMENTED();
  return false;
}


void LCodeGen::GenerateOsrPrologue() {
  UNIMPLEMENTED();
}


bool LCodeGen::GenerateDeferredCode() {
  UNIMPLEMENTED();
  return false;
}


bool LCodeGen::GenerateDeoptJumpTable() {
  UNIMPLEMENTED();
  return false;
}


bool LCodeGen::GenerateSafepointTable() {
  UNIMPLEMENTED();
  return false;
}


Register LCodeGen::ToRegister(int index) const {
  UNIMPLEMENTED();
  return no_reg;
}


DwVfpRegister LCodeGen::ToDoubleRegister(int index) const {
  UNIMPLEMENTED();
  return no_dreg;
}


Register LCodeGen::ToRegister(LOperand* op) const {
  UNIMPLEMENTED();
  return no_reg;
}


Register LCodeGen::EmitLoadRegister(LOperand* op, Register scratch) {
  UNIMPLEMENTED();
  return no_reg;
}


DwVfpRegister LCodeGen::ToDoubleRegister(LOperand* op) const {
  UNIMPLEMENTED();
  return no_dreg;
}


DwVfpRegister LCodeGen::EmitLoadDoubleRegister(LOperand* op,
                                               SwVfpRegister flt_scratch,
                                               DwVfpRegister dbl_scratch) {
  UNIMPLEMENTED();
  return no_dreg;
}


Handle<Object> LCodeGen::ToHandle(LConstantOperand* op) const {
  UNIMPLEMENTED();
  return Handle<Object>();
}


bool LCodeGen::IsInteger32(LConstantOperand* op) const {
  UNIMPLEMENTED();
  return false;
}


bool LCodeGen::IsSmi(LConstantOperand* op) const {
  UNIMPLEMENTED();
  return false;
}


int32_t LCodeGen::ToInteger32(LConstantOperand* op) const {
  UNIMPLEMENTED();
  return -1;
}


int32_t LCodeGen::ToRepresentation(LConstantOperand* op,
                                   const Representation& r) const {
  UNIMPLEMENTED();
  return -1;
}


Smi* LCodeGen::ToSmi(LConstantOperand* op) const {
  UNIMPLEMENTED();
  return NULL;
}


double LCodeGen::ToDouble(LConstantOperand* op) const {
  UNIMPLEMENTED();
  return 0.0;
}


Operand LCodeGen::ToOperand(LOperand* op) {
  UNIMPLEMENTED();
  return Operand::Zero();
}


MemOperand LCodeGen::ToMemOperand(LOperand* op) const {
  UNIMPLEMENTED();
  return MemOperand(no_reg, 0);
}


MemOperand LCodeGen::ToHighMemOperand(LOperand* op) const {
  UNIMPLEMENTED();
  return MemOperand(no_reg, 0);
}


void LCodeGen::WriteTranslation(LEnvironment* environment,
                                Translation* translation) {
  UNIMPLEMENTED();
}


void LCodeGen::AddToTranslation(LEnvironment* environment,
                                Translation* translation,
                                LOperand* op,
                                bool is_tagged,
                                bool is_uint32,
                                int* object_index_pointer,
                                int* dematerialized_index_pointer) {
  UNIMPLEMENTED();
}


void LCodeGen::CallCode(Handle<Code> code,
                        RelocInfo::Mode mode,
                        LInstruction* instr,
                        TargetAddressStorageMode storage_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::CallCodeGeneric(Handle<Code> code,
                               RelocInfo::Mode mode,
                               LInstruction* instr,
                               SafepointMode safepoint_mode,
                               TargetAddressStorageMode storage_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::CallRuntime(const Runtime::Function* function,
                           int num_arguments,
                           LInstruction* instr,
                           SaveFPRegsMode save_doubles) {
  UNIMPLEMENTED();
}


void LCodeGen::CallRuntimeFromDeferred(Runtime::FunctionId id,
                                       int argc,
                                       LInstruction* instr,
                                       LOperand* context) {
  UNIMPLEMENTED();
}


void LCodeGen::RegisterEnvironmentForDeoptimization(LEnvironment* environment,
                                                    Safepoint::DeoptMode mode) {
  UNIMPLEMENTED();
}


void LCodeGen::DeoptimizeIf(Condition condition,
                            LEnvironment* environment,
                            Deoptimizer::BailoutType bailout_type) {
  UNIMPLEMENTED();
}


void LCodeGen::DeoptimizeIf(Condition condition,
                            LEnvironment* environment) {
  UNIMPLEMENTED();
}


void LCodeGen::PopulateDeoptimizationData(Handle<Code> code) {
  UNIMPLEMENTED();
}


int LCodeGen::DefineDeoptimizationLiteral(Handle<Object> literal) {
  UNIMPLEMENTED();
  return -1;
}


void LCodeGen::PopulateDeoptimizationLiteralsWithInlinedFunctions() {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepointWithLazyDeopt(
    LInstruction* instr, SafepointMode safepoint_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepoint(
    LPointerMap* pointers,
    Safepoint::Kind kind,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepoint(LPointerMap* pointers,
                               Safepoint::DeoptMode deopt_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepoint(Safepoint::DeoptMode deopt_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepointWithRegisters(LPointerMap* pointers,
                                            int arguments,
                                            Safepoint::DeoptMode deopt_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepointWithRegistersAndDoubles(
    LPointerMap* pointers,
    int arguments,
    Safepoint::DeoptMode deopt_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordAndWritePosition(int position) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLabel(LLabel* label) {
  UNIMPLEMENTED();
}


void LCodeGen::DoParallelMove(LParallelMove* move) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGap(LGap* gap) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInstructionGap(LInstructionGap* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoParameter(LParameter* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallStub(LCallStub* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoUnknownOSRValue(LUnknownOSRValue* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoModI(LModI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::EmitSignedIntegerDivisionByConstant(
    Register result,
    Register dividend,
    int32_t divisor,
    Register remainder,
    Register scratch,
    LEnvironment* environment) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDivI(LDivI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMultiplyAddD(LMultiplyAddD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMultiplySubD(LMultiplySubD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathFloorOfDiv(LMathFloorOfDiv* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMulI(LMulI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBitI(LBitI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoShiftI(LShiftI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoSubI(LSubI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoRSubI(LRSubI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantI(LConstantI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantS(LConstantS* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantD(LConstantD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantE(LConstantE* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantT(LConstantT* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMapEnumLength(LMapEnumLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoElementsKind(LElementsKind* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoValueOf(LValueOf* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDateField(LDateField* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoSeqStringSetChar(LSeqStringSetChar* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoThrow(LThrow* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoAddI(LAddI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathMinMax(LMathMinMax* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArithmeticD(LArithmeticD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArithmeticT(LArithmeticT* instr) {
  UNIMPLEMENTED();
}


template<class InstrType>
void LCodeGen::EmitBranch(InstrType instr, Condition condition) {
  UNIMPLEMENTED();
}


template<class InstrType>
void LCodeGen::EmitFalseBranch(InstrType instr, Condition condition) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDebugBreak(LDebugBreak* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBranch(LBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::EmitGoto(int block) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGoto(LGoto* instr) {
  UNIMPLEMENTED();
}


Condition LCodeGen::TokenToCondition(Token::Value op, bool is_unsigned) {
  UNIMPLEMENTED();
  return ne;
}


void LCodeGen::DoCompareNumericAndBranch(LCompareNumericAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpObjectEqAndBranch(LCmpObjectEqAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpHoleAndBranch(LCmpHoleAndBranch* instr) {
  UNIMPLEMENTED();
}


Condition LCodeGen::EmitIsObject(Register input,
                                 Register temp1,
                                 Label* is_not_object,
                                 Label* is_object) {
  UNIMPLEMENTED();
  return ne;
}


void LCodeGen::DoIsObjectAndBranch(LIsObjectAndBranch* instr) {
  UNIMPLEMENTED();
}


Condition LCodeGen::EmitIsString(Register input,
                                 Register temp1,
                                 Label* is_not_string,
                                 SmiCheck check_needed = INLINE_SMI_CHECK) {
  UNIMPLEMENTED();
  return ne;
}


void LCodeGen::DoIsStringAndBranch(LIsStringAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsSmiAndBranch(LIsSmiAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsUndetectableAndBranch(LIsUndetectableAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStringCompareAndBranch(LStringCompareAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoHasInstanceTypeAndBranch(LHasInstanceTypeAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGetCachedArrayIndex(LGetCachedArrayIndex* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoHasCachedArrayIndexAndBranch(
    LHasCachedArrayIndexAndBranch* instr) {
  UNIMPLEMENTED();
}


// Branches to a label or falls through with the answer in flags.  Trashes
// the temp registers, but not the input.
void LCodeGen::EmitClassOfTest(Label* is_true,
                               Label* is_false,
                               Handle<String>class_name,
                               Register input,
                               Register temp,
                               Register temp2) {
  UNIMPLEMENTED();
}


void LCodeGen::DoClassOfTestAndBranch(LClassOfTestAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpMapAndBranch(LCmpMapAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInstanceOf(LInstanceOf* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr,
                                               Label* map_check) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpT(LCmpT* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoReturn(LReturn* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadGlobalCell(LLoadGlobalCell* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadGlobalGeneric(LLoadGlobalGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreGlobalCell(LStoreGlobalCell* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreGlobalGeneric(LStoreGlobalGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadContextSlot(LLoadContextSlot* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreContextSlot(LStoreContextSlot* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadNamedField(LLoadNamedField* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadNamedGeneric(LLoadNamedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadFunctionPrototype(LLoadFunctionPrototype* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadRoot(LLoadRoot* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadExternalArrayPointer(
    LLoadExternalArrayPointer* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoAccessArgumentsAt(LAccessArgumentsAt* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedExternalArray(LLoadKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedFixedDoubleArray(LLoadKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedFixedArray(LLoadKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyed(LLoadKeyed* instr) {
  UNIMPLEMENTED();
}


MemOperand LCodeGen::PrepareKeyedOperand(Register key,
                                         Register base,
                                         bool key_is_constant,
                                         int constant_key,
                                         int element_size,
                                         int shift_size,
                                         int additional_index,
                                         int additional_offset) {
  UNIMPLEMENTED();
  return MemOperand(no_reg, 0);
}


void LCodeGen::DoLoadKeyedGeneric(LLoadKeyedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArgumentsElements(LArgumentsElements* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArgumentsLength(LArgumentsLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoWrapReceiver(LWrapReceiver* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoApplyArguments(LApplyArguments* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoPushArgument(LPushArgument* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDrop(LDrop* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoThisFunction(LThisFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoContext(LContext* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoOuterContext(LOuterContext* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeclareGlobals(LDeclareGlobals* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGlobalObject(LGlobalObject* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGlobalReceiver(LGlobalReceiver* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::CallKnownFunction(Handle<JSFunction> function,
                                 int formal_parameter_count,
                                 int arity,
                                 LInstruction* instr,
                                 CallKind call_kind,
                                 R1State r1_state) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallConstantFunction(LCallConstantFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredMathAbsTaggedHeapNumber(LMathAbs* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::EmitIntegerMathAbs(LMathAbs* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathAbs(LMathAbs* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathFloor(LMathFloor* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathRound(LMathRound* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathSqrt(LMathSqrt* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathPowHalf(LMathPowHalf* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoPower(LPower* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoRandom(LRandom* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathExp(LMathExp* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathLog(LMathLog* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathTan(LMathTan* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathCos(LMathCos* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathSin(LMathSin* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInvokeFunction(LInvokeFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallKeyed(LCallKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallNamed(LCallNamed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallFunction(LCallFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallGlobal(LCallGlobal* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallKnownGlobal(LCallKnownGlobal* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallNew(LCallNew* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallNewArray(LCallNewArray* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallRuntime(LCallRuntime* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreCodeEntry(LStoreCodeEntry* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInnerAllocatedObject(LInnerAllocatedObject* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreNamedField(LStoreNamedField* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreNamedGeneric(LStoreNamedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::ApplyCheckIf(Condition condition, LBoundsCheck* check) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBoundsCheck(LBoundsCheck* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedExternalArray(LStoreKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedFixedDoubleArray(LStoreKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedFixedArray(LStoreKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyed(LStoreKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedGeneric(LStoreKeyedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTransitionElementsKind(LTransitionElementsKind* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTrapAllocationMemento(LTrapAllocationMemento* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStringAdd(LStringAdd* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStringCharCodeAt(LStringCharCodeAt* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredStringCharCodeAt(LStringCharCodeAt* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStringCharFromCode(LStringCharFromCode* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredStringCharFromCode(LStringCharFromCode* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInteger32ToDouble(LInteger32ToDouble* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInteger32ToSmi(LInteger32ToSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoUint32ToDouble(LUint32ToDouble* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoUint32ToSmi(LUint32ToSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoNumberTagI(LNumberTagI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoNumberTagU(LNumberTagU* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredNumberTagI(LInstruction* instr,
                                    LOperand* value,
                                    IntegerSignedness signedness) {
  UNIMPLEMENTED();
}


void LCodeGen::DoNumberTagD(LNumberTagD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredNumberTagD(LNumberTagD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoSmiTag(LSmiTag* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoSmiUntag(LSmiUntag* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::EmitNumberUntagD(Register input_reg,
                                DwVfpRegister result_reg,
                                bool can_convert_undefined_to_nan,
                                bool deoptimize_on_minus_zero,
                                LEnvironment* env,
                                NumberUntagDMode mode) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredTaggedToI(LTaggedToI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTaggedToI(LTaggedToI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoNumberUntagD(LNumberUntagD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDoubleToI(LDoubleToI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDoubleToSmi(LDoubleToSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckSmi(LCheckSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckNonSmi(LCheckNonSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckInstanceType(LCheckInstanceType* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckValue(LCheckValue* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredInstanceMigration(LCheckMaps* instr, Register object) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckMaps(LCheckMaps* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoClampDToUint8(LClampDToUint8* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoClampIToUint8(LClampIToUint8* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoClampTToUint8(LClampTToUint8* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoAllocate(LAllocate* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredAllocate(LAllocate* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoToFastProperties(LToFastProperties* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoRegExpLiteral(LRegExpLiteral* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoFunctionLiteral(LFunctionLiteral* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTypeof(LTypeof* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTypeofIsAndBranch(LTypeofIsAndBranch* instr) {
  UNIMPLEMENTED();
}


Condition LCodeGen::EmitTypeofIs(Label* true_label,
                                 Label* false_label,
                                 Register input,
                                 Handle<String> type_name) {
  UNIMPLEMENTED();
  return ne;
}


void LCodeGen::DoIsConstructCallAndBranch(LIsConstructCallAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::EmitIsConstructCall(Register temp1, Register temp2) {
  UNIMPLEMENTED();
}


void LCodeGen::EnsureSpaceForLazyDeopt(int space_needed) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLazyBailout(LLazyBailout* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeoptimize(LDeoptimize* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDummyUse(LDummyUse* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredStackCheck(LStackCheck* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStackCheck(LStackCheck* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoOsrEntry(LOsrEntry* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoForInPrepareMap(LForInPrepareMap* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoForInCacheArray(LForInCacheArray* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckMapValue(LCheckMapValue* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadFieldByIndex(LLoadFieldByIndex* instr) {
  UNIMPLEMENTED();
}


#undef __

} }  // namespace v8::internal
