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

#if defined(V8_TARGET_ARCH_SH4)

#include "sh4/lithium-codegen-sh4.h"
#include "code-stubs.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {


class SafepointGenerator : public CallWrapper {
 public:
  SafepointGenerator(LCodeGen* codegen,
                     LPointerMap* pointers,
                     int deoptimization_index)
      : codegen_(codegen),
        pointers_(pointers),
        deoptimization_index_(deoptimization_index) { }
  virtual ~SafepointGenerator() { }

  virtual void BeforeCall(int call_size) const {
    UNIMPLEMENTED();
  }

  virtual void AfterCall() const {
    codegen_->RecordSafepoint(pointers_, deoptimization_index_);
  }

 private:
  LCodeGen* codegen_;
  LPointerMap* pointers_;
  int deoptimization_index_;
};


bool LCodeGen::GenerateCode() {
  UNIMPLEMENTED();
}


void LCodeGen::FinishCode(Handle<Code> code) {
  UNIMPLEMENTED();
}


void LCodeGen::Abort(const char* format, ...) {
  UNIMPLEMENTED();
}


void LCodeGen::Comment(const char* format, ...) {
  UNIMPLEMENTED();
}


bool LCodeGen::GeneratePrologue() {
  UNIMPLEMENTED();
}


bool LCodeGen::GenerateBody() {
  UNIMPLEMENTED();
}


LInstruction* LCodeGen::GetNextInstruction() {
  UNIMPLEMENTED();
}


bool LCodeGen::GenerateDeferredCode() {
  UNIMPLEMENTED();
}


bool LCodeGen::GenerateDeoptJumpTable() {
  UNIMPLEMENTED();
}


bool LCodeGen::GenerateSafepointTable() {
  UNIMPLEMENTED();
}


Register LCodeGen::ToRegister(int index) const {
  UNIMPLEMENTED();
}


DoubleRegister LCodeGen::ToDoubleRegister(int index) const {
  UNIMPLEMENTED();
}


Register LCodeGen::ToRegister(LOperand* op) const {
  UNIMPLEMENTED();
}


Register LCodeGen::EmitLoadRegister(LOperand* op, Register scratch) {
  UNIMPLEMENTED();
}


DoubleRegister LCodeGen::ToDoubleRegister(LOperand* op) const {
  UNIMPLEMENTED();
}


DoubleRegister LCodeGen::EmitLoadDoubleRegister(LOperand* op,
                                                SwVfpRegister flt_scratch,
                                                DoubleRegister dbl_scratch) {
  UNIMPLEMENTED();
}


int LCodeGen::ToInteger32(LConstantOperand* op) const {
  UNIMPLEMENTED();
}


Operand LCodeGen::ToOperand(LOperand* op) {
  UNIMPLEMENTED();
}


MemOperand LCodeGen::ToMemOperand(LOperand* op) const {
  UNIMPLEMENTED();
}


MemOperand LCodeGen::ToHighMemOperand(LOperand* op) const {
  UNIMPLEMENTED();
}


void LCodeGen::WriteTranslation(LEnvironment* environment,
                                Translation* translation) {
  UNIMPLEMENTED();
}


void LCodeGen::AddToTranslation(Translation* translation,
                                LOperand* op,
                                bool is_tagged) {
  UNIMPLEMENTED();
}


void LCodeGen::CallCode(Handle<Code> code,
                        RelocInfo::Mode mode,
                        LInstruction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::CallCodeGeneric(Handle<Code> code,
                               RelocInfo::Mode mode,
                               LInstruction* instr,
                               SafepointMode safepoint_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::CallRuntime(const Runtime::Function* function,
                           int num_arguments,
                           LInstruction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::CallRuntimeFromDeferred(Runtime::FunctionId id,
                                       int argc,
                                       LInstruction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::RegisterLazyDeoptimization(LInstruction* instr,
                                          SafepointMode safepoint_mode) {
  UNIMPLEMENTED();
}


void LCodeGen::RegisterEnvironmentForDeoptimization(LEnvironment* environment) {
  UNIMPLEMENTED();
}


void LCodeGen::DeoptimizeIf(Condition cc, LEnvironment* environment) {
  UNIMPLEMENTED();
}


void LCodeGen::PopulateDeoptimizationData(Handle<Code> code) {
  UNIMPLEMENTED();
}


int LCodeGen::DefineDeoptimizationLiteral(Handle<Object> literal) {
  UNIMPLEMENTED();
}


void LCodeGen::PopulateDeoptimizationLiteralsWithInlinedFunctions() {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepoint(
    LPointerMap* pointers,
    Safepoint::Kind kind,
    int arguments,
    int deoptimization_index) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepoint(LPointerMap* pointers,
                               int deoptimization_index) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepoint(int deoptimization_index) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepointWithRegisters(LPointerMap* pointers,
                                            int arguments,
                                            int deoptimization_index) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordSafepointWithRegistersAndDoubles(
    LPointerMap* pointers,
    int arguments,
    int deoptimization_index) {
  UNIMPLEMENTED();
}


void LCodeGen::RecordPosition(int position) {
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


void LCodeGen::DoDivI(LDivI* instr) {
  UNIMPLEMENTED();
}


template<int T>
void LCodeGen::DoDeferredBinaryOpStub(LTemplateInstruction<1, 2, T>* instr,
                                      Token::Value op) {
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


void LCodeGen::DoConstantI(LConstantI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantD(LConstantD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantT(LConstantT* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoJSArrayLength(LJSArrayLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoFixedArrayBaseLength(LFixedArrayBaseLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoElementsKind(LElementsKind* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoValueOf(LValueOf* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBitNotI(LBitNotI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoThrow(LThrow* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoAddI(LAddI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArithmeticD(LArithmeticD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArithmeticT(LArithmeticT* instr) {
  UNIMPLEMENTED();
}


int LCodeGen::GetNextEmittedBlock(int block) {
  UNIMPLEMENTED();
}


void LCodeGen::EmitBranch(int left_block, int right_block, Condition cc) {
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
}


void LCodeGen::EmitCmpI(LOperand* left, LOperand* right) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpIDAndBranch(LCmpIDAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpObjectEqAndBranch(LCmpObjectEqAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpConstantEqAndBranch(LCmpConstantEqAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsNullAndBranch(LIsNullAndBranch* instr) {
  UNIMPLEMENTED();
}


Condition LCodeGen::EmitIsObject(Register input,
                                 Register temp1,
                                 Label* is_not_object,
                                 Label* is_object) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsObjectAndBranch(LIsObjectAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsSmiAndBranch(LIsSmiAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsUndetectableAndBranch(LIsUndetectableAndBranch* instr) {
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
// the temp registers, but not the input.  Only input and temp2 may alias.
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


void LCodeGen::DoDeferredLInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr,
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


void LCodeGen::EmitLoadFieldOrConstantFunction(Register result,
                                               Register object,
                                               Handle<Map> type,
                                               Handle<String> name) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadNamedFieldPolymorphic(LLoadNamedFieldPolymorphic* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadNamedGeneric(LLoadNamedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadFunctionPrototype(LLoadFunctionPrototype* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadElements(LLoadElements* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadExternalArrayPointer(
    LLoadExternalArrayPointer* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoAccessArgumentsAt(LAccessArgumentsAt* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedFastElement(LLoadKeyedFastElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedFastDoubleElement(
    LLoadKeyedFastDoubleElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedSpecializedArrayElement(
    LLoadKeyedSpecializedArrayElement* instr) {
  UNIMPLEMENTED();
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


void LCodeGen::DoApplyArguments(LApplyArguments* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoPushArgument(LPushArgument* instr) {
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


void LCodeGen::DoGlobalObject(LGlobalObject* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGlobalReceiver(LGlobalReceiver* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::CallKnownFunction(Handle<JSFunction> function,
                                 int arity,
                                 LInstruction* instr,
                                 CallKind call_kind) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallConstantFunction(LCallConstantFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredMathAbsTaggedHeapNumber(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::EmitIntegerMathAbs(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathAbs(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathFloor(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathRound(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathSqrt(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathPowHalf(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoPower(LPower* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathLog(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathCos(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMathSin(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoUnaryMathOperation(LUnaryMathOperation* instr) {
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


void LCodeGen::DoCallRuntime(LCallRuntime* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreNamedField(LStoreNamedField* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreNamedGeneric(LStoreNamedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBoundsCheck(LBoundsCheck* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedFastElement(LStoreKeyedFastElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedFastDoubleElement(
    LStoreKeyedFastDoubleElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedSpecializedArrayElement(
    LStoreKeyedSpecializedArrayElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedGeneric(LStoreKeyedGeneric* instr) {
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


void LCodeGen::DoStringLength(LStringLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInteger32ToDouble(LInteger32ToDouble* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoNumberTagI(LNumberTagI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeferredNumberTagI(LNumberTagI* instr) {
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
                                DoubleRegister result_reg,
                                bool deoptimize_on_undefined,
                                LEnvironment* env) {
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


void LCodeGen::DoCheckSmi(LCheckSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckNonSmi(LCheckNonSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckInstanceType(LCheckInstanceType* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckFunction(LCheckFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckMap(LCheckMap* instr) {
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


void LCodeGen::LoadHeapObject(Register result,
                              Handle<HeapObject> object) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckPrototypeMaps(LCheckPrototypeMaps* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArrayLiteral(LArrayLiteral* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoObjectLiteral(LObjectLiteral* instr) {
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
}


void LCodeGen::DoIsConstructCallAndBranch(LIsConstructCallAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::EmitIsConstructCall(Register temp1, Register temp2) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLazyBailout(LLazyBailout* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeoptimize(LDeoptimize* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeleteProperty(LDeleteProperty* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIn(LIn* instr) {
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


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
