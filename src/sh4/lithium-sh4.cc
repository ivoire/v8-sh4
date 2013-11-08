// Copyright 2012 the V8 project authors. All rights reserved.
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

#include "lithium-allocator-inl.h"
#include "sh4/lithium-sh4.h"
#include "sh4/lithium-codegen-sh4.h"

namespace v8 {
namespace internal {

#define DEFINE_COMPILE(type)                            \
  void L##type::CompileToNative(LCodeGen* generator) {  \
    generator->Do##type(this);                          \
  }
LITHIUM_CONCRETE_INSTRUCTION_LIST(DEFINE_COMPILE)
#undef DEFINE_COMPILE

#ifdef DEBUG
void LInstruction::VerifyCall() {
  // Call instructions can use only fixed registers as temporaries and
  // outputs because all registers are blocked by the calling convention.
  // Inputs operands must use a fixed register or use-at-start policy or
  // a non-register policy.
  ASSERT(Output() == NULL ||
         LUnallocated::cast(Output())->HasFixedPolicy() ||
         !LUnallocated::cast(Output())->HasRegisterPolicy());
  for (UseIterator it(this); !it.Done(); it.Advance()) {
    LUnallocated* operand = LUnallocated::cast(it.Current());
    ASSERT(operand->HasFixedPolicy() ||
           operand->IsUsedAtStart());
  }
  for (TempIterator it(this); !it.Done(); it.Advance()) {
    LUnallocated* operand = LUnallocated::cast(it.Current());
    ASSERT(operand->HasFixedPolicy() ||!operand->HasRegisterPolicy());
  }
}
#endif


void LInstruction::PrintTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LInstruction::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LInstruction::PrintOutputOperandTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LLabel::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


bool LGap::IsRedundant() const {
  UNIMPLEMENTED();
}


void LGap::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


const char* LArithmeticD::Mnemonic() const {
  UNIMPLEMENTED();
}


const char* LArithmeticT::Mnemonic() const {
  UNIMPLEMENTED();
}


bool LGoto::HasInterestingComment(LCodeGen* gen) const {
  UNIMPLEMENTED();
}


void LGoto::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LCompareNumericAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LIsObjectAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LIsStringAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LIsSmiAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LIsUndetectableAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LStringCompareAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LHasInstanceTypeAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LHasCachedArrayIndexAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LClassOfTestAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LTypeofIsAndBranch::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LStoreCodeEntry::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LInnerAllocatedObject::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LCallConstantFunction::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LLoadContextSlot::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LStoreContextSlot::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LInvokeFunction::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LCallKeyed::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LCallNamed::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LCallGlobal::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LCallKnownGlobal::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LCallNew::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LCallNewArray::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LAccessArgumentsAt::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LStoreNamedField::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LStoreNamedGeneric::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LLoadKeyed::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LStoreKeyed::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LStoreKeyedGeneric::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LTransitionElementsKind::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


int LPlatformChunk::GetNextSpillIndex(RegisterKind kind) {
  UNIMPLEMENTED();
}


LOperand* LPlatformChunk::GetNextSpillSlot(RegisterKind kind) {
  UNIMPLEMENTED();
}


LPlatformChunk* LChunkBuilder::Build() {
  UNIMPLEMENTED();
}


void LChunkBuilder::Abort(BailoutReason reason) {
  UNIMPLEMENTED();
}


LUnallocated* LChunkBuilder::ToUnallocated(Register reg) {
  UNIMPLEMENTED();
}


LUnallocated* LChunkBuilder::ToUnallocated(DoubleRegister reg) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseFixed(HValue* value, Register fixed_register) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseFixedDouble(HValue* value, DoubleRegister reg) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseRegister(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseRegisterAtStart(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseTempRegister(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::Use(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseAtStart(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseOrConstant(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseOrConstantAtStart(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseRegisterOrConstant(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseRegisterOrConstantAtStart(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseConstant(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::UseAny(HValue* value) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::Use(HValue* value, LUnallocated* operand) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::Define(LTemplateInstruction<1, I, T>* instr,
                                    LUnallocated* result) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineAsRegister(
    LTemplateInstruction<1, I, T>* instr) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineAsSpilled(
    LTemplateInstruction<1, I, T>* instr, int index) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineSameAsFirst(
    LTemplateInstruction<1, I, T>* instr) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineFixed(
    LTemplateInstruction<1, I, T>* instr, Register reg) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineFixedDouble(
    LTemplateInstruction<1, I, T>* instr, DoubleRegister reg) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::AssignEnvironment(LInstruction* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::MarkAsCall(LInstruction* instr,
                                        HInstruction* hinstr,
                                        CanDeoptimize can_deoptimize) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::AssignPointerMap(LInstruction* instr) {
  UNIMPLEMENTED();
}


LUnallocated* LChunkBuilder::TempRegister() {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::FixedTemp(Register reg) {
  UNIMPLEMENTED();
}


LOperand* LChunkBuilder::FixedTemp(DoubleRegister reg) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBlockEntry(HBlockEntry* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoDummyUse(HDummyUse* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoEnvironmentMarker(HEnvironmentMarker* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoDeoptimize(HDeoptimize* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoShift(Token::Value op,
                                     HBitwiseBinaryOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoArithmeticD(Token::Value op,
                                           HArithmeticBinaryOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoArithmeticT(Token::Value op,
                                           HBinaryOperation* instr) {
  UNIMPLEMENTED();
}


void LChunkBuilder::DoBasicBlock(HBasicBlock* block, HBasicBlock* next_block) {
  UNIMPLEMENTED();
}


void LChunkBuilder::VisitInstruction(HInstruction* current) {
  UNIMPLEMENTED();
}


LEnvironment* LChunkBuilder::CreateEnvironment(
    HEnvironment* hydrogen_env,
    int* argument_index_accumulator,
    ZoneList<HValue*>* objects_to_materialize) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoGoto(HGoto* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBranch(HBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoDebugBreak(HDebugBreak* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCompareMap(HCompareMap* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoArgumentsLength(HArgumentsLength* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoArgumentsElements(HArgumentsElements* elems) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoInstanceOf(HInstanceOf* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoInstanceOfKnownGlobal(
    HInstanceOfKnownGlobal* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoWrapReceiver(HWrapReceiver* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoApplyArguments(HApplyArguments* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoPushArgument(HPushArgument* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreCodeEntry(
    HStoreCodeEntry* store_code_entry) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoInnerAllocatedObject(
    HInnerAllocatedObject* inner_object) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoThisFunction(HThisFunction* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoContext(HContext* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoOuterContext(HOuterContext* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoDeclareGlobals(HDeclareGlobals* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoGlobalObject(HGlobalObject* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoGlobalReceiver(HGlobalReceiver* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallConstantFunction(
    HCallConstantFunction* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoInvokeFunction(HInvokeFunction* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoUnaryMathOperation(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathFloor(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathRound(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathAbs(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathLog(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathSin(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathCos(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathTan(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathExp(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathSqrt(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathPowHalf(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallKeyed(HCallKeyed* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallNamed(HCallNamed* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallGlobal(HCallGlobal* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallKnownGlobal(HCallKnownGlobal* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallNew(HCallNew* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallNewArray(HCallNewArray* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallFunction(HCallFunction* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallRuntime(HCallRuntime* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoRor(HRor* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoShr(HShr* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoSar(HSar* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoShl(HShl* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBitwise(HBitwise* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoDiv(HDiv* instr) {
  UNIMPLEMENTED();
}


bool LChunkBuilder::HasMagicNumberForDivisor(int32_t divisor) {
  UNIMPLEMENTED();
}


HValue* LChunkBuilder::SimplifiedDivisorForMathFloorOfDiv(HValue* divisor) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathFloorOfDiv(HMathFloorOfDiv* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMod(HMod* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMul(HMul* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoSub(HSub* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoRSub(HSub* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMultiplyAdd(HMul* mul, HValue* addend) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMultiplySub(HValue* minuend, HMul* mul) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoAdd(HAdd* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMathMinMax(HMathMinMax* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoPower(HPower* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoRandom(HRandom* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCompareGeneric(HCompareGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCompareNumericAndBranch(
    HCompareNumericAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCompareObjectEqAndBranch(
    HCompareObjectEqAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCompareHoleAndBranch(
    HCompareHoleAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsObjectAndBranch(HIsObjectAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsStringAndBranch(HIsStringAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsSmiAndBranch(HIsSmiAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsUndetectableAndBranch(
    HIsUndetectableAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStringCompareAndBranch(
    HStringCompareAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoHasInstanceTypeAndBranch(
    HHasInstanceTypeAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoGetCachedArrayIndex(
    HGetCachedArrayIndex* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoHasCachedArrayIndexAndBranch(
    HHasCachedArrayIndexAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoClassOfTestAndBranch(
    HClassOfTestAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoMapEnumLength(HMapEnumLength* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoElementsKind(HElementsKind* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoValueOf(HValueOf* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoDateField(HDateField* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoSeqStringSetChar(HSeqStringSetChar* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBoundsCheck(HBoundsCheck* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBoundsCheckBaseIndexInformation(
    HBoundsCheckBaseIndexInformation* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoAbnormalExit(HAbnormalExit* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoThrow(HThrow* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoUseConst(HUseConst* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoForceRepresentation(HForceRepresentation* bad) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoChange(HChange* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckHeapObject(HCheckHeapObject* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckSmi(HCheckSmi* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckInstanceType(HCheckInstanceType* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckValue(HCheckValue* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckMaps(HCheckMaps* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoClampToUint8(HClampToUint8* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoReturn(HReturn* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoConstant(HConstant* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadGlobalCell(HLoadGlobalCell* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadGlobalGeneric(HLoadGlobalGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreGlobalCell(HStoreGlobalCell* instr) {
  UNIMPLEMENTED();


LInstruction* LChunkBuilder::DoStoreGlobalGeneric(HStoreGlobalGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadContextSlot(HLoadContextSlot* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreContextSlot(HStoreContextSlot* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadNamedField(HLoadNamedField* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadNamedGeneric(HLoadNamedGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadFunctionPrototype(
    HLoadFunctionPrototype* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadRoot(HLoadRoot* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadExternalArrayPointer(
    HLoadExternalArrayPointer* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadKeyed(HLoadKeyed* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadKeyedGeneric(HLoadKeyedGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreKeyed(HStoreKeyed* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreKeyedGeneric(HStoreKeyedGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoTransitionElementsKind(
    HTransitionElementsKind* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoTrapAllocationMemento(
    HTrapAllocationMemento* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreNamedField(HStoreNamedField* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreNamedGeneric(HStoreNamedGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStringAdd(HStringAdd* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStringCharCodeAt(HStringCharCodeAt* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStringCharFromCode(HStringCharFromCode* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoAllocate(HAllocate* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoRegExpLiteral(HRegExpLiteral* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoFunctionLiteral(HFunctionLiteral* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoOsrEntry(HOsrEntry* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoParameter(HParameter* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoUnknownOSRValue(HUnknownOSRValue* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallStub(HCallStub* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoArgumentsObject(HArgumentsObject* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCapturedObject(HCapturedObject* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoAccessArgumentsAt(HAccessArgumentsAt* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoToFastProperties(HToFastProperties* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoTypeof(HTypeof* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoTypeofIsAndBranch(HTypeofIsAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsConstructCallAndBranch(
    HIsConstructCallAndBranch* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoSimulate(HSimulate* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStackCheck(HStackCheck* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoEnterInlined(HEnterInlined* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLeaveInlined(HLeaveInlined* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoForInPrepareMap(HForInPrepareMap* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoForInCacheArray(HForInCacheArray* instr) {
  UNIMPLEMENTED();
}


  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadFieldByIndex(HLoadFieldByIndex* instr) {
  UNIMPLEMENTED();
}


} }  // namespace v8::internal
