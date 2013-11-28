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
#include "hydrogen-osr.h"

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
  return false;
}


void LGap::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


const char* LArithmeticD::Mnemonic() const {
  UNIMPLEMENTED();
  return NULL;
}


const char* LArithmeticT::Mnemonic() const {
  UNIMPLEMENTED();
  return NULL;
}


bool LGoto::HasInterestingComment(LCodeGen* gen) const {
  UNIMPLEMENTED();
  return false;
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
  return -1;
}


LOperand* LPlatformChunk::GetNextSpillSlot(RegisterKind kind) {
  UNIMPLEMENTED();
  return NULL;
}


LPlatformChunk* LChunkBuilder::Build() {
  UNIMPLEMENTED();
  return NULL;
}


void LChunkBuilder::Abort(BailoutReason reason) {
  UNIMPLEMENTED();
}


LUnallocated* LChunkBuilder::ToUnallocated(Register reg) {
  UNIMPLEMENTED();
  return NULL;
}


LUnallocated* LChunkBuilder::ToUnallocated(DoubleRegister reg) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseFixed(HValue* value, Register fixed_register) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseFixedDouble(HValue* value, DoubleRegister reg) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseRegister(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseRegisterAtStart(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseTempRegister(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::Use(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseAtStart(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseOrConstant(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseOrConstantAtStart(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseRegisterOrConstant(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseRegisterOrConstantAtStart(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseConstant(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::UseAny(HValue* value) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::Use(HValue* value, LUnallocated* operand) {
  UNIMPLEMENTED();
  return NULL;
}


template<int I, int T>
LInstruction* LChunkBuilder::Define(LTemplateInstruction<1, I, T>* instr,
                                    LUnallocated* result) {
  UNIMPLEMENTED();
  return NULL;
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineAsRegister(
    LTemplateInstruction<1, I, T>* instr) {
  UNIMPLEMENTED();
  return NULL;
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineAsSpilled(
    LTemplateInstruction<1, I, T>* instr, int index) {
  UNIMPLEMENTED();
  return NULL;
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineSameAsFirst(
    LTemplateInstruction<1, I, T>* instr) {
  UNIMPLEMENTED();
  return NULL;
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineFixed(
    LTemplateInstruction<1, I, T>* instr, Register reg) {
  UNIMPLEMENTED();
  return NULL;
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineFixedDouble(
    LTemplateInstruction<1, I, T>* instr, DoubleRegister reg) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::AssignEnvironment(LInstruction* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::MarkAsCall(LInstruction* instr,
                                        HInstruction* hinstr,
                                        CanDeoptimize can_deoptimize) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::AssignPointerMap(LInstruction* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LUnallocated* LChunkBuilder::TempRegister() {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::FixedTemp(Register reg) {
  UNIMPLEMENTED();
  return NULL;
}


LOperand* LChunkBuilder::FixedTemp(DoubleRegister reg) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoBlockEntry(HBlockEntry* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoDummyUse(HDummyUse* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoEnvironmentMarker(HEnvironmentMarker* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoDeoptimize(HDeoptimize* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoShift(Token::Value op,
                                     HBitwiseBinaryOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoArithmeticD(Token::Value op,
                                           HArithmeticBinaryOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoArithmeticT(Token::Value op,
                                           HBinaryOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
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
  return NULL;
}


LInstruction* LChunkBuilder::DoGoto(HGoto* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoBranch(HBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoDebugBreak(HDebugBreak* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCompareMap(HCompareMap* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoArgumentsLength(HArgumentsLength* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoArgumentsElements(HArgumentsElements* elems) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoInstanceOf(HInstanceOf* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoInstanceOfKnownGlobal(
    HInstanceOfKnownGlobal* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoWrapReceiver(HWrapReceiver* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoApplyArguments(HApplyArguments* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoPushArgument(HPushArgument* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStoreCodeEntry(
    HStoreCodeEntry* store_code_entry) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoInnerAllocatedObject(
    HInnerAllocatedObject* inner_object) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoThisFunction(HThisFunction* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoContext(HContext* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoOuterContext(HOuterContext* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoDeclareGlobals(HDeclareGlobals* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoGlobalObject(HGlobalObject* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoGlobalReceiver(HGlobalReceiver* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallConstantFunction(
    HCallConstantFunction* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoInvokeFunction(HInvokeFunction* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoUnaryMathOperation(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathFloor(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathRound(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathAbs(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathLog(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathSin(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathCos(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathTan(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathExp(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathSqrt(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathPowHalf(HUnaryMathOperation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallKeyed(HCallKeyed* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallNamed(HCallNamed* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallGlobal(HCallGlobal* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallKnownGlobal(HCallKnownGlobal* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallNew(HCallNew* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallNewArray(HCallNewArray* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallFunction(HCallFunction* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallRuntime(HCallRuntime* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoRor(HRor* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoShr(HShr* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoSar(HSar* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoShl(HShl* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoBitwise(HBitwise* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoDiv(HDiv* instr) {
  UNIMPLEMENTED();
  return NULL;
}


bool LChunkBuilder::HasMagicNumberForDivisor(int32_t divisor) {
  UNIMPLEMENTED();
  return NULL;
}


HValue* LChunkBuilder::SimplifiedDivisorForMathFloorOfDiv(HValue* divisor) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathFloorOfDiv(HMathFloorOfDiv* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMod(HMod* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMul(HMul* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoSub(HSub* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoRSub(HSub* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMultiplyAdd(HMul* mul, HValue* addend) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMultiplySub(HValue* minuend, HMul* mul) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoAdd(HAdd* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMathMinMax(HMathMinMax* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoPower(HPower* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoRandom(HRandom* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCompareGeneric(HCompareGeneric* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCompareNumericAndBranch(
    HCompareNumericAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCompareObjectEqAndBranch(
    HCompareObjectEqAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCompareHoleAndBranch(
    HCompareHoleAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoIsObjectAndBranch(HIsObjectAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoIsStringAndBranch(HIsStringAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoIsSmiAndBranch(HIsSmiAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoIsUndetectableAndBranch(
    HIsUndetectableAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStringCompareAndBranch(
    HStringCompareAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoHasInstanceTypeAndBranch(
    HHasInstanceTypeAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoGetCachedArrayIndex(
    HGetCachedArrayIndex* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoHasCachedArrayIndexAndBranch(
    HHasCachedArrayIndexAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoClassOfTestAndBranch(
    HClassOfTestAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoMapEnumLength(HMapEnumLength* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoElementsKind(HElementsKind* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoValueOf(HValueOf* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoDateField(HDateField* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoSeqStringSetChar(HSeqStringSetChar* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoBoundsCheck(HBoundsCheck* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoBoundsCheckBaseIndexInformation(
    HBoundsCheckBaseIndexInformation* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoAbnormalExit(HAbnormalExit* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoThrow(HThrow* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoUseConst(HUseConst* instr) {
  return NULL;
}


LInstruction* LChunkBuilder::DoForceRepresentation(HForceRepresentation* bad) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoChange(HChange* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCheckHeapObject(HCheckHeapObject* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCheckSmi(HCheckSmi* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCheckInstanceType(HCheckInstanceType* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCheckValue(HCheckValue* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCheckMaps(HCheckMaps* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoClampToUint8(HClampToUint8* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoReturn(HReturn* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoConstant(HConstant* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadGlobalCell(HLoadGlobalCell* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadGlobalGeneric(HLoadGlobalGeneric* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStoreGlobalCell(HStoreGlobalCell* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStoreGlobalGeneric(HStoreGlobalGeneric* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadContextSlot(HLoadContextSlot* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStoreContextSlot(HStoreContextSlot* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadNamedField(HLoadNamedField* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadNamedGeneric(HLoadNamedGeneric* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadFunctionPrototype(
    HLoadFunctionPrototype* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadRoot(HLoadRoot* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadExternalArrayPointer(
    HLoadExternalArrayPointer* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadKeyed(HLoadKeyed* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadKeyedGeneric(HLoadKeyedGeneric* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStoreKeyed(HStoreKeyed* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStoreKeyedGeneric(HStoreKeyedGeneric* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoTransitionElementsKind(
    HTransitionElementsKind* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoTrapAllocationMemento(
    HTrapAllocationMemento* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStoreNamedField(HStoreNamedField* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStoreNamedGeneric(HStoreNamedGeneric* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStringAdd(HStringAdd* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStringCharCodeAt(HStringCharCodeAt* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStringCharFromCode(HStringCharFromCode* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoAllocate(HAllocate* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoRegExpLiteral(HRegExpLiteral* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoFunctionLiteral(HFunctionLiteral* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoOsrEntry(HOsrEntry* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoParameter(HParameter* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoUnknownOSRValue(HUnknownOSRValue* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCallStub(HCallStub* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoArgumentsObject(HArgumentsObject* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCapturedObject(HCapturedObject* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoAccessArgumentsAt(HAccessArgumentsAt* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoToFastProperties(HToFastProperties* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoTypeof(HTypeof* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoTypeofIsAndBranch(HTypeofIsAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoIsConstructCallAndBranch(
    HIsConstructCallAndBranch* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoSimulate(HSimulate* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoStackCheck(HStackCheck* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoEnterInlined(HEnterInlined* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLeaveInlined(HLeaveInlined* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoForInPrepareMap(HForInPrepareMap* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoForInCacheArray(HForInCacheArray* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoCheckMapValue(HCheckMapValue* instr) {
  UNIMPLEMENTED();
  return NULL;
}


LInstruction* LChunkBuilder::DoLoadFieldByIndex(HLoadFieldByIndex* instr) {
  UNIMPLEMENTED();
  return NULL;
}


} }  // namespace v8::internal
