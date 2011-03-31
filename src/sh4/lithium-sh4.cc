// Copyright 2011 the V8 project authors. All rights reserved.
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

LOsrEntry::LOsrEntry() {
  UNIMPLEMENTED();
}


void LOsrEntry::MarkSpilledRegister(int allocation_index,
                                    LOperand* spill_operand) {
  UNIMPLEMENTED();
}


void LOsrEntry::MarkSpilledDoubleRegister(int allocation_index,
                                          LOperand* spill_operand) {
  UNIMPLEMENTED();
}


#ifdef DEBUG
void LInstruction::VerifyCall() {
  UNIMPLEMENTED();
}
#endif


void LInstruction::PrintTo(StringStream* stream) {
  UNIMPLEMENTED();
}


template<int R, int I, int T>
void LTemplateInstruction<R, I, T>::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


template<int R, int I, int T>
void LTemplateInstruction<R, I, T>::PrintOutputOperandTo(StringStream* stream) {
  UNIMPLEMENTED();
}


template<typename T, int N>
void OperandContainer<T, N>::PrintOperandsTo(StringStream* stream) {
  UNIMPLEMENTED();
}


void LLabel::PrintDataTo(StringStream* stream) {
  UNIMPLEMENTED();
}


bool LGap::IsRedundant() const {
  UNIMPLEMENTED();
}


void LGap::PrintDataTo(StringStream* stream) {
  for (int i = 0; i < 4; i++) {
    stream->Add("(");
    if (parallel_moves_[i] != NULL) {
      parallel_moves_[i]->PrintDataTo(stream);
    }
    stream->Add(") ");
  }
}


const char* LArithmeticD::Mnemonic() const {
  UNIMPLEMENTED();
}


const char* LArithmeticT::Mnemonic() const {
  UNIMPLEMENTED();
}


void LGoto::PrintDataTo(StringStream* stream) {
  stream->Add("B%d", block_id());
}


void LBranch::PrintDataTo(StringStream* stream) {
  stream->Add("B%d | B%d on ", true_block_id(), false_block_id());
  InputAt(0)->PrintTo(stream);
}


void LCmpIDAndBranch::PrintDataTo(StringStream* stream) {
  stream->Add("if ");
  InputAt(0)->PrintTo(stream);
  stream->Add(" %s ", Token::String(op()));
  InputAt(1)->PrintTo(stream);
  stream->Add(" then B%d else B%d", true_block_id(), false_block_id());
}


void LIsNullAndBranch::PrintDataTo(StringStream* stream) {
  stream->Add("if ");
  InputAt(0)->PrintTo(stream);
  stream->Add(is_strict() ? " === null" : " == null");
  stream->Add(" then B%d else B%d", true_block_id(), false_block_id());
}


void LIsObjectAndBranch::PrintDataTo(StringStream* stream) {
  stream->Add("if is_object(");
  InputAt(0)->PrintTo(stream);
  stream->Add(") then B%d else B%d", true_block_id(), false_block_id());
}


void LIsSmiAndBranch::PrintDataTo(StringStream* stream) {
  stream->Add("if is_smi(");
  InputAt(0)->PrintTo(stream);
  stream->Add(") then B%d else B%d", true_block_id(), false_block_id());
}


void LHasInstanceTypeAndBranch::PrintDataTo(StringStream* stream) {
  stream->Add("if has_instance_type(");
  InputAt(0)->PrintTo(stream);
  stream->Add(") then B%d else B%d", true_block_id(), false_block_id());
}


void LHasCachedArrayIndexAndBranch::PrintDataTo(StringStream* stream) {
  stream->Add("if has_cached_array_index(");
  InputAt(0)->PrintTo(stream);
  stream->Add(") then B%d else B%d", true_block_id(), false_block_id());
}


void LClassOfTestAndBranch::PrintDataTo(StringStream* stream) {
  stream->Add("if class_of_test(");
  InputAt(0)->PrintTo(stream);
  stream->Add(", \"%o\") then B%d else B%d",
              *hydrogen()->class_name(),
              true_block_id(),
              false_block_id());
}


void LTypeofIs::PrintDataTo(StringStream* stream) {
  InputAt(0)->PrintTo(stream);
  stream->Add(" == \"%s\"", *hydrogen()->type_literal()->ToCString());
}


void LTypeofIsAndBranch::PrintDataTo(StringStream* stream) {
  stream->Add("if typeof ");
  InputAt(0)->PrintTo(stream);
  stream->Add(" == \"%s\" then B%d else B%d",
              *hydrogen()->type_literal()->ToCString(),
              true_block_id(), false_block_id());
}


void LCallConstantFunction::PrintDataTo(StringStream* stream) {
  stream->Add("#%d / ", arity());
}


void LUnaryMathOperation::PrintDataTo(StringStream* stream) {
  stream->Add("/%s ", hydrogen()->OpName());
  InputAt(0)->PrintTo(stream);
}


void LLoadContextSlot::PrintDataTo(StringStream* stream) {
  InputAt(0)->PrintTo(stream);
  stream->Add("[%d]", slot_index());
}


void LStoreContextSlot::PrintDataTo(StringStream* stream) {
  InputAt(0)->PrintTo(stream);
  stream->Add("[%d] <- ", slot_index());
  InputAt(1)->PrintTo(stream);
}


void LCallKeyed::PrintDataTo(StringStream* stream) {
  stream->Add("[ecx] #%d / ", arity());
}


void LCallNamed::PrintDataTo(StringStream* stream) {
  SmartPointer<char> name_string = name()->ToCString();
  stream->Add("%s #%d / ", *name_string, arity());
}


void LCallGlobal::PrintDataTo(StringStream* stream) {
  SmartPointer<char> name_string = name()->ToCString();
  stream->Add("%s #%d / ", *name_string, arity());
}


void LCallKnownGlobal::PrintDataTo(StringStream* stream) {
  stream->Add("#%d / ", arity());
}


void LCallNew::PrintDataTo(StringStream* stream) {
  stream->Add("= ");
  InputAt(0)->PrintTo(stream);
  stream->Add(" #%d / ", arity());
}


void LClassOfTest::PrintDataTo(StringStream* stream) {
  stream->Add("= class_of_test(");
  InputAt(0)->PrintTo(stream);
  stream->Add(", \"%o\")", *hydrogen()->class_name());
}


void LAccessArgumentsAt::PrintDataTo(StringStream* stream) {
  arguments()->PrintTo(stream);

  stream->Add(" length ");
  length()->PrintTo(stream);

  stream->Add(" index ");
  index()->PrintTo(stream);
}


int LChunk::GetNextSpillIndex(bool is_double) {
  // Skip a slot if for a double-width slot.
  if (is_double) spill_slot_count_++;
  return spill_slot_count_++;
}


LOperand* LChunk::GetNextSpillSlot(bool is_double) {
  UNIMPLEMENTED();
}


void LChunk::MarkEmptyBlocks() {
  UNIMPLEMENTED();
}


void LStoreNamedField::PrintDataTo(StringStream* stream) {
  object()->PrintTo(stream);
  stream->Add(".");
  stream->Add(*String::cast(*name())->ToCString());
  stream->Add(" <- ");
  value()->PrintTo(stream);
}


void LStoreNamedGeneric::PrintDataTo(StringStream* stream) {
  object()->PrintTo(stream);
  stream->Add(".");
  stream->Add(*String::cast(*name())->ToCString());
  stream->Add(" <- ");
  value()->PrintTo(stream);
}


void LStoreKeyedFastElement::PrintDataTo(StringStream* stream) {
  object()->PrintTo(stream);
  stream->Add("[");
  key()->PrintTo(stream);
  stream->Add("] <- ");
  value()->PrintTo(stream);
}


void LStoreKeyedGeneric::PrintDataTo(StringStream* stream) {
  object()->PrintTo(stream);
  stream->Add("[");
  key()->PrintTo(stream);
  stream->Add("] <- ");
  value()->PrintTo(stream);
}


void LChunk::AddInstruction(LInstruction* instr, HBasicBlock* block) {
}


LConstantOperand* LChunk::DefineConstantOperand(HConstant* constant) {
  return LConstantOperand::Create(constant->id());
}


int LChunk::GetParameterStackSlot(int index) const {
  UNIMPLEMENTED();
}

// A parameter relative to ebp in the arguments stub.
int LChunk::ParameterAt(int index) {
  UNIMPLEMENTED();
}


LGap* LChunk::GetGapAt(int index) const {
  UNIMPLEMENTED();
}


bool LChunk::IsGapAt(int index) const {
  UNIMPLEMENTED();
}


int LChunk::NearestGapPos(int index) const {
  UNIMPLEMENTED();
}


void LChunk::AddGapMove(int index, LOperand* from, LOperand* to) {
  UNIMPLEMENTED();
}


Handle<Object> LChunk::LookupLiteral(LConstantOperand* operand) const {
  UNIMPLEMENTED();
}


Representation LChunk::LookupLiteralRepresentation(
    LConstantOperand* operand) const {
  UNIMPLEMENTED();
}


LChunk* LChunkBuilder::Build() {
  UNIMPLEMENTED();
}


void LChunkBuilder::Abort(const char* format, ...) {
  UNIMPLEMENTED();
}


LRegister* LChunkBuilder::ToOperand(Register reg) {
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
LInstruction* LChunkBuilder::Define(LTemplateInstruction<1, I, T>* instr) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineAsRegister(
    LTemplateInstruction<1, I, T>* instr) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineAsSpilled(
    LTemplateInstruction<1, I, T>* instr,
    int index) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineSameAsFirst(
    LTemplateInstruction<1, I, T>* instr) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineFixed(LTemplateInstruction<1, I, T>* instr,
                                         Register reg) {
  UNIMPLEMENTED();
}


template<int I, int T>
LInstruction* LChunkBuilder::DefineFixedDouble(
    LTemplateInstruction<1, I, T>* instr,
    DoubleRegister reg) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::AssignEnvironment(LInstruction* instr) {
  HEnvironment* hydrogen_env = current_block_->last_environment();
  instr->set_environment(CreateEnvironment(hydrogen_env));
}


LInstruction* LChunkBuilder::SetInstructionPendingDeoptimizationEnvironment(
    LInstruction* instr, int ast_id) {
  UNIMPLEMENTED();
}


void LChunkBuilder::ClearInstructionPendingDeoptimizationEnvironment() {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::MarkAsCall(LInstruction* instr,
                                        HInstruction* hinstr,
                                        CanDeoptimize can_deoptimize) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::MarkAsSaveDoubles(LInstruction* instr) {
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


LInstruction* LChunkBuilder::DoDeoptimize(HDeoptimize* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBit(Token::Value op,
                                   HBitwiseBinaryOperation* instr) {
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
                                           HArithmeticBinaryOperation* instr) {
  UNIMPLEMENTED();
}

void LChunkBuilder::DoBasicBlock(HBasicBlock* block, HBasicBlock* next_block) {
  UNIMPLEMENTED();
}


void LChunkBuilder::VisitInstruction(HInstruction* current) {
  UNIMPLEMENTED();
}


LEnvironment* LChunkBuilder::CreateEnvironment(HEnvironment* hydrogen_env) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoGoto(HGoto* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoTest(HTest* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCompareMap(HCompareMap* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoArgumentsLength(HArgumentsLength* length) {
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


LInstruction* LChunkBuilder::DoApplyArguments(HApplyArguments* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoPushArgument(HPushArgument* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoContext(HContext* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoOuterContext(HOuterContext* instr) {
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


LInstruction* LChunkBuilder::DoUnaryMathOperation(HUnaryMathOperation* instr) {
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


LInstruction* LChunkBuilder::DoCallFunction(HCallFunction* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCallRuntime(HCallRuntime* instr) {
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


LInstruction* LChunkBuilder::DoBitAnd(HBitAnd* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBitNot(HBitNot* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBitOr(HBitOr* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBitXor(HBitXor* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoDiv(HDiv* instr) {
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


LInstruction* LChunkBuilder::DoAdd(HAdd* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoPower(HPower* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCompare(HCompare* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCompareJSObjectEq(
    HCompareJSObjectEq* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsNull(HIsNull* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsObject(HIsObject* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsSmi(HIsSmi* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoHasInstanceType(HHasInstanceType* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoGetCachedArrayIndex(
    HGetCachedArrayIndex* instr)  {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoHasCachedArrayIndex(
    HHasCachedArrayIndex* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoClassOfTest(HClassOfTest* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoJSArrayLength(HJSArrayLength* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoFixedArrayLength(HFixedArrayLength* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoExternalArrayLength(
    HExternalArrayLength* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoValueOf(HValueOf* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoBoundsCheck(HBoundsCheck* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoAbnormalExit(HAbnormalExit* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoThrow(HThrow* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoChange(HChange* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckNonSmi(HCheckNonSmi* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckInstanceType(HCheckInstanceType* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckPrototypeMaps(HCheckPrototypeMaps* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckSmi(HCheckSmi* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckFunction(HCheckFunction* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoCheckMap(HCheckMap* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoReturn(HReturn* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoConstant(HConstant* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadGlobal(HLoadGlobal* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreGlobal(HStoreGlobal* instr) {
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


LInstruction* LChunkBuilder::DoLoadNamedFieldPolymorphic(
    HLoadNamedFieldPolymorphic* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadNamedGeneric(HLoadNamedGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadFunctionPrototype(
    HLoadFunctionPrototype* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadElements(HLoadElements* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadExternalArrayPointer(
    HLoadExternalArrayPointer* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadKeyedFastElement(
    HLoadKeyedFastElement* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadKeyedSpecializedArrayElement(
    HLoadKeyedSpecializedArrayElement* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoLoadKeyedGeneric(HLoadKeyedGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreKeyedFastElement(
    HStoreKeyedFastElement* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreKeyedSpecializedArrayElement(
    HStoreKeyedSpecializedArrayElement* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreKeyedGeneric(HStoreKeyedGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreNamedField(HStoreNamedField* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStoreNamedGeneric(HStoreNamedGeneric* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStringCharCodeAt(HStringCharCodeAt* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStringCharFromCode(HStringCharFromCode* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoStringLength(HStringLength* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoArrayLiteral(HArrayLiteral* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoObjectLiteral(HObjectLiteral* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoRegExpLiteral(HRegExpLiteral* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoFunctionLiteral(HFunctionLiteral* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoDeleteProperty(HDeleteProperty* instr) {
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


LInstruction* LChunkBuilder::DoAccessArgumentsAt(HAccessArgumentsAt* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoToFastProperties(HToFastProperties* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoTypeof(HTypeof* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoTypeofIs(HTypeofIs* instr) {
  UNIMPLEMENTED();
}


LInstruction* LChunkBuilder::DoIsConstructCall(HIsConstructCall* instr) {
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


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
