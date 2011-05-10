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

#include "sh4/lithium-codegen-sh4.h"
#include "code-stubs.h"
#include "deoptimizer.h"
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
        deoptimization_index_(deoptimization_index) {}
  virtual ~SafepointGenerator() { }

  virtual void BeforeCall(int call_size) {
    UNIMPLEMENTED();
  }

  virtual void AfterCall() {
    codegen_->RecordSafepoint(pointers_, deoptimization_index_);
  }

 private:
  LCodeGen* codegen_;
  LPointerMap* pointers_;
  int deoptimization_index_;
};


void LCodeGen::DoAccessArgumentsAt(LAccessArgumentsAt* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoAddI(LAddI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoApplyArguments(LApplyArguments* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArgumentsElements(LArgumentsElements* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArgumentsLength(LArgumentsLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArithmeticD(LArithmeticD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArithmeticT(LArithmeticT* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoArrayLiteral(LArrayLiteral* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBitI(LBitI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBitNotI(LBitNotI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBoundsCheck(LBoundsCheck* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoBranch(LBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallConstantFunction(LCallConstantFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallFunction(LCallFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallGlobal(LCallGlobal* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInvokeFunction(LInvokeFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallKeyed(LCallKeyed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallKnownGlobal(LCallKnownGlobal* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallNamed(LCallNamed* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadNamedFieldPolymorphic(LLoadNamedFieldPolymorphic* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallNew(LCallNew* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallRuntime(LCallRuntime* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCallStub(LCallStub* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckFunction(LCheckFunction* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckInstanceType(LCheckInstanceType* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckMap(LCheckMap* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckNonSmi(LCheckNonSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedSpecializedArrayElement(
    LLoadKeyedSpecializedArrayElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckPrototypeMaps(LCheckPrototypeMaps* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCheckSmi(LCheckSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoClassOfTest(LClassOfTest* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoClassOfTestAndBranch(LClassOfTestAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpID(LCmpID* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpIDAndBranch(LCmpIDAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpJSObjectEq(LCmpJSObjectEq* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpJSObjectEqAndBranch(LCmpJSObjectEqAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpMapAndBranch(LCmpMapAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpT(LCmpT* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoCmpTAndBranch(LCmpTAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantD(LConstantD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantI(LConstantI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoConstantT(LConstantT* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoContext(LContext* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeleteProperty(LDeleteProperty* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDeoptimize(LDeoptimize* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDivI(LDivI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoDoubleToI(LDoubleToI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoExternalArrayLength(LExternalArrayLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoFixedArrayLength(LFixedArrayLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoFunctionLiteral(LFunctionLiteral* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGap(LGap* gap) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGetCachedArrayIndex(LGetCachedArrayIndex* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGlobalObject(LGlobalObject* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGlobalReceiver(LGlobalReceiver* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoGoto(LGoto* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoHasCachedArrayIndex(LHasCachedArrayIndex* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoHasCachedArrayIndexAndBranch(
    LHasCachedArrayIndexAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoHasInstanceType(LHasInstanceType* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoHasInstanceTypeAndBranch(LHasInstanceTypeAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInstanceOf(LInstanceOf* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInstanceOfAndBranch(LInstanceOfAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInstanceOfKnownGlobal(LInstanceOfKnownGlobal* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoInteger32ToDouble(LInteger32ToDouble* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsConstructCall(LIsConstructCall* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsConstructCallAndBranch(LIsConstructCallAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedSpecializedArrayElement(
    LStoreKeyedSpecializedArrayElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsNull(LIsNull* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsNullAndBranch(LIsNullAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsObject(LIsObject* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsObjectAndBranch(LIsObjectAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsSmi(LIsSmi* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoIsSmiAndBranch(LIsSmiAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoJSArrayLength(LJSArrayLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLabel(LLabel* label) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLazyBailout(LLazyBailout* instr) {
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


void LCodeGen::DoLoadElements(LLoadElements* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadExternalArrayPointer(
    LLoadExternalArrayPointer* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadFunctionPrototype(LLoadFunctionPrototype* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedFastElement(LLoadKeyedFastElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadKeyedGeneric(LLoadKeyedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadNamedField(LLoadNamedField* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoLoadNamedGeneric(LLoadNamedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoModI(LModI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoMulI(LMulI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoNumberTagD(LNumberTagD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoNumberTagI(LNumberTagI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoNumberUntagD(LNumberUntagD* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoObjectLiteral(LObjectLiteral* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoOsrEntry(LOsrEntry* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoOuterContext(LOuterContext* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoParameter(LParameter* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoPower(LPower* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoPushArgument(LPushArgument* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoRegExpLiteral(LRegExpLiteral* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoReturn(LReturn* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoShiftI(LShiftI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoSmiTag(LSmiTag* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoSmiUntag(LSmiUntag* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStackCheck(LStackCheck* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreContextSlot(LStoreContextSlot* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedFastElement(LStoreKeyedFastElement* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreKeyedGeneric(LStoreKeyedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStringAdd(LStringAdd* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreNamedField(LStoreNamedField* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStoreNamedGeneric(LStoreNamedGeneric* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStringCharCodeAt(LStringCharCodeAt* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStringCharFromCode(LStringCharFromCode* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoStringLength(LStringLength* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoSubI(LSubI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTaggedToI(LTaggedToI* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoThrow(LThrow* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoToFastProperties(LToFastProperties* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTypeof(LTypeof* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTypeofIs(LTypeofIs* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoTypeofIsAndBranch(LTypeofIsAndBranch* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoUnaryMathOperation(LUnaryMathOperation* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoUnknownOSRValue(LUnknownOSRValue* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::DoValueOf(LValueOf* instr) {
  UNIMPLEMENTED();
}


void LCodeGen::FinishCode(Handle<Code> code) {
  UNIMPLEMENTED();
}


bool LCodeGen::GenerateCode() {
  UNIMPLEMENTED();
}


void LCodeGen::PopulateDeoptimizationLiteralsWithInlinedFunctions() {
  UNIMPLEMENTED();
}




} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
