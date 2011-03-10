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

#include "code-stubs.h"
#include "codegen-inl.h"
#include "compiler.h"
#include "debug.h"
#include "full-codegen.h"
#include "parser.h"
#include "scopes.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {

void FullCodeGenerator::ClearAccumulator() {
  UNIMPLEMENTED();
}


void FullCodeGenerator::DeclareGlobals(Handle<FixedArray> pairs) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::DoTest(Label* if_true,
                               Label* if_false,
                               Label* fall_through) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitArguments(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitArgumentsLength(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitBinaryOp(Token::Value op,
                                     OverwriteMode mode) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitCallFunction(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitClassOf(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitFastAsciiArrayJoin(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitGetCachedArrayIndex(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitGetFromCache(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitHasCachedArrayIndex(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitInlineSmiBinaryOp(Expression* expr,
                                              Token::Value op,
                                              OverwriteMode mode,
                                              Expression* left,
                                              Expression* right) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsArray(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsConstructCall(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsFunction(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsNonNegativeSmi(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsObject(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsRegExp(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsRegExpEquivalent(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsSmi(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsSpecObject(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsStringWrapperSafeForDefaultValueOf(
    ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsUndetectableObject(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitLog(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathCos(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathLog(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathPow(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathSin(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathSqrt(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitNewClosure(Handle<SharedFunctionInfo> info,
                                       bool pretenure) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitNumberToString(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitObjectEquals(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitRandomHeapNumber(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitRegExpConstructResult(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitRegExpExec(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitReturnSequence() {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitSetValueOf(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStackCheck(IterationStatement* stmt) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringAdd(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringCharAt(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringCharCodeAt(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringCharFromCode(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringCompare(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitSubString(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitSwapElements(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EmitValueOf(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EnterFinallyBlock() {
  UNIMPLEMENTED();
}


void FullCodeGenerator::ExitFinallyBlock() {
  UNIMPLEMENTED();
}


void FullCodeGenerator::Generate(CompilationInfo* info) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::LoadContextField(Register dst, int context_index) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::PrepareForBailoutBeforeSplit(State state,
                                                     bool should_normalize,
                                                     Label* if_true,
                                                     Label* if_false) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::StoreToFrameField(int frame_offset, Register value) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitArrayLiteral(ArrayLiteral* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitAssignment(Assignment* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCall(Call* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCallNew(CallNew* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCallRuntime(CallRuntime* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCompareOperation(CompareOperation* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCompareToNull(CompareToNull* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCountOperation(CountOperation* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitDeclaration(Declaration* decl) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitForInStatement(ForInStatement* stmt) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitObjectLiteral(ObjectLiteral* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitProperty(Property* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitRegExpLiteral(RegExpLiteral* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitSwitchStatement(SwitchStatement* stmt) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitThisFunction(ThisFunction* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitUnaryOperation(UnaryOperation* expr) {
  UNIMPLEMENTED();
}


void FullCodeGenerator::VisitVariableProxy(VariableProxy* expr) {
  UNIMPLEMENTED();
}


Register FullCodeGenerator::context_register() {
  UNIMPLEMENTED();
}


Register FullCodeGenerator::result_register() {
  UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(bool flag) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Label* materialize_true,
    Label* materialize_false) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(Slot* slot) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Handle<Object> lit) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Heap::RootListIndex index) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::DropAndPlug(
    int count,
    Register reg) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(Slot* slot) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(
    Heap::RootListIndex index) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(Handle<Object> lit) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::DropAndPlug(int count,
                                                       Register reg) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(
    Label* materialize_true,
    Label* materialize_false) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(bool flag) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(Slot* slot) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(Heap::RootListIndex index) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(Handle<Object> lit) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::DropAndPlug(int count,
                                                 Register reg) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(Label* materialize_true,
                                          Label* materialize_false) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(bool flag) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(Slot* slot) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(Heap::RootListIndex index) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(Handle<Object> lit) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::DropAndPlug(int count,
                                                   Register reg) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(Label* materialize_true,
                                            Label* materialize_false) const {
  UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(bool flag) const {
  UNIMPLEMENTED();
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
