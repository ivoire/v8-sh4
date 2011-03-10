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

#include "codegen-inl.h"
#include "bootstrapper.h"
#include "code-stubs.h"
#include "compiler.h"
#include "debug.h"
#include "ic-inl.h"
#include "parser.h"
#include "regexp-macro-assembler.h"
#include "register-allocator-inl.h"
#include "scopes.h"
#include "virtual-frame-inl.h"


namespace v8 {
namespace internal {

CodeGenerator::CodeGenerator(MacroAssembler* masm)
    : deferred_(8) {
  UNIMPLEMENTED();
}


void CodeGenerator::DeclareGlobals(Handle<FixedArray> pairs) {
  UNIMPLEMENTED();
}


void CodeGenerator::Generate(CompilationInfo* info) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateArguments(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateArgumentsLength(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateCallFunction(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateClassOf(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateFastAsciiArrayJoin(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateGetCachedArrayIndex(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateGetFromCache(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateHasCachedArrayIndex(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsArray(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsConstructCall(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsFunction(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsNonNegativeSmi(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsObject(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsRegExp(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsRegExpEquivalent(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsSmi(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsSpecObject(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsStringWrapperSafeForDefaultValueOf(
    ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateIsUndetectableObject(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateLog(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateMathCos(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateMathLog(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateMathPow(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateMathSin(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateMathSqrt(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateNumberToString(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateObjectEquals(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateRandomHeapNumber(
    ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateRegExpConstructResult(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateRegExpExec(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateSetValueOf(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateStringAdd(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateStringCharAt(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateStringCharCodeAt(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateStringCharFromCode(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateStringCompare(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateSubString(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateSwapElements(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void CodeGenerator::GenerateValueOf(ZoneList<Expression*>* args) {
  UNIMPLEMENTED();
}


void DeferredCode::RestoreRegisters() {
  UNIMPLEMENTED();
}


void DeferredCode::SaveRegisters() {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitDeclaration(v8::internal::Declaration*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitStatements(ZoneList<Statement*>* statements) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitBlock(v8::internal::Block*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitExpressionStatement(v8::internal::ExpressionStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitEmptyStatement(v8::internal::EmptyStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitIfStatement(v8::internal::IfStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitContinueStatement(v8::internal::ContinueStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitBreakStatement(v8::internal::BreakStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitReturnStatement(v8::internal::ReturnStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitWithEnterStatement(v8::internal::WithEnterStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitWithExitStatement(v8::internal::WithExitStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitSwitchStatement(v8::internal::SwitchStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitDoWhileStatement(v8::internal::DoWhileStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitWhileStatement(v8::internal::WhileStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitForStatement(v8::internal::ForStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitForInStatement(v8::internal::ForInStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitTryCatchStatement(v8::internal::TryCatchStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitTryFinallyStatement(v8::internal::TryFinallyStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitDebuggerStatement(v8::internal::DebuggerStatement*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitFunctionLiteral(v8::internal::FunctionLiteral*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitSharedFunctionInfoLiteral(v8::internal::SharedFunctionInfoLiteral*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitConditional(v8::internal::Conditional*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitVariableProxy(v8::internal::VariableProxy*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitLiteral(v8::internal::Literal*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitRegExpLiteral(v8::internal::RegExpLiteral*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitObjectLiteral(v8::internal::ObjectLiteral*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitArrayLiteral(v8::internal::ArrayLiteral*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitCatchExtensionObject(v8::internal::CatchExtensionObject*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitAssignment(v8::internal::Assignment*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitThrow(v8::internal::Throw*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitProperty(v8::internal::Property*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitCall(v8::internal::Call*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitCallNew(v8::internal::CallNew*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitCallRuntime(v8::internal::CallRuntime*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitUnaryOperation(v8::internal::UnaryOperation*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitCountOperation(v8::internal::CountOperation*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitBinaryOperation(v8::internal::BinaryOperation*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitCompareOperation(v8::internal::CompareOperation*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitCompareToNull(v8::internal::CompareToNull*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitThisFunction(v8::internal::ThisFunction*) {
  UNIMPLEMENTED();
}


void CodeGenerator::VisitSlot(Slot* slot) {
  UNIMPLEMENTED();
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
