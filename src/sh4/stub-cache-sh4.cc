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

#include "ic-inl.h"
#include "codegen-inl.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {


MaybeObject* LoadStubCompiler::CompileLoadField(JSObject* object,
                                                JSObject* holder,
                                                int index,
                                                String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadInterceptor(JSObject* receiver,
                                                      JSObject* holder,
                                                      String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadCallback(String* name,
                                                   JSObject* object,
                                                   JSObject* holder,
                                                   AccessorInfo* callback) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadConstant(JSObject* object,
                                                   JSObject* holder,
                                                   Object* value,
                                                   String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadField(String* name,
                                                     JSObject* receiver,
                                                     JSObject* holder,
                                                     int index) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadGlobal(JSObject* object,
                                                 GlobalObject* holder,
                                                 JSGlobalPropertyCell* cell,
                                                 String* name,
                                                 bool is_dont_delete) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadInterceptor(JSObject* receiver,
                                                           JSObject* holder,
                                                           String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadNonexistent(String* name,
                                                      JSObject* object,
                                                      JSObject* last) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedStoreStubCompiler::CompileStoreField(JSObject* object,
                                                       int index,
                                                       Map* transition,
                                                       String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* StoreStubCompiler::CompileStoreCallback(JSObject* object,
                                                     AccessorInfo* callback,
                                                     String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* StoreStubCompiler::CompileStoreGlobal(GlobalObject* object,
                                                   JSGlobalPropertyCell* cell,
                                                   String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* StoreStubCompiler::CompileStoreInterceptor(JSObject* receiver,
                                                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* StoreStubCompiler::CompileStoreField(JSObject* object,
                                                  int index,
                                                  Map* transition,
                                                  String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileArrayPushCall(Object* object,
                                                    JSObject* holder,
                                                    JSGlobalPropertyCell* cell,
                                                    JSFunction* function,
                                                    String* name) {
  UNIMPLEMENTED();
  return NULL;
}

MaybeObject* CallStubCompiler::CompileArrayPopCall(Object* object,
                                                   JSObject* holder,
                                                   JSGlobalPropertyCell* cell,
                                                   JSFunction* function,
                                                   String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileCallConstant(Object* object,
                                                   JSObject* holder,
                                                   JSFunction* function,
                                                   String* name,
                                                   CheckType check) {
  UNIMPLEMENTED();
  return NULL;
}


MUST_USE_RESULT MaybeObject* CallStubCompiler::CompileCallField(
    JSObject* object,
    JSObject* holder,
    int index,
    String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileCallGlobal(JSObject* object,
                                                 GlobalObject* holder,
                                                 JSGlobalPropertyCell* cell,
                                                 JSFunction* function,
                                                 String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileCallInterceptor(JSObject* object,
                                                      JSObject* holder,
                                                      String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileFastApiCall(
                        const CallOptimization& optimization,
                        Object* object,
                        JSObject* holder,
                        JSGlobalPropertyCell* cell,
                        JSFunction* function,
                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileMathAbsCall(Object* object,
                                                  JSObject* holder,
                                                  JSGlobalPropertyCell* cell,
                                                  JSFunction* function,
                                                  String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileMathFloorCall(Object* object,
                                                    JSObject* holder,
                                                    JSGlobalPropertyCell* cell,
                                                    JSFunction* function,
                                                    String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileStringCharAtCall(
                        Object* object,
                        JSObject* holder,
                        JSGlobalPropertyCell* cell,
                        JSFunction* function,
                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileStringCharCodeAtCall(
                        Object* object,
                        JSObject* holder,
                        JSGlobalPropertyCell* cell,
                        JSFunction* function,
                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileStringFromCharCodeCall(
                        Object* object,
                        JSObject* holder,
                        JSGlobalPropertyCell* cell,
                        JSFunction* function,
                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


void ICCompareStub::GenerateHeapNumbers(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void ICCompareStub::GenerateMiss(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void ICCompareStub::GenerateObjects(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void ICCompareStub::GenerateSmis(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadArrayLength(String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadCallback(
    String* name,
    JSObject* receiver,
    JSObject* holder,
    AccessorInfo* callback) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadConstant(String* name,
                                                        JSObject* receiver,
                                                        JSObject* holder,
                                                        Object* value) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadFunctionPrototype(String* name) {
  UNIMPLEMENTED();
  return NULL;
}



MaybeObject* KeyedLoadStubCompiler::CompileLoadSpecialized(JSObject* receiver) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadStringLength(String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* ConstructStubCompiler::CompileConstructStub(JSFunction* function) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* ExternalArrayStubCompiler::CompileKeyedLoadStub(
    JSObject*receiver, ExternalArrayType array_type, Code::Flags flags) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* ExternalArrayStubCompiler::CompileKeyedStoreStub(
    JSObject* receiver, ExternalArrayType array_type, Code::Flags flags) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedStoreStubCompiler::CompileStoreSpecialized(
    JSObject* receiver) {
  UNIMPLEMENTED();
  return NULL;
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
