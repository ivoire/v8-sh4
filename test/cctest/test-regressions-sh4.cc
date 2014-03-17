// Copyright 2009 the V8 project authors. All rights reserved.
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

#include <stdlib.h>

#include "v8.h"

#include "api.h"
#include "factory.h"
#include "objects.h"
#include "cctest.h"
#include "zone-inl.h"

using namespace v8::internal;

// TODO(stm): check for activation in new code base
// // Simply returns the undefined object
// static v8::Handle<Value> CallBack(Local<String> name,
//                                     const AccessorInfo& info) {
//   return v8::Undefined();
// }

// // Regression test for a defect in -use-ic mode.
// // The bug returns a:
// // Fatal error in src/objects.h, line 732
// // CHECK(!IsFailure()) failed
// // The problem was in sh4 MacroAssembler::TryCallApiFunctionAndReturn()
// // that used r0 as scratch register for DirectCEntryStub().
// // This scratch must not be a return value register.
// // Note: 3 iterations of the loop are necessary to exhibit the issue.
// THREADED_TEST(IssueLoadCallback) {
//   v8::HandleScope scope;
//   v8::Handle<v8::ObjectTemplate> obj = ObjectTemplate::New();
//   i::StringStream::ClearMentionedObjectCache();
//   obj->SetAccessor(v8_str("xxx"), CallBack);
//   LocalContext env;
//   env->Global()->Set(v8_str("obj"), obj->NewInstance());
//   Script::Compile(String::New(
//       "function foo() {"
//       "  return obj.xxx;"
//       "}"
//       "for (var i = 0; i < 3; i++) {"
//       "  foo();"
//       "}"))->Run();
// }

// // Regression test for a defect in -use-ic mode.
// // Handling of non-smi value on sh4 was wrong in
// // KeyedLoadStubCompiler::GenerateLoadExternalArray().
// // Used to return -1 instead of INT_MAX.
// // Note: 3 iterations of the loop are necessary to exhibit the issue.
// THREADED_TEST(IssueKeyedLoadExtIntArray) {
//   v8::HandleScope scope;
//   LocalContext context;
//   v8::Handle<v8::Object> obj = v8::Object::New();
//   int kElementCount = 1;
//   int32_t* array_data =
//     static_cast<int32_t*>(malloc(kElementCount * sizeof(int32_t)));
//   array_data[0] = 2147483647; // Initialize the first element with MAX_INT
//   obj->SetIndexedPropertiesToExternalArrayData(array_data,
//                                                v8::kExternalIntArray,
//                                                kElementCount);
//   context->Global()->Set(v8_str("ext_array"), obj);
//   const char* boundary_program =
//       "var res = 0;"
//       "for (var i = 0; i < 3; i++) {"
//       "  res = ext_array[0];" // Get the first element
//       "}"
//       "res;";
//   v8::Handle<v8::Value> result = CompileRun(boundary_program);
//   CHECK_EQ((int64_t)2147483647, result->IntegerValue());
// }

// // Regression test for a defect in the actual number of argument count.
// // Handling of actual arguments was wrong when the arguments number
// // was greater than the formal arguments number in
// // Builtins::Generate_ArgumentsAdaptorTrampoline().
// // Used to return always 0 (formal arguments number of Foo)
// // in this test instead of the actual arguments number.
// THREADED_TEST(IssueActualArgumentsNum) {
//   v8::HandleScope scope;
//   LocalContext context;
//   CompileRun(
//     "function Foo() {"
//     "  return arguments.length;"
//     "}");
//   Local<Function> Foo =
//       Local<Function>::Cast(context->Global()->Get(v8_str("Foo")));

//   v8::Handle<Value>* args0 = NULL;
//   Local<v8::Integer> l0 = Local<v8::Integer>::Cast(Foo->Call(Foo, 0, args0));
//   CHECK_EQ((int64_t)0, l0->Value());

//   v8::Handle<Value> args1[] = { v8_num(1.1) };
//   Local<v8::Integer> l1 = Local<v8::Integer>::Cast(Foo->Call(Foo, 1, args1));
//   CHECK_EQ((int64_t)1, l1->Value());

//   v8::Handle<Value> args2[] = { v8_num(2.2),
//                                 v8_num(3.3) };
//   Local<v8::Integer> l2 = Local<v8::Integer>::Cast(Foo->Call(Foo, 2, args2));
//   CHECK_EQ((int64_t)2, l2->Value());

//   v8::Handle<Value> args3[] = { v8_num(4.4),
//                                 v8_num(5.5),
//                                 v8_num(6.6) };
//   Local<v8::Integer> l3 = Local<v8::Integer>::Cast(Foo->Call(Foo, 3, args3));
//   CHECK_EQ((int64_t)3, l3->Value());

//   v8::Handle<Value> args4[] = { v8_num(7.7),
//                                 v8_num(8.8),
//                                 v8_num(9.9),
//                                 v8_num(10.11) };
//   Local<v8::Integer> l4 = Local<v8::Integer>::Cast(Foo->Call(Foo, 4, args4));
//   CHECK_EQ((int64_t)4, l4->Value());
// }


// Extracted from test-api/CatchStackOverflow
// Fails with a sigsegv on sh4 QEMU.
TEST(IssueCatchStackOverflow) {
  i::FLAG_stack_size = 400; // Reduce stack size to speedup test
  CcTest::InitializeVM();
  HandleScope scope(CcTest::i_isolate());
  LocalContext context;
  v8::TryCatch try_catch;
  const char *source_exception =
    "function f() {"
    "  return f();"
    "}"
    ""
    "f();";
  CompileRun(source_exception);
  CHECK(try_catch.HasCaught());
  v8::Handle<v8::Message> message = try_catch.Message();
  CHECK(!message.IsEmpty());
  CHECK_EQ(1, message->GetLineNumber());
  v8::String::Utf8Value message_str(message->Get());
  CHECK(strstr(*message_str, "Maximum call stack size exceeded"));
}


// Extracted from test-api/Regress528
// Does not return the correct line source information in the exception.
// Returns 0 as line number instead of 1.
// The problem was in assembler-sh4 where one must call
// positions_recorder()->WriteRecordedPositions() for all jsr/jmp
TEST(IssueTryCatchSourceInfo) {
  CcTest::InitializeVM();
  HandleScope scope(CcTest::i_isolate());
  LocalContext context;
  v8::TryCatch try_catch;
  const char* source_exception = "function f(){throw 1;} f()";
  CompileRun(source_exception);
  CHECK(try_catch.HasCaught());
  v8::Handle<v8::Message> message = try_catch.Message();
  CHECK(!message.IsEmpty());
  CHECK_EQ(1, message->GetLineNumber());
}


// Extracted from mjsunit/stack-traces.js
// Does not return the correct eval location
// when compiled with debuggersupport=off.
// For SH4, we activate debuggersupport=on but skip tests in
// test-debug.cc until SH4 debug-sh4.cc is implemented (ref test-debug.cc).
TEST(IssueEvalStackTrace) {
  CcTest::InitializeVM();
  HandleScope scope(CcTest::i_isolate());
  LocalContext context;
  v8::TryCatch try_catch;
  const char *source_exception = "try { eval(\"FAIL\") } catch(e) { e.stack }";
  v8::Handle<v8::Script> script =
    v8::Script::Compile(v8::String::New(source_exception));
  v8::Handle<v8::String> result = v8::Handle<v8::String>::Cast(script->Run());
  v8::String::Utf8Value message_str(result);
  CHECK(strstr(*message_str, "FAIL is not defined")); // Check error
  CHECK(strstr(*message_str, "at eval")); // check that there is a stack trace
  CHECK(strstr(*message_str, "(eval at")); // check that source info is present
}
