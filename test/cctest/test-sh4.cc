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

#include <stdlib.h>

#include "v8.h"

#include "platform.h"
#include "cctest.h"

using namespace v8::internal;

/*
  Use this macro to setup the VM at the start of each test.
  By default use BEGIN_WITH_NATIVE(false) if natives are
  not needed. Only when needed, use BEGIN_WITH_NATIVE(true).
*/
#define BEGIN_WITH_NATIVE(native)                       \
  /* Set compilation of natives. */                     \
  i::FLAG_disable_native_files = !(native);             \
  /* Disable slow asserts that require natives. */      \
  i::FLAG_enable_slow_asserts = false;                  \
                                                        \
  CcTest::InitializeVM();                               \
  Isolate* isolate = CcTest::i_isolate();               \
  HandleScope scope(isolate);

TEST(sh4) {
  BEGIN_WITH_NATIVE(false)

  const char* c_source = "0x1234;";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0x1234,  script->Run()->Int32Value());
}

TEST(sh4_1) {
  BEGIN_WITH_NATIVE(false)

  const char* c_source = "0x1234 + 0x10;";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0x1244,  script->Run()->Int32Value());
}

TEST(sh4_2) {
  BEGIN_WITH_NATIVE(false)

  const char* c_source = "function foo() { return 0x1234; }; foo();";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0x1234,  script->Run()->Int32Value());
}

TEST(sh4_3) {
  BEGIN_WITH_NATIVE(true)

  const char* c_source = "\"foo\" + \"bar\" != \"foobar\";";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0,  script->Run()->Int32Value());
}

TEST(sh4_4) {
  BEGIN_WITH_NATIVE(false)

  const char* c_source = "0x1235 - 1;";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0X1234,  script->Run()->Int32Value());
}

TEST(sh4_5) {
  BEGIN_WITH_NATIVE(false)

  const char* c_source = "0x123 * 16;";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0X1230,  script->Run()->Int32Value());
}

