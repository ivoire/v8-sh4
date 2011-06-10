#include <stdlib.h>

#include "v8.h"

#include "platform.h"
#include "cctest.h"

using namespace v8::internal;


TEST(sh4) {
  // Disable compilation of natives.
  i::FLAG_disable_native_files = true;
  i::FLAG_full_compiler = false;

  v8::HandleScope scope;
  LocalContext env;  // from cctest.h

  const char* c_source = "0x1234;";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0x1234,  script->Run()->Int32Value());
}

TEST(sh4_1) {
  // Disable compilation of natives.
  i::FLAG_disable_native_files = true;
  i::FLAG_full_compiler = false;

  v8::HandleScope scope;
  LocalContext env;  // from cctest.h

  const char* c_source = "0x1234 + 0x10;";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0x1244,  script->Run()->Int32Value());
}

TEST(sh4_2) {
  // Disable compilation of natives.
  i::FLAG_disable_native_files = true;
  i::FLAG_full_compiler = false;

  v8::HandleScope scope;
  LocalContext env;  // from cctest.h

  const char* c_source = "function foo() { return 0x1234; }; foo();";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0x1234,  script->Run()->Int32Value());
}

TEST(sh4_3) {
  // Disable compilation of natives.
  i::FLAG_disable_native_files = false;
  i::FLAG_full_compiler = false;

  v8::HandleScope scope;
  LocalContext env;  // from cctest.h

  const char* c_source = "\"foo\" + \"bar\" != \"foobar\";";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0,  script->Run()->Int32Value());
}

TEST(sh4_4) {
  // Disable compilation of natives.
  i::FLAG_disable_native_files = true;
  i::FLAG_full_compiler = false;

  v8::HandleScope scope;
  LocalContext env;  // from cctest.h

  const char* c_source = "0x1235 - 1;";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0X1234,  script->Run()->Int32Value());
}

TEST(sh4_5) {
  // Disable compilation of natives.
  i::FLAG_disable_native_files = true;
  i::FLAG_full_compiler = false;

  v8::HandleScope scope;
  LocalContext env;  // from cctest.h

  const char* c_source = "0x123 * 16;";
  v8::Handle<v8::String> source = v8::String::New(c_source);
  v8::Handle<v8::Script> script = v8::Script::Compile(source);
  CHECK_EQ(0X1230,  script->Run()->Int32Value());
}

