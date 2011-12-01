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
#include "cctest.h"
#include "frames-inl.h"
#include "string-stream.h"

using ::v8::ObjectTemplate;
using ::v8::Value;
using ::v8::Context;
using ::v8::Local;
using ::v8::String;
using ::v8::Script;
using ::v8::Function;
using ::v8::AccessorInfo;
using ::v8::Extension;

// Simply returns the undefined object
static v8::Handle<Value> CallBack(Local<String> name,
                                    const AccessorInfo& info) {
  return v8::Undefined();
}

// Regression test for a defect in -use-ic mode.
// The bug returns a:
// Fatal error in src/objects.h, line 732
// CHECK(!IsFailure()) failed
// Note: 3 iterations of the loop are necessary to exhibit the issue
THREADED_TEST(CallbackAccessor) {
  v8::HandleScope scope;
  v8::Handle<v8::ObjectTemplate> obj = ObjectTemplate::New();
  i::StringStream::ClearMentionedObjectCache();
  obj->SetAccessor(v8_str("xxx"), CallBack);
  LocalContext env;
  env->Global()->Set(v8_str("obj"), obj->NewInstance());
  Script::Compile(String::New(
      "function foo() {"
      "  return obj.xxx;"
      "}"
      "for (var i = 0; i < 3; i++) {"
      "  foo();"
      "}"))->Run();
}
