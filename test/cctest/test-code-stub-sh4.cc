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

#include "macro-assembler.h"
#include "factory.h"
#include "platform.h"
#include "serialize.h"
#include "cctest.h"

using namespace v8::internal;

Handle<JSFunction> 
CreateJSFunctionFromCode(const char *name, Code *code, Isolate* isolate) {
  fprintf(stderr, "%s\n", __FUNCTION__);

  Factory* factory = isolate->factory();
  
  // Allocate the function
  Handle<String> symbol = factory->LookupAsciiSymbol(name);
  Handle<JSFunction> new_function =
      factory->NewFunctionWithoutPrototype(symbol, kNonStrictMode);

  // Bind the code
  new_function->set_code(code);
  new_function->shared()->set_code(code);
  Handle<String> source = factory->NewStringFromAscii(CStrVector("() {}"));
  Handle<Script> script = factory->NewScript(source);
  script->set_type(Smi::FromInt(Script::TYPE_NATIVE));
  new_function->shared()->set_script(*script);
  new_function->shared()->set_start_position(0);
  new_function->shared()->set_end_position(source->length());
  new_function->shared()->DontAdaptArguments();

  fprintf(stderr, "%s: %p\n", __FUNCTION__, (void *)*new_function);

  return new_function;
}

#define BEGIN() \
  FLAG_disable_native_files = true; \
  FLAG_full_compiler = false; \
  v8::HandleScope scope; \
  Isolate* isolate = Isolate::Current(); \
  LocalContext env; \
  MacroAssembler assm(Isolate::Current(), NULL, 0);

  
#define JIT() \
  CodeDesc desc; \
  assm.GetCode(&desc); \
  Code* code = Code::cast(HEAP-> \
			  CreateCode(desc, \
				     Code::ComputeFlags(Code::BUILTIN), \
				     Handle<Object>(HEAP->null_value()))-> \
			  ToObjectChecked()); \
  Handle<JSFunction> func = CreateJSFunctionFromCode(__FUNCTION__, \
						     code, isolate);

#define CALL() \
  CHECK(func->code()->IsCode());		\
  bool exc; \
  Handle<Object> receiver(isolate->context()->global_proxy(), isolate); \
  Handle<Object> result = Execution::Call(func, receiver, 0, NULL, &exc); \
  if (exc) { result = isolate->factory()->nan_value(); } \
  (void)0

#ifdef DEBUG
#define PRINT()  Code::cast(func->code())->Print()
#else
#define PRINT() (void)0
#endif

    
#define __ assm.


// Test Empty function call
// This covers JSEntryStub (call to the empty function) builtin.
// This covers CEntryStub (call to kEmptyFunction) builtin.
// This Covers also Generate_Adaptor() and JumpToExternalReference()
TEST(sh4_cs_0) {
  BEGIN();

  // Force use of empty function (global_context()->closure())
  Handle<JSFunction> func = 
    Handle<JSFunction>(isolate->global_context()->closure(), isolate);
  
  fprintf(stderr, "isolate = Isolate::Current(): %p\n", 
	  (void *)isolate);
  fprintf(stderr, "isolate_context = isolate->context(): %p\n", 
	  (void *)isolate->context());
  fprintf(stderr, "global_context = isolate->global_context(): %p\n",
	  (void *)*isolate->global_context());
  fprintf(stderr, "global_context_closure = global_context->closure(): %p\n",
	  (void *)isolate->global_context()->closure());
  fprintf(stderr, "global = isolate->global(): %p\n", 
	  (void *)*isolate->global());
  fprintf(stderr, "global_proxy = isolate->global_proxy(): %p\n", 
	  (void *)isolate->context()->global_proxy());
  fprintf(stderr, "closure_code = global_context_closure->code(): %p \n",
	  (void *)func->code());
  fprintf(stderr, "closure_entry = code->entry(): %p \n",
	  (void *)func->code()->entry());

  PRINT();

  CALL();

  // The empty function returns the undefined value.
  CHECK(result->IsHeapObject());
  CHECK(HeapObject::cast(*result)->IsUndefined());
}

// Test empty function that scratches all JS registers 
// (except roots, cp, fp, sp)
TEST(sh4_cs_1) {
  BEGIN();

  // We are in a JS context, thus we can scratch caller and callee saved
  __ Dead(r0, r1, r2, r3);
  __ Dead(r4, r5, r6, r7);
  __ Dead(r8, r9, r10, r11);
  __ mov(r0, Immediate(Smi::FromInt(0)));
  __ rts();

  JIT();
  
  PRINT();

  CALL();

  // The function must return a result as Smi.
  CHECK(result->IsSmi());
  CHECK_EQ(0, Smi::cast(*result)->value());
}						     

