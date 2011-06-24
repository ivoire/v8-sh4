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

#define GLOBAL_FUNCTION_PTR() (isolate->global_context()->closure())
#define GLOBAL_PTR() (*(isolate->global()))
#define GLOBAL_CONTEXT_PTR() (*(isolate->global_context()))
#define GLOBAL_CLOSURE_PTR() (isolate->global_context()->closure())
#define GLOBAL_OBJECT_FUNCTION_PTR() (isolate->global_context()->object_function())

#define CMT(msg) { Comment cmnt(&assm, msg); } while(0)

#define __ assm.

// This macro stores in r10 the line number before branching to the error label.
// At the error label r10 can be moved to r0 such that return code of the
// function if not 0 indicates an error at the line of the branch.
#define B_LINE(cond, target) do { \
    __ mov(r10, Immediate(__LINE__)); \
    if (cond == al) { \
      __ b(target);  \
    } else { \
      __ b(cond, target);  \
    } \
  } while(0);


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


// Test CompareInstanceType(), CompareObjectType()
TEST(sh4_cs_2) {
  BEGIN();

  Label error;

  // Check that roots is actually te root object
  CMT("Check roots");
  __ mov(r0, Operand(ExternalReference::roots_address(assm.isolate())));
  __ cmp(r0, roots);
  B_LINE(ne, &error);
  
  // Check that cp is actually the current context
  CMT("Check cp");
  __ mov(r0, Operand((intptr_t)GLOBAL_CONTEXT_PTR(), 
		     RelocInfo::EXTERNAL_REFERENCE));
  __ cmp(r0, cp);
  B_LINE(ne, &error);

  CMT("Check MemOperand(cp, GLOBAL_INDEX) == GLOBAL_PTR())");
  __ ldr(r0, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  __ mov(r1, Operand((intptr_t)GLOBAL_PTR(), RelocInfo::EXTERNAL_REFERENCE));
  __ cmp(r0, r1);
  B_LINE(ne, &error);

  CMT("Check FieldMemOperand(GLOBAL_PTR(), GlobalContextOffset) == GLOBAL_CONTEXT_PTR())");
  __ ldr(r0, FieldMemOperand(r0,
			     GlobalObject::kGlobalContextOffset));
  __ mov(r1, Operand((intptr_t)GLOBAL_CONTEXT_PTR(), 
		     RelocInfo::EXTERNAL_REFERENCE));
  __ cmp(r0, r1);
  B_LINE(ne, &error);

  CMT("Check MemOperand(GLOBAL_CONTEXT_PTR(), CLOSURE_INDEX) == GLOBAL_CLOSURE_PTR())");
  __ ldr(r0, MemOperand(r0,
			Context::SlotOffset(Context::CLOSURE_INDEX)));
  __ mov(r1, Operand((intptr_t)GLOBAL_CLOSURE_PTR(), 
		     RelocInfo::EXTERNAL_REFERENCE));
  __ cmp(r0, r1);
  B_LINE(ne, &error);

  CMT("Check CompareInstanceType(GLOBAL_PTR()) == JS_GLOBAL_OBJECT_TYPE");
  __ mov(r0, Operand((intptr_t)GLOBAL_PTR(), 
		     RelocInfo::EXTERNAL_REFERENCE));
  __ ldr(r1, FieldMemOperand(r0, HeapObject::kMapOffset));
  __ CompareInstanceType(r1, r2/*type*/ , JS_GLOBAL_OBJECT_TYPE, eq);
  B_LINE(f, &error);
  __ cmp(r2, Immediate(JS_GLOBAL_OBJECT_TYPE));
  B_LINE(f, &error);
  CMT("Check CompareInstanceType(GLOBAL_PTR()) != JS_VALUE_TYPE");
  __ CompareInstanceType(r1, r2/*type*/ , JS_VALUE_TYPE, eq);
  B_LINE(t, &error);
  CMT("Check CompareInstanceType(GLOBAL_PTR()) >= (ge) FIRST_JS_OBJECT_TYPE");
  __ CompareInstanceType(r1, r2/*type*/ , FIRST_JS_OBJECT_TYPE, hs);
  B_LINE(f, &error);
  CMT("Check CompareInstanceType(GLOBAL_PTR()) >= (hs) FIRST_JS_OBJECT_TYPE");
  __ CompareInstanceType(r1, r2/*type*/ , FIRST_JS_OBJECT_TYPE, ge);
  B_LINE(f, &error);
  CMT("Check CompareInstanceType(GLOBAL_PTR()) ! >= FIRST_FUNCTION_CLASS_TYPE");
  __ CompareInstanceType(r1, r2/*type*/ , FIRST_FUNCTION_CLASS_TYPE, ge);
  B_LINE(t, &error);

  CMT("Check CompareObjectType(GLOBAL_PTR()) == JS_GLOBAL_OBJECT_TYPE");
  __ CompareObjectType(r0, r3/*map*/, r4/*type*/ , JS_GLOBAL_OBJECT_TYPE, eq);
  B_LINE(f, &error);
  __ cmp(r1, r3); // check that map matches
  B_LINE(f, &error);
  __ cmp(r2, r4); // check that type matches

  // All ok.
  __ mov(r0, Immediate(Smi::FromInt(0)));
  __ rts();

  __ bind(&error);
  __ SmiTag(r0, r10);
  __ rts();

  JIT();
  
  PRINT();

  CALL();

  // The function must return a result as Smi.
  CHECK(result->IsSmi());
  CHECK_EQ(0, Smi::cast(*result)->value());
}


// Test LoadGlobalFunction(), LoadContext(), LoadGlobalFunctionInitialMap()
// TryGetFunctionPrototype()
TEST(sh4_cs_3) {
  BEGIN();

  Label error;

  // Check LoadContext()
  __ LoadContext(r0, 0);
  __ mov(r1, Operand(ExternalReference(Isolate::k_context_address, isolate)));
  __ ldr(r1, MemOperand(r1));
  __ cmp(r0, r1);
  B_LINE(ne, &error);
  // Get cp from isolate directly
  __ mov(r1, Operand((intptr_t)isolate->context(), 
		     RelocInfo::EXTERNAL_REFERENCE));
  __ cmp(r0, r1);
  B_LINE(ne, &error);

  // Check LoadContext with context level 1 and 2.
  // As we are in global context, it alwasy returns the global context
  // TODO: force a deeper context when initalizing the VM
  __ LoadContext(r1, 1);
  __ cmp(r0, r1);
  B_LINE(ne, &error);
  __ LoadContext(r1, 2);
  __ cmp(r0, r1);
  B_LINE(ne, &error);


  // Check LoadGlobalFunction()
  __ LoadGlobalFunction(Context::CLOSURE_INDEX, r0/* Resulting closure*/);
  __ mov(r1, Operand((intptr_t)GLOBAL_CLOSURE_PTR(), 
		     RelocInfo::EXTERNAL_REFERENCE));
  __ cmp(r0, r1);
  B_LINE(ne, &error);

  // Check LoadGlobalFunctionInitialMap() on global object function
  __ LoadGlobalFunction(Context::OBJECT_FUNCTION_INDEX, r0/* Resulting closure*/);
  __ mov(r1, Operand((intptr_t)GLOBAL_OBJECT_FUNCTION_PTR(), 
		     RelocInfo::EXTERNAL_REFERENCE));
  
  __ cmp(r0, r1);
  B_LINE(ne, &error);
  __ LoadGlobalFunctionInitialMap(r0, r1/*result map*/, r2/*scratch*/);
  __ mov(r0, Operand((intptr_t)GLOBAL_OBJECT_FUNCTION_PTR()->
		     prototype_or_initial_map(), 
		     RelocInfo::EXTERNAL_REFERENCE));
  __ cmp(r1, r0);
  B_LINE(ne, &error);
  
  Label miss1, miss2, miss3, miss4, skip_proto4;
  CMT("Check TryGetFunctionPrototype: Smi");
  __ mov(r0, Immediate(Smi::FromInt(1)));
  __ TryGetFunctionPrototype(r0, r1 /*proto*/, r2 /*scratch*/, &miss1);
  B_LINE(al, &error);
  __ bind(&miss1);
  CMT("Check TryGetFunctionPrototype: Not a function");
  __ LoadRoot(r0, Heap::kTheHoleValueRootIndex);
  __ TryGetFunctionPrototype(r0, r1 /*proto*/, r2 /*scratch*/, &miss2);
  B_LINE(al, &error);
  __ bind(&miss2);
  CMT("Check TryGetFunctionPrototype(empty_function) == the_hole_value");
  __ LoadGlobalFunction(Context::CLOSURE_INDEX, r0/* Resulting closure*/);
  __ TryGetFunctionPrototype(r0, r1 /*proto*/, r2 /*scratch*/, &miss3);
  B_LINE(al, &error);
  __ bind(&miss3);
  __ LoadRoot(r2, Heap::kTheHoleValueRootIndex);
  __ cmp(r1, r2);
  B_LINE(ne, &error);

  CMT("Check TryGetFunctionPrototype(global_object_function) == initial_map->prototype()");
  __ LoadGlobalFunction(Context::OBJECT_FUNCTION_INDEX, r0/* Resulting closure*/);
  __ TryGetFunctionPrototype(r0, r1 /*proto*/, r2 /*scratch*/, &miss4);
  __ jmp(&skip_proto4);
  __ bind(&miss4);
  B_LINE(al, &error);
  __ bind(&skip_proto4);
  __ LoadGlobalFunctionInitialMap(r0, r3/*result map*/, r2/*scratch*/);
  __ ldr(r3, FieldMemOperand(r3, Map::kPrototypeOffset));
  __ cmp(r1, r3);
  B_LINE(ne, &error);

  // All ok.
  __ mov(r0, Immediate(Smi::FromInt(0)));
  __ rts();

  __ bind(&error);
  __ SmiTag(r0, r10);
  __ rts();

  JIT();
  
  PRINT();

  CALL();

  // The function must return a result as Smi.
  CHECK(result->IsSmi());
  CHECK_EQ(0, Smi::cast(*result)->value());
}						     

