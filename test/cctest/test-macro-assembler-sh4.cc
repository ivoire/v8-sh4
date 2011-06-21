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


typedef int (*F0)();


static v8::Persistent<v8::Context> env;


static void InitializeVM() {
  if (env.IsEmpty()) {
    env = v8::Context::New();
  }
}

// We allow use of callee saved r8-r11 for the regression tests
#define PROLOGUE() \
  do { \
  __ push(r8); \
  __ push(r9); \
  __ push(r10); \
  __ push(r11); \
  } while (0)

#define EPILOGUE() \
  do { \
  __ pop(r11); \
  __ pop(r10);	\
  __ pop(r9); \
  __ pop(r8); \
  } while (0)


#define BEGIN()                                         \
  /* Disable compilation of natives. */                 \
  i::FLAG_disable_native_files = true;                  \
  i::FLAG_full_compiler = false;                        \
                                                        \
  InitializeVM();                                       \
  v8::HandleScope scope;                                \
  MacroAssembler assm(Isolate::Current(), NULL, 0);

#define JIT()                                                           \
  CodeDesc desc;                                                        \
  assm.GetCode(&desc);                                                  \
  Object* code = HEAP->CreateCode(                                      \
      desc,                                                             \
      Code::ComputeFlags(Code::STUB),                                   \
      Handle<Object>(HEAP->undefined_value()))->ToObjectChecked();      \
  CHECK(code->IsCode());

#define __ assm.

// This macro stores in r10 the line number before branching to the error label.
// At the error label r10 can be moved to r0 such that return code of the
// function if not 0 indicates an error at the line of the branch.
#define B_LINE(cond, target) do {	   \
  __ mov(r10, Immediate(__LINE__)); \
  __ b(cond, target);  \
  } while(0);


// Test Move(...)
TEST(sh4_ma_0) {
  BEGIN();

  Label error;

  PROLOGUE();

  // Verify Move with Immediate
  __ Move(r0, Handle<Object>((Object *)0x12345678));
  __ mov(r1, Immediate(0x12345678));
  __ cmp(r0, r1);
  B_LINE(ne, &error);


  // Verify register move
  __ mov(r0, Immediate(~0x00110011)); // encodes 0xffeeffee
  __ Move(r1, r0);
  __ cmp(r0, r1);
  B_LINE(ne, &error); 

  
  // Verify register move to same register does not emit anything
  __ mov(r0, Immediate(~0x11001100)); // encodes 0xeeffeeff
  int offset = assm.pc_offset();
  __ Move(r0, r0);
  offset = assm.pc_offset() - offset;
  __ mov(r1, Immediate(offset));
  __ cmp(r1, Immediate(0));
  B_LINE(ne, &error); 

  
  // All ok.
  __ mov(r0, Immediate(0));
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ mov(r0, r10);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  int res = FUNCTION_CAST<F0>(Code::cast(code)->entry())();
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// Test Bfc/Ubfx/Sbfx
TEST(sh4_ma_1) {
  BEGIN();

  Label error;

  PROLOGUE();

  // Verify Bfc
  __ mov(r0, Immediate(~0x01233210)); // encodes 0xfdeccdef
  __ mov(r1, r0);
  __ Bfc(r1, 0, 31);
  __ cmp(r1, Immediate(~0x7fffffff)); // encodes 0x80000000
  B_LINE(ne, &error); 
  __ mov(r1, r0);
  __ Bfc(r1, 1, 30);
  __ cmp(r1, Immediate(~0x7ffffffe)); // encodes 0x80000001
  B_LINE(ne, &error); 
  __ mov(r1, r0);
  __ Bfc(r1, 9, 16);
  __ cmp(r1, Immediate(~0x01fffe10)); // encodes 0x0xfe0001ef
  B_LINE(ne, &error); 

  // Verify Ubfx
  __ mov(r0, Immediate(~0x01233210)); // encodes 0xfdeccdef
  __ Ubfx(r1, r0, 0, 32);
  __ cmp(r1, Immediate(~0x01233210)); // encodes 0xfdeccdef
  B_LINE(ne, &error); 
  __ Ubfx(r1, r0, 0, 31);
  __ cmp(r1, Immediate(0x7edccdef));
  B_LINE(ne, &error); 
  __ Ubfx(r1, r0, 1, 30);
  __ cmp(r1, Immediate(0x3f6e66f7));
  B_LINE(ne, &error); 
  __ Ubfx(r1, r0, 9, 16);
  __ cmp(r1, Immediate(0x6e66));
  B_LINE(ne, &error); 


  // Verify Sbfx
  __ mov(r0, Immediate(~0x01233210)); // encodes 0xfdeccdef
  __ Sbfx(r1, r0, 0, 31); // a mov actually
  __ cmp(r1, Immediate(~0x01233210)); // encodes 0xfdeccdef
  B_LINE(ne, &error); 
  __ Sbfx(r1, r0, 0, 31);
  __ cmp(r1, Immediate(~0x01233210)); // encodes 0xfedccdef
  B_LINE(ne, &error); 
  __ Sbfx(r1, r0, 1, 30);
  __ cmp(r1, Immediate(~0x00919908));  // encodes 0xff6e66f7
  B_LINE(ne, &error); 
  __ Sbfx(r1, r0, 9, 16);
  __ cmp(r1, Immediate(0x6e66));
  B_LINE(ne, &error); 

  // All ok.
  __ mov(r0, Immediate(0));
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ mov(r0, r10);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  int res = FUNCTION_CAST<F0>(Code::cast(code)->entry())();
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// Test Stack Pointer changes
TEST(sh4_ma_2) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r8, sp);

  __ add(sp, sp, Immediate(-16));

  __ mov(sp, r8);
    
  __ mov(r0, Immediate(0));

  EPILOGUE();
  __ rts();
  
  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  int res = FUNCTION_CAST<F0>(Code::cast(code)->entry())();
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


// Test Pop/Push
TEST(sh4_ma_3) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r8, sp);

  // Verify Pop and Push
  __ mov(r0, Immediate(3));
  __ mov(r1, Immediate(5));
  __ mov(r2, Immediate(7));
  __ mov(r3, Immediate(11));
  __ Push(r0,r1);
  __ mov(r0, Immediate(0xdeadbeef));
  __ mov(r1, Immediate(0xdeadbeef));
  __ Pop(r0,r1);
  __ cmp(r0, Immediate(3));
  B_LINE(ne, &error);
  __ cmp(r1, Immediate(5));
  B_LINE(ne, &error);

  __ Push(r0,r1,r2);
  __ mov(r0, Immediate(0xdeadbeef));
  __ mov(r1, Immediate(0xdeadbeef));
  __ mov(r2, Immediate(0xdeadbeef));
  __ pop(r2);
  __ Pop(r0,r1);
  __ cmp(r0, Immediate(3));
  B_LINE(ne, &error);
  __ cmp(r1, Immediate(5));
  B_LINE(ne, &error);
  __ cmp(r2, Immediate(7));
  B_LINE(ne, &error);

  __ Push(r0,r1,r2,r3);
  __ mov(r0, Immediate(0xdeadbeef));
  __ mov(r1, Immediate(0xdeadbeef));
  __ mov(r2, Immediate(0xdeadbeef));
  __ mov(r3, Immediate(0xdeadbeef));
  __ Pop(r2, r3);
  __ Pop(r0, r1);
  __ cmp(r0, Immediate(3));
  B_LINE(ne, &error);
  __ cmp(r1, Immediate(5));
  B_LINE(ne, &error);
  __ cmp(r2, Immediate(7));
  B_LINE(ne, &error);
  __ cmp(r3, Immediate(11));
  B_LINE(ne, &error);


  // All ok.
  __ mov(sp, r8);
  __ mov(r0, Immediate(0));
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ mov(sp, r8);
  __ mov(r0, r10);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  int res = FUNCTION_CAST<F0>(Code::cast(code)->entry())();
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// Test Ldrd/Strd
TEST(sh4_ma_4) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r8, sp);

  // Verify Pop and Push
  __ mov(r0, Immediate(3));
  __ mov(r1, Immediate(5));
  __ mov(r2, Immediate(7));
  __ mov(r3, Immediate(11));
  __ Push(r0,r1,r2,r3);
  __ mov(r0, Immediate(0xdeadbeef));
  __ mov(r1, Immediate(0xdeadbeef));
  __ mov(r2, Immediate(0xdeadbeef));
  __ mov(r3, Immediate(0xdeadbeef));

  __ Ldrd(r0, r1, MemOperand(sp, 8));
  __ cmp(r1, Immediate(3));
  B_LINE(ne, &error);
  __ cmp(r0, Immediate(5));
  B_LINE(ne, &error);

  __ Ldrd(r2, r3, MemOperand(sp, 0));
  __ cmp(r2, Immediate(11));
  B_LINE(ne, &error);
  __ cmp(r3, Immediate(7));
  B_LINE(ne, &error);

  __ Strd(r0, r1, MemOperand(sp, 0));
  __ Strd(r2, r3, MemOperand(sp, 8));
  __ Pop(r1,r0);
  __ Pop(r3,r2);
  __ cmp(r0, Immediate(5));
  B_LINE(ne, &error);
  __ cmp(r1, Immediate(3));
  B_LINE(ne, &error);
  __ cmp(r2, Immediate(11));
  B_LINE(ne, &error);
  __ cmp(r3, Immediate(7));
  B_LINE(ne, &error);
  
  // All ok.
  __ mov(sp, r8);
  __ mov(r0, Immediate(0));
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ mov(sp, r8);
  __ mov(r0, r10);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  int res = FUNCTION_CAST<F0>(Code::cast(code)->entry())();
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}
