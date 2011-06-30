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

#include "disassembler.h"
#include "factory.h"
#include "sh4/simulator-sh4.h"
#include "sh4/assembler-sh4-inl.h"
#include "cctest.h"

using namespace v8::internal;


// Define these function prototypes to match JSEntryFunction in execution.cc.
typedef Object* (*F1)(int x, int p1, int p2, int p3, int p4);
typedef Object* (*F2)(int x, int y, int p2, int p3, int p4);
typedef Object* (*F3)(void* p0, int p1, int p2, int p3, int p4);
typedef Object* (*F4)(void* p0, void* p1, int p2, int p3, int p4);


static v8::Persistent<v8::Context> env;


static void InitializeVM() {
  if (env.IsEmpty()) {
    env = v8::Context::New();
  }
}

#define BEGIN()                                         \
  /* Disable compilation of natives. */                 \
  i::FLAG_disable_native_files = true;                  \
  i::FLAG_full_compiler = false;                        \
                                                        \
  InitializeVM();                                       \
  v8::HandleScope scope;                                \
  Assembler assm(Isolate::Current(), NULL, 0);

#define JIT()                                                           \
  CodeDesc desc;                                                        \
  assm.GetCode(&desc);                                                  \
  Object* code = HEAP->CreateCode(                                      \
      desc,                                                             \
      Code::ComputeFlags(Code::STUB),                                   \
      Handle<Object>(HEAP->undefined_value()))->ToObjectChecked();      \
  CHECK(code->IsCode());

#define __ assm.

TEST(0) {
  BEGIN();

  __ add(r0, r4, r5);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 3, 4, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(3+4, res);
}

TEST(1) {
  BEGIN();

  __ add(r0, r4, r5);
  __ add(r0, r0, Immediate(123456789), r4);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, -10, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(1-10+123456789, res);
}

TEST(2) {
  BEGIN();

  __ add(r0, r4, r5);
  __ sub(r0, r0, Immediate(987654), r4);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, -10, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(1-10-987654, res);
}

TEST(3) {
  BEGIN();

  __ rsb(r0, r4, r5);
  __ rsb(r0, r0, Immediate(5678), r4);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 5, 123, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(5678-(123-5), res);
}

TEST(4) {
  BEGIN();

  __ asl(r0, r4, Immediate(17), r1);
  __ asl(r0, r0, Immediate(1));
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 42, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(42<<(17+1), res);
}

TEST(5) {
  BEGIN();

  __ asr(r1, r4, r5, r3);
  __ asr(r1, r1, Immediate(1), r3);
  __ asr(r4, r4, r5, r3);
  __ asr(r4, r4, Immediate(2), r3);
  __ add(r0, r4, r1);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x7fecba98, 4, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ((0x7fecba98>>(4+1))+(0x7fecba98>>(4+2)), res);
}


TEST(6) {
  BEGIN();

  __ lsl(r0, r4, Immediate(14), r1);
  __ lsl(r0, r0, Immediate(1));
  __ lsl(r4, r0, Immediate(2));
  __ mov(r1, Immediate(1));
  __ lsl(r4, r4, r1);
  __ lsl(r0, r4, r1);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 42, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(42<<(14+1+2+1+1), res);
}

TEST(7) {
  BEGIN();

  __ lsr(r0, r4, r5);
  __ mov(r1, Immediate(1));
  __ lsr(r0, r0, r1, r3);
  __ lsr(r4, r0, Immediate(1));
  __ lsr(r4, r4, Immediate(2));
  __ lsr(r0, r4, Immediate(3));
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x7fecba98, 4, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ((uint32_t)0x7fecba98>>(4+1+1+2+3), res);
}

TEST(8) {
  BEGIN();

  __ mov(r0, r4);
  for(int i = 0; i < 10000; i++)
    __ add(r0, r0, Immediate(1));
  __ rts();

  JIT();
#ifdef DEBUG
//  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 12, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(10000+12, res);
}

TEST(9) {
  BEGIN();

  Label top, middle, bottom;

  __ cmpeq(r5, Immediate(12), r1);
  __ bt(&bottom);
  __ mov(r0, Immediate(0));
  __ rts();

  __ bind(&top);
  __ mov(r0, r4);
  __ rts();

  __ bind(&middle);
  __ add(r4, r4, Immediate(1));
  __ jmp(&top);

  __ bind(&bottom);
  __ add(r4, r4, Immediate(3));
  __ jmp(&middle);

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 42, 12, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(42+3+1, res);
}

TEST(10) {
  BEGIN();

  Label end;
  __ mov(r0, r4);

  for(int i = 0; i < 2000; i++)
    __ add(r0, r0, Immediate(1));
  __ cmpeq(r0, Immediate(0), r1);
  __ bt(&end);

  for(int i = 0; i < 2000; i++)
    __ add(r0, r0, Immediate(1));
  __ cmpeq(r0, Immediate(0), r1);
  __ bt(&end);

  for(int i = 0; i < 2000; i++)
    __ add(r0, r0, Immediate(1));
  __ cmpeq(r0, Immediate(0), r1);
  __ bf(&end);

  for(int i = 0; i < 2000; i++)
    __ add(r0, r0, Immediate(1));

  __ bind(&end);
  __ rts();

  JIT();
#ifdef DEBUG
//  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 12, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(3*2000+12, res);
}


TEST(11) {
  BEGIN();

  Label error;

  __ mov(r0, Immediate(0));
  __ cmpeq(r4, Immediate(-27), r1);     // true
  __ bf(&error);

  __ cmpgt(r5, Immediate(546), r1);     // false
  __ bt(&error);

  __ cmpgt(r5, Immediate(545), r1);     // true
  __ bf(&error);

  __ cmpgtu(r4, Immediate(-27), r1);    // false
  __ bt(&error);

  __ cmpgtu(r4, Immediate(-28), r1);    // true
  __ bf(&error);

  __ cmpge(r5, Immediate(546), r1);     // true
  __ bf(&error);

  __ cmpge(r5, Immediate(545), r1);     // false
  __ bt(&error);

  __ cmpgeu(r4, Immediate(-27), r1);    // true
  __ bf(&error);

  __ cmpgeu(r4, Immediate(-26), r1);    // false
  __ bt(&error);

  __ mov(r0, Immediate(1));
  __ rts();

  __ bind(&error);
  __ mov(r0, Immediate(1));
  __ rts();


  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, -27, 546, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(1, res);
}

TEST(12) {
  BEGIN();

  Label error;

  __ pushm(r4.bit() | r5.bit() | r6.bit() | r7.bit());
  __ popm(r0.bit() | r1.bit() | r2.bit() | r3.bit());

  __ cmpeq(r0, r4);
  __ bf(&error);
  __ cmpeq(r1, r5);
  __ bf(&error);
  __ cmpeq(r2, r6);
  __ bf(&error);
  __ cmpeq(r3, r7);
  __ bf(&error);

  __ pushm(r4.bit() | r5.bit() | r6.bit() | r7.bit());
  __ pop(r0);
  __ pop(r1);
  __ add(r0, r0, r1);

  __ pop(r1);
  __ add(r0, r0, r1);

  __ pop(r1);
  __ add(r0, r0, r1);

  __ rts();


  __ bind(&error);
  __ mov(r0, Immediate(0));
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 12, 816, 53, 6543, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(12+816+53+6543, res);
}

TEST(13) {
  BEGIN();

  Label error;

  __ mov(r0, Immediate(2));
  __ push(r0);
  __ mov(r0, Immediate(3));
  __ push(r0);
  __ mov(r0, Immediate(5));
  __ push(r0);
  __ mov(r0, Immediate(7));
  __ push(r0);

  __ mov(r0, MemOperand(sp, 3 * sizeof(void*)));
  __ cmpeq(r0, Immediate(2), r1);
  __ bf(&error);

  __ mov(r0, MemOperand(sp, 2 * sizeof(void*)));
  __ cmpeq(r0, Immediate(3), r1);
  __ bf(&error);

  __ mov(r0, MemOperand(sp, 1 * sizeof(void*)));
  __ cmpeq(r0, Immediate(5), r1);
  __ bf(&error);

  __ mov(r0, MemOperand(sp, 0 * sizeof(void*)));
  __ cmpeq(r0, Immediate(7), r1);
  __ bf(&error);

  __ mov(r0, Immediate(1));
  __ pop(r1);
  __ pop(r1);
  __ pop(r1);
  __ pop(r1);
  __ rts();

  __ bind(&error);
  __ pop(r1);
  __ pop(r1);
  __ pop(r1);
  __ pop(r1);
  __ mov(r0, Immediate(0));
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(1, res);
}

TEST(14) {
  BEGIN();

  Label begin, end;

  __ mov(r0, Immediate(0));
  __ bind(&begin);
  __ cmpeq(r0, Immediate(0), r1);
  __ bf(&end);

  for(int i = 0; i < 10000; i++)
    __ add(r0, r0, Immediate(1), r1);

  __ jmp(&begin);

  __ bind(&end);
  __ rts();

  JIT();
#ifdef DEBUG
//  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(10000, res);
}

TEST(15) {
  BEGIN();

  Label error;

  __ mov(r1, Immediate(1));
  __ mov(r4, Immediate(2147483647));
  __ addv(r1, r1, r4);          // set the T bit
  __ bf(&error, r2);

  __ mov(r1, Immediate(1));
  __ mov(r4, Immediate(2147483647-1));
  __ addv(r1, r4, r1);          // does not set the T bit
  __ bt(&error, r2);

  __ mov(r1, Immediate(1));
  __ mov(r4, Immediate(0));
  __ addv(r0, r1, r4);          // does not set the T bit
  __ bt(&error, r2);


  __ mov(r1, Immediate(1));
  __ mov(r4, Immediate(-2147483647-1));
  __ subv(r1, r4, r1);          // set the T bit
  __ bf(&error, r2);

  __ mov(r1, Immediate(1));
  __ mov(r4, Immediate(-2147483647));
  __ subv(r1, r4, r1);          // does not set the T bit
  __ bt(&error, r2);

  __ mov(r1, Immediate(1));
  __ mov(r4, Immediate(0));
  __ subv(r0, r1, r4);          // does not set the T bit
  __ bt(&error, r2);


  __ mov(r0, Immediate(1));
  __ rts();

  __ bind(&error);
  __ mov(r0, Immediate(0));
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(1, res);
}

// This macro stores in r10 the line number before branching to the error label.
// At the error label r10 can be moved to r0 such that return code of the
// function if not 0 indicates an error at the line of the branch.
#define B_LINE(cond, target) do {	   \
  __ mov(r10, Immediate(__LINE__)); \
  __ b(cond, target);  \
  } while(0);

// Test logical and, or, xor
TEST(16) {
  BEGIN();

  Label error;

  __ mov(r1, Immediate(0x00ff00ff));
  __ land(r1, r1, Immediate(0xff00ff00), r2);
  __ cmpeq(r1, Immediate(0));                   // true
  B_LINE(f, &error);

  __ mov(r1, Immediate(0x00ffff00));
  __ land(r1, r1, Immediate(0xff00ff00));
  __ cmpeq(r1, Immediate(0x0000ff00));      // true
  B_LINE(f, &error);

  __ mov(r0, Immediate(0xff));
  __ land(r0, r0, Immediate(0x0f));
  __ cmpeq(r0, Immediate(0x0f));            // true
  B_LINE(f, &error);

  __ mov(r1, Immediate(0x0f0));
  __ mov(r2, Immediate(0xfff));
  __ land(r3, r1, r2);
  __ cmpeq(r3, Immediate(0x0f0));           // true
  B_LINE(f, &error);

  __ land(r1, r1, r2); // left auto-modifying
  __ cmpeq(r1, Immediate(0x0f0));           // true
  B_LINE(f, &error);


  __ mov(r1, Immediate(0x0f0));
  __ land(r1, r2, r1); // right auto-modifying
  __ cmpeq(r1, Immediate(0x0f0));           // true
  B_LINE(f, &error);


  __ mov(r1, Immediate(0x00ff00ff));
  __ lor(r1, r1, Immediate(0xff00ff00));
  __ cmpeq(r1, Immediate(0xffffffff));          // true
  B_LINE(f, &error);

  __ mov(r1, Immediate(0x00ffff00));
  __ lor(r1, r1, Immediate(0xff00ff00));
  __ cmpeq(r1, Immediate(0xffffff00));      // true
  B_LINE(f, &error);

  __ mov(r0, Immediate(0xf0));
  __ lor(r0, r0, Immediate(0x0e));
  __ cmpeq(r0, Immediate(0xfe));            // true
  B_LINE(f, &error);

  __ mov(r1, Immediate(0x0f0));
  __ mov(r2, Immediate(0xf12));
  __ lor(r3, r1, r2);
  __ cmpeq(r3, Immediate(0xff2));           // true
  B_LINE(f, &error);

  __ lor(r1, r1, r2); // left auto-modifying
  __ cmpeq(r1, Immediate(0xff2));           // true
  B_LINE(f, &error);

  __ mov(r1, Immediate(0x0f0));
  __ lor(r1, r2, r1); // right auto-modifying
  __ cmpeq(r1, Immediate(0xff2));           // true
  B_LINE(f, &error);


  __ mov(r1, Immediate(0xffffffff));
  __ lxor(r1, r1, Immediate(0xff00ff00));
  __ cmpeq(r1, Immediate(0x00ff00ff));          // true
  B_LINE(f, &error);

  __ mov(r0, Immediate(0xff));
  __ lxor(r0, r0, Immediate(0x0e));
  __ cmpeq(r0, Immediate(0xf1));            // true
  B_LINE(f, &error);

  __ mov(r1, Immediate(0xff));
  __ mov(r2, Immediate(0x0f));
  __ lxor(r3, r1, r2);
  __ cmpeq(r3, Immediate(0xf0));           // true
  B_LINE(f, &error);

  __ lxor(r1, r1, r2); // left auto-modifying
  __ cmpeq(r1, Immediate(0xf0));           // true
  B_LINE(f, &error);

  __ mov(r1, Immediate(0x0ff));
  __ lxor(r1, r2, r1); // right auto-modifying
  __ cmpeq(r1, Immediate(0xf0));           // true
  B_LINE(f, &error);

  // All ok.
  __ mov(r0, Immediate(0));
  __ rts();

  __ bind(&error);
  __ mov(r0, r10);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


// Test conditional moves
TEST(17) {
  BEGIN();

  Label error;
  
  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(0));
  __ mov(r2, Immediate(0));
  __ tst(r0, r0);
  __ mov(r1, Immediate(1), eq);
  __ mov(r2, Immediate(1), ne);
  __ cmpeq(r1, Immediate(1));
  B_LINE(f, &error);
  __ cmpeq(r2, Immediate(0));
  B_LINE(f, &error);

  __ mov(r0, Immediate(1));
  __ mov(r1, Immediate(0));
  __ mov(r2, Immediate(0));
  __ tst(r0, r0);
  __ mov(r1, Immediate(1), eq);
  __ mov(r2, Immediate(1), ne);
  __ cmpeq(r1, Immediate(0));
  B_LINE(f, &error);
  __ cmpeq(r2, Immediate(1));
  B_LINE(f, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(0));
  __ mov(r2, Immediate(0));
  __ tst(r0, r0);
  __ mov(r1, Immediate(0xffff), eq);
  __ mov(r2, Immediate(0xffff), ne);
  __ cmpeq(r1, Immediate(0xffff));
  B_LINE(f, &error);
  __ cmpeq(r2, Immediate(0));
  B_LINE(f, &error);

  __ mov(r0, Immediate(1));
  __ mov(r1, Immediate(0));
  __ mov(r2, Immediate(0));
  __ tst(r0, r0);
  __ mov(r1, Immediate(0xffff), eq);
  __ mov(r2, Immediate(0xffff), ne);
  __ cmpeq(r1, Immediate(0));
  B_LINE(f, &error);
  __ cmpeq(r2, Immediate(0xffff));
  B_LINE(f, &error);
  
  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(0));
  __ mov(r2, Immediate(0));
  __ mov(r3, Immediate(1));
  __ tst(r0, r0);
  __ mov(r1, r3, eq);
  __ mov(r2, r3, ne);
  __ cmpeq(r1, Immediate(1));
  B_LINE(f, &error);
  __ cmpeq(r2, Immediate(0));
  B_LINE(f, &error);
  
  // All ok.
  __ mov(r0, Immediate(0));
  __ rts();

  __ bind(&error);
  __ mov(r0, r10);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


// Test addc/subc
TEST(18) {
  BEGIN();

  Label error;
  
  __ mov(r0, Immediate(0xFFFFFFFE));
  __ mov(r1, Immediate(1));
  __ addc(r2, r0, r1);
  B_LINE(cs, &error); // check that carry is clear
  __ cmpeq(r2, Immediate(0xFFFFFFFF));
  B_LINE(f, &error);

  __ addc(r1, r1, r0); // left auto-modified
  B_LINE(cs, &error);
  __ cmpeq(r1, Immediate(0xFFFFFFFF));
  B_LINE(f, &error);

  __ mov(r1, Immediate(1));
  __ addc(r1, r0, r1); // right auto-modified
  B_LINE(cs, &error);
  __ cmpeq(r1, Immediate(0xFFFFFFFF));
  B_LINE(f, &error);

  __ mov(r0, Immediate(0xFFFFFFFF));
  __ mov(r1, Immediate(1));
  __ addc(r2, r0, r1);
  B_LINE(cc, &error); // check that carry is set
  __ cmpeq(r2, Immediate(0));
  B_LINE(f, &error);

  __ addc(r1, r1, r0); // left auto-modified
  B_LINE(cc, &error);
  __ cmpeq(r1, Immediate(0));
  B_LINE(f, &error);

  __ mov(r1, Immediate(1));
  __ addc(r1, r0, r1); // right auto-modified
  B_LINE(cc, &error);
  __ cmpeq(r1, Immediate(0));
  B_LINE(f, &error);

   __ mov(r0, Immediate(1));
   __ mov(r1, Immediate(1));
   __ subc(r2, r0, r1);
   B_LINE(cs, &error); // check that carry is clear
   __ cmpeq(r2, Immediate(0));
   B_LINE(f, &error);

   __ subc(r0, r0, r1); // left auto-modified
   B_LINE(cs, &error);
   __ cmpeq(r0, Immediate(0));
   B_LINE(f, &error);

   __ mov(r0, Immediate(1));
   __ subc(r1, r0, r1); // right auto-modified
   B_LINE(cs, &error);
   __ cmpeq(r1, Immediate(0));
   B_LINE(f, &error);

   __ mov(r0, Immediate(0));
   __ mov(r1, Immediate(1));
   __ subc(r2, r0, r1);
   B_LINE(cc, &error); // check that carry is set
   __ cmpeq(r2, Immediate(-1));
   B_LINE(f, &error);

   __ subc(r0, r0, r1); // left auto-modified
   B_LINE(cc, &error);
   __ cmpeq(r0, Immediate(-1));
   B_LINE(f, &error);

   __ mov(r0, Immediate(0));
   __ subc(r1, r0, r1); // right auto-modified
   B_LINE(cc, &error);
   __ cmpeq(r1, Immediate(-1));
   B_LINE(f, &error);

  // All ok.
  __ mov(r0, Immediate(0));
  __ rts();

  __ bind(&error);
  __ mov(r0, r10);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// Test addv/subv
TEST(19) {
  BEGIN();

  Label error;
  
  __ mov(r0, Immediate(0x7FFFFFFE));
  __ mov(r1, Immediate(1));
  __ addv(r2, r0, r1);
  B_LINE(vs, &error); // check that overflow is clear
  __ cmpeq(r2, Immediate(0x7FFFFFFF));
  B_LINE(f, &error);

  __ addv(r1, r1, r0); // left auto-modified
  B_LINE(vs, &error);
  __ cmpeq(r1, Immediate(0x7FFFFFFF));
  B_LINE(f, &error);

  __ mov(r1, Immediate(1));
  __ addv(r1, r0, r1); // right auto-modified
  B_LINE(vs, &error);
  __ cmpeq(r1, Immediate(0x7FFFFFFF));
  B_LINE(f, &error);

  __ mov(r0, Immediate(0x7FFFFFFF));
  __ mov(r1, Immediate(1));
  __ addv(r2, r0, r1);
  B_LINE(vc, &error); // check that overflow is set
  __ cmpeq(r2, Immediate(0x80000000));
  B_LINE(f, &error);

  __ addv(r1, r1, r0); // left auto-modified
  B_LINE(vc, &error);
  __ cmpeq(r1, Immediate(0x80000000));
  B_LINE(f, &error);

  __ mov(r1, Immediate(1));
  __ addv(r1, r0, r1); // right auto-modified
  B_LINE(vc, &error);
  __ cmpeq(r1, Immediate(0x80000000));
  B_LINE(f, &error);

  __ mov(r0, Immediate(0x80000001));
  __ mov(r1, Immediate(1));
  __ subv(r2, r0, r1);
  B_LINE(vs, &error); // check that overflow is clear
  __ cmpeq(r2, Immediate(0x80000000));
  B_LINE(f, &error);

  __ subv(r0, r0, r1); // left auto-modified
  B_LINE(vs, &error);
  __ cmpeq(r0, Immediate(0x80000000));
  B_LINE(f, &error);

  __ mov(r0, Immediate(0x80000001));
  __ subv(r1, r0, r1); // right auto-modified
  B_LINE(vs, &error);
  __ cmpeq(r1, Immediate(0x80000000));
  B_LINE(f, &error);

  __ mov(r0, Immediate(0x80000000));
  __ mov(r1, Immediate(1));
  __ subv(r2, r0, r1);
  B_LINE(cc, &error); // check that carry is set
  __ cmpeq(r2, Immediate(0x7FFFFFFF));
  B_LINE(f, &error);

  __ subv(r0, r0, r1); // left auto-modified
  B_LINE(cc, &error);
  __ cmpeq(r0, Immediate(0x7FFFFFFF));
  B_LINE(f, &error);

  __ mov(r0, Immediate(0x80000000));
  __ subv(r1, r0, r1); // right auto-modified
  B_LINE(cc, &error);
  __ cmpeq(r1, Immediate(0x7FFFFFFF));
  B_LINE(f, &error);

  // All ok.
  __ mov(r0, Immediate(0));
  __ rts();

  __ bind(&error);
  __ mov(r0, r10);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

TEST(20) {
  BEGIN();

  Label error;
  Condition cond;

  cond = eq;
  __ cmp(cond, r4, Immediate(0), r0);
  CHECK_EQ(eq, cond);
  B_LINE(cond, &error);

  cond = ge;
  __ cmp(cond, r4, Immediate(0), r0);
  CHECK_EQ(eq, cond);
  B_LINE(f, &error);

  cond = lt;
  __ cmp(cond, r4, Immediate(654), r0);
  CHECK_EQ(ne, cond);
  B_LINE(t, &error);

  __ mov(r0, Immediate(0));
  __ rts();

  __ bind(&error);
  __ mov(r0, Immediate(1));
  __ rts();

  JIT();

#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 456, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

TEST(21) {
  BEGIN();

  Label top;
  __ bind(&top);
  __ cmpeq(r0, r1);
  __ cmpgt(r0, r1);
  __ cmpeq_r0_raw_immediate(73);
  __ bt(&top);
  __ bf(&top);
  __ mov(r0, Immediate(12));

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  CHECK_EQ(true, __ IsCmpRegister(((Instr*)desc.buffer)[0]));
  CHECK_EQ(false, __ IsCmpRegister(((Instr*)desc.buffer)[1]));

  CHECK_EQ(false, __ IsCmpImmediate(((Instr*)desc.buffer)[1]));
  CHECK_EQ(true, __ IsCmpImmediate(((Instr*)desc.buffer)[2]));
  CHECK_EQ(true, (__ GetCmpImmediateRegister(((Instr*)desc.buffer)[2])).is(r0));
  CHECK_EQ(73, __ GetCmpImmediateRawImmediate(((Instr*)desc.buffer)[2]));

  CHECK_EQ(true, (__ GetRn(((Instr*)desc.buffer)[0])).is(r0));
  CHECK_EQ(true, (__ GetRm(((Instr*)desc.buffer)[0])).is(r1));

  CHECK_EQ(eq, __ GetCondition(((Instr*)desc.buffer)[3]));
  CHECK_EQ(ne, __ GetCondition(((Instr*)desc.buffer)[5]));        // __ bt() generate bt/nop

  CHECK_EQ(true, __ IsMovImmediate(((Instr*)desc.buffer)[7]));
}

TEST(22) {
  BEGIN();

  Label function, end_function;

  __ mov(r5, Immediate(1));
  __ mov(r0, r4);
  __ add(r0, Immediate(1), r1);
  __ push(pr);
  __ jsr(&function);
  __ pop(pr);
  __ rts();

  __ bind(&function);
  __ add(r0, Immediate(1), r1);
  __ add(r0, Immediate(2), r1);
  __ add(r0, Immediate(3), r1);
  __ add(r0, Immediate(4), r1);
  __ cmpeq(r5, Immediate(0), r1);
  __ bt(&end_function);
  __ mov(r5, Immediate(0));
  __ push(pr);
  __ jsr(&function);
  __ pop(pr);

  __ bind(&end_function);
  __ rts();

  JIT();

#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 53, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(53+1+(1+2+3+4)*2, res);
}

TEST(22_bis) {
  BEGIN();

  Label function, label;

  __ mov(r0, Immediate(0));
  __ bind(&function);
  __ tst(r0, r0);
  __ bt(&label);
  __ rts();
  __ bind(&label);
  for(int i = 0; i < 10000; i++)
    __ add(r0, r0, Immediate(1), r1);

  __ push(pr);
  __ jsr(&function);
  __ pop(pr);
  __ rts();

  JIT();

#ifdef DEBUG
//  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(10000, res);
}

TEST(23) {
  BEGIN();
  Label error;

  __ mov(r0, Immediate(0));
  __ cmpeq(r0, Immediate(0), r4);
  __ mov(r1, Immediate(0x00ff00ff));
  __ mov(r2, Immediate(0xff00ff00));
  __ lor(r1, r1, r2, eq);
  __ cmpeq(r1, Immediate(0xffffffff));
  __ bf(&error);

  __ cmpeq(r0, Immediate(0), r4);
  __ mov(r1, Immediate(0x0000ff00));
  __ mov(r2, Immediate(0x000000ff));
  __ lor(r1, r1, r2, ne);
  __ cmpeq(r1, Immediate(0x0000ff00), r4);
  __ bf(&error);

  __ mov(r0, Immediate(0));
  __ cmpeq(r0, Immediate(0), r4);
  __ mov(r1, Immediate(0xffff00ff));
  __ lor(r1, r1, Immediate(0xffffeeff), eq, r2);
  __ cmpeq(r1, Immediate(0xffffeeff), r2);
  __ bf(&error);

  __ cmpeq(r0, Immediate(0), r4);
  __ mov(r1, Immediate(0x0000ff00));
  __ lor(r1, r1, Immediate(0x000000ff), ne, r2);
  __ cmpeq(r1, Immediate(0x0000ff00), r2);
  __ bf(&error);


  __ mov(r0, Immediate(0));
  __ rts();

  __ bind(&error);
  __ mov(r0, Immediate(1));
  __ rts();

  JIT();

#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

TEST(24) {
  BEGIN();

  Label error;

  __ dmuls(r1, r0, r4, r5);
  __ cmpeq(r0, r6);
  __ bf(&error);
  __ cmpeq(r1, r7);
  __ bf(&error);

  __ mov(r0, Immediate(0));
  __ rts();

  __ bind(&error);
  __ mov(r0, Immediate(1));

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 2, 0, 0, 0, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 3, 4, 0, 12, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 10, 45, 0, 10*45, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1000000000, 10000, 2328, 1316134912, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 123465879, 123465780, 3549226, 1458007724, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 13246579, 0, 0, 0, 0)));
}

