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

  __ asl(r0, r4, Immediate(17));
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

  __ asr(r0, r4, r5);
  __ asr(r0, r0, Immediate(1), r4);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 78945613, 4, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(78945613>>(4+1), res);
}


TEST(6) {
  BEGIN();

  __ lsl(r0, r4, Immediate(17), r4);
  __ lsl(r0, r0, Immediate(1));
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

TEST(7) {
  BEGIN();

  __ lsr(r0, r4, r5);
  __ lsr(r0, r0, Immediate(1), r4);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 78945613, 4, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(78945613>>(4+1), res);
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
  __ cmpeq(r0, Immediate(0));
  __ bt(&end);

  for(int i = 0; i < 2000; i++)
    __ add(r0, r0, Immediate(1));
  __ cmpeq(r0, Immediate(0));
  __ bt(&end);

  for(int i = 0; i < 2000; i++)
    __ add(r0, r0, Immediate(1));
  __ cmpeq(r0, Immediate(0));
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


#undef __
