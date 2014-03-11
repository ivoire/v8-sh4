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
#include "sh4/constants-sh4.h"
#include "sh4/simulator-sh4.h"
#include "sh4/assembler-sh4-inl.h"
#include "cctest.h"

using namespace v8::internal;


// Define these function prototypes to match JSEntryFunction in execution.cc.
typedef Object* (*F1)(int x, int p1, int p2, int p3, int p4);
typedef Object* (*F2)(int x, int y, int p2, int p3, int p4);
typedef Object* (*F3)(void* p0, int p1, int p2, int p3, int p4);
typedef Object* (*F4)(void* p0, void* p1, int p2, int p3, int p4);
typedef Object* (*F5)(double x, double y, int p2, int p3, int p4);


#define BEGIN()                                         \
  /* Disable compilation of natives. */                 \
  i::FLAG_disable_native_files = true;                  \
  /* Disable slow asserts that require natives. */      \
  i::FLAG_enable_slow_asserts = false;                  \
                                                        \
  CcTest::InitializeVM();                               \
  Isolate* isolate = CcTest::i_isolate();               \
  HandleScope scope(isolate);                           \
  Assembler assm(isolate, NULL, 0);

// Saves sh4_rtmp (r11) and sh4_ip (r10) which are calle saved
#define PROLOGUE() \
  __ push(r10); __ push(r11)
#define EPILOGUE() \
  __ pop(r11); __ pop(r10)

#define JIT()                                                           \
  CodeDesc desc;                                                        \
  assm.GetCode(&desc);                                                  \
  Object* code = isolate->heap()->CreateCode(                           \
      desc,                                                             \
      Code::ComputeFlags(Code::STUB),                                   \
      Handle<Code>())->ToObjectChecked();                               \
  CHECK(code->IsCode());

#define __ assm.

TEST(0) {
  BEGIN();

  PROLOGUE();
  __ add(r0, r4, r5);
  EPILOGUE();
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

  PROLOGUE();
  __ add(r0, r4, r5);
  __ add(r0, r0, Operand(123456789), r4);
  EPILOGUE();
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

  PROLOGUE();
  __ add(r0, r4, r5);
  __ sub(r0, r0, Operand(987654), r4);
  EPILOGUE();
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

  PROLOGUE();
  __ rsb(r0, r4, r5);
  __ rsb(r0, r0, Operand(5678), r4);
  EPILOGUE();
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

  PROLOGUE();
  __ asl(r0, r4, Operand(17), r1);
  __ asl(r0, r0, Operand(1));
  EPILOGUE();
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

  PROLOGUE();
  __ asr(r1, r4, r5, false, r3);
  __ asr(r1, r1, Operand(1), r3);
  __ asr(r4, r4, r5, false, r3);
  __ asr(r4, r4, Operand(2), r3);
  __ add(r0, r4, r1);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x7fecba98, 4,
                                                      0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ((0x7fecba98>>(4+1))+(0x7fecba98>>(4+2)), res);
}

TEST(5b) {
  BEGIN();

  PROLOGUE();
  __ asr(r0, r4, r5, false, r3);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 2,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 2,
                                                      0, 0, 0));
  CHECK_EQ(4, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 32,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, -16, 32,
                                                      0, 0, 0));
  CHECK_EQ(-1, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x80000000, 33,
                                                      0, 0, 0));
  CHECK_EQ(-1, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x7fffffff, 33,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
}

TEST(5c) {
  BEGIN();

  PROLOGUE();
  __ asr(r0, r4, r5, true, r3);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 2,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 2,
                                                      0, 0, 0));
  CHECK_EQ(4, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, 0,
                                                      0, 0, 0));
  CHECK_EQ(1, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x80000000, 31,
                                                      0, 0, 0));
  CHECK_EQ(-1, res);
}

TEST(6) {
  BEGIN();

  PROLOGUE();
  __ lsl(r0, r4, Operand(14), r1);
  __ lsl(r0, r0, Operand(1));
  __ lsl(r4, r0, Operand(2));
  __ mov(r0, Operand(0));
  __ add(r4, r4, r0);
  __ mov(r1, Operand(1));
  __ lsl(r4, r4, r1);
  __ lsl(r1, r4, r1);
  __ mov(r0, r1);
  EPILOGUE();
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


TEST(6b) {
  BEGIN();

  PROLOGUE();
  __ lsl(r0, r4, r5, false, r3);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 2,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 2,
                                                      0, 0, 0));
  CHECK_EQ(64, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 32,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, -16, 32,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x80000000, 33,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x7fffffff, 33,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
}


TEST(6c) {
  BEGIN();

  PROLOGUE();
  __ lsl(r0, r4, r5, true, r3);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 2,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 2,
                                                      0, 0, 0));
  CHECK_EQ(64, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, 0,
                                                      0, 0, 0));
  CHECK_EQ(1, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, 31,
                                                      0, 0, 0));
  CHECK_EQ((1<<31), res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, 32,
                                                      0, 0, 0));
  CHECK_EQ(1, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, 33,
                                                      0, 0, 0));
  CHECK_EQ(2, res);
}


TEST(7) {
  BEGIN();

  PROLOGUE();
  __ lsr(r0, r4, r5);
  __ mov(r1, Operand(1));
  __ lsr(r0, r0, r1, false, r3);
  __ lsr(r4, r0, Operand(1));
  __ lsr(r4, r4, Operand(2));
  __ lsr(r0, r4, Operand(3));

 EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x7fecba98, 4,
                                                      0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ((uint32_t)0x7fecba98>>(4+1+1+2+3), res);
}

TEST(7b) {
  BEGIN();

  PROLOGUE();
  __ lsr(r0, r4, r5, false, r3);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 2,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 2,
                                                      0, 0, 0));
  CHECK_EQ(4, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 32,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, -16, 32,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x80000000, 33,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x7fffffff, 33,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
}

TEST(7c) {
  BEGIN();

  PROLOGUE();
  __ lsr(r0, r4, r5, true, r3);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 2,
                                                      0, 0, 0));
  CHECK_EQ(0, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 16, 2,
                                                      0, 0, 0));
  CHECK_EQ(4, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, 0,
                                                      0, 0, 0));
  CHECK_EQ(1, res);
  res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0x80000000, 31,
                                                      0, 0, 0));
  CHECK_EQ(1, res);
}

TEST(8) {
  BEGIN();

  PROLOGUE();
  __ mov(r0, r4);
  for (int i = 0; i < 10000; i++)
    __ add(r0, r0, Operand(1));
  EPILOGUE();
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

  PROLOGUE();

  __ cmpeq(r5, Operand(12), r1);
  __ bt(&bottom);
  __ mov(r0, Operand(0));
  EPILOGUE();
  __ rts();

  __ bind(&top);
  __ mov(r0, r4);
  EPILOGUE();
  __ rts();

  __ bind(&middle);
  __ add(r4, r4, Operand(1));
  __ jmp(&top);

  __ bind(&bottom);
  __ add(r4, r4, Operand(3));
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

  PROLOGUE();
  __ mov(r0, r4);

  for (int i = 0; i < 2000; i++)
    __ add(r0, r0, Operand(1));
  __ cmpeq(r0, Operand(0), r1);
  __ bt(&end);

  for (int i = 0; i < 2000; i++)
    __ add(r0, r0, Operand(1));
  __ cmpeq(r0, Operand(0), r1);
  __ bt(&end);

  for (int i = 0; i < 2000; i++)
    __ add(r0, r0, Operand(1));
  __ cmpeq(r0, Operand(0), r1);
  __ bf(&end);

  for (int i = 0; i < 2000; i++)
    __ add(r0, r0, Operand(1));

  __ bind(&end);
  EPILOGUE();
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

  PROLOGUE();
  __ mov(r0, Operand(0));
  __ cmpeq(r4, Operand(-27), r1);     // true
  __ bf(&error);

  __ cmpgt(r5, Operand(546), r1);     // false
  __ bt(&error);

  __ cmpgt(r5, Operand(545), r1);     // true
  __ bf(&error);

  __ cmphi(r4, Operand(-27), r1);     // false
  __ bt(&error);

  __ cmphi(r4, Operand(-28), r1);     // true
  __ bf(&error);

  __ cmpge(r5, Operand(546), r1);     // true
  __ bf(&error);

  __ cmpge(r5, Operand(545), r1);     // false
  __ bt(&error);

  __ cmphs(r4, Operand(-27), r1);     // true
  __ bf(&error);

  __ cmphs(r4, Operand(-26), r1);     // false
  __ bt(&error);

  __ mov(r0, Operand(1));
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ mov(r0, Operand(1));
  EPILOGUE();
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

  PROLOGUE();
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

  EPILOGUE();
  __ rts();


  __ bind(&error);
  __ mov(r0, Operand(0));
  EPILOGUE();
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

  PROLOGUE();
  __ mov(r0, Operand(2));
  __ push(r0);
  __ mov(r0, Operand(3));
  __ push(r0);
  __ mov(r0, Operand(5));
  __ push(r0);
  __ mov(r0, Operand(7));
  __ push(r0);

  __ mov(r0, MemOperand(sp, 3 * sizeof(void*)));
  __ cmpeq(r0, Operand(2), r1);
  __ bf(&error);

  __ mov(r0, MemOperand(sp, 2 * sizeof(void*)));
  __ cmpeq(r0, Operand(3), r1);
  __ bf(&error);

  __ mov(r0, MemOperand(sp, 1 * sizeof(void*)));
  __ cmpeq(r0, Operand(5), r1);
  __ bf(&error);

  __ mov(r0, MemOperand(sp, 0 * sizeof(void*)));
  __ cmpeq(r0, Operand(7), r1);
  __ bf(&error);

  __ mov(r0, Operand(1));
  __ pop(r1);
  __ pop(r1);
  __ pop(r1);
  __ pop(r1);
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ pop(r1);
  __ pop(r1);
  __ pop(r1);
  __ pop(r1);
  __ mov(r0, Operand(0));
  EPILOGUE();
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

  Label begin, middle, end;

  PROLOGUE();
  __ mov(r0, Operand(0));
  __ b_near(&middle); // jump over landing area

  // make it a difficult landing for the backward jump
  __ misalign();
  __ bkpt();
  __ bkpt();
  __ bkpt();
  __ bkpt();
  __ bind(&begin);
  __ b_near(&middle); // branch wants to be misaligned
  __ bkpt();
  __ bkpt();
  __ bkpt();
  __ bkpt();

  __ bind(&middle);

  __ cmpeq(r0, Operand(0), r1);
  __ bf(&end); // not taken

  for (int i = 0; i < 10000; i++)
    __ add(r0, r0, Operand(1), r1);

  // good luck
  __ jmp(&begin);

  __ bind(&end);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(10000, res);
}

TEST(15) {
  BEGIN();

  Label error;

  PROLOGUE();
  __ mov(r1, Operand(1));
  __ mov(r4, Operand(2147483647));
  __ addv(r1, r1, r4);          // set the T bit
  __ bf(&error, r2);

  __ mov(r1, Operand(1));
  __ mov(r4, Operand(2147483647-1));
  __ addv(r1, r4, r1);          // does not set the T bit
  __ bt(&error, r2);

  __ mov(r1, Operand(1));
  __ mov(r4, Operand(0));
  __ addv(r0, r1, r4);          // does not set the T bit
  __ bt(&error, r2);


  __ mov(r1, Operand(1));
  __ mov(r4, Operand(-2147483647-1));
  __ subv(r1, r4, r1);          // set the T bit
  __ bf(&error, r2);

  __ mov(r1, Operand(1));
  __ mov(r4, Operand(-2147483647));
  __ subv(r1, r4, r1);          // does not set the T bit
  __ bt(&error, r2);

  __ mov(r1, Operand(1));
  __ mov(r4, Operand(0));
  __ subv(r0, r1, r4);          // does not set the T bit
  __ bt(&error, r2);


  __ mov(r0, Operand(1));
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ mov(r0, Operand(0));
  EPILOGUE();
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
#define B_LINE(cond, target) do {           \
  __ mov(r10, Operand(__LINE__)); \
  __ b(cond, target);  \
  } while (0);

// Test logical and, or, xor
TEST(16) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r1, Operand(0x00ff00ff));
  __ land(r1, r1, Operand(0xff00ff00), r2);
  __ cmpeq(r1, Operand(0));                   // true
  B_LINE(f, &error);

  __ mov(r1, Operand(0x00ffff00));
  __ land(r1, r1, Operand(0xff00ff00));
  __ cmpeq(r1, Operand(0x0000ff00));      // true
  B_LINE(f, &error);

  __ mov(r0, Operand(0xff));
  __ land(r0, r0, Operand(0x0f));
  __ cmpeq(r0, Operand(0x0f));            // true
  B_LINE(f, &error);

  __ mov(r1, Operand(0x0f0));
  __ mov(r2, Operand(0xfff));
  __ land(r3, r1, r2);
  __ cmpeq(r3, Operand(0x0f0));           // true
  B_LINE(f, &error);

  __ land(r1, r1, r2);  // left auto-modifying
  __ cmpeq(r1, Operand(0x0f0));           // true
  B_LINE(f, &error);


  __ mov(r1, Operand(0x0f0));
  __ land(r1, r2, r1);  // right auto-modifying
  __ cmpeq(r1, Operand(0x0f0));           // true
  B_LINE(f, &error);


  __ mov(r1, Operand(0x00ff00ff));
  __ lor(r1, r1, Operand(0xff00ff00));
  __ cmpeq(r1, Operand(0xffffffff));          // true
  B_LINE(f, &error);

  __ mov(r1, Operand(0x00ffff00));
  __ lor(r1, r1, Operand(0xff00ff00));
  __ cmpeq(r1, Operand(0xffffff00));      // true
  B_LINE(f, &error);

  __ mov(r0, Operand(0xf0));
  __ lor(r0, r0, Operand(0x0e));
  __ cmpeq(r0, Operand(0xfe));            // true
  B_LINE(f, &error);

  __ mov(r1, Operand(0x0f0));
  __ mov(r2, Operand(0xf12));
  __ lor(r3, r1, r2);
  __ cmpeq(r3, Operand(0xff2));           // true
  B_LINE(f, &error);

  __ lor(r1, r1, r2);  // left auto-modifying
  __ cmpeq(r1, Operand(0xff2));           // true
  B_LINE(f, &error);

  __ mov(r1, Operand(0x0f0));
  __ lor(r1, r2, r1);  // right auto-modifying
  __ cmpeq(r1, Operand(0xff2));           // true
  B_LINE(f, &error);


  __ mov(r1, Operand(0xffffffff));
  __ lxor(r1, r1, Operand(0xff00ff00));
  __ cmpeq(r1, Operand(0x00ff00ff));          // true
  B_LINE(f, &error);

  __ mov(r0, Operand(0xff));
  __ lxor(r0, r0, Operand(0x0e));
  __ cmpeq(r0, Operand(0xf1));            // true
  B_LINE(f, &error);

  __ mov(r1, Operand(0xff));
  __ mov(r2, Operand(0x0f));
  __ lxor(r3, r1, r2);
  __ cmpeq(r3, Operand(0xf0));           // true
  B_LINE(f, &error);

  __ lxor(r1, r1, r2);  // left auto-modifying
  __ cmpeq(r1, Operand(0xf0));           // true
  B_LINE(f, &error);

  __ mov(r1, Operand(0x0ff));
  __ lxor(r1, r2, r1);  // right auto-modifying
  __ cmpeq(r1, Operand(0xf0));           // true
  B_LINE(f, &error);

  __ mov(r1, Operand(0x0ff));
  __ mov(r2, Operand(0x040));
  __ bic(r1, r1, r2);
  __ cmpeq(r1, Operand(0xbf));           // true
  B_LINE(f, &error);
  __ bic(r0, r1, Operand(0x80));
  __ cmpeq(r0, Operand(0x3f));           // true
  B_LINE(f, &error);

  // All ok.
  __ mov(r0, Operand(0));
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

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


// Test conditional moves
TEST(17) {
  BEGIN();

  Label error;

  PROLOGUE();
  __ mov(r0, Operand(0));
  __ mov(r1, Operand(0));
  __ mov(r2, Operand(0));
  __ tst(r0, r0);
  __ mov(r1, Operand(1), eq);
  __ mov(r2, Operand(1), ne);
  __ cmpeq(r1, Operand(1));
  B_LINE(f, &error);
  __ cmpeq(r2, Operand(0));
  B_LINE(f, &error);

  __ mov(r0, Operand(1));
  __ mov(r1, Operand(0));
  __ mov(r2, Operand(0));
  __ tst(r0, r0);
  __ mov(r1, Operand(1), eq);
  __ mov(r2, Operand(1), ne);
  __ cmpeq(r1, Operand(0));
  B_LINE(f, &error);
  __ cmpeq(r2, Operand(1));
  B_LINE(f, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(0));
  __ mov(r2, Operand(0));
  __ tst(r0, r0);
  __ mov(r1, Operand(0xffff), eq);
  __ mov(r2, Operand(0xffff), ne);
  __ cmpeq(r1, Operand(0xffff));
  B_LINE(f, &error);
  __ cmpeq(r2, Operand(0));
  B_LINE(f, &error);

  __ mov(r0, Operand(1));
  __ mov(r1, Operand(0));
  __ mov(r2, Operand(0));
  __ tst(r0, r0);
  __ mov(r1, Operand(0xffff), eq);
  __ mov(r2, Operand(0xffff), ne);
  __ cmpeq(r1, Operand(0));
  B_LINE(f, &error);
  __ cmpeq(r2, Operand(0xffff));
  B_LINE(f, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(0));
  __ mov(r2, Operand(0));
  __ mov(r3, Operand(1));
  __ tst(r0, r0);
  __ mov(r1, r3, eq);
  __ mov(r2, r3, ne);
  __ cmpeq(r1, Operand(1));
  B_LINE(f, &error);
  __ cmpeq(r2, Operand(0));
  B_LINE(f, &error);

  // All ok.
  __ mov(r0, Operand(0));
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

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


// Test addc/subc
TEST(18) {
  BEGIN();

  Label error;

  PROLOGUE();
  __ mov(r0, Operand(0xFFFFFFFE));
  __ mov(r1, Operand(1));
  __ addc(r2, r0, r1);
  B_LINE(t, &error);  // check that carry is clear
  __ cmpeq(r2, Operand(0xFFFFFFFF));
  B_LINE(f, &error);

  __ addc(r1, r1, r0);  // left auto-modified
  B_LINE(t, &error);
  __ cmpeq(r1, Operand(0xFFFFFFFF));
  B_LINE(f, &error);

  __ mov(r1, Operand(1));
  __ addc(r1, r0, r1);  // right auto-modified
  B_LINE(t, &error);
  __ cmpeq(r1, Operand(0xFFFFFFFF));
  B_LINE(f, &error);

  __ mov(r0, Operand(0xFFFFFFFF));
  __ mov(r1, Operand(1));
  __ addc(r2, r0, r1);
  B_LINE(f, &error);  // check that carry is set
  __ cmpeq(r2, Operand(0));
  B_LINE(f, &error);

  __ addc(r1, r1, r0);  // left auto-modified
  B_LINE(f, &error);
  __ cmpeq(r1, Operand(0));
  B_LINE(f, &error);

  __ mov(r1, Operand(1));
  __ addc(r1, r0, r1);  // right auto-modified
  B_LINE(f, &error);
  __ cmpeq(r1, Operand(0));
  B_LINE(f, &error);

  __ mov(r0, Operand(1));
  __ mov(r1, Operand(1));
  __ subc(r2, r0, r1);
  B_LINE(t, &error);  // check that carry is clear
  __ cmpeq(r2, Operand(0));
  B_LINE(f, &error);

  __ subc(r0, r0, r1);  // left auto-modified
  B_LINE(t, &error);
  __ cmpeq(r0, Operand(0));
  B_LINE(f, &error);

  __ mov(r0, Operand(1));
  __ subc(r1, r0, r1);  // right auto-modified
  B_LINE(t, &error);
  __ cmpeq(r1, Operand(0));
  B_LINE(f, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(1));
  __ subc(r2, r0, r1);
  B_LINE(f, &error);  // check that carry is set
  __ cmpeq(r2, Operand(-1));
  B_LINE(f, &error);

  __ subc(r0, r0, r1);  // left auto-modified
  B_LINE(f, &error);
  __ cmpeq(r0, Operand(-1));
  B_LINE(f, &error);

  __ mov(r0, Operand(0));
  __ subc(r1, r0, r1);  // right auto-modified
  B_LINE(f, &error);
  __ cmpeq(r1, Operand(-1));
  B_LINE(f, &error);

  // All ok.
  __ mov(r0, Operand(0));
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

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// Test addv/subv
TEST(19) {
  BEGIN();

  Label error;

  PROLOGUE();
  __ mov(r0, Operand(0x7FFFFFFE));
  __ mov(r1, Operand(1));
  __ addv(r2, r0, r1);
  B_LINE(t, &error);  // check that overflow is clear
  __ cmpeq(r2, Operand(0x7FFFFFFF));
  B_LINE(f, &error);

  __ addv(r1, r1, r0);  // left auto-modified
  B_LINE(t, &error);
  __ cmpeq(r1, Operand(0x7FFFFFFF));
  B_LINE(f, &error);

  __ mov(r1, Operand(1));
  __ addv(r1, r0, r1);  // right auto-modified
  B_LINE(t, &error);
  __ cmpeq(r1, Operand(0x7FFFFFFF));
  B_LINE(f, &error);

  __ mov(r0, Operand(0x7FFFFFFF));
  __ mov(r1, Operand(1));
  __ addv(r2, r0, r1);
  B_LINE(f, &error);  // check that overflow is set
  __ cmpeq(r2, Operand(0x80000000));
  B_LINE(f, &error);

  __ addv(r1, r1, r0);  // left auto-modified
  B_LINE(f, &error);
  __ cmpeq(r1, Operand(0x80000000));
  B_LINE(f, &error);

  __ mov(r1, Operand(1));
  __ addv(r1, r0, r1);  // right auto-modified
  B_LINE(f, &error);
  __ cmpeq(r1, Operand(0x80000000));
  B_LINE(f, &error);

  __ mov(r0, Operand(0x80000001));
  __ mov(r1, Operand(1));
  __ subv(r2, r0, r1);
  B_LINE(t, &error);  // check that overflow is clear
  __ cmpeq(r2, Operand(0x80000000));
  B_LINE(f, &error);

  __ subv(r0, r0, r1);  // left auto-modified
  B_LINE(t, &error);
  __ cmpeq(r0, Operand(0x80000000));
  B_LINE(f, &error);

  __ mov(r0, Operand(0x80000001));
  __ subv(r1, r0, r1);  // right auto-modified
  B_LINE(t, &error);
  __ cmpeq(r1, Operand(0x80000000));
  B_LINE(f, &error);

  __ mov(r0, Operand(0x80000000));
  __ mov(r1, Operand(1));
  __ subv(r2, r0, r1);
  B_LINE(f, &error);  // check that carry is set
  __ cmpeq(r2, Operand(0x7FFFFFFF));
  B_LINE(f, &error);

  __ subv(r0, r0, r1);  // left auto-modified
  B_LINE(f, &error);
  __ cmpeq(r0, Operand(0x7FFFFFFF));
  B_LINE(f, &error);

  __ mov(r0, Operand(0x80000000));
  __ subv(r1, r0, r1);  // right auto-modified
  B_LINE(f, &error);
  __ cmpeq(r1, Operand(0x7FFFFFFF));
  B_LINE(f, &error);

  // All ok.
  __ mov(r0, Operand(0));
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

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

TEST(20) {
  BEGIN();

  Label error;
  Condition cond;

  PROLOGUE();
  cond = eq;
  __ cmp(&cond, r4, Operand(0), r0);
  CHECK_EQ(eq, cond);
  B_LINE(cond, &error);

  cond = ge;
  __ cmp(&cond, r4, Operand(0), r0);
  CHECK_EQ(eq, cond);
  B_LINE(f, &error);

  cond = lt;
  __ cmp(&cond, r4, Operand(654), r0);
  CHECK_EQ(ne, cond);
  B_LINE(t, &error);

  __ mov(r0, Operand(0));
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

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 456, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// test storing uint8 value in the cmp immediate
TEST(21) {
  BEGIN();

  Label top;
  __ bind(&top);
  __ cmpeq(r0, r1);
  __ cmpgt(r0, r1);
  // > 127, ie. would normally exceed signed range
  __ cmpeq_r0_unsigned_imm(173);
  __ bt(&top);
  __ bf(&top);
  __ mov(r0, Operand(12));

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  CHECK_EQ(true, __ IsCmpRegister((reinterpret_cast<Instr*>(desc.buffer)[0])));
  CHECK_EQ(false, __ IsCmpRegister((reinterpret_cast<Instr*>(desc.buffer)[1])));

  CHECK_EQ(false, __ IsCmpImmediate(reinterpret_cast<Instr*>(desc.buffer)[1]));
  CHECK_EQ(true, __ IsCmpImmediate(reinterpret_cast<Instr*>(desc.buffer)[2]));
  CHECK_EQ(true, (__ GetCmpImmediateRegister(
                        reinterpret_cast<Instr*>(desc.buffer)[2])).is(r0));
  CHECK_EQ(173, __ GetCmpImmediateAsUnsigned(
                        reinterpret_cast<Instr*>(desc.buffer)[2]));

  CHECK_EQ(true, (__ GetRn((reinterpret_cast<Instr*>(desc.buffer)[0])).is(r0)));
  CHECK_EQ(true, (__ GetRm((reinterpret_cast<Instr*>(desc.buffer)[0])).is(r1)));

  // __ bt() generate bt/nop
  CHECK_EQ(eq, __ GetCondition((reinterpret_cast<Instr*>(desc.buffer)[3])));
  CHECK_EQ(ne, __ GetCondition((reinterpret_cast<Instr*>(desc.buffer)[5])));

  CHECK_EQ(true, __ IsMovImmediate(reinterpret_cast<Instr*>(desc.buffer)[7]));
}

TEST(22) {
  BEGIN();

  Label function, end_function;

  PROLOGUE();
  __ mov(r5, Operand(1));
  __ mov(r0, r4);
  __ add(r0, Operand(1), r1);
  __ push(pr);
  __ jsr(&function);
  __ pop(pr);
  EPILOGUE();
  __ rts();

  __ bind(&function);
  __ add(r0, Operand(1), r1);
  __ add(r0, Operand(2), r1);
  __ add(r0, Operand(3), r1);
  __ add(r0, Operand(4), r1);
  __ cmpeq(r5, Operand(0), r1);
  __ bt(&end_function);
  __ mov(r5, Operand(0));
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

  PROLOGUE();
  __ mov(r0, Operand(0));
  __ bind(&function);
  __ tst(r0, r0);
  __ bt(&label);
  __ rts();
  __ bind(&label);
  for (int i = 0; i < 10000; i++)
    __ add(r0, r0, Operand(1), r1);

  __ push(pr);
  __ jsr(&function);
  __ pop(pr);
  EPILOGUE();
  __ rts();

  JIT();

#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(10000, res);
}

TEST(23) {
  BEGIN();
  Label error;

  PROLOGUE();
  __ mov(r0, Operand(0));
  __ cmpeq(r0, Operand(0), r4);
  __ mov(r1, Operand(0x00ff00ff));
  __ mov(r2, Operand(0xff00ff00));
  __ lor(r1, r1, r2, eq);
  __ cmpeq(r1, Operand(0xffffffff));
  __ bf(&error);

  __ cmpeq(r0, Operand(0), r4);
  __ mov(r1, Operand(0x0000ff00));
  __ mov(r2, Operand(0x000000ff));
  __ lor(r1, r1, r2, ne);
  __ cmpeq(r1, Operand(0x0000ff00), r4);
  __ bf(&error);

  __ mov(r0, Operand(0));
  __ cmpeq(r0, Operand(0), r4);
  __ mov(r1, Operand(0xffff00ff));
  __ lor(r1, r1, Operand(0xffffeeff), eq, r2);
  __ cmpeq(r1, Operand(0xffffeeff), r2);
  __ bf(&error);

  __ cmpeq(r0, Operand(0), r4);
  __ mov(r1, Operand(0x0000ff00));
  __ lor(r1, r1, Operand(0x000000ff), ne, r2);
  __ cmpeq(r1, Operand(0x0000ff00), r2);
  __ bf(&error);


  __ mov(r0, Operand(0));
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ mov(r0, Operand(1));
  EPILOGUE();
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

  PROLOGUE();
  __ dmuls(r1, r0, r4, r5);
  __ cmpeq(r0, r6);
  __ bf(&error);
  __ cmpeq(r1, r7);
  __ bf(&error);

  __ mov(r0, Operand(0));
  EPILOGUE();
  __ rts();

  __ bind(&error);
  __ mov(r0, Operand(1));
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 2, 0, 0, 0, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 3, 4, 0, 12, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 10, 45, 0, 10*45,
                                                        0)));
  CHECK_EQ(0, reinterpret_cast<int>(
                CALL_GENERATED_CODE(f, 1000000000, 10000, 2328,
                                    1316134912, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(
                CALL_GENERATED_CODE(f, 123465879, 123465780, 3549226,
                                    1458007724, 0)));
  CHECK_EQ(0, reinterpret_cast<int>(
                CALL_GENERATED_CODE(f, 13246579, 0, 0, 0, 0)));
}

TEST(25) {
  BEGIN();

  Label function;

  PROLOGUE();
  __ mov(r0, Operand(0));
  __ push(pr);

  __ call(&function);
  __ add(r0, r0, Operand(1), r2);
  __ add(r0, r0, Operand(1), r2);
  __ add(r0, r0, Operand(1), r2);
  __ add(r0, r0, Operand(1), r2);

  __ pop(pr);
  EPILOGUE();
  __ rts();

  __ bind(&function);
  // return to the second add (skiping the one just after the call)
  __ strpr(r1);
  __ add(r1, r1, Operand(10), r2);
  __ ldrpr(r1);
  __ add(r0, r0, Operand(1), r2);
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  CHECK_EQ(4, reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0)));
}

TEST(26) {
  CHECK_EQ(0, strcmp(Registers::Name(1), "r1"));
  CHECK_EQ(0, strcmp(Registers::Name(10), "r10"));
  CHECK_EQ(0, strcmp(Registers::Name(15), "r15"));
  CHECK_EQ(0, strcmp(Registers::Name(16), "noreg"));

  CHECK_EQ(0, Registers::Number("r0"));
  CHECK_EQ(5, Registers::Number("r5"));
  CHECK_EQ(14, Registers::Number("r14"));
  CHECK_EQ(15, Registers::Number("r15"));
  CHECK_EQ(14, Registers::Number("fp"));
  CHECK_EQ(15, Registers::Number("sp"));
  CHECK_EQ(kNoRegister, Registers::Number("r16"));
}


// Test load/store operations
TEST(27) {
  BEGIN();

  Label error;

  PROLOGUE();
  __ mov(r3, sp);
  __ sub(sp, sp, Operand(16));

  // Test str/ldr
  __ mov(r1, Operand(3));
  __ str(r1, MemOperand(sp, 0));
  __ mov(r1, Operand(7));
  __ mov(r2, Operand(4));
  __ str(r1, MemOperand(sp, r2));
  __ mov(r1, Operand(9));
  __ str(r1, MemOperand(sp, 8));
  __ mov(r0, Operand(0xdeadbeef));
  __ ldr(r0, MemOperand(sp, 0));
  __ cmpeq(r0, Operand(3));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldr(r0, MemOperand(sp, r2));
  __ cmpeq(r0, Operand(7));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldr(r0, MemOperand(sp, 8));
  __ cmpeq(r0, Operand(9));
  B_LINE(f, &error);

  // Test strh/ldrh/ldrsh
  __ mov(r1, Operand(3));
  __ strh(r1, MemOperand(sp, 0));
  __ mov(r1, Operand(0xff07));
  __ strh(r1, MemOperand(sp, 2));
  __ mov(r2, Operand(4));
  __ mov(r1, Operand(0x09));
  __ strh(r1, MemOperand(sp, r2));
  __ mov(r0, Operand(0xdeadbeef));
  __ ldr(r0, MemOperand(sp, 0));
  __ cmpeq(r0, Operand(0xff070003));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldrh(r0, MemOperand(sp, 0));
  __ cmpeq(r0, Operand(0x03));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldrh(r0, MemOperand(sp, 2));
  __ cmpeq(r0, Operand(0xff07));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldrh(r0, MemOperand(sp, r2));
  __ cmpeq(r0, Operand(0x09));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldrsh(r0, MemOperand(sp, 2));
  __ cmpeq(r0, Operand(0xffffff07));
  B_LINE(f, &error);

  // Test strb/ldrb
  __ mov(r1, Operand(3));
  __ strb(r1, MemOperand(sp, 0));
  __ mov(r1, Operand(0xf7));
  __ mov(r2, Operand(1));
  __ strb(r1, MemOperand(sp, r2));
  __ mov(r1, Operand(5));
  __ strb(r1, MemOperand(sp, 2));
  __ mov(r1, Operand(1));
  __ strb(r1, MemOperand(sp, 3));
  __ mov(r0, Operand(0xdeadbeef));
  __ ldr(r0, MemOperand(sp, 0));
  __ cmpeq(r0, Operand(0x0105f703));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldrb(r0, MemOperand(sp, 0));
  __ cmpeq(r0, Operand(0x03));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldrb(r0, MemOperand(sp, r2));
  __ cmpeq(r0, Operand(0xf7));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldrb(r0, MemOperand(sp, 2));
  __ cmpeq(r0, Operand(0x05));
  B_LINE(f, &error);
  __ mov(r0, Operand(0xdeadbeef));
  __ ldrsb(r0, MemOperand(sp, 1));
  __ cmpeq(r0, Operand(0xfffffff7));
  B_LINE(f, &error);


  // Test ldr/str with Pre/post Index
  __ mov(r1, Operand(3));
  __ str(r1, MemOperand(sp, 0));
  __ mov(r1, Operand(7));
  __ str(r1, MemOperand(sp, 4));
  __ mov(r1, Operand(9));
  __ str(r1, MemOperand(sp, 8));

  __ mov(r2, sp);
  __ ldr(r0, MemOperand(r2, 0, PostIndex));
  __ cmpeq(r0, Operand(3));
  B_LINE(f, &error);
  __ cmpeq(r2, sp);
  B_LINE(f, &error);

  __ ldr(r0, MemOperand(r2, 4, PostIndex));
  __ cmpeq(r0, Operand(3));
  B_LINE(f, &error);
  __ add(r1, sp, Operand(4));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);

  __ ldr(r0, MemOperand(r2, 4, PostIndex));
  __ cmpeq(r0, Operand(7));
  B_LINE(f, &error);
  __ add(r1, sp, Operand(8));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);

  __ ldr(r0, MemOperand(r2, 4, PostIndex));
  __ cmpeq(r0, Operand(9));
  B_LINE(f, &error);
  __ add(r1, sp, Operand(12));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);

  __ sub(r2, sp, Operand(4));
  __ ldr(r0, MemOperand(r2, 4, PreIndex));
  __ cmpeq(r0, Operand(3));
  B_LINE(f, &error);
  __ cmpeq(sp, r2);
  B_LINE(f, &error);

  __ ldr(r0, MemOperand(r2, 4, PreIndex));
  __ cmpeq(r0, Operand(7));
  B_LINE(f, &error);
  __ add(r1, sp, Operand(4));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);

  __ ldr(r0, MemOperand(r2, 4, PreIndex));
  __ cmpeq(r0, Operand(9));
  B_LINE(f, &error);
  __ add(r1, sp, Operand(8));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);


  __ mov(r2, sp);
  __ mov(r1, Operand(4212));
  __ str(r1, MemOperand(r2, 4, PostIndex));
  __ mov(r1, Operand(1234));
  __ str(r1, MemOperand(r2, 4, PostIndex));
  __ add(r1, sp, Operand(8));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);

  __ mov(r2, sp);
  __ ldr(r1, MemOperand(r2, 4, PostIndex));
  __ cmpeq(r1, Operand(4212));
  B_LINE(f, &error);
  __ ldr(r1, MemOperand(r2, 4, PostIndex));
  __ cmpeq(r1, Operand(1234));
  B_LINE(f, &error);
  __ add(r1, sp, Operand(8));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);

  __ mov(r2, sp);
  __ mov(r1, Operand(98765));
  __ str(r1, MemOperand(r2, 4, PreIndex));
  __ mov(r1, Operand(0));
  __ str(r1, MemOperand(r2, 4, PreIndex));
  __ add(r1, sp, Operand(8));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);

  __ mov(r2, sp);
  __ ldr(r1, MemOperand(r2));
  __ cmpeq(r1, Operand(4212));
  B_LINE(f, &error);
  __ ldr(r1, MemOperand(r2, 4, PreIndex));
  __ cmpeq(r1, Operand(98765));
  B_LINE(f, &error);
  __ ldr(r1, MemOperand(r2, 4, PreIndex));
  __ cmpeq(r1, Operand(0));
  B_LINE(f, &error);
  __ add(r1, sp, Operand(8));
  __ cmpeq(r1, r2);
  B_LINE(f, &error);


  // All ok.
  __ mov(sp, r3);
  __ mov(r0, Operand(0));
  EPILOGUE();
  __ rts();
  __ bind(&error);
  __ mov(sp, r3);
  __ mov(r0, r10);
  EPILOGUE();
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

// Test near labels
TEST(28) {
  BEGIN();

  Label l1, l2, l3, l4;

  PROLOGUE();

  __ mov(r0, Operand(0));
  __ cmpeq(r4, r5);
  __ bf_near(&l1);
  __ bf_near(&l1);
  __ bf_near(&l1);

  __ add(r0, Operand(1));
  __ bind(&l1);
  __ cmpeq(r4, r5);
  __ bt_near(&l2);
  __ bt_near(&l2);
  __ bt_near(&l2);
  __ bt_near(&l2);

  __ sub(r0, r0, Operand(1));
  __ sub(r0, r0, Operand(1));
  __ sub(r0, r0, Operand(1));
  __ sub(r0, r0, Operand(1));
  __ bind(&l2);
  __ add(r0, Operand(1));
  __ add(r0, Operand(1));
  __ add(r0, Operand(1));
  __ add(r0, Operand(1));

  __ bind(&l3);
  __ add(r4, Operand(1));
  __ cmpeq(r4, Operand(2));
  __ bf_near(&l3);

  __ jmp_near(&l4);
  __ jmp_near(&l4);
  __ add(r0, Operand(1));
  __ bind(&l4);

  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());

  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 1, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

TEST(29) {
  BEGIN();

  Label l1;

  PROLOGUE();

  __ mov(r0, Operand(0));

  __ cmpeq(r4, r5);
  __ bf_near(&l1);
  for (int i = 0; i < 50; i++)
    __ nop();
  __ add(r0, Operand(1));
  __ bind(&l1);
  __ add(r0, Operand(2));

  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
//  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 12, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(2, res);
}


TEST(30) {
  typedef struct {
    double a;
    double b;
  } T;
  T t;
  t.a = 12345.012456021864;
  t.b = 0.1348714347684218;

  BEGIN();
  PROLOGUE();
  __ dldr(sh4_dr0, MemOperand(r4, OFFSET_OF(T, a)), r6);
  __ dldr(sh4_dr2, MemOperand(r4, OFFSET_OF(T, b)), r6);

  __ dstr(sh4_dr0, MemOperand(r4, OFFSET_OF(T, b)), r6);
  __ dstr(sh4_dr2, MemOperand(r4, OFFSET_OF(T, a)), r6);

  __ mov(r0, r5);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F3 f = FUNCTION_CAST<F3>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, &t, 123156, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(123156, res);
  CHECK_EQ(t.a, 0.1348714347684218);
  CHECK_EQ(t.b, 12345.012456021864);
}


// Test Assembler::dfloat()
TEST(31) {
  BEGIN();

  PROLOGUE();
  Label error;

  // Check (double)123
  __ dfloat(sh4_dr0, Operand(123));
  __ dcmpeq(sh4_dr0, sh4_dr4);
  B_LINE(f, &error);

  // Check (double)(int)0x80000000
  __ dfloat(sh4_dr0, Operand(0x80000000));
  __ dcmpeq(sh4_dr0, sh4_dr6);
  B_LINE(f, &error);

  __ mov(r0, Operand(0));
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

  F5 f = FUNCTION_CAST<F5>(Code::cast(code)->entry());
#if defined(USE_SIMULATOR)
  int res = reinterpret_cast<int>(CALL_GENERATED_FPU_CODE(f, 123.0, (double)(int)0x80000000));
#else
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 123.0, (double)(int)0x80000000, 0, 0, 0));
#endif
  CHECK_EQ(0, res);
}

// Test Assembler::dufloat()
// SH4: dufloat implementation is complicated as unsigned conversion is
// not available on HW. Thus, test some case where the unsigned value would
// overflow a signed convertion (0x80000000U for instance).
TEST(31b) {
  BEGIN();

  PROLOGUE();
  Label error;

  // Check (double)123
  __ mov(r0, Operand(123));
  __ dufloat(sh4_dr0, r0);
  __ dcmpeq(sh4_dr0, sh4_dr4);
  B_LINE(f, &error);

  // Check (double)0x80000000U
  __ mov(r0, Operand(0x80000000));
  __ dufloat(sh4_dr0, r0);
  __ dcmpeq(sh4_dr0, sh4_dr6);
  B_LINE(f, &error);

  __ mov(r0, Operand(0));
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

  F5 f = FUNCTION_CAST<F5>(Code::cast(code)->entry());
#if defined(USE_SIMULATOR)
  int res = reinterpret_cast<int>(CALL_GENERATED_FPU_CODE(f, 123.0, (double)0x80000000U));
#else
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 123.0, (double)0x80000000U, 0, 0, 0));
#endif
  CHECK_EQ(0, res);
}


TEST(32) {
  BEGIN();

  Label error;
  PROLOGUE();

  __ dfloat(sh4_dr0, Operand(34));
  __ fadd(sh4_dr0, sh4_dr4);
  __ dfloat(sh4_dr2, Operand(456 + 34));
  __ dcmpeq(sh4_dr2, sh4_dr0);
  B_LINE(f, &error);

  __ dfloat(sh4_dr2, Operand(56));
  __ fsub(sh4_dr0, sh4_dr2);
  __ dfloat(sh4_dr2, Operand(456 + 34 - 56));
  __ dcmpeq(sh4_dr2, sh4_dr0);
  B_LINE(f, &error);

  __ dfloat(sh4_dr2, Operand(7));
  __ fmul(sh4_dr0, sh4_dr2);
  __ dfloat(sh4_dr2, Operand((456 + 34 - 56) * 7));
  __ dcmpeq(sh4_dr2, sh4_dr0);
  B_LINE(f, &error);

  __ dfloat(sh4_dr2, Operand(2));
  __ fdiv(sh4_dr0, sh4_dr2);
  __ dfloat(sh4_dr2, Operand(((456 + 34 - 56) * 7) / 2));
  __ dcmpeq(sh4_dr2, sh4_dr0);
  B_LINE(f, &error);

  // All ok
  __ mov(r0, Operand(0));
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

  F5 f = FUNCTION_CAST<F5>(Code::cast(code)->entry());
#if defined(USE_SIMULATOR)
  int res = reinterpret_cast<int>(CALL_GENERATED_FPU_CODE(f, 456, 0));
#else
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 456, 0, 0, 0, 0));
#endif
  CHECK_EQ(0, res);
}


TEST(33) {
  BEGIN();

  Label error;
  PROLOGUE();

  // Check (int)4212.0
  __ idouble(r1, sh4_dr4, r2/*FPSCR*/);
  __ tst(r2, Operand(1<<16/*CauseV bit*/)); // Check is valid
  B_LINE(f, &error);
  __ tst(r2, Operand(1<<12/*CauseI bit*/)); // Check is exact
  B_LINE(f, &error);
  __ cmpeq(r1, Operand(4212));
  B_LINE(f, &error);

  // Check (int)(double)0x80000000U (overflows)
  __ idouble(r1, sh4_dr6, r2/*FPSCR*/);
  __ tst(r2, Operand(1<<16/*CauseV bit*/)); // Check is invalid
  B_LINE(t, &error);

  __ cmpeq(r1, Operand(0x7fffffff)); // Check is clamped to MAX_INT
  B_LINE(f, &error);

  // Check int->double->int provde same result
  __ dfloat(sh4_dr2, Operand(343575789));
  __ idouble(r3, sh4_dr2, r2/*FPSCR*/);
  __ tst(r2, Operand(1<<16/*CauseV bit*/)); // Check is valid
  B_LINE(f, &error);
  __ tst(r2, Operand(1<<12/*CauseI bit*/)); // Check is exact
  B_LINE(f, &error);
  __ cmpeq(r3, Operand(343575789));
  B_LINE(f, &error);

  // Check that (int)sqrt(2) is inexact and truncates to 1
  __ dfloat(sh4_dr2, Operand(2));
  __ fsqrt(sh4_dr2);
  __ idouble(r3, sh4_dr2, r2/*FPSCR*/);
  __ tst(r2, Operand(1<<12/*CauseI bit*/)); // Check is inexact
  B_LINE(t, &error);
  __ cmpeq(r3, Operand(1)); // Check truncated to 1
  B_LINE(f, &error);

  // All ok
  __ mov(r0, Operand(0));
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

  F5 f = FUNCTION_CAST<F5>(Code::cast(code)->entry());
#if defined(USE_SIMULATOR)
  int res = reinterpret_cast<int>(CALL_GENERATED_FPU_CODE(f, 4212.0, (double)0x80000000U));
#else
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 4212.0, (double)0x80000000U, 0, 0, 0));
#endif
  CHECK_EQ(0, res);
}


// These test case are taken from the arm ones
TEST(from_arm_2) {
  BEGIN();
  Label L, C;

  PROLOGUE();
  __ mov(r1, r4);
  __ mov(r0, Operand(1));
  __ b(&C);

  __ bind(&L);
  __ mul(r0, r1, r0);
  __ sub(r1, r1, Operand(1));

  __ bind(&C);
  __ teq(r1, Operand::Zero());
  __ b(ne, &L);
  EPILOGUE();
  __ rts();

  // some relocated stuff here, not executed
  __ RecordComment("dead code, just testing relocations");
  __ mov(r0, Operand(isolate->factory()->true_value()));
  __ RecordComment("dead code, just testing immediate operands");
  __ mov(r0, Operand(-1));
  __ mov(r0, Operand(0xFF000000));
  __ mov(r0, Operand(0xF0F0F0F0));
  __ mov(r0, Operand(0xFFF0FFFF));

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif
  F1 f = FUNCTION_CAST<F1>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 10, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(3628800, res);
}


TEST(from_arm_3) {
  BEGIN();

  typedef struct {
    int i;
    char c;
    int16_t s;
  } T;
  T t;

  Label L, C;
  PROLOGUE();
  __ mov(r0, r4);
  __ push(pr);
  __ push(fp);
  __ sub(fp, sp, Operand(4));

  __ ldr(r0, MemOperand(r4, OFFSET_OF(T, i)), r5);
  __ asr(r2, r0, Operand(1), r5);
  __ str(r2, MemOperand(r4, OFFSET_OF(T, i)), r5);

  __ ldrsb(r2, MemOperand(r4, OFFSET_OF(T, c)));
  __ add(r0, r2, r0);
  __ lsl(r2, r2, Operand(2));
  __ strb(r2, MemOperand(r4, OFFSET_OF(T, c)), r5);

  __ ldrsh(r2, MemOperand(r4, OFFSET_OF(T, s)), r5);
  __ add(r0, r2, r0);
  __ asr(r2, r2, Operand(3));
  __ strh(r2, MemOperand(r4, OFFSET_OF(T, s)), r5);

  __ pop(fp);
  __ pop(pr);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif
  F3 f = FUNCTION_CAST<F3>(Code::cast(code)->entry());
  t.i = 100000;
  t.c = 10;
  t.s = 1000;
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, &t, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(101010, res);
  CHECK_EQ(100000/2, t.i);
  CHECK_EQ(10*4, t.c);
  CHECK_EQ(1000/8, t.s);
}


TEST(from_arm_4) {
  // Test the FPU floating point instructions.
  BEGIN();

  typedef struct {
    double a;
    double b;
    double c;
    double d;
    double e;
    double f;
    double g;
    double h;
    int i;
    float x;
    float y;
  } T;
  T t;

  Label L, C;
  PROLOGUE();
  __ mov(r0, sp);
  __ push(pr);
  __ push(fp);
  __ push(r4);
  __ sub(fp, r0, Operand(4));

  __ dldr(sh4_dr0, MemOperand(r4, OFFSET_OF(T, a)));
  __ dldr(sh4_dr2, MemOperand(r4, OFFSET_OF(T, b)));
  __ fadd(sh4_dr0, sh4_dr2);
  __ dstr(sh4_dr0, MemOperand(r4, OFFSET_OF(T, c)));

  __ movd(r2, r3, sh4_dr0);
  __ movd(sh4_dr0, r2, r3);
  __ dstr(sh4_dr0, MemOperand(r4, OFFSET_OF(T, b)));

  // Load t.x and t.y, switch values, and store back to the struct.
  __ fldr(fr0, MemOperand(r4, OFFSET_OF(T, x)));
  __ fldr(fr1, MemOperand(r4, OFFSET_OF(T, y)));
  __ fstr(fr1, MemOperand(r4, OFFSET_OF(T, x)));
  __ fstr(fr0, MemOperand(r4, OFFSET_OF(T, y)));

  // Load a double and store it as an integer
  __ dldr(sh4_dr0, MemOperand(r4, OFFSET_OF(T, d)));
  __ idouble(r0, sh4_dr0);
  __ str(r0, MemOperand(r4, OFFSET_OF(T, i)));

  // Divisions and multiplications
  __ dldr(sh4_dr0, MemOperand(r4, OFFSET_OF(T, e)));
  __ dldr(sh4_dr2, MemOperand(r4, OFFSET_OF(T, f)));
  __ fmul(sh4_dr2, sh4_dr0);
  __ dstr(sh4_dr2, MemOperand(r4, OFFSET_OF(T, e)));

  __ dldr(sh4_dr2, MemOperand(r4, OFFSET_OF(T, f)));
  __ fdiv(sh4_dr2, sh4_dr0);
  __ dstr(sh4_dr2, MemOperand(r4, OFFSET_OF(T, f)));

  __ dldr(sh4_dr4, MemOperand(r4, OFFSET_OF(T, g)));
  __ dldr(sh4_dr6, MemOperand(r4, OFFSET_OF(T, h)));
  __ fsub(sh4_dr4, sh4_dr6);
  __ dstr(sh4_dr4, MemOperand(r4, OFFSET_OF(T, g)));

  __ pop(r4);
  __ pop(fp);
  __ pop(pr);
  EPILOGUE();
  __ rts();

    JIT();
#ifdef DEBUG
    Code::cast(code)->Print();
#endif
    F3 f = FUNCTION_CAST<F3>(Code::cast(code)->entry());
    t.a = 1.5;
    t.b = 2.75;
    t.c = 17.17;
    t.d = 17.17;
    t.e = 1234.56;
    t.f = 12.1;
    t.g = 2718.2818;
    t.h = 31415926.5;
    t.i = 0;
    t.x = 4.5;
    t.y = 9.0;
    Object* dummy = CALL_GENERATED_CODE(f, &t, 0, 0, 0, 0);
    USE(dummy);
    CHECK_EQ(1.5, t.a);
    CHECK_EQ(1.5+2.75, t.b);
    CHECK_EQ(1.5+2.75, t.c);
    CHECK_EQ(4.5, t.y);
    CHECK_EQ(9.0, t.x);
    CHECK_EQ(17, t.i);
    CHECK_EQ(1234.56*12.1, t.e);
    CHECK_EQ(12.1/1234.56, t.f);
    CHECK_EQ(2718.2818-31415926.5, t.g);
    CHECK_EQ(31415926.5, t.h);
}

TEST(from_arm_12) {
  // Test chaining of label usages within instructions (issue 1644).
  BEGIN();

  Label target;
  __ b(eq, &target);
  __ b(ne, &target);
  __ bind(&target);
  __ nop();
}

TEST(memcpy) {
  BEGIN();
  PROLOGUE();
  __ memcpy(r4, r5, r6, r1, r2, r3, r7);
  __ mov(r0, Operand(0));
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  const char *psz_buffer = "this string will be copied to the second buffer";
  char psz_dest[47 + 1] = { 0 };

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, (int)psz_dest,
                                    (int)psz_buffer, 47 + 1, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
  CHECK_EQ(0, strcmp(psz_buffer, psz_dest));
}

TEST(cp_0) {
  BEGIN();

  PROLOGUE();
  __ mov(r0, Operand(0));
  __ mov(r1, Operand(123456789));
  for (int i = 0; i < 512; i++)
    __ add(r0, Operand(1));
  __ add(r0, r1, r0);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, -10, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(123456789 + 512, res);
}

TEST(cp_1) {
  BEGIN();
  i::FLAG_code_comments = true;

  PROLOGUE();
  __ mov(r0, Operand(0));
  __ RecordComment("constant pool mov");
  __ mov(r0, Operand(0xf00d));
  __ RecordComment("3rd mov");
  __ RecordComment("2nd comment");
  __ mov(r1, Operand(0x0d060000));
  __ add(r0, r0, r1);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 1, -10, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0x0d06f00d, res);
}

TEST(cp_2) {
  // Tests conditional far backward branch (and far forward jump)
  BEGIN();

  Label begin, end;

  PROLOGUE();

  __ mov(r0, Operand(0));
  __ jmp(&end);

  __ bkpt();
  __ bkpt();
  __ bkpt();
  __ bkpt();

  __ bind(&begin);
  for (int i = 0; i < 10000; i++)
    __ add(r0, r0, Operand(1), r1);

  __ bind(&end);

  __ cmpeq(r0, Operand(0));
  __ b(t, &begin);

  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(10000, res);
}


TEST(cp_3) {
  // Test for utterly big constant pool and see if everything is still ok
  const int loops = 10000;

  BEGIN();

  PROLOGUE();

  for (int i = 0; i < loops; i++) {
    __ mov(r0, Operand(i));
  }

  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(loops - 1, res);
}


TEST(cp_4) {
  // Test for constant pool emission before the backward branch.
  // This is forbidden due to the layout of the current code
  Label begin, end;
  int loops = 252;

  BEGIN();

  PROLOGUE();
  __ mov(r0, Operand(0));

  __ bind(&begin);
  __ cmpeq(r0, Operand(0));
  __ bf(&end);

  for(int i = 1024; i < 1024 + loops; i++) {
    __ mov(r0, Operand(i));
  }
  __ cmpeq(r0, Operand(0));
  __ bt(&begin);

  __ bind(&end);
  EPILOGUE();
  __ rts();

 JIT();

#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(1024 + loops - 1, res);
}


TEST(cp_5) {
  int loops = 200;

  BEGIN();

  PROLOGUE();
  __ mov(r0, Operand(0));

  for(int i = 1024; i < 1024 + loops; i++)
    __ mov(r0, Operand(i));

  for(int i = 0; i < 2000; i++)
    __ nop();

  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F2 f = FUNCTION_CAST<F2>(Code::cast(code)->entry());
  int res = reinterpret_cast<int>(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
}
