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
#include "macro-assembler.h"
#include "sh4/macro-assembler-sh4.h"
#include "sh4/simulator-sh4.h"
#include "cctest.h"


using namespace v8::internal;


typedef Object* (*F0)(int p0, int p1, int p2, int p3, int p4);


// We allow use of callee saved r8-r11 for the regression tests
#define PROLOGUE() \
  do { \
  __ push(r8); \
  __ push(r9); \
  __ push(r10); \
  __ push(r11); \
  __ push(roots); \
  ExternalReference roots_address = \
    ExternalReference::roots_array_start(assm.isolate()); \
  __ mov(roots, Operand(roots_address)); \
  } while (0)

#define EPILOGUE() \
  do { \
  __ pop(roots); \
  __ pop(r11); \
  __ pop(r10);        \
  __ pop(r9); \
  __ pop(r8); \
  } while (0)

#define THE_HOLE_VALUE() (assm.isolate()->factory()->the_hole_value())

#define NAN_VALUE() (assm.isolate()->factory()->nan_value())

#define FALSE_VALUE() (assm.isolate()->factory()->false_value())

#define EMPTY_STRING() (assm.isolate()->factory()->empty_string())

#define GLOBAL_CONTEXT() (assm.isolate()->global_context())

#define GLOBAL_CONTEXT_MAP() (assm.isolate()->factory()->global_context_map())

#define HEAP_NUMBER_MAP() (assm.isolate()->factory()->heap_number_map())

#define STRING_MAP() (assm.isolate()->factory()->string_map())

#define BEGIN()                                         \
  /* Disable compilation of natives. */                 \
  i::FLAG_disable_native_files = true;                  \
  /* Disable slow asserts that require natives. */      \
  i::FLAG_enable_slow_asserts = false;                  \
                                                        \
  CcTest::InitializeVM();                               \
  Isolate* isolate = CcTest::i_isolate();               \
  HandleScope scope(isolate);                         \
  MacroAssembler assm(isolate, NULL, 0);

#define JIT()                                                           \
  CodeDesc desc;                                                        \
  assm.GetCode(&desc);                                                  \
  Object* code = isolate->heap()->CreateCode(                           \
      desc,                                                             \
      Code::ComputeFlags(Code::STUB),                                   \
      Handle<Code>())->ToObjectChecked();                               \
  CHECK(code->IsCode());

#define CMT(msg) do { Comment cmnt(&assm, msg); } while (0)

#define __ assm.

// This macro stores in r10 the line number before branching to the error label.
// At the error label r10 can be moved to r0 such that return code of the
// function if not 0 indicates an error at the line of the branch.
#define B_LINE(cond, target) do { \
    __ mov(r10, Operand(__LINE__)); \
    if (cond == al) { \
      __ b(target);  \
    } else { \
      __ b(cond, target);  \
    } \
  } while (0);

# define CAST(a) reinterpret_cast<int>((a))

// Test Move(...)
TEST(sh4_ma_0) {
  BEGIN();

  Label error;

  PROLOGUE();

  // Verify Move with Immediate
  __ Move(r0, Handle<Object>(reinterpret_cast<Object *>(0x12345678), isolate));
  __ mov(r1, Operand(0x12345678));
  __ cmp(r0, r1);
  B_LINE(ne, &error);


  // Verify register move
  __ mov(r0, Operand(~0x00110011));  // encodes 0xffeeffee
  __ Move(r1, r0);
  __ cmp(r0, r1);
  B_LINE(ne, &error);


  // Verify register move to same register does not emit anything
  __ mov(r0, Operand(~0x11001100));  // encodes 0xeeffeeff
  int offset = assm.pc_offset();
  __ Move(r0, r0);
  offset = assm.pc_offset() - offset;
  __ mov(r1, Operand(offset));
  __ cmp(r1, Operand(0));
  B_LINE(ne, &error);


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

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


// Test Bfc/Ubfx/Sbfx/Bfi/Usat
TEST(sh4_ma_1) {
  BEGIN();

  Label error;

  PROLOGUE();

  CMT("Verify Bfc(0xfdeccdefu, 0, 32) == 0");
  __ mov(r0, Operand(0xfdeccdefu));
  __ mov(r1, r0);
  __ Bfc(r1, r1, 0, 32);  // a full clear
  __ cmp(r1, Operand(0));
  B_LINE(ne, &error);
  CMT("Verify Bfc(0xfdeccdefu, 0, 0) == 0xfdeccdefu");
  __ mov(r0, Operand(0xfdeccdefu));
  __ mov(r1, r0);
  __ Bfc(r1, r1, 0, 0);  // a mov
  __ cmp(r1, Operand(0xfdeccdefu));
  B_LINE(ne, &error);
  CMT("Verify Bfc(0xfdeccdefu, 4, 0) == 0xfdeccdefu");
  __ mov(r0, Operand(0xfdeccdefu));
  __ mov(r1, r0);
  __ Bfc(r1, r1, 4, 0);  // a mov whatever lsb
  __ cmp(r1, Operand(0xfdeccdefu));
  B_LINE(ne, &error);
  CMT("Verify Bfc(0xfdeccdefu, 0, 31) == 0x80000000");
  __ mov(r1, r0);
  __ Bfc(r1, r1, 0, 31);
  __ cmp(r1, Operand(0x80000000u));
  B_LINE(ne, &error);
  CMT("Verify Bfc(0xfdeccdefu, 1, 31) == 0x80000000");
  __ mov(r1, r0);
  __ Bfc(r1, r1, 1, 30);
  __ cmp(r1, Operand(0x80000001u));
  B_LINE(ne, &error);
  CMT("Verify Bfc(0xfdeccdefu, 9, 16) == 0xfc0001ef");
  __ mov(r1, r0);
  __ Bfc(r1, r1, 9, 16);
  __ cmp(r1, Operand(0xfc0001efu));
  B_LINE(ne, &error);

  CMT("Verify Ubfx(0xfdeccdefu, 0, 32) == 0xfdeccdef");
  __ mov(r0, Operand(0xfdeccdefu));
  __ Ubfx(r1, r0, 0, 32);  // a mov actually
  __ cmp(r1, Operand(0xfdeccdefu));
  B_LINE(ne, &error);
  CMT("Verify Ubfx(0xfdeccdef, 0, 31) == 0x7deccdef");
  __ Ubfx(r1, r0, 0, 31);
  __ cmp(r1, Operand(0x7deccdef));
  B_LINE(ne, &error);
  CMT("Verify Ubfx(0xfdeccdef, 1, 30) == 0x3ef666f7");
  __ Ubfx(r1, r0, 1, 30);
  __ cmp(r1, Operand(0x3ef666f7));
  B_LINE(ne, &error);
  CMT("Verify Ubfx(0xfdeccdef, 9, 16) == 0xf666");
  __ Ubfx(r1, r0, 9, 16);
  __ cmp(r1, Operand(0xf666));
  B_LINE(ne, &error);

  CMT("Verify Sbfx(0xfdeccdef, 0, 32) == 0xfdeccdef");
  __ mov(r0, Operand(0xfdeccdefu));
  __ Sbfx(r1, r0, 0, 32);  // a mov actually
  __ cmp(r1, Operand(0xfdeccdefu));
  B_LINE(ne, &error);
  CMT("Verify Sbfx(0xfdeccdef, 0, 31) == 0xfdeccdef");
  __ Sbfx(r1, r0, 0, 31);
  __ cmp(r1, Operand(0xfdeccdefu));
  B_LINE(ne, &error);
  CMT("Verify Sbfx(0xfdeccdef, 1, 31) == 0xfef666f7");
  __ Sbfx(r1, r0, 1, 30);
  __ cmp(r1, Operand(0xfef666f7u));
  B_LINE(ne, &error);
  CMT("Verify Sbfx(0xfdeccdef, 9, 16) == 0xfffff666");
  __ Sbfx(r1, r0, 9, 16);
  __ cmp(r1, Operand(0xfffff666));
  B_LINE(ne, &error);

  CMT("Verify Bfi(0xfdeccdef, 0xaaaaaaaa, 0, 0) == 0xfdeccdef");
  __ mov(r2, Operand(0xaaaaaa));
  __ mov(r0, Operand(0xfdeccdefu));
  __ mov(r1, r0);
  __ Bfi(r1, r2, r4, 0, 0);  // a nop
  __ cmp(r1, r0);
  B_LINE(ne, &error);
  CMT("Verify Bfi(0xfdeccdef, 0xaaaaaaaa, 4, 0) == 0xfdeccdef");
  __ mov(r2, Operand(0xaaaaaa));
  __ mov(r0, Operand(0xfdeccdefu));
  __ mov(r1, r0);
  __ Bfi(r1, r2, r4, 0, 0);  // a nop whatever lsb
  __ cmp(r1, r0);
  B_LINE(ne, &error);
  CMT("Verify Bfi(0xfdeccdef, 0xaaaaaaaa, 0, 32) == 0xaaaaaaaa");
  __ mov(r1, r0);
  __ Bfi(r1, r2, r4, 0, 32);  // a mov actually
  __ cmp(r1, r2);
  B_LINE(ne, &error);
  CMT("Verify Bfi(0xfdeccdef, 0xaaaaaaaa, 0, 16) == 0xfdecaaaa");
  __ mov(r1, r0);
  __ Bfi(r1, r2, r4, 0, 16);
  __ cmp(r1, Operand(0xfdecaaaau));
  B_LINE(ne, &error);
  CMT("Verify Bfi(0xfdeccdef, 0xaaaaaaaa, 4, 28) == 0xfd5555ef");
  __ mov(r1, r0);
  __ Bfi(r1, r2, r4, 9, 16);
  __ cmp(r1, Operand(0xfd5555efu));
  B_LINE(ne, &error);

  CMT("Verify Usat(-10, 8) == 0");
  __ mov(r0, Operand(-10));
  __ Usat(r1, 8, r0);
  __ cmp(r1, Operand(0));
  B_LINE(ne, &error);
  CMT("Verify Usat(256, 8) == 255");
  __ mov(r0, Operand(256));
  __ Usat(r0, 8, r0);
  __ cmp(r0, Operand(255));
  B_LINE(ne, &error);

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

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// Test Stack Pointer changes
TEST(sh4_ma_2) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r8, sp);

  __ add(sp, sp, Operand(-16));

  __ mov(sp, r8);

  __ mov(r0, Operand(0));

  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
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
  __ mov(r0, Operand(3));
  __ mov(r1, Operand(5));
  __ mov(r2, Operand(7));
  __ mov(r3, Operand(11));
  __ Push(r0, r1);
  __ mov(r0, Operand(0xdeadbeef));
  __ mov(r1, Operand(0xdeadbeef));
  __ Pop(r0, r1);
  __ cmp(r0, Operand(3));
  B_LINE(ne, &error);
  __ cmp(r1, Operand(5));
  B_LINE(ne, &error);

  __ Push(r0, r1, r2);
  __ mov(r0, Operand(0xdeadbeef));
  __ mov(r1, Operand(0xdeadbeef));
  __ mov(r2, Operand(0xdeadbeef));
  __ pop(r2);
  __ Pop(r0, r1);
  __ cmp(r0, Operand(3));
  B_LINE(ne, &error);
  __ cmp(r1, Operand(5));
  B_LINE(ne, &error);
  __ cmp(r2, Operand(7));
  B_LINE(ne, &error);

  __ Push(r0, r1, r2, r3);
  __ mov(r0, Operand(0xdeadbeef));
  __ mov(r1, Operand(0xdeadbeef));
  __ mov(r2, Operand(0xdeadbeef));
  __ mov(r3, Operand(0xdeadbeef));
  __ Pop(r2, r3);
  __ Pop(r0, r1);
  __ cmp(r0, Operand(3));
  B_LINE(ne, &error);
  __ cmp(r1, Operand(5));
  B_LINE(ne, &error);
  __ cmp(r2, Operand(7));
  B_LINE(ne, &error);
  __ cmp(r3, Operand(11));
  B_LINE(ne, &error);


  // All ok.
  __ mov(sp, r8);
  __ mov(r0, Operand(0));
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

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
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
  __ mov(r0, Operand(3));
  __ mov(r1, Operand(5));
  __ mov(r2, Operand(7));
  __ mov(r3, Operand(11));
  __ Push(r0, r1, r2, r3);
  __ mov(r0, Operand(0xdeadbeef));
  __ mov(r1, Operand(0xdeadbeef));
  __ mov(r2, Operand(0xdeadbeef));
  __ mov(r3, Operand(0xdeadbeef));

  __ Ldrd(r0, r1, MemOperand(sp, 8));
  __ cmp(r1, Operand(3));
  B_LINE(ne, &error);
  __ cmp(r0, Operand(5));
  B_LINE(ne, &error);

  __ Ldrd(r2, r3, MemOperand(sp, 0));
  __ cmp(r2, Operand(11));
  B_LINE(ne, &error);
  __ cmp(r3, Operand(7));
  B_LINE(ne, &error);

  __ mov(r2, Operand(0xdeadbeef));
  __ mov(r3, Operand(0xdeadbeef));
  __ mov(r2, sp);  // Test case where base is also destination
  __ Ldrd(r2, r3, MemOperand(r2, 0));
  __ cmp(r2, Operand(11));
  B_LINE(ne, &error);
  __ cmp(r3, Operand(7));
  B_LINE(ne, &error);

  __ Strd(r0, r1, MemOperand(sp, 0));
  __ Strd(r2, r3, MemOperand(sp, 8));
  __ Pop(r1, r0);
  __ Pop(r3, r2);
  __ cmp(r0, Operand(5));
  B_LINE(ne, &error);
  __ cmp(r1, Operand(3));
  B_LINE(ne, &error);
  __ cmp(r2, Operand(11));
  B_LINE(ne, &error);
  __ cmp(r3, Operand(7));
  B_LINE(ne, &error);

  // All ok.
  __ mov(sp, r8);
  __ mov(r0, Operand(0));
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

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


// Test JumpIfSmi(), JumpIfNotSmi(), LoadRoot(), StoreRoot(), CompareRoot(),
// CheckMap(), IsObjectStringType()
TEST(sh4_ma_5) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ LoadRoot(r1, Heap::kRealStackLimitRootIndex);
  __ tst(r1, Operand(1));  // Check that it is a Smi
  B_LINE(ne, &error);
  __ cmphs(sp, r1);  // Check that stack limit is lower than sp
  B_LINE(ne, &error);

  __ LoadRoot(r0, Heap::kRealStackLimitRootIndex);
  __ mov(r1, Operand(0xdead));
  __ StoreRoot(r1, Heap::kRealStackLimitRootIndex);
  __ LoadRoot(r1, Heap::kRealStackLimitRootIndex);
  __ cmp(r1, Operand(0xdead));
  B_LINE(ne, &error);
  __ StoreRoot(r0, Heap::kRealStackLimitRootIndex);

  __ mov(r0, Operand(THE_HOLE_VALUE()));
  __ LoadRoot(r1, Heap::kTheHoleValueRootIndex);
  __ cmp(r0, r1);
  B_LINE(ne, &error);
  __ CompareRoot(r0, Heap::kTheHoleValueRootIndex);
  B_LINE(ne, &error);

  Label is_smi1, fail_is_smi1, fail_is_smi2, skip_is_smi1, skip_is_smi2;
  __ mov(r0, Operand(2));  // Smi integer 1
  __ JumpIfSmi(r0, &is_smi1);
  B_LINE(al, &error);  // should not be there
  __ bind(&is_smi1);

  __ mov(r0, Operand(1));  // Heap object 0
  __ JumpIfSmi(r0, &fail_is_smi1);
  __ jmp(&skip_is_smi1);
  __ bind(&fail_is_smi1);
  B_LINE(al, &error);
  __ bind(&skip_is_smi1);

  __ mov(r0, Operand(3));  // Failure object 0
  __ JumpIfSmi(r0, &fail_is_smi2);
  __ jmp(&skip_is_smi2);
  __ bind(&fail_is_smi2);
  B_LINE(al, &error);
  __ bind(&skip_is_smi2);


  Label is_not_smi1, is_not_smi2, fail_is_not_smi1, skip_is_not_smi1;
  __ mov(r0, Operand(1));  // Heap object 0
  __ JumpIfNotSmi(r0, &is_not_smi1);
  B_LINE(al, &error);  // should not be there
  __ bind(&is_not_smi1);

  __ mov(r0, Operand(3));  // Failure object 0
  __ JumpIfNotSmi(r0, &is_not_smi2);
  B_LINE(al, &error);  // should not be there
  __ bind(&is_not_smi2);

  __ mov(r0, Operand(2));  // Smi integer 1
  __ JumpIfNotSmi(r0, &fail_is_not_smi1);
  __ jmp(&skip_is_not_smi1);
  __ bind(&fail_is_not_smi1);
  B_LINE(al, &error);
  __ bind(&skip_is_not_smi1);

  Label no_map1, no_map2, no_map3, no_map4, no_map5, skip_no_map1;
  __ mov(r0, Operand(2));  // Smi integer 1
  __ CheckMap(r0, r1/*scratch*/, GLOBAL_CONTEXT_MAP(),
              &no_map1, DO_SMI_CHECK);  // Check that Smi fails
  B_LINE(al, &error);  // should not be there
  __ bind(&no_map1);

  __ mov(r0, Operand(EMPTY_STRING()));  // String object
  __ CheckMap(r0, r1/*scratch*/, HEAP_NUMBER_MAP(),
              &no_map2, DO_SMI_CHECK);  // Not the right map
  B_LINE(al, &error);  // should not be there
  __ bind(&no_map2);

  __ mov(r0, Operand(EMPTY_STRING()));  // String object
  __ CheckMap(r0, r1/*scratch*/, HEAP_NUMBER_MAP(),
              &no_map3, DO_SMI_CHECK);  // Heap object but not the right map
  B_LINE(al, &error);  // should not be there
  __ bind(&no_map3);

  __ mov(r0, Operand(EMPTY_STRING()));  // String object
  __ CheckMap(r0, r1/*scratch*/, Heap::kHeapNumberMapRootIndex,
              &no_map4, DONT_DO_SMI_CHECK);  // Heap object but not the right map
  B_LINE(al, &error);  // should not be there
  __ bind(&no_map4);

  __ mov(r0, Operand(NAN_VALUE()));  // HeapNumber object
  __ CheckMap(r0, r1/*scratch*/, HEAP_NUMBER_MAP(),
               &no_map5, DO_SMI_CHECK);  // This is the right map
  __ CheckMap(r0, r1/*scratch*/, HEAP_NUMBER_MAP(),
               &no_map5, DONT_DO_SMI_CHECK);
  __ CheckMap(r0, r1/*scratch*/, Heap::kHeapNumberMapRootIndex,
               &no_map5, DO_SMI_CHECK);
  __ CheckMap(r0, r1/*scratch*/, Heap::kHeapNumberMapRootIndex,
               &no_map5, DONT_DO_SMI_CHECK);
  __ jmp(&skip_no_map1);
  __ bind(&no_map5);
  B_LINE(al, &error);  // should not be there
  __ bind(&skip_no_map1);

  Label skip_is_string;
  __ mov(r0, Operand(EMPTY_STRING()));  // String object
  Condition cond = assm.IsObjectStringType(r0, r1);
  __ b(cond, &skip_is_string);
  B_LINE(al, &error);
  __ bind(&skip_is_string);
  __ mov(r0, Operand(NAN_VALUE()));  // HeapNumber object
  cond = assm.IsObjectStringType(r0, r1);
  B_LINE(cond, &error);

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

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


// Test SmiTag(), SmiUntag(), TrySmiTag(),
// GetLeastBitsFromSmi(), GetLeastBitsFromInt32()
// JumpIfNotPowerOfTwoOrZero(), JumpIfNotPowerOfTwoOrZeroAndNeg()
// JumpIfNotBothSmi()/JumpIfEitherSmi()
TEST(sh4_ma_6) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r0, Operand(1));
  __ SmiTag(r0);
  __ cmp(r0, Operand(2));
  B_LINE(ne, &error);
  __ SmiUntag(r0);
  __ cmp(r0, Operand(1));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0x3fffffff));
  __ SmiTag(r1, r0);
  __ cmp(r1, Operand(0x7ffffffe));
  B_LINE(ne, &error);
  __ SmiUntag(r0, r1);
  __ cmp(r0, Operand(0x3fffffff));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0xc0000000));
  __ SmiTag(r0);
  __ cmp(r0, Operand(0x80000000));
  B_LINE(ne, &error);
  __ SmiUntag(r1, r0);
  __ cmp(r1, Operand(0xc0000000));
  B_LINE(ne, &error);

  CMT("Check GetLeastBitsFromSmi(Smi(33), 5) == 1");
  __ mov(r0, Operand(33<<1));
  __ mov(r2, Operand(1));
  __ GetLeastBitsFromSmi(r1, r0, 5);
  __ cmp(r1, r2);
  B_LINE(ne, &error);

  CMT("Check GetLeastBitsFromInt32(34, 5) == 2");
  __ mov(r0, Operand(34));
  __ mov(r2, Operand(2));
  __ GetLeastBitsFromInt32(r1, r0, 5);
  __ cmp(r1, r2);
  B_LINE(ne, &error);

  Label not_smi1, not_smi2, not_smi3, skip_smi1;
  __ mov(r0, Operand(0xc0000000));
  __ TrySmiTag(r0, &not_smi1);
  __ cmp(r0, Operand(0x80000000));
  B_LINE(ne, &error);
  __ jmp(&skip_smi1);
  __ bind(&not_smi1);
  B_LINE(ne, &error);
  __ bind(&skip_smi1);

  __ mov(r0, Operand(0x80000000));  // not a smi
  __ TrySmiTag(r0, &not_smi2);
  B_LINE(ne, &error);
  __ bind(&not_smi2);

  __ mov(r0, Operand(0x40000000));  // not a smi
  __ TrySmiTag(r0, &not_smi3);
  B_LINE(ne, &error);
  __ bind(&not_smi3);


  Label not_power1, not_power2, not_power3, not_power4,
    skip_power1, skip_power2, skip_power3;
  __ mov(r0, Operand(0));
  __ JumpIfNotPowerOfTwoOrZero(r0, r1, &not_power1);
  B_LINE(al, &error);
  __ bind(&not_power1);

  __ mov(r0, Operand(0x80000000));
  // r1 will contain (r0 - 1) (the power of two mask)
  __ JumpIfNotPowerOfTwoOrZero(r0, r1, &not_power2);
  __ jmp(&skip_power2);
  __ bind(&not_power2);
  B_LINE(al, &error);
  __ bind(&skip_power2);
  __ cmp(r1, Operand(0x7fffffff));
    B_LINE(ne, &error);

  __ mov(r0, Operand(1));
  __ JumpIfNotPowerOfTwoOrZero(r0, r1, &not_power2);
  __ jmp(&skip_power3);
  __ bind(&not_power3);
  B_LINE(al, &error);
  __ bind(&skip_power3);
  __ cmp(r1, Operand(0));
    B_LINE(ne, &error);

  __ mov(r0, Operand(0x30));
  __ JumpIfNotPowerOfTwoOrZero(r0, r1, &not_power4);
  B_LINE(al, &error);
  __ bind(&not_power4);

  Label not_pow1, zero_neg1, not_pow2, zero_neg2, not_pow3, zero_neg3,
    not_pow4, zero_neg4, not_pow5, zero_neg5, not_pow6, zero_neg6,
    skip_pow5, skip_pow6, skip_pow2;
  __ mov(r0, Operand(0));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1, &zero_neg1, &not_pow1);
  B_LINE(al, &error);
  __ bind(&not_pow1);
  B_LINE(al, &error);
  __ bind(&zero_neg1);

  __ mov(r0, Operand(0x80000000));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1, &zero_neg2, &not_pow2);
  __ jmp(&skip_pow2);
  __ bind(&not_pow2);
  B_LINE(al, &error);
  __ bind(&zero_neg2);
  B_LINE(al, &error);
  __ bind(&skip_pow2);
  __ cmp(r1, Operand(0x7fffffff));
    B_LINE(ne, &error);

  __ mov(r0, Operand(-1));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1, &zero_neg3, &not_pow3);
  B_LINE(al, &error);
  __ bind(&not_pow3);
  B_LINE(al, &error);
  __ bind(&zero_neg3);

  __ mov(r0, Operand(0x30));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1, &zero_neg4, &not_pow4);
  B_LINE(al, &error);
  __ bind(&zero_neg4);
  B_LINE(al, &error);
  __ bind(&not_pow4);

  __ mov(r0, Operand(1));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1,  &zero_neg5, &not_pow5);
  __ jmp(&skip_pow5);
  __ bind(&not_pow5);
  B_LINE(al, &error);
  __ bind(&zero_neg5);
  B_LINE(al, &error);
  __ bind(&skip_pow5);
  __ cmp(r1, Operand(0));
    B_LINE(ne, &error);

  __ mov(r0, Operand(0x40000000));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1,  &zero_neg6, &not_pow6);
  __ jmp(&skip_pow6);
  __ bind(&not_pow6);
  B_LINE(al, &error);
  __ bind(&zero_neg6);
  B_LINE(al, &error);
  __ bind(&skip_pow6);
  __ cmp(r1, Operand(0x3fffffff));
  B_LINE(ne, &error);

  Label not_both_smi1, skip_not_both_smi1, not_both_smi2, not_both_smi3,
    not_both_smi4;
  CMT("Check JumpIfNotBothSmi(): both Smi");
  __ mov(r0, Operand(Smi::FromInt(1)));
  __ mov(r1, Operand(Smi::FromInt(2)));
  __ JumpIfNotBothSmi(r0, r1, &not_both_smi1);
  __ jmp(&skip_not_both_smi1);
  __ bind(&not_both_smi1);
  B_LINE(al, &error);
  __ bind(&skip_not_both_smi1);

  CMT("Check JumpIfNotBothSmi(): left Smi");
  __ mov(r0, Operand(Smi::FromInt(1)));
  __ mov(r1, Operand(0x41));  // not a smi
  __ JumpIfNotBothSmi(r0, r1, &not_both_smi2);
  B_LINE(al, &error);
  __ bind(&not_both_smi2);

  CMT("Check JumpIfNotBothSmi(): right Smi");
  __ JumpIfNotBothSmi(r1, r0, &not_both_smi3);
  B_LINE(al, &error);
  __ bind(&not_both_smi3);

  CMT("Check JumpIfNotBothSmi(): none Smi");
  __ mov(r0, Operand(0x33));  // not a smi
  __ JumpIfNotBothSmi(r1, r0, &not_both_smi4);
  B_LINE(al, &error);
  __ bind(&not_both_smi4);


  Label either_smi1, either_smi2, either_smi3, either_smi4,
    skip_either_smi4;
  CMT("Check JumpIfEitherSmi(): both Smi");
  __ mov(r0, Operand(Smi::FromInt(1)));
  __ mov(r1, Operand(Smi::FromInt(2)));
  __ JumpIfEitherSmi(r0, r1, &either_smi1);
  B_LINE(al, &error);
  __ bind(&either_smi1);

  CMT("Check JumpIfEitherSmi(): left Smi");
  __ mov(r0, Operand(Smi::FromInt(1)));
  __ mov(r1, Operand(0x41));  // not a smi
  __ JumpIfEitherSmi(r0, r1, &either_smi2);
  B_LINE(al, &error);
  __ bind(&either_smi2);

  CMT("Check JumpIfEitherSmi(): right Smi");
  __ JumpIfEitherSmi(r1, r0, &either_smi3);
  B_LINE(al, &error);
  __ bind(&either_smi3);

  CMT("Check JumpIfEitherSmi(): none Smi");
  __ mov(r0, Operand(0x33));  // not a smi also
  __ JumpIfEitherSmi(r1, r0, &either_smi4);
  __ jmp(&skip_either_smi4);
  __ bind(&either_smi4);
  B_LINE(al, &error);
  __ bind(&skip_either_smi4);


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

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}


static void DoubleAsTwoUInt32(double d, uint32_t* lo, uint32_t* hi) {
  uint64_t i;
  OS::MemCopy(&i, &d, 8);

  *lo = i & 0xffffffff;
  *hi = i >> 32;
}

// Test TryDoubleToInt32Exact()
TEST(sh4_ma_7) {
  BEGIN();

  Label error;
  PROLOGUE();

  uint32_t low, high;

  CMT("Check TryDoubleToInt32Exact(4.0) -> exact");
  DoubleAsTwoUInt32(4.0, &low, &high);
  __ mov(r0, Operand(4));
  __ mov(r1, Operand(low));
  __ mov(r2, Operand(high));
  __ movd(sh4_dr0, r1, r2);
  __ TryDoubleToInt32Exact(r1, sh4_dr0, sh4_dr2/*scratch*/);
  B_LINE(ne, &error); // error if not found exact
  __ cmp(r1, r0);
  B_LINE(ne, &error);

  CMT("Check TryDoubleToInt32Exact(4.1) -> not exact");
  DoubleAsTwoUInt32(4.1, &low, &high);
  __ mov(r1, Operand(low));
  __ mov(r2, Operand(high));
  __ movd(sh4_dr0, r1, r2);
  __ TryDoubleToInt32Exact(r1, sh4_dr0, sh4_dr2/*scratch*/);
  B_LINE(eq, &error); // error if found exact

  CMT("Check TryDoubleToInt32Exact(0x80000000) -> not exact (overflow)");
  DoubleAsTwoUInt32((double)0x80000000U, &low, &high);
  __ mov(r1, Operand(low));
  __ mov(r2, Operand(high));
  __ movd(sh4_dr0, r1, r2);
  __ TryDoubleToInt32Exact(r1, sh4_dr0, sh4_dr2/*scratch*/);
  B_LINE(eq, &error); // error if found exact

  CMT("Check TryDoubleToInt32Exact(+inf) -> not exact");
  DoubleAsTwoUInt32(V8_INFINITY, &low, &high);
  __ mov(r1, Operand(low));
  __ mov(r2, Operand(high));
  __ movd(sh4_dr0, r1, r2);
  __ TryDoubleToInt32Exact(r1, sh4_dr0, sh4_dr2/*scratch*/);
  B_LINE(eq, &error); // error if found exact

  // TODO(stm)
  // CMT("Check TryDoubleToInt32Exact(nan) -> not exact");

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

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// Test CountLeadingZeros
TEST(sh4_ma_8) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(0));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(32));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(1));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(31));
  B_LINE(ne, &error);

  // Test same register for input and output
  __ mov(r0, Operand(3));
  __ CountLeadingZeros(r0, r0, r3);
  __ cmpeq(r0, Operand(30));
  B_LINE(ne, &error);

  // Test same register for input and scratch
  __ mov(r0, Operand(0));
  __ mov(r1, Operand(5));
  __ CountLeadingZeros(r0, r1, r1);
  __ cmpeq(r0, Operand(29));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(13));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(28));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(24));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(27));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(41));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(26));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(83));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(25));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(211));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(24));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(467));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(23));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(726));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(22));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(1782));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(21));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(3824));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(20));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(4336));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(19));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(8388607));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(9));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(1082130431));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(1));
  B_LINE(ne, &error);

  __ mov(r0, Operand(0));
  __ mov(r1, Operand(3229614079u));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Operand(0));
  B_LINE(ne, &error);



  EPILOGUE();
  __ mov(r0, Operand(0));
  __ rts();

  __ bind(&error);
  __ mov(r0, r10);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}

// Test CopyBytes
TEST(sh4_ma_9) {
  BEGIN();

  Label error;

  PROLOGUE();

  // Push garbage values on the stack
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ push(Operand(0xdeadbeef));
  __ mov(r0, sp);

  // Push nice values that will be copied
  __ push(Operand(0xf0000000));
  __ push(Operand(0x0f000000));
  __ push(Operand(0x00f00000));
  __ push(Operand(0x000f0000));
  __ push(Operand(0x00000f00));
  __ push(Operand(0x000000f0));
  __ push(Operand(0x0000000f));
  __ push(Operand(0x13453abb));
  __ push(Operand(0xabcdef01));
  __ push(Operand(0x789abcde));
  __ mov(r1, sp);

  __ mov(r2, Operand(40));
  __ CopyBytes(r1, r0, r2, r3);

  // Check that the src is untouched
#define POP_AND_CHECK(addr, val)        \
  __ pop(r0);                           \
  __ cmpeq(r0, Operand(addr));        \
  __ mov(r1, Operand(val));           \
  B_LINE(ne, &error);

  POP_AND_CHECK(0x789abcde, 19 * 4);
  POP_AND_CHECK(0xabcdef01, 18 * 4);
  POP_AND_CHECK(0x13453abb, 17 * 4);
  POP_AND_CHECK(0x0000000f, 16 * 4);
  POP_AND_CHECK(0x000000f0, 15 * 4);
  POP_AND_CHECK(0x00000f00, 14 * 4);
  POP_AND_CHECK(0x000f0000, 13 * 4);
  POP_AND_CHECK(0x00f00000, 12 * 4);
  POP_AND_CHECK(0x0f000000, 11 * 4);
  POP_AND_CHECK(0xf0000000, 10 * 4);

  // check that the dst is modified
  POP_AND_CHECK(0x789abcde, 9 * 4);
  POP_AND_CHECK(0xabcdef01, 8 * 4);
  POP_AND_CHECK(0x13453abb, 7 * 4);
  POP_AND_CHECK(0x0000000f, 6 * 4);
  POP_AND_CHECK(0x000000f0, 5 * 4);
  POP_AND_CHECK(0x00000f00, 4 * 4);
  POP_AND_CHECK(0x000f0000, 3 * 4);
  POP_AND_CHECK(0x00f00000, 2 * 4);
  POP_AND_CHECK(0x0f000000, 1 * 4);
  POP_AND_CHECK(0xf0000000, 0 * 4);

#undef POP_AND_CHECK

  // Test that copying 0 bytes dos not do anything
  __ push(Operand(0xdeadbeef));
  __ mov(r0, sp);
  __ push(Operand(0x00000ff0));
  __ mov(r1, sp);

  __ mov(r2, Operand(0));
  __ CopyBytes(r1, r0, r2, r3);
  __ pop(r0);
  __ cmpeq(r0, Operand(0x00000ff0));
  __ mov(r1, Operand(1 * 4));
  B_LINE(ne, &error);

  __ pop(r0);
  __ cmpeq(r0, Operand(0xdeadbeef));
  __ mov(r1, Operand(0 * 4));
  B_LINE(ne, &error);

  EPILOGUE();
  __ mov(r0, Operand(0));
  __ rts();

  __ bind(&error);
  __ add(sp, sp, r1);
  __ mov(r0, r10);
  EPILOGUE();
  __ rts();

  JIT();
#ifdef DEBUG
  Code::cast(code)->Print();
#endif

  F0 f = FUNCTION_CAST<F0>(Code::cast(code)->entry());
  int res = CAST(CALL_GENERATED_CODE(f, 0, 0, 0, 0, 0));
  ::printf("f() = %d\n", res);
  CHECK_EQ(0, res);
}
