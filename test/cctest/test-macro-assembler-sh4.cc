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
  __ push(roots); \
  ExternalReference roots_address = \
    ExternalReference::roots_address(assm.isolate()); \
  __ mov(roots, Operand(roots_address)); \
  } while (0)

#define EPILOGUE() \
  do { \
  __ pop(roots); \
  __ pop(r11); \
  __ pop(r10);	\
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
  i::FLAG_full_compiler = false;                        \
  i::FLAG_code_comments = true;                         \
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
  __ Bfc(r1, 0, 32); // a full clear
  __ cmp(r1, Immediate(0));
  B_LINE(ne, &error); 
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
  __ Ubfx(r1, r0, 0, 32); // a mov actually
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
  __ Sbfx(r1, r0, 0, 32); // a mov actually
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

  __ mov(r2, Immediate(0xdeadbeef));
  __ mov(r3, Immediate(0xdeadbeef));
  __ mov(r2, sp); // Test case where base is also destination
  __ Ldrd(r2, r3, MemOperand(r2, 0));
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


// Test JumpIfSmi(), JumpIfNotSmi(), LoadRoot(), StoreRoot(), CompareRoot(),
// CheckMap(), IsObjectStringType()
TEST(sh4_ma_5) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ LoadRoot(r1, Heap::kRealStackLimitRootIndex);
  __ tst(r1, Immediate(1)); // Check that it is a Smi
  B_LINE(ne, &error);
  __ cmphs(sp, r1); // Check that stack limit is lower than sp
  B_LINE(ne, &error);

  __ LoadRoot(r0, Heap::kRealStackLimitRootIndex);
  __ mov(r1, Immediate(0xdead));
  __ StoreRoot(r1, Heap::kRealStackLimitRootIndex);
  __ LoadRoot(r1, Heap::kRealStackLimitRootIndex);
  __ cmp(r1, Immediate(0xdead));
  B_LINE(ne, &error);
  __ StoreRoot(r0, Heap::kRealStackLimitRootIndex);
  
  __ mov(r0, Immediate(THE_HOLE_VALUE()));
  __ LoadRoot(r1, Heap::kTheHoleValueRootIndex);
  __ cmp(r0, r1);
  B_LINE(ne, &error);
  __ CompareRoot(r0, Heap::kTheHoleValueRootIndex);
  B_LINE(ne, &error);

  Label is_smi1, fail_is_smi1, fail_is_smi2, skip_is_smi1, skip_is_smi2;
  __ mov(r0, Immediate(2)); // Smi integer 1
  __ JumpIfSmi(r0, &is_smi1);
  B_LINE(al, &error); // should not be there
  __ bind(&is_smi1);

  __ mov(r0, Immediate(1)); // Heap object 0
  __ JumpIfSmi(r0, &fail_is_smi1);
  __ jmp(&skip_is_smi1);
  __ bind(&fail_is_smi1);
  B_LINE(al, &error);
  __ bind(&skip_is_smi1);

  __ mov(r0, Immediate(3)); // Failure object 0
  __ JumpIfSmi(r0, &fail_is_smi2);
  __ jmp(&skip_is_smi2);
  __ bind(&fail_is_smi2);
  B_LINE(al, &error);
  __ bind(&skip_is_smi2);


  Label is_not_smi1, is_not_smi2, fail_is_not_smi1, skip_is_not_smi1;
  __ mov(r0, Immediate(1)); // Heap object 0
  __ JumpIfNotSmi(r0, &is_not_smi1);
  B_LINE(al, &error); // should not be there
  __ bind(&is_not_smi1);

  __ mov(r0, Immediate(3)); // Failure object 0
  __ JumpIfNotSmi(r0, &is_not_smi2);
  B_LINE(al, &error); // should not be there
  __ bind(&is_not_smi2);

  __ mov(r0, Immediate(2)); // Smi integer 1
  __ JumpIfNotSmi(r0, &fail_is_not_smi1);
  __ jmp(&skip_is_not_smi1);
  __ bind(&fail_is_not_smi1);
  B_LINE(al, &error);
  __ bind(&skip_is_not_smi1);

  Label no_map1, no_map2, no_map3, no_map4, no_map5, skip_no_map1;
  __ mov(r0, Immediate(2)); // Smi integer 1
  __ CheckMap(r0, r1/*scratch*/, GLOBAL_CONTEXT_MAP(), 
	      &no_map1, false); // Check that Smi fails
  B_LINE(al, &error); // should not be there
  __ bind(&no_map1);

  __ mov(r0, Immediate(EMPTY_STRING())); // String object
  __ CheckMap(r0, r1/*scratch*/, HEAP_NUMBER_MAP(), 
	      &no_map2, false); // Not the right map
  B_LINE(al, &error); // should not be there
  __ bind(&no_map2);

  __ mov(r0, Immediate(EMPTY_STRING())); // String object
  __ CheckMap(r0, r1/*scratch*/, HEAP_NUMBER_MAP(), 
	      &no_map3, true); // Heap object but not the right map
  B_LINE(al, &error); // should not be there
  __ bind(&no_map3);

  __ mov(r0, Immediate(EMPTY_STRING())); // String object
  __ CheckMap(r0, r1/*scratch*/, Heap::kHeapNumberMapRootIndex, 
	      &no_map4, true); // Heap object but not the right map
  B_LINE(al, &error); // should not be there
  __ bind(&no_map4);

  __ mov(r0, Immediate(NAN_VALUE())); // HeapNumber object
  __ CheckMap(r0, r1/*scratch*/, HEAP_NUMBER_MAP(), 
 	      &no_map5, false); // This is the right map 
  __ CheckMap(r0, r1/*scratch*/, HEAP_NUMBER_MAP(), 
 	      &no_map5, true);
  __ CheckMap(r0, r1/*scratch*/, Heap::kHeapNumberMapRootIndex, 
 	      &no_map5, false);
  __ CheckMap(r0, r1/*scratch*/, Heap::kHeapNumberMapRootIndex, 
 	      &no_map5, true);
  __ jmp(&skip_no_map1);
  __ bind(&no_map5);
  B_LINE(al, &error); // should not be there
  __ bind(&skip_no_map1);
 
  Label skip_is_string;
  __ mov(r0, Immediate(EMPTY_STRING())); // String object
  Condition cond = assm.IsObjectStringType(r0, r1);
  __ b(cond, &skip_is_string);
  B_LINE(al, &error);
  __ bind(&skip_is_string);
  __ mov(r0, Immediate(NAN_VALUE())); // HeapNumber object
  cond = assm.IsObjectStringType(r0, r1);
  B_LINE(cond, &error);

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


// Test SmiTag(), SmiUntag(), TrySmiTag(),
// GetLeastBitsFromSmi(), GetLeastBitsFromInt32()
// JumpIfNotPowerOfTwoOrZero(), JumpIfNotPowerOfTwoOrZeroAndNeg()
// JumpIfNotBothSmi()/JumpIfEitherSmi()
TEST(sh4_ma_6) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r0, Immediate(1));
  __ SmiTag(r0);
  __ cmp(r0, Immediate(2));
  B_LINE(ne, &error);
  __ SmiUntag(r0);
  __ cmp(r0, Immediate(1));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0x3fffffff));
  __ SmiTag(r1, r0);
  __ cmp(r1, Immediate(0x7ffffffe));
  B_LINE(ne, &error);
  __ SmiUntag(r0, r1);
  __ cmp(r0, Immediate(0x3fffffff));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0xc0000000));
  __ SmiTag(r0);
  __ cmp(r0, Immediate(0x80000000));
  B_LINE(ne, &error);
  __ SmiUntag(r1, r0);
  __ cmp(r1, Immediate(0xc0000000));
  B_LINE(ne, &error);

  CMT("Check GetLeastBitsFromSmi(Smi(33), 5) == 1");
  __ mov(r0, Immediate(33<<1)); 
  __ mov(r2, Immediate(1)); 
  __ GetLeastBitsFromSmi(r1, r0, 5);
  __ cmp(r1, r2);
  B_LINE(ne, &error);

  CMT("Check GetLeastBitsFromInt32(34, 5) == 2");
  __ mov(r0, Immediate(34)); 
  __ mov(r2, Immediate(2)); 
  __ GetLeastBitsFromInt32(r1, r0, 5);
  __ cmp(r1, r2);
  B_LINE(ne, &error);

  Label not_smi1, not_smi2, not_smi3, skip_smi1;
  __ mov(r0, Immediate(0xc0000000));
  __ TrySmiTag(r0, &not_smi1, r1/*scratch*/);
  __ cmp(r0, Immediate(0x80000000));
  B_LINE(ne, &error);
  __ jmp(&skip_smi1);
  __ bind(&not_smi1);
  B_LINE(ne, &error);
  __ bind(&skip_smi1);

  __ mov(r0, Immediate(0x80000000));  // not a smi
  __ TrySmiTag(r0, &not_smi2, r1/*scratch*/);
  B_LINE(ne, &error);
  __ bind(&not_smi2);

  __ mov(r0, Immediate(0x40000000));  // not a smi
  __ TrySmiTag(r0, &not_smi3, r1/*scratch*/);
  B_LINE(ne, &error);
  __ bind(&not_smi3);
  

  Label not_power1, not_power2, not_power3, not_power4,
    skip_power1, skip_power2, skip_power3;
  __ mov(r0, Immediate(0));
  __ JumpIfNotPowerOfTwoOrZero(r0, r1, &not_power1);
  B_LINE(al, &error);
  __ bind(&not_power1);

  __ mov(r0, Immediate(0x80000000));
  // r1 will contain (r0 - 1) (the power of two mask)
  __ JumpIfNotPowerOfTwoOrZero(r0, r1, &not_power2);
  __ jmp(&skip_power2);
  __ bind(&not_power2);
  B_LINE(al, &error);
  __ bind(&skip_power2);
  __ cmp(r1, Immediate(0x7fffffff));
    B_LINE(ne, &error);

  __ mov(r0, Immediate(1));
  __ JumpIfNotPowerOfTwoOrZero(r0, r1, &not_power2);
  __ jmp(&skip_power3);
  __ bind(&not_power3);
  B_LINE(al, &error);
  __ bind(&skip_power3);
  __ cmp(r1, Immediate(0));
    B_LINE(ne, &error);

  __ mov(r0, Immediate(0x30));
  __ JumpIfNotPowerOfTwoOrZero(r0, r1, &not_power4);
  B_LINE(al, &error);
  __ bind(&not_power4);

  Label not_pow1, zero_neg1, not_pow2, zero_neg2, not_pow3, zero_neg3,
    not_pow4, zero_neg4, not_pow5, zero_neg5, not_pow6, zero_neg6,
    skip_pow5, skip_pow6, skip_pow2;
  __ mov(r0, Immediate(0));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1, &zero_neg1, &not_pow1);
  B_LINE(al, &error);
  __ bind(&not_pow1);
  B_LINE(al, &error);
  __ bind(&zero_neg1);

  __ mov(r0, Immediate(0x80000000));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1, &zero_neg2, &not_pow2);
  __ jmp(&skip_pow2);
  __ bind(&not_pow2);
  B_LINE(al, &error);
  __ bind(&zero_neg2);
  B_LINE(al, &error);
  __ bind (&skip_pow2);
  __ cmp(r1, Immediate(0x7fffffff));
    B_LINE(ne, &error);

  __ mov(r0, Immediate(-1));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1, &zero_neg3, &not_pow3);
  B_LINE(al, &error);
  __ bind(&not_pow3);
  B_LINE(al, &error);
  __ bind(&zero_neg3);

  __ mov(r0, Immediate(0x30));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1, &zero_neg4, &not_pow4);
  B_LINE(al, &error);
  __ bind(&zero_neg4);
  B_LINE(al, &error);
  __ bind(&not_pow4);

  __ mov(r0, Immediate(1));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1,  &zero_neg5, &not_pow5);
  __ jmp(&skip_pow5);
  __ bind(&not_pow5);
  B_LINE(al, &error);
  __ bind(&zero_neg5);
  B_LINE(al, &error);
  __ bind(&skip_pow5);
  __ cmp(r1, Immediate(0));
    B_LINE(ne, &error);

  __ mov(r0, Immediate(0x40000000));
  __ JumpIfNotPowerOfTwoOrZeroAndNeg(r0, r1,  &zero_neg6, &not_pow6);
  __ jmp(&skip_pow6);
  __ bind(&not_pow6);
  B_LINE(al, &error);
  __ bind(&zero_neg6);
  B_LINE(al, &error);
  __ bind(&skip_pow6);
  __ cmp(r1, Immediate(0x3fffffff));
  B_LINE(ne, &error);

  Label not_both_smi1, skip_not_both_smi1, not_both_smi2, not_both_smi3,
    not_both_smi4;
  CMT("Check JumpIfNotBothSmi(): both Smi");
  __ mov(r0, Immediate(Smi::FromInt(1)));
  __ mov(r1, Immediate(Smi::FromInt(2)));
  __ JumpIfNotBothSmi(r0, r1, &not_both_smi1);
  __ jmp(&skip_not_both_smi1);
  __ bind(&not_both_smi1);
  B_LINE(al, &error);
  __ bind(&skip_not_both_smi1);
  
  CMT("Check JumpIfNotBothSmi(): left Smi");
  __ mov(r0, Immediate(Smi::FromInt(1)));
  __ mov(r1, Immediate(0x41)); // not a smi
  __ JumpIfNotBothSmi(r0, r1, &not_both_smi2);
  B_LINE(al, &error);
  __ bind(&not_both_smi2);

  CMT("Check JumpIfNotBothSmi(): right Smi");
  __ JumpIfNotBothSmi(r1, r0, &not_both_smi3);
  B_LINE(al, &error);
  __ bind(&not_both_smi3);

  CMT("Check JumpIfNotBothSmi(): none Smi");
  __ mov(r0, Immediate(0x33)); // not a smi
  __ JumpIfNotBothSmi(r1, r0, &not_both_smi4);
  B_LINE(al, &error);
  __ bind(&not_both_smi4);
  
  
  Label either_smi1, either_smi2, either_smi3, either_smi4, 
    skip_either_smi4;
  CMT("Check JumpIfEitherSmi(): both Smi");
  __ mov(r0, Immediate(Smi::FromInt(1)));
  __ mov(r1, Immediate(Smi::FromInt(2)));
  __ JumpIfEitherSmi(r0, r1, &either_smi1);
  B_LINE(al, &error);
  __ bind(&either_smi1);
  
  CMT("Check JumpIfEitherSmi(): left Smi");
  __ mov(r0, Immediate(Smi::FromInt(1)));
  __ mov(r1, Immediate(0x41)); // not a smi
  __ JumpIfEitherSmi(r0, r1, &either_smi2);
  B_LINE(al, &error);
  __ bind(&either_smi2);

  CMT("Check JumpIfEitherSmi(): right Smi");
  __ JumpIfEitherSmi(r1, r0, &either_smi3);
  B_LINE(al, &error);
  __ bind(&either_smi3);

  CMT("Check JumpIfEitherSmi(): none Smi");
  __ mov(r0, Immediate(0x33)); // not a smi also
  __ JumpIfEitherSmi(r1, r0, &either_smi4);
  __ jmp(&skip_either_smi4);
  __ bind(&either_smi4);
  B_LINE(al, &error);
  __ bind(&skip_either_smi4);
  
  
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


// Test ConvertToInt32()
TEST(sh4_ma_7) {
  BEGIN();

  Label error;
  PROLOGUE();

  Label not_int32_1, skip_int32_1, not_int32_2, not_int32_3;
  Handle<Object> num;
  CMT("Check ConvertToInt32(4.1) == 4");
  num = assm.isolate()->factory()->NewNumber(4.1, TENURED);
  __ mov(r0, Immediate(4));
  __ mov(r1, Operand(num));
  __ ConvertToInt32(r1, r2, r3, r4, no_reg, &not_int32_1);
  __ cmp(r2, r0);
  B_LINE(ne, &error);
  __ jmp(&skip_int32_1);
  __ bind(&not_int32_1);
  B_LINE(al, &error);
  __ bind(&skip_int32_1);

  CMT("Check ConvertToInt32(nan) == not int32");
  __ mov(r1, Operand(NAN_VALUE()));
  __ ConvertToInt32(r1, r2, r3, r4, no_reg, &not_int32_2);
  B_LINE(al, &error);
  __ bind(&not_int32_2);

  CMT("Check ConvertToInt32(0X80000000) == not int32");
  num = assm.isolate()->factory()->NewNumber((double)0x80000000U, TENURED);
  __ mov(r1, Operand(num));
  __ ConvertToInt32(r1, r2, r3, r4, no_reg, &not_int32_3);
  B_LINE(al, &error);
  __ bind(&not_int32_3);
  
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

// Test CountLeadingZeros
TEST(sh4_ma_8) {
  BEGIN();

  Label error;

  PROLOGUE();

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(0));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(32));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(1));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(31));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(3));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(30));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(5));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(29));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(13));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(28));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(24));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(27));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(41));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(26));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(83));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(25));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(211));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(24));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(467));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(23));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(726));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(22));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(1782));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(21));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(3824));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(20));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(4336));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(19));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(8388607));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(9));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(1082130431));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(1));
  B_LINE(ne, &error);

  __ mov(r0, Immediate(0));
  __ mov(r1, Immediate(3229614079));
  __ CountLeadingZeros(r0, r1, r3);
  __ cmpeq(r0, Immediate(0));
  B_LINE(ne, &error);



  EPILOGUE();
  __ mov(r0, Immediate(0));
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
