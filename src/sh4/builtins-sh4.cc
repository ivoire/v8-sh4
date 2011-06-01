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

#if defined(V8_TARGET_ARCH_SH4)

#include "codegen.h"
#include "debug.h"
#include "deoptimizer.h"
#include "full-codegen.h"
#include "runtime.h"

namespace v8 {
namespace internal {


#define __ ACCESS_MASM(masm)


void Builtins::Generate_Adaptor(MacroAssembler* masm,
                                CFunctionId id,
                                BuiltinExtraArguments extra_args) {
  // ----------- S t a t e -------------
  //  -- r0                 : number of arguments excluding receiver
  //  -- r1                 : called function (only guaranteed when
  //                          extra_args requires it)
  //  -- cp                 : context
  //  -- sp[0]              : last argument
  //  -- ...
  //  -- sp[4 * (argc - 1)] : first argument (argc == r0)
  //  -- sp[4 * argc]       : receiver
  // -----------------------------------

  // Insert extra arguments.
  int num_extra_args = 0;
  if (extra_args == NEEDS_CALLED_FUNCTION) {
    num_extra_args = 1;
    __ push(r1);
  } else {
    ASSERT(extra_args == NO_EXTRA_ARGUMENTS);
  }

  // JumpToExternalReference expects r0 to contain the number of arguments
  // including the receiver and the extra arguments.
  __ add(r0, Immediate(num_extra_args + 1));
  __ JumpToExternalReference(ExternalReference(id, masm->isolate()));
}


void Builtins::Generate_ArrayCode(MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


// This constant has the same value as JSArray::kPreallocatedArrayElements and
// if JSArray::kPreallocatedArrayElements is changed handling of loop unfolding
// below should be reconsidered.
static const int kLoopUnfoldLimit = 4;


// Allocate an empty JSArray. The allocated array is put into the result
// register. An elements backing store is allocated with size initial_capacity
// and filled with the hole values.
static void AllocateEmptyJSArray(MacroAssembler* masm,
                                 Register array_function,
                                 Register result,
                                 Register scratch1,
                                 Register scratch2,
                                 Register scratch3,
                                 int initial_capacity,
                                 Label* gc_required) {
  ASSERT(initial_capacity > 0);
  // Load the initial map from the array function.
  __ ldr(scratch1, FieldMemOperand(array_function,
                                   JSFunction::kPrototypeOrInitialMapOffset));

  // Allocate the JSArray object together with space for a fixed array with the
  // requested elements.
  int size = JSArray::kSize + FixedArray::SizeFor(initial_capacity);
  __ AllocateInNewSpace(size,
                        result,
                        scratch2,
                        scratch3,
                        gc_required,
                        TAG_OBJECT);

  // Allocated the JSArray. Now initialize the fields except for the elements
  // array.
  // result: JSObject
  // scratch1: initial map
  // scratch2: start of next object
  __ str(scratch1, FieldMemOperand(result, JSObject::kMapOffset), scratch3);
  __ LoadRoot(scratch1, Heap::kEmptyFixedArrayRootIndex);
  __ str(scratch1, FieldMemOperand(result, JSArray::kPropertiesOffset), scratch3);
  // Field JSArray::kElementsOffset is initialized later.
  __ mov(scratch3,  Immediate(0));
  __ str(scratch3, FieldMemOperand(result, JSArray::kLengthOffset));

  // Calculate the location of the elements array and set elements array member
  // of the JSArray.
  // result: JSObject
  // scratch2: start of next object
  __ add(scratch1, result, Immediate(JSArray::kSize));
  __ str(scratch1, FieldMemOperand(result, JSArray::kElementsOffset), scratch3);

  // Clear the heap tag on the elements array.
  ASSERT(kSmiTag == 0);
  __ sub(scratch1, scratch1, Immediate(kHeapObjectTag));

  // Initialize the FixedArray and fill it with holes. FixedArray length is
  // stored as a smi.
  // result: JSObject
  // scratch1: elements array (untagged)
  // scratch2: start of next object
  __ LoadRoot(scratch3, Heap::kFixedArrayMapRootIndex);
  ASSERT_EQ(0 * kPointerSize, FixedArray::kMapOffset);
  __ str(scratch3, MemOperand(scratch1, kPointerSize)); __ add(scratch3, scratch3, Immediate(kPointerSize));
  __ mov(scratch3,  Immediate(Smi::FromInt(initial_capacity)));
  ASSERT_EQ(1 * kPointerSize, FixedArray::kLengthOffset);
  __ str(scratch3, MemOperand(scratch1, kPointerSize)); __ add(scratch3, scratch3, Immediate(kPointerSize));

  // Fill the FixedArray with the hole value.
  ASSERT_EQ(2 * kPointerSize, FixedArray::kHeaderSize);
  ASSERT(initial_capacity <= kLoopUnfoldLimit);
  __ LoadRoot(scratch3, Heap::kTheHoleValueRootIndex);
  for (int i = 0; i < initial_capacity; i++) {
    __ str(scratch3, MemOperand(scratch1, kPointerSize)); __ add(scratch3, scratch3, Immediate(kPointerSize));
  }
}

// Allocate a JSArray with the number of elements stored in a register. The
// register array_function holds the built-in Array function and the register
// array_size holds the size of the array as a smi. The allocated array is put
// into the result register and beginning and end of the FixedArray elements
// storage is put into registers elements_array_storage and elements_array_end
// (see  below for when that is not the case). If the parameter fill_with_holes
// is true the allocated elements backing store is filled with the hole values
// otherwise it is left uninitialized. When the backing store is filled the
// register elements_array_storage is scratched.
static void AllocateJSArray(MacroAssembler* masm,
                            Register array_function,  // Array function.
                            Register array_size,  // As a smi.
                            Register result,
                            Register elements_array_storage,
                            Register elements_array_end,
                            Register scratch1,
                            Register scratch2,
                            bool fill_with_hole,
                            Label* gc_required) {
  Label not_empty, allocated;

  // Load the initial map from the array function.
  __ ldr(elements_array_storage,
         FieldMemOperand(array_function,
                         JSFunction::kPrototypeOrInitialMapOffset));

  // Check whether an empty sized array is requested.
  __ tst(array_size, array_size);
  __ bf(&not_empty);

  // If an empty array is requested allocate a small elements array anyway. This
  // keeps the code below free of special casing for the empty array.
  int size = JSArray::kSize +
             FixedArray::SizeFor(JSArray::kPreallocatedArrayElements);
  __ AllocateInNewSpace(size,
                        result,
                        elements_array_end,
                        scratch1,
                        gc_required,
                        TAG_OBJECT);
  __ jmp(&allocated);

  // Allocate the JSArray object together with space for a FixedArray with the
  // requested number of elements.
  __ bind(&not_empty);
  ASSERT(kSmiTagSize == 1 && kSmiTag == 0);
  __ mov(elements_array_end,
         Operand((JSArray::kSize + FixedArray::kHeaderSize) / kPointerSize));
  __ asr(scratch1, array_size, Immediate(kSmiTagSize));
  __ add(elements_array_end,
         elements_array_end,
         scratch1);
  __ AllocateInNewSpace(
      elements_array_end,
      result,
      scratch1,
      scratch2,
      gc_required,
      static_cast<AllocationFlags>(TAG_OBJECT | SIZE_IN_WORDS));

  // Allocated the JSArray. Now initialize the fields except for the elements
  // array.
  // result: JSObject
  // elements_array_storage: initial map
  // array_size: size of array (smi)
  __ bind(&allocated);
  __ str(elements_array_storage, FieldMemOperand(result, JSObject::kMapOffset));
  __ LoadRoot(elements_array_storage, Heap::kEmptyFixedArrayRootIndex);
  __ str(elements_array_storage,
         FieldMemOperand(result, JSArray::kPropertiesOffset));
  // Field JSArray::kElementsOffset is initialized later.
  __ str(array_size, FieldMemOperand(result, JSArray::kLengthOffset));

  // Calculate the location of the elements array and set elements array member
  // of the JSArray.
  // result: JSObject
  // array_size: size of array (smi)
  __ add(elements_array_storage, result, Immediate(JSArray::kSize));
  __ str(elements_array_storage,
         FieldMemOperand(result, JSArray::kElementsOffset));

  // Clear the heap tag on the elements array.
  ASSERT(kSmiTag == 0);
  __ sub(elements_array_storage,
         elements_array_storage,
         Immediate(kHeapObjectTag));
  // Initialize the fixed array and fill it with holes. FixedArray length is
  // stored as a smi.
  // result: JSObject
  // elements_array_storage: elements array (untagged)
  // array_size: size of array (smi)
  __ LoadRoot(scratch1, Heap::kFixedArrayMapRootIndex);
  ASSERT_EQ(0 * kPointerSize, FixedArray::kMapOffset);
  __ str(scratch1, MemOperand(elements_array_storage, kPointerSize));
  __ add(elements_array_storage, elements_array_storage, Immediate(kPointerSize));
  ASSERT(kSmiTag == 0);
  Label dst;
  __ tst(array_size, array_size);
  // Length of the FixedArray is the number of pre-allocated elements if
  // the actual JSArray has length 0 and the size of the JSArray for non-empty
  // JSArrays. The length of a FixedArray is stored as a smi.
  __ bf(&dst);
  __ mov(array_size,
         Immediate(Smi::FromInt(JSArray::kPreallocatedArrayElements)));
  __ bind(&dst);
  ASSERT_EQ(1 * kPointerSize, FixedArray::kLengthOffset);
  __ str(array_size,
         MemOperand(elements_array_storage, kPointerSize));
  __ add(elements_array_storage, elements_array_storage, Immediate(kPointerSize));

  // Calculate elements array and elements array end.
  // result: JSObject
  // elements_array_storage: elements array element storage
  // array_size: smi-tagged size of elements array
  ASSERT(kSmiTag == 0 && kSmiTagSize < kPointerSizeLog2);
  __ lsl(scratch1, array_size, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(elements_array_end,
         elements_array_storage,
         scratch1);

  // Fill the allocated FixedArray with the hole value if requested.
  // result: JSObject
  // elements_array_storage: elements array element storage
  // elements_array_end: start of next object
  if (fill_with_hole) {
    Label loop, entry;
    __ LoadRoot(scratch1, Heap::kTheHoleValueRootIndex);
    __ jmp(&entry);
    __ bind(&loop);
    __ str(scratch1,
           MemOperand(elements_array_storage, kPointerSize));
    __ add(elements_array_storage, elements_array_storage, Immediate(kPointerSize));
    __ bind(&entry);
    __ cmpge(elements_array_storage, elements_array_end);
    __ bf(&loop);
  }
}


// Create a new array for the built-in Array function. This function allocates
// the JSArray object and the FixedArray elements array and initializes these.
// If the Array cannot be constructed in native code the runtime is called. This
// function assumes the following state:
//   r0: argc
//   r1: constructor (built-in Array function)
//   lr: return address
//   sp[0]: last argument
// This function is used for both construct and normal calls of Array. The only
// difference between handling a construct call and a normal call is that for a
// construct call the constructor function in r1 needs to be preserved for
// entering the generic code. In both cases argc in r0 needs to be preserved.
// Both registers are preserved by this code so no need to differentiate between
// construct call and normal call.
static void ArrayNativeCode(MacroAssembler* masm,
                            Label* call_generic_code) {
  Counters* counters = masm->isolate()->counters();
  Label argc_one_or_more, argc_two_or_more;

  // Check for array construction with zero arguments or one.
  __ cmpeq(r0, Immediate(0));
  __ bf(&argc_one_or_more);

  // Handle construction of an empty array.
  AllocateEmptyJSArray(masm,
                       r1,
                       r2,
                       r3,
                       r4,
                       r5,
                       JSArray::kPreallocatedArrayElements,
                       call_generic_code);
  __ IncrementCounter(counters->array_function_native(), 1, r3, r4);
  // Setup return value, remove receiver from stack and return.
  __ mov(r0, r2);
  __ add(sp, sp, Immediate(kPointerSize));
  __ rts();

  // Check for one argument. Bail out if argument is not smi or if it is
  // negative.
  __ bind(&argc_one_or_more);
  __ cmpeq(r0, Immediate(1));
  __ bf(&argc_two_or_more);
  ASSERT(kSmiTag == 0);
  __ ldr(r2, MemOperand(sp));  // Get the argument from the stack.
  __ land(r3, r2, Immediate(kIntptrSignBit | kSmiTagMask));
  __ cmpeq(r3, Immediate(0));
  __ bf(call_generic_code);

  // Handle construction of an empty array of a certain size. Bail out if size
  // is too large to actually allocate an elements array.
  ASSERT(kSmiTag == 0);
  __ cmpge(r2, Immediate(JSObject::kInitialMaxFastElementArray << kSmiTagSize));
  __ bt(call_generic_code);

  // r0: argc
  // r1: constructor
  // r2: array_size (smi)
  // sp[0]: argument
  AllocateJSArray(masm,
                  r1,
                  r2,
                  r3,
                  r4,
                  r5,
                  r6,
                  r7,
                  true,
                  call_generic_code);
  __ IncrementCounter(counters->array_function_native(), 1, r2, r4);
  // Setup return value (already done), remove receiver and argument from stack and return.
  __ mov(r0, r3);
  __ add(sp, sp, Immediate(2 * kPointerSize));
  __ rts();

  // Handle construction of an array from a list of arguments.
  __ bind(&argc_two_or_more);
  __ lsl(r2, r0, Immediate(kSmiTagSize));  // Convet argc to a smi.

  // r0: argc
  // r1: constructor
  // r2: array_size (smi)
  // sp[0]: last argument
  AllocateJSArray(masm,
                  r1,
                  r2,
                  r3,
                  r4,
                  r5,
                  r6,
                  r7,
                  false,
                  call_generic_code);
  __ IncrementCounter(counters->array_function_native(), 1, r2, r6);

  // Fill arguments as array elements. Copy from the top of the stack (last
  // element) to the array backing store filling it backwards. Note:
  // elements_array_end points after the backing store therefore PreIndex is
  // used when filling the backing store.
  // r0: argc
  // r3: JSArray
  // r4: elements_array storage start (untagged)
  // r5: elements_array_end (untagged)
  // sp[0]: last argument
  Label loop, entry;
  __ jmp(&entry);
  __ bind(&loop);
  __ ldr(r2, MemOperand(sp, kPointerSize)); __ add(sp, sp, Immediate(kPointerSize));
  __ sub(r5, r5, Immediate(kPointerSize));  __ str(r2, MemOperand(r5, -kPointerSize));
  __ bind(&entry);
  __ cmpge(r4, r5);
  __ bf(&loop);

  // Remove caller arguments and receiver from the stack, setup return value and
  // return.
  // r0: argc
  // r3: JSArray
  // sp[0]: receiver
  __ add(sp, sp, Immediate(kPointerSize));
  __ mov(r0, r3);
  __ rts();
}


void Builtins::Generate_ArrayConstructCode(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0     : number of arguments
  //  -- r1     : constructor function
  //  -- lr     : return address
  //  -- sp[...]: constructor arguments
  // -----------------------------------
  Label generic_constructor;

  if (FLAG_debug_code) {
    // The array construct code is only set for the builtin and internal
    // Array functions which always have a map.
    // Initial map for the builtin Array function should be a map.
    __ ldr(r2, FieldMemOperand(r1, JSFunction::kPrototypeOrInitialMapOffset));
    __ tst(r2, Immediate(kSmiTagMask));
    __ Assert(ne, "Unexpected initial map for Array function");
    __ CompareObjectType(r2, r3, r4, MAP_TYPE);
    __ Assert(eq, "Unexpected initial map for Array function");
  }

  // Run the native code for the Array function called as a constructor.
  ArrayNativeCode(masm, &generic_constructor);

  // Jump to the generic construct code in case the specialized code cannot
  // handle the construction.
  __ bind(&generic_constructor);
  Handle<Code> generic_construct_stub =
      masm->isolate()->builtins()->JSConstructStubGeneric();
  __ Jump(generic_construct_stub, RelocInfo::CODE_TARGET);
}


void Builtins::Generate_StringConstructCode(MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Builtins::Generate_JSConstructCall(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0     : number of arguments
  //  -- r1     : constructor function
  //  -- pr     : return address
  //  -- sp[...]: constructor arguments
  // -----------------------------------
  Label non_function_call;

  // Check that the function is not a smi.
  __ tst(r1, Immediate(kSmiTagMask));
  __ bt(&non_function_call);
  // Check that the function is a JSFunction.
  __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE);
  __ bf(&non_function_call);

  // Jump to the function-specific construct stub.
  __ ldr(r2, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
  __ ldr(r2, FieldMemOperand(r2, SharedFunctionInfo::kConstructStubOffset));
  __ add(r3, r2, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ jsr(r3);   // FIXME: jsr or jmp ???

  // r0: number of arguments
  // r1: called object
  __ bind(&non_function_call);
  // r2: expected number of arguments to zero (not changing r0).
  __ mov(r2, Immediate(0));
  // r3: builtin entry
  __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION_AS_CONSTRUCTOR);
  __ jmp(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
          RelocInfo::CODE_TARGET);
}

static void Generate_JSConstructStubHelper(MacroAssembler* masm,
                                           bool is_api_function,
                                           bool count_constructions) {
  __ UNIMPLEMENTED_BREAK();
}

void Builtins::Generate_JSConstructStubCountdown(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, false, true);
}


void Builtins::Generate_JSConstructStubGeneric(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, false, false);
}


void Builtins::Generate_JSConstructStubApi(MacroAssembler* masm) {
  Generate_JSConstructStubHelper(masm, true, false);
}


static void Generate_JSEntryTrampolineHelper(MacroAssembler* masm,
                                             bool is_construct) {
  // Called from Generate_JS_Entry
  // r0: code entry
  // r1: function
  // r2: receiver
  // r3: argc
  // r4: argv (JSEntryStub does set it)
  // r5-r7, cp may be clobbered

  // Clear the context before we push it when entering the JS frame.
  __ mov(cp, Immediate(0));

  // Enter an internal frame.
  __ EnterInternalFrame();

  // Set up the context from the function argument.
  __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));

  // Set up the roots register.
  ExternalReference roots_address =
      ExternalReference::roots_address(masm->isolate());
  __ mov(roots, Operand(roots_address));

  // Push the function and the receiver onto the stack.
  __ push(r1);
  __ push(r2);

  // Copy arguments to the stack in a loop.
  // r1: function
  // r3: argc
  // r4: argv, i.e. points to first arg
  Label loop, entry;
  __ lsl(r5, r3, Immediate(kPointerSizeLog2));
  __ add(r2, r4, r5);
  // r2 points past last arg.
  __ jmp(&entry);
  __ bind(&loop);
    __ ldr(r0, MemOperand(r4, kPointerSize));     // read next parameter
    __ add(r4, r4, Immediate(kPointerSize));      // mov the r4 pointer onto the next parameter
    __ ldr(r0, MemOperand(r0));  // dereference handle
    __ push(r0);  // push parameter
  __ bind(&entry);
    __ cmpeq(r4, r2);
  __ bf(&loop);

  // Initialize all JavaScript callee-saved registers, since they will be seen
  // by the garbage collector as part of handlers.
  __ LoadRoot(r4, Heap::kUndefinedValueRootIndex);
  __ mov(r5, r4);
  __ mov(r6, r4);
  __ mov(r7, r4);
  __ mov(r8, r4);
  __ mov(r9, r4);

  // Invoke the code and pass argc as r0.
  __ mov(r0, r3);

  // r0: argc
  // r1: function
  if (is_construct) {
    __ Call(masm->isolate()->builtins()->JSConstructCall(),
            RelocInfo::CODE_TARGET);
  } else {
    ParameterCount actual(r0);
    __ InvokeFunction(r1, actual, CALL_FUNCTION);
  }

  // Exit the JS frame and remove the parameters (except function), and return.
  // Respect ABI stack constraint.
  __ LeaveInternalFrame();
  __ rts();

  // r0: result
}


void Builtins::Generate_JSEntryTrampoline(MacroAssembler* masm) {
  Generate_JSEntryTrampolineHelper(masm, false);
}


void Builtins::Generate_JSConstructEntryTrampoline(MacroAssembler* masm) {
  Generate_JSEntryTrampolineHelper(masm, true);
}


static void Generate_LazyCompileHelper(MacroAssembler* masm, Runtime::FunctionId fid) {
  // Enter an internal frame.
  __ EnterInternalFrame();

  // Preserve the function.
  __ push(r1);

  // Push the function on the stack as the argument to the runtime function.
  __ push(r1);
  __ CallRuntime(fid, 1);
  // Calculate the entry point.
  __ add(r2, r0, Immediate(Code::kHeaderSize - kHeapObjectTag));
  // Restore saved function.
  __ pop(r1);

  // Tear down temporary frame.
  __ LeaveInternalFrame();

  // Do a tail-call of the compiled function.
  __ jmp(r2);
}


void Builtins::Generate_LazyCompile(MacroAssembler* masm) {
  Generate_LazyCompileHelper(masm, Runtime::kLazyCompile);
}


void Builtins::Generate_LazyRecompile(MacroAssembler* masm) {
  Generate_LazyCompileHelper(masm, Runtime::kLazyRecompile);
}


static void Generate_NotifyDeoptimizedHelper(MacroAssembler* masm,
                                             Deoptimizer::BailoutType type) {
  __ EnterInternalFrame();
  // Pass the function and deoptimization type to the runtime system.
  __ mov(r0, Immediate(Smi::FromInt(static_cast<int>(type))));
  __ push(r0);
  __ CallRuntime(Runtime::kNotifyDeoptimized, 1);
  __ LeaveInternalFrame();

  // Get the full codegen state from the stack and untag it -> r6.
  __ ldr(r6, MemOperand(sp, 0 * kPointerSize));
  __ SmiUntag(r6);
  // Switch on the state.
  Label with_tos_register, unknown_state;
  __ cmpeq(r6, Immediate(FullCodeGenerator::NO_REGISTERS));
  __ bf(&with_tos_register);
  __ add(sp, sp, Immediate(1 * kPointerSize));  // Remove state.
  __ rts();

  __ bind(&with_tos_register);
  __ ldr(r0, MemOperand(sp, 1 * kPointerSize));
  __ cmpeq(r6, Immediate(FullCodeGenerator::TOS_REG));
  __ bf(&unknown_state);
  __ add(sp, sp, Immediate(2 * kPointerSize));  // Remove state.
  __ rts();

  __ bind(&unknown_state);
  __ stop("no cases left");
}


void Builtins::Generate_NotifyDeoptimized(MacroAssembler* masm) {
  Generate_NotifyDeoptimizedHelper(masm, Deoptimizer::EAGER);
}


void Builtins::Generate_NotifyLazyDeoptimized(MacroAssembler* masm) {
  Generate_NotifyDeoptimizedHelper(masm, Deoptimizer::LAZY);
}


void Builtins::Generate_NotifyOSR(MacroAssembler* masm) {
  // For now, we are relying on the fact that Runtime::NotifyOSR
  // doesn't do any garbage collection which allows us to save/restore
  // the registers without worrying about which of them contain
  // pointers. This seems a bit fragile.
  __ pushm(kJSCallerSaved | kCalleeSaved);
  __ Push(pr, fp);
  __ EnterInternalFrame();
  __ CallRuntime(Runtime::kNotifyOSR, 0);
  __ LeaveInternalFrame();
  __ Pop(pr, fp);
  __ popm(kJSCallerSaved | kCalleeSaved);
  __ rts();
}


void Builtins::Generate_OnStackReplacement(MacroAssembler* masm) {
  __ UNIMPLEMENTED_BREAK();
}


void Builtins::Generate_FunctionCall(MacroAssembler* masm) {
  // 1. Make sure we have at least one argument.
  // r0: actual number of arguments
  { Label done;
    __ tst(r0, r0);
    __ bf(&done);
    __ LoadRoot(r2, Heap::kUndefinedValueRootIndex);
    __ push(r2);
    __ add(r0, r0, Immediate(1));
    __ bind(&done);
  }

  // 2. Get the function to call (passed as receiver) from the stack, check
  //    if it is a function.
  // r0: actual number of arguments
  Label non_function;
  __ lsl(r11, r0, Immediate(kPointerSizeLog2));
  __ mov(r1, MemOperand(sp, r11));
  __ tst(r1, Immediate(kSmiTagMask));
  __ bt(&non_function);
  __ CompareObjectType(r1, r2, r2, JS_FUNCTION_TYPE);
  __ bf(&non_function);

  // 3a. Patch the first argument if necessary when calling a function.
  // r0: actual number of arguments
  // r1: function
  Label shift_arguments;
  { Label convert_to_object, use_global_receiver, patch_receiver;
    // Change context eagerly in case we need the global receiver.
    __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));

    // Do not transform the receiver for strict mode functions.
    __ ldr(r2, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
    __ ldr(r2, FieldMemOperand(r2, SharedFunctionInfo::kCompilerHintsOffset));
    __ tst(r2, Immediate(1 << (SharedFunctionInfo::kStrictModeFunction +
                               kSmiTagSize)));
    __ bf(&shift_arguments);

    // Compute the receiver in non-strict mode.
    __ lsl(r11, r0, Immediate(kPointerSizeLog2));
    __ add(r2, sp, r11);
    __ ldr(r2, MemOperand(r2, -kPointerSize));
    // r0: actual number of arguments
    // r1: function
    // r2: first argument
    __ tst(r2, Immediate(kSmiTagMask));
    __ bt(&convert_to_object);

    __ LoadRoot(r3, Heap::kNullValueRootIndex);
    __ cmpeq(r2, r3);
    __ bt(&use_global_receiver);
    __ LoadRoot(r3, Heap::kUndefinedValueRootIndex);
    __ cmpeq(r2, r3);
    __ bt(&use_global_receiver);

    __ CompareObjectType(r2, r3, r3, FIRST_JS_OBJECT_TYPE, ge);
    __ bf(&convert_to_object);
    __ cmpgt(r3, Immediate(LAST_JS_OBJECT_TYPE));
    __ bf(&shift_arguments);

    __ bind(&convert_to_object);
    __ EnterInternalFrame();  // In order to preserve argument count.
    __ lsl(r0, r0, Immediate(kSmiTagSize)); // Smi-tagged.
    __ push(r0);

    __ push(r2);
    __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
    __ mov(r2, r0);

    __ pop(r0);
    __ asr(r0, r0, Immediate(kSmiTagSize));
    __ LeaveInternalFrame();
    // Restore the function to r1.
    __ lsl(r1, r0, Immediate(kPointerSizeLog2));
    __ ldr(r1, MemOperand(sp, r1));
    __ jmp(&patch_receiver);

    // Use the global receiver object from the called function as the
    // receiver.
    __ bind(&use_global_receiver);
    const int kGlobalIndex =
        Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
    __ ldr(r2, FieldMemOperand(cp, kGlobalIndex));
    __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalContextOffset));
    __ ldr(r2, FieldMemOperand(r2, kGlobalIndex));
    __ ldr(r2, FieldMemOperand(r2, GlobalObject::kGlobalReceiverOffset));

    __ bind(&patch_receiver);
    __ lsl(r3, r0, Immediate(kPointerSizeLog2));
    __ add(r3, sp, r3);
    __ str(r2, MemOperand(r3, -kPointerSize));

    __ jmp(&shift_arguments);
  }

  // 3b. Patch the first argument when calling a non-function.  The
  //     CALL_NON_FUNCTION builtin expects the non-function callee as
  //     receiver, so overwrite the first argument which will ultimately
  //     become the receiver.
  // r0: actual number of arguments
  // r1: function
  __ bind(&non_function);
  __ lsl(r2, r0, Immediate(kPointerSizeLog2));
  __ add(r2, sp, r2);
  __ str(r1, MemOperand(r2, -kPointerSize));
  // Clear r1 to indicate a non-function being called.
  __ mov(r1, Immediate(0));

  // 4. Shift arguments and return address one slot down on the stack
  //    (overwriting the original receiver).  Adjust argument count to make
  //    the original first argument the new receiver.
  // r0: actual number of arguments
  // r1: function
  __ bind(&shift_arguments);
  { Label loop;
    // Calculate the copy start address (destination). Copy end address is sp.
    __ lsl(r2, r0, Immediate(kPointerSizeLog2));
    __ add(r2, sp, r2);

    __ bind(&loop);
    __ ldr(r11, MemOperand(r2, -kPointerSize));
    __ str(r11, MemOperand(r2));
    __ sub(r2, r2, Immediate(kPointerSize));
    __ cmpeq(r2, sp);
    __ bf(&loop);
    // Adjust the actual number of arguments and remove the top element
    // (which is a copy of the last argument).
    __ sub(r0, r0, Immediate(1));
    __ add(sp, sp, Immediate(kPointerSize));    // Pop one pointer without saving it
  }

  // 5a. Call non-function via tail call to CALL_NON_FUNCTION builtin.
  // r0: actual number of arguments
  // r1: function
  { Label function;
    __ tst(r1, r1);
    __ bf(&function);
    // Expected number of arguments is 0 for CALL_NON_FUNCTION.
    __ mov(r2, Immediate(0));
    __ GetBuiltinEntry(r3, Builtins::CALL_NON_FUNCTION);
    __ Call(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
            RelocInfo::CODE_TARGET);
    __ bind(&function);
  }

  // 5b. Get the code to call from the function and check that the number of
  //     expected arguments matches what we're providing.  If so, jump
  //     (tail-call) to the code in register edx without checking arguments.
  // r0: actual number of arguments
  // r1: function
 Label end;
  __ ldr(r3, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
  __ ldr(r2,
         FieldMemOperand(r3, SharedFunctionInfo::kFormalParameterCountOffset));
  __ asr(r2, r2, Immediate(kSmiTagSize));
  __ ldr(r3, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
  __ cmpeq(r2, r0);  // Check formal and actual parameter counts.
  __ bt(&end);
  __ Call(masm->isolate()->builtins()->ArgumentsAdaptorTrampoline(),
          RelocInfo::CODE_TARGET);
  __ bind(&end);
  ParameterCount expected(0);
  __ InvokeCode(r3, expected, expected, JUMP_FUNCTION);
}


void Builtins::Generate_FunctionApply(MacroAssembler* masm) {
  const int kIndexOffset    = -5 * kPointerSize;
  const int kLimitOffset    = -4 * kPointerSize;
  const int kArgsOffset     =  2 * kPointerSize;
  const int kRecvOffset     =  3 * kPointerSize;
  const int kFunctionOffset =  4 * kPointerSize;

  __ EnterInternalFrame();

  __ ldr(r0, MemOperand(fp, kFunctionOffset));  // get the function
  __ push(r0);
  __ ldr(r0, MemOperand(fp, kArgsOffset));  // get the args array
  __ push(r0);
  __ InvokeBuiltin(Builtins::APPLY_PREPARE, CALL_JS);

  // Check the stack for overflow. We are not trying need to catch
  // interruptions (e.g. debug break and preemption) here, so the "real stack
  // limit" is checked.
  Label okay;
  __ LoadRoot(r2, Heap::kRealStackLimitRootIndex);
  // Make r2 the space we have left. The stack might already be overflowed
  // here which will cause r2 to become negative.
  __ sub(r2, sp, r2);
  // Check if the arguments will overflow the stack.
  __ lsl(r11, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ cmpgt(r2, r11);
  __ bt(&okay);  // Signed comparison.

  // Out of stack space.
  __ ldr(r1, MemOperand(fp, kFunctionOffset));
  __ push(r1);
  __ push(r0);
  __ InvokeBuiltin(Builtins::APPLY_OVERFLOW, CALL_JS);
  // End of stack check.

  // Push current limit and index.
  __ bind(&okay);
  __ push(r0);  // limit
  __ mov(r1, Immediate(0));  // initial index
  __ push(r1);

  // Change context eagerly to get the right global object if necessary.
  __ ldr(r0, MemOperand(fp, kFunctionOffset));
  __ ldr(cp, FieldMemOperand(r0, JSFunction::kContextOffset));
  // Load the shared function info while the function is still in r0.
  __ ldr(r1, FieldMemOperand(r0, JSFunction::kSharedFunctionInfoOffset));

  // Compute the receiver.
  Label call_to_object, use_global_receiver, push_receiver;
  __ ldr(r0, MemOperand(fp, kRecvOffset));

  // Do not transform the receiver for strict mode functions.
  __ ldr(r1, FieldMemOperand(r1, SharedFunctionInfo::kCompilerHintsOffset));
  __ tst(r1, Immediate(1 << (SharedFunctionInfo::kStrictModeFunction +
                             kSmiTagSize)));
  __ bf(&push_receiver);

  // Compute the receiver in non-strict mode.
  __ tst(r0, Immediate(kSmiTagMask));
  __ bt(&call_to_object);
  __ LoadRoot(r1, Heap::kNullValueRootIndex);
  __ cmpeq(r0, r1);
  __ bt(&use_global_receiver);
  __ LoadRoot(r1, Heap::kUndefinedValueRootIndex);
  __ cmpeq(r0, r1);
  __ bt(&use_global_receiver);

  // Check if the receiver is already a JavaScript object.
  // r0: receiver
  __ CompareObjectType(r0, r1, r1, FIRST_JS_OBJECT_TYPE, ge);
  __ bf(&call_to_object);
  __ cmpgt(r1, Immediate(LAST_JS_OBJECT_TYPE));
  __ bf(&push_receiver);

  // Convert the receiver to a regular object.
  // r0: receiver
  __ bind(&call_to_object);
  __ push(r0);
  __ InvokeBuiltin(Builtins::TO_OBJECT, CALL_JS);
  __ jmp(&push_receiver);

  // Use the current global receiver object as the receiver.
  __ bind(&use_global_receiver);
  const int kGlobalOffset =
      Context::kHeaderSize + Context::GLOBAL_INDEX * kPointerSize;
  __ ldr(r0, FieldMemOperand(cp, kGlobalOffset));
  __ ldr(r0, FieldMemOperand(r0, GlobalObject::kGlobalContextOffset));
  __ ldr(r0, FieldMemOperand(r0, kGlobalOffset));
  __ ldr(r0, FieldMemOperand(r0, GlobalObject::kGlobalReceiverOffset));

  // Push the receiver.
  // r0: receiver
  __ bind(&push_receiver);
  __ push(r0);

  // Copy all arguments from the array to the stack.
  Label entry, loop;
  __ ldr(r0, MemOperand(fp, kIndexOffset));
  __ jmp(&entry);

  // Load the current argument from the arguments array and push it to the
  // stack.
  // r0: current argument index
  __ bind(&loop);
  __ ldr(r1, MemOperand(fp, kArgsOffset));
  __ push(r1);
  __ push(r0);

  // Call the runtime to access the property in the arguments array.
  __ CallRuntime(Runtime::kGetProperty, 2);
  __ push(r0);

  // Use inline caching to access the arguments.
  __ ldr(r0, MemOperand(fp, kIndexOffset));
  __ add(r0, r0, Immediate(1 << kSmiTagSize));
  __ str(r0, MemOperand(fp, kIndexOffset));

  // Test if the copy loop has finished copying all the elements from the
  // arguments object.
  __ bind(&entry);
  __ ldr(r1, MemOperand(fp, kLimitOffset));
  __ cmpeq(r0, r1);
  __ bf(&loop);

  // Invoke the function.
  ParameterCount actual(r0);
  __ asr(r0, r0, Immediate(kSmiTagSize));
  __ ldr(r1, MemOperand(fp, kFunctionOffset));
  __ InvokeFunction(r1, actual, CALL_FUNCTION);

  // Tear down the internal frame and remove function, receiver and args.
  __ LeaveInternalFrame();
  __ add(sp, sp, Immediate(3 * kPointerSize));
  __ rts();
}


static void EnterArgumentsAdaptorFrame(MacroAssembler* masm) {
  __ lsl(r0, r0, Immediate(kSmiTagSize));
  __ mov(r4, Immediate(Smi::FromInt(StackFrame::ARGUMENTS_ADAPTOR)));
  __ Push(r0, r1, r4, fp);
  __ push(pr);
  __ add(fp, sp, Immediate(3 * kPointerSize));
}


static void LeaveArgumentsAdaptorFrame(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0 : result being passed through
  // -----------------------------------
  // Get the number of arguments passed (as a smi), tear down the frame and
  // then tear down the parameters.
  __ ldr(r1, MemOperand(fp, -3 * kPointerSize));
  __ mov(sp, fp);
  __ Pop(pr, fp);
  __ lsl(r11, r1, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ add(sp, sp, r11);
  __ add(sp, sp, Immediate(kPointerSize));  // adjust for receiver
}


void Builtins::Generate_ArgumentsAdaptorTrampoline(MacroAssembler* masm) {
  // ----------- S t a t e -------------
  //  -- r0 : actual number of arguments
  //  -- r1 : function (passed through to callee)
  //  -- r2 : expected number of arguments
  //  -- r3 : code entry to call
  // -----------------------------------

  Label invoke, dont_adapt_arguments;

  Label enough, too_few;
  __ cmpge(r0, r2);     // actual number of argument lower than expected number
  __ bf(&too_few);
  __ cmpeq(r2, Immediate(SharedFunctionInfo::kDontAdaptArgumentsSentinel));     // FIXME: Immediate or Operand ??
  __ bt(&dont_adapt_arguments);

  {  // Enough parameters: actual >= expected
    __ bind(&enough);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into r0 and copy end address into r2.
    // r0: actual number of arguments as a smi
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    __ lsl(r0, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
    __ add(r0, fp, r0);
    // adjust for return address and receiver
    __ add(r0, r0, Immediate(2 * kPointerSize));
    __ lsl(r2, r2, Immediate(kPointerSizeLog2));
    __ sub(r2, r0, r2);

    // Copy the arguments (including the receiver) to the new stack frame.
    // r0: copy start address
    // r1: function
    // r2: copy end address
    // r3: code entry to call

    Label copy;
    __ bind(&copy);
    __ ldr(r11, MemOperand(r4, 0));
    __ push(r11);
    __ cmpeq(r0, r2);  // Compare before moving to next argument.
    __ sub(r0, r0, Immediate(kPointerSize));
    __ bf(&copy);

    __ jmp(&invoke);
  }

  {  // Too few parameters: Actual < expected
    __ bind(&too_few);
    EnterArgumentsAdaptorFrame(masm);

    // Calculate copy start address into r0 and copy end address is fp.
    // r0: actual number of arguments as a smi
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    __ lsl(r0, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
    __ add(r0, fp, r0);

    // Copy the arguments (including the receiver) to the new stack frame.
    // r0: copy start address
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    Label copy;
    __ bind(&copy);
    // Adjust load for return address and receiver.
    __ mov(r11, MemOperand(r0, 2 * kPointerSize));
    __ push(r11);
    __ cmpeq(r0, fp);  // Compare before moving to next argument.
    __ sub(r0, r0, Immediate(kPointerSize));
    __ bf(&copy);

    // Fill the remaining expected arguments with undefined.
    // r1: function
    // r2: expected number of arguments
    // r3: code entry to call
    __ LoadRoot(r4, Heap::kUndefinedValueRootIndex);
    __ lsl(r2, r2, Immediate(kPointerSizeLog2));
    __ sub(r2, fp, r2);
    __ sub(r2, r2, Immediate(4 * kPointerSize));  // Adjust for frame.

    Label fill;
    __ bind(&fill);
    __ push(r4);        // kept from the previous load
    __ cmpeq(sp, r2);
    __ bf(&fill);
  }

  // Call the entry point.
  __ bind(&invoke);
  __ jsr(r3);
  // Exit frame and return.
  LeaveArgumentsAdaptorFrame(masm);
  __ rts();


  // -------------------------------------------
  // Dont adapt arguments.
  // -------------------------------------------
  __ bind(&dont_adapt_arguments);
  __ jmp(r3);
}

#undef __

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
