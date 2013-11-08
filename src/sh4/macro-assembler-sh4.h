// Copyright 2011-2012 the V8 project authors. All rights reserved.
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

#ifndef V8_SH4_MACRO_ASSEMBLER_SH4_H_
#define V8_SH4_MACRO_ASSEMBLER_SH4_H_

#include "assembler.h"
#include "v8globals.h"

namespace v8 {
namespace internal {

// ----------------------------------------------------------------------------
// Static helper functions

// Generate a MemOperand for loading a field from an object.
static inline MemOperand FieldMemOperand(Register object, int offset) {
  return MemOperand(object, offset - kHeapObjectTag);
}

// Flags used for the AllocateInNewSpace functions.
enum AllocationFlags {
  // No special flags.
  NO_ALLOCATION_FLAGS = 0,
  // Return the pointer to the allocated already tagged as a heap object.
  TAG_OBJECT = 1 << 0,
  // The content of the result register already contains the allocation top in
  // new space.
  RESULT_CONTAINS_TOP = 1 << 1,
  // Specify that the requested size of the space to allocate is specified in
  // words instead of bytes.
  SIZE_IN_WORDS = 1 << 2
};


// Flags used for the ObjectToDoubleVFPRegister function.
enum ObjectToDoubleFlags {
  // No special flags.
  NO_OBJECT_TO_DOUBLE_FLAGS = 0,
  // Object is known to be a non smi.
  OBJECT_NOT_SMI = 1 << 0,
  // Don't load NaNs or infinities, branch to the non number case instead.
  AVOID_NANS_AND_INFINITIES = 1 << 1
};


// MacroAssembler implements a collection of frequently used macros.
class MacroAssembler: public Assembler {
 public:
  // The isolate parameter can be NULL if the macro assembler should
  // not use isolate-dependent functionality. In this case, it's the
  // responsibility of the caller to never invoke such function on the
  // macro assembler.
  MacroAssembler(Isolate* isolate, void* buffer, int size);

  // Load an object from the root table.
  void LoadRoot(Register destination,
                Heap::RootListIndex index);
  // Store an object to the root table.
  void StoreRoot(Register source,
                 Heap::RootListIndex index);

  // ---------------------------------------------------------------------------
  // GC Support

  // For page containing |object| mark region covering |addr| dirty.
  // RecordWriteHelper only works if the object is not in new
  // space.
  void RecordWriteHelper(Register object,
                         Register addr,
                         Register scratch);

  // Check if object is in new space.
  // scratch can be object itself, but it will be clobbered.
  void InNewSpace(Register object,
                  Register scratch,
                  // eq for "in new space?", ne for "not in new space?"
                  Condition cond,
                  Label* branch);

  // For the page containing |object| mark the region covering
  // [object+offset] dirty. The object address must be in the first 8K
  // of an allocated page.  The 'scratch' registers are used in the
  // implementation and all 3 registers are clobbered by the
  // operation, as well as the ip register. RecordWrite updates the
  // write barrier even when storing smis.
  void RecordWrite(Register object,
                   Operand offset,
                   Register scratch0,
                   Register scratch1);

  void RecordWrite(Register object,
                   Register offset,
                   Register scratch0,
                   Register scratch1);
  // For the page containing |object| mark the region covering
  // [address] dirty. The object address must be in the first 8K of an
  // allocated page.  All 3 registers are clobbered by the operation,
  // as well as the ip register. RecordWrite updates the write barrier
  // even when storing smis.
  void RecordWrite(Register object,
                   Register address,
                   Register scratch);

  // Mark the register as dead. Put the pattern 0xFFFFFFDE into the register.
  void Dead(Register dead) {
    // Fit in a single SH4 mov_imm_ instruction
    mov(dead, Operand(static_cast<int>(0xFFFFFFDE)));
  }

  // Mark up to four registers dead at a time.
  void Dead(Register d1, Register d2, Register d3 = no_reg,
            Register d4 = no_reg) {
    Dead(d1);
    Dead(d2);
    if (d3.is_valid()) Dead(d3);
    if (d4.is_valid()) Dead(d4);
  }

  void Push(Handle<Object> handle) { push(Operand(handle), sh4_ip); }

  // Push two registers.  Pushes leftmost register first (to highest address).
  void Push(Register src1, Register src2) {
    ASSERT(!src1.is(src2));
    push(src1);
    push(src2);
  }

  // Push three registers.  Pushes leftmost register first (to highest address).
  void Push(Register src1, Register src2, Register src3) {
    ASSERT(!src1.is(src2));
    ASSERT(!src2.is(src3));
    ASSERT(!src1.is(src3));
    push(src1);
    push(src2);
    push(src3);
  }

  // Push four registers.  Pushes leftmost register first (to highest address).
  void Push(Register src1, Register src2,
            Register src3, Register src4) {
    ASSERT(!src1.is(src2));
    ASSERT(!src2.is(src3));
    ASSERT(!src1.is(src3));
    ASSERT(!src1.is(src4));
    ASSERT(!src2.is(src4));
    ASSERT(!src3.is(src4));
    push(src1);
    push(src2);
    push(src3);
    push(src4);
  }

  // Pop two registers. Pops rightmost register first (from lower address).
  void Pop(Register src1, Register src2) {
    ASSERT(!src1.is(src2));
    pop(src2);
    pop(src1);
  }

  // Pop three registers. Pops rightmost register first (from lower address).
  void Pop(Register src1, Register src2, Register src3) {
    ASSERT(!src1.is(src2));
    ASSERT(!src2.is(src3));
    ASSERT(!src1.is(src3));
    pop(src3);
    pop(src2);
    pop(src1);
  }

  // Pop four registers. Pops rightmost register first (from lower address).
  void Pop(Register src1, Register src2, Register src3, Register src4) {
    ASSERT(!src1.is(src2));
    ASSERT(!src2.is(src3));
    ASSERT(!src1.is(src3));
    ASSERT(!src1.is(src4));
    ASSERT(!src2.is(src4));
    ASSERT(!src3.is(src4));
    pop(src4);
    pop(src3);
    pop(src2);
    pop(src1);
  }

  // Load two consecutive registers with two consecutive memory locations.
  void Ldrd(Register dst1,
            Register dst2,
            const MemOperand& src);

  // Store two consecutive registers to two consecutive memory locations.
  void Strd(Register src1,
            Register src2,
            const MemOperand& dst);

  void IsObjectJSStringType(Register object,
                            Register scratch,
                            Label* fail);

#ifdef ENABLE_DEBUGGER_SUPPORT
  // ---------------------------------------------------------------------------
  // Debugger Support

  void DebugBreak();
#endif

  // ---------------------------------------------------------------------------
  // Support for marking unimplemented code generator function
  // Should be called with UNIMPLEMENTED_BREAK define below.
  void UnimplementedBreak(const char *file, int line);
#ifdef DEBUG
#define UNIMPLEMENTED_BREAK() UnimplementedBreak(__FILE__, __LINE__)
#else
#define UNIMPLEMENTED_BREAK() UnimplementedBreak("", 0)
#endif

  // ---------------------------------------------------------------------------
  // Activation frames

  void EnterInternalFrame() { EnterFrame(StackFrame::INTERNAL); }
  void LeaveInternalFrame() { LeaveFrame(StackFrame::INTERNAL); }

  void EnterConstructFrame() { EnterFrame(StackFrame::CONSTRUCT); }
  void LeaveConstructFrame() { LeaveFrame(StackFrame::CONSTRUCT); }

  // Enter exit frame.
  // stack_space - extra stack space, used for alignment before call to C.
  void EnterExitFrame(bool save_doubles, int stack_space = 0, Register scratch = sh4_ip);

  // Leave the current exit frame. Expects the return value in r0.
  // Expect the number of values, pushed prior to the exit frame, to
  // remove in a register (or no_reg, if there is nothing to remove).
  void LeaveExitFrame(bool save_doubles, Register argument_count);

  // Find the function context up the context chain.
  void LoadContext(Register dst, int context_chain_length);

  // Load the global function with the given index.
  void LoadGlobalFunction(int index, Register function);

  // Load the initial map from the global function. The registers
  // function and map can be the same, function is then overwritten.
  void LoadGlobalFunctionInitialMap(Register function,
                                    Register map,
                                    Register scratch);

  // Push and pop the registers that can hold pointers.
  void PushSafepointRegisters() { UNIMPLEMENTED(); }
  void PopSafepointRegisters() { UNIMPLEMENTED(); }
  // Store the value in register/immediate src in the safepoint
  // register stack slot for register dst.
  void StoreToSafepointRegisterSlot(Register dst, Register src);
  void StoreToSafepointRegisterSlot(Register dst, Operand src);
  void LoadFromSafepointRegisterSlot(Register dst, Register src);

  // ---------------------------------------------------------------------------
  // JavaScript invokes

  // Setup call kind marking in ecx. The method takes ecx as an
  // explicit first parameter to make the code more readable at the
  // call sites.
  void SetCallKind(Register dst, CallKind kind);

  // Invoke the JavaScript function code by either calling or jumping.
  void InvokeCode(Register code,
                  const ParameterCount& expected,
                  const ParameterCount& actual,
                  InvokeFlag flag,
                  const CallWrapper& call_wrapper,
                  CallKind call_kind);

  void InvokeCode(Handle<Code> code,
                  const ParameterCount& expected,
                  const ParameterCount& actual,
                  RelocInfo::Mode rmode,
                  InvokeFlag flag,
                  CallKind call_kind);

  // Invoke the JavaScript function in the given register. Changes the
  // current context to the context in the function before invoking.
  void InvokeFunction(Register function,
                      const ParameterCount& actual,
                      InvokeFlag flag,
                      const CallWrapper& call_wrapper,
                      CallKind call_kind);

  void InvokeFunction(JSFunction* function,
                      const ParameterCount& actual,
                      InvokeFlag flag,
                      CallKind call_kind);

  // Invoke specified builtin JavaScript function. Adds an entry to
  // the unresolved list if the name does not resolve.
  void InvokeBuiltin(Builtins::JavaScript id,
                     InvokeFlag flag,
                     const CallWrapper& call_wrapper = NullCallWrapper());

  // Store the function for the given builtin in the target register.
  void GetBuiltinFunction(Register target, Builtins::JavaScript id);

  // Store the code object for the given builtin in the target register.
  void GetBuiltinEntry(Register target, Builtins::JavaScript id);

  // Expression support
  void Set(Register dst, const Operand& x);
  void Set(const Operand& dst, const Operand& x);

  // Compare object type for heap object.
  // Incoming register is heap_object and outgoing register is map.
  void CmpObjectType(Register heap_object, InstanceType type, Register map);

  // Compare instance type for map.
  void CmpInstanceType(Register map, InstanceType type);

  // Check if the map of an object is equal to a specified map (either
  // given directly or as an index into the root list) and branch to
  // label if not. Skip the smi check if not required (object is known
  // to be a heap object)
  void CheckMap(Register obj,
                Register scratch,
                Handle<Map> map,
                Label* fail,
                bool is_heap_object);

  void CheckMap(Register obj,
                Register scratch,
                Heap::RootListIndex index,
                Label* fail,
                bool is_heap_object);

  // Check if the map of an object is equal to a specified map and branch to a
  // specified target if equal. Skip the smi check if not required (object is
  // known to be a heap object)
  void DispatchMap(Register obj,
                   Register scratch,
                   Handle<Map> map,
                   Handle<Code> success,
                   SmiCheckType smi_check_type);

  // Compare the object in a register to a value from the root list.
  // Uses the ip register as scratch.
  void CompareRoot(Register obj, Heap::RootListIndex index);

  // Check if a heap object's type is in the JSObject range, not including
  // JSFunction.  The object's map will be loaded in the map register.
  // Any or all of the three registers may be the same.
  // The contents of the scratch register will always be overwritten.
  void IsObjectJSObjectType(Register heap_object,
                            Register map,
                            Register scratch,
                            Label* fail);

  // The contents of the scratch register will be overwritten.
  void IsInstanceJSObjectType(Register map, Register scratch, Label* fail);

  // FCmp is similar to integer cmp, but requires unsigned
  // jcc instructions (je, ja, jae, jb, jbe, je, and jz).
  void FCmp();

  // Jump the register contains a smi.
  inline void JumpIfSmi(Register value, Label* smi_label, Label::Distance distance = Label::kFar) {
    tst(value, Operand(kSmiTagMask));
    if (distance == Label::kFar)
      bt(smi_label);
    else
      bt_near(smi_label);
  }
  // Jump if the register contains a non-smi.
  inline void JumpIfNotSmi(Register value, Label* not_smi_label, Label::Distance distance = Label::kFar) {
    tst(value, Operand(kSmiTagMask));
    if (distance == Label::kFar)
      bf(not_smi_label);
    else
      bf_near(not_smi_label);
  }

  // Jump if either of the registers contain a non-smi.
  void JumpIfNotBothSmi(Register reg1, Register reg2, Label* on_not_both_smi,
                        Label::Distance distance = Label::kFar);
  // Jump if either of the registers contain a smi.
  void JumpIfEitherSmi(Register reg1, Register reg2, Label* on_either_smi,
                       Label::Distance distance = Label::kFar);

  // ---------------------------------------------------------------------------
  // String utilities

  // Checks if both objects are sequential ASCII strings and jumps to label
  // if either is not. Assumes that neither object is a smi.
  void JumpIfNonSmisNotBothSequentialAsciiStrings(Register object1,
                                                  Register object2,
                                                  Register scratch1,
                                                  Register scratch2,
                                                  Label* failure);

  // Checks if both objects are sequential ASCII strings and jumps to label
  // if either is not.
  void JumpIfNotBothSequentialAsciiStrings(Register first,
                                           Register second,
                                           Register scratch1,
                                           Register scratch2,
                                           Label* not_flat_ascii_strings);

  // Checks if both instance types are sequential ASCII strings and jumps to
  // label if either is not.
  void JumpIfBothInstanceTypesAreNotSequentialAscii(
      Register first_object_instance_type,
      Register second_object_instance_type,
      Register scratch1,
      Register scratch2,
      Label* failure);

  // Check if instance type is sequential ASCII string and jump to label if
  // it is not.
  void JumpIfInstanceTypeIsNotSequentialAscii(Register type,
                                              Register scratch,
                                              Label* failure);

  // Load the value of a smi object into a FPU double register. The register
  // scratch1 can be the same register as smi in which case smi will hold the
  // untagged value afterwards.
  void SmiToDoubleFPURegister(Register smi,
                              DwVfpRegister value,
                              Register scratch1);


  // Convert the HeapNumber pointed to by source to a 32bits signed integer
  // dest. If the HeapNumber does not fit into a 32bits signed integer branch
  // to not_int32 label. If VFP3 is available double_scratch is used but not
  // scratch2. TODO: SH4, use fp reg.
  void ConvertToInt32(Register source,
                      Register dest,
                      Register scratch,
                      Register scratch2,
                      DwVfpRegister double_scratch,
                      Label *not_int32);

  // Truncates a double using a specific rounding mode.
  // Clears the z flag (ne condition) if an overflow occurs.
  // If exact_conversion is true, the z flag is also cleared if the conversion
  // was inexact, ie. if the double value could not be converted exactly
  // to a 32bit integer.
  void EmitFPUTruncate(FPURoundingMode rounding_mode,
                       Register result,
                       DwVfpRegister double_input,
                       Register scratch,
                       CheckForInexactConversion check
                           = kDontCheckForInexactConversion);

  // Helper for EmitECMATruncate.
  // This will truncate a floating-point value outside of the singed 32bit
  // integer range to a 32bit signed integer.
  // Expects the double value loaded in input_high and input_low.
  // Exits with the answer in 'result'.
  // Note that this code does not work for values in the 32bit range!
  void EmitOutOfInt32RangeTruncate(Register result,
                                   Register input_high,
                                   Register input_low,
                                   Register scratch);

  // Abort execution if argument is not a number. Used in debug code.
  void AbortIfNotNumber(Register object);

  // Abort execution if argument is not a smi. Used in debug code.
  void AbortIfNotSmi(Register object);

  // Abort execution if argument is a smi. Used in debug code.
  void AbortIfSmi(Register object);

  // Abort execution if argument is a string. Used in debug code.
  void AbortIfNotString(Register object);

  // Abort execution if argument is not the root value with the given index.
  void AbortIfNotRootValue(Register src,
                           Heap::RootListIndex root_value_index,
                           const char* message);

  // Prints the value of the register to stdout. Use for debug only.
  void PrintRegisterValue(Register reg);

  // ---------------------------------------------------------------------------
  // Patching helpers.

  // Get the location of a relocated constant (its address in the constant pool)
  // from its load site.
  void GetRelocatedValueLocation(Register ldr_location,
                                 Register result);

  void LoadInstanceDescriptors(Register map, Register descriptors);


  // ---------------------------------------------------------------------------
  // HeapNumber utilities

  void JumpIfNotHeapNumber(Register object,
                           Register heap_number_map,
                           Register scratch,
                           Label* on_not_heap_number);


  // ---------------------------------------------------------------------------
  // Exception handling

  // Push a new try handler and link into try handler chain.  The return
  // address must be pushed before calling this helper.
  void PushTryHandler(CodeLocation try_location, HandlerType type);

  // Unlink the stack handler on top of the stack from the try handler chain.
  void PopTryHandler();

  // Activate the top handler in the try hander chain.
  void Throw(Register value);

  void ThrowUncatchable(UncatchableExceptionType type, Register value);

  // ---------------------------------------------------------------------------
  // Inline caching support

  // Generate code for checking access rights - used for security checks
  // on access to global objects across environments. The holder register
  // is left untouched, but the scratch register is clobbered.
  void CheckAccessGlobalProxy(Register holder_reg,
                              Register scratch,
                              Label* miss);


  void LoadFromNumberDictionary(Label* miss,
                                Register elements,
                                Register key,
                                Register result,
                                Register t0,
                                Register t1,
                                Register t2);

  // ---------------------------------------------------------------------------
  // Allocation support

  // Allocate an object in new space. If the new space is exhausted control
  // continues at the gc_required label. The allocated object is returned in
  // result and end of the new object is returned in result_end. The register
  // scratch can be passed as no_reg in which case an additional object
  // reference will be added to the reloc info. The returned pointers in result
  // and result_end have not yet been tagged as heap objects. If
  // result_contains_top_on_entry is true the content of result is known to be
  // the allocation top on entry (could be result_end from a previous call to
  // AllocateInNewSpace). If result_contains_top_on_entry is true scratch
  // should be no_reg as it is never used.
  void AllocateInNewSpace(int object_size,
                          Register result,
                          Register result_end,
                          Register scratch,
                          Label* gc_required,
                          AllocationFlags flags);

  void AllocateInNewSpace(int header_size,
                          ScaleFactor element_size,
                          Register element_count,
                          Register result,
                          Register result_end,
                          Register scratch,
                          Label* gc_required,
                          AllocationFlags flags);

  void AllocateInNewSpace(Register object_size,
                          Register result,
                          Register result_end,
                          Register scratch,
                          Label* gc_required,
                          AllocationFlags flags);

  // Undo allocation in new space. The object passed and objects allocated after
  // it will no longer be allocated. The caller must make sure that no pointers
  // are left to the object(s) no longer allocated as they would be invalid when
  // allocation is undone.
  void UndoAllocationInNewSpace(Register object, Register scratch);


  // Allocate a heap number in new space with undefined value. The
  // register scratch2 can be passed as no_reg; the others must be
  // valid registers. Returns tagged pointer in result register, or
  // jumps to gc_required if new space is full.
  void AllocateHeapNumber(Register result,
                          Register scratch1,
                          Register scratch2,
                          Register heap_number_map,
                          Label* gc_required);

  // Allocate a sequential string. All the header fields of the string object
  // are initialized.
  void AllocateTwoByteString(Register result,
                             Register length,
                             Register scratch1,
                             Register scratch2,
                             Register scratch3,
                             Label* gc_required);
  void AllocateAsciiString(Register result,
                           Register length,
                           Register scratch1,
                           Register scratch2,
                           Register scratch3,
                           Label* gc_required);
  void AllocateAsciiString(Register result,
                           int length,
                           Register scratch1,
                           Register scratch2,
                           Label* gc_required);

  // Allocate a raw cons string object. Only the map field of the result is
  // initialized.
  void AllocateTwoByteConsString(Register result,
                                 Register length,
                                 Register scratch1,
                                 Register scratch2,
                                 Label* gc_required);

  void AllocateAsciiConsString(Register result,
                               Register length,
                               Register scratch1,
                               Register scratch2,
                               Label* gc_required);
  void AllocateTwoByteSlicedString(Register result,
                                   Register length,
                                   Register scratch1,
                                   Register scratch2,
                                   Label* gc_required);
  void AllocateAsciiSlicedString(Register result,
                                 Register length,
                                 Register scratch1,
                                 Register scratch2,
                                 Label* gc_required);

  // Copies a fixed number of fields of heap objects from src to dst.
  void CopyFields(Register dst, Register src, RegList temps, int field_count);

  // Copy memory, byte-by-byte, from source to destination.  Not optimized for
  // long or aligned copies.
  // The contents of index and scratch are destroyed.
  void CopyBytes(Register source,
                 Register destination,
                 Register length,
                 Register scratch);

  // ---------------------------------------------------------------------------
  // Support functions.

  // Compare object type for heap object.  heap_object contains a non-Smi
  // whose object type should be compared with the given type.  This both
  // sets the flags and leaves the object type in the type_reg register.
  // It leaves the map in the map register (unless the type_reg and map register
  // are the same register).  It leaves the heap object in the heap_object
  // register unless the heap_object register is the same register as one of the
  // other registers.
  void CompareObjectType(Register heap_object,
                         Register map,
                         Register type_reg,
                         InstanceType type,
                         Condition cond);

  // Compare instance type in a map.  map contains a valid map object whose
  // object type should be compared with the given type.  This both
  // sets the flags and leaves the object type in the type_reg register.  It
  // leaves the heap object in the heap_object register unless the heap_object
  // register is the same register as type_reg.
  void CompareInstanceType(Register map,
                           Register type_reg,
                           InstanceType type,
                           Condition cond);

  // Check if a map for a JSObject indicates that the object has fast elements.
  // Jump to the specified label if it does not.
  void CheckFastElements(Register map,
                         Register scratch,
                         Label* fail);

  // Check if result is zero and op is negative.
  void NegativeZeroTest(Register result, Register op, Label* then_label);

  // Check if result is zero and op is negative in code using jump targets.
  void NegativeZeroTest(CodeGenerator* cgen,
                        Register result,
                        Register op);

  // Check if result is zero and any of op1 and op2 are negative.
  // Register scratch is destroyed, and it must be different from op2.
  void NegativeZeroTest(Register result, Register op1, Register op2,
                        Register scratch, Label* then_label);

  // Try to get function prototype of a function and puts the value in
  // the result register. Checks that the function really is a
  // function and jumps to the miss label if the fast checks fail. The
  // function register will be untouched; the other registers may be
  // clobbered.
  void TryGetFunctionPrototype(Register function,
                               Register result,
                               Register scratch,
                               Label* miss);

  // Load and check the instance type of an object for being a string.
  // Loads the type into the second argument register.
  // Returns a condition that will be enabled if the object was a string.
  Condition IsObjectStringType(Register obj,
                               Register type) {
    ldr(type, FieldMemOperand(obj, HeapObject::kMapOffset));
    ldrb(type, FieldMemOperand(type, Map::kInstanceTypeOffset));
    tst(type, Operand(kIsNotStringMask));
    ASSERT_EQ(0, kStringTag);
    return eq;
  }

  // Generates code for reporting that an illegal operation has
  // occurred.
  void IllegalOperation(int num_arguments);

  // Picks out an array index from the hash field.
  // Register use:
  //   hash - holds the index's hash. Clobbered.
  //   index - holds the overwritten index on exit.
  void IndexFromHash(Register hash, Register index);

  void EmitECMATruncate(Register result,
                        DwVfpRegister double_input,
                        SwVfpRegister single_scratch,
                        Register scratch,
                        Register input_high,
                        Register input_low) ;

  // Get the number of least significant bits from a register
  void GetLeastBitsFromSmi(Register dst, Register src, int num_least_bits);
  void GetLeastBitsFromInt32(Register dst, Register src, int mun_least_bits);

  // Count leading zeros in a 32 bit word.  On ARM5 and later it uses the clz
  // instruction.  On pre-ARM5 hardware this routine gives the wrong answer
  // for 0 (31 instead of 32).  Source and scratch can be the same in which case
  // the source is clobbered.  Source and zeros can also be the same in which
  // case scratch should be a different register.
  void CountLeadingZeros(Register zeros,
                         Register source,
                         Register scratch);

  // ---------------------------------------------------------------------------
  // Runtime calls

  // Call a code stub.  Generate the code if necessary.
  void CallStub(CodeStub* stub);

  // Call a code stub and return the code object called.  Try to generate
  // the code if necessary.  Do not perform a GC but instead return a retry
  // after GC failure.
  MUST_USE_RESULT MaybeObject* TryCallStub(CodeStub* stub);

  // Tail call a code stub (jump).  Generate the code if necessary.
  void TailCallStub(CodeStub* stub);

  // Tail call a code stub (jump) and return the code object called.  Try to
  // generate the code if necessary.  Do not perform a GC but instead return
  // a retry after GC failure.
  MUST_USE_RESULT MaybeObject* TryTailCallStub(CodeStub* stub);

  // Return from a code stub after popping its arguments.
  void StubReturn(int argc);

  // Call a runtime routine.
  void CallRuntime(const Runtime::Function* f, int num_arguments);
  void CallRuntimeSaveDoubles(Runtime::FunctionId id);

  // Call a runtime function, returning the CodeStub object called.
  // Try to generate the stub code if necessary.  Do not perform a GC
  // but instead return a retry after GC failure.
  MUST_USE_RESULT MaybeObject* TryCallRuntime(const Runtime::Function* f,
                                              int num_arguments);

  // Convenience function: Same as above, but takes the fid instead.
  void CallRuntime(Runtime::FunctionId id, int num_arguments);

  // Convenience function: Same as above, but takes the fid instead.
  MUST_USE_RESULT MaybeObject* TryCallRuntime(Runtime::FunctionId id,
                                              int num_arguments);

  // Convenience function: call an external reference.
  void CallExternalReference(const ExternalReference& ref, int num_arguments);

  // Tail call of a runtime routine (jump).
  // Like JumpToExternalReference, but also takes care of passing the number
  // of parameters.
  void TailCallExternalReference(const ExternalReference& ext,
                                 int num_arguments,
                                 int result_size);

  // Tail call of a runtime routine (jump). Try to generate the code if
  // necessary. Do not perform a GC but instead return a retry after GC failure.
  MUST_USE_RESULT MaybeObject* TryTailCallExternalReference(
      const ExternalReference& ext, int num_arguments, int result_size);

  // Convenience function: tail call a runtime routine (jump).
  void TailCallRuntime(Runtime::FunctionId fid,
                       int num_arguments,
                       int result_size);

  int CalculateStackPassedWords(int num_reg_arguments,
                                int num_double_arguments);

  // Before calling a C-function from generated code, align arguments on stack.
  // After aligning the frame, non-register arguments must be stored in
  // sp[0], sp[4], etc., not pushed. The argument count assumes all arguments
  // are word sized. If double arguments are used, this function assumes that
  // all double arguments are stored before core registers; otherwise the
  // correct alignment of the double values is not guaranteed.
  // Some compilers/platforms require the stack to be aligned when calling
  // C++ code.
  // Needs a scratch register to do some arithmetic. This register will be
  // trashed.
  void PrepareCallCFunction(int num_reg_arguments,
                            int num_double_arguments,
                            Register scratch);
  void PrepareCallCFunction(int num_reg_arguments,
                            Register scratch) {
    PrepareCallCFunction(num_reg_arguments, 0, scratch);
  }

  // Calls a C function and cleans up the space for arguments allocated
  // by PrepareCallCFunction. The called function is not allowed to trigger a
  // garbage collection, since that might move the code and invalidate the
  // return address (unless this is somehow accounted for by the called
  // function).
  void CallCFunction(ExternalReference function, int num_reg_arguments, int num_double_arguments = 0);
  void CallCFunction(Register function, int num_arguments);

  // Calls an API function. Allocates HandleScope, extracts
  // returned value from handle and propagates exceptions.
  // Clobbers ebx, edi and caller-save registers. Restores context.
  // On return removes stack_space * kPointerSize (GCed).
  MaybeObject* TryCallApiFunctionAndReturn(ExternalReference function,
                                           int stack_space);

  // Jump to a runtime routine.
  void JumpToExternalReference(const ExternalReference& ext);

  MaybeObject* TryJumpToExternalReference(const ExternalReference& ext);


  // ---------------------------------------------------------------------------
  // Utilities

  void Ret(Condition cond = al);

  void Drop(int stack_elements);
  void Ret(int drop);

  void Jump(intptr_t target, RelocInfo::Mode rmode);
  void Jump(Handle<Code> code, RelocInfo::Mode rmode);
  void Call(Label* target) { call(target); }
  void Call(Handle<Code> code, RelocInfo::Mode rmode = RelocInfo::CODE_TARGET, unsigned ast_id = kNoASTId);

  // Emit call to the code we are currently generating.
  void CallSelf() {
    UNIMPLEMENTED();
  }

  void Move(Register dst, Handle<Object> value);
  // Move if the registers are not identical.
  void Move(Register target, Register source);

  void Usat(Register dst, int satpos, Register src);
  void Ubfx(Register dst, Register src, int lsb, int width);
  void Sbfx(Register dst, Register src, int lsb, int width);
  void Bfc(Register dst, int lsb, int width);
  // scratch can be the same register as src (in which case it is trashed), but
  // not the same as dst.
  void Bfi(Register dst,
           Register src,
           Register scratch,
           int lsb,
           int width);

  Handle<Object> CodeObject() {
    ASSERT(!code_object_.is_null());
    return code_object_;
  }

  // Record code generator line mapping through comments.
  // Use -code_comments to enable.
  MacroAssembler* RecordFunctionLine(const char* function, int line);

  // ---------------------------------------------------------------------------
  // StatsCounter support

  void SetCounter(StatsCounter* counter, int value,
                  Register scratch1, Register scratch2);
  void IncrementCounter(StatsCounter* counter, int value,
                        Register scratch1, Register scratch2);
  void DecrementCounter(StatsCounter* counter, int value,
                        Register scratch1, Register scratch2);


  // ---------------------------------------------------------------------------
  // Debugging

  // Calls Abort(msg) if the condition cc is not satisfied.
  // Use --debug_code to enable.
  void Assert(Condition cond, const char* msg);

  void AssertRegisterIsRoot(Register reg, Register scratch,
                            Heap::RootListIndex index);
  void AssertFastElements(Register elements);

  // Like Assert(), but always enabled.
  void Check(Condition cond, const char* msg);

  // Print a message to stdout and abort execution.
  void Abort(const char* msg);

  // Print an object to stdout.
  void DebugPrint(Register obj);

  // Check that the stack is aligned.
  void CheckStackAlignment();

  // Verify restrictions about code generated in stubs.
  void set_generating_stub(bool value) { generating_stub_ = value; }
  bool generating_stub() { return generating_stub_; }
  void set_allow_stub_calls(bool value) { allow_stub_calls_ = value; }
  bool allow_stub_calls() { return allow_stub_calls_; }

  // ---------------------------------------------------------------------------
  // Number utilities

  // Check whether the value of reg is a power of two and not zero. If not
  // control continues at the label not_power_of_two. If reg is a power of two
  // the register scratch contains the value of (reg - 1) when control falls
  // through.
  void JumpIfNotPowerOfTwoOrZero(Register reg,
                                 Register scratch,
                                 Label* not_power_of_two_or_zero);
  // Check whether the value of reg is a power of two and not zero.
  // Control falls through if it is, with scratch containing the mask
  // value (reg - 1).
  // Otherwise control jumps to the 'zero_and_neg' label if the value of reg is
  // zero or negative, or jumps to the 'not_power_of_two' label if the value is
  // strictly positive but not a power of two.
  void JumpIfNotPowerOfTwoOrZeroAndNeg(Register reg,
                                       Register scratch,
                                       Label* zero_and_neg,
                                       Label* not_power_of_two);

  // ---------------------------------------------------------------------------
  // Smi utilities

  void SmiTag(Register reg) {
    add(reg, reg, reg);
  }
  void SmiTag(Register dst, Register src) {
    add(dst, src, src);
  }

  // Try to convert int32 to smi. If the value is to large, preserve
  // the original value and jump to not_a_smi. Destroys scratch and
  // sets flags.
  void TrySmiTag(Register reg, Label* not_a_smi, Register scratch) {
    addv(scratch, reg, reg);
    b(t, not_a_smi);
    mov(reg, scratch);
  }

  void SmiUntag(Register reg) {
    asr(reg, reg, Operand(kSmiTagSize));
  }
  void SmiUntag(Register dst, Register src) {
    asr(dst, src, Operand(kSmiTagSize));
  }


 private:
  void CallCFunctionHelper(Register function,
                           ExternalReference function_reference,
                           Register scratch,
                           int num_reg_arguments,
                           int num_double_arguments);

  bool generating_stub_;
  bool allow_stub_calls_;
  // This handle will be patched with the code object on installation.
  Handle<Object> code_object_;

  // Helper functions for generating invokes.
  void InvokePrologue(const ParameterCount& expected,
                      const ParameterCount& actual,
                      Handle<Code> code_constant,
                      Register code_reg,
                      Label* done,
                      InvokeFlag flag,
                      const CallWrapper& call_wrapper,
                      CallKind call_kind);

  // Activation support.
  void EnterFrame(StackFrame::Type type);
  void LeaveFrame(StackFrame::Type type);

  void LeaveExitFrameEpilogue();

  void InitializeNewString(Register string,
                           Register length,
                           Heap::RootListIndex map_index,
                           Register scratch1,
                           Register scratch2);

  // Allocation support helpers.
  void LoadAllocationTopHelper(Register result,
                               Register scratch,
                               AllocationFlags flags);
  void UpdateAllocationTopHelper(Register result_end, Register scratch);

  // Helper for PopHandleScope.  Allowed to perform a GC and returns
  // NULL if gc_allowed.  Does not perform a GC if !gc_allowed, and
  // possibly returns a failure object indicating an allocation failure.
  MUST_USE_RESULT MaybeObject* PopHandleScopeHelper(Register saved,
                                                    Register scratch,
                                                    bool gc_allowed);


  // Compute memory operands for safepoint stack slots.
  MemOperand SafepointRegisterSlot(Register reg);
  static int SafepointRegisterStackIndex(int reg_code);

  // Needs access to SafepointRegisterStackIndex for optimized frame
  // traversal.
  friend class OptimizedFrame;
};


// The code patcher is used to patch (typically) small parts of code e.g. for
// debugging and other types of instrumentation. When using the code patcher
// the exact number of bytes specified must be emitted. Is not legal to emit
// relocation information. If any of these constraints are violated it causes
// an assertion.
class CodePatcher {
 public:
  CodePatcher(byte* address, int instructions);
  virtual ~CodePatcher();

  // Macro assembler to emit code.
  MacroAssembler* masm() { return &masm_; }

  // Emit the condition part of an instruction leaving the rest of the current
  // instruction unchanged.
  void EmitCondition(Condition cond);

 private:
  byte* address_;  // The address of the code being patched.
  int instructions_;  // Number of instructions of the expected patch size.
  int size_;  // Number of bytes of the expected patch size.
  MacroAssembler masm_;  // Macro assembler used to generate the code.
};


// -----------------------------------------------------------------------------
// Static helper functions.

static MemOperand ContextOperand(Register context, int index) {
  return MemOperand(context, Context::SlotOffset(index));
}


static inline MemOperand GlobalObjectOperand()  {
  return ContextOperand(cp, Context::GLOBAL_INDEX);
}


#ifdef DEBUG
# define ACCESS_MASM(masm) masm->RecordFunctionLine(__FUNCTION__, __LINE__)->
#else
# define ACCESS_MASM(masm) masm->
#endif


} }  // namespace v8::internal

#endif  // V8_SH4_MACRO_ASSEMBLER_SH4_H_
