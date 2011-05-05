// Copyright 2010 the V8 project authors. All rights reserved.
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
#include "type-info.h"

namespace v8 {
namespace internal {

// Forward declaration.
class CallWrapper;
class PostCallGenerator;

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



// MacroAssembler implements a collection of frequently used macros.
class MacroAssembler: public Assembler {
 public:
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
                  int eq_0_ne_1,  // 0 for "in new space?", 1 for "not in new space?"
                  Label* branch);

  // For the page containing |object| mark the region covering
  // [object+offset] dirty. The object address must be in the first 8K
  // of an allocated page.  The 'scratch' registers are used in the
  // implementation and all 3 registers are clobbered by the
  // operation, as well as the ip register. RecordWrite updates the
  // write barrier even when storing smis.
  void RecordWrite(Register object,
                   int offset,
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


#ifdef ENABLE_DEBUGGER_SUPPORT
  // ---------------------------------------------------------------------------
  // Debugger Support

  void DebugBreak();
#endif

  // ---------------------------------------------------------------------------
  // Activation frames

  void EnterInternalFrame() { EnterFrame(StackFrame::INTERNAL); }
  void LeaveInternalFrame() { LeaveFrame(StackFrame::INTERNAL); }

  void EnterConstructFrame() { EnterFrame(StackFrame::CONSTRUCT); }
  void LeaveConstructFrame() { LeaveFrame(StackFrame::CONSTRUCT); }

  // Enter exit frame.
  // stack_space - extra stack space, used for alignment before call to C.
  void EnterExitFrame(bool save_doubles, int stack_space = 0);

  // Leave the current exit frame. Expects the return value in r0.
  // Expect the number of values, pushed prior to the exit frame, to
  // remove in a register (or no_reg, if there is nothing to remove).
  void LeaveExitFrame(bool save_doubles, Register argument_count);

  // Find the function context up the context chain.
  void LoadContext(Register dst, int context_chain_length);

  // Load the global function with the given index.
  void LoadGlobalFunction(int index, Register function);

  // Load the initial map from the global function. The registers
  // function and map can be the same.
  void LoadGlobalFunctionInitialMap(Register function, Register map);

  // Push and pop the registers that can hold pointers.
  void PushSafepointRegisters() { UNIMPLEMENTED(); }
  void PopSafepointRegisters() { UNIMPLEMENTED(); }
  // Store the value in register/immediate src in the safepoint
  // register stack slot for register dst.
  void StoreToSafepointRegisterSlot(Register dst, Register src);
  void StoreToSafepointRegisterSlot(Register dst, Immediate src);
  void LoadFromSafepointRegisterSlot(Register dst, Register src);

  // ---------------------------------------------------------------------------
  // JavaScript invokes

  // Invoke the JavaScript function code by either calling or jumping.
  void InvokeCode(const Operand& code,
                  const ParameterCount& expected,
                  const ParameterCount& actual,
                  InvokeFlag flag,
                  PostCallGenerator* post_call_generator = NULL);

  void InvokeCode(Handle<Code> code,
                  const ParameterCount& expected,
                  const ParameterCount& actual,
                  RelocInfo::Mode rmode,
                  InvokeFlag flag,
                  PostCallGenerator* post_call_generator = NULL);

  // Invoke the JavaScript function in the given register. Changes the
  // current context to the context in the function before invoking.
  void InvokeFunction(Register function,
                      const ParameterCount& actual,
                      InvokeFlag flag,
                      PostCallGenerator* post_call_generator = NULL);

  void InvokeFunction(JSFunction* function,
                      const ParameterCount& actual,
                      InvokeFlag flag,
                      PostCallGenerator* post_call_generator = NULL);

  // Invoke specified builtin JavaScript function. Adds an entry to
  // the unresolved list if the name does not resolve.
  void InvokeBuiltin(Builtins::JavaScript id,
                     InvokeFlag flag,
                     PostCallGenerator* post_call_generator = NULL);

  // Store the function for the given builtin in the target register.
  void GetBuiltinFunction(Register target, Builtins::JavaScript id);

  // Store the code object for the given builtin in the target register.
  void GetBuiltinEntry(Register target, Builtins::JavaScript id);

  // Expression support
  void Set(Register dst, const Immediate& x);
  void Set(const Operand& dst, const Immediate& x);

  // Compare object type for heap object.
  // Incoming register is heap_object and outgoing register is map.
  void CmpObjectType(Register heap_object, InstanceType type, Register map);

  // Compare instance type for map.
  void CmpInstanceType(Register map, InstanceType type);

  // Check if the map of an object is equal to a specified map and
  // branch to label if not. Skip the smi check if not required
  // (object is known to be a heap object)
  void CheckMap(Register obj,
                Handle<Map> map,
                Label* fail,
                bool is_heap_object);

  // Check if the object in register heap_object is a string. Afterwards the
  // register map contains the object map and the register instance_type
  // contains the instance_type. The registers map and instance_type can be the
  // same in which case it contains the instance type afterwards. Either of the
  // registers map and instance_type can be the same as heap_object.
  Condition IsObjectStringType(Register heap_object,
                               Register map,
                               Register instance_type);

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

  // Smi tagging support.
  void SmiTag(Register reg) {
    UNIMPLEMENTED();
  }
  void SmiUntag(Register reg) {
    UNIMPLEMENTED();
  }

  // Modifies the register even if it does not contain a Smi!
  void SmiUntag(Register reg, TypeInfo info, Label* non_smi) {
    UNIMPLEMENTED();
  }

  // Modifies the register even if it does not contain a Smi!
  void SmiUntag(Register reg, Label* is_smi) {
    UNIMPLEMENTED();
  }

  // Jump the register contains a smi.
  inline void JumpIfSmi(Register value, Label* smi_label) {
    tst(value, Immediate(kSmiTagMask));
    bt(smi_label);
  }
  // Jump if the register contains a non-smi.
  inline void JumpIfNotSmi(Register value, Label* not_smi_label) {
    tst(value, Immediate(kSmiTagMask));
    bf(not_smi_label);
  }

  // Assumes input is a heap object.
  void JumpIfNotNumber(Register reg, TypeInfo info, Label* on_not_number);

  // Assumes input is a heap number.  Jumps on things out of range.  Also jumps
  // on the min negative int32.  Ignores frational parts.
  void ConvertToInt32(Register dst,
                      Register src,      // Can be the same as dst.
                      Register scratch,  // Can be no_reg or dst, but not src.
                      TypeInfo info,
                      Label* on_not_int32);

  // Abort execution if argument is not a number. Used in debug code.
  void AbortIfNotNumber(Register object);

  // Abort execution if argument is not a smi. Used in debug code.
  void AbortIfNotSmi(Register object);

  // Abort execution if argument is a smi. Used in debug code.
  void AbortIfSmi(Register object);

  // Abort execution if argument is a string. Used in debug code.
  void AbortIfNotString(Register object);

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
  // it will no longer be allocated. Make sure that no pointers are left to the
  // object(s) no longer allocated as they would be invalid when allocation is
  // un-done.
  void UndoAllocationInNewSpace(Register object);

  // Allocate a heap number in new space with undefined value. The
  // register scratch2 can be passed as no_reg; the others must be
  // valid registers. Returns tagged pointer in result register, or
  // jumps to gc_required if new space is full.
  void AllocateHeapNumber(Register result,
                          Register scratch1,
                          Register scratch2,
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
  void AllocateConsString(Register result,
                          Register scratch1,
                          Register scratch2,
                          Label* gc_required);
  void AllocateAsciiConsString(Register result,
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

  // Generates code for reporting that an illegal operation has
  // occurred.
  void IllegalOperation(int num_arguments);

  // Picks out an array index from the hash field.
  // Register use:
  //   hash - holds the index's hash. Clobbered.
  //   index - holds the overwritten index on exit.
  void IndexFromHash(Register hash, Register index);

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
  void CallExternalReference(ExternalReference ref, int num_arguments);

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

  // Convenience function: tail call a runtime routine (jump). Try to generate
  // the code if necessary. Do not perform a GC but instead return a retry after
  // GC failure.
  MUST_USE_RESULT MaybeObject* TryTailCallRuntime(Runtime::FunctionId fid,
                                                  int num_arguments,
                                                  int result_size);

  // Before calling a C-function from generated code, align arguments on stack.
  // After aligning the frame, arguments must be stored in esp[0], esp[4],
  // etc., not pushed. The argument count assumes all arguments are word sized.
  // Some compilers/platforms require the stack to be aligned when calling
  // C++ code.
  // Needs a scratch register to do some arithmetic. This register will be
  // trashed.
  void PrepareCallCFunction(int num_arguments, Register scratch);

  // Calls a C function and cleans up the space for arguments allocated
  // by PrepareCallCFunction. The called function is not allowed to trigger a
  // garbage collection, since that might move the code and invalidate the
  // return address (unless this is somehow accounted for by the called
  // function).
  void CallCFunction(ExternalReference function, int num_arguments);
  void CallCFunction(Register function, int num_arguments);

  // Prepares stack to put arguments (aligns and so on). Reserves
  // space for return value if needed (assumes the return value is a handle).
  // Uses callee-saved esi to restore stack state after call. Arguments must be
  // stored in ApiParameterOperand(0), ApiParameterOperand(1) etc. Saves
  // context (esi).
  void PrepareCallApiFunction(int argc, Register scratch);

  // Calls an API function. Allocates HandleScope, extracts
  // returned value from handle and propagates exceptions.
  // Clobbers ebx, edi and caller-save registers. Restores context.
  // On return removes stack_space * kPointerSize (GCed).
  MaybeObject* TryCallApiFunctionAndReturn(ApiFunction* function,
                                           int stack_space);

  // Jump to a runtime routine.
  void JumpToExternalReference(const ExternalReference& ext);

  MaybeObject* TryJumpToExternalReference(const ExternalReference& ext);


  // ---------------------------------------------------------------------------
  // Utilities

  void Ret();

  void Drop(int stack_elements);

  void Call(Label* target) { call(target); }

  // Emit call to the code we are currently generating.
  void CallSelf() {
    UNIMPLEMENTED();
  }

  // Move if the registers are not identical.
  void Move(Register target, Register source);

  void Ubfx(Register dst, Register src, int lsb, int width);
  void Sbfx(Register dst, Register src, int lsb, int width);
  void Bfc(Register dst, int lsb, int width);

  Handle<Object> CodeObject() { return code_object_; }


  // ---------------------------------------------------------------------------
  // StatsCounter support

  void SetCounter(StatsCounter* counter, int value);
  void IncrementCounter(StatsCounter* counter, int value);
  void DecrementCounter(StatsCounter* counter, int value);
  void IncrementCounter(Condition cc, StatsCounter* counter, int value);
  void DecrementCounter(Condition cc, StatsCounter* counter, int value);


  // ---------------------------------------------------------------------------
  // Debugging

  // Calls Abort(msg) if the condition cc is not satisfied.
  // Use --debug_code to enable.
  void Assert(const char* msg);

  void AssertFastElements(Register elements);

  // Like Assert(), but always enabled.
  void Check(const char* msg);

  // Print a message to stdout and abort execution.
  void Abort(const char* msg);

  // Check that the stack is aligned.
  void CheckStackAlignment();

  // Verify restrictions about code generated in stubs.
  void set_generating_stub(bool value) { generating_stub_ = value; }
  bool generating_stub() { return generating_stub_; }
  void set_allow_stub_calls(bool value) { allow_stub_calls_ = value; }
  bool allow_stub_calls() { return allow_stub_calls_; }

  // ---------------------------------------------------------------------------
  // String utilities.

  // Check whether the instance type represents a flat ascii string. Jump to the
  // label if not. If the instance type can be scratched specify same register
  // for both instance type and scratch.
  void JumpIfInstanceTypeIsNotSequentialAscii(Register instance_type,
                                              Register scratch,
                                              Label* on_not_flat_ascii_string);

  // Checks if both objects are sequential ASCII strings, and jumps to label
  // if either is not.
  void JumpIfNotBothSequentialAsciiStrings(Register object1,
                                           Register object2,
                                           Register scratch1,
                                           Register scratch2,
                                           Label* on_not_flat_ascii_strings);

 private:
  void CallCFunctionHelper(Register function,
                           ExternalReference function_reference,
                           Register scratch,
                           int num_arguments);

  bool generating_stub_;
  bool allow_stub_calls_;
  // This handle will be patched with the code object on installation.
  Handle<Object> code_object_;

  // Helper functions for generating invokes.
  void InvokePrologue(const ParameterCount& expected,
                      const ParameterCount& actual,
                      Handle<Code> code_constant,
                      const Operand& code_operand,
                      NearLabel* done,
                      InvokeFlag flag,
                      PostCallGenerator* post_call_generator = NULL);

  // Activation support.
  void EnterFrame(StackFrame::Type type) { UNIMPLEMENTED(); }
  void LeaveFrame(StackFrame::Type type) { UNIMPLEMENTED(); }

  void LeaveExitFrameEpilogue();

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
  Operand SafepointRegisterSlot(Register reg);
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
  CodePatcher(byte* address, int size);
  virtual ~CodePatcher();

  // Macro assembler to emit code.
  MacroAssembler* masm() { return &masm_; }

 private:
  byte* address_;  // The address of the code being patched.
  int size_;  // Number of bytes of the expected patch size.
  MacroAssembler masm_;  // Macro assembler used to generate the code.
};


// Helper class for generating code or data associated with the code
// right after a call instruction. As an example this can be used to
// generate safepoint data after calls for crankshaft.
class PostCallGenerator {
 public:
  PostCallGenerator() { }
  virtual ~PostCallGenerator() { }
  virtual void Generate() = 0;
};


// Generates an Operand for saving parameters after PrepareCallApiFunction.
Operand ApiParameterOperand(int index);


#define ACCESS_MASM(masm) masm->


} }  // namespace v8::internal

#endif  // V8_SH4_MACRO_ASSEMBLER_SH4_H_
