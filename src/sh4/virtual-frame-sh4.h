// Copyright 2009 the V8 project authors. All rights reserved.
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

#ifndef V8_SH4_VIRTUAL_FRAME_SH4_H_
#define V8_SH4_VIRTUAL_FRAME_SH4_H_

#include "codegen.h"
#include "register-allocator.h"
#include "scopes.h"
#include "type-info.h"

namespace v8 {
namespace internal {

// -------------------------------------------------------------------------
// Virtual frames
//
// The virtual frame is an abstraction of the physical stack frame.  It
// encapsulates the parameters, frame-allocated locals, and the expression
// stack.  It supports push/pop operations on the expression stack, as well
// as random access to the expression stack elements, locals, and
// parameters.

class VirtualFrame: public ZoneObject {
 public:
  // A utility class to introduce a scope where the virtual frame is
  // expected to remain spilled.  The constructor spills the code
  // generator's current frame, but no attempt is made to require it
  // to stay spilled.  It is intended as documentation while the code
  // generator is being transformed.
  class SpilledScope BASE_EMBEDDED {
   public:
    SpilledScope() : previous_state_(cgen()->in_spilled_code()) {
      ASSERT(cgen()->has_valid_frame());
      cgen()->frame()->SpillAll();
      cgen()->set_in_spilled_code(true);
    }

    ~SpilledScope() {
      cgen()->set_in_spilled_code(previous_state_);
    }

   private:
    bool previous_state_;

    CodeGenerator* cgen() {
      return CodeGeneratorScope::Current(Isolate::Current());
    }
  };

  // An illegal index into the virtual frame.
  static const int kIllegalIndex = -1;

  // Construct an initial virtual frame on entry to a JS function.
  inline VirtualFrame();

  // Construct a virtual frame as a clone of an existing one.
  explicit inline VirtualFrame(VirtualFrame* original);

  // Push a copy of a frame slot (typically a local or parameter) on top of
  // the frame.
  inline void PushFrameSlotAt(int index);

  inline void Push(Register reg, TypeInfo info = TypeInfo::Unknown());
  inline void Push(Smi* value);

  inline bool ConstantPoolOverflowed();

  inline bool Equals(VirtualFrame* other);

  inline void SetTypeForLocalAt(int index, TypeInfo info);

  // Spill all values from the frame to memory.
  inline void SpillAll();


  // Prepare for returning from the frame by spilling locals.  This
  // avoids generating unnecessary merge code when jumping to the
  // shared return site.  Emits code for spills.
  inline void PrepareForReturn();

  // Update the type information of a variable frame element directly.
  inline void SetTypeForParamAt(int index, TypeInfo info);

  // Nip removes zero or more elements from immediately below the top
  // of the frame, leaving the previous top-of-frame value on top of
  // the frame.  Nip(k) is equivalent to x = Pop(), Drop(k), Push(x).
  inline void Nip(int num_dropped);

  inline int register_location(Register reg);
  inline void set_register_location(Register reg, int index);

  inline bool is_used(Register reg);

  // Set a frame element to a constant.  The index is frame-top relative.
  inline void SetElementAt(int index, Handle<Object> value);

  // Call stub given the number of arguments it expects on (and
  // removes from) the stack.
  inline Result CallStub(CodeStub* stub, int arg_count);

  // The number of frame-allocated locals and parameters respectively.
  inline int parameter_count();

  inline int local_count();

  // Random-access store to a frame-top relative frame element.  The result
  // becomes owned by the frame and is invalidated.
  void SetElementAt(int index, Result* value);

  // Pop an element from the top of the expression stack.  Returns a
  // Result, which may be a constant or a register.
  Result Pop();

  // Push an element on the virtual frame.
  void Push(Handle<Object> value);

  // Pushing an expression expects that the expression is trivial (according
  // to Expression::IsTrivial).
  void Push(Expression* expr);

  // Invalidates a frame slot (puts an invalid frame element in it).
  // Copies on the frame are correctly handled, and if this slot was
  // the backing store of copies, the index of the new backing store
  // is returned.  Otherwise, returns kIllegalIndex.
  // Register counts are correctly updated.
  int InvalidateFrameSlotAt(int index);

  // Sync a single unsynced element that lies beneath or at the stack pointer.
  void SyncElementBelowStackPointer(int index);

  // Sync a single unsynced element that lies just above the stack pointer.
  void SyncElementByPushing(int index);

  // Sync the range of elements in [begin, end] with memory.
  void SyncRange(int begin, int end);

  CodeGenerator* cgen() {
    return CodeGeneratorScope::Current(Isolate::Current());
  }

  MacroAssembler* masm() { return cgen()->masm(); }

  // Create a duplicate of an existing valid frame element.
  FrameElement CopyElementAt(int index,
    TypeInfo info = TypeInfo::Uninitialized());

  // The number of elements on the virtual frame.
  int element_count() { return elements_.length(); }

  // The height of the virtual expression stack.
  int height() { return element_count() - expression_base_index(); }

  int register_location(int num) {
    ASSERT(num >= 0 && num < RegisterAllocator::kNumRegisters);
    return register_locations_[num];
  }

  bool is_used(int num) {
    ASSERT(num >= 0 && num < RegisterAllocator::kNumRegisters);
    return register_locations_[num] != kIllegalIndex;
  }

  // Add extra in-memory elements to the top of the frame to match an actual
  // frame (eg, the frame after an exception handler is pushed).  No code is
  // emitted.
  void Adjust(int count);

  // Forget count elements from the top of the frame all in-memory
  // (including synced) and adjust the stack pointer downward, to
  // match an external frame effect (examples include a call removing
  // its arguments, and exiting a try/catch removing an exception
  // handler).  No code will be emitted.
  void Forget(int count) {
    ASSERT(count >= 0);
    ASSERT(stack_pointer_ == element_count() - 1);
    stack_pointer_ -= count;
    ForgetElements(count);
  }

  // Forget count elements from the top of the frame without adjusting
  // the stack pointer downward.  This is used, for example, before
  // merging frames at break, continue, and return targets.
  void ForgetElements(int count);

  // Spill all occurrences of a specific register from the frame.
  void Spill(Register reg) {
    if (is_used(reg)) SpillElementAt(register_location(reg));
  }

  // Make the two registers distinct and spill them.  Returns the second
  // register.  If the registers were not distinct then it returns the new
  // second register.
  Result MakeDistinctAndSpilled(Result* left, Result* right) {
    UNIMPLEMENTED();
  }

  // Spill all occurrences of an arbitrary register if possible.  Return the
  // register spilled or no_reg if it was not possible to free any register
  // (ie, they all have frame-external references).
  Register SpillAnyRegister();

  // Spill the top element of the frame.
  void SpillTop() { SpillElementAt(element_count() - 1); }

  // Make this frame so that an arbitrary frame of the same height can
  // be merged to it.  Copies and constants are removed from the frame.
  void MakeMergable();

  // Prepare this virtual frame for merging to an expected frame by
  // performing some state changes that do not require generating
  // code.  It is guaranteed that no code will be generated.
  void PrepareMergeTo(VirtualFrame* expected);

  // Make this virtual frame have a state identical to an expected virtual
  // frame.  As a side effect, code may be emitted to make this frame match
  // the expected one.
  void MergeTo(VirtualFrame* expected);

  // Detach a frame from its code generator, perhaps temporarily.  This
  // tells the register allocator that it is free to use frame-internal
  // registers.  Used when the code generator's frame is switched from this
  // one to NULL by an unconditional jump.
  void DetachFromCodeGenerator() {
    RegisterAllocator* cgen_allocator = cgen()->allocator();
    for (int i = 0; i < RegisterAllocator::kNumRegisters; i++) {
      if (is_used(i)) cgen_allocator->Unuse(i);
    }
  }

  // (Re)attach a frame to its code generator.  This informs the register
  // allocator that the frame-internal register references are active again.
  // Used when a code generator's frame is switched from NULL to this one by
  // binding a label.
  void AttachToCodeGenerator() {
    RegisterAllocator* cgen_allocator = cgen()->allocator();
    for (int i = 0; i < RegisterAllocator::kNumRegisters; i++) {
      if (is_used(i)) cgen_allocator->Use(i);
    }
  }

  // Emit code for the physical JS entry and exit frame sequences.  After
  // calling Enter, the virtual frame is ready for use; and after calling
  // Exit it should not be used.  Note that Enter does not allocate space in
  // the physical frame for storing frame-allocated locals.
  void Enter();
  void Exit();

  // Number of local variables after when we use a loop for allocating.
  static const int kLocalVarBound = 10;

  // Drop a number of elements from the top of the expression stack.  May
  // emit code to affect the physical frame.  Does not clobber any registers
  // excepting possibly the stack pointer.
  void Drop(int count);

  // Drop one element.
  void Drop() {
    Drop(1);
  }

  // Pushing a result invalidates it (its contents become owned by the
  // frame).
  void Push(Result* result) {
    // This assert will trigger if you try to push the same value twice.
    ASSERT(result->is_valid());
    if (result->is_register()) {
      Push(result->reg(), result->type_info());
    } else {
      ASSERT(result->is_constant());
      Push(result->handle());
    }
    if (cgen()->in_safe_int32_mode()) {
      ASSERT(result->is_untagged_int32());
      elements_[element_count() - 1].set_untagged_int32(true);
    }
    result->Unuse();
  }

 private:
  static const int kLocal0Offset = JavaScriptFrameConstants::kLocal0Offset;
  static const int kFunctionOffset = JavaScriptFrameConstants::kFunctionOffset;
  static const int kContextOffset = StandardFrameConstants::kContextOffset;

  static const int kHandlerSize = StackHandlerConstants::kSize / kPointerSize;
  static const int kPreallocatedElements = 5 + 8;  // 8 expression stack slots.

  ZoneList<FrameElement> elements_;

  // The index of the element that is at the processor's stack pointer
  // (the esp register).
  int stack_pointer_;

  // The index of the register frame element using each register, or
  // kIllegalIndex if a register is not on the frame.
  int register_locations_[RegisterAllocator::kNumRegisters];

  // The index of the element that is at the processor's frame pointer
  // (the ebp register).  The parameters, receiver, and return address
  // are below the frame pointer.
  int frame_pointer() {
    return parameter_count() + 2;
  }

  // The index of the first parameter.  The receiver lies below the first
  // parameter.
  int param0_index() {
    return 1;
  }

  // The index of the context slot in the frame.  It is immediately
  // above the frame pointer.
  int context_index() {
    return frame_pointer() + 1;
  }

  // The index of the function slot in the frame.  It is above the frame
  // pointer and the context slot.
  int function_index() {
    return frame_pointer() + 2;
  }

  // The index of the first local.  Between the frame pointer and the
  // locals lie the context and the function.
  int local0_index() {
    return frame_pointer() + 3;
  }

  // The index of the base of the expression stack.
  int expression_base_index() {
    return local0_index() + local_count();
  }

  // Convert a frame index into a frame pointer relative offset into the
  // actual stack.
  int fp_relative(int index) {
    ASSERT(index < element_count());
    ASSERT(frame_pointer() < element_count());  // FP is on the frame.
    return (frame_pointer() - index) * kPointerSize;
  }

  // Record an occurrence of a register in the virtual frame.  This has the
  // effect of incrementing the register's external reference count and
  // of updating the index of the register's location in the frame.
  void Use(Register reg, int index) {
    ASSERT(!is_used(reg));
    set_register_location(reg, index);
    cgen()->allocator()->Use(reg);
  }

  // Record that a register reference has been dropped from the frame.  This
  // decrements the register's external reference count and invalidates the
  // index of the register's location in the frame.
  void Unuse(Register reg) {
    ASSERT(is_used(reg));
    set_register_location(reg, kIllegalIndex);
    cgen()->allocator()->Unuse(reg);
  }

  // Spill the element at a particular index---write it to memory if
  // necessary, free any associated register, and forget its value if
  // constant.
  void SpillElementAt(int index);

  // Sync the element at a particular index.  If it is a register or
  // constant that disagrees with the value on the stack, write it to memory.
  // Keep the element type as register or constant, and clear the dirty bit.
  void SyncElementAt(int index);

  // Spill all elements in registers. Spill the top spilled_args elements
  // on the frame.  Sync all other frame elements.
  // Then drop dropped_args elements from the virtual frame, to match
  // the effect of an upcoming call that will drop them from the stack.
  void PrepareForCall(int spilled_args, int dropped_args);

  // Call a code stub that has already been prepared for calling (via
  // PrepareForCall).
  Result RawCallStub(CodeStub* stub);

  // Calls a code object which has already been prepared for calling
  // (via PrepareForCall).
  Result RawCallCodeObject(Handle<Code> code, RelocInfo::Mode rmode);

  // Classes that need raw access to the elements_ array.
  friend class FrameRegisterState;
  friend class JumpTarget;
};

} }  // namespace v8::internal

#endif  // V8_SH4_VIRTUAL_FRAME_SH4_H_
