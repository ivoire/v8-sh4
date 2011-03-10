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

// A light-weight SH4 Assembler.

#ifndef V8_SH4_ASSEMBLER_SH4_H_
#define V8_SH4_ASSEMBLER_SH4_H_

#include "isolate.h"
#include "serialize.h"

namespace v8 {
namespace internal {

// CPU Registers.
//
// 1) We would prefer to use an enum, but enum values are assignment-
// compatible with int, which has caused code-generation bugs.
//
// 2) We would prefer to use a class instead of a struct but we don't like
// the register initialization to depend on the particular initialization
// order (which appears to be different on OS X, Linux, and Windows for the
// installed versions of C++ we tried). Using a struct permits C-style
// "initialization". Also, the Register objects cannot be const as this
// forces initialization stubs in MSVC, making us dependent on initialization
// order.
//
// 3) By not using an enum, we are possibly preventing the compiler from
// doing certain constant folds, which may significantly reduce the
// code generated for some assembly instructions (because they boil down
// to a few constants). If this is a problem, we could change the code
// such that we use an enum in optimized mode, and the struct in debug
// mode. This way we get the compile-time error checking in debug mode
// and best performance in optimized code.
//
struct Register {
  static const int kNumAllocatableRegisters = 6;
  static const int kNumRegisters = 8;

  static inline const char* AllocationIndexToString(int index);

  static inline int ToAllocationIndex(Register reg);

  static inline Register FromAllocationIndex(int index);

  static Register from_code(int code) {
    Register r = { code };
    return r;
  }
  bool is_valid() const { return 0 <= code_ && code_ < kNumRegisters; }
  bool is(Register reg) const { return code_ == reg.code_; }
  // eax, ebx, ecx and edx are byte registers, the rest are not.
  bool is_byte_register() const { return code_ <= 3; }
  int code() const {
    ASSERT(is_valid());
    return code_;
  }
  int bit() const {
    ASSERT(is_valid());
    return 1 << code_;
  }

  // Unfortunately we can't make this private in a struct.
  int code_;
};

const Register no_reg = { -1 };


inline const char* Register::AllocationIndexToString(int index) {
  ASSERT(index >= 0 && index < kNumAllocatableRegisters);
  // This is the mapping of allocation indices to registers.
  UNIMPLEMENTED();
}


inline int Register::ToAllocationIndex(Register reg) {
  UNIMPLEMENTED();
}


inline Register Register::FromAllocationIndex(int index)  {
  UNIMPLEMENTED();
}


struct XMMRegister {
  static const int kNumAllocatableRegisters = 7;
  static const int kNumRegisters = 8;

  static int ToAllocationIndex(XMMRegister reg) {
    UNIMPLEMENTED();
  }

  static XMMRegister FromAllocationIndex(int index) {
    UNIMPLEMENTED();
  }

  static const char* AllocationIndexToString(int index) {
    ASSERT(index >= 0 && index < kNumAllocatableRegisters);
    UNIMPLEMENTED();
  }

  static XMMRegister from_code(int code) {
    UNIMPLEMENTED();
  }

  bool is_valid() const { return 0 <= code_ && code_ < kNumRegisters; }
  bool is(XMMRegister reg) const { return code_ == reg.code_; }
  int code() const {
    ASSERT(is_valid());
    return code_;
  }

  int code_;
};


typedef XMMRegister DoubleRegister;


enum Condition {
  // any value < 0 is considered no_condition
  no_condition  = -1
};


// Returns the equivalent of !cc.
// Negation of the default no_condition (-1) results in a non-default
// no_condition value (-2). As long as tests for no_condition check
// for condition < 0, this will work as expected.
inline Condition NegateCondition(Condition cc) {
  UNIMPLEMENTED();
}


// Corresponds to transposing the operands of a comparison.
inline Condition ReverseCondition(Condition cc) {
  UNIMPLEMENTED();
}


enum Hint {
  no_hint = 0
};


// The result of negating a hint is as if the corresponding condition
// were negated by NegateCondition.  That is, no_hint is mapped to
// itself and not_taken and taken are mapped to each other.
inline Hint NegateHint(Hint hint) {
  UNIMPLEMENTED();
}


// -----------------------------------------------------------------------------
// Machine instruction Immediates

class Immediate BASE_EMBEDDED {
 public:
  inline explicit Immediate(int x);
  inline explicit Immediate(const ExternalReference& ext);
  inline explicit Immediate(Handle<Object> handle);
  inline explicit Immediate(Smi* value);
  inline explicit Immediate(Address addr);

  static Immediate CodeRelativeOffset(Label* label) {
    return Immediate(label);
  }

  bool is_zero() const { return x_ == 0 && rmode_ == RelocInfo::NONE; }
  bool is_int8() const {
    return -128 <= x_ && x_ < 128 && rmode_ == RelocInfo::NONE;
  }
  bool is_int16() const {
    return -32768 <= x_ && x_ < 32768 && rmode_ == RelocInfo::NONE;
  }

 private:
  inline explicit Immediate(Label* value);

  int x_;
  RelocInfo::Mode rmode_;

  friend class Assembler;
};


// -----------------------------------------------------------------------------
// Machine instruction Operands

enum ScaleFactor {
  times_1 = 0,
  times_2 = 1,
  times_4 = 2,
  times_8 = 3,
  times_int_size = times_4,
  times_half_pointer_size = times_2,
  times_pointer_size = times_4,
  times_twice_pointer_size = times_8
};


class Operand BASE_EMBEDDED {
 public:
  // reg
  INLINE(explicit Operand(Register reg));

  // XMM reg
  INLINE(explicit Operand(XMMRegister xmm_reg));

  // [disp/r]
  INLINE(explicit Operand(int32_t disp, RelocInfo::Mode rmode));
  // disp only must always be relocated

  // [base + disp/r]
  explicit Operand(Register base, int32_t disp,
                   RelocInfo::Mode rmode = RelocInfo::NONE);

  // [base + index*scale + disp/r]
  explicit Operand(Register base,
                   Register index,
                   ScaleFactor scale,
                   int32_t disp,
                   RelocInfo::Mode rmode = RelocInfo::NONE);

  // [index*scale + disp/r]
  explicit Operand(Register index,
                   ScaleFactor scale,
                   int32_t disp,
                   RelocInfo::Mode rmode = RelocInfo::NONE);

  static Operand StaticVariable(const ExternalReference& ext) {
    return Operand(reinterpret_cast<int32_t>(ext.address()),
                   RelocInfo::EXTERNAL_REFERENCE);
  }

  static Operand StaticArray(Register index,
                             ScaleFactor scale,
                             const ExternalReference& arr) {
    return Operand(index, scale, reinterpret_cast<int32_t>(arr.address()),
                   RelocInfo::EXTERNAL_REFERENCE);
  }

  static Operand Cell(Handle<JSGlobalPropertyCell> cell) {
    return Operand(reinterpret_cast<int32_t>(cell.location()),
                   RelocInfo::GLOBAL_PROPERTY_CELL);
  }

  // Returns true if this Operand is a wrapper for the specified register.
  bool is_reg(Register reg) const;

 private:
  byte buf_[6];
  // The number of bytes in buf_.
  unsigned int len_;
  // Only valid if len_ > 4.
  RelocInfo::Mode rmode_;

  // Set the ModRM byte without an encoded 'reg' register. The
  // register is encoded later as part of the emit_operand operation.
  inline void set_modrm(int mod, Register rm);

  inline void set_sib(ScaleFactor scale, Register index, Register base);
  inline void set_disp8(int8_t disp);
  inline void set_dispr(int32_t disp, RelocInfo::Mode rmode);

  friend class Assembler;
};


// -----------------------------------------------------------------------------
// A Displacement describes the 32bit immediate field of an instruction which
// may be used together with a Label in order to refer to a yet unknown code
// position. Displacements stored in the instruction stream are used to describe
// the instruction and to chain a list of instructions using the same Label.
// A Displacement contains 2 different fields:
//
// next field: position of next displacement in the chain (0 = end of list)
// type field: instruction type
//
// A next value of null (0) indicates the end of a chain (note that there can
// be no displacement at position zero, because there is always at least one
// instruction byte before the displacement).
//
// Displacement _data field layout
//
// |31.....2|1......0|
// [  next  |  type  |

class Displacement BASE_EMBEDDED {
 public:
  enum Type {
    UNCONDITIONAL_JUMP,
    CODE_RELATIVE,
    OTHER
  };

  int data() const { return data_; }
  Type type() const { return TypeField::decode(data_); }
  void next(Label* L) const {
    int n = NextField::decode(data_);
    n > 0 ? L->link_to(n) : L->Unuse();
  }
  void link_to(Label* L) { init(L, type()); }

  explicit Displacement(int data) { data_ = data; }

  Displacement(Label* L, Type type) { init(L, type); }

  void print() {
    PrintF("%s (%x) ", (type() == UNCONDITIONAL_JUMP ? "jmp" : "[other]"),
                       NextField::decode(data_));
  }

 private:
  int data_;

  class TypeField: public BitField<Type, 0, 2> {};
  class NextField: public BitField<int,  2, 32-2> {};

  void init(Label* L, Type type);
};



// CpuFeatures keeps track of which features are supported by the target CPU.
// Supported features must be enabled by a Scope before use.
// Example:
//   if (CpuFeatures::IsSupported(SSE2)) {
//     CpuFeatures::Scope fscope(SSE2);
//     // Generate SSE2 floating point code.
//   } else {
//     // Generate standard x87 floating point code.
//   }
class CpuFeatures {
 public:
  // Detect features of the target CPU. If the portable flag is set,
  // the method sets safe defaults if the serializer is enabled
  // (snapshots must be portable).
  void Probe(bool portable);
  void Clear() { supported_ = 0; }

  // Check whether a feature is supported by the target CPU.
  bool IsSupported(CpuFeature f) const {
    UNIMPLEMENTED();
  }
  // Check whether a feature is currently enabled.
  bool IsEnabled(CpuFeature f) const {
    UNIMPLEMENTED();
  }
  // Enable a specified feature within a scope.
  class Scope BASE_EMBEDDED {
#ifdef DEBUG
   public:
    explicit Scope(CpuFeature f)
        : cpu_features_(Isolate::Current()->cpu_features()),
          isolate_(Isolate::Current()) {
      UNIMPLEMENTED();
    }
    ~Scope() {
      UNIMPLEMENTED();
    }
   private:
    uint64_t old_enabled_;
    CpuFeatures* cpu_features_;
    Isolate* isolate_;
#else
   public:
    explicit Scope(CpuFeature f) {}
#endif
  };

 private:
  CpuFeatures();

  uint64_t supported_;
  uint64_t enabled_;
  uint64_t found_by_runtime_probing_;

  friend class Isolate;

  DISALLOW_COPY_AND_ASSIGN(CpuFeatures);
};


class Assembler : public AssemblerBase {
 private:
  // We check before assembling an instruction that there is sufficient
  // space to write an instruction and its relocation information.
  // The relocation writer's position must be kGap bytes above the end of
  // the generated instructions. This leaves enough space for the
  // longest possible ia32 instruction, 15 bytes, and the longest possible
  // relocation information encoding, RelocInfoWriter::kMaxLength == 16.
  // (There is a 15 byte limit on ia32 instruction length that rules out some
  // otherwise valid instructions.)
  // This allows for a single, fast space check per instruction.
  static const int kGap = 32;

 public:
  // Create an assembler. Instructions and relocation information are emitted
  // into a buffer, with the instructions starting from the beginning and the
  // relocation information starting from the end of the buffer. See CodeDesc
  // for a detailed comment on the layout (globals.h).
  //
  // If the provided buffer is NULL, the assembler allocates and grows its own
  // buffer, and buffer_size determines the initial buffer size. The buffer is
  // owned by the assembler and deallocated upon destruction of the assembler.
  //
  // If the provided buffer is not NULL, the assembler uses the provided buffer
  // for code generation and assumes its size to be buffer_size. If the buffer
  // is too small, a fatal error occurs. No deallocation of the buffer is done
  // upon destruction of the assembler.
  Assembler(void* buffer, int buffer_size);
  ~Assembler();

  // Overrides the default provided by FLAG_debug_code.
  void set_emit_debug_code(bool value) { emit_debug_code_ = value; }

  // GetCode emits any pending (non-emitted) code and fills the descriptor
  // desc. GetCode() is idempotent; it returns the same result if no other
  // Assembler functions are invoked in between GetCode() calls.
  void GetCode(CodeDesc* desc);

  // Read/Modify the code target in the branch/call instruction at pc.
  inline static Address target_address_at(Address pc);
  inline static void set_target_address_at(Address pc, Address target);

  // This sets the branch destination (which is in the instruction on x86).
  // This is for calls and branches within generated code.
  inline static void set_target_at(Address instruction_payload,
                                   Address target) {
    set_target_address_at(instruction_payload, target);
  }

  // This sets the branch destination (which is in the instruction on x86).
  // This is for calls and branches to runtime code.
  inline static void set_external_target_at(Address instruction_payload,
                                            Address target) {
    set_target_address_at(instruction_payload, target);
  }

  static const int kCallTargetSize = kPointerSize;
  static const int kExternalTargetSize = kPointerSize;

  // Distance between the address of the code target in the call instruction
  // and the return address
  static const int kCallTargetAddressOffset = kPointerSize;
  // Distance between start of patched return sequence and the emitted address
  // to jump to.
  static const int kPatchReturnSequenceAddressOffset = 1;  // JMP imm32.

  // Distance between start of patched debug break slot and the emitted address
  // to jump to.
  static const int kPatchDebugBreakSlotAddressOffset = 1;  // JMP imm32.

  static const int kCallInstructionLength = 5;
  static const int kJSReturnSequenceLength = 6;

  // The debug break slot must be able to contain a call instruction.
  static const int kDebugBreakSlotLength = kCallInstructionLength;

  // One byte opcode for test eax,0xXXXXXXXX.
  static const byte kTestEaxByte = 0xA9;
  // One byte opcode for test al, 0xXX.
  static const byte kTestAlByte = 0xA8;
  // One byte opcode for nop.
  static const byte kNopByte = 0x90;

  // One byte opcode for a short unconditional jump.
  static const byte kJmpShortOpcode = 0xEB;
  // One byte prefix for a short conditional jump.
  static const byte kJccShortPrefix = 0x70;


  // ---------------------------------------------------------------------------
  // Code generation

  // Insert the smallest number of nop instructions
  // possible to align the pc offset to a multiple
  // of m. m must be a power of 2 (>= 4).
  void Align(int m);

  void nop();

  void push(Register src);

  void pop(Register dst);

  void call(Label* L);

  void bind(Label* L);

  void jmp(Label* L);

  // Mark address of the ExitJSFrame code.
  void RecordJSReturn();

  // Use --code-comments to enable, or provide "force = true" flag to always
  // write a comment.
  void RecordComment(const char* msg, bool force = false);

  // Writes a single byte or word of data in the code stream.  Used for
  // inline tables, e.g., jump-tables.
  void db(uint8_t data);
  void dd(uint32_t data);

  int pc_offset() const { return pc_ - buffer_; }

  PositionsRecorder* positions_recorder() { return &positions_recorder_; }


 protected:


 private:
  // record reloc info for current pc_
  void RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data = 0);

  friend class CodePatcher;
  friend class EnsureSpace;

  // Code buffer:
  // The buffer into which code and relocation info are generated.
  byte* buffer_;
  int buffer_size_;
  // True if the assembler owns the buffer, false if buffer is external.
  bool own_buffer_;

  // code generation
  byte* pc_;  // the program counter; moves forward
  RelocInfoWriter reloc_info_writer;

  // push-pop elimination
  byte* last_pc_;

  PositionsRecorder positions_recorder_;

  bool emit_debug_code_;

  friend class PositionsRecorder;
};


// Helper class that ensures that there is enough space for generating
// instructions and relocation information.  The constructor makes
// sure that there is enough space and (in debug mode) the destructor
// checks that we did not generate too much.
class EnsureSpace BASE_EMBEDDED {
 public:
  explicit EnsureSpace(Assembler* assembler) : assembler_(assembler) {
    UNIMPLEMENTED();
  }

#ifdef DEBUG
  ~EnsureSpace() {
    UNIMPLEMENTED();
  }
#endif

 private:
  Assembler* assembler_;
#ifdef DEBUG
  int space_before_;
#endif
};

} }  // namespace v8::internal

#endif  // V8_SH4_ASSEMBLER_SH4_H_
