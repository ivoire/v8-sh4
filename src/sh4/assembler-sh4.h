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
  static const int kNumRegisters = 16;
  static const int kNumAllocatableRegisters = 8;

  static int ToAllocationIndex(Register reg) {
    ASSERT(reg.code() < kNumAllocatableRegisters);
    return reg.code();
  }

  static Register FromAllocationIndex(int index) {
    ASSERT(index >= 0 && index < kNumAllocatableRegisters);
    return from_code(index);
  }

  static const char* AllocationIndexToString(int index) {
    ASSERT(index >= 0 && index < kNumAllocatableRegisters);
    const char* const names[] = {
      "r0",
      "r1",
      "r2",
      "r3",
      "r4",
      "r5",
      "r6",
      "r7",
    };
    return names[index];
  }

  static Register from_code(int code) {
    Register r = { code };
    return r;
  }

  bool is_valid() const { return 0 <= code_ && code_ < kNumRegisters; }
  bool is(Register reg) const { return code_ == reg.code_; }

  int code() const {
    ASSERT(is_valid());
    return code_;
  }
  int bit() const {
    ASSERT(is_valid());
    return 1 << code_;
  }

  void set_code(int code) {
    code_ = code;
    ASSERT(is_valid());
  }

  // Unfortunately we can't make this private in a struct.
  int code_;
};

const Register no_reg = { -1 };
const Register r0 = { 0 };
const Register r1 = { 1 };
const Register r2 = { 2 };
const Register r3 = { 3 };
const Register r4 = { 4 };
const Register r5 = { 5 };
const Register r6 = { 6 };
const Register r7 = { 7 };
const Register r8 = { 8 };
const Register r9 = { 9 };
const Register r10 = { 10 };
const Register r11 = { 11 };
const Register r12 = { 12 };
const Register r13 = { 13 };
const Register r14 = { 14 };
const Register r15 = { 15 };


// Single word VFP register.
struct SwVfpRegister {
  bool is_valid() const { return 0 <= code_ && code_ < 32; }
  bool is(SwVfpRegister reg) const { return code_ == reg.code_; }
  int code() const {
    ASSERT(is_valid());
    return code_;
  }
  int bit() const {
    ASSERT(is_valid());
    return 1 << code_;
  }
  void split_code(int* vm, int* m) const {
    ASSERT(is_valid());
    *m = code_ & 0x1;
    *vm = code_ >> 1;
  }

  int code_;
};


// Double word VFP register.
struct DwVfpRegister {
  // d0 has been excluded from allocation. This is following ia32
  // where xmm0 is excluded. This should be revisited.
  // Currently d0 is used as a scratch register.
  // d1 has also been excluded from allocation to be used as a scratch
  // register as well.
  static const int kNumRegisters = 16;
  static const int kNumAllocatableRegisters = 15;

  static int ToAllocationIndex(DwVfpRegister reg) {
    ASSERT(reg.code() != 0);
    return reg.code() - 1;
  }

  static DwVfpRegister FromAllocationIndex(int index) {
    ASSERT(index >= 0 && index < kNumAllocatableRegisters);
    return from_code(index + 1);
  }

  static const char* AllocationIndexToString(int index) {
    ASSERT(index >= 0 && index < kNumAllocatableRegisters);
    const char* const names[] = {
      "dr1",
      "dr2",
      "dr3",
      "dr4",
      "dr5",
      "dr6",
      "dr7",
      "dr8",
      "dr9",
      "dr10",
      "dr11",
      "dr12",
      "dr13",
      "dr14",
      "dr15"
    };
    return names[index];
  }

  static DwVfpRegister from_code(int code) {
    DwVfpRegister r = { code };
    return r;
  }

  // Supporting dr0 to dr15
  bool is_valid() const { return 0 <= code_ && code_ < 16; }
  bool is(DwVfpRegister reg) const { return code_ == reg.code_; }
  SwVfpRegister low() const {
    SwVfpRegister reg;
    reg.code_ = code_ * 2;

    ASSERT(reg.is_valid());
    return reg;
  }
  SwVfpRegister high() const {
    SwVfpRegister reg;
    reg.code_ = (code_ * 2) + 1;

    ASSERT(reg.is_valid());
    return reg;
  }
  int code() const {
    ASSERT(is_valid());
    return code_;
  }
  int bit() const {
    ASSERT(is_valid());
    return 1 << code_;
  }
  void split_code(int* vm, int* m) const {
    ASSERT(is_valid());
    *m = (code_ & 0x10) >> 4;
    *vm = code_ & 0x0F;
  }

  int code_;
};

typedef DwVfpRegister DoubleRegister;

// Support for the VFP registers fr0 to fr31 (dr0 to dr15).
// Note that "fr(N):fr(N+1)" is the same as "dr(N/2)".
const SwVfpRegister fr0  = {  0 };
const SwVfpRegister fr1  = {  1 };
const SwVfpRegister fr2  = {  2 };
const SwVfpRegister fr3  = {  3 };
const SwVfpRegister fr4  = {  4 };
const SwVfpRegister fr5  = {  5 };
const SwVfpRegister fr6  = {  6 };
const SwVfpRegister fr7  = {  7 };
const SwVfpRegister fr8  = {  8 };
const SwVfpRegister fr9  = {  9 };
const SwVfpRegister fr10 = { 10 };
const SwVfpRegister fr11 = { 11 };
const SwVfpRegister fr12 = { 12 };
const SwVfpRegister fr13 = { 13 };
const SwVfpRegister fr14 = { 14 };
const SwVfpRegister fr15 = { 15 };
const SwVfpRegister fr16 = { 16 };
const SwVfpRegister fr17 = { 17 };
const SwVfpRegister fr18 = { 18 };
const SwVfpRegister fr19 = { 19 };
const SwVfpRegister fr20 = { 20 };
const SwVfpRegister fr21 = { 21 };
const SwVfpRegister fr22 = { 22 };
const SwVfpRegister fr23 = { 23 };
const SwVfpRegister fr24 = { 24 };
const SwVfpRegister fr25 = { 25 };
const SwVfpRegister fr26 = { 26 };
const SwVfpRegister fr27 = { 27 };
const SwVfpRegister fr28 = { 28 };
const SwVfpRegister fr29 = { 29 };
const SwVfpRegister fr30 = { 30 };
const SwVfpRegister fr31 = { 31 };

const DwVfpRegister no_dreg = { -1 };
const DwVfpRegister dr0  = {  0 };
const DwVfpRegister dr1  = {  1 };
const DwVfpRegister dr2  = {  2 };
const DwVfpRegister dr3  = {  3 };
const DwVfpRegister dr4  = {  4 };
const DwVfpRegister dr5  = {  5 };
const DwVfpRegister dr6  = {  6 };
const DwVfpRegister dr7  = {  7 };
const DwVfpRegister dr8  = {  8 };
const DwVfpRegister dr9  = {  9 };
const DwVfpRegister dr10 = { 10 };
const DwVfpRegister dr11 = { 11 };
const DwVfpRegister dr12 = { 12 };
const DwVfpRegister dr13 = { 13 };
const DwVfpRegister dr14 = { 14 };
const DwVfpRegister dr15 = { 15 };


enum Condition {
  // any value < 0 is considered no_condition
  no_condition  = -1,

  eq = 0,       // equal
  ne = 1,       // not equal
  gt = 2,       // greater
  ge = 3,       // greater or equal
  hi = 4,       // unsigned higher
  hs = 5,       // unsigned higher or equal
  lt = 6,       // lesser
  le = 7,       // lesser or equal
  ui = 8,       // unsigned lower
  us = 9,       // unsigned lower or equal
  pl = 10,      // positiv
  pz = 11,      // positiv or null
  ql = 12,      // negativ
  qz = 13       // negativ or null
};


// Returns the equivalent of !cc.
// Negation of the default no_condition (-1) results in a non-default
// no_condition value (-2). As long as tests for no_condition check
// for condition < 0, this will work as expected.
inline Condition NegateCondition(Condition cc) {
  switch(cc) {
    case eq:
      return ne;
    case ne:
      return eq;
    case gt:
      return le;
    case ge:
      return lt;
    case hi:
      return us;
    case hs:
      return ui;
    case lt:
      return ge;
    case le:
      return gt;
    case ui:
      return hs;
    case us:
      return hi;
    case pl:
      return qz;
    case pz:
      return ql;
    case ql:
      return pz;
    case qz:
      return pl;
    default:
      return cc;
  }
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
    explicit Scope(CpuFeature f) {
      UNIMPLEMENTED();
    }
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


typedef uint16_t Instr;

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
  void addc(Register Ry, Register Rx);

  void add_imm(int imm, Register Rx);

  void add(Register Ry, Register Rx);

  void addv(Register Ry, Register Rx);

  void andb_imm_dispR0GBR(int imm);

  void and_imm_R0(int imm);

  void and_(Register Ry, Register Rx);

  void bf(int imm);

  void bfs(int imm);

  void braf(Register Rx);

  void bra(int imm);

  void bsrf(Register Rx);

  void bsr(int imm);

  void bt(int imm);

  void bts(int imm);

  void clrmac();

  void clrs();

  void clrt();

  void cmpeq_imm_R0(int imm);

  void cmpeq(Register Ry, Register Rx);

  void cmpge(Register Ry, Register Rx);

  void cmpgt(Register Ry, Register Rx);

  void cmphi(Register Ry, Register Rx);

  void cmphs(Register Ry, Register Rx);

  void cmppl(Register Rx);

  void cmppz(Register Rx);

  void cmpstr(Register Ry, Register Rx);

  void div0s(Register Ry, Register Rx);

  void div0u();

  void div1(Register Ry, Register Rx);

  void dmulsl(Register Ry, Register Rx);

  void dmulul(Register Ry, Register Rx);

  void dt(Register Rx);

  void extsb(Register Ry, Register Rx);

  void extsw(Register Ry, Register Rx);

  void extub(Register Ry, Register Rx);

  void extuw(Register Ry, Register Rx);

  void fabs_double(Register Rx);

  void fabs(Register Rx);

  void fadd_double(Register Ry, Register Rx);

  void fadd(Register Ry, Register Rx);

  void fcmpeq_double(Register Ry, Register Rx);

  void fcmpeq(Register Ry, Register Rx);

  void fcmpgt_double(Register Ry, Register Rx);

  void fcmpgt(Register Ry, Register Rx);

  void fcnvds_double_FPUL(Register Rx);

  void fcnvsd_FPUL_double(Register Rx);

  void fdiv_double(Register Ry, Register Rx);

  void fdiv(Register Ry, Register Rx);

  void fipr(Register Ry, Register Rx);

  void fldi0(Register Rx);

  void fldi1(Register Rx);

  void flds_FPUL(Register Rx);

  void float_FPUL_double(Register Rx);

  void float_FPUL(Register Rx);

  void fmac(Register Ry, Register Rx);

  void fmovd_dispR0Ry_Xdouble(Register Ry, Register Rx);

  void fmov_decRx(Register Ry, Register Rx);

  void fmovd_incRy_Xdouble(Register Ry, Register Rx);

  void fmovd_indRy_Xdouble(Register Ry, Register Rx);

  void fmov_dispR0Rx(Register Ry, Register Rx);

  void fmov_dispR0Ry(Register Ry, Register Rx);

  void fmov_dispR0Ry_Xdouble(Register Ry, Register Rx);

  void fmovd_Xdouble_decRx(Register Ry, Register Rx);

  void fmovd_Xdouble_dispR0Rx(Register Ry, Register Rx);

  void fmovd_Xdouble_indRx(Register Ry, Register Rx);

  void fmov_incRy(Register Ry, Register Rx);

  void fmov_incRy_Xdouble(Register Ry, Register Rx);

  void fmov_indRx(Register Ry, Register Rx);

  void fmov_indRy(Register Ry, Register Rx);

  void fmov_indRy_Xdouble(Register Ry, Register Rx);

  void fmov(Register Ry, Register Rx);

  void fmovs_decRx(Register Ry, Register Rx);

  void fmovs_dispR0Rx(Register Ry, Register Rx);

  void fmovs_dispR0Ry(Register Ry, Register Rx);

  void fmovs_incRy(Register Ry, Register Rx);

  void fmovs_indRx(Register Ry, Register Rx);

  void fmovs_indRy(Register Ry, Register Rx);

  void fmov_Xdouble_decRx(Register Ry, Register Rx);

  void fmov_Xdouble_dispR0Rx(Register Ry, Register Rx);

  void fmov_Xdouble_indRx(Register Ry, Register Rx);

  void fmov_Xdouble_Xdouble(Register Ry, Register Rx);

  void fmul_double(Register Ry, Register Rx);

  void fmul(Register Ry, Register Rx);

  void fneg_double(Register Rx);

  void fneg(Register Rx);

  void fpchg();

  void frchg();

  void fsca_FPUL_double(Register Rx);

  void fschg();

  void fsqrt_double(Register Rx);

  void fsqrt(Register Rx);

  void fsrra(Register Rx);

  void fsts_FPUL(Register Rx);

  void fsub_double(Register Ry, Register Rx);

  void fsub(Register Ry, Register Rx);

  void ftrc_double_FPUL(Register Rx);

  void ftrc_FPUL(Register Rx);

  void ftrv(Register Rx);

  void icbi_indRx(Register Rx);

  void jmp_indRx(Register Rx);

  void jsr_indRx(Register Rx);

  void ldc_bank(Register Rx, int imm);

  void ldc_DBR(Register Rx);

  void ldc_GBR(Register Rx);

  void ldcl_incRx_bank(Register Rx, int imm);

  void ldcl_incRx_DBR(Register Rx);

  void ldcl_incRx_GBR(Register Rx);

  void ldcl_incRx_SGR(Register Rx);

  void ldcl_incRx_SPC(Register Rx);

  void ldcl_incRx_SR(Register Rx);

  void ldcl_incRx_SSR(Register Rx);

  void ldcl_incRx_VBR(Register Rx);

  void ldc_SGR(Register Rx);

  void ldc_SPC(Register Rx);

  void ldc_SR(Register Rx);

  void ldc_SSR(Register Rx);

  void ldc_VBR(Register Rx);

  void lds_FPSCR(Register Ry);

  void lds_FPUL(Register Ry);

  void ldsl_incRx_MACH(Register Rx);

  void ldsl_incRx_MACL(Register Rx);

  void ldsl_incRx_PR(Register Rx);

  void ldsl_incRy_FPSCR(Register Ry);

  void ldsl_incRy_FPUL(Register Ry);

  void lds_MACH(Register Rx);

  void lds_MACL(Register Rx);

  void lds_PR(Register Rx);

  void ldtlb();

  void macl_incRy_incRx(Register Ry, Register Rx);

  void macw_incRy_incRx(Register Ry, Register Rx);

  void mova_dispPC_R0(int imm);

  void movb_decRx(Register Ry, Register Rx);

  void movb_dispGBR_R0(int imm);

  void movb_dispR0Rx(Register Ry, Register Rx);

  void movb_dispR0Ry(Register Ry, Register Rx);

  void movb_dispRy_R0(int imm, Register Ry);

  void movb_incRy(Register Ry, Register Rx);

  void movb_indRx(Register Ry, Register Rx);

  void movb_indRy(Register Ry, Register Rx);

  void movb_R0_dispGBR(int imm);

  void movb_R0_dispRx(int imm, Register Rx);

  void movcal_R0_indRx(Register Rx);

  void movcol_R0_indRx(Register Rx);

  void mov_imm(int imm, Register Rx);

  void movl_decRx(Register Ry, Register Rx);

  void movl_dispGBR_R0(int imm);

  void movl_dispPC(int imm, Register Rx);

  void movl_dispR0Rx(Register Ry, Register Rx);

  void movl_dispR0Ry(Register Ry, Register Rx);

  void movl_dispRx(Register Ry, int imm, Register Rx);

  void movl_dispRy(int imm, Register Ry, Register Rx);

  void movlil_indRy_R0(Register Ry);

  void movl_incRy(Register Ry, Register Rx);

  void movl_indRx(Register Ry, Register Rx);

  void movl_indRy(Register Ry, Register Rx);

  void movl_R0_dispGBR(int imm);

  void mov(Register Ry, Register Rx);

  void movt(Register Rx);

  void movual_incRy_R0(Register Ry);

  void movual_indRy_R0(Register Ry);

  void movw_decRx(Register Ry, Register Rx);

  void movw_dispGBR_R0(int imm);

  void movw_dispPC(int imm, Register Rx);

  void movw_dispR0Rx(Register Ry, Register Rx);

  void movw_dispR0Ry(Register Ry, Register Rx);

  void movw_dispRy_R0(int imm, Register Ry);

  void movw_incRy(Register Ry, Register Rx);

  void movw_indRx(Register Ry, Register Rx);

  void movw_indRy(Register Ry, Register Rx);

  void movw_R0_dispGBR(int imm);

  void movw_R0_dispRx(int imm, Register Rx);

  void mull(Register Ry, Register Rx);

  void muls(Register Ry, Register Rx);

  void mulsw(Register Ry, Register Rx);

  void mulu(Register Ry, Register Rx);

  void muluw(Register Ry, Register Rx);

  void negc(Register Ry, Register Rx);

  void neg(Register Ry, Register Rx);

  void nop();

  void not_(Register Ry, Register Rx);

  void ocbi_indRx(Register Rx);

  void ocbp_indRx(Register Rx);

  void ocbwb_indRx(Register Rx);

  void orb_imm_dispR0GBR(int imm);

  void or_imm_R0(int imm);

  void or_(Register Ry, Register Rx);

  void prefi_indRx(Register Rx);

  void pref_indRx(Register Rx);

  void rotcl(Register Rx);

  void rotcr(Register Rx);

  void rotl(Register Rx);

  void rotr(Register Rx);

  void rte();

  void rts();

  void sets();

  void sett();

  void shad(Register Ry, Register Rx);

  void shal(Register Rx);

  void shar(Register Rx);

  void shld(Register Ry, Register Rx);

  void shll16(Register Rx);

  void shll2(Register Rx);

  void shll8(Register Rx);

  void shll(Register Rx);

  void shlr16(Register Rx);

  void shlr2(Register Rx);

  void shlr8(Register Rx);

  void shlr(Register Rx);

  void sleep();

  void stc_bank(int imm, Register Rx);

  void stc_DBR(Register Rx);

  void stc_GBR(Register Rx);

  void stcl_bank_decRx(int imm, Register Rx);

  void stcl_DBR_decRx(Register Rx);

  void stcl_GBR_decRx(Register Rx);

  void stcl_SGR_decRx(Register Rx);

  void stcl_SPC_decRx(Register Rx);

  void stcl_SR_decRx(Register Rx);

  void stcl_SSR_decRx(Register Rx);

  void stcl_VBR_decRx(Register Rx);

  void stc_SGR(Register Rx);

  void stc_SPC(Register Rx);

  void stc_SR(Register Rx);

  void stc_SSR(Register Rx);

  void stc_VBR(Register Rx);

  void sts_FPSCR(Register Rx);

  void sts_FPUL(Register Rx);

  void stsl_FPSCR_decRx(Register Rx);

  void stsl_FPUL_decRx(Register Rx);

  void stsl_MACH_decRx(Register Rx);

  void stsl_MACL_decRx(Register Rx);

  void stsl_PR_decRx(Register Rx);

  void sts_MACH(Register Rx);

  void sts_MACL(Register Rx);

  void sts_PR(Register Rx);

  void subc(Register Ry, Register Rx);

  void sub(Register Ry, Register Rx);

  void subv(Register Ry, Register Rx);

  void swapb(Register Ry, Register Rx);

  void swapw(Register Ry, Register Rx);

  void synco();

  void tasb_indRx(Register Rx);

  void trapa_imm(int imm);

  void tstb_imm_dispR0GBR(int imm);

  void tst_imm_R0(int imm);

  void tst(Register Ry, Register Rx);

  void xorb_imm_dispR0GBR(int imm);

  void xor_imm_R0(int imm);

  void xor_(Register Ry, Register Rx);

  void xtrct(Register Ry, Register Rx);


  // Insert the smallest number of nop instructions
  // possible to align the pc offset to a multiple
  // of m. m must be a power of 2 (>= 4).
  void Align(int m);

  void push(Register src);

  void pop(Register dst);

  void call(Label* L);

  void bind(Label* L);

  void jmp(Label* L);

  void emit(Instr x) {}

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
