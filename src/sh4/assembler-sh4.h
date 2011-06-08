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

#include "assembler.h"
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

const Register r0 = { 0 };      // ABI caller saved/return, idem for JIT
const Register r1 = { 1 };      // ABI caller saved/return, idem for JIT
const Register r2 = { 2 };      // ABI caller saved/return, idem for JIT
const Register r3 = { 3 };      // ABI caller saved/return, idem for JIT
const Register r4 = { 4 };      // ABI caller saved/param, idem for JIT
const Register r5 = { 5 };      // ABI caller saved/param, idem for JIT
const Register r6 = { 6 };      // ABI caller saved/param, idem for JIT
const Register r7 = { 7 };      // ABI caller saved/param, idem for JIT
const Register r8 = { 8 };      // ABI callee saved, idem for JIT
const Register r9 = { 9 };      // ABI callee saved, idem for JIT
const Register r10 = { 10 };    // ABI callee saved, idem for JIT
const Register r11 = { 11 };    // ABI callee saved, idem for JIT
const Register roots = { 12 };  // ABI GP, root table pointer for JIT
const Register cp = { 13 };     // ABI callee saved, context pointer for JIT
const Register fp = { 14 };     // ABI FP, idem for JIT
const Register sp = { 15 };     // ABI SP, idem for JIT

const Register pr = { -2 };     // Link register

const Register sh4_r0 = r0;
const Register sh4_r1 = r1;
const Register sh4_r2 = r2;
const Register sh4_r3 = r3;
const Register sh4_r4 = r4;
const Register sh4_r5 = r5;
const Register sh4_r6 = r6;
const Register sh4_r7 = r7;
const Register sh4_r8 = r8;
const Register sh4_r9 = r9;
const Register sh4_r10 = r10;
const Register sh4_r11 = r11;


// Single word VFP register.
struct SwVfpRegister {
  bool is_valid() const { return 0 <= code_ && code_ < 16; }
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
  static const int kNumRegisters = 8;
  static const int kNumAllocatableRegisters = 8;

  static int ToAllocationIndex(DwVfpRegister reg) {
    ASSERT(reg.code() != 0);
    return reg.code();
  }

  static DwVfpRegister FromAllocationIndex(int index) {
    ASSERT(index >= 0 && index < kNumAllocatableRegisters);
    return from_code(index);
  }

  static const char* AllocationIndexToString(int index) {
    ASSERT(index >= 0 && index < kNumAllocatableRegisters);
    const char* const names[] = {
      "dr0",
      "dr2",
      "dr4",
      "dr6",
      "dr8",
      "dr10",
      "dr12",
      "dr14",
    };
    return names[index];
  }

  static DwVfpRegister from_code(int code) {
    DwVfpRegister r = { code };
    return r;
  }

  // Supporting dr0 to dr8
  bool is_valid() const { return 0 <= code_ && code_ < kNumRegisters * 2 - 1; }
  bool is(DwVfpRegister reg) const { return code_ == reg.code_; }
  SwVfpRegister low() const {
    SwVfpRegister reg;
    reg.code_ = code_;

    ASSERT(reg.is_valid());
    return reg;
  }
  SwVfpRegister high() const {
    SwVfpRegister reg;
    reg.code_ = code_ + 1;

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

// Support for the VFP registers fr0 to fr15 (dr0 to dr7).
// Note that "fr(N):fr(N+1)" is the same as "dr(N)".
const SwVfpRegister no_freg = { -1 };
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

const DwVfpRegister no_dreg = { -1 };
const DwVfpRegister dr0   = {  0  };
const DwVfpRegister dr2   = {  2  };
const DwVfpRegister dr4   = {  4  };
const DwVfpRegister dr6   = {  6  };
const DwVfpRegister dr8   = {  8  };
const DwVfpRegister dr10  = {  10 };
const DwVfpRegister dr12  = {  12 };
const DwVfpRegister dr14  = {  14 };

enum Condition {
  // any value < 0 is considered no_condition
  kNoCondition = -1,

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
  qz = 13,      // negativ or null
  al = 14,	// Always
  
  // Aliases
  t = eq,	// cmp eq; if SH4 cmpeq/cmp sets the T bit, t == eq
  f = ne,	// cmp ne: if SH4 cmpeq/cmp clears the T bit, f == ne
  vs = t,	// overflow set: if SH4 addv/subv sets the T bit, vs == t
  vc = f,	// overflow clear: if SH4 addv/subv clears the T bit, vc == f
  cs = t,	// carry set: if SH4 addc/subc sets the T bit, cs == t
  cc = f	// carry clear: if SH4 addc/subc clears the T bit, vc == f

};


// Returns the equivalent of !cc.
// Negation of the default no_condition (-1) results in a non-default
// no_condition value (-2). As long as tests for no_condition check
// for condition < 0, this will work as expected.
inline Condition NegateCondition(Condition cc) {
  switch (cc) {
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
  inline explicit Immediate(int x, RelocInfo::Mode rmode = RelocInfo::NONE);
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
  INLINE(explicit Operand(int32_t immediate, RelocInfo::Mode rmode = RelocInfo::NONE));
  INLINE(explicit Operand(const ExternalReference& f));
  explicit Operand(Handle<Object> handle);

 private:
  Register rx_;
  int32_t imm32_;
  RelocInfo::Mode rmode_;

  friend class Assembler;
};


class MemOperand BASE_EMBEDDED {
 public:
  INLINE(explicit MemOperand(Register Rd, int32_t offset = 0));
  INLINE(explicit MemOperand(Register Rd, Register offset));

  void set_offset(int32_t offset) {
      ASSERT(rm_.is(no_reg));
      offset_ = offset;
  }

  uint32_t offset() const {
      ASSERT(rm_.is(no_reg));
      return offset_;
  }

  Register rn() const { return rn_; }
  Register rm() const { return rm_; }

 private:
  Register rm_;
  Register rn_;
  int32_t offset_;

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
class CpuFeatures : public AllStatic {
 public:
  // Detect features of the target CPU. Set safe defaults if the serializer
  // is enabled (snapshots must be portable).
  static void Probe();

  // Check whether a feature is supported by the target CPU.
  static bool IsSupported(CpuFeature f) {
    UNIMPLEMENTED();
  }

#ifdef DEBUG
  // Check whether a feature is currently enabled.
  static bool IsEnabled(CpuFeature f) {
    UNIMPLEMENTED();
  }
#endif

  // Enable a specified feature within a scope.
  class Scope BASE_EMBEDDED {
#ifdef DEBUG
   public:
    explicit Scope(CpuFeature f) {
      UNIMPLEMENTED();
    }
    ~Scope() {
      UNIMPLEMENTED();
    }
   private:
    Isolate* isolate_;
    unsigned old_enabled_;
#else
   public:
    explicit Scope(CpuFeature f) {
      UNIMPLEMENTED();
    }
#endif
  };

  class TryForceFeatureScope BASE_EMBEDDED {
   public:
    explicit TryForceFeatureScope(CpuFeature f)
        : old_supported_(CpuFeatures::supported_) {
      UNIMPLEMENTED();
    }

    ~TryForceFeatureScope() {
      UNIMPLEMENTED();
    }

   private:
    static bool CanForce() {
      // It's only safe to temporarily force support of CPU features
      // when there's only a single isolate, which is guaranteed when
      // the serializer is enabled.
      return Serializer::enabled();
    }

    const unsigned old_supported_;
  };

 private:
#ifdef DEBUG
  static bool initialized_;
#endif
  static unsigned supported_;
  static unsigned found_by_runtime_probing_;

  DISALLOW_COPY_AND_ASSIGN(CpuFeatures);
};


typedef uint16_t Instr;

class Assembler : public AssemblerBase {
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
  Assembler(Isolate* isolate, void* buffer, int buffer_size);
  ~Assembler();

  // Overrides the default provided by FLAG_debug_code.
  void set_emit_debug_code(bool value) { emit_debug_code_ = value; }

  // GetCode emits any pending (non-emitted) code and fills the descriptor
  // desc. GetCode() is idempotent; it returns the same result if no other
  // Assembler functions are invoked in between GetCode() calls.
  void GetCode(CodeDesc* desc);

  // Label operations & relative jumps (PPUM Appendix D)
  //
  // Takes a branch opcode (cc) and a label (L) and generates
  // either a backward branch or a forward branch and links it
  // to the label fixup chain. Usage:
  //
  // Label L;    // unbound label
  // j(cc, &L);  // forward branch to unbound label
  // bind(&L);   // bind label to the current pc
  // j(cc, &L);  // backward branch to bound label
  // bind(&L);   // illegal: a label may be bound only once
  //
  // Note: The same Label can be used for forward and backward branches
  // but it may be bound only once.

  void bind(Label* L);  // binds an unbound label L to the current code position


  // Return the address in the constant pool of the code target address used by
  // the branch/call instruction at pc.
  INLINE(static Address target_address_address_at(Address pc));

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

  // Size of an instruction.
  static const int kInstrSize = sizeof(Instr);

  // Distance between the instruction referring to the address of the call
  // target and the return address.
  // The call sequence is:
  // mov.l const_pool, rx     @ call sequence start address
  // nop
  // bra skip
  // nop
  // const_pool:
  // .long call_address       
  // skip:
  // jsr rx
  // nop
  // ...                      @ return address (put in pr by the jsr)
  static const int kCallTargetAddressOffset = 2 * kInstrSize + 4 + 4 * kInstrSize;

  // Distance between start of patched return sequence and the emitted address
  // to jump to.
  static const int kPatchReturnSequenceAddressOffset = 0 * kInstrSize; // FIXME(STM)

  // Distance between start of patched debug break slot and the emitted address
  // to jump to.
  static const int kPatchDebugBreakSlotAddressOffset = 0 * kInstrSize; // FIXME(STM)

  // The debug break slot must be able to contain a call instruction.
  static const int kDebugBreakSlotLength = kInstrSize; // FIXME(STM)


  // branch type
  enum branch_type {
    branch_true          = 1 << 1,
    branch_false         = 1 << 2,
    branch_unconditional = 1 << 3,
    branch_subroutine    = 1 << 4
  };

  static const RegList kAllRegisters = 0xffffffff;


  // ---------------------------------------------------------------------------
  // Wrappers around the code generators
  void add(Register Rd, const Immediate& imm, Register rtmp = r11);
  void add(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);
  void add(Register Rd, Register Rs, Register Rt);

  void bt(Label* L, Register rtmp = r11)    { branch(L, rtmp, branch_true); }
  void bf(Label* L, Register rtmp = r11)    { branch(L, rtmp, branch_false); }
  void jmp(Label* L, Register rtmp = r11)   { branch(L, rtmp, branch_unconditional); }
  void b(Label* L, Register rtmp = r11)   { jmp(L, rtmp); }
  void b(Condition cond, Label* L, Register rtmp = r11)   { 
    ASSERT(cond == ne || cond == eq);
    branch(L, rtmp, cond == eq ? branch_true: branch_false);
  }

  void jsr(Label* L, Register rtmp = r11)   { branch(L, rtmp, branch_subroutine); }

  // Check the code size generated from label to here.
  int InstructionsGeneratedSince(Label* l) {
    return (pc_offset() - l->pos()) / kInstrSize;
  }

  void jmp(Register Rd)                    { jmp_indRd_(Rd); nop_(); }
  void jsr(Register Rd)                    { jsr_indRd_(Rd); nop_(); }

  void jmp(Handle<Code> code, RelocInfo::Mode rmode, Register rtmp = r11);
  void jsr(Handle<Code> code, RelocInfo::Mode rmode, Register rtmp = r11);

  void cmpeq(Register Rd, Register Rs) { cmpeq_(Rs, Rd); }
  void cmpgt(Register Rd, Register Rs) { cmpgt_(Rs, Rd); }      // is Rd > Rs ?
  void cmpge(Register Rd, Register Rs) { cmpge_(Rs, Rd); }      // is Rd >= Rs ?
  void cmpgtu(Register Rd, Register Rs) { cmphi_(Rs, Rd); }     // is Rd u> Rs ?
  void cmpgeu(Register Rd, Register Rs) { cmphs_(Rs, Rd); }     // is Rd u>= Rs ?
  void cmpeq(Register Rd, const Immediate& imm, Register rtmp = r11) { 
    mov(rtmp, imm); cmpeq_(rtmp, Rd); }
  void cmpgt(Register Rd, const Immediate& imm, Register rtmp = r11) {
    mov(rtmp, imm); cmpgt_(rtmp, Rd); }
  void cmpge(Register Rd, const Immediate& imm, Register rtmp = r11) {
    mov(rtmp, imm); cmpge_(rtmp, Rd); }
  void cmpgtu(Register Rd, const Immediate& imm, Register rtmp = r11) {
    mov(rtmp, imm); cmphi_(rtmp, Rd); }
  void cmpgeu(Register Rd, const Immediate& imm, Register rtmp = r11) {
    mov(rtmp, imm); cmphs_(rtmp, Rd); }

  void cmp(Register Rd, Register Rs) { cmpeq(Rd, Rs); } // Alias for cmpeq
  void cmp(Register Rd, const Immediate& imm, Register rtmp = r11) { // Alias for cmpeq
    cmpeq(Rd, imm, rtmp); }

  INLINE(void cmp(Condition& cond, Register Rd, Register Rs));
  void cmp(Condition& cond, Register Rd, const Immediate& imm, Register rtmp = r11) {
    mov(rtmp, imm); return cmp(cond, Rd, rtmp); }

  void cmphs(Register Rd, Register Rs) { // Alias for cmpgeu
    cmpgeu(Rd, Rs); }
  void cmphs(Register Rd, const Immediate& imm, // Alias for cmpgeu
	     Register rtmp = r11) { cmpgeu(Rd, imm, rtmp); }

  void cmphi(Register Rd, Register Rs) { // Alias for cmpgtu
    cmpgtu(Rd, Rs); }
  void cmphi(Register Rd, const Immediate& imm, // Alias for cmpgtu
	     Register rtmp = r11) { cmpgtu(Rd, imm, rtmp); }

  void cmpeq_r0_raw_immediate(int raw_immediate) { cmpeq_imm_R0_((int8_t)raw_immediate); }
  int fits_raw_immediate(int raw_immediate) { return (raw_immediate & ~0xFF) == 0; };

  // Read/patch instructions
  static Instr instr_at(byte* pc) { return *reinterpret_cast<Instr*>(pc); }
  static void instr_at_put(byte* pc, Instr instr) {
    *reinterpret_cast<Instr*>(pc) = instr;
  }
  static Condition GetCondition(Instr instr);
  static bool IsBranch(Instr instr);
  static Register GetRn(Instr instr);
  static Register GetRm(Instr instr);
  static bool IsCmpRegister(Instr instr);
  static bool IsCmpImmediate(Instr instr);
  static Register GetCmpImmediateRegister(Instr instr);
  static int GetCmpImmediateRawImmediate(Instr instr);
  static bool IsMovImmediate(Instr instr);

  void sub(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);
  void sub(Register Rd, Register Rs, Register Rt);

  void rsb(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11) { // Reverse sub: imm - Rs
    mov(rtmp, imm); rsb(Rd, Rs, rtmp);
  }
  void rsb(Register Rd, Register Rs, Register Rt) { // Reverse sub: Rt - Rs
    sub(Rd, Rt, Rs);
  }

  void addv(Register Rd, Register Rs, Register Rt);
  void subv(Register Rd, Register Rs, Register Rt, Register rtmp = r11);

  void addc(Register Rd, Register Rs, Register Rt);
  void subc(Register Rd, Register Rs, Register Rt, Register rtmp = r11);

  void asr(Register Rd, Register Rs, Register Rt, Register rtmp = r11);	// arithmetic shift right
  void asr(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);    // arithmetic shift right
  void asl(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);    // arithmetic shift left

  void lsl(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);
  void lsl(Register Rd, Register Rs, Register Rt);
  void lsr(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);
  void lsr(Register Rd, Register Rs, Register Rt, Register rtmp = r11);

  void land(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);
  void land(Register Rd, Register Rs, Register Rt);
  void bic(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11) { //bit clear
    land(Rd, Rs, Immediate(~imm.x_), rtmp);
  }

  void lnot(Register Rd, Register Rs) { not_(Rs, Rd); }
  void mvn(Register Rd, Register Rs) { lnot(Rd, Rs); } // Alias for lnot()

  void lor(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);
  void lor(Register Rd, Register Rs, Register Rt);

  void lxor(Register Rd, Register Rs, const Immediate& imm, Register rtmp = r11);
  void lxor(Register Rd, Register Rs, Register Rt);

  void eor(Register Rd, Register Rs, const Immediate& imm,// Alias for lxor
	   Register rtmp = r11) { lxor(Rd, Rs, imm, rtmp); }
  void eor(Register Rd, Register Rs, Register Rt)  { // Alias for lxor
    lxor(Rd, Rs, Rt);
  }

  void orr(Register Rd, Register Rs, const Immediate& imm,// Alias for lor
	   Register rtmp = r11) { lor(Rd, Rs, imm, rtmp); }
  void orr(Register Rd, Register Rs, Register Rt)  { // Alias for lor
    lor(Rd, Rs, Rt);
  }
    
  void tst(Register Rd, Register Rs) { tst_(Rs, Rd); };
  void tst(Register Rd, const Immediate& imm, Register rtmp = r11);

  void mov(Register Rd, Register Rs, Condition cond); // Conditional move
  void mov(Register Rd, const Immediate& imm, Condition cond); // Conditional move
  void mov(Register Rd, Register Rs) { mov_(Rs, Rd); }
  void mov(Register Rd, const Immediate& imm);
  void mov(Register Rd, const Operand& src);

  void mov(Register Rd, const MemOperand& src, Register rtmp = r11);  // load op.
  void movb(Register Rd, const MemOperand& src, Register rtmp = r11); // unsigned 8 bit load op.
  void movw(Register Rd, const MemOperand& src, Register rtmp = r11); // unsigned 16 bit load op.
  void mov(const MemOperand& dst, Register Rd, Register rtmp = r11);  // store op.
  void movw(const MemOperand& dst, Register Rd, Register rtmp = r11);  // store 16 bits op.

  void ldr(Register Rd, const MemOperand& src, Register rtmp = r11) { mov(Rd, src, rtmp); }
  void ldrb(Register Rd, const MemOperand& src, Register rtmp = r11) { movb(Rd, src, rtmp); }
  void ldrh(Register Rd, const MemOperand& src, Register rtmp = r11) { movw(Rd, src, rtmp); }
  void str(Register Rs, const MemOperand& dst, Register rtmp = r11) { mov(dst, Rs, rtmp); }
  void strh(Register Rs, const MemOperand& dst, Register rtmp = r11) { movw(dst, Rs, rtmp); }
  void strb(Register Rs, const MemOperand& dst, Register rtmp = r11) { mov(dst, Rs, rtmp); }

  void mul(Register Rd, Register Rs, Register Rt);

  void nop() { nop_(); }

  void push(Register src);
  void push(DwVfpRegister src);
  // push an immediate on the stack: use rtmp register for that
  void push(const Immediate& imm, Register rtmp = r11);
  void push(const Operand& op, Register rtmp = r11);
  void pushm(RegList src, bool doubles = false);

  void pop(Register dst);
  void pop(DwVfpRegister dst);
  void popm(RegList dst, bool doubles = false);

  void rts() { rts_(); nop_(); }

  // Exception-generating instructions and debugging support
  void stop(const char* msg);
  void bkpt();

  // Align the code
  int align() { int count = 0; while (((unsigned)pc_ & 0x3) != 0) { nop_(); count++; } return count; }

  // Insert the smallest number of nop instructions
  // possible to align the pc offset to a multiple
  // of m. m must be a power of 2 (>= 4).
  void Align(int m);

  void call(Label* L);

  void emit(Instr x) { CheckBuffer(); *reinterpret_cast<uint16_t*>(pc_) = x; pc_ += sizeof(uint16_t); }


  // Mark address of the ExitJSFrame code.
  void RecordJSReturn();

  // Use -code_comments to enable, or provide "force = true" flag to always
  // write a comment.
  void RecordComment(const char* msg, bool force = false);

  // Record code generator line mapping through comments.
  // Use -code_comments to enable.
  void RecordFunctionLine(const char* function, int line);

  // Writes a single byte or word of data in the code stream.  Used for
  // inline tables, e.g., jump-tables.
  void db(uint8_t data);
  void dw(uint16_t data);
  void dd(uint32_t data);

  int pc_offset() const { return pc_ - buffer_; }

  // Return in Rd the value of pc_after + offset.
  // Where pc_after is the pc after this operation.
  void addpc(Register Rd, int offset, Register rtmp = r11);

  // Check if there is less than kGap bytes available in the buffer.
  // If this is the case, we need to grow the buffer before emitting
  // an instruction or relocation information.
  inline bool overflow() const { return pc_ >= reloc_info_writer.pos() - kGap; }

  // Get the number of bytes available in the buffer.
  inline int available_space() const { return reloc_info_writer.pos() - pc_; }

  PositionsRecorder* positions_recorder() { return &positions_recorder_; }


 protected:
  bool emit_debug_code() const { return emit_debug_code_; }

  int buffer_space() const { return reloc_info_writer.pos() - pc_; }


 private:
  // code generation wrappers
  void branch(Label* L, Register rtmp, branch_type type);
  void branch(int offset, Register rtmp, branch_type type, bool patched_later);
  void bt(int offset, Register rtmp, bool patched_later);
  void bf(int offset, Register rtmp, bool patched_later);
  void jmp(int offset, Register rtmp, bool patched_later);
  void jsr(int offset, Register rtmp, bool patched_later);


  void writeBranchTag(int nop_count, branch_type type);
  void patchBranchOffset(int fixup_pos, uint16_t *p_pos);

  // The bound position, before this we cannot do instruction elimination.
  int last_bound_pos_;

  // Code emission
  inline void CheckBuffer();
  void GrowBuffer();

  void next(Label *L);

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
  // The relocation writer's position is at least kGap bytes below the end of
  // the generated instructions. This is so that multi-instruction sequences do
  // not have to check for overflow. The same is true for writes of large
  // relocation info entries.
  static const int kGap = 32;
  byte* pc_;  // the program counter; moves forward
  RelocInfoWriter reloc_info_writer;

  // push-pop elimination
  byte* last_pc_;

  PositionsRecorder positions_recorder_;

  bool emit_debug_code_;

  friend class PositionsRecorder;


  // ---------------------------------------------------------------------------
  // low level code generation (opcodes)
  #include "opcodes-sh4.h"
};


// Helper class that ensures that there is enough space for generating
// instructions and relocation information.  The constructor makes
// sure that there is enough space and (in debug mode) the destructor
// checks that we did not generate too much.
class EnsureSpace BASE_EMBEDDED {
 public:
  explicit EnsureSpace(Assembler* assembler) : assembler_(assembler) {
    if (assembler_->overflow()) assembler_->GrowBuffer();
#ifdef DEBUG
    space_before_ = assembler_->available_space();
#endif
  }

#ifdef DEBUG
  ~EnsureSpace() {
    int bytes_generated = space_before_ - assembler_->available_space();
    ASSERT(bytes_generated < assembler_->kGap);
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
