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

// A light-weight SH4 Assembler.

#ifndef V8_SH4_ASSEMBLER_SH4_H_
#define V8_SH4_ASSEMBLER_SH4_H_

#include "assembler.h"
#include "constants-sh4.h"
#include "serialize.h"

namespace v8 {
namespace internal {

// CpuFeatures keeps track of which features are supported by the target CPU.
// Supported features must be enabled by a CpuFeatureScope before use.
class CpuFeatures : public AllStatic {
 public:
  // Detect features of the target CPU. Set safe defaults if the serializer
  // is enabled (snapshots must be portable).
  static void Probe();

  // Display target use when compiling.
  static void PrintTarget();

  // Display features.
  static void PrintFeatures();

  // Check whether a feature is supported by the target CPU.
  static bool IsSupported(CpuFeature f) {
    ASSERT(initialized_);
    return Check(f, supported_);
  }

  static bool IsFoundByRuntimeProbingOnly(CpuFeature f) {
    ASSERT(initialized_);
    return Check(f, found_by_runtime_probing_only_);
  }

  static bool IsSafeForSnapshot(CpuFeature f) {
    return Check(f, cross_compile_) ||
           (IsSupported(f) &&
            (!Serializer::enabled() || !IsFoundByRuntimeProbingOnly(f)));
  }

  static unsigned cache_line_size() { return cache_line_size_; }

  static bool VerifyCrossCompiling() {
    return cross_compile_ == 0;
  }

  static bool VerifyCrossCompiling(CpuFeature f) {
    unsigned mask = flag2set(f);
    return cross_compile_ == 0 ||
           (cross_compile_ & mask) == mask;
  }

 private:
  static bool Check(CpuFeature f, unsigned set) {
    return (set & flag2set(f)) != 0;
  }

  static unsigned flag2set(CpuFeature f) {
    return 1u << f;
  }

#ifdef DEBUG
  static bool initialized_;
#endif
  static unsigned supported_;
  static unsigned found_by_runtime_probing_only_;
  static unsigned cache_line_size_;

  static unsigned cross_compile_;

  friend class ExternalReference;
  friend class PlatformFeatureScope;
  DISALLOW_COPY_AND_ASSIGN(CpuFeatures);
};


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

// These constants are used in several locations, including static initializers
const int kRegister_no_reg_Code = -1;
const int kRegister_r0_Code = 0;
const int kRegister_r1_Code = 1;
const int kRegister_r2_Code = 2;
const int kRegister_r3_Code = 3;
const int kRegister_r4_Code = 4;
const int kRegister_r5_Code = 5;
const int kRegister_r6_Code = 6;
const int kRegister_r7_Code = 7;
const int kRegister_r8_Code = 8;
const int kRegister_r9_Code = 9;
const int kRegister_r10_Code = 10;
const int kRegister_r11_Code = 11;
const int kRegister_roots_Code = 12;
const int kRegister_cp_Code = 13;
const int kRegister_fp_Code = 14;
const int kRegister_sp_Code = 15;
const int kRegister_pr_Code = -2;

// Core register
struct Register {
  static const int kNumRegisters = 16;
  static const int kNumAllocatableRegisters = 8;
  static const int kSizeInBytes = 4;

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

const Register no_reg = { kRegister_no_reg_Code };

const Register r0 = { kRegister_r0_Code };      // ABI caller saved/return, idem for JIT
const Register r1 = { kRegister_r1_Code };      // ABI caller saved/return, idem for JIT
const Register r2 = { kRegister_r2_Code };      // ABI caller saved/return, idem for JIT
const Register r3 = { kRegister_r3_Code };      // ABI caller saved/return, idem for JIT
const Register r4 = { kRegister_r4_Code };      // ABI caller saved/param, idem for JIT
const Register r5 = { kRegister_r5_Code };      // ABI caller saved/param, idem for JIT
const Register r6 = { kRegister_r6_Code };      // ABI caller saved/param, idem for JIT
const Register r7 = { kRegister_r7_Code };      // ABI caller saved/param, idem for JIT
const Register r8 = { kRegister_r8_Code };      // ABI callee saved, idem for JIT
const Register r9 = { kRegister_r9_Code };      // ABI callee saved, idem for JIT
const Register r10 = { kRegister_r10_Code };    // ABI callee saved, idem for JIT
const Register r11 = { kRegister_r11_Code };    // ABI callee saved, idem for JIT
const Register roots = { kRegister_roots_Code };  // ABI GP, root table pointer for JIT
const Register cp = { kRegister_cp_Code };     // ABI callee saved, context pointer for JIT
const Register fp = { kRegister_fp_Code };     // ABI FP, idem for JIT
const Register sp = { kRegister_sp_Code };     // ABI SP, idem for JIT

const Register pr = { kRegister_pr_Code };     // Link register

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
const Register sh4_r12 = roots;
const Register sh4_r13 = cp;
const Register sh4_rtmp = r11;  // Used for low level (assembler-sh4.cc)
const Register sh4_ip = r10;    // Used as additional scratch in JS code


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
  static SwVfpRegister from_code(int code) {
    SwVfpRegister r = { code };
    return r;
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
  static const int kMaxNumRegisters = 8;
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
// Callee saved registers
// Using these registers is forbidden for the moment as we do not save/restaure
// them on ABI frontiers.
//const SwVfpRegister fr12 = { 12 };
//const SwVfpRegister fr13 = { 13 };
//const SwVfpRegister fr14 = { 14 };
//const SwVfpRegister fr15 = { 15 };


// Caller saved registers
const DwVfpRegister no_dreg = { -1 };
const DwVfpRegister dr0   = {  0  };
const DwVfpRegister dr2   = {  2  };
const DwVfpRegister dr4   = {  4  };
const DwVfpRegister dr6   = {  6  };
const DwVfpRegister dr8   = {  8  };
const DwVfpRegister dr10  = {  10 };
// Callee saved registers.
// The uses of theses registers is forbidden for the moment as we do not
// save/restaure them on ABI frontiers.
//const DwVfpRegister dr12  = {  12 };
//const DwVfpRegister dr14  = {  14 };

enum Condition {
  // any value < 0 is considered no_condition
  kNoCondition = -1,

  eq = 0 << 28,       // equal
  ne = 1 << 28,       // not equal
  gt = 2 << 28,       // greater
  ge = 3 << 28,       // greater or equal
  hi = 4 << 28,       // unsigned higher
  hs = 5 << 28,       // unsigned higher or equal
  lt = 6 << 28,       // lesser
  le = 7 << 28,       // lesser or equal
  ui = 8 << 28,       // unsigned lower
  us = 9 << 28,       // unsigned lower or equal
  pl = 10 << 28,      // positiv
  pz = 11 << 28,      // positiv or null
  ql = 12 << 28,      // negativ
  qz = 13 << 28,      // negativ or null
  al = 14 << 28,      // Always

  // Aliases
  t = eq,        // cmp eq; if SH4 cmpeq/cmp sets the T bit, t == eq
  f = ne         // cmp ne: if SH4 cmpeq/cmp clears the T bit, f == ne
};

enum AddrMode {
  PreIndex,
  PostIndex,
  Offset
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
  inline explicit Operand(int32_t immediate,
                          RelocInfo::Mode rmode = RelocInfo::NONE32);
  inline explicit Operand(const ExternalReference& f);
  inline explicit Operand(Smi* value);
  explicit Operand(Handle<Object> handle);

  bool is_int8() const {
    return -128 <= imm32_ && imm32_ < 128 && rmode_ == RelocInfo::NONE32;
  }

 private:
  int32_t imm32_;
  RelocInfo::Mode rmode_;

  friend class Assembler;
};


class MemOperand BASE_EMBEDDED {
 public:
  INLINE(explicit MemOperand(Register Rd, int32_t offset = 0, AddrMode mode = Offset));
  INLINE(explicit MemOperand(Register Rd, Register offset));

  void set_offset(int32_t offset) {
      ASSERT(rn_.is(no_reg));
      offset_ = offset;
  }

  uint32_t offset() const {
      ASSERT(rn_.is(no_reg));
      return offset_;
  }

  Register rn() const { return rn_; }
  Register rm() const { return rm_; }

 private:
  Register rm_;
  Register rn_;
  int32_t offset_;
  AddrMode mode_;

  friend class Assembler;
};


typedef uint16_t Instr;

// Use a target specfic value instead of kZapValue
const int kSH4ZapValue = 0xbadbaffe;


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
  virtual ~Assembler();

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

  // binds an unbound label L to the current code position
  void bind(Label* L);

  // Puts a labels target address at the given position.
  // This function is only used with not-bound labels
  void load_label(Label* L);

  // Return the address in the constant pool of the code target address used by
  // the branch/call instruction at pc, or the object in a mov.
  INLINE(static Address target_pointer_address_at(Address pc));

  // Read/Modify the pointer in the branch/call/move instruction at pc.
  INLINE(static Address target_pointer_at(Address pc));
  INLINE(static void set_target_pointer_at(Address pc, Address target));

  // Read/Modify the code target address in the branch/call instruction at pc.
  INLINE(static Address target_address_at(Address pc));
  INLINE(static void set_target_address_at(Address pc, Address target));

  // Return the code target address at a call site from the return address
  // of that call in the instruction stream.
  INLINE(static Address target_address_from_return_address(Address pc));

  // Given the address of the beginning of a call, return the address
  // in the instruction stream that the call will return from.
  INLINE(static Address return_address_from_call_start(Address pc));

  // This sets the branch destination (which is in the constant pool on SH4).
  // This is for calls and branches within generated code.
  inline static void deserialization_set_special_target_at(
      Address constant_pool_entry, Address target);

  // This sets the branch destination (which is in the constant pool on SH4).
  // This is for calls and branches to runtime code.
  inline static void set_external_target_at(Address constant_pool_entry,
                                            Address target) {
    // same as above, this function is currently not used anywhere.
    UNREACHABLE();
  }

  static const int kSpecialTargetSize = kPointerSize;

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
  static const int kOldStyleCallTargetAddressOffset = 2 * kInstrSize +
                                                      2 * kInstrSize +
                                                      4 * kInstrSize;
  static const int kNewStyleCallTargetAddressOffset = 3 * kInstrSize;

  int GetCallTargetAddressOffset() const {
    // constant_pool_pool_ is not meant to be switched mid-flight
    if (constant_pool_pool_)
      return kNewStyleCallTargetAddressOffset; // mov.l; jsr; nop
    else
      return kOldStyleCallTargetAddressOffset; // see above
  }

  static int ResolveCallTargetAddressOffset(byte *address);
  // Distance between start of patched return sequence and the emitted address
  // to jump to.
  static const int kPatchReturnSequenceAddressOffset = 0 * kInstrSize;

  // Distance between start of patched debug break slot and the emitted address
  // to jump to.
  static const int kPatchDebugBreakSlotAddressOffset = 0 * kInstrSize;

  // Difference between address of current opcode and value read from pc
  // register.
  static const int kPcLoadDelta = 2;

  // The debug break slot must be able to contain a call instruction.
  static const int kDebugBreakSlotLength = kInstrSize;

  // branch type
  enum branch_type {
    branch_true          = 1 << 1,
    branch_false         = 1 << 2,
    branch_unconditional = 1 << 3,
    branch_subroutine    = 1 << 4
  };

  static const RegList kAllRegisters = 0xffffffff;

  // Negative pc_offset value (aligned value)
  static const int kEndOfChain = -4;


  // ---------------------------------------------------------------------------
  // Wrappers around the code generators
  void add(Register Rd, const Operand& imm, Register rtmp = sh4_rtmp);
  void add(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp);
  void add(Register Rd, Register Rs, Register Rt);

  void bt(Label* L, Register rtmp = sh4_rtmp)
        { ASSERT(!L->is_near_linked());
          branch(L, rtmp, branch_true); }
  void bt_near(Label* L, Register rtmp = sh4_rtmp)
        { ASSERT(L->is_bound() || L->is_unused() || L->is_near_linked());
          branch(L, rtmp, branch_true, Label::kNear); }

  void bf(Label* L, Register rtmp = sh4_rtmp)
        { ASSERT(!L->is_near_linked());
          branch(L, rtmp, branch_false); }
  void bf_near(Label* L, Register rtmp = sh4_rtmp)
        { ASSERT(L->is_bound() || L->is_unused() || L->is_near_linked());
          branch(L, rtmp, branch_false, Label::kNear); }

  void jmp(Label* L, Register rtmp = sh4_rtmp)
        { ASSERT(!L->is_near_linked());
          branch(L, rtmp, branch_unconditional); }
  void jmp_near(Label* L, Register rtmp = sh4_rtmp)
        { ASSERT(L->is_bound() || L->is_unused() || L->is_near_linked());
          branch(L, rtmp, branch_unconditional, Label::kNear); }

  void b(Label* L, Register rtmp = sh4_rtmp)
        { jmp(L, rtmp); }
  void b_near(Label* L, Register rtmp = sh4_rtmp)
        { jmp_near(L, rtmp); }

  void b(Condition cond, Label* L, Label::Distance distance = Label::kFar,
         Register rtmp = sh4_rtmp) {
    ASSERT((distance == Label::kNear && (L->is_bound() || L->is_unused() || L->is_near_linked())) ||
           (distance == Label::kFar  && (!L->is_near_linked())));
    ASSERT(cond == ne || cond == eq);
    branch(L, rtmp, cond == eq ? branch_true: branch_false, distance);
  }

  void jsr(Label* L, Register rtmp = sh4_rtmp)
        { branch(L, rtmp, branch_subroutine); }


  // Check the code size generated from label to here.
  int SizeOfCodeGeneratedSince(Label* label) {
    return pc_offset() - label->pos();
  }

  // Check the number of instructions generated from label to here.
  int InstructionsGeneratedSince(Label* label) {
    return SizeOfCodeGeneratedSince(label) / kInstrSize;
  }

  void jmp(Register Rd);
  void jsr(Register Rd);
  void jmp(Handle<Code> code, RelocInfo::Mode rmode, Register rtmp = sh4_rtmp);
  void jsr(Handle<Code> code, RelocInfo::Mode rmode, Register rtmp = sh4_rtmp);

  void cmpeq(Register Rd, Register Rs) { ASSERT(!Rs.is(Rd)); cmpeq_(Rs, Rd); }
  void cmpgt(Register Rd, Register Rs) { ASSERT(!Rs.is(Rd)); cmpgt_(Rs, Rd); }     // is Rd > Rs ?
  void cmpge(Register Rd, Register Rs) { ASSERT(!Rs.is(Rd)); cmpge_(Rs, Rd); }     // is Rd >= Rs ?
  void cmphi(Register Rd, Register Rs) { ASSERT(!Rs.is(Rd)); cmphi_(Rs, Rd); }    // is Rd u> Rs ?
  void cmphs(Register Rd, Register Rs) { ASSERT(!Rs.is(Rd)); cmphs_(Rs, Rd); }    // is Rd u>= Rs ?

  inline void cmpeq(Register Rd, const Operand& imm,
                    Register rtmp = sh4_rtmp);
  inline void cmpgt(Register Rd, const Operand& imm,
                    Register rtmp = sh4_rtmp);
  inline void cmpge(Register Rd, const Operand& imm,
                    Register rtmp = sh4_rtmp);
  inline void cmphi(Register Rd, const Operand& imm,
                    Register rtmp = sh4_rtmp);
  inline void cmphs(Register Rd, const Operand& imm,
                    Register rtmp = sh4_rtmp);

  // ALiases for cmpeq
  void cmp(Register Rd, Register Rs) { cmpeq_(Rs, Rd); }
  void cmp(Register Rd, const Operand& imm, Register rtmp = sh4_rtmp)
        { cmpeq(Rd, imm, rtmp); }

  inline void cmp(Condition *cond, Register Rd, Register Rs);
  void cmp(Condition *cond, Register Rd, const Operand& imm,
           Register rtmp = sh4_rtmp) {
    mov(rtmp, imm);
    cmp(cond, Rd, rtmp);
  }

  void cmpeq_r0_unsigned_imm(int imm) {
    ASSERT(is_uint8(imm));
    cmpeq_imm_R0_((int8_t)imm); }
  bool fits_cmp_unsigned_imm(int imm) { return is_uint8(imm); }

  void dt(Register Rd)  { dt_(Rd); }

  // FPU support
  // Load float
  void fldr(SwVfpRegister dst, const MemOperand& src, Register rtmp = sh4_rtmp);
  // Load double
  void dldr(DwVfpRegister dst, const MemOperand& src, Register rtmp = sh4_rtmp);
 // Store float
  void fstr(SwVfpRegister src, const MemOperand& dst, Register rtmp = sh4_rtmp);
  // Store double
  void dstr(DwVfpRegister src, const MemOperand& dst, Register rtmp = sh4_rtmp);

  // Double conversion from register: Dd = (double)Rs
  void dfloat(DwVfpRegister Dd, Register Rs);
  // Double conversion from int operand: Dd = (double)imm
  void dfloat(DwVfpRegister Dd, const Operand &imm, Register rtmp = sh4_rtmp);

  // Double conversion from unsigned int register: Dd = (double)Rs(unsigned)
  void dufloat(DwVfpRegister Dd, Register Rs, DwVfpRegister drtmp, Register rtmp);

  // Interger conversion from double: Rs = (int)Dd
  void idouble(Register Rd, DwVfpRegister Ds, Register fpscr = no_reg);
  // Interger conversion from dingle: Rs = (int)Frs
  void isingle(Register Rd, SwVfpRegister Frs);

  // Conversion from simple to double
  void fcnvsd(DwVfpRegister Dd, SwVfpRegister Fs)   { flds_FPUL_(Fs); fcnvsd_FPUL_double_(Dd); }
  // Conversion from double to simple
  void fcnvds(SwVfpRegister Fd, DwVfpRegister Ds)   { fcnvds_double_FPUL_(Ds); fsts_FPUL_(Fd); }

  // Double comparisons
  void dcmpeq(DwVfpRegister Dd, DwVfpRegister Ds)   { fcmpeq_double_(Ds, Dd); }
  void dcmpgt(DwVfpRegister Dd, DwVfpRegister Ds)   { fcmpgt_double_(Ds, Dd); }

  // FPU operations
  void fadd(DwVfpRegister Dd, DwVfpRegister Ds)     { fadd_double_(Ds, Dd); }
  void fsub(DwVfpRegister Dd, DwVfpRegister Ds)     { fsub_double_(Ds, Dd); }
  void fmul(DwVfpRegister Dd, DwVfpRegister Ds)     { fmul_double_(Ds, Dd); }
  void fdiv(DwVfpRegister Dd, DwVfpRegister Ds)     { fdiv_double_(Ds, Dd); }

  // Read/patch instructions
  static Instr instr_at(byte* pc)
        { return *reinterpret_cast<Instr*>(pc); }
  static void instr_at_put(byte* pc, Instr instr)
        { *reinterpret_cast<Instr*>(pc) = instr; }
  static Condition GetCondition(Instr instr);
  static bool IsBranch(Instr instr);
  static bool IsJsr(Instr instr);
  static Register GetRn(Instr instr);
  static Register GetRm(Instr instr);
  static bool IsCmpRegister(Instr instr);
  static bool IsCmpImmediate(Instr instr);
  static Register GetCmpImmediateRegister(Instr instr);
  static int GetCmpImmediateAsUnsigned(Instr instr);
  static bool IsMovImmediate(Instr instr);
  static bool IsMovlPcRelative(Instr instr)
      { return (instr & (0xf << 12)) == 0xd000; }

  void sub(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp);
  void sub(Register Rd, Register Rs, Register Rt);

  // Reverse sub: imm - Rs
  inline void rsb(Register Rd, Register Rs, const Operand& imm,
                  Register rtmp = sh4_rtmp);
  // Reverse sub: Rt - Rs
  inline void rsb(Register Rd, Register Rs, Register Rt);
  inline void rsb(Register Rd, Register Rs, const Operand& imm,
                  Condition cond, Register rtmp = sh4_rtmp);

  void addv(Register Rd, Register Rs, Register Rt);
  void addv(Register Rd, Register Rs, const Operand& imm,
            Register rtmp = sh4_rtmp);
  void subv(Register Rd, Register Rs, Register Rt, Register rtmp = sh4_rtmp);

  void addc(Register Rd, Register Rs, Register Rt);
  void subc(Register Rd, Register Rs, Register Rt, Register rtmp = sh4_rtmp);

  // Note for shifts with shift amount in register:
  // the default behavior is mapped on ARM behavior which flushes when shift amount
  // is >= 32. The implementation is quite slow in this case as SH4 shifts
  // do not flush.
  // In case where the register shift amount is known to be in range [0,31] one
  // can call the shift methods with in_range parameter set to true. This case
  // generates faster code.
  // In addition, in the case of lsl (but not asr nor lsr) the semantic is
  // to wrap on SH4 (i.e. the least 5 bits are extracted before doing the shift), thus
  // in this case the boolean parameter is called wrap and this semantic can be used
  // on purpose whatever the shift amount.

  // arithmetic shift right
  void asr(Register Rd, Register Rs, Register Rt, bool in_range = false, Register rtmp = sh4_rtmp);
  void asr(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp);
  // arithmetic shift left
  void asl(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp);

  void lsl(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp);
  void lsl(Register Rd, Register Rs, Register Rt, bool wrap = false, Register rtmp = sh4_rtmp);
  void lsr(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp);
  void lsr(Register Rd, Register Rs, Register Rt, bool in_range = false, Register rtmp = sh4_rtmp);

  void land(Register Rd, Register Rs, const Operand& imm,
            Register rtmp = sh4_rtmp);
  void land(Register Rd, Register Rs, Register Rt);

  // bit clear
  void bic(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp)
        { land(Rd, Rs, Operand(~imm.imm32_), rtmp); }
  void bic(Register Rd, Register Rs, Register Rt, Register rtmp = sh4_rtmp) {
    lnot(rtmp, Rt);
    land(Rd, Rs, rtmp);
  }

  void lnot(Register Rd, Register Rs) { not_(Rs, Rd); }
  void mvn(Register Rd, Register Rs)  { lnot(Rd, Rs); }  // Alias for lnot()

  void lor(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp);
  void lor(Register Rd, Register Rs, Register Rt);
  void lor(Register Rd, Register Rs, const Operand& imm, Condition cond,
           Register rtmp = sh4_rtmp);
  void lor(Register Rd, Register Rs, Register Rt, Condition cond);

  void lxor(Register Rd, Register Rs, const Operand& imm,
            Register rtmp = sh4_rtmp);
  void lxor(Register Rd, Register Rs, Register Rt);

  // Aliases for lxor
  void eor(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp) { lxor(Rd, Rs, imm, rtmp); }
  void eor(Register Rd, Register Rs, Register Rt)  { lxor(Rd, Rs, Rt); }

  // Aliases for lor
  void orr(Register Rd, Register Rs, const Operand& imm,
           Register rtmp = sh4_rtmp) { lor(Rd, Rs, imm, rtmp); }
  void orr(Register Rd, Register Rs, Register Rt)  { lor(Rd, Rs, Rt); }
  void orr(Register Rd, Register Rs, const Operand& imm,
           Condition cond, Register rtmp = sh4_rtmp)
        { lor(Rd, Rs, imm, cond, rtmp); }
  void orr(Register Rd, Register Rs, Register Rt, Condition cond)
        { lor(Rd, Rs, Rt, cond); }

  void tst(Register Rd, Register Rs) { tst_(Rs, Rd); }
  void tst(Register Rd, const Operand& imm, Register rtmp = sh4_rtmp);

  void teq(Register Rd, const Operand& imm, Register rtmp = sh4_rtmp) {
    lxor(rtmp, Rd, imm);
    tst(rtmp, rtmp);
  }
  void teq(Register Rd, Register Rs, Register rtmp = sh4_rtmp) {
    lxor(rtmp, Rd, Rs);
    tst(rtmp, rtmp);
  }

  // Moves and conditional moves.
  // This one allows pr as src or dst.
  void mov(Register Rd, Register Rs, Condition cond = al);
  void mov(Register Rd, const Operand& src, bool force = false);
  void mov_pool(Register Rd, const Operand& imm, bool force);
  void mov(Register Rd, const Operand& imm, Condition cond);

  // load op.
  void mov(Register Rd, const MemOperand& src, Register rtmp = sh4_rtmp);
  // unsigned 8 bit load op.
  void movb(Register Rd, const MemOperand& src, Register rtmp = sh4_rtmp);
  // unsigned 16 bit load op.
  void movw(Register Rd, const MemOperand& src, Register rtmp = sh4_rtmp);
  // store op.
  void mov(const MemOperand& dst, Register Rd, Register rtmp = sh4_rtmp);
  // store 8 bits op.
  void movb(const MemOperand& dst, Register Rd, Register rtmp = sh4_rtmp);
  // store 16 bits op.
  void movw(const MemOperand& dst, Register Rd, Register rtmp = sh4_rtmp);

  void movd(DwVfpRegister Dd, Register Rs1, Register Rs2);
  void movd(Register Rd1, Register Rd2, DwVfpRegister Ds);

  inline void ldr(Register Rd, const MemOperand& src,
                  Register rtmp = sh4_rtmp);
  void ldrb(Register Rd, const MemOperand& src, Register rtmp = sh4_rtmp)
        { movb(Rd, src, rtmp); }
  void ldrh(Register Rd, const MemOperand& src, Register rtmp = sh4_rtmp)
        { movw(Rd, src, rtmp); }
  // signed 8 bit load op.
  void ldrsb(Register Rd, const MemOperand& src, Register rtmp = sh4_rtmp);
  // signed 16 bit load op.
  void ldrsh(Register Rd, const MemOperand& src, Register rtmp = sh4_rtmp);

  inline void str(Register Rs, const MemOperand& dst,
                  Register rtmp = sh4_rtmp);
  void strh(Register Rs, const MemOperand& dst, Register rtmp = sh4_rtmp)
        { movw(dst, Rs, rtmp); }
  void strb(Register Rs, const MemOperand& dst, Register rtmp = sh4_rtmp)
        { movb(dst, Rs, rtmp); }

  void ldrpr(Register Rd) { lds_PR_(Rd); }
  void strpr(Register Rs) { sts_PR_(Rs); }

  void ldr_fpscr(Register Rs) { lds_FPSCR_(Rs); }
  void str_fpscr(Register Rd) { sts_FPSCR_(Rd); }

  void mul(Register Rd, Register Rs, Register Rt);
  void dmuls(Register dstL, Register dstH, Register src1, Register src2);

  void nop() { nop_(); }

  void push(Register src);
  void push(DwVfpRegister src);
  // push an immediate on the stack: use rtmp register for that
  void push(const Operand& op, Register rtmp = sh4_rtmp);
  void pushm(RegList src, bool doubles = false);

  void pop(Register dst);
  void pop() { add(sp, sp, Operand(kPointerSize)); }
  void pop(DwVfpRegister dst);
  void popm(RegList dst, bool doubles = false);

  inline void rts();

  // Exception-generating instructions and debugging support
  void stop(const char* msg);
  void bkpt();

  // Align the code
  inline int align();
  inline int misalign();

  // Copy some bytes
  // The count argument is scratched
  void memcpy(Register dst, Register src, Register count,
              Register scratch1, Register scratch2,
              Register scratch3, Register scratch4);

  // Compare some bytes using a loop
  void memcmp(Register left, Register right, Register length,
              Register scratch1, Register scratch2, Label *not_equal);

  // Insert the smallest number of nop instructions
  // possible to align the pc offset to a multiple
  // of m. m must be a power of 2 (>= 4).
  void Align(int m);

  void call(Label* L);

  inline void emit(Instr x);

  // Class for scoping postponing the constant pool generation.
  class BlockConstPoolScope {
   public:
    explicit BlockConstPoolScope(Assembler* assem, int hint = 256) : assem_(assem) {
      assem_->StartBlockConstPool(hint);
    }
    ~BlockConstPoolScope() {
      assem_->EndBlockConstPool();
    }

   private:
    Assembler* assem_;

    DISALLOW_IMPLICIT_CONSTRUCTORS(BlockConstPoolScope);
  };

  // Mark address of the ExitJSFrame code.
  void RecordJSReturn();

  // Record the AST id of the CallIC being compiled, so that it can be placed
  // in the relocation information.
  void SetRecordedAstId(TypeFeedbackId ast_id) {
    ASSERT(recorded_ast_id_.IsNone());
    recorded_ast_id_ = ast_id;
  }

  TypeFeedbackId RecordedAstId() {
    ASSERT(!recorded_ast_id_.IsNone());
    return recorded_ast_id_;
  }

  void ClearRecordedAstId() { recorded_ast_id_ = TypeFeedbackId::None(); }

  // Use -code_comments to enable, or provide "force = true" flag to always
  // write a comment.
  void RecordComment(const char* msg, bool force = false);

  // Record the emission of a constant pool.
  //
  // The emission of constant pool depends on the size of the code generated and
  // the number of RelocInfo recorded.
  // The Debug mechanism needs to map code offsets between two versions of a
  // function, compiled with and without debugger support (see for example
  // Debug::PrepareForBreakPoints()).
  // Compiling functions with debugger support generates additional code
  // (Debug::GenerateSlot()). This may affect the emission of the constant
  // pools and cause the version of the code with debugger support to have
  // constant pools generated in different places.
  // Recording the position and size of emitted constant pools allows to
  // correctly compute the offset mappings between the different versions of a
  // function in all situations.
  //
  // The parameter indicates the size of the constant pool (in bytes), including
  // the marker and branch over the data.
  void RecordConstPool(int size);

  // Writes a single byte or word of data in the code stream.  Used for
  // inline tables, e.g., jump-tables.
  void db(uint8_t data);
  void dw(uint16_t data);
  void dd(uint32_t data);

  void ddLegacyBranchConst(uint32_t data);
  void emitPatchableNearBranch(uint16_t data);

  void emitConstPool(uint32_t data);

  // Emits the address of the code stub's first instruction.
  void emit_code_stub_address(Code* stub);

  // Return in Rd the value of pc_after + offset.
  // Where pc_after is the pc after this operation.
  // It clobbers pr which must be always passed in the Pr parameter
  void addpc(Register Rd, int offset, Register Pr);

  // Check if there is less than kGap bytes available in the buffer.
  // If this is the case, we need to grow the buffer before emitting
  // an instruction or relocation information.
  inline bool overflow() const { return pc_ >= reloc_info_writer.pos() - kGap; }

  // Get the number of bytes available in the buffer.
  inline int available_space() const { return reloc_info_writer.pos() - pc_; }

  PositionsRecorder* positions_recorder() { return &positions_recorder_; }

  // Constants in pools are accessed via pc relative addressing, which can
  // reach +1KB thereby defining a maximum distance between the instruction
  // and the accessed constant.
  // However, the branch, alignement and potential nops can decrease it a bit.
  // Be conservative and lower it more.
  static const int kMaxDistToPool = 1 * KB - 5 * kInstrSize;

  // Different to ARM the SH4 pc relative branch has a larger offset than
  // the pc relative load. With kMaxDistToPool always being triggered first,
  // However, some dummy code only doing "mov @(disp, PC), Rn" can trigger this
  // limit first
  static const int kMaxNumPendingRelocInfo = 254;

  // Postpone the generation of the constant pool for the specified number of
  // instructions.
  void BlockConstPoolFor(unsigned instructions);

  /**
   * Check if is time to emit a constant pool.
   * @param force_emit: dump the constant pool right now
   * @param require_jump: true if a jump over the constant pool as to be
   *                      emited
   * @param recursive: true if the function is called while constant pools
   *                   are blocked (this can only happen on the first call to
   *                   StartBlockConstPool
  */
  void CheckConstPool(bool force_emit, bool require_jump, bool recursive = false, int hint = 0);
  int GetFirstConstPoolUse() const { return first_const_pool_use_; }
  void AssertDataEmit(const char *str);
  void SwitchConstantPoolMode(bool enabled) { constant_pool_pool_ = enabled; }

 protected:
  // Relocation for a type-recording IC has the AST id added to it.  This
  // member variable is a way to pass the information from the call site to
  // the relocation info.
  TypeFeedbackId recorded_ast_id_;

  int buffer_space() const { return reloc_info_writer.pos() - pc_; }

  // Prevent contant pool emission until EndBlockConstPool is called.
  // Call to this function can be nested but must be followed by an equal
  // number of call to EndBlockConstpool.
  void StartBlockConstPool(int hint = 256) {
    if (const_pool_blocked_nesting_++ == 0) {
      // Check that the constant pool has to be emited before
      // By default we block for 256 instructions (a near jump)
      if (!emiting_const_pool_) {
        CheckConstPool(false, true, true, hint);
      }
      // Prevent constant pool checks happening by setting the next check to
      // the biggest possible offset.
      next_buffer_check_ = kMaxInt;
    }
  }

  // Resume constant pool emission. Need to be called as many time as
  // StartBlockConstPool to have an effect.
  void EndBlockConstPool() {
    if (--const_pool_blocked_nesting_ == 0) {
      // Check the constant pool hasn't been blocked for too long.
      ASSERT((num_pending_reloc_info_ == 0) ||
             (pc_offset() < (first_const_pool_use_ + kMaxDistToPool)));
      // Two cases:
      //  * no_const_pool_before_ >= next_buffer_check_ and the emission is
      //    still blocked
      //  * no_const_pool_before_ < next_buffer_check_ and the next emit will
      //    trigger a check.
      next_buffer_check_ = no_const_pool_before_;
    }
  }

  bool is_const_pool_blocked() const {
    return (const_pool_blocked_nesting_ > 0) ||
           (pc_offset() < no_const_pool_before_);
  }

 private:
  // code generation wrappers
  void branch(Label* L, Register rtmp, branch_type type, Label::Distance distance = Label::kFar);
  void branch(int offset, Register rtmp, branch_type type, Label::Distance distance, bool patched_later);
  void conditional_branch(int offset, Register rtmp, Label::Distance distance, bool patched_later, bool type);
  void conditional_branch_pool(int offset, Register rtmp, Label::Distance distance, bool patched_later, bool type);
  void jmp(int offset, Register rtmp, Label::Distance distance, bool patched_later);
  void jmp_pool(int offset, Register rtmp, Label::Distance distance, bool patched_later);
  void jsr(int offset, Register rtmp, bool patched_later);
  void jsr_pool(int offset, Register rtmp, bool patched_later);

  void writeBranchTag(int nop_count, branch_type type);
  void patchBranchOffset(int fixup_pos, uint16_t *p_pos, int is_near_linked);

  // The bound position, before this we cannot do instruction elimination.
  int last_bound_pos_;

  // Code emission
  inline void CheckBuffer();
  void GrowBuffer();

  void next(Label *L, Label::Distance distance = Label::kFar);

  // record reloc info for current pc_
  void RecordRelocInfo(RelocInfo::Mode rmode, intptr_t data = 0);
  void RecordRelocInfo_pool(RelocInfo::Mode rmode, intptr_t data = 0);

  friend class CodePatcher;
  friend class EnsureSpace;
  friend class BlockConstPoolScope;

  int next_buffer_check_;  // pc offset of next buffer check

  // code generation
  // The relocation writer's position is at least kGap bytes below the end of
  // the generated instructions. This is so that multi-instruction sequences do
  // not have to check for overflow. The same is true for writes of large
  // relocation info entries.
  static const int kGap = 32;
  RelocInfoWriter reloc_info_writer;

  // These were taken verbatim from ARM
  // TODO: find the right interval
  static const int kCheckPoolIntervalInst = 32;
  static const int kCheckPoolInterval = kCheckPoolIntervalInst * kInstrSize;


  // Average distance beetween a constant pool and the first instruction
  // accessing the constant pool. Longer distance should result in less I-cache
  // pollution.
  // In practice the distance will be smaller since constant pool emission is
  // forced after function return and sometimes after unconditional branches.
  static const int kAvgDistToPool = kMaxDistToPool - kCheckPoolInterval;

  // Emission of the constant pool may be blocked in some code sequences.
  int const_pool_blocked_nesting_;  // Block emission if this is not zero.
  int no_const_pool_before_;  // Block emission before this pc offset.
  bool emiting_const_pool_;  // True while emiting the constant pool

  // the buffer of pending relocation info
  RelocInfo pending_reloc_info_[kMaxNumPendingRelocInfo];

  // number of pending reloc info entries in the buffer
  int num_pending_reloc_info_;

  // Record the first and last positions of constants
  // In fact the distance between both is bounded
  int first_const_pool_use_;
  int last_const_pool_use_;

  PositionsRecorder positions_recorder_;

  // Transient impl helper
  bool constant_pool_pool_;

  friend class PositionsRecorder;
  friend class MacroAssembler;

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
