// Copyright 2012 the V8 project authors. All rights reserved.
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


// Declares a Simulator for SH4 instructions if we are not generating a native
// SH4 binary. This Simulator allows us to run and debug SH4 code generation on
// regular desktop machines.
// V8 calls into generated code by "calling" the CALL_GENERATED_CODE macro,
// which will start execution in the Simulator or forwards to the real entry
// on a SH4 HW platform.

#ifndef V8_SH4_SIMULATOR_SH4_H_
#define V8_SH4_SIMULATOR_SH4_H_

#include "allocation.h"

#if !defined(USE_SIMULATOR)
// Running without a simulator on a native sh4 platform.

namespace v8 {
namespace internal {

// When running without a simulator we call the entry directly.
#define CALL_GENERATED_CODE(entry, p0, p1, p2, p3, p4) \
  (entry(p0, p1, p2, p3, p4))

typedef int (*sh4_regexp_matcher)(String*, int, const byte*, const byte*,
                                  void*, int*, int, Address, int, Isolate*);


// Call the generated regexp code directly. The code at the entry address
// should act as a function matching the type sh4_regexp_matcher.
// The fifth argument is a dummy that reserves the space used for
// the return address added by the ExitFrame in native calls.
#define CALL_GENERATED_REGEXP_CODE(entry, p0, p1, p2, p3, p4, p5, p6, p7, p8) \
  (FUNCTION_CAST<sh4_regexp_matcher>(entry)(                              \
      p0, p1, p2, p3, NULL, p4, p5, p6, p7, p8))

#define TRY_CATCH_FROM_ADDRESS(try_catch_address) \
  reinterpret_cast<TryCatch*>(try_catch_address)

// The stack limit beyond which we will throw stack overflow errors in
// generated code. Because generated code on sh4 uses the C stack, we
// just use the C stack limit.
class SimulatorStack : public v8::internal::AllStatic {
 public:
  static inline uintptr_t JsLimitFromCLimit(v8::internal::Isolate* isolate,
                                            uintptr_t c_limit) {
    USE(isolate);
    return c_limit;
  }

  static inline uintptr_t RegisterCTryCatch(uintptr_t try_catch_address) {
    return try_catch_address;
  }

  static inline void UnregisterCTryCatch() { }
};

} }  // namespace v8::internal

#else  // !defined(USE_SIMULATOR)
// Running with a simulator.

#include "constants-sh4.h"
#include "hashmap.h"
#include "assembler.h"

namespace v8 {
namespace internal {

class CachePage {
 public:
  static const int LINE_VALID = 0;
  static const int LINE_INVALID = 1;

  static const int kPageShift = 12;
  static const int kPageSize = 1 << kPageShift;
  static const int kPageMask = kPageSize - 1;
  static const int kLineShift = 2;  // The cache line is only 4 bytes right now.
  static const int kLineLength = 1 << kLineShift;
  static const int kLineMask = kLineLength - 1;

  CachePage() {
    memset(&validity_map_, LINE_INVALID, sizeof(validity_map_));
  }

  char* ValidityByte(int offset) {
    return &validity_map_[offset >> kLineShift];
  }

  char* CachedData(int offset) {
    return &data_[offset];
  }

 private:
  char data_[kPageSize];   // The cached data.
  static const int kValidityMapSize = kPageSize >> kLineShift;
  char validity_map_[kValidityMapSize];  // One byte per line.
};


class Simulator {
 public:
  friend class Sh4Debugger;
enum Register {
    no_reg = -1,
    r0 = 0, r1, r2, r3, r4, r5, r6, r7,
    r8, r9, r10, r11, r12, r13, r14, r15,
    num_registers,
    fp = 14,
    sp = 15,
    dr0 = 0, dr2 = 2, dr4 = 4, dr6 = 6, dr8 = 8, dr10 = 10, dr12 = 12, dr14 = 14,
    num_dregisters = 14,
    fr0 = 0, fr1 = 1, fr2 = 2, fr3 = 3, fr4 = 4, fr5 = 5, fr6 = 6, fr7 = 7, fr8 = 8,
    fr9 = 9, fr10 = 10, fr11 = 11, fr12 = 12, fr13 = 13, fr14 = 14, fr15 = 15,
    num_fregisters = 16
  };

  // Status registers
  enum {
    mach  = 0,
    macl  = 1,
    pr    = 2,
    fpul  = 5,
    fpscr = 6
  };

  explicit Simulator(Isolate* isolate);
  ~Simulator();

  // The currently executing Simulator instance. Potentially there can be one
  // for each native thread.
  static Simulator* current(v8::internal::Isolate* isolate);

  // Accessors for register state. Reading the pc value adheres to the SH4
  // architecture specification
  void set_register(int reg, int32_t value);
  int32_t get_register(int reg) const;

  void set_fregister(int reg, int32_t value);
  int32_t get_fregister(int reg) const;

  void set_dregister(int reg, double value);
  double get_dregister(int reg) const;

  // Special case of set_register and get_register to access the raw PC value.
  void set_pc(int32_t value);
  int32_t get_pc() const;

  Address get_sp() {
    return reinterpret_cast<Address>(static_cast<intptr_t>(get_register(sp)));
  }

  // Get/set the status registers
  void set_sregister(int num, int32_t value);
  int32_t get_sregister(int num);

  // Get/Set status registers
  bool get_t_flag()             { return t_flag_; }
  void set_t_flag(bool value)   { t_flag_ = value; }
  bool get_s_flag()             { return s_flag_; }
  void set_s_flag(bool value)   { s_flag_ = value; }
  bool get_q_flag()             { return q_flag_; }
  void set_q_flag(bool value)   { q_flag_ = value; }
  bool get_m_flag()             { return m_flag_; }
  void set_m_flag(bool value)   { m_flag_ = value; }

  // helpers for some operations
  void mul_helper(int sign, unsigned int rm, unsigned int rn);
  void div1 (int iRn2, int iRn1);

  // Double helpers
  void dsub(int n, int m);
  void dadd(int n, int m);
  void dmul(int n, int m);
  void ddiv(int n, int m);
  void dsqrt(int n);
  void dabs(int n);
  void dneg(int n);

  // Accessor to the internal simulator stack area.
  uintptr_t StackLimit() const;

  // Executes ARM instructions until the PC reaches end_sim_pc.
  void Execute();

  // Call on program start.
  static void Initialize(Isolate* isolate);

  // V8 generally calls into generated JS code with 5 parameters and into
  // generated RegExp code with 7 parameters. This is a convenience function,
  // which sets up the simulator state and grabs the result on return.
  int32_t Call(byte* entry, int int_args, int double_args, ...);

  // Push an address onto the JS stack.
  uintptr_t PushAddress(uintptr_t address);

  // Pop an address from the JS stack.
  uintptr_t PopAddress();

  // Debugger input.
  void set_last_debugger_input(char* input);
  char* last_debugger_input() { return last_debugger_input_; }

  // ICache checking.
  static void FlushICache(v8::internal::HashMap* i_cache, void* start,
                          size_t size);

  // Returns true if pc register contains one of the 'special_values' defined
  // below (bad_pr, end_sim_pc).
  bool has_bad_pc() const;

 private:
  enum special_values {
    // Known bad pc value to ensure that the simulator does not execute
    // without being properly setup.
    bad_pr = -1,
    // A pc value used to signal the simulator to stop execution.  Generally
    // the lr is set to this value on transition from native C code to
    // simulated execution, so that the simulator can "return" to the native
    // C code.
    end_sim_pc = -2
  };

  struct {
    int32_t mach;
    int32_t macl;
    int32_t pr;
    int32_t fpul;
    int32_t fpscr;
  } sregs_;

  void SoftwareInterrupt(Instruction* instr, int signal);

  // Read and write memory.
  inline uint8_t ReadBU(int32_t addr, Instruction* instr);
  inline int8_t ReadB(int32_t addr, Instruction* instr);
  inline void WriteBU(int32_t addr, uint8_t value, Instruction* instr);
  inline void WriteB(int32_t addr, int8_t value, Instruction* instr);

  inline uint16_t ReadHU(int32_t addr, Instruction* instr);
  inline int16_t ReadH(int32_t addr, Instruction* instr);
  // Note: Overloaded on the sign of the value.
  inline void WriteHU(int32_t addr, uint16_t value, Instruction* instr);
  inline void WriteH(int32_t addr, int16_t value, Instruction* instr);

  inline int ReadW(int32_t addr, Instruction* instr);
  inline void WriteW(int32_t addr, int value, Instruction* instr);

  bool IsInvalidStackAccess(uintptr_t addr, uintptr_t stack);

  // Executes one instruction.
  void InstructionDecode(Instruction* instr);
  void InstructionDecodeDelaySlot(Instruction* instr);

  // ICache.
  static void CheckICache(v8::internal::HashMap* i_cache, Instruction* instr);
  static void FlushOnePage(v8::internal::HashMap* i_cache, intptr_t start,
                           int size);
  static CachePage* GetCachePage(v8::internal::HashMap* i_cache, void* page);

  // Runtime call support.
  static void* RedirectExternalReference(
      void* external_function,
      v8::internal::ExternalReference::Type type);

  void CallInternal(byte* entry);

  // Architecture state.
  int32_t registers_[16];

  union {
    struct {
      uint32_t high;
      uint32_t low;
    } f;
    double d;
  } fpu_registers_[8];
  
  int32_t pc_;
  bool t_flag_;
  bool s_flag_;
  bool q_flag_;
  bool m_flag_;

  // Stack
  byte* stack_;
  static const intptr_t stack_protection_size_ = KB;
  intptr_t stack_size_;
  byte* stack_limit_;

  // Simulator support.
  bool pc_modified_;
  int icount_;

  // delay slot
  bool in_delay_slot_;

  // Debugger input.
  char* last_debugger_input_;

  // Icache simulation
  v8::internal::HashMap* i_cache_;

  // Registered breakpoints.
  Instruction* break_pc_;
  Instr break_instr_;

  v8::internal::Isolate* isolate_;

  // Dump states
  bool dump_set_register;
  bool dump_get_register;
};


// When running with the simulator transition into simulated execution at this
// point.
#define CALL_GENERATED_CODE(entry, p0, p1, p2, p3, p4) \
  reinterpret_cast<Object*>(Simulator::current(Isolate::Current())->Call( \
      FUNCTION_ADDR(entry), 5, 0, p0, p1, p2, p3, p4))
#define CALL_GENERATED_FPU_CODE(entry, p0, p1) \
  reinterpret_cast<Object*>(Simulator::current(Isolate::Current())->Call( \
      FUNCTION_ADDR(entry), 4, 2, 0, 0, 0, 0, (double)p0, (double)p1))

#define CALL_GENERATED_REGEXP_CODE(entry, p0, p1, p2, p3, p4, p5, p6, p7, p8) \
  Simulator::current(Isolate::Current())->Call( \
      entry, 10, 0, p0, p1, p2, p3, NULL, p4, p5, p6, p7, p8)

#define TRY_CATCH_FROM_ADDRESS(try_catch_address)                              \
  try_catch_address == NULL ?                                                  \
      NULL : *(reinterpret_cast<TryCatch**>(try_catch_address))


// The simulator has its own stack. Thus it has a different stack limit from
// the C-based native code.  Setting the c_limit to indicate a very small
// stack cause stack overflow errors, since the simulator ignores the input.
// This is unlikely to be an issue in practice, though it might cause testing
// trouble down the line.
class SimulatorStack : public v8::internal::AllStatic {
 public:
  static inline uintptr_t JsLimitFromCLimit(v8::internal::Isolate* isolate,
                                            uintptr_t c_limit) {
    return Simulator::current(isolate)->StackLimit();
  }

  static inline uintptr_t RegisterCTryCatch(uintptr_t try_catch_address) {
    Simulator* sim = Simulator::current(Isolate::Current());
    return sim->PushAddress(try_catch_address);
  }

  static inline void UnregisterCTryCatch() {
    Simulator::current(Isolate::Current())->PopAddress();
  }
};

} }  // namespace v8::internal

#endif  // !defined(USE_SIMULATOR)
#endif  // V8_SH4_SIMULATOR_SH4_H_
