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

#ifndef V8_SH4_CONSTANTS_SH4_H_
#define V8_SH4_CONSTANTS_SH4_H_

namespace v8 {
namespace internal {

// Number of registers in normal SH4 mode.
const int kNumRegisters = 16;
const int kNumAllocatableRegisters = 8;
const int kNumFPUSingleRegisters = 16;
const int kNumFPUDoubleRegisters = 8;
const int kNumFPURegisters = kNumFPUDoubleRegisters + kNumFPUSingleRegisters;

// Marker for end of register list.
static const int kNoRegister = -1;

// -----------------------------------------------------------------------------
// Instructions encoding.

// Instr is merely used by the Assembler to distinguish 32bit integers
// representing instructions from usual 32 bit values.
// Instruction objects are pointers to 32bit values, and provide methods to
// access the various ISA fields.
typedef uint16_t Instr;

// Instruction encoding bits and masks.
enum {
  kOff8Mask  = (1 << 8) - 1
};


// -----------------------------------------------------------------------------
// Supervisor Call (svc) specific support.

// Special Software Interrupt codes when used in the presence of the SH4
// simulator.
enum SoftwareInterruptCodes {
  // illegal instruction
  kIllegalInstruction = 0,
  // unsupported instruction
  kUnsupportedInstruction = 1,
  // forbidden instruction in a delay slot
  kDelaySlot = 2,
  // transition to C code
  kCallRtRedirected = 0xf00,
  // break point
  kBreakpoint = 0xf01,
  // stop instruction
  kStoppoint = 0xf02
};


// We use the cause fields bcause they are set to 1 or 0 depending on the
// action. So they don't need to be reseted but they mist be use immediately
static const uint32_t kFPUExceptionMask          = 0x1f << 12;
static const uint32_t kFPUInexactExceptionBit    = 1 << 12;
static const uint32_t kFPUUnderflowExceptionBit  = 1 << 13;
static const uint32_t kFPUOverflowExceptionBit   = 1 << 14;
static const uint32_t kFPUDividezeroExceptionBit = 1 << 15;
static const uint32_t kFPUInvalidExceptionBit    = 1 << 16;

// FPU rounding modes.
enum FPURoundingMode {
  RN = 0,   // Round to Nearest.
  RZ = 1,   // Round towards zero.

  // Aliases.
  kRoundToNearest = RN,
  kRoundToZero = RZ
};

static const uint32_t kFPURoundingModeMask = 1;

enum CheckForInexactConversion {
  kCheckForInexactConversion,
  kDontCheckForInexactConversion
};


// -----------------------------------------------------------------------------
// Instruction abstraction.

class Instruction {
 public:
  enum {
    kInstrSize = 2,
    kInstrSizeLog2 = 1
  };

  // Helper macro to define static accessors.
  // We use the cast to char* trick to bypass the strict anti-aliasing rules.
  #define DECLARE_STATIC_TYPED_ACCESSOR(return_type, Name)                     \
    static inline return_type Name(Instr instr) {                              \
      char* temp = reinterpret_cast<char*>(&instr);                            \
      return reinterpret_cast<Instruction*>(temp)->Name();                     \
    }

  #define DECLARE_STATIC_ACCESSOR(Name) DECLARE_STATIC_TYPED_ACCESSOR(int, Name)

  // Get the raw instruction bits.
  inline Instr InstructionBits() const {
    return *reinterpret_cast<const Instr*>(this);
  }

  // Set the raw instruction bits to value.
  inline void SetInstructionBits(Instr value) {
    *reinterpret_cast<Instr*>(this) = value;
  }

  // Read one particular bit out of the instruction bits.
  inline int Bit(int nr) const {
    return (InstructionBits() >> nr) & 1;
  }

  // Read a bit field's value out of the instruction bits.
  inline int Bits(int hi, int lo) const {
    return (InstructionBits() >> lo) & ((2 << (hi - lo)) - 1);
  }

  // Read a signed bit field's value out of the instruction bits.
  inline int SBits(int hi, int lo) const {
    return (Bits(hi, lo) << (sizeof(int)*8 - hi - 1)) >>
      (sizeof(int)*8 - hi - 1);
  }

  // Read a bit field out of the instruction bits.
  inline int BitField(int hi, int lo) const {
    return InstructionBits() & (((2 << (hi - lo)) - 1) << lo);
  }

  // Static support.

  // Read one particular bit out of the instruction bits.
  static inline int Bit(Instr instr, int nr) {
    return (instr >> nr) & 1;
  }

  // Read the value of a bit field out of the instruction bits.
  static inline int Bits(Instr instr, int hi, int lo) {
    return (instr >> lo) & ((2 << (hi - lo)) - 1);
  }


  // Read the value of a signed bit field out of the instruction bits.
  inline int SBits(Instr instr, int hi, int lo) const {
    return (Bits(instr, hi, lo) << (sizeof(int)*8 - hi - 1)) >>
      (sizeof(int)*8 - hi - 1);
  }

  // Read a bit field out of the instruction bits.
  static inline int BitField(Instr instr, int hi, int lo) {
    return instr & (((2 << (hi - lo)) - 1) << lo);
  }



  // Instructions are read of out a code stream. The only way to get a
  // reference to an instruction is to convert a pointer. There is no way
  // to allocate or create instances of class Instruction.
  // Use the At(pc) function to create references to Instruction.
  static Instruction* At(byte* pc) {
    return reinterpret_cast<Instruction*>(pc);
  }


 private:
  // We need to prevent the creation of instances of class Instruction.
  DISALLOW_IMPLICIT_CONSTRUCTORS(Instruction);
};


// Helper functions for converting between register numbers and names.
class Registers {
 public:
  // Return the name of the register.
  static const char* Name(int reg);

  // Lookup the register number for the name provided.
  static int Number(const char* name);

  struct RegisterAlias {
    int reg;
    const char* name;
  };

 private:
  static const char* names_[kNumRegisters];
  static const RegisterAlias aliases_[];
};


// Helper functions for converting between FPU register numbers and names.
class FPURegisters {
 public:
  // Return the name of the register.
  static const char* Name(int reg, bool is_double);

  // Lookup the register number for the name provided.
  // Set flag pointed by is_double to true if register
  // is double-precision.
  static int Number(const char* name, bool* is_double);

 private:
  static const char* names_[kNumFPURegisters];
};



} }  // namespace v8::internal

#endif  // V8_SH4_CONSTANTS_SH4_H_
