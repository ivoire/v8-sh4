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

#include "v8.h"

#if defined(V8_TARGET_ARCH_SH4)

#include "constants-sh4.h"


namespace v8 {
namespace internal {

// These register names are defined in a way to match the native disassembler
// formatting. See for example the command "objdump -d <binary file>".
const char* Registers::names_[kNumRegisters] = {
  "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
  "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15",
};


// List of alias names which can be used when referring to SH4 registers.
const Registers::RegisterAlias Registers::aliases_[] = {
  {14, "fp"},
  {15, "sp"},
  {kNoRegister, NULL}
};


const char* Registers::Name(int reg) {
  const char* result;
  if ((0 <= reg) && (reg < kNumRegisters)) {
    result = names_[reg];
  } else {
    result = "noreg";
  }
  return result;
}


// Support for FPU registers fr0 to fr15 (dr0 to dr14).
// Note that "frN:frM" is the same as "drN/2"
const char* FPURegisters::names_[kNumFPURegisters] = {
    "fr0", "fr1", "fr2", "fr3", "fr4", "fr5", "fr6", "fr7",
    "fr8", "fr9", "fr10", "fr11", "fr12", "fr13", "fr14", "fr15",
    "dr0", "dr2", "dr4", "dr6", "dr8", "dr10", "dr12", "dr14",
};


const char* FPURegisters::Name(int reg, bool is_double) {
  ASSERT((0 <= reg) && (reg < kNumFPURegisters));
  if (is_double) {
    ASSERT(reg % 2 == 0);
    return names_[kNumFPUSingleRegisters + reg / 2];
   } else {
    return names_[reg];
  }
}


int FPURegisters::Number(const char* name, bool* is_double) {
  for (int i = 0; i < kNumFPURegisters; i++) {
    if (strcmp(names_[i], name) == 0) {
      if (i < kNumFPUSingleRegisters) {
        *is_double = false;
        return i;
      } else {
        *is_double = true;
        return (i - kNumFPUSingleRegisters) * 2;
      }
    }
  }

  // No register with the requested name found.
  return kNoRegister;
}



int Registers::Number(const char* name) {
  // Look through the canonical names.
  for (int i = 0; i < kNumRegisters; i++) {
    if (strcmp(names_[i], name) == 0) {
      return i;
    }
  }

  // Look through the alias names.
  int i = 0;
  while (aliases_[i].reg != kNoRegister) {
    if (strcmp(aliases_[i].name, name) == 0) {
      return aliases_[i].reg;
    }
    i++;
  }

  // No register with the requested name found.
  return kNoRegister;
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
