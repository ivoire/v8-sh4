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

// Mapping of ARM registers to SH4 registers.
// Include this file the first time for defining the mapping.
// Include this file a second time for undefining it.
#ifndef MAP_SH4
// First include, we define the mapping
#define MAP_SH4
#define r8 "should be cp"
#define ip sh4_r10
#define lr pr
#define r10 "should be roots"
#define r11 "Unexpected"
#define r12 "Unexpected"
#define r13 "Unexpected"
#define r14 "Unexpected"
#define r15 "Unexpected"

// Mapping from ARM VFP doubles to SH4 Doubles
// This is a name mapping allowing reuse of ARM
// code without modifying register names.
// Unfortunately ARM features 16 double registers
// whereas for SH4 we use only the 8 registers
// of the bank 0.
// Also ARM callee saved are d8-d15 while
// SH4 callee-saved are dr12-dr14 (only 2).
// Arm defines a Zero reg and a scratch reg
// both callee saved, hence we use the two
// SH4 callee saved for this.
#define d0 sh4_dr0
#define d1 sh4_dr2
#define d2 sh4_dr4
#define d3 sh4_dr6
#define d4 "Unexpected: no free register"
#define d5 "Unexpected: no free register"
#define d6 sh4_dr8
#define d7 sh4_dr10
#define d14 "Unexpected: use kDoubleZeroReg(sh4_dr12)"
#define d15 "Unexpected: use kDoubleScratchReg(sh4_dr14)"
#else
// Second include, we undefine the mapping
#undef MAP_SH4
#undef r8
#undef ip
#undef lr
#undef r10
#undef r11
#undef r12
#undef r13
#undef r14
#undef r15

#undef d0
#undef d1
#undef d2
#undef d3
#undef d4
#undef d5
#undef d6
#undef d7
#endif
