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

#include "v8.h"

#if defined(V8_TARGET_ARCH_SH4)

#include "disasm.h"
#include "constant-sh4.h"


// Define this to use the disassembler in opcodes-disasm-sh4.cc.
// Due to GPL licencing of the opcodes-disasm-sh4.c file, this solution
// is temporary. We should generate the decoder from the machine desc.
#define USE_KERNEL_DISASM 1

//------------------------------------------------------------------------------


#ifndef USE_KERNEL_DISASM

namespace v8 {
namespace internal {


//------------------------------------------------------------------------------

// Decoder decodes and disassembles instructions into an output buffer.
// It uses the converter to convert register names and call destinations into
// more informative description.
class Decoder {
 public:
  Decoder(Vector<char> out_buffer)
    : out_buffer_(out_buffer),
      out_buffer_pos_(0) {
    out_buffer_[out_buffer_pos_] = '\0';
  }

  ~Decoder() {}

  // Writes one disassembled instruction into 'buffer' (0-terminated).
  // Returns the length of the disassembled machine instruction in bytes.
  int InstructionDecode(byte* instruction);

 private:
  // Bottleneck functions to print into the out_buffer.
  void PrintChar(const char ch);
  void Print(const char* str);

  // Print a formated stream.
  void Format(const char *format, ...);

  // All Decode... function are decoding the set of instructions
  // from the tables defined in thhe architecture manual.
  // From Architecture Manual ADCS 7182230G SH-4 CPU Core Architecture
  // Revision: 30 April 2003
  // Section 7.3, p. 149: Instruciton set summary
  
  // Decode instructions from Table 39, p.150
  int DecodeTable39(Instruction* instr);

  Vector<char> out_buffer_;
  int out_buffer_pos_;

  DISALLOW_COPY_AND_ASSIGN(Decoder);
};



#define FMT_T39_1 1
#define FMT_T39_1_FMT "\t%s\t#%d,%s"
#define FMT_T39_1_i8(instr) (instr->SBits(7,0))
#define FMT_T39_1_n(instr) (instr->Bits(11,8))
#define FMT_T39_1_OPC(instr) (instr->Bits(15,12))
#define FMT_T39_1_STR "mov"

#define FMT_T39_2 2
#define FMT_T39_2_FMT "\t%s\t@(%u,pc),%s"
#define FMT_T39_2_d8(instr) (instr->Bits(7,0))
#define FMT_T39_2_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_2_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_2_STR "mov.w"

#define FMT_T39_3 3
#define FMT_T39_3_FMT FMT_T39_2_FMT
#define FMT_T39_3_d8(instr) FMT_T39_2_d8(instr)
#define FMT_T39_3_n(instr) FMT_T39_2_n(instr)
#define FMT_T39_3_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_3_STR "mov.l"

#define FMT_T39_4 4
#define FMT_T39_4_FMT "\t%s\t%s,%s"
#define FMT_T39_4_m(instr) (instr->Bits(7,4))
#define FMT_T39_4_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_4_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_4_OPC2(instr) (instr->Bits(3,0))
#define FMT_T39_4_STR "mov"

#define FMT_T39_5 5
#define FMT_T39_5_FMT "\t%s\t%s,@%s"
#define FMT_T39_5_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_5_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_5_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_5_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_5_STR "mov.b"

#define FMT_T39_6 6
#define FMT_T39_6_FMT FMT_T39_5_FMT
#define FMT_T39_6_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_6_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_6_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_6_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_6_STR "mov.w"

#define FMT_T39_7 7
#define FMT_T39_7_FMT FMT_T39_5_FMT
#define FMT_T39_7_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_7_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_7_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_7_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_7_STR "mov.l"

#define FMT_T39_8 8
#define FMT_T39_8_FMT "\t%s\t@%s,%s"
#define FMT_T39_8_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_8_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_8_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_8_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_8_STR "mov.b"

#define FMT_T39_9 9
#define FMT_T39_9_FMT FMT_T39_8_FMT
#define FMT_T39_9_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_9_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_9_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_9_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_9_STR "mov.w"

#define FMT_T39_10 10
#define FMT_T39_10_FMT FMT_T39_8_FMT
#define FMT_T39_10_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_10_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_10_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_10_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_10_STR "mov.l"

#define FMT_T39_11 11
#define FMT_T39_11_FMT "\t%s\t%s,@-%s"
#define FMT_T39_11_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_11_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_11_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_11_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_11_STR "mov.b"

#define FMT_T39_12 12
#define FMT_T39_12_FMT FMT_T39_11_FMT
#define FMT_T39_12_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_12_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_12_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_12_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_12_STR "mov.w"

#define FMT_T39_13 13
#define FMT_T39_13_FMT FMT_T39_11_FMT
#define FMT_T39_13_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_13_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_13_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_13_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_13_STR "mov.l"

#define FMT_T39_14 14
#define FMT_T39_14_FMT "\t%s\t@%s+,%s"
#define FMT_T39_14_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_14_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_14_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_14_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_14_STR "mov.b"

#define FMT_T39_15 15
#define FMT_T39_15_FMT FMT_T39_14_FMT
#define FMT_T39_15_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_15_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_15_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_15_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_15_STR "mov.w"

#define FMT_T39_16 16
#define FMT_T39_16_FMT FMT_T39_14_FMT
#define FMT_T39_16_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_16_n(instr) FMT_T39_1_n(instr)
#define FMT_T39_16_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_16_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_16_STR "mov.l"

#define FMT_T39_17 17
#define FMT_T39_17_FMT "\t%s\tr0,@(%u,%s)"
#define FMT_T39_17_d(instr) (instr->Bits(3,0))
#define FMT_T39_17_n(instr) (instr->Bits(7,4))
#define FMT_T39_17_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_17_OPC3(instr) (instr->Bits(11,8))
#define FMT_T39_17_STR "mov.b"

#define FMT_T39_18 18
#define FMT_T39_18_FMT FMT_T39_17_FMT
#define FMT_T39_18_d(instr) FMT_T39_17_d(instr)
#define FMT_T39_18_n(instr) FMT_T39_17_n(instr)
#define FMT_T39_18_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_18_OPC3(instr) FMT_T39_17_OPC3(instr)
#define FMT_T39_18_STR "mov.w"

#define FMT_T39_19 19
#define FMT_T39_19_FMT "\t%s\t%s,@(%u,%s)"
#define FMT_T39_19_d(instr) FMT_T39_17_d(instr)
#define FMT_T39_19_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_19_n(instr)  FMT_T39_1_n(instr)
#define FMT_T39_19_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_19_STR "mov.l"

#define FMT_T39_20 20
#define FMT_T39_20_FMT "\t%s\t@(%u,%s),r0"
#define FMT_T39_20_d(instr) FMT_T39_17_d(instr)
#define FMT_T39_20_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_20_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_20_OPC3(instr) FMT_T39_17_OPC3(instr)
#define FMT_T39_20_STR "mov.b"

#define FMT_T39_21 21
#define FMT_T39_21_FMT FMT_T39_20_FMT
#define FMT_T39_21_d(instr) FMT_T39_17_d(instr)
#define FMT_T39_21_m(instr) FMT_T39_20_m(instr)
#define FMT_T39_21_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_21_OPC3(instr) FMT_T39_17_OPC3(instr)
#define FMT_T39_21_STR "mov.w"

#define FMT_T39_22 22
#define FMT_T39_22_FMT "\t%s\t@(%u,%s),%s"
#define FMT_T39_22_d(instr) FMT_T39_17_d(instr)
#define FMT_T39_22_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_22_n(instr)  FMT_T39_1_n(instr)
#define FMT_T39_22_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_22_STR "mov.l"

#define FMT_T39_23 23
#define FMT_T39_23_FMT "\t%s\t%s,@(r0,%s)"
#define FMT_T39_23_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_23_n(instr)  FMT_T39_1_n(instr)
#define FMT_T39_23_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_23_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_23_STR "mov.b"

#define FMT_T39_24 24
#define FMT_T39_24_FMT FMT_T39_23_FMT
#define FMT_T39_24_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_24_n(instr)  FMT_T39_1_n(instr)
#define FMT_T39_24_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_24_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_24_STR "mov.w"

#define FMT_T39_25 25
#define FMT_T39_25_FMT FMT_T39_23_FMT
#define FMT_T39_25_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_25_n(instr)  FMT_T39_1_n(instr)
#define FMT_T39_25_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_25_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_25_STR "mov.l"


#define FMT_T39_26 26
#define FMT_T39_26_FMT "\t%s\t@(r0,%s),%s"
#define FMT_T39_26_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_26_n(instr)  FMT_T39_1_n(instr)
#define FMT_T39_26_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_26_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_26_STR "mov.b"

#define FMT_T39_27 27
#define FMT_T39_27_FMT FMT_T39_26_FMT
#define FMT_T39_27_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_27_n(instr)  FMT_T39_1_n(instr)
#define FMT_T39_27_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_27_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_27_STR "mov.w"

#define FMT_T39_28 28
#define FMT_T39_28_FMT FMT_T39_23_FMT
#define FMT_T39_28_m(instr) FMT_T39_4_m(instr)
#define FMT_T39_28_n(instr)  FMT_T39_1_n(instr)
#define FMT_T39_28_OPC(instr) FMT_T39_1_OPC(instr)
#define FMT_T39_28_OPC2(instr) FMT_T39_4_OPC2(instr)
#define FMT_T39_28_STR "mov.l"


enum OPC {
  FMT_T39_1_OPC_MOV = 0xe,
  FMT_T39_2_OPC_MOV_W = 0x9,
  FMT_T39_3_OPC_MOV_L = 0xd,
  FMT_T39_4_OPC_MOV = 0x6,
  FMT_T39_5_OPC_MOV_B = 0x2,
  FMT_T39_17_OPC_MOV_B = 0x8,
  FMT_T39_19_OPC_MOV_L = 0x1,
  FMT_T39_22_OPC_MOV_L = 0x5,
  FMT_T39_23_OPC_MOV_B = 0x0
};

enum OPC2 {
  FMT_T39_4_OPC2_MOV = 0x3,
  FMT_T39_5_OPC2_MOV_B = 0x0,
  FMT_T39_6_OPC2_MOV_W = 0x1,
  FMT_T39_7_OPC2_MOV_L = 0x2,
  FMT_T39_8_OPC2_MOV_B = 0x0,
  FMT_T39_9_OPC2_MOV_W = 0x1,
  FMT_T39_10_OPC2_MOV_L = 0x2,
  FMT_T39_11_OPC2_MOV_B = 0x4,
  FMT_T39_12_OPC2_MOV_W = 0x5,
  FMT_T39_13_OPC2_MOV_L = 0x6,
  FMT_T39_14_OPC2_MOV_B = 0x4,
  FMT_T39_15_OPC2_MOV_W = 0x5,
  FMT_T39_16_OPC2_MOV_L = 0x6,
  FMT_T39_23_OPC2_MOV_B = 0x4,
  FMT_T39_24_OPC2_MOV_W = 0x5,
  FMT_T39_25_OPC2_MOV_L = 0x6,
  FMT_T39_26_OPC2_MOV_B = 0xc,
  FMT_T39_27_OPC2_MOV_W = 0xd,
  FMT_T39_28_OPC2_MOV_L = 0xe
};

enum OPC3 {
  FMT_T39_17_OPC3_MOV_B = 0x0,
  FMT_T39_18_OPC3_MOV_W = 0x1,
  FMT_T39_20_OPC3_MOV_B = 0x4,
  FMT_T39_21_OPC3_MOV_W = 0x5
};

int Decoder::DecodeTable39(Instruction *instr) {
  switch(FMT_T39_1_OPC(instr)) {
#ifdef FMT_T39_1
  case FMT_T39_1_OPC_MOV:
    Format(FMT_T39_1_FMT, FMT_T39_1_STR,
	   FMT_T39_1_i8(instr),
	   Registers::Name(FMT_T39_1_n(instr))
	   );
    break;
#endif
#ifdef FMT_T39_2
  case FMT_T39_2_OPC_MOV_W:
    Format(FMT_T39_2_FMT, FMT_T39_2_STR, 
	   FMT_T39_2_d8(instr),
	   Registers::Name(FMT_T39_2_n(instr))
	   );
    break;
#endif
#ifdef FMT_T39_3
  case FMT_T39_3_OPC_MOV_L:
    Format(FMT_T39_3_FMT, FMT_T39_3_STR, 
	   FMT_T39_3_d8(instr),
	   Registers::Name(FMT_T39_3_n(instr))
	   );
    break;
#endif
  case FMT_T39_4_OPC_MOV:
    switch(FMT_T39_4_OPC2(instr)) {
#ifdef FMT_T39_4
    case FMT_T39_4_OPC2_MOV:
      Format(FMT_T39_4_FMT, FMT_T39_4_STR, 
	     Registers::Name(FMT_T39_4_m(instr)),
	     Registers::Name(FMT_T39_4_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_8
    case FMT_T39_8_OPC2_MOV_B:
      Format(FMT_T39_8_FMT, FMT_T39_8_STR, 
	     Registers::Name(FMT_T39_8_m(instr)),
	     Registers::Name(FMT_T39_8_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_9
    case FMT_T39_9_OPC2_MOV_W:
      Format(FMT_T39_9_FMT, FMT_T39_9_STR, 
	     Registers::Name(FMT_T39_9_m(instr)),
	     Registers::Name(FMT_T39_9_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_10
    case FMT_T39_10_OPC2_MOV_L:
      Format(FMT_T39_10_FMT, FMT_T39_10_STR, 
	     Registers::Name(FMT_T39_10_m(instr)),
	     Registers::Name(FMT_T39_10_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_14
    case FMT_T39_14_OPC2_MOV_B:
      Format(FMT_T39_14_FMT, FMT_T39_14_STR, 
	     Registers::Name(FMT_T39_14_m(instr)),
	     Registers::Name(FMT_T39_14_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_15
    case FMT_T39_15_OPC2_MOV_W:
      Format(FMT_T39_15_FMT, FMT_T39_15_STR, 
	     Registers::Name(FMT_T39_15_m(instr)),
	     Registers::Name(FMT_T39_15_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_16
    case FMT_T39_16_OPC2_MOV_L:
      Format(FMT_T39_16_FMT, FMT_T39_16_STR, 
	     Registers::Name(FMT_T39_16_m(instr)),
	     Registers::Name(FMT_T39_16_n(instr))
	     );
      break;
#endif
    default:
      return 0;
    }
    break;
  case FMT_T39_5_OPC_MOV_B:
    switch(FMT_T39_5_OPC2(instr)) {
#ifdef FMT_T39_5
    case FMT_T39_5_OPC2_MOV_B:
      Format(FMT_T39_5_FMT, FMT_T39_5_STR, 
	     Registers::Name(FMT_T39_5_m(instr)),
	     Registers::Name(FMT_T39_5_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_6
    case FMT_T39_6_OPC2_MOV_W:
      Format(FMT_T39_6_FMT, FMT_T39_6_STR, 
	     Registers::Name(FMT_T39_6_m(instr)),
	     Registers::Name(FMT_T39_6_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_7
    case FMT_T39_7_OPC2_MOV_L:
      Format(FMT_T39_7_FMT, FMT_T39_7_STR, 
	     Registers::Name(FMT_T39_7_m(instr)),
	     Registers::Name(FMT_T39_7_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_11
    case FMT_T39_11_OPC2_MOV_B:
      Format(FMT_T39_11_FMT, FMT_T39_11_STR, 
	     Registers::Name(FMT_T39_11_m(instr)),
	     Registers::Name(FMT_T39_11_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_12
    case FMT_T39_12_OPC2_MOV_W:
      Format(FMT_T39_12_FMT, FMT_T39_12_STR, 
	     Registers::Name(FMT_T39_12_m(instr)),
	     Registers::Name(FMT_T39_12_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_13
    case FMT_T39_13_OPC2_MOV_L:
      Format(FMT_T39_13_FMT, FMT_T39_13_STR, 
	     Registers::Name(FMT_T39_13_m(instr)),
	     Registers::Name(FMT_T39_13_n(instr))
	     );
      break;
#endif
    default:
      return 0;
    }
    break;
  case FMT_T39_17_OPC_MOV_B:
    switch(FMT_T39_17_OPC3(instr)) {
#ifdef FMT_T39_17
    case FMT_T39_17_OPC3_MOV_B:
      Format(FMT_T39_17_FMT, FMT_T39_17_STR, 
	     FMT_T39_17_d(instr),
	     Registers::Name(FMT_T39_17_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_18
    case FMT_T39_18_OPC3_MOV_W:
      Format(FMT_T39_18_FMT, FMT_T39_18_STR, 
	     FMT_T39_18_d(instr),
	     Registers::Name(FMT_T39_18_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_20
    case FMT_T39_20_OPC3_MOV_B:
      Format(FMT_T39_20_FMT, FMT_T39_20_STR, 
	     FMT_T39_20_d(instr),
	     Registers::Name(FMT_T39_20_m(instr))
	     );
      break;
#endif
#ifdef FMT_T39_21
    case FMT_T39_21_OPC3_MOV_W:
      Format(FMT_T39_21_FMT, FMT_T39_21_STR, 
	     FMT_T39_21_d(instr),
	     Registers::Name(FMT_T39_21_m(instr))
	     );
      break;
#endif
    default:
      return 0;
    }
    break;
#ifdef FMT_T39_19
  case FMT_T39_19_OPC_MOV_L:
    Format(FMT_T39_19_FMT, FMT_T39_19_STR, 
	   Registers::Name(FMT_T39_19_m(instr)),
	   FMT_T39_19_d(instr),
	   Registers::Name(FMT_T39_19_n(instr))
	   );
    break;
#endif
#ifdef FMT_T39_22
  case FMT_T39_22_OPC_MOV_L:
    Format(FMT_T39_22_FMT, FMT_T39_22_STR,
	   FMT_T39_22_d(instr),
	   Registers::Name(FMT_T39_22_m(instr)),
	   Registers::Name(FMT_T39_22_n(instr))
	   );
    break;
#endif
  case FMT_T39_23_OPC_MOV_B:
    switch(FMT_T39_23_OPC2(instr)) {
#ifdef FMT_T39_23
    case FMT_T39_23_OPC2_MOV_B:
      Format(FMT_T39_23_FMT, FMT_T39_23_STR,
	     Registers::Name(FMT_T39_23_m(instr)),
	     Registers::Name(FMT_T39_23_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_24
    case FMT_T39_24_OPC2_MOV_W:
      Format(FMT_T39_24_FMT, FMT_T39_24_STR,
	     Registers::Name(FMT_T39_24_m(instr)),
	     Registers::Name(FMT_T39_24_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_25
    case FMT_T39_25_OPC2_MOV_L:
      Format(FMT_T39_25_FMT, FMT_T39_25_STR,
	     Registers::Name(FMT_T39_25_m(instr)),
	     Registers::Name(FMT_T39_25_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_26
    case FMT_T39_26_OPC2_MOV_B:
      Format(FMT_T39_26_FMT, FMT_T39_26_STR,
	     Registers::Name(FMT_T39_26_m(instr)),
	     Registers::Name(FMT_T39_26_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_27
    case FMT_T39_27_OPC2_MOV_W:
      Format(FMT_T39_27_FMT, FMT_T39_27_STR,
	     Registers::Name(FMT_T39_27_m(instr)),
	     Registers::Name(FMT_T39_27_n(instr))
	     );
      break;
#endif
#ifdef FMT_T39_28
    case FMT_T39_28_OPC2_MOV_L:
      Format(FMT_T39_28_FMT, FMT_T39_28_STR,
	     Registers::Name(FMT_T39_28_m(instr)),
	     Registers::Name(FMT_T39_28_n(instr))
	     );
      break;
#endif
    default:
      return 0;
    }
  default:
    return 0;
  }
  return 1;
}
  
// Append the ch to the output buffer.
void Decoder::PrintChar(const char ch) {
  out_buffer_[out_buffer_pos_++] = ch;
}


// Append the str to the output buffer.
void Decoder::Print(const char* str) {
  char cur = *str++;
  while (cur != '\0' && (out_buffer_pos_ < (out_buffer_.length() - 1))) {
    PrintChar(cur);
    cur = *str++;
  }
  out_buffer_[out_buffer_pos_] = 0;
}


void Decoder::Format(const char *format, ...) {
  va_list args;
  va_start(args, format);
  out_buffer_pos_ += OS::VSNPrintF(out_buffer_ + out_buffer_pos_, format, args);
  va_end(args);
}

int Decoder::InstructionDecode(byte* instr_ptr) {
  Instruction* instr = Instruction::At(instr_ptr);
  uint16_t bits = (uint16_t)(instr->InstructionBits());
  // Print raw instruction bytes.
  out_buffer_pos_ += OS::SNPrintF(out_buffer_ + out_buffer_pos_,
                                  "%02x %02x     ", 
				  (bits & 0xff), ((bits>>8) & 0xff)
                                  );
  DecodeTable39(instr);
  return Instruction::kInstrSize;
}


} }  // namespace v8::internal


#endif /* !USE_KERNEL_DISASM */


#ifdef USE_KERNEL_DISASM


namespace v8 {
namespace internal {


//------------------------------------------------------------------------------

#define V8_IN_TYPE_DECLS 1
#include "opcodes-disasm-sh4.c"
#undef  V8_IN_TYPE_DECLS

class Decoder {
 public:
  Decoder(Vector<char> out_buffer)
    : out_buffer_(out_buffer),
      out_buffer_pos_(0),
      last_pool_offset_(0),
      last_pool_size_(0) {
    out_buffer_[out_buffer_pos_] = '\0';
  }

  ~Decoder() {}

  // Writes one disassembled instruction into 'buffer' (0-terminated).
  // Returns the length of the disassembled machine instruction in bytes.
  int InstructionDecode(byte* instruction);

  // Returns the offset from the current PC (before the instruction)
  // and size of the reference if a PC relative dereference was decoded.
  int LastPoolSize();
  int LastPoolReference();

 private:
  // Print a formated stream.
  void Format(const char *format, ...);

  // This method is implemented in opcodes-disasm-sh4.c
  void print_sh_insn(uint32_t memaddr, uint16_t insn);

  Vector<char> out_buffer_;
  int out_buffer_pos_;
  int last_pool_offset_;
  int last_pool_size_;

  DISALLOW_COPY_AND_ASSIGN(Decoder);
};

#define printk Format
#define __get_user(val,ptr) \
  do { last_pool_offset_ = (u32)(ptr) - memaddr;	\
    last_pool_size_ = sizeof(*(ptr));			\
    ((val) = *(ptr)); } while(0)
#define u16 uint16_t
#define u32 uint32_t
#include "opcodes-disasm-sh4.c"
#undef u16
#undef u32
#undef __get_user
#undef printk


void Decoder::Format(const char *format, ...) {
  va_list args;
  va_start(args, format);
  out_buffer_pos_ += OS::VSNPrintF(out_buffer_ + out_buffer_pos_, format, args);
  va_end(args);
}


int Decoder::InstructionDecode(byte* instr_ptr) {
  uint16_t bits = *reinterpret_cast<const uint16_t *>(instr_ptr);
  unsigned long pc = (unsigned long)(instr_ptr);
  // Print raw instruction bytes.
  out_buffer_pos_ += OS::SNPrintF(out_buffer_ + out_buffer_pos_,
                                  "%02x %02x     ", 
				  (bits & 0xff), ((bits>>8) & 0xff)
                                  );
  print_sh_insn(pc, bits);
  return Instruction::kInstrSize;
}

int Decoder::LastPoolReference() {
  return last_pool_offset_;
}

int Decoder::LastPoolSize() {
  return last_pool_size_;
}

} }  // namespace v8::internal


#endif /* USE_KERNEL_DISASM */


//------------------------------------------------------------------------------


namespace disasm {

const char* NameConverter::NameOfAddress(byte* addr) const {
  v8::internal::OS::SNPrintF(tmp_buffer_, "%p", addr);
  return tmp_buffer_.start();
}


const char* NameConverter::NameOfConstant(byte* addr) const {
  return NameOfAddress(addr);
}


const char* NameConverter::NameOfCPURegister(int reg) const {
  return v8::internal::Registers::Name(reg);
}


const char* NameConverter::NameOfByteCPURegister(int reg) const {
  UNREACHABLE();  // ARM does not have the concept of a byte register
  return "nobytereg";
}


const char* NameConverter::NameOfXMMRegister(int reg) const {
  UNREACHABLE();  // ARM does not have any XMM registers
  return "noxmmreg";
}


const char* NameConverter::NameInCode(byte* addr) const {
  // The default name converter is called for unknown code. So we will not try
  // to access any memory.
  return "";
}


//------------------------------------------------------------------------------

class DisassemblerSH4: public DisassemblerInterface {
public:
  explicit DisassemblerSH4() : start_pc_(0) {
    memset(pools_locs_, 9, sizeof(pools_locs_));
  }

  virtual ~DisassemblerSH4() {}

  virtual int InstructionDecode(v8::internal::Vector<char> buffer,
				byte* instruction);

  virtual int ConstantPoolSizeAt(byte* instruction);

private:
  // Management of pool references.
  void SetPoolAt(int offset, int size);
  void ClearPoolAt(int offset, int size);
  int HasPoolAt(int offset, int size);

  uint32_t start_pc_;
  static const int pools_locs_count_ = 1024;
  uint32_t pools_locs_[pools_locs_count_/sizeof(uint32_t)];
};


int DisassemblerSH4::InstructionDecode(v8::internal::Vector<char> buffer,
				       byte* instruction) {
  if (start_pc_ == 0)
    start_pc_ = (uint32_t)instruction;
  v8::internal::Decoder d(buffer);
  int n = d.InstructionDecode(instruction);
  int offset = d.LastPoolReference();
  int size = d.LastPoolSize();
  if (offset > 0)
    SetPoolAt((uint32_t)instruction - start_pc_ + offset, size);
  return n;
}


int DisassemblerSH4::ConstantPoolSizeAt(byte* instruction) {
  // Constant pool size is a multiple of 4 from caller point of view,
  // we make this a post condition.
  // Though SH4 does not support misaligned pools, hence we
  // proceed by steps of 2.
  int size = 0;
  if (start_pc_ == 0)
    return -1;
  for (int offset = (uint32_t)instruction - start_pc_;
       HasPoolAt(offset, 2);
       offset += 2) {
    size += 2;
    ClearPoolAt(offset, 2);
  }
  ASSERT(size % 4 == 0);
  return size/4 - 1;
}


void DisassemblerSH4::SetPoolAt(int offset, int size) {
  ASSERT(size >= 1 && size <= 4);
  ASSERT(offset/4 == (offset+size-1)/4);
  int elt = (offset % pools_locs_count_)/(sizeof(*pools_locs_)*8);
  int pos = (offset % pools_locs_count_)%(sizeof(*pools_locs_)*8);
  uint32_t mask = ((uint32_t)1<<size)-1;
  pools_locs_[elt] |= mask<<pos;
}


void DisassemblerSH4::ClearPoolAt(int offset, int size) {
  ASSERT(size >= 1 && size <= 4);
  ASSERT(offset/4 == (offset+size-1)/4);
  int elt = (offset % pools_locs_count_)/(sizeof(*pools_locs_)*8);
  int pos = (offset % pools_locs_count_)%(sizeof(*pools_locs_)*8);
  uint32_t mask = ((uint32_t)1<<size)-1;
  pools_locs_[elt] &= ~(mask<<pos);
}


int DisassemblerSH4::HasPoolAt(int offset, int size) {
  ASSERT(size >= 1 && size <= 4);
  ASSERT(offset/4 == (offset+size-1)/4);
  int elt = (offset % pools_locs_count_)/(sizeof(*pools_locs_)*8);
  int pos = (offset % pools_locs_count_)%(sizeof(*pools_locs_)*8);
  uint32_t mask = ((uint32_t)1<<size)-1;
  return (pools_locs_[elt] & (mask<<pos)) == (mask<<pos);
}


void Disassembler::Disassemble(FILE* f, byte* begin, byte* end) {
  NameConverter converter;
  DisassemblerInterface *d = DisassemblerFactory::NewDisassembler(converter);
  for (byte* pc = begin; pc < end;) {
    v8::internal::EmbeddedVector<char, 128> buffer;
    buffer[0] = '\0';
    byte* prev_pc = pc;
    pc += d->InstructionDecode(buffer, pc);
    fprintf(f, "%p    %02x %02x      %s\n",
            prev_pc, *reinterpret_cast<const uint8_t*>(prev_pc), *reinterpret_cast<const uint8_t*>(prev_pc+1), buffer.start());
  }
  delete d;
}


/*static*/ DisassemblerInterface *
DisassemblerFactory::NewDisassembler(const NameConverter& converter) {
  return new DisassemblerSH4::DisassemblerSH4();
}

}  // namespace disasm

#endif  // V8_TARGET_ARCH_SH4
