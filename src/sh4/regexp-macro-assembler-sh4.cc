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

#include "unicode.h"
#include "log.h"
#include "code-stubs.h"
#include "regexp-stack.h"
#include "macro-assembler.h"
#include "regexp-macro-assembler.h"
#include "sh4/regexp-macro-assembler-sh4.h"

namespace v8 {
namespace internal {

#include "map-sh4.h"  // For ARM -> SH4 register mapping

#ifndef V8_INTERPRETED_REGEXP
/*
 * This assembler uses the following register assignment convention
 * - r5 : Pointer to current code object (Code*) including heap object tag.
 * - r6 : Current position in input, as negative offset from end of string.
 *        Please notice that this is the byte offset, not the character offset!
 * - r7 : Currently loaded character. Must be loaded using
 *        LoadCurrentCharacter before using any of the dispatch methods.
 * - r8 : points to tip of backtrack stack
 * - r9 : End of input (points to byte after last character in input).
 * - fp : Frame pointer. Used to access arguments, local variables and
 *         RegExp registers.
 * - r10 : IP register, used by assembler. Very volatile.
 * - sp : points to tip of C stack.
 *
 * The remaining registers are free for computations.
 * Each call to a public method should retain this convention.
 *
 * The stack will have the following structure:
 *  - fp[48]  Isolate* isolate   (Address of the current isolate)
 *  - fp[44]  direct_call  (if 1, direct call from JavaScript code,
 *                          if 0, call through the runtime system).
 *  - fp[40]  stack_area_base (High end of the memory area to use as
 *                             backtracking stack).
 *  - fp[46]  int* capture_array (int[num_saved_registers_], for output).
 *  - fp[32]  secondary link/return address used by native call.
 *  --- sp when called ---
 *  - fp[28]  return address (pr).
 *  - fp[24]  old frame pointer (r14).
 *  - fp[0..20]  backup of registers r8..r13.
 *  --- frame pointer ----
 *  - fp[-4]  end of input       (Address of end of string).
 *  - fp[-8]  start of input     (Address of first character in string).
 *  - fp[-12] start index        (character index of start).
 *  - fp[-16] void* input_string (location of a handle containing the string).
 *  - fp[-20] Offset of location before start of input (effectively character
 *            position -1). Used to initialize capture registers to a
 *            non-position.
 *  - fp[-24] At start (if 1, we are starting at the start of the
 *    string, otherwise 0)
 *  - fp[-28] register 0         (Only positions must be stored in the first
 *  -         register 1          num_saved_registers_ registers)
 *  -         ...
 *  -         register num_registers-1
 *  --- sp ---
 *
 * The first num_saved_registers_ registers are initialized to point to
 * "character -1" in the string (i.e., char_size() bytes before the first
 * character of the string). The remaining registers start out as garbage.
 *
 * The data up to the return address must be placed there by the calling
 * code and the remaining arguments are passed in registers, e.g. by calling the
 * code entry as cast to a function with the signature:
 * int (*match)(String* input_string,
 *              int start_index,
 *              Address start,
 *              Address end,
 *              Address secondary_return_address,  // Only used by native call.
 *              int* capture_output_array,
 *              byte* stack_area_base,
 *              bool direct_call = false)
 * The call is performed by NativeRegExpMacroAssembler::Execute()
 * (in regexp-macro-assembler.cc) via the CALL_GENERATED_REGEXP_CODE macro
 * in sh4/simulator-sh4.h.
 * When calling as a non-direct call (i.e., from C++ code), the return address
 * area is overwritten with the LR register by the RegExp code. When doing a
 * direct call from generated code, the return address is placed there by
 * the calling code, as in a normal exit frame.
 */

#define __ ACCESS_MASM(masm_)

RegExpMacroAssemblerSH4::RegExpMacroAssemblerSH4(
    Mode mode,
    int registers_to_save)
    : masm_(new MacroAssembler(Isolate::Current(), NULL, kRegExpCodeSize)),
      mode_(mode),
      num_registers_(registers_to_save),
      num_saved_registers_(registers_to_save),
      entry_label_(),
      start_label_(),
      success_label_(),
      backtrack_label_(),
      exit_label_() {
  ASSERT_EQ(0, registers_to_save % 2);
  __ jmp(&entry_label_);   // We'll write the entry code later.
  __ bind(&start_label_);  // And then continue from here.
}


RegExpMacroAssemblerSH4::~RegExpMacroAssemblerSH4() {
  delete masm_;
  // Unuse labels in case we throw away the assembler without calling GetCode.
  entry_label_.Unuse();
  start_label_.Unuse();
  success_label_.Unuse();
  backtrack_label_.Unuse();
  exit_label_.Unuse();
  check_preempt_label_.Unuse();
  stack_overflow_label_.Unuse();
}


int RegExpMacroAssemblerSH4::stack_limit_slack()  {
  return RegExpStack::kStackLimitSlack;
}


void RegExpMacroAssemblerSH4::AdvanceCurrentPosition(int by) {
  if (by != 0) {
    __ add(current_input_offset(),
           current_input_offset(), Operand(by * char_size()));
  }
}


void RegExpMacroAssemblerSH4::AdvanceRegister(int reg, int by) {
  ASSERT(reg >= 0);
  ASSERT(reg < num_registers_);
  if (by != 0) {
    __ ldr(r0, register_location(reg));
    __ add(r0, r0, Operand(by));
    __ str(r0, register_location(reg));
  }
}


void RegExpMacroAssemblerSH4::Backtrack() {
  CheckPreemption();
  // Pop Code* offset from backtrack stack, add Code* and jump to location.
  Pop(r0);
  __ add(r0, r0, code_pointer());
  __ jmp(r0);

}


void RegExpMacroAssemblerSH4::Bind(Label* label) {
  __ bind(label);
}


void RegExpMacroAssemblerSH4::CheckCharacter(uint32_t c, Label* on_equal) {
  __ cmp(current_character(), Operand(c));
  BranchOrBacktrack(eq, on_equal);
}


void RegExpMacroAssemblerSH4::CheckCharacterGT(uc16 limit, Label* on_greater) {
  __ cmpgt(current_character(), Operand(limit));
  BranchOrBacktrack(eq, on_greater);
}


void RegExpMacroAssemblerSH4::CheckAtStart(Label* on_at_start) {
  Label not_at_start;
  // Did we start the match at the start of the string at all?
  __ ldr(r0, MemOperand(frame_pointer(), kAtStart));
  __ cmp(r0, Operand(0, RelocInfo::NONE));
  BranchOrBacktrack(eq, &not_at_start);

  // If we did, are we still at the start of the input?
  __ ldr(r1, MemOperand(frame_pointer(), kInputStart));
  __ add(r0, end_of_input_address(), current_input_offset());
  __ cmp(r0, r1);
  BranchOrBacktrack(eq, on_at_start);
  __ bind(&not_at_start);
}


void RegExpMacroAssemblerSH4::CheckNotAtStart(Label* on_not_at_start) {
  // Did we start the match at the start of the string at all?
  __ ldr(r0, MemOperand(frame_pointer(), kAtStart));
  __ cmp(r0, Operand(0, RelocInfo::NONE));
  BranchOrBacktrack(eq, on_not_at_start);
  // If we did, are we still at the start of the input?
  __ ldr(r1, MemOperand(frame_pointer(), kInputStart));
  __ add(r0, end_of_input_address(), current_input_offset());
  __ cmp(r0, r1);
  BranchOrBacktrack(ne, on_not_at_start);
}


void RegExpMacroAssemblerSH4::CheckCharacterLT(uc16 limit, Label* on_less) {
  __ cmpge(current_character(), Operand(limit));
  BranchOrBacktrack(ne, on_less);
}


void RegExpMacroAssemblerSH4::CheckCharacters(Vector<const uc16> str,
                                              int cp_offset,
                                              Label* on_failure,
                                              bool check_end_of_string) {
  if (on_failure == NULL) {
    // Instead of inlining a backtrack for each test, (re)use the global
    // backtrack target.
    on_failure = &backtrack_label_;
  }

  if (check_end_of_string) {
    // Is last character of required match inside string.
    CheckPosition(cp_offset + str.length() - 1, on_failure);
  }

  __ add(r0, end_of_input_address(), current_input_offset());
  if (cp_offset != 0) {
    int byte_offset = cp_offset * char_size();
    __ add(r0, r0, Operand(byte_offset));
  }

  // r0 : Address of characters to match against str.
  int stored_high_byte = 0;
  for (int i = 0; i < str.length(); i++) {
    if (mode_ == ASCII) {
      __ ldrb(r1, MemOperand(r0));
      __ add(r0, r0, Operand(char_size()));
      ASSERT(str[i] <= String::kMaxAsciiCharCode);
      __ cmp(r1, Operand(str[i]));
    } else {
      __ ldrh(r1, MemOperand(r0));
      __ add(r0, r0, Operand(char_size()));
      uc16 match_char = str[i];
      int match_high_byte = (match_char >> 8);
      if (match_high_byte == 0) {
        __ cmp(r1, Operand(str[i]));
      } else {
        if (match_high_byte != stored_high_byte) {
          __ mov(r2, Operand(match_high_byte));
          stored_high_byte = match_high_byte;
        }
        __ add(r3, r2, Operand(match_char & 0xff));
        __ cmp(r1, r3);
      }
    }
    BranchOrBacktrack(ne, on_failure);
  }
}


void RegExpMacroAssemblerSH4::CheckGreedyLoop(Label* on_equal) {
  __ ldr(r0, MemOperand(backtrack_stackpointer(), 0));
  __ cmp(current_input_offset(), r0);
  Label skip;
  __ b(ne, &skip, Label::kNear);
  __ add(backtrack_stackpointer(),
         backtrack_stackpointer(), Operand(kPointerSize));
  __ bind(&skip);
  BranchOrBacktrack(eq, on_equal);
}


void RegExpMacroAssemblerSH4::CheckNotBackReferenceIgnoreCase(
    int start_reg,
    Label* on_no_match) {
  Label fallthrough;
  __ ldr(r0, register_location(start_reg));  // Index of start of capture
  __ ldr(r1, register_location(start_reg + 1));  // Index of end of capture
  __ sub(r1, r1, r0);  // Length of capture.
  __ tst(r1, r1);

  // If length is zero, either the capture is empty or it is not participating.
  // In either case succeed immediately.
  __ b(eq, &fallthrough);

  // Check that there are enough characters left in the input.
  __ cmpgt(r1, current_input_offset());
  BranchOrBacktrack(ne, on_no_match);

  if (mode_ == ASCII) {
    Label success;
    Label fail;
    Label loop_check;

    // r0 - offset of start of capture
    // r1 - length of capture
    __ add(r0, r0, end_of_input_address());
    __ add(r2, end_of_input_address(), current_input_offset());
    __ add(r1, r0, r1);

    // r0 - Address of start of capture.
    // r1 - Address of end of capture
    // r2 - Address of current input position.

    Label loop;
    __ bind(&loop);
    __ ldrb(r3, MemOperand(r0)); __ add(r0, r0, Operand(char_size()));
    __ ldrb(r4, MemOperand(r2)); __ add(r2, r2, Operand(char_size()));
    __ cmp(r4, r3);
    __ b(eq, &loop_check);

    // Mismatch, try case-insensitive match (converting letters to lower-case).
    __ orr(r3, r3, Operand(0x20));  // Convert capture character to lower-case.
    __ orr(r4, r4, Operand(0x20));  // Also convert input character.
    __ cmp(r4, r3);
    __ b(ne, &fail);
    __ sub(r3, r3, Operand('a'));
    __ cmphi(r3, Operand('z' - 'a'));  // Is r3 a lowercase letter?
    __ b(eq, &fail);


    __ bind(&loop_check);
    __ cmpge(r0, r1);
    __ b(ne, &loop);
    __ jmp(&success);

    __ bind(&fail);
    BranchOrBacktrack(al, on_no_match);

    __ bind(&success);
    // Compute new value of character position after the matched part.
    __ sub(current_input_offset(), r2, end_of_input_address());
  } else {
    ASSERT(mode_ == UC16);
    int argument_count = 4;
    __ PrepareCallCFunction(argument_count, r2);

    // r0 - offset of start of capture
    // r1 - length of capture

    // Put arguments into arguments registers.
    // Parameters are
    //   r0: Address byte_offset1 - Address captured substring's start.
    //   r1: Address byte_offset2 - Address of current character position.
    //   r2: size_t byte_length - length of capture in bytes(!)
    //   r3: Isolate* isolate

    __ Push(r4, r5, r6, r7);
    // Address of start of capture.
    __ add(r4, r0, end_of_input_address());
    // Address of current input position.
    __ add(r5, current_input_offset(), end_of_input_address());
    // Length of capture.
    __ mov(r6, r1);
    // Save length on stack for use on return.
    __ push(r1);
    // Isolate.
    __ mov(r7, Operand(ExternalReference::isolate_address()));

    ExternalReference function =
        ExternalReference::re_case_insensitive_compare_uc16(masm_->isolate());
    __ CallCFunction(function, argument_count);

    __ pop(r1);
    __ Pop(r4, r5, r6, r7);

    // Check if function returned non-zero for success or zero for failure.
    __ cmp(r0, Operand(0, RelocInfo::NONE));
    BranchOrBacktrack(eq, on_no_match);
    // On success, increment position by length of capture.
    __ add(current_input_offset(), current_input_offset(), r1);
  }

  __ bind(&fallthrough);
}


void RegExpMacroAssemblerSH4::CheckNotBackReference(
    int start_reg,
    Label* on_no_match) {
  Label fallthrough;
  Label success;

  // Find length of back-referenced capture.
  __ ldr(r0, register_location(start_reg));
  __ ldr(r1, register_location(start_reg + 1));
  __ sub(r1, r1, r0);  // Length to check.
  __ tst(r1, r1);
  // Succeed on empty capture (including no capture).
  __ b(eq, &fallthrough);

  // Check that there are enough characters left in the input.
  __ cmpgt(r1, current_input_offset());
  BranchOrBacktrack(ne, on_no_match);

  // Compute pointers to match string and capture string
  __ add(r0, r0, end_of_input_address());
  __ add(r2, end_of_input_address(), current_input_offset());
  __ add(r1, r1, r0);

  Label loop;
  __ bind(&loop);
  if (mode_ == ASCII) {
    __ ldrb(r3, MemOperand(r0)); __ add(r0, r0, Operand(char_size()));
    __ ldrb(r4, MemOperand(r2)); __ add(r2, r2, Operand(char_size()));
  } else {
    ASSERT(mode_ == UC16);
    __ ldrh(r3, MemOperand(r0)); __ add(r0, r0, Operand(char_size()));
    __ ldrh(r4, MemOperand(r2)); __ add(r2, r2, Operand(char_size()));
  }
  __ cmp(r3, r4);
  BranchOrBacktrack(ne, on_no_match);
  __ cmpge(r0, r1);
  __ b(ne, &loop);

  // Move current character position to position after match.
  __ sub(current_input_offset(), r2, end_of_input_address());
  __ bind(&fallthrough);
}


void RegExpMacroAssemblerSH4::CheckNotRegistersEqual(int reg1,
                                                     int reg2,
                                                     Label* on_not_equal) {
  __ ldr(r0, register_location(reg1));
  __ ldr(r1, register_location(reg2));
  __ cmp(r0, r1);
  BranchOrBacktrack(ne, on_not_equal);
}


void RegExpMacroAssemblerSH4::CheckNotCharacter(unsigned c,
                                                Label* on_not_equal) {
  __ cmp(current_character(), Operand(c));
  BranchOrBacktrack(ne, on_not_equal);
}


void RegExpMacroAssemblerSH4::CheckCharacterAfterAnd(uint32_t c,
                                                     uint32_t mask,
                                                     Label* on_equal) {
  __ land(r0, current_character(), Operand(mask));
  __ cmp(r0, Operand(c));
  BranchOrBacktrack(eq, on_equal);
}


void RegExpMacroAssemblerSH4::CheckNotCharacterAfterAnd(unsigned c,
                                                        unsigned mask,
                                                        Label* on_not_equal) {
  __ land(r0, current_character(), Operand(mask));
  __ cmp(r0, Operand(c));
  BranchOrBacktrack(ne, on_not_equal);
}


void RegExpMacroAssemblerSH4::CheckNotCharacterAfterMinusAnd(
    uc16 c,
    uc16 minus,
    uc16 mask,
    Label* on_not_equal) {
  ASSERT(minus < String::kMaxUC16CharCode);
  __ sub(r0, current_character(), Operand(minus));
  __ land(r0, r0, Operand(mask));
  __ cmp(r0, Operand(c));
  BranchOrBacktrack(ne, on_not_equal);
}


bool RegExpMacroAssemblerSH4::CheckSpecialCharacterClass(uc16 type,
                                                         Label* on_no_match) {
  // Range checks (c in min..max) are generally implemented by an unsigned
  // (c - min) <= (max - min) check
  switch (type) {
  case 's':
    // Match space-characters
    if (mode_ == ASCII) {
      // ASCII space characters are '\t'..'\r' and ' '.
      Label success;
      __ cmp(current_character(), Operand(' '));
      __ b(eq, &success);
      // Check range 0x09..0x0d
      __ sub(r0, current_character(), Operand('\t'));
      __ cmphi(r0, Operand('\r' - '\t'));
      BranchOrBacktrack(eq, on_no_match);
      __ bind(&success);
      return true;
    }
    return false;
  case 'S':
    // Match non-space characters.
    if (mode_ == ASCII) {
      // ASCII space characters are '\t'..'\r' and ' '.
      __ cmp(current_character(), Operand(' '));
      BranchOrBacktrack(eq, on_no_match);
      __ sub(r0, current_character(), Operand('\t'));
      __ cmphi(r0, Operand('\r' - '\t'));
      BranchOrBacktrack(ne, on_no_match);
      return true;
    }
    return false;
  case 'd':
    // Match ASCII digits ('0'..'9')
    __ sub(r0, current_character(), Operand('0'));
    __ cmphi(current_character(), Operand('9' - '0'));
    BranchOrBacktrack(eq, on_no_match);
    return true;
  case 'D':
    // Match non ASCII-digits
    __ sub(r0, current_character(), Operand('0'));
    __ cmphi(r0, Operand('9' - '0'));
    BranchOrBacktrack(ne, on_no_match);
    return true;
  case '.': {
    // Match non-newlines (not 0x0a('\n'), 0x0d('\r'), 0x2028 and 0x2029)
    __ eor(r0, current_character(), Operand(0x01));
    // See if current character is '\n'^1 or '\r'^1, i.e., 0x0b or 0x0c
    __ sub(r0, r0, Operand(0x0b));
    __ cmphi(r0, Operand(0x0c - 0x0b));
    BranchOrBacktrack(ne, on_no_match);
    if (mode_ == UC16) {
      // Compare original value to 0x2028 and 0x2029, using the already
      // computed (current_char ^ 0x01 - 0x0b). I.e., check for
      // 0x201d (0x2028 - 0x0b) or 0x201e.
      __ sub(r0, r0, Operand(0x2028 - 0x0b));
      __ cmphi(r0, Operand(1));
      BranchOrBacktrack(ne, on_no_match);
    }
    return true;
  }
  case 'n': {
    // Match newlines (0x0a('\n'), 0x0d('\r'), 0x2028 and 0x2029)
    __ eor(r0, current_character(), Operand(0x01));
    // See if current character is '\n'^1 or '\r'^1, i.e., 0x0b or 0x0c
    __ sub(r0, r0, Operand(0x0b));
    __ cmphi(r0, Operand(0x0c - 0x0b));
    if (mode_ == ASCII) {
      BranchOrBacktrack(eq, on_no_match);
    } else {
      Label done;
      __ b(ne, &done);
      // Compare original value to 0x2028 and 0x2029, using the already
      // computed (current_char ^ 0x01 - 0x0b). I.e., check for
      // 0x201d (0x2028 - 0x0b) or 0x201e.
      __ sub(r0, r0, Operand(0x2028 - 0x0b));
      __ cmphi(r0, Operand(1));
      BranchOrBacktrack(eq, on_no_match);
      __ bind(&done);
    }
    return true;
  }
  case 'w': {
    if (mode_ != ASCII) {
      // Table is 128 entries, so all ASCII characters can be tested.
      __ cmphi(current_character(), Operand('z'));
      BranchOrBacktrack(eq, on_no_match);
    }
    ExternalReference map = ExternalReference::re_word_character_map();
    __ mov(r0, Operand(map));
    __ ldrb(r0, MemOperand(r0, current_character()));
    __ tst(r0, r0);
    BranchOrBacktrack(eq, on_no_match);
    return true;
  }
  case 'W': {
    Label done;
    if (mode_ != ASCII) {
      // Table is 128 entries, so all ASCII characters can be tested.
      __ cmphi(current_character(), Operand('z'));
      __ b(eq, &done);
    }
    ExternalReference map = ExternalReference::re_word_character_map();
    __ mov(r0, Operand(map));
    __ ldrb(r0, MemOperand(r0, current_character()));
    __ tst(r0, r0);
    BranchOrBacktrack(ne, on_no_match);
    if (mode_ != ASCII) {
      __ bind(&done);
    }
    return true;
  }
  case '*':
    // Match any character.
    return true;
  // No custom implementation (yet): s(UC16), S(UC16).
  default:
    return false;
  }
}


void RegExpMacroAssemblerSH4::Fail() {
  __ mov(r0, Operand(FAILURE));
  __ jmp(&exit_label_);
}


Handle<HeapObject> RegExpMacroAssemblerSH4::GetCode(Handle<String> source) {
  // Finalize code - write the entry point code now we know how many
  // registers we need.

  // Entry code:
  __ bind(&entry_label_);
  // Push arguments
  // Save callee-save registers.
  // Start new stack frame.
  // Store link register in existing stack-cell.
  // Order here should correspond to order of offset constants in header file.
  // WARNING: should change the value of kReturnAddress (depend on the number
  // of saved registers.
  RegList registers_to_retain = kCalleeSaved;
  RegList argument_registers = r4.bit() | r5.bit() | r6.bit() | r7.bit();
  __ push(pr);
  __ pushm(registers_to_retain);
  __ pushm(argument_registers);
  // Set frame pointer in space for it if this is not a direct call
  // from generated code.
  __ add(frame_pointer(), sp, Operand(4 * kPointerSize));
  __ push(r0);  // Make room for "position - 1" constant (value is irrelevant).
  __ push(r0);  // Make room for "at start" constant (value is irrelevant).
  // Check if we have space on the stack for registers.
  Label stack_limit_hit;
  Label stack_ok;

  ExternalReference stack_limit =
      ExternalReference::address_of_stack_limit(masm_->isolate());
  __ mov(r0, Operand(stack_limit));
  __ ldr(r0, MemOperand(r0));
  // Handle it if the stack pointer is already below the stack limit.
  __ cmphi(sp, r0);
  __ sub(r0, sp, r0);
  __ b(ne, &stack_limit_hit);
  // Check if there is room for the variable number of registers above
  // the stack limit.
  __ cmphs(r0, Operand(num_registers_ * kPointerSize));
  __ b(eq, &stack_ok, Label::kNear);
  // Exit with OutOfMemory exception. There is not enough space on the stack
  // for our working registers.
  __ mov(r0, Operand(EXCEPTION));
  __ jmp(&exit_label_);

  __ bind(&stack_limit_hit);
  CallCheckStackGuardState(r0);
  __ cmp(r0, Operand(0, RelocInfo::NONE));
  // If returned value is non-zero, we exit with the returned value as result.
  __ b(ne, &exit_label_);

  __ bind(&stack_ok);

  // Allocate space on stack for registers.
  __ sub(sp, sp, Operand(num_registers_ * kPointerSize));
  // Load string end.
  __ ldr(end_of_input_address(), MemOperand(frame_pointer(), kInputEnd));
  // Load input start.
  __ ldr(r0, MemOperand(frame_pointer(), kInputStart));
  // Find negative length (offset of start relative to end).
  __ sub(current_input_offset(), r0, end_of_input_address());
  // Set r0 to address of char before start of the input string
  // (effectively string position -1).
  __ ldr(r1, MemOperand(frame_pointer(), kStartIndex));
  __ sub(r0, current_input_offset(), Operand(char_size()));
  __ lsl(ip, r1, Operand((mode_ == UC16) ? 1 : 0));
  __ sub(r0, r0, ip);
  // Store this value in a local variable, for use when clearing
  // position registers.
  __ str(r0, MemOperand(frame_pointer(), kInputStartMinusOne));

  // Determine whether the start index is zero, that is at the start of the
  // string, and store that value in a local variable.
  __ tst(r1, r1);
  __ mov(r1, Operand(1), eq);
  __ mov(r1, Operand(0, RelocInfo::NONE), ne);
  __ str(r1, MemOperand(frame_pointer(), kAtStart));

  if (num_saved_registers_ > 0) {  // Always is, if generated from a regexp.
    // Fill saved registers with initial value = start offset - 1

    // Address of register 0.
    __ add(r1, frame_pointer(), Operand(kRegisterZero));
    __ mov(r2, Operand(num_saved_registers_));
    Label init_loop;
    __ bind(&init_loop);
    __ str(r0, MemOperand(r1));
    __ add(r1, r1, Operand(-kPointerSize));
    __ sub(r2, r2, Operand(1));
    __ tst(r2, r2);
    __ b(ne, &init_loop);
  }

  // Initialize backtrack stack pointer.
  __ ldr(backtrack_stackpointer(), MemOperand(frame_pointer(), kStackHighEnd));
  // Initialize code pointer register
  __ mov(code_pointer(), Operand(masm_->CodeObject()));
  // Load previous char as initial value of current character register.
  Label at_start;
  __ ldr(r0, MemOperand(frame_pointer(), kAtStart));
  __ cmp(r0, Operand(0, RelocInfo::NONE));
  __ b(ne, &at_start);
  LoadCurrentCharacterUnchecked(-1, 1);  // Load previous char.
  __ jmp(&start_label_);
  __ bind(&at_start);
  __ mov(current_character(), Operand('\n'));
  __ jmp(&start_label_);


  // Exit code:
  if (success_label_.is_linked()) {
    // Save captures when successful.
    __ bind(&success_label_);
    if (num_saved_registers_ > 0) {
      // copy captures to output
      __ ldr(r1, MemOperand(frame_pointer(), kInputStart));
      __ ldr(r0, MemOperand(frame_pointer(), kRegisterOutput));
      __ ldr(r2, MemOperand(frame_pointer(), kStartIndex));
      __ sub(r1, end_of_input_address(), r1);
      // r1 is length of input in bytes.
      if (mode_ == UC16) {
        __ lsr(r1, r1, Operand(1));
      }
      // r1 is length of input in characters.
      __ add(r1, r1, r2);
      // r1 is length of string in characters.

      ASSERT_EQ(0, num_saved_registers_ % 2);
      // Always an even number of capture registers. This allows us to
      // unroll the loop once to add an operation between a load of a register
      // and the following use of that register.
      for (int i = 0; i < num_saved_registers_; i += 2) {
        __ ldr(r2, register_location(i));
        __ ldr(r3, register_location(i + 1));
        if (mode_ == UC16) {
          __ asr(r2, r2, Operand(1));
          __ add(r2, r1, r2);
          __ asr(r3, r3, Operand(1));
          __ add(r3, r1, r3);
        } else {
          __ add(r2, r1, r2);
          __ add(r3, r1, r3);
        }
        __ str(r2, MemOperand(r0, kPointerSize, PostIndex));
        __ str(r3, MemOperand(r0, kPointerSize, PostIndex));
      }
    }
    __ mov(r0, Operand(SUCCESS));
  }
  // Exit and return r0
  __ bind(&exit_label_);
  // Skip sp past regexp registers and local variables..
  __ mov(sp, frame_pointer());
  // Restore registers r4..r11 and return (restoring lr to pc).
  __ popm(registers_to_retain);
  __ pop(pr);
  __ rts();

  // Backtrack code (branch target for conditional backtracks).
  if (backtrack_label_.is_linked()) {
    __ bind(&backtrack_label_);
    Backtrack();
  }

  Label exit_with_exception;

  // Preempt-code
  if (check_preempt_label_.is_linked()) {
    SafeCallTarget(&check_preempt_label_);

    CallCheckStackGuardState(r0);
    __ cmp(r0, Operand(0, RelocInfo::NONE));
    // If returning non-zero, we should end execution with the given
    // result as return value.
    __ b(ne, &exit_label_);

    // String might have moved: Reload end of string from frame.
    __ ldr(end_of_input_address(), MemOperand(frame_pointer(), kInputEnd));
    SafeReturn();
  }

  // Backtrack stack overflow code.
  if (stack_overflow_label_.is_linked()) {
    SafeCallTarget(&stack_overflow_label_);
    // Reached if the backtrack-stack limit has been hit.
    Label grow_failed;

    // Call GrowStack(backtrack_stackpointer(), &stack_base)
    __ Push(r4, r5, r6, r7);
    static const int num_arguments = 3;
    __ mov(r4, backtrack_stackpointer());
    __ add(r5, frame_pointer(), Operand(kStackHighEnd));
    __ mov(r6, Operand(ExternalReference::isolate_address()));
    __ PrepareCallCFunction(num_arguments, r0);
    ExternalReference grow_stack =
        ExternalReference::re_grow_stack(masm_->isolate());
    __ CallCFunction(grow_stack, num_arguments);
    // If return NULL, we have failed to grow the stack, and
    // must exit with a stack-overflow exception.
    __ Pop(r4, r5, r6, r7);
    __ cmp(r0, Operand(0, RelocInfo::NONE));
    __ b(eq, &exit_with_exception);
    // Otherwise use return value as new stack pointer.
    __ mov(backtrack_stackpointer(), r0);
    // Restore saved registers and continue.
    SafeReturn();
  }

  if (exit_with_exception.is_linked()) {
    // If any of the code above needed to exit with an exception.
    __ bind(&exit_with_exception);
    // Exit with Result EXCEPTION(-1) to signal thrown exception.
    __ mov(r0, Operand(EXCEPTION));
    __ jmp(&exit_label_);
  }

  CodeDesc code_desc;
  masm_->GetCode(&code_desc);
  Handle<Code> code = FACTORY->NewCode(code_desc,
                                       Code::ComputeFlags(Code::REGEXP),
                                       masm_->CodeObject());
  PROFILE(Isolate::Current(), RegExpCodeCreateEvent(*code, *source));
  return Handle<HeapObject>::cast(code);
}


void RegExpMacroAssemblerSH4::GoTo(Label* to) {
  BranchOrBacktrack(al, to);
}


void RegExpMacroAssemblerSH4::IfRegisterGE(int reg,
                                           int comparand,
                                           Label* if_ge) {
  __ ldr(r0, register_location(reg));
  __ cmpge(r0, Operand(comparand));
  BranchOrBacktrack(eq, if_ge);
}


void RegExpMacroAssemblerSH4::IfRegisterLT(int reg,
                                           int comparand,
                                           Label* if_lt) {
  __ ldr(r0, register_location(reg));
  __ cmpge(r0, Operand(comparand));
  BranchOrBacktrack(ne, if_lt);
}


void RegExpMacroAssemblerSH4::IfRegisterEqPos(int reg,
                                              Label* if_eq) {
  __ ldr(r0, register_location(reg));
  __ cmp(r0, current_input_offset());
  BranchOrBacktrack(eq, if_eq);
}


RegExpMacroAssembler::IrregexpImplementation
    RegExpMacroAssemblerSH4::Implementation() {
  return kSH4Implementation;
}


void RegExpMacroAssemblerSH4::LoadCurrentCharacter(int cp_offset,
                                                   Label* on_end_of_input,
                                                   bool check_bounds,
                                                   int characters) {
  ASSERT(cp_offset >= -1);      // ^ and \b can look behind one character.
  ASSERT(cp_offset < (1<<30));  // Be sane! (And ensure negation works)
  if (check_bounds) {
    CheckPosition(cp_offset + characters - 1, on_end_of_input);
  }
  LoadCurrentCharacterUnchecked(cp_offset, characters);
}


void RegExpMacroAssemblerSH4::PopCurrentPosition() {
  Pop(current_input_offset());
}


void RegExpMacroAssemblerSH4::PopRegister(int register_index) {
  Pop(r0);
  __ str(r0, register_location(register_index));
}


void RegExpMacroAssemblerSH4::PushBacktrack(Label* label) {
  if (label->is_bound()) {
    int target = label->pos();
    __ mov(r0, Operand(target + Code::kHeaderSize - kHeapObjectTag));
  } else {
    masm_->load_label(label);
  }
  Push(r0);
  CheckStackLimit();
}


void RegExpMacroAssemblerSH4::PushCurrentPosition() {
  Push(current_input_offset());
}


void RegExpMacroAssemblerSH4::PushRegister(int register_index,
                                           StackCheckFlag check_stack_limit) {
  __ ldr(r0, register_location(register_index));
  Push(r0);
  if (check_stack_limit) CheckStackLimit();
}


void RegExpMacroAssemblerSH4::ReadCurrentPositionFromRegister(int reg) {
  __ ldr(current_input_offset(), register_location(reg));
}


void RegExpMacroAssemblerSH4::ReadStackPointerFromRegister(int reg) {
  __ ldr(backtrack_stackpointer(), register_location(reg));
  __ ldr(r0, MemOperand(frame_pointer(), kStackHighEnd));
  __ add(backtrack_stackpointer(), backtrack_stackpointer(), r0);
}


void RegExpMacroAssemblerSH4::SetCurrentPositionFromEnd(int by) {
  Label after_position;
  __ cmpge(current_input_offset(), Operand(-by * char_size()));
  __ b(eq, &after_position);
  __ mov(current_input_offset(), Operand(-by * char_size()));
  // On RegExp code entry (where this operation is used), the character before
  // the current position is expected to be already loaded.
  // We have advanced the position, so it's safe to read backwards.
  LoadCurrentCharacterUnchecked(-1, 1);
  __ bind(&after_position);
}


void RegExpMacroAssemblerSH4::SetRegister(int register_index, int to) {
  ASSERT(register_index >= num_saved_registers_);  // Reserved for positions!
  __ mov(r0, Operand(to));
  __ str(r0, register_location(register_index));
}


void RegExpMacroAssemblerSH4::Succeed() {
  __ jmp(&success_label_);
}


void RegExpMacroAssemblerSH4::WriteCurrentPositionToRegister(int reg,
                                                             int cp_offset) {
  if (cp_offset == 0) {
    __ str(current_input_offset(), register_location(reg));
  } else {
    __ add(r0, current_input_offset(), Operand(cp_offset * char_size()));
    __ str(r0, register_location(reg));
  }
}


void RegExpMacroAssemblerSH4::ClearRegisters(int reg_from, int reg_to) {
  ASSERT(reg_from <= reg_to);
  __ ldr(r0, MemOperand(frame_pointer(), kInputStartMinusOne));
  for (int reg = reg_from; reg <= reg_to; reg++) {
    __ str(r0, register_location(reg));
  }
}


void RegExpMacroAssemblerSH4::WriteStackPointerToRegister(int reg) {
  __ ldr(r1, MemOperand(frame_pointer(), kStackHighEnd));
  __ sub(r0, backtrack_stackpointer(), r1);
  __ str(r0, register_location(reg));
}


// Private methods:

void RegExpMacroAssemblerSH4::CallCheckStackGuardState(Register scratch) {
  static const int num_arguments = 3;
  // We do not have to save r5 as this value is then putted back by
  // CallCFunctionUsingStub (from the CodeObject)
  __ Push(r4, r6, r7);
  __ PrepareCallCFunction(num_arguments, scratch);
  // RegExp code frame pointer.
  __ mov(r6, frame_pointer());
  // Code* of self.
  __ mov(r5, Operand(masm_->CodeObject()));
  // r0 becomes return address pointer.
  ExternalReference stack_guard_check =
      ExternalReference::re_check_stack_guard_state(masm_->isolate());
  CallCFunctionUsingStub(stack_guard_check, num_arguments);
  __ Pop(r4, r6, r7);
}


// Helper function for reading a value out of a stack frame.
template <typename T>
static T& frame_entry(Address re_frame, int frame_offset) {
  return reinterpret_cast<T&>(Memory::int32_at(re_frame + frame_offset));
}


int RegExpMacroAssemblerSH4::CheckStackGuardState(Address* return_address,
                                                  Code* re_code,
                                                  Address re_frame) {
  Isolate* isolate = frame_entry<Isolate*>(re_frame, kIsolate);
  ASSERT(isolate == Isolate::Current());
  if (isolate->stack_guard()->IsStackOverflow()) {
    isolate->StackOverflow();
    return EXCEPTION;
  }

  // If not real stack overflow the stack guard was used to interrupt
  // execution for another purpose.

  // If this is a direct call from JavaScript retry the RegExp forcing the call
  // through the runtime system. Currently the direct call cannot handle a GC.
  if (frame_entry<int>(re_frame, kDirectCall) == 1) {
    return RETRY;
  }

  // Prepare for possible GC.
  HandleScope handles(isolate);
  Handle<Code> code_handle(re_code);

  Handle<String> subject(frame_entry<String*>(re_frame, kInputString));

  // Current string.
  bool is_ascii = subject->IsAsciiRepresentationUnderneath();

  ASSERT(re_code->instruction_start() <= *return_address);
  ASSERT(*return_address <=
      re_code->instruction_start() + re_code->instruction_size());

  MaybeObject* result = Execution::HandleStackGuardInterrupt();

  if (*code_handle != re_code) {  // Return address no longer valid
    int delta = code_handle->address() - re_code->address();
    // Overwrite the return address on the stack.
    *return_address += delta;
  }

  if (result->IsException()) {
    return EXCEPTION;
  }

  Handle<String> subject_tmp = subject;
  int slice_offset = 0;

  // Extract the underlying string and the slice offset.
  if (StringShape(*subject_tmp).IsCons()) {
    subject_tmp = Handle<String>(ConsString::cast(*subject_tmp)->first());
  } else if (StringShape(*subject_tmp).IsSliced()) {
    SlicedString* slice = SlicedString::cast(*subject_tmp);
    subject_tmp = Handle<String>(slice->parent());
    slice_offset = slice->offset();
  }

  // String might have changed.
  if (subject_tmp->IsAsciiRepresentation() != is_ascii) {
    // If we changed between an ASCII and an UC16 string, the specialized
    // code cannot be used, and we need to restart regexp matching from
    // scratch (including, potentially, compiling a new version of the code).
    return RETRY;
  }

  // Otherwise, the content of the string might have moved. It must still
  // be a sequential or external string with the same content.
  // Update the start and end pointers in the stack frame to the current
  // location (whether it has actually moved or not).
  ASSERT(StringShape(*subject_tmp).IsSequential() ||
      StringShape(*subject_tmp).IsExternal());

  // The original start address of the characters to match.
  const byte* start_address = frame_entry<const byte*>(re_frame, kInputStart);

  // Find the current start address of the same character at the current string
  // position.
  int start_index = frame_entry<int>(re_frame, kStartIndex);
  const byte* new_address = StringCharacterPosition(*subject_tmp,
                                                    start_index + slice_offset);

  if (start_address != new_address) {
    // If there is a difference, update the object pointer and start and end
    // addresses in the RegExp stack frame to match the new value.
    const byte* end_address = frame_entry<const byte* >(re_frame, kInputEnd);
    int byte_length = static_cast<int>(end_address - start_address);
    frame_entry<const String*>(re_frame, kInputString) = *subject;
    frame_entry<const byte*>(re_frame, kInputStart) = new_address;
    frame_entry<const byte*>(re_frame, kInputEnd) = new_address + byte_length;
  }

  return 0;
}


MemOperand RegExpMacroAssemblerSH4::register_location(int register_index) {
  ASSERT(register_index < (1<<30));
  if (num_registers_ <= register_index) {
    num_registers_ = register_index + 1;
  }
  return MemOperand(frame_pointer(),
                    kRegisterZero - register_index * kPointerSize);
}


void RegExpMacroAssemblerSH4::CheckPosition(int cp_offset,
                                            Label* on_outside_input) {
  __ cmpge(current_input_offset(), Operand(-cp_offset * char_size()));
  BranchOrBacktrack(eq, on_outside_input);
}


void RegExpMacroAssemblerSH4::BranchOrBacktrack(Condition condition,
                                                Label* to) {
  if (condition == al) {  // Unconditional.
    if (to == NULL) {
      Backtrack();
      return;
    }
    __ jmp(to);
    return;
  }
  if (to == NULL) {
    __ b(condition, &backtrack_label_);
    return;
  }
  __ b(condition, to);
}


void RegExpMacroAssemblerSH4::SafeCall(Label* to, Condition cond) {
  Label skip;
  ASSERT(cond == eq || cond == ne);
  __ b(NegateCondition(cond), &skip, Label::kNear);
  __ jsr(to);
  __ bind(&skip);
}


void RegExpMacroAssemblerSH4::SafeReturn() {
  __ pop(ip);
  __ add(ip, ip, Operand(masm_->CodeObject()));
  __ mov(pr, ip);
  __ rts();
}


void RegExpMacroAssemblerSH4::SafeCallTarget(Label* name) {
  __ bind(name);
  __ mov(ip, pr);
  __ sub(ip, ip, Operand(masm_->CodeObject()));
  __ mov(pr, ip);
  __ push(lr);
}


void RegExpMacroAssemblerSH4::Push(Register source) {
  ASSERT(!source.is(backtrack_stackpointer()));
  __ add(backtrack_stackpointer(), backtrack_stackpointer(), Operand(-kPointerSize));
  __ str(source, MemOperand(backtrack_stackpointer()));
}


void RegExpMacroAssemblerSH4::Pop(Register target) {
  ASSERT(!target.is(backtrack_stackpointer()));
  __ ldr(target,
         MemOperand(backtrack_stackpointer(), kPointerSize, PostIndex));
}


void RegExpMacroAssemblerSH4::CheckPreemption() {
  // Check for preemption.
  ExternalReference stack_limit =
      ExternalReference::address_of_stack_limit(masm_->isolate());
  __ mov(r0, Operand(stack_limit));
  __ ldr(r0, MemOperand(r0));
  __ cmphi(sp, r0);
  SafeCall(&check_preempt_label_, ne);
}


void RegExpMacroAssemblerSH4::CheckStackLimit() {
  ExternalReference stack_limit =
      ExternalReference::address_of_regexp_stack_limit(masm_->isolate());
  __ mov(r0, Operand(stack_limit));
  __ ldr(r0, MemOperand(r0));
  __ cmphi(backtrack_stackpointer(), r0);
  SafeCall(&stack_overflow_label_, ne);
}


void RegExpMacroAssemblerSH4::CallCFunctionUsingStub(
    ExternalReference function,
    int num_arguments) {
  // Must pass all arguments in registers. The stub pushes on the stack.
  ASSERT(num_arguments <= 4);
  __ mov(r1, Operand(function));
  RegExpCEntryStub stub;
  __ CallStub(&stub);
  __ mov(code_pointer(), Operand(masm_->CodeObject()));
}


void RegExpMacroAssemblerSH4::LoadCurrentCharacterUnchecked(int cp_offset,
                                                            int characters) {
  Register offset = current_input_offset();
  if (cp_offset != 0) {
    __ add(r0, current_input_offset(), Operand(cp_offset * char_size()));
    offset = r0;
  }
  // The ldr, str, ldrh, strh instructions can do unaligned accesses, if the CPU
  // and the operating system running on the target allow it.
  // If unaligned load/stores are not supported then this function must only
  // be used to load a single character at a time.
#if !V8_TARGET_CAN_READ_UNALIGNED
  ASSERT(characters == 1);
#endif

  if (mode_ == ASCII) {
    if (characters == 4) {
      __ ldr(current_character(), MemOperand(end_of_input_address(), offset));
    } else if (characters == 2) {
      __ ldrh(current_character(), MemOperand(end_of_input_address(), offset));
    } else {
      ASSERT(characters == 1);
      __ ldrb(current_character(), MemOperand(end_of_input_address(), offset));
    }
  } else {
    ASSERT(mode_ == UC16);
    if (characters == 2) {
      __ ldr(current_character(), MemOperand(end_of_input_address(), offset));
    } else {
      ASSERT(characters == 1);
      __ ldrh(current_character(), MemOperand(end_of_input_address(), offset));
    }
  }
}


void RegExpCEntryStub::Generate(MacroAssembler* masm_) {
  // TODO(STM): do we have to align the stack arguments ?
  //// Decrement it by kPointerSize to make room for pushing and poping pr
  //int stack_alignment = OS::ActivationFrameAlignment() - kPointerSize;
  //// Stack is already aligned for call, so decrement by alignment
  //// to make room for storing the link register.
  __ push(pr);
  // The first argument should be sp
  __ mov(r4, sp);
  __ jsr(r1);
  __ pop(pr);
  __ rts();
}

#undef __

#endif  // V8_INTERPRETED_REGEXP

}}  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
