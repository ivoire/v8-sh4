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

#ifndef V8_SH4_FRAMES_SH4_H_
#define V8_SH4_FRAMES_SH4_H_

#include "memory.h"

namespace v8 {
namespace internal {


// Register lists
// Note that the bit values must match those used in actual instruction encoding
static const int kNumRegs = 16;


// Caller-saved registers
static const RegList kJSCallerSaved =
  1 << 0 |      // r0
  1 << 1 |      // r1
  1 << 2 |      // r2
  1 << 3 |      // r3
  1 << 4 |      // r4
  1 << 5 |      // r5
  1 << 6 |      // r6
  1 << 7;       // r7

static const int kNumJSCallerSaved = 8;

// Callee-saved registers preserved when switching from C to JavaScript
static const RegList kCalleeSaved =
  1 <<  8  |  //  r8
  1 <<  9  |  //  r9
  1 <<  10 |  //  r10
  1 <<  11 |  //  r11
  1 <<  12 |  //  r12 (roots in JS)
  1 <<  13 |  //  r13 (cp in JS)
  1 <<  14;   //  r14 (fp in JS)

static const int kNumCalleeSaved = 7;


typedef Object* JSCallerSavedBuffer[kNumJSCallerSaved];


// Number of registers for which space is reserved in safepoints.
static const int kNumSafepointRegisters = 16;

// ----------------------------------------------------


class StackHandlerConstants : public AllStatic {
 public:
  static const int kNextOffset    = 0 * kPointerSize;
  static const int kStateOffset   = 1 * kPointerSize;
  static const int kContextOffset = 2 * kPointerSize;
  static const int kFPOffset      = 3 * kPointerSize;
  static const int kPCOffset      = 4 * kPointerSize;

  static const int kSize = kPCOffset + kPointerSize;
};


class EntryFrameConstants : public AllStatic {
 public:
  static const int kCallerFPOffset      = -3 * kPointerSize;
};


class ExitFrameConstants : public AllStatic {
 public:
  static const int kCodeOffset = -2 * kPointerSize;
  static const int kSPOffset = -1 * kPointerSize;

  // The caller fields are below the frame pointer on the stack.
  static const int kCallerFPOffset = 0 * kPointerSize;
  // The calling JS function is below FP.
  static const int kCallerPCOffset = 1 * kPointerSize;

  // FP-relative displacement of the caller's SP.  It points just
  // below the saved PC.
  static const int kCallerSPDisplacement = 2 * kPointerSize;
};


class StandardFrameConstants : public AllStatic {
 public:
  static const int kExpressionsOffset = -3 * kPointerSize;
  static const int kMarkerOffset      = -2 * kPointerSize;
  static const int kContextOffset     = -1 * kPointerSize;
  static const int kCallerFPOffset    =  0 * kPointerSize;
  static const int kCallerPCOffset    =  1 * kPointerSize;
  static const int kCallerSPOffset    =  2 * kPointerSize;
};


class JavaScriptFrameConstants : public AllStatic {
 public:
  // FP-relative.
  static const int kLocal0Offset = StandardFrameConstants::kExpressionsOffset;
  static const int kLastParameterOffset = +2 * kPointerSize;
  static const int kFunctionOffset = StandardFrameConstants::kMarkerOffset;

  // Caller SP-relative.
  static const int kParam0Offset   = -2 * kPointerSize;
  static const int kReceiverOffset = -1 * kPointerSize;
};


class ArgumentsAdaptorFrameConstants : public AllStatic {
 public:
  static const int kLengthOffset = StandardFrameConstants::kExpressionsOffset;
};


class InternalFrameConstants : public AllStatic {
 public:
  static const int kCodeOffset = StandardFrameConstants::kExpressionsOffset;
};


inline Object* JavaScriptFrame::function_slot_object() const {
  const int offset = JavaScriptFrameConstants::kFunctionOffset;
  return Memory::Object_at(fp() + offset);
}


} }  // namespace v8::internal

#endif  // V8_SH4_FRAMES_SH4_H_
