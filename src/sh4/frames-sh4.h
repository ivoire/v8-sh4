// Copyright 2012 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_SH4_FRAMES_SH4_H_
#define V8_SH4_FRAMES_SH4_H_

namespace v8 {
namespace internal {


// Register lists
// Note that the bit values must match those used in actual instruction encoding
const int kNumRegs = 16;


// Caller-saved registers
const RegList kJSCallerSaved =
  1 << 0 |      // r0
  1 << 1 |      // r1
  1 << 2 |      // r2
  1 << 3 |      // r3
  1 << 4 |      // r4
  1 << 5 |      // r5
  1 << 6 |      // r6
  1 << 7;       // r7

const int kNumJSCallerSaved = 8;

typedef Object* JSCallerSavedBuffer[kNumJSCallerSaved];

// Return the code of the n-th caller-saved register available to JavaScript
// e.g. JSCallerSavedReg(0) returns r0.code() == 0
int JSCallerSavedCode(int n);


// Callee-saved registers preserved when switching from C to JavaScript
const RegList kCalleeSaved =
  1 <<  8  |  //  r8
  1 <<  9  |  //  r9
  1 <<  10 |  //  r10 (ip in JS)
  1 <<  11 |  //  r11 (rtmp in JS)
  1 <<  12 |  //  r12 (roots in JS)
  1 <<  13 |  //  r13 (cp in JS)
  1 <<  14;   //  r14 (fp in JS)

// When calling into C++ (only for C++ calls that can't cause a GC).
// The call code will take care of lr, fp, etc.
const RegList kCallerSaved = kJSCallerSaved; // Same as JS caller saved

const int kNumCalleeSaved = 7;

// Double registers sh4_dr12 and sh4_dr14 are callee-saved.
const int kNumDoubleCalleeSaved = 2;

// Number of registers for which space is reserved in safepoints. Must be a
// multiple of 8.
// TODO(regis): Only 8 registers may actually be sufficient. Revisit.
const int kNumSafepointRegisters = 16;

// Define the list of registers actually saved at safepoints.
// Note that the number of saved registers may be smaller than the reserved
// space, i.e. kNumSafepointSavedRegisters <= kNumSafepointRegisters.
const RegList kSafepointSavedRegisters = kJSCallerSaved | kCalleeSaved;
const int kNumSafepointSavedRegisters = kNumJSCallerSaved + kNumCalleeSaved;

// ----------------------------------------------------


class EntryFrameConstants : public AllStatic {
 public:
  static const int kCallerFPOffset =
      -(StandardFrameConstants::kFixedFrameSizeFromFp + kPointerSize);
};


class ExitFrameConstants : public AllStatic {
 public:
  static const int kFrameSize      = 2 * kPointerSize;
  static const int kConstantPoolOffset   = 0;  // Not used
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
  // FP-relative.
  static const int kLengthOffset = StandardFrameConstants::kExpressionsOffset;

  static const int kFrameSize =
      StandardFrameConstants::kFixedFrameSize + kPointerSize;
};


class ConstructFrameConstants : public AllStatic {
 public:
  // FP-relative.
  static const int kImplicitReceiverOffset = -6 * kPointerSize;
  static const int kConstructorOffset      = -5 * kPointerSize;
  static const int kLengthOffset           = -4 * kPointerSize;
  static const int kCodeOffset = StandardFrameConstants::kExpressionsOffset;

  static const int kFrameSize =
      StandardFrameConstants::kFixedFrameSize + 4 * kPointerSize;
};


class InternalFrameConstants : public AllStatic {
 public:
  // FP-relative.
  static const int kCodeOffset = StandardFrameConstants::kExpressionsOffset;
};


inline Object* JavaScriptFrame::function_slot_object() const {
  const int offset = JavaScriptFrameConstants::kFunctionOffset;
  return Memory::Object_at(fp() + offset);
}


inline void StackHandler::SetFp(Address slot, Address fp) {
  Memory::Address_at(slot) = fp;
}


} }  // namespace v8::internal

#endif  // V8_SH4_FRAMES_SH4_H_
