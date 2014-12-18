// Copyright 2011 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "v8.h"

#if V8_TARGET_ARCH_SH4 // FILE: SAMEAS: arm, REVIEWEDBY: CG

#include "assembler.h"
#include "assembler-sh4.h"
#include "assembler-sh4-inl.h"
#include "frames.h"
#include "macro-assembler.h"
#include "macro-assembler-sh4.h"

namespace v8 {
namespace internal {


Register JavaScriptFrame::fp_register() { return v8::internal::fp; }
Register JavaScriptFrame::context_register() { return cp; }


Register StubFailureTrampolineFrame::fp_register() { return v8::internal::fp; }
Register StubFailureTrampolineFrame::context_register() { return cp; }


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4
