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

#include <unistd.h>

#include "v8.h"

#if !defined(USE_SIMULATOR)
#include <sys/syscall.h>
#include <asm/cachectl.h>
#endif

#if defined(V8_TARGET_ARCH_SH4)

#include "cpu.h"
#include "macro-assembler.h"
#include "simulator.h"  // for cache flushing.

namespace v8 {
namespace internal {

void CPU::Setup() {
  CpuFeatures::Probe();
}


bool CPU::SupportsCrankshaft() {
  return false;
}


void CPU::FlushICache(void* start, size_t size) {
#if defined(USE_SIMULATOR)
  Simulator::FlushICache(Isolate::Current()->simulator_i_cache(), start, size);
#else
  // Invalidate the instruction and the data cache
  syscall(__NR_cacheflush, start, size, CACHEFLUSH_D_WB | CACHEFLUSH_I);
#endif
}


void CPU::DebugBreak() {
#if defined(__sh4__)
  asm("ldtlb");
#else
  UNIMPLEMENTED();
#endif
}

} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_SH4