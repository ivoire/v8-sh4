// Copyright 2007-2008 the V8 project authors. All rights reserved.
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

#ifndef V8_DISASM_H_
#define V8_DISASM_H_

#include "allocation.h"

namespace disasm {

typedef unsigned char byte;

// Interface and default implementation for converting addresses and
// register-numbers to text.  The default implementation is machine
// specific.
class NameConverter {
 public:
  virtual ~NameConverter() {}
  virtual const char* NameOfCPURegister(int reg) const;
  virtual const char* NameOfByteCPURegister(int reg) const;
  virtual const char* NameOfXMMRegister(int reg) const;
  virtual const char* NameOfAddress(byte* addr) const;
  virtual const char* NameOfConstant(byte* addr) const;
  virtual const char* NameInCode(byte* addr) const;

 protected:
  v8::internal::EmbeddedVector<char, 128> tmp_buffer_;
};


// A generic Disassembler interface
class DisassemblerInterface {
 public:
  virtual ~DisassemblerInterface() {}

  // Writes one disassembled instruction into 'buffer' (0-terminated).
  // Returns the length of the disassembled machine instruction in bytes.
  virtual int InstructionDecode(v8::internal::Vector<char> buffer,
                                byte* instruction) = 0;

  // Returns -1 if instruction does not mark the beginning of a constant pool,
  // or the number of entries in the constant pool beginning here.
  virtual int ConstantPoolSizeAt(byte* instruction) = 0;
};


// A generic Disassembler implementation.
// Target specific code is implemented in target dependent files.
class Disassembler : public DisassemblerInterface {
 public:
  explicit Disassembler(const NameConverter& converter);
  ~Disassembler();
  int InstructionDecode(v8::internal::Vector<char> buffer, byte* instruction);
  int ConstantPoolSizeAt(byte* instruction);

  // Write disassembly into specified file 'f' using specified NameConverter
  // (see constructor).
  static void Disassemble(FILE* f, byte* begin, byte* end);

 protected:
  const NameConverter& converter_;
};


// Factory for Disassembler class implemented in target dependent code.
class DisassemblerFactory : public v8::internal::AllStatic {
 public:
  static DisassemblerInterface *NewDisassembler(const NameConverter& converter);
};


}  // namespace disasm

#endif  // V8_DISASM_H_
