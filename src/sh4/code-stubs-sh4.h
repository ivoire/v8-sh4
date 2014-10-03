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

#ifndef V8_SH4_CODE_STUBS_SH4_H_
#define V8_SH4_CODE_STUBS_SH4_H_

#include "ic-inl.h"

namespace v8 {
namespace internal {


void ArrayNativeCode(MacroAssembler* masm, Label* call_generic_code);


class StoreBufferOverflowStub: public PlatformCodeStub {
 public:
  StoreBufferOverflowStub(Isolate* isolate, SaveFPRegsMode save_fp)
      : PlatformCodeStub(isolate), save_doubles_(save_fp) {}

  void Generate(MacroAssembler* masm);

  static void GenerateFixedRegStubsAheadOfTime(Isolate* isolate);
  virtual bool SometimesSetsUpAFrame() { return false; }

 private:
  SaveFPRegsMode save_doubles_;

  Major MajorKey() { return StoreBufferOverflow; }
  int MinorKey() { return (save_doubles_ == kSaveFPRegs) ? 1 : 0; }
};


class StringHelper : public AllStatic {
 public:
  // Generate code for copying a large number of characters. This function
  // is allowed to spend extra time setting up conditions to make copying
  // faster. Copying of overlapping regions is not supported.
  // Dest register ends at the position after the last character written.
  static void GenerateCopyCharactersLong(MacroAssembler* masm,
                                         Register dest,
                                         Register src,
                                         Register count,
                                         Register scratch1,
                                         Register scratch2,
                                         Register scratch3,
                                         Register scratch4,
                                         int flags);


  // Generate string hash.
  static void GenerateHashInit(MacroAssembler* masm,
                               Register hash,
                               Register character);

  static void GenerateHashAddCharacter(MacroAssembler* masm,
                                       Register hash,
                                       Register character);

  static void GenerateHashGetHash(MacroAssembler* masm,
                                  Register hash);

 private:
  DISALLOW_IMPLICIT_CONSTRUCTORS(StringHelper);
};


class SubStringStub: public PlatformCodeStub {
 public:
  explicit SubStringStub(Isolate* isolate) : PlatformCodeStub(isolate) {}

 private:
  Major MajorKey() { return SubString; }
  int MinorKey() { return 0; }

  void Generate(MacroAssembler* masm);
};



class StringCompareStub: public PlatformCodeStub {
 public:
  explicit StringCompareStub(Isolate* isolate) : PlatformCodeStub(isolate) { }

  // Compares two flat ASCII strings and returns result in r0.
  static void GenerateCompareFlatAsciiStrings(MacroAssembler* masm,
                                              Register left,
                                              Register right,
                                              Register scratch1,
                                              Register scratch2,
                                              Register scratch3,
                                              Register scratch4);

  // Compares two flat ASCII strings for equality and returns result
  // in r0.
  static void GenerateFlatAsciiStringEquals(MacroAssembler* masm,
                                            Register left,
                                            Register right,
                                            Register scratch1,
                                            Register scratch2,
                                            Register scratch3);

 private:
  virtual Major MajorKey() { return StringCompare; }
  virtual int MinorKey() { return 0; }
  virtual void Generate(MacroAssembler* masm);

  static void GenerateAsciiCharsCompareLoop(MacroAssembler* masm,
                                            Register left,
                                            Register right,
                                            Register length,
                                            Register scratch1,
                                            Register scratch2,
                                            Label* chars_not_equal);
};


// This stub can convert a signed int32 to a heap number (double).  It does
// not work for int32s that are in Smi range!  No GC occurs during this stub
// so you don't have to set up the frame.
class WriteInt32ToHeapNumberStub : public PlatformCodeStub {
 public:
  WriteInt32ToHeapNumberStub(Isolate* isolate,
                             Register the_int,
                             Register the_heap_number,
                             Register scratch)
      : PlatformCodeStub(isolate),
        the_int_(the_int),
        the_heap_number_(the_heap_number),
        scratch_(scratch) { }

  static void GenerateFixedRegStubsAheadOfTime(Isolate* isolate);

 private:
  Register the_int_;
  Register the_heap_number_;
  Register scratch_;

  // Minor key encoding in 16 bits.
  class IntRegisterBits: public BitField<int, 0, 4> {};
  class HeapNumberRegisterBits: public BitField<int, 4, 4> {};
  class ScratchRegisterBits: public BitField<int, 8, 4> {};

  Major MajorKey() { return WriteInt32ToHeapNumber; }
  int MinorKey() {
    // Encode the parameters in a unique 16 bit value.
    return IntRegisterBits::encode(the_int_.code())
           | HeapNumberRegisterBits::encode(the_heap_number_.code())
           | ScratchRegisterBits::encode(scratch_.code());
  }

  void Generate(MacroAssembler* masm);
};


class RecordWriteStub: public PlatformCodeStub {
 public:
  RecordWriteStub(Isolate* isolate,
                  Register object,
                  Register value,
                  Register address,
                  RememberedSetAction remembered_set_action,
                  SaveFPRegsMode fp_mode)
      : PlatformCodeStub(isolate),
        object_(object),
        value_(value),
        address_(address),
        remembered_set_action_(remembered_set_action),
        save_fp_regs_mode_(fp_mode),
        regs_(object,   // An input reg.
              address,  // An input reg.
              value) {  // One scratch reg.
  }

  enum Mode {
    STORE_BUFFER_ONLY,
    INCREMENTAL,
    INCREMENTAL_COMPACTION
  };

  virtual bool SometimesSetsUpAFrame() { return false; }

  // Position of branches that will be patched. Ref to ::Generate()
  static const int kFirstBranchPos = 2 * Assembler::kInstrSize; // DIFF: codegen
  static const int kSecondBranchPos = 4 * Assembler::kInstrSize; // DIFF: codegen

  static void PatchBranchIntoNop(MacroAssembler* masm, int pos) {
    // SH4: change bf (branch) into bt (nop) ref to ::Generate()
    masm->instr_at_put(pos, (masm->instr_at(pos) & ~0x200/*bt*/)); // DIFF: codegen
    ASSERT(Assembler::IsBt(masm->instr_at(pos)));
  }

  static void PatchNopIntoBranch(MacroAssembler* masm, int pos) {
    // SH4: change bt (nop) into bf (branch) ref to ::Generate()
    masm->instr_at_put(pos, (masm->instr_at(pos) | 0x200/*bf*/)); // DIFF: codegen
    ASSERT(Assembler::IsBf(masm->instr_at(pos)));
  }

  static Mode GetMode(Code* stub) {
    Instr first_instruction = Assembler::instr_at(stub->instruction_start() +
                                                  kFirstBranchPos);
    Instr second_instruction = Assembler::instr_at(stub->instruction_start() +
                                                  kSecondBranchPos);

    if (Assembler::IsBf(first_instruction)) {
      ASSERT(Assembler::IsBt(second_instruction));
      return INCREMENTAL;
    }

    ASSERT(Assembler::IsBt(first_instruction));

    if (Assembler::IsBf(second_instruction)) {
      return INCREMENTAL_COMPACTION;
    }

    ASSERT(Assembler::IsBt(second_instruction));

    return STORE_BUFFER_ONLY;
  }

  static void Patch(Code* stub, Mode mode) { // SAMEAS: arm
    MacroAssembler masm(NULL,
                        stub->instruction_start(),
                        stub->instruction_size());
    switch (mode) {
      case STORE_BUFFER_ONLY:
        ASSERT(GetMode(stub) == INCREMENTAL ||
               GetMode(stub) == INCREMENTAL_COMPACTION);
        PatchBranchIntoNop(&masm, kFirstBranchPos); // DIFF: codegen
        PatchBranchIntoNop(&masm, kSecondBranchPos); // DIFF: codegen
        break;
      case INCREMENTAL:
        ASSERT(GetMode(stub) == STORE_BUFFER_ONLY);
        PatchNopIntoBranch(&masm, kFirstBranchPos); // DIFF: codegen
        break;
      case INCREMENTAL_COMPACTION:
        ASSERT(GetMode(stub) == STORE_BUFFER_ONLY);
        PatchNopIntoBranch(&masm, kSecondBranchPos); // DIFF: codegen
        break;
    }
    ASSERT(GetMode(stub) == mode);
    CPU::FlushICache(stub->instruction_start(), kSecondBranchPos + Assembler::kInstrSize); // DIFF: codegen
  }

 private:
  // This is a helper class for freeing up 3 scratch registers.  The input is
  // two registers that must be preserved and one scratch register provided by
  // the caller.
  class RegisterAllocation {
   public:
    RegisterAllocation(Register object,
                       Register address,
                       Register scratch0)
        : object_(object),
          address_(address),
          scratch0_(scratch0) {
      ASSERT(!AreAliased(scratch0, object, address, no_reg));
      scratch1_ = GetRegisterThatIsNotOneOf(object_, address_, scratch0_);
    }

    void Save(MacroAssembler* masm) { // SAMEAS: arm
      ASSERT(!AreAliased(object_, address_, scratch1_, scratch0_));
      // We don't have to save scratch0_ because it was given to us as
      // a scratch register.
      masm->push(scratch1_);
    }

    void Restore(MacroAssembler* masm) { // SAMEAS: arm
      masm->pop(scratch1_);
    }

    // If we have to call into C then we need to save and restore all caller-
    // saved registers that were not already preserved.  The scratch registers
    // will be restored by other means so we don't bother pushing them here.
    void SaveCallerSaveRegisters(MacroAssembler* masm, SaveFPRegsMode mode) { // SAMEAS: arm
      masm->push(pr); // DIFF: codegen
      masm->pushm(kCallerSaved & ~scratch1_.bit()); // DIFF: codegen
      if (mode == kSaveFPRegs) {
        masm->SaveFPRegs(sp, scratch0_);
      }
    }

    inline void RestoreCallerSaveRegisters(MacroAssembler*masm,
                                           SaveFPRegsMode mode) {
      if (mode == kSaveFPRegs) {
        masm->RestoreFPRegs(sp, scratch0_);
      }
      masm->popm(kCallerSaved & ~scratch1_.bit()); // DIFF: codegen
      masm->pop(pr); // DIFF: codegen
    }

    inline Register object() { return object_; }
    inline Register address() { return address_; }
    inline Register scratch0() { return scratch0_; }
    inline Register scratch1() { return scratch1_; }

   private:
    Register object_;
    Register address_;
    Register scratch0_;
    Register scratch1_;

    friend class RecordWriteStub;
  };

  enum OnNoNeedToInformIncrementalMarker {
    kReturnOnNoNeedToInformIncrementalMarker,
    kUpdateRememberedSetOnNoNeedToInformIncrementalMarker
  };

  void Generate(MacroAssembler* masm);
  void GenerateIncremental(MacroAssembler* masm, Mode mode);
  void CheckNeedsToInformIncrementalMarker(
      MacroAssembler* masm,
      OnNoNeedToInformIncrementalMarker on_no_need,
      Mode mode);
  void InformIncrementalMarker(MacroAssembler* masm);

  Major MajorKey() { return RecordWrite; }

  int MinorKey() {
    return ObjectBits::encode(object_.code()) |
        ValueBits::encode(value_.code()) |
        AddressBits::encode(address_.code()) |
        RememberedSetActionBits::encode(remembered_set_action_) |
        SaveFPRegsModeBits::encode(save_fp_regs_mode_);
  }

  void Activate(Code* code) {
    code->GetHeap()->incremental_marking()->ActivateGeneratedStub(code);
  }

  class ObjectBits: public BitField<int, 0, 4> {};
  class ValueBits: public BitField<int, 4, 4> {};
  class AddressBits: public BitField<int, 8, 4> {};
  class RememberedSetActionBits: public BitField<RememberedSetAction, 12, 1> {};
  class SaveFPRegsModeBits: public BitField<SaveFPRegsMode, 13, 1> {};

  Register object_;
  Register value_;
  Register address_;
  RememberedSetAction remembered_set_action_;
  SaveFPRegsMode save_fp_regs_mode_;
  Label slow_;
  RegisterAllocation regs_;
};


// Trampoline stub to call into native code. To call safely into native code
// in the presence of compacting GC (which can move code objects) we need to
// keep the code which called into native pinned in the memory. Currently the
// simplest approach is to generate such stub early enough so it can never be
// moved by GC
class DirectCEntryStub: public PlatformCodeStub {
 public:
  explicit DirectCEntryStub(Isolate* isolate) : PlatformCodeStub(isolate) {}
  void Generate(MacroAssembler* masm);
  void GenerateCall(MacroAssembler* masm, Register target);

 private:
  Major MajorKey() { return DirectCEntry; }
  int MinorKey() { return 0; }

  bool NeedsImmovableCode() { return true; }
};


class NameDictionaryLookupStub: public PlatformCodeStub {
 public:
  enum LookupMode { POSITIVE_LOOKUP, NEGATIVE_LOOKUP };

  NameDictionaryLookupStub(Isolate* isolate, LookupMode mode)
      : PlatformCodeStub(isolate), mode_(mode) { }

  void Generate(MacroAssembler* masm);

  static void GenerateNegativeLookup(MacroAssembler* masm,
                                     Label* miss,
                                     Label* done,
                                     Register receiver,
                                     Register properties,
                                     Handle<Name> name,
                                     Register scratch0);

  static void GeneratePositiveLookup(MacroAssembler* masm,
                                     Label* miss,
                                     Label* done,
                                     Register elements,
                                     Register name,
                                     Register r0,
                                     Register r1);

  virtual bool SometimesSetsUpAFrame() { return false; }

 private:
  static const int kInlinedProbes = 4;
  static const int kTotalProbes = 20;

  static const int kCapacityOffset =
      NameDictionary::kHeaderSize +
      NameDictionary::kCapacityIndex * kPointerSize;

  static const int kElementsStartOffset =
      NameDictionary::kHeaderSize +
      NameDictionary::kElementsStartIndex * kPointerSize;

  Major MajorKey() { return NameDictionaryLookup; }

  int MinorKey() {
    return LookupModeBits::encode(mode_);
  }

  class LookupModeBits: public BitField<LookupMode, 0, 1> {};

  LookupMode mode_;
};


struct PlatformCallInterfaceDescriptor {
  explicit PlatformCallInterfaceDescriptor(
      TargetAddressStorageMode storage_mode)
      : storage_mode_(storage_mode) { }

  TargetAddressStorageMode storage_mode() { return storage_mode_; }

 private:
  TargetAddressStorageMode storage_mode_;
};


} }  // namespace v8::internal

#endif  // V8_SH4_CODE_STUBS_SH4_H_
