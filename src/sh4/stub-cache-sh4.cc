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

#include "ic-inl.h"
#include "codegen.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm)

void StubCompiler::GenerateLoadArrayLength(MacroAssembler* masm,
                                           Register receiver,
                                           Register scratch,
                                           Label* miss_label) {
  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ bt(miss_label);

  // Check that the object is a JS array.
  __ CompareObjectType(receiver, scratch, scratch, JS_ARRAY_TYPE);
  __ bf(miss_label);

  // Load length directly from the JS array.
  __ mov(r0, FieldMemOperand(receiver, JSArray::kLengthOffset));
  __ rts();
}


void StubCompiler::GenerateLoadMiss(MacroAssembler* masm, Code::Kind kind) {
  ASSERT(kind == Code::LOAD_IC || kind == Code::KEYED_LOAD_IC);
  Code* code = NULL;
  if (kind == Code::LOAD_IC) {
    code = masm->isolate()->builtins()->builtin(Builtins::kLoadIC_Miss);
  } else {
    code = masm->isolate()->builtins()->builtin(Builtins::kKeyedLoadIC_Miss);
  }

  Handle<Code> ic(code);
  __ jmp(ic, RelocInfo::CODE_TARGET);
}


// Generate code to check if an object is a string.  If the object is a
// heap object, its map's instance type is left in the scratch1 register.
// If this is not needed, scratch1 and scratch2 may be the same register.
static void GenerateStringCheck(MacroAssembler* masm,
                                Register receiver,
                                Register scratch1,
                                Register scratch2,
                                Label* smi,
                                Label* non_string_object) {
  ASSERT(!receiver.is(r3) && !scratch1.is(r3) && !scratch2.is(r3));

  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ bt(smi);

  // Check that the object is a string.
  __ mov(scratch1, FieldMemOperand(receiver, HeapObject::kMapOffset));
  __ mov(scratch1, FieldMemOperand(scratch1, Map::kInstanceTypeOffset));        // FIXME: mov.b ??
  __ land(scratch2, scratch1, Immediate(kIsNotStringMask));
  // The cast is to resolve the overload for the argument of 0x0.
  __ mov(r3, Immediate(static_cast<int32_t>(kStringTag)));
  __ cmpeq(scratch2, r3);
  __ bf(non_string_object);
}


// Generate code to load the length from a string object and return the length.
// If the receiver object is not a string or a wrapped string object the
// execution continues at the miss label. The register containing the
// receiver is potentially clobbered.
void StubCompiler::GenerateLoadStringLength(MacroAssembler* masm,
                                            Register receiver,
                                            Register scratch1,
                                            Register scratch2,
                                            Label* miss,
                                            bool support_wrappers) {
  ASSERT(!receiver.is(r3) && !scratch1.is(r2) && !scratch2.is(r3));
  Label check_wrapper;

  // Check if the object is a string leaving the instance type in the
  // scratch1 register.
  GenerateStringCheck(masm, receiver, scratch1, scratch2, miss,
                      support_wrappers ? &check_wrapper : miss);

  // Load length directly from the string.
  __ mov(r0, FieldMemOperand(receiver, String::kLengthOffset));
  __ rts();

  if (support_wrappers) {
    // Check if the object is a JSValue wrapper.
    __ bind(&check_wrapper);
    __ mov(r3, Immediate(JS_VALUE_TYPE));
    __ cmpeq(scratch1, r3);
    __ bf(miss);

    // Unwrap the value and check if the wrapped value is a string.
    __ mov(scratch1, FieldMemOperand(receiver, JSValue::kValueOffset));
    GenerateStringCheck(masm, scratch1, scratch2, scratch2, miss, miss);
    __ mov(r0, FieldMemOperand(scratch1, String::kLengthOffset));
    __ rts();
  }
}


void StubCompiler::GenerateLoadFunctionPrototype(MacroAssembler* masm,
                                                 Register receiver,
                                                 Register scratch1,
                                                 Register scratch2,
                                                 Label* miss_label) {
  __ TryGetFunctionPrototype(receiver, scratch1, scratch2, miss_label);
  __ mov(r0, scratch1);
  __ rts();
}


static void ProbeTable(Isolate* isolate,
                       MacroAssembler* masm,
                       Code::Flags flags,
                       StubCache::Table table,
                       Register name,
                       Register offset,
                       Register scratch,
                       Register scratch2) {
  ExternalReference key_offset(isolate->stub_cache()->key_reference(table));
  ExternalReference value_offset(isolate->stub_cache()->value_reference(table));

  uint32_t key_off_addr = reinterpret_cast<uint32_t>(key_offset.address());
  uint32_t value_off_addr = reinterpret_cast<uint32_t>(value_offset.address());

  // Check the relative positions of the address fields.
  ASSERT(value_off_addr > key_off_addr);
  ASSERT((value_off_addr - key_off_addr) % 4 == 0);
  ASSERT((value_off_addr - key_off_addr) < (256 * 4));

  // Check that r3 is not used
  ASSERT(!name.is(r3) && !offset.is(r3) && !scratch.is(r3) && !scratch2.is(r3));

  Label miss;
  Register offsets_base_addr = scratch;

  // Check that the key in the entry matches the name.
  __ mov(offsets_base_addr, Immediate(key_offset));
  __ lsl(r3, offset, Immediate(1));
  __ mov(r3, MemOperand(offsets_base_addr, r3));
  __ cmpeq(name, r3);
  __ bf(&miss);

  // Get the code entry from the cache.
  __ add(offsets_base_addr, offsets_base_addr,
         Immediate(value_off_addr - key_off_addr));
  __ lsl(r3, offset, Immediate(1));
  __ mov(scratch2, MemOperand(offsets_base_addr, r3));

  // Check that the flags match what we're looking for.
  __ mov(scratch2, FieldMemOperand(scratch2, Code::kFlagsOffset));
  __ land(scratch2, scratch2, Immediate(Code::kFlagsNotUsedInLookup));
  __ mov(r3, Immediate(flags));
  __ cmpeq(scratch2, r3);
  __ bf(&miss);

  // Re-load code entry from cache.
  __ lsl(r3, offset, Immediate(1));
  __ mov(offset, MemOperand(offsets_base_addr, r3));

  // Jump to the first instruction in the code stub.
  __ add(offset, offset, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ jmp(offset);

  // Miss: fall through.
  __ bind(&miss);
}


void StubCache::GenerateProbe(MacroAssembler* masm,
                              Code::Flags flags,
                              Register receiver,
                              Register name,
                              Register scratch,
                              Register extra,
                              Register extra2) {
  Isolate* isolate = masm->isolate();
  Label miss;

  // Make sure that code is valid. The shifting code relies on the
  // entry size being 8.
  ASSERT(sizeof(Entry) == 8);

  // Make sure the flags does not name a specific type.
  ASSERT(Code::ExtractTypeFromFlags(flags) == 0);

  // Make sure that there are no register conflicts.
  ASSERT(!scratch.is(receiver));
  ASSERT(!scratch.is(name));
  ASSERT(!extra.is(receiver));
  ASSERT(!extra.is(name));
  ASSERT(!extra.is(scratch));
  ASSERT(!extra2.is(receiver));
  ASSERT(!extra2.is(name));
  ASSERT(!extra2.is(scratch));
  ASSERT(!extra2.is(extra));

  // Check scratch, extra and extra2 registers are valid.
  ASSERT(!scratch.is(no_reg));
  ASSERT(!extra.is(no_reg));
  ASSERT(!extra2.is(no_reg));

  // Check that r3 is not used
  ASSERT(!receiver.is(r3) && !name.is(r3) && !scratch.is(r3) && !extra.is(r3) && !extra2.is(r3));

  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ bt(&miss);

  // Get the map of the receiver and compute the hash.
  __ mov(scratch, FieldMemOperand(name, String::kHashFieldOffset));
  __ mov(r3, FieldMemOperand(receiver, HeapObject::kMapOffset));
  __ add(scratch, scratch, r3);
  __ lxor(scratch, scratch, Immediate(flags));
  __ land(scratch, scratch, Immediate((kPrimaryTableSize - 1) << kHeapObjectTagSize));

  // Probe the primary table.
  ProbeTable(isolate, masm, flags, kPrimary, name, scratch, extra, extra2);

  // Primary miss: Compute hash for secondary probe.
  __ sub(scratch, scratch, name);
  __ add(scratch, scratch, Immediate(flags));
  __ land(scratch, scratch, Immediate((kSecondaryTableSize - 1) << kHeapObjectTagSize));

  // Probe the secondary table.
  ProbeTable(isolate, masm, flags, kSecondary, name, scratch, extra, extra2);

  // Cache miss: Fall-through and let caller handle the miss by
  // entering the runtime system.
  __ bind(&miss);
}


MaybeObject* LoadStubCompiler::CompileLoadField(JSObject* object,
                                                JSObject* holder,
                                                int index,
                                                String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadInterceptor(JSObject* receiver,
                                                      JSObject* holder,
                                                      String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadCallback(String* name,
                                                   JSObject* object,
                                                   JSObject* holder,
                                                   AccessorInfo* callback) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadConstant(JSObject* object,
                                                   JSObject* holder,
                                                   Object* value,
                                                   String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadField(String* name,
                                                     JSObject* receiver,
                                                     JSObject* holder,
                                                     int index) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadGlobal(JSObject* object,
                                                 GlobalObject* holder,
                                                 JSGlobalPropertyCell* cell,
                                                 String* name,
                                                 bool is_dont_delete) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadInterceptor(JSObject* receiver,
                                                           JSObject* holder,
                                                           String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* LoadStubCompiler::CompileLoadNonexistent(String* name,
                                                      JSObject* object,
                                                      JSObject* last) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedStoreStubCompiler::CompileStoreField(JSObject* object,
                                                       int index,
                                                       Map* transition,
                                                       String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* StoreStubCompiler::CompileStoreCallback(JSObject* object,
                                                     AccessorInfo* callback,
                                                     String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* StoreStubCompiler::CompileStoreGlobal(GlobalObject* object,
                                                   JSGlobalPropertyCell* cell,
                                                   String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* StoreStubCompiler::CompileStoreInterceptor(JSObject* receiver,
                                                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* StoreStubCompiler::CompileStoreField(JSObject* object,
                                                  int index,
                                                  Map* transition,
                                                  String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileArrayPushCall(Object* object,
                                                    JSObject* holder,
                                                    JSGlobalPropertyCell* cell,
                                                    JSFunction* function,
                                                    String* name) {
  UNIMPLEMENTED();
  return NULL;
}

MaybeObject* CallStubCompiler::CompileArrayPopCall(Object* object,
                                                   JSObject* holder,
                                                   JSGlobalPropertyCell* cell,
                                                   JSFunction* function,
                                                   String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileCallConstant(Object* object,
                                                   JSObject* holder,
                                                   JSFunction* function,
                                                   String* name,
                                                   CheckType check) {
  UNIMPLEMENTED();
  return NULL;
}


MUST_USE_RESULT MaybeObject* CallStubCompiler::CompileCallField(
    JSObject* object,
    JSObject* holder,
    int index,
    String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileCallGlobal(JSObject* object,
                                                 GlobalObject* holder,
                                                 JSGlobalPropertyCell* cell,
                                                 JSFunction* function,
                                                 String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileCallInterceptor(JSObject* object,
                                                      JSObject* holder,
                                                      String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileFastApiCall(
                        const CallOptimization& optimization,
                        Object* object,
                        JSObject* holder,
                        JSGlobalPropertyCell* cell,
                        JSFunction* function,
                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileMathAbsCall(Object* object,
                                                  JSObject* holder,
                                                  JSGlobalPropertyCell* cell,
                                                  JSFunction* function,
                                                  String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileMathFloorCall(Object* object,
                                                    JSObject* holder,
                                                    JSGlobalPropertyCell* cell,
                                                    JSFunction* function,
                                                    String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileStringCharAtCall(
                        Object* object,
                        JSObject* holder,
                        JSGlobalPropertyCell* cell,
                        JSFunction* function,
                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileStringCharCodeAtCall(
                        Object* object,
                        JSObject* holder,
                        JSGlobalPropertyCell* cell,
                        JSFunction* function,
                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* CallStubCompiler::CompileStringFromCharCodeCall(
                        Object* object,
                        JSObject* holder,
                        JSGlobalPropertyCell* cell,
                        JSFunction* function,
                        String* name) {
  UNIMPLEMENTED();
  return NULL;
}


void ICCompareStub::GenerateHeapNumbers(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void ICCompareStub::GenerateMiss(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void ICCompareStub::GenerateObjects(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


void ICCompareStub::GenerateSmis(MacroAssembler* masm) {
  UNIMPLEMENTED();
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadArrayLength(String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadCallback(
    String* name,
    JSObject* receiver,
    JSObject* holder,
    AccessorInfo* callback) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadConstant(String* name,
                                                        JSObject* receiver,
                                                        JSObject* holder,
                                                        Object* value) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadFunctionPrototype(String* name) {
  UNIMPLEMENTED();
  return NULL;
}



MaybeObject* KeyedLoadStubCompiler::CompileLoadSpecialized(JSObject* receiver) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadStringLength(String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* ConstructStubCompiler::CompileConstructStub(JSFunction* function) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* ExternalArrayStubCompiler::CompileKeyedLoadStub(
    JSObject*receiver, ExternalArrayType array_type, Code::Flags flags) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* ExternalArrayStubCompiler::CompileKeyedStoreStub(
    JSObject* receiver, ExternalArrayType array_type, Code::Flags flags) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedStoreStubCompiler::CompileStoreSpecialized(
    JSObject* receiver) {
  UNIMPLEMENTED();
  return NULL;
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
