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

#include "map-sh4.h" // Define register map

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

  // Check that ip is not used
  ASSERT(!name.is(ip) && !offset.is(ip) && !scratch.is(ip) && !scratch2.is(ip));

  Label miss;
  Register offsets_base_addr = scratch;

  // Check that the key in the entry matches the name.
  __ mov(offsets_base_addr, Immediate(key_offset));
  __ lsl(ip, offset, Immediate(1));
  __ ldr(ip, MemOperand(offsets_base_addr, ip));
  __ cmpeq(name, ip);
  __ bf(&miss);

  // Get the code entry from the cache.
  __ add(offsets_base_addr, offsets_base_addr,
         Immediate(value_off_addr - key_off_addr));
  __ lsl(ip, offset, Immediate(1));
  __ ldr(scratch2, MemOperand(offsets_base_addr, ip));

  // Check that the flags match what we're looking for.
  __ ldr(scratch2, FieldMemOperand(scratch2, Code::kFlagsOffset));
  __ bic(scratch2, scratch2, Immediate(Code::kFlagsNotUsedInLookup));
  __ cmpeq(scratch2, Immediate(flags));
  __ bf(&miss);

  // Re-load code entry from cache.
  __ lsl(ip, offset, Immediate(1));
  __ ldr(offset, MemOperand(offsets_base_addr, ip));

  // Jump to the first instruction in the code stub.
  __ add(offset, offset, Immediate(Code::kHeaderSize - kHeapObjectTag));
  __ jmp(offset);

  // Miss: fall through.
  __ bind(&miss);
}


// Helper function used to check that the dictionary doesn't contain
// the property. This function may return false negatives, so miss_label
// must always call a backup property check that is complete.
// This function is safe to call if the receiver has fast properties.
// Name must be a symbol and receiver must be a heap object.
static void GenerateDictionaryNegativeLookup(MacroAssembler* masm,
                                             Label* miss_label,
                                             Register receiver,
                                             String* name,
                                             Register scratch0,
                                             Register scratch1) {
  ASSERT(name->IsSymbol());
  Counters* counters = masm->isolate()->counters();
  __ IncrementCounter(counters->negative_lookups(), 1, scratch0, scratch1);
  __ IncrementCounter(counters->negative_lookups_miss(), 1, scratch0, scratch1);

  Label done;

  const int kInterceptorOrAccessCheckNeededMask =
      (1 << Map::kHasNamedInterceptor) | (1 << Map::kIsAccessCheckNeeded);

  // Bail out if the receiver has a named interceptor or requires access checks.
  Register map = scratch1;
  __ ldr(map, FieldMemOperand(receiver, HeapObject::kMapOffset));
  __ ldrb(scratch0, FieldMemOperand(map, Map::kBitFieldOffset));
  __ tst(scratch0, Immediate(kInterceptorOrAccessCheckNeededMask));
  __ b(ne, miss_label);

  // Check that receiver is a JSObject.
  __ ldrb(scratch0, FieldMemOperand(map, Map::kInstanceTypeOffset));
  __ cmpge(scratch0, Immediate(FIRST_JS_OBJECT_TYPE));
  __ b(ne, miss_label);

  // Load properties array.
  Register properties = scratch0;
  __ ldr(properties, FieldMemOperand(receiver, JSObject::kPropertiesOffset));
  // Check that the properties array is a dictionary.
  __ ldr(map, FieldMemOperand(properties, HeapObject::kMapOffset));
  Register tmp = properties;
  __ LoadRoot(tmp, Heap::kHashTableMapRootIndex);
  __ cmp(map, tmp);
  __ b(ne, miss_label);

  // Restore the temporarily used register.
  __ ldr(properties, FieldMemOperand(receiver, JSObject::kPropertiesOffset));

  // Compute the capacity mask.
  const int kCapacityOffset =
      StringDictionary::kHeaderSize +
      StringDictionary::kCapacityIndex * kPointerSize;

  // Generate an unrolled loop that performs a few probes before
  // giving up.
  static const int kProbes = 4;
  const int kElementsStartOffset =
      StringDictionary::kHeaderSize +
      StringDictionary::kElementsStartIndex * kPointerSize;

  // If names of slots in range from 1 to kProbes - 1 for the hash value are
  // not equal to the name and kProbes-th slot is not used (its name is the
  // undefined value), it guarantees the hash table doesn't contain the
  // property. It's true even if some slots represent deleted properties
  // (their names are the null value).
  for (int i = 0; i < kProbes; i++) {
    // scratch0 points to properties hash.
    // Compute the masked index: (hash + i + i * i) & mask.
    Register index = scratch1;
    // Capacity is smi 2^n.
    __ ldr(index, FieldMemOperand(properties, kCapacityOffset));
    __ sub(index, index, Immediate(1));
    __ land(index, index, Immediate(
        Smi::FromInt(name->Hash() + StringDictionary::GetProbeOffset(i))));

    // Scale the index by multiplying by the entry size.
    ASSERT(StringDictionary::kEntrySize == 3);
    __ lsl(scratch1, index, Immediate(1));
    __ add(index, index, scratch1);  // index *= 3.

    Register entity_name = scratch1;
    // Having undefined at this place means the name is not contained.
    ASSERT_EQ(kSmiTagSize, 1);
    Register tmp = properties;
    __ lsl(tmp, index, Immediate(1));
    __ add(tmp, properties, tmp);
    __ ldr(entity_name, FieldMemOperand(tmp, kElementsStartOffset));

    ASSERT(!tmp.is(entity_name));
    __ LoadRoot(tmp, Heap::kUndefinedValueRootIndex);
    __ cmp(entity_name, tmp);
    if (i != kProbes - 1) {
      __ b(eq, &done);

      // Stop if found the property.
      __ cmp(entity_name, Immediate(Handle<String>(name)));
      __ b(eq, miss_label);

      // Check if the entry name is not a symbol.
      __ ldr(entity_name, FieldMemOperand(entity_name, HeapObject::kMapOffset));
      __ ldrb(entity_name,
              FieldMemOperand(entity_name, Map::kInstanceTypeOffset));
      __ tst(entity_name, Immediate(kIsSymbolMask));
      __ b(eq, miss_label);

      // Restore the properties.
      __ ldr(properties,
             FieldMemOperand(receiver, JSObject::kPropertiesOffset));
    } else {
      // Give up probing if still not found the undefined value.
      __ b(ne, miss_label);
    }
  }
  __ bind(&done);
  __ DecrementCounter(counters->negative_lookups_miss(), 1, scratch0, scratch1);
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

  // Check that ip is not used
  ASSERT(!receiver.is(ip) && !name.is(ip) && !scratch.is(ip) && !extra.is(ip) && !extra2.is(ip));

  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ b(eq, &miss);

  // Get the map of the receiver and compute the hash.
  __ ldr(scratch, FieldMemOperand(name, String::kHashFieldOffset));
  __ ldr(ip, FieldMemOperand(receiver, HeapObject::kMapOffset));
  __ add(scratch, scratch, ip);
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


void StubCompiler::GenerateLoadGlobalFunctionPrototype(MacroAssembler* masm,
                                                       int index,
                                                       Register prototype) {
  // Load the global or builtins object from the current context.
  __ ldr(prototype, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  // Load the global context from the global or builtins object.
  __ ldr(prototype,
         FieldMemOperand(prototype, GlobalObject::kGlobalContextOffset));
  // Load the function from the global context.
  __ ldr(prototype, MemOperand(prototype, Context::SlotOffset(index)));
  // Load the initial map.  The global functions all have initial maps.
  __ ldr(prototype,
         FieldMemOperand(prototype, JSFunction::kPrototypeOrInitialMapOffset));
  // Load the prototype from the initial map.
  __ ldr(prototype, FieldMemOperand(prototype, Map::kPrototypeOffset));
}


void StubCompiler::GenerateDirectLoadGlobalFunctionPrototype(
    MacroAssembler* masm, int index, Register prototype, Label* miss) {
  Isolate* isolate = masm->isolate();
  // Check we're still in the same context.
  __ ldr(prototype, MemOperand(cp, Context::SlotOffset(Context::GLOBAL_INDEX)));
  __ mov(ip, Immediate(isolate->global()));
  __ cmp(prototype, ip);
  __ b(ne, miss);
  // Get the global function with the given index.
  JSFunction* function =
      JSFunction::cast(isolate->global_context()->get(index));
  // Load its initial map. The global functions all have initial maps.
  __ mov(prototype, Immediate(Handle<Map>(function->initial_map())));
  // Load the prototype from the initial map.
  __ ldr(prototype, FieldMemOperand(prototype, Map::kPrototypeOffset));
}


// Load a fast property out of a holder object (src). In-object properties
// are loaded directly otherwise the property is loaded from the properties
// fixed array.
void StubCompiler::GenerateFastPropertyLoad(MacroAssembler* masm,
                                            Register dst, Register src,
                                            JSObject* holder, int index) {
  // Adjust for the number of properties stored in the holder.
  index -= holder->map()->inobject_properties();
  if (index < 0) {
    // Get the property straight out of the holder.
    int offset = holder->map()->instance_size() + (index * kPointerSize);
    __ ldr(dst, FieldMemOperand(src, offset));
  } else {
    // Calculate the offset into the properties array.
    int offset = index * kPointerSize + FixedArray::kHeaderSize;
    __ ldr(dst, FieldMemOperand(src, JSObject::kPropertiesOffset));
    __ ldr(dst, FieldMemOperand(dst, offset));
  }
}


void StubCompiler::GenerateLoadArrayLength(MacroAssembler* masm,
                                           Register receiver,
                                           Register scratch,
                                           Label* miss_label) {
  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ b(eq, miss_label);

  // Check that the object is a JS array.
  __ CompareObjectType(receiver, scratch, scratch, JS_ARRAY_TYPE);
  __ b(ne, miss_label);

  // Load length directly from the JS array.
  __ ldr(r0, FieldMemOperand(receiver, JSArray::kLengthOffset));
  __ Ret();
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
  ASSERT(!receiver.is(ip) && !scratch1.is(ip) && !scratch2.is(ip));

  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ b(eq, smi);

  // Check that the object is a string.
  __ ldr(scratch1, FieldMemOperand(receiver, HeapObject::kMapOffset));
  __ ldrb(scratch1, FieldMemOperand(scratch1, Map::kInstanceTypeOffset));
  __ land(scratch2, scratch1, Immediate(kIsNotStringMask));
  // The cast is to resolve the overload for the argument of 0x0.
  __ cmp(scratch2, Immediate(static_cast<int32_t>(kStringTag)));
  __ b(ne, non_string_object);
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
  ASSERT(!receiver.is(ip) && !scratch1.is(ip) && !scratch2.is(ip));
  Label check_wrapper;

  // Check if the object is a string leaving the instance type in the
  // scratch1 register.
  GenerateStringCheck(masm, receiver, scratch1, scratch2, miss,
                      support_wrappers ? &check_wrapper : miss);

  // Load length directly from the string.
  __ ldr(r0, FieldMemOperand(receiver, String::kLengthOffset));
  __ Ret();

  if (support_wrappers) {
    // Check if the object is a JSValue wrapper.
    __ bind(&check_wrapper);
    __ cmp(scratch1, Immediate(JS_VALUE_TYPE));
    __ b(ne, miss);

    // Unwrap the value and check if the wrapped value is a string.
    __ ldr(scratch1, FieldMemOperand(receiver, JSValue::kValueOffset));
    GenerateStringCheck(masm, scratch1, scratch2, scratch2, miss, miss);
    __ ldr(r0, FieldMemOperand(scratch1, String::kLengthOffset));
    __ Ret();
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


// Generate StoreField code, value is passed in r0 register.
// When leaving generated code after success, the receiver_reg and name_reg
// may be clobbered.  Upon branch to miss_label, the receiver and name
// registers have their original values.
void StubCompiler::GenerateStoreField(MacroAssembler* masm,
                                      JSObject* object,
                                      int index,
                                      Map* transition,
                                      Register receiver_reg,
                                      Register name_reg,
                                      Register scratch,
                                      Label* miss_label) {
  // r0 : value
  Label exit;

  // Check that the receiver isn't a smi.
  __ tst(receiver_reg, Immediate(kSmiTagMask));
  __ b(eq, miss_label);

  // Check that the map of the receiver hasn't changed.
  __ ldr(scratch, FieldMemOperand(receiver_reg, HeapObject::kMapOffset));
  __ cmp(scratch, Immediate(Handle<Map>(object->map())));
  __ b(ne, miss_label);

  // Perform global security token check if needed.
  if (object->IsJSGlobalProxy()) {
    __ CheckAccessGlobalProxy(receiver_reg, scratch, miss_label);
  }

  // Stub never generated for non-global objects that require access
  // checks.
  ASSERT(object->IsJSGlobalProxy() || !object->IsAccessCheckNeeded());

  // Perform map transition for the receiver if necessary.
  if ((transition != NULL) && (object->map()->unused_property_fields() == 0)) {
    // The properties must be extended before we can store the value.
    // We jump to a runtime call that extends the properties array.
    __ push(receiver_reg);
    __ mov(r2, Operand(Handle<Map>(transition)));
    __ Push(r2, r0);
    __ TailCallExternalReference(
        ExternalReference(IC_Utility(IC::kSharedStoreIC_ExtendStorage),
                          masm->isolate()),
        3,
        1);
    return;
  }

  if (transition != NULL) {
    // Update the map of the object; no write barrier updating is
    // needed because the map is never in new space.
    __ mov(ip, Operand(Handle<Map>(transition)));
    __ str(ip, FieldMemOperand(receiver_reg, HeapObject::kMapOffset));
  }

  // Adjust for the number of properties stored in the object. Even in the
  // face of a transition we can use the old map here because the size of the
  // object and the number of in-object properties is not going to change.
  index -= object->map()->inobject_properties();

  if (index < 0) {
    // Set the property straight into the object.
    int offset = object->map()->instance_size() + (index * kPointerSize);
    __ str(r0, FieldMemOperand(receiver_reg, offset));

    // Skip updating write barrier if storing a smi.
    __ tst(r0, Immediate(kSmiTagMask));
    __ b(eq, &exit);

    // Update the write barrier for the array address.
    // Pass the now unused name_reg as a scratch register.
    __ RecordWrite(receiver_reg, offset, name_reg, scratch);
  } else {
    // Write to the properties array.
    int offset = index * kPointerSize + FixedArray::kHeaderSize;
    // Get the properties array
    __ ldr(scratch, FieldMemOperand(receiver_reg, JSObject::kPropertiesOffset));
    __ str(r0, FieldMemOperand(scratch, offset));

    // Skip updating write barrier if storing a smi.
    __ tst(r0, Immediate(kSmiTagMask));
    __ b(eq, &exit);

    // Update the write barrier for the array address.
    // Ok to clobber receiver_reg and name_reg, since we return.
    __ RecordWrite(scratch, offset, name_reg, receiver_reg);
  }

  // Return the value (register r0).
  __ bind(&exit);
  __ Ret();
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


// Generate code to check that a global property cell is empty. Create
// the property cell at compilation time if no cell exists for the
// property.
MUST_USE_RESULT static MaybeObject* GenerateCheckPropertyCell(
    MacroAssembler* masm,
    GlobalObject* global,
    String* name,
    Register scratch,
    Label* miss) {
  Object* probe;
  { MaybeObject* maybe_probe = global->EnsurePropertyCell(name);
    if (!maybe_probe->ToObject(&probe)) return maybe_probe;
  }
  JSGlobalPropertyCell* cell = JSGlobalPropertyCell::cast(probe);
  ASSERT(cell->value()->IsTheHole());
  __ mov(scratch, Operand(Handle<Object>(cell)));
  __ ldr(scratch,
         FieldMemOperand(scratch, JSGlobalPropertyCell::kValueOffset));
  __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
  __ cmp(scratch, ip);
  __ b(ne, miss);
  return cell;
}

// Calls GenerateCheckPropertyCell for each global object in the prototype chain
// from object to (but not including) holder.
MUST_USE_RESULT static MaybeObject* GenerateCheckPropertyCells(
    MacroAssembler* masm,
    JSObject* object,
    JSObject* holder,
    String* name,
    Register scratch,
    Label* miss) {
  JSObject* current = object;
  while (current != holder) {
    if (current->IsGlobalObject()) {
      // Returns a cell or a failure.
      MaybeObject* result = GenerateCheckPropertyCell(
          masm,
          GlobalObject::cast(current),
          name,
          scratch,
          miss);
      if (result->IsFailure()) return result;
    }
    ASSERT(current->IsJSObject());
    current = JSObject::cast(current->GetPrototype());
  }
  return NULL;
}

#undef __
#define __ ACCESS_MASM(masm())

Register StubCompiler::CheckPrototypes(JSObject* object,
                                       Register object_reg,
                                       JSObject* holder,
                                       Register holder_reg,
                                       Register scratch1,
                                       Register scratch2,
                                       String* name,
                                       int save_at_depth,
                                       Label* miss) {
  // Make sure there's no overlap between holder and object registers.
  ASSERT(!scratch1.is(object_reg) && !scratch1.is(holder_reg));
  ASSERT(!scratch2.is(object_reg) && !scratch2.is(holder_reg)
         && !scratch2.is(scratch1));

  // Keep track of the current object in register reg.
  Register reg = object_reg;
  int depth = 0;

  if (save_at_depth == depth) {
    __ str(reg, MemOperand(sp));
  }

  // Check the maps in the prototype chain.
  // Traverse the prototype chain from the object and do map checks.
  JSObject* current = object;
  while (current != holder) {
    depth++;

    // Only global objects and objects that do not require access
    // checks are allowed in stubs.
    ASSERT(current->IsJSGlobalProxy() || !current->IsAccessCheckNeeded());

    ASSERT(current->GetPrototype()->IsJSObject());
    JSObject* prototype = JSObject::cast(current->GetPrototype());
    if (!current->HasFastProperties() &&
        !current->IsJSGlobalObject() &&
        !current->IsJSGlobalProxy()) {
      if (!name->IsSymbol()) {
        MaybeObject* maybe_lookup_result = heap()->LookupSymbol(name);
        Object* lookup_result = NULL;  // Initialization to please compiler.
        if (!maybe_lookup_result->ToObject(&lookup_result)) {
          set_failure(Failure::cast(maybe_lookup_result));
          return reg;
        }
        name = String::cast(lookup_result);
      }
      ASSERT(current->property_dictionary()->FindEntry(name) ==
             StringDictionary::kNotFound);

      GenerateDictionaryNegativeLookup(masm(),
                                       miss,
                                       reg,
                                       name,
                                       scratch1,
                                       scratch2);
      __ ldr(scratch1, FieldMemOperand(reg, HeapObject::kMapOffset));
      reg = holder_reg;  // from now the object is in holder_reg
      __ ldr(reg, FieldMemOperand(scratch1, Map::kPrototypeOffset));
    } else if (heap()->InNewSpace(prototype)) {
      // Get the map of the current object.
      __ ldr(scratch1, FieldMemOperand(reg, HeapObject::kMapOffset));
      __ cmp(scratch1, Immediate(Handle<Map>(current->map())));

      // Branch on the result of the map check.
      __ b(ne, miss);

      // Check access rights to the global object.  This has to happen
      // after the map check so that we know that the object is
      // actually a global object.
      if (current->IsJSGlobalProxy()) {
        __ CheckAccessGlobalProxy(reg, scratch1, miss);
        // Restore scratch register to be the map of the object.  In the
        // new space case below, we load the prototype from the map in
        // the scratch register.
        __ ldr(scratch1, FieldMemOperand(reg, HeapObject::kMapOffset));
      }

      reg = holder_reg;  // from now the object is in holder_reg
      // The prototype is in new space; we cannot store a reference
      // to it in the code. Load it from the map.
      __ ldr(reg, FieldMemOperand(scratch1, Map::kPrototypeOffset));
    } else {
      // Check the map of the current object.
      __ ldr(scratch1, FieldMemOperand(reg, HeapObject::kMapOffset));
      __ cmp(scratch1, Immediate(Handle<Map>(current->map())));
      // Branch on the result of the map check.
      __ b(ne, miss);
      // Check access rights to the global object.  This has to happen
      // after the map check so that we know that the object is
      // actually a global object.
      if (current->IsJSGlobalProxy()) {
        __ CheckAccessGlobalProxy(reg, scratch1, miss);
      }
      // The prototype is in old space; load it directly.
      reg = holder_reg;  // from now the object is in holder_reg
      __ mov(reg, Immediate(Handle<JSObject>(prototype)));
    }

    if (save_at_depth == depth) {
      __ str(reg, MemOperand(sp));
    }

    // Go to the next object in the prototype chain.
    current = prototype;
  }

  // Check the holder map.
  __ ldr(scratch1, FieldMemOperand(reg, HeapObject::kMapOffset));
  __ cmp(scratch1, Immediate(Handle<Map>(current->map())));
  __ b(ne, miss);

  // Log the check depth.
  LOG(masm()->isolate(), IntEvent("check-maps-depth", depth + 1));

  // Perform security check for access to the global object.
  ASSERT(holder->IsJSGlobalProxy() || !holder->IsAccessCheckNeeded());
  if (holder->IsJSGlobalProxy()) {
    __ CheckAccessGlobalProxy(reg, scratch1, miss);
  };

  // If we've skipped any global objects, it's not enough to verify
  // that their maps haven't changed.  We also need to check that the
  // property cell for the property is still empty.
  MaybeObject* result = GenerateCheckPropertyCells(masm(),
                                                   object,
                                                   holder,
                                                   name,
                                                   scratch1,
                                                   miss);
  if (result->IsFailure()) set_failure(Failure::cast(result));

  // Return the register containing the holder.
  return reg;
}


void StubCompiler::GenerateLoadField(JSObject* object,
                                     JSObject* holder,
                                     Register receiver,
                                     Register scratch1,
                                     Register scratch2,
                                     Register scratch3,
                                     int index,
                                     String* name,
                                     Label* miss) {
  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ b(eq, miss);

  // Check that the maps haven't changed.
  Register reg =
      CheckPrototypes(object, receiver, holder, scratch1, scratch2, scratch3,
                      name, miss);
  GenerateFastPropertyLoad(masm(), r0, reg, holder, index);
  __ Ret();
}


void StubCompiler::GenerateLoadConstant(JSObject* object,
                                        JSObject* holder,
                                        Register receiver,
                                        Register scratch1,
                                        Register scratch2,
                                        Register scratch3,
                                        Object* value,
                                        String* name,
                                        Label* miss) {
  // Check that the receiver isn't a smi.
  __ tst(receiver, Immediate(kSmiTagMask));
  __ b(eq, miss);

  // Check that the maps haven't changed.
  Register reg =
      CheckPrototypes(object, receiver, holder,
                      scratch1, scratch2, scratch3, name, miss);

  // Return the constant value.
  __ mov(r0, Immediate(Handle<Object>(value)));
  __ Ret();
}


void CallStubCompiler::GenerateNameCheck(String* name, Label* miss) {
  if (kind_ == Code::KEYED_CALL_IC) {
    __ cmp(r2, Immediate(Handle<String>(name)));
    __ b(ne, miss);
  }
}


void CallStubCompiler::GenerateGlobalReceiverCheck(JSObject* object,
                                                   JSObject* holder,
                                                   String* name,
                                                   Label* miss) {
  ASSERT(holder->IsGlobalObject());

  // Get the number of arguments.
  const int argc = arguments().immediate();

  // Get the receiver from the stack.
  __ ldr(r0, MemOperand(sp, argc * kPointerSize));

  // If the object is the holder then we know that it's a global
  // object which can only happen for contextual calls. In this case,
  // the receiver cannot be a smi.
  if (object != holder) {
    __ tst(r0, Immediate(kSmiTagMask));
    __ b(eq, miss);
  }

  // Check that the maps haven't changed.
  CheckPrototypes(object, r0, holder, r3, r1, r4, name, miss);
}


void CallStubCompiler::GenerateLoadFunctionFromCell(JSGlobalPropertyCell* cell,
                                                    JSFunction* function,
                                                    Label* miss) {
  // Get the value from the cell.
  __ mov(r3, Operand(Handle<JSGlobalPropertyCell>(cell)));
  __ ldr(r1, FieldMemOperand(r3, JSGlobalPropertyCell::kValueOffset));

  // Check that the cell contains the same function.
  if (heap()->InNewSpace(function)) {
    // We can't embed a pointer to a function in new space so we have
    // to verify that the shared function info is unchanged. This has
    // the nice side effect that multiple closures based on the same
    // function can all use this call IC. Before we load through the
    // function, we have to verify that it still is a function.
    __ tst(r1, Immediate(kSmiTagMask));
    __ b(eq, miss);
    __ CompareObjectType(r1, r3, r3, JS_FUNCTION_TYPE);
    __ b(ne, miss);

    // Check the shared function info. Make sure it hasn't changed.
    __ mov(r3, Operand(Handle<SharedFunctionInfo>(function->shared())));
    __ ldr(r4, FieldMemOperand(r1, JSFunction::kSharedFunctionInfoOffset));
    __ cmp(r4, r3);
    __ b(ne, miss);
  } else {
    __ cmp(r1, Immediate(Handle<JSFunction>(function)));
    __ b(ne, miss);
  }
}


MaybeObject* CallStubCompiler::GenerateMissBranch() {
  MaybeObject* maybe_obj = masm()->isolate()->stub_cache()->ComputeCallMiss(
      arguments().immediate(), kind_);
  Object* obj;
  if (!maybe_obj->ToObject(&obj)) return maybe_obj;
  __ Jump(Handle<Code>(Code::cast(obj)), RelocInfo::CODE_TARGET);
  return obj;
}


MaybeObject* CallStubCompiler::CompileCallField(JSObject* object,
                                                JSObject* holder,
                                                int index,
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


MaybeObject* LoadStubCompiler::CompileLoadField(JSObject* object,
                                                JSObject* holder,
                                                int index,
                                                String* name) {
  // ----------- S t a t e -------------
  //  -- r0    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  Label miss;

  GenerateLoadField(object, holder, r0, r3, r1, r4, index, name, &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::LOAD_IC);

  // Return the generated code.
  return GetCode(FIELD, name);
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
  // ----------- S t a t e -------------
  //  -- r0    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  Label miss;

  GenerateLoadConstant(object, holder, r0, r3, r1, r4, value, name, &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::LOAD_IC);

  // Return the generated code.
  return GetCode(CONSTANT_FUNCTION, name);
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
  // ----------- S t a t e -------------
  //  -- r0    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  Label miss;

  // If the object is the holder then we know that it's a global
  // object which can only happen for contextual calls. In this case,
  // the receiver cannot be a smi.
  if (object != holder) {
    __ tst(r0, Immediate(kSmiTagMask));
    __ b(eq, &miss);
  }

  // Check that the map of the global has not changed.
  CheckPrototypes(object, r0, holder, r3, r4, r1, name, &miss);

  // Get the value from the cell.
  __ mov(r3, Operand(Handle<JSGlobalPropertyCell>(cell)));
  __ ldr(r4, FieldMemOperand(r3, JSGlobalPropertyCell::kValueOffset));

  // Check for deleted property if property can actually be deleted.
  if (!is_dont_delete) {
    __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
    __ cmp(r4, ip);
    __ b(eq, &miss);
  }

  __ mov(r0, r4);
  Counters* counters = masm()->isolate()->counters();
  __ IncrementCounter(counters->named_load_global_stub(), 1, r1, r3);
  __ Ret();

  __ bind(&miss);
  __ IncrementCounter(counters->named_load_global_stub_miss(), 1, r1, r3);
  GenerateLoadMiss(masm(), Code::LOAD_IC);

  // Return the generated code.
  return GetCode(NORMAL, name);
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
  // ----------- S t a t e -------------
  //  -- r0    : receiver
  //  -- lr    : return address
  // -----------------------------------
  Label miss;

  // Check that receiver is not a smi.
  __ tst(r0, Immediate(kSmiTagMask));
  __ b(eq, &miss);

  // Check the maps of the full prototype chain.
  CheckPrototypes(object, r0, last, r3, r1, r4, name, &miss);

  // If the last object in the prototype chain is a global object,
  // check that the global property cell is empty.
  if (last->IsGlobalObject()) {
    MaybeObject* cell = GenerateCheckPropertyCell(masm(),
                                                  GlobalObject::cast(last),
                                                  name,
                                                  r1,
                                                  &miss);
    if (cell->IsFailure()) {
      miss.Unuse();
      return cell;
    }
  }

  // Return undefined if maps of the full prototype chain are still the
  // same and no global property with this name contains a value.
  __ LoadRoot(r0, Heap::kUndefinedValueRootIndex);
  __ Ret();

  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::LOAD_IC);

  // Return the generated code.
  return GetCode(NONEXISTENT, heap()->empty_string());
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
  // ----------- S t a t e -------------
  //  -- r0    : value
  //  -- r1    : receiver
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  Label miss;

  GenerateStoreField(masm(),
                     object,
                     index,
                     transition,
                     r1, r2, r3,
                     &miss);
  __ bind(&miss);
  Handle<Code> ic = masm()->isolate()->builtins()->StoreIC_Miss();
  __ Jump(ic, RelocInfo::CODE_TARGET);

  // Return the generated code.
  return GetCode(transition == NULL ? FIELD : MAP_TRANSITION, name);
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
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------
  if (HasCustomCallGenerator(function)) {
    MaybeObject* maybe_result = CompileCustomCall(
        object, holder, NULL, function, name);
    Object* result;
    if (!maybe_result->ToObject(&result)) return maybe_result;
    // undefined means bail out to regular compiler.
    if (!result->IsUndefined()) return result;
  }

  Label miss;

  GenerateNameCheck(name, &miss);

  // Get the receiver from the stack
  const int argc = arguments().immediate();
  __ ldr(r1, MemOperand(sp, argc * kPointerSize));

  // Check that the receiver isn't a smi.
  if (check != NUMBER_CHECK) {
    __ tst(r1, Immediate(kSmiTagMask));
    __ b(eq, &miss);
  }

  // Make sure that it's okay not to patch the on stack receiver
  // unless we're doing a receiver map check.
  ASSERT(!object->IsGlobalObject() || check == RECEIVER_MAP_CHECK);

  SharedFunctionInfo* function_info = function->shared();
  switch (check) {
    case RECEIVER_MAP_CHECK:
      __ IncrementCounter(masm()->isolate()->counters()->call_const(),
                          1, r0, r3);

      // Check that the maps haven't changed.
      CheckPrototypes(JSObject::cast(object), r1, holder, r0, r3, r4, name,
                      &miss);

      // Patch the receiver on the stack with the global proxy if
      // necessary.
      if (object->IsGlobalObject()) {
        __ ldr(r3, FieldMemOperand(r1, GlobalObject::kGlobalReceiverOffset));
        __ str(r3, MemOperand(sp, argc * kPointerSize));
      }
      break;

    case STRING_CHECK:
      if (!function->IsBuiltin() && !function_info->strict_mode()) {
        // Calling non-strict non-builtins with a value as the receiver
        // requires boxing.
        __ jmp(&miss);
      } else {
        // Check that the object is a two-byte string or a symbol.
        __ CompareObjectType(r1, r3, r3, FIRST_NONSTRING_TYPE, hs);
        __ b(t, &miss);
        // Check that the maps starting from the prototype haven't changed.
        GenerateDirectLoadGlobalFunctionPrototype(
            masm(), Context::STRING_FUNCTION_INDEX, r0, &miss);
        CheckPrototypes(JSObject::cast(object->GetPrototype()), r0, holder, r3,
                        r1, r4, name, &miss);
      }
      break;

    case NUMBER_CHECK: {
      if (!function->IsBuiltin() && !function_info->strict_mode()) {
        // Calling non-strict non-builtins with a value as the receiver
        // requires boxing.
        __ jmp(&miss);
      } else {
        Label fast;
        // Check that the object is a smi or a heap number.
        __ tst(r1, Immediate(kSmiTagMask));
        __ b(eq, &fast);
        __ CompareObjectType(r1, r0, r0, HEAP_NUMBER_TYPE);
        __ b(ne, &miss);
        __ bind(&fast);
        // Check that the maps starting from the prototype haven't changed.
        GenerateDirectLoadGlobalFunctionPrototype(
            masm(), Context::NUMBER_FUNCTION_INDEX, r0, &miss);
        CheckPrototypes(JSObject::cast(object->GetPrototype()), r0, holder, r3,
                        r1, r4, name, &miss);
      }
      break;
    }

    case BOOLEAN_CHECK: {
      if (!function->IsBuiltin() && !function_info->strict_mode()) {
        // Calling non-strict non-builtins with a value as the receiver
        // requires boxing.
        __ jmp(&miss);
      } else {
        Label fast;
        // Check that the object is a boolean.
        __ LoadRoot(ip, Heap::kTrueValueRootIndex);
        __ cmp(r1, ip);
        __ b(eq, &fast);
        __ LoadRoot(ip, Heap::kFalseValueRootIndex);
        __ cmp(r1, ip);
        __ b(ne, &miss);
        __ bind(&fast);
        // Check that the maps starting from the prototype haven't changed.
        GenerateDirectLoadGlobalFunctionPrototype(
            masm(), Context::BOOLEAN_FUNCTION_INDEX, r0, &miss);
        CheckPrototypes(JSObject::cast(object->GetPrototype()), r0, holder, r3,
                        r1, r4, name, &miss);
      }
      break;
    }

    default:
      UNREACHABLE();
  }

  __ InvokeFunction(function, arguments(), JUMP_FUNCTION);

  // Handle call cache miss.
  __ bind(&miss);
  MaybeObject* maybe_result = GenerateMissBranch();
  if (maybe_result->IsFailure()) return maybe_result;

  // Return the generated code.
  return GetCode(function);
}


MaybeObject* CallStubCompiler::CompileCallGlobal(JSObject* object,
                                                 GlobalObject* holder,
                                                 JSGlobalPropertyCell* cell,
                                                 JSFunction* function,
                                                 String* name) {
  // ----------- S t a t e -------------
  //  -- r2    : name
  //  -- lr    : return address
  // -----------------------------------

  if (HasCustomCallGenerator(function)) {
    MaybeObject* maybe_result = CompileCustomCall(
        object, holder, cell, function, name);
    Object* result;
    if (!maybe_result->ToObject(&result)) return maybe_result;
    // undefined means bail out to regular compiler.
    if (!result->IsUndefined()) return result;
  }

  Label miss;

  GenerateNameCheck(name, &miss);

  // Get the number of arguments.
  const int argc = arguments().immediate();

  GenerateGlobalReceiverCheck(object, holder, name, &miss);

  GenerateLoadFunctionFromCell(cell, function, &miss);

  // Patch the receiver on the stack with the global proxy if
  // necessary.
  if (object->IsGlobalObject()) {
    __ ldr(r3, FieldMemOperand(r0, GlobalObject::kGlobalReceiverOffset));
    __ str(r3, MemOperand(sp, argc * kPointerSize));
  }

  // Setup the context (function already in r1).
  __ ldr(cp, FieldMemOperand(r1, JSFunction::kContextOffset));

  // Jump to the cached code (tail call).
  Counters* counters = masm()->isolate()->counters();
  __ IncrementCounter(counters->call_global_inline(), 1, r3, r4);
  ASSERT(function->is_compiled());
  Handle<Code> code(function->code());
  ParameterCount expected(function->shared()->formal_parameter_count());
  if (V8::UseCrankshaft()) {
    // TODO(kasperl): For now, we always call indirectly through the
    // code field in the function to allow recompilation to take effect
    // without changing any of the call sites.
    __ ldr(r3, FieldMemOperand(r1, JSFunction::kCodeEntryOffset));
    __ InvokeCode(r3, expected, arguments(), JUMP_FUNCTION);
  } else {
    __ InvokeCode(code, expected, arguments(),
                  RelocInfo::CODE_TARGET, JUMP_FUNCTION);
  }

  // Handle call cache miss.
  __ bind(&miss);
  __ IncrementCounter(counters->call_global_inline_miss(), 1, r1, r3);
  MaybeObject* maybe_result = GenerateMissBranch();
  if (maybe_result->IsFailure()) return maybe_result;

  // Return the generated code.
  return GetCode(NORMAL, name);
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


void ICCompareStub::GenerateObjects(MacroAssembler* masm) {
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
  // ----------- S t a t e -------------
  //  -- lr    : return address
  //  -- r0    : key
  //  -- r1    : receiver
  // -----------------------------------
  Label miss;

  // Check the key is the cached one.
  __ cmp(r0, Immediate(Handle<String>(name)));
  __ b(ne, &miss);

  GenerateLoadConstant(receiver, holder, r1, r2, r3, r4, value, name, &miss);
  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);

  // Return the generated code.
  return GetCode(CONSTANT_FUNCTION, name);
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadFunctionPrototype(String* name) {
  UNIMPLEMENTED();
  return NULL;
}


MaybeObject* KeyedLoadStubCompiler::CompileLoadSpecialized(JSObject* receiver) {
  // ----------- S t a t e -------------
  //  -- lr    : return address
  //  -- r0    : key
  //  -- r1    : receiver
  // -----------------------------------
  Label miss;

  // Check that the receiver isn't a smi.
  __ tst(r1, Immediate(kSmiTagMask));
  __ b(eq, &miss);

  // Check that the map matches.
  __ ldr(r2, FieldMemOperand(r1, HeapObject::kMapOffset));
  __ cmp(r2, Immediate(Handle<Map>(receiver->map())));
  __ b(ne, &miss);

  // Check that the key is a smi.
  __ tst(r0, Immediate(kSmiTagMask));
  __ b(ne, &miss);

  // Get the elements array.
  __ ldr(r2, FieldMemOperand(r1, JSObject::kElementsOffset));
  __ AssertFastElements(r2);

  // Check that the key is within bounds.
  __ ldr(r3, FieldMemOperand(r2, FixedArray::kLengthOffset));
  __ cmphs(r0, r3);
  __ b(t, &miss);

  // Load the result and make sure it's not the hole.
  __ add(r3, r2, Immediate(FixedArray::kHeaderSize - kHeapObjectTag));
  ASSERT(kSmiTag == 0 && kSmiTagSize < kPointerSizeLog2);
  __ lsl(ip, r0, Immediate(kPointerSizeLog2 - kSmiTagSize));
  __ ldr(r4, MemOperand(r3, ip));
  __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
  __ cmp(r4, ip);
  __ b(eq, &miss);
  __ mov(r0, r4);
  __ Ret();

  __ bind(&miss);
  GenerateLoadMiss(masm(), Code::KEYED_LOAD_IC);

  // Return the generated code.
  return GetCode(NORMAL, NULL);
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
