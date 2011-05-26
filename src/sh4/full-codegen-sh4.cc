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

#include "code-stubs.h"
#include "codegen.h"
#include "compiler.h"
#include "debug.h"
#include "full-codegen.h"
#include "parser.h"
#include "scopes.h"
#include "stub-cache.h"

namespace v8 {
namespace internal {

#define __ ACCESS_MASM(masm_)


// A patch site is a location in the code which it is possible to patch. This
// class has a number of methods to emit the code which is patchable and the
// method EmitPatchInfo to record a marker back to the patchable code. 
// On SH4 this marker is a cmp #uu, r0 operation, this limits the range
// of #uu to 0..+255 instructions for the distance betwen the patch and the label.
// The #uu (8 bits interpreted as unsigned value) is the delta from the pc to 
// the first instruction of the patchable code.
// Note: the delta is always positive, hence, we can encode the unsigned value
// and use the full 8 bits range.
class JumpPatchSite BASE_EMBEDDED {
 public:
  explicit JumpPatchSite(MacroAssembler* masm) : masm_(masm) {
#ifdef DEBUG
    info_emitted_ = false;
#endif
  }

  ~JumpPatchSite() {
    ASSERT(patch_site_.is_bound() == info_emitted_);
  }

  // When initially emitting this ensure that a jump is always generated to skip
  // the inlined smi code.
  void EmitJumpIfNotSmi(Register reg, Label* target) {
    ASSERT(!patch_site_.is_bound() && !info_emitted_);
    __ bind(&patch_site_);
    __ cmpeq(reg, reg);
    // Don't use b(al, ...) as that might emit the constant pool right after the
    // branch. After patching when the branch is no longer unconditional
    // execution can continue into the constant pool.
    __ bt(target);  // Always taken before patched.
  }

  // When initially emitting this ensure that a jump is never generated to skip
  // the inlined smi code.
  void EmitJumpIfSmi(Register reg, Label* target) {
    ASSERT(!patch_site_.is_bound() && !info_emitted_);
    __ bind(&patch_site_);
    __ cmpeq(reg, reg);
    __ bf(target);  // Never taken before patched.
  }

  void EmitPatchInfo() {
    int delta_to_patch_site = masm_->InstructionsGeneratedSince(&patch_site_);
    ASSERT(delta_to_patch_site >= 0);
    // Ensure that the delta fits into the raw immediate.
    ASSERT(masm_->fits_raw_immediate(delta_to_patch_site));
    __ cmpeq_r0_raw_immediate(delta_to_patch_site);
#ifdef DEBUG
    info_emitted_ = true;
#endif
  }

  bool is_bound() const { return patch_site_.is_bound(); }

 private:
  MacroAssembler* masm_;
  Label patch_site_;
#ifdef DEBUG
  bool info_emitted_;
#endif
};


// Generate code for a JS function.  On entry to the function the receiver
// and arguments have been pushed on the stack left to right.  The actual
// argument count matches the formal parameter count expected by the
// function.
//
// The live registers are:
//   o r5: the JS function object being called (ie, ourselves)
//   o cp: our context
//   o fp: our caller's frame pointer
//   o sp: stack pointer
//   o pr: return address
//
// The function builds a JS frame.  Please see JavaScriptFrameConstants in
// frames-sh4.h for its layout.
void FullCodeGenerator::Generate(CompilationInfo* info) {
  ASSERT(info_ == NULL);
  info_ = info;
  SetFunctionPosition(function());
  Comment cmnt(masm_, "[ function compiled by full code generator");

#ifdef DEBUG
  if (strlen(FLAG_stop_at) > 0 &&
      info->function()->name()->IsEqualTo(CStrVector(FLAG_stop_at))) {
    __ stop("stop-at");
  }
#endif

  int locals_count = scope()->num_stack_slots();

  __ Push(pr, fp, cp, r5);
  if (locals_count > 0) {
    // Load undefined value here, so the value is ready for the loop
    // below.
    __ LoadRoot(r0, Heap::kUndefinedValueRootIndex);
  }
  // Adjust fp to point to caller's fp.
  __ add(fp, sp, Immediate(2 * kPointerSize));

  { Comment cmnt(masm_, "[ Allocate locals");
    for (int i = 0; i < locals_count; i++) {
      __ push(r0);
    }
  }

  bool function_in_register = true;

  // Possibly allocate a local context.
  int heap_slots = scope()->num_heap_slots() - Context::MIN_CONTEXT_SLOTS;
  if (heap_slots > 0) {
    Comment cmnt(masm_, "[ Allocate local context");
    // Argument to NewContext is the function, which is in r5.
    __ push(r5);
    if (heap_slots <= FastNewContextStub::kMaximumSlots) {
      FastNewContextStub stub(heap_slots);
      __ CallStub(&stub);
    } else {
      __ CallRuntime(Runtime::kNewContext, 1);
    }
    function_in_register = false;
    // Context is returned in both r0 and cp.  It replaces the context
    // passed to us.  It's saved in the stack and kept live in cp.
    __ mov(MemOperand(fp, StandardFrameConstants::kContextOffset), cp);
    // Copy any necessary parameters into the context.
    int num_parameters = scope()->num_parameters();
    for (int i = 0; i < num_parameters; i++) {
      Slot* slot = scope()->parameter(i)->AsSlot();
      if (slot != NULL && slot->type() == Slot::CONTEXT) {
        int parameter_offset = StandardFrameConstants::kCallerSPOffset +
            (num_parameters - 1 - i) * kPointerSize;
        // Load parameter from stack.
        __ mov(r0, MemOperand(fp, parameter_offset));
        // Store it in the context.
        __ mov(MemOperand(cp, Context::SlotOffset(slot->index())), r0);
        // Update the write barrier. This clobbers all involved
        // registers, so we have to use two more registers to avoid
        // clobbering cp.
        __ mov(r0, cp);
        __ RecordWrite(r0/*input/scratch*/, Context::SlotOffset(slot->index()),
		       r1 /*scratch*/, r2 /*scratch*/);
      }
    }
  }

  Variable* arguments = scope()->arguments();
  if (arguments != NULL) {
    // Function uses arguments object.
    Comment cmnt(masm_, "[ Allocate arguments object");
    if (!function_in_register) {
      // Load this again, if it's used by the local context below.
      __ mov(r0, MemOperand(fp, JavaScriptFrameConstants::kFunctionOffset));
    } else {
      __ mov(r0, r5);
    }
    // Receiver is just before the parameters on the caller's stack.
    int offset = scope()->num_parameters() * kPointerSize;
    __ add(r2, fp,
           Immediate(StandardFrameConstants::kCallerSPOffset + offset));
    __ mov(r1, Immediate(Smi::FromInt(scope()->num_parameters())));
    __ Push(r0, r2, r1);

    // Arguments to ArgumentsAccessStub:
    //   function, receiver address, parameter count.
    // The stub will rewrite receiever and parameter count if the previous
    // stack frame was an arguments adapter frame.
    ArgumentsAccessStub stub(
        is_strict_mode() ? ArgumentsAccessStub::NEW_STRICT
                         : ArgumentsAccessStub::NEW_NON_STRICT);
    __ CallStub(&stub);

    Variable* arguments_shadow = scope()->arguments_shadow();
    if (arguments_shadow != NULL) {
      // Duplicate the value; move-to-slot operation might clobber registers.
      __ mov(r4, r0);
      Move(arguments_shadow->AsSlot(), r4/*src*/, r1/*scratch*/, r2/*scratch*/);
    }
    Move(arguments->AsSlot(), r0/*src*/, r1/*scratch*/, r2/*scratch*/);
  }

  if (FLAG_trace) {
    __ CallRuntime(Runtime::kTraceEnter, 0);
  }

  // Visit the declarations and body unless there is an illegal
  // redeclaration.
  if (scope()->HasIllegalRedeclaration()) {
    Comment cmnt(masm_, "[ Declarations");
    scope()->VisitIllegalRedeclaration(this);

  } else {
    { Comment cmnt(masm_, "[ Declarations");
      // For named function expressions, declare the function name as a
      // constant.
      if (scope()->is_function_scope() && scope()->function() != NULL) {
        EmitDeclaration(scope()->function(), Variable::CONST, NULL);
      }
      VisitDeclarations(scope()->declarations());
    }

    { Comment cmnt(masm_, "[ Stack check");
      PrepareForBailoutForId(AstNode::kFunctionEntryId, NO_REGISTERS);
      Label ok;
      __ LoadRoot(r0, Heap::kStackLimitRootIndex);
      __ cmpgeu(sp, r0);
      __ bt(&ok);
      StackCheckStub stub;
      __ CallStub(&stub);
      __ bind(&ok);
    }

    { Comment cmnt(masm_, "[ Body");
      ASSERT(loop_depth() == 0);
      VisitStatements(function()->body());
      ASSERT(loop_depth() == 0);
    }
  }

  // Always emit a 'return undefined' in case control fell off the end of
  // the body.
  { Comment cmnt(masm_, "[ return <undefined>;");
    __ LoadRoot(r0, Heap::kUndefinedValueRootIndex);
  }
  EmitReturnSequence();

  // TODO: implement this when const pool are active
  // Force emit the constant pool, so it doesn't get emitted in the middle
  // of the stack check table.
  //masm()->CheckConstPool(true, false);
}


//
// ************************************************
// *** WARNING: ARM to SH4 mapping. READ CAREFULLY.
// ************************************************
// Starting from this point, we use a systematic mapping for converting
// ARM code to SH4 code.
// Some registers must be checked, see defines below.
// if needing an additional register, use sh4_r8
// all other registers unchanged
//
// For this we define a fixed mapping based on #define.
// All functions should come from the ARM implementation
// in ic-arm.cc
// If this latter file is updated, please also update this one.
//
#define r0 sh4_r0
#define r1 sh4_r1
#define r2 sh4_r2
#define r3 sh4_r10
#define r4 sh4_r4
#define r5 sh4_r5
#define r6 sh4_r6
#define r7 sh4_r7
#define r8 "should be cp"
#define r9 sh4_r9
#define ip sh4_r11
#define lr pr
#define pc "to be checked"
#define r10 "should be roots"
#define r11 "Unexpected"
#define r12 "Unexpected"
#define r13 "Unexpected"
#define r14 "Unexpected"
#define r15 "Unexpected"


MemOperand FullCodeGenerator::EmitSlotSearch(Slot* slot, Register scratch) {
  switch (slot->type()) {
    case Slot::PARAMETER:
    case Slot::LOCAL:
      return MemOperand(fp, SlotOffset(slot));
    case Slot::CONTEXT: {
      int context_chain_length =
          scope()->ContextChainLength(slot->var()->scope());
      __ LoadContext(scratch, context_chain_length);
      return ContextOperand(scratch, slot->index());
    }
    case Slot::LOOKUP:
      UNREACHABLE();
  }
  UNREACHABLE();
  return MemOperand(r0, 0);
}


void FullCodeGenerator::Move(Register destination, Slot* source) {
  // Use destination as scratch.
  MemOperand slot_operand = EmitSlotSearch(source, destination);
  __ mov(destination, slot_operand);
}


void FullCodeGenerator::Move(Slot* dst,
                             Register src,
                             Register scratch1,
                             Register scratch2) {
  ASSERT(dst->type() != Slot::LOOKUP);  // Not yet implemented.
  ASSERT(!scratch1.is(src) && !scratch2.is(src));
  MemOperand location = EmitSlotSearch(dst, scratch1);
  __ mov(location, src);
  // Emit the write barrier code if the location is in the heap.
  if (dst->type() == Slot::CONTEXT) {
    __ RecordWrite(scratch1,
                   Context::SlotOffset(dst->index()),
                   scratch2,
                   src);
  }
}


void FullCodeGenerator::ClearAccumulator() {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::DeclareGlobals(Handle<FixedArray> pairs) {
  // Call the runtime to declare the globals.
  // The context is the first argument.
  __ mov(r2, Operand(pairs));
  __ mov(r1, Immediate(Smi::FromInt(is_eval() ? 1 : 0)));
  __ mov(r0, Immediate(Smi::FromInt(strict_mode_flag())));
  __ Push(cp, r2, r1, r0);
  __ CallRuntime(Runtime::kDeclareGlobals, 4);
  // Return value is ignored.
}


void FullCodeGenerator::DoTest(Label* if_true,
                               Label* if_false,
                               Label* fall_through) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitArguments(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitArgumentsLength(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitBinaryOp(Token::Value op,
                                     OverwriteMode mode) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitAssignment(Expression* expr, int bailout_ast_id) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitVariableAssignment(Variable* var,
                                               Token::Value op) {
  // Left-hand sides that rewrite to explicit property accesses do not reach
  // here.
  ASSERT(var != NULL);
  ASSERT(var->is_global() || var->AsSlot() != NULL);

  if (var->is_global()) {
    ASSERT(!var->is_this());
    // Assignment to a global variable.  Use inline caching for the
    // assignment.  Right-hand-side value is passed in r0, variable name in
    // r2, and the global object in r1.
    __ mov(r2, Operand(var->name()));
    __ ldr(r1, GlobalObjectOperand());
    Handle<Code> ic = is_strict_mode()
        ? isolate()->builtins()->StoreIC_Initialize_Strict()
        : isolate()->builtins()->StoreIC_Initialize();
    EmitCallIC(ic, RelocInfo::CODE_TARGET_CONTEXT);

  } else if (op == Token::INIT_CONST) {
    // Like var declarations, const declarations are hoisted to function
    // scope.  However, unlike var initializers, const initializers are able
    // to drill a hole to that function context, even from inside a 'with'
    // context.  We thus bypass the normal static scope lookup.
    Slot* slot = var->AsSlot();
    Label skip;
    switch (slot->type()) {
      case Slot::PARAMETER:
        // No const parameters.
        UNREACHABLE();
        break;
      case Slot::LOCAL:
        // Detect const reinitialization by checking for the hole value.
        __ ldr(r1, MemOperand(fp, SlotOffset(slot)));
        __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
        __ cmpeq(r1, ip);
        __ bf(&skip);
        __ str(result_register(), MemOperand(fp, SlotOffset(slot)));
        break;
      case Slot::CONTEXT: {
        __ ldr(r1, ContextOperand(cp, Context::FCONTEXT_INDEX));
        __ ldr(r2, ContextOperand(r1, slot->index()));
        __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
        __ cmpeq(r2, ip);
        __ bf(&skip);
        __ str(r0, ContextOperand(r1, slot->index()));
        int offset = Context::SlotOffset(slot->index());
        __ mov(r3, r0);  // Preserve the stored value in r0.
	__ RecordWrite(r1, offset, r3, r2);
        break;
      }
      case Slot::LOOKUP:
        __ push(r0);
        __ mov(r0, Operand(slot->var()->name()));
        __ Push(cp, r0);  // Context and name.
        __ CallRuntime(Runtime::kInitializeConstContextSlot, 3);
        break;
    }
    __ bind(&skip);

  } else if (var->mode() != Variable::CONST) {
    // Perform the assignment for non-const variables.  Const assignments
    // are simply skipped.
    Slot* slot = var->AsSlot();
    switch (slot->type()) {
      case Slot::PARAMETER:
      case Slot::LOCAL:
        // Perform the assignment.
        __ str(result_register(), MemOperand(fp, SlotOffset(slot)));
        break;

      case Slot::CONTEXT: {
        MemOperand target = EmitSlotSearch(slot, r1);
        // Perform the assignment and issue the write barrier.
        __ str(result_register(), target);
        // RecordWrite may destroy all its register arguments.
        __ mov(r3, result_register());
        int offset = FixedArray::kHeaderSize + slot->index() * kPointerSize;
        __ RecordWrite(r1, offset, r2, r3);
        break;
      }

      case Slot::LOOKUP:
        // Call the runtime for the assignment.
        __ push(r0);  // Value.
        __ mov(r1, Operand(slot->var()->name()));
        __ mov(r0, Immediate/*TODO:Operand*/(Smi::FromInt(strict_mode_flag())));
        __ Push(cp, r1, r0);  // Context, name, strict mode.
        __ CallRuntime(Runtime::kStoreContextSlot, 4);
        break;
    }
  }
}


void FullCodeGenerator::EmitCallFunction(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitClassOf(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitFastAsciiArrayJoin(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitGetCachedArrayIndex(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitGetFromCache(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitHasCachedArrayIndex(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitInlineSmiBinaryOp(Expression* expr,
                                              Token::Value op,
                                              OverwriteMode mode,
                                              Expression* left,
                                              Expression* right) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsArray(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsConstructCall(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsFunction(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsNonNegativeSmi(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsObject(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsRegExp(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsRegExpEquivalent(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsSmi(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsSpecObject(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsStringWrapperSafeForDefaultValueOf(
    ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitIsUndetectableObject(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitLog(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitMathCos(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitMathLog(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitMathPow(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitMathSin(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitMathSqrt(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitNewClosure(Handle<SharedFunctionInfo> info,
                                       bool pretenure) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitNumberToString(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitObjectEquals(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitRandomHeapNumber(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitRegExpConstructResult(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitRegExpExec(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitReturnSequence() {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitSetValueOf(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitStackCheck(IterationStatement* stmt) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitStringAdd(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitStringCharAt(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitStringCharCodeAt(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitStringCharFromCode(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitStringCompare(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitSubString(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitSwapElements(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitValueOf(ZoneList<Expression*>* args) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EnterFinallyBlock() {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::ExitFinallyBlock() {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::LoadContextField(Register dst, int context_index) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::PrepareForBailoutBeforeSplit(State state,
                                                     bool should_normalize,
                                                     Label* if_true,
                                                     Label* if_false) {
  __ UNIMPLEMENTED_BREAK();
}


// Clobbers: r0, r1, r2, r3
// Live-in: fp, cp
// Live-out: fp, cp
void FullCodeGenerator::EmitDeclaration(Variable* variable,
                                        Variable::Mode mode,
                                        FunctionLiteral* function) {
  Comment cmnt(masm_, "[ Declaration");
  ASSERT(variable != NULL);  // Must have been resolved.
  Slot* slot = variable->AsSlot();
  Property* prop = variable->AsProperty();

  if (slot != NULL) {
    switch (slot->type()) {
      case Slot::PARAMETER:
      case Slot::LOCAL:
        if (mode == Variable::CONST) {
          __ LoadRoot(r1, Heap::kTheHoleValueRootIndex);
          __ mov(MemOperand(fp, SlotOffset(slot)), r1);
        } else if (function != NULL) {
          VisitForAccumulatorValue(function);
          __ mov(MemOperand(fp, SlotOffset(slot)), result_register());
        }
        break;

      case Slot::CONTEXT:
        // We bypass the general EmitSlotSearch because we know more about
        // this specific context.

        // The variable in the decl always resides in the current function
        // context.
        ASSERT_EQ(0, scope()->ContextChainLength(variable->scope()));
        if (FLAG_debug_code) {
          // Check that we're not inside a 'with'.
          __ mov(r1, ContextOperand(cp, Context::FCONTEXT_INDEX));
          __ cmpeq(r1, cp);
          __ Check("Unexpected declaration in current context.");
        }
        if (mode == Variable::CONST) {
          __ LoadRoot(r1, Heap::kTheHoleValueRootIndex);
          __ mov(ContextOperand(cp, slot->index()), r1);
          // No write barrier since the_hole_value is in old space.
        } else if (function != NULL) {
          VisitForAccumulatorValue(function);
          __ mov(ContextOperand(cp, slot->index()), result_register());
          int offset = Context::SlotOffset(slot->index());
          // We know that we have written a function, which is not a smi.
          __ mov(r1, cp);
          __ RecordWrite(r1, offset, r2, result_register());
        }
        break;

      case Slot::LOOKUP: {
        __ mov(r2, Operand(variable->name()));
        // Declaration nodes are always introduced in one of two modes.
        ASSERT(mode == Variable::VAR ||
               mode == Variable::CONST);
        PropertyAttributes attr =
            (mode == Variable::VAR) ? NONE : READ_ONLY;
        __ mov(r1, Immediate(Smi::FromInt(attr)));
        // Push initial value, if any.
        // Note: For variables we must not push an initial value (such as
        // 'undefined') because we may have a (legal) redeclaration and we
        // must not destroy the current value.
        if (mode == Variable::CONST) {
          __ LoadRoot(r0, Heap::kTheHoleValueRootIndex);
          __ Push(cp, r2, r1, r0);
        } else if (function != NULL) {
          __ Push(cp, r2, r1);
          // Push initial value for function declaration.
          VisitForStackValue(function);
        } else {
          __ mov(r0, Immediate(Smi::FromInt(0)));  // No initial value!
          __ Push(cp, r2, r1, r0);
        }
        __ CallRuntime(Runtime::kDeclareContextSlot, 4);
        break;
      }
    }

  } else if (prop != NULL) {
    if (function != NULL || mode == Variable::CONST) {
      // We are declaring a function or constant that rewrites to a
      // property.  Use (keyed) IC to set the initial value.  We
      // cannot visit the rewrite because it's shared and we risk
      // recording duplicate AST IDs for bailouts from optimized code.
      ASSERT(prop->obj()->AsVariableProxy() != NULL);
      { AccumulatorValueContext for_object(this);
        EmitVariableLoad(prop->obj()->AsVariableProxy()->var());
      }
      if (function != NULL) {
        __ push(r0);
        VisitForAccumulatorValue(function);
        __ pop(r2);
      } else {
        __ mov(r2, r0);
        __ LoadRoot(r0, Heap::kTheHoleValueRootIndex);
      }
      ASSERT(prop->key()->AsLiteral() != NULL &&
             prop->key()->AsLiteral()->handle()->IsSmi());
      __ mov(r1, Operand(prop->key()->AsLiteral()->handle()));

      Handle<Code> ic = is_strict_mode()
          ? isolate()->builtins()->KeyedStoreIC_Initialize_Strict()
          : isolate()->builtins()->KeyedStoreIC_Initialize();
      EmitCallIC(ic, RelocInfo::CODE_TARGET);
      // Value in r0 is ignored (declarations are statements).
    }
  }
}


void FullCodeGenerator::EmitCallIC(Handle<Code> ic, JumpPatchSite* patch_site) {
  Counters* counters = isolate()->counters();
  switch (ic->kind()) {
    case Code::LOAD_IC:
      __ IncrementCounter(counters->named_load_full(), 1, r1, r2);
      break;
    case Code::KEYED_LOAD_IC:
      __ IncrementCounter(counters->keyed_load_full(), 1, r1, r2);
      break;
    case Code::STORE_IC:
      __ IncrementCounter(counters->named_store_full(), 1, r1, r2);
      break;
    case Code::KEYED_STORE_IC:
      __ IncrementCounter(counters->keyed_store_full(), 1, r1, r2);
    default:
      break;
  }
  __ Call(ic, RelocInfo::CODE_TARGET);
  if (patch_site != NULL && patch_site->is_bound()) {
    patch_site->EmitPatchInfo();
  } else {
    __ nop();  // Signals no inlined code.
  }
}


void FullCodeGenerator::StoreToFrameField(int frame_offset, Register value) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitArrayLiteral(ArrayLiteral* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitAssignment(Assignment* expr) {
  Comment cmnt(masm_, "[ Assignment");
  // Invalid left-hand sides are rewritten to have a 'throw ReferenceError'
  // on the left-hand side.
  if (!expr->target()->IsValidLeftHandSide()) {
    VisitForEffect(expr->target());
    return;
  }

  // Left-hand side can only be a property, a global or a (parameter or local)
  // slot. Variables with rewrite to .arguments are treated as KEYED_PROPERTY.
  enum LhsKind { VARIABLE, NAMED_PROPERTY, KEYED_PROPERTY };
  LhsKind assign_type = VARIABLE;
  Property* property = expr->target()->AsProperty();
  if (property != NULL) {
    assign_type = (property->key()->IsPropertyName())
        ? NAMED_PROPERTY
        : KEYED_PROPERTY;
  }

  // Evaluate LHS expression.
  switch (assign_type) {
    case VARIABLE:
      // Nothing to do here.
      break;
    case NAMED_PROPERTY:
      if (expr->is_compound()) {
        // We need the receiver both on the stack and in the accumulator.
        VisitForAccumulatorValue(property->obj());
        __ push(result_register());
      } else {
        VisitForStackValue(property->obj());
      }
      break;
    case KEYED_PROPERTY:
      if (expr->is_compound()) {
        if (property->is_arguments_access()) {
          VariableProxy* obj_proxy = property->obj()->AsVariableProxy();
          __ ldr(r0, EmitSlotSearch(obj_proxy->var()->AsSlot(), r0));
          __ push(r0);
          __ mov(r0, Operand(property->key()->AsLiteral()->handle()));
        } else {
          VisitForStackValue(property->obj());
          VisitForAccumulatorValue(property->key());
        }
        __ ldr(r1, MemOperand(sp, 0));
        __ push(r0);
      } else {
        if (property->is_arguments_access()) {
          VariableProxy* obj_proxy = property->obj()->AsVariableProxy();
          __ ldr(r1, EmitSlotSearch(obj_proxy->var()->AsSlot(), r0));
          __ mov(r0, Operand(property->key()->AsLiteral()->handle()));
          __ Push(r1, r0);
        } else {
          VisitForStackValue(property->obj());
          VisitForStackValue(property->key());
        }
      }
      break;
  }

  // For compound assignments we need another deoptimization point after the
  // variable/property load.
  if (expr->is_compound()) {
    { AccumulatorValueContext context(this);
      switch (assign_type) {
        case VARIABLE:
          EmitVariableLoad(expr->target()->AsVariableProxy()->var());
          PrepareForBailout(expr->target(), TOS_REG);
          break;
        case NAMED_PROPERTY:
          EmitNamedPropertyLoad(property);
          PrepareForBailoutForId(expr->CompoundLoadId(), TOS_REG);
          break;
        case KEYED_PROPERTY:
          EmitKeyedPropertyLoad(property);
          PrepareForBailoutForId(expr->CompoundLoadId(), TOS_REG);
          break;
      }
    }

    Token::Value op = expr->binary_op();
    __ push(r0);  // Left operand goes on the stack.
    VisitForAccumulatorValue(expr->value());

    OverwriteMode mode = expr->value()->ResultOverwriteAllowed()
        ? OVERWRITE_RIGHT
        : NO_OVERWRITE;
    SetSourcePosition(expr->position() + 1);
    AccumulatorValueContext context(this);
    if (ShouldInlineSmiCase(op)) {
      EmitInlineSmiBinaryOp(expr,
                            op,
                            mode,
                            expr->target(),
                            expr->value());
    } else {
      EmitBinaryOp(op, mode);
    }

    // Deoptimization point in case the binary operation may have side effects.
    PrepareForBailout(expr->binary_operation(), TOS_REG);
  } else {
    VisitForAccumulatorValue(expr->value());
  }

  // Record source position before possible IC call.
  SetSourcePosition(expr->position());

  // Store the value.
  switch (assign_type) {
    case VARIABLE:
      EmitVariableAssignment(expr->target()->AsVariableProxy()->var(),
                             expr->op());
      PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
      context()->Plug(r0);
      break;
    case NAMED_PROPERTY:
      EmitNamedPropertyAssignment(expr);
      break;
    case KEYED_PROPERTY:
      EmitKeyedPropertyAssignment(expr);
      break;
  }
}


void FullCodeGenerator::EmitNamedPropertyLoad(Property* prop) {
  SetSourcePosition(prop->position());
  Literal* key = prop->key()->AsLiteral();
  __ mov(r2, Operand(key->handle()));
  // Call load IC. It has arguments receiver and property name r0 and r2.
  Handle<Code> ic = isolate()->builtins()->LoadIC_Initialize();
  EmitCallIC(ic, RelocInfo::CODE_TARGET);
}


void FullCodeGenerator::EmitKeyedPropertyLoad(Property* prop) {
  SetSourcePosition(prop->position());
  // Call keyed load IC. It has arguments key and receiver in r0 and r1.
  Handle<Code> ic = isolate()->builtins()->KeyedLoadIC_Initialize();
  EmitCallIC(ic, RelocInfo::CODE_TARGET);
}


void FullCodeGenerator::EmitKeyedCallWithIC(Call* expr,
                                            Expression* key,
                                            RelocInfo::Mode mode) {
  // Load the key.
  VisitForAccumulatorValue(key);

  // Swap the name of the function and the receiver on the stack to follow
  // the calling convention for call ICs.
  __ pop(r1);
  __ push(r0);
  __ push(r1);

  // Code common for calls using the IC.
  ZoneList<Expression*>* args = expr->arguments();
  int arg_count = args->length();
  { PreservePositionScope scope(masm()->positions_recorder());
    for (int i = 0; i < arg_count; i++) {
      VisitForStackValue(args->at(i));
    }
  }
  // Record source position for debugger.
  SetSourcePosition(expr->position());
  // Call the IC initialization code.
  InLoopFlag in_loop = (loop_depth() > 0) ? IN_LOOP : NOT_IN_LOOP;
  Handle<Code> ic =
      isolate()->stub_cache()->ComputeKeyedCallInitialize(arg_count, in_loop);
  __ ldr(r2, MemOperand(sp, (arg_count + 1) * kPointerSize));  // Key.
  EmitCallIC(ic, mode);
  RecordJSReturnSite(expr);
  // Restore context register.
  __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  context()->DropAndPlug(1, r0);  // Drop the key still on the stack.
}


void FullCodeGenerator::EmitCallWithStub(Call* expr) {
  // Code common for calls using the call stub.
  ZoneList<Expression*>* args = expr->arguments();
  int arg_count = args->length();
  { PreservePositionScope scope(masm()->positions_recorder());
    for (int i = 0; i < arg_count; i++) {
      VisitForStackValue(args->at(i));
    }
  }
  // Record source position for debugger.
  SetSourcePosition(expr->position());
  InLoopFlag in_loop = (loop_depth() > 0) ? IN_LOOP : NOT_IN_LOOP;
  CallFunctionStub stub(arg_count, in_loop, RECEIVER_MIGHT_BE_VALUE);
  __ CallStub(&stub);
  RecordJSReturnSite(expr);
  // Restore context register.
  __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  context()->DropAndPlug(1, r0);
}


void FullCodeGenerator::EmitResolvePossiblyDirectEval(ResolveEvalFlag flag,
                                                      int arg_count) {
  // Push copy of the first argument or undefined if it doesn't exist.
  if (arg_count > 0) {
    __ ldr(r1, MemOperand(sp, arg_count * kPointerSize));
  } else {
    __ LoadRoot(r1, Heap::kUndefinedValueRootIndex);
  }
  __ push(r1);

  // Push the receiver of the enclosing function and do runtime call.
  __ ldr(r1, MemOperand(fp, (2 + scope()->num_parameters()) * kPointerSize));
  __ push(r1);
  // Push the strict mode flag.
  __ mov(r1, Immediate/*TODO:Operand*/(Smi::FromInt(strict_mode_flag())));
  __ push(r1);

  __ CallRuntime(flag == SKIP_CONTEXT_LOOKUP
                 ? Runtime::kResolvePossiblyDirectEvalNoLookup
                 : Runtime::kResolvePossiblyDirectEval, 4);
}


void FullCodeGenerator::VisitCall(Call* expr) {
#ifdef DEBUG
  // We want to verify that RecordJSReturnSite gets called on all paths
  // through this function.  Avoid early returns.
  expr->return_is_recorded_ = false;
#endif

  Comment cmnt(masm_, "[ Call");
  Expression* fun = expr->expression();
  Variable* var = fun->AsVariableProxy()->AsVariable();

  if (var != NULL && var->is_possibly_eval()) {
    // In a call to eval, we first call %ResolvePossiblyDirectEval to
    // resolve the function we need to call and the receiver of the
    // call.  Then we call the resolved function using the given
    // arguments.
    ZoneList<Expression*>* args = expr->arguments();
    int arg_count = args->length();

    { PreservePositionScope pos_scope(masm()->positions_recorder());
      VisitForStackValue(fun);
      __ LoadRoot(r2, Heap::kUndefinedValueRootIndex);
      __ push(r2);  // Reserved receiver slot.

      // Push the arguments.
      for (int i = 0; i < arg_count; i++) {
        VisitForStackValue(args->at(i));
      }

      // If we know that eval can only be shadowed by eval-introduced
      // variables we attempt to load the global eval function directly
      // in generated code. If we succeed, there is no need to perform a
      // context lookup in the runtime system.
      Label done;
      if (var->AsSlot() != NULL && var->mode() == Variable::DYNAMIC_GLOBAL) {
        Label slow;
        EmitLoadGlobalSlotCheckExtensions(var->AsSlot(),
                                          NOT_INSIDE_TYPEOF,
                                          &slow);
        // Push the function and resolve eval.
        __ push(r0);
        EmitResolvePossiblyDirectEval(SKIP_CONTEXT_LOOKUP, arg_count);
        __ jmp(&done);
        __ bind(&slow);
      }

      // Push copy of the function (found below the arguments) and
      // resolve eval.
      __ ldr(r1, MemOperand(sp, (arg_count + 1) * kPointerSize));
      __ push(r1);
      EmitResolvePossiblyDirectEval(PERFORM_CONTEXT_LOOKUP, arg_count);
      if (done.is_linked()) {
        __ bind(&done);
      }

      // The runtime call returns a pair of values in r0 (function) and
      // r1 (receiver). Touch up the stack with the right values.
      __ str(r0, MemOperand(sp, (arg_count + 1) * kPointerSize));
      __ str(r1, MemOperand(sp, arg_count * kPointerSize));
    }

    // Record source position for debugger.
    SetSourcePosition(expr->position());
    InLoopFlag in_loop = (loop_depth() > 0) ? IN_LOOP : NOT_IN_LOOP;
    CallFunctionStub stub(arg_count, in_loop, RECEIVER_MIGHT_BE_VALUE);
    __ CallStub(&stub);
    RecordJSReturnSite(expr);
    // Restore context register.
    __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
    context()->DropAndPlug(1, r0);
  } else if (var != NULL && !var->is_this() && var->is_global()) {
    // Push global object as receiver for the call IC.
    __ ldr(r0, GlobalObjectOperand());
    __ push(r0);
    EmitCallWithIC(expr, var->name(), RelocInfo::CODE_TARGET_CONTEXT);
  } else if (var != NULL && var->AsSlot() != NULL &&
             var->AsSlot()->type() == Slot::LOOKUP) {
    // Call to a lookup slot (dynamically introduced variable).
    Label slow, done;

    { PreservePositionScope scope(masm()->positions_recorder());
      // Generate code for loading from variables potentially shadowed
      // by eval-introduced variables.
      EmitDynamicLoadFromSlotFastCase(var->AsSlot(),
                                      NOT_INSIDE_TYPEOF,
                                      &slow,
                                      &done);
    }

    __ bind(&slow);
    // Call the runtime to find the function to call (returned in r0)
    // and the object holding it (returned in edx).
    __ push(context_register());
    __ mov(r2, Operand(var->name()));
    __ push(r2);
    __ CallRuntime(Runtime::kLoadContextSlot, 2);
    __ Push(r0, r1);  // Function, receiver.

    // If fast case code has been generated, emit code to push the
    // function and receiver and have the slow path jump around this
    // code.
    if (done.is_linked()) {
      Label call;
      __ jmp(&call);
      __ bind(&done);
      // Push function.
      __ push(r0);
      // Push global receiver.
      __ ldr(r1, GlobalObjectOperand());
      __ ldr(r1, FieldMemOperand(r1, GlobalObject::kGlobalReceiverOffset));
      __ push(r1);
      __ bind(&call);
    }

    EmitCallWithStub(expr);
  } else if (fun->AsProperty() != NULL) {
    // Call to an object property.
    Property* prop = fun->AsProperty();
    Literal* key = prop->key()->AsLiteral();
    if (key != NULL && key->handle()->IsSymbol()) {
      // Call to a named property, use call IC.
      { PreservePositionScope scope(masm()->positions_recorder());
        VisitForStackValue(prop->obj());
      }
      EmitCallWithIC(expr, key->handle(), RelocInfo::CODE_TARGET);
    } else {
      // Call to a keyed property.
      // For a synthetic property use keyed load IC followed by function call,
      // for a regular property use keyed CallIC.
      if (prop->is_synthetic()) {
        // Do not visit the object and key subexpressions (they are shared
        // by all occurrences of the same rewritten parameter).
        ASSERT(prop->obj()->AsVariableProxy() != NULL);
        ASSERT(prop->obj()->AsVariableProxy()->var()->AsSlot() != NULL);
        Slot* slot = prop->obj()->AsVariableProxy()->var()->AsSlot();
        MemOperand operand = EmitSlotSearch(slot, r1);
        __ ldr(r1, operand);

        ASSERT(prop->key()->AsLiteral() != NULL);
        ASSERT(prop->key()->AsLiteral()->handle()->IsSmi());
        __ mov(r0, Operand(prop->key()->AsLiteral()->handle()));

        // Record source code position for IC call.
        SetSourcePosition(prop->position());

        Handle<Code> ic = isolate()->builtins()->KeyedLoadIC_Initialize();
        EmitCallIC(ic, RelocInfo::CODE_TARGET);
        __ ldr(r1, GlobalObjectOperand());
        __ ldr(r1, FieldMemOperand(r1, GlobalObject::kGlobalReceiverOffset));
        __ Push(r0, r1);  // Function, receiver.
        EmitCallWithStub(expr);
      } else {
        { PreservePositionScope scope(masm()->positions_recorder());
          VisitForStackValue(prop->obj());
        }
        EmitKeyedCallWithIC(expr, prop->key(), RelocInfo::CODE_TARGET);
      }
    }
  } else {
    { PreservePositionScope scope(masm()->positions_recorder());
      VisitForStackValue(fun);
    }
    // Load global receiver object.
    __ ldr(r1, GlobalObjectOperand());
    __ ldr(r1, FieldMemOperand(r1, GlobalObject::kGlobalReceiverOffset));
    __ push(r1);
    // Emit function call.
    EmitCallWithStub(expr);
  }

#ifdef DEBUG
  // RecordJSReturnSite should have been called.
  ASSERT(expr->return_is_recorded_);
#endif
}


void FullCodeGenerator::VisitCallNew(CallNew* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitCallRuntime(CallRuntime* expr) {
  Handle<String> name = expr->name();
  if (name->length() > 0 && name->Get(0) == '_') {
    Comment cmnt(masm_, "[ InlineRuntimeCall");
    EmitInlineRuntimeCall(expr);
    return;
  }

  Comment cmnt(masm_, "[ CallRuntime");
  ZoneList<Expression*>* args = expr->arguments();

  if (expr->is_jsruntime()) {
    // Prepare for calling JS runtime function.
    __ ldr(r0, GlobalObjectOperand());
    __ ldr(r0, FieldMemOperand(r0, GlobalObject::kBuiltinsOffset));
    __ push(r0);
  }

  // Push the arguments ("left-to-right").
  int arg_count = args->length();
  for (int i = 0; i < arg_count; i++) {
    VisitForStackValue(args->at(i));
  }

  if (expr->is_jsruntime()) {
    // Call the JS runtime function.
    __ mov(r2, Operand(expr->name()));
    Handle<Code> ic =
        isolate()->stub_cache()->ComputeCallInitialize(arg_count, NOT_IN_LOOP);
    EmitCallIC(ic, RelocInfo::CODE_TARGET);
    // Restore context register.
    __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  } else {
    // Call the C runtime function.
    __ CallRuntime(expr->function(), arg_count);
  }
  context()->Plug(r0);
}


void FullCodeGenerator::VisitCompareOperation(CompareOperation* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitCompareToNull(CompareToNull* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitCountOperation(CountOperation* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitDeclaration(Declaration* decl) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitForInStatement(ForInStatement* stmt) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitObjectLiteral(ObjectLiteral* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EmitNamedPropertyAssignment(Assignment* expr) {
  // Assignment to a property, using a named store IC.
  Property* prop = expr->target()->AsProperty();
  ASSERT(prop != NULL);
  ASSERT(prop->key()->AsLiteral() != NULL);

  // If the assignment starts a block of assignments to the same object,
  // change to slow case to avoid the quadratic behavior of repeatedly
  // adding fast properties.
  if (expr->starts_initialization_block()) {
    __ push(result_register());
    __ ldr(ip, MemOperand(sp, kPointerSize));  // Receiver is now under value.
    __ push(ip);
    __ CallRuntime(Runtime::kToSlowProperties, 1);
    __ pop(result_register());
  }

  // Record source code position before IC call.
  SetSourcePosition(expr->position());
  __ mov(r2, Operand(prop->key()->AsLiteral()->handle()));
  // Load receiver to r1. Leave a copy in the stack if needed for turning the
  // receiver into fast case.
  if (expr->ends_initialization_block()) {
    __ ldr(r1, MemOperand(sp));
  } else {
    __ pop(r1);
  }

  Handle<Code> ic = is_strict_mode()
      ? isolate()->builtins()->StoreIC_Initialize_Strict()
      : isolate()->builtins()->StoreIC_Initialize();
  EmitCallIC(ic, RelocInfo::CODE_TARGET);

  // If the assignment ends an initialization block, revert to fast case.
  if (expr->ends_initialization_block()) {
    __ push(r0);  // Result of assignment, saved even if not needed.
    // Receiver is under the result value.
    __ ldr(ip, MemOperand(sp, kPointerSize));
    __ push(ip);
    __ CallRuntime(Runtime::kToFastProperties, 1);
    __ pop(r0);
    __ Drop(1);
  }
  PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
  context()->Plug(r0);
}


void FullCodeGenerator::EmitKeyedPropertyAssignment(Assignment* expr) {
  // Assignment to a property, using a keyed store IC.

  // If the assignment starts a block of assignments to the same object,
  // change to slow case to avoid the quadratic behavior of repeatedly
  // adding fast properties.
  if (expr->starts_initialization_block()) {
    __ push(result_register());
    // Receiver is now under the key and value.
    __ ldr(ip, MemOperand(sp, 2 * kPointerSize));
    __ push(ip);
    __ CallRuntime(Runtime::kToSlowProperties, 1);
    __ pop(result_register());
  }

  // Record source code position before IC call.
  SetSourcePosition(expr->position());
  __ pop(r1);  // Key.
  // Load receiver to r2. Leave a copy in the stack if needed for turning the
  // receiver into fast case.
  if (expr->ends_initialization_block()) {
    __ ldr(r2, MemOperand(sp));
  } else {
    __ pop(r2);
  }

  Handle<Code> ic = is_strict_mode()
      ? isolate()->builtins()->KeyedStoreIC_Initialize_Strict()
      : isolate()->builtins()->KeyedStoreIC_Initialize();
  EmitCallIC(ic, RelocInfo::CODE_TARGET);

  // If the assignment ends an initialization block, revert to fast case.
  if (expr->ends_initialization_block()) {
    __ push(r0);  // Result of assignment, saved even if not needed.
    // Receiver is under the result value.
    __ ldr(ip, MemOperand(sp, kPointerSize));
    __ push(ip);
    __ CallRuntime(Runtime::kToFastProperties, 1);
    __ pop(r0);
    __ Drop(1);
  }
  PrepareForBailoutForId(expr->AssignmentId(), TOS_REG);
  context()->Plug(r0);
}


void FullCodeGenerator::VisitProperty(Property* expr) {
  Comment cmnt(masm_, "[ Property");
  Expression* key = expr->key();

  if (key->IsPropertyName()) {
    VisitForAccumulatorValue(expr->obj());
    EmitNamedPropertyLoad(expr);
    context()->Plug(r0);
  } else {
    VisitForStackValue(expr->obj());
    VisitForAccumulatorValue(expr->key());
    __ pop(r1);
    EmitKeyedPropertyLoad(expr);
    context()->Plug(r0);
  }
}


void FullCodeGenerator::EmitCallWithIC(Call* expr,
                                       Handle<Object> name,
                                       RelocInfo::Mode mode) {
  // Code common for calls using the IC.
  ZoneList<Expression*>* args = expr->arguments();
  int arg_count = args->length();
  { PreservePositionScope scope(masm()->positions_recorder());
    for (int i = 0; i < arg_count; i++) {
      VisitForStackValue(args->at(i));
    }
    __ mov(r2, Operand(name));
  }
  // Record source position for debugger.
  SetSourcePosition(expr->position());
  // Call the IC initialization code.
  InLoopFlag in_loop = (loop_depth() > 0) ? IN_LOOP : NOT_IN_LOOP;
  Handle<Code> ic =
      isolate()->stub_cache()->ComputeCallInitialize(arg_count, in_loop);
  EmitCallIC(ic, mode);
  RecordJSReturnSite(expr);
  // Restore context register.
  __ ldr(cp, MemOperand(fp, StandardFrameConstants::kContextOffset));
  context()->Plug(r0);
}


void FullCodeGenerator::EmitLoadGlobalSlotCheckExtensions(
    Slot* slot,
    TypeofState typeof_state,
    Label* slow) {
  Register current = cp;
  Register next = r1;
  Register temp = r2;

  Scope* s = scope();
  while (s != NULL) {
    if (s->num_heap_slots() > 0) {
      if (s->calls_eval()) {
        // Check that extension is NULL.
        __ ldr(temp, ContextOperand(current, Context::EXTENSION_INDEX));
        __ tst(temp, temp);
        __ bf(slow);
      }
      // Load next context in chain.
      __ ldr(next, ContextOperand(current, Context::CLOSURE_INDEX));
      __ ldr(next, FieldMemOperand(next, JSFunction::kContextOffset));
      // Walk the rest of the chain without clobbering cp.
      current = next;
    }
    // If no outer scope calls eval, we do not need to check more
    // context extensions.
    if (!s->outer_scope_calls_eval() || s->is_eval_scope()) break;
    s = s->outer_scope();
  }

  if (s->is_eval_scope()) {
    Label loop, fast;
    if (!current.is(next)) {
      __ Move(next, current);
    }
    __ bind(&loop);
    // Terminate at global context.
    __ ldr(temp, FieldMemOperand(next, HeapObject::kMapOffset));
    __ LoadRoot(ip, Heap::kGlobalContextMapRootIndex);
    __ cmpeq(temp, ip);
    __ bt(&fast);
    // Check that extension is NULL.
    __ ldr(temp, ContextOperand(next, Context::EXTENSION_INDEX));
    __ tst(temp, temp);
    __ bf(slow);
    // Load next context in chain.
    __ ldr(next, ContextOperand(next, Context::CLOSURE_INDEX));
    __ ldr(next, FieldMemOperand(next, JSFunction::kContextOffset));
    __ b(&loop);
    __ bind(&fast);
  }

  __ ldr(r0, GlobalObjectOperand());
  __ mov(r2, Operand(slot->var()->name()));
  RelocInfo::Mode mode = (typeof_state == INSIDE_TYPEOF)
      ? RelocInfo::CODE_TARGET
      : RelocInfo::CODE_TARGET_CONTEXT;
  Handle<Code> ic = isolate()->builtins()->LoadIC_Initialize();
  EmitCallIC(ic, mode);
}


// clobbers: r0, r1, r3
// live-in: fp, sp, cp
// live-out: fp, sp, cp
void FullCodeGenerator::EmitVariableLoad(Variable* var) {
  // Four cases: non-this global variables, lookup slots, all other
  // types of slots, and parameters that rewrite to explicit property
  // accesses on the arguments object.
  Slot* slot = var->AsSlot();
  Property* property = var->AsProperty();

  if (var->is_global() && !var->is_this()) {
    Comment cmnt(masm_, "Global variable");
    // Use inline caching. Variable name is passed in r2 and the global
    // object (receiver) in r0.
    __ mov(r0, GlobalObjectOperand());
    __ mov(r2, Operand(var->name()));
    Handle<Code> ic = isolate()->builtins()->LoadIC_Initialize();
    EmitCallIC(ic, RelocInfo::CODE_TARGET_CONTEXT);
    context()->Plug(r0);

  } else if (slot != NULL && slot->type() == Slot::LOOKUP) {
    Label done, slow;

    // Generate code for loading from variables potentially shadowed
    // by eval-introduced variables.
    EmitDynamicLoadFromSlotFastCase(slot, NOT_INSIDE_TYPEOF, &slow, &done);

    __ bind(&slow);
    Comment cmnt(masm_, "Lookup slot");
    __ mov(r1, Operand(var->name()));
    __ Push(cp, r1);  // Context and name.
    __ CallRuntime(Runtime::kLoadContextSlot, 2);
    __ bind(&done);

    context()->Plug(r0);

  } else if (slot != NULL) {
    Comment cmnt(masm_, (slot->type() == Slot::CONTEXT)
                            ? "Context slot"
                            : "Stack slot");
    if (var->mode() == Variable::CONST) {
      // Constants may be the hole value if they have not been initialized.
      // Unhole them.
      Label no_hole;

      MemOperand slot_operand = EmitSlotSearch(slot, r0);
      __ mov(r0, slot_operand);
      __ LoadRoot(r3, Heap::kTheHoleValueRootIndex);
      __ cmpeq(r0, r3);
      __ bf(&no_hole);
      __ LoadRoot(r0, Heap::kUndefinedValueRootIndex);
      __ bind(&no_hole);
      context()->Plug(r0);
    } else {
      context()->Plug(slot);
    }
  } else {
    Comment cmnt(masm_, "Rewritten parameter");
    ASSERT_NOT_NULL(property);
    // Rewritten parameter accesses are of the form "slot[literal]".

    // Assert that the object is in a slot.
    Variable* object_var = property->obj()->AsVariableProxy()->AsVariable();
    ASSERT_NOT_NULL(object_var);
    Slot* object_slot = object_var->AsSlot();
    ASSERT_NOT_NULL(object_slot);

    // Load the object.
    Move(r1, object_slot);

    // Assert that the key is a smi.
    Literal* key_literal = property->key()->AsLiteral();
    ASSERT_NOT_NULL(key_literal);
    ASSERT(key_literal->handle()->IsSmi());

    // Load the key.
    __ mov(r0, Operand(key_literal->handle()));

    // Call keyed load IC. It has arguments key and receiver in r0 and r1.
    Handle<Code> ic = isolate()->builtins()->KeyedLoadIC_Initialize();
    EmitCallIC(ic, RelocInfo::CODE_TARGET);
    context()->Plug(r0);
  }
}


void FullCodeGenerator::EmitDynamicLoadFromSlotFastCase(
    Slot* slot,
    TypeofState typeof_state,
    Label* slow,
    Label* done) {
  __ UNIMPLEMENTED_BREAK();
  // // Generate fast-case code for variables that might be shadowed by
  // // eval-introduced variables.  Eval is used a lot without
  // // introducing variables.  In those cases, we do not want to
  // // perform a runtime call for all variables in the scope
  // // containing the eval.
  // if (slot->var()->mode() == Variable::DYNAMIC_GLOBAL) {
  //   EmitLoadGlobalSlotCheckExtensions(slot, typeof_state, slow);
  //   __ jmp(done);
  // } else if (slot->var()->mode() == Variable::DYNAMIC_LOCAL) {
  //   Slot* potential_slot = slot->var()->local_if_not_shadowed()->AsSlot();
  //   Expression* rewrite = slot->var()->local_if_not_shadowed()->rewrite();
  //   if (potential_slot != NULL) {
  //     // Generate fast case for locals that rewrite to slots.
  //     __ ldr(r0, ContextSlotOperandCheckExtensions(potential_slot, slow));
  //     if (potential_slot->var()->mode() == Variable::CONST) {
  //       __ LoadRoot(ip, Heap::kTheHoleValueRootIndex);
  //       __ cmp(r0, ip);
  //       __ LoadRoot(r0, Heap::kUndefinedValueRootIndex, eq);
  //     }
  //     __ jmp(done);
  //   } else if (rewrite != NULL) {
  //     // Generate fast case for calls of an argument function.
  //     Property* property = rewrite->AsProperty();
  //     if (property != NULL) {
  //       VariableProxy* obj_proxy = property->obj()->AsVariableProxy();
  //       Literal* key_literal = property->key()->AsLiteral();
  //       if (obj_proxy != NULL &&
  //           key_literal != NULL &&
  //           obj_proxy->IsArguments() &&
  //           key_literal->handle()->IsSmi()) {
  //         // Load arguments object if there are no eval-introduced
  //         // variables. Then load the argument from the arguments
  //         // object using keyed load.
  //         __ ldr(r1,
  //                ContextSlotOperandCheckExtensions(obj_proxy->var()->AsSlot(),
  //                                                  slow));
  //         __ mov(r0, Operand(key_literal->handle()));
  //         Handle<Code> ic =
  //             isolate()->builtins()->KeyedLoadIC_Initialize();
  //         EmitCallIC(ic, RelocInfo::CODE_TARGET);
  //         __ jmp(done);
  //       }
  //     }
  //   }
  // }
}


void FullCodeGenerator::VisitRegExpLiteral(RegExpLiteral* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitSwitchStatement(SwitchStatement* stmt) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitThisFunction(ThisFunction* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitUnaryOperation(UnaryOperation* expr) {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::VisitVariableProxy(VariableProxy* expr) {
  __ UNIMPLEMENTED_BREAK();
}


Register FullCodeGenerator::result_register() {
  return r0;
}


Register FullCodeGenerator::context_register() {
  return cp;
}


void FullCodeGenerator::EmitCallIC(Handle<Code> ic, RelocInfo::Mode mode) {
  ASSERT(mode == RelocInfo::CODE_TARGET ||
         mode == RelocInfo::CODE_TARGET_CONTEXT);
  Counters* counters = isolate()->counters();
  switch (ic->kind()) {
    case Code::LOAD_IC:
      __ IncrementCounter(counters->named_load_full(), 1, r1, r2);
      break;
    case Code::KEYED_LOAD_IC:
      __ IncrementCounter(counters->keyed_load_full(), 1, r1, r2);
      break;
    case Code::STORE_IC:
      __ IncrementCounter(counters->named_store_full(), 1, r1, r2);
      break;
    case Code::KEYED_STORE_IC:
      __ IncrementCounter(counters->keyed_store_full(), 1, r1, r2);
    default:
      break;
  }
  __ Call(ic, mode);
}


void FullCodeGenerator::AccumulatorValueContext::Plug(bool flag) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::StackValueContext::Plug(Slot* slot) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Label* materialize_true,
    Label* materialize_false) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(Slot* slot) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Handle<Object> lit) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Heap::RootListIndex index) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::StackValueContext::Plug(
    Heap::RootListIndex index) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::AccumulatorValueContext::DropAndPlug(
    int count,
    Register reg) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::StackValueContext::Plug(Handle<Object> lit) const {
  // Immediates cannot be pushed directly.
  __ mov(result_register(), Operand(lit));
  __ push(result_register());
}


void FullCodeGenerator::StackValueContext::DropAndPlug(int count,
                                                       Register reg) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::StackValueContext::Plug(
    Label* materialize_true,
    Label* materialize_false) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::StackValueContext::Plug(bool flag) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::TestContext::Plug(Slot* slot) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::TestContext::Plug(Heap::RootListIndex index) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::TestContext::Plug(Handle<Object> lit) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::TestContext::DropAndPlug(int count,
                                                 Register reg) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::TestContext::Plug(Label* materialize_true,
                                          Label* materialize_false) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::TestContext::Plug(bool flag) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EffectContext::Plug(Slot* slot) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EffectContext::Plug(Heap::RootListIndex index) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EffectContext::Plug(Handle<Object> lit) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EffectContext::DropAndPlug(int count,
                                                   Register reg) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EffectContext::Plug(Label* materialize_true,
                                            Label* materialize_false) const {
  __ UNIMPLEMENTED_BREAK();
}


void FullCodeGenerator::EffectContext::Plug(bool flag) const {
  __ UNIMPLEMENTED_BREAK();
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
