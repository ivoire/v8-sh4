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
    // Argument to NewContext is the function, which is in r1.
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
  __ bkpt();__ dw(0x0);//UNIMPLEMENTED();
}


void FullCodeGenerator::DeclareGlobals(Handle<FixedArray> pairs) {
  __ bkpt();__ dw(0x1);//UNIMPLEMENTED();
}


void FullCodeGenerator::DoTest(Label* if_true,
                               Label* if_false,
                               Label* fall_through) {
  __ bkpt();__ dw(0x2);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitArguments(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x3);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitArgumentsLength(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x4);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitBinaryOp(Token::Value op,
                                     OverwriteMode mode) {
  __ bkpt();__ dw(0x5);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitCallFunction(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x6);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitClassOf(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x7);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitFastAsciiArrayJoin(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x8);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitGetCachedArrayIndex(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x9);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitGetFromCache(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0xa);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitHasCachedArrayIndex(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0xb);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitInlineSmiBinaryOp(Expression* expr,
                                              Token::Value op,
                                              OverwriteMode mode,
                                              Expression* left,
                                              Expression* right) {
  __ bkpt();__ dw(0xc);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsArray(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0xd);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsConstructCall(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0xe);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsFunction(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0xf);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsNonNegativeSmi(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x10);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsObject(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x11);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsRegExp(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x12);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsRegExpEquivalent(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x13);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsSmi(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x14);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsSpecObject(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x15);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsStringWrapperSafeForDefaultValueOf(
    ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x16);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitIsUndetectableObject(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x17);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitLog(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x18);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathCos(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x19);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathLog(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x1a);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathPow(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x1b);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathSin(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x1c);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitMathSqrt(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x1d);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitNewClosure(Handle<SharedFunctionInfo> info,
                                       bool pretenure) {
  __ bkpt();__ dw(0x1e);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitNumberToString(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x1f);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitObjectEquals(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x20);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitRandomHeapNumber(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x21);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitRegExpConstructResult(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x22);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitRegExpExec(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x23);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitReturnSequence() {
  __ bkpt();__ dw(0x24);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitSetValueOf(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x25);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStackCheck(IterationStatement* stmt) {
  __ bkpt();__ dw(0x26);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringAdd(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x27);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringCharAt(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x28);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringCharCodeAt(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x29);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringCharFromCode(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x2a);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitStringCompare(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x2b);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitSubString(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x2c);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitSwapElements(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x2d);//UNIMPLEMENTED();
}


void FullCodeGenerator::EmitValueOf(ZoneList<Expression*>* args) {
  __ bkpt();__ dw(0x2e);//UNIMPLEMENTED();
}


void FullCodeGenerator::EnterFinallyBlock() {
  __ bkpt();__ dw(0x2f);//UNIMPLEMENTED();
}


void FullCodeGenerator::ExitFinallyBlock() {
  __ bkpt();__ dw(0x30);//UNIMPLEMENTED();
}


void FullCodeGenerator::LoadContextField(Register dst, int context_index) {
  __ bkpt();__ dw(0x31);//UNIMPLEMENTED();
}


void FullCodeGenerator::PrepareForBailoutBeforeSplit(State state,
                                                     bool should_normalize,
                                                     Label* if_true,
                                                     Label* if_false) {
  __ bkpt();__ dw(0x32);//UNIMPLEMENTED();
}


// Clobbers: rtmp, r0, r1, r2
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


void FullCodeGenerator::StoreToFrameField(int frame_offset, Register value) {
  __ bkpt();__ dw(0x33);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitArrayLiteral(ArrayLiteral* expr) {
  __ bkpt();__ dw(0x34);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitAssignment(Assignment* expr) {
  __ bkpt();__ dw(0x35);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCall(Call* expr) {
  __ bkpt();__ dw(0x36);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCallNew(CallNew* expr) {
  __ bkpt();__ dw(0x37);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCallRuntime(CallRuntime* expr) {
  __ bkpt();__ dw(0x38);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCompareOperation(CompareOperation* expr) {
  __ bkpt();__ dw(0x39);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCompareToNull(CompareToNull* expr) {
  __ bkpt();__ dw(0x3a);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitCountOperation(CountOperation* expr) {
  __ bkpt();__ dw(0x3b);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitDeclaration(Declaration* decl) {
  __ bkpt();__ dw(0x3c);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitForInStatement(ForInStatement* stmt) {
  __ bkpt();__ dw(0x3d);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitObjectLiteral(ObjectLiteral* expr) {
  __ bkpt();__ dw(0x3e);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitProperty(Property* expr) {
  __ bkpt();__ dw(0x3f);//UNIMPLEMENTED();
}

// clobbers: rtmp, r0, r1
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
      __ LoadRoot(rtmp, Heap::kTheHoleValueRootIndex);
      __ cmpeq(r0, rtmp);
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
  __ bkpt();__ dw(0x40);//UNIMPLEMENTED();
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
  __ bkpt();__ dw(0x41);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitSwitchStatement(SwitchStatement* stmt) {
  __ bkpt();__ dw(0x42);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitThisFunction(ThisFunction* expr) {
  __ bkpt();__ dw(0x43);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitUnaryOperation(UnaryOperation* expr) {
  __ bkpt();__ dw(0x44);//UNIMPLEMENTED();
}


void FullCodeGenerator::VisitVariableProxy(VariableProxy* expr) {
  __ bkpt();__ dw(0x45);//UNIMPLEMENTED();
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
  __ bkpt();__ dw(0x46);//UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(Slot* slot) const {
  __ bkpt();__ dw(0x47);//UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Label* materialize_true,
    Label* materialize_false) const {
  __ bkpt();__ dw(0x48);//UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(Slot* slot) const {
  __ bkpt();__ dw(0x49);//UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Handle<Object> lit) const {
  __ bkpt();__ dw(0x4a);//UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::Plug(
    Heap::RootListIndex index) const {
  __ bkpt();__ dw(0x4b);//UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(
    Heap::RootListIndex index) const {
  __ bkpt();__ dw(0x4c);//UNIMPLEMENTED();
}


void FullCodeGenerator::AccumulatorValueContext::DropAndPlug(
    int count,
    Register reg) const {
  __ bkpt();__ dw(0x4d);//UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(Handle<Object> lit) const {
  __ bkpt();__ dw(0x4e);//UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::DropAndPlug(int count,
                                                       Register reg) const {
  __ bkpt();__ dw(0x4f);//UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(
    Label* materialize_true,
    Label* materialize_false) const {
  __ bkpt();__ dw(0x50);//UNIMPLEMENTED();
}


void FullCodeGenerator::StackValueContext::Plug(bool flag) const {
  __ bkpt();__ dw(0x51);//UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(Slot* slot) const {
  __ bkpt();__ dw(0x52);//UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(Heap::RootListIndex index) const {
  __ bkpt();__ dw(0x53);//UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(Handle<Object> lit) const {
  __ bkpt();__ dw(0x54);//UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::DropAndPlug(int count,
                                                 Register reg) const {
  __ bkpt();__ dw(0x55);//UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(Label* materialize_true,
                                          Label* materialize_false) const {
  __ bkpt();__ dw(0x56);//UNIMPLEMENTED();
}


void FullCodeGenerator::TestContext::Plug(bool flag) const {
  __ bkpt();__ dw(0x57);//UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(Slot* slot) const {
  __ bkpt();__ dw(0x58);//UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(Heap::RootListIndex index) const {
  __ bkpt();__ dw(0x59);//UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(Handle<Object> lit) const {
  __ bkpt();__ dw(0x5a);//UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::DropAndPlug(int count,
                                                   Register reg) const {
  __ bkpt();__ dw(0x5b);//UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(Label* materialize_true,
                                            Label* materialize_false) const {
  __ bkpt();__ dw(0x5c);//UNIMPLEMENTED();
}


void FullCodeGenerator::EffectContext::Plug(bool flag) const {
  __ bkpt();__ dw(0x5d);//UNIMPLEMENTED();
}


} }  // namespace v8::internal

#endif  // V8_TARGET_ARCH_IA32
