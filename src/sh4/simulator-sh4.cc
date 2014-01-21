// Copyright 2012 the V8 project authors. All rights reserved.
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

#include <stdlib.h>
#include <cmath>
#include <cstdarg>
#include "v8.h"

#if V8_TARGET_ARCH_SH4

#include "disasm.h"
#include "assembler.h"
#include "codegen.h"
#include "sh4/constants-sh4.h"
#include "sh4/simulator-sh4.h"

#if defined(USE_SIMULATOR)

// Only build the simulator if not compiling for real SH4 hardware.
namespace v8 {
namespace internal {

// This macro provides a platform independent use of sscanf. The reason for
// SScanF not being implemented in a platform independent way through
// ::v8::internal::OS in the same way as SNPrintF is that the
// Windows C Run-Time Library does not provide vsscanf.
#define SScanF sscanf  // NOLINT

// The Sh4Debugger class is used by the simulator while debugging simulated SH4
// code.
class Sh4Debugger {
 public:
  explicit Sh4Debugger(Simulator* sim) : sim_(sim) { }
  ~Sh4Debugger();

  void Stop(Instruction* instr);
  void Debug();

 private:
  static const Instr kBreakpointInstr = kBreakpoint;

  Simulator* sim_;

  int32_t GetRegisterValue(int regnum);
  double GetFPUDoubleRegisterValue(int regnum);
  bool GetValue(const char* desc, int32_t* value);
  bool GetFPUSingleValue(const char* desc, float* value);
  bool GetFPUDoubleValue(const char* desc, double* value);

  // Set or delete a breakpoint. Returns true if successful.
  bool SetBreakpoint(Instruction* breakpc);
  bool DeleteBreakpoint(Instruction* breakpc);

  // Undo and redo all breakpoints. This is needed to bracket disassembly and
  // execution to skip past breakpoints when run from the debugger.
  void UndoBreakpoints();
  void RedoBreakpoints();
};


Sh4Debugger::~Sh4Debugger() {
}


void Sh4Debugger::Stop(Instruction* instr) {
  // Retrieve the encoded address, which comes just after this stop.
  char* msg = *reinterpret_cast<char**>(sim_->get_pc()
                                        + Instruction::kInstrSize);

  PrintF("Simulator hit %s\n", msg);
  sim_->set_pc(sim_->get_pc() + 3 * Instruction::kInstrSize);
  Debug();
}


int32_t Sh4Debugger::GetRegisterValue(int regnum) {
  return sim_->get_register(regnum);
}


double Sh4Debugger::GetFPUDoubleRegisterValue(int regnum) {
  return sim_->get_dregister(regnum);
}


bool Sh4Debugger::GetValue(const char* desc, int32_t* value) {
  int regnum = Registers::Number(desc);
  if (regnum != kNoRegister) {
    *value = GetRegisterValue(regnum);
    return true;
  } else {
    if (strncmp(desc, "0x", 2) == 0) {
      return SScanF(desc + 2, "%x", reinterpret_cast<uint32_t*>(value)) == 1;
    } else {
      return SScanF(desc, "%u", reinterpret_cast<uint32_t*>(value)) == 1;
    }
  }
  return false;
}


bool Sh4Debugger::GetFPUSingleValue(const char* desc, float* value) {
  bool is_double;
  int regnum = FPURegisters::Number(desc, &is_double);
  if (regnum != kNoRegister && !is_double) {
    *value = sim_->get_fregister(regnum);
    return true;
  }
  return false;
}


bool Sh4Debugger::GetFPUDoubleValue(const char* desc, double* value) {
  bool is_double;
  int regnum = FPURegisters::Number(desc, &is_double);
  if (regnum != kNoRegister && is_double) {
    *value = sim_->get_dregister(regnum);
    return true;
  }
  return false;
}


bool Sh4Debugger::SetBreakpoint(Instruction* breakpc) {
  // Check if a breakpoint can be set. If not return without any side-effects.
  if (sim_->break_pc_ != NULL) {
    return false;
  }

  // Set the breakpoint.
  sim_->break_pc_ = breakpc;
  sim_->break_instr_ = breakpc->InstructionBits();
  // Not setting the breakpoint instruction in the code itself. It will be set
  // when the debugger shell continues.
  return true;
}


bool Sh4Debugger::DeleteBreakpoint(Instruction* breakpc) {
  if (sim_->break_pc_ != NULL) {
    sim_->break_pc_->SetInstructionBits(sim_->break_instr_);
  }

  sim_->break_pc_ = NULL;
  sim_->break_instr_ = 0;
  return true;
}


void Sh4Debugger::UndoBreakpoints() {
  if (sim_->break_pc_ != NULL) {
    sim_->break_pc_->SetInstructionBits(sim_->break_instr_);
  }
}


void Sh4Debugger::RedoBreakpoints() {
  if (sim_->break_pc_ != NULL) {
    sim_->break_pc_->SetInstructionBits(kBreakpointInstr);
  }
}


void Sh4Debugger::Debug() {
  intptr_t last_pc = -1;
  bool done = false;

#define COMMAND_SIZE 63
#define ARG_SIZE 255

#define STR(a) #a
#define XSTR(a) STR(a)

  char cmd[COMMAND_SIZE + 1];
  char arg1[ARG_SIZE + 1];
  char arg2[ARG_SIZE + 1];
  char* argv[3] = { cmd, arg1, arg2 };

  // make sure to have a proper terminating character if reaching the limit
  cmd[COMMAND_SIZE] = 0;
  arg1[ARG_SIZE] = 0;
  arg2[ARG_SIZE] = 0;

  // Undo all set breakpoints while running in the debugger shell. This will
  // make them invisible to all commands.
  UndoBreakpoints();

  while (!done && !sim_->has_bad_pc()) {
    if (last_pc != sim_->get_pc()) {
      disasm::NameConverter converter;
      disasm::Disassembler dasm(converter);
      // use a reasonably large buffer
      v8::internal::EmbeddedVector<char, 256> buffer;
      dasm.InstructionDecode(buffer,
                             reinterpret_cast<byte*>(sim_->get_pc()));
      PrintF("  0x%08x  %s\n", sim_->get_pc(), buffer.start());
      last_pc = sim_->get_pc();
    }
    char* line = ReadLine("sim> ");
    if (line == NULL) {
      break;
    } else {
      char* last_input = sim_->last_debugger_input();
      if (strcmp(line, "\n") == 0 && last_input != NULL) {
        line = last_input;
      } else {
        // Ownership is transferred to sim_;
        sim_->set_last_debugger_input(line);
      }
      // Use sscanf to parse the individual parts of the command line. At the
      // moment no command expects more than two parameters.
      int argc = SScanF(line,
                        "%" XSTR(COMMAND_SIZE) "s "
                        "%" XSTR(ARG_SIZE) "s "
                        "%" XSTR(ARG_SIZE) "s",
                        cmd, arg1, arg2);
      if ((strcmp(cmd, "si") == 0) || (strcmp(cmd, "stepi") == 0)) {
        sim_->InstructionDecode(reinterpret_cast<Instruction*>(sim_->get_pc()));
      } else if ((strcmp(cmd, "c") == 0) || (strcmp(cmd, "cont") == 0)) {
        // Execute the one instruction we broke at with breakpoints disabled.
        sim_->InstructionDecode(reinterpret_cast<Instruction*>(sim_->get_pc()));
        // Leave the debugger shell.
        done = true;
      } else if ((strcmp(cmd, "p") == 0) || (strcmp(cmd, "print") == 0)) {
        if (argc == 2 || (argc == 3 && strcmp(arg2, "fp") == 0)) {
          int32_t value;
          float svalue;
          double dvalue;
          if (strcmp(arg1, "all") == 0) {
            for (int i = 0; i < kNumRegisters; i++) {
              value = GetRegisterValue(i);
              PrintF("%3s: 0x%08x %10d\n", Registers::Name(i), value, value);
            }
            for (int i = 0; i < kNumFPUDoubleRegisters; i++) {
              dvalue = GetFPUDoubleRegisterValue(i * 2);
              uint64_t as_words = BitCast<uint64_t>(dvalue);
              PrintF("%3s: %f 0x%08x %08x\n",
                     FPURegisters::Name(i * 2, true),
                     dvalue,
                     static_cast<uint32_t>(as_words >> 32),
                     static_cast<uint32_t>(as_words & 0xffffffff));
            }
          } else {
            if (GetValue(arg1, &value)) {
              PrintF("%s: 0x%08x %d \n", arg1, value, value);
            } else if (GetFPUSingleValue(arg1, &svalue)) {
              uint32_t as_word = BitCast<uint32_t>(svalue);
              PrintF("%s: %f 0x%08x\n", arg1, svalue, as_word);
            } else if (GetFPUDoubleValue(arg1, &dvalue)) {
              uint64_t as_words = BitCast<uint64_t>(dvalue);
              PrintF("%s: %f 0x%08x %08x\n",
                     arg1,
                     dvalue,
                     static_cast<uint32_t>(as_words >> 32),
                     static_cast<uint32_t>(as_words & 0xffffffff));
            } else {
              PrintF("%s unrecognized\n", arg1);
            }
          }
        } else {
          PrintF("print <register>\n");
        }
      } else if ((strcmp(cmd, "po") == 0)
                 || (strcmp(cmd, "printobject") == 0)) {
        if (argc == 2) {
          int32_t value;
          if (GetValue(arg1, &value)) {
            Object* obj = reinterpret_cast<Object*>(value);
            PrintF("%s: \n", arg1);
#ifdef DEBUG
            obj->PrintLn();
#else
            obj->ShortPrint();
            PrintF("\n");
#endif
          } else {
            PrintF("%s unrecognized\n", arg1);
          }
        } else {
          PrintF("printobject <value>\n");
        }
      } else if (strcmp(cmd, "stack") == 0 || strcmp(cmd, "mem") == 0) {
        int32_t* cur = NULL;
        int32_t* end = NULL;
        int next_arg = 1;

        if (strcmp(cmd, "stack") == 0) {
          cur = reinterpret_cast<int32_t*>(sim_->get_register(Simulator::sp));
        } else {  // "mem"
          int32_t value;
          if (!GetValue(arg1, &value)) {
            PrintF("%s unrecognized\n", arg1);
            continue;
          }
          cur = reinterpret_cast<int32_t*>(value);
          next_arg++;
        }

        int32_t words;
        if (argc == next_arg) {
          words = 10;
        } else {
          if (!GetValue(argv[next_arg], &words)) {
            words = 10;
          }
        }
        end = cur + words;

        while (cur < end) {
          PrintF("  0x%08x:  0x%08x %10d",
                 reinterpret_cast<intptr_t>(cur), *cur, *cur);
          HeapObject* obj = reinterpret_cast<HeapObject*>(*cur);
          int value = *cur;
          Heap* current_heap = v8::internal::Isolate::Current()->heap();
          if (((value & 1) == 0) || current_heap->Contains(obj)) {
            PrintF(" (");
            if ((value & 1) == 0) {
              PrintF("smi %d", value / 2);
            } else {
              obj->ShortPrint();
            }
            PrintF(")");
          }
          PrintF("\n");
          cur++;
        }
      } else if (strcmp(cmd, "disasm") == 0 || strcmp(cmd, "di") == 0) {
        disasm::NameConverter converter;
        disasm::Disassembler dasm(converter);
        // use a reasonably large buffer
        v8::internal::EmbeddedVector<char, 256> buffer;

        byte* prev = NULL;
        byte* cur = NULL;
        byte* end = NULL;

        if (argc == 1) {
          cur = reinterpret_cast<byte*>(sim_->get_pc());
          end = cur + (10 * Instruction::kInstrSize);
        } else if (argc == 2) {
          int regnum = Registers::Number(arg1);
          if (regnum != kNoRegister || strncmp(arg1, "0x", 2) == 0) {
            // The argument is an address or a register name.
            int32_t value;
            if (GetValue(arg1, &value)) {
              cur = reinterpret_cast<byte*>(value);
              // Disassemble 10 instructions at <arg1>.
              end = cur + (10 * Instruction::kInstrSize);
            }
          } else {
            // The argument is the number of instructions.
            int32_t value;
            if (GetValue(arg1, &value)) {
              cur = reinterpret_cast<byte*>(sim_->get_pc());
              // Disassemble <arg1> instructions.
              end = cur + (value * Instruction::kInstrSize);
            }
          }
        } else {
          int32_t value1;
          int32_t value2;
          if (GetValue(arg1, &value1) && GetValue(arg2, &value2)) {
            cur = reinterpret_cast<byte*>(value1);
            end = cur + (value2 * Instruction::kInstrSize);
          }
        }

        while (cur < end) {
          prev = cur;
          cur += dasm.InstructionDecode(buffer, cur);
          PrintF("  0x%08x  %s\n",
                 reinterpret_cast<intptr_t>(prev), buffer.start());
        }
      } else if (strcmp(cmd, "gdb") == 0) {
        PrintF("relinquishing control to gdb\n");
        v8::internal::OS::DebugBreak();
        PrintF("regaining control from gdb\n");
      } else if (strcmp(cmd, "break") == 0) {
        if (argc == 2) {
          int32_t value;
          if (GetValue(arg1, &value)) {
            if (!SetBreakpoint(reinterpret_cast<Instruction*>(value))) {
              PrintF("setting breakpoint failed\n");
            }
          } else {
            PrintF("%s unrecognized\n", arg1);
          }
        } else {
          PrintF("break <address>\n");
        }
      } else if (strcmp(cmd, "del") == 0) {
        if (!DeleteBreakpoint(NULL)) {
          PrintF("deleting breakpoint failed\n");
        }
      } else if (strcmp(cmd, "flags") == 0) {
        int fpscr = sim_->get_sregister(Simulator::fpscr);
        PrintF("pc: 0x%08x; pr: 0x%08x\n", sim_->get_pc(),
                                           sim_->get_sregister(Simulator::pr));
        PrintF("T flag: %d; ", sim_->get_t_flag());
        PrintF("S flag: %d; ", sim_->get_s_flag());
        PrintF("Q flag: %d; ", sim_->get_q_flag());
        PrintF("M flag: %d\n", sim_->get_m_flag());
        PrintF("FPSCR: 0x%08x\n", fpscr);
        PrintF(" INVALID OP flag:  %d / %d\n", fpscr & (1 << 11), fpscr & (1 << 16));
        PrintF(" DIV BY ZERO flag: %d / %d\n", fpscr & (1 << 10), fpscr & (1 << 15));
        PrintF(" OVERFLOW flag:    %d / %d\n", fpscr & (1 << 9), fpscr & (1 << 14));
        PrintF(" UNDERFLOW flag:   %d / %d\n", fpscr & (1 << 8), fpscr & (1 << 13));
        PrintF(" INEXACT flag:     %d / %d\n", fpscr & (1 << 7), fpscr & (1 << 12));
      }
#if 0
else if (strcmp(cmd, "stop") == 0) {
        int32_t value;
        intptr_t stop_pc = sim_->get_pc() - 2 * Instruction::kInstrSize;
        Instruction* stop_instr = reinterpret_cast<Instruction*>(stop_pc);
        Instruction* msg_address =
          reinterpret_cast<Instruction*>(stop_pc + Instruction::kInstrSize);
        if ((argc == 2) && (strcmp(arg1, "unstop") == 0)) {
          // Remove the current stop.
          if (sim_->isStopInstruction(stop_instr)) {
            stop_instr->SetInstructionBits(kNopInstr);
            msg_address->SetInstructionBits(kNopInstr);
          } else {
            PrintF("Not at debugger stop.\n");
          }
        } else if (argc == 3) {
          // Print information about all/the specified breakpoint(s).
          if (strcmp(arg1, "info") == 0) {
            if (strcmp(arg2, "all") == 0) {
              PrintF("Stop information:\n");
              for (uint32_t i = 0; i < sim_->kNumOfWatchedStops; i++) {
                sim_->PrintStopInfo(i);
              }
            } else if (GetValue(arg2, &value)) {
              sim_->PrintStopInfo(value);
            } else {
              PrintF("Unrecognized argument.\n");
            }
          } else if (strcmp(arg1, "enable") == 0) {
            // Enable all/the specified breakpoint(s).
            if (strcmp(arg2, "all") == 0) {
              for (uint32_t i = 0; i < sim_->kNumOfWatchedStops; i++) {
                sim_->EnableStop(i);
              }
            } else if (GetValue(arg2, &value)) {
              sim_->EnableStop(value);
            } else {
              PrintF("Unrecognized argument.\n");
            }
          } else if (strcmp(arg1, "disable") == 0) {
            // Disable all/the specified breakpoint(s).
            if (strcmp(arg2, "all") == 0) {
              for (uint32_t i = 0; i < sim_->kNumOfWatchedStops; i++) {
                sim_->DisableStop(i);
              }
            } else if (GetValue(arg2, &value)) {
              sim_->DisableStop(value);
            } else {
              PrintF("Unrecognized argument.\n");
            }
          }
        } else {
          PrintF("Wrong usage. Use help command for more information.\n");
        }
#endif
      else if ((strcmp(cmd, "t") == 0) || strcmp(cmd, "trace") == 0) {
        ::v8::internal::FLAG_trace_sim = !::v8::internal::FLAG_trace_sim;
        PrintF("Trace of executed instructions is %s\n",
               ::v8::internal::FLAG_trace_sim ? "on" : "off");
      } else if (strcmp(cmd, "unimp") == 0) {
        const char* psz_file;
        int file_id = sim_->get_register(Simulator::r0);
        switch (file_id) {
        case 1518: psz_file = "assembler-sh4.cc"; break;
        case 1424: psz_file = "assembler-sh4.h"; break;
        case 1792: psz_file = "assembler-sh4-inl.h"; break;
        case 1434: psz_file = "builtins-sh4.cc"; break;
        case 1285: psz_file = "codegen-sh4.cc"; break;
        case 1191: psz_file = "codegen-sh4.h"; break;
        case 1577: psz_file = "code-stubs-sh4.cc"; break;
        case 1483: psz_file = "code-stubs-sh4.h"; break;
        case 1549: psz_file = "constants-sh4.cc"; break;
        case 1455: psz_file = "constants-sh4.h"; break;
        case 888:  psz_file = "cpu-sh4.cc"; break;
        case 1765: psz_file = "full-codegen-sh4.cc"; break;
        case 764:  psz_file = "ic-sh4.cc"; break;
        case 2093: psz_file = "macro-assembler-sh4.cc"; break;
        case 2789: psz_file = "regexp-macro-assembler-sh4.cc"; break;
        case 1551: psz_file = "stub-cache-sh4.cc"; break;
        default:   psz_file = "???";
        }
        PrintF("File: %s (%d)\nLine: %d\n", psz_file, file_id, sim_->get_register(Simulator::r1));
      } else if ((strcmp(cmd, "h") == 0) || (strcmp(cmd, "help") == 0)) {
        PrintF("cont\n");
        PrintF("  continue execution (alias 'c')\n");
        PrintF("stepi\n");
        PrintF("  step one instruction (alias 'si')\n");
        PrintF("print <register>\n");
        PrintF("  print register content (alias 'p')\n");
        PrintF("  use register name 'all' to print all registers\n");
        PrintF("  add argument 'fp' to print register pair double values\n");
        PrintF("printobject <register>\n");
        PrintF("  print an object from a register (alias 'po')\n");
        PrintF("flags\n");
        PrintF("  print flags\n");
        PrintF("stack [<words>]\n");
        PrintF("  dump stack content, default dump 10 words)\n");
        PrintF("mem <address> [<words>]\n");
        PrintF("  dump memory content, default dump 10 words)\n");
        PrintF("disasm [<instructions>]\n");
        PrintF("disasm [<address/register>]\n");
        PrintF("disasm [[<address/register>] <instructions>]\n");
        PrintF("  disassemble code, default is 10 instructions\n");
        PrintF("  from pc (alias 'di')\n");
        PrintF("gdb\n");
        PrintF("  enter gdb\n");
        PrintF("break <address>\n");
        PrintF("  set a break point on the address\n");
        PrintF("del\n");
        PrintF("  delete the breakpoint\n");
        PrintF("trace (alias 't')\n");
        PrintF("  toogle the tracing of all executed statements\n");
        PrintF("unimp\n");
        PrintF("  give __FILE__ and __LINE__ of the current break\n");
#if 0
        PrintF("stop feature:\n");
        PrintF("  Description:\n");
        PrintF("    Stops are debug instructions inserted by\n");
        PrintF("    the Assembler::stop() function.\n");
        PrintF("    When hitting a stop, the Simulator will\n");
        PrintF("    stop and and give control to the ArmDebugger.\n");
        PrintF("    The first %d stop codes are watched:\n",
               Simulator::kNumOfWatchedStops);
        PrintF("    - They can be enabled / disabled: the Simulator\n");
        PrintF("      will / won't stop when hitting them.\n");
        PrintF("    - The Simulator keeps track of how many times they \n");
        PrintF("      are met. (See the info command.) Going over a\n");
        PrintF("      disabled stop still increases its counter. \n");
        PrintF("  Commands:\n");
        PrintF("    stop info all/<code> : print infos about number <code>\n");
        PrintF("      or all stop(s).\n");
        PrintF("    stop enable/disable all/<code> : enables / disables\n");
        PrintF("      all or number <code> stop(s)\n");
        PrintF("    stop unstop\n");
        PrintF("      ignore the stop instruction at the current location\n");
        PrintF("      from now on\n");
#endif
      } else {
        PrintF("Unknown command: %s\n", cmd);
      }
    }
  }

  // Add all the breakpoints back to stop execution and enter the debugger
  // shell when hit.
  RedoBreakpoints();

#undef COMMAND_SIZE
#undef ARG_SIZE

#undef STR
#undef XSTR
}


static bool ICacheMatch(void* one, void* two) {
  ASSERT((reinterpret_cast<intptr_t>(one) & CachePage::kPageMask) == 0);
  ASSERT((reinterpret_cast<intptr_t>(two) & CachePage::kPageMask) == 0);
  return one == two;
}


static uint32_t ICacheHash(void* key) {
  return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(key)) >> 2;
}


static bool AllOnOnePage(uintptr_t start, int size) {
  intptr_t start_page = (start & ~CachePage::kPageMask);
  intptr_t end_page = ((start + size) & ~CachePage::kPageMask);
  return start_page == end_page;
}


void Simulator::set_last_debugger_input(char* input) {
  DeleteArray(last_debugger_input_);
  last_debugger_input_ = input;
}


void Simulator::FlushICache(v8::internal::HashMap* i_cache,
                            void* start_addr,
                            size_t size) {
  intptr_t start = reinterpret_cast<intptr_t>(start_addr);
  int intra_line = (start & CachePage::kLineMask);
  start -= intra_line;
  size += intra_line;
  size = ((size - 1) | CachePage::kLineMask) + 1;
  int offset = (start & CachePage::kPageMask);
  while (!AllOnOnePage(start, size - 1)) {
    int bytes_to_flush = CachePage::kPageSize - offset;
    FlushOnePage(i_cache, start, bytes_to_flush);
    start += bytes_to_flush;
    size -= bytes_to_flush;
    ASSERT_EQ(0, start & CachePage::kPageMask);
    offset = 0;
  }
  if (size != 0) {
    FlushOnePage(i_cache, start, size);
  }
}


CachePage* Simulator::GetCachePage(v8::internal::HashMap* i_cache, void* page) {
  v8::internal::HashMap::Entry* entry = i_cache->Lookup(page,
                                                        ICacheHash(page),
                                                        true);
  if (entry->value == NULL) {
    CachePage* new_page = new CachePage();
    entry->value = new_page;
  }
  return reinterpret_cast<CachePage*>(entry->value);
}


// Flush from start up to and not including start + size.
void Simulator::FlushOnePage(v8::internal::HashMap* i_cache,
                             intptr_t start,
                             int size) {
  ASSERT(size <= CachePage::kPageSize);
  ASSERT(AllOnOnePage(start, size - 1));
  ASSERT((start & CachePage::kLineMask) == 0);
  ASSERT((size & CachePage::kLineMask) == 0);
  void* page = reinterpret_cast<void*>(start & (~CachePage::kPageMask));
  int offset = (start & CachePage::kPageMask);
  CachePage* cache_page = GetCachePage(i_cache, page);
  char* valid_bytemap = cache_page->ValidityByte(offset);
  memset(valid_bytemap, CachePage::LINE_INVALID, size >> CachePage::kLineShift);
}


void Simulator::CheckICache(v8::internal::HashMap* i_cache,
                            Instruction* instr) {
  intptr_t address = reinterpret_cast<intptr_t>(instr);
  void* page = reinterpret_cast<void*>(address & (~CachePage::kPageMask));
  void* line = reinterpret_cast<void*>(address & (~CachePage::kLineMask));
  int offset = (address & CachePage::kPageMask);
  CachePage* cache_page = GetCachePage(i_cache, page);
  char* cache_valid_byte = cache_page->ValidityByte(offset);
  bool cache_hit = (*cache_valid_byte == CachePage::LINE_VALID);
  char* cached_line = cache_page->CachedData(offset & ~CachePage::kLineMask);
  if (cache_hit) {
    // Check that the data in memory matches the contents of the I-cache.
    CHECK(memcmp(reinterpret_cast<void*>(instr),
                 cache_page->CachedData(offset),
                 Instruction::kInstrSize) == 0);
  } else {
    // Cache miss.  Load memory into the cache.
    OS::MemCopy(cached_line, line, CachePage::kLineLength);
    *cache_valid_byte = CachePage::LINE_VALID;
  }
}


void Simulator::Initialize(Isolate* isolate) {
  if (isolate->simulator_initialized()) return;
  isolate->set_simulator_initialized(true);
  ::v8::internal::ExternalReference::set_redirector(isolate,
                                                    &RedirectExternalReference);
}


Simulator::Simulator(Isolate* isolate) : isolate_(isolate) {
  i_cache_ = isolate_->simulator_i_cache();
  if (i_cache_ == NULL) {
    i_cache_ = new v8::internal::HashMap(&ICacheMatch);
    isolate_->set_simulator_i_cache(i_cache_);
  }
  Initialize(isolate);
  // Set up simulator support first. Some of this information is needed to
  // setup the architecture state.
  size_t stack_size = 1 * 1024*1024;  // allocate 1MB for stack
  stack_ = reinterpret_cast<char*>(malloc(stack_size));
  pc_modified_ = false;
  icount_ = 0;

  // Set up architecture state.
  // All registers are initialized to zero to start with.
  for (int i = 0; i < num_registers; i++) {
    registers_[i] = 0;
  }
  t_flag_ = false;
  s_flag_ = false;
  q_flag_ = false;
  m_flag_ = false;

  sregs_.mach = 0;
  sregs_.macl = 0;
  sregs_.pr = 0;
  sregs_.fpul = 0;
  sregs_.fpscr = 1 | 1 << 19;

  // The sp is initialized to point to the bottom (high address) of the
  // allocated stack area. To be safe in potential stack underflows we leave
  // some buffer below.
  registers_[sp] = reinterpret_cast<int32_t>(stack_) + stack_size - 64;
  // The pr and pc are initialized to a known bad value that will cause an
  // access violation if the simulator ever tries to execute it.
  pc_ = bad_pr;
  sregs_.pr = bad_pr;

  in_delay_slot_ = false;
}


// When the generated code calls an external reference we need to catch that in
// the simulator.  The external reference will be a function compiled for the
// host architecture.  We need to call that function instead of trying to
// execute it with the simulator.  We do that by redirecting the external
// reference to a svc (Supervisor Call) instruction that is handled by
// the simulator.  We write the original destination of the jump just at a known
// offset from the svc instruction so the simulator knows what to call.
class Redirection {
 public:
  Redirection(void* external_function, ExternalReference::Type type)
      : external_function_(external_function),
        swi_instruction_(kCallRtRedirected),
        type_(type),
        next_(NULL) {
    Isolate* isolate = Isolate::Current();
    next_ = isolate->simulator_redirection();
    Simulator::current(isolate)->
        FlushICache(isolate->simulator_i_cache(),
                    reinterpret_cast<void*>(&swi_instruction_),
                    Instruction::kInstrSize);
    isolate->set_simulator_redirection(this);
  }

  void* address_of_swi_instruction() {
    return reinterpret_cast<void*>(&swi_instruction_);
  }

  void* external_function() { return external_function_; }
  ExternalReference::Type type() { return type_; }

  static Redirection* Get(void* external_function,
                          ExternalReference::Type type) {
    Isolate* isolate = Isolate::Current();
    Redirection* current = isolate->simulator_redirection();
    for (; current != NULL; current = current->next_) {
      if (current->external_function_ == external_function) {
        ASSERT_EQ(current->type(), type);
        return current;
      }
    }
    return new Redirection(external_function, type);
  }

  static Redirection* FromSwiInstruction(Instruction* swi_instruction) {
    char* addr_of_swi = reinterpret_cast<char*>(swi_instruction);
    char* addr_of_redirection =
        addr_of_swi - OFFSET_OF(Redirection, swi_instruction_);
    return reinterpret_cast<Redirection*>(addr_of_redirection);
  }

 private:
  void* external_function_;
  uint32_t swi_instruction_;
  ExternalReference::Type type_;
  Redirection* next_;
};


void* Simulator::RedirectExternalReference(void* external_function,
                                           ExternalReference::Type type) {
  Redirection* redirection = Redirection::Get(external_function, type);
  return redirection->address_of_swi_instruction();
}


// Get the active Simulator for the current thread.
Simulator* Simulator::current(Isolate* isolate) {
  v8::internal::Isolate::PerIsolateThreadData* isolate_data =
      isolate->FindOrAllocatePerThreadDataForThisThread();
  ASSERT(isolate_data != NULL);

  Simulator* sim = isolate_data->simulator();
  if (sim == NULL) {
    // TODO(146): delete the simulator object when a thread/isolate goes away.
    sim = new Simulator(isolate);
    isolate_data->set_simulator(sim);
  }
  return sim;
}


// Sets the register in the architecture state.
void Simulator::set_register(int reg, int32_t value) {
  ASSERT((reg >= 0) && (reg < num_registers));
  // See: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43949
  if (reg >= num_registers) return;
  // End stupid code.

  registers_[reg] = value;
}


// Get the register from the architecture state. This function does handle
// the special case of accessing the PC register.
int32_t Simulator::get_register(int reg) const {
  ASSERT((reg >= 0) && (reg < num_registers));
  // Stupid code added to avoid bug in GCC.
  // See: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43949
  if (reg >= num_registers) return 0;
  // End stupid code.
  return registers_[reg];
}


void Simulator::set_fregister(int reg, int32_t value) {
  ASSERT((reg >= 0) && (reg < num_fregisters));
  // See: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43949
  if (reg >= num_fregisters) return;
  // End stupid code.

  if (!(reg % 2))
    fpu_registers_[reg / 2].f.low = value;
  else
    fpu_registers_[reg / 2].f.high = value;
}


int32_t Simulator::get_fregister(int reg) const {
  ASSERT((reg >= 0) && (reg < num_fregisters));
  // See: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43949
  if (reg >= num_fregisters) return 0;
  // End stupid code.

  if (!(reg % 2))
    return fpu_registers_[reg / 2].f.low;
  else
    return fpu_registers_[reg / 2].f.high;
}


void Simulator::set_dregister(int reg, double value) {
  int register_num = reg / 2;
  ASSERT((reg % 2 == 0) && (register_num >= 0) && (register_num < num_dregisters));
  // See: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43949
  if (register_num >= num_dregisters) return;
  // End stupid code.

  fpu_registers_[register_num].d = value;
}


double Simulator::get_dregister(int reg) const {
  int register_num = reg / 2;
  ASSERT((reg % 2 == 0) && (register_num >= 0) && (register_num < num_dregisters));
  // See: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=43949
  if (register_num >= num_dregisters) return 0;
  // End stupid code.

  return fpu_registers_[register_num].d;
}


// Raw access to the PC register.
void Simulator::set_pc(int32_t value) {
  pc_modified_ = true;
  pc_ = value;
}


bool Simulator::has_bad_pc() const {
  return ((pc_ == bad_pr) || (pc_ == end_sim_pc));
}


// Raw access to the PC register without the special adjustment when reading.
int32_t Simulator::get_pc() const {
  return pc_;
}


void Simulator::set_sregister(int num, int32_t value) {
  switch (num) {
  case mach:
    sregs_.mach = value;
    break;
  case macl:
    sregs_.macl = value;
    break;
  case pr:
    sregs_.pr = value;
    break;
  case fpul:
    sregs_.fpul = value;
    break;
  case fpscr:
    sregs_.fpscr = value;
    break;
  default:
    UNIMPLEMENTED();
  }
}


int32_t Simulator::get_sregister(int num) {
  switch (num) {
  case mach:
    return sregs_.mach;
  case macl:
    return sregs_.macl;
  case pr:
    return sregs_.pr;
  case fpul:
    return sregs_.fpul;
  case fpscr:
    return sregs_.fpscr;
  default:
    UNREACHABLE();
    return -1;
  }
}


int Simulator::ReadW(int32_t addr, Instruction* instr) {
  if ((addr & 3) == 0) {
    intptr_t* ptr = reinterpret_cast<intptr_t*>(addr);
    return *ptr;
  } else {
    PrintF("Unaligned read at 0x%08x, pc=0x%08" V8PRIxPTR "\n",
           addr,
           reinterpret_cast<intptr_t>(instr));
    UNIMPLEMENTED();
    return 0;
  }
}


void Simulator::WriteW(int32_t addr, int value, Instruction* instr) {
  if ((addr & 3) == 0) {
    intptr_t* ptr = reinterpret_cast<intptr_t*>(addr);
    *ptr = value;
  } else {
    PrintF("Unaligned write at 0x%08x, pc=0x%08" V8PRIxPTR "\n",
           addr,
           reinterpret_cast<intptr_t>(instr));
    UNIMPLEMENTED();
  }
}


uint16_t Simulator::ReadHU(int32_t addr, Instruction* instr) {
  if ((addr & 1) == 0) {
    uint16_t* ptr = reinterpret_cast<uint16_t*>(addr);
    return *ptr;
  } else {
    PrintF("Unaligned unsigned halfword read at 0x%08x, pc=0x%08"
           V8PRIxPTR "\n",
           addr,
           reinterpret_cast<intptr_t>(instr));
    UNIMPLEMENTED();
    return 0;
  }
}


int16_t Simulator::ReadH(int32_t addr, Instruction* instr) {
  if ((addr & 1) == 0) {
    int16_t* ptr = reinterpret_cast<int16_t*>(addr);
    return *ptr;
  } else {
    PrintF("Unaligned signed halfword read at 0x%08x\n", addr);
    UNIMPLEMENTED();
    return 0;
  }
}


void Simulator::WriteHU(int32_t addr, uint16_t value, Instruction* instr) {
  if ((addr & 1) == 0) {
    uint16_t* ptr = reinterpret_cast<uint16_t*>(addr);
    *ptr = value;
  } else {
    PrintF("Unaligned unsigned halfword write at 0x%08x, pc=0x%08"
           V8PRIxPTR "\n",
           addr,
           reinterpret_cast<intptr_t>(instr));
    UNIMPLEMENTED();
  }
}


void Simulator::WriteH(int32_t addr, int16_t value, Instruction* instr) {
  if ((addr & 1) == 0) {
    int16_t* ptr = reinterpret_cast<int16_t*>(addr);
    *ptr = value;
  } else {
    PrintF("Unaligned halfword write at 0x%08x, pc=0x%08" V8PRIxPTR "\n",
           addr,
           reinterpret_cast<intptr_t>(instr));
    UNIMPLEMENTED();
  }
}


uint8_t Simulator::ReadBU(int32_t addr) {
  uint8_t* ptr = reinterpret_cast<uint8_t*>(addr);
  return *ptr;
}


int8_t Simulator::ReadB(int32_t addr) {
  int8_t* ptr = reinterpret_cast<int8_t*>(addr);
  return *ptr;
}


void Simulator::WriteBU(int32_t addr, uint8_t value) {
  uint8_t* ptr = reinterpret_cast<uint8_t*>(addr);
  *ptr = value;
}


void Simulator::WriteB(int32_t addr, int8_t value) {
  int8_t* ptr = reinterpret_cast<int8_t*>(addr);
  *ptr = value;
}


int32_t* Simulator::ReadDW(int32_t addr) {
  if ((addr & 3) == 0) {
    int32_t* ptr = reinterpret_cast<int32_t*>(addr);
    return ptr;
  } else {
    PrintF("Unaligned read at 0x%08x\n", addr);
    UNIMPLEMENTED();
    return 0;
  }
}


void Simulator::WriteDW(int32_t addr, int32_t value1, int32_t value2) {
  if ((addr & 3) == 0) {
    int32_t* ptr = reinterpret_cast<int32_t*>(addr);
    *ptr++ = value1;
    *ptr = value2;
  } else {
    PrintF("Unaligned write at 0x%08x\n", addr);
    UNIMPLEMENTED();
  }
}


// Returns the limit of the stack area to enable checking for stack overflows.
uintptr_t Simulator::StackLimit() const {
  // Leave a safety margin of 1024 bytes to prevent overrunning the stack when
  // pushing values.
  return reinterpret_cast<uintptr_t>(stack_) + 1024;
}


// Calls into the V8 runtime are based on this very simple interface.
// Note: To be able to return two values from some calls the code in runtime.cc
// uses the ObjectPair which is essentially two 32-bit values stuffed into a
// 64-bit value. With the code below we assume that all runtime calls return
// 64 bits of result. If they don't, the r1 result register contains a bogus
// value, which is fine because it is caller-saved.
typedef int64_t (*SimulatorRuntimeCall)(int32_t arg0,
                                        int32_t arg1,
                                        int32_t arg2,
                                        int32_t arg3,
                                        int32_t arg4,
                                        int32_t arg5);
typedef double (*SimulatorRuntimeCompareCall)(double arg0, double arg1);
typedef double (*SimulatorRuntimeFPCall)(double arg0);
typedef double (*SimulatorRuntimeFPFPCall)(double arg0, double arg1);
typedef double (*SimulatorRuntimeFPIntCall)(double arg0, int32_t arg1);

// This signature supports direct call in to API function native callback
// (refer to InvocationCallback in v8.h).
typedef v8::Handle<v8::Value> (*SimulatorRuntimeDirectApiCall)(int32_t arg0);

// This signature supports direct call to accessor getter callback.
typedef v8::Handle<v8::Value> (*SimulatorRuntimeDirectGetterCall)(int32_t arg0,
                               int32_t arg1);

// Software interrupt instructions are used by the simulator to call into the
// C-based V8 runtime.
void Simulator::SoftwareInterrupt(Instruction* instr, int signal) {
  switch (signal) {
    case kCallRtRedirected: {
      // Check if stack is aligned. Error if not aligned is reported below to
      // include information on the function called.
      bool stack_aligned =
          (get_register(sp)
           & (::v8::internal::FLAG_sim_stack_alignment - 1)) == 0;
      Redirection* redirection = Redirection::FromSwiInstruction(instr);
      int32_t arg0 = get_register(r4);
      int32_t arg1 = get_register(r5);
      int32_t arg2 = get_register(r6);
      int32_t arg3 = get_register(r7);
      int32_t* stack_pointer = reinterpret_cast<int32_t*>(get_register(sp));
      int32_t arg4 = stack_pointer[0];
      int32_t arg5 = stack_pointer[1];
      bool fp_call =
         (redirection->type() == ExternalReference::BUILTIN_FP_FP_CALL) ||
         (redirection->type() == ExternalReference::BUILTIN_COMPARE_CALL) ||
         (redirection->type() == ExternalReference::BUILTIN_FP_CALL) ||
         (redirection->type() == ExternalReference::BUILTIN_FP_INT_CALL);
      // This is dodgy but it works because the C entry stubs are never moved.
      // See comment in codegen-sh4.cc and bug 1242173.
      int32_t saved_pr = get_sregister(pr);
      intptr_t external =
          reinterpret_cast<intptr_t>(redirection->external_function());
      if (fp_call) {
        // Trace the execution
        if (::v8::internal::FLAG_trace_sim || !stack_aligned) {
          SimulatorRuntimeFPCall target =
              reinterpret_cast<SimulatorRuntimeFPCall>(external);
          switch (redirection->type()) {
          case ExternalReference::BUILTIN_FP_FP_CALL:
          case ExternalReference::BUILTIN_COMPARE_CALL:
            PrintF("Call to host function (builtin fp-fp) at %p with args %f, %f",
                FUNCTION_ADDR(target), get_dregister(dr4), get_dregister(dr6));
            break;
          case ExternalReference::BUILTIN_FP_CALL:
            PrintF("Call to host function (builtin fp) at %p with arg %f",
                FUNCTION_ADDR(target), get_dregister(dr4));
            break;
          case ExternalReference::BUILTIN_FP_INT_CALL:
            PrintF("Call to host function at (builtin fp-int) %p with args %f, %d",
                FUNCTION_ADDR(target), get_dregister(dr4), get_register(r4));
            break;
          default:
            UNREACHABLE();
            break;
          }
          if (!stack_aligned) {
            PrintF(" with unaligned stack %08x\n", get_register(sp));
          }
          PrintF("\n");
        }
        CHECK(stack_aligned);
        if (redirection->type() != ExternalReference::BUILTIN_COMPARE_CALL) {
          double result = -1;
          switch(redirection->type()) {
          case ExternalReference::BUILTIN_FP_FP_CALL: {
            SimulatorRuntimeFPFPCall target =
                reinterpret_cast<SimulatorRuntimeFPFPCall>(external);
            result = target(get_dregister(dr4), get_dregister(dr6));
            }
            break;
          case ExternalReference::BUILTIN_FP_CALL: {
            SimulatorRuntimeFPCall target =
                reinterpret_cast<SimulatorRuntimeFPCall>(external);
            result = target(get_dregister(dr4));
            }
            break;
          case ExternalReference::BUILTIN_FP_INT_CALL: {
            SimulatorRuntimeFPIntCall target =
                reinterpret_cast<SimulatorRuntimeFPIntCall>(external);
            result = target(get_dregister(dr4), get_register(r4));
            }
            break;
          default:
            break;
          }
          if (::v8::internal::FLAG_trace_sim) {
            PrintF("Returned %f\n", result);
          }
          set_dregister(dr0, result);
        } else {
          SimulatorRuntimeFPFPCall target =
              reinterpret_cast<SimulatorRuntimeFPFPCall>(external);
          int64_t result = target(get_dregister(dr4), get_dregister(dr6));
          int32_t lo_res = static_cast<int32_t>(result);
          int32_t hi_res = static_cast<int32_t>(result >> 32);
          if (::v8::internal::FLAG_trace_sim) {
            PrintF("Returned %08x\n", lo_res);
          }
          set_register(r0, lo_res);
          set_register(r1, hi_res);
        }
      } else if (redirection->type() == ExternalReference::DIRECT_API_CALL) {
        SimulatorRuntimeDirectApiCall target =
            reinterpret_cast<SimulatorRuntimeDirectApiCall>(external);
        if (::v8::internal::FLAG_trace_sim || !stack_aligned) {
          PrintF("Call to host function (direct api) at %p args %08x",
              FUNCTION_ADDR(target), arg0);
          if (!stack_aligned) {
            PrintF(" with unaligned stack %08x\n", get_register(sp));
          }
          PrintF("\n");
        }
        CHECK(stack_aligned);
        v8::Handle<v8::Value> result = target(arg0);
        if (::v8::internal::FLAG_trace_sim) {
          PrintF("Returned %p\n", reinterpret_cast<void *>(*result));
        }
        set_register(r0, (int32_t) *result);
      } else if (redirection->type() == ExternalReference::DIRECT_GETTER_CALL) {
        SimulatorRuntimeDirectGetterCall target =
            reinterpret_cast<SimulatorRuntimeDirectGetterCall>(external);
        if (::v8::internal::FLAG_trace_sim || !stack_aligned) {
          PrintF("Call to host function (getter) at %p args %08x %08x",
              FUNCTION_ADDR(target), arg0, arg1);
          if (!stack_aligned) {
            PrintF(" with unaligned stack %08x\n", get_register(sp));
          }
          PrintF("\n");
        }
        CHECK(stack_aligned);
        v8::Handle<v8::Value> result = target(arg0, arg1);
        if (::v8::internal::FLAG_trace_sim) {
          PrintF("Returned %p\n", reinterpret_cast<void *>(*result));
        }
        set_register(r0, (int32_t) *result);
      } else {
        // builtin call.
        ASSERT(redirection->type() == ExternalReference::BUILTIN_CALL);
        SimulatorRuntimeCall target =
            reinterpret_cast<SimulatorRuntimeCall>(external);
        if (::v8::internal::FLAG_trace_sim || !stack_aligned) {
          PrintF(
              "Call to host function (builtin) at %p "
              "args %08x, %08x, %08x, %08x, %08x, %08x",
              FUNCTION_ADDR(target),
              arg0,
              arg1,
              arg2,
              arg3,
              arg4,
              arg5);
          if (!stack_aligned) {
            PrintF(" with unaligned stack %08x\n", get_register(sp));
          }
          PrintF("\n");
        }
        CHECK(stack_aligned);
        int64_t result = target(arg0, arg1, arg2, arg3, arg4, arg5);
        int32_t lo_res = static_cast<int32_t>(result);
        int32_t hi_res = static_cast<int32_t>(result >> 32);
        if (::v8::internal::FLAG_trace_sim) {
          PrintF("Returned %08x\n", lo_res);
        }
        set_register(r0, lo_res);
        set_register(r1, hi_res);
      }
      set_sregister(pr, saved_pr);
      set_pc(get_sregister(pr));
      break;
    }
    case kIllegalInstruction:
    case kUnsupportedInstruction:
    case kDelaySlot: {
        // Disassemble the current instruction
        disasm::NameConverter converter;
        disasm::Disassembler dasm(converter);
        v8::internal::EmbeddedVector<char, 256> buffer;
        dasm.InstructionDecode(buffer,
                               reinterpret_cast<byte*>(get_pc()));

        // Print the signal state
        switch (signal) {
        case kIllegalInstruction:
            PrintF("Signal received 'illegal instruction'\n");
            break;
        case kUnsupportedInstruction:
            PrintF("Signal received 'unsupported instruction'\n");
            break;
        case kDelaySlot:
            PrintF("Signal received 'instruction forbidden in delay slot'\n");
            break;
        }

        // Print the current processor state
        PrintF(" PC: 0x%08x\n SP: 0x%08x\n PR: 0x%08x\n", get_pc(), get_register(sp), get_sregister(pr));
        for (int i = 0; i < num_registers; i++)
            PrintF(" R%01d: 0x%08x %10d\n", i, get_register(i), get_register(i));

        // launch the debugger
        Sh4Debugger dbg(this);
        dbg.Debug();

        // Abort
        v8::internal::OS::DebugBreak();
        break;
    }
    case kBreakpoint: {
        Sh4Debugger dbg(this);
        dbg.Debug();
        break;
    }
    case kStoppoint: {
        Sh4Debugger dbg(this);
        dbg.Stop(instr);
        break;
    }
    default:
      UNREACHABLE();
      break;
  }
}

// Exceptions
#define RAISE_EXCEPTION() SoftwareInterrupt(instr, kIllegalInstruction)
#define RAISE_UNSUPPORTED_INSTR() SoftwareInterrupt(instr, kUnsupportedInstruction)
#define RAISE_EXCEPTION_IF_IN_DELAY_SLOT() if (in_delay_slot_) SoftwareInterrupt(instr, kDelaySlot)

// Delay slots
#define Delay_Slot(old_pc)  InstructionDecodeDelaySlot(reinterpret_cast<Instruction*>(old_pc))

// R/W in memory
#define WLAT(addr, value)   WriteW(addr, value, instr)
#define WWAT(addr, value)   WriteH(addr, value, instr)
#define WBAT(addr, value)   WriteB(addr, value)
#define RLAT(addr)          ReadW(addr, instr)
#define RSLAT(addr)         ReadW(addr, instr)
#define RSWAT(addr)         ReadH(addr, instr)
#define RBAT(addr)          ReadBU(addr)
#define RSBAT(addr)         ReadB(addr)

// Get/Set status flags
#define SET_SR_T(value)     set_t_flag(value)
#define SET_SR_S(value)     set_s_flag(value)
#define SET_SR_Q(value)     set_q_flag(value)
#define SET_SR_M(value)     set_m_flag(value)

// FPSCR handling
#define FPSCR_MASK_FR (1 << 21)
#define FPSCR_MASK_SZ (1 << 20)
#define FPSCR_MASK_PR (1 << 19)

#define FPSCR_FR  ((get_sregister(fpscr) & FPSCR_MASK_FR) != 0)
#define FPSCR_SZ  ((get_sregister(fpscr) & FPSCR_MASK_SZ) != 0)
#define FPSCR_PR  ((get_sregister(fpscr) & FPSCR_MASK_PR) != 0)

// Sign extension
#define SEXT(value)         (((value &  0xff) ^ (~0x7f))+0x80)
#define SEXT12(value)       (((value & 0xfff) ^ 0x800) - 0x800)
#define SEXTW(value)        ((int) ((short) value))
#define sbit                ((unsigned int) 1 << 31)


// Executes the current instruction.
void Simulator::InstructionDecode(Instruction* instr) {
  if (v8::internal::FLAG_check_icache) {
    CheckICache(isolate_->simulator_i_cache(), instr);
  }
  pc_modified_ = false;
  if (::v8::internal::FLAG_trace_sim) {
    disasm::NameConverter converter;
    disasm::Disassembler dasm(converter);
    // use a reasonably large buffer
    v8::internal::EmbeddedVector<char, 256> buffer;
    dasm.InstructionDecode(buffer,
                           reinterpret_cast<byte*>(instr));
    PrintF("  0x%08x  %s\n", reinterpret_cast<intptr_t>(instr), buffer.start());
  }

  uint16_t iword = instr->InstructionBits();

  if (iword == kCallRtRedirected ||
      iword == kBreakpoint ||
      iword == kStoppoint) {
    SoftwareInterrupt(instr, iword);
  } else {
#include "sh4/jump-table-sh4.h"
#include "sh4/autogen-simulator-sh4.cc"
  }

  if (!pc_modified_) {
    set_pc(reinterpret_cast<int32_t>(instr) + Instruction::kInstrSize);
  }
}


// Execute a delay slot instruction
void Simulator::InstructionDecodeDelaySlot(Instruction* instr) {
  // TODO(ivoire): factorize with Simulator::InstructionDecode
  in_delay_slot_ = true;
  if (v8::internal::FLAG_check_icache) {
    CheckICache(isolate_->simulator_i_cache(), instr);
  }
  if (::v8::internal::FLAG_trace_sim) {
    disasm::NameConverter converter;
    disasm::Disassembler dasm(converter);
    // use a reasonably large buffer
    v8::internal::EmbeddedVector<char, 256> buffer;
    dasm.InstructionDecode(buffer,
                           reinterpret_cast<byte*>(instr));
    PrintF("  0x%08x  %s\n", reinterpret_cast<intptr_t>(instr), buffer.start());
  }

  uint32_t iword = instr->InstructionBits();
  if (iword == kCallRtRedirected ||
      iword == kBreakpoint ||
      iword == kStoppoint) {
    SoftwareInterrupt(instr, iword);
  } else {
#include "sh4/jump-table-sh4.h"
#include "sh4/autogen-simulator-sh4.cc"
  }

  in_delay_slot_ = false;
}


void Simulator::Execute() {
  int program_counter = get_pc();

  if (::v8::internal::FLAG_stop_sim_at == 0) {
    // Fast version of the dispatch loop without checking whether the simulator
    // should be stopping at a particular executed instruction.
    while (program_counter != end_sim_pc) {
      // sh4 processors cannot execute at unaligned address
      if (program_counter & 1) {
        PrintF("Trying to execute at unaligned address: pc=0x%08" V8PRIxPTR "\n", program_counter);
        Sh4Debugger dbg(this);
        dbg.Debug();
        break;
      }
      Instruction* instr = reinterpret_cast<Instruction*>(program_counter);
      icount_++;
      InstructionDecode(instr);
      program_counter = get_pc();
    }
  } else {
    // FLAG_stop_sim_at is at the non-default value. Stop in the debugger when
    // we reach the particular instuction count.
    while (program_counter != end_sim_pc) {
      // sh4 processors cannot execute at unaligned address
      if (program_counter & 1) {
        PrintF("Trying to execute at unaligned address: pc=0x%08" V8PRIxPTR "\n", program_counter);
        Sh4Debugger dbg(this);
        dbg.Debug();
        break;
      }

      Instruction* instr = reinterpret_cast<Instruction*>(program_counter);
      icount_++;
      if (icount_ == ::v8::internal::FLAG_stop_sim_at) {
        Sh4Debugger dbg(this);
        dbg.Debug();
      } else {
        InstructionDecode(instr);
      }
      program_counter = get_pc();
    }
  }
}


void Simulator::CallInternal(byte* entry) {
  // Prepare to execute the code at entry
  set_pc(reinterpret_cast<int32_t>(entry));
  // Put down marker for end of simulation. The simulator will stop simulation
  // when the PC reaches this value. By saving the "end simulation" value into
  // the LR the simulation stops when returning to this call point.
  set_sregister(pr, end_sim_pc);

  // Remember the values of callee-saved registers.
  // The code below assumes that r9 is not used as sb (static base) in
  // simulator code and therefore is regarded as a callee-saved register.
  int32_t r8_val = get_register(r8);
  int32_t r9_val = get_register(r9);
  int32_t r10_val = get_register(r10);
  int32_t r11_val = get_register(r11);
  int32_t r12_val = get_register(r12);
  int32_t r13_val = get_register(r13);

  // Set up the callee-saved registers with a known value. To be able to check
  // that they are preserved properly across JS execution.
  int32_t callee_saved_value = icount_;
  set_register(r8, callee_saved_value);
  set_register(r9, callee_saved_value);
  set_register(r10, callee_saved_value);
  set_register(r11, callee_saved_value);
  set_register(r12, callee_saved_value);
  set_register(r13, callee_saved_value);

  // Start the simulation
  Execute();

  // Check that the callee-saved registers have been preserved.
  CHECK_EQ(callee_saved_value, get_register(r8));
  CHECK_EQ(callee_saved_value, get_register(r9));
  CHECK_EQ(callee_saved_value, get_register(r10));
//  CHECK_EQ(callee_saved_value, get_register(r11));
  CHECK_EQ(callee_saved_value, get_register(r12));
  CHECK_EQ(callee_saved_value, get_register(r13));

  // Restore callee-saved registers with the original value.
  set_register(r8, r8_val);
  set_register(r9, r9_val);
  set_register(r10, r10_val);
  set_register(r11, r11_val);
  set_register(r12, r12_val);
  set_register(r13, r13_val);
}


int32_t Simulator::Call(byte* entry, int int_args, int double_args, ...) {
  va_list parameters;
  va_start(parameters, double_args);
  // Set up arguments

  // First four arguments passed in registers.
  ASSERT(int_args >= 4);
  set_register(r4, va_arg(parameters, int32_t));
  set_register(r5, va_arg(parameters, int32_t));
  set_register(r6, va_arg(parameters, int32_t));
  set_register(r7, va_arg(parameters, int32_t));

  // Remaining arguments passed on stack.
  int original_stack = get_register(sp);
  // Compute position of stack on entry to generated code.
  int entry_stack = (original_stack - (int_args - 4) * sizeof(int32_t));
  if (OS::ActivationFrameAlignment() != 0) {
    entry_stack &= -OS::ActivationFrameAlignment();
  }

  // Store remaining arguments on stack, from low to high memory.
  intptr_t* stack_argument = reinterpret_cast<intptr_t*>(entry_stack);
  for (int i = 4; i < int_args; i++) {
    stack_argument[i - 4] = va_arg(parameters, int32_t);
  }

  // Next arguments on double registers
  if(double_args > 0) {
    ASSERT(double_args == 2);
    set_dregister(dr4, va_arg(parameters, double));
    set_dregister(dr6, va_arg(parameters, double));
  }

  va_end(parameters);
  set_register(sp, entry_stack);

  CallInternal(entry);

  // Pop stack passed arguments.
  CHECK_EQ(entry_stack, get_register(sp));
  set_register(sp, original_stack);

  int32_t result = get_register(r0);
  return result;
}


uintptr_t Simulator::PushAddress(uintptr_t address) {
  int new_sp = get_register(sp) - sizeof(uintptr_t);
  uintptr_t* stack_slot = reinterpret_cast<uintptr_t*>(new_sp);
  *stack_slot = address;
  set_register(sp, new_sp);
  return new_sp;
}


uintptr_t Simulator::PopAddress() {
  int current_sp = get_register(sp);
  uintptr_t* stack_slot = reinterpret_cast<uintptr_t*>(current_sp);
  uintptr_t address = *stack_slot;
  set_register(sp, current_sp + sizeof(uintptr_t));
  return address;
}


void Simulator::mul_helper(int sign, unsigned int rm, unsigned int rn) {
  unsigned int RnL, RnH;
  unsigned int RmL, RmH;
  unsigned int temp0, temp1, temp2, temp3;
  unsigned int Res2, Res1, Res0;

  RnL = rn & 0xffff;
  RnH = (rn >> 16) & 0xffff;
  RmL = rm & 0xffff;
  RmH = (rm >> 16) & 0xffff;
  temp0 = RmL * RnL;
  temp1 = RmH * RnL;
  temp2 = RmL * RnH;
  temp3 = RmH * RnH;
  Res2 = 0;
  Res1 = temp1 + temp2;
  if (Res1 < temp1)
    Res2 += 0x00010000;
  temp1 = (Res1 << 16) & 0xffff0000;
  Res0 = temp0 + temp1;
  if (Res0 < temp0)
    Res2 += 1;
  Res2 += ((Res1 >> 16) & 0xffff) + temp3;

  if (sign)
    {
      if (rn & 0x80000000)
	Res2 -= rm;
      if (rm & 0x80000000)
	Res2 -= rn;
    }

  sregs_.mach = Res2;
  sregs_.macl = Res0;
}


void Simulator::div1 (int iRn2, int iRn1) {
  int tmp0;
  char old_q, tmp1;

  old_q = get_q_flag();
  set_q_flag((unsigned char) ((0x80000000 & get_register(iRn1)) != 0));
  set_register(iRn1, get_register(iRn1) << 1);
  set_register(iRn1, get_register(iRn1) | (unsigned int) get_t_flag());

  switch (old_q) {
    case 0:
      switch (get_m_flag()) {
	  case 0:
	    tmp0 = get_register(iRn1);
	    set_register(iRn1, get_register(iRn1) - get_register(iRn2));
        tmp1 = (get_register(iRn1) > tmp0);
        switch (get_q_flag()) {
	    case 0:
	      set_q_flag(tmp1);
	      break;
	    case 1:
	      set_q_flag((unsigned char) (tmp1 == 0));
	      break;
	    }
	    break;
	  case 1:
	    tmp0 = get_register(iRn1);
	    set_register(iRn1, get_register(iRn1) + get_register(iRn2));
	    tmp1 = (get_register(iRn1) < tmp0);
	    switch (get_q_flag()) {
	    case 0:
	      set_q_flag((unsigned char) (tmp1 == 0));
	      break;
	    case 1:
          set_q_flag(tmp1);
	      break;
	    }
	    break;
	  }
      break;
    case 1:
      switch (get_m_flag()) {
	  case 0:
	    tmp0 = get_register(iRn1);
	    set_register(iRn1, get_register(iRn1) + get_register(iRn2));
	    tmp1 = (get_register(iRn1) < tmp0);
	    switch (get_q_flag()) {
	    case 0:
	      set_q_flag(tmp1);
	      break;
	    case 1:
	      set_q_flag((unsigned char) (tmp1 == 0));
	      break;
	    }
	    break;
	  case 1:
	    tmp0 = get_register(iRn1);
	    set_register(iRn1, get_register(iRn1) - get_register(iRn2));
	    tmp1 = (get_register(iRn1) > tmp0);
	    switch (get_q_flag()) {
	    case 0:
	      set_q_flag((unsigned char) (tmp1 == 0));
	      break;
	    case 1:
	      set_q_flag(tmp1);
	      break;
	    }
	    break;
	  }
      break;
    }
  /*T = (Q == M);*/
  set_t_flag(get_q_flag() == get_m_flag());
}


void Simulator::dsub(int n, int m) {
  ASSERT(FPSCR_PR);
  double dn = get_dregister(n);
  double dm = get_dregister(m);

  set_dregister(n, dn - dm);
}


void Simulator::dadd(int n, int m) {
  ASSERT(FPSCR_PR);
  double dn = get_dregister(n);
  double dm = get_dregister(m);

  set_dregister(n, dn + dm);
}


void Simulator::dmul(int n, int m) {
  ASSERT(FPSCR_PR);
  double dn = get_dregister(n);
  double dm = get_dregister(m);

  set_dregister(n, dn * dm);
}


void Simulator::ddiv(int n, int m) {
  ASSERT(FPSCR_PR);
  double dn = get_dregister(n);
  double dm = get_dregister(m);

  set_dregister(n, dn / dm);
}


} }  // namespace v8::internal

#endif  // USE_SIMULATOR

#endif  // V8_TARGET_ARCH_SH4
