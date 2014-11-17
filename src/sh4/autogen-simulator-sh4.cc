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

// This file is a derivation of an automatically generated file from
// the SH4 toolchain.

{
/* REG_xy = [r4, r5, r0, r1].  */
#define REG_xy(R) ((R)==0 ? 4 : (R)==2 ? 5 : (R)==1 ?  0 :  1)
/* REG_yx = [r6, r7, r2, r3].  */
#define REG_yx(R) ((R)==0 ? 6 : (R)==1 ? 7 : (R)==2 ?  2 :  3)
/* DSP_ax = [a0, a1, x0, x1].  */
#define DSP_ax(R) ((R)==0 ? 7 : (R)==2 ? 5 : (R)==1 ?  8 :  9)
/* DSP_ay = [a0, a1, y0, y1].  */
#define DSP_ay(R) ((R)==0 ? 7 : (R)==1 ? 5 : (R)==2 ? 10 : 11)
/* DSP_xy = [x0, x1, y0, y1].  */
#define DSP_xy(R) ((R)==0 ? 8 : (R)==2 ? 9 : (R)==1 ? 10 : 11)
/* DSP_yx = [y0, y1, x0, x1].  */
#define DSP_yx(R) ((R)==0 ? 10 : (R)==1 ? 11 : (R)==2 ? 8 : 9)
  switch (sh_jump_table[iword]) {
  /* stc <CREG_M>,<REG_N> 0000nnnnmmmm0010 */
  case 3:      
    {
      RAISE_UNSUPPORTED_INSTR();
      break;
    }
  /* braf <REG_N> 0000nnnn00100011 */
  case 4:      
    {
      int n = (iword >> 8) & 0xf;
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_pc(get_pc() + 4 + get_register(n));
        Delay_Slot (old_pc + 2);
      }
      break;
    }
  /* bsrf <REG_N> 0000nnnn00000011 */
  case 5:      
    {
      int n = (iword >> 8) & 0xf;
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_sregister(pr, get_pc() + 4);
        set_pc(get_pc() + 4 + get_register(n));
        Delay_Slot (old_pc + 2);
      }
      break;
    }
  /* movca.l R0, @<REG_N> 0000nnnn11000011 */
  case 6:      
    {
      int n = (iword >> 8) & 0xf;
      {
        /* We don't simulate cache, so this insn is identical to mov.  */
        WLAT (get_register(n), get_register(0));
      }
      break;
    }
  /* icbi @<REG_N> 0000nnnn11100011 */
  case 9:      
    {
      // int n = (iword >> 8) & 0xf;
      {
        /* Except for the effect on the cache - which is not simulated -
           this is like a nop.  */
      }
      break;
    }
  /* ocbi @<REG_N> 0000nnnn10010011 */
  case 10:      
    {
      int n = (iword >> 8) & 0xf;
      {
        RSBAT (get_register(n)); /* Take exceptions like byte load, otherwise noop.  */
        /* FIXME: Cache not implemented */
      }
      break;
    }
  /* ocbp @<REG_N> 0000nnnn10100011 */
  case 11:      
    {
      int n = (iword >> 8) & 0xf;
      {
        RSBAT (get_register(n)); /* Take exceptions like byte load, otherwise noop.  */
        /* FIXME: Cache not implemented */
      }
      break;
    }
  /* ocbwb @<REG_N> 0000nnnn10110011 */
  case 12:      
    {
      int n = (iword >> 8) & 0xf;
      {
        RSBAT (get_register(n)); /* Take exceptions like byte load, otherwise noop.  */
        /* FIXME: Cache not implemented */
      }
      break;
    }
  /* pref @<REG_N> 0000nnnn10000011 */
  case 13:      
    {
      // int n = (iword >> 8) & 0xf;
      {
        /* Except for the effect on the cache - which is not simulated -
           this is like a nop.  */
      }
      break;
    }
  /* mov.b <REG_M>,@(R0,<REG_N>) 0000nnnnmmmm0100 */
  case 15:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        WBAT (get_register(n) + get_register(0), get_register(m));
      }
      break;
    }
  /* mov.w <REG_M>,@(R0,<REG_N>) 0000nnnnmmmm0101 */
  case 16:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        WWAT (get_register(0) + get_register(n), get_register(m));
      }
      break;
    }
  /* mov.l <REG_M>,@(R0,<REG_N>) 0000nnnnmmmm0110 */
  case 17:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        WLAT (get_register(0) + get_register(n), get_register(m));
      }
      break;
    }
  /* mul.l <REG_M>,<REG_N> 0000nnnnmmmm0111 */
  case 18:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_sregister(macl, ((int) get_register(n)) * ((int) get_register(m)));
      }
      break;
    }
  /* clrmac 0000000000101000 */
  case 20:      
    {
      {
        set_sregister(mach, 0);
        set_sregister(macl, 0);
      }
      break;
    }
  /* clrs 0000000001001000 */
  case 21:      
    {
      {
        SET_SR_S (0);
      }
      break;
    }
  /* clrt 0000000000001000 */
  case 22:      
    {
      {
        SET_SR_T (0);
      }
      break;
    }
  /* ldtlb 0000000000111000 */
  case 24:      
    {
      RAISE_EXCEPTION();
      break;
    }
  /* sets 0000000001011000 */
  case 27:      
    {
      {
        SET_SR_S (1);
      }
      break;
    }
  /* sett 0000000000011000 */
  case 28:      
    {
      {
        SET_SR_T (1);
      }
      break;
    }
  /* div0u 0000000000011001 */
  case 29:      
    {
      {
        SET_SR_M (0);
        SET_SR_Q (0);
        SET_SR_T (0);
      }
      break;
    }
  /* movt <REG_N> 0000nnnn00101001 */
  case 30:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, get_t_flag());
      }
      break;
    }
  /* nop 0000000000001001 */
  case 32:      
    {
      {
        /* nop */
      }
      break;
    }
  /* stc SGR,<REG_N> 0000nnnn00111010 */
  case 33:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* stc DBR,<REG_N> 0000nnnn11111010 */
  case 34:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* sts <SREG_M>,<REG_N> 0000nnnnssss1010 */
  case 36:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_sregister(m));
      }
      break;
    }
  /* synco 0000000010101011 */
  case 38:      
    {
      {
        /* Except for the effect on the pipeline - which is not simulated -
           this is like a nop.  */
      }
      break;
    }
  /* rte 0000000000101011 */
  case 39:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* rts 0000000000001011 */
  case 40:      
    {
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_pc(get_sregister(pr));
        Delay_Slot (old_pc + 2);
      }
      break;
    }
  /* sleep 0000000000011011 */
  case 43:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* mov.b @(R0,<REG_M>),<REG_N> 0000nnnnmmmm1100 */
  case 44:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RSBAT (get_register(0) + get_register(m)));
      }
      break;
    }
  /* mov.w @(R0,<REG_M>),<REG_N> 0000nnnnmmmm1101 */
  case 45:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RSWAT (get_register(0) + get_register(m)));
      }
      break;
    }
  /* mov.l @(R0,<REG_M>),<REG_N> 0000nnnnmmmm1110 */
  case 46:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RLAT (get_register(0) + get_register(m)));
      }
      break;
    }
  /* mac.l @<REG_M>+,@<REG_N>+ 0000nnnnmmmm1111 */
  case 47:      
    {
#if 0
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        macl (&R0, memory, n, m);
      }
#endif
     UNIMPLEMENTED();
      break;
    }
  /* mov.l <REG_M>,@(<disp>,<REG_N>) 0001nnnnmmmmi4*4 */
  case 48:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      int i = (iword & 0xf) << 2;
      {
        WLAT (i + get_register(n), get_register(m));
      }
      break;
    }
  /* mov.b <REG_M>,@<REG_N> 0010nnnnmmmm0000 */
  case 49:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        WBAT (get_register(n), get_register(m));
      }
      break;
    }
  /* mov.w <REG_M>,@<REG_N> 0010nnnnmmmm0001 */
  case 50:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        WWAT (get_register(n), get_register(m));
      }
      break;
    }
  /* mov.l <REG_M>,@<REG_N> 0010nnnnmmmm0010 */
  case 51:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        WLAT (get_register(n), get_register(m));
      }
      break;
    }
  /* mov.b <REG_M>,@-<REG_N> 0010nnnnmmmm0100 */
  case 52:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        int t = get_register(m);
        set_register(n, get_register(n) - 1);
        WBAT (get_register(n), t);
      }
      break;
    }
  /* mov.w <REG_M>,@-<REG_N> 0010nnnnmmmm0101 */
  case 53:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        int t = get_register(m);
        set_register(n, get_register(n) - 2);
        WWAT (get_register(n), t);
      }
      break;
    }
  /* mov.l <REG_M>,@-<REG_N> 0010nnnnmmmm0110 */
  case 54:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        int t = get_register(m);
        set_register(n, get_register(n) - 4);
        WLAT (get_register(n), t);
      }
      break;
    }
  /* div0s <REG_M>,<REG_N> 0010nnnnmmmm0111 */
  case 55:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        SET_SR_Q ((get_register(n) & sbit) != 0);
        SET_SR_M ((get_register(m) & sbit) != 0);
        SET_SR_T (get_m_flag() != get_q_flag());
      }
      break;
    }
  /* tst <REG_M>,<REG_N> 0010nnnnmmmm1000 */
  case 56:
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        SET_SR_T ((get_register(n) & get_register(m)) == 0);
      }
      break;
    }

  /* and <REG_M>,<REG_N> 0010nnnnmmmm1001 */
  case 57:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(n) & get_register(m));
      }
      break;
    }
  /* xor <REG_M>,<REG_N> 0010nnnnmmmm1010 */
  case 58:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(n) ^ get_register(m));
      }
      break;
    }
  /* or <REG_M>,<REG_N> 0010nnnnmmmm1011 */
  case 59:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(n) | get_register(m));
      }
      break;
    }
  /* cmp/str <REG_M>,<REG_N> 0010nnnnmmmm1100 */
  case 60:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        unsigned int ult = get_register(n) ^ get_register(m);
        SET_SR_T (((ult & 0xff000000) == 0)
                  | ((ult & 0xff0000) == 0)
                  | ((ult & 0xff00) == 0)
                  | ((ult & 0xff) == 0));
      }
      break;
    }
  /* xtrct <REG_M>,<REG_N> 0010nnnnmmmm1101 */
  case 61:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, ((get_register(n) >> 16) & 0xffff)
                | ((get_register(m) << 16) & 0xffff0000));
      }
      break;
    }
  /* mulu <REG_M>,<REG_N> 0010nnnnmmmm1110 */
  case 62:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_sregister(macl, ((unsigned int) (unsigned short) get_register(n))
                          * ((unsigned int) (unsigned short) get_register(m)));
      }
      break;
    }
  /* muls <REG_M>,<REG_N> 0010nnnnmmmm1111 */
  case 63:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_sregister(macl, ((int) (short) get_register(n)) * ((int) (short) get_register(m)));
      }
      break;
    }
  /* cmp/eq <REG_M>,<REG_N> 0011nnnnmmmm0000 */
  case 64:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        SET_SR_T (get_register(n) == get_register(m));
      }
      break;
    }
  /* cmp/hs <REG_M>,<REG_N> 0011nnnnmmmm0010 */
  case 66:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        SET_SR_T ((unsigned int)get_register(n) >= (unsigned int)get_register(m));
      }
      break;
    }
  /* cmp/ge <REG_M>,<REG_N> 0011nnnnmmmm0011 */
  case 67:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        SET_SR_T (get_register(n) >= get_register(m));
      }
      break;
    }
  /* div1 <REG_M>,<REG_N> 0011nnnnmmmm0100 */
  case 68:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        div1 (m, n);
      }
      break;
    }
  /* dmulu.l <REG_M>,<REG_N> 0011nnnnmmmm0101 */
  case 69:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        mul_helper (0/*unsigned*/, get_register(n), get_register(m));
      }
      break;
    }
  /* cmp/hi <REG_M>,<REG_N> 0011nnnnmmmm0110 */
  case 70:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        SET_SR_T ((unsigned int)get_register(n) > (unsigned int)get_register(m));
      }
      break;
    }
  /* cmp/gt <REG_M>,<REG_N> 0011nnnnmmmm0111 */
  case 71:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        SET_SR_T (get_register(n) > get_register(m));
      }
      break;
    }
  /* sub <REG_M>,<REG_N> 0011nnnnmmmm1000 */
  case 72:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(n) - get_register(m));
      }
      break;
    }
  /* subc <REG_M>,<REG_N> 0011nnnnmmmm1010 */
  case 74:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        unsigned int ult = get_register(n) - get_t_flag();
        SET_SR_T (ult > (unsigned int)get_register(n));
        set_register(n, ult - get_register(m));
        SET_SR_T (get_t_flag() || ((unsigned int)get_register(n) > ult));
      }
      break;
    }
  /* subv <REG_M>,<REG_N> 0011nnnnmmmm1011 */
  case 75:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        unsigned int ult = get_register(n) - get_register(m);
        SET_SR_T (((get_register(n) ^ get_register(m)) & (ult ^ get_register(n))) >> 31);
        set_register(n, ult);
      }
      break;
    }
  /* add <REG_M>,<REG_N> 0011nnnnmmmm1100 */
  case 76:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(n) + get_register(m));
      }
      break;
    }
  /* dmuls.l <REG_M>,<REG_N> 0011nnnnmmmm1101 */
  case 77:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        mul_helper (1/*signed*/, get_register(n), get_register(m));
      }
      break;
    }
  /* addc <REG_M>,<REG_N> 0011nnnnmmmm1110 */
  case 78:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        unsigned int ult = get_register(n) + get_t_flag();
        SET_SR_T (ult < (unsigned int)get_register(n));
        set_register(n, ult + get_register(m));
        SET_SR_T (get_t_flag() || ((unsigned int)get_register(n) < ult));
      }
      break;
    }
  /* addv <REG_M>,<REG_N> 0011nnnnmmmm1111 */
  case 79:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        unsigned int ult = get_register(n) + get_register(m);
        SET_SR_T ((~(get_register(n) ^ get_register(m)) & (ult ^ get_register(n))) >> 31);
        set_register(n, ult);
      }
      break;
    }
  /* dt <REG_N> 0100nnnn00010000 */
  case 82:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, get_register(n) - 1);
        SET_SR_T (get_register(n) == 0);
      }
      break;
    }
  /* shal <REG_N> 0100nnnn00100000 */
  case 83:      
    {
      int n = (iword >> 8) & 0xf;
      {
        SET_SR_T (get_register(n) < 0);
        set_register(n, get_register(n) << 1);
      }
      break;
    }
  /* shll <REG_N> 0100nnnn00000000 */
  case 84:      
    {
      int n = (iword >> 8) & 0xf;
      {
        SET_SR_T (get_register(n) < 0);
        set_register(n, get_register(n) << 1);
      }
      break;
    }
  /* cmp/pz <REG_N> 0100nnnn00010001 */
  case 89:      
    {
      int n = (iword >> 8) & 0xf;
      {
        SET_SR_T (get_register(n) >= 0);
      }
      break;
    }
  /* shar <REG_N> 0100nnnn00100001 */
  case 90:      
    {
      int n = (iword >> 8) & 0xf;
      {
        SET_SR_T (get_register(n) & 1);
        set_register(n, get_register(n) >> 1);
      }
      break;
    }
  /* shlr <REG_N> 0100nnnn00000001 */
  case 91:      
    {
      int n = (iword >> 8) & 0xf;
      {
        SET_SR_T (get_register(n) & 1);
        set_register(n, (unsigned int)get_register(n) >> 1);
      }
      break;
    }
  /* stc.l SGR,@-<REG_N> 0100nnnn00110010 */
  case 92:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* stc.l DBR,@-<REG_N> 0100nnnn11110010 */
  case 93:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* sts.l <SREG_M>,@-<REG_N> 0100nnnnssss0010 */
  case 94:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(n) - 4);
        WLAT (get_register(n), get_sregister(m));
      }
      break;
    }
  /* stc.l <CREG_M>,@-<REG_N> 0100nnnnmmmm0011 */
  case 95:      
    {
      /* We do not support this instruction that can't be found in JIT code */
      RAISE_EXCEPTION();
      break;
    }
  /* rotcl <REG_N> 0100nnnn00100100 */
  case 100:      
    {
      int n = (iword >> 8) & 0xf;
      {
        unsigned int ult = get_register(n) < 0;
        set_register(n, (get_register(n) << 1) | get_t_flag());
        SET_SR_T (ult);
      }
      break;
    }
  /* rotl <REG_N> 0100nnnn00000100 */
  case 101:      
    {
      int n = (iword >> 8) & 0xf;
      {
        SET_SR_T (get_register(n) < 0);
        set_register(n, get_register(n) << 1);
        set_register(n, get_register(n) | get_t_flag());
      }
      break;
    }
  /* cmp/pl <REG_N> 0100nnnn00010101 */
  case 107:      
    {
      int n = (iword >> 8) & 0xf;
      {
        SET_SR_T (get_register(n) > 0);
      }
      break;
    }
  /* rotcr <REG_N> 0100nnnn00100101 */
  case 108:      
    {
      int n = (iword >> 8) & 0xf;
      {
        unsigned int ult = get_register(n) & 1;
        set_register(n, ((unsigned int)get_register(n) >> 1) | (get_t_flag() << 31));
        SET_SR_T (ult);
      }
      break;
    }
  /* rotr <REG_N> 0100nnnn00000101 */
  case 109:      
    {
      int n = (iword >> 8) & 0xf;
      {
        SET_SR_T (get_register(n) & 1);
        set_register(n, (unsigned int)get_register(n) >> 1);
        set_register(n, get_register(n) | (get_t_flag() << 31));
      }
      break;
    }
  /* ldc.l @<REG_N>+,DBR 0100nnnn11110110 */
  case 110:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* ldc.l @<REG_N>+,SGR 0100nnnn00110110 */
  case 111:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* lds.l @<REG_N>+,<SREG_M> 0100nnnnssss0110 */
  case 112:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_sregister(m, RLAT (get_register(n)));
        set_register(n, get_register(n) + 4);
      }
      break;
    }
  /* lds.l @<REG_N>+,FPSCR 0100nnnn01100110 */
  case 113:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_sregister(fpscr, RLAT (get_register(n)));
        set_register(n, get_register(n) + 4);
      }
      break;
    }
  /* ldc.l @<REG_N>+,<CREG_M> 0100nnnnmmmm0111 */
  case 114:      
    {
      /* We do not support this instruction that can't be found in JIT code */
      RAISE_EXCEPTION();
      break;
    }
  /* ldc.l @<REG_N>+,SR 0100nnnn00000111 */
  case 115:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* shll2 <REG_N> 0100nnnn00001000 */
  case 117:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, get_register(n) << 2);
      }
      break;
    }
  /* shll8 <REG_N> 0100nnnn00011000 */
  case 118:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, get_register(n) << 8);
      }
      break;
    }
  /* shll16 <REG_N> 0100nnnn00101000 */
  case 119:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, get_register(n) << 16);
      }
      break;
    }
  /* shlr2 <REG_N> 0100nnnn00001001 */
  case 122:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, (unsigned int)get_register(n) >> 2);
      }
      break;
    }
  /* shlr8 <REG_N> 0100nnnn00011001 */
  case 123:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, (unsigned int)get_register(n) >> 8);
      }
      break;
    }
  /* shlr16 <REG_N> 0100nnnn00101001 */
  case 124:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, (unsigned int)get_register(n) >> 16);
      }
      break;
    }
  /* ldc <REG_N>,DBR 0100nnnn11111010 */
  case 125:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* ldc <REG_N>,SGR 0100nnnn00111010 */
  case 126:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* lds <REG_N>,<SREG_M> 0100nnnnssss1010 */
  case 128:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_sregister(m, get_register(n));
      }
      break;
    }
  /* lds <REG_N>,FPSCR 0100nnnn01101010 */
  case 129:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_sregister(fpscr, get_register(n));
      }
      break;
    }
  /* jmp @<REG_N> 0100nnnn00101011 */
  case 130:      
    {
      int n = (iword >> 8) & 0xf;
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_pc(get_register(n));
        Delay_Slot (old_pc + 2);
      }
      break;
    }
  /* jsr @<REG_N> 0100nnnn00001011 */
  case 131:      
    {
      int n = (iword >> 8) & 0xf;
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_sregister(pr, old_pc + 4);
        set_pc(get_register(n));
        Delay_Slot(old_pc + 2);
      }
      break;
    }
  /* mov.b @-<REG_N>,R0 0100nnnn11001011 */
  case 133:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, get_register(n) - 1);
        set_register(0, RSBAT (get_register(n)));
      }
      break;
    }
  /* mov.b R0,@<REG_N>+ 0100nnnn10001011 */
  case 134:      
    {
      int n = (iword >> 8) & 0xf;
      {
        WBAT (get_register(n), get_register(0));
        set_register(n, get_register(n) + 1);
      }
      break;
    }
  /* mov.l @-<REG_N>,R0 0100nnnn11101011 */
  case 135:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, get_register(n) - 4);
        set_register(0, RLAT (get_register(n)));
      }
      break;
    }
  /* mov.l R0,@<REG_N>+ 0100nnnn10101011 */
  case 136:      
    {
      int n = (iword >> 8) & 0xf;
      {
        WLAT (get_register(n), get_register(0));
        set_register(n, get_register(n) + 4);
      }
      break;
    }
  /* mov.w @-<REG_N>,R0 0100nnnn11011011 */
  case 137:      
    {
      int n = (iword >> 8) & 0xf;
      {
        set_register(n, get_register(n) - 2);
        set_register(0, RSWAT (get_register(n)));
      }
      break;
    }
  /* mov.w R0,@<REG_N>+ 0100nnnn10011011 */
  case 138:      
    {
      int n = (iword >> 8) & 0xf;
      {
        WWAT (get_register(n), get_register(0));
        set_register(n, get_register(n) + 2);
      }
      break;
    }
  /* tas.b @<REG_N> 0100nnnn00011011 */
  case 139:      
    {
      int n = (iword >> 8) & 0xf;
      {
        unsigned int ult = RBAT (get_register(n));
        SET_SR_T (ult == 0);
        WBAT (get_register(n), ult | 0x80);
      }
      break;
    }
  /* shad <REG_M>,<REG_N> 0100nnnnmmmm1100 */
  case 140:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, (get_register(m) < 0) ?
                            (get_register(m)&0x1f ?
                                get_register(n) >> ((-get_register(m))&0x1f) :
                                get_register(n) >> 31) :
                            (get_register(n) << (get_register(m) & 0x1f)));
      }
      break;
    }
  /* shld <REG_M>,<REG_N> 0100nnnnmmmm1101 */
  case 141:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, (get_register(m) < 0) ?
                            (get_register(m) & 0x1f ?
                                (unsigned int)get_register(n) >> ((-get_register(m)) & 0x1f) :
                                0) :
                            (get_register(n) << (get_register(m) & 0x1f)));
      }
      break;
    }
  /* ldc <REG_N>,<CREG_M> 0100nnnnmmmm1110 */
  case 142:      
    {
      /* We do not support this instruction that can't be found in JIT code */
      RAISE_EXCEPTION();
      break;
    }
  /* ldc <REG_N>,SR 0100nnnn00001110 */
  case 143:      
    {
      RAISE_EXCEPTION(); /* user mode */
      break;
    }
  /* mac.w @<REG_M>+,@<REG_N>+ 0100nnnnmmmm1111 */
  case 145:      
    {
      /* we don not implement this instruction for the moment */
      UNIMPLEMENTED();
      break;
    }
  /* mov.l @(<disp>,<REG_M>),<REG_N> 0101nnnnmmmmi4*4 */
  case 146:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      int i = (iword & 0xf) << 2;
      {
        set_register(n, RLAT (i + get_register(m)));
      }
      break;
    }
  /* mov.b @<REG_M>,<REG_N> 0110nnnnmmmm0000 */
  case 147:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RSBAT (get_register(m)));
      }
      break;
    }
  /* mov.w @<REG_M>,<REG_N> 0110nnnnmmmm0001 */
  case 148:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RSWAT (get_register(m)));
      }
      break;
    }
  /* mov.l @<REG_M>,<REG_N> 0110nnnnmmmm0010 */
  case 149:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RLAT (get_register(m)));
      }
      break;
    }
  /* mov <REG_M>,<REG_N> 0110nnnnmmmm0011 */
  case 150:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(m));
      }
      break;
    }
  /* mov.b @<REG_M>+,<REG_N> 0110nnnnmmmm0100 */
  case 151:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RSBAT (get_register(m)));
        set_register(m, get_register(m) + 1);
      }
      break;
    }
  /* mov.w @<REG_M>+,<REG_N> 0110nnnnmmmm0101 */
  case 152:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RSWAT (get_register(m)));
        set_register(m, get_register(m) + 2);
      }
      break;
    }
  /* mov.l @<REG_M>+,<REG_N> 0110nnnnmmmm0110 */
  case 153:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, RLAT (get_register(m)));
        set_register(m, get_register(m) + 4);
      }
      break;
    }
  /* not <REG_M>,<REG_N> 0110nnnnmmmm0111 */
  case 154:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, ~get_register(m));
      }
      break;
    }
  /* swap.b <REG_M>,<REG_N> 0110nnnnmmmm1000 */
  case 155:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, ((get_register(m) & 0xffff0000))
                | ((get_register(m) << 8) & 0xff00)
                | ((get_register(m) >> 8) & 0x00ff));
      }
      break;
    }
  /* swap.w <REG_M>,<REG_N> 0110nnnnmmmm1001 */
  case 156:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, (((get_register(m) << 16) & 0xffff0000))
                | ((get_register(m) >> 16) & 0x00ffff));
      }
      break;
    }
  /* negc <REG_M>,<REG_N> 0110nnnnmmmm1010 */
  case 157:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        unsigned int ult = -get_t_flag();
        SET_SR_T (ult > 0);
        set_register(n, ult - get_register(m));
        SET_SR_T (get_t_flag() || ((unsigned int)get_register(n) > ult));
      }
      break;
    }
  /* neg <REG_M>,<REG_N> 0110nnnnmmmm1011 */
  case 158:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, - get_register(m));
      }
      break;
    }
  /* extu.b <REG_M>,<REG_N> 0110nnnnmmmm1100 */
  case 159:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(m) & 0xff);
      }
      break;
    }
  /* extu.w <REG_M>,<REG_N> 0110nnnnmmmm1101 */
  case 160:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, get_register(m) & 0xffff);
      }
      break;
    }
  /* exts.b <REG_M>,<REG_N> 0110nnnnmmmm1110 */
  case 161:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, SEXT (get_register(m)));
      }
      break;
    }
  /* exts.w <REG_M>,<REG_N> 0110nnnnmmmm1111 */
  case 162:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        set_register(n, SEXTW (get_register(m)));
      }
      break;
    }
  /* add #<imm>,<REG_N> 0111nnnni8*1.... */
  case 163:      
    {
      int n = (iword >> 8) & 0xf;
      int i = (iword & 0xff);
      {
        set_register(n, get_register(n) + SEXT (i));
        if (i == 0) {
          break;
        }
      }
      break;
    }
  /* bf <bdisp8> 10001011i8p1.... */
  case 164:      
    {
      int i = (iword & 0xff);
      {
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        if (!get_t_flag()) {
          set_pc(get_pc() + 4 + (SEXT (i) * 2));
        }
      }
      break;
    }
  /* bf.s <bdisp8> 10001111i8p1.... */
  case 165:      
    {
      int i = (iword & 0xff);
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        if (!get_t_flag()) {
          set_pc(get_pc() + 4 + (SEXT (i) * 2));
          Delay_Slot (old_pc + 2);
        }
      }
      break;
    }
  /* bt <bdisp8> 10001001i8p1.... */
  case 166:      
    {
      int i = (iword & 0xff);
      {
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        if (get_t_flag()) {
          set_pc(get_pc() + 4 + (SEXT (i) * 2));
        }
      }
      break;
    }
  /* bt.s <bdisp8> 10001101i8p1.... */
  case 167:      
    {
      int i = (iword & 0xff);
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        if (get_t_flag()) {
          set_pc(get_pc() + 4 + (SEXT (i) * 2));
          Delay_Slot (old_pc + 2);
        }
      }
      break;
    }
  /* cmp/eq #<imm>,R0 10001000i8*1.... */
  case 168:      
    {
      int i = (iword & 0xff);
      {
        SET_SR_T (get_register(0) == SEXT (i));
      }
      break;
    }
  /* mov.b @(<disp>,<REG_M>),R0 10000100mmmmi4*1 */
  case 176:      
    {
      int m = (iword >> 4) & 0xf;
      int i = (iword & 0xf);
      {
        set_register(0, RSBAT (i + get_register(m)));
      }
      break;
    }
  /* mov.b R0,@(<disp>,<REG_M>) 10000000mmmmi4*1 */
  case 177:      
    {
      int m = (iword >> 4) & 0xf;
      int i = (iword & 0xf);
      {
        WBAT (i + get_register(m), get_register(0));
      }
      break;
    }
  /* mov.w @(<disp>,<REG_M>),R0 10000101mmmmi4*2 */
  case 178:      
    {
      int m = (iword >> 4) & 0xf;
      int i = (iword & 0xf) << 1;
      {
        set_register(0, RSWAT (i + get_register(m)));
      }
      break;
    }
  /* mov.w R0,@(<disp>,<REG_M>) 10000001mmmmi4*2 */
  case 179:      
    {
      int m = (iword >> 4) & 0xf;
      int i = (iword & 0xf) << 1;
      {
        WWAT (i + get_register(m), get_register(0));
      }
      break;
    }
  /* mov.w @(<disp>,PC),<REG_N> 1001nnnni8p2.... */
  case 180:      
    {
      int n = (iword >> 8) & 0xf;
      int i = (iword & 0xff) << 1;
      {
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_register(n, RSWAT (get_pc() + 4 + i));
      }
      break;
    }
  /* bra <bdisp12> 1010i12......... */
  case 181:      
    {
      int i = (iword & 0xfff);
      i = (i ^ (1 << 11)) - (1 << 11);
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_pc(old_pc + 4 + (SEXT12 (i) * 2));
        Delay_Slot (old_pc + 2);
      }
      break;
    }
  /* bsr <bdisp12> 1011i12......... */
  case 182:      
    {
      int i = (iword & 0xfff);
      i = (i ^ (1 << 11)) - (1 << 11);
      {
        int32_t old_pc = get_pc();
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_sregister(pr, old_pc + 4);
        set_pc(old_pc + 4 + (SEXT12 (i) * 2));
        Delay_Slot(old_pc + 2);
      }
      break;
    }
  /* and #<imm>,R0 11001001i8*1.... */
  case 183:      
    {
      int i = (iword & 0xff);
      {
        set_register(0, get_register(0) & i);
      }
      break;
    }
#if 0
  /* and.b #<imm>,@(R0,GBR) 11001101i8*1.... */
  case 184:      
    {
      int i = (iword & 0xff);
      {
        WBAT (GBR + get_register(0), RBAT (GBR + get_register(0)) & i);
      }
      break;
    }
  /* mov.b @(<disp>,GBR),R0 11000100i8*1.... */
  case 185:      
    {
      int i = (iword & 0xff);
      {
        set_register(0, RSBAT (i + GBR));
      }
      break;
    }
  /* mov.b R0,@(<disp>,GBR) 11000000i8*1.... */
  case 186:      
    {
      int i = (iword & 0xff);
      {
        WBAT (i + GBR, get_register(0));
      }
      break;
    }
  /* mov.l @(<disp>,GBR),R0 11000110i8*4.... */
  case 187:      
    {
      int i = (iword & 0xff) << 2;
      {
        set_register(0, RLAT (i + GBR));
      }
      break;
    }
  /* mov.l R0,@(<disp>,GBR) 11000010i8*4.... */
  case 188:      
    {
      int i = (iword & 0xff) << 2;
      {
        WLAT (i + GBR, get_register(0));
      }
      break;
    }
  /* mov.w @(<disp>,GBR),R0 11000101i8*2.... */
  case 189:      
    {
      int i = (iword & 0xff) << 1;
      {
        set_register(0, RSWAT (i + GBR));
      }
      break;
    }
  /* mov.w R0,@(<disp>,GBR) 11000001i8*2.... */
  case 190:      
    {
      int i = (iword & 0xff) << 1;
      {
        WWAT (i + GBR, get_register(0));
      }
      break;
    }
#endif
  /* mova @(<disp>,PC),R0 11000111i8p4.... */
  case 191:      
    {
      int i = (iword & 0xff) << 2;
      {
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_register(0, (i + 4 + get_pc()) & ~0x3);
      }
      break;
    }
  /* or #<imm>,R0 11001011i8*1.... */
  case 192:      
    {
      int i = (iword & 0xff);
      {
        set_register(0, get_register(0) | i);
      }
      break;
    }
  /* or.b #<imm>,@(R0,GBR) 11001111i8*1.... */
  case 193:      
    {
      /* We do not support this instruction that can't be found in JIT code */
      RAISE_UNSUPPORTED_INSTR();
      break;
    }
  /* trapa #<imm> 11000011i8*1.... */
  case 194:      
    {
      /* We cannot encounter this instruction */
      RAISE_EXCEPTION();
      break;
    }
  /* tst #<imm>,R0 11001000i8*1.... */
  case 195:      
    {
      int i = (iword & 0xff);
      {
        SET_SR_T ((get_register(0) & i) == 0);
      }
      break;
    }
  /* tst.b #<imm>,@(R0,GBR) 11001100i8*1.... */
  case 196:      
    {
      /* We do not support this instruction that can't be found in JIT code */
      RAISE_EXCEPTION();
      break;
    }
  /* xor #<imm>,R0 11001010i8*1.... */
  case 197:      
    {
      int i = (iword & 0xff);
      {
        set_register(0, get_register(0) ^ i);
      }
      break;
    }
  /* xor.b #<imm>,@(R0,GBR) 11001110i8*1.... */
  case 198:      
    {
      /* We do not support this instruction that can't be found in JIT code */
      RAISE_EXCEPTION();
      break;
    }
  /* mov.l @(<disp>,PC),<REG_N> 1101nnnni8p4.... */
  case 199:      
    {
      int n = (iword >> 8) & 0xf;
      int i = (iword & 0xff) << 2;
      {
        RAISE_EXCEPTION_IF_IN_DELAY_SLOT ();
        set_register(n, RLAT ((get_pc() & ~3) + 4 + i));
      }
      break;
    }
  /* mov #<imm>,<REG_N> 1110nnnni8*1.... */
  case 200:      
    {
      int n = (iword >> 8) & 0xf;
      int i = (iword & 0xff);
      {
        set_register(n, SEXT (i));
      }
      break;
    }
  /* fadd <FREG_M>,<FREG_N> 1111nnnnmmmm0000 */
  case 201:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if(FPSCR_PR)
          dadd(n, m);
        else
          UNIMPLEMENTED();
      }
      break;
    }
  /* fsub <FREG_M>,<FREG_N> 1111nnnnmmmm0001 */
  case 202:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if(FPSCR_PR)
          dsub(n, m);
        else
          UNIMPLEMENTED();
      }
      break;
    }
  /* fmul <FREG_M>,<FREG_N> 1111nnnnmmmm0010 */
  case 203:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if(FPSCR_PR)
          dmul(n, m);
        else
          UNIMPLEMENTED();
      }
      break;
    }
  /* fdiv <FREG_M>,<FREG_N> 1111nnnnmmmm0011 */
  case 204:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if(FPSCR_PR)
          ddiv(n, m);
        else
          UNIMPLEMENTED();
      }
      break;
    }
  /* fcmp/eq <FREG_M>,<FREG_N> 1111nnnnmmmm0100 */
  case 205:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if(FPSCR_PR)
          set_t_flag(get_dregister(n) == get_dregister(m));
        else
          UNIMPLEMENTED();
      }
      break;
    }
  /* fcmp/gt <FREG_M>,<FREG_N> 1111nnnnmmmm0101 */
  case 206:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if(FPSCR_PR)
          set_t_flag(get_dregister(n) > get_dregister(m));
        else
          UNIMPLEMENTED();
      }
      break;
    }
#if 0
  /* fmov.s @(R0,<REG_M>),<FREG_N> 1111nnnnmmmm0110 */
  case 207:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if (FPSCR_SZ) {
          RDAT (get_register(0) + get_register(m), n);
        }
        else
        {
          SET_FI (n, RLAT (get_register(0) + get_register(m)));
        }
      }
      break;
    }
  /* fmov.s <FREG_M>,@(R0,<REG_N>) 1111nnnnmmmm0111 */
  case 208:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if (FPSCR_SZ) {
          WDAT (get_register(0)+get_register(n), m);
        }
        else
        {
          WLAT ((get_register(0)+get_register(n)), FI (m));
        }
      }
      break;
    }
#endif
  /* fmov.s @<REG_M>,<FREG_N> 1111nnnnmmmm1000 */
  case 209:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if (FPSCR_SZ) {
          UNIMPLEMENTED();
        }
        else
        {
          set_fregister(n, RLAT (get_register(m)));
        }
      }
      break;
    }
  /* fmov.s @<REG_M>+,<FREG_N> 1111nnnnmmmm1001 */
  case 210:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if (FPSCR_SZ) {
          UNIMPLEMENTED();
        }
        else
        {
          set_fregister(n, RLAT (get_register(m)));
          set_register(m, get_register(m) + 4);
        }
      }
      break;
    }
  /* fmov.s <FREG_M>,@<REG_N> 1111nnnnmmmm1010 */
  case 211:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if (FPSCR_SZ) {
          UNIMPLEMENTED();
        }
        else
        {
          WLAT (get_register(n), get_fregister(m));
        }
      }
      break;
    }
  /* fmov.s <FREG_M>,@-<REG_N> 1111nnnnmmmm1011 */
  case 212:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if (FPSCR_SZ) {
          UNIMPLEMENTED();
        }
        else
        {
          set_register(n, get_register(n) - 4);
          WLAT (get_register(n), get_fregister(m));
        }
      }
      break;
    }
  /* fmov <FREG_M>,<FREG_N> 1111nnnnmmmm1100 */
  case 213:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if (FPSCR_SZ) {
          UNIMPLEMENTED();
        }
        else
        {
          set_fregister(n, get_fregister(m));
        }
      }
      break;
    }
  /* fabs <FREG_N> 1111nnnn01011101 */
  case 214:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR)
          dabs(n);
        else
          UNIMPLEMENTED();
      }
      break;
    }
  /* fcnvds <DR_N>,FPUL 1111nnnn10111101 */
  case 215:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (! FPSCR_PR || n & 1)
          RAISE_EXCEPTION();
        else
        {
          if (!FPSCR_PR)
            UNIMPLEMENTED();
          union
          {
            int i;
            float f;
          } u;
          u.f = get_dregister(n);
          set_sregister(fpul, u.i);
        }
      }
      break;
    }
  /* fcnvsd FPUL,<DR_N> 1111nnnn10101101 */
  case 216:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (! FPSCR_PR || n & 1)
          RAISE_EXCEPTION();
        else
        {
          if (!FPSCR_PR)
            UNIMPLEMENTED();
          union
          {
            int i;
            float f;
          } u;
          u.i = get_sregister(fpul);
          set_dregister(n, u.f);
        }
      }
      break;
    }
#if 0
  /* fipr <FV_M>,<FV_N> 1111vvVV11101101 */
  case 217:      
    {
      int v1 = ((iword >> 10) & 3) * 4;
      int v2 = ((iword >> 8)  & 3) * 4;
      {
        if (FPSCR_PR)
          RAISE_EXCEPTION();
        else
        {
          double fsum = 0;
          /* FIXME: check for nans and infinities.  */
          fsum += FR (v1+0) * FR (v2+0);
          fsum += FR (v1+1) * FR (v2+1);
          fsum += FR (v1+2) * FR (v2+2);
          fsum += FR (v1+3) * FR (v2+3);
          SET_FR (v1+3, fsum);
        }
      }
      break;
    }
#endif
  /* fldi0 <FREG_N> 1111nnnn10001101 */
  case 218:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR)
          RAISE_EXCEPTION();
        USE(n);
        UNIMPLEMENTED();
      }
      break;
    }
  /* fldi1 <FREG_N> 1111nnnn10011101 */
  case 219:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR)
          RAISE_EXCEPTION();
        USE(n);
        UNIMPLEMENTED();
      }
      break;
    }
  /* flds <FREG_N>,FPUL 1111nnnn00011101 */
  case 220:      
    {
      int n = (iword >> 8) & 0xf;
      {
          union
          {
            int i;
            float f;
          } u;
          u.f = get_fregister(n);
          set_sregister(fpul, u.i);
      }
      break;
    }
  /* float FPUL,<FREG_N> 1111nnnn00101101 */
  case 221:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR)
          set_dregister(n, get_sregister(fpul));
        else
          UNIMPLEMENTED();
      }
      break;
    }
  /* fneg <FREG_N> 1111nnnn01001101 */
  case 222:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR)
          dneg(n);
        else
          UNIMPLEMENTED();
      }
      break;
    }
#if 0
  /* fpchg 1111011111111101 */
  case 223:      
    {
      {
        set_sregister(fpscr, get_sregister(fpscr) ^ FPSCR_MASK_PR);
      }
      break;
    }
  /* frchg 1111101111111101 */
  case 224:      
    {
      {
        if (FPSCR_PR)
          RAISE_EXCEPTION();
        else
          set_sregister(fpscr, get_sregister(fpscr) ^ FPSCR_MASK_FR);
      }
      break;
    }
  /* fsca 1111eeee11111101 */
  case 225:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR)
          RAISE_EXCEPTION();
        else
          {
            SET_FR (n, fsca_s (get_sregister(fpul), &sin));
            SET_FR (n+1, fsca_s (get_sregister(fpul), &cos));
          }
      }
      break;
    }
  /* fschg 1111001111111101 */
  case 226:      
    {
      {
        set_sregister(fpscr, get_sregister(fpscr) ^ FPSCR_MASK_SZ);
      }
      break;
    }
#endif
  /* fsqrt <FREG_N> 1111nnnn01101101 */
  case 227:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR)
          dsqrt(n);
        else
          UNIMPLEMENTED();
      }
      break;
    }
#if 0
  /* fsrra <FREG_N> 1111nnnn01111101 */
  case 228:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR)
          RAISE_EXCEPTION();
        else
          SET_FR (n, fsrra_s (FR (n)));
      }
      break;
    }
#endif
  /* ftrc <FREG_N>, FPUL 1111nnnn00111101 */
  case 229:      
    {
      int n = (iword >> 8) & 0xf;
      {
        if (FPSCR_PR) {
          double src = get_dregister(n);
          unsigned flags = get_sregister(fpscr);
          flags &= ~(0x3f << 12); /* Clear the 6 Cause bits. */
          if (isnan(src)) {
            set_sregister(fpul, 0x80000000);
            flags |= (1<<16)/*CauseV*/;
          } else {
            double trc = trunc(src);
            if ((isinf(src) && src > 0) || trc > 2147483647) {
              set_sregister(fpul, 2147483647);
              flags |= (1<<16)/*CauseV*/;
            } else if ((isinf(src) && src < 0) || trc < (-2147483647-1)) {
              set_sregister(fpul, -2147483647-1);
              flags |= (1<<16)/*CauseV*/;
            } else {
              set_sregister(fpul, (int)trc);
              if (src != trc)
                flags |= (1<<12)/*CauseI*/;
            }
            set_sregister(fpscr, flags);
          }
        }
        else {
          UNIMPLEMENTED();
        }
      }
      break;
    }
#if 0
  /* ftrv <FV_N> 1111vv0111111101 */
  case 230:      
    {
      int v1 = ((iword >> 10) & 3) * 4;
      {
        if (FPSCR_PR)
          RAISE_EXCEPTION();
        else
          UNIMPLEMENTED();
      }
      break;
    }
#endif
  /* fsts FPUL,<FREG_N> 1111nnnn00001101 */
  case 231:      
    {
      int n = (iword >> 8) & 0xf;
      {
          union
          {
            int i;
            float f;
          } u;
          u.i = get_sregister(fpul);
          set_fregister(n, u.f);
      }
      break;
    }
#if 0
  /* fmac <FREG_0>,<FREG_M>,<FREG_N> 1111nnnnmmmm1110 */
  case 232:      
    {
      int n = (iword >> 8) & 0xf;
      int m = (iword >> 4) & 0xf;
      {
        if (FPSCR_PR)
          RAISE_EXCEPTION();
        SET_FR (n, FR (m) * FR (0) + FR (n));
      }
      break;
    }
#endif
  default:
    {
        RAISE_UNSUPPORTED_INSTR();
    }
  }
}
