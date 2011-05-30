// Mapping of ARM registers to SH4 registers.
// Include this file the first time for defining the mapping.
// Include this file a second time for undefining it.
#ifndef MAP_SH4
// First include, we define the mapping
#define MAP_SH4
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
#else
// Second include, we define the mapping
#undef MAP_SH4
#undef r0
#undef r1
#undef r2
#undef r3
#undef r4
#undef r5
#undef r6
#undef r7
#undef r8
#undef r9
#undef ip
#undef lr
#undef pc
#undef r10
#undef r11
#undef r12
#undef r13
#undef r14
#undef r15
#endif
