// Mapping of ARM registers to SH4 registers.
// Include this file the first time for defining the mapping.
// Include this file a second time for undefining it.
#ifndef MAP_SH4
// First include, we define the mapping
#define MAP_SH4
#define r8 "should be cp"
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
#undef r8
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
