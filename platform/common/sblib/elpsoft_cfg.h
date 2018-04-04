/*-----------------------------------------------------------------------
//
// Proprietary Information of Elliptic Technologies
// Copyright (C) 2002-2012, all rights reserved
// Elliptic Technologies Inc.
//
// As part of our confidentiality  agreement, Elliptic  Technologies and
// the Company, as  a  Receiving Party, of  this  information  agrees to
// keep strictly  confidential  all Proprietary Information  so received
// from Elliptic  Technologies. Such Proprietary Information can be used
// solely for  the  purpose  of evaluating  and/or conducting a proposed
// business  relationship  or  transaction  between  the  parties.  Each
// Party  agrees  that  any  and  all  Proprietary  Information  is  and
// shall remain confidential and the property of Elliptic  Technologies.
// The  Company  may  not  use  any of  the  Proprietary  Information of
// Elliptic  Technologies for any purpose other  than  the  above-stated
// purpose  without the prior written consent of Elliptic  Technologies.
//
*/

#ifndef ELPSOFT_CFG_H_
#define ELPSOFT_CFG_H_

/* detect i386/x86 */
#if !defined(__STRICT_ANSI__) && (defined(INTEL_CC) || (defined(_MSC_VER) && defined(WIN32)) || (defined(__GNUC__) && (defined(__DJGPP__) || defined(__CYGWIN__) || defined(__MINGW32__) || defined(__i386__))))
   #define ENDIAN_LITTLE
   #define ENDIAN_32BITWORD
   #ifndef ELPFAST
      #define ELPFAST
      #define ELPFAST_TYPE ulong32
   #endif
#endif

/* detect amd64 */
#if !defined(__STRICT_ANSI__) && defined(__x86_64__)
   #define ENDIAN_LITTLE
   #define ENDIAN_64BITWORD
   #ifndef ELPFAST
      #define ELPFAST
      #define ELPFAST_TYPE ulong64
   #endif
#endif

#ifdef ELP_PPC32
   #define ENDIAN_BIG
   #define ENDIAN_32BITWORD
   #ifndef ELPFAST
      #define ELPFAST
      #define ELPFAST_TYPE ulong32
   #endif
#endif

/* detects MIPS R5900 processors (PS2) */
#if (defined(__R5900) || defined(R5900) || defined(__R5900__)) && (defined(_mips) || defined(__mips__) || defined(mips))
   #define ENDIAN_LITTLE
   #define ENDIAN_64BITWORD
#endif

#ifdef ELP_MIPS32
   #define ENDIAN_LITTLE
   #define ENDIAN_32BITWORD
   #define ELPFAST
   #define ELPFAST_TYPE ulong32
#endif

/* ARM */
#if defined(ELP_ARM) || defined(TFM_ARM)
   #define ELPFAST
   #define ELPFAST_TYPE ulong32
   #ifndef ELPFAST_UNALIGN
      #define ELPFAST_ALIGN
   #endif
   #ifndef ENDIAN_BIG
      #define ENDIAN_LITTLE
   #endif
   #define ENDIAN_32BITWORD
#endif

#if defined(ELPFAST) && !(defined(ENDIAN_LITTLE) || defined(ENDIAN_BIG))
   #error Cannot define ELPFAST without ENDIAN_LITTLE or ENDIAN_BIG
#endif

#if (defined(ELFPAST) && !defined(ELPFAST_TYPE)) || (!defined(ELPFAST) && defined(ELPFAST_TYPE))
   #error Cannot define ELPFAST without ELPFAST_TYPE (and vice versa)
#endif

#if !(defined(ENDIAN_BIG) || defined(ENDIAN_LITTLE))
   #define ENDIAN_NEUTRAL
#endif

/* 64-bit constants */
#ifndef TFM_H_
#ifdef _MSC_VER
   #define CONST64(n) n ## ui64
   typedef unsigned __int64 ulong64;
#else
   #define CONST64(n) n ## ULL
   typedef unsigned long long ulong64;
#endif
#endif

/* this is the "32-bit at least" data type
 * Re-define it to suit your platform but it must be at least 32-bits
 */
#if defined(__x86_64__) || (defined(__sparc__) && defined(__arch64__))
   typedef unsigned ulong32;
#else
   typedef unsigned long ulong32;
#endif

/* ---- HELPER MACROS ---- */
#ifdef ENDIAN_NEUTRAL

#define STORE32L(x, y)                                                                     \
     { (y)[3] = (unsigned char)(((x)>>24)&255); (y)[2] = (unsigned char)(((x)>>16)&255);   \
       (y)[1] = (unsigned char)(((x)>>8)&255); (y)[0] = (unsigned char)((x)&255); }

#define LOAD32L(x, y)                            \
     { x = ((unsigned long)((y)[3] & 255)<<24) | \
           ((unsigned long)((y)[2] & 255)<<16) | \
           ((unsigned long)((y)[1] & 255)<<8)  | \
           ((unsigned long)((y)[0] & 255)); }

#define STORE64L(x, y)                                                                     \
     { (y)[7] = (unsigned char)(((x)>>56)&255); (y)[6] = (unsigned char)(((x)>>48)&255);   \
       (y)[5] = (unsigned char)(((x)>>40)&255); (y)[4] = (unsigned char)(((x)>>32)&255);   \
       (y)[3] = (unsigned char)(((x)>>24)&255); (y)[2] = (unsigned char)(((x)>>16)&255);   \
       (y)[1] = (unsigned char)(((x)>>8)&255); (y)[0] = (unsigned char)((x)&255); }

#define LOAD64L(x, y)                                                       \
     { x = (((ulong64)((y)[7] & 255))<<56)|(((ulong64)((y)[6] & 255))<<48)| \
           (((ulong64)((y)[5] & 255))<<40)|(((ulong64)((y)[4] & 255))<<32)| \
           (((ulong64)((y)[3] & 255))<<24)|(((ulong64)((y)[2] & 255))<<16)| \
           (((ulong64)((y)[1] & 255))<<8)|(((ulong64)((y)[0] & 255))); }

#define STORE32H(x, y)                                                                     \
     { (y)[0] = (unsigned char)(((x)>>24)&255); (y)[1] = (unsigned char)(((x)>>16)&255);   \
       (y)[2] = (unsigned char)(((x)>>8)&255); (y)[3] = (unsigned char)((x)&255); }

#define LOAD32H(x, y)                            \
     { x = ((unsigned long)((y)[0] & 255)<<24) | \
           ((unsigned long)((y)[1] & 255)<<16) | \
           ((unsigned long)((y)[2] & 255)<<8)  | \
           ((unsigned long)((y)[3] & 255)); }

#define STORE64H(x, y)                                                                     \
   { (y)[0] = (unsigned char)(((x)>>56)&255); (y)[1] = (unsigned char)(((x)>>48)&255);     \
     (y)[2] = (unsigned char)(((x)>>40)&255); (y)[3] = (unsigned char)(((x)>>32)&255);     \
     (y)[4] = (unsigned char)(((x)>>24)&255); (y)[5] = (unsigned char)(((x)>>16)&255);     \
     (y)[6] = (unsigned char)(((x)>>8)&255); (y)[7] = (unsigned char)((x)&255); }

#define LOAD64H(x, y)                                                      \
   { x = (((ulong64)((y)[0] & 255))<<56)|(((ulong64)((y)[1] & 255))<<48) | \
         (((ulong64)((y)[2] & 255))<<40)|(((ulong64)((y)[3] & 255))<<32) | \
         (((ulong64)((y)[4] & 255))<<24)|(((ulong64)((y)[5] & 255))<<16) | \
         (((ulong64)((y)[6] & 255))<<8)|(((ulong64)((y)[7] & 255))); }

#endif /* ENDIAN_NEUTRAL */

#ifdef ENDIAN_LITTLE

#if defined(ELP_ARMV6)
// ARM friendly big endian load/store for little endian cores
   #if !defined(__ARMCC_VERSION) && defined(__GNUC__)
      // routines for GCC
      #define STORE32H(x, y)  asm("rev %0,%0\n\tstr %0,%1\n\t"::"r"(x), "m"(((ulong32 *)(y))[0]):"0")
      #define LOAD32H(x, y)   asm("ldr %0,%2\n\trev %0,%0\n\t":"=r"(x):"r"(x), "m"(((ulong32 *)(y))[0]):"0")
   #else
      // routines for ARMcc
      #define STORE32H(x, y) ((ulong32 *)(y))[0] = __rev(x)
      #define LOAD32H(x, y)  { x = __rev(((ulong32 *)(y))[0]); }
   #endif
#else
// portable load/store
   #define STORE32H(x, y)                                                                     \
        { (y)[0] = (unsigned char)(((x)>>24)&255); (y)[1] = (unsigned char)(((x)>>16)&255);   \
          (y)[2] = (unsigned char)(((x)>>8)&255); (y)[3] = (unsigned char)((x)&255); }

   #define LOAD32H(x, y)                            \
        { x = ((unsigned long)((y)[0] & 255)<<24) | \
              ((unsigned long)((y)[1] & 255)<<16) | \
              ((unsigned long)((y)[2] & 255)<<8)  | \
              ((unsigned long)((y)[3] & 255)); }
#endif

#define STORE64H(x, y)                                                                     \
   { (y)[0] = (unsigned char)(((x)>>56)&255); (y)[1] = (unsigned char)(((x)>>48)&255);     \
     (y)[2] = (unsigned char)(((x)>>40)&255); (y)[3] = (unsigned char)(((x)>>32)&255);     \
     (y)[4] = (unsigned char)(((x)>>24)&255); (y)[5] = (unsigned char)(((x)>>16)&255);     \
     (y)[6] = (unsigned char)(((x)>>8)&255); (y)[7] = (unsigned char)((x)&255); }

#define LOAD64H(x, y)                                                      \
   { x = (((ulong64)((y)[0] & 255))<<56)|(((ulong64)((y)[1] & 255))<<48) | \
         (((ulong64)((y)[2] & 255))<<40)|(((ulong64)((y)[3] & 255))<<32) | \
         (((ulong64)((y)[4] & 255))<<24)|(((ulong64)((y)[5] & 255))<<16) | \
         (((ulong64)((y)[6] & 255))<<8)|(((ulong64)((y)[7] & 255))); }

#ifdef ENDIAN_32BITWORD

#define STORE32L(x, y)        \
     { ulong32  __t = (x); XMEMCPY(y, &__t, 4); }

#define LOAD32L(x, y)         \
     XMEMCPY(&(x), y, 4);

#define STORE64L(x, y)                                                                     \
     { (y)[7] = (unsigned char)(((x)>>56)&255); (y)[6] = (unsigned char)(((x)>>48)&255);   \
       (y)[5] = (unsigned char)(((x)>>40)&255); (y)[4] = (unsigned char)(((x)>>32)&255);   \
       (y)[3] = (unsigned char)(((x)>>24)&255); (y)[2] = (unsigned char)(((x)>>16)&255);   \
       (y)[1] = (unsigned char)(((x)>>8)&255); (y)[0] = (unsigned char)((x)&255); }

#define LOAD64L(x, y)                                                       \
     { x = (((ulong64)((y)[7] & 255))<<56)|(((ulong64)((y)[6] & 255))<<48)| \
           (((ulong64)((y)[5] & 255))<<40)|(((ulong64)((y)[4] & 255))<<32)| \
           (((ulong64)((y)[3] & 255))<<24)|(((ulong64)((y)[2] & 255))<<16)| \
           (((ulong64)((y)[1] & 255))<<8)|(((ulong64)((y)[0] & 255))); }

#else /* 64-bit words then  */

#define STORE32L(x, y)        \
     { ulong32 __t = (x); XMEMCPY(y, &__t, 4); }

#define LOAD32L(x, y)         \
     { XMEMCPY(&(x), y, 4); x &= 0xFFFFFFFF; }

#define STORE64L(x, y)        \
     { ulong64 __t = (x); XMEMCPY(y, &__t, 8); }

#define LOAD64L(x, y)         \
    { XMEMCPY(&(x), y, 8); }

#endif /* ENDIAN_64BITWORD */

#endif /* ENDIAN_LITTLE */

#ifdef ENDIAN_BIG
#define STORE32L(x, y)                                                                     \
     { (y)[3] = (unsigned char)(((x)>>24)&255); (y)[2] = (unsigned char)(((x)>>16)&255);   \
       (y)[1] = (unsigned char)(((x)>>8)&255); (y)[0] = (unsigned char)((x)&255); }

#define LOAD32L(x, y)                            \
     { x = ((unsigned long)((y)[3] & 255)<<24) | \
           ((unsigned long)((y)[2] & 255)<<16) | \
           ((unsigned long)((y)[1] & 255)<<8)  | \
           ((unsigned long)((y)[0] & 255)); }

#define STORE64L(x, y)                                                                     \
   { (y)[7] = (unsigned char)(((x)>>56)&255); (y)[6] = (unsigned char)(((x)>>48)&255);     \
     (y)[5] = (unsigned char)(((x)>>40)&255); (y)[4] = (unsigned char)(((x)>>32)&255);     \
     (y)[3] = (unsigned char)(((x)>>24)&255); (y)[2] = (unsigned char)(((x)>>16)&255);     \
     (y)[1] = (unsigned char)(((x)>>8)&255); (y)[0] = (unsigned char)((x)&255); }

#define LOAD64L(x, y)                                                      \
   { x = (((ulong64)((y)[7] & 255))<<56)|(((ulong64)((y)[6] & 255))<<48) | \
         (((ulong64)((y)[5] & 255))<<40)|(((ulong64)((y)[4] & 255))<<32) | \
         (((ulong64)((y)[3] & 255))<<24)|(((ulong64)((y)[2] & 255))<<16) | \
         (((ulong64)((y)[1] & 255))<<8)|(((ulong64)((y)[0] & 255))); }

#ifdef ENDIAN_32BITWORD

#define STORE32H(x, y)        \
     { ulong32 __t = (x); XMEMCPY(y, &__t, 4); }

#define LOAD32H(x, y)         \
     XMEMCPY(&(x), y, 4);

#define STORE64H(x, y)                                                                     \
     { (y)[0] = (unsigned char)(((x)>>56)&255); (y)[1] = (unsigned char)(((x)>>48)&255);   \
       (y)[2] = (unsigned char)(((x)>>40)&255); (y)[3] = (unsigned char)(((x)>>32)&255);   \
       (y)[4] = (unsigned char)(((x)>>24)&255); (y)[5] = (unsigned char)(((x)>>16)&255);   \
       (y)[6] = (unsigned char)(((x)>>8)&255);  (y)[7] = (unsigned char)((x)&255); }

#define LOAD64H(x, y)                                                       \
     { x = (((ulong64)((y)[0] & 255))<<56)|(((ulong64)((y)[1] & 255))<<48)| \
           (((ulong64)((y)[2] & 255))<<40)|(((ulong64)((y)[3] & 255))<<32)| \
           (((ulong64)((y)[4] & 255))<<24)|(((ulong64)((y)[5] & 255))<<16)| \
           (((ulong64)((y)[6] & 255))<<8)| (((ulong64)((y)[7] & 255))); }

#else /* 64-bit words then  */

#define STORE32H(x, y)        \
     { ulong32 __t = (x); XMEMCPY(y, &__t, 4); }

#define LOAD32H(x, y)         \
     { XMEMCPY(&(x), y, 4); x &= 0xFFFFFFFF; }

#define STORE64H(x, y)        \
     { ulong64 __t = (x); XMEMCPY(y, &__t, 8); }

#define LOAD64H(x, y)         \
    { XMEMCPY(&(x), y, 8); }

#endif /* ENDIAN_64BITWORD */
#endif /* ENDIAN_BIG */

#define BSWAP(x)  ( ((x>>24)&0x000000FFUL) | ((x<<24)&0xFF000000UL)  | \
                    ((x>>8)&0x0000FF00UL)  | ((x<<8)&0x00FF0000UL) )


/* 32-bit Rotates */
#if defined(_MSC_VER)

/* instrinsic rotate */
#include <stdlib.h>
#pragma intrinsic(_lrotr,_lrotl)
#define ROR(x,n) _lrotr(x,n)
#define ROL(x,n) _lrotl(x,n)
#define RORc(x,n) _lrotr(x,n)
#define ROLc(x,n) _lrotl(x,n)

#elif !defined(__STRICT_ANSI__) && defined(__GNUC__) && (defined(__i386__) || defined(__x86_64__)) && !defined(INTEL_CC) && !defined(ELP_NO_ASM)

static inline unsigned ROL(unsigned word, int i)
{
   asm ("roll %%cl,%0"
      :"=r" (word)
      :"0" (word),"c" (i));
   return word;
}

static inline unsigned ROR(unsigned word, int i)
{
   asm ("rorl %%cl,%0"
      :"=r" (word)
      :"0" (word),"c" (i));
   return word;
}

#ifndef ELP_NO_ROLC

static inline unsigned ROLc(unsigned word, const int i)
{
   asm ("roll %2,%0"
      :"=r" (word)
      :"0" (word),"I" (i));
   return word;
}

static inline unsigned RORc(unsigned word, const int i)
{
   asm ("rorl %2,%0"
      :"=r" (word)
      :"0" (word),"I" (i));
   return word;
}

#else

#define ROLc ROL
#define RORc ROR

#endif

#elif !defined(__STRICT_ANSI__) && defined(ELP_PPC32)

static inline unsigned ROL(unsigned word, int i)
{
   asm ("rotlw %0,%0,%2"
      :"=r" (word)
      :"0" (word),"r" (i));
   return word;
}

static inline unsigned ROR(unsigned word, int i)
{
   asm ("rotlw %0,%0,%2"
      :"=r" (word)
      :"0" (word),"r" (32-i));
   return word;
}

#ifndef ELP_NO_ROLC

static inline unsigned ROLc(unsigned word, const int i)
{
   asm ("rotlwi %0,%0,%2"
      :"=r" (word)
      :"0" (word),"I" (i));
   return word;
}

static inline unsigned RORc(unsigned word, const int i)
{
   asm ("rotrwi %0,%0,%2"
      :"=r" (word)
      :"0" (word),"I" (i));
   return word;
}

#else

#define ROLc ROL
#define RORc ROR

#endif


#else

/* rotates the hard way */
#define ROL(x, y) ( (((unsigned long)(x)<<(unsigned long)((y)&31)) | (((unsigned long)(x)&0xFFFFFFFFUL)>>(unsigned long)(32-((y)&31)))) & 0xFFFFFFFFUL)
#define ROR(x, y) ( ((((unsigned long)(x)&0xFFFFFFFFUL)>>(unsigned long)((y)&31)) | ((unsigned long)(x)<<(unsigned long)(32-((y)&31)))) & 0xFFFFFFFFUL)
#define ROLc(x, y) ( (((unsigned long)(x)<<(unsigned long)((y)&31)) | (((unsigned long)(x)&0xFFFFFFFFUL)>>(unsigned long)(32-((y)&31)))) & 0xFFFFFFFFUL)
#define RORc(x, y) ( ((((unsigned long)(x)&0xFFFFFFFFUL)>>(unsigned long)((y)&31)) | ((unsigned long)(x)<<(unsigned long)(32-((y)&31)))) & 0xFFFFFFFFUL)

#endif


/* 64-bit Rotates */
#if !defined(__STRICT_ANSI__) && defined(__GNUC__) && defined(__x86_64__) && !defined(ELP_NO_ASM)

static inline unsigned long ROL64(unsigned long word, int i)
{
   asm("rolq %%cl,%0"
      :"=r" (word)
      :"0" (word),"c" (i));
   return word;
}

static inline unsigned long ROR64(unsigned long word, int i)
{
   asm("rorq %%cl,%0"
      :"=r" (word)
      :"0" (word),"c" (i));
   return word;
}

#ifndef ELP_NO_ROLC

static inline unsigned long ROL64c(unsigned long word, const int i)
{
   asm("rolq %2,%0"
      :"=r" (word)
      :"0" (word),"J" (i));
   return word;
}

static inline unsigned long ROR64c(unsigned long word, const int i)
{
   asm("rorq %2,%0"
      :"=r" (word)
      :"0" (word),"J" (i));
   return word;
}

#else /* ELP_NO_ROLC */

#define ROL64c ROL64
#define ROR64c ROR64

#endif

#else /* Not x86_64  */

#define ROL64(x, y) \
    ( (((x)<<((ulong64)(y)&63)) | \
      (((x)&CONST64(0xFFFFFFFFFFFFFFFF))>>((ulong64)64-((y)&63)))) & CONST64(0xFFFFFFFFFFFFFFFF))

#define ROR64(x, y) \
    ( ((((x)&CONST64(0xFFFFFFFFFFFFFFFF))>>((ulong64)(y)&CONST64(63))) | \
      ((x)<<((ulong64)(64-((y)&CONST64(63)))))) & CONST64(0xFFFFFFFFFFFFFFFF))

#define ROL64c(x, y) \
    ( (((x)<<((ulong64)(y)&63)) | \
      (((x)&CONST64(0xFFFFFFFFFFFFFFFF))>>((ulong64)64-((y)&63)))) & CONST64(0xFFFFFFFFFFFFFFFF))

#define ROR64c(x, y) \
    ( ((((x)&CONST64(0xFFFFFFFFFFFFFFFF))>>((ulong64)(y)&CONST64(63))) | \
      ((x)<<((ulong64)(64-((y)&CONST64(63)))))) & CONST64(0xFFFFFFFFFFFFFFFF))

#endif

#ifndef MAX
   #define MAX(x, y) ( ((x)>(y))?(x):(y) )
#endif

#ifndef MIN
   #define MIN(x, y) ( ((x)<(y))?(x):(y) )
#endif

/* extract a byte portably */
#ifdef _MSC_VER
   #define byte(x, n) ((unsigned char)((x) >> (8 * (n))))
#else
   #define byte(x, n) (((x) >> (8 * (n))) & 255)
#endif

#ifndef ELPYIELD
#define ELPYIELD()
#endif

#ifdef CLUEPROF
ulong64 rdtsc();
extern int clueprof_print;
#endif

#ifndef ELPTESTING
   #ifdef ELPSMALLCODE
      #ifndef NO_XREPORT
          #define NO_XREPORT
      #endif
  #endif
#endif

#ifdef NO_XREPORT
   #define XREPORT(m)
   #define XDUMP()
   #define XREPORT_OFF()
   #define XREPORT_ON()
#endif

#ifdef XREPORT_STDERR

   #define XREPORT(m) fprintf(stderr, "%s:%d %s\n", __FILE__, __LINE__, m)
   #define XDUMP()
   #define XREPORT_OFF()
   #define XREPORT_ON()

#endif

#ifndef XREPORT
extern volatile int elp_report_ignore;

   #define XREPORT(m) elp_report(__FILE__, __LINE__, m)
   #define XDUMP()    elp_dump()
   #define XREPORT_OFF() elp_report_ignore = 1;
   #define XREPORT_ON()  elp_report_ignore = 0;

void elp_report(char *file, int lineno, char *msg);
void elp_dump(void);
#endif

#ifdef ELPDEBUG_TRACE

#include <stdarg.h>

extern void (*elp_vtrace)(const char *loc, const char *fmt, va_list ap);
extern void (*elp_trace)(const char *loc, const char *fmt, ...);
void elp_debug_set_tracing(int enable);

#define XVTRACE(fmt, ap) elp_vtrace(__func__, fmt, ap)
#define XTRACE_VA(...) elp_trace(__func__, __VA_ARGS__)
#define XTRACE(args) XTRACE_VA args

#else

/*
 * Note: even though elp_debug_set_tracing is declared here, the library must
 * have been compiled with ELPDEBUG_TRACE enabled for it to work.
 */
void elp_debug_set_tracing(int enable);

#define XVTRACE(loc, fmt, ap) ((void)0)
#define XTRACE(args) ((void)0)

#endif

#ifdef ELPSOFT_EMIT_LIB_ID
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#ifdef ELPMATH
#include <tfm.h>
#endif
const char *elp_library_id =
#else
extern const char *elp_library_id;
#endif

#ifdef ELPSOFT_EMIT_APP_ID
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#ifdef ELPMATH
#include <tfm.h>
#endif
const char *elpsoft_application_id =
#endif

#if defined(ELPSOFT_EMIT_LIB_ID) || defined(ELPSOFT_EMIT_APP_ID)
   "Ellipsys Middleware Library "
#ifdef ELPAES
   "ELPAES "
#endif
#ifdef ELPAES128
   "ELPAES128 "
#endif
#ifdef ELPAES192
   "ELPAES192 "
#endif
#ifdef ELPAES256
   "ELPAES256 "
#endif
#ifdef ELPTDES
   "ELPTDES "
#endif
#ifdef ELPKASUMI
   "ELPKASUMI "
#endif
#ifdef ELPKSEED
   "ELPKSEED "
#endif
#ifdef ELPCAMELLIA
   "ELPCAMELLIA "
#endif
#ifdef ELPRC2
   "ELPRC2 "
#endif
#ifdef ELPRC5
   "ELPRC5 "
#endif
#ifdef ELPCAST5
   "ELPCAST5 "
#endif
#ifdef ELPBLOWFISH
   "ELPBLOWFISH "
#endif
#ifdef ELPTWOFISH
   "ELPTWOFISH "
#endif
#ifdef ELPM6
   "ELPM6 "
#endif
#ifdef ELPMULTI2
   "ELPMULTI2 "
#endif
#ifdef ELPSMS4
   "ELPSMS4 "
#endif
#ifdef ELPCSA2
   "ELPCSA2 "
#endif
#ifdef ELPCBC
   "ELPCBC "
#endif
#ifdef ELPCBCCS
   "ELPCBCCS "
#endif
#ifdef ELPCTR
   "ELPCTR "
#endif
#ifdef ELPCFB
   "ELPCFB "
#endif
#ifdef ELPOFB
   "ELPOFB "
#endif
#ifdef ELPF8
   "ELPF8 "
#endif
#ifdef ELPMD2
   "ELPMD2 "
#endif
#ifdef ELPMD5
   "ELPMD5 "
#endif
#ifdef ELPSHA1
   "ELPSHA1 "
#endif
#ifdef ELPSHA256
   "ELPSHA256 "
#endif
#ifdef ELPSHA224
   "ELPSHA224 "
#endif
#ifdef ELPSHA512
   "ELPSHA512 "
#endif
#ifdef ELPSHA512_224
   "ELPSHA512_224 "
#endif
#ifdef ELPSHA512_256
   "ELPSHA512_256 "
#endif
#ifdef ELPSHA384
   "ELPSHA384 "
#endif
#ifdef ELPTIGER
   "ELPTIGER "
#endif
#ifdef ELPCRC32
   "ELPCRC32 "
#endif
#ifdef ELPEME2
   "ELPEME2 "
#endif
#ifdef ELPXCB
   "ELPXCB "
#endif
#ifdef ELPCMAC
   "ELPCMAC "
#endif
#ifdef ELPF9
   "ELPF9 "
#endif
#ifdef ELPHMAC
   "ELPHMAC "
#endif
#ifdef ELPXCBC
   "ELPXCBC "
#endif
#ifdef ELPCCM
   "ELPCCM "
#endif
#ifdef ELPGCM
   "ELPGCM "
#endif
#ifdef ELPGCM_TABLES
   "ELPGCM_TABLES "
#endif
#ifdef ELPXEX
   "ELPXEX "
#endif
#ifdef ELPKEYWRAP
   "ELPKEYWRAP "
#endif
#ifdef ELPHASHMEM
   "ELPHASHMEM "
#endif
#ifdef ELPMATH
   "ELPMATH "
#endif
#ifdef ELPASN1
   "ELPASN1 "
#endif
#ifdef ELPX509
   "ELPX509 "
#endif
#ifdef ELPSPRNG
   "ELPSPRNG "
#endif
#ifdef ELPRC4
   "ELPRC4 "
#endif
#ifdef ELPLFG
   "ELPLFG "
#endif
#ifdef ELPSNOW3G
   "ELPSNOW3G "
#endif
#ifdef ELPZUC
   "ELPZUC "
#endif
#ifdef ELPX931AES
   "ELPX931AES "
#endif
#ifdef ELPPKCS1
   "ELPPKCS1 "
#endif
#ifdef ELPPKCS5
   "ELPPKCS5 "
#endif
#ifdef ELPPKCS7
   "ELPPKCS7 "
#endif
#ifdef ELPPKCS8
   "ELPPKCS8 "
#endif
#ifdef ELPPKCS9
   "ELPPKCS9 "
#endif
#ifdef ELPX931
   "ELPX931 "
#endif
#ifdef ELPRSA
   "ELPRSA "
#endif
#ifdef ELPDSA
   "ELPDSA "
#endif
#ifdef ELPECC
   "ELPECC "
#endif
#ifdef ELPECC_FP
   "ELPECC_FP "
#endif
#ifdef ELPECC_SHAMIR
   "ELPECC_SHAMIR "
#endif
#ifdef ELPECC112
   "ELPECC112 "
#endif
#ifdef ELPECC128
   "ELPECC128 "
#endif
#ifdef ELPECC160
   "ELPECC160 "
#endif
#ifdef ELPECC192
   "ELPECC192 "
#endif
#ifdef ELPECC224
   "ELPECC224 "
#endif
#ifdef ELPECC256
   "ELPECC256 "
#endif
#ifdef ELPECC384
   "ELPECC384 "
#endif
#ifdef ELPECC521
   "ELPECC521 "
#endif
#ifdef ELPECC2
   "ELPECC2 "
#endif
#ifdef ELPECC2_163
   "ELPECC2_163 "
#endif
#ifdef ELPECC2_233
   "ELPECC2_233 "
#endif
#ifdef ELPECC2_283
   "ELPECC2_283 "
#endif
#ifdef ELPECC2_409
   "ELPECC2_409 "
#endif
#ifdef ELPECC2_571
   "ELPECC2_571 "
#endif
#ifdef ELPECC2_FP
   "ELPECC2_FP "
#endif
#ifdef ELPECC2_KARATSUBA
   "ELPECC2_KARATSUBA "
#endif
#ifdef ELPSMALLMEM
   "ELPSMALLMEM "
#endif
#ifdef ELPSMALLCODE
   "ELPSMALLCODE "
#endif
#ifdef ELPFAST_ALIGN
   "ELPFAST_ALIGN "
#endif
#ifdef ELPFAST_UNALIGN
   "ELPFAST_UNALIGN "
#endif
#ifdef ELPSTACK_LOG
   "ELPSTACK_LOG "
#endif
#ifdef ELPMEM_LOG
   "ELPMEM_LOG "
#endif
"FP_MAX_SIZE==[" STR(FP_MAX_SIZE) "] "
#ifdef FP_ENTRIES
"FP_ENTRIES==[" STR(FP_ENTRIES) "] "
#endif
#ifdef FP_LUT
"FP_LUT==[" STR(FP_LUT) "] "
#endif
" EOL";
#endif /* EMIT_ID */

#ifdef ELPSOFT_EMIT_APP_ID
int elp_check_config(void)
{
    const char *a, *b;
    a = elpsoft_application_id;
    b = elp_library_id;
    while (*a && *b) {
       if (*a != *b) return -1;
       ++a; ++b;
    }
    return 0;
}
#endif

#endif /* ELPSOFT_CFG_H_ */
