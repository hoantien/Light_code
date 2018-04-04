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

#ifndef ELPSOFT_H_
#define ELPSOFT_H_

#ifdef __KERNEL__

   #include <linux/kernel.h>
   #include <linux/module.h>
   #include <linux/sort.h>
   #include <linux/slab.h>
   #include <linux/ctype.h>
   typedef __u16 wchar_t;

   /* change memory allocation to kernel calls */
    void *  kern_malloc(size_t n);
    void *  kern_realloc(const void *p, size_t n);
    void *  kern_calloc(size_t n, size_t s);
    void    kern_free(void *p);
   /* change qsort to kernel 'sort' */
    void  kern_sort(void *base, size_t nmemb, size_t size, int(*compar)(const void *, const void *));

   #define ELPNOLIBC
   #define TFM_NO_DIV
   #define NO_XREPORT

   #define XMALLOC kern_malloc
   #define XCALLOC kern_calloc 
   #define XREALLOC kern_realloc
   #define XFREE kern_free

   #define XMEMSET memset
   #define XMEMCPY memcpy
   #define XMEMCMP memcmp
   #define XSTRCMP strcmp

   #define XQSORT kern_sort

#else /* __KERNEL__ */
   #ifndef ELPNOLIBC
      #include <stdio.h>
      #include <string.h>
      #include <stdlib.h>

      #ifdef ELPMEM_LOG
         void *b_malloc(size_t len, const char *func, const char *file, int l);
         void b_free(void *len);
         void *b_calloc(size_t n, size_t m, const char *func, const char *file, int l);
         void *b_realloc(void *ptr, size_t n, const char *func, const char *file, int l);
         void elp_report_lost_mem(void);
         unsigned long b_used(void);
         unsigned long b_peak(void);

         #define XMALLOC(m)    b_malloc(m, __FUNCTION__, __FILE__, __LINE__)
         #define XCALLOC(m, n) b_calloc(m, n, __FUNCTION__, __FILE__, __LINE__)
         #define XREALLOC(p, m)    b_realloc(p, m, __FUNCTION__, __FILE__, __LINE__)
         #define XFREE b_free
         #define XUSED() b_used()
         #define XPEAK() b_peak()
         #define XUSAGE() elp_report_lost_mem()
      #else
         #define XUSED() 0UL
         #define XPEAK() 0UL
         #define XUSAGE()
      #endif
      
      #ifdef ELPSTACK_LOG         
         void b_stack(const char *function, const char *file);
         void b_stack_init(void);
         unsigned long b_stack_depth(void);
         unsigned long b_stack_peak(void);
         #define XSTACK_INIT() b_stack_init();
         #define XSTACK() b_stack(__FUNCTION__, __FILE__);
         #define XSTACK_DEPTH() b_stack_depth()
         #define XSTACK_PEAK()  b_stack_peak()
      #else
         #define XSTACK_INIT()
         #define XSTACK()
         #define XSTACK_DEPTH() 0UL
         #define XSTACK_PEAK() 0UL
      #endif 

      #ifdef ELP_MEMORY_PROFILER2
      #warning going away
         void *b_malloc(int line, char *file, size_t len);
         void b_free(void *len);
         void *b_calloc(int line, char *file, size_t n, size_t m);
         void *b_realloc(int line, char *file, void *ptr, size_t n);
         unsigned long b_used(void);
         unsigned long b_peak(void);
         void b_dump_usage(void);

         #define XMALLOC(m)    b_malloc(__LINE__, __FILE__, m)
         #define XCALLOC(m, n) b_calloc(__LINE__, __FILE__, m, n)
         #define XREALLOC(p, m)    b_realloc(__LINE__, __FILE__, p, m)
         #define XFREE b_free
      #endif
      
      #ifdef ELPSOFT_HEAP
         void eheap_init(void *base, size_t size); 
         void *eheap_calloc(size_t p, size_t q);
         void *eheap_realloc(void *ptr, size_t p);
         void *eheap_malloc(size_t n);
         void eheap_free(void *p);
      #endif

      #if defined(ELPSOFT_HEAP)
         #ifndef XMALLOC
            #define XMALLOC(m)     eheap_malloc(m)
         #endif
         #ifndef XCALLOC
            #define XCALLOC(m, n)  eheap_calloc(m, n)
         #endif
         #ifndef XREALLOC
            #define XREALLOC(p, m) eheap_realloc(p, m)
         #endif
         #ifndef XFREE
            #define XFREE(p)       eheap_free(p)
         #endif
      #endif


      #ifndef XMALLOC
         #define XMALLOC malloc
      #endif

      #ifndef XREALLOC 
         #define XREALLOC realloc
      #endif

      #ifndef XCALLOC
         #define XCALLOC calloc
      #endif

      #ifndef XFREE
         #define XFREE   free
      #endif

      #ifndef XMEMSET
         #define XMEMSET memset
      #endif

      #ifndef XMEMCPY
         #define XMEMCPY memcpy
      #endif

      #ifndef XMEMCMP
         #define XMEMCMP memcmp
      #endif

      #ifndef XSTRCMP
         #define XSTRCMP strcmp
      #endif

      #ifndef XQSORT
         #define XQSORT qsort
      #endif

   #else

      #define XSTACK()
      #define XSTACK_INIT()
      #define XSTACK_DEPTH()
      #define XSTACK_PEAK()

      #include <stddef.h>

      #if !defined(ELP_MEMORY_PROFILER) && !defined(ELPMEM_LOG)      

         /* you can change how memory allocation works ... */
         void * XMALLOC(size_t n);
         void * XREALLOC(void *p, size_t n);
         void * XCALLOC(size_t n, size_t s);
         void   XFREE(void *p);

         void  XQSORT(void *base, size_t nmemb, size_t size, int(*compar)(const void *, const void *));

         /* various other functions */
         void * XMEMCPY(void *dest, const void *src, size_t n);
         int    XMEMCMP(const void *s1, const void *s2, size_t n);
         void * XMEMSET(void *s, int c, size_t n);      
         int XSTRCMP(const void * str1, const void * str2);
         unsigned long XPRNG(void *out, unsigned long outlen);
      #endif

   #endif /* ELPNOLIBC */
#endif /* __KERNEL__ */

/* include the other headers */
#ifndef ELP_HAS_ENV
#include <elpsoft_defines.h>
#endif

/*
 * This macro also limits the key size, which may need to be large for some
 * ciphers.
 */
#if defined(ELPRC2) || defined(ELPRC5)
#  define MAXBLOCKSIZE 128
#elif defined(ELPBLOWFISH)
#  define MAXBLOCKSIZE 56
#elif defined(ELPMULTI2)
#  define MAXBLOCKSIZE 40
#else
#  define MAXBLOCKSIZE 32
#endif

#include <elpsoft_error.h>
#include <elpsoft_cfg.h>
#include <elpsoft_cipher.h>
#include <elpsoft_hash.h>
#include <elpsoft_mac.h>
#include <elpsoft_prng.h>
#include <elpsoft_pk.h>
#include <elpsoft_math.h>

#endif /* ELPSOFT_H_ */
