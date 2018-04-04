/*-----------------------------------------------------------------------
//
// Proprietary Information of Elliptic Technologies
// Copyright (C) 2002-2013, all rights reserved
// Elliptic Technologies, Inc.
//
// As part of our confidentiality  agreement, Elliptic Technologies and
// the Company, as  a  Receiving Party, of  this  information  agrees to
// keep strictly  confidential  all Proprietary Information  so received
// from Elliptic Technologies. Such Proprietary Information can be used
// solely for  the  purpose  of evaluating  and/or conducting a proposed
// business  relationship  or  transaction  between  the  parties.  Each
// Party  agrees  that  any  and  all  Proprietary  Information  is  and
// shall remain confidential and the property of Elliptic Technologies.
// The  Company  may  not  use  any of  the  Proprietary  Information of
// Elliptic Technologies for any purpose other  than  the  above-stated
// purpose  without the prior written consent of Elliptic Technologies.
//
// Filename:         $Source: /home/repository/cvsrep/secureboot/sbsdk/src/sb.h,v $
// Current Revision: $Revision: 1.13 $
// Last Updated:     $Date: 2012/09/13 13:13:59 $
// Current Tag:      $Name:  $
*/

#include "elpsoft.h"

// flags for phase0b
#define SB_P0B_SRC_RELATIVE 1
#define SB_P0B_DST_RELATIVE 2
#define SB_P0B_ENCRYPTED    4

typedef int (*proxy_read_cb)(unsigned char *dst, unsigned long long src, unsigned long len);
typedef int (*proxy_write_cb)(unsigned long long dst, const unsigned char *src, unsigned long len);

int elp_secureboot_phase0a(proxy_read_cb proxy_read,
                             unsigned long long  code_base, unsigned long code_len,
                            const unsigned char *hash,
                                            int *stat,
                                  unsigned char *sb_page,        unsigned sb_page_size);

int elp_secureboot_phase0b(proxy_read_cb proxy_read, proxy_write_cb proxy_write,
                           const unsigned char *kdk,
                           const unsigned char *key,      unsigned long key_len,
                           unsigned long long   sys,
                           unsigned long long  *first_address,
                           unsigned long       *version,  int          *stat,
                           unsigned char       *sb_page,  unsigned      sb_page_size,
                           unsigned char       *sys_buf,  unsigned      sys_buf_size);

int elp_secureboot_phase0c(proxy_read_cb proxy_read,
                           const unsigned char *kdk,
                           unsigned long long   encrypted_base,       unsigned long  encrypted_baselen,
                                 unsigned char *dst_base,
                                           int *stat);

int sb_proxy_read(unsigned char *dst, unsigned long long src, unsigned long len);
int sb_proxy_write(unsigned long long dst, const unsigned char *src, unsigned long len);
int sb_proxy_copy(proxy_read_cb proxy_read, proxy_write_cb proxy_write,
                   unsigned long long dst, unsigned long long src, unsigned long len,
                   unsigned char *sb_page, unsigned sb_page_size);

// Error codes
enum {
  SB_OK=0,
  SB_INVALID_INPUT,      // input parameters are invalid
  SB_INVALID_SIZE,       // buffer passed was an invalid size
  SB_INVALID_SIGNATURE,  // signature is invalid
  SB_INVALID_HASH,       // hash is invalid
  SB_CANNOT_READ,        // proxy cannot read from device
  SB_CANNOT_WRITE,       // proxy cannot write to device
  SB_INVALID_ADDRESS,    // invalid address for proxy
  SB_COMP_ERR,           // computation failed (internal error)
  SB_BUILD_ERR,          // invalid build configuration
};


/**** Internal Routines ****/
int raw_verify(proxy_read_cb proxy_read, unsigned long long phase1_base, unsigned long phase1_len,
               const unsigned char *key,         unsigned long key_len,
               const unsigned char *sig,         unsigned long sig_len,
               int                 *stat,
               unsigned char *sb_page,   unsigned sb_page_size);


#ifndef ELPSOFT_H_
typedef unsigned ulong32;
typedef unsigned long long ulong64;
#endif
struct sha256_st {
    ulong64 length;
    ulong32 state[8], curlen;
    unsigned char buf[64];
};

typedef struct {
   union {
      void *place;
      struct sha256_st sha256;
   } state;
} hash_state;
void sha256_init(hash_state * md);
int sha256_process(const unsigned char *in, unsigned long inlen, hash_state *md);
int sha256_done(unsigned char *out, hash_state *md);
int sha256_memory(proxy_read_cb proxy_read, unsigned long long buf, unsigned long buflen, unsigned char *md, unsigned char *sb_page, unsigned sb_page_size);

void *std_memcpy(void *dest, const void *src, int n);
void *std_memset(void *s, int c, int n);
int std_memcmp(const void *s1, const void *s2, int n);

