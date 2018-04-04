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

#ifndef ELPSOFT_HASH_H_
#define ELPSOFT_HASH_H_

/* hash flags */
#define ELPHASH_NOBLOCK 0x1000
#define ELPHASH_LAST    0x2000

struct elphash_plugin;

#ifdef ELPMD2
struct md2_state {
    unsigned char chksum[16], X[48], buf[16];
    unsigned long curlen;
};
#endif

#ifdef ELPMD5
struct md5_state {
    ulong32 length[2];
    ulong32 state[4], curlen;
    unsigned char buf[64];
};
#endif
#ifdef ELPSHA1
struct sha1_state {
    ulong32 length[2];
    ulong32 state[5], curlen;
    unsigned char buf[64];
};
#endif

#ifdef ELPSHA224
#endif
#ifdef ELPSHA256
struct sha256_state {
    ulong32 length[2];
    ulong32 state[8], curlen;
    unsigned char buf[64];
};
#endif

#ifdef ELPSHA384
#endif
#ifdef ELPSHA512
struct sha512_state {
    ulong64  length, state[8];
    unsigned long curlen;
    unsigned char buf[128];
};
#endif

#ifdef ELPTIGER
struct tiger_state {
    ulong64 state[3], length;
    unsigned long curlen;
    unsigned char buf[64];
};
#endif

#ifdef ELPCRC32
struct crc32_state {
   unsigned long reg, poly;
   ulong32 *lut;
};
typedef struct crc32_state elpcrc32_state;
#endif

typedef struct {
   int    flags;
   struct elphash_plugin *hash;

   union {
      void *place;
#ifdef ELPMD2
      struct md2_state md2;
#endif
#ifdef ELPMD5
      struct md5_state md5;
#endif
#ifdef ELPSHA1
      struct sha1_state sha1;
#endif
#ifdef ELPSHA224
#endif
#ifdef ELPSHA256
      struct sha256_state sha256;
#endif
#ifdef ELPSHA384
#endif
#ifdef ELPSHA512
      struct sha512_state sha512;
#endif
#ifdef ELPTIGER
      struct tiger_state tiger;
#endif
   } state;
} elphash_state;

/* OID is made global since it is used by the hashes (and we don't always include the ASN1 lib) */
#define ELPMAXOIDLEN 10
struct elp_asn1_list;
typedef struct elp_asn1_oid {
   char          *name;
   unsigned long  oid_len,
                  oid[ELPMAXOIDLEN];
   struct elp_asn1_list *ref;
} elp_asn1_oid;

typedef struct {
   int blocksize, digestsize;

   struct elp_asn1_oid OID;
   unsigned char ISO10118_byte;

   int (*init)(int flags, elphash_state *md);
   int (*process)(const unsigned char *in, unsigned long inlen, elphash_state *md);
   int (*done)(unsigned char *out, unsigned long *outlen, elphash_state *md);
   int (*test)(void);

/* accelerators */
   int (*hash_memory)(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
   int (*hmac_memory)(const unsigned char *key, unsigned long keylen, const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
} elphash_plugin;

#ifdef ELPMD2
int elpmd2_init(int flags, elphash_state *md);
int elpmd2_process(const unsigned char *in, unsigned long inlen, elphash_state *md);
int elpmd2_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpmd2_test(void);
extern const elphash_plugin elpmd2_plugin;
#endif

#ifdef ELPMD5
int elpmd5_init(int flags, elphash_state *md);
int elpmd5_compress(elphash_state *md, unsigned char *buf);
int elpmd5_process(const unsigned char *in, unsigned long inlen, elphash_state *md);
int elpmd5_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpmd5_test(void);
extern const elphash_plugin elpmd5_plugin;
#endif
#ifdef ELPSHA1
int elpsha1_init(int flags, elphash_state *md);
int elpsha1_compress(elphash_state *md, unsigned char *buf);
int elpsha1_process(const unsigned char *in, unsigned long inlen, elphash_state *md);
int elpsha1_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpsha1_test(void);
extern const elphash_plugin elpsha1_plugin;
#endif
#ifdef ELPSHA224
int elpsha224_init(int flags, elphash_state *md);
#define elpsha224_process elpsha256_process
int elpsha224_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpsha224_test(void);
extern const elphash_plugin elpsha224_plugin;
#endif
#ifdef ELPSHA256
int elpsha256_init(int flags, elphash_state *md);
int elpsha256_compress(elphash_state *md, unsigned char *buf);
int elpsha256_process(const unsigned char *in, unsigned long inlen, elphash_state *md);
int elpsha256_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpsha256_test(void);
extern const elphash_plugin elpsha256_plugin;
#endif
#ifdef ELPSHA384
int elpsha384_init(int flags, elphash_state *md);
#define elpsha384_process elpsha512_process
int elpsha384_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpsha384_test(void);
extern const elphash_plugin elpsha384_plugin;
#endif
#ifdef ELPSHA512
int elpsha512_init(int flags, elphash_state *md);
int elpsha512_compress(elphash_state *md, unsigned char *buf);
int elpsha512_process(const unsigned char *in, unsigned long inlen, elphash_state *md);
int elpsha512_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpsha512_test(void);
extern const elphash_plugin elpsha512_plugin;
#endif
#ifdef ELPSHA512_224
int elpsha512_224_init(int flags, elphash_state *md);
#define elpsha512_224_process elpsha512_process
int elpsha512_224_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpsha512_224_test(void);
extern const elphash_plugin elpsha512_224_plugin;
#endif
#ifdef ELPSHA512_256
int elpsha512_256_init(int flags, elphash_state *md);
#define elpsha512_256_process elpsha512_process
int elpsha512_256_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elpsha512_256_test(void);
extern const elphash_plugin elpsha512_256_plugin;
#endif

#ifdef ELPTIGER
int elptiger_init(int flags, elphash_state *md);
int elptiger_compress(elphash_state *md, unsigned char *buf);
int elptiger_process(const unsigned char *in, unsigned long inlen, elphash_state *md);
int elptiger_done(unsigned char *out, unsigned long *outlen, elphash_state *md);
int elptiger_test(void);
extern const elphash_plugin elptiger_plugin;
#endif

#ifdef ELPCRC32
int elpcrc32_init(elpcrc32_state *state);
void elpcrc32_set_val(unsigned long val, elpcrc32_state *state);
unsigned long elpcrc32_get_val(elpcrc32_state *state);
int elpcrc32_set_poly(unsigned long poly, elpcrc32_state *state);
int elpcrc32_done(elpcrc32_state *state);
int elpcrc32_process(const unsigned char *in, unsigned long inlen_bits, elpcrc32_state *state);
int elpcrc32_test(void);
#endif

/* a simple macro for making hash "process" functions */
#define HASH_PROCESS(func_name, compress_name, state_var, block_size)                       \
int func_name (const unsigned char *in, unsigned long inlen, elphash_state *md)             \
{                                                                                           \
    unsigned long n;                                                                        \
    int           err;                                                                      \
    XSTACK()                                                                                \
    if (in == NULL || md == NULL) {                                                         \
       return ELPEINV;                                                                      \
    }                                                                                       \
    if (md->state. state_var .curlen > sizeof(md->state. state_var .buf)) {                 \
       return ELPEINV;                                                                      \
    }                                                                                       \
    while (inlen > 0) {                                                                     \
        if (md->state. state_var .curlen == 0 && inlen >= block_size) {                     \
           if ((err = compress_name (md, (unsigned char *)in)) != ELPEOK) {                 \
              return err;                                                                   \
           }                                                                                \
           md->state. state_var .length += block_size * 8;                                  \
           in             += block_size;                                                    \
           inlen          -= block_size;                                                    \
        } else {                                                                            \
           n = MIN(inlen, (block_size - md->state. state_var .curlen));                     \
           XMEMCPY(md->state. state_var .buf + md->state. state_var.curlen, in, (size_t)n);  \
           md->state. state_var .curlen += n;                                               \
           in             += n;                                                             \
           inlen          -= n;                                                             \
           if (md->state. state_var .curlen == block_size) {                                \
              if ((err = compress_name (md, md->state. state_var .buf)) != ELPEOK) {        \
                 return err;                                                                \
              }                                                                             \
              md->state. state_var .length += 8*block_size;                                 \
              md->state. state_var .curlen = 0;                                             \
           }                                                                                \
       }                                                                                    \
    }                                                                                       \
    return ELPEOK;                                                                          \
}

/* this version is for 32-bit only platforms */
#define HASH_PROCESS32(func_name, compress_name, state_var, block_size)                     \
int func_name (const unsigned char *in, unsigned long inlen, elphash_state *md)             \
{                                                                                           \
    unsigned long n;                                                                        \
    int           err;                                                                      \
    ulong32       tmp;                                                                      \
    XSTACK()                                                                                \
    if (in == NULL || md == NULL) {                                                         \
       return ELPEINV;                                                                      \
    }                                                                                       \
    if (md->state. state_var .curlen > sizeof(md->state. state_var .buf)) {                 \
       return ELPEINV;                                                                      \
    }                                                                                       \
    while (inlen > 0) {                                                                     \
        if (md->state. state_var .curlen == 0 && inlen >= block_size) {                     \
           if ((err = compress_name (md, (unsigned char *)in)) != ELPEOK) {                 \
              return err;                                                                   \
           }                                                                                \
           tmp = md->state. state_var .length[0];                                           \
           md->state. state_var .length[0] += block_size * 8;                               \
           if (md->state. state_var .length[0] < tmp) { md->state. state_var .length[1] += 1; } \
           in             += block_size;                                                    \
           inlen          -= block_size;                                                    \
        } else {                                                                            \
           n = MIN(inlen, (block_size - md->state. state_var .curlen));                     \
           XMEMCPY(md->state. state_var .buf + md->state. state_var.curlen, in, (size_t)n); \
           md->state. state_var .curlen += n;                                               \
           in             += n;                                                             \
           inlen          -= n;                                                             \
           if (md->state. state_var .curlen == block_size) {                                \
              if ((err = compress_name (md, md->state. state_var .buf)) != ELPEOK) {        \
                 return err;                                                                \
              }                                                                             \
              tmp = md->state. state_var .length[0];                                        \
              md->state. state_var .length[0] += block_size * 8;                            \
              if (md->state. state_var .length[0] < tmp) { md->state. state_var .length[1] += 1; } \
              md->state. state_var .curlen = 0;                                             \
           }                                                                                \
       }                                                                                    \
    }                                                                                       \
    return ELPEOK;                                                                          \
}

#ifdef ELPHASHMEM
int elphash_memory(const elphash_plugin *hash,
                const unsigned char *in,  unsigned long  inlen,
                      unsigned char *out, unsigned long *outlen);
#endif

#ifdef ELPPKCS5
int elppkcs_5_alg1(const unsigned char *password, unsigned long password_len,
                   const unsigned char *salt,
                                   int  iteration_count,
                  const elphash_plugin *hash,
                         unsigned char *out,     unsigned long *outlen);

int elppkcs_5_alg2(const unsigned char *password,        unsigned long password_len,
                   const unsigned char *salt,            unsigned long salt_len,
                                   int  iteration_count,         const elphash_plugin *hash,
                         unsigned char *out,            unsigned long *outlen);
#endif


#endif
