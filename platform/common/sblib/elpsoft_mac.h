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

#ifndef ELPSOFT_MAC_H_
#define ELPSOFT_MAC_H_

/* Forward declaration of elpprng_plugin */
struct elp_prng_plugin;

#ifdef ELPCMAC

typedef struct {
   int             buflen,
                   blklen;
   unsigned char   block[MAXBLOCKSIZE],
                   prev[MAXBLOCKSIZE],
                   Lu[2][MAXBLOCKSIZE];
   elpsymmetric_key   key;
} elpcmac_state;

int elpcmac_init(elpcmac_state *cmac, elpsymmetric_key *key);
int elpcmac_init_ex(const elpcipher_plugin *cipher, const unsigned char *key, unsigned long keylen, int flags, elpcmac_state *cmac);
int elpcmac_process(elpcmac_state *cmac, const unsigned char *in, unsigned long inlen);
int elpcmac_done(elpcmac_state *cmac, unsigned char *out, unsigned long *outlen);
int elpcmac_memory(const elpcipher_plugin *cipher,
                const unsigned char *key, unsigned long keylen,
                   elpsymmetric_key *uskey,
                const unsigned char *in,  unsigned long inlen,
                      unsigned char *out, unsigned long *outlen);
#endif

#ifdef ELPF9
typedef struct {
   unsigned char akey[MAXBLOCKSIZE],
                 ACC[MAXBLOCKSIZE],
                 IV[MAXBLOCKSIZE];

   elpsymmetric_key key;

             int buflen,
                 keylen,
                 blocksize;
} elpf9_state;

int elpf9_init(elpf9_state *f9, elpsymmetric_key *key);
int elpf9_init_ex(const elpcipher_plugin *cipher, const unsigned char *key, unsigned long keylen, int flags, elpf9_state *f9);
int elpf9_process(elpf9_state *f9, const unsigned char *in, unsigned long inlen);
int elpf9_done(elpf9_state *f9, unsigned char *out, unsigned long *outlen);
int elpf9_memory(const elpcipher_plugin *cipher,
                const unsigned char *key, unsigned long keylen,
                   elpsymmetric_key *uskey,
                const unsigned char *in,  unsigned long inlen,
                      unsigned char *out, unsigned long *outlen);
int elpf9_test(const elpcipher_plugin *cipher);
#endif

#ifdef ELPXCBC

#define ELP_XCBC_3KEY   0x1000

typedef struct {
   int             buflen,
                   blklen;
   unsigned char   K[3][MAXBLOCKSIZE],
                   IV[MAXBLOCKSIZE];
   elpsymmetric_key   key;
   const elpcipher_plugin *cipher;
   int             blocksize;
} elpxcbc_state;

int elpxcbc_init(elpxcbc_state *xcbc, elpsymmetric_key *key);
int elpxcbc_init_ex(const elpcipher_plugin *cipher, const unsigned char *key, unsigned long keylen, int flags, elpxcbc_state *xcbc);
int elpxcbc_init_ex_3key(const elpcipher_plugin *cipher, const unsigned char *key, unsigned long keylen, int flags, elpxcbc_state *xcbc);
int elpxcbc_process(elpxcbc_state *xcbc, const unsigned char *in, unsigned long inlen);
int elpxcbc_done(elpxcbc_state *xcbc, unsigned char *out, unsigned long *outlen);
int elpxcbc_memory(const elpcipher_plugin *cipher,
                const unsigned char *key, unsigned long keylen,
                   elpsymmetric_key *uskey,
                const unsigned char *in,  unsigned long inlen,
                      unsigned char *out, unsigned long *outlen);
#endif


#ifdef ELPHMAC
typedef struct elphmac_state {
     const elphash_plugin *hash;
     elphash_state   md;
     elphash_state   hashstate;
     unsigned char  *key;
} elphmac_state;

int elphmac_init(elphmac_state *hmac, const elphash_plugin *hash, const unsigned char *key, unsigned long keylen);
int elphmac_process(elphmac_state *hmac, const unsigned char *in, unsigned long inlen);
int elphmac_done(elphmac_state *hmac, unsigned char *out, unsigned long *outlen);
int elphmac_memory(const elphash_plugin *hash,
                const unsigned char *key, unsigned long keylen,
                const unsigned char *in,  unsigned long inlen,
                      unsigned char *out, unsigned long *outlen);
#endif

#ifdef ELPSNOW3G
int elpsnow3g_uia_memory(const unsigned char *key, 
                         const unsigned char *iv,
                         const unsigned char *input,
                               unsigned long  input_bitlen,
                               unsigned char *tag);
int elpsnow3g_uia_test(void);
#endif

#ifdef ELPZUC
void elp128_eia3_compute_iv(unsigned long bearer,    unsigned long counter,
                            unsigned long direction, unsigned char *out);
int elp128_eia3_memory(const struct elp_prng_plugin *prng,
                       const unsigned char *key, unsigned long keylen,
                       const unsigned char *iv,
                       const unsigned char *in,  unsigned long inlen_bits,
                       unsigned char *tag);
int elp128_eia3_test(void);
#endif


#endif
