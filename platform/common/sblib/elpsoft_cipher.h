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

#ifndef ELPSOFT_CIPHER_H_
#define ELPSOFT_CIPHER_H_

enum {
   ELPECB_MODE=0,
   ELPCBC_MODE,
   ELPCFB_MODE,
   ELPOFB_MODE,
};

/* cipher flags */
#define ELPCIPHER_USETOKEN 0x10000
#define ELPCIPHER_ENCRYPT  0x20000
#define ELPCIPHER_NOBLOCK  0x40000
#define ELPCIPHER_ONECALL  0x80000

#ifdef ELPAES

#ifdef ELPAES256
   #define ELPAESWORDS 60
#elif defined(ELPAES192)
   #define ELPAESWORDS 52
#else
   #define ELPAESWORDS 44
#endif

struct aes_key {
#ifdef ELPAESENCONLY
   ulong32 eK[ELPAESWORDS];
#else
   ulong32 eK[ELPAESWORDS], dK[ELPAESWORDS];
#endif
   int Nr;
};
#endif

#ifdef ELPTDES
struct tdes_key {
    ulong32 ek[3][32], dk[3][32];
};
#endif

#ifdef ELPKASUMI
struct kasumi_key {
    ulong32 KLi1[8], KLi2[8],
            KOi1[8], KOi2[8], KOi3[8],
            KIi1[8], KIi2[8], KIi3[8];
};
#endif

#ifdef ELPKSEED
struct kseed_key {
    ulong32 K[32], dK[32];
};
#endif

#ifdef ELPRC2
struct rc2_key {
   unsigned xkey[64];
};
#endif

#ifdef ELPRC5
struct rc5_key {
   int rounds;
   ulong32 K[50];
};
#endif


#ifdef ELPCAMELLIA
struct camellia_key {
    int R;
    ulong64 kw[4], k[24], kl[6];
};
#endif

#ifdef ELPCAST5
struct cast5_key {
    ulong32 K[32], keylen;
};
#endif

#ifdef ELPBLOWFISH
struct blowfish_key {
   ulong32 S[4][256];
   ulong32 K[18];
};
#endif

#ifdef ELPTWOFISH
#ifndef ELPSMALLCODE
   struct twofish_key {
      ulong32 S[4][256], K[40];
   };
#else
   struct twofish_key {
      ulong32 K[40];
      unsigned char S[32], start;
   };
#endif
#endif

#ifdef ELPM6
struct m6_key {
   unsigned char xkey[8];
   ulong32 key[2];
};
#endif

#ifdef ELPMULTI2
struct multi2_key {
    int N;
    ulong32 uk[8];
};
#endif

#ifdef ELPSMS4
struct sms4_key {
    ulong32 uk[36];
};
#endif    

struct elpcipher_plugin;

typedef struct {
   unsigned char ukey[MAXBLOCKSIZE];

   int flags,
       blocksize,
       keysize;

   const struct elpcipher_plugin *cipher;

   union {
      void *place;
   #ifdef ELPAES
       struct aes_key aeskey;
   #endif
   #ifdef ELPTDES
       struct tdes_key tdeskey;
   #endif
   #ifdef ELPKASUMI
       struct kasumi_key kasumikey;
   #endif
   #ifdef ELPKSEED
       struct kseed_key kseedkey;
   #endif
   #ifdef ELPRC2
       struct rc2_key rc2key;
   #endif
   #ifdef ELPRC5
       struct rc5_key rc5key;
   #endif
   #ifdef ELPCAMELLIA
       struct camellia_key camelliakey;
   #endif
   #ifdef ELPCAST5
       struct cast5_key cast5key;
   #endif
   #ifdef ELPBLOWFISH
       struct blowfish_key blowfishkey;
   #endif
   #ifdef ELPTWOFISH
       struct twofish_key twofishkey;
   #endif
   #ifdef ELPM6
       struct m6_key m6key;
   #endif
   #ifdef ELPMULTI2
       struct multi2_key multi2key;
   #endif
   #ifdef ELPSMS4
       struct sms4_key sms4key;
   #endif
   } key;
} elpsymmetric_key;

#ifdef ELPCTR
typedef struct {
   elpsymmetric_key  key;
   unsigned char  pad[MAXBLOCKSIZE],
                  IV[MAXBLOCKSIZE];
   int            padsize,
                  padidx,
                  mode, 
                  ctrlen;
} elpsymmetric_ctr;
#else
typedef int elpsymmetric_ctr;
#endif

#ifdef ELPCBC
typedef struct {
   elpsymmetric_key  key;
   unsigned char  pad[MAXBLOCKSIZE];
} elpsymmetric_cbc;
#else
typedef int elpsymmetric_cbc;
#endif

#ifdef ELPCFB
typedef struct {
   elpsymmetric_key  key;
   unsigned char  pad[MAXBLOCKSIZE];
   int            padsize,
                  padidx;
} elpsymmetric_cfb;
#else
typedef int elpsymmetric_cfb;
#endif

#ifdef ELPOFB
typedef struct {
   elpsymmetric_key  key;
   unsigned char  pad[MAXBLOCKSIZE];
   int            padsize,
                  padidx;
} elpsymmetric_ofb;
#else
typedef int elpsymmetric_ofb;
#endif

#ifdef ELPF8
/** A block cipher F8 structure */
typedef struct {
   /** The index of the cipher chosen */
   int                 cipher, 
   /** The block size of the given cipher */                        
                       blocklen, 
   /** The padding offset */
                       padlen;
   /** The current IV */
   unsigned char       IV[MAXBLOCKSIZE],
                       MIV[MAXBLOCKSIZE];
   /** Current block count */
   ulong32             blockcnt;
   /** The scheduled key */
   elpsymmetric_key    key;
} elpsymmetric_f8;
#else
typedef int elpsymmetric_f8;
#endif


/* cipher plugin structure */
typedef struct elpcipher_plugin {
   int blocksize;
   const char *name;

   int (*setup)(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
   int (*encrypt)(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
   int (*decrypt)(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
   int (*done)(elpsymmetric_key *skey);
   int (*test)(void);

/* accelerators */
   int (*ctr_process)(const unsigned char *in, unsigned char *out, int bytelen,  int flags, elpsymmetric_ctr *ctr);
   int (*cbc_process)(const unsigned char *in, unsigned char *out, int blocklen, int flags, elpsymmetric_cbc *cbc);
   int (*cfb_process)(const unsigned char *in, unsigned char *out, int bytelen,  int flags, elpsymmetric_cfb *cfb);
   int (*ofb_process)(const unsigned char *in, unsigned char *out, int bytelen,  int flags, elpsymmetric_ofb *ofb);
   int (*f8_process) (const unsigned char *in, unsigned char *out, int bytelen,  int flags, elpsymmetric_f8  *f8);

//ccm
   int (*ccm_memory)(
    const unsigned char *key,      unsigned long keylen,
       elpsymmetric_key *uskey,
    const unsigned char *nonce,    unsigned long noncelen,
    const unsigned char *header,   unsigned long headerlen,
          unsigned char *pt,       unsigned long ptlen,
          unsigned char *ct,
          unsigned char *tag,      unsigned long *taglen,
                    int  direction,
    const unsigned char *BO, const unsigned char *CTR,
                    int  ctrwidth);

//gcm
   int (*gcm_memory)(
               const unsigned char *key,    unsigned long keylen,
                  elpsymmetric_key *uskey,
               const unsigned char *IV,     unsigned long IVlen,
               const unsigned char *adata,  unsigned long adatalen,
                     unsigned char *pt,     unsigned long ptlen,
                     unsigned char *ct,
                     unsigned char *tag,    unsigned long *taglen,
                               int direction);

   int (*cmac_memory)(const unsigned char *key, unsigned long keylen, elpsymmetric_key *uskey, const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
   int (*xcbc_memory)(const unsigned char *key, unsigned long keylen, elpsymmetric_key *uskey, const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
   int (*f9_memory)  (const unsigned char *key, unsigned long keylen, elpsymmetric_key *uskey, const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
} elpcipher_plugin;

/* modes */
#ifdef ELPCTR

#define ELPCTRLITTLE   0x1000
#define ELPCTRBIG      0x2000
#define ELPCTRRFC3686  0x4000

int elpctr_start(elpsymmetric_key *skey, const unsigned char *IV, int mode, elpsymmetric_ctr *ctr);
int elpctr_start_ex(const elpcipher_plugin *cipher, const unsigned char *key, unsigned long keylen, const unsigned char *IV, int flags, int mode, elpsymmetric_ctr *ctr);
int elpctr_process(const unsigned char *in, unsigned char *out, int bytelen, elpsymmetric_ctr *ctr);
int elpctr_getiv(elpsymmetric_ctr *ctr, unsigned char *IV, int *IVlen);
int elpctr_setiv(elpsymmetric_ctr *ctr, unsigned char *IV, int IVlen);
int elpctr_done(elpsymmetric_ctr *ctr);
#endif

#ifdef ELPCBC
int elpcbc_start(elpsymmetric_key *skey, const unsigned char *IV, elpsymmetric_cbc *cbc);
int elpcbc_start_ex(const elpcipher_plugin *cipher, const unsigned char *key, unsigned long keylen, int flags, const unsigned char *IV, elpsymmetric_cbc *cbc);
int elpcbc_encrypt(const unsigned char *pt, unsigned char *ct, int blocklen, elpsymmetric_cbc *cbc);
int elpcbc_decrypt(const unsigned char *ct, unsigned char *pt, int blocklen, elpsymmetric_cbc *cbc);
int elpcbc_getiv(elpsymmetric_cbc *cbc, unsigned char *IV, int *IVlen);
int elpcbc_setiv(elpsymmetric_cbc *cbc, unsigned char *IV, int IVlen);
int elpcbc_done(elpsymmetric_cbc *cbc);
#endif

#ifdef ELPCBCCS
enum {
   ELP_CBCCS_1=0,
   ELP_CBCCS_2,
   ELP_CBCCS_3
};
   
int elpcbc_cs_encrypt(const elpcipher_plugin *cipher, int cs_mode,
                      const unsigned char *key, unsigned long keylen, int flags, const unsigned char *IV,
                      const unsigned char *pt,  unsigned long ptlen,
                            unsigned char *ct);

int elpcbc_cs_decrypt(const elpcipher_plugin *cipher, int cs_mode,
                      const unsigned char *key, unsigned long keylen, int flags, const unsigned char *IV,
                      const unsigned char *ct,  unsigned long ctlen,
                            unsigned char *pt);

#endif                            

#ifdef ELPCFB
int elpcfb_start(elpsymmetric_key *skey, const unsigned char *IV, elpsymmetric_cfb *cfb);
int elpcfb_start_ex(const elpcipher_plugin *cipher, const unsigned char *key, unsigned long keylen, int flags, const unsigned char *IV, elpsymmetric_cfb *cfb);
int elpcfb_encrypt(const unsigned char *pt, unsigned char *ct, int bytelen, elpsymmetric_cfb *cfb);
int elpcfb_decrypt(const unsigned char *ct, unsigned char *pt, int bytelen, elpsymmetric_cfb *cfb);
int elpcfb_getiv(elpsymmetric_cfb *cfb, unsigned char *IV, int *IVlen);
int elpcfb_setiv(elpsymmetric_cfb *cfb, unsigned char *IV, int IVlen, int private);
int elpcfb_done(elpsymmetric_cfb *cfb);
#endif

#ifdef ELPOFB
int elpofb_start(elpsymmetric_key *skey, const unsigned char *IV, elpsymmetric_ofb *ofb);
int elpofb_start_ex(const elpcipher_plugin *cipher, const unsigned char *key, unsigned long keylen, int flags, const unsigned char *IV, elpsymmetric_ofb *ofb);
int elpofb_process(const unsigned char *in, unsigned char *out, int bytelen, elpsymmetric_ofb *ofb);
int elpofb_getiv(elpsymmetric_ofb *ofb, unsigned char *IV, int *IVlen);
int elpofb_setiv(elpsymmetric_ofb *ofb, unsigned char *IV, int IVlen, int private);
int elpofb_done(elpsymmetric_ofb *ofb);
#endif

#ifdef ELPF8
int elpf8_start(const elpcipher_plugin *cipher, const unsigned char *key, int keylen, const unsigned char *IV, const unsigned char *salt, int saltlen, int mode, elpsymmetric_f8 *f8);
int elpf8_setiv(const unsigned char *IV, unsigned long len, elpsymmetric_f8 *f8, int private);
int elpf8_getiv(unsigned char *IV, unsigned long *len, elpsymmetric_f8 *f8);
int elpf8_encrypt(const unsigned char *pt, unsigned char *ct, unsigned long len, elpsymmetric_f8 *f8);
int elpf8_decrypt(const unsigned char *ct, unsigned char *pt, unsigned long len, elpsymmetric_f8 *f8);
int elpf8_done(elpsymmetric_f8 *f8);
int elpf8_test_mode(const elpcipher_plugin *aes, const elpcipher_plugin *kasumi);
#endif

/* ciphers */
#ifdef ELPAES
int elpaes_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpaes_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpaes_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpaes_done(elpsymmetric_key *skey);
int elpaes_test(void);
extern const elpcipher_plugin elpaes_plugin;
extern const ulong32 aes_tab_Te4[];
#endif

#ifdef ELPTDES
int elptdes_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elptdes_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elptdes_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elptdes_done(elpsymmetric_key *skey);
int elptdes_test(void);
extern const elpcipher_plugin elptdes_plugin;

int elpdes_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpdes_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpdes_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpdes_done(elpsymmetric_key *skey);
int elpdes_test(void);
extern const elpcipher_plugin elpdes_plugin;
#endif

#ifdef ELPKASUMI
int elpkasumi_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpkasumi_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpkasumi_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpkasumi_done(elpsymmetric_key *skey);
int elpkasumi_test(void);
extern const elpcipher_plugin elpkasumi_plugin;
#endif

#ifdef ELPKSEED
int elpkseed_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpkseed_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpkseed_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpkseed_done(elpsymmetric_key *skey);
int elpkseed_test(void);
extern const elpcipher_plugin elpkseed_plugin;
#endif

#ifdef ELPRC2
int elprc2_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elprc2_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elprc2_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elprc2_done(elpsymmetric_key *skey);
int elprc2_test(void);
extern const elpcipher_plugin elprc2_plugin;
#endif

#ifdef ELPRC5
int elprc5_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elprc5_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elprc5_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elprc5_done(elpsymmetric_key *skey);
int elprc5_test(void);
extern const elpcipher_plugin elprc5_plugin;
#endif

#ifdef ELPCAMELLIA
int elpcamellia_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpcamellia_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpcamellia_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpcamellia_done(elpsymmetric_key *skey);
int elpcamellia_test(void);
extern const elpcipher_plugin elpcamellia_plugin;
#endif

#ifdef ELPCAST5
int elpcast5_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpcast5_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpcast5_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpcast5_done(elpsymmetric_key *skey);
int elpcast5_test(void);
extern const elpcipher_plugin elpcast5_plugin;
#endif

#ifdef ELPBLOWFISH
int elpblowfish_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpblowfish_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpblowfish_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpblowfish_done(elpsymmetric_key *skey);
int elpblowfish_test(void);
extern const elpcipher_plugin elpblowfish_plugin;
#endif

#ifdef ELPTWOFISH
int elptwofish_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elptwofish_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elptwofish_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elptwofish_done(elpsymmetric_key *skey);
int elptwofish_test(void);
extern const elpcipher_plugin elptwofish_plugin;
#endif

#ifdef ELPMULTI2
int elpmulti2_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpmulti2_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpmulti2_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpmulti2_done(elpsymmetric_key *skey);
int elpmulti2_test(void);
extern const elpcipher_plugin elpmulti2_plugin;
#endif

#ifdef ELPSMS4
int elpsms4_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpsms4_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpsms4_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpsms4_done(elpsymmetric_key *skey);
int elpsms4_test(void);
extern const elpcipher_plugin elpsms4_plugin;
#endif

#ifdef ELPM6
int elpm6_s56_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpm6_ke56_setup(const unsigned char *key, int keybytelen, int flags, elpsymmetric_key *skey);
int elpm6_s56_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpm6_ke56_encrypt(const unsigned char *pt, unsigned char *ct, elpsymmetric_key *skey);
int elpm6_s56_decrypt(const unsigned char *ct, unsigned char *pt, elpsymmetric_key *skey);
int elpm6_done(elpsymmetric_key *skey);
int elpm6_test(void);

extern const elpcipher_plugin elpm6_s56_plugin;
extern const elpcipher_plugin elpm6_ke56_plugin;
#endif

#ifdef ELPCCM

typedef struct {
   elpsymmetric_key skey;
   unsigned char ctr[16], ctr_pad[16], pad[16], c0[16];
   unsigned long ctr_width, ctr_idx, pad_idx;
   int direction, state;
} elpccm_state;

void elpccm_increment_ctr(unsigned char *ctr, unsigned w);

#define ELP_CCM_ENCRYPT 0
#define ELP_CCM_DECRYPT 1

#define ELP_CCM_STATE_INIT 0
#define ELP_CCM_STATE_TEXT 1
#define ELP_CCM_STATE_DONE 2

int elpccm_init(elpccm_state *ccm,
                elpsymmetric_key *skey, const unsigned char *B0,
                const unsigned char *ctr, unsigned long ctr_width);

int elpccm_init_ex(elpccm_state *ccm, const elpcipher_plugin *cipher,
                   const unsigned char *key, int keylen,
                   int flags, const unsigned char *B0,
                   const unsigned char *ctr, unsigned long ctr_width);

int elpccm_add_aad(elpccm_state *ccm,
                   const unsigned char *adata, unsigned long adatalen);

int elpccm_process(elpccm_state *ccm,
                   unsigned char *pt, unsigned long ptlen,
                   unsigned char *ct, int direction);

int elpccm_done(elpccm_state *ccm, unsigned char *tag, unsigned long *taglen);

int elpccm_memory_ex(const elpcipher_plugin *cipher,
    const unsigned char *key,      unsigned long keylen,
       elpsymmetric_key *uskey,
    const unsigned char *nonce,    unsigned long noncelen,
    const unsigned char *header,   unsigned long headerlen,
          unsigned char *pt,       unsigned long ptlen,
          unsigned char *ct,
          unsigned char *tag,      unsigned long *taglen,
                    int  direction,
    const unsigned char *B0, const unsigned char *CTR,
                    int  ctrwidth);

#define elpccm_memory(cipher, key, keylen, uskey, nonce, noncelen, header, headerlen, pt, ptlen, ct, tag, taglen, direction) elpccm_memory_ex(cipher, key, keylen, uskey, nonce, noncelen, header, headerlen, pt, ptlen, ct, tag, taglen, direction, NULL, NULL, 0)

#endif

#ifdef ELPGCM
#define ELP_GCM_ENCRYPT 0
#define ELP_GCM_DECRYPT 1

#ifndef ELP_NO_TABLES
extern const unsigned char elpgcm_shift_table[];
#endif


#define ELP_GCM_MODE_IV    0
#define ELP_GCM_MODE_AAD   1
#define ELP_GCM_MODE_TEXT  2

typedef struct {
   elpsymmetric_key       K;
   unsigned char       H[16],        /* multiplier */
                       X[16],        /* accumulator */
                       Y[16],        /* counter */
                       Y_0[16],      /* initial counter */
                       buf[16];      /* buffer for stuff */

   const elpcipher_plugin   *cipher;

   int                 ivmode,       /* Which mode is the IV in? */
                       mode,         /* mode the GCM code is in */
                       buflen,       /* length of data in buf */
                       direction;

   ulong64             totlen,       /* 64-bit counter used for IV and AAD */
                       pttotlen;     /* 64-bit counter for the PT */

#ifdef ELPGCM_TABLES
   unsigned char       PC[16][256][16];  /* 16 tables of 8x128 */
#endif
} elpgcm_state;

void elpgcm_mult_h(elpgcm_state *gcm, unsigned char *I);
void elpgcm_gf_mult(const unsigned char *a, const unsigned char *b, unsigned char *c);

int elpgcm_init(elpgcm_state *gcm, const elpcipher_plugin *cipher, 
             const unsigned char *key,  int keylen);

int elpgcm_reset(elpgcm_state *gcm);

int elpgcm_add_iv(elpgcm_state *gcm,
               const unsigned char *IV,     unsigned long IVlen);

int elpgcm_add_aad(elpgcm_state *gcm,
               const unsigned char *adata,  unsigned long adatalen);

int elpgcm_process(elpgcm_state *gcm,
                     unsigned char *pt,     unsigned long ptlen,
                     unsigned char *ct,
                     int direction);

int elpgcm_done(elpgcm_state *gcm,
                     unsigned char *tag,    unsigned long *taglen);

int elpgcm_terminate(elpgcm_state *gcm);

int elpgcm_memory(const elpcipher_plugin *cipher,
               const unsigned char *key,    unsigned long keylen,
                  elpsymmetric_key *uskey,
               const unsigned char *IV,     unsigned long IVlen,
               const unsigned char *adata,  unsigned long adatalen,
                     unsigned char *pt,     unsigned long ptlen,
                     unsigned char *ct,
                     unsigned char *tag,    unsigned long *taglen,
                               int direction);

typedef struct {
   unsigned char *data;
   unsigned long len;
} elp_p_len;

int elpgcm_memory_multi(const elpcipher_plugin *cipher,
           const unsigned char *key,    unsigned long keylen,
              elpsymmetric_key *uskey,
               const elp_p_len *IV,
               const elp_p_len *adata,
                     elp_p_len *pt,
                 unsigned char *ct,
                 unsigned char *tag,    unsigned long *taglen,
                           int direction);
#endif


#define elpsymmetric_xts elpsymmetric_xex

#ifdef ELPXEX
#define elpxts_start     elpxex_start
#define elpxts_encrypt   elpxex_encrypt
#define elpxts_decrypt   elpxex_decrypt
#define elpxts_done      elpxex_done

typedef struct {
   elpsymmetric_key  key1, key2;
   const elpcipher_plugin *cipher;
} elpsymmetric_xex;

int elpxex_start(const elpcipher_plugin *cipher, 
                    const unsigned char *key1, 
                    const unsigned char *key2, 
                          unsigned long  keylen,
                                    int  flags, 
                       elpsymmetric_xex *xex);

int elpxex_encrypt(
   const unsigned char *pt, unsigned long ptlen,
         unsigned char *ct,
   const unsigned char *tweak,
         elpsymmetric_xex *xex);

int elpxex_decrypt(
   const unsigned char *ct, unsigned long ptlen,
         unsigned char *pt,
   const unsigned char *tweak,
         elpsymmetric_xex *xex);

int elpxex_done(elpsymmetric_xex *xex);


void elpxex_mult_x(unsigned char *I);
#else
typedef int elpsymmetric_xex;
#endif

#ifdef ELPEME2
typedef struct {
   unsigned char adkey[16], iv[16];
   elpsymmetric_key skey;
} elpsymmetric_eme2;

int elpeme2_start(const elpcipher_plugin *cipher, const unsigned char *key,
                  unsigned long keylen, int flags, elpsymmetric_eme2 *eme2);

int elpeme2_encrypt(const unsigned char *ad, unsigned long adlen,
                    const unsigned char *pt, unsigned long ptlen,
                    unsigned char *ct,       elpsymmetric_eme2 *eme2);

int elpeme2_decrypt(const unsigned char *ad, unsigned long adlen,
                    const unsigned char *ct, unsigned long ctlen,
                    unsigned char *pt,       elpsymmetric_eme2 *eme2);

int elpeme2_done(elpsymmetric_eme2 *eme2);
int elpeme2_test(void);
#else
typedef int elpsymmetric_eme2;
#endif

#ifdef ELPXCB
typedef struct {
   unsigned char hkey[16];
   elpsymmetric_key key_c, key_d, key_e;
} elpsymmetric_xcb;

int elpxcb_start(const elpcipher_plugin *cipher, const unsigned char *key,
                 unsigned long keylen, int flags, elpsymmetric_xcb *xcb);

int elpxcb_encrypt(const unsigned char *ad, unsigned long adlen,
                   const unsigned char *pt, unsigned long ptlen,
                   unsigned char *ct,       elpsymmetric_xcb *xcb);

int elpxcb_decrypt(const unsigned char *ad, unsigned long adlen,
                   const unsigned char *ct, unsigned long ctlen,
                   unsigned char *pt,       elpsymmetric_xcb *xcb);

int elpxcb_done(elpsymmetric_xcb *xcb);
int elpxcb_test(void);
#else
typedef int elpsymmetric_xcb;
#endif

#ifdef ELPKEYWRAP
int elpkeywrap_encrypt(const elpcipher_plugin *cipher,
                          const unsigned char *key, unsigned long keylen, int flags,
                          const unsigned char *pt,  unsigned long blocks,
                                unsigned char *ct);

int elpkeywrap_decrypt(const elpcipher_plugin *cipher,
                          const unsigned char *key, unsigned long keylen, int flags,
                          const unsigned char *ct,  unsigned long blocks,
                                unsigned char *pt);

int elpkeywrap_test(void);
#endif

#ifdef ELPCSA2
typedef unsigned int NUSHORT;
typedef struct
{
   unsigned char common_key[8]; /* Storage of the modified key */
   unsigned char kb[7][8];      /* Working storage for key schedule */
   ulong64 SR_a, SR_b;
   NUSHORT c;
   NUSHORT d;
   NUSHORT e;
   NUSHORT f;
   NUSHORT r;
   NUSHORT bib;
} elp_csa2_state;

int elp_csa2_setup(elp_csa2_state * state, const unsigned char *key);
int elp_csa2_encrypt(elp_csa2_state *csa, unsigned char *payload, unsigned long length);
int elp_csa2_decrypt(elp_csa2_state *csa, unsigned char *payload, unsigned long length);
int elp_csa2_test(void);
#endif


#endif /* ELPSOFT_CIPHER_H_ */
