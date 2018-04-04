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
#ifdef ELPECC2

#if defined(ELPECC2_FP) && defined(ELPSMALLMEM)
   #error Cannot define ELPECC2_FP and ELPSMALLMEM at the same time
#endif   


/* size of our temp buffers for exported keys */
#define ECC2_BUF_SIZE 256

/* maximum size supported by the lib */
#ifdef ELPECC2_571
#define MAXECC2BITS 576
#elif defined(ELPECC2_409)
#define MAXECC2BITS 416
#elif defined(ELPECC2_283)
#define MAXECC2BITS 288
#elif defined(ELPECC2_233)
#define MAXECC2BITS 256
#else
#define MAXECC2BITS 192
#endif

/* # of words per polynomial */
#define ECC2WORDS   (MAXECC2BITS/32)

typedef struct {
   ulong32     x[ECC2WORDS],
               y[ECC2WORDS],
               z[ECC2WORDS];
} elpecc2_point;

typedef struct {
   int   size;  /* number of 32-bit words */
   void (*add)(const ulong32 *a, const ulong32 *b, ulong32 *c);
   void (*mul)(const ulong32 *a, const ulong32 *b, ulong32 *c);
   void (*sqr)(const ulong32 *a, ulong32 *b);
   int (*inv)(ulong32 *a);
   void (*red)(ulong32 *a);

/* accelerators for ptadd, ptmul */
   int (*pmul)(elpecc2_point *P,  elpecc2_point *Q, void *k, int idx, int koblitz);
   int (*padd)(elpecc2_point *P,  elpecc2_point *Q, elpecc2_point *R, int idx, int koblitz);
   int (*pdbl)(elpecc2_point *P,  elpecc2_point *R, int idx, int koblitz);
   int (*pnorm)(elpecc2_point *P, elpecc2_point *R, int idx);
} elpgf2_set;

typedef struct {
   char       *name;
   elpgf2_set  sets[5];
} elpgf2_plugin;

typedef struct {
   void          *k;
   elpecc2_point  pubkey;
   int            idx,
                  size,
                  koblitz,
                  type;
} elpecc2_key;

#ifdef ELPECC2_163
void elpgf_163_add   (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_163_mult  (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_163_sqr   (const ulong32 *a, ulong32 *b);
void elpgf_163_reduce(ulong32 *a);
int elpgf_163_inv   (ulong32 *a);
#endif

#ifdef ELPECC2_233
void elpgf_233_add   (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_233_mult  (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_233_sqr   (const ulong32 *a, ulong32 *b);
void elpgf_233_reduce(ulong32 *a);
int elpgf_233_inv   (ulong32 *a);
#endif

#ifdef ELPECC2_283
void elpgf_283_add   (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_283_mult  (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_283_sqr   (const ulong32 *a, ulong32 *b);
void elpgf_283_reduce(ulong32 *a);
int elpgf_283_inv   (ulong32 *a);
#endif

#ifdef ELPECC2_409
void elpgf_409_add   (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_409_mult  (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_409_sqr   (const ulong32 *a, ulong32 *b);
void elpgf_409_reduce(ulong32 *a);
int elpgf_409_inv   (ulong32 *a);
#endif

#ifdef ELPECC2_571
void elpgf_571_add   (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_571_mult  (const ulong32 *a, const ulong32 *b, ulong32 *c);
void elpgf_571_sqr   (const ulong32 *a, ulong32 *b);
void elpgf_571_reduce(ulong32 *a);
int elpgf_571_inv   (ulong32 *a);
#endif

extern const ulong32 elpecc2_sqrtab[];
extern const elpgf2_plugin elpgf2sw_plugin;

extern elpgf2_plugin elp_gf;

int elpgf_tobin(const ulong32 *in, unsigned char *out, int idx);
int elpgf_frombin(const unsigned char *in, ulong32 *out, int idx);
int elpecc2_is_one(elpecc2_point *P, int size);
void gf_trace(ulong32 *a, int idx, ulong32 *sum);
void gf_halftrace(ulong32 *a, int idx, ulong32 *sum);

/* ---- ECC functions ---- */
extern const char *elpecc2_orders[2][5];
extern const ulong32 elpecc2_b[5][ECC2WORDS]; 
extern const elpecc2_point elpecc2_k_base[5];
extern const elpecc2_point elpecc2_r_base[5];
extern const ulong32 elpecc2_koblitz_a[5];
extern const int elpecc2_sizes[];

/* --- Key Generation --- */
int elpecc2_make_key(elpprng_state *prng, const elpprng_plugin *pprng, elpecc2_key *key, int size, int koblitz);
void elpecc2_free(elpecc2_key *key);

/* --- Import and Export keys in Ellipsys format --- */
int elpecc2_import(const unsigned char *in, unsigned long inlen, elpecc2_key *key);
int elpecc2_export(unsigned char *out, unsigned long *outlen, int type, elpecc2_key *key);

/* --- Import and Export public keys in X9.63 format --- */
int elpecc2_ansi_x963_export(unsigned char *out, unsigned long *outlen, elpecc2_key *key);
int elpecc2_ansi_x963_import(const unsigned char *in, unsigned long inlen, int koblitz, elpecc2_key *key);
#ifdef ELPECC2_LICENSE
int elpecc2_pkv(elpecc2_key *key, int *result);
#endif

/* --- Software based point mult and addition --- */
int elpecc2_pmul(elpecc2_point *P, elpecc2_point *Q, void *k, int idx, int koblitz);
int elpecc2_padd(elpecc2_point *P, elpecc2_point *Q, elpecc2_point *R, int idx, int koblitz);
int elpecc2_pdbl(elpecc2_point *P,  elpecc2_point *R, int idx, int koblitz);
int elpecc2_pnorm(elpecc2_point *P, elpecc2_point *R, int idx);
#ifdef ELPECC2_FP
int elpecc2_fp_mulmod(elpecc2_point *P, elpecc2_point *Q, void *k, int idx, int koblitz);
void elpecc2_fp_free(void);
#endif


/* Perform an ECC DH */
int elpecc2_shared_secret(elpecc2_key *private_key, elpecc2_key *public_key, unsigned char *out, unsigned long *outlen);

/* Perform EC-DSA */
int elpecc2_sign_hash(const unsigned char *in,  unsigned long inlen, 
                            unsigned char *out, unsigned long *outlen, 
                            elpprng_state *prng, const elpprng_plugin *pprng, elpecc2_key *key);
int elpecc2_verify_hash(const unsigned char *sig,  unsigned long  siglen,
                        const unsigned char *hash, unsigned long  hashlen, 
                                        int *stat,    elpecc2_key *key);

/* --- Perform EC-IES --- */
int elpecc2_ansi_kdf(const elphash_plugin *hash, 
                    const unsigned char  *seed,
                          unsigned long   seedlen,
                    const unsigned char  *shareddata,
                          unsigned long   shareddatalen,
                          unsigned char  *out,
                          unsigned long   outlen);

int elpecc2_ecies_generate(
    const elphash_plugin *hash, 
    const elpprng_plugin *prng,        elpprng_state *prngstate,
    const unsigned char  *shareddata1, unsigned long  shareddata1len,
          unsigned char  *out,         unsigned long  outlen,
          elpecc2_key     **ephkey,     elpecc2_key    *recpkey);

int elpecc2_ecies_encrypt(
    const unsigned char  *pt,          unsigned long  ptlen,
          unsigned char  **out,        unsigned long *outlen,
          unsigned long   mackeylen,
    const elphash_plugin *hash, 
    const elpprng_plugin *prng,        elpprng_state *prngstate,
    const unsigned char  *shareddata1, unsigned long  shareddata1len,
    const unsigned char  *shareddata2, unsigned long  shareddata2len,
           elpecc2_key    *recpkey);

int elpecc2_ecies_decrypt(
    const unsigned char  *ct,          unsigned long  ctlen,
          unsigned char  **out,        unsigned long *outlen,
          unsigned long   mackeylen,
    const elphash_plugin *hash, 
    const elpprng_plugin *prng,        elpprng_state *prngstate,
    const unsigned char  *shareddata1, unsigned long  shareddata1len,
    const unsigned char  *shareddata2, unsigned long  shareddata2len,
           elpecc2_key    *recpkey,               int *stat);
	
#endif
