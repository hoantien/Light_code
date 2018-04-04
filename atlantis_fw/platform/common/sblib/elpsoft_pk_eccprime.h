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
#ifdef ELPECC

#if defined(ELPECC_FP) && defined(ELPSMALLMEM)
   #error Cannot define ELPECC_FP and ELPSMALLMEM at the same time
#endif   

/* size of our temp buffers for exported keys */
#define ECC_BUF_SIZE 256

/* max private key size */
#define ECC_MAXSIZE  66

struct ecc_param {
   const unsigned char len, *data;
};

/** Structure defines a NIST GF(p) curve */
typedef struct {
   /** The size of the curve in octets */
   int size;
   
   /** number of bits in curve */
   int bits;

   /** The first two bytes to mask the length of the order of the curve */
   char mask_order[2];
   
   /** OID binding if any */
   const elp_asn1_oid *binding;

   /** name of curve */
   char *name; 

   /** The prime that defines the field the curve is in (encoded in hex) */
   struct ecc_param prime;

   /** The fields B param (hex) */
   struct ecc_param B;

   /** The order of the curve (hex) */
   struct ecc_param order;
  
   /** The x co-ordinate of the base point on the curve (hex) */
   struct ecc_param Gx;
 
   /** The y co-ordinate of the base point on the curve (hex) */
   struct ecc_param Gy;
} elpecc_set_type;

/** A point on a ECC curve, stored in Jacbobian format such that (x,y,z) => (x/z^2, y/z^3, 1) when interpretted as affine */
typedef struct {
    /** The x co-ordinate */
    void *x;

    /** The y co-ordinate */
    void *y;

    /** The z co-ordinate */
    void *z;
    
    /* a hint that the point is affine [Z==1], can be safely ignored */
    int  affine;
} elpecc_point;

/** An ECC key */
typedef struct {
    /** Type of key, PK_PRIVATE or PK_PUBLIC */
    int type;

    /** Index into the elpecc_sets[] for the parameters of this curve; if -1, then this key is using user supplied curve in dp */
    int idx;

	 /** pointer to domain parameters; either points to NIST curves (identified by idx >= 0) or user supplied curve */
	 const elpecc_set_type *dp;

    /** The public key */
    elpecc_point pubkey;

    /** The private key */
    void *k;
} elpecc_key;

/** the ECC params provided */
extern const elpecc_set_type elpecc_sets[];

int elpecc_make_key(elpprng_state *prng, const elpprng_plugin *pprng, int keysize, elpecc_key *key);
int elpecc_make_key_ex(elpprng_state *prng, const elpprng_plugin *pprng, elpecc_key *key, const elpecc_set_type *dp);

int elpecc_shared_secret(elpecc_key *private_key, elpecc_key *public_key, unsigned char *out, unsigned long *outlen);

int elpecc_sign_hash(const unsigned char *in,  unsigned long inlen, unsigned char *out, unsigned long *outlen, elpprng_state *prng, const elpprng_plugin *pprng, elpecc_key *key);
int elpecc_verify_hash(const unsigned char *sig,  unsigned long siglen, const unsigned char *hash, unsigned long hashlen, int *stat, elpecc_key *key);

/* ECIES functionality */
int elpecc_ansi_kdf(const elphash_plugin *hash, 
                    const unsigned char  *seed,
                          unsigned long   seedlen,
                    const unsigned char  *shareddata,
                          unsigned long   shareddatalen,
                          unsigned char  *out,
                          unsigned long   outlen);

int elpecc_ecies_generate(
    const elphash_plugin *hash, 
    const elpprng_plugin *prng,        elpprng_state *prngstate,
    const unsigned char  *shareddata1, unsigned long  shareddata1len,
          unsigned char  *out,         unsigned long  outlen,
          elpecc_key     **ephkey,     elpecc_key    *recpkey);

int elpecc_ecies_encrypt(
    const unsigned char  *pt,          unsigned long  ptlen,
          unsigned char  **out,        unsigned long *outlen,
          unsigned long   mackeylen,
    const elphash_plugin *hash, 
    const elpprng_plugin *prng,        elpprng_state *prngstate,
    const unsigned char  *shareddata1, unsigned long  shareddata1len,
    const unsigned char  *shareddata2, unsigned long  shareddata2len,
           elpecc_key    *recpkey);

int elpecc_ecies_decrypt(
    const unsigned char  *ct,          unsigned long  ctlen,
          unsigned char  **out,        unsigned long *outlen,
          unsigned long   mackeylen,
    const elphash_plugin *hash, 
    const elpprng_plugin *prng,        elpprng_state *prngstate,
    const unsigned char  *shareddata1, unsigned long  shareddata1len,
    const unsigned char  *shareddata2, unsigned long  shareddata2len,
           elpecc_key    *recpkey,               int *stat);

/* Import/Export */
int elpecc_ansi_x963_export(unsigned char *out, unsigned long *outlen, elpecc_key *key);
int elpecc_ansi_x963_import(const unsigned char *in, unsigned long inlen, elpecc_key *key);
int elpecc_ansi_x963_import_ex(const unsigned char *in, unsigned long inlen, elpecc_key *key, const elpecc_set_type *dp);
int elpecc_pkv(elpecc_key *key, int *result);

int elpecc_export(unsigned char *out, unsigned long *outlen, int type, elpecc_key *key);
int elpecc_import(const unsigned char *in, unsigned long inlen, elpecc_key *key);
int elpecc_import_ex(const unsigned char *in, unsigned long inlen, elpecc_key *key, const elpecc_set_type *dp);

void elpecc_free(elpecc_key *key);

int elpecc_projective_add_point(elpecc_point *P, elpecc_point *Q, elpecc_point *R, void *modulus, void *mp);
int elpecc_projective_dbl_point(elpecc_point *P, elpecc_point *R, void *modulus, void *mp);

int elpecc_map(elpecc_point *P, void *modulus, void *mp);
int elpecc_mulmod(void *k, elpecc_point *G, elpecc_point *R, void *modulus, int map);
int elpecc_load_base_point(elpecc_point *p, const elpecc_set_type *dp);

#ifdef ELPECC_SHAMIR
int elpecc_mul2add(elpecc_point *A, void *kA,
                   elpecc_point *B, void *kB,
                   elpecc_point *C,
                           void *modulus);
#endif

#ifdef ELPECC_FP
int elpecc_fp_mulmod(void *k, elpecc_point *G, elpecc_point *R, void *modulus, int map);

int elpecc_fp_mul2add(elpecc_point *A, void *kA,
                       elpecc_point *B, void *kB,
                       elpecc_point *C, void *modulus);
void elpecc_fp_free(void);
#endif

#ifdef FAST_32BIT_ECC
#include <../math/tfm.h>
extern const fp_digit elp_p192[];
extern const fp_digit elp_p224[];
extern const fp_digit elp_p256[];
extern const fp_digit elp_p384[];
extern const fp_digit elp_p521[];
#endif

int elpecc_copy_point(elpecc_point *src, elpecc_point *dest);
elpecc_point *elpecc_new_point(void);
void elpecc_del_point(elpecc_point *p);
int elpecc_is_valid_idx(int n);
#endif
