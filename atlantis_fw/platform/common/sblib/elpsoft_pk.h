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

enum {
   ELP_PK_PUBLIC=0,
   ELP_PK_PRIVATE=1
};

#ifdef ELPSMALLASN1

int elpder_small_encode(unsigned char *out, unsigned long *outlen, void **numbers, unsigned long numlen);
int elpder_small_decode(const unsigned char *out, unsigned long outlen, void **numbers, unsigned long *numlen);

#endif

#ifdef ELPASN1

enum {
 ELP_ASN1_UNIVERSAL,
 ELP_ASN1_APPLICATION,
 ELP_ASN1_CONTEXT_SPECIFIC,
 ELP_ASN1_PRIVATE
};

enum {
 ELP_ASN1_EOL,
 ELP_ASN1_BOOLEAN,
 ELP_ASN1_INTEGER,
 ELP_ASN1_SHORT_INTEGER,
 ELP_ASN1_BIT_STRING,
 ELP_ASN1_OCTET_STRING,
 ELP_ASN1_NULL,
 ELP_ASN1_OBJECT_IDENTIFIER,
 ELP_ASN1_IA5_STRING,
 ELP_ASN1_PRINTABLE_STRING,
 ELP_ASN1_UTF8_STRING,
 ELP_ASN1_UTCTIME,
 ELP_ASN1_CHOICE,
 ELP_ASN1_SEQUENCE,
 ELP_ASN1_SET,
 ELP_ASN1_SETOF,
 ELP_ASN1_CONSTRUCTED_TAG,
 ELP_ASN1_VERBATIM,
};

/** A ELP ASN.1 list type */
typedef struct elp_asn1_list {
   /** The ASN.1 class */
   int           asn1_class;
   /** The ELP ASN.1 enumerated type identifier */
   int           type;
   /** The data to encode or place for decoding */
   void         *data;
   /** The size of the input or resulting output */
   unsigned long size;
   /** The used flag, this is used by the CHOICE ASN.1 type to indicate which choice was made */
   int           used;
   /** cont tag to use */
   int           cont;
   /** prev/next entry in the list */
   struct elp_asn1_list *prev, *next, *child, *parent;

   /** Internal use only */
   const unsigned char *start_coding,
                       *end_coding;
} elp_asn1_list;

#define ELP_SET_ASN1(list, index, Type, Data, Size)  \
   do {                                              \
      int ELP_MACRO_temp            = (index);       \
      elp_asn1_list *ELP_MACRO_list = (list);        \
      ELP_MACRO_list[ELP_MACRO_temp].type = (Type);  \
      ELP_MACRO_list[ELP_MACRO_temp].data = (void*)(Data);  \
      ELP_MACRO_list[ELP_MACRO_temp].size = (Size);  \
      ELP_MACRO_list[ELP_MACRO_temp].used = 0;       \
   } while (0);

unsigned long elpder_boolean_length(void);
int elpder_boolean_encode(int bit, unsigned char *out, unsigned long *outlen);
int elpder_boolean_decode(const unsigned char *in, unsigned long inlen, int *out);

unsigned long elpder_integer_length(void *num);
int elpder_integer_encode(void *num, unsigned char *out, unsigned long *outlen);
int elpder_integer_decode(const unsigned char *in, unsigned long inlen, void *num);

unsigned long elpder_null_length(void);
int elpder_null_encode(unsigned char *out, unsigned long *outlen);
int elpder_null_decode(const unsigned char *in, unsigned long inlen);

unsigned long elpder_bit_string_length(unsigned long inlen);
int elpder_bit_string_encode(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
int elpder_bit_string_decode(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);

unsigned long elpder_octet_string_length(unsigned long inlen);
int elpder_octet_string_encode(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
int elpder_octet_string_decode(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);

unsigned long elpder_ia5_string_length(const unsigned char *octets, unsigned long noctets);
int elpder_ia5_string_encode(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
int elpder_ia5_string_decode(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
int elpder_ia5_value_decode(int v);
int elpder_ia5_char_encode(int c);

unsigned long elpder_printable_string_length(const unsigned char *octets, unsigned long noctets);
int elpder_printable_string_encode(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
int elpder_printable_string_decode(const unsigned char *in, unsigned long inlen, unsigned char *out, unsigned long *outlen);
int elpder_printable_value_decode(int v);
int elpder_printable_char_encode(int c);

unsigned long elpder_utf8_charsize(const unsigned long c);
unsigned long elpder_utf8_string_length(const unsigned long *in, unsigned long noctets);
int elpder_utf8_string_encode(const unsigned long *in,  unsigned long inlen, unsigned char *out, unsigned long *outlen);
int elpder_utf8_string_decode(const unsigned char *in,  unsigned long inlen, unsigned long *out, unsigned long *outlen);


typedef struct {
   unsigned YY, /* year */
            MM, /* month */
            DD, /* day */
            hh, /* hour */
            mm, /* minute */
            ss, /* second */
            off_dir, /* timezone offset direction 0 == +, 1 == - */
            off_hh, /* timezone offset hours */
            off_mm; /* timezone offset minutes */
} elp_utctime;

unsigned long elpder_utctime_length(elp_utctime *utctime);
int elpder_utctime_decode(const unsigned char *in, unsigned long *inlen, elp_utctime *out);
int elpder_utctime_encode(elp_utctime *utctime, unsigned char *out, unsigned long *outlen);

unsigned long elpder_object_identifier_length(unsigned long *words, unsigned long nwords);
int elpder_object_identifier_encode(unsigned long *words, unsigned long  nwords,
                                    unsigned char *out,   unsigned long *outlen);
int elpder_object_identifier_decode(const unsigned char *in,    unsigned long  inlen,
                                          unsigned long *words, unsigned long *outlen);

unsigned long elpder_sequence_length(elp_asn1_list *list, unsigned long inlen);
int elpder_sequence_ex_encode(elp_asn1_list *list, unsigned long inlen,
                           unsigned char *out,  unsigned long *outlen, int type_of);
#define elpder_sequence_encode(in, inlen, out, outlen) elpder_sequence_ex_encode(in, inlen, out, outlen, ELP_ASN1_SEQUENCE)
int elpder_sequence_ex_decode(const unsigned char *in, unsigned long  inlen,
                           elp_asn1_list *list,     unsigned long  outlen, int ordered);
#define elpder_sequence_decode(in, inlen, list, outlen) elpder_sequence_ex_decode(in, inlen, list, outlen, ELP_ASN1_SEQUENCE)
int elpder_sequence_decode_flexi(const unsigned char *in, unsigned long *inlen, elp_asn1_list **out);
int elpder_sequence_encode_flexi(elp_asn1_list *in, unsigned char **out, unsigned long *outlen);
void elpder_sequence_free(elp_asn1_list *in);

int elpder_encode_sequence_multi(unsigned char *out, unsigned long *outlen, ...);
int elpder_decode_sequence_multi(const unsigned char *in, unsigned long inlen, ...);
int elpder_make_list_array(elp_asn1_list **out, ...);

#define elpder_set_decode(in, inlen, list, outlen) elpder_sequence_ex_decode(in, inlen, list, outlen, 0)
#define elpder_set_length der_sequence_length
int elpder_set_encode(elp_asn1_list *list, unsigned long inlen, unsigned char *out,  unsigned long *outlen);
int elpder_setof_encode(elp_asn1_list *list, unsigned long inlen, unsigned char *out,  unsigned long *outlen);

typedef struct {
  const unsigned char *psegment,                    /* current psegment */
                      *end;                        /* end of encoding (or NULL for indefinite) */
  int            segment_done,                     /* are we at the end of this psegment? */
                 done;                             /* are we done the encoding */
  unsigned long  segment_position,                 /* current psegment length */
                 segment_length;                   /* current psegment position */
} elpber_getinfo;

int elpber_get_init(const unsigned char *in, unsigned long len, elpber_getinfo *gi);
int elpber_get_byte(elpber_getinfo *gi);

unsigned long elpber_fetch_length(const unsigned char *in, unsigned long inlen);

int elp_asn1_populateNode(elp_asn1_list *rec, int type, void *data, unsigned long size);
int elp_asn1_addNode(elp_asn1_list **rec, int type, void *data, unsigned long size);
int elp_asn1_addChild(elp_asn1_list **rec, int type, void *data, unsigned long size);

int elp_asn1_find_objid(elp_asn1_list *rec, elp_asn1_oid *oids, int num_oids);
int elp_asn1_oidcmp(const elp_asn1_oid *a, const elp_asn1_oid *b);

#ifdef ELPECC
   #ifdef ELPECC160
   extern const elp_asn1_oid oid_ansiX9p160r1;
   #endif
   #ifdef ELPECC192
   extern const elp_asn1_oid oid_ansiX9p192r1;
   #endif
   #ifdef ELPECC224
   extern const elp_asn1_oid oid_ansiX9p224r1;
   #endif
   #ifdef ELPECC256
   extern const elp_asn1_oid oid_ansiX9p256r1;
   #endif
   #ifdef ELPECC384
   extern const elp_asn1_oid oid_ansiX9p384r1;
   #endif
   #ifdef ELPECC521
   extern const elp_asn1_oid oid_ansiX9p521r1;
   #endif
#endif

#ifdef ELPECC2
   #ifdef ELPECC2_163
   extern const elp_asn1_oid
       oid_ansiX9t163k1,
       oid_ansiX9t163r2;
   #endif
   #ifdef ELPECC2_233
   extern const elp_asn1_oid
       oid_ansiX9t233k1,
       oid_ansiX9t233r1;
   #endif
   #ifdef ELPECC2_283
   extern const elp_asn1_oid
       oid_ansiX9t283k1,
       oid_ansiX9t283r1;
   #endif
   #ifdef ELPECC2_409
   extern const elp_asn1_oid
       oid_ansiX9t409k1,
       oid_ansiX9t409r1;
   #endif
   #ifdef ELPECC2_571
   extern const elp_asn1_oid
       oid_ansiX9t571k1,
       oid_ansiX9t571r1;
   #endif
#endif

extern const elp_asn1_oid
#if defined(ELPX509) || defined(ELPPKCS8)
    oid_rsaEncryption,
    oid_ecPublicKey,
    oid_ecDHKey,
#endif

#ifdef ELPMD2
    oid_md2Hash,
#endif
#ifdef ELPMD5
    oid_md5Hash,
#endif
#ifdef ELPSHA1
    oid_sha1Hash,
#endif
#ifdef ELPSHA224
    oid_sha224Hash,
#endif
#ifdef ELPSHA256
    oid_sha256Hash,
#endif
#ifdef ELPSHA384
    oid_sha384Hash,
#endif
#ifdef ELPSHA512
    oid_sha512Hash,
#endif
#ifdef ELPSHA512_224
    oid_sha512_224Hash,
#endif
#ifdef ELPSHA512_256
    oid_sha512_256Hash,
#endif

#ifdef ELPRSA
   #ifdef ELPMD2
       oid_md2WithRSAEncryption,
   #endif
   #ifdef ELPMD5
       oid_md5WithRSAEncryption,
   #endif
   #ifdef ELPSHA1
       oid_sha1WithRSAEncryption,
   #endif
   #ifdef ELPSHA224
       oid_sha224WithRSAEncryption,
   #endif
   #ifdef ELPSHA256
       oid_sha256WithRSAEncryption,
   #endif
   #ifdef ELPSHA384
       oid_sha384WithRSAEncryption,
   #endif
   #ifdef ELPSHA512
       oid_sha512WithRSAEncryption,
   #endif
#endif

#if defined(ELPECC) || defined(ELPECC2)
   #ifdef ELPSHA1
       oid_ecdsaWithSHA1,
   #endif
   #ifdef ELPSHA224
       oid_ecdsaWithSHA224,
   #endif
   #ifdef ELPSHA256
       oid_ecdsaWithSHA256,
   #endif
   #ifdef ELPSHA384
       oid_ecdsaWithSHA384,
   #endif
   #ifdef ELPSHA512
       oid_ecdsaWithSHA512,
   #endif
#endif    

#ifdef ELPX509
    oid_common_name,
    oid_surname,
    oid_serial_number,
    oid_country_name,
    oid_locality_name,
    oid_state,
    oid_organization_name,
    oid_organization_unit_name,
    oid_title_name,
    oid_given_name,
    oid_initials,
    oid_generational_name,
    oid_distinguished_name,
    oid_pseudonym_name,
    oid_email_addr,

    oid_x509v3_private_key_usage_period,
    oid_x509v3_subject_alt_name,
    oid_x509v3_issuer_alt_name,
    oid_x509v3_crl_number,
    oid_x509v3_reason_code,
    oid_x509v3_instruction_code,
    oid_x509v3_invalidity_date,
    oid_x509v3_delta_crl_indicator,
    oid_x509v3_issuing_distribution_point,
    oid_x509v3_certificate_issuer,
    oid_x509v3_name_constraints,
    oid_x509v3_crl_distribution_points,
    oid_x509v3_certificate_policies,
    oid_x509v3_policy_mappings,
    oid_x509v3_policy_constraints,
    oid_x509v3_ext_key_usage,
    oid_x509v3_authority_attribute_identifier,
    oid_x509v3_role_spec_cert_identifier,
    oid_x509v3_crl_stream_identifier,
    oid_x509v3_basic_att_constraints,
    oid_x509v3_subject_key_identifier,
    oid_x509v3_key_usage,
    oid_x509v3_basic_constraints,
    oid_x509v3_authority_key_identifier,
#endif

#if defined(ELPPKCS7)
    oid_pkcs7_data,
    oid_pkcs7_signedData,
    oid_pkcs7_envelopedData,
    oid_pkcs7_signedAndEnvelopedData,
    oid_pkcs7_digestedData,
    oid_pkcs7_encryptedData,
#endif

#if defined(ELPPKCS9)
    oid_pkcs9_at_emailAddress,
    oid_pkcs9_at_unstructuredName,
    oid_pkcs9_at_contentType,
    oid_pkcs9_at_messageDigest,
    oid_pkcs9_at_signingTime,
    oid_pkcs9_at_counterSignature,
    oid_pkcs9_at_challengePassword,
    oid_pkcs9_at_unstructuredAddress,
    oid_pkcs9_at_extendedCertificateAttributes,

    oid_pkcs9_at_signingDescription,
    oid_pkcs9_at_extensionRequest,
    oid_pkcs9_at_smimeCapabilities,

    oid_pkcs9_at_friendlyName,
    
#endif    

#if defined(ELPPKCS5) && defined(ELPPKCS8)
    oid_pbe_md2_des_cbc,
    oid_pbe_md2_rc2_cbc,
    oid_pbe_md5_des_cbc,
    oid_pbe_sha1_des_cbc,
    oid_pbe_md5_rc2_cbc,
    oid_pbe_sha1_rc2_cbc,
    oid_pkcs5_pbkdf2,
    oid_pkcs5_pbes2,
    oid_pkcs5_pbkdf2_sha1,
    oid_pkcs5_pbkdf2_sha224,
    oid_pkcs5_pbkdf2_sha256,
    oid_pkcs5_pbkdf2_sha384,
    oid_pkcs5_pbkdf2_sha512,
#endif

#ifdef ELPTDES
    oid_des_cbc,
    oid_des3_ede_cbc,
#endif

#ifdef ELPAES
    oid_aes128_ecb,
    oid_aes128_cbc,
    oid_aes128_ofb,
    oid_aes128_cfb,
    oid_aes192_ecb,
    oid_aes192_cbc,
    oid_aes192_ofb,
    oid_aes192_cfb,
    oid_aes256_ecb,
    oid_aes256_cbc,
    oid_aes256_ofb,
    oid_aes256_cfb,
#endif
    oid_last_list;    

typedef struct {
   const elp_asn1_oid *binding,
                      *hash;
   int           keytype;
} elp_asn1_x509_sig_oid_binding;

typedef struct {
   const elp_asn1_oid *binding;
   int                 keytype,
                       keysize,
                       koblitz;
} elp_asn1_ecc_list;

typedef struct {
   const elp_asn1_oid *binding;
   const char         *name;
   int                 keysize,
                       mode;
} elp_asn1_cipher_binding;

typedef struct {
   const elp_asn1_oid *binding,
                      *hash;
} elp_asn1_pbkdf2_binding;


extern const elp_asn1_oid *oid_list[];
extern const elp_asn1_x509_sig_oid_binding oid_binding_list[];
extern const elp_asn1_ecc_list ecc_oid_list[];
extern const elp_asn1_cipher_binding cipher_binding_list[];
extern const elp_asn1_pbkdf2_binding pbkdf2_binding_list[];

#endif

#ifdef ELPPKCS1

int elppkcs_1_mgf1(
   const elphash_plugin      *hash,
   const unsigned char *seed,unsigned long seedlen,
         unsigned char *mask, unsigned long masklen);

int elppkcs1_v15_decode(
   const unsigned char *in,
         unsigned long  inlen,
         unsigned char *out,
         unsigned long *outlen,
         unsigned long  modulus_bytelen,
                   int  mode);

int elppkcs1_v15_encode(
   const unsigned char *in,
         unsigned long  inlen,
         unsigned char *out,
         unsigned long  modulus_bytelen,
                   int  mode,
        const elpprng_plugin *prng,
        elpprng_state  *prngstate);

int elppkcs_1_oaep_encode(const unsigned char *msg,    unsigned long msglen,
                       const unsigned char *lparam, unsigned long lparamlen,
                             unsigned long  modulus_bitlen,
                            elpprng_state  *prngstate, const elpprng_plugin *prng,
                            const elphash_plugin *hash,
                             unsigned char *out,    unsigned long *outlen);

int elppkcs_1_oaep_decode(const unsigned char *msg,    unsigned long msglen,
                       const unsigned char *lparam, unsigned long lparamlen,
                             unsigned long  modulus_bitlen,
                            const elphash_plugin *hash,
                             unsigned char *out,    unsigned long *outlen,
                             int           *res);

int elppkcs_1_pss_encode(const unsigned char *msghash, unsigned long msghashlen,
                            unsigned long  saltlen,
                            elpprng_state *prngstate, const elpprng_plugin *prng,
                         const elphash_plugin *phash,
                            unsigned long  modulus_bitlen,
                            unsigned char *out,     unsigned long *outlen);

int elppkcs_1_pss_decode(const unsigned char *msghash,  unsigned long  msghashlen,
                      const unsigned char *sig,      unsigned long  siglen,
                            unsigned long saltlen,  const elphash_plugin *phash,
                            unsigned long modulus_bitlen, int    *res);
#endif

#ifdef ELPX931

int elpx931_encode(const elphash_plugin *hash, const unsigned char *md, unsigned long mdlen, unsigned char *out, unsigned long IRlen);
int elpx931_decode(const elphash_plugin *hash, const unsigned char *md, unsigned long mdlen, unsigned char *sig, unsigned long IRlen, int *stat);

#endif

#ifdef ELPRSA

enum {
   PKCS1_ENCRYPT,
   PKCS1_SIGN
};

enum {
   ELP_PKCS_1_V1_5,
   ELP_PKCS_1_OAEP,
   ELP_PKCS_1_EME,
   ELP_PKCS_1_PSS,
   ELP_PKCS_1_EMSA,
   ELP_ANSI_X931_SLOW,
   ELP_ANSI_X931_FAST
};

int rand_prime(void *N, long len, elpprng_state *prngstate, const elpprng_plugin *prng);

/* ---- RSA ---- */
/* Min and Max RSA key sizes (in bits) */
#define MIN_RSA_SIZE 1024
#define MAX_RSA_SIZE 4096

/* Default e value */
#define RSA_E_DEFAULT 65537

/** RSA PKCS style key */
typedef struct {
    /** Type of key, ELP_PK_PRIVATE or ELP_PK_PUBLIC */
    int type;
    /** The public exponent */
    void *e;
    /** The private exponent */
    void *d;
    /** The modulus */
    void *N;
    /** The p factor of N */
    void *p;
    /** The q factor of N */
    void *q;
    /** The 1/q mod p CRT param */
    void *qP;
    /** The d mod (p - 1) CRT param */
    void *dP;
    /** The d mod (q - 1) CRT param */
    void *dQ;
} elprsa_key;

int elprsa_make_key(elpprng_state *prngstate, const elpprng_plugin *prng, int size, long e, elprsa_key *key);

int elprsa_exptmod(const unsigned char *in,   unsigned long inlen,
                         unsigned char *out,  unsigned long *outlen, int which,
                            elprsa_key *key);

int elprsa_sign_hash_ex(const unsigned char *in,       unsigned long  inlen,
                              unsigned char *out,      unsigned long *outlen,
                              int            padding,
                              elpprng_state *prngstate, const elpprng_plugin *prng,
                              const elphash_plugin *hash, unsigned long  saltlen,
                              elprsa_key *key);

int elprsa_verify_hash_ex(const unsigned char *sig,      unsigned long siglen,
                          const unsigned char *hash,     unsigned long hashlen,
                                int            padding,
                          const elphash_plugin *phash, unsigned long saltlen,
                                int           *stat,     elprsa_key      *key);

int elprsa_encrypt_key_ex(const unsigned char *in,     unsigned long inlen,
                                unsigned char *out,    unsigned long *outlen,
                          const unsigned char *lparam, unsigned long lparamlen,
                          elpprng_state *prngstate, const elpprng_plugin *prng, const elphash_plugin *hash, int padding, elprsa_key *key);

int elprsa_decrypt_key_ex(const unsigned char *in,       unsigned long  inlen,
                                unsigned char *out,      unsigned long *outlen,
                          const unsigned char *lparam,   unsigned long  lparamlen,
                          const elphash_plugin *hash,     int            padding,
                                int           *stat,     elprsa_key    *key);


int elprsa_export(unsigned char *out, unsigned long *outlen, int type, elprsa_key *key);
int elprsa_import(const unsigned char *in, unsigned long inlen, elprsa_key *key);
void elprsa_free(elprsa_key *key);

/* support for OpenSSL "genrsa" crypto */
int elp_ssl_bytestokey(const elpcipher_plugin *cipher, unsigned keylen,
                       const elphash_plugin   *hash,
                          const unsigned char *data,   unsigned datal,
                          const unsigned char *salt,
                                     unsigned  count,
                                unsigned char *key,    unsigned char *iv);


#if defined(ELPSMALLASN1) || defined(ELPASN1)
struct elp_pkcs1_data_table {
    const char *hash;
    unsigned long derlen;
    unsigned char derhdr[20];
};
extern const struct elp_pkcs1_data_table elp_pkcs1_headers[];
#endif

#ifdef ELPCBC

/* Markers */
#define ELP_SSLRSA_START     "-----BEGIN RSA PRIVATE KEY-----"
#define ELP_SSLRSA_START_LEN 31
#define ELP_SSLRSA_END       "-----END RSA PRIVATE KEY-----"
#define ELP_SSLRSA_END_LEN   29
#define ELP_SSLRSA_DEK       "DEK-Info:"
#define ELP_SSLRSA_DEK_LEN   9

int elp_ssl_import_rsa(const unsigned char *in,         unsigned long inlen,
                       const unsigned char *passwd,     unsigned long pwlen,
                       const elpcipher_plugin **cipher, unsigned long ncipher,
                       const elphash_plugin   *hash,
                       elprsa_key *rsakey);

int elp_ssl_export_rsa(const unsigned char    *passwd,  unsigned long  pwlen,
                       const elpcipher_plugin *cipher,  unsigned long  keylen,
                       const elphash_plugin   *hash,
                       const elpprng_plugin   *pprng,   elpprng_state *sprng,
                                  elprsa_key  *rsakey,
                             unsigned char   **out,     unsigned long *outlen);

typedef struct {
   char *binding;
   int   binding_length;
   char *ciphername;
   unsigned   keylen;
} elp_SSLRSA_binding;
extern const elp_SSLRSA_binding elp_sslrsa_binding_list[];

#endif



int elprand_prime(void *N, long len, elpprng_state *prng, const elpprng_plugin *pprng);
#endif

#ifdef ELPDSA

/* Max diff between group and modulus size in bytes */
#define MDSA_DELTA     512

/* Max DSA group size in bytes (default allows 4k-bit groups) */
#define MDSA_MAX_GROUP 512

/** DSA key structure */
typedef struct {
   /** The key type, PK_PRIVATE or PK_PUBLIC */
   int type;

   /** The order of the sub-group used in octets */
   int qord;

   /** The generator  */
   void *g;

   /** The prime used to generate the sub-group */
   void *q;

   /** The large prime that generats the field the contains the sub-group */
   void *p;

   /** The private key */
   void *x;

   /** The public key */
   void *y;
} elpdsa_key;

int elpdsa_make_key(elpprng_state *prngstate, const elpprng_plugin *prng, int group_size, int modulus_size, elpdsa_key *key);
void elpdsa_free(elpdsa_key *key);

int elpdsa_import(const unsigned char *in, unsigned long inlen, elpdsa_key *key);
int elpdsa_export(unsigned char *out, unsigned long *outlen, int type, elpdsa_key *key);

int elpdsa_shared_secret(void          *private_key, void *base,
                            elpdsa_key *public_key,
                         unsigned char *out,
                         unsigned long *outlen);

int elpdsa_sign_hash_raw(const unsigned char *in,  unsigned long  inlen,
                                        void *r,            void *s,
                               elpprng_state *prngstate,
                        const elpprng_plugin *prng,   elpdsa_key *key);

int elpdsa_verify_hash_raw(           void *r,             void *s,
                       const unsigned char *hash, unsigned long  hashlen,
                                       int *stat,    elpdsa_key *key);

int elpdsa_sign_hash(const unsigned char *in,  unsigned long  inlen,
                           unsigned char *out, unsigned long *outlen,
                           elpprng_state *prngstate,
                    const elpprng_plugin *prng,   elpdsa_key *key);

int elpdsa_verify_hash(const unsigned char *sig,  unsigned long  siglen,
                       const unsigned char *hash, unsigned long  hashlen,
                                       int *stat,    elpdsa_key *key);


int elpdsa_verify_key(elpdsa_key *key, int *stat);

int elpdsa_encrypt_key(const unsigned char *in,   unsigned long inlen,
                             unsigned char *out,  unsigned long *outlen,
                             elpprng_state *prngstate,
                      const elpprng_plugin *prng,
                      const elphash_plugin *hash,
                                elpdsa_key *key);

int elpdsa_decrypt_key(const unsigned char *in,  unsigned long  inlen,
                             unsigned char *out, unsigned long *outlen,
                      const elphash_plugin *hash,   elpdsa_key *key);
#endif

#ifdef ELPDH

#define ELPDH_PUBKEY  1
#define ELPDH_PRIVKEY 2
#define ELPDH_PARAMS  3

typedef struct {
   int type;
   
   void *g,
        *p,
        *y,
        *x;
} elpdh_key;

int elpdh_make_params(elpprng_state *prngstate, const elpprng_plugin *prng, int modulus_size, elpdh_key *key);

int elpdh_make_key(elpprng_state *prngstate, const elpprng_plugin *prng, elpdh_key *params, elpdh_key *key);
void elpdh_free(elpdh_key *key);

// generic functions for import/export, exporting as a pubkey will be in the format
// TLS requires for ServerDHParams
int elpdh_import(const unsigned char *in, unsigned long inlen, elpdh_key *key);
int elpdh_export(unsigned char *out, unsigned long *outlen, int type, elpdh_key *key);

// import/export client DH params for TLS
int elpdh_export_client(unsigned char *out, unsigned long *outlen, elpdh_key *key);
int elpdh_import_client(const unsigned char *in, unsigned long inlen, elpdh_key *params, elpdh_key *key);

int elpdh_shared_secret(elpdh_key *private_key, 
                        elpdh_key *public_key,
                         unsigned char *out,
                         unsigned long *outlen);
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

enum {
   ELP_CERT_RSAKEY = 0,
   ELP_CERT_ECCKEY,
   ELP_CERT_ECC2KEY,
   ELP_CERT_DSAKEY,
};

#include "elpsoft_pk_eccprime.h"
#include "elpsoft_pk_eccbinary.h"
#include "elpsoft_pk_x509.h"

#ifdef ELPPKCS8
int elp_pkcs8_import(const unsigned char *indata, unsigned long inlen,
                                   void **key,    int *keytype);

int elp_pkcs8_export(         void  *key,           int  keytype,
                     unsigned char  *out, unsigned long *outlen);

#ifdef ELPPKCS5
enum {
   ELP_PBES1 = 0,
   ELP_PBES2 = 1
};

int elp_pkcs8_import_enc(const unsigned char *indata, unsigned long inlen,
                         const unsigned char *passwd, unsigned long pwlen,
                         const elpcipher_plugin **ciphers, unsigned ncipher,
                         const elphash_plugin   **hashes,  unsigned nhash,
                                        void  **key,   int *keytype);

int elp_pkcs8_export_enc(void                *key,                 int  keytype,
                         const unsigned char *passwd,    unsigned long  pwlen,
                                         int  pbeversion,          int  iterations,
                         const elpprng_plugin   *pprng,  elpprng_state *sprng,
                         const elpcipher_plugin *cipher, unsigned long  keylen, int ciphermode,
                         const elphash_plugin   *hash,
                                 unsigned char **out,    unsigned long *outlen);

#define elp_pkcs8_export_enc_aes128(key, keytype, passwd, pwlen, out, outlen) elp_pkcs8_export_enc(key, keytype, passwd, pwlen, ELP_PBES2, 0x800, &elpsprng_plugin, NULL, &elpaes_plugin, 16, ELPCBC_MODE, &elpsha1_plugin, out, outlen)
#define elp_pkcs8_export_enc_aes192(key, keytype, passwd, pwlen, out, outlen) elp_pkcs8_export_enc(key, keytype, passwd, pwlen, ELP_PBES2, 0x800, &elpsprng_plugin, NULL, &elpaes_plugin, 24, ELPCBC_MODE, &elpsha1_plugin, out, outlen)
#define elp_pkcs8_export_enc_aes256(key, keytype, passwd, pwlen, out, outlen) elp_pkcs8_export_enc(key, keytype, passwd, pwlen, ELP_PBES2, 0x800, &elpsprng_plugin, NULL, &elpaes_plugin, 32, ELPCBC_MODE, &elpsha1_plugin, out, outlen)
#define elp_pkcs8_export_enc_des3(key, keytype, passwd, pwlen, out, outlen)   elp_pkcs8_export_enc(key, keytype, passwd, pwlen, ELP_PBES2, 0x800, &elpsprng_plugin, NULL, &elptdes_plugin, 24, ELPCBC_MODE, &elpsha1_plugin, out, outlen)

/* OpenSSL default is PBES1 with md5WithDES-CBC */
#define elp_pkcs8_export_enc_ossl(key, keytype, passwd, pwlen, out, outlen)   elp_pkcs8_export_enc(key, keytype, passwd, pwlen, ELP_PBES1, 0x800, &elpsprng_plugin, NULL, &elpdes_plugin, 8, ELPCBC_MODE, &elpmd5_plugin, out, outlen)


int elp_pkcs8_encrypt(const unsigned char *in,        unsigned long inlen,
                      const unsigned char *passwd,    unsigned long pwlen,
                                      int  pbeversion,          int iterations,
                      const elpprng_plugin   *pprng,  elpprng_state *sprng,
                      const elpcipher_plugin *cipher, unsigned long keylen, int ciphermode,
                      const elphash_plugin   *hash,
                            unsigned char **out,   unsigned long *outlen);

int elp_pkcs8_decrypt(const unsigned char *indata, unsigned long inlen,
                      const unsigned char *passwd, unsigned long pwlen,
                      const elpcipher_plugin **ciphers, unsigned ncipher,
                      const elphash_plugin   **hashes,  unsigned nhash,
                            unsigned char **out,   unsigned long *outlen);

#endif
#endif

/* base64 */
int elp_base64_decode(const unsigned char *in,  unsigned long inlen,
                            unsigned char *out, unsigned long *outlen);
int elp_base64_encode(const unsigned char *in,  unsigned long inlen,
                            unsigned char *out, unsigned long *outlen);

