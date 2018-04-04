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
#ifdef ELPX509

typedef struct {
   elp_asn1_list
       *top,                /* pointer to the top of the SEQUENCE OF the RDN */
       *commonName,
       *surname,
       *serial,
       *countryName,        /* pointers to the first found */
       *locality,           /* These are optional elements of the RDN */
       *state,
       *organizationName,
       *organizationUnit,
       *title,
       *given,
       *initials,
       *generation,
       *distinguishedName,
       *pseudonym,
       *email;
} elpx509_name;


/* Content that will eventually be put in elpsoft_x509_crl.h .... */
enum {
   ELP_X509_CRL_EOL = 100, /* set this to 100 because we want to be higher than the X509 enums... */
   
   ELP_X509_CRL_VERSION,
   ELP_X509_CRL_SIGNATURE_IDENTIFIER,

   ELP_X509_CRL_THIS_UPDATE,
   ELP_X509_CRL_NEXT_UPDATE,
};

typedef struct elpx509_cert_crl_revokedCertificates {
   elp_asn1_list
      *userCertificate,
      *revocationDate;
   
   struct elpx509_cert_crl_revokedCertificates *prev, *next;
} elpx509_crl_revokedCertificates;   

typedef struct {
   /* TBS values */
   elp_asn1_list   
      *version,
      *signatureIdentifier;
      
   elpx509_name
      issuer;   
      
   elp_asn1_list   
      *thisUpdate,
      *nextUpdate;
     
   /* cert list inside TBS */
   elpx509_crl_revokedCertificates
      revokedCertificates;
      
   /* outside the TBS */
   elp_asn1_list 
      *signatureAlgorithm,
      *signatureValue;
} elpx509_crl_ref;

   /* [decoding] Scan an ASN1 list and produces an X509 ref */
   int elpx509_crl_scan(elp_asn1_list *rec, elpx509_crl_ref *ref);

   /* [encoding] adds fields to an X509 crl ref */
   int elpx509_crl_ref_build(elpx509_crl_ref **ref, int field, int type, void *data, unsigned long size, ...);


/*---------------------------------------*/

enum {
   ELP_X509_CERT_EOL = 0,

   ELP_X509_CERT_VERSION,
   ELP_X509_CERT_SERIAL_NUMBER,
   ELP_X509_CERT_SIGNATURE_IDENTIFIER,

   ELP_X509_CERT_ISSUER_COUNTRY_NAME,
   ELP_X509_CERT_ISSUER_ORGANIZATION_NAME,
   ELP_X509_CERT_ISSUER_DISTINGUISHED_NAME,
   ELP_X509_CERT_ISSUER_ORGANIZATION_UNIT,
   ELP_X509_CERT_ISSUER_STATE,
   ELP_X509_CERT_ISSUER_SERIAL,
   ELP_X509_CERT_ISSUER_COMMON_NAME,
   ELP_X509_CERT_ISSUER_LOCALITY,
   ELP_X509_CERT_ISSUER_TITLE,
   ELP_X509_CERT_ISSUER_SURNAME,
   ELP_X509_CERT_ISSUER_GIVEN,
   ELP_X509_CERT_ISSUER_INITIALS,
   ELP_X509_CERT_ISSUER_PSEUDONYM,
   ELP_X509_CERT_ISSUER_GENERATION,
   ELP_X509_CERT_ISSUER_EMAIL,

   ELP_X509_CERT_VALID_FROM,
   ELP_X509_CERT_VALID_TO,

   ELP_X509_CERT_SUBJECT_COUNTRY_NAME,
   ELP_X509_CERT_SUBJECT_ORGANIZATION_NAME,
   ELP_X509_CERT_SUBJECT_DISTINGUISHED_NAME,
   ELP_X509_CERT_SUBJECT_ORGANIZATION_UNIT,
   ELP_X509_CERT_SUBJECT_STATE,
   ELP_X509_CERT_SUBJECT_SERIAL,
   ELP_X509_CERT_SUBJECT_COMMON_NAME,
   ELP_X509_CERT_SUBJECT_LOCALITY,
   ELP_X509_CERT_SUBJECT_TITLE,
   ELP_X509_CERT_SUBJECT_SURNAME,
   ELP_X509_CERT_SUBJECT_GIVEN,
   ELP_X509_CERT_SUBJECT_INITIALS,
   ELP_X509_CERT_SUBJECT_PSEUDONYM,
   ELP_X509_CERT_SUBJECT_GENERATION,
   ELP_X509_CERT_SUBJECT_EMAIL,

   ELP_X509_CERT_SUBJECT_KEY_IDENTIFIER,
   ELP_X509_CERT_SUBJECT_KEY_VALUE,

   ELP_X509_CERT_ISSUER_UNIQUE_ID,
   ELP_X509_CERT_SUBJECT_UNIQUE_ID,
   ELP_X509_CERT_EXTENSIONS,

   ELP_X509_CERT_SIGNATURE_ALGORITHM,
   ELP_X509_CERT_SIGNATURE_VALUE,
};


/* these must be COPIES of what would be in the ASN1 since they are free'ed independently of anything else
   the _scan() function copies them out of the ASN1 list, but if you add them manually make sure they're copies, not references */
typedef struct elpx509_cert_ext {
   elp_asn1_list *extnID,
                 *critical,   /* may be NULL indicating default of FALSE */
                 *extnValue;

  struct elpx509_cert_ext *prev, *next;
} elpx509_cert_ext;

typedef struct {
   /* parent node, used when verifying chains */
   int parent, seen;

   elp_asn1_list
/* Fields within the TBSCertificate */
       *version,
       *serialNumber,
       *signatureIdentifier;

   elpx509_name
        issuer;

   elp_asn1_list
       *valid_from,
       *valid_to;

   elpx509_name
        subject;

   elp_asn1_list
       *subjectPublicKeyIdentifier, /* OID of key type, may have optional siblings depending on type */
       *subjectPublicKeyValue,      /* BIT STRING of key data */

       *issuerUniqueID,           /* optional BIT STRING identifiers */
       *subjectUniqueID,          /* optional BIT STRING identifiers */
       *extensionsTop;            /* start of extensions */

   elpx509_cert_ext *ext;             /* linked list of scanned extensions */

/* outside TBS */
   elp_asn1_list
       *signatureAlgorithm,
       *signatureValue;

} elpx509_cert_ref;

/* low level x509 ref functions */
   /* [decoding] Decodes ASN.1 DER into an X509 ref and ASN1 list */
   int elpx509_cert_ref_decode(const unsigned char *in, unsigned long len, elpx509_cert_ref **ref, elp_asn1_list **list);
   /* [decoding] Scan an ASN1 list and produces an X509 ref */
   int elpx509_cert_scan(elp_asn1_list *rec, elpx509_cert_ref *ref);

   /* [encoding] invokes a "build" then encodes as ASN.1 DER */
   int elpx509_cert_ref_encode(elpx509_cert_ref *ref, unsigned char **out, unsigned long *len);
   /* [encoding] builds a new ASN1 list from an X509 ref */
   int elpx509_cert_build(elp_asn1_list **rec, elpx509_cert_ref *ref);
   /* [encoding] adds fields to an X509 ref */
   int elpx509_cert_ref_build(elpx509_cert_ref **ref, int field, int type, void *data, unsigned long size, ...);
   /* [encoding] process an OpenSSL style hash of RDN fields into a ref structure */
   int elpx509_cert_sslhash_to_rdn(const char *sslhash, elpx509_cert_ref *ref, int subjectrdn);
   /* [decoding] process an RDN into an OpenSSL style hash */
   int elpx509_cert_rdn_to_sslhash(elpx509_cert_ref *ref, int subjectrdn, char **sslhash);

   /* [memory] free memory used by scan */
   int elpx509_cert_ref_free(elpx509_cert_ref *ref);
   /* [memory] frees extensions (also called by elpx509_cert_ref_free()) */
   void elpx509_cert_ref_free_ext(elpx509_cert_ref *ref);
   /* [memory] frees a ref/list pair that were allocated at the same time, used to avoid double frees (e.g. elpx509_cert_ref_decode())*/
   int elpx509_cert_ref_list_free(elpx509_cert_ref *ref, elp_asn1_list *list);
   

/* x509 key functions */
   /* import a public key out of a certificate */
   int  elpx509_cert_key_import(elpx509_cert_ref *ref, void **key, int *keytype);
   /* export a public key into a certificate */
   int  elpx509_cert_key_export(elpx509_cert_ref *ref, void *key, int keytype);
   /* free memory allocated by a key import */
   void elpx509_cert_key_free(void *key, int keytype);

/* x509 signature functions */
   /* [sign] Sign a certificate and store the signature in the X509 ref */
   int elpx509_cert_sign_cert(elpx509_cert_ref *cert, const elphash_plugin *hash, void *key, int keytype, elpprng_state *prngstate, const elpprng_plugin *prng);
   /* [verify] Verify the signature on a certificate as stored in an ASN1 list */
   int elpx509_cert_verify_cert(elp_asn1_list *cert, const elphash_plugin *hash, void *key, int keytype, int *stat);
   /* [sign] Assign an RDN from one certificate to another  (subject of signer to issuer of signee) */
   int elpx509_cert_assign_rdn(elpx509_cert_ref *subject, elpx509_cert_ref *issuer);
   /* [sign] computes a hash of an X.509 certificate in ASN1 list format */
   int elpx509_cert_hash_cert(elp_asn1_list *cert, const elphash_plugin *hash, unsigned char *md, unsigned long *outlen);

/* Low Level Extension Functions */
   /* find an extension in a ref */
   int elpx509_cert_ext_find(elpx509_cert_ref *ref, const elp_asn1_oid *oid, elpx509_cert_ext **ext);
   /* Add an extension to a ref (must NOT already be present) */
   int elpx509_cert_ext_add(elpx509_cert_ref *ref, const elp_asn1_oid *oid, const unsigned char *value, unsigned long valuelen, int critical);
   /* Delete an extension from a ref (if not present this is a NOP) */
   int elpx509_cert_ext_del(elpx509_cert_ref *ref, const elp_asn1_oid *oid);

/* extensions */
/* Key Usage Extension (4.2.1.3 of RFC 5280) */
typedef struct {
   int  present;
   char digitalSignature,
        nonRepudiation,
        keyEncipherment,
        dataEncipherment,
        keyAgreement,
        keyCertSign,
        cRLSign,
        encipherOnly,
        decipherOnly;
} elpx509_cert_ext_keyusage;

   /* get/set keyUsage extensions (4.2.1.3 of RFC 5280) */
   int elpx509_cert_ext_keyusage_get(elpx509_cert_ref *ref, elpx509_cert_ext_keyusage *ku);
   int elpx509_cert_ext_keyusage_set(elpx509_cert_ref *ref, elpx509_cert_ext_keyusage *ku);

   /* get/set basic constraints extensions (4.2.1.9 of RFC 5280) */
   int elpx509_cert_ext_basicconstraints_get(elpx509_cert_ref *ref, int *caBool, unsigned long *pathlen);
   int elpx509_cert_ext_basicconstraints_set(elpx509_cert_ref *ref, int  caBool, unsigned long  pathlen);

/* --- BIN FUNCTIONS --- */
   /* Import a public key from a DER encoded certificate */
   int elpx509_cert_key_import_bin(const unsigned char *cert, unsigned long certlen, void **key, int *keytype);
   /* Export a public key to a DER encoded certificate (note: this breaks the signature) */
   int elpx509_cert_key_export_bin(const unsigned char *cert, unsigned long certlen, unsigned char **newcert, unsigned long *newcertlen, void *key, int keytype);

   /* [verify] Verify the signature on a certificate encoded in DER format */
   int elpx509_cert_verify_cert_bin(const unsigned char *cert, unsigned long certlen, const elphash_plugin *hash, void *key, int keytype, int *stat);
   /* [verify] Verify a chain of certificates stored in ASN1 DER format */
   int elpx509_cert_verify_chain(const unsigned char **certs, unsigned long *certlens, unsigned ncerts, unsigned nReq, const elphash_plugin **hashes, unsigned nhash, int *stat);

   /* [sign] Sign a certicate in DER format using credentials from another DER encoded certificate (and a private key), storing signed cert in newly allocated DER encoded array */
   int elpx509_cert_sign_cert_bin(const unsigned char *subjectCert, unsigned long subjectCertlen,
                              const unsigned char *signingCert, unsigned long signingCertlen,
                                             void *key, int keytype,
                                    unsigned char **signedCert, unsigned long *signedCertlen,
                             const elphash_plugin *hash, elpprng_state *prngstate, const elpprng_plugin *prng);

   /* get a field out of an RDN in DER form */
   int elpx509_cert_rdn_get (const unsigned char *cert, unsigned long certlen, int field, char **value, unsigned long *len);

   /* determine the hash algorithm used to sign a certificate encoded in DER format */
   int elpx509_cert_get_hash_plugin(const unsigned char *cert, unsigned long certlen, const elphash_plugin **hashes, unsigned nhash, const elphash_plugin **hash);


#endif

#if defined(ELPX509) || defined(ELPPKCS8)
   /* handy helpers */
   int elpx509_cert_oid_to_ecc_keytype(elp_asn1_list *oid, int *type, int *size, int *koblitz, int *idx);
   int elpx509_cert_keytype_to_oid(elp_asn1_list *aoid, int type, int size, int koblitz);
#endif

