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

#ifdef ELPRC4
struct elprc4_state {
   unsigned char S[256];
   unsigned      x, y;
};
#endif

#ifdef ELPLFG
struct elplfg_state {
   unsigned char S[256];
   unsigned      x, y;
};
#endif

#ifdef ELPSNOW3G
typedef struct {
/* LFSR */
ulong32 LFSR_S0;
ulong32 LFSR_S1;
ulong32 LFSR_S2;
ulong32 LFSR_S3;
ulong32 LFSR_S4;
ulong32 LFSR_S5;
ulong32 LFSR_S6;
ulong32 LFSR_S7;
ulong32 LFSR_S8;
ulong32 LFSR_S9;
ulong32 LFSR_S10;
ulong32 LFSR_S11;
ulong32 LFSR_S12;
ulong32 LFSR_S13;
ulong32 LFSR_S14;
ulong32 LFSR_S15;
/* FSM */
ulong32 FSM_R1;
ulong32 FSM_R2;
ulong32 FSM_R3;
/* params between calls */
ulong32 t, F;
} elpsnow3g_state;
#endif

#ifdef ELPZUC
struct elpzuc_state {
   ulong32 lfsr[16], fsm[2], brc[4];
};
#endif

#ifdef ELPX931AES
typedef struct {
   unsigned char V[16];    /* 128-bit seed value */
   unsigned char DT[16];   /* 128-bit date/time vector */
   unsigned char Key[32];  /* key size 128 up to 256 bits */
   unsigned int  key_size; /* key size in bytes */
} elpx931aes_state;
#endif

#if defined(ELPAES) && defined(ELPCTRDRBG)
typedef struct ctrrbg_instantiation_info_
{   
   unsigned char *nonce;
   unsigned long nonce_length;
   unsigned char *personalization_string;
   unsigned long personalization_string_length;
   unsigned char *additonal_input;
   unsigned long additonal_input_length;
} ctrrbg_seed_info;

typedef struct ctrrbg_generate_info_
{
   unsigned char *entropy_input;
   unsigned long entropy_input_length;   
   unsigned char *additonal_input;
   unsigned long additonal_input_length;      
   int prediction_resistance_request;
} ctrrbg_generate_info;

typedef struct 
{
  int bstart;
  int binst_complete;
  ctrrbg_seed_info seed_info;
  ctrrbg_generate_info gen_info;
  unsigned char v[16];
  unsigned char key[16];
  unsigned char seed[32];  
  int prediction_resistance_flag;
  int use_df;
  int mode;
  int security_strength_length;
  int reseed_flag;
  unsigned long reseed_ctr;
  unsigned long reseed_interval;
  elpcipher_plugin const* aes_cipher;
  elpsymmetric_key aes_key;
  elpsymmetric_ctr aes_ctr;
  int b_aes_ctr_set;
  elpsymmetric_key aes_df_cbc_key;
  elpsymmetric_cbc aes_df_cbc;   
  int b_aes_df_cbc_set;
  elpsymmetric_key block_key;
} ctrdrbg_state;
#endif

#ifdef ELP_TRNG
typedef struct 
{
  unsigned long base_addr;
  void* trng_info;
  int bstart;
  int trng_error;
} trng_state;
#endif

typedef union
{
   void *data;  
#ifdef ELPLFG
   struct elplfg_state lfg;
#endif
#ifdef ELPRC4
   struct elprc4_state rc4;
#endif
#ifdef ELPSNOW3G
   elpsnow3g_state     snow3g;
#endif
#ifdef ELPZUC
   struct elpzuc_state zuc;
#endif
#ifdef ELPX931AES
   elpx931aes_state x931aes;
#endif
#if defined(ELPAES) && defined(ELPCTRDRBG)
   ctrdrbg_state ctrdrgb;
#endif
#ifdef ELP_TRNG
   trng_state trng;
#endif
} elpprng_state;

/** PRNG descriptor */
typedef struct elp_prng_plugin {
    /** Name of the PRNG */
    char *name;
    /** Start a PRNG state
        @param prng   [out] The state to initialize
        @return ELPEOK if successful
    */
    int (*start)(elpprng_state *prng);
    /** Add entropy to the PRNG
        @param in         The entropy
        @param inlen      Length of the entropy (octets)\
        @param prng       The PRNG state
        @return ELPEOK if successful
    */
    int (*add_entropy)(const unsigned char *in, unsigned long inlen, elpprng_state *prng);
    /** Ready a PRNG state to read from
        @param prng       The PRNG state to ready
        @return ELPEOK if successful
    */
    int (*ready)(elpprng_state *prng);
    /** Read from the PRNG
        @param out     [out] Where to store the data
        @param outlen  Length of data desired (octets)
        @param prng    The PRNG state to read from
        @return Number of octets read
    */
    unsigned long (*read)(unsigned char *out, unsigned long outlen, elpprng_state *prng);
    /** Terminate a PRNG state
        @param prng   The PRNG state to terminate
        @return ELPEOK if successful
    */
    int (*done)(elpprng_state *prng);
    /** Self-test the PRNG
        @return ELPEOK if successful
    */
    int (*test)(void);
} elpprng_plugin;

#ifdef ELPSNOW3G
extern const elpprng_plugin elpsnow3g_plugin;
int elpsnow3g_start(elpprng_state *prng);
int elpsnow3g_add_entropy(const unsigned char *in, unsigned long inlen, elpprng_state *prng);
int elpsnow3g_ready(elpprng_state *prng);
unsigned long elpsnow3g_read(unsigned char *out, unsigned long outlen, elpprng_state *prng);
int elpsnow3g_done(elpprng_state *prng);
int  elpsnow3g_test(void);
#endif

#ifdef ELPZUC
extern const elpprng_plugin elpzuc_plugin;
int elpzuc_start(elpprng_state *prng);
int elpzuc_add_entropy(const unsigned char *in, unsigned long inlen, elpprng_state *prng);
int elpzuc_ready(elpprng_state *prng);
unsigned long elpzuc_read(unsigned char *out, unsigned long outlen, elpprng_state *prng);
int elpzuc_done(elpprng_state *prng);
int elpzuc_test(void);
#endif

#ifdef ELPSPRNG
extern const elpprng_plugin elpsprng_plugin;
#endif

#ifdef ELPRC4
extern const elpprng_plugin elprc4_plugin;
int elprc4_start(elpprng_state *prng);
int elprc4_add_entropy(const unsigned char *in, unsigned long inlen, elpprng_state *prng);
int elprc4_ready(elpprng_state *prng);
unsigned long elprc4_read(unsigned char *out, unsigned long outlen, elpprng_state *prng);
int elprc4_done(elpprng_state *prng);
int  elprc4_test(void);
#endif

#ifdef ELPLFG
/* This PRNG is for TESTING only and is not secure */
extern const elpprng_plugin elplfg_plugin;
int elplfg_start(elpprng_state *prng);
int elplfg_add_entropy(const unsigned char *in, unsigned long inlen, elpprng_state *prng);
int elplfg_ready(elpprng_state *prng);
unsigned long elplfg_read(unsigned char *out, unsigned long outlen, elpprng_state *prng);
int elplfg_done(elpprng_state *prng);
int  elplfg_test(void);
#endif


#ifdef ELPX931AES
extern const elpprng_plugin elpx931aes_plugin;
int elpx931aes_start(elpprng_state *prng);
int elpx931aes_add_entropy(const unsigned char *in, unsigned long inlen, elpprng_state *prng);
int elpx931aes_ready(elpprng_state *prng);
unsigned long elpx931aes_read(unsigned char *out, unsigned long outlen, elpprng_state *prng);
int elpx931aes_done(elpprng_state *prng);
int  elpx931aes_test(void);
#endif
