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

#ifndef ELPSOFT_MATH_H_
#define ELPSOFT_MATH_H_

#ifndef ELPRSA
   typedef int elprsa_key;
#endif

#ifndef ELPECC
   typedef int elpecc_point;
#endif

#define ELP_MP_LT   -1
#define ELP_MP_EQ    0
#define ELP_MP_GT    1

#define ELP_MP_NO    0
#define ELP_MP_YES   1

/** math descriptor */
typedef struct {
   /** Name of the math provider */
   char *name;

   /** Bits per digit, amount of bits must fit in an unsigned long */
   int  bits_per_digit;

/* ---- init/deinit functions ---- */

   /** initialize a bignum
     @param   a     The number to initialize
     @return  CRYPT_OK on success
   */
   int (*init)(void **a);

   /** init copy
     @param  dst    The number to initialize and write to
     @param  src    The number to copy from
     @return CRYPT_OK on success
   */
   int (*init_copy)(void **dst, void *src);

   /** deinit
      @param   a    The number to free
      @return CRYPT_OK on success
   */
   void (*deinit)(void *a);

/* ---- data movement ---- */

   /** negate
      @param   src   The number to negate
      @param   dst   The destination
      @return CRYPT_OK on success
   */
   int (*neg)(void *src, void *dst);

   /** copy
      @param   src   The number to copy from
      @param   dst   The number to write to
      @return CRYPT_OK on success
   */
   int (*copy)(void *src, void *dst);

/* ---- trivial low level functions ---- */

   /** set small constant
      @param a    Number to write to
      @param n    Source upto bits_per_digit (actually meant for very small constants)
      @return CRYPT_OK on succcess
   */
   int (*set_int)(void *a, unsigned long n);

   /** get small constant
      @param a    Number to read, only fetches upto bits_per_digit from the number
      @return  The lower bits_per_digit of the integer (unsigned)
   */
   unsigned long (*get_int)(void *a);

   /** get digit n
     @param a  The number to read from
     @param n  The number of the digit to fetch
     @return  The bits_per_digit  sized n'th digit of a
   */
   unsigned long (*get_digit)(void *a, int n);

   /** Get the number of digits that represent the number
     @param a   The number to count
     @return The number of digits used to represent the number
   */
   int (*get_digit_count)(void *a);

   /** compare two integers
     @param a   The left side integer
     @param b   The right side integer
     @return ELP_MP_LT if a < b, ELP_MP_GT if a > b and ELP_MP_EQ otherwise.  (signed comparison)
   */
   int (*compare)(void *a, void *b);

   /** compare against int
     @param a   The left side integer
     @param b   The right side integer (upto bits_per_digit)
     @return ELP_MP_LT if a < b, ELP_MP_GT if a > b and ELP_MP_EQ otherwise.  (signed comparison)
   */
   int (*compare_d)(void *a, unsigned long n);

   /** Count the number of bits used to represent the integer
     @param a   The integer to count
     @return The number of bits required to represent the integer
   */
   int (*count_bits)(void * a);

   /** Count the number of LSB bits which are zero
     @param a   The integer to count
     @return The number of contiguous zero LSB bits
   */
   int (*count_lsb_bits)(void *a);

   /** Compute a power of two
     @param a  The integer to store the power in
     @param n  The power of two you want to store (a = 2^n)
     @return CRYPT_OK on success
   */
   int (*twoexpt)(void *a , int n);

/* ---- radix conversions ---- */

   /** read ascii string
     @param a     The integer to store into
     @param str   The string to read
     @param radix The radix the integer has been represented in (2-64)
     @return CRYPT_OK on success
   */
   int (*read_radix)(void *a, const char *str, int radix);

   /** write number to string
     @param a     The integer to store
     @param str   The destination for the string
     @param radix The radix the integer is to be represented in (2-64)
     @return CRYPT_OK on success
   */
   int (*write_radix)(void *a, char *str, int radix);

   /** get size as unsigned char string
     @param a     The integer to get the size (when stored in array of octets)
     @return The length of the integer
   */
   unsigned long (*unsigned_size)(void *a);

   /** store an integer as an array of octets
     @param src   The integer to store
     @param dst   The buffer to store the integer in
     @return CRYPT_OK on success
   */
   int (*unsigned_write)(void *src, unsigned char *dst);

   /** read an array of octets and store as integer
     @param dst   The integer to load
     @param src   The array of octets
     @param len   The number of octets
     @return CRYPT_OK on success
   */
   int (*unsigned_read)(void *dst, const unsigned char *src, unsigned long len);

/* ---- basic math ---- */

   /** add two integers
     @param a   The first source integer
     @param b   The second source integer
     @param c   The destination of "a + b"
     @return CRYPT_OK on success
   */
   int (*add)(void *a, void *b, void *c);


   /** add two integers
     @param a   The first source integer
     @param b   The second source integer (single digit of upto bits_per_digit in length)
     @param c   The destination of "a + b"
     @return CRYPT_OK on success
   */
   int (*addi)(void *a, unsigned long b, void *c);

   /** subtract two integers
     @param a   The first source integer
     @param b   The second source integer
     @param c   The destination of "a - b"
     @return CRYPT_OK on success
   */
   int (*sub)(void *a, void *b, void *c);

   /** subtract two integers
     @param a   The first source integer
     @param b   The second source integer
     @param c   The destination of "a - b"
     @return CRYPT_OK on success
   */
   int (*subi)(void *a, unsigned long b, void *c);

   /** multiply two integers
     @param a   The first source integer
     @param b   The second source integer (single digit of upto bits_per_digit in length)
     @param c   The destination of "a * b"
     @return CRYPT_OK on success
   */
   int (*mul)(void *a, void *b, void *c);

   /** multiply two integers
     @param a   The first source integer
     @param b   The second source integer
     @param c   The destination of "a * b"
     @return CRYPT_OK on success
   */
   int (*muli)(void *a, unsigned long b, void *c);

   /** Square an integer
     @param a    The integer to square
     @param b    The destination
     @return CRYPT_OK on success
   */
   int (*sqr)(void *a, void *b);

   /** Divide an integer
     @param a    The dividend
     @param b    The divisor
     @param c    The quotient (can be NULL to signify don't care)
     @param d    The remainder (can be NULL to signify don't care)
     @return CRYPT_OK on success
   */
   int (*mpdiv)(void *a, void *b, void *c, void *d);

   /** divide by two
      @param  a   The integer to divide (shift right)
      @param  b   The destination
      @return CRYPT_OK on success
   */
   int (*div_2)(void *a, void *b);

   /** divide by 16
      @param  a   The integer to divide (shift right)
      @param  b   The destination
      @return CRYPT_OK on success
   */
   int (*div_16)(void *a, void *b);

   /** divide by power of two
      @param  a   The integer to divide (shift right)
      @param  n   How many bits to shift right by
      @param  b   The destination
      @return CRYPT_OK on success
   */
   int (*div_2d)(void *a, int n, void *b);

   /** Get remainder (small value)
      @param  a    The integer to reduce
      @param  b    The modulus (upto bits_per_digit in length)
      @param  c    The destination for the residue
      @return CRYPT_OK on success
   */
   int (*modi)(void *a, unsigned long b, unsigned long *c);

   /** gcd
      @param  a     The first integer
      @param  b     The second integer
      @param  c     The destination for (a, b)
      @return CRYPT_OK on success
   */
   int (*gcd)(void *a, void *b, void *c);

   /** lcm
      @param  a     The first integer
      @param  b     The second integer
      @param  c     The destination for [a, b]
      @return CRYPT_OK on success
   */
   int (*lcm)(void *a, void *b, void *c);

   /** Modular multiplication
      @param  a     The first source
      @param  b     The second source
      @param  c     The modulus
      @param  d     The destination (a*b mod c)
      @return CRYPT_OK on success
   */
   int (*mulmod)(void *a, void *b, void *c, void *d);

   /** Modular squaring
      @param  a     The first source
      @param  b     The modulus
      @param  c     The destination (a*a mod b)
      @return CRYPT_OK on success
   */
   int (*sqrmod)(void *a, void *b, void *c);

   /** Modular inversion
      @param  a     The value to invert
      @param  b     The modulus
      @param  c     The destination (1/a mod b)
      @return CRYPT_OK on success
   */
   int (*invmod)(void *, void *, void *);

/* ---- reduction ---- */

   /** setup montgomery
       @param a  The modulus
       @param b  The destination for the reduction digit
       @return CRYPT_OK on success
   */
   int (*montgomery_setup)(void *a, void **b);

   /** get normalization value
       @param a   The destination for the normalization value
       @param b   The modulus
       @return  CRYPT_OK on success
   */
   int (*montgomery_normalization)(void *a, void *b);

   /** reduce a number
       @param a   The number [and dest] to reduce
       @param b   The modulus
       @param c   The value "b" from montgomery_setup()
       @return CRYPT_OK on success
   */
   int (*montgomery_reduce)(void *a, void *b, void *c);

   /** clean up  (frees memory)
       @param a   The value "b" from montgomery_setup()
       @return CRYPT_OK on success
   */
   void (*montgomery_deinit)(void *a);

/* ---- exponentiation ---- */

   /** Modular exponentiation
       @param a    The base integer
       @param b    The power (can be negative) integer
       @param c    The modulus integer
       @param d    The destination
       @return CRYPT_OK on success
   */
   int (*exptmod)(void *a, void *b, void *c, void *d);

   /** Primality testing
       @param a     The integer to test
       @param b     The destination of the result (FP_YES if prime)
       @return CRYPT_OK on success
   */
   int (*isprime)(void *a, int *b);

  /* accelerators */
   int (*rsa_me)(const unsigned char *in,   unsigned long inlen,
                       unsigned char *out,  unsigned long *outlen, int which,
                          elprsa_key *key);

   /** ECC GF(p) point multiplication (from the NIST curves)
       @param k   The integer to multiply the point by
       @param G   The point to multiply
       @param R   The destination for kG
       @param modulus  The modulus for the field
       @param map Boolean indicated whether to map back to affine or not (can be ignored if you work in affine only)
       @return CRYPT_OK on success
   */
   int (*ecc_ptmul)(void *k, elpecc_point *G, elpecc_point *R, void *modulus, int map);

   /** ECC GF(p) point addition
       @param P    The first point
       @param Q    The second point
       @param R    The destination of P + Q
       @param modulus  The modulus
       @param mp   The "b" value from montgomery_setup()
       @return CRYPT_OK on success
   */
   int (*ecc_ptadd)(elpecc_point *P, elpecc_point *Q, elpecc_point *R, void *modulus, void *mp);

   /** ECC GF(p) point double
       @param P    The first point
       @param R    The destination of 2P
       @param modulus  The modulus
       @param mp   The "b" value from montgomery_setup()
       @return CRYPT_OK on success
   */
   int (*ecc_ptdbl)(elpecc_point *P, elpecc_point *R, void *modulus, void *mp);

   /** ECC GF(p) point double-addition
       @param P    The first point
       @param Q    The second point
       @param R    The destination of 2P + Q
       @param modulus  The modulus
       @param mp   The "b" value from montgomery_setup()
       @return CRYPT_OK on success
   */
   int (*ecc_ptdbladd)(elpecc_point *P, elpecc_point *Q, elpecc_point *R, void *modulus, void *mp);

   /** ECC mapping from projective to affine, currently uses (x,y,z) => (x/z^2, y/z^3, 1)
       @param P     The point to map
       @param modulus The modulus
       @param mp    The "b" value from montgomery_setup()
       @return CRYPT_OK on success
       @remark  The mapping can be different but keep in mind a elpecc_point only has three
                integers (x,y,z) so if you use a different mapping you have to make it fit.
   */
   int (*ecc_map)(elpecc_point *P, void *modulus, void *mp);

   /** Computes kA*A + kB*B = C using Shamir's Trick
       @param A        First point to multiply
       @param kA       What to multiple A by
       @param B        Second point to multiply
       @param kB       What to multiple B by
       @param C        [out] Destination point (can overlap with A or B
       @param modulus  Modulus for curve
       @return CRYPT_OK on success
   */
   int (*ecc_mul2add)(elpecc_point *A, void *kA,
                      elpecc_point *B, void *kB,
                      elpecc_point *C,
                               void *modulus);


} elp_math_descriptor;

#ifdef ELPSOFT_MATH_FIX_PLUGIN
   #define elp_mp elpmath_desc
#else
   extern elp_math_descriptor elp_mp;
#endif

int elp_init_multi(void **a, ...);
void elp_deinit_multi(void *a, ...);

int elprand_integer(void *a, void *limit, elpprng_state *prng, const elpprng_plugin *pprng);


#ifdef ELPMATH
extern const elp_math_descriptor elpmath_desc;
#endif

#if !defined(DESC_DEF_ONLY) && defined(ELP_SOURCE)

#define MP_DIGIT_BIT                 elp_mp.bits_per_digit

/* some handy macros */
#define mp_init(a)                   elp_mp.init(a)
#define mp_init_multi                elp_init_multi
#define mp_clear(a)                  elp_mp.deinit(a)
#define mp_clear_multi               elp_deinit_multi
#define mp_init_copy(a, b)           elp_mp.init_copy(a, b)

#define mp_neg(a, b)                 elp_mp.neg(a, b)
#define mp_copy(a, b)                elp_mp.copy(a, b)

#define mp_set(a, b)                 elp_mp.set_int(a, b)
#define mp_set_int(a, b)             elp_mp.set_int(a, b)
#define mp_get_int(a)                elp_mp.get_int(a)
#define mp_get_digit(a, n)           elp_mp.get_digit(a, n)
#define mp_get_digit_count(a)        elp_mp.get_digit_count(a)
#define mp_cmp(a, b)                 elp_mp.compare(a, b)
#define mp_cmp_d(a, b)               elp_mp.compare_d(a, b)
#define mp_count_bits(a)             elp_mp.count_bits(a)
#define mp_cnt_lsb(a)                elp_mp.count_lsb_bits(a)
#define mp_2expt(a, b)               elp_mp.twoexpt(a, b)

#define mp_read_radix(a, b, c)       elp_mp.read_radix(a, b, c)
#define mp_toradix(a, b, c)          elp_mp.write_radix(a, b, c)
#define mp_unsigned_bin_size(a)      elp_mp.unsigned_size(a)
#define mp_to_unsigned_bin(a, b)     elp_mp.unsigned_write(a, b)
#define mp_read_unsigned_bin(a, b, c) elp_mp.unsigned_read(a, b, c)

#define mp_add(a, b, c)              elp_mp.add(a, b, c)
#define mp_add_d(a, b, c)            elp_mp.addi(a, b, c)
#define mp_sub(a, b, c)              elp_mp.sub(a, b, c)
#define mp_sub_d(a, b, c)            elp_mp.subi(a, b, c)
#define mp_mul(a, b, c)              elp_mp.mul(a, b, c)
#define mp_mul_d(a, b, c)            elp_mp.muli(a, b, c)
#define mp_sqr(a, b)                 elp_mp.sqr(a, b)
#define mp_div(a, b, c, d)           elp_mp.mpdiv(a, b, c, d)
#define mp_div_2(a, b)               elp_mp.div_2(a, b)
#define mp_div_16(a, b)              elp_mp.div_16(a, b)
#define mp_div_2d(a, n, b)           elp_mp.div_2d(a, n, b)
#define mp_mod(a, b, c)              elp_mp.mpdiv(a, b, NULL, c)
#define mp_mod_d(a, b, c)            elp_mp.modi(a, b, c)
#define mp_gcd(a, b, c)              elp_mp.gcd(a, b, c)
#define mp_lcm(a, b, c)              elp_mp.lcm(a, b, c)

#define mp_mulmod(a, b, c, d)        elp_mp.mulmod(a, b, c, d)
#define mp_sqrmod(a, b, c)           elp_mp.sqrmod(a, b, c)
#define mp_invmod(a, b, c)           elp_mp.invmod(a, b, c)

#define mp_montgomery_setup(a, b)    elp_mp.montgomery_setup(a, b)
#define mp_montgomery_normalization(a, b) elp_mp.montgomery_normalization(a, b)
#define mp_montgomery_reduce(a, b, c)   elp_mp.montgomery_reduce(a, b, c)
#define mp_montgomery_free(a)        elp_mp.montgomery_deinit(a)

#define mp_exptmod(a,b,c,d)          elp_mp.exptmod(a,b,c,d)
#define mp_prime_is_prime(a, b, c)   elp_mp.isprime(a, c)

#define mp_iszero(a)                 (mp_cmp_d(a, 0) == ELP_MP_EQ ? ELP_MP_YES : ELP_MP_NO)
#define mp_isodd(a)                  (mp_get_digit_count(a) > 0 ? (mp_get_digit(a, 0) & 1 ? ELP_MP_YES : ELP_MP_NO) : ELP_MP_NO)
#define mp_exch(a, b)                do { void *ABC__tmp = a; a = b; b = ABC__tmp; } while(0);

#define mp_tohex(a, b)               mp_toradix(a, b, 16)

#endif

#ifdef ELPECC
#ifdef ELPSMALLCODE

enum {
   MP_OP_SQR,
   MP_OP_MUL,
   MP_OP_ADD,
   MP_OP_SUB,
   MP_OP_D2, // divide by 2
   MP_OP_EOL
};

struct mp_program {
   unsigned char op1, op2, op3, opcode;
};

int mp_run_program(const struct mp_program *program, void **tab, void *modulus, void *mp);

int mp_sqrmont(void *a, void *c, void *m, void *mp);
int mp_mulmont(void *a, void *b, void *c, void *m, void *mp);
int mp_submont(void *a, void *b, void *c, void *modulus);
int mp_addmont(void *a, void *b, void *c, void *modulus);
#endif
#endif

#endif

