/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: distortion.proto */

#ifndef PROTOBUF_C_distortion_2eproto__INCLUDED
#define PROTOBUF_C_distortion_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1002001 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif

#include "point2f.pb-c.h"

typedef struct _Ltpb__Distortion Ltpb__Distortion;
typedef struct _Ltpb__Distortion__Polynomial Ltpb__Distortion__Polynomial;
typedef struct _Ltpb__Distortion__CRA Ltpb__Distortion__CRA;


/* --- enums --- */


/* --- messages --- */

/*
 * Polynomial model of distortion       
 */
struct  _Ltpb__Distortion__Polynomial
{
  ProtobufCMessage base;
  /*
   * Distortion center in sensor pixels
   */
  Ltpb__Point2F *distortion_center;
  /*
   * Normalization x and y in sensor pixels
   */
  Ltpb__Point2F *normalization;
  /*
   * polynomial coefficients
   */
  size_t n_coeffs;
  float *coeffs;
  /*
   * Fit cost
   */
  protobuf_c_boolean has_fit_cost;
  float fit_cost;
};
#define LTPB__DISTORTION__POLYNOMIAL__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ltpb__distortion__polynomial__descriptor) \
    , NULL, NULL, 0,NULL, 0,0 }


/*
 * CRA model of distortion
 */
struct  _Ltpb__Distortion__CRA
{
  ProtobufCMessage base;
  /*
   * Distortion center in sensor pixels
   */
  Ltpb__Point2F *distortion_center;
  /*
   * Distance in mm
   */
  float sensor_distance;
  /*
   * Distance in mm
   */
  float exit_pupil_distance;
  /*
   * in mm
   */
  float pixel_size;
  /*
   * pairs of radial distance in pixels from distortion center vs CRA
   */
  size_t n_cra;
  Ltpb__Point2F **cra;
  /*
   * 
   */
  size_t n_coeffs;
  Ltpb__Point2F **coeffs;
  /*
   * Fit cost
   */
  protobuf_c_boolean has_fit_cost;
  float fit_cost;
};
#define LTPB__DISTORTION__CRA__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ltpb__distortion__cra__descriptor) \
    , NULL, 0, 0, 0, 0,NULL, 0,NULL, 0,0 }


/*
 * Main distortion model container
 */
struct  _Ltpb__Distortion
{
  ProtobufCMessage base;
  Ltpb__Distortion__Polynomial *polynomial;
  Ltpb__Distortion__CRA *cra;
};
#define LTPB__DISTORTION__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ltpb__distortion__descriptor) \
    , NULL, NULL }


/* Ltpb__Distortion__Polynomial methods */
void   ltpb__distortion__polynomial__init
                     (Ltpb__Distortion__Polynomial         *message);
/* Ltpb__Distortion__CRA methods */
void   ltpb__distortion__cra__init
                     (Ltpb__Distortion__CRA         *message);
/* Ltpb__Distortion methods */
void   ltpb__distortion__init
                     (Ltpb__Distortion         *message);
size_t ltpb__distortion__get_packed_size
                     (const Ltpb__Distortion   *message);
size_t ltpb__distortion__pack
                     (const Ltpb__Distortion   *message,
                      uint8_t             *out);
size_t ltpb__distortion__pack_to_buffer
                     (const Ltpb__Distortion   *message,
                      ProtobufCBuffer     *buffer);
Ltpb__Distortion *
       ltpb__distortion__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   ltpb__distortion__free_unpacked
                     (Ltpb__Distortion *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Ltpb__Distortion__Polynomial_Closure)
                 (const Ltpb__Distortion__Polynomial *message,
                  void *closure_data);
typedef void (*Ltpb__Distortion__CRA_Closure)
                 (const Ltpb__Distortion__CRA *message,
                  void *closure_data);
typedef void (*Ltpb__Distortion_Closure)
                 (const Ltpb__Distortion *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor ltpb__distortion__descriptor;
extern const ProtobufCMessageDescriptor ltpb__distortion__polynomial__descriptor;
extern const ProtobufCMessageDescriptor ltpb__distortion__cra__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_distortion_2eproto__INCLUDED */