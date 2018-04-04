/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: point3f.proto */

#ifndef PROTOBUF_C_point3f_2eproto__INCLUDED
#define PROTOBUF_C_point3f_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1002001 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _Ltpb__Point3F Ltpb__Point3F;


/* --- enums --- */


/* --- messages --- */

/*
 * Structure to hold a 3 dimensional point (floating point precision)
 */
struct  _Ltpb__Point3F
{
  ProtobufCMessage base;
  float x;
  float y;
  float z;
};
#define LTPB__POINT3_F__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ltpb__point3_f__descriptor) \
    , 0, 0, 0 }


/* Ltpb__Point3F methods */
void   ltpb__point3_f__init
                     (Ltpb__Point3F         *message);
size_t ltpb__point3_f__get_packed_size
                     (const Ltpb__Point3F   *message);
size_t ltpb__point3_f__pack
                     (const Ltpb__Point3F   *message,
                      uint8_t             *out);
size_t ltpb__point3_f__pack_to_buffer
                     (const Ltpb__Point3F   *message,
                      ProtobufCBuffer     *buffer);
Ltpb__Point3F *
       ltpb__point3_f__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   ltpb__point3_f__free_unpacked
                     (Ltpb__Point3F *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Ltpb__Point3F_Closure)
                 (const Ltpb__Point3F *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor ltpb__point3_f__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_point3f_2eproto__INCLUDED */