/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: point2f.proto */

#ifndef PROTOBUF_C_point2f_2eproto__INCLUDED
#define PROTOBUF_C_point2f_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1002001 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _Ltpb__Point2F Ltpb__Point2F;


/* --- enums --- */


/* --- messages --- */

struct  _Ltpb__Point2F
{
  ProtobufCMessage base;
  float x;
  float y;
};
#define LTPB__POINT2_F__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ltpb__point2_f__descriptor) \
    , 0, 0 }


/* Ltpb__Point2F methods */
void   ltpb__point2_f__init
                     (Ltpb__Point2F         *message);
size_t ltpb__point2_f__get_packed_size
                     (const Ltpb__Point2F   *message);
size_t ltpb__point2_f__pack
                     (const Ltpb__Point2F   *message,
                      uint8_t             *out);
size_t ltpb__point2_f__pack_to_buffer
                     (const Ltpb__Point2F   *message,
                      ProtobufCBuffer     *buffer);
Ltpb__Point2F *
       ltpb__point2_f__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   ltpb__point2_f__free_unpacked
                     (Ltpb__Point2F *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Ltpb__Point2F_Closure)
                 (const Ltpb__Point2F *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor ltpb__point2_f__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_point2f_2eproto__INCLUDED */
