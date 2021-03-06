/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: dead_pixel_map.proto */

#ifndef PROTOBUF_C_dead_5fpixel_5fmap_2eproto__INCLUDED
#define PROTOBUF_C_dead_5fpixel_5fmap_2eproto__INCLUDED

#include <protobuf-c/protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1000000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1002001 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _Ltpb__DeadPixelMap Ltpb__DeadPixelMap;


/* --- enums --- */


/* --- messages --- */

/*
 * structure holding hot pixel map data
 */
struct  _Ltpb__DeadPixelMap
{
  ProtobufCMessage base;
  /*
   * offset to the data       
   */
  uint64_t data_offset;
  /*
   * size of the data in bytes
   */
  uint32_t data_size;
  /*
   * Pixels are marked dead in the sensor if the value is below the black_level_threshold.
   */
  protobuf_c_boolean has_black_level_threshold;
  float black_level_threshold;
};
#define LTPB__DEAD_PIXEL_MAP__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&ltpb__dead_pixel_map__descriptor) \
    , 0, 0, 0,0 }


/* Ltpb__DeadPixelMap methods */
void   ltpb__dead_pixel_map__init
                     (Ltpb__DeadPixelMap         *message);
size_t ltpb__dead_pixel_map__get_packed_size
                     (const Ltpb__DeadPixelMap   *message);
size_t ltpb__dead_pixel_map__pack
                     (const Ltpb__DeadPixelMap   *message,
                      uint8_t             *out);
size_t ltpb__dead_pixel_map__pack_to_buffer
                     (const Ltpb__DeadPixelMap   *message,
                      ProtobufCBuffer     *buffer);
Ltpb__DeadPixelMap *
       ltpb__dead_pixel_map__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   ltpb__dead_pixel_map__free_unpacked
                     (Ltpb__DeadPixelMap *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Ltpb__DeadPixelMap_Closure)
                 (const Ltpb__DeadPixelMap *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor ltpb__dead_pixel_map__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_dead_5fpixel_5fmap_2eproto__INCLUDED */
