#ifndef __BASIC_IMAGE_H__
#define __BASIC_IMAGE_H__

#ifdef __cplusplus

#include <cstring>
#include <type_traits>
#include <cstdlib>
#include <cstdint>
#include <memory>
#include "operator_new.h"
#include "line2.h"
#include "image_pixel_iter.h"

namespace ltaf {


template<typename T>
class Image {

public:

	Image(){}

	Image(std::size_t width, std::size_t height):
#ifdef ASIC_NUM
	_data(new(Heap::DDR) T[width*height]),
#else
	_data( new T[width * height]),
#endif
	_width(width),
	_height(height)
	{
		std::memset(_data.get(), 0, sizeof(T) * _width * _height);
	}

	// assume data is of the same length (number of pixels)
	void setData( const T * data ) { std::memcpy(_data.get(), data, sizeof(T) * _width * _height); }
	void resize(std::uint16_t width, std::uint16_t height) { 
		_width = width; 
		_height = height;
#ifdef ASIC_NUM
		_data.reset(new(Heap::DDR) T[width*height]);
#else
		_data.reset(new T[width*height]);
#endif
		std::memset(_data.get(), 0, sizeof(T) * width * height);
	}
	const T * begin() const { return _data.get(); }
    const T * end() const { return _data.get() + _width * _height; }


    T    &    operator []( int index )       { return _data[index]; }
    const T & operator []( int index ) const { return _data[index]; }

    T    &    operator()( int x, int y )       { return _data[y * _width + x]; }
    const T & operator()( int x, int y ) const { return _data[y * _width + x]; }

    int length( void ) const { return _width * _height; }

    void clear( const T scalar = 0 ) { for( std::size_t i = 0; i < _width * _height; ++i ) { _data[i] = scalar; } }

    void emptify() {_width = 0; _height = 0; _data.reset(); }

    const std::size_t width() const {return _width;}
    const std::size_t height() const {return _height;}

private:
	std::unique_ptr<T[]> _data;
	std::size_t _width  = 0;
	std::size_t _height = 0;
};

using ImageU16 		= 	Image<std::uint16_t>;
using ImageF 		= 	Image<float>;
using ImageRGB16 	=	Image<Vec3u16>;


// generic patch search using SAD metric.
template<typename T>
bool findPatch(const Image<T> & patch, const Image<T> & img, int & offset_x, int & offset_y);

// assume each pixel is 10 bit max, then we require no more than 15 bits 
// for querying sum of 32 pixels or less.
ImageU16 computeIntegralImage(const ImageU16 & img);
ImageRGB16 computeIntegralImage(const ImageRGB16 & img); //explicitly do this for vec3u16. avoid narrowing

// img: destination to store collapsed img
// line_number: count of line being passed in
// line_start: pixel iterator for the current line
// npix_per_line: number of pixels to read from each line
// collapse_factor: how many pixels to average over
void collapseImage(ImageU16 & img, std::uint16_t line_number, image_pixel_iter line_start, std::uint16_t npix_per_line, std::uint16_t collapse_factor);

// given an image that's 10 bit max per element, compute integral image in place, with overflow
void computeIntegralImageInPlace(ImageU16 & img);

// feature is float, due to the need for normalization
// divide patch into 8x8 cells and compute features
template<typename T, typename F> // T is image, U is output aggregate / feature type
void computeFeaturesGeneral(const Image<T> & int_img, std::uint16_t start_x, std::uint16_t start_y, // where the starting point is to compute
							std::uint16_t patch_width, std::uint16_t patch_height,	// size of the patch over which to compute feature
							std::uint16_t cell_width, std::uint16_t cell_height,	// size of cell, must not be more than 64 pixels, try 8x8 usually
							F * features);											// preallocated space for features, must have enough memory


// compute feautures with early abort:
// return early if curr sum is larger than curr best cost
// update the best results if able to
// target feature is what is to be matched.
// returns true if an update is successful (found new best cost)
template<typename T, typename F>
bool computeFeaturesGeneralAndUpdate(const Image<T> & int_img, std::uint16_t start_x, std::uint16_t start_y, // where the starting point is to compute
								std::uint16_t patch_width, std::uint16_t patch_height,	// size of the patch over which to compute feature
								std::uint16_t cell_width, std::uint16_t cell_height,	// size of cell, must not be more than 64 pixels, try 8x8 usually
								const F * target_feature,
								std::uint16_t & best_offset_x, std::uint16_t & best_offset_y, float & best_cost );

std::uint16_t getAreaSum2DIntImg( const Image<std::uint16_t> & intimg,
                                   const int start_x, const int start_y,
                                   const int end_x, const int end_y );
Vec3u16 getAreaSum2DIntImg( const ImageRGB16 & intimg,
                                   const int start_x, const int start_y,
                                   const int end_x, const int end_y );


// if factor is int, then it degenerates to simple decimation for now
// if float, do bilinear interpolation
void downsample(const ImageU16 & src, ImageU16 & dst, float factor);

} // end namespace ltaf

#endif

#endif