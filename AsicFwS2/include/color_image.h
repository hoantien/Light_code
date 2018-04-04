#ifndef __COLOR_IMAGE_H__
#define __COLOR_IMAGE_H__

#ifdef __cplusplus

#include "basic_image.h"

namespace ltaf {
struct RGB10 // 10 bit pixels
{
 RGB10() :_r(0), _g(0), _b(0) {}
 RGB10(uint16_t r, uint16_t g, uint16_t b) : _r(r), _g(g), _b(b) {}
 uint16_t _r, _g, _b;

void print(){
 	printf("[%u, %u, %u]\n", (unsigned) _r, (unsigned) _g, (unsigned) _b);
 }
};

class ImageRGB10 {

public:
	ImageRGB10(std::size_t Width, std::size_t Height):
	_width(Width),
	_height(Height)
	{
		_r_channel = (Image<uint16_t> *) new Image<uint16_t>(_width, _height);
		_g_channel = (Image<uint16_t> *) new Image<uint16_t>(_width, _height);
		_b_channel = (Image<uint16_t> *) new Image<uint16_t>(_width, _height);
	}
	~ImageRGB10(void){ 
		delete _r_channel;
		delete _g_channel;
		delete _b_channel;
	}

	// assume data is of the same length (number of pixels)
	void setRData( const uint16_t * data ) {memcpy(_r_channel->begin(), data, sizeof(uint16_t) * _width * _height);}
	void setGData( const uint16_t * data ) {memcpy(_g_channel->begin(), data, sizeof(uint16_t) * _width * _height);}
	void setBData( const uint16_t * data ) {memcpy(_b_channel->begin(), data, sizeof(uint16_t) * _width * _height);}

	RGB10 getPixel(int x, int y) {return {(*_r_channel)(x, y), (*_g_channel)(x, y), (*_b_channel)(x,y)}; }
	void setPixel(int x, int y, RGB10 val) { (*_r_channel)(x, y) = val._r; (*_g_channel)(x, y) = val._g; (*_b_channel)(x, y) = val._b;}

    int length( void ) const { return _width * _height; }

    void clear( const RGB10 val = {0,0,0} ) { _r_channel->clear(val._r); _g_channel->clear(val._g); _b_channel->clear(val._b); }

    const size_t width() const {return _width;}
    const size_t height() const {return _height;}

    void print(){
    	for( size_t i = 0; i < _height; ++i ) {
    		for ( size_t j = 0; j < _width; ++j){
    			RGB10 p = getPixel(j, i);
    			p.print();
    		}
    	}
    }

private:
	size_t _width;
	size_t _height;
	// easier to do 3 layers?
	Image<uint16_t>  * _r_channel;
	Image<uint16_t>  * _g_channel;
	Image<uint16_t>  * _b_channel;
};

}

#endif

#endif