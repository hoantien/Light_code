#include <cassert>
#include <limits>
#include <cmath>
#include "log.h"
#include "os.h"
#include "basic_image.h"

#define SLOGF_ID                SLOG_ID_AF

namespace ltaf {


static std::uint16_t bilinear_interpolate(const ImageU16 & img, float x, float y ){
	assert(x < img.width() - 1 && y < img.height() - 1);

	std::uint16_t px = std::floor(x);
	std::uint16_t py = std::floor(y);
	float fx = x - px;
	float fy = y - py;
	float fx1 = 1.0f - fx;
	float fy1 = 1.0f - fy;

	std::uint16_t p1 = img(px, py);
	std::uint16_t p2 = img(px + 1, py);
	std::uint16_t p3 = img(px, py + 1);
	std::uint16_t p4 = img(px + 1, py + 1);

	return 	std::uint16_t( 	p1 * fx1 * fy1 
						+	p2 * fx * fy1
						+ 	p3 * fx1 * fy
						+	p4 * fx * fy);
}




/// compute difference between two elements, float or vec4
template <typename T>
__inline__ typename std::enable_if < ( std::is_same<T, ltaf::Vec3f>::value ||
                                         std::is_same<T, ltaf::Vec3u16>::value), float >::type
computeDiff( const T & a, const T & b ) //avoid overflow
{
    return    std::fabs( static_cast<float>(a.x()) - static_cast<float>(b.x()) )
    		+ std::fabs( static_cast<float>(a.y()) - static_cast<float>(b.y()) )
    		+ std::fabs( static_cast<float>(a.z()) - static_cast<float>(b.z()) );
}

template <typename T>
__inline__ typename std::enable_if < ( std::is_same<T, float>::value ||
									   std::is_same<T, uint16_t>::value ), float>::type
computeDiff( const T & a, const T & b )
{
    return std::fabs( static_cast<float>(a) - static_cast<float>(b) );
}

// internally use float for accumulation... this could be slow.
template<typename T>
bool findPatch(const Image<T> & patch, const Image<T> & img, int & offset_x, int & offset_y){
	if (patch.width() > img.width() || patch.height()> img.height()){
		offset_x = -1; offset_y = -1;
		return false;
	}

	if (patch.width() == img.width() && patch.height() == img.height()){
		offset_x = 0; offset_y = 0;
		return true;
	}
	size_t x_end = img.width() - patch.width();
	size_t y_end = img.height() - patch.height();
	float min_cost = std::numeric_limits<float>::max();
	float curr_cost = 0;
	for (size_t x_start = 0; x_start <= x_end; ++x_start){
		for (size_t y_start = 0; y_start<= y_end; ++y_start){
			// compute sum.
			curr_cost = 0;
			// todo: do rolling diffs for speedup
			for (size_t r = 0; r< patch.height(); ++r){
				for (size_t c = 0; c<patch.width(); ++c){
			//		curr_cost += fabs(static_cast<float>(patch(c, r)) - static_cast<float>(img(c+x_start, r+y_start)));
					curr_cost += computeDiff(patch(c, r), img(c+x_start, r+y_start));
				}
			}
			if (curr_cost < min_cost){
				min_cost = curr_cost;
				offset_y = y_start;
				offset_x = x_start;
			}
		}
	}
	return true;
}

template bool findPatch(const Image<float> & patch, const Image<float> & img, int & offset_x, int & offset_y);
template bool findPatch(const Image<uint16_t> & patch, const Image<uint16_t> & img, int & offset_x, int & offset_y);
template bool findPatch(const Image<Vec3u16> & patch, const Image<Vec3u16> & img, int & offset_x, int & offset_y);




// for integral images over 10 bit pixel images, the number of bits required 
// for maximum of 8x8 query areas (64 pixel sum) is 
// ceil( log2 [(2^10-1) * 8 * 8  + 1] ) = ceil (15.9986...) = 16!
// so we can use 16 bit unsigned int safely for 10 bit images
ImageU16 computeIntegralImage(const ImageU16 & img){
	size_t width = img.width();
	size_t height = img.height();
	ImageU16 int_img(width, height);

	int_img(0, 0) = img(0, 0);

	// first row
	for (size_t c = 1; c<width; ++c){
		int_img(c, 0) = img(c, 0) + int_img(c-1, 0);
	}
	// first col
	for (size_t r = 1; r<height; ++r){
		int_img(0, r) = img(0, r) + int_img(0, r - 1);
	}

	for (size_t r = 1; r<height; ++r){
		for (size_t c = 1; c<width; ++c){
			int_img(c, r) = img(c, r) + int_img(c - 1, r) + int_img(c, r-1) - int_img(c-1, r-1);
		}
	}
	return int_img;
}

// explicitly implement this with uint16. vec3 operations default to common type (int), leads to narrowing
ImageRGB16 computeIntegralImage(const ImageRGB16 & img){
	size_t width = img.width();
	size_t height = img.height();
	ImageRGB16 int_img(width, height);

	int_img(0, 0) = img(0, 0);

	// first row
	for (size_t c = 1; c<width; ++c){
		int_img(c, 0).setX( img(c, 0).x() + int_img(c-1, 0).x() );
		int_img(c, 0).setY( img(c, 0).y() + int_img(c-1, 0).y() );
		int_img(c, 0).setZ( img(c, 0).z() + int_img(c-1, 0).z() );
	}
	// first col
	for (size_t r = 1; r<height; ++r){
		int_img(0, r).setX( img(0, r).x() + int_img(0, r - 1).x() );
		int_img(0, r).setY( img(0, r).y() + int_img(0, r - 1).y() );
		int_img(0, r).setZ( img(0, r).z() + int_img(0, r - 1).z() );
	}

	for (size_t r = 1; r<height; ++r){
		for (size_t c = 1; c<width; ++c){
			int_img(c, r).setX( img(c, r).x() + int_img(c - 1, r).x() + int_img(c, r-1).x() - int_img(c-1, r-1).x());
			int_img(c, r).setY( img(c, r).y() + int_img(c - 1, r).y() + int_img(c, r-1).y() - int_img(c-1, r-1).y());
			int_img(c, r).setZ( img(c, r).z() + int_img(c - 1, r).z() + int_img(c, r-1).z() - int_img(c-1, r-1).z());
		}
	}
	return int_img;
}

// img: destination to store collapsed img
// line_number: count of line being passed in
// line_start: pixel iterator for the current line
// npix_per_line: number of pixels to read from each line
// collapse_factor: how many pixels to average over
void collapseImage(ImageU16 & img, std::uint16_t line_number, image_pixel_iter line_start, std::uint16_t npix_per_line, std::uint16_t collapse_factor)
{
	// we will be reading img.width() * collapse_factor pixel values per row
	assert(npix_per_line >= img.width() * collapse_factor);

	std::uint16_t destination_row = line_number / collapse_factor; // which row in the dst image to write to.
	std::uint16_t norm_factor = collapse_factor * collapse_factor; // how much to downweight each pixel value

	for (std::uint16_t col = 0; col<img.width(); col++){
		for (std::uint16_t cc = 0; cc<collapse_factor; ++cc){ // read and aggregate the next few pixels
			img(col, destination_row) += line_start.next_pixel_value() / norm_factor;
		}
	}
}


// given an image that's 10 bit max per element, compute integral image in place, with overflow
void computeIntegralImageInPlace(ImageU16 & img){
	std::uint16_t width = img.width();
	std::uint16_t height = img.height();
	// first row
	for ( std::uint16_t c = 1; c<width; ++c){
		img(c, 0) = img(c, 0) + img(c-1, 0);
	}
	// first col
	for ( std::uint16_t r = 1; r<height; ++r){
		img(0, r) = img(0, r) + img(0, r - 1);
	}

	for (std::uint16_t r = 1; r<height; ++r){
		for (std::uint16_t c = 1; c<width; ++c){
			img(c, r) = img(c, r) + img(c - 1, r) + img(c, r-1) - img(c-1, r-1);
		}
	}
}




// computes the sum of pixels in the area defined by roi
// inclusive of start / end
// return 16 bit, assume sum is always less than 16 bit, query area cannot be too large
std::uint16_t getAreaSum2DIntImg( const Image<std::uint16_t> & intimg,
                                   const int start_x, const int start_y,
                                   const int end_x, const int end_y )
{
    // base case
    if( start_y == 0 && start_x == 0 ) { return intimg( end_x, end_y ); }
    // right - left
    else if( start_y == 0 ) { return intimg( end_x, end_y ) - intimg( start_x - 1, end_y ); }
    // bottom - top
    else if( start_x == 0 ) { return intimg( end_x, end_y ) - intimg( end_x, start_y - 1 ); }
    // general case
    else
        return intimg( end_x, end_y ) - intimg( end_x, start_y - 1 ) 
    		 + intimg( start_x - 1, start_y - 1 ) - intimg( start_x - 1, end_y );
}

Vec3u16 getAreaSum2DIntImg( const ImageRGB16 & intimg,
                                   const int start_x, const int start_y,
                                   const int end_x, const int end_y )
{
    // base case
    if( start_y == 0 && start_x == 0 ) { return intimg( end_x, end_y ); }
    // right - left
    else if( start_y == 0 ) { 
    	return Vec3u16( intimg( end_x, end_y ).x() - intimg( start_x - 1, end_y ).x(),
    					intimg( end_x, end_y ).y() - intimg( start_x - 1, end_y ).y(),
    					intimg( end_x, end_y ).z() - intimg( start_x - 1, end_y ).z() ); 
    }
    // bottom - top
    else if( start_x == 0 ) { 
    	return Vec3u16( intimg( end_x, end_y ).x() - intimg( end_x, start_y - 1 ).x(),
    					intimg( end_x, end_y ).y() - intimg( end_x, start_y - 1 ).y(),
    					intimg( end_x, end_y ).z() - intimg( end_x, start_y - 1 ).z() ); 
    }
    // general case
    else {
        return Vec3u16( intimg( end_x, end_y ).x() - intimg( end_x, start_y - 1 ).x() 
    		 + intimg( start_x - 1, start_y - 1 ).x() - intimg( start_x - 1, end_y ).x(),

    		 intimg( end_x, end_y ).y() - intimg( end_x, start_y - 1 ).y() 
    		 + intimg( start_x - 1, start_y - 1 ).y() - intimg( start_x - 1, end_y ).y(),

    		 intimg( end_x, end_y ).z() - intimg( end_x, start_y - 1 ).z() 
    		 + intimg( start_x - 1, start_y - 1 ).z() - intimg( start_x - 1, end_y ).z());
    }
}


// feature is float, due to the need for normalization
// divide patch into cells and compute features
template<typename T, typename F> // T is image, U is output aggregate / feature type
void computeFeaturesGeneral(const Image<T> & int_img, std::uint16_t start_x, std::uint16_t start_y, // where the starting point is to compute
							std::uint16_t patch_width, std::uint16_t patch_height,	// size of the patch over which to compute feature
							std::uint16_t cell_width, std::uint16_t cell_height,	// size of cell, must not be more than 64 pixels, try 8x8 usually
							F * features){

	assert(cell_width * cell_height <= 64); // avoid overflowing twice
	assert(start_x + patch_width <= int_img.width() && start_y + patch_height <= int_img.height() );

	std::uint16_t write_pos = 0;

	// precompute offsets for within each cell
    std::uint16_t width_half = cell_width / 2;
    std::uint16_t height_half = cell_height / 2;
    std::uint16_t width_third = cell_width / 3;
    std::uint16_t height_third = cell_height / 3;
    std::uint16_t width_2thirds = width_third + width_third;
    std::uint16_t height_2thirds = height_third + height_third;
    std::uint16_t heightminus1 = cell_height - 1;
    std::uint16_t widthminus1 = cell_width - 1;

    float N = ( float )( cell_height * cell_width );
    float N_half = N / 2.0f;
    float N_third = N / 3.0f;
    float N_quarter = N / 4.0f;

    std::uint16_t n_cells_x = patch_width / cell_width; // ignore the trailing cols and rows if need be
    std::uint16_t n_cells_y = patch_height / cell_height;
    assert(n_cells_x > 0 && n_cells_y > 0);

	// for each cell
	for (std::uint16_t y_id = 0; y_id < n_cells_y; ++y_id ){
		for (std::uint16_t x_id = 0; x_id < n_cells_x; ++x_id ){

			// this is the top left point of the current cell
			std::uint16_t min_x = x_id * cell_width + start_x;
			std::uint16_t min_y = y_id * cell_height + start_y;

		    std::uint16_t xmid = min_x + width_half; // mid point
		    std::uint16_t ymid = min_y + height_half; // mid point
		    std::uint16_t xmid1 = min_x + width_third; // 1/3 point
		    std::uint16_t xmid2 = min_x + width_2thirds; // 2/3 point
		    std::uint16_t ymid1 = min_y + height_third; //1/3 point
		    std::uint16_t ymid2 = min_y + height_2thirds; //2/3 point
		    std::uint16_t max_x = min_x + widthminus1;
		    std::uint16_t max_y = min_y + heightminus1;

			// add the following features per cell
			// left - right
			features[write_pos++] = (F(getAreaSum2DIntImg(int_img, min_x,     min_y, xmid - 1, max_y))  	// left
								    -F(getAreaSum2DIntImg(int_img, xmid, min_y, max_x,     max_y)))/N_half;	// right
			// top - bottom
			features[write_pos++] = (F(getAreaSum2DIntImg(int_img, min_x, min_y,     max_x, ymid - 1))		// top
								    -F(getAreaSum2DIntImg(int_img, min_x, ymid, max_x, max_y)))/N_half;	// bottom
			// left + right - mid
			features[write_pos++] = (F(getAreaSum2DIntImg(int_img, min_x,     min_y, xmid1 - 1, max_y)) // left
								   -F(getAreaSum2DIntImg(int_img, xmid1, min_y, xmid2 - 1, max_y)) 		// mid
								   +F(getAreaSum2DIntImg(int_img, xmid2, min_y, max_x,     max_y)) )/N_third; // right
			// top + bottom - mid
			features[write_pos++] = ( F(getAreaSum2DIntImg(int_img, min_x, min_y,     max_x, ymid1 - 1)) // top
								   -F(getAreaSum2DIntImg(int_img, min_x, ymid1, max_x, ymid2 - 1)) // mid
								   +F(getAreaSum2DIntImg(int_img, min_x, ymid2, max_x, max_y)) )/N_third; // bot
			// diag - diag
			features[write_pos++] = (F(getAreaSum2DIntImg(int_img, min_x,		min_y,     xmid - 1, 	ymid - 1)) // top left	
								    +F(getAreaSum2DIntImg(int_img, xmid, 	   	ymid, 		max_x, 		max_y)) // bot right	
								    -F(getAreaSum2DIntImg(int_img, min_x,		ymid, 		xmid - 1, 	max_y)) // bot left	
								    -F(getAreaSum2DIntImg(int_img, xmid, 	min_y, 	   		max_x, 		ymid - 1 )))/N_quarter; // top right					   
		}
	}
}
// only deal with single channel collapsed rgb for now.
template void computeFeaturesGeneral(const ImageU16 & int_img, std::uint16_t start_x, std::uint16_t start_y, // where the starting point is to compute
									std::uint16_t patch_width, std::uint16_t patch_height,	// size of the patch over which to compute feature
									std::uint16_t cell_width, std::uint16_t cell_height,	// size of cell, must not be more than 64 pixels, try 8x8 usually
									float * features);

//#define DEBUG_LOG_COST
#ifndef DEBUG_LOG_COST
#define EARLY_RETURN_OPT
#endif

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
								std::uint16_t & best_offset_x, std::uint16_t & best_offset_y, float & best_cost) {

	assert(cell_width * cell_height <= 64); // avoid overflowing twice
	assert(start_x + patch_width <= int_img.width() && start_y + patch_height <= int_img.height() );

	std::uint16_t index = 0;

	float curr_cost = 0;

	// precompute offsets for within each cell
    std::uint16_t width_half = cell_width / 2;
    std::uint16_t height_half = cell_height / 2;
    std::uint16_t width_third = cell_width / 3;
    std::uint16_t height_third = cell_height / 3;
    std::uint16_t width_2thirds = width_third + width_third;
    std::uint16_t height_2thirds = height_third + height_third;
    std::uint16_t heightminus1 = cell_height - 1;
    std::uint16_t widthminus1 = cell_width - 1;

    float N = cell_height * cell_width;
    float N_half = N / 2.0f;
    float N_third = N / 3.0f;
    float N_quarter = N / 4.0f;

    std::uint16_t n_cells_x = patch_width / cell_width; // ignore the trailing cols and rows if need be
    std::uint16_t n_cells_y = patch_height / cell_height;
    assert(n_cells_x > 0 && n_cells_y > 0);
	// for each cell
	for (std::uint16_t y_id = 0; y_id < n_cells_y; ++y_id ){
		for (std::uint16_t x_id = 0; x_id < n_cells_x; ++x_id ){

			// this is the top left point of the current cell
			std::uint16_t min_x = x_id * cell_width + start_x;
			std::uint16_t min_y = y_id * cell_height + start_y;

		    std::uint16_t xmid = min_x + width_half; // mid point
		    std::uint16_t ymid = min_y + height_half; // mid point
		    std::uint16_t xmid1 = min_x + width_third; // 1/3 point
		    std::uint16_t xmid2 = min_x + width_2thirds; // 2/3 point
		    std::uint16_t ymid1 = min_y + height_third; //1/3 point
		    std::uint16_t ymid2 = min_y + height_2thirds; //2/3 point
		    std::uint16_t max_x = min_x + widthminus1;
		    std::uint16_t max_y = min_y + heightminus1;

			// add the following features per cell, 8x8 = 64 pixels each
			// left - right
			curr_cost += computeDiff( target_feature[index++], 
									(F(getAreaSum2DIntImg(int_img, min_x,     min_y, xmid - 1, max_y))  	// left
								    -F(getAreaSum2DIntImg(int_img, xmid, min_y, max_x,     max_y)))/N_half);	// right
#ifdef EARLY_RETURN_OPT
			if (curr_cost > best_cost)
		        return false;
#endif
			// top - bottom
			curr_cost += computeDiff( target_feature[index++],
									(F(getAreaSum2DIntImg(int_img, min_x, min_y,     max_x, ymid - 1))		// top
								    -F(getAreaSum2DIntImg(int_img, min_x, ymid, max_x, max_y)))/N_half);	// bottom
#ifdef EARLY_RETURN_OPT
            if (curr_cost > best_cost)
                return false;
#endif
			// left + right - mid
			curr_cost += computeDiff( target_feature[index++],
								(F(getAreaSum2DIntImg(int_img, min_x,     min_y, xmid1 - 1, max_y)) // left
								   -F(getAreaSum2DIntImg(int_img, xmid1, min_y, xmid2 - 1, max_y)) 		// mid
								   +F(getAreaSum2DIntImg(int_img, xmid2, min_y, max_x,     max_y)) )/N_third ); // right
#ifdef EARLY_RETURN_OPT
            if (curr_cost > best_cost)
                return false;
#endif
			// top + bottom - mid
			curr_cost += computeDiff( target_feature[index++],
								( F(getAreaSum2DIntImg(int_img, min_x, min_y,     max_x, ymid1 - 1)) // top
								   -F(getAreaSum2DIntImg(int_img, min_x, ymid1, max_x, ymid2 - 1)) // mid
								   +F(getAreaSum2DIntImg(int_img, min_x, ymid2, max_x, max_y)) )/N_third ); // bot
#ifdef EARLY_RETURN_OPT
		    if (curr_cost > best_cost)
		        return false;
#endif
			// diag - diag
			curr_cost += computeDiff( target_feature[index++],
									(F(getAreaSum2DIntImg(int_img, min_x,		min_y,     xmid - 1, 	ymid - 1)) // top left	
								    +F(getAreaSum2DIntImg(int_img, xmid, 	   	ymid, 		max_x, 		max_y)) // bot right	
								    -F(getAreaSum2DIntImg(int_img, min_x,		ymid, 		xmid - 1, 	max_y)) // bot left	
								    -F(getAreaSum2DIntImg(int_img, xmid, 	min_y, 	   		max_x, 		ymid - 1 )))/N_quarter ); // top right	
#ifdef EARLY_RETURN_OPT
            if (curr_cost > best_cost)
                return false;
#endif
		}
	}

#ifdef DEBUG_LOG_COST
    SLOGF(SLOG_DEBUG, "computeFeaturesGeneralAndUpdate(): start_pos: %d, %d; cost: %f", start_x, start_y, curr_cost);
#endif
    if (curr_cost > best_cost)
        return false;
	// If we reached here, it must be better than curr best cost
	best_cost = curr_cost;
	best_offset_x = start_x;
	best_offset_y = start_y;
	return true;
}
// instantiate the basic version for single channel
template bool computeFeaturesGeneralAndUpdate(const ImageU16 & int_img, std::uint16_t start_x, std::uint16_t start_y, // where the starting point is to compute
								std::uint16_t patch_width, std::uint16_t patch_height,	// size of the patch over which to compute feature
								std::uint16_t cell_width, std::uint16_t cell_height,	// size of cell, must not be more than 64 pixels, try 8x8 usually
								const float * target_feature,
								std::uint16_t & best_offset_x, std::uint16_t & best_offset_y, float & best_cost );



// if factor is int, then it degenerates to simple decimation for now
// if float, do bilinear interpolation
void downsample(const ImageU16 & src, ImageU16 & dst, float factor){

    std::uint16_t new_w = std::floor(src.width()/factor);
    std::uint16_t new_h = std::floor(src.height()/factor);

    dst.resize(new_w, new_h);
    for (std::uint16_t row = 0; row<new_h; ++row){
        for (std::uint16_t col= 0; col<new_w; ++col){
            dst(col, row) = bilinear_interpolate(src, col * factor, row * factor );
        }
    }
}

}
