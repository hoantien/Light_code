/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    
 * @author  The LightCo
 * @version V1.0.0
 * @date   
 * @brief   
 *
 ******************************************************************************/


// compile notes:
/*

	g++ -std=c++11 geoff_test.cpp line2.cpp calibclasses.cpp disparityfocus.cpp basic_image.cpp color_image.cpp roitransfer.cpp mirrorsystems.cpp image_pixel_iter.cpp rectangle.cpp image_raw10.cpp -I../include/

*/


#include "stdio.h"
#include "assert.h"
#include "matrix.h"
#include "line2.h"
#include "calibclasses.h"
#include "disparityfocus.h"
#include "basic_image.h"
//#include "color_image.h"
#include "roitransfer.h"

constexpr float kThreshold = 1e-5;

void printMatrix3x3f(const ltaf::Matrix3x3f & mat){
	const float * data = mat.data();
	for (int i = 0; i< 9; ++i){
		printf("%f, ", data[i]);
	}
	printf("\n");
}

void printVec2f(const ltaf::Vec2f & v){
	printf("%f, %f\n", v.x(), v.y());
}

void printVec3f(const ltaf::Vec3f & v){
	printf("%f, %f, %f\n", v.x(), v.y(), v.z());
}

void printVec4f(const ltaf::Vec4f & v){
	printf("%f, %f, %f, %f\n", v.x(), v.y(), v.z(), v.a());
}

void test_vec2(){
	ltaf::Vec2f v1(1.0f, 2.0f);
	v1*=2.0f;
	assert(v1.x() == 2.0f && v1.y() == 4.0f);

	v1+=1.0f;
	assert(v1.x() == 3.0f && v1.y() == 5.0f);

	v1-=1.0f;
	assert(v1.x() == 2.0f && v1.y() == 4.0f);

	v1/=2.0f;
	assert(v1.x() == 1.0f && v1.y() == 2.0f);

	ltaf::Vec2f v2 = v1 + v1;
	assert(v2.x() == 2.0f && v2.y() == 4.0f);

	v2 = v1 - v1;
	assert(v2.x() == .0f && v2.y() == .0f);

	printf("vec2 tests done...\n");
}

void test_vec3(){
	ltaf::Vec3f v1(1.0f, 2.0f, 3.0f);
	v1*=2.0f;
	assert(v1.x() == 2.0f && v1.y() == 4.0f && v1.z() == 6.0f);

	v1+=1.0f;
	assert(v1.x() == 3.0f && v1.y() == 5.0f && v1.z() == 7.0f);

	v1-=1.0f;
	assert(v1.x() == 2.0f && v1.y() == 4.0f && v1.z() == 6.0f);

	v1/=2.0f;
	assert(v1.x() == 1.0f && v1.y() == 2.0f && v1.z() == 3.0f);

	ltaf::Vec3f v2 = v1 + v1;
	assert(v2.x() == 2.0f && v2.y() == 4.0f && v2.z() == 6.0f);

	v2 = v1 - v1;
	assert(v2.x() == .0f && v2.y() == .0f && v2.z() == .0f);

	printf("vec3 tests done...\n");
}

void test_Matrix3x3(){
	// test multiplication and identity
	ltaf::Matrix3x3f mat({1,0,0, 0,1,0, 0,0,1});
	ltaf::Vec3f v(1.0f, 2.0f, 3.0f);
	ltaf::Vec3f v1 = mat * v;
	assert(v1.x() == 1.0f && v1.y() == 2.0f && v1.z() == 3.0f);

	ltaf::Matrix3x3f mat2 = ltaf::inverse(mat);
	v1 = mat2 * v;
	assert(v1.x() == 1.0f && v1.y() == 2.0f && v1.z() == 3.0f);

	mat = ltaf::Matrix3x3f({3,0,0, 0,2,0, 0,0,1});
	v1 = mat * v;
	assert(v1.x() == 3.0f && v1.y() == 4.0f && v1.z() == 3.0f);

//test inverse and multiplication
	mat2 = ltaf::inverse(mat);
	float * data = mat2.data();
//	printf("%f, %f, %f\n", data[0], data[4], data[8]);
	assert( (fabs(data[0] - .333333f) < kThreshold)
		 && (data[4] == .5f)  
		 && (data[8] == 1.0f) );
	v1 = mat2 * v;
	assert((fabs(v1.x() - .333333f) < kThreshold) 
		&& v1.y() == 1.0f 
		&& v1.z() == 3.0f);
	printf("Matrix3x3 tests done...\n");
}

void test_line2(){
	ltaf::Line2f l(2.5f, -20.0f, 100.0f);
	ltaf::Vec2f p1, p2;
	bool intersect = ltaf::IntersectBox(l, 0,0,20,10,p1, p2);
	assert(intersect);
	assert(p1.x() == 0.0f && p1.y() == 5.0f);
	assert(p2.x() == 20.0f && p2.y() == 7.5f);
	printf("line2 tests done...\n");
}

void test_epf_wpf_depth(){
	// test epifield
	ltaf::Matrix3x3f ref_K({455.948, 0, 240,  0 ,455.948, 180,  0 ,0 ,1} );
	ltaf::Matrix3x3f ref_R({1, 0, 0,  0, 0, -1,  0, 1, 0 });
	ltaf::Vec3f ref_T({ 500, 1500, 1600});

	ltaf::Matrix3x3f src_K({961.046, 0, 240, 0, 961.046, 180 ,0 ,0 ,1});
	ltaf::Matrix3x3f src_R({1, 0, 0, 0 ,0 ,-1, 0, 1, 0});
	ltaf::Vec3f src_T({513.19, 1522.33, 1605.25});
// -0.849795,0.527113,6.26685
	ltaf::CalibData cd1(ref_K, ref_R, ref_T);
	ltaf::CalibData cd2(src_K, src_R, src_T);
	ltaf::EpiField epi(cd1, cd2);
	ltaf::Line2f epiline = epi(213, 229); // need valid matrices to work...
	//printf("epiline %f, %f, %f\n", epiline.x(), epiline.y(), epiline.z());
	assert( fabs( epiline.x() + .849795f ) < kThreshold 
		 && fabs( epiline.y() - .527112f ) < kThreshold
		 && fabs( epiline.z() - 6.267082f) < kThreshold );


	// test 2..
	ltaf::Matrix3x3f A1_K({1658.77, 0, 1043.39, 0, 1656.89, 779.809, 0, 0, 1 });
	ltaf::Matrix3x3f A1_R({1, 2.44715e-20, 2.49222e-18, 2.44715e-20, 1, 4.47875e-19, 2.49222e-18, 4.47875e-19, 1 });
	ltaf::Vec3f A1_t({3.55271e-15, 0, 0});

	ltaf::Matrix3x3f A5_K({1657.63, 0, 1026.94, 0, 1655.3, 770.234, 0, 0, 1  });
	ltaf::Matrix3x3f A5_R({0.99998, -0.00590931, -0.00238703, 0.00590509, 0.999981, -0.00177166, 0.00239746, 0.00175753, 0.999996});
	ltaf::Vec3f A5_t({43.8206, -1.45247, 0.938695});

	ltaf::CalibData cd_A1(A1_K, A1_R, A1_t);
	ltaf::CalibData cd_A5(A5_K, A5_R, A5_t);
	ltaf::EpiField epf(cd_A1, cd_A5);
	epiline = epf(1147, 822);
	//printf("epiline %f, %f, %f\n", epiline.x(), epiline.y(), epiline.z());
	assert( fabs( epiline.x() - 0.033638f ) < kThreshold 
		 && fabs( epiline.y() - 0.999434f ) < kThreshold
		 && fabs( epiline.z() + 847.480286f) < kThreshold );
	printf("EpiField tests done...\n");



		// test warp field
	ltaf::WarpField wpf(cd1, cd2);
	ltaf::Vec3f vv = wpf.project3D(100.0f ,100.0f, 1500.0f);
	//printf("%f, %f, %f\n", vv.x(), vv.y(), vv.z());
	//-45.641121, 26.221159, 1505.250000
	assert( fabs( vv.x() + 45.641121f ) < kThreshold 
		 && fabs( vv.y() - 26.221159f ) < kThreshold
		 && fabs( vv.z() - 1505.25f ) < kThreshold );
	printf("WarpField tests done...\n");

		// test depth given correspondence
	float d1 = ltaf::CalibUtils::depthGivenCorrespondence(cd1, cd2, 100.0f, 100.0f, 100.0f, 100.0f);
	float d2 = ltaf::CalibUtils::depthGivenCorrespondence(epi, cd1, cd2, 100.0f, 100.0f, 100.0f, 100.0f);
	assert(d1 == d2 && fabs(d1 - 162.352051f) < kThreshold);
	printf("depth test 1 done...\n");
	printf("depth test 1 done done...\n");
}

void test_depthGivenCorrespondence() {
	// ------------------ first test ----------------
	ltaf::Matrix3x3f A1_k({ 3352.28, 0, 2079.18, 
					  0, 3348.65, 1554.15, 
					  0, 0, 1 });

    ltaf::Matrix3x3f A1_r({ 1, -3.89903e-20, -7.09106e-19, 
    	              -3.89903e-20, 1, 1.37959e-19, 
    	              -7.09106e-19, 1.37959e-19, 1 });

    ltaf::Vec3f A1_t({ 0, 0, 5.20417e-18 });
    ltaf::Matrix3x3f A5_k({3344.64, 0, 2054.65, 
    				 0, 3348.05, 1547.09, 
    				 0, 0, 1 });

    ltaf::Matrix3x3f A5_r({0.999993, -0.00150161, -0.00337484, 
    				0.00149424, 0.999996, -0.00218467, 
    				.00337811, 0.00217961, 0.999992 });
    
    ltaf::Vec3f A5_t({42.4754, 0.245912, -1.41566});
    
    ltaf::CalibData ref(A1_k, A1_r, A1_t);
    ref.scale(.5f);

    ltaf::CalibData src(A5_k, A5_r, A5_t);
    src.scale(.5f);

    float depth = ltaf::CalibUtils::depthGivenCorrespondence(ref, src, 271, 767, 282, 758);
    assert(fabs(depth - 2472.450439f) < kThreshold);

    depth = ltaf::CalibUtils::depthGivenCorrespondence(ref, src, 1712, 298, 1698, 292);
	assert(fabs(depth - 13806.388672f) < kThreshold);


	// ---------------- second test -------------------
	/*
patch_center_x_calib = 2608.000000000		patch_center_y_calib = 1666.000000000
aux_center_x_calib = 2573		aux_center_y_calib = 1643
            depth = 20509.183593750

patch_center_x_calib = 3094.000000000		patch_center_y_calib = 1348.000000000
aux_center_x_calib = 3060		aux_center_y_calib = 1329
            depth = 20934.287109375


patch_center_x_calib = 2120.000000000		patch_center_y_calib = 2839.000000000
aux_center_x_calib = 2109		aux_center_y_calib = 2810
            depth = 3876.527832031
            */
	A1_k = ltaf::Matrix3x3f({ 3317.54, 0, 2086.78, 0, 3313.78, 1559.62, 0, 0, 1  });

   	A1_r = ltaf::Matrix3x3f({  1, 2.44715e-20, 2.49222e-18, 2.44715e-20, 1, 4.47875e-19, 2.49222e-18, 4.47875e-19, 1 });

    A1_t = ltaf::Vec3f({ 3.55271e-15, 0, 0});

    A5_k = ltaf::Matrix3x3f({3315.26 ,0 ,2053.88 ,0, 3310.6, 1540.47, 0, 0 ,1 });

    A5_r = ltaf::Matrix3x3f({0.99998 ,-0.00590931, -0.00238703 , 0.00590509 ,0.999981, -0.00177166 ,0.00239746 ,0.00175753 ,0.999996  });
    
    A5_t = ltaf::Vec3f({43.8206, -1.45247, 0.938695});

    ref.setCameraParameters(A1_k, A1_r, A1_t);
    src.setCameraParameters(A5_k, A5_r, A5_t);

	depth = ltaf::CalibUtils::depthGivenCorrespondence(ref, src, 2608, 1666, 2573, 1643);
	assert(fabs(depth - 20517.835938f) < kThreshold);

    depth = ltaf::CalibUtils::depthGivenCorrespondence(ref, src, 3094, 1348, 3060, 1329);
	assert(fabs(depth - 20943.607422f) < kThreshold);

    depth = ltaf::CalibUtils::depthGivenCorrespondence(ref, src, 2120, 2839, 2109, 2810);
	assert(fabs(depth - 3876.818604f) < kThreshold);

	printf("DepthGivenCorrespondence tests done...\n");    
}

void test_disparityfocus(){
	ltaf::Matrix3x3f A1_K({1658.77, 0, 1043.39, 0, 1656.89, 779.809, 0, 0, 1 });
	ltaf::Matrix3x3f A1_R({1, 2.44715e-20, 2.49222e-18, 2.44715e-20, 1, 4.47875e-19, 2.49222e-18, 4.47875e-19, 1 });
	ltaf::Vec3f A1_t({3.55271e-15, 0, 0});

	ltaf::Matrix3x3f A5_K({1657.63, 0, 1026.94, 0, 1655.3, 770.234, 0, 0, 1  });
	ltaf::Matrix3x3f A5_R({0.99998, -0.00590931, -0.00238703, 0.00590509, 0.999981, -0.00177166, 0.00239746, 0.00175753, 0.999996});
	ltaf::Vec3f A5_t({43.8206, -1.45247, 0.938695});

    ltaf::CalibData A1(A1_K, A1_R, A1_t);
    ltaf::CalibData A5(A5_K, A5_R, A5_t);
    ltaf::CalibData B4;

    ltaf::EpiField epf(A1, A5);
    ltaf::WarpField wpf(A1, A5);

    ltaf::DisparityFocus df(A1, A5, B4); // doesn't matter. we call the internal func()

 	void * img_pri_ptr = nullptr;
 	void * img_aux_ptr = nullptr;

 	int capture_mode = 0;
 	int scale = 1;

 	rectangle rect = df.computeAuxRectangle(rectangle(1, 1, 10, 10), capture_mode);
 	printf("area = %f\n", float(rect.area()));


 	rectangle aux_rect;
    float depth = df.computeDepth(	img_pri_ptr, rectangle(1, 1, 10, 10), rectangle(1,1,10,10),
                        			img_aux_ptr, rectangle(1, 1, 10 ,10), 
                        			capture_mode, scale, &aux_rect );
    printf("depth = %f\n", depth);

}

void test_image(){
	{	// imageu16
		ltaf::ImageU16 img(5,3);
		uint16_t data[15] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
		img.setData(data);

		ltaf::ImageU16 img2(3,3);
		uint16_t data2[9] = {2,3,4,7,8,9,12,13,14};
		img2.setData(data2);

		int offset_x, offset_y;

	// find itself
		bool result = ltaf::findPatch(img, img, offset_x, offset_y);
		assert(result);
		assert(offset_x == 0 && offset_y == 0);
	// find a patch too large
		result = ltaf::findPatch(img, img2, offset_x, offset_y);
		assert(!result);
	// find a proper patch
		result = ltaf::findPatch(img2, img, offset_x, offset_y);
		assert(result);
		assert(offset_x == 1 && offset_y == 0);
		printf("ImageU16 tests done...\n");
	}
	{ // imageF
		ltaf::ImageF img(5,3);
		float data[15] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
		img.setData(data);

		ltaf::ImageF img2(3,3);
		float data2[9] = {2,3,4,7,8,9,12,13,14};
		img2.setData(data2);

		int offset_x, offset_y;

	// find itself
		bool result = ltaf::findPatch(img, img, offset_x, offset_y);
		assert(result);
		assert(offset_x == 0 && offset_y == 0);
	// find a patch too large
		result = ltaf::findPatch(img, img2, offset_x, offset_y);
		assert(!result);
	// find a proper patch
		result = ltaf::findPatch(img2, img, offset_x, offset_y);
		assert(result);
		assert(offset_x == 1 && offset_y == 0);
		printf("ImageF tests done...\n");
	}
	{ // imageRGB16
		ltaf::ImageRGB16 img(50,50);
		img.clear(ltaf::Vec3u16(1023,1023,1023));
		img(25, 25) = ltaf::Vec3u16(0, 0, 0);

		ltaf::ImageRGB16 img2(3,3);
		img2.clear(ltaf::Vec3u16(1023,1023,1023));
		img2(1,1) = ltaf::Vec3u16(0,0,0);

		int offset_x, offset_y;

	// find itself
		bool result = ltaf::findPatch(img, img, offset_x, offset_y);
		assert(result);
		assert(offset_x == 0 && offset_y == 0);
	// find a patch too large
		result = ltaf::findPatch(img, img2, offset_x, offset_y);
		assert(!result);
	// find a proper patch
		result = ltaf::findPatch(img2, img, offset_x, offset_y);
		assert(result);
		assert(offset_x == 24 && offset_y == 24);
		printf("ImageRGB16 tests done...\n");
	}
	{
		// test downsampling decimation
		ltaf::ImageU16 img(5,5);
		img(2,2) = 100;
		ltaf::ImageU16 img2;
		ltaf::downsample(img, img2, 2.0f);
		assert(img2.width() == 3 && img2.height() == 3);
		for (int row = 0; row < 3; ++row){
			for (int col = 0; col < 3; ++col){
				if ( row == 1 && col == 1)
					assert(img2(col, row) == 100);
				else
					assert(img2(col, row) == 0);
			}
		}
		printf("ImageU16 downsampling int works...\n");
	}
	{
		// test downsampling float
		ltaf::ImageU16 img(5,5);
		for (int x = 0; x<5; ++x){
			for (int y =0; y<3; ++y){ // row 0 1 2
				img(x, y) = 100;
			}
		}
		ltaf::ImageU16 img2;
		ltaf::downsample(img, img2, 2.5f);
		assert(img2.width() == 2 && img2.height() == 2);
		for (int row = 0; row < 2; ++row){
			printf("\n");
			for (int col = 0; col < 2; ++col){
				printf("%u ", img2(col, row));
			}
		}
	}
}

#if 0
void test_color_image(){
	ltaf::ImageRGB10 img(3, 5);
	img.print();
	uint16_t data[15] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	img.setRData(data);
	img.setGData(data);
	img.setBData(data);
	img.print();
}
#endif

void test_integral_image_inplace(){
	int width = 110;
	int height = 100;
	ltaf::ImageU16 img(width, height);
	ltaf::ImageU16 int_img_inplace(width, height);
	for (int i = 0; i<img.length(); ++i) { img[i] = i%1024;  int_img_inplace[i] = img[i]; } // 10 bit values

	ltaf::ImageU16 int_img ( ltaf::computeIntegralImage(img) );
	// test compute integral image in place
	computeIntegralImageInPlace( int_img_inplace );

	// verify both are the same:
	for (int x = 0; x<width; ++x){
		for (int y = 0; y<height; ++y){
			assert(int_img_inplace(x, y) == int_img(x, y));
		}
	}

	printf("integral image in place works (1 channel)...\n");

	//check any query patch of length less than 8x8/2
	// looks like we can go as far as 8x8 patches without overflow!
	const int max_w = 32;
	const int max_h = 32;
	for (int sum_width = 1; sum_width <= max_w; ++sum_width){
		for (int sum_height = 1; sum_height<= max_h; ++sum_height){
			if (sum_width * sum_height > 64)
				continue;

			for (int offset_x = 0; offset_x < width - sum_width; ++offset_x){
				for (int offset_y = 0; offset_y < height - sum_height; ++offset_y){
					std::uint16_t curr_sum = ltaf::getAreaSum2DIntImg(int_img, offset_x, offset_y, 
																	  offset_x + sum_width - 1, offset_y + sum_height - 1 ); // inclusive
					//printf("%u, %u, %u, %u\n", offset_x, offset_y, offset_x + sum_width, offset_y + sum_height);
					//printf("%u, %f\n", curr_sum, float(val * (1+sum_width) * (1+sum_height)));

					float curr_sum_target = 0;
					for (int xx = offset_x; xx<=offset_x + sum_width - 1; ++xx){
						for (int yy = offset_y ; yy<=offset_y + sum_height - 1; ++yy){
							curr_sum_target += img(xx, yy);
						}
					}
					assert(float(curr_sum) == curr_sum_target);
				}
			}
		}
	}
	printf("integral image in place and getAreaSum2DIntImg correctness tested (1 channel)...\n");
}

void test_integral_image(){
	// overflow test for integral images
	// suppose we use 16 bits, this should be more than sufficient for any lookup (sum) of 32 pixels
	std::uint16_t topleft  = 65500;
	std::uint16_t topright = 12450; //mod 2^16
	std::uint16_t botleft  = 23456; //mod 2^16
	std::uint16_t botright = 47604; //mod 2^16
	std::uint16_t sum = topleft + botright - topright - botleft;
	//printf("%u\n", sum);  // has to be within 65536
	float topleft_f = topleft;
	float topright_f = topright + 65536;
	float botleft_f = botleft + 65536;
	float botright_f = botright + 65536;
	float sum_f = topleft_f + botright_f - topright_f - botleft_f;
	//printf("%f\n", sum_f);
	assert(sum_f == sum);

	// -------------------------
	// test integral image
	// -------------------------
	{
		int width = 110;
		int height = 100;
		ltaf::ImageU16 img(width, height);
		const int val = 1023; // 10 bit
		img.clear(val); 

		//printf("allocated image of length %i at %lu\n", img.length(), DDR_BASE);
		ltaf::ImageU16 int_img = ltaf::computeIntegralImage(img);

		//check any query patch of length less than 8x8/2
		// looks like we can go as far as 8x8 patches without overflow!
		const int max_w = 32;
		const int max_h = 32;
		for (int sum_width = 1; sum_width <= max_w; ++sum_width){
			for (int sum_height = 1; sum_height<= max_h; ++sum_height){
				if (sum_width * sum_height > 64){
					continue;
				}
				for (int offset_x = 0; offset_x < width - sum_width; ++offset_x){
					for (int offset_y = 0; offset_y < height - sum_height; ++offset_y){
						std::uint16_t curr_sum = ltaf::getAreaSum2DIntImg(int_img, offset_x, offset_y, 
																		  offset_x + sum_width - 1, offset_y + sum_height - 1 );
						//printf("%u, %u, %u, %u\n", offset_x, offset_y, offset_x + sum_width, offset_y + sum_height);
						//printf("%u, %f\n", curr_sum, float(val * (1+sum_width) * (1+sum_height)));
						assert(float(curr_sum) == float(val * sum_width * sum_height));
					}
				}
			}
		}
		printf("10bit->16bit integral image tests done (1 channel)...\n");
	}
	{ // test it on ImageRGB16
		int width = 110;
		int height = 100;
		ltaf::ImageRGB16 img(width, height);
		const ltaf::Vec3u16 val(1023, 1022, 1021); // 10 bit
		img.clear(val); 

		ltaf::ImageRGB16 int_img = ltaf::computeIntegralImage(img);
		// check any query patch of length less than 8x8/2
		// looks like we can go as far as 8x8 patches without overflow!
		const int max_w = 32;
		const int max_h = 32;
		for (int sum_width = 1; sum_width <= max_w; ++sum_width){
			for (int sum_height = 1; sum_height<= max_h; ++sum_height){
				if (sum_width * sum_height > 64){
					continue;
				}

				for (int offset_x = 0; offset_x < width - sum_width; ++offset_x){
					for (int offset_y = 0; offset_y < height - sum_height; ++offset_y){
						ltaf::Vec3u16 curr_sum = ltaf::getAreaSum2DIntImg(int_img, offset_x, offset_y, 
																offset_x + sum_width - 1, offset_y + sum_height - 1 );
						//printf("%u, %u, %u, %u\n", offset_x, offset_y, offset_x + sum_width, offset_y + sum_height);
						//printf("%u, %f\n", curr_sum, float(val * (1+sum_width) * (1+sum_height)));
						assert(float(curr_sum.x()) == float(val.x() * sum_width * sum_height));
						assert(float(curr_sum.y()) == float(val.y() * sum_width * sum_height));
						assert(float(curr_sum.z()) == float(val.z() * sum_width * sum_height));
					}
				}
			}
		}
		printf("10bit->16bit integral image tests done (3 channel)...\n");
	}
	
}


void test_roitransfer(){
	ltaf::ROITransfer rt;
	float depth_mm = 1500.0f;
	int hallcode = 450;
	ltaf::Vec2f out;

	// test on p2-9 calibration data
	

	ltaf::Vec2f in(1000,1000); // pixel coordinate in calibration space
	// A2
	out = rt.transferROI(ltaf::ModuleName::A1, ltaf::ModuleName::A2, 
										in, depth_mm, hallcode );
	assert( abs(out.x() - 1043.507080078f) < kThreshold
		 && abs(out.y() - 1033.982177734f) < kThreshold );
	printf("p2-9 roitransfer A1->A2 pass...\n");


	// A3
	out = rt.transferROI(ltaf::ModuleName::A1, ltaf::ModuleName::A3, 
										in, depth_mm, hallcode );
	assert( abs(out.x() - 963.235534668f) < kThreshold
		 && abs(out.y() - 1020.322021484f) < kThreshold );
	printf("p2-9 roitransfer A1->A3 pass...\n");

	// A5
	out = rt.transferROI(ltaf::ModuleName::A1, ltaf::ModuleName::A5, 
										in, depth_mm, hallcode );
	assert( abs(out.x() - 1124.713378906f) < kThreshold
		 && abs(out.y() - 1003.091308594f) < kThreshold );
	printf("p2-9 roitransfer A1->A5 pass...\n");


	// use diff coords for B's
    in = {2000, 2000};

	//B4
	out = rt.transferROI(ltaf::ModuleName::A1, ltaf::ModuleName::B4, 
										in, depth_mm, hallcode );
	assert( abs(out.x() - 1365.272827148f ) < kThreshold
		 && abs(out.y() - 2310.889404297f ) < kThreshold );
	printf("p2-9 roitransfer A1->B4 pass...\n");

	// B2
	// mirror angle should be 40.510928100
	out = rt.transferROI(ltaf::ModuleName::B4, ltaf::ModuleName::B2, 
										in, depth_mm, hallcode );
	//printf("B4-B2: %f, %f\n",out.x(),out.y());
	assert( abs(out.x() - 1626.123046875f ) < kThreshold
		 && abs(out.y() - 1774.575927734f ) < kThreshold );
	printf("p2-9 roitransfer B4->B2 pass...\n");

	// B3
	out = rt.transferROI(ltaf::ModuleName::B4, ltaf::ModuleName::B3, 
										in, depth_mm, hallcode );
	assert( abs(out.x() - 2722.059326172f ) < kThreshold
		 && abs(out.y() - 2132.954101562f ) < kThreshold );
	printf("p2-9 roitransfer B4->B3 pass...\n");

	// B5
	out = rt.transferROI(ltaf::ModuleName::B4, ltaf::ModuleName::B5, 
										in, depth_mm, hallcode );
	assert( abs(out.x() - 3088.438476562f ) < kThreshold
		 && abs(out.y() - 2396.890625000f ) < kThreshold );
	printf("p2-9 roitransfer B4->B5 pass...\n");

}

// test code.
#ifdef ASIC_NUM
extern "C" 
int run_tests(void) // compile for ASIC. cannot be main()
#else
int main(void)
#endif
{
	//test_vec2();
	//test_vec3();
	//test_Matrix3x3();
	//test_line2();
	//test_epf_wpf_depth();
	//test_depthGivenCorrespondence();
	//test_disparityfocus();
	//test_image();
	//test_integral_image_inplace();
	//test_integral_image(); 
	test_roitransfer();
	//test_color_image();
	return 0;
}
