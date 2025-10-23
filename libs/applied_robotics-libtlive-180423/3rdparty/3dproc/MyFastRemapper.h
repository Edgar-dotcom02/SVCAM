#ifndef MY_FAST_REMAPPER_H
#define MY_FAST_REMAPPER_H

#include <opencv/cv.h>
#include <stdint.h>

namespace tlive {

struct MyFastYuv2Yuv422Compresser
{
	void operator()(CvMat* src, CvMat* dst)
	{
		int w = src->width;
		int h = src->height;
		uint8_t* s = (uint8_t*)src->data.ptr;
		uint8_t* d = (uint8_t*)dst->data.ptr;
		for(int i = 0; i < h; i++)
		{
			uint8_t* max_d = d + w*2;
			while(d < max_d)
			{
				/* Compiler miss unrolling somehow */
				//*d = *s;
				//d++;
				//s += 3;

				/* Compiler miss unrolling somehow */
				d[0] = s[0];
				d[1] = s[1];
				d[2] = s[3];
				d[3] = s[2];

				d[4] = s[6];
				d[5] = s[7];
				d[6] = s[9];
				d[7] = s[8];

				d[8] = s[12];
				d[9] = s[13];
				d[10] = s[15];
				d[11] = s[14];

				d[12] = s[18];
				d[13] = s[19];
				d[14] = s[21];
				d[15] = s[20];
				d += 16;
				s += 24;
			}
		}
	}
};

};
#endif
