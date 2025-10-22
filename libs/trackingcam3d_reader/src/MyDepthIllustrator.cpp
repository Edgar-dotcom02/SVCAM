#include <trackingcam3d_reader/MyDepthIllustrator.h>

cv::Mat myGetRgbDepthMap(const cv::Mat& depth_map, int min_dst, int rg, int gb, int max_dst)
{
	int h = depth_map.rows;
	int w = depth_map.cols;
	cv::Mat colored_depth(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
	const int depth_pix_size = 2;

	for(int i = 0; i < h; i++)
	{
		for(int j = 0; j < w; j++)
		{
			uint16_t depth = depth_map.at<uint16_t>(i, j);
			uint8_t r = 0;
			uint8_t g = 0; 
			uint8_t b = 0;

			if(depth < min_dst)
			{
			}
			else if(depth <= rg)
			{
				g = (255 * depth) / rg;
				r = 255 - g;
			}
			else if(depth <= gb)
			{
				b = (255 * (depth - rg)) / (gb - rg);
				g = 255 -b;
			}
			else if(depth < max_dst)
			{
				b = 255;
			}
			else
			{
			}
			
			colored_depth.at<cv::Vec3b>(i, j) = cv::Vec3b(b, g, r);
		}
	}

	return colored_depth;
}

cv::Mat myGetDispMap(const cv::Mat& disp_map)
{
	CvMat img = disp_map;
	int h = img.rows;
	int w = img.cols;

	CvMat* dst = cvCreateMat(h, w, CV_8UC1);
	cvConvert(&img, dst);
	
	cv::Mat scaled_disp = cv::cvarrToMat(dst);
	return scaled_disp;
}