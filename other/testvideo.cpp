#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

#include <ctime>
#include <iostream>

#define SUBFRAMES_PER_FRAME 20

int main() {

	// Read in the images.
	cv::Mat left, right;

	// Open camera.
	cv::VideoCapture camera("../../data/block1/%06d.pnm");

	if(!camera.isOpened()) {

        std::cout << "Could not open or find the image." << std::endl ;
        return -1;

	}

	// Create window for display.
    cv::namedWindow("Stereo", cv::WINDOW_AUTOSIZE);// Create a window for display.

	// Determining the computation time.
    std::clock_t start_t = std::clock();
    //std::clock_t end_t;

	while(true) {

		// Get the left and right frames.
		camera >> left; if(left.empty()) break;
		camera >> right; if(right.empty()) break;
		cv::cvtColor(left, left, CV_GRAY2RGB);
		 cv::cvtColor(right, right, CV_GRAY2RGB);
		cv::Size leftSize = left.size();
		cv::Size rightSize = right.size();

	    cv::Mat stereo(leftSize.height, leftSize.width + rightSize.width, CV_8UC3);
	    cv::Mat leftRect(stereo, cv::Rect(0, 0, leftSize.width, leftSize.height));
	    left.copyTo(leftRect);
	    cv::Mat rightRect(stereo, cv::Rect(leftSize.width, 0, rightSize.width, rightSize.height));
	    right.copyTo(rightRect);

	 	cv::imshow("Stereo", stereo);
	 	cv::waitKey(1);

		// Determine processing time required.
		//end_t = std::clock();

		//printf(" Processing time: %6.5f seconds\n", ((end_t - start_t) / (double) CLOCKS_PER_SEC));

		while(((std::clock() - start_t) / (double) CLOCKS_PER_SEC) < 1/20.0f);

		start_t = std::clock();

    }


}