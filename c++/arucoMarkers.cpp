#include <stdio.h>
#include <stdlib.h>
#include <list>

// OpenCV
#include <opencv2//core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;


#define ARUCO_MARKER_LENGTH 6.9 // #centimeters
#define SQUARE_SIDE_SIZE 2.35 // #centimeters
#define ESC_CHAR 27

void calibrateCamera(char* calibrationFile, int nCornersRow, int nCornersCol , double squareSideSize, Mat *cameraMatrix, Mat *distCoeff)
{	
	vector< vector< Point3f > > realPoints3D;
	vector< vector< Point2f > > imgPoints2D;

	vector< Point3f > objReal3DPoint;
	for (int i = 0; i < nCornersRow; i++)
	{
		for (int j = 0; j < nCornersCol; j++)
		{
			objReal3DPoint.push_back(Point3f((float)j * squareSideSize, (float)i * squareSideSize, 0));
		}
	}

	Size chessboardSize = Size(nCornersCol, nCornersRow);

	int detectedFrames = 0;
	VideoCapture vidCap(0);
	double frameRate = 30;
	Size imgSize;

	printf("Use camera to identify a chessboard \n");

	while (detectedFrames < 20)  //long captures make calibrateCamera take too long
	//while (true)
	{
		Mat frame, frameGray;

				
			

		// Capture frame-by-frame
		vidCap >> frame;

		imgSize = frame.size();

		cvtColor(frame, frameGray, CV_BGR2GRAY);				

		bool found = false;

		vector< Point2f > corners;

		found = findChessboardCorners(frameGray, chessboardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
				
		if (found)
		{
			cornerSubPix(frameGray, corners, cv::Size(5, 5), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(frame, chessboardSize, corners, found);
			detectedFrames += 1;
			imgPoints2D.push_back(corners);
			realPoints3D.push_back(objReal3DPoint); 
		}			

		imshow("Camera Calibration", frame);

		char c = waitKey((double)1000 / frameRate);

		if (c == ESC_CHAR) //esc char
		{
			break;
		}

	}

	destroyAllWindows();

	int flag = 0;
	flag |= CV_CALIB_FIX_K4;
	flag |= CV_CALIB_FIX_K5;
		
	printf("Calculating camera's intrinsic parameters... \n");
	vector< Vec3d > rvecs, tvecs;			
	calibrateCamera(realPoints3D, imgPoints2D, imgSize, *cameraMatrix, *distCoeff, rvecs, tvecs, CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	printf("Done. \n");

	FileStorage fs(calibrationFile, FileStorage::WRITE);
	fs << "cameraMatrix" << *cameraMatrix;
	fs << "distCoeff" << *distCoeff;
	fs.release();	

	
}

void detectMarkers(int nCornersRow, int nCornersCol, Mat cameraMatrix, Mat distCoeff)
{

	

	 int axisLen = 5;
	 int markerLength = ARUCO_MARKER_LENGTH; 
	 int frameRate = 100;
	 VideoCapture vidCap(0);
	 
	 Ptr<aruco::Dictionary> arucoDict = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);

	 while (true)
	 {
		 Mat frame, frameGray;

		 // Capture frame-by-frame
		 vidCap >> frame;

		 vector<Vec3d> rvecs, tvecs;		 
		 vector<int> ids;
		 vector<vector< Point2f >> corners;		 

		 aruco::detectMarkers(frame, arucoDict, corners, ids);

		 if (ids.size() > 0)
		 {
			 aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeff, rvecs, tvecs);

			 aruco::drawDetectedMarkers(frame, corners, ids, Scalar(0, 255, 0));

			 aruco::drawAxis(frame, cameraMatrix, distCoeff, rvecs, tvecs, axisLen);


		 }

		 imshow("Marker Detection", frame);

		 char c = waitKey((double)1000 / frameRate);

		 if (c == ESC_CHAR) //esc char
		 {
			 break;
		 }

	 }

}

int main(int argc, char *argv[])
{
	Mat cameraMatrix, distCoeff;
	
	char * calibrationFile = "calibration.xml";

	FileStorage fs;

	fs.open(calibrationFile, FileStorage::READ);

	if (!fs.isOpened())
	{
		calibrateCamera(calibrationFile, 6, 9, SQUARE_SIDE_SIZE, &cameraMatrix, &distCoeff);
	}

	else
	{
		fs["cameraMatrix"] >> cameraMatrix;
		fs["distCoeff"] >> distCoeff;
	}

	//printf("cameraMatrix\n");
	//for (int i = 0; i < 3; i++)
	//{
	//	for (int j = 0; j < 3; j++)
	//	{
	//		printf("%f ", cameraMatrix.at <double>(i, j));
	//	}
	//	printf("\n");
	//}

	//printf("distCoeff\n");
	//for (int j = 0; j < 3; j++)
	//{
	//	
	//	//distCoeff.at <double>(0, j) = 0;
	//	printf("%f ", distCoeff.at <double>(0, j));
	//}
	//printf("\n");
	
	detectMarkers(6, 9, cameraMatrix, distCoeff);

    return 0;
}