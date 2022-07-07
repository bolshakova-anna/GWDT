#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
#include <string>
using namespace std;
using namespace cv;


#define INF 3.4e+38 // point of infinity
enum{ 
  ALIVE = -1, 
  TRIAL = 0, 
  FAR = 1}; //Set three states;

static string noMask = "NO_MASK_INPUT";

bool find(queue<int> que, int value){
  int size = que.size();
  for (int i = 0; i < size; ++i){
	int a = que.front();
	if (a == value){
	  return true;
	}
	que.pop();
  }
  return false;
}


Mat GWDT2D(Mat image){
  //Read the image in grayscale mode
  int row = image.rows;
  int col = image.cols;
  //imshow("Original Picture", Scr);

  Mat resPicture; 
  resPicture.create(image.size(), CV_32FC1);

  Mat  mat_mean, mat_stddev;
  meanStdDev(image, mat_mean, mat_stddev);
  //Get the image mean
  double mean = mat_mean.at<double>(0, 0); 
  int* state = (new int[col * row]);

  for (int i = 0; i < row; i++){
	for (int j = 0; j < col; j++){
	  if (image.at<uchar>(i, j) < (mean)){
		//The background point is ALIVE
		resPicture.at<float>(i, j) = image.at<uchar>(i, j); 
		state[i * col + j] = ALIVE;
	  }
	  else{
		resPicture.at<float>(i, j) = INF;
		state[i * col + j] = FAR;
	  }

	}
  }

  //Define the point where the queue is used to store TRAIL
  queue<int> TrailQue; 
  for (int i = 1; i < row - 1; i++){
	for (int j = 1; j < col - 1; j++){
	  //If this point is a front sight, then search if there is a background point around this point, if it is then it is an edge point;
	  if (state[i * col + j] == FAR){
		for (int o = -1; o <= 1; o++){
		  for (int p = -1; p <= 1; p++){
			//Find all the edge points and put them into the queue TrailQue;
			if (state[(i + o) * col + j + p] == ALIVE){
			  state[i * col + j] = TRIAL;
			  resPicture.at<float>(i, j) = image.at<uchar>(i, j);
			  TrailQue.push(i * col + j);
			  break;
			}
		  }
		}
	  }
	}
  }


  while (!TrailQue.empty())
  {
	int P_row = TrailQue.front() / col; ///Get the coordinates of the TrailQue midpoint
	int P_col = TrailQue.front() % col;

	if (P_row < 1) { P_row = 1; }if (P_row > row - 2) { P_row = row - 2; }
	if (P_col < 1) { P_col = 1; }if (P_col > col - 2) { P_col = col - 2; }

	for (int o = -1; o <= 1; o++){
	  for (int p = -1; p <= 1; p++){
		int N = (P_row + o) * col + P_col + p;
		
		
		double len = sqrt(o * o + p * p);
		float gs = resPicture.at<float>(P_row, P_col) + image.at<uchar>(P_row + o, P_col + p) * len;

		//---Compare the existing GWDT value at this point with the value given by point P;
		if (resPicture.at<float>(P_row + o, P_col + p) > gs){
		  state[N] = TRIAL;
		  resPicture.at<float>(P_row + o, P_col + p) = gs;
		  TrailQue.push(N);

		  /*if (!find(TrailQue, N))
		  {
			  TrailQue.push(N);
		  }*/

		}
	  }
	}
	//Remove this point from TrailQue
	TrailQue.pop(); 
  }

  //Find the maximum value;
  float Max = 0;
  for (int i = 0; i < row; i++){
	for (int j = 0; j < col; j++){
	  if (resPicture.at<float>(i, j) > (INF / 100)){
		resPicture.at<float>(i, j) = 0;
	  }
	  // &&gwdtImg.at<float>(i, j)<(INF/10)
	  if (resPicture.at<float>(i, j) > Max){
		Max = resPicture.at<float>(i, j);
	  }

	}
  }

  //Compress the image pixel interval to 0-255
  Mat Dst;
  Dst = image.clone();
  for (int i = 0; i < row; i++){
	for (int j = 0; j < col; j++){
	  Dst.at<uchar>(i, j) = 255 * resPicture.at<float>(i, j) / (Max + 1);
	}
  }

  //imwrite("temp.tif", Dst);
  //Normalized 
  resPicture = resPicture / (Max + 1);
  imshow("GWDT", resPicture);
  waitKey(0);

  return resPicture;
}

//the main method
vector<Mat> GWDTCommon(string& imagePath) {
  //Read the image in grayscale mode
  //Mat scrPicture = imread(imagePath, 0);
  vector<Mat> scrPicture,res;
  if (imreadmulti(imagePath, scrPicture, IMREAD_GRAYSCALE)) {
	for (auto& slice : scrPicture) {
	  Mat resSlice = GWDT2D(slice);
	  res.push_back(resSlice);
	  /*res.push_back(~resSlice);
	  res.push_back(resSlice(Rect(0, 0, resSlice.cols / 2, resSlice.rows / 2)));*/
	}
	imwrite(imagePath + "_GWDT.tif", res);
	for (auto& slice : res) {
	  imshow("GWTD", slice);
	}
	return res;
  }
}

void main(int argc, char* argv[]){
  vector<Mat> res;
  string tmp = "test.tif";
  GWDTCommon(tmp);
  if (argc == 2) {
	string picPath = argv[1];
	GWDTCommon(picPath);
	//imshow("result",res);
	//imwrite("res.tif", res);
  }
  return;
}

