
/** \file     CNN.cpp
    \brief    CNN-based interpolation
*/

#ifndef __CNN__
#define __CNN__



//include files
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>
#include <ctime>
#include "caffe/util/io.hpp"
#include "caffe/caffe.hpp"
#include "caffe/blob.hpp"
#include "caffe/net.hpp"
#include "TComPattern.h"
#include <caffe/layers/memory_data_layer.hpp>


using namespace std;
using namespace cv;
using namespace caffe;


//! \ingroup TLibCommon
//! \{
 
// ====================================================================================================================
// Class definition
// ====================================================================================================================
/*
const std::string model_file = "/home/chipdk/videocoding/caffe/examples/mySRCNN/residue_learning/SRCNN_net_test.prototxt";
const  std::string trainedweight = "/home/chipdk/videocoding/caffe/examples/mySRCNN/residue_learning/Iterations/SRCNN_solver_iter_15000000_keepintegerpixels.caffemodel"; 
*/
const std::string model_file = "/home/chipdk/videocoding/caffe/examples/mySRCNN/residue_learning/SRCNN_net_same_size.prototxt";
const  std::string trainedweight = "/home/chipdk/videocoding/caffe/examples/mySRCNN/residue_learning/Iterations/SRCNN_solver_iter_15000000_keepinterger_samesizeoutput.caffemodel"; 


const int CNNscale = 4;
class CNN
{
private:	
	caffe::shared_ptr < caffe::Net < float > > net_;
	//Net<float> net_;
	
public:
	CNN();//link to net and link to trained weight
	void init();
	void convertBlobtoTcomPattern(std::vector< caffe::Blob< float > * > blobinput, TComPattern* toutput,  int stride,  int bitdepth, int roix, int roiY);
	void convertTComPatterntoBlob(TComPattern* tinput, vector<Blob< float > *>  b_input_CNN);
	void Predict(TComPattern* input, TComPattern* output);
	void myresize_up(float* input, int height, int width, float* output);
	void FixedIntegerPixels(const float* inputCNN, const float* outputCNN, int height, int width, float* output_process);
};
//! \}

#endif