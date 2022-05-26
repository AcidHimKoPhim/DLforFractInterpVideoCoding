
#include "CNN.h"
#include "TComPattern.h"

/** \file     CNN.cpp
    \brief    CNN-based interpolation
*/

//
//load network parameters
//
void CNN::init()
{
	CNN();
}


CNN::CNN()
{
	cout<<"------------------------------------init net------------------------"<<endl;
	CNN::net_.reset(new Net<float>(model_file, TEST));
    CNN::net_->CopyTrainedLayersFrom (trainedweight); //load trained model
	cout<<"------------------------------------got trained net------------------------"<<endl;

}

void CNN::convertBlobtoTcomPattern(vector< Blob< float > * > blobinput, TComPattern* toutput, int stride,  int bitdepth, int roiX, int roiY)
{
	cout<<"--------------------start convertBlob to TcomPattern------------------------"<<endl;
	cout<<"numofpixels = "<<blobinput[0]->width()<<" x " <<blobinput[0]->height() <<endl;
	int numofpixels = blobinput[0]->height()  * blobinput[0]->width();
	cout<<"numofpixels = "<<numofpixels<<endl;
	const float *shape = blobinput[0]->cpu_data(); //(float*)malloc(sizeof(float*)*numofpixels); //short is Pel's data type, convert to float
	
	Pel *block = (Pel*)malloc(sizeof(Pel*)*numofpixels);
	for (int i = 0; i < numofpixels; i++)
	{
		block[i] = (short) (shape[i] * (pow(2,bitdepth - 1)));
		cout<<"block[i] = "<<block[i]<<endl;

	}
	
	toutput->initPattern(block, blobinput[0]->width(), blobinput[0]->height(), stride, bitdepth, roiX, roiY); 

	cout<<"--------------------converted BlobtoTcomPattern------------------------"<<endl;

}



/*Convert Tcompattern to Blob Caffe*/	
void CNN::convertTComPatterntoBlob(TComPattern* tinput, vector<Blob< float > *> b_input_CNN  )
{
	//convertTComPatterntoBlob(input, b_input_CNN); //convert TCompattern to CNN input
	Pel* block = tinput->getROIY();
	// Get the blob
	cout<<"blob"<<endl;
	Blob<float>* blob = new Blob<float>(1, 1,  tinput->getROIYHeight(),tinput->getROIYWidth());
	
	float * a = (float *)malloc(sizeof(float*)*(tinput->getROIYHeight() * tinput->getROIYWidth()));
	for(int i = 0; i < tinput->getROIYHeight() * tinput->getROIYWidth(); i++)
	{
		a[i] = (float)block[i]/(pow(2,tinput->getBitDepthY()) - 1);
	}
	float* b=(float *)malloc(sizeof(float*)*(tinput->getROIYHeight() * tinput->getROIYWidth() * 16));
	myresize_up(a, tinput->getROIYHeight(), tinput->getROIYWidth(),b);
	
	//blob->set_gpu_data(a);
	blob->set_cpu_data(b); 
	
	cout<<"blob->set_cpu_data(a)"<<endl;
	blob->Reshape(1, 1,  tinput->getROIYHeight(),tinput->getROIYWidth());
	
	b_input_CNN.push_back(blob);
	cout<<	"b_input_CNN.push_back(blob)"<<endl;
}


void CNN::Predict(TComPattern* tinput, TComPattern* output)
{	
	vector< Blob< float > * > b_input_CNN=net_->input_blobs();
	
	Pel* block = tinput->getROIY();
	// Get the blob
	cout<<"blob"<<endl;
	Blob<float>* blob = new Blob<float>(1, 1,  tinput->getROIYHeight(),tinput->getROIYWidth());
	
	float * a = (float *)malloc(sizeof(float*)*(tinput->getROIYHeight() * tinput->getROIYWidth()));
	for(int i = 0; i < tinput->getROIYHeight() * tinput->getROIYWidth(); i++)
	{
		a[i] = (float)block[i]/(pow(2,tinput->getBitDepthY()) - 1);
	}
	float* b=(float *)malloc(sizeof(float*)*(tinput->getROIYHeight() * tinput->getROIYWidth() * 16));
	myresize_up(a, tinput->getROIYHeight(), tinput->getROIYWidth(),b);
	
	//blob->set_gpu_data(a);
	blob->set_cpu_data(b); 
	blob->Reshape(1, 1,  tinput->getROIYHeight(),tinput->getROIYWidth());
	
	
	b_input_CNN.push_back(blob);
	int rows = tinput->getROIYHeight();
	int cols = tinput->getROIYWidth();
	
	Datum datum;
	datum.set_channels(1);
	datum.set_height(rows);
	datum.set_width(cols);	
	datum.set_data(b, rows*cols);
	vector<Datum> vectordatum;
	vectordatum.push_back(datum);
	MemoryDataLayer<float> *memory_data_layer = (MemoryDataLayer<float> *)net_->layer_by_name("data").get();
	memory_data_layer->Reset(b, b, 1);
	float type = 0.0;
	vector< Blob< float > * > b_output = CNN::net_->Forward(&type);//b_input_CNN,&type);//b_input_CNN, NULL);
	cout<<"------predict----"<<endl;
	
	//add image into output
	//float* adjust_output = (float *)malloc(sizeof(float*)*(tinput->getROIYHeight() * tinput->getROIYWidth()));
	cout<<"declare adjust_output"<<endl; 
	//cout<<"input size: "<< net_->input_blobs()[0]->height() <<"x"<< net_->input_blobs()[0]->width() <<endl;
	cout<<"output size: "<< net_->output_blobs()[0]->height() <<"x"<< net_->output_blobs()[0]->width()<<endl;
	
	//Blob< float>* inputCNN=net_->input_blobs()[0];//->cpu_data();
	cout<<"outputCNN"<<b_output[0]->height()<<"x"<<b_output[0]->width()<<endl;
	/*cout<<"get inputCNN"<<endl;
	const float* outputCNN=b_output[0]->cpu_data();
	cout<<"declare parameter before fixed integer pixels"<<endl;
	FixedIntegerPixels(inputCNN, outputCNN, b_output[0]->height(), b_output[0]->width(), adjust_output);
	free(adjust_output);
	cout<<"FixedIntegerPixels"<<endl;
	
	int stride = tinput->getPatternLStride();
	int bitdepth = tinput->getBitDepthY();
	int roiX =  tinput->getROIYPosX();
	int roiY = tinput->getROIYPosY();
	convertBlobtoTcomPattern(b_output, output, stride, bitdepth, roiX, roiY); //convert CNN output to TCompattern
	*/
}

void CNN::FixedIntegerPixels(const float* inputCNN, const float* outputCNN, int height, int width, float* output_process)
{
	cout<<height<<"x"<<width<<endl;
	for (int i = 0; i < height; i+=4)//skip row 1, 2, m
	{
		for (int j = 0; j < width; j+=4)
		{
			if(i%4==0 && j%4==0) 
				output_process[i*width+j] =  inputCNN[i*width+j]; //keep the integer pixels
			else
				output_process[i*width+j] =  inputCNN[i*width+j] + outputCNN[i*width+j];
			cout<<"i*width+j = "<<i*width+j<<endl;
		}
	}
}


void CNN::myresize_up(float* input, int height, int width, float* output)
{
		
	for (int i = 0; i < height*width;i++)
	{
		//row and col if convert to 2D array
		int row = i/height;
		int col = i%width;
		output[i]=input[row/CNNscale+col/CNNscale];
	}
	
}



