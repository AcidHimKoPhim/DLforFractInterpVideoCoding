
#include "CNN.h"
#include "TComPattern.h"

/** \file     CNN.cpp
    \brief    CNN-based interpolation
*/

//
//load network parameters
//
void CNN::init(const string& _model_file, const string& _trainedweight)
{	
	modelfile = _model_file;
	trainedweight = _trainedweight;
	isInit = 1;
	fbp=0;
  
}
CNN::CNN(const string& _model_file, const string& _trainedweight)
{
	modelfile = _model_file;
	trainedweight = _trainedweight;
	isInit = 1;

}

/**
Add the residual output to input. If the pixel is integer pixel then kept
*/
/*
Mat CNN::FixedIntegerPixels(Mat inputCNN, Mat outputCNN, int bitdepth)
{
	Mat fixed = Mat(outputCNN.rows, outputCNN.cols, CV_64FC1); 
	int height = outputCNN.rows;
	int width = outputCNN.cols;
	for (int i = 0; i < height; i++)//skip row 1, 2, m
	{
		for (int j = 0; j < width; j++)
		{
			if(j%4==0 && i%4 == 0) 
			{
				fixed.at<double>(i,j) =  inputCNN.at<double>(i,j)*(pow(2,bitdepth) - 1); //keep the integer pixels
			}
			else
			{
				fixed.at<double>(i,j) = outputCNN.at<double>(i,j)*(pow(2,bitdepth) - 1);
				if(fixed.at<double>(i,j) < 0 ) fixed.at<double>(i,j) = 0;
				if(fixed.at<double>(i,j) > pow(2,bitdepth) - 1 ) fixed.at<double>(i,j)= pow(2,bitdepth) - 1;
			}
		}
	}
	return fixed; 
}
*/

//Predict whole frame
//output has no padding
//if we want some paddr motion search and motion compensation, duplicate it in ME and MC
Void CNN::PredictFromHEVCData(TComYuv m_filteredBlock[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS], int height, int width, int bitdepth, Pel * output, int outputstride, string SaveName)
{	
	//this padding is used for convolution. If After feed forward, convolution reduce to X pixels, then padding = X
	int padding = 16;
	int inputheight = height*4+padding*2;
	int inputwidth = width*4+padding*2;
	
	Pel * CNNinput = (Pel*)malloc(sizeof(Pel*)*(inputheight * inputwidth));
	 
	//cout<<"initializetion "<<endl;
	for(int r = 0; r <inputheight; r++)
	{
		for(int c = 0; c < inputwidth; c++)
		{
			CNNinput[r*inputwidth + c] = 1;
		}
	}

	//fill the square inside without padding
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{ 
			Pel * src = m_filteredBlock[i][j].getAddr(COMPONENT_Y);
			int srcStride =  m_filteredBlock[i][j].getStride(COMPONENT_Y);
			/*cout<<endl<<"src from"<<i<<","<<j<<endl;
			cout<<endl<<"src from"<<i<<","<<j<<endl;
			cout<<endl<<"src from"<<i<<","<<j<<endl;
			cout<<endl<<"src from"<<i<<","<<j<<endl; 
			*/
			for(int r = i+padding; r <inputheight; r+=4)
			{
				for(int c = j+padding; c < inputwidth-padding; c+=4)
				{
					//cout<<r*inputwidth+ c<<"\t";
					CNNinput[r*inputwidth + c]=src[(int)((c-padding)/4)] ;
					//if ( (int)((c-padding)/4) < width && (int)((r-padding)/4) < height) cout<<src[(int)((c-padding)/4)]<<"\t";CNNinput
					//if ( (int)((c-padding)/4) < width && (int)((r-padding)/4) < height) cout<<CNNinput[r*inputwidth + c]<<"\t";
					
					//if(src[(int)((c-padding)/4)] == 0) cout<<"found 0 in "<<i<<","<<j<<"  position in inputCNN: "<<r<<", "<<(int)((c-padding)/4)<<endl;
				}
				//if((int)((r-padding)/4) < height) cout<<endl;
				src+=srcStride;
			}		 
		}
	}
	//cout<<"set inside square"<<endl;
	
	//fill the top, bottom, left, right padding (margin padding)
	/* ___________
	//|a********b|
	//|*|      |*|
	//|*|      |*|
	//|*|      |*|
	//|*|______|*|
	//|c********d|
	*/
	for(int r = 0; r <inputheight; r++)
	{
		for(int c = 0; c < inputwidth; c++)
		{
			if(c >= padding && c< inputwidth - padding && r<inputheight-padding&& r>=padding) continue;
			else if(c<padding && r<padding)//a
			{
				CNNinput[r*inputwidth + c] = CNNinput[padding*inputwidth + padding]; //inputCNN.at<double>(padding,padding);
			}
			else if(r<padding && c>= padding+width*4)//b
			{
				CNNinput[r*inputwidth + c] = CNNinput[padding*inputwidth + padding+width*4-1];//inputCNN.at<double>(padding, padding+width*4-1);
			}
			else if(r>= padding+height*4 && c<padding)//c
			{
				CNNinput[r*inputwidth + c] = CNNinput[(padding+height*4-1)*inputwidth + padding];//inputCNN.at<double>(padding+height*4-1, padding);
			}
			else if(r>= padding+height*4 && c>= padding+width*4)//d
			{
				CNNinput[r*inputwidth + c] =  CNNinput[(padding+height*4-1)*inputwidth + (padding+width*4-1)];//inputCNN.at<double>(padding+height*4-1, padding+width*4-1);
			} 
			else
			{
				if(r< padding) 
					CNNinput[r*inputwidth + c] = CNNinput[padding*inputwidth + c];//inputCNN.at<double>(padding,c);
				else if(r>padding+4*height-1)
					CNNinput[r*inputwidth + c] = CNNinput[(height*4+padding-1)*inputwidth + c];//inputCNN.at<double>(height*4+padding-1,c);
				else if (c < padding)
					CNNinput[r*inputwidth + c] = CNNinput[r*inputwidth + padding];//inputCNN.at<double>(r,padding);
				else//c>padding
					CNNinput[r*inputwidth + c] = CNNinput[r*inputwidth + padding+width*4-1];//inputCNN.at<double>(r,padding+width*4-1);
			}
		}
	}
	
	Pel * croppedinput = (Pel*)malloc(sizeof(Pel*)*(height*4 * width*4));
	Pel * croppedoutput = (Pel*)malloc(sizeof(Pel*)*(height*4 * width*4));
	for(int r = 0; r <height*4; r++)
	{
		for(int c = 0; c < width*4; c++)
		{
			croppedinput[r*width*4 + c] = CNNinput[(r+padding)*inputwidth + c+padding];;//inputCNN.at<double>(i+padding,j+padding);
			croppedoutput[r*width*4 + c] = CNNinput[(r+padding)*inputwidth + c+padding];;//inputCNN.at<double>(i+padding,j+padding);
		}
	}
	
	//store file for python forward
	if(SaveName != "")
	{
		cout<<"modelfile "<<modelfile<<endl;
		cout<<"trainedweight "<<trainedweight<<endl;

		ifstream myfile1 (SaveName.c_str());
		if(myfile1.good())
		{	
			ofstream myfile2(SaveName.c_str());
			for(int i = 0; i < inputheight; i++)
			{
				for(int j = 0; j < inputwidth; j++)
				{
					myfile2<<CNNinput[i*inputwidth + j];//inputCNN.at<double>(i+padding,j+padding) * 255;
					myfile2<<"\t";
				} 
				myfile2<<"\n";
			}
			myfile2.close();
		}
		myfile1.close();
	} 
	getDataWithoutPadding(croppedoutput,height*4, width*4, 0, height*4, width*4, output, outputstride);
	
}

void CNN::getDataWithoutPadding(Pel * input, int inputheight, int inputwidth, int padding, int height, int width, Pel *dst, int dstStride)
{
	int heightindex = 0, widthindex =  0;
	for(int i = padding; i < inputheight; i++)
	{
		widthindex = 0;
		for(int j = padding; j < inputwidth; j++)
		{
			dst[widthindex++] = input[i*inputwidth+j];
			if(widthindex == width) break;
		}
		if(heightindex == height) break;
		heightindex++;
		dst += dstStride;
	}
}

Void CNN::CopyToHEVC(Pel* src, int srcStride, int cxHeight, int cxWidth, Pel* dst, int dstStride)
{
	for(int i = 0; i < cxHeight; i++)
	{
		for(int j = 0; j < cxWidth; j++)
		{
			dst[j]=src[j];
		}
		dst+=dstStride;
		src +=srcStride;
	}
}

Void CNN::ToHEVCData(Pel* src, int cxHeight, int cxWidth, int inter,  int yFrac, int xFrac, Pel* dst, int dstStride)
{
	
	for(int i = yFrac; i < cxHeight*inter; i+=inter)
	{
		int dstindex = 0;
		for(int j = xFrac; j < cxWidth*inter; j+=inter)
		{
			dst[dstindex]=src[i*cxWidth*inter+j];
			dstindex++;
		}
		dst+=dstStride;
	}
}
