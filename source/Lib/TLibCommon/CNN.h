
/** \file     CNN.cpp
    \brief    CNN-based interpolation
*/

#ifndef __CNN__
#define __CNN__



//include files
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>
#include <ctime>
#include "TComPattern.h"
#include "TComYuv.h"
#include "CommonDef.h"
#include "TLibEncoder/TEncCfg.h"
using namespace std;



//! \ingroup TLibCommon
//! \{
 
// ====================================================================================================================
// Class definition
// ====================================================================================================================

const int CNNscale = 4;
enum ConvolutionType 
{
   /* Return the full convolution, including border */
  CONVOLUTION_FULL, 
  /* Return only the part that corresponds to the original image */
  CONVOLUTION_SAME,
  /* Return only the submatrix containing elements that were not influenced by the border */
  CONVOLUTION_VALID
};
class CNN
{
private:	
	string modelfile;
	string trainedweight;
	int isInit;
	int fbp;
public:
	CNN(){isInit = 0; fbp = 0;}
	CNN( const string& model_file,  const string& trainedweight);//link to net and link to trained weight 
	int CNNisInited(){return isInit;};
void init(const string& model_file, const string& trainedweight);

void getDataWithoutPadding(Pel * input, int inputheight, int inputwidth, int padding, int height, int width, Pel *dst, int dstStride);
Void CopyToHEVC(Pel* src, int srcStride, int cxHeight, int cxWidth, Pel* dst, int dstStride);
Void ToHEVCData(Pel* src, int cxHeight, int cxWidth, int inter,  int yFrac, int xFrac, Pel* dst, int dstStride);


Void PredictFromHEVCData(TComYuv m_filteredBlock[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS], int height, int width, int bitdepth, Pel * output, int stride, string SaveName);//Predict without changing size
	
};
//! \}

#endif




























