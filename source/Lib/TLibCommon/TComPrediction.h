/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComPrediction.h
    \brief    prediction class (header)
*/

#ifndef __TCOMPREDICTION__
#define __TCOMPREDICTION__


// Include files
#include "TComYuv.h"
#include "TComInterpolationFilter.h"
#include "TComWeightPrediction.h"

#include <stdlib.h>
#include "TLibEncoder/TEncCfg.h"



// forward declaration
class TComMv;
class TComTU; 

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
typedef enum PRED_BUF_E
{
  PRED_BUF_UNFILTERED=0,
  PRED_BUF_FILTERED=1,
  NUM_PRED_BUF=2
} PRED_BUF;

static const UInt MAX_INTRA_FILTER_DEPTHS=5;

class TComPrediction : public TComWeightPrediction
{
private:
  static const UChar m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];
	//PyObject* pInt;

protected:
  int pic0_atv;
  int pic1_atv;
  int pic2_atv;
  int pic3_atv;
  int pic4_atv;
  int pic5_atv;
  int pic6_atv;
  int pic7_atv;
  int pic8_atv;
  int pic9_atv;
  int pic10_atv;
  
  int enc_pic0_atv;
  int enc_pic1_atv;
  int enc_pic2_atv;
  int enc_pic3_atv;
  int enc_pic4_atv;
  int enc_pic5_atv;
  int enc_pic6_atv;
  int enc_pic7_atv;
  int enc_pic8_atv;
  int enc_pic9_atv;
  int write_DCTIF_interp;
  TEncCfg*        m_pcComCfg;

  Pel*      m_piYuvExt[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  Int       m_iYuvExtSize;

  TComYuv   m_acYuvPred[NUM_REF_PIC_LIST_01];
  TComYuv   m_cYuvPredTemp;
  TComYuv m_filteredBlock[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComYuv m_filteredBlockTmp[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  
  TComPicYuv interp_ref_pic0[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic3[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic5[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic6[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic7[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic8[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic9[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv interp_ref_pic10[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
    

  TComInterpolationFilter m_if;
  CNN myCNN;
 int assumeCNN;
  Pel*   m_pLumaRecBuffer;       ///< array for downsampled reconstructed luma sample
  Int    m_iLumaRecStride;       ///< stride of #m_pLumaRecBuffer array

  Void xPredIntraAng            ( Int bitDepth, const Pel* pSrc, Int srcStride, Pel* pDst, Int dstStride, UInt width, UInt height, ChannelType channelType, UInt dirMode, const Bool bEnableEdgeFilters );
  Void xPredIntraPlanar         ( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height );

  // motion compensation functions
  Void xPredInterUni(TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi = false);
  Void xPredInterBi(TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvPred);
  Void xPredInterUni_DCTIF            ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi=false          );
  Void xPredInterBi_DCTIF             ( TComDataCU* pcCU,                          UInt uiPartAddr,               Int iWidth, Int iHeight,                         TComYuv* pcYuvPred          );
  Void xPredInterUni_CNN(TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi = false);
  Void xPredInterBi_CNN(TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvPred);
  
  
  Void xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth );

  void getdata_withmargin(Pel* input, int inputStride, int FrameHeight, int FrameWidth, int width, int height, Pel *dst, int dstStride, int marginY, int marginX, int posy, int posx);
  void copytoMatrix(Pel * src, int srcStride, Pel * dst, int dstStride, int width, int height);
  void xPredInterBlkCNN(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height,
		TComYuv *dstPic, Bool bi, const Int bitDepth, RefPicList eRefPixList, int iRefIdx);

  Void copy(Pel *src, int cxHeight, int cxWidth, Pel *dstPtr, int dstStride);

  Void xWeightedAverage         ( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvDst, const BitDepths &clipBitDepths  );

  Void xGetLLSPrediction ( const Pel* pSrc0, Int iSrcStride, Pel* pDst0, Int iDstStride, UInt uiWidth, UInt uiHeight, UInt uiExt0, const ChromaFormat chFmt  DEBUG_STRING_FN_DECLARE(sDebug) );

  Void xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType );
  Bool xCheckIdenticalMotion    ( TComDataCU* pcCU, UInt PartAddr);
  Void destroy();

  
public:
  TComPrediction();
  virtual ~TComPrediction();
  void WholeFrameInterp(const ComponentID compID, Pel * ReconFrame, int recFrameStride, int orWholeFrameHeight, int orWholeFrameWidth, int bitDepth, string SaveName);
  void FrameInterpolated_Perpixel(Pel * ReconFrame, int reconStride, int orWholeFrameHeight, int orWholeFrameWidth, Pel * or_ref_pel, int or_ref_stride, int bitDepth, string SaveName);
  void getdata_from_pos_withmargin(Pel* input, int inputStride, int FrameHeight, int FrameWidth, int width, int height, Pel *dst, int dstStride, int marginY, int marginX, int posy, int posx);

  Void    initTempBuff(ChromaFormat chromaFormatIDC);

  ChromaFormat getChromaFormat() const { return m_cYuvPredTemp.getChromaFormat(); }

  // inter
  Void motionCompensation         ( TComDataCU*  pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1);
  Void motionCompensation_Selection(TComDataCU*  pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList = REF_PIC_LIST_X, Int iPartIdx = -1, int InterpType=1);

  // motion vector prediction
  Void getMvPredAMVP              ( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred );

  // Angular Intra
  Void predIntraAng               ( const ComponentID compID, UInt uiDirMode, Pel *piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM = false );

  Pel  predIntraGetPredValDC      ( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight);

  Pel*  getPredictorPtr           ( const ComponentID compID, const Bool bUseFilteredPredictions )
  {
    return m_piYuvExt[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED];
  }
void getdata_addmargin(Pel* input, int inputStride, int FrameHeight, int FrameWidth, Pel *dst, int dstStride, int marginY, int marginX, int posy, int posx);
Pel * readoutputfile(const char * outputpath, int height, int width);

  // This function is actually still in TComPattern.cpp
  /// set parameters from CU data for accessing intra data
  Void initIntraPatternChType ( TComTU &rTu,
                              const ComponentID compID, const Bool bFilterRefSamples
                              DEBUG_STRING_FN_DECLARE(sDebug)
                              );

  static Bool filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled);

  static Bool UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode);

#if MCTS_ENC_CHECK
  Bool checkTMctsMvp(TComDataCU* pcCU, Int partIdx = -1);
#endif
};

//! \}

#endif // __TCOMPREDICTION__
