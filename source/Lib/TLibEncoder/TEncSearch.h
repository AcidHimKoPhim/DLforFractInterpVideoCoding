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

/** \file     TEncSearch.h
    \brief    encoder search class (header)
*/

#ifndef __TENCSEARCH__
#define __TENCSEARCH__

// Include files
#include "TLibCommon/TComYuv.h"
#include "TLibCommon/TComMotionInfo.h"
#include "TLibCommon/TComPattern.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComPic.h"
#include "TLibCommon/TComRectangle.h"
#include "TEncEntropy.h"
#include "TEncSbac.h"
#include "TEncCfg.h"


//! \ingroup TLibEncoder
//! \{

class TEncCu;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

static const UInt MAX_NUM_REF_LIST_ADAPT_SR=2;
static const UInt MAX_IDX_ADAPT_SR=33;
static const UInt NUM_MV_PREDICTORS=3;

/// encoder search class
class TEncSearch : public TComPrediction //inherientance TComPrediction class
{
private:
  TCoeff**        m_ppcQTTempCoeff[MAX_NUM_COMPONENT /* 0->Y, 1->Cb, 2->Cr*/];
#if ADAPTIVE_QP_SELECTION
  TCoeff**        m_ppcQTTempArlCoeff[MAX_NUM_COMPONENT];
#endif
  UChar*          m_puhQTTempTrIdx;
  UChar*          m_puhQTTempCbf[MAX_NUM_COMPONENT];

  TComYuv*        m_pcQTTempTComYuv;
  TComYuv         m_tmpYuvPred; // To be used in xGetInterPredictionError() to avoid constant memory allocation/deallocation

  SChar*          m_phQTTempCrossComponentPredictionAlpha[MAX_NUM_COMPONENT];
  Pel*            m_pSharedPredTransformSkip[MAX_NUM_COMPONENT];
  TCoeff*         m_pcQTTempTUCoeff[MAX_NUM_COMPONENT];
  UChar*          m_puhQTTempTransformSkipFlag[MAX_NUM_COMPONENT];
  TComYuv         m_pcQTTempTransformSkipTComYuv;
#if ADAPTIVE_QP_SELECTION
  TCoeff*         m_ppcQTTempTUArlCoeff[MAX_NUM_COMPONENT];
#endif
  TComPicYuv m_interp_ref_pic0[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic3[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic5[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic6[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic7[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic8[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic9[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  TComPicYuv m_interp_ref_pic10[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS*LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS];
  
  int m_pic0_atv;
  int m_pic1_atv;
  int m_pic2_atv;
  int m_pic3_atv;
  int m_pic4_atv;
  int m_pic5_atv;
  int m_pic6_atv;
  int m_pic7_atv;
  int m_pic8_atv;
  int m_pic9_atv;
  int m_pic10_atv;
protected:
  // interface to option
  TEncCfg*        m_pcEncCfg;
  // interface to classes
  TComTrQuant*    m_pcTrQuant;
  TComRdCost*     m_pcRdCost;
  TEncEntropy*    m_pcEntropyCoder;

  // ME parameters
  Int             m_iSearchRange;
  Int             m_bipredSearchRange; // Search range for bi-prediction
  MESearchMethod  m_motionEstimationSearchMethod;
  Int             m_aaiAdaptSR[MAX_NUM_REF_LIST_ADAPT_SR][MAX_IDX_ADAPT_SR];
  TComMv          m_acMvPredictors[NUM_MV_PREDICTORS]; // Left, Above, AboveRight. enum MVP_DIR first NUM_MV_PREDICTORS entries are suitable for accessing.

  // RD computation
  TEncSbac***     m_pppcRDSbacCoder;
  TEncSbac*       m_pcRDGoOnSbacCoder;
  DistParam       m_cDistParam;

  // Misc.
  Pel*            m_pTempPel;

  // AMVP cost computation
  // UInt            m_auiMVPIdxCost[AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS];
  UInt            m_auiMVPIdxCost[AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS+1]; //th array bounds

  TComMv          m_integerMv2Nx2N[NUM_REF_PIC_LIST_01][MAX_NUM_REF];

  Bool            m_isInitialized;
public:
  TEncSearch();
  virtual ~TEncSearch();

  Void init(TEncCfg*       pcEncCfg,
            TComTrQuant*   pcTrQuant,
            Int            iSearchRange,
            Int            bipredSearchRange,
            MESearchMethod motionEstimationSearchMethod,
            const UInt     maxCUWidth,
            const UInt     maxCUHeight,
            const UInt     maxTotalCUDepth,
            TEncEntropy*   pcEntropyCoder,
            TComRdCost*    pcRdCost,
            TEncSbac***    pppcRDSbacCoder,
            TEncSbac*      pcRDGoOnSbacCoder );

  Void destroy();

protected:

  /// sub-function for motion vector refinement used in fractional-pel accuracy
  Distortion  xPatternRefinement( TComPattern* pcPatternKey,
                                  TComMv baseRefMv,
                                  Int iFrac, TComMv& rcMvFrac, Bool bAllowUseOfHadamard
                                 );  
   Distortion  xPatternRefinement_preprocess( TComPattern* pcPatternKey,
                                  TComMv baseRefMv,
                                  Int iFrac, TComMv& rcMvFrac, Bool bAllowUseOfHadamard
								  , TComYuv m_filteredBlock_local[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]
                                 );

  typedef struct
  {
    const Pel*  piRefY;
    Int         iYStride;
    Int         iBestX;
    Int         iBestY;
    UInt        uiBestRound;
    UInt        uiBestDistance;
    Distortion  uiBestSad;
    UChar       ucPointNr;
  } IntTZSearchStruct;

  // sub-functions for ME
  __inline Void xTZSearchHelp         ( const TComPattern* const pcPatternKey, IntTZSearchStruct& rcStruct, const Int iSearchX, const Int iSearchY, const UChar ucPointNr, const UInt uiDistance );
  __inline Void xTZ2PointSearch       ( const TComPattern* const pcPatternKey, IntTZSearchStruct& rcStruct, const TComMv* const pcMvSrchRngLT, const TComMv* const pcMvSrchRngRB );
  __inline Void xTZ8PointSquareSearch ( const TComPattern* const pcPatternKey, IntTZSearchStruct& rcStruct, const TComMv* const pcMvSrchRngLT, const TComMv* const pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist );
  __inline Void xTZ8PointDiamondSearch( const TComPattern* const pcPatternKey, IntTZSearchStruct& rcStruct, const TComMv* const pcMvSrchRngLT, const TComMv* const pcMvSrchRngRB, const Int iStartX, const Int iStartY, const Int iDist, const Bool bCheckCornersAtDist1 );

  Void xGetInterPredictionError_CNN( TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, Distortion& ruiSAD, Bool Hadamard );
  Void xGetInterPredictionError_DCTIF(TComDataCU* pcCU, TComYuv* pcYuvOrg, Int iPartIdx, Distortion& ruiSAD, Bool Hadamard);


public:
  Void  estIntraPredLumaQT      ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcPredYuv,
                                  TComYuv*    pcResiYuv,
                                  TComYuv*    pcRecoYuv,
                                  Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE]
                                  DEBUG_STRING_FN_DECLARE(sDebug));

  Void  estIntraPredChromaQT    ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcPredYuv,
                                  TComYuv*    pcResiYuv,
                                  TComYuv*    pcRecoYuv,
                                  Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE]
                                  DEBUG_STRING_FN_DECLARE(sDebug));

  /// encoder estimation - inter prediction (non-skip)
  Void predInterSearch_DCTIF          ( TComDataCU* pcCU,
                                  TComYuv*    pcOrgYuv,
                                  TComYuv*    pcPredYuv,
                                  TComYuv*    pcResiYuv,
                                  TComYuv*    pcRecoYuv
                                  DEBUG_STRING_FN_DECLARE(sDebug),
                                  Bool        bUseRes = false
#if AMP_MRG
                                 ,Bool        bUseMRG = false
#endif
                                );
  /// encoder estimation - inter prediction (non-skip)
	Void predInterSearch_CNN(TComDataCU* pcCU,
		TComYuv*    pcOrgYuv,
		TComYuv*    pcPredYuv,
		TComYuv*    pcResiYuv,
		TComYuv*    pcRecoYuv
		DEBUG_STRING_FN_DECLARE(sDebug),
		Bool        bUseRes = false
	#if AMP_MRG
		, Bool        bUseMRG = false
	#endif
	);

	Void predInterSearch(TComDataCU* pcCU,
		TComYuv*    pcOrgYuv,
		TComYuv*    pcPredYuv,
		TComYuv*    pcResiYuv,
		TComYuv*    pcRecoYuv
		DEBUG_STRING_FN_DECLARE(sDebug),
		Bool        bUseRes = false
#if AMP_MRG
		, Bool        bUseMRG = false
#endif
	);


  /// encode residual and compute rd-cost for inter mode
  Void encodeResAndCalcRdInterCU( TComDataCU* pcCU,
                                  TComYuv*    pcYuvOrg,
                                  TComYuv*    pcYuvPred,
                                  TComYuv*    pcYuvResi,
                                  TComYuv*    pcYuvResiBest,
                                  TComYuv*    pcYuvRec,
                                  Bool        bSkipResidual
                                  DEBUG_STRING_FN_DECLARE(sDebug)
								  		);

  /// set ME search range
  Void setAdaptiveSearchRange   ( Int iDir, Int iRefIdx, Int iSearchRange) { assert(iDir < MAX_NUM_REF_LIST_ADAPT_SR && iRefIdx<Int(MAX_IDX_ADAPT_SR)); m_aaiAdaptSR[iDir][iRefIdx] = iSearchRange; }

  Void xEncPCM    (TComDataCU* pcCU, UInt uiAbsPartIdx, Pel* piOrg, Pel* piPCM, Pel* piPred, Pel* piResi, Pel* piReco, UInt uiStride, UInt uiWidth, UInt uiHeight, const ComponentID compID );
  Void IPCMSearch (TComDataCU* pcCU, TComYuv* pcOrgYuv, TComYuv* rpcPredYuv, TComYuv* rpcResiYuv, TComYuv* rpcRecoYuv );
protected:

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

  Void  xEncSubdivCbfQT           ( TComTU      &rTu,
                                    Bool         bLuma,
                                    Bool         bChroma );

  Void  xEncCoeffQT               ( TComTU &rTu,
                                    ComponentID  component,
                                    Bool         bRealCoeff );
  Void  xEncIntraHeader           ( TComDataCU*  pcCU,
                                    UInt         uiTrDepth,
                                    UInt         uiAbsPartIdx,
                                    Bool         bLuma,
                                    Bool         bChroma );
  UInt  xGetIntraBitsQT           ( TComTU &rTu,
                                    Bool         bLuma,
                                    Bool         bChroma,
                                    Bool         bRealCoeff );

  UInt  xGetIntraBitsQTChroma    ( TComTU &rTu,
                                   ComponentID compID,
                                   Bool          bRealCoeff );

  Void  xIntraCodingTUBlock       (       TComYuv*      pcOrgYuv,
                                          TComYuv*      pcPredYuv,
                                          TComYuv*      pcResiYuv,
                                          Pel           resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                    const Bool          checkCrossCPrediction,
                                          Distortion&   ruiDist,
                                    const ComponentID   compID,
                                          TComTU        &rTu
                                    DEBUG_STRING_FN_DECLARE(sTest)
                                         ,Int           default0Save1Load2 = 0
                                   );

  Void  xRecurIntraCodingLumaQT   ( TComYuv*    pcOrgYuv,
                                    TComYuv*    pcPredYuv,
                                    TComYuv*    pcResiYuv,
                                    Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                    Distortion& ruiDistY,
#if HHI_RQT_INTRA_SPEEDUP
                                    Bool         bCheckFirst,
#endif
                                    Double&      dRDCost,
                                    TComTU      &rTu
                                    DEBUG_STRING_FN_DECLARE(sDebug));

  Void  xSetIntraResultLumaQT     ( TComYuv*     pcRecoYuv,
                                    TComTU &rTu);

  Void xStoreCrossComponentPredictionResult  (       Pel    *pResiLuma,
                                               const Pel    *pBestLuma,
                                                     TComTU &rTu,
                                               const Int     xOffset,
                                               const Int     yOffset,
                                               const Int     strideResi,
                                               const Int     strideBest );

  SChar xCalcCrossComponentPredictionAlpha   (       TComTU &rTu,
                                               const ComponentID compID,
                                               const Pel*        piResiL,
                                               const Pel*        piResiC,
                                               const Int         width,
                                               const Int         height,
                                               const Int         strideL,
                                               const Int         strideC );

  Void  xRecurIntraChromaCodingQT ( TComYuv*    pcOrgYuv,
                                    TComYuv*    pcPredYuv,
                                    TComYuv*    pcResiYuv,
                                    Pel         resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE],
                                    Distortion& ruiDist,
                                    TComTU      &rTu
                                    DEBUG_STRING_FN_DECLARE(sDebug));

  Void  xSetIntraResultChromaQT   ( TComYuv*    pcRecoYuv, TComTU &rTu);

  Void  xStoreIntraResultQT       ( const ComponentID compID, TComTU &rTu);
  Void  xLoadIntraResultQT        ( const ComponentID compID, TComTU &rTu);


  // -------------------------------------------------------------------------------------------------------------------
  // Inter search (AMP)
  // -------------------------------------------------------------------------------------------------------------------

  Void xEstimateMvPredAMVP_CNN( TComDataCU* pcCU,
                                    TComYuv*    pcOrgYuv,
                                    UInt        uiPartIdx,
                                    RefPicList  eRefPicList,
                                    Int         iRefIdx,
                                    TComMv&     rcMvPred,
                                    Bool        bFilled = false
                                  , Distortion* puiDistBiP = NULL
                                     );

  Void xCheckBestMVP_CNN( TComDataCU* pcCU,
                                    RefPicList  eRefPicList,
                                    TComMv      cMv,
                                    TComMv&     rcMvPred,
                                    Int&        riMVPIdx,
                                    UInt&       ruiBits,
                                    Distortion& ruiCost );

  Distortion xGetTemplateCost_CNN    ( TComDataCU*  pcCU,
                                    UInt        uiPartAddr,
                                    TComYuv*    pcOrgYuv,
                                    TComYuv*    pcTemplateCand,
                                    TComMv      cMvCand,
                                    Int         iMVPIdx,
                                    Int         iMVPNum,
                                    RefPicList  eRefPicList,
                                    Int         iRefIdx,
                                    Int         iSizeX,
                                    Int         iSizeY
                                   );
  Void xEstimateMvPredAMVP_DCTIF(TComDataCU* pcCU,
	  TComYuv*    pcOrgYuv,
	  UInt        uiPartIdx,
	  RefPicList  eRefPicList,
	  Int         iRefIdx,
	  TComMv&     rcMvPred,
	  Bool        bFilled = false
	  , Distortion* puiDistBiP = NULL
  );

  Void xCheckBestMVP_DCTIF(TComDataCU* pcCU,
	  RefPicList  eRefPicList,
	  TComMv      cMv,
	  TComMv&     rcMvPred,
	  Int&        riMVPIdx,
	  UInt&       ruiBits,
	  Distortion& ruiCost);

  Distortion xGetTemplateCost_DCTIF(TComDataCU*  pcCU,
	  UInt        uiPartAddr,
	  TComYuv*    pcOrgYuv,
	  TComYuv*    pcTemplateCand,
	  TComMv      cMvCand,
	  Int         iMVPIdx,
	  Int         iMVPNum,
	  RefPicList  eRefPicList,
	  Int         iRefIdx,
	  Int         iSizeX,
	  Int         iSizeY
  );
  Void xEstimateMvPredAMVP(TComDataCU* pcCU,
	  TComYuv*    pcOrgYuv,
	  UInt        uiPartIdx,
	  RefPicList  eRefPicList,
	  Int         iRefIdx,
	  TComMv&     rcMvPred,
	  Bool        bFilled = false
	  , Distortion* puiDistBiP = NULL
  );

  Void xCheckBestMVP(TComDataCU* pcCU,
	  RefPicList  eRefPicList,
	  TComMv      cMv,
	  TComMv&     rcMvPred,
	  Int&        riMVPIdx,
	  UInt&       ruiBits,
	  Distortion& ruiCost);

  Distortion xGetTemplateCost(TComDataCU*  pcCU,
	  UInt        uiPartAddr,
	  TComYuv*    pcOrgYuv,
	  TComYuv*    pcTemplateCand,
	  TComMv      cMvCand,
	  Int         iMVPIdx,
	  Int         iMVPNum,
	  RefPicList  eRefPicList,
	  Int         iRefIdx,
	  Int         iSizeX,
	  Int         iSizeY
  );

  Void xCopyAMVPInfo              ( AMVPInfo*   pSrc, AMVPInfo* pDst );
  UInt xGetMvpIdxBits             ( Int iIdx, Int iNum );
  Void xGetBlkBits                ( PartSize  eCUMode, Bool bPSlice, Int iPartIdx,  UInt uiLastMode, UInt uiBlkBit[3]);

  Void xMergeEstimation_CNN           ( TComDataCU*  pcCU,
                                    TComYuv*     pcYuvOrg,
                                    Int          iPartIdx,
                                    UInt&        uiInterDir,
                                    TComMvField* pacMvField,
                                    UInt&        uiMergeIndex,
                                    Distortion&  ruiCost,
                                    TComMvField* cMvFieldNeighbours,
                                    UChar*       uhInterDirNeighbours,
                                    Int&         numValidMergeCand
                                   );
  Void xMergeEstimation_DCTIF(TComDataCU*  pcCU,
	  TComYuv*     pcYuvOrg,
	  Int          iPartIdx,
	  UInt&        uiInterDir,
	  TComMvField* pacMvField,
	  UInt&        uiMergeIndex,
	  Distortion&  ruiCost,
	  TComMvField* cMvFieldNeighbours,
	  UChar*       uhInterDirNeighbours,
	  Int&         numValidMergeCand
  );

  Void xRestrictBipredMergeCand   ( TComDataCU*     pcCU,
                                    UInt            puIdx,
                                    TComMvField*    mvFieldNeighbours,
                                    UChar*          interDirNeighbours,
                                    Int             numValidMergeCand );


  // -------------------------------------------------------------------------------------------------------------------
  // motion estimation
  // -------------------------------------------------------------------------------------------------------------------


  Void xMotionEstimation_CNN(TComDataCU*  pcCU,
	  TComYuv*     pcYuvOrg,
	  Int          iPartIdx,
	  RefPicList   eRefPicList,
	  TComMv*      pcMvPred,
	  Int          iRefIdxPred,
	  TComMv&      rcMv,
	  UInt&        ruiBits,
	  Distortion&  ruiCost,
	  Bool         bBi = false);

  Void xMotionEstimation_DCTIF( TComDataCU*  pcCU,
                                    TComYuv*     pcYuvOrg,
                                    Int          iPartIdx,
                                    RefPicList   eRefPicList,
                                    TComMv*      pcMvPred,
                                    Int          iRefIdxPred,
                                    TComMv&      rcMv,
                                    UInt&        ruiBits,
                                    Distortion&  ruiCost,
                                    Bool         bBi = false  );
  Void xMotionEstimation(TComDataCU*  pcCU,
	  TComYuv*     pcYuvOrg,
	  Int          iPartIdx,
	  RefPicList   eRefPicList,
	  TComMv*      pcMvPred,
	  Int          iRefIdxPred,
	  TComMv&      rcMv,
	  UInt&        ruiBits,
	  Distortion&  ruiCost,
	  Bool         bBi = false);

  Void xTZSearch                  ( const TComDataCU* const  pcCU,
                                    const TComPattern* const pcPatternKey,
                                    const Pel* const         piRefY,
                                    const Int                iRefStride,
                                    const TComMv* const      pcMvSrchRngLT,
                                    const TComMv* const      pcMvSrchRngRB,
                                    TComMv&                  rcMv,
                                    Distortion&              ruiSAD,
                                    const TComMv* const      pIntegerMv2Nx2NPred,
                                    const Bool               bExtendedSettings
                                    );

  Void xTZSearchSelective         ( const TComDataCU* const  pcCU,
                                    const TComPattern* const pcPatternKey,
                                    const Pel* const         piRefY,
                                    const Int                iRefStride,
                                    const TComMv* const      pcMvSrchRngLT,
                                    const TComMv* const      pcMvSrchRngRB,
                                    TComMv&                  rcMv,
                                    Distortion&              ruiSAD,
                                    const TComMv* const      pIntegerMv2Nx2NPred
                                    );

  Void xSetSearchRange            ( const TComDataCU* const pcCU,
                                    const TComMv&      cMvPred,
                                    const Int          iSrchRng,
                                    TComMv&      rcMvSrchRngLT,
#if !MCTS_ENC_CHECK
                                    TComMv&      rcMvSrchRngRB );
#else
                                    TComMv&      rcMvSrchRngRB,
                                    const TComPattern* const pcPatternKey );
#endif

#if MCTS_ENC_CHECK
  Void xInitTileBorders(const TComDataCU* const pcCU, TComPattern* pcPatternKey);
#endif

  Void xPatternSearchFast         ( const TComDataCU* const  pcCU,
                                    const TComPattern* const pcPatternKey,
                                    const Pel* const         piRefY,
                                    const Int                iRefStride,
                                    const TComMv* const      pcMvSrchRngLT,
                                    const TComMv* const      pcMvSrchRngRB,
                                    TComMv&                  rcMv,
                                    Distortion&              ruiSAD,
                                    const TComMv* const      pIntegerMv2Nx2NPred
                                  );

  Void xPatternSearch             ( const TComPattern* const pcPatternKey,
                                    const Pel*               piRefY,
                                    const Int                iRefStride,
                                    const TComMv* const      pcMvSrchRngLT,
                                    const TComMv* const      pcMvSrchRngRB,
                                    TComMv&      rcMv,
                                    Distortion&  ruiSAD );

  Void xPatternSearchFracDIF      (//TComDataCU *pcCU,
                                    Bool         bIsLosslessCoded,
                                    TComPattern* pcPatternKey,
                                    Pel*         piRefY,
                                    Int          iRefStride,
                                    TComMv*      pcMvInt,
                                    TComMv&      rcMvHalf,
                                    TComMv&      rcMvQter,
                                    Distortion&  ruiCost
                                   );
	Void xExtDIFUpSamplingHCNN( TComPattern* pcPattern, TComYuv m_filteredBlock_local[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS] ); 
	Void xExtDIFUpSamplingQCNN( TComPattern* pcPatternKey, TComMv halfPelRef, TComYuv m_filteredBlock_local[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]);
  
	Void xExtDIFUpSamplingH( TComPattern* pcPattern );
	Void xExtDIFUpSamplingQ( TComPattern* pcPatternKey, TComMv halfPelRef );

	Void xExtDIFUpSamplingH_preprocess( TComPattern* pattern,TComYuv m_filteredBlock_local[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS] );
    int RoundDown(double x);
	int RoundUp(double x);
	Void xExtDIFUpSamplingQ_preprocess( TComPattern* pcPatternKey, TComYuv m_filteredBlock_local[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS] ,TComMv halfPelRef );
	void duplicate3times(Pel * src, int srcStride, Pel * dst, int dstStride, int height, int width);

	//get block dst size width x height from frame input. Start position is (posy, posx)
	void m_getBlkFromPos(Pel* input, int inputStride, int width, int height, Pel *dst, int dstStride, int posy, int posx);
	
	//predict 16 fractional position (new code for interpolating whole frame)
	void PredInterBlk_Y(TComDataCU* pcCU, TComMv* pcMvInt, UInt uiPartAddr, TComPattern* pattern, RefPicList eRefPicList, Int iRefIdxPred);

	//whole frame interpolation
	//interger pixel will be set at the reconstructed frame, other position will be interpolated by DCTIF and then, CNN.
	//void WholeFrameInterp(Pel * ReconFrame, int reconStride, int orWholeFrameHeight, int orWholeFrameWidth, Pel * or_ref_pel, int or_ref_stride, int bitDepth);
	
	//new pattern refinement code for 49 position
	Distortion xPatternRefinement_orignal(TComPattern* pcPatternKey, TComMv& rcMvInt, TComMv& rcMvFracH, TComMv& rcMvFracQ, Bool bAllowUseOfHadamard);

	//getdaa from position posx and posy from interpolated frame
	void getdata(Pel* input, int inputStride, int FrameHeight, int FrameWidth, Pel *dst, int dstStride, int marginY, int marginX, int posy, int posx);
    
    Pel * readoutputfile(const char* outputpath, int height, int width);
	
    //search for the best block from 49 position around integer pixel
	Distortion xPatternRefinement_modify_49(TComPattern* pcPatternKey, TComMv& rcMvInt, TComMv& rcMvFracH, TComMv& rcMvFracQ, Bool bAllowUseOfHadamard, TComYuv m_filteredBlock_local[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS]);

	Void xPatternSearchFracDIF_CNN(TComDataCU* pcCU,
	  Bool         bIsLosslessCoded,
	  TComPattern* pcPatternKey,
	  Pel*         piRefY,
	  Int          iRefStride,
	  TComMv*      pcMvInt,
	  TComMv&      rcMvHalf,
	  TComMv&      rcMvQter,
	  Distortion&  ruiCost,
	  UInt uiPartAddr,
	  RefPicList eRefPicList,
	  Int iRefIdxPred
  );
  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------


  Void xEncodeInterResidualQT( const ComponentID compID, TComTU &rTu );
  Void xEstimateInterResidualQT( TComYuv* pcResi, Double &rdCost, UInt &ruiBits, Distortion &ruiDist, Distortion *puiZeroDist, TComTU &rTu DEBUG_STRING_FN_DECLARE(sDebug) );
  Void xSetInterResidualQTData( TComYuv* pcResi, Bool bSpatial, TComTU &rTu  );

  UInt  xModeBitsIntra ( TComDataCU* pcCU, UInt uiMode, UInt uiPartOffset, UInt uiDepth, const ChannelType compID );
  UInt  xUpdateCandList( UInt uiMode, Double uiCost, UInt uiFastCandNum, UInt * CandModeList, Double * CandCostList );

  // -------------------------------------------------------------------------------------------------------------------
  // compute symbol bits
  // -------------------------------------------------------------------------------------------------------------------

  Void xAddSymbolBitsInter       ( TComDataCU*   pcCU,
                                   UInt&         ruiBits);

  Void  setWpScalingDistParam( TComDataCU* pcCU, Int iRefIdx, RefPicList eRefPicListCur );
  inline  Void  setDistParamComp( ComponentID compIdx )  { m_cDistParam.compIdx = compIdx; }

};// END CLASS DEFINITION TEncSearch

//! \}

#endif // __TENCSEARCH__
