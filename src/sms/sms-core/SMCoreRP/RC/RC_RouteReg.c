/*
 * GPS Navigation ---An open source GPS navigation core software
 *
 *
 * Copyright (c) 2016  Hitachi, Ltd.
 *
 * This program is dual licensed under GPL version 2 or a commercial license.
 * See the LICENSE file distributed with this source file.
 */

/*
 * RP_RouteReg.c
 *
 *  Created on: 2015/11/06
 *      Author: masutani
 */

#include "sms-core/SMCoreRP/SMCoreRPInternal.h"

#define RC_REG_INFO_SIZE				5		/* 規制情報数サイズ */
#define RC_REG_LINK_SIZE				11		/* 規制リンクリストサイズ */

// 規制パーセルシフト計算用
const static INT8 sParcelShift[10][2] = { { 0, 0 }, { -1, 1 }, { 0, 1 }, { 1, 1 }, { -1, 0 }, { 0, 0 }, { 1, 0 }, { -1, -1 }, { 0, -1 }, {
		1, -1 } };
// 規制パーセルシフト計算マクロ
#define GET_REGPARCELID(linkId, parcelId)			5 == ((linkId >> 27) & 0x0F) ? parcelId : SC_MESH_SftParcelId(parcelId, sParcelShift[((linkId >> 27) & 0x0F)][0], sParcelShift[((linkId >> 27) & 0x0F)][1])

/*-------------------------------------------------------------------
 * 型定義
 *-------------------------------------------------------------------*/

typedef enum {
	e_CONNECT_INVALID = 0,
	e_CONNECT_LV1SAME,			// 同一レベル接続：規制レベル１／候補経路レベル１
	e_CONNECT_LV2SAME,			// 同一レベル接続：規制レベル２／候補経路レベル２
	e_CONNECT_UPLV,				// 上位レベル接続：規制レベル１／候補経路レベル２
	e_CONNECT_DOWNLV			// 下位レベル接続：規制レベル２／候補経路レベル１
} E_REGCONNECT_TYPE;

// 2015/07/31 ライカムパッチ
typedef struct _ROUTE_REG_LINK {
	UINT32 parcelId;								// パーセルID
	UINT32 linkId;									// 規制リンクID
} ROUTE_REG_LINK;

typedef struct _ROUTE_REG_INFO {
	UINT32 listIdx;									// リンクリストIndex
	UINT32 linkVol;									// リンク数
} ROUTE_REG_INFO;

typedef struct _ROUTE_PATCH_LIST {
	ROUTE_REG_INFO regInfo[RC_REG_INFO_SIZE];		// 規制情報
	UINT32 regInfoVol;								// 規制情報数
	ROUTE_REG_LINK linkList[RC_REG_LINK_SIZE];		// 規制リンクリスト
	UINT32 linkListVol;								// 規制リンクリスト数
} ROUTE_PATCH_LIST;

typedef UINT32 (*F_RegJudge)(SCRP_SECTCONTROLER*, SCRP_NETCONTROLER*, SCRC_IOCALCTBL*);
typedef Bool (*F_PatchRegJudge)(SCRP_NETCONTROLER* aNetTab, SCRC_TARGETLINKINFO *aInLink, SCRC_TARGETLINKINFO *aOutLink,
		ROUTE_REG_INFO *aRegInfo);

static ROUTE_PATCH_LIST mRaikamPatch = { };
static F_PatchRegJudge mPatchReg = NULL;
static F_RegJudge m_TimeRegJudge;
static SMRPSEARCHTIME sRouteCalcDayTime;

/*-------------------------------------------------------------------
 * 宣言
 *-------------------------------------------------------------------*/
static UINT32 RC_JudgeTimeReg_Use(SCRP_SECTCONTROLER* aSectCtrl, SCRP_NETCONTROLER* aNetTab, SCRC_IOCALCTBL* aInoutData);
static UINT32 RC_JudgeTimeReg_Unuse(SCRP_SECTCONTROLER* aSectCtrl, SCRP_NETCONTROLER* aNetTab, SCRC_IOCALCTBL* aInoutData);
static UINT32 isInRegulationLinks(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetTab, SCRC_IOCALCTBL* aInoutData);
static Bool judgeRegNetwork(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, SCRC_IOCALCTBL* aInoutData,
		RCREG_REGDATAINFO* aRegInfo);
static Bool judgeRegCandNetwork(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, RCREG_REGDATAINFO* aRegInfo,
		UINT32 aRegJudgeIdx, UINT32 aRegLinkId, UINT32 aRegParcelId);
#if _RP_REG_INDEFINITE
static Bool judgeIndefiniteReg(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, SCRC_IOCALCTBL* aInoutData,
		RCREG_REGDATAINFO* aRegInfo);
static Bool judgeIndefiniteRegCandNetwork(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, RCREG_REGDATAINFO* aRegInfo,
		UINT32 aRegJudgeIdx, UINT32 aRegPclId, UINT32 aRegLinkId, UINT32 aTargetPclId, UINT32 aTargetLinkId);
#endif
static E_REGCONNECT_TYPE getConnectType(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, UINT32 aRegParcelId);
static Bool judgeSameLinkOfDifferLevel(UINT32 aUpParcelId, UINT32 aUpLinkId, UINT32 aParcelId, UINT32 aLinkId);
static E_SC_RESULT regRouteSetRegFlag(SC_RP_RouteMng* aRouteMng, UINT32 aPclIndex, MAL_HDL aRoadBin, RCREG_ROUTEREGLIST *aRegOfsList);
static Bool regRouteJudgeRegLinks(SC_RP_RouteMng* aRouteMng, UINT32 aRoutePclIdx, UINT32 aRouteLinkIdx, RCREG_REGDATAINFO* aRegInfo);
// 2015/07/31 ライカムパッチ
static void setPatchRegRaikam();
static Bool isPatchRegRaikam(SCRP_NETCONTROLER* aNetTab, SCRC_TARGETLINKINFO *aInLink, SCRC_TARGETLINKINFO *aOutLink,
		ROUTE_REG_INFO *aRegInfo);

/**
 * @brief 規制判定処理前準備
 * @param [I]探索設定情報
 * @memo 規制判定関数設定
 *       規制条件設定
 */
E_SC_RESULT RC_RegJudgeFuncSet(SCRP_SEARCHSETTING* aSetting) {
	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_START);

	if (NULL == aSetting) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (e_SC_RESULT_BADPARAM);
	}

	// 2015/07/31 ライカムパッチ
	setPatchRegRaikam();

	// 規制関数設定
	if (RP_SETTING_TIMEREG_ON == aSetting->b_setting.timeReg) {
		m_TimeRegJudge = RC_JudgeTimeReg_Use;
	} else {
		m_TimeRegJudge = RC_JudgeTimeReg_Unuse;
	}

	// 規制判定用探索時刻
	RP_Memcpy(&sRouteCalcDayTime, &aSetting->routeSearchTime, sizeof(SMRPSEARCHTIME));

#if 0 // RC_RegJudgeRegulationResult試験用 残しておく
	UINT32 testCode[3] = {0x80003FFF, 0xE0000000, 0x80000000};
	SMRPSEARCHTIME aTbl[5] = {
		{	2016,6, 1,1,11,30},
		{	2016,6,10,1,11,15},
		{	2016,6,20,1,12,10},
		{	2016,7,20,1, 7,40},
		{	2016,1,20,1,20,30}};
	RCREG_REGDATAINFO bTbl = {0};
	bTbl.pCode = (MAL_HDL) &testCode[0];
	bTbl.codeVol = 1;

	while (sTestCodeON < 5 * 6) {
		RC_RegJudgeRegulationResult(&aTbl[sTestCodeON / 6], &bTbl);
		sTestCodeON++;
	}
#endif

#if 0
	SC_LOG_DebugPrint(SC_TAG_RC, "Regulation setting dump."HERE);
	SC_LOG_DebugPrint(SC_TAG_RC, " day & time        = %02d/%02d(%d) %02d:%02d.%02d", aSetting->routeSearchTime.mon,
			aSetting->routeSearchTime.mday, aSetting->routeSearchTime.wday, aSetting->routeSearchTime.hour, aSetting->routeSearchTime.min,
			aSetting->routeSearchTime.sec);
	SC_LOG_DebugPrint(SC_TAG_RC, " time regulation   = %d", aSetting->b_setting.timeReg);
	SC_LOG_DebugPrint(SC_TAG_RC, " season regulation = %d", aSetting->b_setting.seasonReg);
	SC_LOG_DebugPrint(SC_TAG_RC, " car type          = %d", 2);
	SC_LOG_DebugPrint(SC_TAG_RC, " ride on           = %d", 2);
#endif

	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_END);
	return (e_SC_RESULT_SUCCESS);
}

/**
 * @brief 時間規制判定
 * @param 区間管理情報
 * @param ネットワーク管理情報
 * @param 侵入退出リンク
 */
UINT32 RC_JudgeTimeReg(SCRP_SECTCONTROLER* aSectCtrl, SCRP_NETCONTROLER* aNetTab, SCRC_IOCALCTBL* aInoutData) {
	return (m_TimeRegJudge(aSectCtrl, aNetTab, aInoutData));
}

/**
 * @brief 時間規制：考慮する
 * @param 区間管理情報
 * @param ネットワーク管理情報
 * @param 侵入退出リンク
 */
static UINT32 RC_JudgeTimeReg_Use(SCRP_SECTCONTROLER* aSectCtrl, SCRP_NETCONTROLER* aNetTab, SCRC_IOCALCTBL* aInoutData) {
	return (isInRegulationLinks(aSectCtrl, aNetTab, aInoutData));
}

/**
 * @brief 時間規制：考慮しない
 */
static UINT32 RC_JudgeTimeReg_Unuse(SCRP_SECTCONTROLER* aSectCtrl, SCRP_NETCONTROLER* aNetTab, SCRC_IOCALCTBL* aInoutData) {
	return (RCREG_RESULT_NOREG);
}

/**
 * @brief 侵入退出リンクの組み合わせが規制に該当するかを判断する
 * @param 区間管理情報
 * @param ネットワーク管理情報
 * @param 侵入リンク＆退出リンク
 */
static UINT32 isInRegulationLinks(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetTab, SCRC_IOCALCTBL* aInoutData) {

	E_SC_RESULT result = e_SC_RESULT_SUCCESS;
	UINT32 regResult = RCREG_RESULT_NOREG;	// 規制判定関数戻り値
	MAL_HDL pRecodeTop = NULL;				// レコード先頭
	MAL_HDL pRegGroupTop = NULL;			// 規制群先頭
	UINT32 regOffset = 0;					// 規制情報オフセット(4byte)
	UINT32 i;
	RCREG_REGRECODINFO recodInfo = { };

	// パラメータチェック
	if (NULL == aSectCtrl || NULL == aNetTab || NULL == aInoutData) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (RCREG_RESULT_NOREG);
	}
	// 規制なし判定
	if (ALL_F32 == aInoutData->outLink->linkTable->regOfs || NULL == aInoutData->outLink->pclInfo->mapNetworkRegBin) {
		// 規制なしパーセル
		return (RCREG_RESULT_NOREG);
	}

	pRecodeTop = aInoutData->outLink->pclInfo->mapNetworkRegBin;
	regOffset = aInoutData->outLink->linkTable->regOfs;

	// 同一オフセットで終了 "while (regOffset != aInoutData->outLink->linkTable->regOfs)"
	while (true) {
		MAL_HDL pReg = NULL;			// 規制先頭
		UINT16 hitIndex = ALL_F16;		// 同一リンクリストインデックス

		// オフセットから規制レコード取得
		recodInfo.pRegTop = SC_MA_A_NWRCD_REG_GET_RECOD(pRecodeTop, regOffset);

		recodInfo.pRegDataTop = SC_MA_A_NWRCD_REG_GET_REG(recodInfo.pRegTop);
		recodInfo.regId = SC_MA_D_NWRCD_REG_GET_REGID(recodInfo.pRegTop);
		recodInfo.dataSize = SC_MA_D_NWRCD_REG_GET_DATASIZE(recodInfo.pRegTop);
		recodInfo.regSize = SC_MA_D_NWRCD_REG_GET_REGSIZE(recodInfo.pRegTop);
		recodInfo.sameLinkOfsVol = SC_MA_D_NWRCD_REG_GET_SAMEOFSVOL(recodInfo.pRegTop);

		pReg = recodInfo.pRegDataTop;

		// 規制群（規制サイズ未満は継続）
		while (pReg - recodInfo.pRegDataTop < recodInfo.regSize) {
			Bool isInRegulation = false;
			RCREG_REGDATAINFO regInfo = { };

			// 規制情報収集
			if (e_SC_RESULT_SUCCESS != RC_RegBuildRegInfo(pReg, &regInfo)) {
				SC_LOG_ErrorPrint(SC_TAG_RC, "RC_RegBuildRegInfo is error. "HERE);
				break;
			}

			switch (regInfo.code[0]) {
			case RCREG_REGID1:			// n+1~1 No left turn.     通常規制
			case RCREG_REGID2:			// n+1~1 No right turn.    通常規制
			case RCREG_REGID3:			// n+1~1 No straight on.   通常規制
			case RCREG_REGID4:			// n+1~1 No u turn.        通常規制
			case RCREG_ID_NOENTRY:		// n+m~1 No entry.         侵入不可
			case RCREG_REGID10:			// n+1~1 open.             計画道路
			case RCREG_REGID11:			// n+1~1 close             計画道路
			case RCREG_REGID15:			// n+1~1 generated regulation.  only規制から生成した規制
				// 退出リンク：1本 キーリンク：1本
				if (aInoutData->outLink->linkTable->detaIndex + 1 == read2byte(regInfo.pIndex) ) {
					isInRegulation = judgeRegNetwork(aSectCtrl, aNetTab, aInoutData, &regInfo);
					hitIndex = 0;
				}
				break;
			case RCREG_ID_NOEXIT:		// n+m~m No exit.          退出不可
				// 退出リンク：m本 キーリンク：m本
				for (i = 0; i < regInfo.linkVol1; i++) {
					if (aInoutData->outLink->linkTable->detaIndex + 1 == read2byte(regInfo.pIndex + i * 2) ) {
						isInRegulation = judgeRegNetwork(aSectCtrl, aNetTab, aInoutData, &regInfo);
						hitIndex = i;
						break;
					}
				}
				break;
			case RCREG_REGID7:			// n+1~1 Only right turn.  only規制
			case RCREG_REGID8:			// n+1~1 Only left turn.   only規制
			case RCREG_REGID9:			// n+1~1 Only straight on. only規制
				// 対象としないが同一規制オフセットに該当する
				if (aInoutData->outLink->linkTable->detaIndex + 1 == read2byte(regInfo.pIndex) ) {
					hitIndex = 0;
				}
				break;
			case RCREG_REGID12:			// ..R~R passage reguration. 単独規制
				// 全リンクに対して当て込み
				for (i = 0; i < regInfo.linkVol1; i++) {
					if (aInoutData->outLink->linkTable->detaIndex + 1 == read2byte(regInfo.pIndex + i * 2) ) {
						isInRegulation = true;
						hitIndex = i;
						break;
					}
				}
				break;
#if _RP_REG_INDEFINITE
			case RCREG_ID_INDEFINITE:	// 1+1 リンク不定規制
				// キーリンク：1本 退出リンク1本
				if (aInoutData->outLink->linkTable->detaIndex + 1 == read2byte(regInfo.pIndex) ) {
					UINT32 orFlg = 0;
					if (RCND_LINKOR == RCND_GET_ORIDX(aInoutData->outLink->linkNet->flag)) {
						orFlg = 0x04000000;
					} else {
						orFlg = 0x02000000;
					}
					if (orFlg & read4byte(regInfo.pLink) ) {
						isInRegulation = judgeIndefiniteReg(aSectCtrl, aNetTab, aInoutData, &regInfo);
						hitIndex = 0;
					}
				}
				break;
#endif
			default:
				SC_LOG_ErrorPrint(SC_TAG_RC, " unknown code regulation. regId=0x%08x code=%d "HERE, recodInfo.regId, regInfo.code[0]);
				hitIndex = ALL_F16;
				break;
			}

			// 削除フラグ確認（念のため）
			if (recodInfo.regId & 0x80000000) {
				SC_LOG_WarnPrint(SC_TAG_RC, "delete regulation... regId=0x%08x "HERE, recodInfo.regId);
				break;
			}
			// リンクの組み合わせ一致
			if (isInRegulation) {
				// 規制コードに対して判定：期間内規制ありor規制あり の時点で応答を返す。
				regResult |= RC_RegJudgeRegulationResult(&sRouteCalcDayTime, &regInfo);
				if ((RCREG_RESULT_REG | RCREG_RESULT_TIMEREGIN)& regResult) {
					break;
				}
			}

			// next
			pReg = pReg + 4 + regInfo.linkSize + regInfo.indexSize + regInfo.codeSize;
		}

		// 規制に該当する場合終了（他は見る必要なし）
		if ((RCREG_RESULT_REG | RCREG_RESULT_TIMEREGIN)& regResult) {
			break;
		}
		// 規制がヒットしなかった。不正データとして処理しない。
		if (ALL_F16 == hitIndex) {
			SC_LOG_ErrorPrint(SC_TAG_RC, "regulation link not find ... (index error) "HERE);
			regResult = RCREG_RESULT_NOREG;
			break;
		}
		// next：同一規制リンクへジャンプ（no_exit/passage_reguration はインデックス０以外があり得る。）
		regOffset = read4byte(recodInfo.pRegTop + 12 + recodInfo.regSize + hitIndex * 4);

		// 同じ規制レコードまで戻ったら終了
		if (regOffset == aInoutData->outLink->linkTable->regOfs) {
			break;
		}
#if 0
		SC_LOG_DebugPrint(SC_TAG_RC, "next regulation find. regOffset=%d startOffset=%d "HERE, regOffset,
				aInoutData->outLink->linkTable->regOfs);
#endif
	}

#if 0
	if (RCREG_RESULT_REG == (RCREG_RESULT_REG & regResult) || RCREG_RESULT_INTIMEREG == (RCREG_RESULT_INTIMEREG & regResult)) {
		SC_LOG_DebugPrint(SC_TAG_RC, " Target link is in regulation. result=0x%08x "HERE, regResult);
	}
#endif

	return (regResult);
}

/**
 * @brief 規制情報がネットワークに該当するか判断する
 * @param 区間管理情報
 * @param ネットワーク管理情報
 * @param 侵入退出リンク情報
 * @param 規制情報
 * @memo no_exitの場合
 *         複数の退出リンクに対しては判定を行わない。侵入リンクのみに対して判定を行う。その為本関数の前に退出リンクの判定を行うこと。
 *       no_entryの場合
 *         退出リンク判定後、複数の侵入リンクへの判定を行う。
 *       only_***の場合
 *         対応していない。本関数を呼び出さないこと。
 *       passage regurationの場合
 *         対応していない。本関数を呼び出さないこと。
 *       no_left_turn/no_right_turn/no_straight_on/no_u_turn/open/closeの場合
 *         侵入リンクのみを判定する。
 *       判定中に別レベルへの接続が存在した場合、以降のリンクに対してはsearchCandNetworkにすべての判定を任せる。
 */
static Bool judgeRegNetwork(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, SCRC_IOCALCTBL* aInoutData,
		RCREG_REGDATAINFO* aRegInfo) {

	if (NULL == aSectCtrl || NULL == aNetCtrl || NULL == aInoutData || NULL == aRegInfo) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (false);
	}

#if 0 // 規制リンク列ダンプ
	RPDBG_DumpRegulationLinks(aRegInfo, aInoutData->outLink->pclInfo->parcelId);
#endif

	UINT32 regPclId = 0;			// 規制パーセルID
	UINT32 regLinkId = 0;			// 規制リンクID
	UINT32 targetPclId = 0;			// 判定パーセルID
	UINT32 targetLinkId = 0;		// 判定リンクID
	Bool judge = true;				// 判定結果
	Bool finish = false;
	UINT32 regIdx;

	MAL_HDL pLinkRecod = NULL;
	SCRP_NETDATA* preNet = NULL;
	SCRP_LINKINFO* preLink = NULL;
	SCRP_PCLINFO* prePcl = NULL;
	UINT8 or = 0;

	UINT32 judgeVol = 0;
	UINT32 startIdx = 0;

	// コード別処理
	switch (aRegInfo->code[0]) {
	case RCREG_ID_NOEXIT:
		startIdx = aRegInfo->linkVol1;
		judgeVol = aRegInfo->linkVol1 + aRegInfo->linkVol2;
		break;
	case RCREG_ID_NOENTRY:
		startIdx = 1;
		judgeVol = aRegInfo->linkVol1;
		break;
	default:
		startIdx = 1;
		judgeVol = aRegInfo->linkVol1 + aRegInfo->linkVol2;
		break;
	}
	// 基準パーセルIDはキーリンクのパーセルID
	regPclId = aInoutData->outLink->pclInfo->parcelId;

	// 退出リンク列に対して判定をかける
	for (regIdx = startIdx; regIdx < judgeVol; regIdx++) {
		// 規制対象リンクID・パーセルID・レベルを取得
		regLinkId = read4byte(aRegInfo->pLink + regIdx * 4);
		regPclId = GET_REGPARCELID(regLinkId, regPclId);

		// 侵入リンクID
		if (NULL == preNet) {
			pLinkRecod =
					SC_MA_A_NWRCD_LINK_GET_RECORD(aInoutData->inLink->pclInfo->mapNetworkLinkBin, aInoutData->inLink->linkTable->detaIndex);
			targetLinkId = SC_MA_D_NWRCD_LINK_GET_ID(pLinkRecod);
			targetPclId = aInoutData->inLink->pclInfo->parcelId;

			// 次からはネットワーク履歴から
			preNet = aInoutData->inLink->linkNet;
		} else {
			// 展開ネットワーク内
			preLink = RCNET_GET_HISTLINKINFO(aNetCtrl, preNet);
			prePcl = RCNET_GET_HISTPCLINFO(aNetCtrl, preNet);
			or = RCNET_GET_HISTORIDX(preNet);

			pLinkRecod = SC_MA_A_NWRCD_LINK_GET_RECORD(prePcl->mapNetworkLinkBin, preLink->detaIndex);
			targetLinkId = SC_MA_D_NWRCD_LINK_GET_ID(pLinkRecod);
			targetPclId = prePcl->parcelId;

			preNet = &preLink->linkNet[or];
		}
		// パーセルIDリンクID一致で継続
		if (regPclId != targetPclId) {
			judge = false;
			break;
		}
		if ((regLinkId & MAL_LINKPNTID) != (targetLinkId & MAL_LINKPNTID)) {
			judge = false;
			break;
		}
		// 探索開始リンクの場合、候補経路内規制判定呼び出し
		if (RCND_GET_STARTLINKFLG(preNet->flag)) {
			judge = judgeRegCandNetwork(aSectCtrl, aNetCtrl, aRegInfo, regIdx, regLinkId, regPclId);
			finish = true;
			break;
		}
		// 無効値チェック
		if (ALL_F32 == preNet->inLinkHist) {
			judge = false;
			SC_LOG_DebugPrint(SC_TAG_RC, "inLinkHist is invalid... "HERE);
			break;
		}
	}
	// いずれかのリンクで既に不一致
	if (!judge) {
		return (false);
	}
	// 別レベルへ接続している
	if (finish) {
		return (judge);
	}
	// no_entry以外は終了
	if (RCREG_ID_NOENTRY == aRegInfo->code[0]) {
		UINT32 preRegPclId = regPclId;		// no_entry用fromリンクの直前のパーセルIDからのオフセットを計算する必要がある為保持

		// 判定初期化
		judge = false;

		// 侵入リンク列に対して判定をかける
		if (NULL == preNet) {
			pLinkRecod =
					SC_MA_A_NWRCD_LINK_GET_RECORD(aInoutData->inLink->pclInfo->mapNetworkLinkBin, aInoutData->inLink->linkTable->detaIndex);
			targetLinkId = SC_MA_D_NWRCD_LINK_GET_ID(pLinkRecod);
			targetPclId = aInoutData->inLink->pclInfo->parcelId;
		} else {
			// 侵入履歴あり＝展開ネットワーク内
			preLink = RCNET_GET_HISTLINKINFO(aNetCtrl, preNet);
			prePcl = RCNET_GET_HISTPCLINFO(aNetCtrl, preNet);

			pLinkRecod = SC_MA_A_NWRCD_LINK_GET_RECORD(prePcl->mapNetworkLinkBin, preLink->detaIndex);
			targetLinkId = SC_MA_D_NWRCD_LINK_GET_ID(pLinkRecod);
			targetPclId = prePcl->parcelId;
		}

		// 残りの規制リンク判定
		for (regIdx = aRegInfo->linkVol1; regIdx < (aRegInfo->linkVol1 + aRegInfo->linkVol2); regIdx++) {
			// 規制対象リンクID・パーセルIDを取得
			regLinkId = read4byte(aRegInfo->pLink + regIdx * 4);
			regPclId = GET_REGPARCELID(regLinkId, preRegPclId);

			// パーセルIDリンクID一致で終了
			if (regPclId == targetPclId) {
				if ((regLinkId & MAL_LINKPNTID) == (targetLinkId & MAL_LINKPNTID)) {
					judge = true;
					break;
				}
			}
		}
	}

	return (judge);
}

/**
 * @brief 候補経路から規制リンクに一致するかを確認する
 * @param 区間管理情報
 * @param ネットワーク管理情報
 * @param 規制情報
 * @param 侵入退出リンク情報
 * @param 規制チェック開始インデックス
 * @param 規制リンクID
 * @param 規制パーセルID
 */
static Bool judgeRegCandNetwork(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, RCREG_REGDATAINFO* aRegInfo,
		UINT32 aRegJudgeIdx, UINT32 aRegLinkId, UINT32 aRegParcelId) {
	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_START);

	if (NULL == aSectCtrl || NULL == aNetCtrl || NULL == aRegInfo) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (false);
	}

	SCRP_CANDDATA* wkCand = NULL;			// 候補経路ワーク
	SCRP_CANDDATA* targetCandTop = NULL;	// 候補経路先頭（Split/main）
	SCRP_CANDSTARTLINK* stLinkTop = NULL;
	UINT32 stLinkVol = 0;					// 探索開始リンク数
	UINT32 regParcelId = 0;					// 判定対象規制パーセルID
	UINT32 regLinkId = 0;					// 判定対象規制リンクID
	UINT32 judgeIdx = 0;					// 判定中規制インデックス
	UINT32 judgeVol = 0;				// 初回判定回数（no_entryの場合LinkVol1の数だけ）
	UINT32 i;
	INT32 connectType = 0;					// 接続対象
	Bool judge = false;						// 規制判定結果

	// 候補経路の接続先を判定する
	connectType = getConnectType(aSectCtrl, aNetCtrl, aRegParcelId);

#if 1
	SC_LOG_DebugPrint(SC_TAG_RC, "judge reg connect. code=%d level=%d connectType=%d topLevel=%d divIdx=%d "HERE, aRegInfo->code[0],
			SC_MESH_GetLevel(aRegParcelId), connectType, aNetCtrl->levelTbl->topLevel, aNetCtrl->calculatingDivIdx);
#endif

	// 接続対象別に候補開始リンクを検索する
	switch (connectType) {
	case e_CONNECT_LV1SAME:
	case e_CONNECT_LV2SAME:
		// 候補開始リンク情報取得
		stLinkTop = aSectCtrl->candMng.splitStLink;
		stLinkVol = aSectCtrl->candMng.splitStLinkCurrent;
		// 探索開始リンクを候補経路から検索：同一レベル用（Split）
		for (i = 0; i < stLinkVol; i++) {
			if ((aRegLinkId & MAL_LINKPNTID) != ((stLinkTop + i)->linkId & MAL_LINKPNTID)) {
				continue;
			}
			if (aRegParcelId != (stLinkTop + i)->parcelId) {
				continue;
			}
			// 開始データ一致
			targetCandTop = aSectCtrl->candMng.splitCand;
			wkCand = targetCandTop + (stLinkTop + i)->candIdx;
			break;
		}
		break;
	case e_CONNECT_DOWNLV:
		// 候補開始リンク情報取得
		stLinkTop = aSectCtrl->candMng.stLink + aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV1O].stLinkIdx;
		stLinkVol = aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV1O].stLinkSize;
		// 探索開始リンクを候補経路から検索：下位レベル
		for (i = 0; i < stLinkVol; i++) {
			if (RP_LEVEL2 != (stLinkTop + i)->connectLevel) {
				continue;
			}
			if ((aRegLinkId & MAL_LINKPNTID) != ((stLinkTop + i)->st.linkId & MAL_LINKPNTID)) {
				continue;
			}
			if (aRegParcelId != (stLinkTop + i)->st.parcelId) {
				continue;
			}
			// 開始データ一致
			targetCandTop = aSectCtrl->candMng.cand;
			wkCand = targetCandTop + (stLinkTop + i)->candIdx;
			break;
		}
		break;
	case e_CONNECT_UPLV:
		// 候補開始リンク情報取得
		stLinkTop = aSectCtrl->candMng.stLink + aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV2TOP].stLinkIdx;
		stLinkVol = aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV2TOP].stLinkSize;
		// 探索開始リンクを候補経路から検索：上位レベル
		for (i = 0; i < stLinkVol; i++) {
			if (RP_LEVEL1 != (stLinkTop + i)->connectLevel) {
				continue;
			}
			if ((aRegLinkId & MAL_LINKPNTID) != ((stLinkTop + i)->st.linkId & MAL_LINKPNTID)) {
				continue;
			}
			if (aRegParcelId != (stLinkTop + i)->st.parcelId) {
				continue;
			}
			// 開始データ一致
			targetCandTop = aSectCtrl->candMng.cand;
			wkCand = targetCandTop + (stLinkTop + i)->candIdx;
			break;
		}
		break;
	default:
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (false);
	}

	if (NULL == wkCand) {
		// ありえない
		SC_LOG_ErrorPrint(SC_TAG_RC, "Cand start link can't found. "HERE);
		return (false);
	}

	/*-----------------------------------------------------------------------
	 * 規制コードがno_entryの場合、侵入リンクの形式が大きく違う為処理に注意
	 *-----------------------------------------------------------------------*/
	if (RCREG_ID_NOENTRY == aRegInfo->code[0]) {
		judgeVol = aRegInfo->linkVol1;
	} else {
		judgeVol = aRegInfo->linkVol1 + aRegInfo->linkVol2;
	}

	// 初期化
	regParcelId = aRegParcelId;
	regLinkId = aRegLinkId;

	// 接続対象別に候補経路を判定する
	switch (connectType) {
	case e_CONNECT_LV1SAME:			// 規制レベル１／候補経路レベル１
	case e_CONNECT_LV2SAME:			// 規制レベル２／候補経路レベル２
		// この時点でのwkCandはstCandのリンクとなっている
		for (judgeIdx = aRegJudgeIdx; judgeIdx < judgeVol; judgeIdx++) {
			// 初回以降：パーセルシフトの関係で初回はそのまま
			if (judgeIdx != aRegJudgeIdx) {
				regLinkId = read4byte(aRegInfo->pLink + judgeIdx * 4);
				regParcelId = GET_REGPARCELID(regLinkId, regParcelId);
			}
			// パーセルIDリンクID判定
			if (wkCand->parcelId != regParcelId || (wkCand->linkId & MAL_LINKPNTID) != (regLinkId & MAL_LINKPNTID)) {
				judge = false;
				break;
			}
			if (RC_CAND_STLINK == wkCand->next) {
				// todo 同一レベル接続は全てSplit内で閉じている。これ以降はLv２の分割エリアの場合Lv１へ接続。Lv１分割エリアは区間開始リンクのためデータ終点である。
				// 再帰呼び出しを行うかは要検討
				judge = false;
				break;
			}
			if (RC_CAND_INIT == wkCand->next) {
				judge = false;
				SC_LOG_ErrorPrint(SC_TAG_RC, "Cand next index of value is error. "HERE);
				break;
			}

			// 次 候補経路取得
			wkCand = targetCandTop + wkCand->next;
		}
		// 規制判定結果
		if (judgeVol == judgeIdx) {
			judge = true;
		}
		break;
	case e_CONNECT_DOWNLV:		// 規制レベル２／候補経路レベル１
		// 判定インデックス
		judgeIdx = aRegJudgeIdx;
		// break条件は２回連続して一致判定に失敗した場合or必要判定数を超えた場合
		while (true) {
			if (RC_CAND_STLINK == wkCand->next) {
				judge = false;
				break;
			}
			if (RC_CAND_INIT == wkCand->next) {
				judge = false;
				SC_LOG_ErrorPrint(SC_TAG_RC, "Cand next index value is error. "HERE);
				break;
			}
			// 次 候補経路取得
			wkCand = targetCandTop + wkCand->next;

			// 候補経路リンクが上位レベルのリンクIDと一致するかをチェック
			judge = judgeSameLinkOfDifferLevel(regParcelId, regLinkId, wkCand->parcelId, wkCand->linkId);
			if (!judge) {
				// 「一致しない＆判定数最大」は最終リンクまで一致済みの可能性あり
				if (judgeVol <= judgeIdx + 1) {
					judge = true;
					break;
				}

				// 次 規制情報取得
				judgeIdx++;
				regLinkId = read4byte(aRegInfo->pLink + judgeIdx * 4);
				regParcelId = GET_REGPARCELID(regLinkId, regParcelId);
				// ２回目の判定で一致しなければNG
				judge = judgeSameLinkOfDifferLevel(regParcelId, regLinkId, wkCand->parcelId, wkCand->linkId);
				if (!judge) {
					break;
				}
			}
			// レベル２最終リンクがレベル１リンクと１本でも一致すれば規制対象→最終リンク1本までなのであえてjudgeVolとは比較しない
			if (judgeIdx + 1 == (aRegInfo->linkVol1 + aRegInfo->linkVol2)) {
				judge = true;
				break;
			}
		}
		break;
	case e_CONNECT_UPLV:		// 規制レベル１／候補経路レベル２
		// break条件は２回連続して一致判定に失敗した場合or必要判定数を超えた場合
		for (judgeIdx = aRegJudgeIdx; judgeIdx < judgeVol; judgeIdx++) {
			// 初回以降：パーセルシフトの関係で初回はそのまま
			if (judgeIdx != aRegJudgeIdx) {
				regLinkId = read4byte(aRegInfo->pLink + judgeIdx * 4);
				regParcelId = GET_REGPARCELID(regLinkId, regParcelId);
			}

			// 候補経路リンクが上位レベルのリンクIDと一致するかをチェック
			judge = judgeSameLinkOfDifferLevel(wkCand->parcelId, wkCand->linkId, regParcelId, regLinkId);
			if (judge) {
				// next
				continue;
			}
			if (RC_CAND_STLINK == wkCand->next) {
				// todo Lv２候補経路が開始リンクまで到達。これ以降は下位へ接続する。
				judge = false;
				break;
			}
			if (RC_CAND_INIT == wkCand->next) {
				judge = false;
				SC_LOG_ErrorPrint(SC_TAG_RC, "Cand next index value is error. "HERE);
				break;
			}
			// 次 候補経路取得
			wkCand = targetCandTop + wkCand->next;

			// ２回目の判定で一致しなければNG
			judge = judgeSameLinkOfDifferLevel(wkCand->parcelId, wkCand->linkId, regParcelId, regLinkId);
			if (!judge) {
				break;
			}
		}
		// 判定結果
		if (judgeVol == judgeIdx) {
			judge = true;
		}
		break;
	}
	// いずれかのリンクで既に不一致
	if (!judge) {
		return (false);
	}

	// no_entry用
	do {
		// 以降は1リンクでも一致があった場合規制対象としてtrue返却
		if (RCREG_ID_NOENTRY == aRegInfo->code[0]) {

			// 判定初期化
			judge = false;
			UINT32 preParcelId = regParcelId;		// no_entry用 パーセル相対の為に保存

			// 判定対象の候補経路情報取得（downは不一致候補情報まで取得している）
			if (connectType != e_CONNECT_DOWNLV) {
				if (RC_CAND_STLINK == wkCand->next) {
					break;
				}
				if (RC_CAND_INIT == wkCand->next) {
					SC_LOG_ErrorPrint(SC_TAG_RC, "Cand next index value is error. "HERE);
					break;
				}
				wkCand = targetCandTop + wkCand->next;
			}

			// 侵入リンクに対してのみ判定
			for (judgeIdx = aRegInfo->linkVol1; judgeIdx < (aRegInfo->linkVol1 + aRegInfo->linkVol2); judgeIdx++) {

				// 規制情報取得
				regLinkId = read4byte(aRegInfo->pLink + judgeIdx * 4);
				regParcelId = GET_REGPARCELID(regLinkId, preParcelId);

				// 接続タイプ別に判定
				switch (connectType) {
				case e_CONNECT_LV1SAME:
				case e_CONNECT_LV2SAME:
					if (wkCand->parcelId == regParcelId || (wkCand->linkId & MAL_LINKPNTID) == (regLinkId & MAL_LINKPNTID)) {
						judge = true;
					}
					break;
				case e_CONNECT_DOWNLV:
					judge = judgeSameLinkOfDifferLevel(regParcelId, regLinkId, wkCand->parcelId, wkCand->linkId);
					break;
				case e_CONNECT_UPLV:
					judge = judgeSameLinkOfDifferLevel(wkCand->parcelId, wkCand->linkId, regParcelId, regLinkId);
					break;
				}
				// 一致
				if (judge) {
					break;
				}
			}
		}
	} while (0);

	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_END);
	return (judge);
}

#if _RP_REG_INDEFINITE
/**
 * @brief 規制情報がネットワークに該当するか判断する：リンク不定規制専用
 * @param 区間管理情報
 * @param ネットワーク管理情報
 * @param 侵入退出リンク情報
 * @param 規制情報
 * @memo 不定部分を考慮した規制判定を行う。
 */
static Bool judgeIndefiniteReg(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, SCRC_IOCALCTBL* aInoutData,
		RCREG_REGDATAINFO* aRegInfo) {

	if (NULL == aSectCtrl || NULL == aNetCtrl || NULL == aInoutData || NULL == aRegInfo) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (false);
	}

#if 0 // 規制リンク列ダンプ
	RPDBG_DumpRegulationLinks(aRegInfo, aInoutData->outLink->pclInfo->parcelId);
#endif

	// リンク不定規制以外は対応していない。
	if (RCREG_ID_INDEFINITE != aRegInfo->code[0]) {
		return (false);
	}

	UINT32 i;
	UINT32 regPclId = 0;			// 規制パーセルID
	UINT32 regLinkId = 0;			// 規制リンクID
	UINT32 targetPclId = 0;			// 判定パーセルID
	UINT32 targetLinkId = 0;		// 判定リンクID
	Bool judge = false;				// 判定結果

	MAL_HDL pLinkRecod = NULL;
	SCRP_NETDATA* preNet = NULL;
	SCRP_LINKINFO* preLink = NULL;
	SCRP_PCLINFO* prePcl = NULL;
	UINT8 or = 0;

	// 初回パーセル
	regPclId = aInoutData->outLink->pclInfo->parcelId;

	// 最終リンクのパーセルIDリンクID取得
	for (i = 0; i < aRegInfo->linkVol1; i++) {
		regLinkId = read4byte(aRegInfo->pLink + i * 4);
		regPclId = GET_REGPARCELID(regLinkId, regPclId);
	}

	// ネットワーク追跡
	for (i = 0; i < aRegInfo->linkVol2; i++) {
		// 侵入リンクID
		if (NULL == preNet) {
			pLinkRecod =
					SC_MA_A_NWRCD_LINK_GET_RECORD(aInoutData->inLink->pclInfo->mapNetworkLinkBin, aInoutData->inLink->linkTable->detaIndex);
			targetLinkId = SC_MA_D_NWRCD_LINK_GET_ID(pLinkRecod);
			targetPclId = aInoutData->inLink->pclInfo->parcelId;

			// 次からはネットワーク履歴から
			preNet = aInoutData->inLink->linkNet;
		} else {
			// 展開ネットワーク内
			preLink = RCNET_GET_HISTLINKINFO(aNetCtrl, preNet);
			prePcl = RCNET_GET_HISTPCLINFO(aNetCtrl, preNet);
			or = RCNET_GET_HISTORIDX(preNet);

			pLinkRecod = SC_MA_A_NWRCD_LINK_GET_RECORD(prePcl->mapNetworkLinkBin, preLink->detaIndex);
			targetLinkId = SC_MA_D_NWRCD_LINK_GET_ID(pLinkRecod);
			targetPclId = prePcl->parcelId;

			preNet = &preLink->linkNet[or];
		}
		// パーセルIDリンクID方向一致で終了
		if (regPclId == targetPclId) {
			if ((regLinkId & MAL_LINKPNTID) == (targetLinkId & MAL_LINKPNTID)) {
				UINT32 orFlg = 0;
				if (RCND_LINKOR == RCND_GET_ORIDX(preNet->flag)) {
					orFlg = 0x04000000;
				} else {
					orFlg = 0x02000000;
				}
				if (orFlg & regLinkId) {
					judge = true;
					break;
				}
			}
		}
		// 探索開始リンクの場合、候補経路内規制判定呼び出し
		if (RCND_GET_STARTLINKFLG(preNet->flag)) {
			// TODO 候補経路まで追跡する必要性あり
			judge = judgeIndefiniteRegCandNetwork(aSectCtrl, aNetCtrl, aRegInfo, i, regPclId, regLinkId, targetPclId, targetLinkId);
			break;
		}
		// 無効値チェック
		if (ALL_F32 == preNet->inLinkHist) {
			judge = false;
			SC_LOG_DebugPrint(SC_TAG_RC, "inLinkHist is invalid... "HERE);
			break;
		}
	}

	return (judge);
}

/**
 * @brief リンク不定規制ネットワーク候補経路分判定
 * @param 区間管理
 * @param ネットワーク管理
 * @param 規制情報
 * @param 規制判定済み回数
 * @param 規制パーセルID
 * @param 規制リンクID
 * @param 候補パーセルID
 * @param 候補リンクID
 * @memo 判定回数は一律指定回数。レベル２とレベル１での差は未考慮
 * TODO リンク方向の判定が不可。厳密には不可ではない。
 *      １．すべての下位リンクを収集し、座標点からリンクを並べ替えることで方向が判別する方法。
 *      ２．道路ネットワークを新たに読み込み下位リンクの始終端リンクからネットワークを追いかけることで判別する方法。
 */
static Bool judgeIndefiniteRegCandNetwork(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, RCREG_REGDATAINFO* aRegInfo,
		UINT32 aRegJudgeIdx, UINT32 aRegPclId, UINT32 aRegLinkId, UINT32 aTargetPclId, UINT32 aTargetLinkId) {
	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_START);

	SCRP_CANDDATA* wkCand = NULL;			// 候補経路ワーク
	SCRP_CANDDATA* targetCandTop = NULL;	// 候補経路先頭（Split/main）
	SCRP_CANDSTARTLINK* stLinkTop = NULL;
	UINT32 stLinkVol = 0;					// 探索開始リンク数
	UINT32 regParcelId = 0;					// 判定対象規制パーセルID
	UINT32 regLinkId = 0;					// 判定対象規制リンクID
	UINT32 i;
	INT32 connectType = 0;					// 接続対象
	Bool judge = false;						// 規制判定結果

	// 候補経路の接続先を判定する
	connectType = getConnectType(aSectCtrl, aNetCtrl, aRegPclId);
	switch (connectType) {
	case e_CONNECT_LV1SAME:
	case e_CONNECT_LV2SAME:
		// 探索開始リンクを候補経路から検索：同一レベル用（Split）
		stLinkTop = aSectCtrl->candMng.splitStLink;
		stLinkVol = aSectCtrl->candMng.splitStLinkCurrent;
		for (i = 0; i < stLinkVol; i++) {
			if ((aTargetLinkId & MAL_LINKPNTID) != ((stLinkTop + i)->linkId & MAL_LINKPNTID)) {
				continue;
			}
			if (aTargetPclId != (stLinkTop + i)->parcelId) {
				continue;
			}
			// 開始データ一致
			targetCandTop = aSectCtrl->candMng.splitCand;
			wkCand = targetCandTop + (stLinkTop + i)->candIdx;
			break;
		}
		// この時点でのwkCandはstCandのリンクとなっている
		for (i = aRegJudgeIdx; i < aRegInfo->linkVol2; i++) {
			// パーセルIDリンクID判定
			if (wkCand->parcelId == aRegPclId || (wkCand->linkId & MAL_LINKPNTID) == (aRegLinkId & MAL_LINKPNTID)) {
				judge = true;
				break;
			}
			if (RC_CAND_STLINK == wkCand->next) {
				// todo 同一レベル接続は全てSplit内で閉じている。これ以降はLv２の分割エリアの場合Lv１へ接続。Lv１分割エリアは区間開始リンクのためデータ終点である。
				// 再帰呼び出しを行うかは要検討
				break;
			}
			if (RC_CAND_INIT == wkCand->next) {
				SC_LOG_ErrorPrint(SC_TAG_RC, "Cand next index of value is error. "HERE);
				break;
			}
			// 次 候補経路取得
			wkCand = targetCandTop + wkCand->next;
		}
		break;
	case e_CONNECT_DOWNLV:
		// 探索開始リンクを候補経路から検索：下位レベル
		stLinkTop = aSectCtrl->candMng.stLink + aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV1O].stLinkIdx;
		stLinkVol = aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV1O].stLinkSize;
		for (i = 0; i < stLinkVol; i++) {
			if (RP_LEVEL2 != (stLinkTop + i)->connectLevel) {
				continue;
			}
			if ((aTargetLinkId & MAL_LINKPNTID) != ((stLinkTop + i)->st.linkId & MAL_LINKPNTID)) {
				continue;
			}
			if (aTargetPclId != (stLinkTop + i)->st.parcelId) {
				continue;
			}
			// 開始データ一致
			targetCandTop = aSectCtrl->candMng.cand;
			wkCand = targetCandTop + (stLinkTop + i)->candIdx;
			break;
		}
		// 規定回数判定後終了
		for (i = aRegJudgeIdx; i < aRegInfo->linkVol2; i++) {
			// 候補経路リンクが上位レベルのリンクIDと一致するかをチェック
			if (judgeSameLinkOfDifferLevel(aRegPclId, aRegLinkId, wkCand->parcelId, wkCand->linkId)) {
				// 一致終了
				judge = true;
				break;
			}
			if (RC_CAND_STLINK == wkCand->next) {
				break;
			}
			if (RC_CAND_INIT == wkCand->next) {
				SC_LOG_ErrorPrint(SC_TAG_RC, "Cand next index value is error. "HERE);
				break;
			}
			// 次 候補経路取得
			wkCand = targetCandTop + wkCand->next;
		}
		break;
	case e_CONNECT_UPLV:
		stLinkTop = aSectCtrl->candMng.stLink + aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV2TOP].stLinkIdx;
		stLinkVol = aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV2TOP].stLinkSize;
		// 探索開始リンクを候補経路から検索：上位レベル
		for (i = 0; i < stLinkVol; i++) {
			if (RP_LEVEL1 != (stLinkTop + i)->connectLevel) {
				continue;
			}
			if ((aTargetLinkId & MAL_LINKPNTID) != ((stLinkTop + i)->st.linkId & MAL_LINKPNTID)) {
				continue;
			}
			if (aTargetPclId != (stLinkTop + i)->st.parcelId) {
				continue;
			}
			// 開始データ一致
			targetCandTop = aSectCtrl->candMng.cand;
			wkCand = targetCandTop + (stLinkTop + i)->candIdx;
			break;
		}
		// break条件は２回連続して一致判定に失敗した場合or必要判定数を超えた場合
		for (i = aRegJudgeIdx; i < aRegInfo->linkVol2; i++) {
			// 候補経路リンクが上位レベルのリンクIDと一致するかをチェック
			if (judgeSameLinkOfDifferLevel(wkCand->parcelId, wkCand->linkId, aRegPclId, aRegLinkId)) {
				// 一致終了
				judge = true;
				break;
			}
			if (RC_CAND_STLINK == wkCand->next) {
				// todo Lv２候補経路が開始リンクまで到達。これ以降は下位へ接続する。
				break;
			}
			if (RC_CAND_INIT == wkCand->next) {
				SC_LOG_ErrorPrint(SC_TAG_RC, "Cand next index value is error. "HERE);
				break;
			}
			// 次 候補経路取得
			wkCand = targetCandTop + wkCand->next;
		}
		break;
	default:
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (false);
	}

	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_END);
	return (judge);
}
#endif

/**
 * @brief 規制パーセルID(レベル)と探索処理状況から候補経路の接続先を判定する
 * @param 区間情報
 * @param ネットワーク制御
 * @param パーセルID
 */
static E_REGCONNECT_TYPE getConnectType(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER* aNetCtrl, UINT32 aRegParcelId) {
	E_REGCONNECT_TYPE connectType = e_CONNECT_INVALID;

	do {
		if (NULL == aSectCtrl || NULL == aNetCtrl) {
			SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
			break;
		}

		// 候補経路の接続先を判定する
		if (RP_LEVEL1 == SC_MESH_GetLevel(aRegParcelId)) {
			// 同一レベル接続or終了
			if (RP_LEVEL1 == aNetCtrl->levelTbl->topLevel) {
				if (0 == aNetCtrl->calculatingDivIdx) {
					// 終了
					SC_LOG_DebugPrint(SC_TAG_RC, "Regulation trace target is last link. "HERE);
					break;
				} else {
					// レベル１トップ分割エリア：同一レベル履歴要参照
					connectType = e_CONNECT_LV1SAME;
				}
			}
			// 上位レベル接続or終了
			else {
				if (0 == aSectCtrl->candMng.candTblInfo[RC_CAND_IDX_LV2TOP].stLinkSize) {
					// 終了
					SC_LOG_DebugPrint(SC_TAG_RC, "Regulation trace target is last link. "HERE);
					break;
				} else {
					connectType = e_CONNECT_UPLV;
				}
			}
		}
		// 規制レベル２
		else {
			// 下位レベル接続
			if (0 == aNetCtrl->calculatingDivIdx) {
				connectType = e_CONNECT_DOWNLV;
			}
			// 同一レベル接続
			else {
				connectType = e_CONNECT_LV2SAME;
			}
		}
	} while (0);

	return (connectType);
}

/**
 * @brief レベル２リンクに該当する下位リンク情報にレベル１リンクの情報が含まれているか検査する
 *        レベル１形状情報の上位該当リンクIDレコード内に一致するレベル１リンクIDが含まれているかで判断を行う
 * @param レベル２パーセルID
 * @param レベル２リンクID
 * @param レベル１パーセルID
 * @param レベル１リンクID
 */
static Bool judgeSameLinkOfDifferLevel(UINT32 aUpParcelId, UINT32 aUpLinkId, UINT32 aParcelId, UINT32 aLinkId) {

	E_SC_RESULT result = e_SC_RESULT_SUCCESS;
	SCRP_MAPREADTBL readTbl = { };
	SCRP_MAPDATA readList = { };
	UINT32 index = 0;
	UINT32 i;
	Bool find = false;

	// 1個リストをセット
	readTbl.mapList = &readList;

	// パーセルIDチェック
	if (aUpParcelId != SC_MESH_GetUpperParcelID(aParcelId)) {
		SC_LOG_DebugPrint(SC_TAG_RC, "parcel id is diffarent. pcl=0x%08x!=0x%08x "HERE, aUpParcelId, aParcelId);
		return (false);
	}
	do {
		// Lv1地図読み込み
		result = RC_ReadListMap(&aParcelId, 1, SC_DHC_KIND_SHAPE, &readTbl);
		if (e_SC_RESULT_SUCCESS != result) {
			SC_LOG_ErrorPrint(SC_TAG_RC, "RC_ReadListMap error. [0x%08x] "HERE, result);
			break;
		}
		if (NULL == readTbl.mapList->shape) {
			SC_LOG_DebugPrint(SC_TAG_RC, "map road read failed. pcl=0x%08x [0x%08x] "HERE, aParcelId, result);
			break;
		}

		// 上位リンクID索引レコードから検索
		result = SC_MA_BinSearchShapeUpperIndex(readTbl.mapList->shape, aUpLinkId, &index);
		if (e_SC_RESULT_SUCCESS != result) {
			SC_LOG_DebugPrint(SC_TAG_RC, "SC_MA_BinSearchShapeUpperIndex upper link can't found. [0x%08x] "HERE, result);
			break;
		}

		// 上位該当リンク関連の情報を収集
		MAL_HDL upperIdxRec = SC_MA_A_SHBIN_GET_LV2UPPER_IDXLINK(readTbl.mapList->shape);
		MAL_HDL upperRec = SC_MA_A_SHBIN_GET_UPPER_LINK(readTbl.mapList->shape);
		UINT16 upperIdx = SC_MA_GET_SHBIN_IDXUPLINK_UPIDX(upperIdxRec, index);
		UINT16 upperVol = SC_MA_GET_SHBIN_IDXUPLINK_UPVOL(upperIdxRec, index);

		// 上位該当リンクIDレコード内にLv1リンクが含まれているかをチェック
		for (i = 0; i < upperVol; i++) {
			UINT32 id = SC_MA_GET_SHBIN_UPLINK_ID(upperRec, (upperIdx + i - 1));
			if ((aLinkId & MAL_LINKPNTID) == (id & MAL_LINKPNTID)) {
				find = true;
				break;
			}
		}
	} while (0);

	// 地図解放
	if (e_SC_RESULT_SUCCESS != RC_FreeMapTbl(&readTbl, false)) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "RC_FreeMapTbl error. [0x%08x] "HERE, result);
	}

	return (find);
}

/**
 * @brief 推奨経路用：推奨経路内規制時間外該当リンクへフラグを設定する
 * @param 推奨経路管理
 */
E_SC_RESULT RP_RegRouteBuildLinkRegIndex(SC_RP_RouteMng* aRouteMng) {
	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_START);

	if (NULL == aRouteMng) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (e_SC_RESULT_BADPARAM);
	}

	E_SC_RESULT result = e_SC_RESULT_SUCCESS;
	SCRP_MAPDATA readList = { };
	SCRP_MAPREADTBL readTbl = { 1, &readList };
	RCREG_ROUTEREGLIST *regOffsetList = NULL;
	UINT32 parcelId = 0;
	UINT32 i, e;

	for (i = 0; i < aRouteMng->parcelVol; i++) {
		// 初期化
		regOffsetList = NULL;
		RP_Memset0(readTbl.mapList, sizeof(SCRP_MAPDATA));
		parcelId = (aRouteMng->parcelInfo + i)->parcelId;

		do {
			// 地図読み込み
			result = RC_ReadListMap(&parcelId, 1, SC_DHC_KIND_ROAD, &readTbl);
			if (e_SC_RESULT_SUCCESS != result) {
				SC_LOG_ErrorPrint(SC_TAG_RC, "RC_ReadListMap error. [0x%08x] "HERE, result);
				break;
			}
			if (NULL == readTbl.mapList->road) {
				SC_LOG_DebugPrint(SC_TAG_RC, "map road read failed. pcl=0x%08x [0x%08x] "HERE, parcelId, result);
				break;
			}
			// 規制データがあれば判定
			if (ALL_F32 != SC_MA_D_NWBIN_GET_LINKREG_OFS(readTbl.mapList->road) ) {

				// 規制オフセット格納用領域確保
				regOffsetList = RP_MemAlloc(sizeof(RCREG_ROUTEREGLIST) * (aRouteMng->parcelInfo + i)->linkVol, e_MEM_TYPE_ROUTEPLAN);
				if (NULL == regOffsetList) {
					SC_LOG_ErrorPrint(SC_TAG_RC, "malloc error. size=%d "HERE,
							sizeof(RCREG_ROUTEREGLIST) * (aRouteMng->parcelInfo + i)->linkVol);
					break;
				}
				// テーブル初期化
				for (e = 0; e < (aRouteMng->parcelInfo + i)->linkVol; e++) {
					(regOffsetList + e)->regOffset = ALL_F32;
				}

				// 規制オフセットテーブル構築
				result = RC_RegRouteMakeRegOfsList(aRouteMng, i, readTbl.mapList->road, regOffsetList);
				if (e_SC_RESULT_SUCCESS != result) {
					SC_LOG_ErrorPrint(SC_TAG_RC, "RC_RegRouteMakeRegOfsList error. [0x%08x] "HERE, result);
					break;
				}
				// 規制該当判定
				result = regRouteSetRegFlag(aRouteMng, i, readTbl.mapList->road, regOffsetList);
				if (e_SC_RESULT_SUCCESS != result) {
					SC_LOG_ErrorPrint(SC_TAG_RC, "routeSearchRegIndex error. [0x%08x] "HERE, result);
					break;
				}
			}
		} while (0);

		// 地図解放
		if (e_SC_RESULT_SUCCESS != RC_FreeMapTbl(&readTbl, false)) {
			SC_LOG_ErrorPrint(SC_TAG_RC, "RC_FreeMapTbl error. [0x%08x] "HERE, result);
		}
		// メモリ解放
		if (NULL != regOffsetList) {
			RP_MemFree(regOffsetList, e_MEM_TYPE_ROUTEPLAN);
		}
	}

	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_END);
	return (e_SC_RESULT_SUCCESS);
}

/**
 * @brief 推奨経路用：規制リンク該当判定及びインデックス設定
 * @param 規制レコード先頭
 * @param 推奨経路管理
 * @param 処理対象パーセルインデックス
 * @param 規制オフセットテーブル
 * @memo 収集した規制オフセットテーブルを元に規制リンク列に該当するかを判定する
 */
static E_SC_RESULT regRouteSetRegFlag(SC_RP_RouteMng* aRouteMng, UINT32 aPclIndex, MAL_HDL aRoadBin, RCREG_ROUTEREGLIST *aRegOfsList) {
	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_START);

	if (NULL == aRoadBin || NULL == aRouteMng || NULL == aRegOfsList) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (e_SC_RESULT_BADPARAM);
	}

	SC_RP_ParcelInfo *parcelInfo = NULL;
	SC_RP_LinkInfo *linkInfo = NULL;
	MAL_HDL pRecodeTop = NULL;				// 規制レコード先頭
	MAL_HDL pReg = NULL;					// 規制先頭
	UINT32 regParcelId = 0;					// 判定対象規制パーセルID
	UINT32 regLinkId = 0;					// 判定対象規制リンクID
	UINT32 i, e;

	pRecodeTop = SC_MA_A_NWBIN_GET_REGULATION(aRoadBin);
	parcelInfo = aRouteMng->parcelInfo + aPclIndex;
	linkInfo = aRouteMng->linkInfo + parcelInfo->linkIdx;

	// 当該パーセル内の経路リンクに対して規制判定
	for (i = 0; i < parcelInfo->linkVol; i++) {

		Bool judge = false;							// 規制リンク一致フラグ
		UINT32 hitIndex = ALL_F32;					// 規制キーリンク一致インデックス
		UINT32 regOffset = ALL_F32;					// 規制オフセット
		RCREG_REGRECODINFO recodInfo = { };		// 規制レコード情報
		UINT32 routeLinkIdx = parcelInfo->linkIdx + i;	// 経路リンクインデックス

		// オフセット無効値は次
		if (ALL_F32 == (aRegOfsList + i)->regOffset) {
			continue;
		}
		// オフセット設定
		regOffset = (aRegOfsList + i)->regOffset;

		// 同一規制リンクレコードループ
		while (true) {
			// 規制取得
			recodInfo.pRegTop = SC_MA_A_NWRCD_REG_GET_RECOD(pRecodeTop, regOffset);

			// データ分解
			recodInfo.pRegDataTop = SC_MA_A_NWRCD_REG_GET_REG(recodInfo.pRegTop);
			recodInfo.regId = SC_MA_D_NWRCD_REG_GET_REGID(recodInfo.pRegTop);
			recodInfo.dataSize = SC_MA_D_NWRCD_REG_GET_DATASIZE(recodInfo.pRegTop);
			recodInfo.regSize = SC_MA_D_NWRCD_REG_GET_REGSIZE(recodInfo.pRegTop);
			recodInfo.sameLinkOfsVol = SC_MA_D_NWRCD_REG_GET_SAMEOFSVOL(recodInfo.pRegTop);

			// 規制データトップ
			pReg = recodInfo.pRegDataTop;

			// 削除フラグ
			if (recodInfo.regId & 0x80000000) {
				SC_LOG_WarnPrint(SC_TAG_RC, "delete regulation... 0x%08x "HERE, recodInfo.regId);
				break;
			}

			// 規制群（規制サイズ未満は継続）
			while (pReg - recodInfo.pRegDataTop < recodInfo.regSize) {
				RCREG_REGDATAINFO regInfo = { };

				// 基準パーセルID
				regParcelId = parcelInfo->parcelId;

				// 規制情報収集
				if (e_SC_RESULT_SUCCESS != RC_RegBuildRegInfo(pReg, &regInfo)) {
					SC_LOG_ErrorPrint(SC_TAG_RC, "RC_RegBuildRegInfo error. "HERE);
					break;
				}
#if 0
				SC_LOG_DebugPrint(SC_TAG_RC, "code=%d parcelId=0x%08x link1=%d link2=%d ", regInfo.code[0], parcelInfo->parcelId,
						regInfo.linkVol1, regInfo.linkVol2);
#endif

				switch (regInfo.code[0]) {
				case RCREG_REGID1:			// n+1~1 No left turn.
				case RCREG_REGID2:			// n+1~1 No right turn.
				case RCREG_REGID3:			// n+1~1 No straight on.
				case RCREG_REGID4:			// n+1~1 No u turn.
				case RCREG_ID_NOENTRY:		// n+m~1 No entry.
				case RCREG_REGID10:			// n+1~1 open.
				case RCREG_REGID11:			// n+1~1 close
				case RCREG_REGID15:			// n+1~1 generated regulation.
					// 退出リンク：1本 キーリンク：1本
					regLinkId = read4byte(regInfo.pLink);
					if ((regLinkId & MAL_LINKPNTID) == ((linkInfo + i)->linkId & MAL_LINKPNTID)) {
						// 未判定の侵入リンク側を判定
						judge = regRouteJudgeRegLinks(aRouteMng, aPclIndex, routeLinkIdx, &regInfo);
						hitIndex = 0;
					}
					break;
				case RCREG_ID_NOEXIT:		// n+m~m No exit.
					// 退出リンク：m本 キーリンク：m本
					for (e = 0; e < regInfo.linkVol1; e++) {
						regLinkId = read4byte(regInfo.pLink + e * 4);
						if ((regLinkId & MAL_LINKPNTID) == ((linkInfo + i)->linkId & MAL_LINKPNTID)) {
							// 未判定の侵入リンク側を判定
							judge = regRouteJudgeRegLinks(aRouteMng, aPclIndex, routeLinkIdx, &regInfo);
							hitIndex = e;
							break;
						}
					}
					break;
				case RCREG_REGID7:			// n+1~1 Only right turn.
				case RCREG_REGID8:			// n+1~1 Only left turn.
				case RCREG_REGID9:			// n+1~1 Only straight on.
					// 対象としないが同一規制オフセットに該当する
					regLinkId = read4byte(regInfo.pLink);
					if ((regLinkId & MAL_LINKPNTID) == ((linkInfo + i)->linkId & MAL_LINKPNTID)) {
						hitIndex = 0;
					}
					break;
				case RCREG_REGID12:		// ..R~R passage reguration.
					// 全リンクに対して当て込み
					for (e = 0; e < regInfo.linkVol1; e++) {
						regLinkId = read4byte(regInfo.pLink + e * 4);
						if ((regLinkId & MAL_LINKPNTID) == ((linkInfo + i)->linkId & MAL_LINKPNTID)) {
							judge = true;
							hitIndex = e;
							break;
						}
					}
					break;
				default:
					// リンク不定（14）・リザーブ（13）
					break;
				}

				// 20160628 規制該当フラグを立てるだけなので規制該当でブレイク
				if (judge) {
					break;
				}
				// next
				pReg = pReg + 4 + regInfo.linkSize + regInfo.indexSize + regInfo.codeSize;
			}
			// 規制該当フラグを立てる
			if (judge) {
				SC_LOG_DebugPrint(SC_TAG_RC, "route link is in regulation. pcl=0x%08x link=0x%08x "HERE, parcelInfo->parcelId,
						linkInfo->linkId);
				(linkInfo + i)->regFlag = 1;
				break;
			}
			if (ALL_F32 == hitIndex) {
				SC_LOG_ErrorPrint(SC_TAG_RC, "regulation no hit... "HERE);
				break;
			}
			// 同一リンク規制オフセットを格納
			regOffset = read4byte(recodInfo.pRegTop + 12 + recodInfo.regSize + hitIndex * 4);
			if ((aRegOfsList + i)->regOffset == regOffset) {
				break;
			}
		}
	}

	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_END);
	return (e_SC_RESULT_SUCCESS);
}

/**
 * @brief 推奨経路用：規制情報該当判定
 * @param 推奨経路管理
 * @param 推奨経路パーセルインデックス
 * @param 推奨経路リンクインデックス
 * @param 規制情報
 * @param 規制判定開始インデックス
 * @param 規制判定回数
 */
static Bool regRouteJudgeRegLinks(SC_RP_RouteMng* aRouteMng, UINT32 aRoutePclIdx, UINT32 aRouteLinkIdx, RCREG_REGDATAINFO* aRegInfo) {
	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_START);

	if (NULL == aRouteMng || NULL == aRegInfo) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (false);
	}

	SC_RP_ParcelInfo *parcelInfo = NULL;
	SC_RP_LinkInfo *linkInfo = NULL;
	UINT32 regParcelId = 0;						// 規制：パーセルID
	UINT32 regLinkId = 0;						// 規制：リンクID
	UINT32 pclIdx = aRoutePclIdx;				// パーセルインデックス
	UINT32 judgeCount;							// 判定回数
	UINT32 i;
	Bool judge = true;							// 判定結果
	UINT32 aRegStartIdx;
	UINT32 aRegJudgeVol;

	do {
		// 経路初回リンクは判定不可
		if (0 == aRouteLinkIdx) {
			judge = false;
			break;
		}
		// 規制コード別判断
		switch (aRegInfo->code[0]) {
		case RCREG_ID_NOEXIT:		// no_exit
			aRouteLinkIdx--;
			aRegStartIdx = aRegInfo->linkVol1;
			aRegJudgeVol = aRegInfo->linkVol2;
			break;
		case RCREG_ID_NOENTRY:		// no_entry
			aRegStartIdx = 0;
			aRegJudgeVol = aRegInfo->linkVol1;
			break;
#if _RP_REG_INDEFINITE
		case RCREG_ID_INDEFINITE:	// indefinite
			return (false);
#endif
		default:
			aRegStartIdx = 0;
			aRegJudgeVol = aRegInfo->linkVol1 + aRegInfo->linkVol2;
			break;
		}
		linkInfo = aRouteMng->linkInfo + aRouteLinkIdx;
		parcelInfo = aRouteMng->parcelInfo + pclIdx;

		// 初回パーセルIDは経路リンクIDと同じ
		regParcelId = parcelInfo->parcelId;

		// 規制分を回す
		for (judgeCount = 0; judgeCount < aRegJudgeVol; judgeCount++) {

			// パーセル情報のリンクインデックスを下回る場合デクリメント
			if ((aRouteLinkIdx - judgeCount) < parcelInfo->linkIdx) {
				if (0 > pclIdx - 1) {
					break;
				}
				pclIdx--;
				parcelInfo = aRouteMng->parcelInfo + pclIdx;
			}
			// 規制リンクID・パーセルID取得
			regLinkId = read4byte(aRegInfo->pLink + (aRegStartIdx + judgeCount) * 4);
			regParcelId = GET_REGPARCELID(regLinkId, regParcelId);

#if 0
			SC_LOG_InfoPrint(SC_TAG_RC, " judge...[%d] pcl=0x%08x 0x%08x link=0x%08x 0x%08x ", judgeCount, regParcelId,
					parcelInfo->parcelId, regLinkId, linkInfo->linkId);
#endif
			// 比較
			if (regParcelId != parcelInfo->parcelId) {
				break;
			}
			if ((regLinkId & MAL_LINKPNTID) != (linkInfo->linkId & MAL_LINKPNTID)) {
				break;
			}
			// 更新
			linkInfo--;
		}

		// 全て規制判定終了していれば完全一致
		if (judgeCount != aRegJudgeVol) {
			judge = false;
			break;
		}
		// no_entry用
		if (RCREG_ID_NOENTRY == aRegInfo->code[0]) {
			// パーセル情報のリンクインデックスを下回る場合デクリメント
			if ((aRouteLinkIdx - judgeCount) < parcelInfo->linkIdx) {
				if (0 > pclIdx - 1) {
					judge = false;
					break;
				}
				pclIdx--;
				parcelInfo = aRouteMng->parcelInfo + pclIdx;
			}
			UINT32 prePclId = regParcelId;
			for (i = 0; i < aRegInfo->linkVol2; i++) {
				// 規制リンクID・パーセルID取得
				regLinkId = read4byte(aRegInfo->pLink + (aRegInfo->linkVol1 + i) * 4);
				regParcelId = GET_REGPARCELID(regLinkId, prePclId);
#if 0
				SC_LOG_InfoPrint(SC_TAG_RC, " no_entry judge...[%d] pcl=0x%08x 0x%08x link=0x%08x 0x%08x ", i, regParcelId,
						parcelInfo->parcelId, regLinkId, linkInfo->linkId);
#endif
				// 比較
				if (regParcelId == parcelInfo->parcelId) {
					if ((regLinkId & MAL_LINKPNTID) == (linkInfo->linkId & MAL_LINKPNTID)) {
						break;
					}
				}
			}
			if (i == aRegInfo->linkVol2) {
				judge = false;
				break;
			}
		}
	} while (0);

#if 0
	SC_LOG_InfoPrint(SC_TAG_RC, "routeJudgeRegLinks result=%d ", judge);
#endif

	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_END);
	return (judge);
}

/*---------------------------------------------------------------------------------------------------------------------------*/

/**
 * ライカムパッチとなるか判定
 * @return true:該当する
 *         false:該当しない
 * @memo 2015/07/31 ライカムパッチ
 * @memo 性能確保のため一部パラメタチェックを無効化 2016/01/08
 */
Bool RC_JudgePatchReg(SCRP_NETCONTROLER* aNetTab, SCRC_TARGETLINKINFO *aInLink, SCRC_TARGETLINKINFO *aOutLink) {

	if (NULL == mPatchReg) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "[Reg] bad param"HERE);
		return (false);
	}

	Bool ret = false;
	UINT32 i;
	for (i = 0; i < mRaikamPatch.regInfoVol; i++) {
		if (mPatchReg(aNetTab, aInLink, aOutLink, &mRaikamPatch.regInfo[i])) {
			ret = true;
			break;
		}
	}
	return (ret);
}

/**
 * ライカムパッチ規制判定
 * @param aNetTab
 * @param aInLink
 * @param aOutLink
 * @param aRegInfo
 * @return true対象規制
 * @memo 2015/07/31 ライカムパッチ
 *       性能を考慮しパラメタチェックは無し。(RC_JudgePatchRegで実行済み)
 */
static Bool isPatchRegRaikam(SCRP_NETCONTROLER* aNetTab, SCRC_TARGETLINKINFO *aInLink, SCRC_TARGETLINKINFO *aOutLink,
		ROUTE_REG_INFO *aRegInfo) {
	Bool ret = false;
	void* bin = NULL;
	UINT32 linkId = 0;

	do {
		ROUTE_REG_LINK* linkList = &mRaikamPatch.linkList[aRegInfo->listIdx];

		// out 判定
		if (aOutLink->pclInfo->parcelId != linkList[0].parcelId) {
			break;
		}
		bin = SC_MA_A_NWRCD_LINK_GET_RECORD(aOutLink->pclInfo->mapNetworkLinkBin, aOutLink->linkTable->detaIndex);
		linkId = SC_MA_D_NWRCD_LINK_GET_ID(bin);
		if (linkId != linkList[0].linkId) {
			break;
		}

		// in 判定
		if (aInLink->pclInfo->parcelId != linkList[1].parcelId) {
			break;
		}
		bin = SC_MA_A_NWRCD_LINK_GET_RECORD(aInLink->pclInfo->mapNetworkLinkBin, aInLink->linkTable->detaIndex);
		linkId = SC_MA_D_NWRCD_LINK_GET_ID(bin);
		if (linkId != linkList[1].linkId) {
			break;
		}

		// preリンク判定
		SCRP_NETDATA* wkLink = aInLink->linkNet;
		UINT32 i;
		for (i = 2; i < aRegInfo->linkVol; i++) {
			if (ALL_F32 == wkLink->inLinkHist) {
				break;
			}
			SCRP_LINKINFO* preLink = RCNET_GET_HISTLINKINFO(aNetTab, wkLink);
			SCRP_PCLINFO* preInfo = RCNET_GET_HISTPCLINFO(aNetTab, wkLink);
			UINT32 or = RCNET_GET_HISTORIDX(wkLink);

			if (preInfo->parcelId != linkList[i].parcelId) {
				break;
			}
			bin = SC_MA_A_NWRCD_LINK_GET_RECORD(preInfo->mapNetworkLinkBin, preLink->detaIndex);
			linkId = SC_MA_D_NWRCD_LINK_GET_ID(bin);
			if (linkId != linkList[i].linkId) {
				break;
			}
			wkLink = &preLink->linkNet[or];
		}
		if (i < aRegInfo->linkVol) {
			break;
		}
		ret = true;
	} while (0);

	return (ret);
}

/**
 * ライカムパッチリンク情報設定
 * @memo 2015/07/31 ライカムパッチ
 */
static void setPatchRegRaikam() {
	UINT16 info = 0;
	UINT16 idx = 0;

	mRaikamPatch.regInfo[info].listIdx = idx;
	mRaikamPatch.regInfo[info++].linkVol = 3;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x28000189;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x2800007c;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x2800007d;

	mRaikamPatch.regInfo[info].listIdx = idx;
	mRaikamPatch.regInfo[info++].linkVol = 2;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x2800018a;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x28000190;

	mRaikamPatch.regInfo[info].listIdx = idx;
	mRaikamPatch.regInfo[info++].linkVol = 2;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x280001db;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x280002ad;

	mRaikamPatch.regInfo[info].listIdx = idx;
	mRaikamPatch.regInfo[info++].linkVol = 2;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x28000429;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x2800043c;

	mRaikamPatch.regInfo[info].listIdx = idx;
	mRaikamPatch.regInfo[info++].linkVol = 2;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x28000440;
	mRaikamPatch.linkList[idx].parcelId = 0x13bf0ded;
	mRaikamPatch.linkList[idx++].linkId = 0x28000432;

	mRaikamPatch.regInfoVol = info;
	mRaikamPatch.linkListVol = idx;

	// パッチ関数ポインタ
	mPatchReg = isPatchRegRaikam;
}

#if 0
/**
 * 対象のリンクの上位レベル情報を取得する
 */
static UINT32 getUpLevelLinkId(UINT32 aParcelId, UINT32 aLinkId) {

	E_SC_RESULT result = e_SC_RESULT_SUCCESS;
	SCRP_MAPREADTBL readTab = {0};
	SCRP_MAPDATA readList = {0};
	UINT32 upLinkId = 0;
	UINT16 index = 0;

	do {
		readTab.mapList = &readList;

		result = RC_ReadListMap(&aParcelId, 1, SC_DHC_KIND_ROAD, &readTab);
		if (e_SC_RESULT_SUCCESS != result) {
			SC_LOG_ErrorPrint(SC_TAG_RC, "RC_ReadListMap error. [0x%08x] "HERE, result);
			break;
		}
		if (NULL == readTab.mapList->road) {
			SC_LOG_DebugPrint(SC_TAG_RC, "map road read failed. pcl=0x%08x [0x%08x] "HERE, aParcelId, result);
			break;
		}
		index = SC_MA_BinSearchNwRecord(readTab.mapList->road, aLinkId, SC_MA_BINSRC_TYPE_LINK);
		if (ALL_F16 == index) {
			SC_LOG_ErrorPrint(SC_TAG_RC, "can't find link. link=0x%08x "HERE, aLinkId);
			break;
		}
		// アドレス特定
		MAL_HDL bin = SC_MA_A_NWBIN_GET_NWRCD_LINK(readTab.mapList->road);
		MAL_HDL exBin = SC_MA_A_NWBIN_GET_NWLINKEX(readTab.mapList->road);
		MAL_HDL pLink = SC_MA_A_NWRCD_LINK_GET_RECORD(bin, index - 1);
		UINT16 ofs1 = 0;
		UINT16 ofs2 = 0;

		if (0xFFFFFFFF == read4byte(pLink + 32) ) {
			// error
			break;
		}
		if (0 == (0x80000000 & read4byte(pLink + 32) )) {
			// リンク拡張
			exBin += (0x7FFFFFFF & read4byte(pLink + 32) );
			// リンクID
			ofs1 += 4;
			// 道路形状オフセット有無
			if (0x8000 & read2byte(exBin + 2) ) {
				ofs1 += 4;
			}
			// 上位レベルリンク有無
			if (0x4000 & read2byte(exBin + 2) ) {
				ofs2 = ofs1 + 4;
			}
			// 地域クラス収録情報有無
			if (0x1000 & read2byte(exBin + 2) ) {
				ofs2 += 4;
			}
			if (0 < ((read4byte(exBin + ofs1) >> 28) & 0x0000000F)) {
				// 上位あり
				upLinkId = read4byte(exBin + ofs2);
			} else {
				// 上位なし
			}
		}
	}while (0);

	// 地図解放

	return (0);
}
#endif

