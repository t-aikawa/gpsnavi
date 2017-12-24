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
 * RC_RegModule.c
 *
 *  Created on: 2016/06/07
 *      Author: masutani
 */

#include "../SMCoreRPInternal.h"

#define RCREG_GETNEXTRECODE(p)		((p) + 4 * (read4byte(p)))


/**
 * @brief 規制レコード情報展開汎用関数
 * @param 規制レコード先頭
 * @param 規制レコード展開用テーブル
 */
E_SC_RESULT RC_RegBuildRegInfo(MAL_HDL aRegTop, RCREG_REGDATAINFO *aRegInfo) {

	if (NULL == aRegTop || NULL == aRegInfo) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (e_SC_RESULT_BADPARAM);
	}

	// 単独規制or複合規制
	if (0x80000000 == (read4byte(aRegTop) & 0xC0000000)) {
		// 複合規制
		aRegInfo->linkVol1 = read4byte(aRegTop) & 0x000000FF;
		aRegInfo->linkVol2 = (read4byte(aRegTop) >> 8) & 0x000000FF;
		aRegInfo->linkSize = aRegInfo->linkVol1 * 4 + aRegInfo->linkVol2 * 4;
		aRegInfo->indexSize = aRegInfo->linkVol1 * 2 + aRegInfo->linkVol2 * 2;
		if (aRegInfo->indexSize % 4) {
			aRegInfo->indexSize += 2;
		}
#if _RP_REG_INDEFINITE
	} else if (0x40000000 == (read4byte(aRegTop) & 0xC0000000)) {
		// リンク不定規制
		aRegInfo->linkVol1 = (read4byte(aRegTop) >> 8) & 0x000000FF; // 純粋な格納データ数
		aRegInfo->linkVol2 = read4byte(aRegTop) & 0x000000FF; // 判定を行うリンク数
		aRegInfo->linkSize = aRegInfo->linkVol1 * 4;
		aRegInfo->indexSize = aRegInfo->linkVol1 * 2;
		if (aRegInfo->indexSize % 4) {
			aRegInfo->indexSize += 2;
		}
#endif
	} else {
		// 単独規制
		aRegInfo->linkVol1 = read4byte(aRegTop) & 0x000FFFFF;
		aRegInfo->linkVol2 = 0;
		aRegInfo->linkSize = aRegInfo->linkVol1 * 4;
		aRegInfo->indexSize = aRegInfo->linkVol1 * 2;
		if (aRegInfo->indexSize % 4) {
			aRegInfo->indexSize += 2;
		}
	}
	// リンク先頭
	aRegInfo->pLink = aRegTop + 4;
	// インデックス先頭
	aRegInfo->pIndex = aRegTop + 4 + aRegInfo->linkSize;
	// コード先頭
	aRegInfo->pCode = aRegTop + 4 + aRegInfo->linkSize + aRegInfo->indexSize;
	// 通行コード数
	aRegInfo->codeVol = (read4byte(aRegTop) >> 20) & 0x000000FF;

	// コード取得、サイズ取得
	UINT32 i;
	for (i = 0; i < aRegInfo->codeVol; i++) {
		// block1
		aRegInfo->code[i] = (read4byte(aRegInfo->pCode + aRegInfo->codeSize) >> 20) & 0x0000000F;
		if (0 == (read4byte(aRegInfo->pCode + aRegInfo->codeSize) & 0x80000000)) {
			aRegInfo->codeSize += 4;
			continue;
		}
		// block2
		if (0 == (read4byte(aRegInfo->pCode + aRegInfo->codeSize + 4) & 0x80000000)) {
			aRegInfo->codeSize += 8;
			continue;
		}
		// block3
		if (0 == (read4byte(aRegInfo->pCode + aRegInfo->codeSize + 8) & 0x80000000)) {
			aRegInfo->codeSize += 12;
			continue;
		}
		aRegInfo->codeSize += 12;
	}

	return (e_SC_RESULT_SUCCESS);
}

/**
 * @brief 探索情報テーブルへ規制情報オフセットを格納する
 * @param 区間管理
 * @param ネットワーク管理
 */
E_SC_RESULT RC_RegSetRegOffset(SCRP_SECTCONTROLER *aSectCtrl, SCRP_NETCONTROLER *aNetCtrl) {
	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_START);

	if (NULL == aSectCtrl || NULL == aNetCtrl) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (e_SC_RESULT_BADPARAM);
	}

#if _RPLAPTIME_MAKEREGOFFSET
	UINT32 total = 0;
	RP_SetLapTimeWithStr("start regset."); // ▼時間計測
#endif

	UINT32 i, e, u;
	SCRP_PCLINFO* pclInfo = NULL;		// パーセル情報
	SCRP_LINKINFO* linkInfoTop = NULL;	// リンク情報テーブル先頭
	SCRP_NETDATA* netData = NULL;		// ネットワークデータ
	UINT32 recodeVol = 0;				// レコード数
	MAL_HDL pRecodeTop = NULL;			// レコード先頭

	// パーセル
	for (i = 0; i < aNetCtrl->parcelInfoVol; i++) {
		RCREG_REGRECODINFO recodInfo = { 0 };

		// パーセル・リンク情報テーブル取得
		pclInfo = aNetCtrl->parcelInfo + i;
		linkInfoTop = aNetCtrl->linkTable + pclInfo->linkIdx;

		// 規制なしの場合次
		if (NULL == pclInfo->mapNetworkRegBin) {
			continue;
		}
		pRecodeTop = pclInfo->mapNetworkRegBin;
		recodeVol = read4byte(pRecodeTop + 4);
		recodInfo.pRegTop = pRecodeTop + 8;

#if _RPLAPTIME_MAKEREGOFFSET
		total += recodeVol;
//		SC_LOG_DebugPrint(SC_TAG_RC, " regulation setting pcl=0x%08x recodeVol=%d size=%d"HERE, pclInfo->parcelId, recodeVol,
//				read4byte(pRecodeTop) );
#endif

		// レコード
		for (e = 0; e < recodeVol; e++) {
			MAL_HDL pReg = NULL;			// 規制先頭
			UINT32 myOffset = (recodInfo.pRegTop - pRecodeTop) / 4;

			recodInfo.pRegDataTop = SC_MA_A_NWRCD_REG_GET_REG(recodInfo.pRegTop);
			recodInfo.regId = SC_MA_D_NWRCD_REG_GET_REGID(recodInfo.pRegTop);
			recodInfo.dataSize = SC_MA_D_NWRCD_REG_GET_DATASIZE(recodInfo.pRegTop);
			recodInfo.regSize = SC_MA_D_NWRCD_REG_GET_REGSIZE(recodInfo.pRegTop);
			recodInfo.sameLinkOfsVol = SC_MA_D_NWRCD_REG_GET_SAMEOFSVOL(recodInfo.pRegTop);

			// 規制データ先頭をワークへ格納
			pReg = recodInfo.pRegDataTop;

			// 削除フラグ確認
			if (recodInfo.regId & 0x80000000) {
				recodInfo.pRegTop = recodInfo.pRegTop + recodInfo.dataSize;
				SC_LOG_WarnPrint(SC_TAG_RC, "delete regulation... 0x%08x "HERE, recodInfo.regId);
				continue;
			}

			// 規制群（規制サイズ未満は継続）
			while (pReg - recodInfo.pRegDataTop < recodInfo.regSize) {
				RCREG_REGDATAINFO regInfo = { 0 };
				UINT32 index = 0;

				// 規制情報収集
				if (e_SC_RESULT_SUCCESS != RC_RegBuildRegInfo(pReg, &regInfo)) {
					SC_LOG_ErrorPrint(SC_TAG_RC, "RC_RegBuildRegInfo error. "HERE);
					break;
				}

				// キーリンクに対して自身のオフセット格納
				switch (regInfo.code[0]) {
				case RCREG_REGID1:			// n+1~1 No left turn.
				case RCREG_REGID2:			// n+1~1 No right turn.
				case RCREG_REGID3:			// n+1~1 No straight on.
				case RCREG_REGID4:			// n+1~1 No u turn.
				case RCREG_ID_NOENTRY:		// n+m~1 No entry.
				case RCREG_REGID10:			// n+1~1 open.
				case RCREG_REGID11:			// n+1~1 close
#if _RP_REG_INDEFINITE
				case RCREG_ID_INDEFINITE:	// リンク不定規制
#endif
				case RCREG_REGID15:			// n+1~1 generated regulation.
					// 退出リンク：1本 キーリンク：1本
					index = read2byte(regInfo.pIndex);
					if (index < pclInfo->linkIdVol) {
						(linkInfoTop + index - 1)->regOfs = myOffset;
					}
					break;
				case RCREG_ID_NOEXIT:		// n+m~m No exit.
					// 退出リンク：m本 キーリンク：m本
					for (u = 0; u < regInfo.linkVol1; u++) {
						index = read2byte(regInfo.pIndex + u * 2);
						if (index < pclInfo->linkIdVol) {
							(linkInfoTop + index - 1)->regOfs = myOffset;
						}
					}
					break;
				case RCREG_REGID12:		// ..R~R passage reguration.
					// 全リンクに対して当て込み
					for (u = 0; u < regInfo.linkVol1; u++) {
						index = read2byte(regInfo.pIndex + u * 2);
						if (index < pclInfo->linkIdVol) {
							(linkInfoTop + index - 1)->regOfs = myOffset;
						}
					}
					break;
				case RCREG_REGID7:			// n+1~1 Only right turn.
				case RCREG_REGID8:			// n+1~1 Only left turn.
				case RCREG_REGID9:			// n+1~1 Only straight on.
					break;
				default:
					break;
				}
				// next
				pReg = pReg + 4 + regInfo.linkSize + regInfo.indexSize + regInfo.codeSize;
			}
			// next
			recodInfo.pRegTop = recodInfo.pRegTop + recodInfo.dataSize;
		}
	}

#if _RPLAPTIME_MAKEREGOFFSET
	char buf[128] = { 0 };
	sprintf(buf, "      reg count %d", total);
	RP_SetLapTimeWithStr(buf);
#endif

	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_END);
	return (e_SC_RESULT_SUCCESS);
}

/**
 * @brief 推奨経路の特定パーセルの規制オフセットテーブルを構築する
 * @param 推奨経路管理
 * @param 対象パーセルインデックス
 * @param ネットワークデータ先頭
 * @param 規制オフセットリスト格納ポインタ
 */
E_SC_RESULT RC_RegRouteMakeRegOfsList(SC_RP_RouteMng* aRouteMng, UINT32 aPclIndex, MAL_HDL aRoadBin, RCREG_ROUTEREGLIST *aRegOfsList) {
	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_START);

	if (NULL == aRoadBin || NULL == aRouteMng || NULL == aRegOfsList) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (e_SC_RESULT_BADPARAM);
	}

	UINT32 i, e, u;
	UINT32 recodeVol = 0;					// レコード数
	MAL_HDL pRecodeTop = NULL;				// レコード先頭
	MAL_HDL pRegGroupTop = NULL;			// 規制群先頭
	RCREG_REGRECODINFO recodInfo = { 0 };	// 規制レコード管理

	SC_RP_ParcelInfo *parcelInfo = aRouteMng->parcelInfo + aPclIndex;
	SC_RP_LinkInfo *linkInfo = aRouteMng->linkInfo + parcelInfo->linkIdx;

	// 規制情報有無チェック(事前チェックあり)->規制データ先頭取得
	if (ALL_F32 == SC_MA_D_NWBIN_GET_LINKREG_OFS(aRoadBin) ) {
		SC_LOG_DebugPrint(SC_TAG_RC, "no regulation paracel. "HERE);
		return (e_SC_RESULT_BADPARAM);
	}
	pRecodeTop = SC_MA_A_NWBIN_GET_REGULATION(aRoadBin);
	recodeVol = read4byte(pRecodeTop + 4);
	recodInfo.pRegTop = pRecodeTop + 8;

	// 規制レコードを総舐めして規制オフセットレコードテーブルを構築する
	for (i = 0; i < recodeVol; i++) {
		MAL_HDL pReg = NULL;			// 規制先頭
		UINT32 myOffset = (recodInfo.pRegTop - pRecodeTop) / 4;

		recodInfo.pRegDataTop = SC_MA_A_NWRCD_REG_GET_REG(recodInfo.pRegTop);
		recodInfo.regId = SC_MA_D_NWRCD_REG_GET_REGID(recodInfo.pRegTop);
		recodInfo.dataSize = SC_MA_D_NWRCD_REG_GET_DATASIZE(recodInfo.pRegTop);
		recodInfo.regSize = SC_MA_D_NWRCD_REG_GET_REGSIZE(recodInfo.pRegTop);
		recodInfo.sameLinkOfsVol = SC_MA_D_NWRCD_REG_GET_SAMEOFSVOL(recodInfo.pRegTop);

		// 規制データ先頭をワークへ格納
		pReg = recodInfo.pRegDataTop;

		// 削除フラグ
		if (recodInfo.regId & 0x80000000) {
			recodInfo.pRegTop = recodInfo.pRegTop + recodInfo.dataSize;
			SC_LOG_WarnPrint(SC_TAG_RC, "delete regulation... 0x%08x "HERE, recodInfo.regId);
			continue;
		}

		// 規制群（規制サイズ未満は継続）
		while (pReg - recodInfo.pRegDataTop < recodInfo.regSize) {
			RCREG_REGDATAINFO regInfo = { 0 };

			// 規制情報収集
			if (e_SC_RESULT_SUCCESS != RC_RegBuildRegInfo(pReg, &regInfo)) {
				SC_LOG_ErrorPrint(SC_TAG_RC, "RC_RegBuildRegInfo error. "HERE);
				break;
			}

			// キーリンクに対して自身のオフセット格納
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
				for (e = 0; e < parcelInfo->linkVol; e++) {
					if (((linkInfo + e)->linkId & MAL_LINKPNTID) == (read4byte(regInfo.pLink) & MAL_LINKPNTID)) {
						(aRegOfsList + e)->regOffset = myOffset;
					}
				}
				break;
			case RCREG_ID_NOEXIT:		// n+m~m No exit.
				// 退出リンク：m本 キーリンク：m本
				for (e = 0; e < regInfo.linkVol1; e++) {
					for (u = 0; u < parcelInfo->linkVol; u++) {
						if (((linkInfo + u)->linkId & MAL_LINKPNTID) == (read4byte(regInfo.pLink) & MAL_LINKPNTID)) {
							(aRegOfsList + u)->regOffset = myOffset;
						}
					}
				}
				break;
			case RCREG_REGID12:			// ..R~R passage reguration.
				// 全リンクに対して当て込み
				for (e = 0; e < regInfo.linkVol1; e++) {
					for (u = 0; u < parcelInfo->linkVol; u++) {
						if (((linkInfo + u)->linkId & MAL_LINKPNTID) == (read4byte(regInfo.pLink + e * 4) & MAL_LINKPNTID)) {
							(aRegOfsList + u)->regOffset = myOffset;
						}
					}
				}
				break;
			case RCREG_REGID7:			// n+1~1 Only right turn.
			case RCREG_REGID8:			// n+1~1 Only left turn.
			case RCREG_REGID9:			// n+1~1 Only straight on.
			default:
				break;
			}
			// next
			pReg = pReg + 4 + regInfo.linkSize + regInfo.indexSize + regInfo.codeSize;
		}
		// next
		recodInfo.pRegTop = recodInfo.pRegTop + recodInfo.dataSize;
	}

#if 0 // オフセットテーブルおよびリンクIDダンプ
	for (i = 0; i < parcelInfo->linkVol; i++) {
		SC_LOG_DebugPrint(SC_TAG_RC, "pcl=0x%08x link=0x%08x offset=%d", parcelInfo->parcelId, linkInfo->linkId,
				(aRegOfsList + i)->regOffset);
	}
#endif

	SC_LOG_DebugPrint(SC_TAG_RC, SC_LOG_END);
	return (e_SC_RESULT_SUCCESS);
}

/**
 * @brief 規制コードを参照し規制内かを判断する
 *        時間
 *        期間
 *        車種
 *        カープール
 *        現状全て1関数で賄うが、季節規制や時間規制別に判別が必要な場合関数を分割する必要あり
 * @param 日時
 * @param 規制情報
 * @memo TODO パラメータを規制コードと規制数に分解し他への流用を考える。
 */
UINT32 RC_RegJudgeRegulationResult(SMRPSEARCHTIME *aSearchTime, RCREG_REGDATAINFO* aRegInfo) {

	UINT32 result = RCREG_RESULT_NOREG;
	UINT32 wkResult = RCREG_RESULT_NOREG;
	UINT32 codeSize = 0;
	UINT32 i;
	RCREG_TIME timeReg = { 0 };

	if (NULL == aSearchTime || NULL == aRegInfo) {
		SC_LOG_ErrorPrint(SC_TAG_RC, "bad param. "HERE);
		return (RCREG_RESULT_NOREG);
	}

	for (i = 0; i < aRegInfo->codeVol; i++) {
		// 0クリア
		RP_Memset0(&timeReg, sizeof(timeReg));
		wkResult = RCREG_RESULT_NOREG;

		do {
			// block1
			timeReg.code = (read4byte(aRegInfo->pCode + codeSize) >> 24) & 0x0000007F;
			timeReg.id = (read4byte(aRegInfo->pCode + codeSize) >> 20) & 0x0000000F;
			timeReg.pass = read4byte(aRegInfo->pCode + codeSize) & 0x00003FFF;

			if (read4byte(aRegInfo->pCode + codeSize) & 0x80000000) {
				codeSize += 4;
				timeReg.onlyCode = false;
			} else {
				codeSize += 4;
				timeReg.onlyCode = true;
				break;
			}

			// block2
			timeReg.flag = (read4byte(aRegInfo->pCode + codeSize) >> 22) & 0x000001FF;
			timeReg.stHour = (read4byte(aRegInfo->pCode + codeSize) >> 17) & 0x0000001F;
			timeReg.stMinute = (read4byte(aRegInfo->pCode + codeSize) >> 11) & 0x0000003F;
			timeReg.edHour = (read4byte(aRegInfo->pCode + codeSize) >> 6) & 0x0000001F;
			timeReg.edMinute = read4byte(aRegInfo->pCode + codeSize) & 0x0000003F;

			if (read4byte(aRegInfo->pCode + codeSize) & 0x80000000) {
				codeSize += 4;
			} else {
				codeSize += 4;
				break;
			}

			// block3
			timeReg.stMonth = (read4byte(aRegInfo->pCode + codeSize) >> 27) & 0x0000000F;
			timeReg.stDay = (read4byte(aRegInfo->pCode + codeSize) >> 22) & 0x0000001F;
			timeReg.edMonth = (read4byte(aRegInfo->pCode + codeSize) >> 18) & 0x0000000F;
			timeReg.edDay = (read4byte(aRegInfo->pCode + codeSize) >> 13) & 0x0000001F;
			timeReg.week = (read4byte(aRegInfo->pCode + codeSize) >> 6) & 0x0000007F;

			if (read4byte(aRegInfo->pCode + codeSize) & 0x80000000) {
				codeSize += 4;
			} else {
				codeSize += 4;
				break;
			}
		} while (0);

#if 0 // 規制コードの詳細ダンプ
		RPDBG_DumpRegulationTime(&timeReg);
#endif
		// 車両判定
		if (0 == (timeReg.pass & 0x00000800)) {
			// 普通車対象外
			continue;
		}

		// カープール判断
		if (2 == timeReg.code) {
			// todo
		}

		// ゲート判断
		if (1 == timeReg.code) {
			// todo
		}

		// 1ブロック目のみのデータ
		if (timeReg.onlyCode) {
			// 期間の設定がない規制
			result |= RCREG_RESULT_REG;
			break;
		}
#if 0 // RC_RegJudgeRegulationResult試験用 残しておく
		if (sTestCodeON < 30) {
			RCREG_TIME dummyTime[6] = {
					{0,0,0,0,0x0180,1,6,10,6,20,11,30,11,30},
					{0,0,0,0,0x0180,1,6,10,6,10,11,30,11,10},
					{0,0,0,0,0x0180,1,6,10,6, 5,11,30,11,50},
					{0,0,0,0,0x0180,1,6,10,7,10,11,30,18,50},
					{0,0,0,0,0x0180,1,7,10,6,10,11,30, 5,50},
					{0,0,0,0,0x0180,1,7,10,2,10,11,30,11,30},
			};

			timeReg.stMonth = dummyTime[sTestCodeON % 6].stMonth;
			timeReg.stDay = dummyTime[sTestCodeON % 6].stDay;
			timeReg.stHour = dummyTime[sTestCodeON % 6].stHour;
			timeReg.stMinute = dummyTime[sTestCodeON % 6].stMinute;
			timeReg.edMonth = dummyTime[sTestCodeON % 6].edMonth;
			timeReg.edDay = dummyTime[sTestCodeON % 6].edDay;
			timeReg.edHour = dummyTime[sTestCodeON % 6].edHour;
			timeReg.edMinute = dummyTime[sTestCodeON % 6].edMinute;
			SC_LOG_InfoPrint(SC_TAG_RC, "              %d/%d %d:%d ", aSearchTime->mon, aSearchTime->mday, aSearchTime->hour, aSearchTime->min);
		}
#endif
		// 時間規制規制
		if (timeReg.flag & 0x0100) {

			//SC_LOG_InfoPrint(SC_TAG_RC, " time defo st=%d/%d %d:%d ed=%d/%d %d:%d", timeReg.stMonth, timeReg.stDay, timeReg.stHour, timeReg.stMinute, timeReg.edMonth, timeReg.edDay, timeReg.edHour, timeReg.edMinute);
			// 終日判定
			if (0 == timeReg.stHour && 24 == timeReg.edHour && 0 == timeReg.stMinute && 0 == timeReg.edMinute) {
				// 終日のため規制内
				wkResult |= RCREG_FLAG_TIMEIN;
				break;
			}
			// 逆転正常判定
			else if (timeReg.stHour == timeReg.edHour) {
				// 分を判断
				if (timeReg.stMinute == timeReg.edMinute) {
					// 同時（ありえない？）
				} else if (timeReg.stMinute > timeReg.edMinute) {
					// 逆転
					timeReg.edHour += 24;
				} else {
					// 正常
				}
			} else if (timeReg.stHour > timeReg.edHour) {
				// 逆転
				timeReg.edHour += 24;
			} else {
				// 正常
			}
			//SC_LOG_InfoPrint(SC_TAG_RC, " time edit st=%d/%d %d:%d ed=%d/%d %d:%d", timeReg.stMonth, timeReg.stDay, timeReg.stHour, timeReg.stMinute, timeReg.edMonth, timeReg.edDay, timeReg.edHour, timeReg.edMinute);

			// 補正後の情報で内外判定
			if ((timeReg.stHour <= aSearchTime->hour && timeReg.edHour >= aSearchTime->hour)
					|| (timeReg.stHour <= aSearchTime->hour + 24 && timeReg.edHour >= aSearchTime->hour + 24)) {
				if (timeReg.stHour == aSearchTime->hour) {
					if (timeReg.stMinute <= aSearchTime->min) {
						// 時間内
						wkResult |= RCREG_FLAG_TIMEIN;
					} else {
						// 時間外
						wkResult |= RCREG_FLAG_TIMEOUT;
					}
				} else if ((timeReg.edHour == aSearchTime->hour) || (timeReg.edHour == aSearchTime->hour + 24)) {
					if (timeReg.edMinute > aSearchTime->min) {
						// 時間内
						wkResult |= RCREG_FLAG_TIMEIN;
					} else {
						// 時間外
						wkResult |= RCREG_FLAG_TIMEOUT;
					}
				} else {
					// 時間内
					wkResult |= RCREG_FLAG_TIMEIN;
				}
			} else {
				// 時間外
				wkResult |= RCREG_FLAG_TIMEOUT;
			}
			//SC_LOG_InfoPrint(SC_TAG_RC, " time result=0x%08x", wkResult & RCREG_FLAG_TIMEIN);
		}
		// 期間or季節
		if (timeReg.flag & 0x0080) {
			UINT32 inResult;
			UINT32 outResult;
			//SC_LOG_InfoPrint(SC_TAG_RC, " day defo st=%d/%d %d:%d ed=%d/%d %d:%d", timeReg.stMonth, timeReg.stDay, timeReg.stHour, timeReg.stMinute, timeReg.edMonth, timeReg.edDay, timeReg.edHour, timeReg.edMinute);
			// todo 季節規制
			if (timeReg.flag & 0x0040) {
				// 季節規制マージンは1か月？
				inResult = RCREG_FLAG_SEASONIN;
				outResult = RCREG_FLAG_SEASONOUT;
			}
			// 期間規制
			else {
				if (timeReg.stMonth == timeReg.edMonth) {
					if (timeReg.stDay == timeReg.edDay) {
						// 正常
					} else if (timeReg.stDay > timeReg.edDay) {
						// 逆転
						timeReg.edMonth += 12;
					} else {
						// 正常
					}
				} else if (timeReg.stMonth > timeReg.edMonth) {
					// 逆転
					timeReg.edMonth += 12;
				} else {
					// 正常
				}
				inResult = RCREG_FLAG_DATEIN;
				outResult = RCREG_FLAG_DATEOUT;
			}
			//SC_LOG_InfoPrint(SC_TAG_RC, " day edit st=%d/%d %d:%d ed=%d/%d %d:%d", timeReg.stMonth, timeReg.stDay, timeReg.stHour, timeReg.stMinute, timeReg.edMonth, timeReg.edDay, timeReg.edHour, timeReg.edMinute);
			// 補正後の情報で内外判定
			if ((timeReg.stMonth <= aSearchTime->mon && timeReg.edMonth >= aSearchTime->mon)
					|| (timeReg.stMonth <= aSearchTime->mon + 12 && timeReg.edMonth >= aSearchTime->mon + 12)) {
				if (timeReg.stMonth == timeReg.edMonth || timeReg.stMonth == timeReg.edMonth - 12) {
					if (timeReg.stDay > timeReg.edDay) {
						// 日逆転
						if (timeReg.edDay >= aSearchTime->mday) {
							// 期間内
							wkResult |= inResult;
						} else if (timeReg.stDay <= aSearchTime->mday) {
							// 期間内
							wkResult |= inResult;
						} else {
							// 期間外
							wkResult |= outResult;
						}
					} else {
						// 日正常
						if (timeReg.stDay > aSearchTime->mday) {
							// 期間外
							wkResult |= outResult;
						} else if (timeReg.edDay < aSearchTime->mday) {
							// 期間外
							wkResult |= outResult;
						} else {
							// 期間内
							wkResult |= inResult;
						}
					}
				} else if (timeReg.stMonth == aSearchTime->mon) {
					if (timeReg.stDay <= aSearchTime->mday) {
						// 期間内
						wkResult |= inResult;
					} else {
						// 期間外
						wkResult |= outResult;
					}
				} else if (timeReg.edMonth == aSearchTime->mon) {
					if (timeReg.edDay >= aSearchTime->mday) {
						// 期間内
						wkResult |= inResult;
					} else {
						// 期間外
						wkResult |= outResult;
					}
				} else {
					// 期間内
					wkResult |= inResult;
				}
			} else {
				// 期間外
				wkResult |= outResult;
			}
			//SC_LOG_InfoPrint(SC_TAG_RC, " day result=0x%08x", wkResult & RCREG_FLAG_DATEIN);
		}
		// 祝日規制
		if (timeReg.flag & 0x0020) {
			// todo カレンダー
		}
		// 曜日規制規制
		if (timeReg.flag & 0x0010) {
			// 日：0 月：1 火：2 水：3 木：4 金：5 土：6
			UINT8 sftCount[7] = { 6, 5, 4, 3, 2, 1, 0 };
			// 内外判定
			if ((timeReg.week >> sftCount[aSearchTime->wday]) & 0x01) {
				wkResult |= RCREG_FLAG_WEEKIN;
			} else {
				wkResult |= RCREG_FLAG_WEEKOUT;
			}
		}

		// 規制内外判定
		if (wkResult & (RCREG_FLAG_TIMEOUT | RCREG_FLAG_DATEOUT | RCREG_FLAG_SEASONOUT | RCREG_FLAG_WEEKOUT)) {
			// 何れかの項目で期間外がある
			result |= RCREG_RESULT_TIMEREGOUT;
		} else if (wkResult & (RCREG_FLAG_TIMEIN | RCREG_FLAG_DATEIN | RCREG_FLAG_SEASONIN | RCREG_FLAG_WEEKIN)) {
			// 期間外の項目なし＆期間内の項目がある
			result |= RCREG_RESULT_TIMEREGIN;
		}
	}

	return (result);
}
