/*
 * GPS Navigation ---An open source GPS navigation core software
 *
 *
 * Copyright (c) 2016  Hitachi, Ltd.
 *
 * This program is dual licensed under GPL version 2 or a commercial license.
 * See the LICENSE file distributed with this source file.
 */

//------------------------------------------------------------------------------------------------------------------------
//
//	SMCComDeviceidReq：Deviceid.req処理
//
//------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------
//インクルード
//---------------------------------------------------------------------------------
#include "SMCComInternal.h"

//---------------------------------------------------------------------------------
//プロトタイプ宣言
//---------------------------------------------------------------------------------
//***   コールバック関数   ***
void XMLCALL CC_DeviceidReq_StartElement(void *userData, const char *name, const char **atts);
void XMLCALL CC_DeviceidReq_EndElement(void *userData, const char *name);
void XMLCALL CC_DeviceidReq_CharacterData(void *userData, const XML_Char *data, INT32 len);
//***   内部関数   ***
E_SC_RESULT CC_DeviceidReq_CreateUri(const T_CC_CMN_SMS_API_PRM* pPrm, Char* pUri, INT32 UriMax);	//URI生成
E_SC_RESULT CC_DeviceidReq_Analy(T_CC_CMN_SMS_API_PRM* pPrm, const UINT8* apBuf, INT32 aBufSz, Char* apApiSts, E_CONTEXT_TYPE aContextType);	//レスポンス解析
E_SC_RESULT CC_DeviceidReq_XmlParse(const Char* apXml, T_CC_CMN_SMS_RESPONSE_INFO* p_resp_inf, Char* pTermId);//XML解析


//---------------------------------------------------------------------------------
//変数定義
//---------------------------------------------------------------------------------
static INT32	CB_Result;				// 処理結果 (コールバック関数用)


//---------------------------------------------------------------------------------
//外部インタフェース
//---------------------------------------------------------------------------------
//************************************************
//Deviceid.req送信・受信処理
//************************************************
/**
 * @brief Deviceid.req処理
 * @param[in] pPrm SMS API関連パラメタテーブルポインタ
 * @param[in] pRecvBuf 受信バッファポインタ
 * @param[in] RecvBuf_sz 受信バッファサイズ
 * @param[out] apUsrMsg ユーザメッセージ(地図センタからのユーザ向けエラー文言)
 * @retval 正常終了  :CC_CMN_RESULT_OK
 * @retval エラー終了:CC_CMN_RESULT_OK以外 (コール関数の戻り値)
 */
E_SC_RESULT CC_DeviceidReq_SendRecv(SMCAL* smcal,
									T_CC_CMN_SMS_API_PRM* pPrm,
									Char* pRecvBuf,
									UINT32 RecvBuf_sz,
									Char* pApiSts,
									Bool isPolling)
{
	E_SC_RESULT ret = CC_CMN_RESULT_OK;		// 戻り値(処理結果)
	E_SC_CAL_RESULT	cal_ret = CC_CMN_RESULT_SMCAL_OK;	// HTTPデータ系処理結果
	Char* p_body = NULL;					// httpレスポンスボディ部へのポインタ
	INT32 body_sz = 0;						// httpレスポンスボディ部サイズ
	E_CONTEXT_TYPE contextType = E_TEXT_XML;// Content-Type
	Char *uri = NULL;						// URI生成用文字バッファ
	UINT32 recv_sz = 0;
	SMCALOPT	opt = {};
	INT32	status = 0;

	do {
		// 初期化
		if (isPolling) {
			opt.cancel = SCC_IsCancel_Polling;
		} else {
			opt.cancel = SCC_IsCancel;
		}
#ifdef SC_CMN_BASIC_AUTH_MUPS
		// BASIC認証
		opt.isBasicAuth = true;
		strcpy(opt.basic.basicAuthId, SC_CMN_BASIC_AUTH_ID);
		strcpy(opt.basic.basicAuthPwd, SC_CMN_BASIC_AUTH_PWD);
#endif

		// メモリ確保
		uri = (Char*)SCC_MALLOC(CC_CMN_URI_STR_MAX_LEN);
		if (NULL == uri) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[AuthReq] CC_DeviceidReq_SendRecv() SCC_MALLOC() error " HERE);
			ret = CC_CMN_RESULT_MALLOC_ERR;
			break;
		}

		///URI生成
		ret = CC_DeviceidReq_CreateUri(pPrm, uri, CC_CMN_URI_STR_MAX_LEN);
		if(CC_CMN_RESULT_OK != ret){
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_SendRecv() CC_DeviceidReq_CreateUri() error " HERE);
			break;
		}

		///HTTPデータ送受信
		cal_ret = SC_CAL_GetRequest(smcal, uri, pRecvBuf, RecvBuf_sz, &recv_sz, &opt);
		if(CC_CMN_RESULT_SMCAL_OK != cal_ret){
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_SendRecv() SC_CAL_GetRequest() error：%x " HERE, cal_ret);
			ret = CC_CMN_RESULT_COMM_ERR;				// ひとまず通信エラーにする
			break;
		}

		///HTTPデータ解析
		cal_ret = SC_CAL_AnalyzeResponseStatus(smcal, pRecvBuf, recv_sz, (const Char**)&p_body, &body_sz, &contextType, &status);
		if(CC_CMN_RESULT_SMCAL_OK != cal_ret){
			if (CC_CMN_SERVER_STOP == status) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"server stop..., " HERE);
				ret = e_SC_RESULT_SERVER_STOP;
			} else {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_SendRecv() SC_CAL_AnalyzeResponseStatus() error：%x " HERE, cal_ret);
				ret = CC_CMN_RESULT_COMM_ERR;				// ひとまず通信エラーにする
			}
			break;
		}

		///CICデータ解析
		ret = CC_DeviceidReq_Analy(pPrm, (const UINT8 *)p_body, body_sz, pApiSts, contextType);
		if(CC_CMN_RESULT_OK != ret){
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_SendRecv() CC_DeviceidReq_Analy() error " HERE);
			break;
		}

	} while (0);

	if (NULL != uri) {
		SCC_FREE(uri);
	}

	return (ret);
}


//---------------------------------------------------------------------------------
//コールバック関数
//---------------------------------------------------------------------------------
//************************************************
//XML解析開始
//************************************************
/**
 * @brief タグ解析開始
 * @param [IN/OUT] userData ユーザデータ
 * @param [IN] name     タグ名
 * @param [OUT] atts    属性(未使用)
 * @return 処理結果(E_SC_RESULT)
 */
void XMLCALL CC_DeviceidReq_StartElement(void *userData, const char *name, const char **atts)
{
	T_CC_DEVICEIDREQ_PARSER	*pParser = (T_CC_DEVICEIDREQ_PARSER*)userData;	// パーサ

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		if (CC_CMN_RESULT_OK != CB_Result) {
			break;
		}
		if (CC_ISCANCEL()) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
			CB_Result = e_SC_RESULT_CANCEL;
			break;
		}

		// パラメータチェック
		if (NULL == userData) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_StartElement() prm error[userData], " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			break;
		}
		if (NULL == name) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_StartElement() prm error[name], " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			break;
		}

		// 初期化
		memset(pParser->buf, 0, (CC_CMN_XML_BUF_SIZE + 1));

		if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_CIC)) {
			// <cic_response>
			pParser->state = NODE_NONE;
		}

		// <cic_response>
		if (NODE_NONE == pParser->state) {
			if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_STATUS)) {
				// <status>
				pParser->state = NODE_STATUS;
			} else if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_DEV_ID)) {
				// <dev_id>
				pParser->state = NODE_DEV_ID;
			}
		} else {
			// 上記以外
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_StartElement() format error, " HERE);
			CB_Result = CC_CMN_RESULT_COMM_ERR;
			break;
		}
	} while (0);

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);
}

//************************************************
//XML解析終了
//************************************************
/**
 * @brief タグ解析終了
 * @param [IN/OUT] userData ユーザデータ
 * @param [IN] name     タグ名
 */
void XMLCALL CC_DeviceidReq_EndElement(void *userData, const char *name)
{
	T_CC_DEVICEIDREQ_PARSER	*pParser = (T_CC_DEVICEIDREQ_PARSER*)userData;	// パーサ

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		if (CC_CMN_RESULT_OK != CB_Result) {
			break;
		}
		if (CC_ISCANCEL()) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
			CB_Result = e_SC_RESULT_CANCEL;
			break;
		}

		// パラメータチェック
		if (NULL == userData) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_EndElement() prm error[userData], " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			break;
		}
		if (NULL == name) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_EndElement() prm error[name], " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			break;
		}

		if (NODE_STATUS == pParser->state) {
			// <status>
			*(pParser->deiceidInf.pStatus) = atoi((char*)pParser->buf);
			pParser->state = NODE_NONE;
		} else if (NODE_DEV_ID == pParser->state) {
			// <dev_id>
			strcpy((char*)pParser->deiceidInf.pDevId, (char*)pParser->buf);
			pParser->state = NODE_NONE;
		}
	} while (0);

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);
}

//************************************************
//CharacterData処理
//************************************************
/**
 * @brief 解析データ
 * @param [IN] userData ユーザデータ
 * @param [IN] data     データ
 * @param [IN] len      データ長
 * @return 処理結果(E_SC_RESULT)
 */
void XMLCALL CC_DeviceidReq_CharacterData(void *userData, const XML_Char *data, INT32 len)
{
	T_CC_DEVICEIDREQ_PARSER	*pParser = (T_CC_DEVICEIDREQ_PARSER*)userData;	// パーサ
	INT32	bufLen = 0;

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		if (CC_CMN_RESULT_OK != CB_Result) {
			break;
		}
		if (CC_ISCANCEL()) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
			CB_Result = e_SC_RESULT_CANCEL;
			break;
		}

		// パラメータチェック
		if (NULL == userData) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_CharacterData() prm error[userData], " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			break;
		}
		if (NULL == data) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_CharacterData() prm error[data], " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			break;
		}

		// データをバッファにコピー
		bufLen = strlen((char*)pParser->buf);

		if (NODE_STATUS == pParser->state) {
			// <status>
			if (CC_CMN_XML_RES_STS_SIZE > (bufLen + len)) {
				memcpy(&pParser->buf[bufLen], data, len);
				pParser->buf[bufLen + len] = EOS;
			}
		} else if (NODE_DEV_ID == pParser->state) {
			// <dev_id>
			if (CC_CMN_TARGETID_STR_SIZE > (bufLen + len)) {
				memcpy(&pParser->buf[bufLen], data, len);
				pParser->buf[bufLen + len] = EOS;
			}
		}
	} while (0);

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);
}

//---------------------------------------------------------------------------------
//内部関数
//---------------------------------------------------------------------------------
///************************************************
//URI生成
//************************************************
/**
 * @brief Deviceid.req用URI生成
 * @param[in] pPrm SMS API関連パラメタテーブルポインタ
 * @param[out] pUri 生成URI出力先
 * @param[in] UriMax 生成URI出力先最大サイズ(byte)
 * @retval 正常終了  :CC_CMN_RESULT_OK
 * @retval エラー終了:CC_CMN_RESULT_PROC_ERR
 * @pre SetPrmPreTokenReq()を事前にコール済みであること（不要か？）
 * @note term_sigは直前のToken.Reqのレスポンス処理時に生成した端末アクセスキーを使用
*/
E_SC_RESULT CC_DeviceidReq_CreateUri(const T_CC_CMN_SMS_API_PRM* pPrm,
										Char* pUri,
										INT32 UriMax)
{
	INT32 rslt = 0;
	E_SC_RESULT ret = CC_CMN_RESULT_OK;

	rslt = snprintf(									//成功時には、書き込まれたbyte数を返す(EOSは含まない)
				(char*)pUri,
				UriMax,
				"%s\?method=Deviceid.req&init_item_id=%s&term_id=%s&term_sig=%s",
				pPrm->ApiPrmMups.common_uri,		//「http://～/distribution/」までのURI部分
				pPrm->ApiPrmNavi.init_item_id,		//初期商品ID
				pPrm->ApiPrmNavi.reg_term_id,		//端末ID
				pPrm->ApiPrmMups.term_sig			//端末アクセスキー
				);
	if((rslt < 0) || (UriMax <= rslt)){				//負なら明らかにエラー、EOS終端できなかった場合もエラー
		SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_CreateUri() error len[%d] " HERE, rslt);
		ret = CC_CMN_RESULT_PROC_ERR;
	}

	return (ret);
}

//************************************************
//レスポンス解析
//************************************************
/**
 * @brief Deviceid.req応答解析
 * @param[in] pPrm SMS API関連パラメタテーブルポインタ
 * @param[in] apBuf HTTP受信データ格納先
 * @param[in] apBufSz HTTP受信データサイズ
 * @param[in] aContextType Content-Type(0:XML、1:バイナリ)
 * @retval 正常終了  :CC_CMN_RESULT_OK
 * @retval エラー終了:DAL_DATA_ERROR
 * @note センター採番新規TermIDの退避も行う
 */
E_SC_RESULT CC_DeviceidReq_Analy(T_CC_CMN_SMS_API_PRM* pPrm,
									const UINT8* apBuf,
									INT32 aBufSz,
									Char* apApiSts,
									E_CONTEXT_TYPE aContextType)
{
	E_SC_RESULT ret = CC_CMN_RESULT_OK;
	T_CC_CMN_SMS_RESPONSE_INFO rsp_inf = {};

	USE_IT(aBufSz);

	do {

		if(E_TEXT_XML != aContextType){
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_Analy() Content-Type error " HERE);
			ret = CC_CMN_RESULT_PARAM_ERR;
			break;
		}

		///XML解析部をコール
		rsp_inf.apiSts = apApiSts;
		ret = CC_DeviceidReq_XmlParse((const Char*)apBuf, &rsp_inf, pPrm->ApiPrmMups.new_term_id);
		if(CC_CMN_RESULT_OK != ret){
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_Analy() XmlParse() error " HERE);
			break;
		}

		//正常系のXMLとして解析できなかった場合
		if(CC_CMN_XML_CIC_RES_STS_OK != rsp_inf.sts){
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_Analy() not nomar xml error " HERE);
			ret = e_SC_RESULT_SMS_API_ERR;
			break;
		}

#ifdef CC_DEBUG_MODE		//*******    デバッグ用表示   *******************************************************************
		SCC_LOG_DebugPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_Analy() dev_id = %s ", pPrm->ApiPrmMups.new_term_id);
		SCC_LOG_DebugPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_Analy() status = %d ", rsp_inf.sts);
#endif

	} while (0);

	return (ret);
}

//************************************************
//XML解析
//************************************************
/**
 * @brief Deviceid.req応答メッセージ解析
 * @param[in] apXml XMLファイルのフルパス
 * @param[in] p_resp_inf CICレスポンス情報
 * @param[out] pUserSig dev_id文字列
 * @retval 正常終了  :CC_CMN_RESULT_OK
 * @retval エラー終了:CC_CMN_RESULT_PROC_ERR
 */
E_SC_RESULT CC_DeviceidReq_XmlParse(const Char* apXml,
							T_CC_CMN_SMS_RESPONSE_INFO* p_resp_inf,
							Char* pTermId)
{
	E_SC_RESULT ret = CC_CMN_RESULT_OK;		// 戻り値
	T_CC_DEVICEIDREQ_PARSER devidreqParser = {};	// パーサ
	Char buf[CC_CMN_XML_PARSE_DATA_SIZE + 1] = {};
	XML_Parser parser = NULL;
	INT32	done = 0;
	INT32	len = 0;
	INT32	parsedLen = 0;

	do {

		// パラメータチェック
		if (NULL == apXml) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_XmlParse() prm error[apXml] " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			ret = CB_Result;
			break;
		}
		if (NULL == p_resp_inf || NULL == p_resp_inf->apiSts) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_XmlParse() prm error[p_resp_inf] " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			ret = CB_Result;
			break;
		}
		if (NULL == pTermId) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_XmlParse() prm error[pTermId] " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			ret = CB_Result;
			break;
		}

		// 初期化
		*(pTermId) = EOS;
		p_resp_inf->sts = 0;
		*(p_resp_inf->apiSts) = EOS;
		CB_Result = CC_CMN_RESULT_OK;
		devidreqParser.buf = (Char*)SCC_MALLOC(CC_CMN_XML_BUF_SIZE + 1);
		if (NULL == devidreqParser.buf) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_XmlParse() SCC_MALLOC error, " HERE);
			CB_Result = CC_CMN_RESULT_MALLOC_ERR;
			ret = CB_Result;
			break;
		}
		// XMLパーサ生成
		parser = XML_ParserCreate(NULL);
		if (NULL == parser) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_XmlParse() XML_ParserCreate() error, " HERE);
			CB_Result = CC_CMN_RESULT_PROC_ERR;
			ret = CB_Result;
			break;
		}

		devidreqParser.deiceidInf.pStatus = &p_resp_inf->sts;
		devidreqParser.deiceidInf.pDevId = pTermId;

		// コールバック関数設定
		XML_SetUserData(parser, &devidreqParser);
		XML_SetElementHandler(parser, CC_DeviceidReq_StartElement, CC_DeviceidReq_EndElement);
		XML_SetCharacterDataHandler(parser, CC_DeviceidReq_CharacterData);

		while (!done) {
			if (CC_ISCANCEL()) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
				CB_Result = e_SC_RESULT_CANCEL;
				ret = CB_Result;
				break;
			}

			strncpy((char*)buf, &apXml[parsedLen], (sizeof(buf) - 1));
			len = (INT32)strlen(buf);
			parsedLen += len;
			if (strlen(apXml) <= parsedLen) {
				done = 1;
			} else {
				done = 0;
			}

			// XML解析
			if ((XML_STATUS_ERROR == XML_Parse(parser, (const char*)buf, len, done)) || (CC_CMN_RESULT_OK != CB_Result)) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"[DeviceidReq] CC_DeviceidReq_XmlParse() XML_Parse() error(0x%08x), " HERE, CB_Result);
				CB_Result = CC_CMN_RESULT_COMM_ERR;
				ret = CB_Result;
				break;
			}

			if (!done) {
				// バッファクリア
				memset(buf, 0, (sizeof(buf) - 1));
			}
		}

	}while(0);

	if (NULL != devidreqParser.buf) {
		SCC_FREE(devidreqParser.buf);
	}

	if (NULL != parser) {
		XML_ParserFree(parser);
	}

	return (ret);
}

