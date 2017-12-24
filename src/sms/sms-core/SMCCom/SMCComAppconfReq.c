/*
 * GPS Navigation ---An open source GPS navigation core software
 *
 *
 * Copyright (c) 2016  Hitachi, Ltd.
 *
 * This program is dual licensed under GPL version 2 or a commercial license.
 * See the LICENSE file distributed with this source file.
 */

#include "SMCComInternal.h"

#define	CC_APPCONF_XML_NODE_APPCONF				"appconf"

#define CC_APPCONF_REQ_SEND_BODY_SIZE	512

// アプリ設定取得レスポンスXML情報
typedef struct _APPCONF {
	Char			*appconf;
	INT32			*status;
	Char			*apiStatus;
} APPCONF;

// アプリ設定取得XMLパーサ
typedef struct _APPCONF_PARSER {
	INT32			state;
	Char			*buf;
	APPCONF			appconf;
} APPCONF_PARSER;

enum AppconfStatus {
	CC_APPCONF_NODE_NONE = 0,
	CC_APPCONF_NODE_ELGG,
	CC_APPCONF_NODE_ELGG_CHILD,
	CC_APPCONF_NODE_STATUS,
	CC_APPCONF_NODE_RESULT,
	CC_APPCONF_NODE_RESULT_CHILD,
	CC_APPCONF_NODE_API_STATUS,
	CC_APPCONF_NODE_APPCONF
};

//------------------------------------------------
// 変数定義
//------------------------------------------------
static INT32	CB_Result;				// 処理結果 (コールバック関数用)

//------------------------------------------------
// 関数定義
//------------------------------------------------
static void CC_AppconfReq_CreateUri(const T_CC_CMN_SMS_API_PRM *param, Char *uri);
static void CC_AppconfReq_CreateBody(const T_CC_CMN_SMS_API_PRM *parm, Char *body);
static E_SC_RESULT CC_AppconfReq_AnalyzeHttpResp(const Char *body, E_CONTEXT_TYPE contextType, Char *appconf, Char *apiStatus);
static E_SC_RESULT CC_AppconfReq_XmlParse(const Char* xml, T_CC_CMN_SMS_RESPONSE_INFO* resp_inf, Char *appconf);
static void XMLCALL CC_AppconfReq_StartElement(void *userData, const char *name, const char **atts);
static void XMLCALL CC_AppconfReq_EndElement(void *userData, const char *name);
static void XMLCALL CC_AppconfReq_CharacterData(void *userData, const XML_Char *data, INT32 len);

/**
 * @brief アプリ設定取得
 * @param [IN] smcal        SMCAL
 * @param [IN] parm         APIパラメータ
 * @param [IN] appconf      APP設定
 * @param [IN] appconSize   APP設定サイズ
 * @param [IN] recv         センタ受信データ
 * @param [OUT] apiStatus   APIステータス
 * @return 処理結果(E_SC_RESULT)
 */
E_SC_RESULT CC_AppconfReq_SendRecv(SMCAL *smcal,
								   const T_CC_CMN_SMS_API_PRM *parm,
								   Char *appconf,
								   Char *recv,
								   INT32 recvBufSize,
								   Char *apiStatus)
{
	E_SC_RESULT	ret = e_SC_RESULT_SUCCESS;
	E_SC_CAL_RESULT	calRet = e_SC_CAL_RESULT_SUCCESS;
	Char	*uri = NULL;
	Char	*body = NULL;
	UINT32	bodySize = 0;
	E_CONTEXT_TYPE	contextType = E_TEXT_XML;
	SMCALOPT	opt = {};
	UINT32	recvSize = 0;
	Char	*data = NULL;
	INT32	status = 0;

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		// 初期化
		*apiStatus = EOS;
		opt.cancel = SCC_IsCancel;
#ifdef SC_CMN_BASIC_AUTH_SMS
		// BASIC認証
		opt.isBasicAuth = true;
		strcpy(opt.basic.basicAuthId, SC_CMN_BASIC_AUTH_ID);
		strcpy(opt.basic.basicAuthPwd, SC_CMN_BASIC_AUTH_PWD);
#endif

		// メモリ確保
		uri = (Char*)SCC_MALLOC(CC_CMN_URI_STR_MAX_LEN);
		if (NULL == uri) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"SCC_MALLOC error, " HERE);
			ret = e_SC_RESULT_MALLOC_ERR;
			break;
		}
		data = (Char*)SCC_MALLOC(CC_APPCONF_REQ_SEND_BODY_SIZE);
		if (NULL == data) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"SCC_MALLOC error, " HERE);
			ret = e_SC_RESULT_MALLOC_ERR;
			break;
		}

		// URI生成
		CC_AppconfReq_CreateUri(parm, uri);

		// body生成
		CC_AppconfReq_CreateBody(parm, data);

		// HTTPデータ送受信
		calRet = SC_CAL_PostRequest(smcal, uri, data, strlen(data), recv, recvBufSize, &recvSize, &opt);
		if (e_SC_CAL_RESULT_SUCCESS != calRet) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"SC_CAL_PostRequest error, " HERE);
			ret = ConvertResult(calRet);
			break;
		}

		// HTTPデータ解析
		calRet = SC_CAL_AnalyzeResponseStatus(smcal, recv, recvSize, (const Char**)&body, &bodySize, &contextType, &status);
		if (e_SC_CAL_RESULT_SUCCESS != calRet) {
			if (CC_CMN_SERVER_STOP == status) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"server stop..., " HERE);
				ret = e_SC_RESULT_SERVER_STOP;
			} else {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"SC_CAL_AnalyzeResponseStatus error, " HERE);
				ret = ConvertResult(calRet);
			}
			break;
		}

		// レスポンス解析
		ret = CC_AppconfReq_AnalyzeHttpResp(body, contextType, appconf, apiStatus);
		if (e_SC_RESULT_SUCCESS != ret) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"CC_AppconfReq_AnalyzeHttpResp error, " HERE);
			break;
		}
	} while (0);

	// メモリ解放
	if (NULL != uri) {
		SCC_FREE(uri);
	}
	if (NULL != data) {
		SCC_FREE(data);
	}

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);

	return (ret);
}


/**
 * @brief URL生成
 * @param [IN]  parm        APIパラメータ
 * @param [OUT] uri         URL
 * @return 処理結果(E_SC_RESULT)
 */
void CC_AppconfReq_CreateUri(const T_CC_CMN_SMS_API_PRM *parm, Char *uri)
{
	sprintf((char*)uri,
			"%s\?method=Appconf.req",
			parm->ApiPrmNavi.common_uri
	);
}

/**
 * @brief POSTパラメータ生成
 * @param [IN]  parm        APIパラメータ
 * @param [OUT] uri         URL
 */
void CC_AppconfReq_CreateBody(const T_CC_CMN_SMS_API_PRM *parm,
							  Char *body)
{
	sprintf((char*)body,
			"term_id=%s&term_sig=%s&guid=%s&user_sig=%s",
			parm->ApiPrmMups.new_term_id,
			parm->ApiPrmMups.term_sig,
			parm->ApiPrmMups.guid,
			parm->ApiPrmMups.user_sig
	);
}

/**
 * @brief レスポンス解析
 * @param [IN] body         センタ受信データ(XMLまたはバイナリ)
 * @param [IN] contextType  コンテキスト
 * @param [OUT] appconf     アプリ設定
 * @param [OUT] apiStatus   APIステータス
 * @return 処理結果(E_SC_RESULT)
 */
E_SC_RESULT CC_AppconfReq_AnalyzeHttpResp(const Char *body,
										  E_CONTEXT_TYPE contextType,
										  Char *appconf,
										  Char *apiStatus)
{
	E_SC_RESULT	ret = e_SC_RESULT_SUCCESS;
	T_CC_CMN_SMS_RESPONSE_INFO	rsp_inf;

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		if (E_TEXT_XML == contextType) {
			// XML解析
			rsp_inf.apiSts = apiStatus;

			ret = CC_AppconfReq_XmlParse((const Char*)body, &rsp_inf, appconf);
			if(e_SC_RESULT_SUCCESS != ret) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"CC_AppconfReq_XmlParse error, " HERE);
				ret = e_SC_RESULT_SMS_API_ERR;
				break;
			}

			// 正常系のXMLとして解析できなかった場合
			if (CC_CMN_XML_CIC_RES_STS_OK != rsp_inf.sts) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"status error, " HERE);
				ret = e_SC_RESULT_SMS_API_ERR;
				break;
			}
			if (!CHECK_API_STATUS(rsp_inf.apiSts)) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"api status error, " HERE);
				ret = e_SC_RESULT_SMS_API_ERR;
				break;
			}
		} else {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"Content-Type error, " HERE);
			ret = e_SC_RESULT_SMS_API_ERR;
			break;
		}
	} while (0);

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);

	return (ret);
}

/**
 * @brief Appconf.req応答メッセージ解析
 * @param [IN] xml          XMLファイルのフルパス
 * @param [IN] resp_inf     CICレスポンス情報
 * @param [OUT] appconf     アプリ設定
 * @param [OUT] opt         オプション設定
 * @return 処理結果(E_SC_RESULT)
 */
E_SC_RESULT CC_AppconfReq_XmlParse(const Char* xml,
								   T_CC_CMN_SMS_RESPONSE_INFO* resp_inf,
								   Char *appconf)
{
	E_SC_RESULT		ret = e_SC_RESULT_SUCCESS;
	APPCONF_PARSER	appconfParser = {};
	Char buf[CC_CMN_XML_PARSE_DATA_SIZE + 1] = {};
	XML_Parser parser = NULL;
	INT32	done = 0;
	INT32	len = 0;
	INT32	parsedLen = 0;

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		// 初期化
		resp_inf->sts = 0;
		appconfParser.buf = (Char*)SCC_MALLOC(CC_CMN_XML_BUF_SIZE + 1);
		if (NULL == appconfParser.buf) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"SCC_MALLOC error, " HERE);
			CB_Result = CC_CMN_RESULT_MALLOC_ERR;
			ret = CB_Result;
			break;
		}
		appconfParser.appconf.status = &resp_inf->sts;
		appconfParser.appconf.apiStatus = &resp_inf->apiSts[0];
		appconfParser.appconf.appconf = appconf;
		CB_Result = CC_CMN_RESULT_OK;

		// XMLパーサ生成
		parser = XML_ParserCreate(NULL);
		if (NULL == parser) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"XML_ParserCreate error, " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			ret = CB_Result;
			break;
		}

		// コールバック関数設定
		XML_SetUserData(parser, &appconfParser);
		XML_SetElementHandler(parser, CC_AppconfReq_StartElement, CC_AppconfReq_EndElement);
		XML_SetCharacterDataHandler(parser, CC_AppconfReq_CharacterData);

		while (!done) {
			if (CC_ISCANCEL()) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
				CB_Result = e_SC_RESULT_CANCEL;
				ret = CB_Result;
				break;
			}

			strncpy((char*)buf, &xml[parsedLen], (sizeof(buf) - 1));
			len = (INT32)strlen(buf);
			parsedLen += len;
			if (strlen(xml) <= parsedLen) {
				done = 1;
			} else {
				done = 0;
			}

			// XML解析
			if ((XML_STATUS_ERROR == XML_Parse(parser, (const char*)buf, len, done)) || (e_SC_RESULT_SUCCESS != CB_Result)) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"XML_Parse error(0x%08x), " HERE, CB_Result);
				CB_Result = e_SC_RESULT_SMS_API_ERR;
				ret = CB_Result;
				break;
			}

			if (!done) {
				// バッファクリア
				memset(buf, 0, (sizeof(buf) - 1));
			}
		}
	} while (0);

	if (NULL != appconfParser.buf) {
		SCC_FREE(appconfParser.buf);
	}

	if (NULL != parser) {
		XML_ParserFree(parser);
	}

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);

	return (ret);
}

/**
 * @brief タグ解析開始
 * @param [IN/OUT] userData ユーザデータ
 * @param [IN] name     タグ名
 * @param [OUT] atts    属性(未使用)
 */
void XMLCALL CC_AppconfReq_StartElement(void *userData, const char *name, const char **atts)
{
	APPCONF_PARSER *parser = (APPCONF_PARSER*)userData;

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		if (e_SC_RESULT_SUCCESS != CB_Result) {
			break;
		}
		if (CC_ISCANCEL()) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
			CB_Result = e_SC_RESULT_CANCEL;
			break;
		}

		// パラメータチェック
		if (NULL == userData) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[userData], " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			break;
		}
		if (NULL == name) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[name], " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			break;
		}

		// 初期化
		memset(parser->buf, 0, (CC_CMN_XML_BUF_SIZE + 1));

		if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_ELGG)) {
			// <elgg>
			parser->state = CC_APPCONF_NODE_ELGG;
		}

		// <elgg>
		if (CC_APPCONF_NODE_ELGG == parser->state) {
			if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_STATUS)) {
				// <status>
				parser->state = CC_APPCONF_NODE_STATUS;
			} else if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_RESULT)) {
				// <result>
				parser->state = CC_APPCONF_NODE_RESULT;
			} else if (0 != strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_ELGG)) {
				parser->state = CC_APPCONF_NODE_ELGG_CHILD;
			}
		} else if (CC_APPCONF_NODE_RESULT == parser->state) {
			if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_API_STATUS)) {
				// <api_status>
				parser->state = CC_APPCONF_NODE_API_STATUS;
			} else if (0 == strcmp((char*)name, (char*)CC_APPCONF_XML_NODE_APPCONF)) {
				// <appconf>
				parser->state = CC_APPCONF_NODE_APPCONF;
			} else {
				parser->state = CC_APPCONF_NODE_RESULT_CHILD;
			}
		} else {
			// 上記以外
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"format error, " HERE);
			CB_Result = e_SC_RESULT_SMS_API_ERR;
			break;
		}
	} while (0);

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);
}

/**
 * @brief タグ解析終了
 * @param [IN/OUT] userData ユーザデータ
 * @param [IN] name     タグ名
 */
void XMLCALL CC_AppconfReq_EndElement(void *userData, const char *name)
{
	APPCONF_PARSER *parser = (APPCONF_PARSER*)userData;

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		if (e_SC_RESULT_SUCCESS != CB_Result) {
			break;
		}
		if (CC_ISCANCEL()) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
			CB_Result = e_SC_RESULT_CANCEL;
			break;
		}

		// パラメータチェック
		if (NULL == userData) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[userData], " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			break;
		}
		if (NULL == parser->buf) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[parser->buf], " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			break;
		}
		if (NULL == name) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[name], " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			break;
		}

		if (CC_APPCONF_NODE_STATUS == parser->state) {
			// <status>
			*(parser->appconf.status) = atoi((char*)parser->buf);
			parser->state = CC_APPCONF_NODE_ELGG;
		} else if (CC_APPCONF_NODE_RESULT == parser->state) {
			// <result>
			parser->state = CC_APPCONF_NODE_ELGG;
		} else if (CC_APPCONF_NODE_API_STATUS == parser->state) {
			// <api_status>
			strcpy((char*)parser->appconf.apiStatus, (char*)parser->buf);
			parser->state = CC_APPCONF_NODE_RESULT;
		} else if (CC_APPCONF_NODE_APPCONF == parser->state) {
			// <appconf>
			strcpy((char*)parser->appconf.appconf, (char*)parser->buf);
			parser->state = CC_APPCONF_NODE_RESULT;
		} else if (CC_APPCONF_NODE_ELGG_CHILD == parser->state) {
			 parser->state = CC_APPCONF_NODE_ELGG;
		} else if (CC_APPCONF_NODE_RESULT_CHILD == parser->state) {
			 parser->state = CC_APPCONF_NODE_RESULT;
		}
	} while (0);

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);
}

/**
 * @brief 解析データ
 * @param [IN] userData ユーザデータ
 * @param [IN] data     データ
 * @param [IN] len      データ長
 */
void XMLCALL CC_AppconfReq_CharacterData(void *userData, const XML_Char *data, INT32 len)
{
	APPCONF_PARSER *parser = (APPCONF_PARSER*)userData;
	INT32	bufLen = 0;

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		if (e_SC_RESULT_SUCCESS != CB_Result) {
			break;
		}
		if (CC_ISCANCEL()) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
			CB_Result = e_SC_RESULT_CANCEL;
			break;
		}

		// パラメータチェック
		if (NULL == userData) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[userData], " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			break;
		}
		if (NULL == parser->buf) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[parser->buf], " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			break;
		}
		if (NULL == data) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[data], " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			break;
		}

		// データをバッファにコピー
		bufLen = strlen((char*)parser->buf);

		if (CC_APPCONF_NODE_STATUS == parser->state) {
			// <status>
			if (CC_CMN_XML_RES_STS_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_APPCONF_NODE_API_STATUS == parser->state) {
			// <api_status>
			if (CC_CMN_XML_RES_STS_CODE_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_APPCONF_NODE_APPCONF == parser->state) {
			// <appconf>
			if (SC_CMN_APPCONF_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		}
	} while (0);

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);
}
