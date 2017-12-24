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

#define	CC_PKGREQ_SEND_BODY_SIZE				1024

#define	CC_PKGREQ_XML_NODE_GROUP_LIST			"group_list"
#define	CC_PKGREQ_XML_NODE_GROUP_ITEM			"item"
#define	CC_PKGREQ_XML_NODE_GROUP_ID				"group_id"
#define	CC_PKGREQ_XML_NODE_GROUP_NAME			"group_name"
#define	CC_PKGREQ_XML_NODE_GROUP_DESCRIPTION	"grpup_description"
#define	CC_PKGREQ_XML_NODE_TYPE					"type"
#define	CC_PKGREQ_XML_NODE_START_DATETM			"start_datetm"
#define	CC_PKGREQ_XML_NODE_END_DATETM			"end_datetm"
#define	CC_PKGREQ_XML_NODE_PKG_LIST				"pkg_list"
#define	CC_PKGREQ_XML_NODE_PKG_ITEM				"item"
#define	CC_PKGREQ_XML_NODE_PKG_ID				"id"
#define	CC_PKGREQ_XML_NODE_PKG_NAME				"name"
#define	CC_PKGREQ_XML_NODE_PKG_DESCRIPTION		"description"
#define	CC_PKGREQ_XML_NODE_PKG_AUTHOR			"author"
#define	CC_PKGREQ_XML_NODE_PKG_CONTACT			"contact"
#define	CC_PKGREQ_XML_NODE_PKG_ICON				"icon"
#define	CC_PKGREQ_XML_NODE_PKG_PACKAGE			"package"
#define	CC_PKGREQ_XML_NODE_PKG_VERSION			"version"
#define	CC_PKGREQ_XML_NODE_PKG_STARTDATETM		"start_datetm"
#define	CC_PKGREQ_XML_NODE_PKG_ENDDATETM		"end_datetm"
#define	CC_PKGREQ_XML_NODE_PKG_BUCKET_NM		"bucket_nm"
#define	CC_PKGREQ_XML_NODE_PKG_ACCESS_KEY		"access_key"
#define	CC_PKGREQ_XML_NODE_PKG_SECRET_KEY		"secret_key"

#define	CC_PKGREQ_RES_FILE						"pkg.list"

#define	CC_PKGREQ_XML_DATA_GROUP_ID_SIZE		CC_CMN_PKGMGR_GROUPID
#define	CC_PKGREQ_XML_DATA_GROUP_NAME_SIZE		CC_CMN_PKGMGR_GROUPNAME
#define	CC_PKGREQ_XML_DATA_GROUP_DESCRIPTION_SIZE	CC_CMN_PKGMGR_GROUPDESCRIPTION
#define	CC_PKGREQ_XML_DATA_TYPE_SIZE			1
#define	CC_PKGREQ_XML_DATA_START_DATETM_SIZE	11
#define	CC_PKGREQ_XML_DATA_END_DATETM_SIZE		11
#define	CC_PKGREQ_XML_DATA_PKG_ID_SIZE			CC_CMN_PKGMGR_PACKAGEID
#define	CC_PKGREQ_XML_DATA_PKG_NAME_SIZE		CC_CMN_PKGMGR_PACKAGENAME
#define	CC_PKGREQ_XML_DATA_PKG_DESCRIPTION_SIZE	CC_CMN_PKGMGR_PACKAGEDESCRIPTION
#define	CC_PKGREQ_XML_DATA_PKG_AUTHOR_SIZE		CC_CMN_PKGMGR_AUTHOR
#define	CC_PKGREQ_XML_DATA_PKG_CONTACT_SIZE		CC_CMN_MALEADDR_STR_SIZE
#define	CC_PKGREQ_XML_DATA_PKG_ICON_SIZE		CC_CMN_PKGMGR_ICON
#define	CC_PKGREQ_XML_DATA_PKG_PACKAGE_SIZE		CC_CMN_PKGMGR_PCKAGE
#define	CC_PKGREQ_XML_DATA_PKG_VERSION_SIZE		CC_CMN_PKGMGR_VERSION
#define	CC_PKGREQ_XML_DATA_PKG_STARTDATETM_SIZE	11
#define	CC_PKGREQ_XML_DATA_PKG_ENDDATETM_SIZE	11
#define	CC_PKGREQ_XML_DATA_PKG_BUCKET_NM_SIZE	CC_CMN_PKGMGR_BACKETNAME
#define	CC_PKGREQ_XML_DATA_PKG_ACCESS_KEY_SIZE	CC_CMN_PKGMGR_ACCESSKEY
#define	CC_PKGREQ_XML_DATA_PKG_SECRET_KEY_SIZE	CC_CMN_PKGMGR_SECRETKEY

// パッケージ一覧取得レスポンスXML情報
typedef struct _PACKAGEINFO {
	INT32			pkgGroupInfoNum;
	SMPACKAGEGROUPINFO	*pkgGroupInfo;
	Char			*id;
	Char			*apiStatus;
} PACKAGEINFO;

// パッケージ一覧取得XMLパーサ
typedef struct _PKGREQ_PARSER {
	INT32			state;
	Char			*buf;
	PACKAGEINFO		pkgInfo;
} PKGREQ_PARSER;

// pkg/req/
enum PkgReqStatus {
	CC_PKGREQ_NODE_NONE = 0,
	CC_PKGREQ_NODE_XML,
	CC_PKGREQ_NODE_XML_CHILD,
	CC_PKGREQ_NODE_API_STATUS,
	CC_PKGREQ_NODE_GROUP_LIST,
	CC_PKGREQ_NODE_GROUP_LIST_CHILD,
	CC_PKGREQ_NODE_GROUP_ITEM,
	CC_PKGREQ_NODE_GROUP_ITEM_CHILD,
	CC_PKGREQ_NODE_GROUP_ID,
	CC_PKGREQ_NODE_GROUP_NAME,
	CC_PKGREQ_NODE_GROUP_DESCRIPTION,
	CC_PKGREQ_NODE_TYPE,
	CC_PKGREQ_NODE_START_DATETM,
	CC_PKGREQ_NODE_END_DATETM,
	CC_PKGREQ_NODE_PKG_LIST,
	CC_PKGREQ_NODE_PKG_LIST_CHILD,
	CC_PKGREQ_NODE_PKG_ITEM,
	CC_PKGREQ_NODE_PKG_ITEM_CHILD,
	CC_PKGREQ_NODE_PKG_ID,
	CC_PKGREQ_NODE_PKG_NAME,
	CC_PKGREQ_NODE_PKG_DESCRIPTION,
	CC_PKGREQ_NODE_PKG_AUTHOR,
	CC_PKGREQ_NODE_PKG_CONTACT,
	CC_PKGREQ_NODE_PKG_ICON,
	CC_PKGREQ_NODE_PKG_PACKAGE,
	CC_PKGREQ_NODE_PKG_VERSION,
	CC_PKGREQ_NODE_PKG_START_DATETM,
	CC_PKGREQ_NODE_PKG_END_DATETM,
	CC_PKGREQ_NODE_PKG_BUCKET_NAME,
	CC_PKGREQ_NODE_PKG_ACCESS_KEY,
	CC_PKGREQ_NODE_PKG_SECRET_KEY
};

//------------------------------------------------
// 変数定義
//------------------------------------------------
static INT32	CB_Result;				// 処理結果 (コールバック関数用)
static Bool		CB_IsXmlFile;

//------------------------------------------------
// 関数定義
//------------------------------------------------
static void CC_PkgReq_CreateUri(const T_CC_CMN_SMS_API_PRM *param, Char *uri);
static void CC_PkgReq_CreateBody(const T_CC_CMN_SMS_API_PRM *parm, Char *body);
static E_SC_RESULT CC_PkgReq_AnalyzeHttpResp(const Char *body, E_CONTEXT_TYPE contextType, SMPACKAGEGROUPINFO *pkgInfo, INT32 *pkgInfoNum, SMCALOPT *opt, Char *apiStatus);
static E_SC_RESULT CC_PkgReq_XmlParse(const Char* xml, T_CC_CMN_SMS_RESPONSE_INFO* resp_inf, SMPACKAGEGROUPINFO* pkgInfo, INT32 *pkgInfoNum, SMCALOPT *opt);
static void XMLCALL CC_PkgReq_StartElement(void *userData, const char *name, const char **atts);
static void XMLCALL CC_PkgReq_EndElement(void *userData, const char *name);
static void XMLCALL CC_PkgReq_CharacterData(void *userData, const XML_Char *data, INT32 len);

/**
 * @brief パッケージ一覧取得（通常モード）
 * @param [in] smcal        SMCAL
 * @param [in] parm         APIパラメータ
 * @param [in] dirPath      アイコンファイル格納ディレクトリのフルパス
 * @param [out] pkgInfo     パッケージ一覧
 * @param [out] pkgInfoNum  パッケージグループ数
 * @param [in] recv         センタ受信データ
 * @param [out] apiStatus   APIステータス
 * @return 処理結果(E_SC_RESULT)
 */
E_SC_RESULT CC_PkgReq_SendRecv(SMCAL *smcal,
							   T_CC_CMN_SMS_API_PRM *parm,
							   const Char *dirPath,
							   Bool isDownload,
							   SMPACKAGEGROUPINFO *pkgInfo,
							   INT32 *pkgInfoNum,
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
	INT32	num = 0;
	INT32	num2 = 0;
	Char	*dlDirPath = NULL;

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		// 初期化
		opt.isResOutputFile = true;
		opt.cancel = SCC_IsCancel;
		CC_GetTempDirPath(opt.resFilePath);
		strcat(opt.resFilePath, CC_PKGREQ_RES_FILE);
#ifdef SC_CMN_BASIC_AUTH_SMS
		// BASIC認証
		opt.isBasicAuth = true;
		strcpy(opt.basic.basicAuthId, SC_CMN_BASIC_AUTH_ID);
		strcpy(opt.basic.basicAuthPwd, SC_CMN_BASIC_AUTH_PWD);
#endif

		// メモリ確保
		uri = (Char*)SCC_MALLOC(CC_CMN_URI_STR_MAX_LEN);
		if (NULL == uri) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, "SCC_MALLOC error, " HERE);
			ret = e_SC_RESULT_MALLOC_ERR;
			break;
		}
		data = (Char*)SCC_MALLOC(CC_PKGREQ_SEND_BODY_SIZE);
		if (NULL == data) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"SCC_MALLOC error, " HERE);
			ret = e_SC_RESULT_MALLOC_ERR;
			break;
		}
		dlDirPath = (Char*)SCC_MALLOC(SCC_MAX_PATH);
		if (NULL == dlDirPath) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"SCC_MALLOC error, " HERE);
			ret = e_SC_RESULT_MALLOC_ERR;
			break;
		}

		// URI生成
		CC_PkgReq_CreateUri(parm, uri);

		// body生成
		CC_PkgReq_CreateBody(parm, data);

		// HTTPデータ送受信
		calRet = SC_CAL_PostRequest(smcal, uri, data, strlen(data), recv, recvBufSize, &recvSize, &opt);
		if(e_SC_CAL_RESULT_SUCCESS != calRet){
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
		ret = CC_PkgReq_AnalyzeHttpResp(body, contextType, pkgInfo, pkgInfoNum, &opt, apiStatus);
		if (e_SC_RESULT_SUCCESS != ret) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"CC_PkgReq_AnalyzeHttpResp error, " HERE);
			break;
		}

		// ダウンロードする場合
		if (isDownload) {
			if ('/' == dirPath[strlen(dirPath) - 1]) {
				sprintf((char*)dlDirPath, "%s", dirPath);
			} else {
				sprintf((char*)dlDirPath, "%s/", dirPath);
			}

			// ディレクトリ作成
			ret = CC_MakeDir(dlDirPath);
			if (e_SC_RESULT_SUCCESS != ret) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"CC_MakeDir error[%s], " HERE, dlDirPath);
				break;
			}

			for (num = 0; num < *pkgInfoNum; num++) {
				for (num2 = 0; num2 < pkgInfo[num].pkgInfoNum; num2++) {
					// アイコンファイルダウンロード
					ret = CC_DownloadPackage(smcal, parm, PKGDLDATATYPE_ICON, &pkgInfo[num].pkgInfo[num2], dlDirPath);
					if (e_SC_RESULT_SUCCESS != ret) {
						SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"CC_DownloadPackage error, " HERE);
						ret = e_SC_RESULT_SUCCESS;
					}
				}
			}
			if (e_SC_RESULT_SUCCESS != ret) {
				break;
			}
		}
	} while (0);

	// XMLファイル削除
	remove(opt.resFilePath);

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
 * @param [in]  parm        APIパラメータ
 * @param [out] uri         URL
 * @return 処理結果(E_SC_RESULT)
 */
void CC_PkgReq_CreateUri(const T_CC_CMN_SMS_API_PRM *parm, Char *uri)
{
	sprintf((char*)uri,
			"%spkg/req/",
			parm->ApiPrmNavi.sms_sp_uri
	);
}

/**
 * @brief body生成
 * @param [in]  parm     APIパラメータ
 * @param [out] body     body
 * @return 処理結果(E_SC_RESULT)
 */
void CC_PkgReq_CreateBody(const T_CC_CMN_SMS_API_PRM *parm, Char *body)
{
	sprintf((char*)body,
			"term_id=%s&term_sig=%s&guid=%s&user_sig=%s&app_ver=%s",
			parm->ApiPrmMups.new_term_id,
			parm->ApiPrmMups.term_sig,
			parm->ApiPrmMups.guid,
			parm->ApiPrmMups.user_sig,
			parm->ApiPrmNavi.appVer
	);
}

/**
 * @brief レスポンス解析
 * @param [in] body         xmlデータ
 * @param [in] contextType  コンテキスト
 * @param [out] pkgInfo     パッケージ一覧
 * @param [out] pkgInfoNum  パッケージグループ数
 * @param [in] opt          オプション情報
 * @param [out] apiStatus   APIステータス
 * @return 処理結果(E_SC_RESULT)
 */
E_SC_RESULT CC_PkgReq_AnalyzeHttpResp(const Char *body,
									  E_CONTEXT_TYPE contextType,
									  SMPACKAGEGROUPINFO *pkgInfo,
									  INT32 *pkgInfoNum,
									  SMCALOPT *opt,
									  Char *apiStatus)
{
	E_SC_RESULT	ret = e_SC_RESULT_SUCCESS;
	T_CC_CMN_SMS_RESPONSE_INFO	rsp_inf = {};

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		if (E_TEXT_XML != contextType) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"Content-Type error, " HERE);
			ret = e_SC_RESULT_SMS_API_ERR;
			break;
		}

		// XML解析
		rsp_inf.apiSts = apiStatus;

		ret = CC_PkgReq_XmlParse((const Char*)body, &rsp_inf, pkgInfo, pkgInfoNum, opt);
		if(e_SC_RESULT_SUCCESS != ret) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"CC_PkgReq_XmlParse error, " HERE);
			ret = e_SC_RESULT_SMS_API_ERR;
			break;
		}

		// 正常系のXMLとして解析できなかった場合
		if ((!CHECK_API_STATUS(rsp_inf.apiSts)) && (!CHECK_API_STATUS2(rsp_inf.apiSts))) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"api status error, " HERE);
			ret = e_SC_RESULT_SMS_API_ERR;
			break;
		}
	} while (0);

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);

	return (ret);
}

/**
 * @brief pkg/req/応答メッセージ解析
 * @param [in] xml          XMLファイルのフルパス
 * @param [in] resp_inf     CICレスポンス情報
 * @param [out] pkgInfo     パッケージ一覧
 * @param [out] pkgInfoNum  パッケージグループ数
 * @param [out] opt         オプション情報
 * @return 処理結果(E_SC_RESULT)
 */
E_SC_RESULT CC_PkgReq_XmlParse(const Char* xml,
							   T_CC_CMN_SMS_RESPONSE_INFO* resp_inf,
							   SMPACKAGEGROUPINFO *pkgInfo,
							   INT32 *pkgInfoNum,
							   SMCALOPT *opt)
{
	E_SC_RESULT		ret = e_SC_RESULT_SUCCESS;
	PKGREQ_PARSER	pkgParser = {};
	Char buf[CC_CMN_XML_PARSE_DATA_SIZE + 1] = {};
	XML_Parser parser = NULL;
	INT32	done = 0;
	INT32	len = 0;
	INT32	parsedLen = 0;
	FILE	*fp = NULL;

	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_START);

	do {
		// パラメータチェック
		if ((NULL == opt) || (true != opt->isResOutputFile)) {
			if (NULL == xml) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"param error[xml], " HERE);
				CB_Result = e_SC_RESULT_FAIL;
				ret = CB_Result;
				break;
			}
		}

		// 初期化
		resp_inf->sts = 0;
		memset(pkgInfo, 0, (sizeof(SMPACKAGEGROUPINFO) * CC_CMN_PKGMGR_GROUP_MAXNUM));
		pkgParser.buf = (Char*)SCC_MALLOC(CC_CMN_XML_LARGE_BUF_SIZE + 1);
		if (NULL == pkgParser.buf) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"SCC_MALLOC error, " HERE);
			CB_Result = e_SC_RESULT_MALLOC_ERR;
			ret = CB_Result;
			break;
		}
		pkgParser.pkgInfo.pkgGroupInfo = pkgInfo;
		pkgParser.pkgInfo.pkgGroupInfoNum = 0;
		pkgParser.pkgInfo.apiStatus = &resp_inf->apiSts[0];
		CB_Result = e_SC_RESULT_SUCCESS;

		// XMLパーサ生成
		parser = XML_ParserCreate(NULL);
		if (NULL == parser) {
			SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"XML_ParserCreate error, " HERE);
			CB_Result = e_SC_RESULT_FAIL;
			ret = CB_Result;
			break;
		}

		// コールバック関数設定
		XML_SetUserData(parser, &pkgParser);
		XML_SetElementHandler(parser, CC_PkgReq_StartElement, CC_PkgReq_EndElement);
		XML_SetCharacterDataHandler(parser, CC_PkgReq_CharacterData);

		if ((NULL == opt) || (true != opt->isResOutputFile)) {
			CB_IsXmlFile = false;
		} else {
			CB_IsXmlFile = true;
			fp = fopen((char*)opt->resFilePath, "r");
			if (NULL == fp) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"file open error(%d)[%s], " HERE, errno, opt->resFilePath);
				CB_Result = e_SC_RESULT_FILE_ACCESSERR;
				ret = CB_Result;
				break;
			}
		}

		while (!done) {
			if (CC_ISCANCEL()) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"cancel, " HERE);
				CB_Result = e_SC_RESULT_CANCEL;
				ret = CB_Result;
				break;
			}

			if (!CB_IsXmlFile) {
				strncpy((char*)buf, &xml[parsedLen], (sizeof(buf) - 1));
				len = (INT32)strlen(buf);
				parsedLen += len;
				if (strlen(xml) <= parsedLen) {
					done = 1;
				} else {
					done = 0;
				}
			} else {
				len = (INT32)fread(buf, 1, (sizeof(buf) - 1), fp);
				done = (len < (sizeof(buf) - 1));
			}

			// XML解析
			if ((XML_STATUS_ERROR == XML_Parse(parser, (const char*)buf, len, done)) ||
				((e_SC_RESULT_SUCCESS != CB_Result) && (e_SC_RESULT_DATALIMIT != CB_Result))) {
				SCC_LOG_ErrorPrint(SC_TAG_CC, (Char*)"XML_Parse error(0x%08x), " HERE, CB_Result);
				CB_Result = e_SC_RESULT_SMS_API_ERR;
				ret = CB_Result;
				break;
			}
			if (e_SC_RESULT_DATALIMIT == CB_Result) {
				CB_Result = e_SC_RESULT_SUCCESS;
				break;
			}

			if (!done) {
				// バッファクリア
				memset(buf, 0, (sizeof(buf) - 1));
			}
		}
		if (e_SC_RESULT_SUCCESS == ret) {
			*pkgInfoNum = pkgParser.pkgInfo.pkgGroupInfoNum;
		}
	} while (0);

	if (NULL != pkgParser.buf) {
		SCC_FREE(pkgParser.buf);
	}
	if (NULL != fp) {
		fclose(fp);
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
 * @param [in] name     タグ名
 * @param [out] atts    属性(未使用)
 */
void XMLCALL CC_PkgReq_StartElement(void *userData, const char *name, const char **atts)
{
	PKGREQ_PARSER *parser = (PKGREQ_PARSER*)userData;
	SMPACKAGEGROUPINFO	*pkgGroupInfo = NULL;
	SMPACKAGEINFO	*pkgInfo = NULL;

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

		pkgGroupInfo = &parser->pkgInfo.pkgGroupInfo[parser->pkgInfo.pkgGroupInfoNum];
		pkgInfo = &pkgGroupInfo->pkgInfo[pkgGroupInfo->pkgInfoNum];

		if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_XML)) {
			// <xml>
			parser->state = CC_PKGREQ_NODE_XML;
		}

		// <xml>
		if (CC_PKGREQ_NODE_XML == parser->state) {
			if (0 == strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_API_STATUS)) {
				// <api_status>
				parser->state = CC_PKGREQ_NODE_API_STATUS;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_GROUP_LIST)) {
				// <group_list>
				parser->state = CC_PKGREQ_NODE_GROUP_LIST;
			} else if (0 != strcmp((char*)name, (char*)CC_CMN_XML_NODE_NAME_XML)) {
				parser->state = CC_PKGREQ_NODE_XML_CHILD;
			}
		} else if (CC_PKGREQ_NODE_GROUP_LIST == parser->state) {
			if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_GROUP_ITEM)) {
				// <item>
				parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
				if (CC_CMN_PKGMGR_GROUP_MAXNUM <= parser->pkgInfo.pkgGroupInfoNum) {
					SCC_LOG_WarnPrint(SC_TAG_CC, (Char*)"group item maxnum over, " HERE);
					CB_Result = e_SC_RESULT_DATALIMIT;
					break;
				}
			} else {
				parser->state = CC_PKGREQ_NODE_GROUP_LIST_CHILD;
			}
		} else if (CC_PKGREQ_NODE_GROUP_ITEM == parser->state) {
			if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_GROUP_ID)) {
				// <group_id>
				parser->state = CC_PKGREQ_NODE_GROUP_ID;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_GROUP_NAME)) {
				// <group_name>
				parser->state = CC_PKGREQ_NODE_GROUP_NAME;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_GROUP_DESCRIPTION)) {
				// <grpup_description>
				parser->state = CC_PKGREQ_NODE_GROUP_DESCRIPTION;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_TYPE)) {
				// <type>
				parser->state = CC_PKGREQ_NODE_TYPE;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_START_DATETM)) {
				// <start_datetm>
				parser->state = CC_PKGREQ_NODE_START_DATETM;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_END_DATETM)) {
				// <end_datetm>
				parser->state = CC_PKGREQ_NODE_END_DATETM;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_LIST)) {
				// <pkg_list>
				parser->state = CC_PKGREQ_NODE_PKG_LIST;
			} else {
				parser->state = CC_PKGREQ_NODE_GROUP_ITEM_CHILD;
			}
		} else if (CC_PKGREQ_NODE_PKG_LIST == parser->state) {
			if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_ITEM)) {
				// <item>
				parser->state = CC_PKGREQ_NODE_PKG_ITEM;
				if (CC_CMN_PKGMGR_PACKAGE_MAXNUM <= pkgGroupInfo->pkgInfoNum) {
					SCC_LOG_WarnPrint(SC_TAG_CC, (Char*)"pkg item maxnum over, " HERE);
					break;
				}
			} else {
				parser->state = CC_PKGREQ_NODE_PKG_LIST_CHILD;
			}
		} else if (CC_PKGREQ_NODE_PKG_ITEM == parser->state) {
			if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_ID)) {
				// <id>
				parser->state = CC_PKGREQ_NODE_PKG_ID;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_NAME)) {
				// <name>
				parser->state = CC_PKGREQ_NODE_PKG_NAME;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_DESCRIPTION)) {
				// <description>
				parser->state = CC_PKGREQ_NODE_PKG_DESCRIPTION;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_AUTHOR)) {
				// <author>
				parser->state = CC_PKGREQ_NODE_PKG_AUTHOR;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_CONTACT)) {
				// <contact>
				parser->state = CC_PKGREQ_NODE_PKG_CONTACT;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_ICON)) {
				// <icon>
				parser->state = CC_PKGREQ_NODE_PKG_ICON;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_PACKAGE)) {
				// <package>
				parser->state = CC_PKGREQ_NODE_PKG_PACKAGE;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_VERSION)) {
				// <version>
				parser->state = CC_PKGREQ_NODE_PKG_VERSION;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_STARTDATETM)) {
				// <start_datetm>
				parser->state = CC_PKGREQ_NODE_PKG_START_DATETM;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_ENDDATETM)) {
				// <end_datetm>
				parser->state = CC_PKGREQ_NODE_PKG_END_DATETM;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_BUCKET_NM)) {
				// <bucket_nm>
				parser->state = CC_PKGREQ_NODE_PKG_BUCKET_NAME;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_ACCESS_KEY)) {
				// <access_key>
				parser->state = CC_PKGREQ_NODE_PKG_ACCESS_KEY;
			} else if (0 == strcmp((char*)name, (char*)CC_PKGREQ_XML_NODE_PKG_SECRET_KEY)) {
				// <secret_key>
				parser->state = CC_PKGREQ_NODE_PKG_SECRET_KEY;
			} else {
				parser->state = CC_PKGREQ_NODE_PKG_ITEM_CHILD;
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
 * @param [in] name     タグ名
 */
void XMLCALL CC_PkgReq_EndElement(void *userData, const char *name)
{
	PKGREQ_PARSER *parser = (PKGREQ_PARSER*)userData;
	SMPACKAGEGROUPINFO	*pkgGroupInfo = NULL;
	SMPACKAGEINFO	*pkgInfo = NULL;

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

		pkgGroupInfo = &parser->pkgInfo.pkgGroupInfo[parser->pkgInfo.pkgGroupInfoNum];
		pkgInfo = &pkgGroupInfo->pkgInfo[pkgGroupInfo->pkgInfoNum];

		if (CC_PKGREQ_NODE_API_STATUS == parser->state) {
			// <api_status>
			strcpy((char*)parser->pkgInfo.apiStatus, (char*)parser->buf);
			parser->state = CC_PKGREQ_NODE_XML;
		} else if (CC_PKGREQ_NODE_GROUP_LIST == parser->state) {
			// <group_list>
			parser->state = CC_PKGREQ_NODE_XML;
		} else if (CC_PKGREQ_NODE_GROUP_ITEM == parser->state) {
			// <item>
			parser->state = CC_PKGREQ_NODE_GROUP_LIST;
			parser->pkgInfo.pkgGroupInfoNum++;
		} else if (CC_PKGREQ_NODE_GROUP_ID == parser->state) {
			// <group_id>
			strcpy((char*)pkgGroupInfo->groupId, (char*)parser->buf);
			parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
		} else if (CC_PKGREQ_NODE_GROUP_NAME == parser->state) {
			// <group_name>
			strcpy((char*)pkgGroupInfo->groupName, (char*)parser->buf);
			parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
		} else if (CC_PKGREQ_NODE_GROUP_DESCRIPTION == parser->state) {
			// <grpup_description>
			strcpy((char*)pkgGroupInfo->groupDescription, (char*)parser->buf);
			parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
		} else if (CC_PKGREQ_NODE_TYPE == parser->state) {
			// <type>
			pkgGroupInfo->type = atoi((char*)parser->buf);
			parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
		} else if (CC_PKGREQ_NODE_START_DATETM == parser->state) {
			// <start_datetm>
			pkgGroupInfo->startDatetm =atoi((char*)parser->buf);
			parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
		} else if (CC_PKGREQ_NODE_END_DATETM == parser->state) {
			// <end_datetm>
			pkgGroupInfo->endDatetm = atoi((char*)parser->buf);
			parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_LIST == parser->state) {
			// <pkg_list>
			parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_ITEM == parser->state) {
			// <item>
			parser->state = CC_PKGREQ_NODE_PKG_LIST;
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				pkgGroupInfo->pkgInfoNum++;
			}
		} else if (CC_PKGREQ_NODE_PKG_ID == parser->state) {
			// <id>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->packageId, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_NAME == parser->state) {
			// <name>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->packageName, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_DESCRIPTION == parser->state) {
			// <description>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->packageDescription, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_AUTHOR == parser->state) {
			// <author>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->author, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_CONTACT == parser->state) {
			// <contact>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->contact, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_ICON == parser->state) {
			// <icon>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->iconFileName, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_PACKAGE == parser->state) {
			// <package>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->packageFileName, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_VERSION == parser->state) {
			// <version>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->version, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_START_DATETM == parser->state) {
			// <start_datetm>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				pkgInfo->startDatetm = atoi((char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_END_DATETM == parser->state) {
			// <end_datetm>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				pkgInfo->endDatetm = atoi((char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_BUCKET_NAME == parser->state) {
			// <bucket_nm>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->backetName, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_ACCESS_KEY == parser->state) {
			// <access_key>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->accessKey, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_SECRET_KEY == parser->state) {
			// <secret_key>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				strcpy((char*)pkgInfo->secretKey, (char*)parser->buf);
			}
			parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		} else if (CC_PKGREQ_NODE_XML_CHILD == parser->state) {
			 parser->state = CC_PKGREQ_NODE_XML;
		} else if (CC_PKGREQ_NODE_GROUP_LIST_CHILD == parser->state) {
			 parser->state = CC_PKGREQ_NODE_GROUP_LIST;
		} else if (CC_PKGREQ_NODE_GROUP_ITEM_CHILD == parser->state) {
			 parser->state = CC_PKGREQ_NODE_GROUP_ITEM;
		} else if (CC_PKGREQ_NODE_PKG_LIST_CHILD == parser->state) {
			 parser->state = CC_PKGREQ_NODE_PKG_LIST;
		} else if (CC_PKGREQ_NODE_PKG_ITEM_CHILD == parser->state) {
			 parser->state = CC_PKGREQ_NODE_PKG_ITEM;
		}
	} while (0);

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);
}

/**
 * @brief 解析データ
 * @param [in] userData ユーザデータ
 * @param [in] data     データ
 * @param [in] len      データ長
 */
void XMLCALL CC_PkgReq_CharacterData(void *userData, const XML_Char *data, INT32 len)
{
	PKGREQ_PARSER *parser = (PKGREQ_PARSER*)userData;
	SMPACKAGEGROUPINFO	*pkgGroupInfo = NULL;
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

		pkgGroupInfo = &parser->pkgInfo.pkgGroupInfo[parser->pkgInfo.pkgGroupInfoNum];

		// データをバッファにコピー
		bufLen = strlen((char*)parser->buf);

		if (CC_PKGREQ_NODE_API_STATUS == parser->state) {
			// <api_status>
			if (CC_CMN_XML_RES_STS_CODE_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_PKGREQ_NODE_GROUP_LIST == parser->state) {
			// <group_list>
		} else if (CC_PKGREQ_NODE_GROUP_ITEM == parser->state) {
			// <item>
		} else if (CC_PKGREQ_NODE_GROUP_ID == parser->state) {
			// <group_id>
			if (CC_PKGREQ_XML_DATA_GROUP_ID_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_PKGREQ_NODE_GROUP_NAME == parser->state) {
			// <group_name>
			if (CC_PKGREQ_XML_DATA_GROUP_NAME_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_PKGREQ_NODE_GROUP_DESCRIPTION == parser->state) {
			// <grpup_description>
			if (CC_PKGREQ_XML_DATA_GROUP_DESCRIPTION_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_PKGREQ_NODE_TYPE == parser->state) {
			// <type>
			if (CC_PKGREQ_XML_DATA_TYPE_SIZE >= (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_PKGREQ_NODE_START_DATETM == parser->state) {
			// <start_datetm>
			if (CC_PKGREQ_XML_DATA_START_DATETM_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_PKGREQ_NODE_END_DATETM == parser->state) {
			// <end_datetm>
			if (CC_PKGREQ_XML_DATA_END_DATETM_SIZE > (bufLen + len)) {
				memcpy(&parser->buf[bufLen], data, len);
				parser->buf[bufLen + len] = EOS;
			}
		} else if (CC_PKGREQ_NODE_PKG_LIST == parser->state) {
			// <pkg_list>
		} else if (CC_PKGREQ_NODE_PKG_ITEM == parser->state) {
			// <item>
		} else if (CC_PKGREQ_NODE_PKG_ID == parser->state) {
			// <id>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_ID_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_NAME == parser->state) {
			// <name>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_NAME_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_DESCRIPTION == parser->state) {
			// <description>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_DESCRIPTION_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_AUTHOR == parser->state) {
			// <author>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_AUTHOR_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_CONTACT == parser->state) {
			// <contact>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_CONTACT_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_ICON == parser->state) {
			// <icon>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_ICON_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_PACKAGE == parser->state) {
			// <package>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_PACKAGE_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_VERSION == parser->state) {
			// <version>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_VERSION_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_START_DATETM == parser->state) {
			// <start_datetm>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_STARTDATETM_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_END_DATETM == parser->state) {
			// <end_datetm>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_ENDDATETM_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_BUCKET_NAME == parser->state) {
			// <bucket_nm>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_BUCKET_NM_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_ACCESS_KEY == parser->state) {
			// <access_key>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_ACCESS_KEY_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		} else if (CC_PKGREQ_NODE_PKG_SECRET_KEY == parser->state) {
			// <secret_key>
			if (CC_CMN_PKGMGR_PACKAGE_MAXNUM > pkgGroupInfo->pkgInfoNum) {
				if (CC_PKGREQ_XML_DATA_PKG_SECRET_KEY_SIZE > (bufLen + len)) {
					memcpy(&parser->buf[bufLen], data, len);
					parser->buf[bufLen + len] = EOS;
				}
			}
		}
	} while (0);

//	SCC_LOG_DebugPrint(SC_TAG_CC, SCC_LOG_END);
}
