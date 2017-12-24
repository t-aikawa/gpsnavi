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
 * navicoredef.h
 *
 *  Created on: 2016/05/06
 *      Author:
 */

#ifndef NAVICOREDEFEX_H_
#define NAVICOREDEFEX_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief 言語を設定する
 * @param[in]  language  言語
 * @return 処理結果(1以上：成功　以外:失敗)
 */
INT32 NC_DM_SetLanguage(INT32 language);

/**
 * @brief 音声ＴＴＳ情報を取得する
 */
INT32 NC_Guide_GetVoiceTTS(char ward1[]);

#ifdef __cplusplus
}
#endif

#endif /* NAVICOREDEFEX_H_ */

