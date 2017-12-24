/*
 * GPS Navigation ---An open source GPS navigation core software
 *
 *
 * Copyright (c) 2016  Hitachi, Ltd.
 *
 * This program is dual licensed under GPL version 2 or a commercial license.
 * See the LICENSE file distributed with this source file.
 */

#ifndef SMCORE_ROUTE_COST_CONFIG_INI_H
#define SMCORE_ROUTE_COST_CONFIG_INI_H

//-----------------------------------
// 構造体定義
//-----------------------------------
typedef struct _SC_COST_CONFIG {
	Bool				valid_f;									// ファイル読み込み成功有無

	// [Speed]
	struct {
		UINT32			road[SC_ROADTYPE_NUM][SC_LINKTYPE_NUM];		// 道路種別・リンク種別毎の速度
	} speed;
	// [Weight]
	struct {
		UINT32			road[SC_ROADTYPE_NUM][SC_LINKTYPE_NUM];		// 道路種別・リンク種別毎のコスト
	} weight;
	// [Turn]
	struct {
		UINT32			dir[SC_TURNTYPE_NUM];						// 転向コスト
		UINT32			apply_f[SC_ROADTYPE_NUM][SC_ROADTYPE_NUM];	// 進脱道路種別毎の転向コスト適用有無
	} turn;

} SC_COST_CONFIG;

typedef struct _SC_ROUTE_COST_CONFIG {
	// [Mode Path]
	struct {
		Char			folder[SC_MAX_PATH];						// フォルダパス
		Char			file[SC_SRCHMODE_NUM][SC_MAX_PATH];			// 探索モード別ファイル名
		SC_COST_CONFIG	cost[SC_SRCHMODE_NUM];						// 探索モード別コスト
	} mode;

} SC_ROUTE_COST_CONFIG;


// 道路種別・リンク種別毎の初期速度
extern UINT32 SC_ROUTE_COST_INIT_SPEED[SC_SRCHMODE_NUM][SC_ROADTYPE_NUM][SC_LINKTYPE_NUM];

// 道路種別・リンク種別毎の初期コスト
extern UINT32 SC_ROUTE_COST_INIT_WEIGHT[SC_SRCHMODE_NUM][SC_ROADTYPE_NUM][SC_LINKTYPE_NUM];

// 道路種別毎の初期転向コスト
extern UINT32 SC_ROUTE_COST_INIT_TURN[SC_SRCHMODE_NUM][SC_TURNTYPE_NUM];

// 道路種別毎の初期転向コスト
extern UINT8 SC_ROUTE_COST_INIT_TURN_APPLY[SC_SRCHMODE_NUM][SC_ROADTYPE_NUM][SC_ROADTYPE_NUM];

//-----------------------------------
// 外部I/F定義
//-----------------------------------
E_SC_RESULT SC_CONFIG_LoadRouteCostConfig(const Char *fileName, SC_ROUTE_COST_CONFIG *config);
E_SC_RESULT SC_CONFIG_SaveRouteCostConfig(const Char *fileName, SC_ROUTE_COST_CONFIG *config);

#endif // #ifndef SMCORE_ROUTE_COST_CONFIG_INI_H
