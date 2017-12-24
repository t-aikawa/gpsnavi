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
 * route.c
 *
 *  Created on: 2015/11/05
 *      Author:t.aikawa
 */

#include <stdlib.h>
#include <stdio.h>

#include "navicore.h"
#include "navi.h"

#define ICON_CNT_MAX			(10)

#define START_FLAG_NUM	(0)
#define GOAL_FLAG_NUM	(1)
#define PIN_FLAG_NUM	(2)
#define GUIDE_FLAG_NUM	(3)

SMMAPDYNUDI	demo_icon_info[ICON_CNT_MAX]={};	// ユーザ定義ダイナミックアイコンデータ情報
INT32		demo_icon_num=0;				// ユーザ定義ダイナミックアイコンデータ情報数
Bool		demo_disp_info[ICON_CNT_MAX]={};	// ユーザ定義ダイナミックアイコンデータの表示/非表示

void sample_init_demo_route_icon(void)
{
	demo_icon_info[START_FLAG_NUM].IconID = 21;	// start flag
	demo_icon_info[START_FLAG_NUM].Longititude	= (INT32)(0*1024.0*3600.0);
	demo_icon_info[START_FLAG_NUM].Latitude		= (INT32)(0*1024.0*3600.0);
	demo_icon_info[GOAL_FLAG_NUM ].IconID = 22;	// goal flag
	demo_icon_info[GOAL_FLAG_NUM ].Longititude	= (INT32)(0*1024.0*3600.0);
	demo_icon_info[GOAL_FLAG_NUM ].Latitude		= (INT32)(0*1024.0*3600.0);
	demo_icon_info[PIN_FLAG_NUM  ].IconID = 3;	// pin flag
	demo_icon_info[PIN_FLAG_NUM  ].Longititude	= (INT32)(0*1024.0*3600.0);
	demo_icon_info[PIN_FLAG_NUM  ].Latitude		= (INT32)(0*1024.0*3600.0);
	demo_icon_info[GUIDE_FLAG_NUM].IconID = 27;	// guide flag
	demo_icon_info[GUIDE_FLAG_NUM].Longititude	= (INT32)(0*1024.0*3600.0);
	demo_icon_info[GUIDE_FLAG_NUM].Latitude		= (INT32)(0*1024.0*3600.0);


	demo_disp_info[START_FLAG_NUM] = 0;	// start flag
	demo_disp_info[GOAL_FLAG_NUM ] = 0;	// goal flag
	demo_disp_info[PIN_FLAG_NUM  ] = 0;	// pin flag
	demo_disp_info[GUIDE_FLAG_NUM] = 0;	// guide flag

	demo_icon_num = 4;
}
void sample_set_demo_icon_pin_flag(SMGEOCOORD	*geoCood)
{
	demo_icon_info[PIN_FLAG_NUM].IconID = 3;	// pin flag
	demo_icon_info[PIN_FLAG_NUM].Longititude	= (INT32)(geoCood->longitude);
	demo_icon_info[PIN_FLAG_NUM].Latitude		= (INT32)(geoCood->latitude);
	demo_disp_info[PIN_FLAG_NUM] = 1;

	NC_DM_SetIconInfo(demo_icon_info,demo_icon_num);
	NC_DM_SetDynamicUDIDisplay(demo_disp_info,demo_icon_num);
	sample_hmi_set_pin_mode(1);
}

void sample_clear_demo_route_icon(void)
{
	demo_disp_info[START_FLAG_NUM] = 0;
	demo_disp_info[GOAL_FLAG_NUM ] = 0;
	demo_disp_info[PIN_FLAG_NUM  ] = 0;
	demo_disp_info[GUIDE_FLAG_NUM] = 0;

	//NC_DM_SetIconInfo(demo_icon_info,demo_icon_num);
	NC_DM_SetDynamicUDIDisplay(demo_disp_info,demo_icon_num);
}

void sample_set_demo_icon_guide_flag(SMGEOCOORD	*geoCood)
{
	demo_icon_info[GUIDE_FLAG_NUM].IconID = 27;	// guide flag
	demo_icon_info[GUIDE_FLAG_NUM].Longititude	= (INT32)(geoCood->longitude);
	demo_icon_info[GUIDE_FLAG_NUM].Latitude		= (INT32)(geoCood->latitude);
	demo_disp_info[GUIDE_FLAG_NUM] = 1;

	NC_DM_SetIconInfo(demo_icon_info,demo_icon_num);
	NC_DM_SetDynamicUDIDisplay(demo_disp_info,demo_icon_num);
}

void sample_reset_demo_icon_guide_flag(void)
{
	demo_disp_info[GUIDE_FLAG_NUM] = 0;
	NC_DM_SetDynamicUDIDisplay(demo_disp_info,demo_icon_num);
}

int sample_calc_demo_route(void)
{
	SMCARSTATE carState;
	INT32		ret;
	//E_SC_RESULT	ret = e_SC_RESULT_SUCCESS;
	//T_VIEW_INFO* p_vi;
	float st_lat;
	float st_lon;
	float ed_lat;
	float ed_lon;

	if(demo_disp_info[PIN_FLAG_NUM] == 0) return(1);

	ret = NC_DM_GetCarState(&carState, e_SC_CARLOCATION_NOW);
	if(NC_SUCCESS != ret) {
		printf("NC_DM_GetCarState err (%d)\n", ret);
	}
	//printf("(%d,%d)\n",demo_icon_info[PIN_FLAG_NUM].Longititude,demo_icon_info[PIN_FLAG_NUM].Latitude);
	// 自車位置緯度経度取得
	st_lon = (float)carState.coord.longitude/1024.0/3600.0;
	st_lat = (float)carState.coord.latitude/1024.0/3600.0;

	ed_lon = (float)demo_icon_info[PIN_FLAG_NUM].Longititude/1024.0/3600.0;
	ed_lat = (float)demo_icon_info[PIN_FLAG_NUM].Latitude/1024.0/3600.0;

	SMRPPOINT newPoint[2] = {};

	// 始点
	newPoint[0].coord.latitude = (st_lat*1024.0*3600.0);
	newPoint[0].coord.longitude = (st_lon*1024.0*3600.0);
	newPoint[0].rpPointType = LST_START;
	newPoint[0].rpPointIndex = 0;

	// 終点
	newPoint[1].coord.latitude = (ed_lat*1024.0*3600.0);
	newPoint[1].coord.longitude = (ed_lon*1024.0*3600.0);
	newPoint[1].rpPointType = LST_DEST;
	newPoint[1].rpPointIndex = 1;

	NC_RP_PlanSingleRoute(&newPoint[0],2,0);

	// icon
	demo_icon_info[START_FLAG_NUM].IconID = 21;	// start flag
	demo_icon_info[START_FLAG_NUM].Longititude	= (INT32)(st_lon*1024.0*3600.0);
	demo_icon_info[START_FLAG_NUM].Latitude		= (INT32)(st_lat*1024.0*3600.0);

	demo_icon_info[GOAL_FLAG_NUM ].IconID = 22;	// goal flag
	demo_icon_info[GOAL_FLAG_NUM ].Longititude	= (INT32)(ed_lon*1024.0*3600.0);
	demo_icon_info[GOAL_FLAG_NUM ].Latitude		= (INT32)(ed_lat*1024.0*3600.0);

	demo_icon_info[PIN_FLAG_NUM  ].IconID = 3;	// pin flag
	demo_icon_info[PIN_FLAG_NUM  ].Longititude	= 0;
	demo_icon_info[PIN_FLAG_NUM  ].Latitude		= 0;

	demo_icon_info[GUIDE_FLAG_NUM].IconID = 27;	// guide flag
	demo_icon_info[GUIDE_FLAG_NUM].Longititude	= 0;
	demo_icon_info[GUIDE_FLAG_NUM].Latitude		= 0;

	demo_disp_info[START_FLAG_NUM] = 1;
	demo_disp_info[GOAL_FLAG_NUM ] = 1;
	demo_disp_info[PIN_FLAG_NUM  ] = 0;
	demo_disp_info[GUIDE_FLAG_NUM] = 0;

	NC_DM_SetIconInfo(demo_icon_info,demo_icon_num);
	NC_DM_SetDynamicUDIDisplay(demo_disp_info,demo_icon_num);

	sample_hmi_set_pin_mode(0);

	sample_hmi_request_mapDraw();

	return 0;
}

