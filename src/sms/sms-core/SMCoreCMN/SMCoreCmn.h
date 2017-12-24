/*
 * GPS Navigation ---An open source GPS navigation core software
 *
 *
 * Copyright (c) 2016  Hitachi, Ltd.
 *
 * This program is dual licensed under GPL version 2 or a commercial license.
 * See the LICENSE file distributed with this source file.
 */

#ifndef SMCORE_CMN_H
#define SMCORE_CMN_H

#define	SC_MUTEX_INITIALIZER	{}

//-----------------------------------
// ログ出力
//-----------------------------------
#define	SC_TAG_NC			(Char*)"NaviCore"
#define	SC_TAG_CORE			(Char*)"SMCore"
#define	SC_TAG_MEM			(Char*)"SMCoreMem"
#define	SC_TAG_SHARE		(Char*)"SMCoreShare"
#define	SC_TAG_CASH			(Char*)"SMCoreCash"
#define	SC_TAG_FM			(Char*)"SMCoreFM"
#define	SC_TAG_MP			(Char*)"SMCoreMP"
#define	SC_TAG_RM			(Char*)"SMCoreRM"
#define	SC_TAG_RC			(Char*)"SMCoreRC"
#define	SC_TAG_RG			(Char*)"SMCoreRG"
#define	SC_TAG_RT			(Char*)"SMCoreRT"
#define	SC_TAG_DH			(Char*)"SMCoreDH"
#define	SC_TAG_DHC			(Char*)"SMCoreDHC"
#define	SC_TAG_LC			(Char*)"SMCoreLC"
#define	SC_TAG_CC			(Char*)"SMCC"
#define	SC_TAG_PM			(Char*)"SMPM"
#define	SC_TAG_PU			(Char*)"SMPU"
#define	SC_TAG_PT			(Char*)"SMPT"
#define	SC_TAG_PDAL			(Char*)"SMPDAL"
#define SC_TAG_SDM			(Char*)"SMSDM"
#define SC_TAG_SDD			(Char*)"SMSDD"
#define SC_TAG_SDU			(Char*)"SMSDU"
#define SC_TAG_SDT			(Char*)"SMSDT"
#define SC_TAG_SDP			(Char*)"SMSDP"
#define	SC_TAG_DAL			(Char*)"SMCoreDAL"
#define	SC_TAG_PAL			(Char*)"SMCorePAL"
#define	SC_TAG_TR			(Char*)"SMCoreTR"
#define	SC_TAG_TRT			(Char*)"SMCoreTRT"


#define	_STR(x)				#x
#define	_STR2(x)			_STR(x)
#define	__SLINE__			_STR2(__LINE__)
#define	HERE				"FILE : " __FILE__ "(" __SLINE__ ")\n"

#define _CHECK_ARRAY(array, line)	sizeof(enum { argument_is_not_an_array_##line = ((array)==(const volatile void*)&(array)) })
#define _CHECK_ARRAY2(array, line)	_CHECK_ARRAY(array, line)
#ifdef __SMS_APPLE__
#define COUNTOF(array)				(sizeof(array)/sizeof(*(array)))
#else
#define COUNTOF(array)				(_CHECK_ARRAY2(array, __LINE__), sizeof(array)/sizeof*(array))
#endif /* __SMS_APPLE__ */

#define SC_LOG_START		(Char*)"[START] %s()\n", __FUNCTION__
#define SC_LOG_END			(Char*)"[END]   %s()\n", __FUNCTION__

#define USE_IT(x)			(void)(x)	// 未使用引数に対するコンパイル時Warning警告対策

#define SC_MALLOC(size)		(SC_MEM_Alloc(size, e_MEM_TYPE_DYNAMIC))
#define SC_FREE(ptr)		({SC_MEM_Free(ptr, e_MEM_TYPE_DYNAMIC); ptr = NULL;})

#define ISLITTLENDIAN		(IsLittleEndian())
// リトルエンディアンに変換する
#define CONVERT_ENDIAN_INT16(val)	(((val<<8) & 0xff00) | ((val>>8) & 0x00ff))
#define CONVERT_ENDIAN_INT32(val)	(((val<<24) & 0xff000000) | ((val<<8) & 0x00ff0000) | ((val>>8) & 0x0000ff00) | ((val>>24) & 0x000000ff))
#define CONVERT_LITTLE_ENDIAN_INT32(val)	((ISLITTLENDIAN) ? (val) : (CONVERT_ENDIAN_INT32(val)))
#define CONVERT_LITTLE_ENDIAN_INT16(val)	((ISLITTLENDIAN) ? (val) : (CONVERT_ENDIAN_INT16(val)))

// ビッグエンディアンをエンディアンに合わせて変換する
#define CONVERT_BIG_ENDIAN_INT32(val)		((ISLITTLENDIAN) ? (CONVERT_ENDIAN_INT32(val)) : (val))
#define CONVERT_BIG_ENDIAN_INT16(val)		((ISLITTLENDIAN) ? (CONVERT_ENDIAN_INT16(val)) : (val))

#endif	// #ifdef SMCORE_CMN_H
