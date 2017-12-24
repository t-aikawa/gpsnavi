#include <stdbool.h>

typedef 	bool (*TAR_CANCEL_FNC)(void);				// 中断確認用コールバック関数
typedef 	void (*TAR_PROGRESS_FNC)(unsigned int num);	// 進捗通知用コールバック関数

//-----------------------------------
// API定義
//-----------------------------------
int tar (const char *arcpath, const char *dstpath);
int tar2 (const char *arcpath, const char *dstpath, TAR_CANCEL_FNC cancel, TAR_PROGRESS_FNC progress);
