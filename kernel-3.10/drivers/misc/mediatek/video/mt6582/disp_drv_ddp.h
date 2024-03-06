
// Only for DDP driver.
#ifndef __DISP_DRV_DDP_H__
#define __DISP_DRV_DDP_H__
#include <mach/mt_typedefs.h>

typedef int (*DISP_EXTRA_CHECKUPDATE_PTR)(int);
typedef int (*DISP_EXTRA_CONFIG_PTR)(int);
int DISP_RegisterExTriggerSource(DISP_EXTRA_CHECKUPDATE_PTR pCheckUpdateFunc , DISP_EXTRA_CONFIG_PTR pConfFunc);
void DISP_UnRegisterExTriggerSource(int u4ID);
void GetUpdateMutex(void);
void ReleaseUpdateMutex(void);
BOOL DISP_IsVideoMode(void);
unsigned long DISP_GetLCMIndex(void);

#ifdef CONFIG_WT_GAMMA_PQ_WITH_MULTI_LCM
BOOL DISP_GetGammaParam_LCD(DISPLAY_GAMMA_T *gamma_parameter);
BOOL DISP_GetPQparam_LCD(DISP_PQ_PARAM *pq_parameter);
#endif

#endif