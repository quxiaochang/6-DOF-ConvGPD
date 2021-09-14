/*
 * Copyright (C) 2017 by
 * Shenzhen Inovance Technology Co., Ltd.
 */
#ifndef IMC100API_H
#define IMC100API_H

#ifdef IMC100API_EXPORTS
#define IMC100API __declspec(dllexport)
#else
#define IMC100API __declspec(dllimport)
#endif

#define FUNTYPE __stdcall

typedef struct 
{
    double pos[6];                    /// position J1/X, J2/Y, J3/Z, J4/A, J5/B, J6/C
    int armType[4];                   /// arm parameters
    int coord;                        /// coordinate type
    int toolNo;                       /// tool coordinate number
    int userNo;                       /// user coordinate number
}ROBOT_POS;

typedef struct 
{
    int IONo;                         /// DO number
    int IOVa;                         /// output value
    int Kind;                         /// set type
    double Value;                     /// set value
}MOV_IO;


#ifdef __cplusplus 
extern "C" {
#endif

IMC100API int FUNTYPE IMC100_Init_ETH(unsigned int ipAddr, unsigned short ipPort, int timeOut, int comId);
IMC100API int FUNTYPE IMC100_Exit_ETH(int comId);

IMC100API int FUNTYPE IMC100_EmergStop(int cmd, int comId);
IMC100API int FUNTYPE IMC100_MotorEnable(int cmd, int comId);
IMC100API int FUNTYPE IMC100_ResetErr(int comId);
IMC100API int FUNTYPE IMC100_Set_Mode(int mode, int comId);
IMC100API int FUNTYPE IMC100_Set_CurPrgPath(char prgPath[128], int comId);
IMC100API int FUNTYPE IMC100_PrgCtrl(int cmd, int comId);
IMC100API int FUNTYPE IMC100_Set_Vel(int val, int comId);
IMC100API int FUNTYPE IMC100_DsMode(int cmd, int comId);
IMC100API int FUNTYPE IMC100_Set_DO(int num, int status, int comId);
IMC100API int FUNTYPE IMC100_Set_DOGroup(int num, int status, int comId);
IMC100API int FUNTYPE IMC100_Set_DA(int num, float val, int comId);
IMC100API int FUNTYPE IMC100_InchMode(int cmd, int comId);
IMC100API int FUNTYPE IMC100_Set_InchStep(int val, int comId);

IMC100API int FUNTYPE IMC100_Jog(int mode, int axis, int cmd, int comId);
IMC100API int FUNTYPE IMC100_Home(int num, int comId);
IMC100API int FUNTYPE IMC100_MovJ(int posNum, int vel, int zone, int comId);
IMC100API int FUNTYPE IMC100_MovL(int posNum, int vel, int zone, int comId);
IMC100API int FUNTYPE IMC100_MovJ2(ROBOT_POS pos, int vel, int zone, int comId);
IMC100API int FUNTYPE IMC100_MovL2(ROBOT_POS pos, int vel, int zone, int comId);
IMC100API int FUNTYPE IMC100_MovJ_IO(int posNum, int vel, int zone, MOV_IO *movIo, int ioNum, int comId);
IMC100API int FUNTYPE IMC100_MovL_IO(int posNum, int vel, int zone, MOV_IO *movIo, int ioNum, int comId);
IMC100API int FUNTYPE IMC100_MovJ2_IO(ROBOT_POS pos, int vel, int zone, MOV_IO *movIo, int ioNum, int comId);
IMC100API int FUNTYPE IMC100_MovL2_IO(ROBOT_POS pos, int vel, int zone, MOV_IO *movIo, int ioNum, int comId);

IMC100API int FUNTYPE IMC100_Get_PosHere(ROBOT_POS *pos, int comId);
IMC100API int FUNTYPE IMC100_Get_PosHereJ(ROBOT_POS *pos, int comId);
IMC100API int FUNTYPE IMC100_Get_PosHereC(ROBOT_POS *pos, int comId);
IMC100API int FUNTYPE IMC100_Get_PosCnvt(ROBOT_POS *posSrc, ROBOT_POS *posDst, int comId);
IMC100API int FUNTYPE IMC100_Get_SysErr(int *error, int comId);
IMC100API int FUNTYPE IMC100_Get_CurPrgPath(char prgPath[128], int comId);
IMC100API int FUNTYPE IMC100_Get_PrgSts(int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_StartLine(int *line, int comId);
IMC100API int FUNTYPE IMC100_Get_CurPrgLine(int *line, int comId);
IMC100API int FUNTYPE IMC100_Get_InitSts(int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_Coord(int *type, int comId);
IMC100API int FUNTYPE IMC100_Get_Vel(int *val, int comId);
IMC100API int FUNTYPE IMC100_Get_Mode(int *mode, int comId);
IMC100API int FUNTYPE IMC100_Get_DsMode(int *val, int comId);
IMC100API int FUNTYPE IMC100_Get_InchMode(int *val, int comId);
IMC100API int FUNTYPE IMC100_Get_EStopSts(int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_MotorSts(int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_MotionSts(int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_SysMode(int *mode, int comId);
IMC100API int FUNTYPE IMC100_Get_PrgRunTime(unsigned int *second, int comId);
IMC100API int FUNTYPE IMC100_Get_CurCmdNum(unsigned int *num, int comId);
IMC100API int FUNTYPE IMC100_Get_CurCmdSts(int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_CmdSts(int num, int *sts, int comId);

IMC100API int FUNTYPE IMC100_Get_DINum(int *num, int comId);
IMC100API int FUNTYPE IMC100_Get_DONum(int *num, int comId);
IMC100API int FUNTYPE IMC100_Get_ADNum(int *num, int comId);
IMC100API int FUNTYPE IMC100_Get_DANum(int *num, int comId);
IMC100API int FUNTYPE IMC100_Get_DI(int num, int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_DIGroup(int num, int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_AD(int num, float *val, int comId);
IMC100API int FUNTYPE IMC100_Get_DOCfg(int num, int *val, int comId);
IMC100API int FUNTYPE IMC100_Get_DOGroupCfg(int num, int *val, int comId);
IMC100API int FUNTYPE IMC100_Get_DO(int num, int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_DOGroup(int num, int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_DACfg(int num, int *val, int comId);
IMC100API int FUNTYPE IMC100_Get_DA(int num, float *val, int comId);
IMC100API int FUNTYPE IMC100_Get_DevSts(int sts[6], int comId);
IMC100API int FUNTYPE IMC100_Get_FwVersion(char ver[32], int comId);
IMC100API int FUNTYPE IMC100_Get_SysTime(char time[16], int comId);
IMC100API int FUNTYPE IMC100_Get_RobotType(char type[128], int comId);

IMC100API int FUNTYPE IMC100_Get_ServoSts(int sts[8], int comId);
IMC100API int FUNTYPE IMC100_Get_ServoErr(int num, int *error, int comId);

IMC100API int FUNTYPE IMC100_Get_StrPara(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_StrPara(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_StrParaComp(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_StrParaComp(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_RdctRatio(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_RdctRatio(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_CpParaM(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_CpParaM(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_CpParaS(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_CpParaS(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_HomePos(int num, double pos[6], int comId);
IMC100API int FUNTYPE IMC100_Set_HomePos(int num, double pos[6], int comId);
IMC100API int FUNTYPE IMC100_Get_ZeroPos(int pluse[6], int comId);
IMC100API int FUNTYPE IMC100_Set_ZeroPos(int pluse[6], int comId);
IMC100API int FUNTYPE IMC100_Get_InchStep(int *val, int comId);
IMC100API int FUNTYPE IMC100_Get_StepMotionJ(float *para, int comId);
IMC100API int FUNTYPE IMC100_Set_StepMotionJ(float para, int comId);
IMC100API int FUNTYPE IMC100_Get_StepMotionL(float *para, int comId);
IMC100API int FUNTYPE IMC100_Set_StepMotionL(float para, int comId);
IMC100API int FUNTYPE IMC100_Get_TeachVelLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_TeachVelLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_TeachVelLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Set_TeachVelLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Get_TeachAccLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_TeachAccLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_TeachAccLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Set_TeachAccLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Get_RunVelLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_RunVelLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_RunVelLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Set_RunVelLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Get_RunAccLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_RunAccLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_RunAccLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Set_RunAccLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Get_StopDecLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Set_StopDecLimJ(float para[6], int comId);
IMC100API int FUNTYPE IMC100_Get_StopDecLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Set_StopDecLimL(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Get_ZonePara(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Set_ZonePara(float para[2], int comId);
IMC100API int FUNTYPE IMC100_Get_CInterpMode(int *type, int comId);
IMC100API int FUNTYPE IMC100_Set_CInterpMode(int type, int comId);
IMC100API int FUNTYPE IMC100_Get_AxisNLim(int axis, float *para, int comId);
IMC100API int FUNTYPE IMC100_Set_AxisNLim(int axis, float para, int comId);
IMC100API int FUNTYPE IMC100_Get_AxisPLim(int axis, float *para, int comId);
IMC100API int FUNTYPE IMC100_Set_AxisPLim(int axis, float para, int comId);
IMC100API int FUNTYPE IMC100_Get_ToolC(int num, double pos[6], int comId);
IMC100API int FUNTYPE IMC100_Set_ToolC(int num, double pos[6], int comId);
IMC100API int FUNTYPE IMC100_Get_UserC(int num, double pos[6], int comId);
IMC100API int FUNTYPE IMC100_Set_UserC(int num, double pos[6], int comId);
IMC100API int FUNTYPE IMC100_Get_ToolCNum(int *num, int comId);
IMC100API int FUNTYPE IMC100_Set_ToolCNum(int num, int comId);
IMC100API int FUNTYPE IMC100_Get_UserCNum(int *num, int comId);
IMC100API int FUNTYPE IMC100_Set_UserCNum(int num, int comId);
IMC100API int FUNTYPE IMC100_Set_Coord(int type, int comId);
IMC100API int FUNTYPE IMC100_Get_Interf(int num, double pos[6], int comId);
IMC100API int FUNTYPE IMC100_Set_Interf(int num, double pos[6], int comId);
IMC100API int FUNTYPE IMC100_Get_CurInterf(int *num, int comId);
IMC100API int FUNTYPE IMC100_Set_CurInterf(int num, int comId);
IMC100API int FUNTYPE IMC100_SavePara(int comId);
IMC100API int FUNTYPE IMC100_RecoverPara(int comId);

IMC100API int FUNTYPE IMC100_Get_P(int pNum, ROBOT_POS *pos, int comId);
IMC100API int FUNTYPE IMC100_Set_P(int pNum, ROBOT_POS pos, int comId);
IMC100API int FUNTYPE IMC100_Set_PHere(int pNum, int comId);
IMC100API int FUNTYPE IMC100_Get_PR(int prNum, ROBOT_POS *pos, int comId);
IMC100API int FUNTYPE IMC100_Set_PR(int prNum, ROBOT_POS pos, int comId);
IMC100API int FUNTYPE IMC100_WriteFile_PR(int comId);
IMC100API int FUNTYPE IMC100_Get_LPR(int prNum, ROBOT_POS *pos, int comId);
IMC100API int FUNTYPE IMC100_Set_LPR(int prNum, ROBOT_POS pos, int comId);
IMC100API int FUNTYPE IMC100_Get_B(int num, int *val, int comId);
IMC100API int FUNTYPE IMC100_Set_B(int num, int val, int comId);
IMC100API int FUNTYPE IMC100_Get_R(int num, int *val, int comId);
IMC100API int FUNTYPE IMC100_Set_R(int num, int val, int comId);
IMC100API int FUNTYPE IMC100_Get_D(int num, double *val, int comId);
IMC100API int FUNTYPE IMC100_Set_D(int num, double val, int comId);
IMC100API int FUNTYPE IMC100_Get_LB(int num, int *val, int comId);
IMC100API int FUNTYPE IMC100_Set_LB(int num, int val, int comId);
IMC100API int FUNTYPE IMC100_Get_LR(int num, int *val, int comId);
IMC100API int FUNTYPE IMC100_Set_LR(int num, int val, int comId);
IMC100API int FUNTYPE IMC100_Get_LD(int num, double *val, int comId);
IMC100API int FUNTYPE IMC100_Set_LD(int num, double val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarUchar(int address, unsigned char *val, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarUchar(int address, unsigned char val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarChar(int address, char *val, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarChar(int address, char val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarUshort(int address, unsigned short *val, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarUshort(int address, unsigned short val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarShort(int address, short *val, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarShort(int address, short val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarUint(int address, unsigned int *val, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarUint(int address, unsigned int val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarInt(int address, int *val, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarInt(int address, int val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarFloat(int address, float *val, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarFloat(int address, float val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarDouble(int address, double *val, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarDouble(int address, double val, int comId);
IMC100API int FUNTYPE IMC100_Get_CommonVarP(int address, ROBOT_POS *pos, int comId);
IMC100API int FUNTYPE IMC100_Set_CommonVarP(int address, ROBOT_POS pos, int comId);
IMC100API int FUNTYPE IMC100_Get_ModbusCoil(int address, int sum, int *val, int comId);
IMC100API int FUNTYPE IMC100_Set_ModbusCoil(int address, int sum, int val, int comId);
IMC100API int FUNTYPE IMC100_Get_ModbusRegUshort(int address, int sum, unsigned short val[], int comId);
IMC100API int FUNTYPE IMC100_Set_ModbusRegUshort(int address, int sum, unsigned short val[], int comId);
IMC100API int FUNTYPE IMC100_Get_ModbusRegFloat(int address, int sum, float val[], int comId);
IMC100API int FUNTYPE IMC100_Set_ModbusRegFloat(int address, int sum, float val[], int comId);
IMC100API int FUNTYPE IMC100_Get_PlcVarByte(int num, unsigned char *val, int comId);
IMC100API int FUNTYPE IMC100_Get_PlcVarInt(int num, short *val, int comId);
IMC100API int FUNTYPE IMC100_Get_PlcVarDInt(int num, int *val, int comId);
IMC100API int FUNTYPE IMC100_Get_PlcVarLReal(int num, double *val, int comId);
IMC100API int FUNTYPE IMC100_Get_UserAlarm(int num, char alarm[40], int comId);
IMC100API int FUNTYPE IMC100_Set_UserAlarm(int num, char alarm[40], int comId);
IMC100API int FUNTYPE IMC100_Get_Print(char val[128], int comId);

IMC100API int FUNTYPE IMC100_Get_InCfg(int func, int *diNum, int comId);
IMC100API int FUNTYPE IMC100_Set_InCfg(int func, int diNum, int comId);
IMC100API int FUNTYPE IMC100_Get_OutCfg(int func, int *doNum, int comId);
IMC100API int FUNTYPE IMC100_Set_OutCfg(int func, int doNum, int comId);

IMC100API int FUNTYPE IMC100_CurCtrlDev(int *dev, int comId);
IMC100API int FUNTYPE IMC100_CurPermit(int *owner, unsigned int *ipAddr, unsigned short *ipPort, int comId);
IMC100API int FUNTYPE IMC100_AcqPermit(int cmd, int comId);
IMC100API int FUNTYPE IMC100_RemovePermit(int comId);
IMC100API int FUNTYPE IMC100_CurUserType(int *type, int comId);
IMC100API int FUNTYPE IMC100_UserLogin(int type, char password[8], int comId);
IMC100API int FUNTYPE IMC100_UserLogout(int comId);

IMC100API int FUNTYPE IMC100_Set_SysTime(char time[16], int comId);

IMC100API int FUNTYPE IMC100_LatchEnable(int cmd, int comId);
IMC100API int FUNTYPE IMC100_Get_LatchSts(int *sts, int comId);
IMC100API int FUNTYPE IMC100_Get_LatchSum(int *sum, int comId);
IMC100API int FUNTYPE IMC100_Get_LatchPos(int index, int *sts, double *pos, int comId);
IMC100API int FUNTYPE IMC100_Clr_LatchPos(int comId);

#ifdef __cplusplus 
}
#endif

#endif
