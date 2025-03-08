/*
*************************************************************************************************
* @copy  COPYRIGHT 2018 WHST
* NOTICE
* This software is the property of WHST Technologies. Any information contained in this
* doc should not be reproduced, or used, or disclosed without the written authorization from
* WHST Technologies.
*************************************************************************************************
* @File :
* @Version :
* @Change log :
*
*
*
* @Function :
*
*
*
*
*
*
* @Brief :改文件用于描述DSP至ARM 目标数据传送的共享内存数据
*
*
*
*
* @Author :18054
* @Creat date :2020/5/11 9:42
*************************************************************************************************
*/
#ifndef _TARGETPACK_ACC_H_
#define _TARGETPACK_ACC_H_


/* Standard Include Files. */

#include <stdint.h>

#include <src/parse_data/plat_cfg.h>
#include <common/common_macro_def.h>
#include <common/alg_struct.h>

/* User Include Files */



/*
 trace target macro
 */
#define MAX_TracePack_NUM 64UL              //lzm the max number of trace target for packing lzm
/*
 cluster target macro
 */
#define MAX_ClusterPack_NUM 40UL              //the max number of trace target for packing


/*
 CFAR target macro
 */
#define MAX_CfarPack_NUM 800UL             //the max number of CFAR target for packing

#define M_SECTIONSNUM_ALL 128UL

#pragma pack(push,1)



/**
* @brief AD&CFAR&TRACE OUTPUT
*
*/
typedef struct
{
	int16_t rangeIdx;
	int16_t dopplerIdx;
	int16_t peakVal;
	int16_t azimuthAngle;
	int16_t elevationAngle;
	int16_t speed;
	int16_t SNR;
	int16_t V_order;
	int16_t ambiangle2;
	int16_t ambiangle3;
} EthCfarMsg_Stru1_ACC;

typedef struct
{
	EthCfarMsg_Stru1_ACC CFAR_Array[MAX_CfarPack_NUM];

}EthCfarPack_Str_ACC;
#define     NetADC_CfarPackSize         sizeof(EthCfarPack_Str)

/*END*/


/**END*/

/**
* @brief 车身信号输出监控基础数据结构
*
*/
typedef struct
{
	uint16_t subFrameHeader;              //子协议帧头
	uint8_t subVersion;
	uint16_t subDataLength;               //子协议数据长度
	uint8_t ego_speed_ic;                 //表显车速
	uint8_t ego_SpeedValid;               //车速有效
	float ego_speed;                    //abs车速
	uint8_t ego_accswitch;                  //ACC总开关
	float ego_brake_press;              //主缸压力
	uint8_t ego_brake_press_Valid;          //制动主缸压力信号有效
	float ego_yawrate;                  //横摆角速度
	uint8_t ego_YawRateValid;               //偏航率(横摆角速度)有效位
	uint8_t ego_wiper;                      //雨刷信号
	float ego_steer;                    //方向盘转角
	float ego_steer_speed;              //方向盘转角速率
	uint8_t ego_SteerValid;                 //方向盘转角失效信号
	float ego_throttle;                 //油门踏板
	uint8_t ego_brakepadal;                 //制动踏板
	uint8_t ego_indflag;                    //转向灯信号
	uint8_t ego_error_fcw_flag;             //FCW故障标志
	uint8_t ego_error_aeb_flag;             //AEB故障标志
	uint8_t ego_error_acc_flag;             //ACC故障标志
	float ego_ay;                       //车辆横向加速度
	float ego_ax;                       //纵向加速度
	uint8_t ego_cover_flag;                 //雷达遮挡标志位
	uint8_t ego_hwt_dem_flag;               //跟车时距阈值,0/1/2/3/4,4-最大时距
	uint8_t ego_standstill;               //本车停止状态
	float ego_v_desired;                //巡航设定车速
	float ego_torque;                   //发动机实际扭矩
	float ego_torque_max;               //当前车辆允许最大扭矩
	float ego_torque_min;               //当前车辆允许最小扭矩
	uint8_t ego_override_flag;              //override标志位
	float ego_ratio;                    //转向传动比
	uint8_t ego_gear;                       //挡位  （1-7）
	uint8_t ego_gearvalid;                  //挡位有效位
	uint8_t ego_ShiftPostionValid;          //换挡器位置有效
	uint8_t ego_shift;                      //换挡器位置
	uint8_t ego_fcw_sensitivity_flag;       //FCW灵敏度//FCW开关复用改信号
	uint8_t ego_aebswitch;                  //AEB总开关
	uint8_t ego_AEBAvailable;               //AEB(自动紧急制动)可用
	uint8_t ego_AEBdecActive;               //AEB(自动紧急制动)激活
	uint8_t ego_HDC_STATUS;                 //HDC工作状态
	uint8_t ego_ABSFailStatus;              //ABS故障
	uint8_t ego_OnlyABSActive;              //ABS激活
	uint8_t ego_TCSFailStatus;              //TCS/ESP故障
	uint8_t ego_TCSActive;                  //TCS激活
	uint8_t ego_EPB_Status;                 //EPB状态
	uint8_t	ego_EPB_SwitchPosition;	        //EPB开关位置
	uint8_t ego_DriverDoorStatus;           //左前门状态
	uint8_t ego_RightRearDoorStatus;        //右后门状态
	uint8_t ego_PassengerDoorStatus;        //右前门状态
	uint8_t ego_LeftRearDoorStatus;         //左后门状态
	uint8_t ego_HoodStatus;                 //发动机舱盖状态
	uint8_t ego_DriverBuckleSwitchStatus;   //安全带状态
	float ego_IndicatedDriverReqTorq;   //驾驶员期望燃烧扭矩
	float ego_FrictionalTorq;           //摩擦扭矩
	uint8_t ego_TorqFailure;                //发动机扭矩失效
	uint8_t ego_EngineFuelCutOff;           //断油标志
	float ego_TorqueConstant;           //扭矩基准值
	uint8_t ego_EngineStatus;               //发动机运行状态(powermode)
	uint8_t ego_EngineLimpHome;             //跛行状态
	uint8_t ego_EngineSpeedError;           //发动机转速故障
	float ego_EngineSpeed;              //发动机转速
	uint8_t ego_AccpedelError;              //加速踏板故障
	uint8_t	ego_AccReqPossible;	            //发动机当前可以响应ACC的扭矩请求
	uint8_t	ego_EMS_QECACC;	                //EMS检测到ACC发出的信息有错误
	uint8_t	ego_CruiseSwitchSET;	        //ACC的RESUME开关状态
	uint8_t	ego_CruiseSwitchResume;	        //ACC的SET开关状态
										//int	ego_CruiseDistance;	            //ACC巡航距离开关状态
	uint8_t	ego_ESP_QDCACC;	                //ESP检测到ACC发出的信息有错误（例如checksum, alivecouner, timeout…)
	uint8_t	ego_IP_QDashACCFail;	        //仪表检测到ACC发出的信息有误（例如timeout）
	uint8_t	ego_DriveMode;	                //驾驶模式
	uint8_t	ego_Wheel_Speed_FR_Valid;	    //右前轮速数据有效
	uint8_t	ego_Wheel_Speed_FR_Direction;	//右前轮方向信号
	float ego_Wheel_Speed_FR;	        //右前轮速信号
	uint8_t	ego_Wheel_Speed_FL_Valid;	    //左前轮速数据有效
	uint8_t	ego_Wheel_Speed_FL_Direction;	//左前轮方向信号
	float ego_Wheel_Speed_FL;	        //左前轮速信号
	uint8_t	ego_Wheel_Speed_RR_Direction;	//右后轮方向信号
	uint8_t	ego_Wheel_Speed_RR_Valid;	    //右后轮速数据有效
	float ego_Wheel_Speed_RR;	        //右后轮速信号
	uint8_t	ego_Wheel_Speed_RL_Valid;	    //左后轮速数据有效
	uint8_t	ego_Wheel_Speed_RL_Direction;	//左后轮方向信号
	float ego_Wheel_Speed_RL;	        //左后轮速信号
	uint8_t	ego_HU_ACCObjEnable;	        //车辆识别声音提示使能开关
	uint8_t	ego_HU_CruiseControlSet;	    //巡航状态设置
	uint8_t	ego_BrakeDiscTempOverHeat;	    //制动盘过热
	uint8_t	ego_SRS_CrashOutputStatus;	    //碰撞输出信号
	uint8_t	ego_SRS_CrashOutputStatus_flag;	//碰撞校验信号
	uint8_t	ego_ESP_ESPFunctionStatus;	    //ESP功能开启状态
	uint8_t	ego_ESP_PrefillAvailable;	    //预制动有效
	uint8_t	ego_ESP_ABAactive;	            //ABA(自动制动辅助)激活
	uint8_t	ego_ESP_ABAavailable;	        //ABA(自动制动辅助)可用
	uint8_t	ego_ESP_AWBavailable;	        //AWB(自动警告制动)有效
	uint8_t	ego_ESP_CDD_Active;	            //CDD(减速度控制)激活
	uint8_t	ego_ESP_CDD_Available;	        //CDD(减速度控制)可用
	uint8_t	ego_ESP_VDCActive;	            //VDC激活
	uint8_t	ego_ESP_BrakeForce;	            //制动系统压力
	uint8_t ego_AEBErrorbyESP;              //雷达故障来源：ESP
	uint8_t ego_AEBErrorbySAS;              //雷达故障来源：SAS
	uint8_t ego_AEBErrorbySRS;              //雷达故障来源：SRS
	uint8_t ego_AEBErrorbyEMS;              //雷达故障来源：EMS
	uint8_t ego_AEBErrorbyTCU;              //雷达故障来源：TCU
	uint8_t ego_AEBErrorbyOther;            //雷达故障来源：其他
	uint8_t ego_ACCError_ByEMS;             //故障来源：EMS
	uint8_t ego_ACCError_ByBCM;             //故障来源：BCM
	uint8_t ego_ACCError_ByTCU;             //故障来源：TCU
	uint8_t ego_ACCError_ByIP;              //故障来源：IP
	uint8_t ego_ACCError_BySRS;             //故障来源：SRS
	uint8_t ego_ACCError_ByMFS;             //故障来源：MFS
	uint8_t ego_ACCError_ByEPB;             //故障来源：EPB
	uint8_t ego_HU_NavStatus;               //导航状态"0x0:Normal;0x1:Error"
	//uint8_t ego_CSLAEnable;                 //自动调整巡航车速使能请求 0x0:OFF; 0x1:All Speed - Limit; 0x2:Road Camera; 0x3:invalid;
	uint8_t ego_EEyeSpeedLimit;           //电子眼限速值0x0:Unlimited;0x1:SL 5;  0x2:SL 10;  …  0x20:SL 160;  0xFE:Unknown;0xFF:Invalid
	uint8_t ego_NavSpeedLimitStatus;        //限速状态
	uint8_t ego_NavCurrentRoadType;         //道路类型 2.搭载ADAS:0x0:inactive;0x1:未知;0x2:其他;0x3:城市快速路;//0x4:国道 / 省道;0x5:高速路;0x6:Reserved;0x7:Reserved;
	uint8_t ego_HU_RampInfo;                //匝道信息,0x0:Unknown;0x1:有匝道，但本车不出高速; 0x2:本车通过匝道进入高速; 0x3:本车通过匝道出高速
	float ego_HU_RampDistance;          //当前位置与匝道距离 (0x0~0x3C)*10:0~600m 0x3D:Out of Range; 0x3E:reserved; 0x3F:unknown;
	uint8_t ego_HU_NavGuiganceStatus;       //导航路径引导状态  0x0:没有导航引导路径; 0x1:有导航引导路径; 0x2:Reserved;0x3:Reserved
	uint8_t ego_HU_TollInfo;                //收费站信息   "0x0:路径无收费站0x1:路径有收费站0x2 : 预留0x3 : invalid"
	float ego_HU_TollDis;               //当前位置与收费站距离  "(0x0~0x3C)*10:0~600m;0x3D : Out of Range;0x3E:reserved;0x3F:unknown; "
	uint8_t ego_HU_EnterTunnelInfo;         //隧道信息    "0x0:unknown;0x1:隧道入口; 0x2:隧道中;0x3:隧道出口; "
	float ego_HU_TunnelDistance;        //当前位置与隧道距离 "(0x0~0x3C)*10:0~600m;0x3D:Out of Range;0x3E:reserved;0x3F:unknown; "
	float ego_HU_TunnelLength;          //隧道长度  "(0x0~0x3C)*10:0~600m;0x3D:Out of Range;0x3E:reserved;0x3F:unknown; "
	uint8_t ego_HU_CSL_Error;               //限速信息错误  "0x0:Normal0x1:Error"
	uint8_t ego_ECM_ACC_CancelSw;           //ACC功能取消按键。active→standby
	//uint8_t ego_WindShieldWiperActSts;      //Window shield Wiper active status  0x0:No active  0x1:Active
	uint8_t ego_HazardLightSwSts;           //双闪灯开关状态
	uint8_t reserved[50];
	uint8_t subCheckSum;                  //子协议校验和
} ECU_Moni_Str_ACC;
/*end */




typedef struct{
    uint16_t lost_num_all : 12;
    uint16_t STBC_TypeS : 1;
    uint16_t STBC_TypeT : 1;
    uint16_t STBC_TypeB : 1;
    uint16_t STBC_TypeC : 1;
}LostNumAll_STBC_Type_Str_ACC;

typedef struct {
	int32_t		box_length : 12;            // the target height        目标高度
	int32_t		box_width : 10;             // the target height        目标高度
	int32_t		box_height : 10;            // the target height        目标高度
}Box_Info_Str;

typedef struct {
	uint8_t		score : 4;                  // 航迹得分
	uint8_t		LostNum : 4;                // the lost frame number    丢失帧数
}Score_LostNum_Str;

typedef struct {
	uint8_t		moveState : 4;              // move state
	uint8_t		TarType : 4;				// target type define
}moveState_Type_Str;
typedef struct
{
	uint8_t				Index;                      // the trace index number
	int16_t				Ry;                         // the distance for X       纵向距离-lzm
	int16_t				Rx;							// the distance for Y       横向距离-lzm
	int16_t				height;                     // the target height        离地高度
	Box_Info_Str		BoxInfo;
	int16_t				cross_speed;                // 横向速度
	int16_t				longitu_speed;              // velocity of the target
	int16_t				Aceler_Y;                   // 横向加速度
	int16_t				Aceler_X;                   // 纵向加速度
	uint16_t			dis_clu;					// original distance 原始距离,0.01
	int16_t				ang_clu;                    //  原始角度,0.01度
	int16_t				evangle_clu;                // 俯仰角,0.01度
	int16_t				speed_clu;                  // 原始速度（径向速度）,0.01m/s
	uint16_t			peakVal;					// peak value
	uint8_t				SNR;                        // the signal to noise ratio
	int16_t				RCS;                        // RCS,0.01dbsm
	uint16_t			lifeTime;					// 跟踪周期
	Score_LostNum_Str	score_lostNum;				//score,LostNum
	uint8_t				Multipath;                  // 多径概率
	moveState_Type_Str	moveState_Type;				//moveState,TarType
	uint8_t				recognization_target;       // 目标识别率
	uint8_t				obstacle;                   // 障碍物概率
	uint8_t				ExistProb;                  // 置信度
	int16_t				Yaw;                        // 航向角
	uint8_t				measure_point;              // target type define
	int16_t				Y_distance_std;             // 横向距离标准差
	int16_t				Y_speed_std;                // 横向速度标准差
	int16_t				Y_Acc_std;                  // 横向加速度标准差
	int16_t				X_distance_std;             // 纵向距离标准差
	int16_t				X_speed_std;                // 纵向速度标准差
	int16_t				X_Acc_std;                  // 纵向加速度标准差
	uint16_t			lost_num_all;				// 总丢失帧数
	uint8_t				clusterUid;                 // 聚类编号
	uint8_t				uid_start;                  // 起始航迹编号
	int16_t				vyAbs;						// 横向绝对速度
	int16_t				vxAbs;						// 纵向绝对速度
} Target_TRACE_5s;



/**
 * @brief 航迹目标点基础数据结构
 * 
 */
typedef struct
{
	uint16_t			subFrameHeader;					// 子协议帧头
	uint8_t				subVersion;                     // 子协议版本
	uint16_t			subDataLength;					// 子协议数据长度
	uint16_t			periodBitNum_Trace;				// 航迹循环字节数
	uint16_t			maxVldTraceNum;					// 最大有效航迹数
	Target_TRACE_5s		trace[MAX_TracePack_NUM];		//Lateral prediction position
	uint8_t				subCheckSum;					//子协议校验和
} Target_TRACE_Str_ACC;

typedef struct
{
	uint8_t		clusterId : 6;					// 聚类编号
	uint8_t		IsRectOrDbscan : 2;				// 聚类类型，默认值为0、1-框聚类、2-dbscan聚类、3-两者合并聚类
} clusterId_Type_Str;

typedef struct
{
	uint16_t	height : 13;					// 目标离地高度
	uint16_t	cfar_flag : 3;					// cfar标志
} Height_Flag_Str;

typedef struct
{
	clusterId_Type_Str	clusterId_Type;					//clusterId ,IsRectOrDbscan
	uint8_t				traceId;						// 航迹编号
	int16_t				Ry;								// 横向距离
	uint16_t			Rx;								// 纵向距离
	uint16_t			range;							// 径向距离
	int16_t				Rz;								// 高度距离，离地高度
	int16_t				speed;							// 径向速度
	int16_t				peakVal;						// 幅度
	int16_t				azimuthAngle;					// 方位角
	int16_t				elevationAngle;					// 俯仰角
	uint8_t				SNR;							// SNR
	uint16_t			rangeIdx;						// 距离通道
	uint8_t				element_num;					// 类中元素个数
	int16_t				cornersInfo[8];					// 目标框信息
	Height_Flag_Str		Height_CfarFlag;				// height,cfar_flag
	int16_t				reserve;						// 预留
} Target_CLUSTER;
/**
* @brief 聚类目标点基础数据结构
*
*/
typedef struct
{
	uint16_t			subFrameHeader;                             // 子协议帧头（聚类信息）
	uint8_t				subVersion;                                 // 子协议版本号
	uint16_t			subDataLength;                              // 子协议数据长度 lzm
	uint16_t			periodBitNum_Cluster;						//聚类循环字节数
	uint16_t			maxVldClusterNum;							//最大有效聚类数
	Target_CLUSTER		cluster[MAX_ClusterPack_NUM];
	uint8_t				subCheckSum;								//子协议校验和
} Target_CLUSTER_Str;

typedef struct
{
	uint8_t MD_Tar_Trace_Index;   //目标航迹编号
	uint8_t MD_Tar_Range_Bin;     //目标距离通道
	uint8_t MD_Tar_frame_num;     //目标帧数
	uint8_t MD_Tar_state;         //目标状态
	uint8_t MD_Tar_type;          //目标类型
	uint8_t Tar_MD_Data[95];      //目标微多普勒内容
	uint8_t MD_Reserve[28];       //预留
}SingleTarget_MicroDoppler_Str;
/**
* @brief 微多普勒目标基础数据结构
*
*/
typedef struct
{
	SingleTarget_MicroDoppler_Str SingleTarget_MicroDoppler[30];
	uint8_t Reserve[1280];
} Target_MicroDoppler_Str;


typedef struct{
    uint16_t elevationAngle : 12;
    uint16_t motion_flg : 4;
}evAngle_motionFlg_Str;


typedef struct{
    int8_t V_order : 3;
    uint8_t cfar_flag : 5;
}Vorder_cfarflag_Str;

typedef struct{
    uint16_t rangeIdx : 10;
    uint16_t clusterId : 6;
}rangeIdx_clusterId_Str;
/**
 * @brief CFAR 点目标基础数据结构
 * 
 */
typedef struct
{
	uint8_t					dopplerIdx;					//t he doppler index [Y]
	uint8_t					moveState;					// 运动属性,0：未知；1：静止；2：运动；3：无效；4：打开过；5：道路边缘；6：停止；7：可通行静止；8：低速；9：切向运动；10：保护；11：不可通行；12：删除；
	uint8_t					peakVal;                    // peak value
	int16_t					azimuthAngle;               // the azimuth angle    方位角
	evAngle_motionFlg_Str	evAngle_motionFlg;          // the elevation angle  俯仰角
	int16_t					speed;                      // velocity of the target point
	uint8_t					SNR;                        // signal to noise ratio
	Vorder_cfarflag_Str		Vorder_cfarflag;			// the azimuth ratio
	rangeIdx_clusterId_Str	rangeIdx_clusterId;         // the azimuth angle 2
	uint16_t				Rx;							// the distance for Y  纵向距离
	int16_t					Ry;                         // the distance for X  横向距离
	int16_t					Rz;                         // the distance for X  高度距离
	int16_t					RCS;						// RCS,0.01
	uint8_t					traceId;					// 航迹编号
	uint16_t				Range;						// 径向距离
} Target_CFAR;

typedef struct
{
	uint16_t		subFrameHeader;					//子协议帧头
	uint8_t			subVersion;                     //子协议版本
	uint16_t		subDataLength;					//子协议数据长度
	uint16_t		periodBitNum_Cfar;				//Cfar循环字节数
	uint16_t		maxVldCfarNum;					//最大有效Cfar数  800
	Target_CFAR		cfar[MAX_CfarPack_NUM];
	uint8_t			subCheckSum;					//子协议校验和
} Target_CFAR_Str_ACC;



/**
* @brief 监测数据区
*
*/
typedef struct
{
	uint8_t LeakOut_Amp0[18];//V0.2.9版本协议使用
	uint8_t LeakOut_Amp1[18];
	uint8_t LeakOut_Amp2[18];
	uint8_t LeakOut_Amp3[18];
} LeakOut_Amp_T_ACC;                        //宽窄波束泄漏区




typedef struct
{
	uint16_t Clutter_Amp;               //幅度
	uint16_t Clutter_X;                 //X坐标
	uint16_t Clutter_Y;                 //Y坐标
	uint8_t Blockage_Flag;              //遮挡标志位,0xA9:遮挡  0X00:未遮挡
} Clutter_Block_T_ACC;                      //遮挡检测监控数据

typedef struct
{
	uint8_t Azi_Angle_Num;              //水平补偿角个数
	int16_t Azi_Rec_Angle;              //水平补偿角度
	int16_t Prest_Azi_Angle;            //当前水平补偿值
	uint8_t Azi_Misalign_Flag;          //水平偏离标志

	uint8_t Pitch_Angle_Num;            //俯仰补偿角个数
	int16_t Pitch_Rec_Angle;            //俯仰推荐补偿角
	int16_t Prest_Pitch_Angle;          //当前帧俯仰补偿值
	uint8_t Pitch_Misalign_Flag;        //俯仰偏离标志
} AutoAngle_T_ACC;                          //自动标定监测数据


typedef struct
{
	float       RGC_RoadGradient;       //坡度
	uint8_t     RGC_RoadGradientFlag;   //坡度标志  0：平路  1：上坡  2：下坡
	uint8_t     RGC_RoadConfidence;     //坡度置信度
}RGC_Result_Str_ACC;


typedef struct
{
	uint16_t			subFrameHeader;        //帧头,0x33DD
	uint8_t				versions;               //子版本号
	uint16_t			subDataLength;         //子协议数据长度
	Clutter_Block_T_ACC Clutter_Block;      //clutter for blockage detect
	AutoAngle_T_ACC     AutoAngle_Para;     //auto angle alignment parameter
	RGC_Result_Str_ACC  RGC_Result;         //坡度计算结果输出
	LeakOut_Amp_T_ACC   LeakOut_Amp;        //泄漏区
	uint8_t				reserved[26];       //保留26字节//V0.2.9版本协议使用
	uint8_t				subCheckSum;   //子协议校验和
} MonitorData_DSP_t_ACC;                    //监测数据


/**end */





/**
* @brief 窄波束RD 数据监控区
*
*/
typedef struct
{
	uint8_t R[10];                      //10个距离通道数据
} CoverBase_Str_ACC;

typedef struct
{
	uint16_t subFrameHeader;                //子协议帧头
	uint8_t subVersion;                     //子协议版本
	uint16_t subDataLength;                 //子协议数据长度
	uint8_t RDValue[110];                  //部分RD图
	uint8_t subCheckSum;                    //子协议校验和
} CoverRD_Str_ACC;
/**end*/


/**
* @brief 协议头
*
*/
typedef struct
{
	uint16_t HeadCode1;
	uint8_t totalVersion;				//总协议版本
	uint32_t DataLength;                //package data length
}Protocol_Head_Str_ACC;


/**
 * @brief 帧头数据
 * 
 */
typedef struct
{
	uint16_t	subFrameHeader;             // 子协议帧头
	uint8_t		subVersion;
	uint16_t	subDataLength;              // 子协议数据长度
    uint8_t		RadarState;                 // working status of the radar
	uint8_t		TraceNum;                   // trace target number
	uint8_t		ClusterNum;
    uint16_t	CFAR_Num;					// CFAR point number for wide
    uint32_t	FrameCount;                 // frame pack counter for every times
	float		frametime;                  // 帧处理时间
    float		CarSpeed;                   // 车体速度DSP 获得的
	float		CarYawrate;                 // 横摆角速度
    float		SteerWheelAngle;            // 方向盘转角
    float		SteerWheelSpeed;            // 方向盘转角速度
    float		DSP_Acceleration_X;         // 车辆纵向加速度
    float		DSP_Acceleration_Y;         // 车辆横向加速度
	float		CarRadius;                  // 转弯半径
	uint8_t		Scene_VRU_Ped;				// VRU标志
	uint8_t		SystemClockHeader;			// 系统时间帧头
	uint8_t		SystemClockHour;			// 系统时间-时
	uint8_t		SystemClockMinute;			// 系统时间帧头
	uint8_t		SystemClockSec;				// 系统时间-分
	uint16_t	SystemClockMilliSec;		// 系统时间-秒
	uint8_t		PointCoordinate;			// 点云坐标系，0：雷达坐标系，1：车辆坐标系
	uint8_t		SceneryMode;				// 场景标志位，0：正常模式，1：隧道，tunnel  2：地库模式；
	uint8_t		RTK_TestMode;				// RTK测试标志，0：正常模式，1：RTK测试模式
	float		fitEgoSpeed;				//通过算法库修正后的车速
	uint8_t		fitSpeed_validFlag;			//通过算法库修正车速有效标志
	uint8_t		Reserved[19];				// 保留
	uint8_t		subCheckSum;				// 子协议校验和
}Frame_Head_Date_Str_ACC;


typedef struct
{
	uint16_t	subFrameHeader;					// 子协议帧头，固定0x11DD
	uint8_t		subVersion;						// 子协议版本
	uint16_t	subDataLength;					// 子协议数据长度
    float		rangeResolution;                // working status of the radar
    float		dopplerResolution;              // trace target number
    uint16_t	M_FFT_NUM_1_D_NARROW;           // CFAR point number for narrow
	uint16_t	M_FFT_NUM_2_D_NARROW;           // CFAR point number for wide
    uint16_t	M_FFT_NUM_3_D_NARROW;           // frame pack counter for every times
	float		centerCarrierFreq;				// 中心频率
	float		bandwidth;						// 扫频带宽，MHz
	float		lamda;							// 波长m
	float		speedofLight;					// 光速,m/s
	float		PRT;							// 脉冲重复周期,us
	uint16_t	FrameTime;						// 帧周期,ms
	float		SampleRateHz;					// 采样率,MHz
	float		startFreqConst;					// 起始频点,GHz
	float		stepFreqdelta;					// 步进频步长,MHz
	uint16_t	SampleNum;						// 采样点数
	uint16_t	ChipNum;						// Chirp个数
	uint8_t		bpmCodeNum;						// bpm序号
    uint8_t		numTxAntennas;					// 发天线数量
	uint8_t		numRxAntennas;					// 收天线数量
	float		installYaw;						// 雷达安装方位角,deg
    float		installPitch;					// 雷达安装俯仰角，deg
    float		installY;						// 雷达安装横向偏置
    float		installX;						// 雷达安装纵向偏置
    float		installZ;						// 雷达安装高度偏置
	float		installHeight;					// 雷达安装高度
	uint8_t		ConnectDirec;					// 连接器朝向,"00：未定义；,01：朝向主驾；,02：朝向副驾；"
	uint8_t		InstallSite;					// 安装位置,"00：未定义；,01：前；,02：后；"
	float		LengthVehicle;					// 自车长度,自车的长度尺寸
	float		WidthVehicle;					// 自车宽度,自车的宽度尺寸
	uint8_t     MultiPrtFlag;                   //多重频切换标志
	uint8_t     AntennaStatus;                  //雷达天线硬件  1：波导 2：微带
	uint8_t		Reserved[20];					//保留
	uint8_t		subCheckSum;					//子协议校验和
}RF_ConfigPara_Str_ACC;
/**
* @brief 雷达状态参数区
*
*/
typedef struct
{
#if(0)
	uint8_t DSP_HandleTime[11];         /*1：MIMO process 1维2维FFT时间
										2：地杂波标定
										3：窄波束处理
										4：宽波束处理
										5：聚类
										6：航迹
										7：遮挡检测
										8：自动标定处理
										9：机器学习
										10：道路边缘
										11：AEB决策 */
#endif
	uint32_t RadarStaCode;              //当前DTC，雷达应用类故障和车辆信息通讯类故障
	uint32_t TimeStamp;             //时间戳
	uint32_t SequenceNum;           //sequence_num
	uint8_t WorkMode;               //工作模式
	uint8_t  VeicleSensorStaCode;       //车身传感器异常，移动到车身数据区
	float RangeBias;                    //距离偏差      m       
} Voltage_Date_Str_ACC;                     //DSP处理时间及其他参数监控

											/**
											* @brief 道路边缘数据基础数据结构
											*
											*/
typedef struct
{
	float   roadside_factor_a;          //道路左边缘系数a
	float   roadside_factor_b;          //道路左边缘系数b
	float   roadside_factor_c;          //道路左边缘系数c
	int8_t  roadside_flag;              //道路边缘flag  -100：左边缘无效； -3~3：左边缘有效
	uint8_t roadside_frame;             //道路边缘帧数
	uint8_t roadside_lower_flag;        //道路边缘下界
	uint8_t roadside_upper_flag;        //道路边缘上界
	uint8_t roadside_status;            //道路边缘有效标志
	uint8_t roadside_confidence;        //道路边缘置信度
}Roadside_Date_Base_Str_ACC;

typedef struct
{
	uint8_t X;
	uint8_t Y;
	int8_t v_order;
	uint8_t fudu;
	int16_t angle;
	int16_t evangle;
	float speed;
}GroundPointInfo_PackOut_in;

typedef struct
{
	uint16_t	subFrameHeader;					// 帧头
	uint8_t		versions;						// 子版本号
	uint16_t	subDataLength;					// 子协议数据长度
	uint8_t		WorkMode;						// 工作模式
	float		roadside_factor_a;				// 道路左边缘系数a
	float		roadside_factor_b;				// 道路左边缘系数b
	float		roadside_factor_c;				// 道路左边缘系数c
	int8_t		roadside_flag;					// 道路左边缘flag
	uint8_t		roadside_frame;					// 道路左边缘帧数
	uint8_t		roadside_lower_flag;			// 道路左边缘下界
	uint8_t		roadside_upper_flag;			// 道路左边缘上界
	uint8_t		left_status;					// 道路左边缘有效标志
	uint8_t		left_confidence;				// 道路左边缘置信度
	float		right_roadside_factor_a;		// 道路右边缘系数a
	float		right_roadside_factor_b;		// 道路右边缘系数b
	float		right_roadside_factor_c;		// 道路右边缘系数c
	int8_t		right_roadside_flag;			// 道路右边缘flag
	uint8_t		right_roadside_frame;			// 道路右边缘帧数
	uint8_t		right_roadside_lower_flag;		// 道路右边缘下界
	uint8_t		right_roadside_upper_flag;		// 道路右边缘上界
	uint8_t		right_status;					// 道路右边缘有效标志
	uint8_t		right_confidence;				// 道路右边缘置信度
	int16_t		fixAngle_Azi;					// 方位角
	int16_t		fixAngle_Pitch;					// 俯仰角
	int16_t		Fix_Angle_Horizontal;			// 实时标定方位补偿角
	int16_t		Fix_Angle_Vertical;				// 实时标定俯仰补偿角
	uint8_t		code_cmd;						// 自动标定协议码
	uint8_t		RadarCalibrationStatus;			// 雷达标定状态
	uint16_t	PowerADC;						// 采样电压,0.01V
	uint32_t	PowerCnt;						// 上电次数
	uint8_t		dsp_temp;						// DSP温度
	uint8_t		hwa_temp;						// wa温度
	uint8_t		hsm_temp;						// hsm温度
	uint8_t		radarTemperature;				// 射频温度
	uint8_t		hardwareVersion[3];				// 硬件版本号
	uint8_t		softwareVersion[3];				// 软件版本号
	uint8_t		BTVersion[3];					// BT版本号
	uint8_t		HashAPP[8];						// APP哈希值
	uint8_t		HashALG[8];						// ALG哈希值
	GroundPointInfo_PackOut_in Rn_PackOut1[6];     //地杂波信息5+1
	uint8_t		Reserved[40];					// 保留
	uint8_t		subCheckSum;					// 子协议校验和
}Radar_Status_Date_Str_ACC;      //雷达参数状态区数据结构


typedef struct
{
	float A_VehiclSpeed;
	uint8_t A_VehiclSpeed_IC;
	uint16_t A_MasterCylinderPressure;
	float A_YawRate;
	float A_SteeringWheelAngle;
	float A_SteeringWheelSpeed;
	uint16_t A_AccelerationPedalPos;
	uint8_t A_BrakePedalSt;
	uint8_t A_WiperSpeed_Front;
	uint8_t A_TurningLigntSt;
	uint8_t A_VehicleLostFlg;
	float A_Acceleration_X; //车辆纵向加速度 lzm
	uint8_t A_Hwt_dem; //跟车时距设置 0:no gap  1:stage 1   2:stage 2    3:stage 3  4:stage 4
	uint8_t A_v_desired;
	uint16_t A_CombustionTorque;
	uint16_t A_MaximumTorque;
	uint16_t A_MinimumTorque;
	uint8_t  Reserved[101];
}DSP_ACC_AEB_CarInfo_Str;         //DSP算法部分所用车辆信息数据结构

typedef struct
{
	uint8_t ABP_Flag;
	uint8_t FCW_Flag;
	uint8_t AWB_Flag;
	uint8_t AEB_Flag;
} Alarm_AEB_T_ACC;                          //AEB监测数据


typedef struct
{
	uint8_t	AEB_out_lock_targetid;	              //输出ttc最小的目标的ID
	float AEB_out_final_ttc_min;	          //最危险目标的ttc
	float	AEB_out_brake_dec;	                  //减速度值
	uint8_t	AEB_out_brake_req;	                  //刹车请求输出
	uint8_t	AEB_out_abp_final_flag;	              //ABP激活标志位
	uint8_t	AEB_out_awb_final_flag;	              //AWB激活标志位
	uint8_t	AEB_out_awb_level;	                  //AWB等级
	uint8_t	AEB_out_VehilceHoldReq;	              //AEB停车保压请求
	uint8_t	AEB_out_CtrlType;	                  //AEB请求类型
	//uint8_t	AEB_out_TargetDirection;	          //AEB目标方向
	uint8_t	AEB_out_AEBEnable;	                  //AEB使能开关状态
	uint8_t AEB_out_TargetDetection;	          //AEB目标识别情况
	uint8_t AEB_out_TargetType;	                  //AEB锁定目标类型
	float AEB_out_TargetLngRange;	          //AEB目标纵向距离
	float AEB_out_TargetRelSpeed;	          //AEB目标纵向相对速度
	float AEB_out_TargetLatRange;	          //AEB目标横向距离
	uint8_t	AEB_out_Status;	                      //AEB系统状态
	uint8_t	AEB_out_TextInfo;	                  //AEB信息提示
	uint8_t	AEB_out_Targetmode;	                  //AEB目标模式，只发单雷达模式
	uint8_t	AEB_out_FCWSettingStatus;	          //FCW设置状态
	uint8_t	AEB_out_FCWActive;	                  //前碰撞预警激活
	uint8_t	AEB_out_FCWLatentWarning;	          //安全距离提醒报警
	uint8_t	AEB_out_FCWwaring;	                  //前碰撞预警提醒
	uint8_t	AEB_outABAlevel; 	                  //ABA等级
	uint8_t	AEB_outABAActive; 	                  //ABA激活
	uint8_t AEB_outEmergencyLightingRequest;      //危险报警灯点亮请求
	//uint8_t Lock_Target_ID;
	//Alarm_AEB_T_ACC Alarm_AEB_dsp;
	//uint16_t    Lock_Target_TTC;        //锁定目标TTC
	//uint8_t     Sbend_Flag;             //S弯道标志
	//uint8_t     Sence_Flag;             //场景分析结果
	//uint8_t     IDCount_Flag;           //目标是否出现达到20帧
	//uint8_t     IDLockCount_Flag;       //目标锁定是否满足5帧
	//uint8_t     LockTarM_Flag;          //目标锁定方式
	//uint8_t     FCW_Status;
	//uint8_t     AEB_Status;
	//Alarm_AEB_T_ACC Alarm_AEB_Logic;
	//uint16_t    TTC_FCW_L1;             //一级报警TTC阈值
	//uint16_t    TTC_FCW_L2;             //二级报警TTC阈值
	//uint16_t    TTC_AWB;                //点刹TTC阈值
	//uint16_t    TTC_ABP;                //预填充TTC阈值
	//uint16_t    TTC_AEB_P;              //部分制动TTC阈值
	//uint16_t    TTC_AEB_F;              //全部制动TTC阈值
	uint8_t  Reserved[36];
}AEB_TempInfo_Str;         //AEB相关控制数据和中间量数据结构

typedef struct
{
	float kp;
	float ki;
	float kd;
}Acc_PidParaPack_Str_DEF;

typedef struct
{
	Acc_PidParaPack_Str_DEF Acc_PidParaPack_Str[100];
} AccParaSetMonitor_Str;

typedef struct
{
	uint8_t ACC_Status;
	uint8_t ACC_Deceleration_Req_Logic;
	float   ACC_Deceleration_Val_Logic;
	uint8_t ACC_Acceleration_Req_Logic;
	float   ACC_Acceleration_TorqueVal_Logic;
	uint8_t ACC_Standstill_request_Logic;
	uint8_t ACC_Object_captured_status_Logic;
	uint8_t ACC_Take_over_request_Logic;
	uint8_t ACC_Target_distance_Logic;
	uint8_t ACC_Desire_speed_Logic;

} ACC_Logic_out_Para_str;
typedef struct
{
	float coef_a;
	float coef_b;
	float coef_c;
	float max_y;
	float min_y;
}Acc_EgoVehCurveInfo_Str;

typedef struct
{
	Acc_EgoVehCurveInfo_Str packEgoCurve_coef[2];
	Acc_EgoVehCurveInfo_Str packTargetCurve_coef[3];
	uint8_t EgoVehCurveConfidence;
	uint8_t EgoVehCurveStatus;
	uint8_t reserved[16];
}Acc_VehTrackCurve_Str;

typedef struct
{
	uint8_t ACC_out_dec_req;                   //减速度生效标志
	float ACC_out_dec;                     //输出减速度
	uint8_t ACC_out_torque_req;                //加速请求
	float ACC_out_torque;                  //加速扭矩
	uint8_t ACC_out_sd_req;                    //standstill请求
	float ACC_out_tarspeed_des;            //驾驶员设置的期望速度
	uint8_t ACC_out_tardis_stage;              //驾驶员设置的期望距离等级，4等级为最远的等级
	uint8_t ACC_out_takeover_req;              //ACC请求驾驶员接管
	uint8_t ACC_out_tarcaptured_flag;          //目标捕获标志
	uint8_t ACC_out_driveoff_flag;             //驶离控制信号
	uint8_t ACC_out_EmergencyDataRrdReq;       //ACC紧急数据记录请求（EDR）
	uint8_t	ACC_out_ACCMode;	               //ACC工作模式（状态机）
	uint8_t	ACC_out_FuelCutOffPrevention;	   //ACC禁止断油信号
	uint8_t	ACC_out_DistanceLevel;	           //ACC跟车距离等级（根据实际距离计算等级）
	float ACC_out_ACCTargetLngRange;	   //ACC目标车纵向距离
	uint8_t	ACC_out_ACCTargetType;	           //ACC目标类型
	uint8_t	ACC_out_TextInfoForDriver;	       //ACC文字提示
	float ACC_out_ACCTargetRelSpd;	       //ACC目标纵向相对速度
	float ACC_out_ACCTargetLatRange;	   //ACC目标横向距离
	uint8_t	ACC_out_ACCTargetID;	           //ACC目标ID
	uint8_t	ACC_out_TCUCreepProhibition;	   //禁止TCU进入爬行模式
	uint8_t	ACC_ACCObjSoundEnable;	           //车辆识别声音提示使能开关状态
	uint8_t	ACC_CruiseSetStatus;	           //巡航模式状态
	uint8_t	ACC_Voiceinfo;	                   //驾驶辅助声音信息
	uint8_t ACC_out_EPBrequest;                //EPB介入请求
	uint8_t ACC_out_HostTargetDetection;       //本车道目标的识别情况
	//float ACC_Out_ACCTargetAcceleration;   //ACC目标纵向加速度
										   /**************CSL控制信号**************/
	uint8_t ACC_outCSL_CSLAEnableStatus;      //自动调整巡航车速使能状态 0x0:OFF; 0x1:All Speed - Limit; 0x2:Road Camera; 0x3:invalid;
	uint8_t ACC_outCSL_CSLSetReq;             //一键设定巡航车速请求    "0x0:no request; 0x1:request"
	uint8_t ACC_outCSL_Valid_Flag;            //限速退出是否驾驶员设置：0x0:非驾驶员设置; 0x1:驾驶员设置
	uint8_t ACC_outCSL_Out_Spd;             //巡航限速输出目标车速
	uint8_t ACC_outCSL_SpdLimit_Type;         //限速类型：0x0:无限速类型,0x1:CSL-A, 0x2:CSL-M, 0x3:CSL-R,0x4:CSL-T,0x5:CSL-Toll,0x6:CSL-C 注：一键限速输出的限速值直接改变ACC的期望速度。
	uint8_t TJA_ACC_SelSts;                   //ACC主开关选择ACC——TJA/ICA状态，上电默认为3-off,0-off selected, 1-ACC selected, 2-TJA/ICA selected
	uint8_t TJA_ICA_InterSysInfoDisp;	      //显示ACC、TJA/ICA系统信息;0-Nodisp 1--不能激活ACC、TJA，2--TJA/ICAactive control is cancelled 3--TJA/ICA暂时不可用
	uint8_t TJA_ICA_Sts;					//0-OFF 1-PASSIVE 2-ACTIVE 3-RESERVED 4-FAILURE 5-RESERVED
	uint8_t ACC_FctSts;					//0--fuction not available 1--fuction available 2--performance degradation
	float ACC_out_ACCTargetAbsoluteSpd;   //ACC锁定目标的绝对速度
	float ACC_out_Expectdistance;    //期望跟车距离
	uint8_t reserved[36];
}ACC_TempInfo_Str;         //AEB相关控制数据和中间量数据结构

typedef struct
{
	float Acc_middata_one;                 //预留
	float Acc_middata_two;                 //预留
	float Acc_middata_three;               //预留
	float Acc_middata_four;                //预留
	float Acc_middata_five;                //预留
	float Acc_middata_six;                 //预留
	float Acc_middata_seven;               //预留
	float Acc_middata_eight;               //预留
	float Acc_middata_nine;                //预留
	float Acc_middata_ten;                 //预留
	float Acc_middata_eleven;              //预留
	float Acc_middata_twelve;              //预留
	float Acc_middata_thirteen;            //预留
	float Acc_middata_fourteen;            //预留
	float Acc_middata_fifteen;             //预留
	float Acc_middata_sixteen;             //预留
	float Acc_middata_seventeen;           //预留
	float Acc_middata_eighteen;            //预留
	float Acc_middata_nineteen;            //预留
	float Acc_middata_twenty;              //预留
	float Acc_middata_twenty_one;          //预留
	float Acc_middata_twenty_two;          //预留
	float Acc_middata_twenty_three;        //预留
	float Acc_middata_twenty_four;         //预留
	float Acc_middata_twenty_five;         //预留
	float Acc_middata_twenty_six;          //预留
	float Acc_middata_twenty_seven;        //预留
	float Acc_middata_twenty_eight;        //预留
	float Acc_middata_twenty_nine;         //预留
	float Acc_middata_thirty;              //预留
	uint8_t  reserve[16];  //预留
}ACC_MidData_Str;

typedef struct
{
	uint8_t targetid_lock;                    //锁定的轨迹上的目标ID
	uint8_t tartype_lock;                     //锁定的轨迹上的目标类型
	uint16_t lock_differ;                    //锁定目标距轨迹中线横向差值
	uint8_t targetid_ptl_f;                   //前方潜在风险目标ID
	uint8_t tartype_ptl_f;                    //前方潜在风险目标类型
	uint16_t ptlf_differ;                    //前方潜在风险目标距轨迹中线横向差值
	uint8_t targetid_ptl_l;                   //左侧潜在风险目标ID
	uint8_t tartype_ptl_l;                    //左侧潜在风险目标类型
	uint16_t ptll_differ;                    //左侧潜在风险目标距轨迹中线横向差值
	uint8_t targetid_ptl_r;                   //右侧潜在风险目标ID
	uint8_t tartype_ptl_r;                    //右侧潜在风险目标类型
	uint16_t ptlr_differ;                    //右侧潜在风险目标距轨迹中线横向差值
	uint8_t reserved[16];
}ACC_TarId_Info_Str;

typedef struct
{
	float coef_ego_a;                  //自车轨迹预测二次项
	float coef_ego_b;                  //自车轨迹预测一次项
	float coef_ego_c;                  //自车轨迹预测常数
	float coef_road_a;                 //道路边缘预测二次项
	float coef_road_b;                 //道路边缘预测一次项
	float coef_road_c;                 //道路边缘预测常数
	float coef_fuse_y;                 //融合点纵向距离（函数分段点）
	uint8_t fuse_flag;                     //是否采取融合显示标志
	uint8_t path_flag;                     //轨迹标志  0：直行，1：弯道
	uint8_t reserved[16];
}ADAS_PathPre_Info_Str;

typedef struct
{
	float Aeb_middata_one;                 //预留
	float Aeb_middata_two;                 //预留
	float Aeb_middata_three;               //预留
	float Aeb_middata_four;                //预留
	float Aeb_middata_five;                //预留
	float Aeb_middata_six;                 //预留
	float Aeb_middata_seven;               //预留
	float Aeb_middata_eight;               //预留
	float Aeb_middata_nine;                //预留
	float Aeb_middata_ten;                 //预留
	float Aeb_middata_eleven;              //预留
	float Aeb_middata_twelve;              //预留
	float Aeb_middata_thirteen;            //预留
	float Aeb_middata_fourteen;            //预留
	float Aeb_middata_fifteen;             //预留
	float Aeb_middata_sixteen;             //预留
	float Aeb_middata_seventeen;           //预留
	float Aeb_middata_eighteen;            //预留
	float Aeb_middata_nineteen;            //预留
	float Aeb_middata_twenty;              //预留
	float Aeb_middata_twenty_one;          //预留
	float Aeb_middata_twenty_two;          //预留
	float Aeb_middata_twenty_three;        //预留
	float Aeb_middata_twenty_four;         //预留
	float Aeb_middata_twenty_five;         //预留
	float Aeb_middata_twenty_six;          //预留
	float Aeb_middata_twenty_seven;        //预留
	float Aeb_middata_twenty_eight;        //预留
	float Aeb_middata_twenty_nine;         //预留
	float Aeb_middata_thirty;              //预留
	uint8_t reserved[128];
}AEB_MidData_Str;

typedef struct
{
	uint16_t  subFrameHeader;   //帧头
	uint8_t  versions;  //子版本号
	uint16_t  subDataLength;    //子协议数据长度
	//DSP_ACC_AEB_CarInfo_Str       DSP_ACC_AEB_CarInfo;           //雷达参数监测
	ACC_TempInfo_Str ACC_TempInfo;		//ACC输出
	AEB_TempInfo_Str AEB_TempInfo;      //AEB输出
	AccParaSetMonitor_Str  ParaSetInfo; //PID参数区
	ACC_MidData_Str  ACC_MiddataInfo;
	ACC_TarId_Info_Str ACC_TarID_Info;
	ADAS_PathPre_Info_Str PathPre_Info;
	Acc_VehTrackCurve_Str VehTrackCurve_Info;
	AEB_MidData_Str  AEB_MiddataInfo;
	uint8_t  subCheckSum;   //子协议校验和
}ADAS_Monitor_Str;                             //ADAS(AEB+ACC)监控数据结构

typedef struct
{
	uint8_t			lateralRoadIndex;
	uint8_t			longituRoadIndex;
}FS_Result_Str;//FS输出方式一

typedef struct
{
	uint16_t		Range : 10;//扇区径向距离
	uint16_t		Confidence : 2;//扇区置信度
	uint16_t		MoveState : 4;//扇区运动状态
}FS_Result_Str2;//FS输出方式二

typedef struct
{
	uint16_t		subFrameHeader;			// 子协议帧头,固定0x99DD
	uint8_t			versions;				// 子版本号
	uint16_t		subDataLength;			// 子协议数据长度
	int8_t			lateralStartRange;		// 地图横向起始距离
	uint8_t			longituStartRange;		// 地图纵向起始距离
	uint8_t			lateralSize;			// 地图横向大小
	uint8_t			longitudinalSize;		// 地图纵向大小
	uint8_t			lateralStep;			// 横向步长
	uint8_t			longitudinalStep;		// 纵向步长
	uint8_t			Reserved1[22];			// 预留位
	FS_Result_Str	FS_Result1[M_SECTIONSNUM_ALL];//输出位置索引值，根据步长和起始距离计算横纵向位置
	FS_Result_Str2	FS_Result2[360];		// -90~89.5度，0.5度一个间隔
	uint8_t			Reserved2[280];			// 预留位
	uint8_t         subCheckSum;			// 子协议校验和
}FreeSpaceData_Str;

typedef struct
{
	uint16_t		subFrameHeader;			// 子协议帧头,固定0x99DD
	uint8_t			versions;				// 子版本号
	uint16_t		subDataLength;			// 子协议数据长度
	GroundPointInfo_PackOut_in Rn_PackOut20[5];//地杂波信息
	uint8_t			RDValue[50];			//50字节空闲空间
	uint8_t         subCheckSum;			// 子协议校验和
}RD_Data_Str;

typedef struct
{
	uint16_t subFrameHeader;   //帧头
	uint8_t  versions;         //子版本号
	uint16_t subDataLength;    //子协议数据长度
	float    monitorFloatType[32];
	uint16_t monitorUint16Type[32];
	int16_t  monitorInt16Type[32];
	uint8_t  subCheckSum;   //子协议校验和
}TempData_Monitor_Str;

#ifdef __linux__
/**
* @brief Linux系统下需要的数据转换
*
*/
typedef struct
{
	// 纵向距离
	float longitudinal_distance;
	// 横向距离
	float lateral_distance;
}Point2DPosition_Linux;
typedef struct
{
	int32_t track_id;								//目标跟踪ID
	float longitudinal_distance;					//纵向距离
	float lateral_distance;							//横向距离
	float vertical_distance;						//垂直距离
	float longitudinal_absolute_velocity;			//纵向绝对速度
	float lateral_absolute_velocity;				//横向绝对速度
	float longitudinal_relative_velocity;			//纵向相对速度
	float lateral_relative_velocity;				//横向绝对速度
	float longitudinal_absolute_acceleration;		//纵向绝对加速度
	float lateral_absolute_acceleration;			//横向绝对加速度
	float longitudinal_relative_acceleration;		//纵向相对加速度
	float lateral_relative_acceleration;			//横向相对加速度
	float length;									//目标长度
	float width;									//目标宽度
	float height;									//目标高度
	float heading_angle;							//航向角
	int32_t classfication;							//目标类别
	float exist_prob;								//目标存在概率
	float obstacle_prob;							//障碍物概率
	int32_t loss_num;								//目标丢失帧数
	int32_t points_count;							//目标包含点云数量
	int32_t age;									//目标存在帧数
	int32_t region_id;								//目标区域ID
	Point2DPosition_Linux obj_refer_points;			//目标测量参考点
	int32_t motionstatus;							//目标运动状态
	float class_conf;								//目标置信度
	float height_ground;							//目标离地距离
	float location;									//相对本车道位置
	uint32_t measured;								//目标是被真实测量到的还是被预测到的
	float refpointdy;								//参考点横向位置
	float refpointdx;								//参考点纵向位置
	float refpointloc;								//参考点位置
	float dy_vnce;									//横向距离的方差
	float motionstatusconf;							//运动状态置信度
	float dx_vnce;									//纵向距离的方差
	float relvy_std;								//纵向距离的方差
	float relvx_std;								//纵向速度标准差
	int32_t valid;									//目标有效
}Target_TRACE_Str_Linux;
typedef struct
{
	float longitudinal_distance;					//纵向距离
	float lateral_distance;							//横向距离
	float vertical_distance;						//垂直距离
	float azimuth;									//方位角
	float elevation_angle;							//俯仰角
	uint32_t elevation_validy;							//俯仰角是否有效
	float radial_acceleration;						//径向加速度
	float radial_velocity;							//径向速度 
	float radius;									//径向距离
	float rcs;										//反射强度
	float snr;										//信噪比
	int32_t point_id;								//点id
	int32_t obj_id;									//所属目标ID
	int32_t motion_status;							//点运动类型
	uint32_t validity;								//点是否有效
	uint32_t qly;									//点质量
	uint32_t position;								//用于区分角雷达在左还是右
}Target_CFAR_Str_Linux;
typedef struct
{
	int32_t PointType;								//freespace点类型
	Point2DPosition_Linux position;					//Freeespace点位置
	int32_t motion_state;							//Freeespace点运动类型
}FreeSpaceData_Str_Linux;

#endif
/**
 * @brief 核间数据交互顶层结构数据
 *
 */
typedef struct
{
	Protocol_Head_Str_ACC       Protocol_Head;          //协议头
	Frame_Head_Date_Str_ACC     Frame_Head_Date;        //帧头数据区
	RF_ConfigPara_Str_ACC       RF_ConfigPara_ACC;
	Radar_Status_Date_Str_ACC   Radar_Status_Date;      //雷达状态参数区
	MonitorData_DSP_t_ACC       MonitorData;            //监控数据
	ECU_Moni_Str_ACC            ECU_DataOut;            //车身信号输出
	ADAS_Monitor_Str			ADAS_Monitor;
	Target_TRACE_Str_ACC        TraceMsg;   //航迹目标数据区
    Target_CLUSTER_Str          ClusterMsg;//聚类数据区
	Target_CFAR_Str_ACC         CfarMsg;
// #ifdef M_PROTOCOL_VERSION_200
// 	FreeSpaceData_Str			FreeSpaceData;
// 	RD_Data_Str					RD_Data;
// #endif
	TempData_Monitor_Str        TempData_Monitor;
	uint8_t						totalCheckSum;
}EthCombinePack_Str_ACC;                                //此数据结构对应网口协议2.7.6版本
													   /**end*/


#pragma pack(pop)

#endif
													   /*Local variable and declaration*/
#ifdef _TARGETPACK_C_





#endif


													   /*Global interface and declaration*/








													   /*End of file*/
													   /*
													   @copy COPYRIGHT 2018 WHST
													   @TEMPLATE [20180913]
													   */









