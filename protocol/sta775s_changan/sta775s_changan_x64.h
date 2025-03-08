#include <stdint.h>

typedef struct {
	uint8_t RadarType_u8;
	uint8_t OtherFail_u8;
	uint16_t  Reserved1;

	uint32_t RadarObj_num_u32;
	uint32_t RadarTimeHigh_u32;
	uint32_t RadarTimeLow_u32;
	double current_radar_time;
} RdrObjStsType_Replay_Fusion;

typedef struct {
	uint8_t ID_u8;
	uint8_t ObjType_u8;
	uint8_t IsStationary_u8;
	uint8_t Reserved1;

	float PosX_f32;
	float PosY_f32;
	float RelVelX_f32;
	float RelVelY_f32;
	float RelAccX_f32;
	float RelAccY_f32;
	float AbsVelX_f32;
	float AbsVelY_f32;
	float AbsAccX_f32;
	float AbsAccY_f32;
	float ObstacleProb_f32;
	float Lenth_f32;
	float Width_f32;
	float Height_f32;
	float Heading_f32;

	uint32_t Age_u32;
} RdrObjType_Replay_Fusion;

typedef struct RadarFsType_Replay {
	float Range_f32;
	uint8_t FsConfidence_u8;
	uint8_t Reserved1_u8;
	uint8_t Reserved_u8_1;
	uint8_t Reserved_u8_2;
} RadarFsType_Replay;
typedef struct RadarFsListType_Replay {
	uint32_t TimeStampHigh_u32;
	uint32_t TimeStampLow_u32;
	RadarFsType_Replay RadarFsType_Replay_st[360];
} RadarFsListType_Replay;

typedef struct {
	RdrObjStsType_Replay_Fusion SensorStatusFlags_st;
	RdrObjType_Replay_Fusion SensorObject_st[32];
	RadarFsListType_Replay RadarFs;
} RdrObjListType_Replay_Fusion;