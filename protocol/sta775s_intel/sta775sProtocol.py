import util.calUtil as cu
import util.binTools as bt
import time
from protocol.sta775s_intel.trackFrameDetail import TrackFrameDetail
from protocol.sta775s_intel.shelterInfo import ShelterInfo
import csv
import os
import numpy as np
import math
# import points_process

# TODO: !!!注意!!!这个协议对于动静目标\目标位置所在的坐标系与原处理方式不同,后续如果需要联合处理,需要注意这个问题

PROTOCOL_AVAILABLE = [b'\x66\xdd']
MAX_TRACE_NUM = 64
MAX_POINTS_NUM = 800
STATIC_LATERAL_POSITION_THRESHOLD = 1
class Sta775sFrame:
    def __init__(self, with_points=False, with_fs=False, data_path="", 
                 lidar_distinguish=False, with_ego=False, with_adas=False) -> None:
        self.prototolFunc = {b'\x66\xdd': self.decodeTrackProtocol, b'\x00\xdd': self.decodeBaseHeader, 
                            }
        if with_points:
            self.prototolFunc[b'\x88\xdd'] = self.decodePointsProtocol
            self.prototolFunc[b'\x33\xdd'] = self.decodeMonitorProtocol
        if with_fs:
            self.prototolFunc[b'\x99\xdd'] = self.decodeFsProtocol
        if lidar_distinguish:
            self.prototolFunc[b'\x33\xdd'] = self.decodeMonitorProtocol
        if with_ego:
            self.prototolFunc[b'\x44\xdd'] = self.decodeEgoProtocol
        if with_adas:
            self.prototolFunc[b'\x55\xdd'] = self.decodeAdasProtocol
            self.prototolFunc[b'\x22\xdd'] = self.decodeStateProtocol
            self.prototolFunc[b'\x11\xdd'] = self.decodeRadarParamProtocol
        self.baseProtocol = {}
        self.trackProtocol = {}
        self.trackProtocol['trackList_intel'] = []
        self.trackProtocol['trackList'] = []
        self.trackProtocol['radar_shelter_flag'] = []
        self.monitorProtocol = {}
        self.pointsProtocol = {}
        self.stateProtocol = {}
        self.fsProtocol = {}
        self.egoProtocol = {}
        self.adasProtocol = {}
        self.radarParamProtocol = {}
        self.data_path = data_path
        self.frame_index_in_file = 0
        
    # 解析协议的主循环
    def decodeProtocol(self, data):
        # TODO: 解析协议头和版本，确认受否接受或者转向分支。
        totalDataLength = data[3:5]
        self.totalDataLength = bt.unpackUint16(totalDataLength)
        count = 0
        proto_left_num = len(self.prototolFunc)
        bias = 7
        while(bias < self.totalDataLength-1 and count < 15):
            if proto_left_num <= 0:
                break
            subFrameHeader = data[bias: bias + 2]
            subDataLength = data[bias + 3: bias + 5]
            subDataLength = bt.unpackUint16(subDataLength)
            # 判断是否是需要处理的协议，需要处理的调用相关的处理方法，不需要处理的跳过。
            if subFrameHeader in self.prototolFunc:
                # print("find protocol", subFrameHeader)
                self.prototolFunc[subFrameHeader](
                    data[bias: bias + subDataLength])
                proto_left_num -= 1
            bias += subDataLength
            count += 1
        # self.dataFilter()

    def decodeBaseHeader(self, data):
        # 跳过无用信息, 跳过4个字节
        bias = 6
        # 解析航迹数目
        traceNum_data = data[bias: bias + 1]
        traceNum = bt.unpackUint8(traceNum_data)
        self.baseProtocol['traceNum'] = traceNum
        bias += 1

        # 跳过无用信息, 跳过1个字节
        bias += 1
        # 解析点云数目
        pointsNum_data = data[bias: bias + 2]
        pointsNum = bt.unpackUint16(pointsNum_data)
        self.baseProtocol['pointsNum'] = pointsNum
        bias += 2

        # 解析帧号
        frameCount_data = data[bias: bias + 4]
        frameCount = bt.unpackUint32(frameCount_data)
        self.baseProtocol['frameCount'] = frameCount
        bias += 4

        # 跳过处理时间
        bias += 4
        # 解析自车速度
        carSpeed_data = data[bias: bias + 4]
        carSpeed = bt.unpackFloat(carSpeed_data)
        self.baseProtocol['carSpeed'] = carSpeed
        bias += 4

        # 解析自车角速度
        carYawrate_data = data[bias: bias + 4]
        carYawrate = bt.unpackFloat(carYawrate_data)
        self.baseProtocol['carYawrate'] = carYawrate
        bias += 4

        # 解析轮角速度
        steerWheelAngle_data = data[bias: bias + 4]
        steerWheelAngle = bt.unpackFloat(steerWheelAngle_data)
        self.baseProtocol['steerWheelAngle'] = steerWheelAngle
        bias += 4

        # 解析轮速
        SteerWheelSpeed_data = data[bias: bias + 4]
        SteerWheelSpeed = bt.unpackFloat(SteerWheelSpeed_data)
        self.baseProtocol['SteerWheelSpeed'] = SteerWheelSpeed
        bias += 4

        # 解析轮速
        DSP_Acceleration_X_data = data[bias: bias + 4]
        DSP_Acceleration_X = bt.unpackFloat(DSP_Acceleration_X_data)
        self.baseProtocol['DSP_Acceleration_X'] = DSP_Acceleration_X
        bias += 4
        # 解析轮速
        DSP_Acceleration_Y_data = data[bias: bias + 4]
        DSP_Acceleration_Y = bt.unpackFloat(DSP_Acceleration_Y_data)
        self.baseProtocol['DSP_Acceleration_Y'] = DSP_Acceleration_Y
        bias += 4

        # 跳过无用信息, 跳过40个字节
        bias += 12
        sceneryMode_data = data[bias: bias + 1]
        sceneryMode = bt.unpackUint8(sceneryMode_data)
        self.baseProtocol['sceneryMode'] = sceneryMode

    def decodeRadarParamProtocol(self, data):
        subDataLength = data[3:5]
        self.radarParamProtocol['subDataLength'] = bt.unpackUint16(subDataLength)

        FrameTime = data[39:41]
        self.radarParamProtocol['FrameTime'] = bt.unpackUint16(FrameTime)
        

    def decodeTrackProtocol(self, data):
        subDataLength = data[3:5]
        self.trackProtocol['subDataLength'] = bt.unpackUint16(subDataLength)
        periodBitNum_Trace = data[5:7]
        self.trackProtocol['periodBitNum_Trace'] = bt.unpackUint16(periodBitNum_Trace)
        maxVIdTraceNum = data[7:9]
        self.trackProtocol['maxVIdTraceNum'] = bt.unpackUint16(maxVIdTraceNum)
        self.trackProtocol['trackList'] = []
        self.trackProtocol['radar_shelter_flag'] = []
        bias = 9
        if 'traceNum' in self.baseProtocol:
            trace_num = self.baseProtocol['traceNum']
        else:
            trace_num = MAX_TRACE_NUM
        for _ in range(trace_num):
            track_start_bias = bias
            index_data = data[bias: bias + 1]
            index = bt.unpackUint8(index_data)
            bias += 1
            Ry_data = data[bias: bias + 2]
            Ry = bt.unpackInt16(Ry_data)
            bias += 2
            Rx_data = data[bias: bias + 2]
            Rx = bt.unpackInt16(Rx_data)
            bias += 2
            Rz_data = data[bias: bias + 2]
            Rz = bt.unpackInt16(Rz_data)
            bias += 2
            # 目标框数据包uint32，包含长度12位、宽度10位、高度10位。
            box_data_pack = data[bias: bias + 4]
            box_data_pack_num = bt.unpackUint32(box_data_pack)
            box_length = box_data_pack_num & 4095
            box_width = (box_data_pack_num >> 12) & 1023
            box_height = (box_data_pack_num >> 22) & 1023
            bias += 4
            
            # 增加4个int16的字段:横向速度\纵向速度\横向加速度\纵向加速度
            # 横向速度cross_speed
            lateralSpeed_data = data[bias: bias + 2]
            lateralSpeed = bt.unpackInt16(lateralSpeed_data)
            bias += 2
            # 纵向速度longitu_speed
            longitudinalSpeed_data = data[bias: bias + 2]
            longitudinalSpeed = bt.unpackInt16(longitudinalSpeed_data)
            bias += 2
            # 横向加速度Aceler_Y
            lateralAcceleration_data = data[bias: bias + 2]
            lateralAcceleration = bt.unpackInt16(lateralAcceleration_data)
            bias += 2
            # 纵向加速度Aceler_X
            longitudinalAcceleration_data = data[bias: bias + 2]
            longitudinalAcceleration = bt.unpackInt16(longitudinalAcceleration_data)
            bias += 2

            # 中间数据暂时与评测无关,暂时不解,直接跳到lifetime, 跳过13个字节
            bias += 13

            lifeTime_data = data[bias: bias + 2]
            lifeTime = bt.unpackInt16(lifeTime_data)
            bias += 2


            bias += 1

            # # 解析高空目标概率、遮挡标志、静转动目标
            shelter_flag_pack = data[bias: bias + 1]
            shelter_flag_pack_num = bt.unpackUint8(shelter_flag_pack)
            road_barrie_prob = shelter_flag_pack_num & 3
            self.trackProtocol['radar_shelter_flag'] = (shelter_flag_pack_num >> 2) & 1
            slow_static_flag = (shelter_flag_pack_num >> 3) & 1


            # 跳过无用字段
            bias += 1

            # 运动状态,目标类型数据包uint8，包含moveState 4位、TarType 4位。
            moveState_TarType_pack = data[bias: bias + 1]
            moveState_TarType_pack_num = bt.unpackUint8(moveState_TarType_pack)
            moveState = moveState_TarType_pack_num & 15
            TarType = (moveState_TarType_pack_num >> 4) & 15
            bias += 1

            # 中间数据暂时与评测无关,暂时不解,直接跳到障碍物概率, 跳过1个字节
            bias += 1

            # uint8 障碍物概率 obstacle
            obstacle_data = data[bias: bias + 1]
            obstacle = bt.unpackUint8(obstacle_data)
            bias += 1

            # 中间数据暂时与评测无关,暂时不解,直接跳到Yaw, 跳过1个字节
            bias += 1

            yaw_data = data[bias: bias + 2]
            yaw = bt.unpackInt16(yaw_data)
            bias += 2
            # measure_point
            bias +=1
            # Y_distance_std
            bias +=1
            # Y_speed_std
            bias +=1
            # Y_Acc_std
            bias +=1
            # X_distance_std
            bias +=1
            # X_speed_std
            bias +=1
            # X_Acc_std
            bias +=1
            # acc_lostNum PACK
            bias +=4
            # vyAbs
            vyAbs_data = data[bias: bias + 2]
            vyAbs = bt.unpackInt16(vyAbs_data)
            bias +=2
            # vxAbs
            vxAbs_data = data[bias: bias + 2]
            vxAbs = bt.unpackInt16(vxAbs_data)
            bias +=2
            # 解完所有有效数据，开始构造航迹
            frame_cnt = self.baseProtocol['frameCount']
            track = {'frame_cnt': frame_cnt, 'index': index, 'Ry': Ry, 'Rx': Rx, 'Rz': Rz, 'box_length': box_length, 'box_width': box_width, 'box_height': box_height, 'moveState': moveState, 'TarType': TarType, 'yaw': yaw, 'lateralSpeed': lateralSpeed, 'longitudinalSpeed': longitudinalSpeed, 'lateralAcceleration': lateralAcceleration, 'longitudinalAcceleration': longitudinalAcceleration, 'obstacle': obstacle, 'lifeTime': lifeTime}
            track_intel = {
                'index': index,
                'lifeTime': lifeTime,
                'Yaw': yaw,
                'moveState': moveState,
                'box_width': box_width,
                'box_length': box_length,
                'height': Rz, # 目标离地高度
                'TarType': TarType,
                'obstacle': obstacle,
                'Ry': Ry,
                'Rx': Rx,
                'Aceler_X': longitudinalAcceleration,
                'Aceler_Y': lateralAcceleration,
                'longitu_speed': longitudinalSpeed,
                'cross_speed': lateralSpeed,
                'vxAbs': vxAbs,
                'vyAbs': vyAbs
            }
            new_track = TrackFrameDetail(track, protocolType='775s')
            self.trackProtocol['trackList_intel'].append(track_intel)
            self.trackProtocol['trackList'].append(new_track)
            # 偏移量增加，跳到下一个航迹的位置
            bias = track_start_bias + 63
            
        # 划分需要比较的运动和静止目标
        self.divMoveStaticTrack()
        # 计算遮挡率
        shelter = ShelterInfo(self.trackProtocol['trackList'])
        for track in self.trackProtocol['trackList']:
            shelter_flag = shelter.check_shelter_track(track)
            track.update_shelter(shelter_flag)
            if not shelter_flag:
                track.attention_level += 1

    def decodeMonitorProtocol(self, data):
        subDataLength = data[3:5]
        self.monitorProtocol['subDataLength'] = bt.unpackUint16(subDataLength)

        # 遮挡标志位
        blockage_flag = data[11:12]
        self.monitorProtocol['blockage_flag'] =bt.unpackUint8(blockage_flag)
        year_data = data[120:121]
        year = bt.unpackUint8(year_data) + 2000
        month_data = data[121:122]
        month = bt.unpackUint8(month_data)
        day_data = data[122:123]
        day = bt.unpackUint8(day_data)
        hour_data = data[123:124]
        hour = bt.unpackUint8(hour_data)
        minute_data = data[124:125]
        minute = bt.unpackUint8(minute_data)
        second_data = data[125:126]
        second = bt.unpackUint8(second_data)
        millisecond_data = data[126:128]
        millisecond = bt.unpackUint16(millisecond_data)
        
        syncTime_data = data[102:110]
        syncTime = bt.unpackUint64(syncTime_data)
        od_latency_data = data[110:114]
        od_latency = bt.unpackUint32(od_latency_data)
        self.monitorProtocol['timestamp'] = cu.time2unixStamp(year, month, day, hour, minute, second, millisecond)
        self.monitorProtocol['syncTime'] = syncTime
        self.monitorProtocol['od_latency'] = od_latency
    def set_timestamp(self, timestamp):
        self.monitorProtocol['timestamp'] = timestamp

    def decodeStateProtocol(self, data):
        subDataLength = data[3:5]
        self.stateProtocol['subDataLength'] = bt.unpackUint16(subDataLength)

        bias = 104
        part_blockage_flag_data = data[bias: bias + 1]
        self.stateProtocol['part_blockage_flag'] = bt.unpackUint8(part_blockage_flag_data)
        bias += 1
        bias += 58

        hashALG_list = []
        for i in range(8):
            tmp_data = data[bias: bias + 1]
            hashALG = tmp_data.decode()
            hashALG_list.append(hashALG)
            bias += 1
        # print("hashALG_list", hashALG_list)
        self.stateProtocol['hashALG_list'] = hashALG_list


        # protocol_version_list = []
        # for i in range(6):
        #     tmp_data = data[bias: bias + 1]
        #     protocol_version = bt.unpackUint8(tmp_data)
        #     protocol_version_list.append(protocol_version)
        # # print("protocol_version_list", protocol_version_list)
        # self.stateProtocol['protocol_version_list'] = protocol_version_list
        
    def decodePointsProtocol(self, data):
        subDataLength = data[3:5]
        self.pointsProtocol['subDataLength'] = bt.unpackUint16(subDataLength)
        periodBitNum_Cfar = data[5:7]
        self.pointsProtocol['periodBitNum_Cfar'] = bt.unpackUint16(periodBitNum_Cfar)
        maxVldCfarNum = data[7:9]
        self.pointsProtocol['maxVldCfarNum'] = bt.unpackUint16(maxVldCfarNum)
        self.pointsProtocol['pointsList'] = []
        bias = 9
        if 'pointsNum' in self.baseProtocol:
            points_num = self.baseProtocol['pointsNum']
        else:
            points_num = MAX_POINTS_NUM
        for _ in range(points_num):
            points_start_bias = bias
            # version 2.0.9
            # datapack1_data = data[bias: bias + 1]
            # datapack1 = bt.unpackUint8(datapack1_data)
            # motion_flag = datapack1 & 1
            # dopplerIdx = datapack1 >> 1
            # version 2.0.6
            dopplerIdx_data = data[bias: bias + 1]
            dopplerIdx = bt.unpackUint8(dopplerIdx_data)
            bias += 1

            moveState_data = data[bias: bias + 1]
            moveState = bt.unpackUint8(moveState_data)
            moveState = moveState
            bias += 1

            peakVal_data = data[bias: bias + 1]
            peakVal = bt.unpackUint8(peakVal_data)
            bias += 1

            azimuthAngle_data = data[bias: bias + 2]
            azimuthAngle = bt.unpackInt16(azimuthAngle_data)
            bias += 2

            datapack2_data = data[bias: bias + 2]
            datapack2 = bt.unpackUint16(datapack2_data)
            elevationAngle = datapack2 & 4095
            # version 2.0.9
            # quality_L = (datapack2 >> 12) & 1
            # quality_H = (datapack2 >> 13) & 1
            # resoFlag_A = (datapack2 >> 14) & 1
            # resoFlag_S = (datapack2 >> 15) & 1
            # version 2.0.6
            motion_flag = (datapack2 >> 12) & 15
            bias += 2

            doppler_data = data[bias: bias + 2]
            doppler = bt.unpackInt16(doppler_data)
            bias += 2

            snr_data = data[bias: bias + 1]
            snr = bt.unpackUint8(snr_data)
            bias += 1

            datapack3_data = data[bias: bias + 1]
            datapack3 = bt.unpackInt8(datapack3_data)
            V_order = datapack3 & 7
            cfar_flag = (datapack3 >> 3) & 31
            bias += 1

            datapack4_data = data[bias: bias + 2]
            datapack4 = bt.unpackUint16(datapack4_data)
            rangeIdx = datapack4 & 1023
            clusterId = (datapack4 >> 10) & 63
            bias += 2

            Rx_data = data[bias: bias + 2]
            Rx = bt.unpackUint16(Rx_data)
            bias += 2

            Ry_data = data[bias: bias + 2]
            Ry = bt.unpackInt16(Ry_data)
            bias += 2

            Rz_data = data[bias: bias + 2]
            Rz = bt.unpackInt16(Rz_data)
            bias += 2

            rcs_data = data[bias: bias + 1]
            rcs = bt.unpackInt8(rcs_data)
            bias += 1

            anglePeakRatio_data = data[bias: bias + 1]
            anglePeakRatio = bt.unpackUint8(anglePeakRatio_data)
            bias += 1

            TraceID_data = data[bias: bias + 1]
            TraceID = bt.unpackUint8(TraceID_data)
            bias += 1

            Range_data = data[bias: bias + 2]
            Range = bt.unpackUint16(Range_data)
            bias += 2

            point_vector = [motion_flag, dopplerIdx, moveState, peakVal, azimuthAngle, elevationAngle, 
                            doppler, snr, V_order, cfar_flag, rangeIdx, Rx, Ry, Rz, rcs, anglePeakRatio,
                            Range]
            
            self.pointsProtocol['pointsList'].append(point_vector)
            
        self.preprocess_points()

    def decodeFsProtocol(self, data):
        subDataLength = data[3:5]
        self.fsProtocol['subDataLength'] = bt.unpackUint16(subDataLength)
        lateralStartRange_data = data[5:6]
        lateralStartRange = bt.unpackInt8(lateralStartRange_data)
        longitudinalStartRange_data = data[6:7]
        longitudinalStartRange = bt.unpackUint8(longitudinalStartRange_data)
        lateralSize_data = data[7:8]
        lateralSize = bt.unpackUint8(lateralSize_data)
        longitudinalSize_data = data[8:9]
        longitudinalSize = bt.unpackUint8(longitudinalSize_data)
        lateralStep_data = data[9:10]
        lateralStep = bt.unpackUint8(lateralStep_data)
        longitudinalStep_data = data[10:11]
        longitudinalStep = bt.unpackUint8(longitudinalStep_data)
        self.fsProtocol['lateralStartRange'] = lateralStartRange
        self.fsProtocol['longitudinalStartRange'] = longitudinalStartRange
        self.fsProtocol['lateralSize'] = lateralSize
        self.fsProtocol['longitudinalSize'] = longitudinalSize
        self.fsProtocol['lateralStep'] = lateralStep
        self.fsProtocol['longitudinalStep'] = longitudinalStep
        self.fsProtocol['fsList'] = []
        bias = 289
        for i in range(360):
            fs_datapack_data = data[bias: bias + 2]
            fs_datapack = bt.unpackUint16(fs_datapack_data)
            dis = fs_datapack & 1023
            confidence = (fs_datapack >> 10) & 3
            move_state = (fs_datapack >> 12) & 15
            fs = [dis, confidence, move_state]
            self.fsProtocol['fsList'].append(fs)
            bias += 2
        # self.fsProtocol['fsList'] = np.asarray(self.fsProtocol['fsList'])

    def decodeEgoProtocol(self, data):
        subDataLength = data[3:5]
        self.egoProtocol['subDataLength'] = bt.unpackUint16(subDataLength)
        ego_speed_ic_data = data[5:6]
        ego_speed_ic = bt.unpackUint8(ego_speed_ic_data)
        ego_speedValid_data = data[6:7]
        ego_speedValid = bt.unpackUint8(ego_speedValid_data)
        # ego speed float start 7
        ego_speed_data = data[7:11]
        ego_speed = bt.unpackFloat(ego_speed_data)
        # ego yawrate float start 17
        ego_yawrate_data = data[17:21]
        ego_yawrate = bt.unpackFloat(ego_yawrate_data)
        ego_yawrateValid = data[21:22]
        ego_yawrateValid = bt.unpackUint8(ego_yawrateValid)
        self.egoProtocol["ego_speed_ic"] = ego_speed_ic
        self.egoProtocol["ego_speedValid"] = ego_speedValid
        self.egoProtocol["ego_speed"] = ego_speed
        self.egoProtocol["ego_yawrate"] = ego_yawrate
        self.egoProtocol["ego_yawrateValid"] = ego_yawrateValid

    def decodeAdasProtocol(self, data):
        subDataLength = data[3:5]
        self.adasProtocol['subDataLength'] = bt.unpackUint16(subDataLength)

        bias = 5 + 97
        AEB_out_lock_targetid_data = data[bias: bias + 1]
        AEB_out_lock_targetid = bt.unpackUint8(AEB_out_lock_targetid_data)
        self.adasProtocol['AEB_out_lock_targetid'] = AEB_out_lock_targetid
        bias += 1

        AEB_out_final_ttc_min_data = data[bias: bias + 4]
        AEB_out_final_ttc_min = bt.unpackFloat(AEB_out_final_ttc_min_data)
        self.adasProtocol['AEB_out_final_ttc_min'] = AEB_out_final_ttc_min
        bias += 4

        # 跳过AEB_out_brake_dec:float
        bias += 4

        AEB_out_brake_req_data = data[bias: bias + 1]
        AEB_out_brake_req = bt.unpackUint8(AEB_out_brake_req_data)
        self.adasProtocol['AEB_out_brake_req'] = AEB_out_brake_req
        bias += 1

        AEB_out_abp_final_flag_data = data[bias: bias + 1]
        AEB_out_abp_final_flag = bt.unpackUint8(AEB_out_abp_final_flag_data)
        self.adasProtocol['AEB_out_abp_final_flag'] = AEB_out_abp_final_flag
        bias += 1

        AEB_out_awb_final_flag_data = data[bias: bias + 1]
        AEB_out_awb_final_flag = bt.unpackUint8(AEB_out_awb_final_flag_data)
        self.adasProtocol['AEB_out_awb_final_flag'] = AEB_out_awb_final_flag
        bias += 1

        # 跳过中间字段
        bias += 22

        AEB_out_FCWActive_data = data[bias: bias + 1]
        AEB_out_FCWActive = bt.unpackUint8(AEB_out_FCWActive_data)
        self.adasProtocol['AEB_out_FCWActive'] = AEB_out_FCWActive
        bias += 1

        # 跳过AEB剩余区域
        bias += 41

        # 跳过PID配置区域
        bias += 1200

        # 跳过ACC区域中的部分数据
        bias += 136

        targetid_lock_data = data[bias: bias + 1]
        targetid_lock = bt.unpackUint8(targetid_lock_data)
        self.adasProtocol['targetid_lock'] = targetid_lock
        bias += 1

        # 跳过中间的属性
        bias += 7

        targetid_ptl_l_data = data[bias: bias + 1]
        targetid_ptl_l = bt.unpackUint8(targetid_ptl_l_data)
        self.adasProtocol['targetid_ptl_l'] = targetid_ptl_l
        bias += 1

        bias += 3

        targetid_ptl_r_data = data[bias: bias + 1]
        targetid_ptl_r = bt.unpackUint8(targetid_ptl_r_data)
        self.adasProtocol['targetid_ptl_r'] = targetid_ptl_r
        bias += 1






    def staticTrackFilter(self, track: TrackFrameDetail):
        if abs(track.lateralPosition) < STATIC_LATERAL_POSITION_THRESHOLD:
            return True
        else:
            return False

    def divMoveStaticTrack(self):
        self.trackProtocol['moveTrackList'] = []
        self.trackProtocol['staticTrackList'] = []
        for track in self.trackProtocol['trackList']:
            if track.trackMovingState < 5:
                self.trackProtocol['moveTrackList'].append(track)
            else:
                if self.staticTrackFilter(track):
                    self.trackProtocol['staticTrackList'].append(track)

    def preprocess_points(self):
        # 将点云数据转换为numpy数组
        points = self.pointsProtocol["pointsList"]
        points_array = np.array(points, dtype=np.float32)
        # mul 0.01 azimuthAngle, doppler, Range, Rx, Ry, Rz
        # (elevationAngle - 2048) * 0.01
        # mul 0.1 anglePeakRatio
        # [motion_flag, dopplerIdx, moveState, peakVal, azimuthAngle, elevationAngle, 
        # doppler, snr, V_order, cfar_flag, rangeIdx, Rx, Ry, Rz, rcs, anglePeakRatio,
        # Range]
        points_array[:, 4] = points_array[:, 4] * 0.01
        points_array[:, 5] = (points_array[:, 5] - 2048) * 0.01
        points_array[:, 6] = points_array[:, 6] * 0.01
        points_array[:, 16] = points_array[:, 16] * 0.01
        points_array[:, 11] = points_array[:, 11] * 0.01
        points_array[:, 12] = points_array[:, 12] * 0.01
        points_array[:, 13] = points_array[:, 13] * 0.01
        points_array[:, 15] = points_array[:, 15] * 0.1
        self.pointsProtocol["pointsList"] = points_array
        # points_process.draw_points(points_array, move_state_filter=[])
    
    def save_points(self, save_dir):
        if "pointsList" in self.pointsProtocol:
            frame_cnt = self.baseProtocol['frameCount']
            points = self.pointsProtocol["pointsList"]
            timestamp = int(self.monitorProtocol['timestamp'] * 1000)
            data_path = self.data_path
            
            # save points as a csv file, which title is frame_cnt
            points_file_path = os.path.join(save_dir, "{2}_{0}_{1}.csv".format(frame_cnt, timestamp, data_path.replace(".bin", "")))
            with open(points_file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["motion_flag", "dopplerIdx", "moveState", "peakVal", "azimuthAngle", "elevationAngle", "doppler", "snr", "V_order", "cfar_flag", "rangeIdx", "Rx", "Ry", "Rz", "rcs", "anglePeakRatio", "Range"])
                for point in points:
                    writer.writerow(point)
            # print("Save points to {0}".format(points_file_path))

    def fs_to_distance(self):
        fs_list = self.fsProtocol['fsList']
        fs_dis_list = []
        for i in range(len(fs_list)):
            fs = fs_list[i]
            confidence = fs[1]
            if confidence > 0:
                r = fs[0]
                degree = 90 - 0.5 * (i - 1)
                la = r * 0.1 * math.sin(math.radians(degree))
                lo = r * 0.1 * math.cos(math.radians(degree))
                fs_dis_list.append([lo, la])
        return fs_dis_list

    def set_frame_index_in_file(self, frame_index):
        self.frame_index_in_file = frame_index

def get_points_raw_data(data_dir, save_dir):
    from tqdm import tqdm
    data_list = os.listdir(data_dir)
    for data_file in tqdm(sorted(data_list)):
        bin_path = os.path.join(data_dir, data_file)
        with open(bin_path, 'rb') as f:
            totalFrameHeader = f.read(2)
            test = bt.unpackUint16(totalFrameHeader)
            totleVersion = f.read(1)
            totalDataLength = f.read(2)
            frame_length = bt.unpackUint16(totalDataLength)
            f.seek(0)
            # data = f.read(frame_length)
            # test_ins = Sta775sFrame()
            # test_ins.decodeProtocol(data)

            start_time = time.time()
            for i in range(5000):
                data = f.read(frame_length)
                if not data:
                    break
                input_ins = Sta775sFrame(with_points=True, data_path=data_file)
                input_ins.decodeProtocol(data)
                input_ins.save_points(save_dir)
            end_time = time.time()
            print("Run 5000 times cost {0} seconds.".format(end_time-start_time))

if __name__ == "__main__":
    # check system windows or linux
    import platform
    os_info = "linux"
    if platform.system() == 'Windows':
        os_info = "windows"
        # import call_dll
        dll_path = r'./dll/DataProcess.dll'
        bin_path = r'data\FRADAR_20240730-190513_075_0.bin'
        # dll = call_dll.AlglibDll(dll_path)
    else:
        dll_path = "/home/renzo/code/sta77-5s-cross-platform-alg-lib/cross-platform-vc-project/Debug/DataProcess.dll"
        # bin_path = "/home/renzo/data/D701/20231226_154645_3831-新程序验证-常规路测/dat/20231226_155205_2901_4574.bin"
        bin_path = "/home/renzo/ssd3/77-5s_lidar/20240131_103610_787/dat/20240131_110251_521_57461.bin"
        # save_dir = "/home/renzo/ssd3/77-5s_lidar/tmp/points"
        # data_dir = "/home/renzo/rs1_media/176TB1/front_sensor_data/v2/fr775s/20240328"

    with open(bin_path, 'rb') as f:
        totalFrameHeader = f.read(2)
        test = bt.unpackUint16(totalFrameHeader)
        totleVersion = f.read(1)
        totalDataLength = f.read(2)
        frame_length = bt.unpackUint16(totalDataLength)
        f.seek(0)
        # data = f.read(frame_length)
        # test_ins = Sta775sFrame()
        # test_ins.decodeProtocol(data)

        start_time = time.time()
        for i in range(5000):
            data = f.read(frame_length)
            if not data:
                break
            print("frame {0}".format(i))
            input_ins = Sta775sFrame(with_fs=True)
            input_ins.decodeProtocol(data)
            # input_ins.save_points(save_dir)
            if os_info == "windows":
                # res_data = dll.run(data, frame_length)
                output_ins = Sta775sFrame(with_fs=True)
                output_ins.decodeProtocol(data)
                # output_ins.decodeProtocol(bytes(res_data))
                print("Input track num {}, output track num {}".format(input_ins.baseProtocol['traceNum'], output_ins.baseProtocol['traceNum']))
            else:
                print("Input track num {}".format(input_ins.baseProtocol['traceNum']))
            print("=============================")

        end_time = time.time()
        print("Run 5000 times cost {0} seconds.".format(end_time-start_time))
    # get_points_raw_data(data_dir, save_dir)