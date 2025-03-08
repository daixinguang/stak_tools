'''
Author:  
Date: 2022-07-22 11:31:16
LastEditors:  
LastEditTime: 2022-08-02 14:04:22
FilePath: \algCompare\trackFrameDetail.py
Description: 帧内的每个目标的详细信息
'''
from util.calUtil import decode_bool, decode_float
import math
import numpy as np
from protocol.sta775s_intel.carPose import CarPose
# 保存每帧中每个跟踪目标的信息
class TrackFrameDetail:
    def __init__(self, track, protocolType = '79-3m') -> None:
        # 将所有属性初始化为0，而后调用协议对应的解析方法进行解析。
        self.track_dict               = track
        self.frame_num                = 0
        self.headingAngle             = 0
        self.longitudinalPosition     = 0
        self.lateralPosition          = 0
        self.longitudinalVelocity     = 0
        self.boxLength                = 0
        self.boxWidth                 = 0
        self.boxHeight                = 0
        self.lateralVelocity          = 0
        self.lateralAcceleration      = 0
        self.longitudinalAcceleration = 0
        self.trackMovingState         = 0
        self.trackObjectType          = 0
        self.trackState               = 0
        self.trackId                  = 0
        self.lifeTime                 = 0
        self.cipv                     = 0
        self.cipv_bak                 = 0
        self.pos_z                    = 0
        self.obstacle                 = 0
        self.my_VehiclSpeed           = 0
        self.my_YawRate               = 0
        self.my_SteeringWheelAngle    = 0
        self.my_SteeringWheelSpeed    = 0
        self.my_Acceleration_X        = 0
        self.my_Acceleration_Y        = 0
        self.lidar_speed              = 0
        self.lidar_yaw                = 0
        self.lidar_pos_x              = 0
        self.lidar_pos_y              = 0
        self.lidar_roll               = 0
        self.lidar_pitch              = 0
        self.shelter_flag             = 0
        self.prev_track               = None
        # 采用二进制表示，每一位代表一种情况,
        # 末位为1表示该目标未遮挡, 倒数第二位表示在自车道,倒数第三位表示在临车道
        self.attention_level          = 0    
        # 模糊策略所需的flag
        self.fov_edge_flag            = 0
        self.static_buffer_flag       = 0
        self.static_iou_match_flag    = 0
        self.fuzzy_fs_flag            = 0
        # 为了解决正前方激光雷达与毫米波雷达运动目标与静止目标的不匹配现象，
        # 增加的特殊逻辑，即将运动目标放入静止目标中进行匹配，
        # 为了方便后续统计处理，值为0时是普通，值为1时是运动目标列表中的目标
        # 值为2时是静止列表中的目标。
        self.move2static_flag         = 0
        self.car_turn_flag            = 0
        self.obj_turn_flag            = 0
        self.jump_flag                = 0
        if protocolType == '79-3m':
            self._unpack_corner(track)
            self.adapt_corner_protocal()
        elif protocolType == 'suteng':
            self._unpack_suteng(track)
            self.trans_coordinate()
        elif protocolType == '775s':
            self._unpack_775s(track)
            self.adapt_775s_protocal()
            self.preprocess_775s()
        self.cal_box()

    def _unpack_corner(self, track):
        pass

    def _unpack_775s(self, track):
        # {'frame_cnt': frame_cnt, 'index': index, 'Ry': Ry, 'Rx': Rx, 'Rz': Rz, 'box_length': box_length, 'box_width': box_width, 'box_height': box_height, 'moveState': moveState, 'TarType': TarType, 'yaw': yaw}
        # Rz不需要
        self.frame_num                = int(track['frame_cnt'])
        self.headingAngle             = decode_float(track['yaw'])
        self.longitudinalPosition     = decode_float(track['Rx'])
        self.lateralPosition          = decode_float(track['Ry'])
        self.boxLength                = decode_float(track['box_length'])
        self.boxWidth                 = decode_float(track['box_width'])
        self.boxHeight                = decode_float(track['box_height'])
        self.trackId                  = int(track['index'])
        self.trackMovingState         = int(track['moveState'])
        self.trackObjectType          = int(track['TarType'])
        self.lateralVelocity          = decode_float(track['lateralSpeed'])
        self.longitudinalVelocity     = decode_float(track['longitudinalSpeed'])
        self.lateralAcceleration      = decode_float(track['lateralAcceleration'])
        self.longitudinalAcceleration = decode_float(track['longitudinalAcceleration'])
        self.obstacle                 = int(track['obstacle'])
        self.lifeTime                 = int(track['lifeTime'])


    def _unpack_suteng(self, track):
        self.frame_num                = int(track['frame_num'])
        self.headingAngle             = decode_float(track['obj_yaw'])
        self.longitudinalPosition     = decode_float(track['center.x'])
        self.lateralPosition          = decode_float(track['center.y'])
        self.longitudinalVelocity     = decode_float(track['velocity.x'])
        self.boxLength                = decode_float(track['length'])
        self.boxWidth                 = decode_float(track['width'])
        self.boxHeight                = decode_float(track['height'])
        self.lateralVelocity          = decode_float(track['velocity.y'])
        self.lateralAcceleration      = decode_float(track['acceleration.y'])
        self.longitudinalAcceleration = decode_float(track['acceleration.x'])
        self.trackMovingState         = 1 if not track['move_status'] else int(float(track['move_status']))
        self.trackObjectType          = int(track['type'])
        self.trackState               = decode_bool(track['is_tracked'])
        self.trackId                  = int(track['track_id'])
        self.cipv                     = 0
        self.cipv_bak                 = 0 if not track['cipv'] else int(track['cipv']) # 保留激光雷达计算的cipv结果，为未来留出可能
        self.pos_z                    = decode_float(track['center.z'])
        
        # 自车信息
        self.my_VehiclSpeed           = decode_float(track['my_VehiclSpeed']) if 'my_VehiclSpeed' in track else 0
        self.my_YawRate               = decode_float(track['my_YawRate']) if 'my_YawRate' in track else 0
        self.my_SteeringWheelAngle    = decode_float(track['my_SteeringWheelAngle']) if 'my_SteeringWheelAngle' in track else 0
        self.my_SteeringWheelSpeed    = decode_float(track['my_SteeringWheelSpeed']) if 'my_SteeringWheelSpeed' in track else 0
        self.my_Acceleration_X        = decode_float(track['my_Acceleration_X']) if 'my_Acceleration_X' in track else 0
        self.my_Acceleration_Y        = decode_float(track['my_Acceleration_Y']) if 'my_Acceleration_Y' in track else 0

        # 真值自车信息，因自车信息不参与计算，仅用来匹配，所以暂时没有做坐标系的变换，仍在全局坐标系中。
        self.lidar_speed              = decode_float(track['car_twist']) if 'car_twist' in track else 0
        self.lidar_yaw                = decode_float(track['yaw']) if 'yaw' in track else 0
        self.lidar_pos_x              = decode_float(track['pose.pos.x']) if 'pose.pos.x' in track else 0
        self.lidar_pos_y              = decode_float(track['pose.pos.y']) if 'pose.pos.y' in track else 0
        self.lidar_roll               = decode_float(track['roll']) if 'roll' in track else 0
        self.lidar_pitch              = decode_float(track['pitch']) if 'pitch' in track else 0

        # 角雷达聚类信息.
        self.cluster_lo               = decode_float(track['clusterlongitu']) if 'clusterlongitu' in track else 0
        self.cluster_la               = decode_float(track['clusterlateral']) if 'clusterlateral' in track else 0
        self.cluster_width            = decode_float(track['clusterwidth']) if 'clusterwidth' in track else 0
        self.cluster_length           = decode_float(track['clusterlength']) if 'clusterlength' in track else 0
        self.cluster_uid              = decode_float(track['clusteruid']) if 'clusteruid' in track else 0



    # 对角雷达协议中传入的数据进行单位变换和偏置还原，让每个变量有标准物理意义
    def adapt_corner_protocal(self):
        self.headingAngle = self.headingAngle * 0.1 - 204.8
        self.longitudinalPosition = self.longitudinalPosition * 0.01 - 327.68
        self.lateralPosition = self.lateralPosition * 0.01 - 163.84
        self.longitudinalVelocity = self.longitudinalVelocity * 0.01 - 163.84
        self.boxLength = self.boxLength * 0.01
        self.boxWidth = self.boxWidth * 0.1
        self.boxHeight = self.boxHeight * 0.1
        self.lateralVelocity = self.lateralVelocity * 0.01 - 81.92
        self.lateralAcceleration = self.lateralAcceleration * 0.1 - 25.6
        self.longitudinalAcceleration = self.longitudinalAcceleration * 0.1 - 25.6

    # 对775s协议中传入的数据进行单位变换和偏置还原，让每个变量有标准物理意义
    def adapt_775s_protocal(self):
        self.headingAngle = self.headingAngle * 0.01
        self.longitudinalPosition = self.longitudinalPosition * 0.01
        self.lateralPosition = self.lateralPosition * 0.01
        self.boxLength = self.boxLength * 0.01
        self.boxWidth = self.boxWidth * 0.01
        self.boxHeight = self.boxHeight * 0.01
        self.lateralVelocity = self.lateralVelocity * 0.01
        self.longitudinalVelocity = self.longitudinalVelocity * 0.01
        self.lateralAcceleration = self.lateralAcceleration * 0.01
        self.longitudinalAcceleration = self.longitudinalAcceleration * 0.01
        self.obstacle = self.obstacle * 0.01

    def preprocess_775s(self, lane_width=3.75):
        abs_la = abs(self.lateralPosition)
        if abs_la < lane_width * 0.5:
            self.attention_level += 2
        elif abs_la < lane_width * 1.5:
            self.attention_level += 4
    
    # 为了重用常见的工具，将车辆坐标系转换成图像坐标系，横向x，纵向y，下正，右正。
    def trans_coordinate(self):
        # 车坐标航向角左正右负
        self.headingAngle             = -self.headingAngle
        self.longitudinalPosition     = -self.longitudinalPosition
        self.lateralPosition          = -self.lateralPosition
        self.longitudinalVelocity     = -self.longitudinalVelocity
        self.lateralVelocity          = -self.lateralVelocity
        self.lateralAcceleration      = -self.lateralAcceleration
        self.longitudinalAcceleration = -self.longitudinalAcceleration
        self.pos_z                    = -self.pos_z
        # 角雷达聚类信息坐标变换。
        self.cluster_la               = -self.cluster_la
        self.cluster_lo               = -self.cluster_lo


    # 计算box的左上右下点位置
    def cal_box(self):
        # 此处转换为横向x，纵向y的坐标系，为了使后续操作与常规工具兼容。
        half_width = self.boxWidth / 2
        half_length = self.boxLength / 2
        lx = self.lateralPosition - half_width
        ly = self.longitudinalPosition - half_length
        rx = self.lateralPosition + half_width
        ry = self.longitudinalPosition + half_length
        self.box = [lx, ly, rx, ry]

    # 时间同步时，航迹位置更新。
    def fix_time_diff(self, time_diff):
        if time_diff != 0:
            # tmp_lo = self.longitudinalPosition
            self.longitudinalPosition += self.longitudinalVelocity * time_diff + 0.5 * self.longitudinalAcceleration * time_diff * time_diff
            self.lateralPosition += self.lateralVelocity * time_diff + 0.5 * self.lateralAcceleration * time_diff * time_diff
            # if self.lidar_speed != 0:
            #     speed_lo = self.lidar_speed * math.cos(self.lidar_yaw)
            #     speed_la = self.lidar_speed * math.sin(self.lidar_yaw)
            # if abs( self.longitudinalVelocity) > 2:
            #     print("lo: {1}, v:{2}, dt:{3}. fix lo delta:{0}".format(self.longitudinalPosition - tmp_lo, self.longitudinalPosition, self.longitudinalVelocity, time_diff))
            self.cal_box()
        self.update_info4save()

    def update_info4save(self):
        self.track_dict['fix_center.x'] = str(-self.longitudinalPosition)
        self.track_dict['fix_center.y'] = str(-self.lateralPosition)

    def update_car_pos(self, car_pose:CarPose):
        self.lidar_pos_x = car_pose.lo
        self.lidar_pos_y = car_pose.la
        self.track_dict['fix_my_pos_x'] = str(self.lidar_pos_x)
        self.track_dict['fix_my_pos_y'] = str(self.lidar_pos_y)

    def update_cipv(self, cipv):
        self.cipv = cipv

    def get_move_state(self):
        if self.trackMovingState in [1, 2]:
            return 1
        else:
            return 0

    def update_shelter(self, shelter):
        self.shelter_flag = shelter

    def update_fov_edge_flag(self, flag):
        self.fov_edge_flag = flag

    def updata_static_buffer_flag(self, flag):
        self.static_buffer_flag = flag

    def update_static_iou_match_flag(self, flag):
        self.static_iou_match_flag = flag

    def update_fuzzy_fs_flag(self, flag):
        self.fuzzy_fs_flag = flag
        
    def update_move2static_flag(self, flag):
        self.move2static_flag = flag

    def update_car_turn_flag(self, flag):
        self.car_turn_flag = flag

    def update_obj_turn_flag(self, flag):
        self.obj_turn_flag = flag

    def update_jump_flag(self, flag):
        self.jump_flag = flag

    def update_prev_track(self, track):
        self.prev_track = track

    # 获取目标框上的4个角点和4个中心点。
    def get_round_points(self, is_out = True):
        a = self.boxLength
        b = self.boxWidth
        lo = self.longitudinalPosition
        la = self.lateralPosition
        angle = self.headingAngle
        # if self.headingAngle > math.pi or self.headingAngle < -math.pi:
        #     angle = angle / 180 * math.pi
        if is_out:
            lo = -lo
            la = -la
        tt = np.asarray([[1, 0, 0], 
                        [0, 1, 0], 
                        [lo, la, 1]])
        tz = np.asarray([[math.cos(angle), -math.sin(angle), 0], 
                         [math.sin(angle), math.cos(angle), 0], 
                         [0, 0, 1]])
        bias = []
        for i in range(3):
            for j in range(3):
                if i == 1 and j == 1:
                    continue
                bias.append([(i-1) * a / 2, (j-1) * b / 2, 1])
        m_bias = np.asarray(bias)
        m_dest = np.matmul(m_bias, tz)
        m_dest = np.matmul(m_dest, tt)
        res = []
        for d in m_dest:
            r_lo = d[0]
            r_la = d[1]
            res.append((r_lo, r_la))
        return res

    # 因存在移动向静止队列转移的情况，所以需要判断是否是正常的静止目标
    def get_static_save_flag(self):
        if self.move2static_flag:
            return False
        else:
            return True
        
    def add_gt_extra_info(self, comp_frame_cnt):
        self.track_dict['radar_frame_num'] = str(comp_frame_cnt)
        self.track_dict['center_z'] = self.track_dict['center.z']
        self.track_dict['center_x'] = self.track_dict['fix_center.x']
        self.track_dict['center_y'] = self.track_dict['fix_center.y']
        self.track_dict['velocity_x'] = self.track_dict['velocity.x']
        self.track_dict['velocity_y'] = self.track_dict['velocity.y']
        self.track_dict['velocity_z'] = self.track_dict['velocity.z']
        round_points = self.get_round_points(True)
        order = ["right_back", "center_back", "left_back", "right_center", "left_center", "right_front", "center_front", "left_front"]
        for i in range(8):
            self.track_dict["{0}_x".format(order[i])] = str(round_points[i][0])
            self.track_dict["{0}_y".format(order[i])] = str(round_points[i][1])