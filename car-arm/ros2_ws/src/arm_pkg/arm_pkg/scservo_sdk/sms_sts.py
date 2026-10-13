#!/usr/bin/env python

from .scservo_def import *
from .protocol_packet_handler import *
from .group_sync_read import *
from .group_sync_write import *

#波特率定义
SMS_STS_1M = 0
SMS_STS_0_5M = 1
SMS_STS_250K = 2
SMS_STS_128K = 3
SMS_STS_115200 = 4
SMS_STS_76800 = 5
SMS_STS_57600 = 6
SMS_STS_38400 = 7

# #内存表定义
# #-------EPROM(只读)--------
# SMS_STS_MODEL_L = 3
# SMS_STS_MODEL_H = 4

# #-------EPROM(读写)--------
# SMS_STS_ID = 5
# SMS_STS_BAUD_RATE = 6
# SMS_STS_MIN_ANGLE_LIMIT_L = 9
# SMS_STS_MIN_ANGLE_LIMIT_H = 10
# SMS_STS_MAX_ANGLE_LIMIT_L = 11
# SMS_STS_MAX_ANGLE_LIMIT_H = 12
# SMS_STS_CW_DEAD = 26
# SMS_STS_CCW_DEAD = 27
# SMS_STS_OFS_L = 31
# SMS_STS_OFS_H = 32
# SMS_STS_MODE = 33

# #-------SRAM(读写)--------
# SMS_STS_TORQUE_ENABLE = 40
# SMS_STS_ACC = 41
# SMS_STS_GOAL_POSITION_L = 42
# SMS_STS_GOAL_POSITION_H = 43
# SMS_STS_GOAL_TIME_L = 44
# SMS_STS_GOAL_TIME_H = 45
# SMS_STS_GOAL_SPEED_L = 46
# SMS_STS_GOAL_SPEED_H = 47
# SMS_STS_LOCK = 55

# #-------SRAM(只读)--------
# SMS_STS_PRESENT_POSITION_L = 56
# SMS_STS_PRESENT_POSITION_H = 57
# SMS_STS_PRESENT_SPEED_L = 58
# SMS_STS_PRESENT_SPEED_H = 59
# SMS_STS_PRESENT_LOAD_L = 60
# SMS_STS_PRESENT_LOAD_H = 61
# SMS_STS_PRESENT_VOLTAGE = 62
# SMS_STS_PRESENT_TEMPERATURE = 63
# SMS_STS_MOVING = 66
# SMS_STS_PRESENT_CURRENT_L = 69
# SMS_STS_PRESENT_CURRENT_H = 70

# 内存表定义
# ------- EPROM (只读: 舵机出厂信息) --------
SMS_STS_MODEL_L = 3                # 舵机型号低位
SMS_STS_MODEL_H = 4                # 舵机型号高位

# ------- EPROM (读写: 修改后断电保存) --------
SMS_STS_ID = 5                     # 舵机通信ID (0-253)
SMS_STS_BAUD_RATE = 6              # 串口通信波特率
SMS_STS_MIN_ANGLE_LIMIT_L = 9      # 最小角度限制低位
SMS_STS_MIN_ANGLE_LIMIT_H = 10     # 最小角度限制高位
SMS_STS_MAX_ANGLE_LIMIT_L = 11     # 最大角度限制低位
SMS_STS_MAX_ANGLE_LIMIT_H = 12     # 最大角度限制高位
SMS_STS_CW_DEAD = 26               # 顺时针死区 (控制精度/抖动)
SMS_STS_CCW_DEAD = 27              # 逆时针死区 (控制精度/抖动)
SMS_STS_OFS_L = 31                 # 中位偏移量校准低位
SMS_STS_OFS_H = 32                 # 中位偏移量校准高位
SMS_STS_MODE = 33                  # 工作模式 (0:位置模式, 1:电机模式, 2:步进模式)

# ------- SRAM (读写: 实时控制指令，断电不保存) --------
SMS_STS_TORQUE_ENABLE = 40         # 扭矩开关 (1:开启, 0:释放/手转)
SMS_STS_ACC = 41                   # 加速度 (数值越大启动越快)
SMS_STS_GOAL_POSITION_L = 42       # 目标位置低位 (0-4095)
SMS_STS_GOAL_POSITION_H = 43       # 目标位置高位
SMS_STS_GOAL_TIME_L = 44           # 运行时间/扭矩低位 (运动到目标点所需时间)
SMS_STS_GOAL_TIME_H = 45           # 运行时间/扭矩高位
SMS_STS_GOAL_SPEED_L = 46          # 目标速度低位 (50步/秒为单位)
SMS_STS_GOAL_SPEED_H = 47          # 目标速度高位
SMS_STS_LOCK = 55                  # 锁存EEPROM (写1时可修改EPROM参数)

# ------- SRAM (只读: 实时状态反馈) --------
SMS_STS_PRESENT_POSITION_L = 56    # 当前位置低位
SMS_STS_PRESENT_POSITION_H = 57    # 当前位置高位
SMS_STS_PRESENT_SPEED_L = 58       # 当前速度低位
SMS_STS_PRESENT_SPEED_H = 59       # 当前速度高位
SMS_STS_PRESENT_LOAD_L = 60        # 当前负载低位 (输出扭矩大小)
SMS_STS_PRESENT_LOAD_H = 61        # 当前负载高位
SMS_STS_PRESENT_VOLTAGE = 62       # 当前电压 (单位: 0.1V)
SMS_STS_PRESENT_TEMPERATURE = 63   # 当前温度 (单位: ℃)
SMS_STS_MOVING = 66                # 是否正在移动 (1:是, 0:到达目标/停止)
SMS_STS_PRESENT_CURRENT_L = 69     # 当前电流低位 (单位: 6.5mA)
SMS_STS_PRESENT_CURRENT_H = 70     # 当前电流高位

class sms_sts(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        self.groupSyncWrite = GroupSyncWrite(self, SMS_STS_ACC, 7)

    def WritePosEx(self, scs_id, position, speed, acc):
        position = self.scs_toscs(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def ReadPos(self, scs_id):
        scs_present_position, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
        return self.scs_tohost(scs_present_position, 15), scs_comm_result, scs_error

    def ReadSpeed(self, scs_id):
        scs_present_speed, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SMS_STS_PRESENT_SPEED_L)
        return self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadPosSpeed(self, scs_id):
        scs_present_position_speed, scs_comm_result, scs_error = self.read4ByteTxRx(scs_id, SMS_STS_PRESENT_POSITION_L)
        scs_present_position = self.scs_loword(scs_present_position_speed)
        scs_present_speed = self.scs_hiword(scs_present_position_speed)
        return self.scs_tohost(scs_present_position, 15), self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadMoving(self, scs_id):
        moving, scs_comm_result, scs_error = self.read1ByteTxRx(scs_id, SMS_STS_MOVING)
        return moving, scs_comm_result, scs_error

    def SyncWritePosEx(self, scs_id, position, speed, acc):
        position = self.scs_toscs(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.groupSyncWrite.addParam(scs_id, txpacket)

    def RegWritePosEx(self, scs_id, position, speed, acc):
        position = self.scs_toscs(position, 15)
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.regWriteTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WheelMode(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_MODE, 1)

    def WriteSpec(self, scs_id, speed, acc):
        speed = self.scs_toscs(speed, 15)
        txpacket = [acc, 0, 0, 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, SMS_STS_ACC, len(txpacket), txpacket)

    def LockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_LOCK, 1)

    def unLockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SMS_STS_LOCK, 0)
