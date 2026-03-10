#!/usr/bin/env python3
"""
- OFFBOARD模式下：自动解锁起飞 -> 巡航 -> 发现目标 -> 悬停并广播固定位置。
- 非OFFBOARD模式下：保持位置，但持续检测，发现目标后广播实时位置。
- 自动响应无人车/无人船到达通知，停止位置发布并恢复巡航。
"""

import rospy
import time
import math
import copy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import State, GPSRAW
from mavros_msgs.srv import SetMode, CommandBool

class UAV1Controller:
    def __init__(self):
        rospy.init_node('uav1_controller', anonymous=True)
        
        #状态管理
        self.is_armed = False
        self.is_offboard = False
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        #GPS位置管理
        self.current_gps = NavSatFix()
        self.current_gps_raw = GPSRAW()
        self.has_gps_fix = False
        self.gps_quality_good = False
        self.home_position = None
        
        #飞行状态与目标管理
        self.flight_state = "IDLE"  
        self.target_hover_pose = None 
        self.target_gps_position = None # 悬停的目标GPS坐标
        self.takeoff_complete = False
        self.auto_armed = False
        
        #发布状态管理
        self.is_publishing_position = False
        self.rescue_target = 0 # 0: none, 1: fall, 2: drown
        self.last_publish_time = rospy.Time(0)
        
        #巡航参数
        self.cruise_height = 3.0
        self.takeoff_height = 2.5
        self.cruise_radius = 3.0
        self.cruise_speed = 0.5
        self.cruise_center_x = 0.0
        self.cruise_center_y = 0.0
        self.cruise_angle = 0.0
        self.cruise_started = False
        
        #检测参数
        self.detection_threshold = 1
        self.fall_detected_count = 0
        self.drown_detected_count = 0
        self.last_fall_time = 0
        self.last_drown_time = 0
        
        #发布器
        self.ugv1_pos_pub = rospy.Publisher("/uav1/position1", String, queue_size=10)
        self.ugv2_pos_pub = rospy.Publisher("/uav1/position2", String, queue_size=10)
        self.local_pos_pub = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        #订阅器
        self.fall_sub = rospy.Subscriber("fall_alert", String, self.fall_callback)
        self.drown_sub = rospy.Subscriber("drowning_alert", String, self.drown_callback)
        self.manual_control_sub = rospy.Subscriber("uav1/manual_control", String, self.manual_control_callback)
        #订阅无人车和无人船的到达通知
        self.uav2_arrival_sub = rospy.Subscriber("/uav2/arrived", String, self.uav2_arrival_callback)
        self.uav3_arrival_sub = rospy.Subscriber("/uav3/arrived", String, self.uav3_arrival_callback)
        
        self.state_sub = rospy.Subscriber("/uav1/mavros/state", State, self.state_callback)
        self.gps_sub = rospy.Subscriber("/uav1/mavros/gpsstatus/gps1/raw", GPSRAW, self.gps_raw_callback)
        self.global_pos_sub = rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, self.global_pos_callback)
        self.local_pos_sub = rospy.Subscriber("/uav1/mavros/local_position/pose", PoseStamped, self.local_pos_callback)
        
        rospy.wait_for_service("/uav1/mavros/set_mode")
        rospy.wait_for_service("/uav1/mavros/cmd/arming")
        self.set_mode_srv = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)
        self.arm_srv = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
        
        #控制参数
        self.loop_rate = rospy.Rate(20)
        
        rospy.loginfo("控制器已启动")
        rospy.loginfo("'reset'指令可使悬停中的无人机恢复巡航")

        #飞控回调函数
    def state_callback(self, msg):
        prev_offboard = self.is_offboard
        self.current_state = msg
        self.is_armed = msg.armed
        self.is_offboard = (msg.mode == "OFFBOARD")
        
        if self.is_offboard and not prev_offboard:
            rospy.loginfo("检测到OFFBOARD模式，开始自动解锁起飞")
            self.start_offboard_sequence()
        elif not self.is_offboard and prev_offboard:
            rospy.loginfo("退出OFFBOARD模式，停止所有自主飞行任务")
            self.stop_offboard_sequence()

         #GPS回调函数
    def global_pos_callback(self, msg):
        self.current_gps = msg
        
        # 检查GPS质量
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.has_gps_fix = True
            # 检查位置协方差来判断GPS质量
            if len(msg.position_covariance) > 0:
                xy_variance = max(msg.position_covariance[0], msg.position_covariance[4])
                self.gps_quality_good = (xy_variance < 2.0)
            else:
                self.gps_quality_good = True
                
            if self.gps_quality_good:
                rospy.loginfo_throttle(30, f"高精度GPS定位良好: 纬度={msg.latitude:.8f}, 经度={msg.longitude:.8f}")
            else:
                rospy.logwarn_throttle(10, f"GPS精度较差，方差={xy_variance:.2f}")
        else:
            self.has_gps_fix = False
            self.gps_quality_good = False
            rospy.logwarn_throttle(10, f"GPS定位异常: status={msg.status.status}")

         #GPS位置回调
    def gps_raw_callback(self, msg):
        self.current_gps_raw = msg
        
    def local_pos_callback(self, msg):
        self.current_pose = msg
        
        #设置巡航中心点
        if self.home_position is None and msg.header.stamp != rospy.Time(0):
            self.cruise_center_x = msg.pose.position.x
            self.cruise_center_y = msg.pose.position.y
            self.home_position = (msg.pose.position.x, msg.pose.position.y)
            rospy.loginfo(f"设置巡航中心点: ({self.cruise_center_x:.2f}, {self.cruise_center_y:.2f})")
        
    def start_offboard_sequence(self):
        self.flight_state = "TAKING_OFF"
        self.cruise_started = False
        self.takeoff_complete = False
        # 重置auto_armed标记，允许在每次进入offboard时都尝试解锁
        self.auto_armed = False
        rospy.loginfo("开始自动解锁起飞序列")
        
    def stop_offboard_sequence(self):
        self.flight_state = "IDLE"
        self.cruise_started = False
        self.takeoff_complete = False
        self.auto_armed = False

        rospy.loginfo("停止OFFBOARD，待机状态")
        
    def auto_arm_and_takeoff(self):
        if self.is_armed:
            return True

        # 如果已经尝试过自动解锁，则不再尝试
        if self.auto_armed:
            rospy.logwarn_throttle(5, "自动解锁已尝试过一次但未成功，请检查起飞条件。")
            return False

        rospy.loginfo("未解锁，尝试自动解锁...")
        try:
            # 标记为已尝试，防止重复调用
            self.auto_armed = True
            response = self.arm_srv(True)
            if response.success:
                rospy.loginfo("自动解锁成功。")
                rospy.sleep(0.5)  # 等待MAVROS更新状态
                return True
            else:
                rospy.logwarn("自动解锁失败。")
                return False
        except Exception as e:
            rospy.logwarn(f"自动解锁服务调用异常: {e}")
            return False

    def check_takeoff_complete(self):
        #检查起飞是否完成
        if self.current_pose.pose.position.z >= (self.takeoff_height - 0.2):
            if not self.takeoff_complete:
                rospy.loginfo("起飞完成，切换到巡航状态。")
                self.takeoff_complete = True
                self.flight_state = "CRUISING"
                self.cruise_started = True
            return True
        else:
            #使用throttle减少日志刷新
            rospy.loginfo_throttle(2, f"起飞中: 当前高度={self.current_pose.pose.position.z:.2f}m / {self.takeoff_height}m")
            return False

    def get_current_gps_position(self):
       #获取当前GPS位置 - 使用高精度数据
        if not self.has_gps_fix:
            rospy.logwarn("无有效GPS定位，无法获取位置")
            return None
            
        # NavSatFix 直接使用度、米单位，无需转换
        return {
            'lat': self.current_gps.latitude,
            'lon': self.current_gps.longitude,
            'alt': self.current_gps.altitude
        }
        
    def calculate_takeoff_position(self):
        takeoff_pose = PoseStamped()
        takeoff_pose.header.stamp = rospy.Time.now()
        takeoff_pose.header.frame_id = "map"
        
        # 使用home点作为起飞的XY坐标，更加稳定
        if self.home_position:
            takeoff_pose.pose.position.x = self.home_position[0]
            takeoff_pose.pose.position.y = self.home_position[1]
        else:
            takeoff_pose.pose.position.x = self.current_pose.pose.position.x
            takeoff_pose.pose.position.y = self.current_pose.pose.position.y
            
        takeoff_pose.pose.position.z = self.takeoff_height
        takeoff_pose.pose.orientation.w = 1.0
        return takeoff_pose
        
    def calculate_cruise_position(self):
        # 计算圆形轨迹上的位置
        x = self.cruise_center_x + self.cruise_radius * math.cos(self.cruise_angle)
        y = self.cruise_center_y + self.cruise_radius * math.sin(self.cruise_angle)
        
        # 创建目标位置
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = self.cruise_height
        target_pose.pose.orientation.w = 1.0
        
        return target_pose
        
    def update_cruise_angle(self):
        # 根据速度和半径计算角度增量
        # 角速度 = 线速度 / 半径
        angular_velocity = self.cruise_speed / self.cruise_radius
        angle_increment = angular_velocity / 20.0 
        
        self.cruise_angle += angle_increment
        
        # 保持角度在0-2π范围内
        if self.cruise_angle >= 2 * math.pi:
            self.cruise_angle -= 2 * math.pi
            
    def trigger_rescue_protocol(self, target_type):
    #悬停或广播实时位置。

        if self.is_publishing_position:
            rospy.loginfo_throttle(10, "已在发布救援位置，忽略新的检测信号")
            return

        self.is_publishing_position = True
        self.rescue_target = target_type

        if self.is_offboard and self.flight_state in ["CRUISING", "TAKING_OFF"]:
            rospy.loginfo("OFFBOARD模式下发现目标！停止巡航，悬停在目标上方...")
            # 保存当前位置作为悬停点
            self.target_hover_pose = copy.deepcopy(self.current_pose)
            self.target_gps_position = copy.deepcopy(self.current_gps)
            self.flight_state = "HOVERING_OVER_TARGET"
            rospy.loginfo(f"已记录悬停点: Local(x={self.target_hover_pose.pose.position.x:.2f}, y={self.target_hover_pose.pose.position.y:.2f}), "
                          f"GPS(lat={self.target_gps_position.latitude:.8f}, lon={self.target_gps_position.longitude:.8f})")
        else:
            # 非OFFBOARD模式，或在起飞前等状态
            rospy.loginfo("非巡航状态下发现目标，开始广播无人机实时位置...")
            # 在这种情况下，位置发布循环将自动发送current_gps

    def fall_callback(self, msg):
        """摔倒识别触发"""
        rospy.loginfo(f"收到摔倒检测信号: {msg.data} ({self.fall_detected_count + 1}/{self.detection_threshold})")
        current_time = time.time()
        self.fall_detected_count += 1
        self.last_fall_time = current_time
        
        if self.fall_detected_count >= self.detection_threshold:
            rospy.loginfo("确认摔倒事件，触发救援协议。")
            self.trigger_rescue_protocol(1) # 1 for fall
        else:
            remaining = self.detection_threshold - self.fall_detected_count
            rospy.loginfo(f"还需{remaining}次摔倒检测才能触发")

    def drown_callback(self, msg):
        """溺水识别触发"""
        rospy.loginfo(f"收到溺水检测信号: {msg.data} ({self.drown_detected_count + 1}/{self.detection_threshold})")
        current_time = time.time()
        self.drown_detected_count += 1
        self.last_drown_time = current_time
        
        if self.drown_detected_count >= self.detection_threshold:
            rospy.loginfo("确认溺水事件，触发救援协议。")
            self.trigger_rescue_protocol(2) # 2 for drown
        else:
            remaining = self.detection_threshold - self.drown_detected_count
            rospy.loginfo(f"还需{remaining}次溺水检测才能触发")

    def uav2_arrival_callback(self, msg):
        """无人车到达通知回调"""
        if msg.data == "arrived" and self.rescue_target == 1:
            rospy.loginfo("收到无人车到达通知，自动停止位置发布并恢复巡航")
            self.reset_detection_and_publishing()
        
    def uav3_arrival_callback(self, msg):
        """无人船到达通知回调"""
        if msg.data == "arrived" and self.rescue_target == 2:
            rospy.loginfo("收到无人船到达通知，自动停止位置发布并恢复巡航")
            self.reset_detection_and_publishing()

    def manual_control_callback(self, msg):
        """手动控制回调函数"""
        cmd = msg.data.strip().lower()
        
        if cmd == 'status':
            self.print_status()
        elif cmd == 'reset':
            rospy.loginfo("收到手动重置指令")
            self.reset_detection_and_publishing()
        elif cmd == 'fback':
            rospy.loginfo("收到无人车返回起点指令")
            self.send_back_command_to_ugv()
        elif cmd == 'dback':
            rospy.loginfo("收到无人船返回起点指令")
            self.send_back_command_to_usv()
        else:
            rospy.logwarn(f"未知指令: {cmd}，支持: status/reset/fback/dback")

    def send_back_command_to_ugv(self):
        """发送返回起点指令给无人车"""
        msg = String()
        msg.data = "back_to_start"
        self.ugv1_pos_pub.publish(msg)
        rospy.loginfo("已发送返回起点指令给无人车")

    def send_back_command_to_usv(self):
        """发送返回起点指令给无人船"""
        msg = String()
        msg.data = "back_to_start"
        self.ugv2_pos_pub.publish(msg)
        rospy.loginfo("已发送返回起点指令给无人船")

    def reset_detection_and_publishing(self):
        """重置检测计数器和发布状态。如果正在悬停，则恢复巡航。"""
        self.fall_detected_count = 0
        self.drown_detected_count = 0
        self.last_fall_time = 0
        self.last_drown_time = 0
        
        was_publishing = self.is_publishing_position
        self.is_publishing_position = False
        self.rescue_target = 0
        
        self.target_hover_pose = None
        self.target_gps_position = None
        
        if self.is_offboard and self.flight_state == "HOVERING_OVER_TARGET":
            rospy.loginfo("检测和发布状态已重置，恢复巡航...")
            self.flight_state = "CRUISING"
        else:
            if was_publishing:
                rospy.loginfo("检测和发布状态已重置")
            else:
                rospy.loginfo("检测和发布状态已重置（之前未在发布状态）")

    def publish_rescue_position(self):
        """
        发布救援位置。
        如果正在悬停，发布固定的目标GPS。
        否则，发布无人机当前的实时GPS。
        """
        if self.rescue_target == 0:
            return

        pos_to_publish = None
        log_prefix = ""
        
        # 决定使用哪个GPS源
        if self.flight_state == "HOVERING_OVER_TARGET" and self.target_gps_position:
            pos_to_publish = self.target_gps_position
            log_prefix = "发布悬停点GPS"
        else:
            # 在非OFFBOARD模式或巡航模式下，发布实时位置
            if self.has_gps_fix:
                pos_to_publish = self.current_gps
                log_prefix = "发布实时GPS"
            else:
                rospy.logwarn_throttle(5, "无有效GPS定位，无法发布位置")
                return

        if pos_to_publish is None:
            rospy.logwarn("无位置信息可发布")
            return

        lat = pos_to_publish.latitude
        lon = pos_to_publish.longitude
        alt = pos_to_publish.altitude
        
        msg = String()
        msg.data = f"lat:{lat:.8f},lon:{lon:.8f},alt:{alt:.2f}"
        
        if self.rescue_target == 1:  # 无人车
            self.ugv1_pos_pub.publish(msg)
            rospy.loginfo(f"{log_prefix}到无人车: {msg.data}")
        elif self.rescue_target == 2:  # 无人船
            self.ugv2_pos_pub.publish(msg)
            rospy.loginfo(f"{log_prefix}到无人船: {msg.data}")
        
    def print_status(self):
        """打印当前系统状态"""
        rospy.loginfo("=== UAV1控制器状态 ===")
        rospy.loginfo(f"飞控连接: {'是' if self.current_state.connected else '否'}")
        rospy.loginfo(f"当前模式: {self.current_state.mode}")
        rospy.loginfo(f"解锁状态: {'是' if self.is_armed else '否'}")
        rospy.loginfo(f"OFFBOARD模式: {'是' if self.is_offboard else '否'}")
        rospy.loginfo(f"飞行状态: {self.flight_state}")
        rospy.loginfo(f"位置发布状态: {'持续发布中' if self.is_publishing_position else '待命中'}")
        
        # GPS状态
        if self.has_gps_fix:
            gps_pos = self.get_current_gps_position()
            if gps_pos:
                quality_str = "优秀" if self.gps_quality_good else "一般"
                rospy.loginfo(f"GPS定位({quality_str}): 纬度={gps_pos['lat']:.8f}, 经度={gps_pos['lon']:.8f}, 高度={gps_pos['alt']:.2f}m")
            else:
                rospy.loginfo("GPS定位: 数据异常")
        else:
            rospy.loginfo("GPS定位: 无有效定位")
            
        rospy.loginfo(f"摔倒检测: {self.fall_detected_count}/{self.detection_threshold}")
        rospy.loginfo(f"溺水检测: {self.drown_detected_count}/{self.detection_threshold}")
        
        if self.flight_state == "HOVERING_OVER_TARGET" and self.target_hover_pose:
             rospy.loginfo(f"悬停目标点: x={self.target_hover_pose.pose.position.x:.2f}, y={self.target_hover_pose.pose.position.y:.2f}")

        rospy.loginfo(f"巡航状态: {'运行中' if self.cruise_started else '停止'}")
        rospy.loginfo("======================")

    def run(self):
        """主循环"""
        rospy.loginfo("开始主循环...")
        
        # 等待飞控连接
        rospy.loginfo("等待飞控连接...")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo_throttle(5, "等待飞控连接...")
            self.loop_rate.sleep()
        
        rospy.loginfo("飞控已连接")
        
        # 发送初始设定点
        rospy.loginfo("发送初始设定点...")
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.position.z = self.cruise_height
        initial_pose.pose.orientation.w = 1.0
        
        for _ in range(100):
            if rospy.is_shutdown(): return
            self.local_pos_pub.publish(initial_pose)
            self.loop_rate.sleep()
        
        rospy.loginfo("初始设定点发送完成，等待遥控器模式切换...")
        
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()

                # 持续发布位置的逻辑
                if self.is_publishing_position and (now - self.last_publish_time > rospy.Duration(1.0)):
                    self.publish_rescue_position()
                    self.last_publish_time = now
                
                # OFFBOARD模式下的自动飞行控制
                if self.is_offboard:
                    if self.flight_state == "TAKING_OFF":
                        if self.auto_arm_and_takeoff():
                            takeoff_pose = self.calculate_takeoff_position()
                            self.local_pos_pub.publish(takeoff_pose)
                            self.check_takeoff_complete()
                            rospy.loginfo_throttle(5, f"起飞中: 当前高度={self.current_pose.pose.position.z:.2f}m, 目标高度={self.takeoff_height}m")
                    
                    elif self.flight_state == "CRUISING":
                        cruise_pose = self.calculate_cruise_position()
                        self.local_pos_pub.publish(cruise_pose)
                        self.update_cruise_angle()
                        rospy.loginfo_throttle(10, f"巡航中: 角度={self.cruise_angle:.2f}rad, 高度={self.cruise_height}m")
                        
                    elif self.flight_state == "HOVERING_OVER_TARGET":
                        if self.target_hover_pose:
                            hover_pose = copy.deepcopy(self.target_hover_pose)
                            hover_pose.header.stamp = rospy.Time.now()
                            self.local_pos_pub.publish(hover_pose)
                            rospy.loginfo_throttle(5, f"保持在目标位置上方悬停: x={hover_pose.pose.position.x:.2f}, y={hover_pose.pose.position.y:.2f}")
                        else:
                            # 容错：如果悬停点丢失，则恢复巡航
                            rospy.logwarn("悬停点丢失，恢复巡航模式")
                            self.flight_state = "CRUISING"
                
                # 非OFFBOARD模式下的位置保持
                else:
                    if self.current_state.armed and self.current_pose.header.stamp != rospy.Time(0):
                        hold_pose = copy.deepcopy(self.current_pose)
                        hold_pose.header.stamp = rospy.Time.now()
                        self.local_pos_pub.publish(hold_pose)
                        rospy.loginfo_throttle(30, "非OFFBOARD模式：保持当前位置，仅做检测")
                
                self.loop_rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("程序被中断")
        finally:
            rospy.loginfo("程序终止")

if __name__ == '__main__':
    try:
        controller = UAV1Controller()
        controller.run()
    except Exception as e:
        rospy.logfatal(f"程序异常: {e}")
        rospy.signal_shutdown("程序异常")


