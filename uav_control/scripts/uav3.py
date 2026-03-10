#!/usr/bin/env python3
"""
UGV控制器
"""
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import SetMode, CommandBool

class UGVFollowerPureGPS:
    def __init__(self):
        rospy.init_node('ugv_follower_pure_gps', anonymous=True)
        
        # 状态变量
        self.current_state = State()
        self.current_gps = NavSatFix()
        self.has_gps_fix = False
        
        # 目标变量 (使用字典存储GPS)
        self.target_gps = None
        self.target_received = False
        self.reached_target = False
        
        self.start_gps = None
        self.start_recorded = False
        
        # 控制参数
        self.loop_rate = rospy.Rate(20)
        self.stop_distance = 0.8  # 到达目标的最终停止距离 (米)

        # 订阅无人机发布的救援位置
        self.target_pos_sub = rospy.Subscriber("/uav1/position2", String, self.target_callback)
        
        # MAVROS 订阅
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)
        self.global_pos_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_pos_callback)
        
        self.global_pos_pub = rospy.Publisher("mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)
        self.arrival_pub = rospy.Publisher("/uav3/arrived", String, queue_size=10)
        
        # 服务代理
        rospy.loginfo("船等待MAVROS服务...")
        rospy.wait_for_service("mavros/set_mode")
        rospy.wait_for_service("mavros/cmd/arming")
        self.set_mode_srv = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.arm_srv = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        rospy.loginfo("船MAVROS服务已连接。")
        
        rospy.loginfo("无人船纯GPS控制器启动，等待指令...")

    def state_callback(self, msg):
        self.current_state = msg
    
    def global_pos_callback(self, msg):
        self.current_gps = msg
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.has_gps_fix = True
            #记录起点GPS坐标
            if not self.start_recorded:
                self.start_gps = {'lat': msg.latitude, 'lon': msg.longitude, 'alt': msg.altitude}
                self.start_recorded = True
                rospy.loginfo(f"无人船起点已记录: 纬度={msg.latitude:.8f}, 经度={msg.longitude:.8f}")
        else:
            self.has_gps_fix = False
    
    def target_callback(self, msg):
        """接收并解析无人机发来的GPS坐标或返回起点指令"""
        rospy.loginfo(f"船接收到指令: '{msg.data}'")
        
        #处理返回起点指令
        if msg.data == "back_to_start":
            if self.start_gps:
                self.target_gps = self.start_gps.copy()
                self.target_received = True
                self.reached_target = False
                rospy.loginfo(f"无人船设置返回起点: 纬度={self.start_gps['lat']:.8f}, 经度={self.start_gps['lon']:.8f}")
            else:
                rospy.logwarn("无人船起点坐标未记录，无法返回起点")
            return
        
        #原有的GPS坐标解析逻辑
        try:
            parts = msg.data.split(',')
            lat = float(parts[0].split(':')[1])
            lon = float(parts[1].split(':')[1])
            alt = float(parts[2].split(':')[1])
            
            self.target_gps = {'lat': lat, 'lon': lon, 'alt': alt}
            self.target_received = True
            self.reached_target = False  # 收到新目标，重置到达状态
            rospy.loginfo(f"无人船已设置GPS目标点: 纬度={lat:.8f}, 经度={lon:.8f}")
        except (ValueError, IndexError) as e:
            rospy.logwarn(f"无人船GPS坐标解析失败: {e}，忽略该指令。")

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """计算两个GPS坐标之间的距离(米)"""
        R = 6378137.0 # 地球半径
        
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

    def create_global_setpoint(self, lat, lon, alt):
        """创建一个GlobalPositionTarget消息"""
        setpoint = GlobalPositionTarget()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        setpoint.type_mask = (GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY | GlobalPositionTarget.IGNORE_VZ |
                               GlobalPositionTarget.IGNORE_AFX | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ |
                               GlobalPositionTarget.IGNORE_YAW | GlobalPositionTarget.IGNORE_YAW_RATE)
        setpoint.latitude = lat
        setpoint.longitude = lon
        setpoint.altitude = alt
        return setpoint

    def run(self):
        """主循环 - GPS导航的最终稳定流程"""
        rospy.loginfo("船等待有效的GPS定位...")
        while not rospy.is_shutdown() and not self.has_gps_fix:
            rospy.loginfo_throttle(5, "船等待GPS信号...")
            self.loop_rate.sleep()
        
        rospy.loginfo("船GPS定位已就绪。您现在可以随时切换到OFFBOARD模式。")
        
        while not rospy.is_shutdown():
            #自动解锁
            if self.current_state.mode == "OFFBOARD" and not self.current_state.armed:
                rospy.logwarn_throttle(2, "船在OFFBOARD模式下但未解锁，尝试自动解锁...")
                try:
                    self.arm_srv(True)
                except rospy.ServiceException as e:
                    rospy.logerr_throttle(2, f"船解锁服务调用失败: {e}")

            #核心导航逻辑
            setpoint_to_send = None

            if self.current_state.mode == "OFFBOARD" and self.current_state.armed and self.target_received and self.target_gps and self.has_gps_fix:
                distance_to_final_target = self.haversine_distance(
                    self.current_gps.latitude, self.current_gps.longitude,
                    self.target_gps['lat'], self.target_gps['lon']
                )

                if distance_to_final_target > self.stop_distance:
                    #导航到最终目标
                    self.reached_target = False
                    rospy.loginfo_throttle(2, f"船导航中, 直接前往最终目标, 距离: {distance_to_final_target:.2f}m.")
                    setpoint_to_send = self.create_global_setpoint(self.target_gps['lat'], self.target_gps['lon'], self.current_gps.altitude)
                
                else:
                    #到达目标区域，切换模式
                    if not self.reached_target:
                        rospy.loginfo(f"船已到达目标区域，误差 {distance_to_final_target:.2f}m。切换到位置保持模式。")
                        self.reached_target = True
                        #发布到达通知
                        arrival_msg = String()
                        arrival_msg.data = "arrived"
                        self.arrival_pub.publish(arrival_msg)
                        rospy.loginfo("已通知无人机：无人船已到达目标位置")
                    setpoint_to_send = self.create_global_setpoint(self.current_gps.latitude, self.current_gps.longitude, self.current_gps.altitude)
            

            if setpoint_to_send is None:
                if self.has_gps_fix:
                    setpoint_to_send = self.create_global_setpoint(self.current_gps.latitude, self.current_gps.longitude, self.current_gps.altitude)
                    if self.current_state.mode != "OFFBOARD":
                        rospy.loginfo_throttle(5, f"船非OFFBOARD模式 (当前: {self.current_state.mode})。发送心跳，等待切换...")
                    else:
                        rospy.loginfo_throttle(5, "船OFFBOARD模式已激活，发送心跳，等待目标指令...")
                else:
                    rospy.logwarn_throttle(5, "船GPS信号丢失，无法发送设定点！")

            if setpoint_to_send:
                self.global_pos_pub.publish(setpoint_to_send)

            self.loop_rate.sleep()

if __name__ == '__main__':
    try:
        follower = UGVFollowerPureGPS()
        follower.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("无人船程序被中断。")
    except Exception as e:
        rospy.logerr(f"出现未处理异常: {e}")



