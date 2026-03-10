#!/usr/bin/env python3
"""
- 订阅YOLO识别消息
- 当识别到溺水时向串口发送"drown"
- 当识别到摔倒时向串口发送"fall"
- 订阅三个设备的位置信息并发送到串口
"""

import rospy
import serial
import time
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class SerialYOLOSender:
    def __init__(self):
        rospy.init_node('serial_yolo_sender', anonymous=True)
        
        # 串口配置
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyTHS1')  # 串口设备名
        self.baud_rate = rospy.get_param('~baud_rate', 115200)  # 波特率
        self.serial_connection = None
        
        # 检测参数
        self.detection_threshold = rospy.get_param('~detection_threshold', 2)  # 检测阈值
        self.fall_detected_count = 0
        self.drown_detected_count = 0
        self.last_fall_time = 0
        self.last_drown_time = 0
        self.timeout_seconds = rospy.get_param('~timeout_seconds', 20.0)  # 超时时间
        
        # 状态管理
        self.current_status = "normal"  # normal, fall, drown
        self.last_status_sent = 0
        self.status_send_interval = rospy.get_param('~status_send_interval', 2.0)  # 状态发送间隔
        self.last_fall_sent = 0
        self.last_drown_sent = 0
        self.send_interval = rospy.get_param('~send_interval', 2.0)  # 发送间隔
        
        # 位置信息管理
        self.position_send_interval = rospy.get_param('~position_send_interval', 2.0)  # 位置发送间隔
        self.last_position_sent = 0
        self.uav1_position = None
        self.uav2_position = None
        self.uav3_position = None
        
        # 订阅器 - YOLO检测
        self.fall_sub = rospy.Subscriber("fall_alert", String, self.fall_callback)
        self.drown_sub = rospy.Subscriber("drowning_alert", String, self.drown_callback)
        
        # 订阅器 - 位置信息
        self.uav1_pos_sub = rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, self.uav1_position_callback)
        self.uav2_pos_sub = rospy.Subscriber("/uav2/mavros/global_position/global", NavSatFix, self.uav2_position_callback)
        self.uav3_pos_sub = rospy.Subscriber("/uav3/mavros/global_position/global", NavSatFix, self.uav3_position_callback)
        
        # 初始化串口连接
        self.init_serial_connection()
        
        rospy.loginfo("=== 串口YOLO发送器已启动 ===")
        rospy.loginfo(f"串口设备: {self.serial_port}")
        rospy.loginfo(f"波特率: {self.baud_rate}")
        rospy.loginfo(f"检测阈值: {self.detection_threshold}")
        rospy.loginfo(f"发送间隔: {self.send_interval}秒")
        rospy.loginfo(f"位置和状态发送间隔: {self.position_send_interval}秒")
        rospy.loginfo(f"当前状态: {self.current_status}")

    def init_serial_connection(self):
        """初始化串口连接"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1,
                write_timeout=1
            )
            rospy.loginfo(f"串口连接成功: {self.serial_port}")
        except serial.SerialException as e:
            rospy.logerr(f"串口连接失败: {e}")
            rospy.logerr("请检查串口设备是否存在且权限正确")
            self.serial_connection = None

    def send_serial_command(self, command):
        """向串口发送命令"""
        if self.serial_connection is None:
            rospy.logwarn("串口未连接，无法发送命令")
            return False
            
        try:
            command_with_newline = command + '\n'
            self.serial_connection.write(command_with_newline.encode('utf-8'))
            self.serial_connection.flush()  # 确保数据立即发送
            rospy.loginfo(f"串口发送成功: {command}")
            return True
        except serial.SerialException as e:
            rospy.logerr(f"串口发送失败: {e}")
            # 尝试重新连接
            self.reconnect_serial()
            return False

    def reconnect_serial(self):
        """重新连接串口"""
        if self.serial_connection:
            try:
                self.serial_connection.close()
            except:
                pass
        
        rospy.loginfo("尝试重新连接串口...")
        time.sleep(1)
        self.init_serial_connection()

    def fall_callback(self, msg):
        """摔倒识别回调"""
        current_time = time.time()
        rospy.loginfo(f"收到摔倒检测信号: {msg.data} ({self.fall_detected_count + 1}/{self.detection_threshold})")
        
        # 检查是否超时重置计数器
        if current_time - self.last_fall_time > self.timeout_seconds:
            self.fall_detected_count = 0
            
        self.fall_detected_count += 1
        self.last_fall_time = current_time
        
        if self.fall_detected_count >= self.detection_threshold:
            # 检查发送间隔
            if current_time - self.last_fall_sent > self.send_interval:
                rospy.loginfo("确认摔倒事件，更新状态为fall")
                self.current_status = "fall"
                self.last_fall_sent = current_time
            else:
                rospy.loginfo_throttle(1, "摔倒状态更新间隔未到，跳过更新")
        else:
            remaining = self.detection_threshold - self.fall_detected_count
            rospy.loginfo(f"还需{remaining}次摔倒检测才能触发状态更新")

    def drown_callback(self, msg):
        """溺水识别回调"""
        current_time = time.time()
        rospy.loginfo(f"收到溺水检测信号: {msg.data} ({self.drown_detected_count + 1}/{self.detection_threshold})")
        
        # 检查是否超时重置计数器
        if current_time - self.last_drown_time > self.timeout_seconds:
            self.drown_detected_count = 0
            
        self.drown_detected_count += 1
        self.last_drown_time = current_time
        
        if self.drown_detected_count >= self.detection_threshold:
            # 检查发送间隔
            if current_time - self.last_drown_sent > self.send_interval:
                rospy.loginfo("确认溺水事件，更新状态为drown")
                self.current_status = "drown"
                self.last_drown_sent = current_time
            else:
                rospy.loginfo_throttle(1, "溺水状态更新间隔未到，跳过更新")
        else:
            remaining = self.detection_threshold - self.drown_detected_count
            rospy.loginfo(f"还需{remaining}次溺水检测才能触发状态更新")

    def uav1_position_callback(self, msg):
        """UAV1位置回调"""
        self.uav1_position = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }
        rospy.loginfo_throttle(5, f"UAV1位置更新: 纬度={msg.latitude:.8f}, 经度={msg.longitude:.8f}, 高度={msg.altitude:.2f}m")

    def uav2_position_callback(self, msg):
        """UAV2位置回调"""
        self.uav2_position = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }
        rospy.loginfo_throttle(5, f"UAV2位置更新: 纬度={msg.latitude:.8f}, 经度={msg.longitude:.8f}, 高度={msg.altitude:.2f}m")

    def uav3_position_callback(self, msg):
        """UAV3位置回调"""
        self.uav3_position = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }
        rospy.loginfo_throttle(5, f"UAV3位置更新: 纬度={msg.latitude:.8f}, 经度={msg.longitude:.8f}, 高度={msg.altitude:.2f}m")

    def check_status_reset(self):
        """检查状态是否需要重置为normal"""
        current_time = time.time()
        
        # 如果当前状态是fall或drown，检查是否超过一定时间没有新的检测
        if self.current_status == "fall":
            if current_time - self.last_fall_time > self.timeout_seconds * 2:
                rospy.loginfo("摔倒检测超时，状态重置为normal")
                self.current_status = "normal"
                self.fall_detected_count = 0
        elif self.current_status == "drown":
            if current_time - self.last_drown_time > self.timeout_seconds * 2:
                rospy.loginfo("溺水检测超时，状态重置为normal")
                self.current_status = "normal"
                self.drown_detected_count = 0

    def send_positions_to_serial(self):
        """发送所有设备的位置信息到串口"""
        current_time = time.time()
        
        # 检查发送间隔
        if current_time - self.last_position_sent < self.position_send_interval:
            return
            
        sent_count = 0

        # UAV1
        if self.uav1_position:
            pos_str = f"uav1:{self.uav1_position['lat']:.8f},{self.uav1_position['lon']:.8f},{self.uav1_position['alt']:.2f}"
        else:
            pos_str = "uav1:0.00000000,0.00000000,0.00"
        if self.send_serial_command(pos_str):
            sent_count += 1

        # UAV2
        if self.uav2_position:
            pos_str = f"uav2:{self.uav2_position['lat']:.8f},{self.uav2_position['lon']:.8f},{self.uav2_position['alt']:.2f}"
        else:
            pos_str = "uav2:0.00000000,0.00000000,0.00"
        if self.send_serial_command(pos_str):
            sent_count += 1

        # UAV3
        if self.uav3_position:
            pos_str = f"uav3:{self.uav3_position['lat']:.8f},{self.uav3_position['lon']:.8f},{self.uav3_position['alt']:.2f}"
        else:
            pos_str = "uav3:0.00000000,0.00000000,0.00"
        if self.send_serial_command(pos_str):
            sent_count += 1

        # 发送状态信息
        status_str = f"status:{self.current_status}"
        if self.send_serial_command(status_str):
            sent_count += 1

        if sent_count > 0:
            self.last_position_sent = current_time
            rospy.loginfo(f"位置和状态信息发送成功: {sent_count}条信息")
        else:
            rospy.logwarn_throttle(5, "没有可用的位置信息")

    def print_status(self):
        """打印当前状态"""
        rospy.loginfo("=== 串口YOLO发送器状态 ===")
        rospy.loginfo(f"串口连接: {'已连接' if self.serial_connection else '未连接'}")
        if self.serial_connection:
            rospy.loginfo(f"串口设备: {self.serial_port}")
            rospy.loginfo(f"波特率: {self.baud_rate}")
        rospy.loginfo(f"当前状态: {self.current_status}")
        rospy.loginfo(f"摔倒检测: {self.fall_detected_count}/{self.detection_threshold}")
        rospy.loginfo(f"溺水检测: {self.drown_detected_count}/{self.detection_threshold}")
        rospy.loginfo(f"检测超时: {self.timeout_seconds}秒")
        rospy.loginfo(f"发送间隔: {self.send_interval}秒")
        rospy.loginfo(f"位置和状态发送间隔: {self.position_send_interval}秒")
        
        # 位置信息状态
        rospy.loginfo("位置信息状态:")
        rospy.loginfo(f"  UAV1: {'可用' if self.uav1_position else '不可用'}")
        rospy.loginfo(f"  UAV2: {'可用' if self.uav2_position else '不可用'}")
        rospy.loginfo(f"  UAV3: {'可用' if self.uav3_position else '不可用'}")
        rospy.loginfo("========================")

    def run(self):
        rospy.loginfo("主循环")
        
        # 发送测试命令确认串口工作正常
        if self.serial_connection:
            rospy.loginfo("发送测试命令...")
            self.send_serial_command("test")
        
        rate = rospy.Rate(10)  # 10Hz
        
        try:
            while not rospy.is_shutdown():
                # 定期检查串口连接状态
                if self.serial_connection and not self.serial_connection.is_open:
                    rospy.logwarn("串口连接断开，尝试重连...")
                    self.reconnect_serial()
                
                # 检查状态是否需要重置为normal
                self.check_status_reset()
                
                # 发送位置信息和状态信息
                self.send_positions_to_serial()
                
                rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("程序被中断")
        finally:
            if self.serial_connection:
                self.serial_connection.close()
            rospy.loginfo("程序终止")

if __name__ == '__main__':
    try:
        sender = SerialYOLOSender()
        sender.run()
    except Exception as e:
        rospy.logfatal(f"程序异常: {e}")
        rospy.signal_shutdown("程序异常") 
