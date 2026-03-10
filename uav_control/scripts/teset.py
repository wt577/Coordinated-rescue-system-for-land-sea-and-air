#!/usr/bin/env python3
"""
救援系统测试脚本
专门用于测试控制器的检测触发和载具控制功能
"""

import rospy
import time
from std_msgs.msg import String

class RescueSystemTester:
    def __init__(self):
        rospy.init_node('rescue_system_tester', anonymous=True)
        
        # 发布器
        self.fall_pub = rospy.Publisher('fall_alert', String, queue_size=10)
        self.drown_pub = rospy.Publisher('drowning_alert', String, queue_size=10)
        self.manual_control_pub = rospy.Publisher('/uav1/manual_control', String, queue_size=10)
        
        # 等待发布器准备就绪
        rospy.sleep(1)
        
        rospy.loginfo("救援系统测试器已启动")
        rospy.loginfo("支持的命令:")
        rospy.loginfo("  'fall' - 模拟摔倒检测")
        rospy.loginfo("  'drown' - 模拟溺水检测")
        rospy.loginfo("  'fback' - 无人车返回起点")
        rospy.loginfo("  'dback' - 无人船返回起点")
        rospy.loginfo("  'reset' - 重置检测计数")
        rospy.loginfo("  'status' - 查看无人机状态")
        rospy.loginfo("  'stop' - 终止程序")
        rospy.loginfo("  'help' - 显示帮助信息")
        
        self.rate = rospy.Rate(1)
        
    def send_fall_detection(self):
        """发送摔倒检测信号"""
        msg = String()
        msg.data = "person_fallen_detected"
        self.fall_pub.publish(msg)
        rospy.loginfo("已发送摔倒检测信号")
        
    def send_drown_detection(self):
        """发送溺水检测信号"""
        msg = String()
        msg.data = "person_drowning_detected"
        self.drown_pub.publish(msg)
        rospy.loginfo("已发送溺水检测信号")
        
    def send_fback_command(self):
        """发送无人车返回起点指令"""
        msg = String()
        msg.data = "fback"
        self.manual_control_pub.publish(msg)
        rospy.loginfo("已发送无人车返回起点指令")
        
    def send_dback_command(self):
        """发送无人船返回起点指令"""
        msg = String()
        msg.data = "dback"
        self.manual_control_pub.publish(msg)
        rospy.loginfo("已发送无人船返回起点指令")
        
    def send_reset_command(self):
        """发送重置指令"""
        msg = String()
        msg.data = "reset"
        self.manual_control_pub.publish(msg)
        rospy.loginfo("已发送重置指令")
        
    def send_status_command(self):
        """发送状态查询指令"""
        msg = String()
        msg.data = "status"
        self.manual_control_pub.publish(msg)
        rospy.loginfo("已发送状态查询指令")
        
    def trigger_fall_rescue(self):
        """触发摔倒救援流程（发送2次检测信号）"""
        rospy.loginfo("开始触发摔倒救援流程...")
        for i in range(2):
            self.send_fall_detection()
            rospy.loginfo(f"摔倒检测信号 {i+1}/2")
            time.sleep(0.5)
        rospy.loginfo("摔倒救援流程触发完成")
        
    def trigger_drown_rescue(self):
        """触发溺水救援流程（发送2次检测信号）"""
        rospy.loginfo("开始触发溺水救援流程...")
        for i in range(2):
            self.send_drown_detection()
            rospy.loginfo(f"溺水检测信号 {i+1}/2")
            time.sleep(0.5)
        rospy.loginfo("溺水救援流程触发完成")
        
    def show_help(self):
        """显示帮助信息"""
        rospy.loginfo("=== 救援系统测试器帮助信息 ===")
        rospy.loginfo("检测信号命令:")
        rospy.loginfo("  fall - 发送1次摔倒检测信号")
        rospy.loginfo("  drown - 发送1次溺水检测信号")
        rospy.loginfo("  fall2 - 发送2次摔倒检测信号（触发救援）")
        rospy.loginfo("  drown2 - 发送2次溺水检测信号（触发救援）")
        rospy.loginfo("")
        rospy.loginfo("载具控制命令:")
        rospy.loginfo("  fback - 无人车返回起点")
        rospy.loginfo("  dback - 无人船返回起点")
        rospy.loginfo("")
        rospy.loginfo("系统控制命令:")
        rospy.loginfo("  reset - 重置检测计数")
        rospy.loginfo("  status - 查看无人机状态")
        rospy.loginfo("  stop - 终止程序")
        rospy.loginfo("  help - 显示此帮助信息")
        rospy.loginfo("===============================")
        
    def show_quick_test_menu(self):
        """显示快速测试菜单"""
        rospy.loginfo("=== 快速测试菜单 ===")
        rospy.loginfo("完整流程测试:")
        rospy.loginfo("  test1 - 摔倒救援完整流程")
        rospy.loginfo("  test2 - 溺水救援完整流程")
        rospy.loginfo("  test3 - 载具返回起点测试")
        rospy.loginfo("==================")
        
    def run_fall_rescue_test(self):
        """运行摔倒救援完整流程测试"""
        rospy.loginfo("开始摔倒救援完整流程测试...")
        rospy.loginfo("步骤1: 重置系统")
        self.send_reset_command()
        time.sleep(1)
        
        rospy.loginfo("步骤2: 触发摔倒救援")
        self.trigger_fall_rescue()
        time.sleep(2)
        
        rospy.loginfo("步骤3: 查看系统状态")
        self.send_status_command()
        time.sleep(1)
        
        rospy.loginfo("步骤4: 发送无人车返回指令")
        self.send_fback_command()
        rospy.loginfo("摔倒救援完整流程测试完成")
        
    def run_drown_rescue_test(self):
        """运行溺水救援完整流程测试"""
        rospy.loginfo("开始溺水救援完整流程测试...")
        rospy.loginfo("步骤1: 重置系统")
        self.send_reset_command()
        time.sleep(1)
        
        rospy.loginfo("步骤2: 触发溺水救援")
        self.trigger_drown_rescue()
        time.sleep(2)
        
        rospy.loginfo("步骤3: 查看系统状态")
        self.send_status_command()
        time.sleep(1)
        
        rospy.loginfo("步骤4: 发送无人船返回指令")
        self.send_dback_command()
        rospy.loginfo("溺水救援完整流程测试完成")
        
    def run_vehicle_return_test(self):
        """运行载具返回起点测试"""
        rospy.loginfo("开始载具返回起点测试...")
        rospy.loginfo("步骤1: 无人车返回起点")
        self.send_fback_command()
        time.sleep(1)
        
        rospy.loginfo("步骤2: 无人船返回起点")
        self.send_dback_command()
        time.sleep(1)
        
        rospy.loginfo("步骤3: 查看系统状态")
        self.send_status_command()
        rospy.loginfo("载具返回起点测试完成")
        
    def run(self):
        """主循环"""
        try:
            while not rospy.is_shutdown():
                # 等待用户输入
                try:
                    command = input("请输入命令: ").strip().lower()
                    
                    if command == 'fall':
                        self.send_fall_detection()
                    elif command == 'drown':
                        self.send_drown_detection()
                    elif command == 'fall2':
                        self.trigger_fall_rescue()
                    elif command == 'drown2':
                        self.trigger_drown_rescue()
                    elif command == 'fback':
                        self.send_fback_command()
                    elif command == 'dback':
                        self.send_dback_command()
                    elif command == 'reset':
                        self.send_reset_command()
                    elif command == 'status':
                        self.send_status_command()
                    elif command == 'stop':
                        rospy.loginfo("收到指令，退出测试程序")
                        break
                    elif command == 'help':
                        self.show_help()
                    elif command == 'menu':
                        self.show_quick_test_menu()
                    elif command == 'test1':
                        self.run_fall_rescue_test()
                    elif command == 'test2':
                        self.run_drown_rescue_test()
                    elif command == 'test3':
                        self.run_vehicle_return_test()
                    else:
                        rospy.logwarn(f"无效命令: {command}")
                        rospy.loginfo("输入 'help' 查看支持的命令")
                        
                except (EOFError, KeyboardInterrupt):
                    rospy.loginfo("用户中断，退出程序")
                    break
                    
                self.rate.sleep()
                
        except rospy.ROSInterruptException:
            rospy.loginfo("程序被中断")
        finally:
            rospy.loginfo("测试程序结束")

if __name__ == '__main__':
    try:
        tester = RescueSystemTester()
        tester.run()
    except Exception as e:
        rospy.logfatal(f"测试程序异常: {e}")
        rospy.signal_shutdown("程序异常") 
