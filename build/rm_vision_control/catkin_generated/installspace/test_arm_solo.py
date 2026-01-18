#!/usr/bin/env python3
# test_arm_solo.py - 单独测试机械臂控制模块
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from rm_msgs.msg import MoveJ, MoveJ_P, MoveL, Plan_State

class ArmSoloTest:
    def __init__(self):
        rospy.init_node('arm_solo_test')
        
        # 发布者
        self.movej_pub = rospy.Publisher('/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
        self.movej_p_pub = rospy.Publisher('/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)
        self.movel_pub = rospy.Publisher('/rm_driver/MoveL_Cmd', MoveL, queue_size=10)
        
        # 状态跟踪
        self.plan_state = None
        rospy.Subscriber('/rm_driver/Plan_State', Plan_State, self.plan_state_cb)
        
        rospy.loginfo("Arm Solo Test initialized")
    
    def plan_state_cb(self, msg):
        """规划状态回调"""
        self.plan_state = msg.state
        rospy.loginfo(f"Plan State: {'Success' if msg.state else 'Failed'}")
    
    def test_movej(self):
        """测试MoveJ指令"""
        rospy.loginfo("=== Testing MoveJ ===")
        
        # 获取当前机械臂状态（通过rostopic echo手动记录）
        # 或使用一个已知的安全位置
        movej_cmd = MoveJ()
        movej_cmd.joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 零点位置
        movej_cmd.speed = 0.2
        movej_cmd.trajectory_connect = 0
        
        self.plan_state = None
        self.movej_pub.publish(movej_cmd)
        
        # 等待结果
        timeout = rospy.Duration(10)
        start = rospy.Time.now()
        while self.plan_state is None and (rospy.Time.now() - start) < timeout:
            rospy.sleep(0.1)
        
        return self.plan_state is True
    
    def test_movej_p_simple(self):
        """测试简单的MoveJ_P指令"""
        rospy.loginfo("=== Testing MoveJ_P (Simple) ===")
        
        # 创建一个简单的位姿（机械臂工作空间内）
        pose = Pose()
        pose.position.x = - 0.3
        pose.position.y = 0.0
        pose.position.z = 0.3
        pose.orientation.x = -0.983404
        pose.orientation.y = -0.178432
        pose.orientation.z = 0.032271
        pose.orientation.w =0.006129
        
        movej_p_cmd = MoveJ_P()
        movej_p_cmd.Pose = pose
        movej_p_cmd.speed = 0.1  # 低速测试
        movej_p_cmd.trajectory_connect = 0
        
        self.plan_state = None
        self.movej_p_pub.publish(movej_p_cmd)
        
        timeout = rospy.Duration(10)
        start = rospy.Time.now()
        while self.plan_state is None and (rospy.Time.now() - start) < timeout:
            rospy.sleep(0.1)
        
        return self.plan_state is True
    
    def test_movel_simple(self):
        """测试简单的MoveL指令"""
        rospy.loginfo("=== Testing MoveL (Simple) ===")
        
        # 使用与MoveJ_P相同的起始位置
        pose = Pose()
        pose.position.x = - 0.3
        pose.position.y = 0.0
        pose.position.z = 0.4
        pose.orientation.x = -0.983404
        pose.orientation.y = -0.178432
        pose.orientation.z = 0.032271
        pose.orientation.w =0.006129
        
        movel_cmd = MoveL()
        movel_cmd.Pose = pose
        movel_cmd.speed = 0.05  # 更低速
        movel_cmd.trajectory_connect = 0
        
        self.plan_state = None
        self.movel_pub.publish(movel_cmd)
        
        timeout = rospy.Duration(15)
        start = rospy.Time.now()
        while self.plan_state is None and (rospy.Time.now() - start) < timeout:
            rospy.sleep(0.1)
        
        return self.plan_state is True
    
    def run_tests(self):
        """运行所有测试"""
        rospy.sleep(2.0)  # 等待节点初始化
        
        # 测试顺序
        tests = [
            ("MoveJ", self.test_movej),
            ("MoveJ_P", self.test_movej_p_simple),
            ("MoveL", self.test_movel_simple)
        ]
        
        results = {}
        for test_name, test_func in tests:
            rospy.loginfo(f"\n{'='*50}")
            rospy.loginfo(f"Starting test: {test_name}")
            try:
                result = test_func()
                results[test_name] = result
                rospy.loginfo(f"{test_name}: {'PASS' if result else 'FAIL'}")
                rospy.sleep(2.0)  # 测试间隔
            except Exception as e:
                rospy.logerr(f"{test_name} error: {e}")
                results[test_name] = False
        
        # 输出测试总结
        rospy.loginfo(f"\n{'='*50}")
        rospy.loginfo("TEST SUMMARY:")
        for test_name, result in results.items():
            rospy.loginfo(f"  {test_name}: {'✓ PASS' if result else '✗ FAIL'}")
        
        return all(results.values())

if __name__ == '__main__':
    try:
        tester = ArmSoloTest()
        success = tester.run_tests()
        exit(0 if success else 1)
    except rospy.ROSInterruptException:
        pass