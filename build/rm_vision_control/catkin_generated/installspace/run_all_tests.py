#!/usr/bin/env python3
# run_all_tests.py - 运行所有测试
import subprocess
import os
import time
import sys

class TestRunner:
    def __init__(self):
        self.results = {}
        self.test_sequence = [
            ("Arm Control", "python test_arm_solo.py"),
            ("Camera", "python test_camera_solo.py"),
            ("Vision", "python test_vision_solo.py"),
            ("Arm-Camera Integration", "python test_arm_camera_integration.py"),
            ("Task 1 Full", "python test_task1_full.py")
        ]
    
    def run_test(self, name, command):
        """运行单个测试"""
        print(f"\n{'='*60}")
        print(f"Running: {name}")
        print(f"Command: {command}")
        print('='*60)
        
        try:
            # 运行测试命令
            result = subprocess.run(
                command, 
                shell=True,
                capture_output=True,
                text=True,
                timeout=120  # 2分钟超时
            )
            
            # 打印输出
            if result.stdout:
                print(result.stdout)
            if result.stderr:
                print(f"STDERR: {result.stderr}")
            
            # 检查结果
            success = (result.returncode == 0)
            self.results[name] = success
            
            status = "✓ PASS" if success else "✗ FAIL"
            print(f"\n{name}: {status}")
            
            return success
            
        except subprocess.TimeoutExpired:
            print(f"\n{name}: ✗ TIMEOUT (exceeded 2 minutes)")
            self.results[name] = False
            return False
        except Exception as e:
            print(f"\n{name}: ✗ ERROR: {e}")
            self.results[name] = False
            return False
    
    def print_summary(self):
        """打印测试总结"""
        print(f"\n{'='*60}")
        print("TEST SUITE SUMMARY")
        print('='*60)
        
        passed = 0
        total = len(self.results)
        
        for name, success in self.results.items():
            status = "✓ PASS" if success else "✗ FAIL"
            print(f"{name:30} {status}")
            if success:
                passed += 1
        
        print('-'*60)
        print(f"Total: {passed}/{total} tests passed ({passed/total*100:.1f}%)")
        
        if passed == total:
            print("\n✓ ALL TESTS PASSED!")
            return True
        else:
            print("\n✗ SOME TESTS FAILED")
            return False
    
    def run_all(self):
        """运行所有测试"""
        print("Starting comprehensive test suite...")
        print("Make sure ROS master is running (roscore)")
        
        all_passed = True
        for name, command in self.test_sequence:
            success = self.run_test(name, command)
            all_passed = all_passed and success
            
            # 测试间隔
            if success:
                time.sleep(2.0)
            else:
                # 失败时暂停，允许用户检查
                input(f"\n{name} failed. Press Enter to continue or Ctrl+C to abort...")
        
        final_success = self.print_summary()
        
        if final_success:
            print("\nReady for full system deployment!")
            print("Next steps:")
            print("1. Update actual pose coordinates in arm_controller.py")
            print("2. Configure camera calibration parameters")
            print("3. Load trained YOLOv8 models")
            print("4. Run: roslaunch rm_vision_control vision_control.launch")
        else:
            print("\nSome tests failed. Please fix issues before proceeding.")
        
        return final_success

if __name__ == '__main__':
    runner = TestRunner()
    success = runner.run_all()
    sys.exit(0 if success else 1)