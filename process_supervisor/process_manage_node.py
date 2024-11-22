import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import json
import signal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from ament_index_python.packages import get_package_share_directory

class DynamicLaunchManager(Node):
    def __init__(self):
        super().__init__('dynamic_launch_manager')
        # まずはうごくものでOK 
        
        ### CONFIGURATION ###
        # command of launch each process
        self._command_1 = "bash -c 'source ~/ros2_ws/install/setup.bash && ros2 launch pure_pursuit_planner pure_pursuit_planner.py'"
        self._command_2 = "bash -c 'source ~/ros2_ws/install/setup.bash && ros2 launch arcanain_simulator simulator.py'"
        self._command_3 = "bash -c 'source ~/ros2_ws/install/setup.bash && ros2 launch pure_pursuit_planner pure_pursuit_planner.py'"
        
        # topic config TODO: make config file
        # GPS
        self.ps_1_topic = "/swich_odom" # gps  /gnss_odom, to swich from gps to emcl
        self.ps_1_topic_type = Odometry
        # EMCL
        self.ps_2_topic = "mcl_pose" # emcl /mcl_pose,  to swich from emcl to gps  
        self.ps_2_topic_type = PoseWithCovarianceStamped

        # switch condition
        # GPS -> EMCL
        self.th_gps = 1.0
        self.target_x_1 = 0.0
        self.target_y_1 = 0.0
        # EMCL -> GPS
        self.th_emcl = 1.0
        self.target_x_2 = 0.0
        self.target_y_2 = 0.0
        ### ############ ###
    

        # subprocesses
        self.process_1 = None # start to the front of the roof: GPS
        self.process_2 = None # unter the roof: Lider, EMCL
        self.process_3 = None # roof to end: GPS


        ## Subscriber 
        # For process 1
        self.gps_subscriber = self.create_subscription(
            self.ps_1_topic_type, # TODO change
            self.ps_1_topic, # topic name
            self.ps_1_callback,
            10
            )
        
        # For process 2 
        self.subscriber = self.create_subscription(
            self.ps_2_topic_type,
            self.ps_2_topic,
            self.ps_2_callback,
            10
            )


        # 最初のプロセスを起動
        self.get_logger().info("Starting launch file...")
        self.start_process_1()
        

    # Process 1
    def start_process_1(self)->bool:
        """指定されたlaunchファイルを起動"""
        
        self.get_logger().info(f"Executing command: {self._command_1}")
        # サブプロセスを開始（bash環境を指定）
        self.process_1 = subprocess.Popen(self._command_1, shell=True, executable='/bin/bash')
        self.get_logger().info("Launch file started.")

        return True # TODO 成功判定

    def stop_process_1(self)->bool:
        '''terminate subprocess process 1'''
        if self.process_1:
            self.get_logger().info("Sending SIGINT (Ctrl+C) to the subprocess...")
            try:
                # SIGINT を送信
                os.kill(self.process_1.pid, signal.SIGINT)
                self.process_1.wait()  # プロセスが終了するまで待つ
                self.get_logger().info("Launch file stopped.")
                result = True
            except Exception as e:
                self.get_logger().error(f"Failed to stop subprocess: {e}")
                result = False
            finally:
                self.process_1 = None
                return result


    # Process 2
    def start_process_2(self)->bool:
        """指定されたlaunchファイルを起動"""

        self.get_logger().info(f"Executing command: {self._command_2}")
        # サブプロセスを開始（bash環境を指定）
        self.process_2 = subprocess.Popen(self._command_2, shell=True, executable='/bin/bash')
        self.get_logger().info("Launch file started.")

        return True  # TODO 成功判定

    def stop_process_2(self)->bool:
        '''terminate subprocess: process 2'''
        if self.process_2:
            self.get_logger().info("Sending SIGINT (Ctrl+C) to the subprocess...")
            try:
                # SIGINT を送信
                os.kill(self.process_2.pid, signal.SIGINT)
                self.process_2.wait()  # プロセスが終了するまで待つ
                self.get_logger().info("Launch file stopped.")
                result = True
            except Exception as e:
                self.get_logger().error(f"Failed to stop subprocess: {e}")
                result = False
            finally:
                self.process_2 = None
                return result


    # Process 3
    def start_process_3(self)->bool:
        """指定されたlaunchファイルを起動"""

        self.get_logger().info(f"Executing command: {self._command_3}")
        # サブプロセスを開始（bash環境を指定）
        self.process_3 = subprocess.Popen(self._command_3, shell=True, executable='/bin/bash')
        self.get_logger().info("Launch file started.")
        
        return True # TODO 起動の成功判定


    def ps_1_callback(self,msg:Odometry):
        '''
        条件True: call stop process 1, after that, call launch_process2 
        False: 何もしない
        '''
         # メッセージから座標を取得
        current_x_gps = msg.pose.pose.position.x
        current_y_gps = msg.pose.pose.position.y
        
        # ユークリッド距離を計算
        distance = ((current_x_gps - self.target_x_1) ** 2 + (current_y_gps - self.target_y_1) ** 2) ** 0.5
        #Reach = [True if distance <= 1.0 else False]

        # TODO condition check
        if  distance <= self.th_gps and (self.process_1 is not None):
            self.get_logger().info(f"Flag received on {self.ps_1_topic}. Switching to next process...")
            stop_result = self.stop_process_1()
            if stop_result:  # stop_process_1 Sucess
                self.start_process_2()




    def ps_2_callback(self,msg:PoseWithCovarianceStamped):
        '''
        条件True: call stop process 2, after that, call launch_process_3
        False: 何もしない
        '''
        # TODO condition check

        current_x_emcl = msg.pose.pose.position.x
        current_y_emcl = msg.pose.pose.position.y

        # ユークリッド距離を計算
        distance = ((current_x_emcl - self.target_x_2) ** 2 + (current_y_emcl - self.target_y_2) ** 2) ** 0.5
        #Reach = [True if distance <= 1.0 else False]

        if  distance <= self.th_emcl and (self.process_2 is not None):
            self.get_logger().info(f"Flag received on {self.ps_2_topic}. Switching to next process...")
            stop_result = self.stop_process_2()
            if stop_result:  # stop_process_1 Sucess
                self.start_process_3()
                # TODO error handling, stop_result == False, then,,,,?



    



def main(args=None):
    rclpy.init(args=args)
    node = DynamicLaunchManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DynamicLaunchManager.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
