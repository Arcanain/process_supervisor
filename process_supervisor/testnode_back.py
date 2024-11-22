import os
import subprocess
import signal
import rclpy
from rclpy.node import Node


class LaunchManager(Node):
    def __init__(self):
        super().__init__('launch_manager')
        self.process = None

        # Launchファイルを実行
        self.get_logger().info("Starting launch file...")
        self.start_launch_file()

        # 10秒後に終了するタイマー
        self.create_timer(10.0, self.stop_launch_file)

    def start_launch_file(self):
        """指定されたlaunchファイルを起動"""
        command = "bash -c 'source ~/ros2_ws/install/setup.bash && ros2 launch pure_pursuit_planner pure_pursuit_planner.py'"
        self.get_logger().info(f"Executing command: {command}")
        # サブプロセスを開始（bash環境を指定）
        self.process = subprocess.Popen(command, shell=True, executable='/bin/bash')
        self.get_logger().info("Launch file started.")

    def stop_launch_file(self):
        """サブプロセスを停止"""
        if self.process:
            self.get_logger().info("Sending SIGINT (Ctrl+C) to the subprocess...")
            try:
                # SIGINT を送信
                os.kill(self.process.pid, signal.SIGINT)
                self.process.wait()  # プロセスが終了するまで待つ
                self.get_logger().info("Launch file stopped.")
            except Exception as e:
                self.get_logger().error(f"Failed to stop subprocess: {e}")
            finally:
                self.process = None

            # ノードを終了
            #self.destroy_node()
            #rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LaunchManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DynamicLaunchManager.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()