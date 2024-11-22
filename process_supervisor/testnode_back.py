import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from ament_index_python.packages import get_package_share_directory

class DynamicLaunchManager(Node):
    def __init__(self):
        super().__init__('dynamic_launch_manager')

        # 設定ファイルの読み込み
        package_dir = get_package_share_directory("process_supervisor")
        process_config_file = os.path.join(package_dir, "config", 'process_config.json')
        print(process_config_file)
        with open(process_config_file, 'r') as f:
            self.process_config = json.load(f)['processes']

        self.current_process = None  # 現在のサブプロセス
        self.current_index = 0       # 現在のプロセスインデックス
        self.subscriber = None       # サブスクライバの初期化

        # 最初のプロセスを起動
        self.start_process(self.current_index)

    def start_process(self, index):
        """指定されたインデックスのプロセスを起動"""
        # サブプロセスの情報を取得
        process_info = self.process_config[index]

        # 現在のプロセスを終了
        self.stop_current_process()

        # 新しいプロセスを開始
        command = process_info['launch_command']
        self.get_logger().info(f"Starting {process_info['name']} with command: {command}")
        self.current_process = subprocess.Popen(command, shell=True, executable='/bin/bash')

        # トピックを監視するサブスクライバを登録
        self.current_flag_topic = process_info['flag_topic']
        if self.subscriber:
            self.subscriber.destroy()  # 古いサブスクライバを削除
        self.subscriber = self.create_subscription(
            String,
            self.current_flag_topic,
            self.flag_callback,
            10
        )
        self.get_logger().info(f"Now monitoring: {self.current_flag_topic}")

    def stop_current_process(self):
        """現在のプロセスを停止"""
        if self.current_process is not None:
            self.get_logger().info("Stopping current subprocess...")
            self.current_process.terminate()
            self.current_process.wait()
            self.current_process = None

    def flag_callback(self, msg):
        """現在のフラグがTrueになったら次のプロセスを起動"""
        if msg.data == 'true':
            self.get_logger().info(f"Flag received on {self.current_flag_topic}. Switching to next process...")
            # 現在のプロセスを停止
            self.stop_current_process()

            # 次のプロセスに切り替え
            self.current_index = (self.current_index + 1) % len(self.process_config)
            self.start_process(self.current_index)

    def destroy_node(self):
        """ノード終了時にプロセスをクリーンアップ"""
        self.get_logger().info("Destroying DynamicLaunchManager and cleaning up resources...")
        self.stop_current_process()
        if self.subscriber:
            self.subscriber.destroy()  # サブスクライバを削除
        super().destroy_node()

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
