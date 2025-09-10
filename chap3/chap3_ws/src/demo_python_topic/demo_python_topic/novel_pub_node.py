import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
	def __init__(self, node_name: str, *, context: rclpy.Context | None = None, cli_args: rclpy.List[str] | None = None, namespace: str | None = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] | None = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
		super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
		self.novels_queue_ = Queue()

		self.novels_publisher_ = self.create_publisher(String, 'novel', 10)
		self.timer_ = self.create_timer(5, self.timer_callback)

	def download_novel(self, path):
		# 判断是网络路径还是本地路径
		if path.startswith('http'):
			# 网络路径逻辑（保持原有功能）
			response = requests.get(path)
			response.encoding = 'utf-8'
			self.get_logger().info(f'Download Finshed: {path}')
			for line in response.text.splitlines():
				self.novels_queue_.put(line)
		else:
			# 本地路径逻辑
			with open(path, 'r', encoding='utf-8') as file:
				self.get_logger().info(f'Read Finished: {path}')
				for line in file:
					# 去除行末换行符并添加到队列
					self.novels_queue_.put(line.rstrip('\n'))

	def timer_callback(self):
		if self.novels_queue_.qsize() > 0:
			msg = String()
			msg.data = self.novels_queue_.get()
			self.novels_publisher_.publish(msg)
			self.get_logger().info(f'Novel: {msg.data}')

def main():
    rclpy.init()
    node = NovelPubNode('novel_pub')
    # 修改为本地文件路径
    node.download_novel('/home/moco/FishRos2Study/chap3/notebook/part_1.txt')
    rclpy.spin(node)
    rclpy.shutdown()