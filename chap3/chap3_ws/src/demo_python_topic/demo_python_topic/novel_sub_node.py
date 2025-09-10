import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import threading
from queue import Queue
import time
import espeakng

class NovelSubNode(Node):
    def __init__(self, node_name: str, *, context: rclpy.Context | None = None, cli_args: rclpy.List[str] | None = None, namespace: str | None = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: rclpy.List[rclpy.Parameter] | None = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.novels_queue_ = Queue()
        self.novel_subscriber_ = self.create_subscription(String, 'novel', self.novel_callback, 10)
        self.speech_thread_ = threading.Thread(target=self.speak_thread)
        self.speech_thread_.start()

    def novel_callback(self, msg):
        self.novels_queue_.put(msg.data)

    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'zh'  # type: ignore
        while rclpy.ok():
            if self.novels_queue_.qsize() > 0:
                text = self.novels_queue_.get()
                self.get_logger().info(f'正在朗读 {text}')
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = NovelSubNode("novel_read")
    rclpy.spin(node)
    rclpy.shutdown()