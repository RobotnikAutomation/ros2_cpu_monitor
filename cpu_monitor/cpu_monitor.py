import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String
from os.path import exists
import psutil


class CPUMonitor(Node):
    def __init__(self):
        super().__init__(
            "cpu_monitor",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.init_parameters()
        self.init_publishers()
        self.init_vars()
        self.timer = self.create_timer(1, self.publish_cpu_stats)

    def init_parameters(self):
        self.cpu_id = "x86_pkg_temp"
        # self.cpu_id = self.get_parameter(
        #     "cpu_type_id"
        # ).get_parameter_value().string_value
        self.publish_rate = 1.0
        # self.publish_rate = self.get_parameter(
        #     "publish_rate",
        # ).get_parameter_value().double_value



    def init_publishers(self):
        self.cpu_temp_output_topic = "cpu_temperature"
        self.cpu_load_output_topic = "cpu_load"
        # self.cpu_temp_output_topic = self.get_parameter(
        #     "cpu_temp_output_topic"
        # ).get_parameter_value().string_value
        self.cpu_temp_publisher = self.create_publisher(
            Temperature,
            self.cpu_temp_output_topic,
            10
        )
        self.cpu_load_publisher = self.create_publisher(
            String,
            self.cpu_load_output_topic,
            10
        )

    def init_vars(self):
        self.cpu_temp_msg = Temperature()
        self.cpu_load_msg = String()
        self.cpu_temp_msg.header.frame_id = "CPU"
        self.cpu_zone = self.find_cpu_zone()

    def find_cpu_zone(self):
        i = 0
        while(exists(f'/sys/class/thermal/thermal_zone{str(i)}/type')):
            file = open(f'/sys/class/thermal/thermal_zone{str(i)}/type', "r")
            data = file.read().strip()
            if data == self.cpu_id:
                return i
            else:
                i += 1
        self.get_logger().warning(
            "No CPU Zone Found. Unable to read temperature."
        )
        return -1

    def get_cpu_temperature(self):
        if self.cpu_zone != -1:
            temperature_file_path = f'/sys/class/thermal/thermal_zone{str(self.cpu_zone)}/temp'
            temperature_file = open(temperature_file_path, "r")
            temperature = int(temperature_file.read().strip())
            temperature_file.close()
            self.cpu_temp_msg.temperature = float(temperature / 1000)

    def get_cpu_load(self):
        cpu_load = psutil.cpu_percent(interval=0.5)
        self.cpu_load.string = str(cpu_load)

    def publish_cpu_stats(self):
        self.get_cpu_temperature()
        self.get_cpu_load()
        self.cpu_temp_publisher.publish(self.cpu_temp_msg)
        self.cpu_load_publisher.publish(self.cpu_load)


def main(
    args=None
):
    rclpy.init(args=args)
    tracker = CPUMonitor()
    while rclpy.ok():
        rclpy.spin(tracker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
