import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from os.path import exists


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
        self.timer = self.create_timer(1, self.publish_temperatures)

    def init_parameters(self):
        self.publish_cpu_temperature = self.get_parameter_or(
            "publish_cpu_temperature",
            True
        ).get_parameter_value().bool_value
        self.cpu_id = self.get_parameter_or(
            "cpu_type_id",
            "x86_pkg_temp"
        ).get_parameter_value().string_value
        self.publish_rate = self.get_parameter_or(
            "publish_rate",
            "1"
        ).get_parameter_value().double_value

    def init_publishers(self):
        if self.publish_cpu_temperature:
            self.cpu_output_topic = self.get_parameter_or(
                "cpu_output_topic",
                "cpu_temperature"
            ).get_parameter_value().string_value
            self.cpu_publisher = self.create_publisher(
                Temperature,
                self.cpu_output_topic,
                10
            )
        if not self.publish_cpu_temperature:
            self.get_logger().warning(
                "Not publishing CPU. Is this intentional?"
            )

    def init_vars(self):
        if self.publish_cpu_temperature:
            self.cpu_temp_msg = Temperature()
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

    def get_gpu_temperature(self):
        self.gpu_temp_msg.temperature = self.gpu.temperature

    def publish_temperatures(self):
        if self.publish_cpu_temperature:
            self.get_cpu_temperature()
            self.cpu_publisher.publish(self.cpu_temp_msg)
        if self.publish_gpu_temperature:
            self.get_gpu_temperature()
            self.gpu_publisher.publish(self.gpu_temp_msg)


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
