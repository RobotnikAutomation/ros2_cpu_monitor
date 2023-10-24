import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String
from os.path import exists
import psutil
from influxdb import InfluxDBClient
import time
import statistics
import sys
import os


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
        self.__setup_influxdb()
        self.timer = self.create_timer(
            timer_period_sec=self.publish_rate,
            callback=self.publish_cpu_stats
        )

    def init_parameters(self):
        self.cpu_id = "x86_pkg_temp"
        # self.cpu_id = self.get_parameter(
        #     "cpu_type_id"
        # ).get_parameter_value().string_value
        self.publish_rate = 1.0
        # self.publish_rate = self.get_parameter(
        #     "publish_rate",
        # ).get_parameter_value().double_value
        self.iperf_port = 5201
        self.influxdb_host = 'localhost'
        self.influxdb_port = 8086
        self.influxdb_user = 'admin'
        self.influxdb_pass = 'admin'
        self.influxdb_db_name = 'openwrt'
        self.influxdb = None
        self.influxdb_health = False

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

    def __setup_influxdb(self):
        self.influxdb = InfluxDBClient(
            host=self.influxdb_host,
            port=self.influxdb_port,
            username=self.influxdb_user,
            password=self.influxdb_pass,
            database=self.influxdb_db_name,
        )
        return self.__get_influxdb_health()

    def __get_influxdb_health(self):
        status = False
        try:
            self.influxdb.ping()
            status = True
            self.influxdb_health = True
            self.get_logger().info(
                "influxdb connected"
            )
        except Exception as e:
            self.get_logger().error(
                f"Influxdb Connection failure: {e}!"
            )
        finally:
            self.influxdb.close()
        return status

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
        if self.cpu_zone == -1:
            return False
        temperature_file_path = f'/sys/class/thermal/thermal_zone{str(self.cpu_zone)}/temp'
        temperature_file = open(temperature_file_path, "r")
        temperature = int(temperature_file.read().strip())
        temperature_file.close()
        now = time.time_ns()
        self.cpu_temp_msg.temperature = float(temperature / 1000)
        if not self.influxdb_health:
            return False
        data = {
            "measurement": "cpu",
            "tags": {
                "unit": "Celsius",
            },
            "time": now,
            "fields": {
                "temperature": self.cpu_temp_msg.temperature,
            }
        }
        json_payload = []
        data['fields']['size'] = sys.getsizeof(str(data))
        json_payload.append(data)
        try:
            self.influxdb.write_points(json_payload)
        except Exception as e:
            self.get_logger().error(
                f"Error writing latency data to InfluxDB: {e}"
            )
        return True

    def get_cpu_load(self):
        cpu_load = psutil.cpu_percent(interval=0.5, percpu=True)
        self.get_logger().info(f"cpu load: {cpu_load}")
        self.cpu_load_msg.data = str(cpu_load)
        now = time.time_ns()
        cpu_load_average = statistics.mean(cpu_load)
        if not self.influxdb_health:
            return False
        fields = {f'load_cpu_{i}': cpu_load[i] for i in range(len(cpu_load))}
        fields["load_average"] = cpu_load_average
        data = {
            "measurement": "cpu",
            "tags": {
                "unit": "percent",
            },
            "time": now,
            "fields": fields,
        }
        json_payload = []
        data['fields']['size'] = sys.getsizeof(str(data))
        json_payload.append(data)
        try:
            self.influxdb.write_points(json_payload)
        except Exception as e:
            self.get_logger().error(
                f"Error writing latency data to InfluxDB: {e}"
            )
        return True

    def publish_cpu_stats(self):
        self.get_cpu_temperature()
        self.get_cpu_load()
        self.cpu_temp_publisher.publish(self.cpu_temp_msg)
        self.cpu_load_publisher.publish(self.cpu_load_msg)


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
