import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
# from std_msgs.msg import String
from robotnik_msgs.msg import BatteryStatus
from influxdb import InfluxDBClient
import time
import sys
import os


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__(
            "battery_monitor",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.init_parameters()
        self.init_suscribers()
        self.init_vars()
        self.__setup_influxdb()

    def init_parameters(self):
        self.influxdb_host = os.getenv("INFLUXDB_HOST", 'localhost')
        self.influxdb_port = int(os.getenv("INFLUXDB_PORT", 8086))
        self.influxdb_user = os.getenv("INFLUXDB_USER", 'admin')
        self.influxdb_pass = os.getenv("INFLUXDB_PASS", 'admin')
        self.influxdb_db_name = os.getenv("INFLUXDB_DB_NAME", 'openwrt')
        self.influxdb = None
        self.influxdb_health = False

    def init_suscribers(self):
        self.battery_topic = "/robot/battery_estimator/data"
        self.subscription = self.create_subscription(
            BatteryStatus,
            self.battery_topic,
            self.battery_callback,
            10
        )
        self.subscription

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
        pass
        # self.cpu_temp_msg = Temperature()
        # self.cpu_load_msg = String()
        # self.cpu_temp_msg.header.frame_id = "CPU"
        # self.cpu_zone = self.find_cpu_zone()


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


    def battery_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.voltage)
        now = time.time_ns()
        if not self.influxdb_health:
            return False
        data = {
            "measurement": "battery",
            "tags": {
                "model": "daly",
            },
            "time": now,
            "fields": {
                "voltage": float(msg.voltage),
                "current": float(msg.current),
                "level": int(msg.level),
                "time_remaining": int(msg.time_remaining),
                "time_charging": int(msg.time_charging),
                "is_charging": bool(msg.is_charging),
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


def main(
    args=None
):
    rclpy.init(args=args)
    tracker = BatteryMonitor()
    while rclpy.ok():
        rclpy.spin(tracker)
    rclpy.shutdown()

