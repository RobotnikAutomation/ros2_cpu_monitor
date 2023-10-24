import time
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ping3 import ping
import iperf3
from influxdb import InfluxDBClient
import os


class NetworkMonitor(Node):
    def __init__(
        self,
        node_name="network_monitor",
        latency=True,
        throughput=True,
    ):
        super().__init__(
            node_name=node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.iperf = None
        self.__provide_latency = latency
        self.__provide_throughput = throughput
        self.init_parameters()
        self.init_publishers()
        self.init_vars()
        self.__setup_influxdb()
        self.timer = self.create_timer(
            timer_period_sec=self.publish_rate,
            callback=self.publish_net_stats
        )

    def init_parameters(self):
        self.publish_rate = 1.0
        self.edge_ip = os.getenv("EDGE_IP", "10.10.10.212")
        self.iperf_port = os.getenv("IPERF3_PORT", 5201)
        self.iperf_port = int(os.getenv("IPERF3_PORT", 5201))
        self.influxdb_host = os.getenv("INFLUXDB_HOST", 'localhost')
        self.influxdb_port = int(os.getenv("INFLUXDB_PORT", 8086))
        self.influxdb_user = os.getenv("INFLUXDB_USER", 'admin')
        self.influxdb_pass = os.getenv("INFLUXDB_PASS", 'admin')
        self.influxdb_db_name = os.getenv("INFLUXDB_DB_NAME", 'openwrt')
        self.influxdb = None
        self.influxdb_health = False

    def __setup_iperf(self):
        if not self.__provide_throughput:
            pass
        self.iperf.duration = 1
        self.iperf.server_hostname = self.edge_ip
        self.iperf.port = self.iperf_port
        self.iperf.protocol = 'tcp'
        self.iperf.zerocopy = True
        #self.iperf.num_streams = 1
        #self.iperf.blksize = 1024

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

    def init_publishers(self):
        self.edge_latency_topic = "edge_latency"
        self.iperf_mbps_topic = "edge_throughput"
        self.edge_latency_publisher = self.create_publisher(
            String,
            self.edge_latency_topic,
            10
        )
        self.edge_throughput_publisher = self.create_publisher(
            String,
            self.iperf_mbps_topic,
            10
        )

    def init_vars(self):
        self.edge_latency_msg = String()
        self.edge_throughput_msg = String()

    def get_edge_latency(self):
        now = time.time_ns()
        unit = 'ms'
        timeout = 0.800
        edge_latency = ping(
            self.edge_ip,
            unit=unit,
            timeout=timeout,
            ttl=5,
        )
        self.get_logger().info(f"edge latency: {edge_latency}")
        self.edge_latency_msg.data = str(edge_latency)
        if not edge_latency:
            self.get_logger().warning(
                f"could not get latency from ${self.edge_ip}"
            )
            return False
        if not self.influxdb_health:
            return False
            self.get_logger().warning(
                f"could not get latency from ${self.edge_ip}"
            )
        data = {
            "measurement": "latency",
            "tags": {
                "unit": unit,
                "timeout": timeout,
                "url": self.edge_ip,
            },
            "time": now,
            "fields": {
                "latency": float(edge_latency),
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

    def __parse_tcp(self, results):
        server = str(results.remote_host)
        throughput = ""
        throughput += f"thrp: {results.sent_MB_s}MB/s\n"
        throughput += f"thrp-down: {results.received_MB_s}MB/s\n"
        throughput += f"retransmits: {results.retransmits}%\n"
        throughput += f"srv:  {server}\n"
        throughput += f"prot: {results.protocol}\n"
        throughput += f"dur: {results.duration}s\n"
        return throughput

    def __parse_udp(self, results):
        sent_mb_s = str(results.MB_s)
        lost_per = str(results.lost_percent)
        server = str(results.remote_host)
        throughput = ""
        throughput += f"thrp: {sent_mb_s}MB/s\n"
        throughput += f"lost: {lost_per}%\n"
        throughput += f"pkgs:  {results.packets}\n"
        throughput += f"srv:  {server}\n"
        throughput += f"prot: {results.protocol}\n"
        throughput += f"dur: {results.duration}s\n"
        throughput += f"jit: {results.jitter_ms}ms\n"
        return throughput

    def __iperf_influx(self, results):
        if not self.influxdb_health:
            pass
        if results.protocol == 'UDP':
            data = self.__iperf_influx_udp(results)
        if results.protocol == 'TCP':
            data = self.__iperf_influx_tcp(results)
        json_payload = []
        data['fields']['size'] = sys.getsizeof(str(data))
        json_payload.append(data)
        try:
            self.influxdb.write_points(json_payload)
        except Exception as e:
            self.get_logger().error(
                f"Error writing throughput data to InfluxDB: {e}"
            )

    def __iperf_influx_tcp(self, results):
        if not self.influxdb_health:
            pass
        data = {
            "measurement": "throughput",
            "tags": {
                "unit": "MB/s",
                "protocol": results.protocol,
                "port": results.remote_port,
                "url": str(results.remote_host),
                "duration": results.duration,
            },
            "time": results.time,
            "fields": {
                "throughput": float(results.sent_MB_s),
                "throughput-upload": float(results.sent_MB_s),
                "throughput-download": float(results.received_MB_s),
                "retransmits": float(results.retransmits),
            }
        }
        return data

    def __iperf_influx_udp(self, results):
        if not self.influxdb_health:
            pass
        data = {
            "measurement": "throughput",
            "tags": {
                "unit": "MB/s",
                "protocol": results.protocol,
                "port": results.remote_port,
                "url": str(results.remote_host),
                "duration": results.duration,
            },
            "time": results.time,
            "fields": {
                "throughput": float(results.MB_s),
                "lost_percent": float(results.lost_percent)
            }
        }
        return data

    def __parse_iperf_results(self, results):
        print(results.protocol)
        if results.protocol == 'UDP':
            throughput = self.__parse_udp(results)
        if results.protocol == 'TCP':
            throughput = self.__parse_tcp(results)
        self.edge_throughput_msg.data = throughput
        self.get_logger().info("iperf results:")
        self.get_logger().info(throughput)
        self.__iperf_influx(results)

    def get_edge_thoughput(self):
        self.iperf = iperf3.Client()
        self.__setup_iperf()
        try:
            results = self.iperf.run()
        except OSError as bad:
            self.get_logger().info(bad)
        finally:
            if results.error:
                self.get_logger().info(results.error)
                self.edge_throughput_msg.data = "ERROR"
                self.get_logger().info("iperf error")
            else:
                self.__parse_iperf_results(results)
        self.iperf = None

    def publish_net_stats(self):
        if self.__provide_latency:
            self.get_edge_latency()
            self.edge_latency_publisher.publish(self.edge_latency_msg)
        if self.__provide_throughput:
            self.get_edge_thoughput()
            self.edge_throughput_publisher.publish(self.edge_throughput_msg)


def main(
    args=None
):
    rclpy.init(args=args)
    tracker = NetworkMonitor()
    while rclpy.ok():
        rclpy.spin(tracker)
    rclpy.shutdown()


def latency(
    args=None
):
    rclpy.init(args=args)
    tracker = NetworkMonitor(
        node_name="latency_monitor",
        latency=True,
        throughput=False,
    )
    while rclpy.ok():
        rclpy.spin(tracker)
    rclpy.shutdown()


def throughput(
    args=None
):
    rclpy.init(args=args)
    tracker = NetworkMonitor(
        node_name="throughput_monitor",
        latency=False,
        throughput=True,
    )
    while rclpy.ok():
        rclpy.spin(tracker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
