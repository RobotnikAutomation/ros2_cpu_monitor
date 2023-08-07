import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ping3 import ping
import iperf3


class NetworkMonitor(Node):
    def __init__(self):
        super().__init__(
            "cpu_monitor",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        self.iperf = None
        self.init_parameters()
        self.init_publishers()
        self.init_vars()
        self.timer = self.create_timer(
            timer_period_sec=self.publish_rate,
            callback=self.publish_net_stats
        )

    def init_parameters(self):
        self.publish_rate = 1.0
        self.edge_ip = '10.10.10.212'
        self.iperf_port = 5201

    def __setup_iperf(self):
        self.iperf.duration = 1
        self.iperf.server_hostname = self.edge_ip
        self.iperf.port = self.iperf_port
        self.iperf.protocol = 'udp'
        self.iperf.zerocopy = True

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
        edge_latency = ping(self.edge_ip, unit='ms', timeout=0.800)
        self.get_logger().info(f"edge edge_latency: {edge_latency}")
        self.edge_latency_msg.data = str(edge_latency)

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
                self.edge_throughput_msg.data = throughput
                self.get_logger().info("iperf results:")
                self.get_logger().info(throughput)

        self.iperf = None

    def publish_net_stats(self):
        self.get_edge_latency()
        self.get_edge_thoughput()
        self.edge_latency_publisher.publish(self.edge_latency_msg)
        self.edge_throughput_publisher.publish(self.edge_throughput_msg)


def main(
    args=None
):
    rclpy.init(args=args)
    tracker = NetworkMonitor()
    while rclpy.ok():
        rclpy.spin(tracker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
