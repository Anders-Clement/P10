import psutil
import dataclasses
import time
import rclpy
from rclpy.node import Node
from spice_msgs.msg import Performance

@dataclasses.dataclass
class Rate:
    old_value: float = 0
    new_value: float = 0

    def update(self, new_value) -> float:
        self.old_value = self.new_value
        self.new_value = new_value
        return self.new_value - self.old_value

# Performance monitor for CPU usage and network statistics like usage and packet loss
class PerformanceMonitor:
    def __init__(self) -> None:
        self.bytes_sent_reading = Rate()
        self.bytes_receive_reading = Rate()
        self.packets_sent_reading = Rate()
        self.packets_receive_reading = Rate()
        self.packets_dropin_reading = Rate()
        self.packets_dropout_reading = Rate()

        self.get_readings()        

    # get performance readings, since last call to the function
    def get_readings(self) -> Performance:
        net_statistics = psutil.net_io_counters()

        cpu_usage = psutil.cpu_percent()
        packets_sent = self.packets_sent_reading.update(net_statistics.packets_sent)
        packets_receive = self.packets_receive_reading.update(net_statistics.packets_recv)
        packets_dropin = self.packets_dropin_reading.update(net_statistics.dropin + net_statistics.errin)
        packets_dropout = self.packets_dropout_reading.update(net_statistics.dropout + net_statistics.errout)
        bytes_sent = self.bytes_sent_reading.update(net_statistics.bytes_sent)
        bytes_receive = self.bytes_receive_reading.update(net_statistics.bytes_recv)

        packets_transmitted = packets_sent+packets_receive
        packets_dropped = packets_dropin+packets_dropout

        # avoid division by zero
        if packets_dropped == 0:
            packet_loss = 0.0
        else:
            packet_loss = packets_transmitted/packets_dropped

        return Performance(
            cpu_usage = cpu_usage,
            mega_bit_download = bytes_receive/1024./1024.*8, # bytes to megabit
            mega_bit_upload = bytes_sent/1024./1024.*8, # bytes to megabit
            packet_loss_percent = packet_loss,
            packets_sent = packets_sent,
            packets_sent_dropped=packets_dropout,
            packets_received = packets_receive,
            packets_received_dropped = packets_dropin
        )


class PerformanceMonitorNode(Node) :
    def __init__(self) -> None:
        super().__init__('performance_monitor_node')
        self.publish_timer = self.create_timer(1.0, self.publish_readings)
        self.performance_publisher = self.create_publisher(Performance, self.get_namespace()+'system_performance',10)
        self.performance_monitor = PerformanceMonitor()

    def publish_readings(self):
        reading = self.performance_monitor.get_readings()
        self.performance_publisher.publish(reading)


if __name__ == "__main__":
    rclpy.init()
    p = PerformanceMonitorNode()
    rclpy.spin(p)
    rclpy.shutdown()