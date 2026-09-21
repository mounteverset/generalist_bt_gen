#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_srvs.srv import Trigger


def format_temperature(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError('temperature must be finite')
    return f'{value:.3f} °C'


class BlueBoatTemperatureService(Node):
    def __init__(self) -> None:
        super().__init__('blueboat_temperature_service')
        topic = self.declare_parameter(
            'temperature_topic', '/green/temperature'
        ).value
        service = self.declare_parameter(
            'service_name', '/green/read_temp_cached'
        ).value
        self._temperature = None
        self._topic = str(topic)
        self.create_subscription(Temperature, self._topic, self._on_temperature, 10)
        self.create_service(Trigger, str(service), self._on_read_temperature)
        self.get_logger().info(
            f'Caching {self._topic}; serving the latest value on {service}'
        )

    def _on_temperature(self, message: Temperature) -> None:
        try:
            format_temperature(message.temperature)
        except ValueError:
            self.get_logger().warning('Ignoring non-finite temperature sample')
            return
        self._temperature = float(message.temperature)

    def _on_read_temperature(self, _request, response):
        if self._temperature is None:
            response.success = False
            response.message = f'No temperature received on {self._topic}'
            return response
        response.success = True
        response.message = format_temperature(self._temperature)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueBoatTemperatureService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
