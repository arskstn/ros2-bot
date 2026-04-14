#!/usr/bin/env python3
import sys
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


LINEAR_SPEED  = 0.3
ANGULAR_SPEED = 0.5
FRONT_THRESH  = 0.55  # расстояние до стены спереди при котором тормозим
WALL_CLOSE    = 0.35  # слишком близко к левой стене
WALL_FAR      = 1.0   # стена слева пропала (ширина коридора ~0.7м в центре)
EXIT_DIST     = 2.5   # все стороны свободны — выход
MIN_TRAVEL    = 6.0   # минимум метров пути до проверки выхода

SEARCHING     = 'searching'
FOLLOWING     = 'following'
TURNING_LEFT  = 'turning_left'


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')
        self.pub      = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.state     = SEARCHING
        self.done      = False
        self.travelled = 0.0
        self.prev_pos  = None
        self.get_logger().info('Старт. Ctrl+C для остановки.')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.prev_pos is not None:
            dx = x - self.prev_pos[0]
            dy = y - self.prev_pos[1]
            self.travelled += math.sqrt(dx * dx + dy * dy)
        self.prev_pos = (x, y)

    def sector_min(self, msg, center_deg, half_deg):
        center = math.radians(center_deg)
        half   = math.radians(half_deg)
        result = float('inf')
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle - center) <= half and math.isfinite(r) and r > 0.01:
                result = min(result, r)
        return result

    def scan_cb(self, msg):
        if self.done:
            return

        front = self.sector_min(msg,   0, 25)
        left  = self.sector_min(msg,  90, 25)
        right = self.sector_min(msg, -90, 25)

        # Выход: открытое пространство со всех сторон, только после MIN_TRAVEL
        if (self.travelled > MIN_TRAVEL
                and front > EXIT_DIST
                and left  > EXIT_DIST
                and right > EXIT_DIST):
            self.stop()
            self.get_logger().info(
                f'Лабиринт пройден! Пройдено {self.travelled:.1f} м.')
            self.done = True
            return

        twist = Twist()

        if front < FRONT_THRESH:
            # Стена впереди → поворот направо на месте, сброс состояния
            self.state         = SEARCHING
            twist.linear.x     = 0.0
            twist.angular.z    = -ANGULAR_SPEED

        elif self.state == TURNING_LEFT:
            # Доворачиваем влево пока не найдём левую стену
            if left < WALL_FAR:
                self.state = FOLLOWING
            else:
                twist.linear.x  = LINEAR_SPEED * 0.4
                twist.angular.z = ANGULAR_SPEED

        elif left < WALL_FAR:
            # Левая стена найдена → следуем вдоль
            self.state = FOLLOWING
            if left < WALL_CLOSE:
                # Слишком близко → корректируем вправо
                twist.linear.x  = LINEAR_SPEED * 0.5
                twist.angular.z = -ANGULAR_SPEED * 0.4
            else:
                # Хорошее расстояние → прямо
                twist.linear.x  = LINEAR_SPEED
                twist.angular.z = 0.0

        elif self.state == FOLLOWING:
            # Следовали, стена пропала → поворот налево (угол коридора)
            self.state         = TURNING_LEFT
            twist.linear.x     = LINEAR_SPEED * 0.4
            twist.angular.z    = ANGULAR_SPEED

        else:
            # SEARCHING: стен нет → едем прямо
            twist.linear.x  = LINEAR_SPEED
            twist.angular.z = 0.0

        self.pub.publish(twist)

    def stop(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = MazeSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nПрерывание пользователем. Остановка...')
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
