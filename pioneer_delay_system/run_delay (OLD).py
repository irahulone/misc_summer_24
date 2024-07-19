import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import statistics

'''
This program implements the controller delay using a linked list to increase efficiency
It can run delays from 0.1 to 15+ seconds with a ~16ms margin of error
'''

#Node is already used for ROS so LL node is listNode
class listNode:
    def __init__(self, linear_x, angular_z, timestamp):
        self.linear_x = linear_x
        self.angular_z = angular_z
        self.timestamp = timestamp
        self.next = None
        self.prev = None
class DoublyLinkedList:
    def __init__(self):
        self.head = None
        self.tail = None
    def append(self, linear_x, angular_z, timestamp):
        new_node = listNode(linear_x, angular_z, timestamp)
        if not self.head:
            self.head = new_node
            self.tail = new_node
        else:
            self.tail.next = new_node
            new_node.prev = self.tail
            self.tail = new_node

    #function that updates the head node to the closest node to the target time
    def update(self, delay):
        #get the current time
        current_time = time.perf_counter()
        current_node = self.head
        closest_node = current_node

        #iterate starting at head (oldest) until reach closest node to
        while current_node:
            #if time difference is less than delay, return the previous node as closest
            if (current_time - current_node.timestamp) < delay:
                #if the absolute val of node is closer than current closest, set current as closest
                if (abs(delay - current_time - current_node.timestamp) < (abs(delay - current_time - closest_node.timestamp))):
                    closest_node = current_node
                break
            closest_node = current_node
            current_node = current_node.next
        
        #remove all outdated nodes by setting head to found closest node
        if closest_node:
            self.head = closest_node
            self.head.prev = None
        return closest_node

    def get_head(self):
        return self.head
    
    def get_tail(self):
        return self.tail
    
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('delay_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_td_cmd_vel = self.create_publisher(Twist, 'mr1/td_cmd_vel', 1)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.delay = 1  # Delay in seconds, set delay here
        self.linked_list = DoublyLinkedList()

        # #optimization stats
        # self.delay_durations = []   #store the delay difference of each delay signal for statistics
        # self.collect_data = False

        # #start collecting data after buffer has been populated
        # self.data_collecton_timer = self.create_timer(self.delay*2, self.start_collecting_data)

        self.subscription = self.create_subscription(
            Twist,
            'mr1/cmd_vel',
            self.joy_callback,
            1)
        self.subscription  # prevent unused variable warning
    def joy_callback(self, msg):
        current_time = time.perf_counter()

        self.rt_lx = msg.linear.x
        self.rt_az = msg.angular.z

        #self.get_logger().info('real time "%s"' % msg.linear.x)
        #print('real time %s' % self.rt_lx)

        self.linked_list.append(self.rt_lx, self.rt_az, current_time)

    # #function that is called after 'delay' seconds pass to start collecting data
    # def start_collecting_data(self):
    #     self.collect_data = True
    #     print('\n Data Collection Start \n')
    #     self.data_collecton_timer.cancel()

    def timer_callback(self):
        # #get the most recently added node to 'represent' the real time signal at a known frequency of timer_period
        # tail_node = self.linked_list.get_tail()
        # if tail_node:
        #     close_rt_vel = Twist()
        #     close_rt_vel.linear.x = tail_node.linear_x
        #     close_rt_vel.angular.z = tail_node.angular_z
        #     print('%s (real time)' % close_rt_vel.linear.x)

        #call returns closest node to target delay and removes all prev nodes
        delayed_node = self.linked_list.update(self.delay)

        #get current time after function call for more accurate time
        current_time = time.perf_counter()

        if delayed_node:
            td_vel = Twist()
            td_vel.linear.x = delayed_node.linear_x
            td_vel.angular.z = delayed_node.angular_z

            #print('%s (delayed)' % td_vel.linear.x)

            self.publisher_td_cmd_vel.publish(td_vel)

            delay_duration = current_time - delayed_node.timestamp
            #print('delay_duration: %f' % delay_duration)

            # #Data Collection Code
            # #collect delay duration data once 'delay' seconds have passed
            # if self.collect_data:
            #     self.delay_durations.append(delay_duration)
            #     if len(self.delay_durations) >= 500:
            #         mean_delay = statistics.mean(self.delay_durations)
            #         stdev_delay = statistics.stdev(self.delay_durations)
            #         max_delay = max(self.delay_durations)
            #         min_delay = min(self.delay_durations)
            #         print('mean delay duration: %f' % mean_delay)
            #         print('stdev delay duration: %f' % stdev_delay)
            #         print('max delay: %f' % max_delay)
            #         print('min delay: %f' % min_delay)
            #         #terminate program once reached 500 signals
            #         rclpy.shutdown()
            #self.get_logger().info('time delay value: "%s", delay duration: "%f" seconds' % (td_vel.linear.x, delay_duration))
            #print(" ")
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

