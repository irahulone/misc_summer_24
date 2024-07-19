import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import statistics

'''
This program implements the controller delay using a linked list to increase efficiency
It uses a predefined 'refresh rate' to standardize the controller input which improves reliablilty
but at the cost of losing some input signals (which can lead to incosistencies in input signal 
and output signal) if a stream comes in faster than the refresh rate.
Continued optimizations should improve the refresh rate to account for this issue.
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
        self.size = 0

    def append(self, linear_x, angular_z, timestamp):
        new_node = listNode(linear_x, angular_z, timestamp)
        if not self.head:
            self.head = new_node
            self.tail = new_node
        else:
            self.tail.next = new_node
            new_node.prev = self.tail
            self.tail = new_node
        self.size += 1
    
    def remove_head(self):
        if self.head:
            self.head = self.head.next
            if self.head:
                self.head.prev = None
            else:
                self.tail = None
            self.size -= 1

    def get_head(self):
        return self.head
    
    def get_tail(self):
        return self.tail
    def get_size(self):
        return self.size

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('delay_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher_td_cmd_vel = self.create_publisher(Twist, 'mr1/td_cmd_vel', 1)
        timer_period = 0.05  # seconds (only works consistenly at 0.05 seconds)

        self.timer_period_copy = timer_period  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.delay = 0.1  # Delay in seconds, set delay here
        self.linked_list = DoublyLinkedList()

        self.prev_joy_call = time.perf_counter()
        self.joy_calls = []

        self.prev_timer_call = time.perf_counter()

        self.last_real_lx = 0.0
        self.last_real_az = 0.0
        self.last_real_time = self.prev_joy_call

        #optimization stats
        self.delay_durations = []   #store the delay difference of each delay signal for statistics
        self.collect_data = False

        #start collecting data after buffer has been populated
        self.data_collecton_timer = self.create_timer(self.delay*2, self.start_collecting_data)

        self.subscription = self.create_subscription(
            Twist,
            'mr1/cmd_vel',
            self.joy_callback,
            1)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, msg):
        current_time = time.perf_counter()

        ##Refresh Time Tests
        # joy_refresh_time = current_time - self.prev_joy_call
        # # print('joy_refresh_time: %f' % joy_refresh_time)
        # self.joy_calls.append(joy_refresh_time)
        # self.prev_joy_call = current_time

        # if len(self.joy_calls) % 500 == 0 and len(self.joy_calls) != 0:
        #     mean_delay = statistics.mean(self.joy_calls)
        #     stdev_delay = statistics.stdev(self.joy_calls)
        #     max_delay = max(self.joy_calls)
        #     min_delay = min(self.joy_calls)
        #     print('mean joy delay duration: %f' % mean_delay)
        #     print('stdev joy delay duration: %f' % stdev_delay)
        #     print('max joy delay: %f' % max_delay)
        #     print('min joy delay: %f' % min_delay)
        #     #terminate program once reached 500 signals
        #     #rclpy.shutdown()

        self.rt_lx = msg.linear.x
        self.rt_az = msg.angular.z

        self.last_real_lx = self.rt_lx
        self.last_real_az = self.rt_az
        self.last_real_time = current_time

        #self.get_logger().info('real time "%s"' % msg.linear.x)
        #print('real time %s' % self.rt_lx)

        self.linked_list.append(self.rt_lx, self.rt_az, current_time)

    #function that is called after 'delay' seconds pass to start collecting data
    def start_collecting_data(self):
        self.collect_data = True
        print('\n Data Collection Start \n')
        self.data_collecton_timer.cancel()

    def timer_callback(self):
        ##Testing
        # #get the most recently added node to 'represent' the real time signal at a known frequency of timer_period
        # tail_node = self.linked_list.get_tail()
        # if tail_node:
        #     close_rt_vel = Twist()
        #     close_rt_vel.linear.x = tail_node.linear_x
        #     close_rt_vel.angular.z = tail_node.angular_z
        #     print('%s (real time)' % close_rt_vel.linear.x)

        current_time = time.perf_counter()

        # timer_refresh_error = self.timer_period_copy - (current_time - self.prev_timer_call)
        # print('timer_refresh_time: %f' % timer_refresh_error)
        # self.prev_timer_call = current_time

        # #if the time since last real signal exceeds dummy signal frequency and append dummy node if so
        # if (current_time - self.last_real_time) > 0.005:
        #     self.linked_list.append(self.last_real_lx, self.last_real_az, self.last_real_time)

        #by only adding to the linked list at a constant frequencies, removes inconsistencies
        # uses shared set of variables so if there was a new real value, it will be added, otherwise it adds a copy of the last real node
        #but uses current time to guratee the frequency
        
        self.linked_list.append(self.last_real_lx, self.last_real_az, current_time)
        list_size = self.linked_list.get_size()
        # print('linked list size: %d'%list_size)

        #call returns closest node to target delay and removes all prev nodes
        delayed_node = self.update()

        #get current time after function call for more accurate time
        current_time = time.perf_counter()

        if delayed_node:
            td_vel = Twist()
            td_vel.linear.x = delayed_node.linear_x
            td_vel.angular.z = delayed_node.angular_z

            #print('%s (delayed)' % td_vel.linear.x)

            self.publisher_td_cmd_vel.publish(td_vel)

            current_time = time.perf_counter()
            # print the error of delayed node from specified delay
            # delay_margin = self.delay - abs(current_time - delayed_node.timestamp)
            # print('delay_margin: %f' % delay_margin)

            # delay_duration = current_time - delayed_node.timestamp
            # print('delay_duration: %f' % delay_duration)

            # #Data Collection Code
            # #collect delay duration data once 'delay' seconds have passed
            # if self.collect_data:
            #     self.delay_durations.append(delay_duration)
            #     if len(self.delay_durations) % 100 == 0 and len(self.delay_durations) != 0:
            #         mean_delay = statistics.mean(self.delay_durations)
            #         stdev_delay = statistics.stdev(self.delay_durations)
            #         max_delay = max(self.delay_durations)
            #         min_delay = min(self.delay_durations)
            #         print('mean td delay duration: %f' % mean_delay)
            #         print('stdev td delay duration: %f' % stdev_delay)
            #         print('max td delay: %f' % max_delay)
            #         print('min td delay: %f' % min_delay)
            #         print('linked list size: %d'%list_size)

            #         #terminate program once reached 500 signals
            #         rclpy.shutdown()
            #self.get_logger().info('time delay value: "%s", delay duration: "%f" seconds' % (td_vel.linear.x, delay_duration))
            #print(" ")

    def update(self):
            current_time = time.perf_counter()
            current_node = self.linked_list.get_head()
            closest_node = current_node
            # Iterate through the linked list to find the closest node
            while current_node:
                # If time difference is less than delay, return the previous node as closest
                if (current_time - current_node.timestamp) < self.delay:
                    # If the absolute val of node is closer than current closest, set current as closest
                    if abs((self.delay - abs(current_time - current_node.timestamp))) < abs((self.delay - abs(current_time - closest_node.timestamp))):
                        closest_node = current_node
                    break
                closest_node = current_node
                current_node = current_node.next
                # Remove the head node
                self.linked_list.remove_head()
            return closest_node


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

