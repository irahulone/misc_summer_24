import numpy as np
import cv2
import time
import threading

class Node:
    def __init__(self, value, time_stamp=0, next_node=None, prev_node=None):
        self.value = value
        self.time_stamp = time_stamp
        self.next_node = next_node
        self.prev_node = prev_node

class DoublyLinkedList:
    def __init__(self):
        self.head_node = None
        self.tail_node = None
        self.lock = threading.Lock()

    def add_to_tail(self, new_value, time):
        new_tail = Node(new_value, time)
        with self.lock:
            if self.tail_node is None:
                self.head_node = new_tail
                self.tail_node = new_tail
            else:
                self.tail_node.next_node = new_tail
                new_tail.prev_node = self.tail_node
                self.tail_node = new_tail

    def remove_head(self):
        with self.lock:
            if self.head_node is None:
                return None
            removed_head = self.head_node
            self.head_node = self.head_node.next_node
            if self.head_node is not None:
                self.head_node.prev_node = None
            if removed_head == self.tail_node:
                self.tail_node = None
            return removed_head

class CaptureDisplay:
    def __init__(self, delay: float, frame_rate: float):
        self.delay = delay
        self.frame_refresh_period = 1.0 / frame_rate
        self.last_update_time = time.perf_counter()
        self.frame_node = None

def terminate(capture):
    if capture.isOpened():
        capture.release()
        cv2.destroyAllWindows()
    exit()

def get_webcam_index():
    for camera_index in range(10):
        cap = cv2.VideoCapture(camera_index)
        if cap.isOpened():
            print(f'\033[92mCamera index available: {camera_index}\033[0m')
            cap.release()
            return camera_index
    print("\033[91mCamera not detected, terminating\033[0m")
    terminate(None)

def capture_frames(capture, frame_buffer, frame_interval):
    while True:
        start_time = time.perf_counter()
        
        ret, frame = capture.read()
        if not ret:
            print('\033[91mError: Unable to read frame\033[0m')
            terminate(capture)

        now = time.perf_counter()
        frame_buffer.add_to_tail(frame, now)

        # Sleep for most of the interval
        sleep_time = frame_interval - (time.perf_counter() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

        # Busy-wait for the remaining time to fine-tune precision
        while (time.perf_counter() - start_time) < frame_interval:
            pass

def display_frames(frame_buffer, displays):
    screenshot_counter = 0
    while True:
        now = time.perf_counter()

        for display in displays:
            if now - display.last_update_time >= display.frame_refresh_period:
                while display.frame_node and display.frame_node.time_stamp + display.delay < now:
                    if display.frame_node.next_node is not None:
                        display.frame_node = display.frame_node.next_node
                    else:
                        break
                if display.frame_node:
                    cv2.imshow(f'Display {display.delay}s delay', display.frame_node.value)
                    display.last_update_time = now

        while frame_buffer.head_node and frame_buffer.head_node != displays[-1].frame_node:
            frame_buffer.remove_head()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            terminate(capture)
            break
        elif key == ord('s'):
            combined_image = None
            for display in displays:
                if combined_image is None:
                    combined_image = display.frame_node.value
                else:
                    combined_image = np.hstack((combined_image, display.frame_node.value))
            screenshot_counter += 1
            screenshot_name = f'combined_screenshot_{screenshot_counter}.png'
            cv2.imwrite(screenshot_name, combined_image)
            print(f'\033[92mCombined screenshot saved as {screenshot_name}\033[0m')
        elif key == ord('t'):
            time_diffs = []
            with open('display_time_differences.txt', 'w') as f:
                for display in displays:
                    time_diffs.append(now - display.frame_node.time_stamp if display.frame_node else 0)
                f.write(f'{time_diffs}\n')
            print('\033[92mDisplay time differences saved to display_time_differences.txt\033[0m')

if __name__ == "__main__":
    print("\033[2J\033[H")  # Clear screen
    capture = cv2.VideoCapture(get_webcam_index())
    max_camera_fps = capture.get(cv2.CAP_PROP_FPS)
    print(f"\033[93mMax camera FPS: {max_camera_fps}\033[0m")

    # Interactive CLI interface with error handling
    while True:
        try:
            num_displays = int(input("\033[94mEnter the number of displays: \033[0m"))
            if num_displays <= 0:
                raise ValueError("The number of displays must be a positive integer.")
            break
        except ValueError as e:
            print(f"\033[91mInvalid input: {e}. Please try again.\033[0m")

    displays = []
    
    for i in range(num_displays):
        while True:
            try:
                delay = float(input(f"\033[94mEnter the delay for display {i+1} (in seconds): \033[0m"))
                if delay < 0:
                    raise ValueError("Delay must be a non-negative value.")
                break
            except ValueError as e:
                print(f"\033[91mInvalid input: {e}. Please try again.\033[0m")
        
        while True:
            try:
                frame_rate = float(input(f"\033[94mEnter the frame rate for display {i+1} (in fps, max {max_camera_fps}): \033[0m"))
                if frame_rate <= 0 or frame_rate > max_camera_fps:
                    raise ValueError(f"Frame rate must be a positive value and not exceed {max_camera_fps} fps.")
                frame_rate = min(frame_rate, max_camera_fps)  # Cap the frame rate at the camera's frame rate
                break
            except ValueError as e:
                print(f"\033[91mInvalid input: {e}. Please try again.\033[0m")
        
        displays.append(CaptureDisplay(delay, frame_rate))

    frame_interval = 1.0 / 1000 # Interval for capturing frames

    frame_buffer = DoublyLinkedList()
    ret, frame = capture.read()
    if not ret:
        print('\033[91mError: Unable to read initial frame\033[0m')
        terminate(capture)

    frame_buffer.add_to_tail(frame, time.perf_counter())

    for display in displays:
        display.frame_node = frame_buffer.head_node

    capture_thread = threading.Thread(target=capture_frames, args=(capture, frame_buffer, frame_interval))
    capture_thread.start()

    display_frames(frame_buffer, displays)

    capture_thread.join()
    terminate(capture)

