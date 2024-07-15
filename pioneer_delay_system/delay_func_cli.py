import numpy as np
import cv2
import time
import threading
import curses

class Node:
    """
    A class representing a node in a doubly linked list.
    Each node contains a frame, a timestamp, and pointers to the next and previous nodes.
    """
    def __init__(self, value, time_stamp=0, next_node=None, prev_node=None):
        self.value = value
        self.time_stamp = time_stamp
        self.next_node = next_node
        self.prev_node = prev_node

class DoublyLinkedList:
    """
    A class representing a doubly linked list.
    This list will be used to store frames captured from the webcam.
    """
    def __init__(self):
        self.head_node = None
        self.tail_node = None
        self.lock = threading.Lock()

    def add_to_tail(self, new_value, time):
        """
        Add a new node with a given frame to the tail of the list.
        """
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
        """
        Remove the head node of the list.
        """
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
    """
    A class representing the settings for displaying captured frames.
    """
    def __init__(self, delay: float, frame_rate: float):
        self.delay = delay
        self.frame_refresh_period = 1.0 / frame_rate
        self.last_update_time = time.perf_counter()
        self.frame_node = None

    def set_delay(self, delay: float):
        """
        Set the delay for the display.
        """
        self.delay = delay

    def set_frame_rate(self, frame_rate: float):
        """
        Set the frame rate for the display.
        """
        self.frame_refresh_period = 1.0 / frame_rate

def terminate(capture, terminate_event):
    """
    Terminate the capture and close all windows.
    """
    terminate_event.set()
    if capture and capture.isOpened():
        capture.release()
    cv2.destroyAllWindows()



def get_webcam_indices():
    """
    Get a list of available webcam indices.
    """
    available_indices = []
    for camera_index in range(10):
        cap = cv2.VideoCapture(camera_index)
        if cap.isOpened():
            available_indices.append(camera_index)
            cap.release()
        else:
            print("\033[A\033[2K\033[A\033[2K",end="")

    return available_indices

def capture_frames(capture_ref, frame_buffer, frame_interval, thread_events):
    """
    Capture frames from the webcam and add them to the frame buffer.
    """
    while not thread_events[0].is_set():
        if thread_events[1].is_set():
            thread_events[2].set()
            while thread_events[1].is_set():
                time.sleep(0.001)
            thread_events[2].clear()

        if capture_ref[0].isOpened():
            start_time = time.perf_counter()
            
            ret, frame = capture_ref[0].read()
            '''
            if not ret:
                print('Error: Unable to read frame')
                terminate(capture, thread_events[0])
            '''
            now = time.perf_counter()
            frame_buffer.add_to_tail(frame, now)

            # Sleep for most of the interval
            sleep_time = frame_interval - (time.perf_counter() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Busy-wait for the remaining time to fine-tune precision
            while (time.perf_counter() - start_time) < frame_interval:
                pass

def display_frames(frame_buffer, displays, thread_events):
    """
    Display frames from the buffer according to the settings in displays.
    """
    screenshot_counter = 0
    while not thread_events[0].is_set():
        if thread_events[1].is_set():
            thread_events[3].set()
            while thread_events[1].is_set():
                time.sleep(0.001)
            thread_events[3].clear()


        if displays:
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
                thread_events[0].set()
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
                #print(f'Combined screenshot saved as {screenshot_name}')
            elif key == ord('t'):
                time_diffs = []
                with open('display_time_differences.txt', 'w') as f:
                    for display in displays:
                        time_diffs.append(now - display.frame_node.time_stamp if display.frame_node else 0)
                    f.write(f'{time_diffs}\n')
                #print('Display time differences saved to display_time_differences.txt')


def menu(stdscr, displays, thread_events, capture_ref, frame_buffer, camera_indices):
    """
    Display the main menu for configuring displays and selecting a camera.
    """
    curses.curs_set(0)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    current_display = 0

    while not thread_events[0].is_set():
        stdscr.clear()
        height, width = stdscr.getmaxyx()

        stdscr.attron(curses.color_pair(4))
        stdscr.addstr(0, 0, "Display Configuration Menu", curses.A_BOLD)
        stdscr.attroff(curses.color_pair(4))

        if not capture_ref[0].isOpened():
            stdscr.attron(curses.color_pair(5) | curses.A_REVERSE if current_display == 0 else curses.color_pair(5))
            stdscr.addstr(2, 0, "Select Camera")
            stdscr.attroff(curses.color_pair(5) | curses.A_REVERSE if current_display == 0 else curses.color_pair(5))
        else:
            for idx, display in enumerate(displays):
                if idx == current_display:
                    stdscr.attron(curses.color_pair(5) | curses.A_REVERSE)
                    stdscr.addstr(idx * 2 + 2, 0, f"Display {idx + 1} - Delay: {display.delay}s, Frame Rate: {1.0 / display.frame_refresh_period:.2f}fps")
                    stdscr.attroff(curses.color_pair(5) | curses.A_REVERSE)
                else:
                    stdscr.attron(curses.color_pair(5))
                    stdscr.addstr(idx * 2 + 2, 0, f"Display {idx + 1} - Delay: {display.delay}s, Frame Rate: {1.0 / display.frame_refresh_period:.2f}fps")
                    stdscr.attroff(curses.color_pair(5))

            stdscr.attron(curses.color_pair(5) | curses.A_REVERSE if current_display == len(displays) else curses.color_pair(5))
            stdscr.addstr(len(displays) * 2 + 2, 0, "Add Display")
            stdscr.attroff(curses.color_pair(5) | curses.A_REVERSE if current_display == len(displays) else curses.color_pair(5))
            
            stdscr.attron(curses.color_pair(5) | curses.A_REVERSE if current_display == len(displays) + 1 else curses.color_pair(5))
            stdscr.addstr(len(displays) * 2 + 4, 0, "Select Camera")
            stdscr.attroff(curses.color_pair(5) | curses.A_REVERSE if current_display == len(displays) + 1 else curses.color_pair(5))

        stdscr.attron(curses.color_pair(2))
        stdscr.addstr(height - 2, 0, "Use TAB to navigate, SHIFT+TAB to go back, ENTER to modify/add/select, Q to quit.")
        stdscr.attroff(curses.color_pair(2))
        stdscr.refresh()

        key = stdscr.getch()

        if key == ord('\t'):
            current_display = (current_display + 1) % (len(displays) + 2)
        elif key == curses.KEY_BTAB:
            current_display = (current_display - 1) % (len(displays) + 2)
        elif key == ord('\n'):
            if not capture_ref[0].isOpened() or current_display == len(displays) + 1:
                select_camera(stdscr, capture_ref, displays, frame_buffer, camera_indices, thread_events)
            elif current_display == len(displays):
                add_display(stdscr, displays, capture_ref[0], frame_buffer)
            else:
                modify_display(stdscr, displays[current_display], thread_events)
        elif key == ord('q'):
            thread_events[0].set()

    curses.endwin()  # Ensure curses window is closed correctly on exit


def add_display(stdscr, displays, capture, frame_buffer):
    """
    Add a new display configuration.
    """
    curses.curs_set(1)
    curses.echo()
    stdscr.clear()
    stdscr.addstr(0, 0, "Enter delay for new display (seconds): ", curses.color_pair(3))
    stdscr.refresh()

    try:
        delay = float(stdscr.getstr(1, 0).decode('utf-8'))
        if delay < 0:
            raise ValueError("Delay must be a non-negative value.")
        stdscr.addstr(2, 0, "Enter frame rate for new display (fps): ", curses.color_pair(3))
        stdscr.refresh()
        frame_rate = float(stdscr.getstr(3, 0).decode('utf-8'))
        max_camera_fps = capture.get(cv2.CAP_PROP_FPS)
        if frame_rate <= 0 or frame_rate > max_camera_fps:
            raise ValueError(f"Frame rate must be a positive value and not exceed {max_camera_fps} fps.")
        new_display = CaptureDisplay(delay, frame_rate)
        if displays:
            new_display.frame_node = displays[0].frame_node  # Assign the current head frame to the new display
        else:
            new_display.frame_node = frame_buffer.head_node
        displays.append(new_display)
    except ValueError as e:
        stdscr.attron(curses.color_pair(2))
        stdscr.addstr(4, 0, f"Invalid input: {e}. Press any key to continue.")
        stdscr.attroff(curses.color_pair(2))
        stdscr.getch()

    curses.noecho()
    curses.curs_set(0)

def select_camera(stdscr, capture_ref, displays, frame_buffer, camera_indices, thread_events):
    """
    Select a camera from the available indices.
    """
    curses.curs_set(0)
    curses.echo()
    stdscr.clear()
    current_index = 0

    while True:
        stdscr.clear()
        stdscr.addstr(0, 0, "Select a camera index:", curses.color_pair(4))

        for idx, cam_index in enumerate(camera_indices):
            if idx == current_index:
                stdscr.attron(curses.color_pair(5) | curses.A_REVERSE)
                stdscr.addstr(idx + 2, 0, f"Camera index {cam_index}")
                stdscr.attroff(curses.color_pair(5) | curses.A_REVERSE)
            else:
                stdscr.attron(curses.color_pair(5))
                stdscr.addstr(idx + 2, 0, f"Camera index {cam_index}")
                stdscr.attroff(curses.color_pair(5))

        stdscr.refresh()
        key = stdscr.getch()

        if key == ord('\n'):
            thread_events[1].set()  # Pause the threads
            while not thread_events[2].is_set() and not thread_events[3].is_set():
                time.sleep(0.001)

            camera_index = camera_indices[current_index]
            if capture_ref[0] and capture_ref[0].isOpened():
                capture_ref[0].release()

            new_capture = cv2.VideoCapture(camera_index)
            capture_ref[0] = new_capture  # Update the capture reference
            if not new_capture.isOpened():
                stdscr.addstr(len(camera_indices) + 4, 0, f"Unable to open camera with index: {camera_index}")
                stdscr.refresh()
                stdscr.getch()
                thread_events[1].clear()  # Resume the threads
                continue

            stdscr.attron(curses.color_pair(4))
            stdscr.addstr(len(camera_indices) + 4, 0, f"Camera {camera_index} selected. Press any key to continue.")
            stdscr.attroff(curses.color_pair(4))
            stdscr.refresh()
            stdscr.getch()

            frame_interval = 1.0 / new_capture.get(cv2.CAP_PROP_FPS)
            ret, frame = new_capture.read()
            if not ret:
                stdscr.addstr(len(camera_indices) + 6, 0, 'Error: Unable to read initial frame. Press any key to continue.')
                stdscr.refresh()
                stdscr.getch()
                thread_events[1].clear()  # Resume the threads
                continue

            frame_buffer.add_to_tail(frame, time.perf_counter())
            for display in displays:
                display.frame_node = frame_buffer.head_node

            thread_events[1].clear()  # Resume the threads

            break
        elif key == ord('\t'):
            current_index = (current_index + 1) % len(camera_indices)
        elif key == curses.KEY_BTAB or key == curses.KEY_UP:
            current_index = (current_index - 1) % len(camera_indices)
        elif key == ord('q'):
            break

    curses.noecho()
    curses.curs_set(1)

def modify_display(stdscr, display, terminate_event):
    """
    Modify the settings of an existing display.
    """
    curses.curs_set(1)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    options = ["Delay", "Frame Rate", "Remove Display"]
    current_option = 0

    while not terminate_event.is_set():
        stdscr.clear()
        height, width = stdscr.getmaxyx()

        stdscr.attron(curses.color_pair(4))
        stdscr.addstr(0, 0, f"Modifying Display - Delay: {display.delay}s, Frame Rate: {1.0 / display.frame_refresh_period:.2f}fps", curses.A_BOLD)
        stdscr.attroff(curses.color_pair(4))

        for idx, option in enumerate(options):
            if idx == current_option:
                stdscr.attron(curses.color_pair(1) | curses.A_REVERSE)
                stdscr.addstr(2 + idx * 2, 0, f"{option}: ", curses.A_BOLD)
                if option == "Delay":
                    stdscr.addstr(2 + idx * 2, 12, f"{display.delay}s")
                elif option == "Frame Rate":
                    stdscr.addstr(2 + idx * 2, 12, f"{1.0 / display.frame_refresh_period:.2f}fps")
                stdscr.attroff(curses.color_pair(1) | curses.A_REVERSE)
            else:
                stdscr.attron(curses.color_pair(3))
                stdscr.addstr(2 + idx * 2, 0, f"{option}: ")
                if option == "Delay":
                    stdscr.addstr(2 + idx * 2, 12, f"{display.delay}s")
                elif option == "Frame Rate":
                    stdscr.addstr(2 + idx * 2, 12, f"{1.0 / display.frame_refresh_period:.2f}fps")
                stdscr.attroff(curses.color_pair(3))

        stdscr.attron(curses.color_pair(2))
        stdscr.addstr(height - 2, 0, "Use TAB to navigate, ENTER to edit/remove, ESC to go back.")
        stdscr.attroff(curses.color_pair(2))
        stdscr.refresh()

        key = stdscr.getch()

        if key == ord('\t'):
            current_option = (current_option + 1) % len(options)
        elif key == curses.KEY_BTAB or key == curses.KEY_UP:
            current_option = (current_option - 1) % len(options)
        elif key == ord('\n'):
            if options[current_option] == "Delay":
                edit_delay(stdscr, display)
            elif options[current_option] == "Frame Rate":
                edit_frame_rate(stdscr, display)
            elif options[current_option] == "Remove Display":
                remove_display(stdscr, displays, display)
                break
        elif key == 27:  # ESC key
            break

    curses.curs_set(0)

def edit_delay(stdscr, display):
    """
    Edit the delay setting for a display.
    """
    curses.curs_set(1)
    curses.echo()
    stdscr.clear()
    stdscr.addstr(0, 0, "Enter new delay (seconds): ", curses.color_pair(3))
    stdscr.refresh()

    try:
        delay = float(stdscr.getstr(1, 0).decode('utf-8'))
        if delay < 0:
            raise ValueError("Delay must be a non-negative value.")
        cv2.destroyWindow(f'Display {display.delay}s delay')
        display.set_delay(delay)
    except ValueError as e:
        stdscr.attron(curses.color_pair(2))
        stdscr.addstr(2, 0, f"Invalid input: {e}. Press any key to continue.")
        stdscr.attroff(curses.color_pair(2))
        stdscr.getch()

    curses.noecho()
    curses.curs_set(0)

def edit_frame_rate(stdscr, display):
    """
    Edit the frame rate setting for a display.
    """
    curses.curs_set(1)
    curses.echo()
    stdscr.clear()
    stdscr.addstr(0, 0, "Enter new frame rate (fps): ", curses.color_pair(3))
    stdscr.refresh()

    try:
        frame_rate = float(stdscr.getstr(1, 0).decode('utf-8'))
        max_camera_fps = 30  # Replace with actual camera FPS if needed
        if frame_rate <= 0 or frame_rate > max_camera_fps:
            raise ValueError(f"Frame rate must be a positive value and not exceed {max_camera_fps} fps.")
        display.set_frame_rate(frame_rate)
    except ValueError as e:
        stdscr.attron(curses.color_pair(2))
        stdscr.addstr(2, 0, f"Invalid input: {e}. Press any key to continue.")
        stdscr.attroff(curses.color_pair(2))
        stdscr.getch()

    curses.noecho()
    curses.curs_set(0)

def remove_display(stdscr, displays, display):
    """
    Remove a display from the list.
    """
    displays.remove(display)
    cv2.destroyWindow(f'Display {display.delay}s delay')
    
def run_menu(stdscr, displays, terminate_event, capture, frame_buffer, camera_indecies):
    try:
        menu(stdscr, displays, terminate_event, capture, frame_buffer, camera_indecies)
    finally:
        curses.endwin()


if __name__ == "__main__":
    capture = cv2.VideoCapture()
    capture_ref = [capture]
    camera_indecies = get_webcam_indices()
    displays = []
    frame_buffer = DoublyLinkedList()
    frame_interval = 1.0 / 1000

    terminate_event = threading.Event()
    pause = threading.Event()
    pause_capture_ack = threading.Event()
    pause_display_ack = threading.Event()

    thread_events = [terminate_event, pause, pause_capture_ack, pause_display_ack]

    # Start the menu in its own thread
    menu_thread = threading.Thread(target=curses.wrapper, args=(run_menu, displays, thread_events, capture_ref, frame_buffer, camera_indecies))
    menu_thread.start()

    # Start the capture and display frames in their own threads
    capture_thread = threading.Thread(target=capture_frames, args=(capture_ref, frame_buffer, frame_interval, thread_events))
    capture_thread.start()
    
    display_frames(frame_buffer, displays, thread_events)

    # Join the threads and call terminate function
    menu_thread.join()
    capture_thread.join()
    terminate(capture, terminate_event)


