#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#include <list>
#include <chrono>
#include <fstream>
#include <atomic>
#include <mutex>
#include <cmath>
#include <numeric>
#include <random>

#include "list"

#define clock std::chrono::steady_clock
#define clock_resolution std::chrono::microseconds
#define Frame_Capture_Period 0.00001

class FrameNode {
  public:
    std::shared_ptr<cv::Mat> frame;
    std::chrono::time_point<clock> time_stamp;
    FrameNode() : frame(), time_stamp(clock::now()) {}
    FrameNode(std::shared_ptr<cv::Mat> _frame, std::chrono::time_point<clock> _time_stamp) : frame(_frame), time_stamp(_time_stamp) {}
    void set(std::shared_ptr<cv::Mat> _frame, std::chrono::time_point<clock> _time_stamp){
      frame = _frame;
      time_stamp = _time_stamp;
    }

};

class CaptureData {
  private:
    double target_delay; //in milliseconds
  public:
    std::vector<double> values;  
    CaptureData(double _target_delay){
      target_delay = _target_delay;
    }
    const double get_average() const {
      double sum = std::accumulate(values.begin(), values.end(), 0.0);
      return sum/values.size();
    }
    const double get_std_dev(const double& average) const {
      double sum = 0.0;
      for (const double& value : values){
        sum += std::pow(value - average, 2);
      }
      return std::sqrt(sum/values.size());
    }
    const double get_target_std_dev() const {
      return get_std_dev(target_delay);
    }
    const double get_target_delay() const {
      return target_delay;
    }
};

class CaptureDisplay {
  public:
    std::string name;
    clock_resolution delay;
    clock_resolution real_delay;
    clock_resolution frame_refresh_period;
    std::chrono::time_point<clock> last_update_time;
    std::list<FrameNode>::iterator frame_node;

    CaptureDisplay
     (std::string _name, 
      clock_resolution _real_delay,
      clock_resolution _delay, 
      clock_resolution _frame_refresh_period, 
      std::chrono::time_point<clock> _last_update_time):
      name(_name),
      delay(_delay), 
      real_delay(_real_delay),
      frame_refresh_period(_frame_refresh_period), 
      last_update_time(_last_update_time){}
    void printInfo(){
      std::cout << "delay: " << delay.count() << ", frame_refresh_period: " << frame_refresh_period.count() << std::endl; 
    }
};


void terminate_capture(cv::VideoCapture* capture) {
  if (capture && capture->isOpened()) {
    capture->release();
    cv::destroyAllWindows();
  }
  exit(0);
}

int get_webcam_index() {
  for (int camera_index = 0; camera_index < 10; ++camera_index) {
    cv::VideoCapture cap(camera_index, cv::CAP_V4L2); // Use V4L2 backend explicitly
    if (cap.isOpened()) {
      std::cout << "\033[92mCamera index available: " << camera_index << "\033[0m" << std::endl;
      cap.release();
      return camera_index;
    }
  }
  std::cout << "\033[91mCamera not detected, terminating\033[0m" << std::endl;
  terminate_capture(nullptr);
  return -1;
}

void capture_frames(cv::VideoCapture& capture, std::list<FrameNode>& frame_buffer, std::mutex& mtx, std::atomic<bool>& run, std::atomic<bool>& read){
  while(run.load()){
    std::chrono::time_point<clock> start_time = clock::now();
    
    // wait till new frame is available
    if (!capture.grab()){
      std::this_thread::sleep_for(clock_resolution(100));
      continue;
    }
    std::shared_ptr<cv::Mat> frame_ptr = std::make_shared<cv::Mat>();

    // retrieve new frame
    bool ret = capture.retrieve(*frame_ptr);
    mtx.lock();
    frame_buffer.push_back(FrameNode(frame_ptr, clock::now() - (clock::now()-start_time)));
    mtx.unlock();
    //terminate program if retrieve fails
    if (!ret){
      std::cout << "\033[91mError: Unable to read frame\033[0m" << std::endl;
      run.store(false);
      continue;   
    }
    //signal update_frames to create new shared pointer frame
    read.store(true);
    //std::cout << "Frame capture duration: " << std::chrono::duration_cast<clock_resolution>(clock::now()-start_time).count() << " microseconds" << std::endl;

  }     
}

void update_frames(cv::VideoCapture& capture, std::list<FrameNode>& frame_buffer, clock_resolution target_frame_interval, std::mutex& mtx, std::atomic<bool>& run, std::atomic<bool>& read) {
  std::chrono::time_point<clock> prev_time = clock::now();
  std::shared_ptr<cv::Mat> prev_frame = std::make_shared<cv::Mat>(*frame_buffer.front().frame);
  while (run.load()) {
    std::chrono::time_point<clock> start_time = clock::now();
    mtx.lock();

    if(read.load()){
      prev_frame = frame_buffer.back().frame;
      read.store(false);
    }    
    frame_buffer.push_back(FrameNode(prev_frame, clock::now()));
    mtx.unlock();
    clock_resolution duration = std::chrono::duration_cast<clock_resolution>(clock::now()-start_time);
    if (duration < target_frame_interval){
      std::this_thread::sleep_for((target_frame_interval-duration)*0.9383);
    }
    //std::cout << "Frame update duration: " << std::chrono::duration_cast<clock_resolution>(clock::now()-start_time).count() << " microseconds" << std::endl;
  }
}

void update_displays(std::list<FrameNode>& frame_buffer, std::vector<CaptureDisplay>& displays, std::mutex& mtx, std::atomic<bool>& run){
  while (run.load()) {
    std::chrono::time_point<clock> frame_start = clock::now();
    mtx.lock();
    for (CaptureDisplay& display : displays) {
      //display.printInfo();
      std::list<FrameNode>::iterator it = display.frame_node;
      ++it;
      while (it != frame_buffer.end() && display.frame_node->time_stamp + display.delay < frame_start) {
        display.frame_node = it;
        ++it;
        //std::cout<<std::chrono::duration_cast<clock_resolution>(clock::now()-display.frame_node->Value->time_stamp).count()<<std::endl;

      }
    }
    mtx.unlock();
    //std::this_thread::sleep_for(clock_resolution(1));
    //std::cout << "Display pointer traversal duration: " << std::chrono::duration_cast<clock_resolution>(clock::now()-frame_start).count() << " microseconds" << std::endl;
  }

}

void cleanup(std::list<FrameNode>& frame_buffer, FrameNode*& last_display, std::mutex& mtx, std::atomic<bool>& updated, std::atomic<bool>& run){
  while(run.load()){
    mtx.lock();
    while(&frame_buffer.front() != last_display && updated.load() && !frame_buffer.empty()){
      //std::cout<<"Last displayed frame time: "<<std::chrono::duration_cast<clock_resolution>(clock::now()-last_display->Value->time_stamp).count()<<std::endl;
      //std::cout<<"deleting frame time: "<<std::chrono::duration_cast<clock_resolution>(clock::now()-frame_buffer.getHead()->Value->time_stamp).count()<<std::endl;
      //std::cout << frame_buffer.getCount() << std::endl;    
      
      frame_buffer.pop_front();
    } 
    mtx.unlock();
    updated.store(false);
    std::this_thread::sleep_for(clock_resolution(10));
  }
}

void display_frames(std::list<FrameNode>& frame_buffer, std::vector<CaptureDisplay>& displays, FrameNode*& last_display, std::mutex& mtx, std::atomic<bool>& updated, std::atomic<bool>& run) {
  int screenshot_counter = 0;
  while (run.load()) {
    std::chrono::time_point<clock> frame_start = clock::now();
    for (int x = 0; x < displays.size(); ++x) {
      if (std::chrono::duration_cast<clock_resolution>(frame_start - displays[x].last_update_time) < displays[x].frame_refresh_period) continue;
      if (displays[x].frame_node != frame_buffer.end()) {
        if(x == displays.size()-1) {
          last_display = &(*displays[x].frame_node);
          updated.store(true);
        }
        cv::imshow("Display " + displays[x].name + "s delay", *displays[x].frame_node->frame);
        displays[x].last_update_time = clock::now();
        //std::cout<<"Reported Delay in microseconds: "<<std::chrono::duration_cast<clock_resolution>(clock::now()-displays[x].frame_node->time_stamp).count()<<std::endl;
        continue;
      }

      
    }
    int key = cv::waitKey(1) & 0xFF;
    
    if (key == 'q') {
      run.store(false);
    } else if (key == 's') {
      cv::Mat combined_image;
      for (const auto& display : displays) {
        if (combined_image.empty()) {
          combined_image = *(display.frame_node->frame);
        } else {
          cv::hconcat(combined_image, *(display.frame_node->frame), combined_image);
        }
      }
      screenshot_counter++;
      std::string screenshot_name = "combined_screenshot_" + std::to_string(screenshot_counter) + ".png";
      cv::imwrite(screenshot_name, combined_image);
      std::cout << "\033[92mCombined screenshot saved as " << screenshot_name << "\033[0m" << std::endl;
    } else if (key == 't') {
      std::vector<double> time_diffs;//signal update_frames to create new shared pointer frame
      std::ofstream file("display_time_differences.txt");
      std::chrono::time_point<clock> now = clock::now();
      for (const auto& display : displays) {
        double time_diff = &(*display.frame_node) != &frame_buffer.back() ? std::chrono::duration<double, std::milli>(now - display.frame_node->time_stamp).count() : 0;
        time_diffs.push_back(time_diff);
      }
      for (const auto& diff : time_diffs) {
        file << diff << "\n";
      }
      file.close();
    //std::cout<<delay_duration.count()<<std::endl;     std::list<FrameNode>::iterator _frame_node
      std::cout << "\033[92mDisplay time differences saved to display_time_differences.txt\033[0m" << std::endl;
    }
    //std::cout << "Display update duration: " << std::chrono::duration_cast<clock_resolution>(clock::now()-frame_start).count() << " microseconds" << std::endl;

  }
}

std::random_device rd;  // Obtain a random number from hardware
std::mt19937 gen(rd()); // Seed the generator

inline clock_resolution generate_random_duration(int min_us, int max_us) {
    std::uniform_int_distribution<int> dis(min_us, max_us); // Define the range
    return std::chrono::milliseconds(dis(gen));
}

void collect_data(std::vector<CaptureDisplay>& displays, std::atomic<bool>& run){
  std::vector<CaptureData> data_vec;
  for (const auto& display : displays){
    data_vec.emplace_back(CaptureData(std::chrono::duration_cast<std::chrono::duration<double>>(display.real_delay).count()));
  }
  std::this_thread::sleep_for(displays.back().delay);
  while(run.load()){
    clock_resolution sleep_duration = generate_random_duration(50, 1000); 
    std::this_thread::sleep_for(sleep_duration);
    if(!run.load()) break;
    for (int x = 0; x < displays.size(); ++x){
      std::chrono::time_point<clock> now = clock::now();
      data_vec[x].values.emplace_back(std::chrono::duration_cast<std::chrono::duration<double>>(now - displays[x].frame_node->time_stamp).count());
    }
  }
  std::ofstream data_file("Collected_Data.txt");
    for(const auto& data_seg : data_vec){
    data_file << "Target delay: " << data_seg.get_target_delay() << "; Average: " << data_seg.get_average() << "; Std dev: " << data_seg.get_std_dev(data_seg.get_average()) << "; Target Std dev: " << data_seg.get_target_std_dev() << "; High: " << *std::max_element(data_seg.values.begin(), data_seg.values.end()) << "; Low: "<< *std::min_element(data_seg.values.begin(), data_seg.values.end()) <<"\n";
    std::ostringstream filename;
    filename << data_seg.get_target_delay() << "s_delay_data.txt";
    std::ofstream raw_data_file(filename.str());
    raw_data_file << "Target Delay: " << data_seg.get_target_delay() << "\n[ " << data_seg.values[0];
    for (int x = 1; x < data_seg.values.size(); ++x){
      raw_data_file << ", " << data_seg.values[x];
    }
    raw_data_file << "]";
    raw_data_file.close();
  }
  data_file.close();

}

inline const bool display_comparator(CaptureDisplay& a, CaptureDisplay& b){
  return a.delay < b.delay;
}

int main() {
  std::chrono::time_point program_start_time = clock::now();
  std::atomic<bool> run;
  run.store(true);
  std::atomic<bool> read;
  read.store(false);
  std::atomic<bool> updated;
  updated.store(false);
  std::mutex mtx;



  // Camera Init
  // Get Camera Index
  // Set Capture to First Camera

  int camera_index = get_webcam_index();
  if (camera_index == -1) {
    return -1;
  }

  cv::VideoCapture capture(camera_index, cv::CAP_V4L2);
  if (!capture.isOpened()) {
    std::cout << "\033[91mError: Could not open camera\033[0m" << std::endl;
    return -1;
  }
  double max_camera_fps = capture.get(cv::CAP_PROP_FPS);
  
  // Initialisation Menu
  // Ask for number of displays
  // Ask for delay and framerate of each display

  int num_displays;
  std::cout << "\033[94mEnter the number of displays: \033[0m";
  std::cin >> num_displays;
  if (num_displays <= 0) {
    std::cout << "\033[91mInvalid input. Number of displays must be a positive integer.\033[0m" << std::endl;
    return -1;
  }

  // Get vars for each display
  clock_resolution frame_interval = std::chrono::duration_cast<clock_resolution>(std::chrono::duration<double>(Frame_Capture_Period)); 
  std::vector<CaptureDisplay> displays;
  for (int i = 0; i < num_displays; ++i) {

    // Get Delay

    double delay, frame_rate;
    while (true) {
      std::cout << "\033[94mEnter the delay for display " << i + 1 << " (in seconds): \033[0m";
      std::cin >> delay;
      if (delay >= 0) {
        break;
      }
      std::cout << "\033[91mInvalid input. Delay must be a non-negative value.\033[0m" << std::endl;
    }


    // Get Frame_rate

    while (true) {
      std::cout << "\033[94mEnter the frame rate for display " << i + 1 << " (in fps, max " << max_camera_fps << "): \033[0m";
      std::cin >> frame_rate;
      if (frame_rate > 0 && frame_rate <= max_camera_fps) {
        break;
      }
      std::cout << "\033[91mInvalid input. Frame rate must be a positive value and not exceed " << max_camera_fps<< " fps.\033[0m" << std::endl;
    }

    // Init display class
    std::string name = std::to_string(delay);
    clock_resolution real_delay = std::chrono::duration_cast<clock_resolution>(std::chrono::duration<double>(delay));
    clock_resolution delay_duration = std::chrono::duration_cast<clock_resolution>(std::chrono::duration<double>(delay) + (frame_interval*0.5));
    clock_resolution frame_refresh_period = std::chrono::duration_cast<clock_resolution>(std::chrono::duration<double>(1.0 / frame_rate));
    displays.emplace_back(CaptureDisplay(name, real_delay, delay_duration, frame_refresh_period, clock::now()));
    cv::namedWindow("Display " + name + "s delay", cv::WINDOW_AUTOSIZE); // Create window for each display
  }

  // Sort delay list for cleanup

  std::sort(displays.begin(), displays.end(), display_comparator);


  // 1ms target capture period
  // Initial frame capture
  
  std::list<FrameNode> frame_buffer;
  std::shared_ptr<cv::Mat> frame = std::make_shared<cv::Mat>();
  capture >> *frame;
  if (frame->empty()) {
      std::cout << "\033[91mError: Unable to read initial frame\033[0m" << std::endl;
      terminate_capture(&capture);
  }
  
  std::chrono::time_point now = clock::now();
  frame_buffer.push_back(FrameNode(frame, now));
  //std::cout<<std::chrono::duration_cast<clock_resolution>(clock::now() - frame_buffer.getHead()->Value->time_stamp).count()<<std::endl;
  FrameNode* last_display = &frame_buffer.front();

  for (CaptureDisplay& display : displays) {
      display.frame_node = frame_buffer.begin();
  }
  // Thread Init/start
  
  std::thread read_capture_thread(capture_frames, std::ref(capture), std::ref(frame_buffer), std::ref(mtx), std::ref(run), std::ref(read));
  std::thread update_capture_thread(update_frames, std::ref(capture), std::ref(frame_buffer), frame_interval, std::ref(mtx), std::ref(run), std::ref(read));
  std::thread update_display_thread(update_displays, std::ref(frame_buffer), std::ref(displays), std::ref(mtx), std::ref(run));
  std::thread cleanup_thread(cleanup, std::ref(frame_buffer), std::ref(last_display), std::ref(mtx), std::ref(updated), std::ref(run));
  std::thread collect_data_thread(collect_data, std::ref(displays), std::ref(run));

  display_frames(frame_buffer, displays, last_display, mtx, updated, run);

  read_capture_thread.join();
  update_capture_thread.join();
  update_display_thread.join();
  cleanup_thread.join();
  collect_data_thread.join();
  terminate_capture(&capture);

  return 0;
}

