// Copyright HMND
// A simple watchdog that monitors a topic and restarts the target process
// if no messages are received within configured timeouts.

#include <chrono>
#include <csignal>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>
#include <sstream>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <neo_msgs2/msg/relay_board_v3.hpp>

using namespace std::chrono_literals;

namespace {
bool readFile(const std::string &path, std::string &out) {
  std::ifstream in(path, std::ios::in | std::ios::binary);
  if (!in.is_open()) return false;
  std::ostringstream ss;
  ss << in.rdbuf();
  out = ss.str();
  return true;
}

// Read /proc/<pid>/cmdline and join arguments with spaces
bool readCmdline(pid_t pid, std::string &out) {
  std::string raw;
  if (!readFile("/proc/" + std::to_string(pid) + "/cmdline", raw)) {
    return false;
  }
  std::ostringstream ss;
  for (size_t i = 0; i < raw.size(); ++i) {
    char c = raw[i];
    if (c == '\0') {
      if (i + 1 < raw.size()) ss << ' ';
    } else {
      ss << c;
    }
  }
  out = ss.str();
  return true;
}

// Collect PIDs whose /proc/<pid>/comm matches the given process name exactly.
std::vector<pid_t> findPidsByComm(const std::string &process_name) {
  std::vector<pid_t> result;
  DIR *dir = opendir("/proc");
  if (!dir) return result;
  struct dirent *entry = nullptr;
  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_type != DT_DIR) continue;
    const char *name = entry->d_name;
    // numeric directories only
    for (const char *p = name; *p; ++p) {
      if (*p < '0' || *p > '9') {
        name = nullptr;
        break;
      }
    }
    if (!name) continue;
    pid_t pid = static_cast<pid_t>(std::stoi(entry->d_name));
    std::string comm_path = std::string("/proc/") + entry->d_name + "/comm";
    std::string comm;
    if (readFile(comm_path, comm)) {
      // comm includes trailing newline
      if (!comm.empty() && comm.back() == '\n') comm.pop_back();
      if (comm == process_name) {
        result.push_back(pid);
      }
    }
  }
  closedir(dir);
  return result;
}

// Collect PIDs where either comm matches process_name, or cmdline contains "__node:=<node_name>"
std::vector<pid_t> findPidsByTarget(const std::string &process_name, const std::string &node_name) {
  std::vector<pid_t> result;
  DIR *dir = opendir("/proc");
  if (!dir) return result;
  struct dirent *entry = nullptr;
  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_type != DT_DIR) continue;
    const char *name = entry->d_name;
    for (const char *p = name; *p; ++p) {
      if (*p < '0' || *p > '9') {
        name = nullptr;
        break;
      }
    }
    if (!name) continue;
    pid_t pid = static_cast<pid_t>(std::stoi(entry->d_name));
    std::string comm;
    bool matched = false;
    if (readFile(std::string("/proc/") + entry->d_name + "/comm", comm)) {
      if (!comm.empty() && comm.back() == '\n') comm.pop_back();
      matched = (comm == process_name);
    }
    if (!matched) {
      std::string cmdline;
      if (readCmdline(pid, cmdline)) {
        const std::string needle = std::string("__node:=") + node_name;
        matched = (cmdline.find(needle) != std::string::npos);
      }
    }
    if (matched) result.push_back(pid);
  }
  closedir(dir);
  return result;
}

void terminatePids(const std::vector<pid_t> &pids, int sig = SIGTERM) {
  for (pid_t pid : pids) {
    if (pid > 1) {
      kill(pid, sig);
    }
  }
}
}  // namespace

class RelayboardWatchdog : public rclcpp::Node {
 public:
  RelayboardWatchdog()
      : rclcpp::Node("relayboard_watchdog"),
        last_msg_time_(this->now()),
        received_any_(false),
        last_restart_time_(this->now()) {
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/relayboard_v3/state");
    startup_timeout_sec_ = this->declare_parameter<double>("startup_timeout_sec", 5.0);
    deadman_timeout_sec_ = this->declare_parameter<double>("deadman_timeout_sec", 2.0);
    target_node_name_ = this->declare_parameter<std::string>("target_node_name", "/relayboardv3_node");

    // Derive process name from target_node_name (strip leading slash).
    process_name_ = target_node_name_;
    if (!process_name_.empty() && process_name_.front() == '/') {
      process_name_.erase(process_name_.begin());
    }

    using MsgT = neo_msgs2::msg::RelayBoardV3;
    sub_ = this->create_subscription<MsgT>(
        state_topic_, rclcpp::QoS(1),
        [this](const MsgT &) {
          received_any_ = true;
          last_msg_time_ = this->now();
        });

    timer_ = this->create_wall_timer(200ms, std::bind(&RelayboardWatchdog::onTimer, this));
    RCLCPP_INFO(this->get_logger(),
                "relayboard_watchdog started. Monitoring '%s', target '%s' (proc='%s'), "
                "startup_timeout=%.2fs, deadman_timeout=%.2fs",
                state_topic_.c_str(), target_node_name_.c_str(), process_name_.c_str(),
                startup_timeout_sec_, deadman_timeout_sec_);
  }

 private:
  void onTimer() {
    const auto now_ros = this->now();
    const double since_last_msg = (now_ros - last_msg_time_).seconds();

    // Debug: log current time and target process PIDs
    {
      std::string node_name = target_node_name_;
      if (!node_name.empty() && node_name.front() == '/') node_name.erase(node_name.begin());
      auto pids = findPidsByTarget(process_name_, node_name);
      if (pids.size() == 1) {
        RCLCPP_INFO(this->get_logger(),
                    "watchdog tick t=%.3f target='%s' target_pid=%d",
                    now_ros.seconds(), target_node_name_.c_str(), static_cast<int>(pids[0]));
      } else {
        std::ostringstream pid_list;
        for (size_t i = 0; i < pids.size(); ++i) {
          if (i) pid_list << ",";
          pid_list << pids[i];
        }
        RCLCPP_INFO(this->get_logger(),
                    "watchdog tick t=%.3f target='%s' target_pids=[%s]",
                    now_ros.seconds(), target_node_name_.c_str(), pid_list.str().c_str());
      }
    }

    // Startup timeout: no messages seen at all within startup_timeout_sec_
    if (!received_any_ && since_last_msg >= startup_timeout_sec_) {
      RCLCPP_WARN(this->get_logger(),
                  "No messages on '%s' within startup timeout (%.2fs). Restarting '%s'.",
                  state_topic_.c_str(), startup_timeout_sec_, process_name_.c_str());
      restartTarget();
      // After restart, give it startup_timeout_sec_ again before checking startup condition
      last_msg_time_ = this->now();
      return;
    }

    // Deadman timeout: messages stopped for longer than deadman_timeout_sec_
    if (received_any_ && since_last_msg >= deadman_timeout_sec_) {
      // Avoid restart storms: enforce a minimal 1s between restarts
      const double since_restart = (now_ros - last_restart_time_).seconds();
      if (since_restart >= 1.0) {
        RCLCPP_WARN(this->get_logger(),
                    "No messages on '%s' for %.2fs (deadman=%.2fs). Restarting '%s'.",
                    state_topic_.c_str(), since_last_msg, deadman_timeout_sec_, process_name_.c_str());
        restartTarget();
        last_msg_time_ = this->now();
      }
    }
  }

  void restartTarget() {
    last_restart_time_ = this->now();
    // Find and terminate target process(es)
    std::string node_name = target_node_name_;
    if (!node_name.empty() && node_name.front() == '/') node_name.erase(node_name.begin());
    auto pids = findPidsByTarget(process_name_, node_name);
    if (pids.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Target process '%s' not found by comm. Nothing to kill; expecting external respawn.",
                  process_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Sending SIGTERM to %zu process(es) named '%s'.",
                  pids.size(), process_name_.c_str());
      terminatePids(pids, SIGTERM);
    }
  }

 private:
  std::string state_topic_;
  double startup_timeout_sec_;
  double deadman_timeout_sec_;
  std::string target_node_name_;
  std::string process_name_;

  rclcpp::Subscription<neo_msgs2::msg::RelayBoardV3>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_msg_time_;
  bool received_any_;
  rclcpp::Time last_restart_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelayboardWatchdog>());
  rclcpp::shutdown();
  return 0;
}

