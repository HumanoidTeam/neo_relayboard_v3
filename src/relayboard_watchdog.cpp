// Copyright HMND
// Watchdog that monitors the relayboard state topic for a single timeout.
// After the timeout: if unhealthy (no message received), restart the target and exit;
// if healthy, exit the watchdog only. Verbose logging for testing.

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

std::string pidsToString(const std::vector<pid_t> &pids) {
  std::ostringstream ss;
  for (size_t i = 0; i < pids.size(); ++i) {
    if (i) ss << ",";
    ss << pids[i];
  }
  return ss.str();
}
}  // namespace

class RelayboardWatchdog : public rclcpp::Node {
 public:
  RelayboardWatchdog()
      : rclcpp::Node("relayboard_watchdog"),
        start_time_(this->now()),
        received_any_(false),
        timeout_handled_(false) {
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/relayboard_v3/state");
    timeout_sec_ = this->declare_parameter<double>("timeout_sec", 5.0);
    target_node_name_ = this->declare_parameter<std::string>("target_node_name", "/relayboardv3_node");
    node_name_ = target_node_name_;
    if (!node_name_.empty() && node_name_.front() == '/') {
      node_name_.erase(node_name_.begin());
    }

    using MsgT = neo_msgs2::msg::RelayBoardV3;
    sub_ = this->create_subscription<MsgT>(
        state_topic_, rclcpp::QoS(1),
        [this](const MsgT &) {
          received_any_ = true;
        });

    timer_ = this->create_wall_timer(200ms, std::bind(&RelayboardWatchdog::onTimer, this));
    RCLCPP_INFO(this->get_logger(),
                "[relayboard_watchdog] Started. Monitoring '%s', target '%s' (proc='%s'), timeout=%.2fs. State: waiting for first message or timeout.",
                state_topic_.c_str(), target_node_name_.c_str(), node_name_.c_str(), timeout_sec_);
  }

 private:
  void onTimer() {
    if (timeout_handled_) {
      // Only restart node once
      return;
    }
    const auto now_ros = this->now();
    const double elapsed = (now_ros - start_time_).seconds();

    // Verbose tick (for testing)
    const char * health_str = received_any_ ? "healthy" : "unhealthy (no message yet)";
    const std::vector<pid_t> pids = findPidsByTarget(node_name_, node_name_);
    RCLCPP_INFO(this->get_logger(),
                "[relayboard_watchdog] tick t=%.3f elapsed=%.3fs state=%s target_pids=[%s]",
                now_ros.seconds(), elapsed, health_str, pidsToString(pids).c_str());

    if (elapsed < timeout_sec_) {
      return;
    }

    timeout_handled_ = true;
    const bool healthy = received_any_;

    RCLCPP_INFO(this->get_logger(),
                "[relayboard_watchdog] Timeout reached (%.2fs). Target is %s.",
                timeout_sec_, healthy ? "healthy" : "unhealthy");

    if (!healthy) {
      RCLCPP_INFO(this->get_logger(),
                  "[relayboard_watchdog] Shutting down target node '%s' (SIGTERM).",
                  node_name_.c_str());
      restartTarget();
    } else {
      RCLCPP_INFO(this->get_logger(), "[relayboard_watchdog] Target is healthy. Shutting down watchdog only.");
    }

    RCLCPP_INFO(this->get_logger(), "[relayboard_watchdog] Shutting down watchdog.");
    rclcpp::shutdown();
  }

  void restartTarget() {
    const std::vector<pid_t> pids = findPidsByTarget(node_name_, node_name_);
    if (pids.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "[relayboard_watchdog] Target process '%s' not found. Nothing to kill; expecting external respawn.",
                  node_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "[relayboard_watchdog] Sending SIGTERM to %zu process(es) named '%s'.",
                  pids.size(), node_name_.c_str());
      terminatePids(pids, SIGTERM);
    }
  }

 private:
  std::string state_topic_;
  double timeout_sec_;
  std::string target_node_name_;
  std::string node_name_;  // target_node_name_ with leading slash stripped

  rclcpp::Subscription<neo_msgs2::msg::RelayBoardV3>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  bool received_any_;
  bool timeout_handled_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelayboardWatchdog>());
  rclcpp::shutdown();
  return 0;
}

