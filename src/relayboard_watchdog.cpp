// Watchdog that monitors the relayboard state topic for a specific duration.
// After the timeout: if unhealthy (no message received), restart the target and exit;
// if healthy, exit the watchdog only.

#include <chrono>
#include <cstdlib>
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

void terminatePids(const std::vector<pid_t> &pids, int sig = SIGKILL) {
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
        received_any_(false) {
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
                "Started. Monitoring '%s', target '%s' (proc='%s'), timeout=%.2fs. State: waiting for first message or timeout.",
                state_topic_.c_str(), target_node_name_.c_str(), node_name_.c_str(), timeout_sec_);
  }

 private:
  void onTimer() {
    const double elapsed_sec = (this->get_clock()->now() - start_time_).seconds();

    // Verbose tick (for testing)
    const char * health_str = received_any_ ? "healthy" : "unhealthy (no message yet)";
    const std::vector<pid_t> pids = findPidsByTarget(node_name_, node_name_);
    RCLCPP_INFO(this->get_logger(),
                "elapsed=%.3fs/%.2fs state=%s target_pids=[%s]",
                elapsed_sec, timeout_sec_, health_str, pidsToString(pids).c_str());

    const bool healthy = received_any_;
    if (elapsed_sec < timeout_sec_) {
      if (healthy) {
        // Healthy within timeout: terminate the watchdog.
        RCLCPP_INFO(this->get_logger(), "Target is healthy. Shutting down watchdog.");
        rclcpp::shutdown();
        std::exit(0);
      }
      // Not healthy yet: keep waiting for time elapsed.
      return;
    }

    // Timeout reached and still not healthy: restart target, then terminate watchdog.
    RCLCPP_INFO(this->get_logger(),
                "Timeout reached (%.2fs). Target unhealthy.",
                timeout_sec_);
    RCLCPP_INFO(this->get_logger(),
                "Shutting down target node '%s' (SIGKILL).",
                node_name_.c_str());
    restartTarget();
    RCLCPP_INFO(this->get_logger(), "Shutting down watchdog.");
    rclcpp::shutdown();
    std::exit(0);
  }

  void restartTarget() {
    const std::vector<pid_t> pids = findPidsByTarget(node_name_, node_name_);
    if (pids.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Target process '%s' not found. Nothing to kill; expecting external respawn.",
                  node_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Sending SIGTERM to %zu process(es) named '%s'.",
                  pids.size(), node_name_.c_str());
      terminatePids(pids, SIGKILL);
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelayboardWatchdog>());
  rclcpp::shutdown();
  return 0;
}

