// fr3_duo_home: record and return to a home pose for both FR3s of the Duo,
// directly over FCI with libfranka. Standalone: no ROS, no GELLO.
//
// Home poses are stored PER ROBOT IP, so left/right naming can never be swapped:
//   fr3_duo_home --save        read both robots' current joints, store as home
//   fr3_duo_home --show        print stored home poses and current joints (no motion)
//   fr3_duo_home               show the planned motion, ask for Enter, move both home
//   fr3_duo_home --yes         move without asking (for a foot pedal / script)
// Options:  --vel <rad/s> (default 0.3, max 1.0)   --ips <ip1> <ip2>
//           --file <path>  (default ~/fr3_tools/fr3_duo_home/home_poses.txt)
//
// Before moving: robots unlocked, FCI active in Desk, nothing else connected
// (GELLO, franka_ros2, Desk apps). Keep the E-stop in reach.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

using Joints = std::array<double, 7>;
constexpr double kRadToDeg = 180.0 / M_PI;

std::mutex g_print;
void say(const std::string& ip, const std::string& msg) {
  std::lock_guard<std::mutex> lock(g_print);
  std::cout << "[" << ip << "] " << msg << std::endl;
}

std::string fmt(const Joints& q) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(4);
  for (int i = 0; i < 7; ++i) s << (i ? " " : "") << q[i];
  return s.str();
}

std::map<std::string, Joints> load(const std::string& path) {
  std::map<std::string, Joints> poses;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream s(line);
    std::string ip;
    Joints q{};
    if (s >> ip >> q[0] >> q[1] >> q[2] >> q[3] >> q[4] >> q[5] >> q[6]) poses[ip] = q;
  }
  return poses;
}

bool save(const std::string& path, const std::map<std::string, Joints>& poses) {
  std::ofstream f(path);
  if (!f) return false;
  f << "# FR3 home poses: <robot_ip> q1..q7 [rad]. Written by fr3_duo_home --save\n";
  for (const auto& [ip, q] : poses) f << ip << " " << fmt(q) << "\n";
  return true;
}

bool move_home(franka::Robot& robot, const std::string& ip, const Joints& q_goal, double v_max) {
  try {
    robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    Joints q0{};
    double T = 0.0, t = 0.0;
    robot.control([&](const franka::RobotState& s, franka::Duration dt) -> franka::JointPositions {
      t += dt.toSec();
      if (t == 0.0) {
        q0 = s.q_d;
        double d = 0.0;
        for (int i = 0; i < 7; ++i) d = std::max(d, std::abs(q_goal[i] - q0[i]));
        T = std::max(3.0, 1.875 * d / v_max);  // min-jerk: peak speed = 1.875 * d / T
      }
      const double x = std::min(t / T, 1.0);
      const double b = x * x * x * (10.0 - 15.0 * x + 6.0 * x * x);
      Joints q{};
      for (int i = 0; i < 7; ++i) q[i] = q0[i] + (q_goal[i] - q0[i]) * b;
      franka::JointPositions out(q);
      return x >= 1.0 ? franka::MotionFinished(out) : out;
    });
    say(ip, "at home");
    return true;
  } catch (const franka::Exception& e) {
    say(ip, std::string("FAILED: ") + e.what());
    return false;
  }
}

int main(int argc, char** argv) {
  std::vector<std::string> ips = {"192.168.1.51", "192.168.1.50"};
  std::string file = std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") +
                     "/fr3_tools/fr3_duo_home/home_poses.txt";
  double v_max = 0.3;
  bool do_save = false, do_show = false, yes = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--save") do_save = true;
    else if (a == "--show") do_show = true;
    else if (a == "--yes" || a == "-y") yes = true;
    else if (a == "--vel" && i + 1 < argc) v_max = std::atof(argv[++i]);
    else if (a == "--file" && i + 1 < argc) file = argv[++i];
    else if (a == "--ips" && i + 2 < argc) ips = {argv[i + 1], argv[i + 2]}, i += 2;
    else { std::cerr << "unknown argument: " << a << " (see the header of fr3_duo_home.cpp)\n"; return 2; }
  }
  if (v_max <= 0.0 || v_max > 1.0) { std::cerr << "--vel must be in (0, 1] rad/s\n"; return 2; }

  // Connect to both robots and read where they are (no motion).
  std::vector<std::unique_ptr<franka::Robot>> robots;
  std::map<std::string, Joints> current;
  for (const auto& ip : ips) {
    try {
      robots.emplace_back(std::make_unique<franka::Robot>(ip));
      robots.back()->automaticErrorRecovery();
      current[ip] = robots.back()->readOnce().q;
    } catch (const franka::Exception& e) {
      say(ip, std::string("cannot connect: ") + e.what());
      return 1;
    }
  }

  if (do_save) {
    if (!save(file, current)) { std::cerr << "cannot write " << file << "\n"; return 1; }
    for (const auto& ip : ips) say(ip, "home saved: " + fmt(current[ip]));
    std::cout << "Saved to " << file << std::endl;
    return 0;
  }

  auto homes = load(file);
  for (const auto& ip : ips) {
    if (!homes.count(ip)) {
      std::cerr << "No home pose stored for " << ip << " in " << file
                << ".\nMove both robots to the home pose (e.g. hand-guide them in Desk) and run:"
                << "  fr3_duo_home --save\n";
      return 1;
    }
  }

  // Show the plan.
  std::cout << std::fixed << std::setprecision(1);
  for (const auto& ip : ips) {
    const Joints &q = current[ip], &h = homes[ip];
    double d = 0.0;
    std::ostringstream s;
    s << std::fixed << std::setprecision(1) << "joint change [deg]:";
    for (int i = 0; i < 7; ++i) {
      s << " " << (h[i] - q[i]) * kRadToDeg;
      d = std::max(d, std::abs(h[i] - q[i]));
    }
    say(ip, s.str());
    say(ip, "largest " + std::to_string(int(std::round(d * kRadToDeg))) + " deg, takes about " +
                std::to_string(int(std::round(std::max(3.0, 1.875 * d / v_max)))) + " s");
  }
  if (do_show) return 0;

  if (!yes) {
    std::cout << "Press Enter to move BOTH robots home (Ctrl+C to cancel): " << std::flush;
    std::string dummy;
    std::getline(std::cin, dummy);
  }

  std::vector<std::thread> threads;
  std::vector<char> ok(ips.size(), 0);
  for (size_t k = 0; k < ips.size(); ++k)
    threads.emplace_back([&, k] { ok[k] = move_home(*robots[k], ips[k], homes[ips[k]], v_max); });
  for (auto& th : threads) th.join();

  if (std::all_of(ok.begin(), ok.end(), [](char v) { return v; })) {
    std::cout << "Both robots at home." << std::endl;
    return 0;
  }
  std::cerr << "Homing incomplete." << std::endl;
  return 1;
}
