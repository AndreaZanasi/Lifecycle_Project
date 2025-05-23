#ifndef SIGNAL_HANDLER_HPP
#define SIGNAL_HANDLER_HPP

#include <csignal>
#include <csetjmp>
#include <array>
#include <iostream>
#include <string>
#include <unordered_map>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"

class SignalHandler
{
public:
  static void init()
  {
    struct sigaction sa {};
    sa.sa_sigaction = handler;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);

    for (int sig : signals) {
      sigaction(sig, &sa, nullptr);
    }
  }

  static void set_jump_buffer(sigjmp_buf& buf)
  {
    jump_buf_ptr = &buf;
  }

private:
  static inline sigjmp_buf* jump_buf_ptr = nullptr;

  static void handler(int signum, siginfo_t* info, void* context)
  {
    const char* signal_name = strsignal(signum);
    RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Caught signal \033[1;31m%d (%s)\033[0m", signum, signal_name);

    if (info) {
      RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Signal code: %d", info->si_code);

      if (signum == SIGSEGV || signum == SIGBUS || signum == SIGFPE) {
        RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Fault address: %p", info->si_addr);

        switch (info->si_code) {
            case SEGV_MAPERR:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Address not mapped to object (SEGV_MAPERR)");
                break;
            case SEGV_ACCERR:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Invalid permissions for mapped object (SEGV_ACCERR)");
                break;
            case FPE_FLTDIV:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Floating point divide by zero (FPE_FLTDIV)");
                break;
            case FPE_FLTOVF:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Floating point overflow (FPE_FLTOVF)");
                break;
            case FPE_FLTUND:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Floating point underflow (FPE_FLTUND)");
                break;
            case FPE_FLTRES:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Floating point inexact result (FPE_FLTRES)");
                break;
            case FPE_FLTINV:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Invalid floating point operation (FPE_FLTINV)");
                break;
            case FPE_FLTSUB:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Subscript out of range (FPE_FLTSUB)");
                break;

            default:
                RCLCPP_FATAL(rclcpp::get_logger("SignalHandler"), "Cause: Unknown (si_code: %d)", info->si_code);
                break;
            }
      }
    }

    if (jump_buf_ptr) {
      siglongjmp(*jump_buf_ptr, 1);
    }
  }

  static constexpr std::array<int, 3> signals = {SIGFPE, SIGABRT, SIGSEGV};
};

#endif  // SIGNAL_HANDLER_HPP
