#include "IntegratedControl.h"
#include <memory>

std::shared_ptr<IntegratedControl> Control;

void signal_callback_handler([[maybe_unused]] int signum)
{
    Control.reset();
}

int main()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }

    Control = std::make_shared<IntegratedControl>();
    signal(SIGINT, signal_callback_handler);
    Control->Spin();

    return 0;
}