#include "ViewerWindow.h"
#include <csignal>

std::atomic<bool> Terminated = ATOMIC_VAR_INIT(false);

void SignalHandler(int)
{
    Terminated = true;
}

int main() {
    try {
        ViewerWindow viewer;
        viewer.Initialize();
        std::signal(SIGINT, SignalHandler);

        while (!viewer.IsTerminated() && !Terminated) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        viewer.Shutdown();
    } catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
