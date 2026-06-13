// On Windows the app is built as a GUI-subsystem executable (see CMakeLists.txt)
// so launching it from Explorer / a Start-menu shortcut no longer spawns a CMD
// window. To keep the "launched from a terminal still prints logs" behavior, we
// reattach to the parent console when there is one.
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <cstdio>
#endif

#include "ViewerWindow.h"
#include <csignal>
#include "cmdparser.h"
#include "Log.h"

std::atomic<bool> Terminated = ATOMIC_VAR_INIT(false);

void SignalHandler(int)
{
    Terminated = true;
}

#ifdef _WIN32
// If the process was started from an existing console (cmd / PowerShell / CI),
// adopt it and point the C/C++ standard streams at it so logs appear there.
// When launched from Explorer there is no parent console and this is a no-op,
// so no console window pops up.
static void AttachParentConsole()
{
    if (AttachConsole(ATTACH_PARENT_PROCESS))
    {
        FILE* dummy = nullptr;
        freopen_s(&dummy, "CONOUT$", "w", stdout);
        freopen_s(&dummy, "CONOUT$", "w", stderr);
        freopen_s(&dummy, "CONIN$", "r", stdin);
        std::ios::sync_with_stdio(true);
        std::cout.clear();
        std::cerr.clear();
        std::cin.clear();
    }
}
#endif

int main(int argc, char **argv)
{
#ifdef _WIN32
    AttachParentConsole();
#endif
    // Tee std::cout/std::cerr into the in-app log console. Installed before any
    // logging so startup messages are captured too; the static instance restores
    // the original buffers at process exit even on early-exit paths.
    LogConsole::Get().Install();

    try
    {
        std::string inputFilePath = "";
        CmdParser::OptionParser cmd_parser;
        cmd_parser.RegisterOption("-h|--help", "Prints this help", [&]()
                                {
                                    std::cout << "ir-tracking-app [options] <realsense_recording.bag> \n"
                                        << std::endl;
                                    cmd_parser.PrintOptions();
                                    exit(0); 
                                });
        cmd_parser.RegisterOption("-i|--input", "RealSense recording file path", 1,
                                  [&](const std::vector<char *> &args)
                                {
                                    inputFilePath = args[0];
                                });

        try
        {
            cmd_parser.ParseCmd(argc, argv);
        }
        catch (CmdParser::ArgumentError &e)
        {
            std::cerr << e.option() << ": " << e.what() << std::endl;
            return 1;
        }
        ViewerWindow viewer;
        viewer.Initialize(inputFilePath);
        std::signal(SIGINT, SignalHandler);

        while (!viewer.IsTerminated() && !Terminated)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        viewer.Shutdown();
    }
    catch (const std::exception &e)
    {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        LogConsole::Get().Restore();
        return EXIT_FAILURE;
    }

    LogConsole::Get().Restore();
    return EXIT_SUCCESS;
}
