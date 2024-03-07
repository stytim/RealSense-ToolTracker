#include "ViewerWindow.h"
#include <csignal>
#include "cmdparser.h"

std::atomic<bool> Terminated = ATOMIC_VAR_INIT(false);

void SignalHandler(int)
{
    Terminated = true;
}

int main(int argc, char **argv)
{
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
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
