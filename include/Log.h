#pragma once

#ifndef LOG_CONSOLE_H
#define LOG_CONSOLE_H

#include <deque>
#include <memory>
#include <mutex>
#include <streambuf>
#include <string>

// In-app log console.
//
// Install() tees std::cout / std::cerr so every line written through them is
// BOTH forwarded to the original stream (so a launch from a terminal keeps
// printing logs as before) AND captured into a bounded ring buffer that the GUI
// renders as a docked panel at the bottom of the window. This means the 50-odd
// existing std::cout/std::cerr call sites need no changes — they light up the
// in-app console automatically.
//
// Thread-safe: AddLine() may be called concurrently from worker threads (the
// UDP / CSV / tracking threads all log), while DrawContents() reads on the GUI
// thread.
class LogConsole
{
public:
    enum class Level
    {
        Info,    // captured from std::cout
        Error    // captured from std::cerr
    };

    struct Entry
    {
        Level level;
        std::string text; // includes a leading "HH:MM:SS " timestamp
    };

    static LogConsole& Get();

    // Redirect std::cout / std::cerr through the tee. Idempotent.
    void Install();
    // Restore the original stream buffers. Idempotent; safe to call at shutdown.
    void Restore();

    // Append a fully-formed log line (no trailing newline). Thread-safe.
    void AddLine(Level level, const std::string& text);

    void Clear();

    // Render the console body (toolbar + scrolling region) into the *current*
    // ImGui window. The caller owns the window (position / size / Begin/End).
    void DrawContents();

    LogConsole(const LogConsole&) = delete;
    LogConsole& operator=(const LogConsole&) = delete;

private:
    LogConsole() = default;
    ~LogConsole();

    std::mutex mutex_;
    std::deque<Entry> entries_;
    std::size_t maxEntries_ = 2000;

    bool installed_ = false;
    bool autoScroll_ = true;

    // Saved originals so Restore() can put them back.
    std::streambuf* originalCout_ = nullptr;
    std::streambuf* originalCerr_ = nullptr;
    // Tee buffers that wrap the originals; owned here.
    std::unique_ptr<std::streambuf> coutTee_;
    std::unique_ptr<std::streambuf> cerrTee_;
};

#endif // LOG_CONSOLE_H
