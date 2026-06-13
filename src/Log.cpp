#include "Log.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <vector>

#include <imgui.h>

namespace
{
    std::string Timestamp()
    {
        using namespace std::chrono;
        const auto now = system_clock::now();
        const std::time_t t = system_clock::to_time_t(now);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);
        return std::string(buf);
    }

    // A streambuf that forwards every character to a wrapped "original" buffer
    // (so the terminal still sees output) while assembling complete lines and
    // pushing them into the LogConsole ring buffer.
    //
    // All writes are serialized on its own mutex_, which also makes concurrent
    // std::cout/std::cerr writes from worker threads non-interleaving at the
    // character level. The nesting order is always (tee mutex_ -> LogConsole
    // mutex); LogConsole never calls back into the tee, so there is no deadlock.
    class TeeStreamBuf : public std::streambuf
    {
    public:
        TeeStreamBuf(std::streambuf* original, LogConsole::Level level, LogConsole* console)
            : original_(original), level_(level), console_(console)
        {
        }

    protected:
        int overflow(int ch) override
        {
            if (ch == traits_type::eof())
            {
                return traits_type::not_eof(ch);
            }
            std::lock_guard<std::mutex> lock(mutex_);
            const char c = static_cast<char>(ch);
            if (original_)
            {
                original_->sputc(c);
            }
            Consume(c);
            return ch;
        }

        std::streamsize xsputn(const char* s, std::streamsize n) override
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (original_)
            {
                original_->sputn(s, n);
            }
            for (std::streamsize i = 0; i < n; ++i)
            {
                Consume(s[i]);
            }
            return n;
        }

        int sync() override
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return original_ ? original_->pubsync() : 0;
        }

    private:
        // Caller holds mutex_.
        void Consume(char c)
        {
            if (c == '\n')
            {
                // Drop a trailing CR so Windows "\r\n" lines render cleanly.
                if (!line_.empty() && line_.back() == '\r')
                {
                    line_.pop_back();
                }
                console_->AddLine(level_, line_);
                line_.clear();
            }
            else
            {
                line_.push_back(c);
            }
        }

        std::mutex mutex_;
        std::streambuf* original_ = nullptr;
        LogConsole::Level level_;
        LogConsole* console_ = nullptr;
        std::string line_;
    };
} // namespace

LogConsole& LogConsole::Get()
{
    static LogConsole instance;
    return instance;
}

LogConsole::~LogConsole()
{
    Restore();
}

void LogConsole::Install()
{
    if (installed_)
    {
        return;
    }
    originalCout_ = std::cout.rdbuf();
    originalCerr_ = std::cerr.rdbuf();
    coutTee_ = std::make_unique<TeeStreamBuf>(originalCout_, Level::Info, this);
    cerrTee_ = std::make_unique<TeeStreamBuf>(originalCerr_, Level::Error, this);
    std::cout.rdbuf(coutTee_.get());
    std::cerr.rdbuf(cerrTee_.get());
    installed_ = true;
}

void LogConsole::Restore()
{
    if (!installed_)
    {
        return;
    }
    // Put the originals back before destroying the tees, so nothing keeps a
    // dangling rdbuf pointer.
    std::cout.rdbuf(originalCout_);
    std::cerr.rdbuf(originalCerr_);
    coutTee_.reset();
    cerrTee_.reset();
    originalCout_ = nullptr;
    originalCerr_ = nullptr;
    installed_ = false;
}

void LogConsole::AddLine(Level level, const std::string& text)
{
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.push_back(Entry{level, Timestamp() + " " + text});
    while (entries_.size() > maxEntries_)
    {
        entries_.pop_front();
    }
}

void LogConsole::Clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
}

void LogConsole::DrawContents()
{
    // Toolbar.
    if (ImGui::SmallButton("Clear"))
    {
        Clear();
    }
    ImGui::SameLine();
    const bool copy = ImGui::SmallButton("Copy");
    ImGui::SameLine();
    ImGui::Checkbox("Auto-scroll", &autoScroll_);
    ImGui::Separator();

    // Snapshot the lines under the lock, then render WITHOUT holding it. This
    // keeps the (non-recursive) LogConsole mutex entirely off the ImGui call
    // path, so there is no way for an ImGui code path that happens to write to
    // cout/cerr to re-enter AddLine() and self-deadlock; it also means a worker
    // thread logging via cout/cerr never blocks on the GUI render. The copy is
    // bounded by maxEntries_ and trivially cheap for a log view.
    std::vector<Entry> snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot.assign(entries_.begin(), entries_.end());
    }

    if (copy)
    {
        std::string all;
        for (const auto& e : snapshot)
        {
            all += e.text;
            all.push_back('\n');
        }
        ImGui::SetClipboardText(all.c_str());
    }

    // Scrolling region. Reserve nothing extra — fill the rest of the window.
    ImGui::BeginChild("LogScrollRegion", ImVec2(0, 0), false,
                      ImGuiWindowFlags_HorizontalScrollbar);

    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 1));
    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(snapshot.size()));
    while (clipper.Step())
    {
        for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i)
        {
            const Entry& e = snapshot[static_cast<std::size_t>(i)];
            const bool isError = (e.level == Level::Error);
            if (isError)
            {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
            }
            ImGui::TextUnformatted(e.text.c_str());
            if (isError)
            {
                ImGui::PopStyleColor();
            }
        }
    }
    clipper.End();
    ImGui::PopStyleVar();

    // Keep pinned to the bottom while the user hasn't scrolled away.
    if (autoScroll_ && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
    {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
}
