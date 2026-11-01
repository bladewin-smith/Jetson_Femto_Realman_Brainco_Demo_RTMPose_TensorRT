#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Terminal {
    const char* name;
};

std::string shellQuote(const std::string& value) {
    std::string out = "'";
    for (char ch : value) {
        if (ch == '\'') {
            out += "'\"'\"'";
        } else {
            out.push_back(ch);
        }
    }
    out += "'";
    return out;
}

bool commandExists(const std::string& cmd) {
    std::string testCmd = "command -v " + shellQuote(cmd) + " >/dev/null 2>&1";
    return std::system(testCmd.c_str()) == 0;
}

std::string findTerminal() {
    const std::vector<Terminal> terminals = {
        {"gnome-terminal"},
        {"konsole"},
        {"xfce4-terminal"},
        {"mate-terminal"},
        {"xterm"}
    };

    for (const auto& terminal : terminals) {
        if (commandExists(terminal.name)) {
            return std::string(terminal.name);
        }
    }
    return {};
}

std::string getHomeDir() {
    const char* home = std::getenv("HOME");
    if (home && *home) {
        return std::string(home);
    }
    return "/tmp";
}

void printUsage() {
    std::cout << "Usage: rm_brainco_orbbec_demo_terminal [--command <cmd>] [--log <path>]\n";
    std::cout << "  default command: /usr/local/bin/rm_brainco_orbbec_demo\n";
    std::cout << "  default log:     ~/.local/share/rm_brainco_orbbec_demo/autostart.log\n";
}

} // namespace

int main(int argc, char* argv[]) {
    std::string command = "/usr/local/bin/rm_brainco_orbbec_demo";
    std::string logFile = getHomeDir() + "/.local/share/rm_brainco_orbbec_demo/autostart.log";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        }
        if (arg == "--command" && i + 1 < argc) {
            command = argv[++i];
            continue;
        }
        if (arg == "--log" && i + 1 < argc) {
            logFile = argv[++i];
            continue;
        }
        std::cerr << "Unknown arg: " << arg << "\n";
        printUsage();
        return 1;
    }

    const std::string homeDir = getHomeDir();
    const std::string scriptPath = "/tmp/rm_brainco_orbbec_demo_launch.sh";
    const std::string logDir = fs::path(logFile).parent_path().string();

    try {
        fs::create_directories(logDir);
    } catch (const fs::filesystem_error&) {
        std::cerr << "Failed to create log directory: " << logDir << "\n";
        return 2;
    }

    std::ofstream script(scriptPath, std::ios::trunc);
    if (!script.is_open()) {
        std::cerr << "Failed to create launch script: " << scriptPath << "\n";
        return 3;
    }
    script << "#!/usr/bin/env bash\n";
    script << "set -euo pipefail\n";
    script << "mkdir -p " << shellQuote(logDir) << "\n";
    script << "echo \"=================================================\"\n";
    script << "echo \"RM BrainCo Orbbec Demo 启动器\"\n";
    script << "echo \"按下 H 键可让机械臂回到设定原位（由程序内键盘监听处理）\"\n";
    script << "echo \"按 Ctrl+C 退出程序\"\n";
    script << "echo \"启动命令: " << command << "\"\n";
    script << "echo \"日志文件: " << logFile << "\"\n";
    script << "export DISPLAY=${DISPLAY:-:0}\n";
    script << "echo \"使用 DISPLAY=$DISPLAY\"\n";
    script << "exec " << shellQuote(command) << " >> " << shellQuote(logFile) << " 2>&1\n";
    script.close();

    std::system((std::string("chmod +x ") + shellQuote(scriptPath)).c_str());

    std::string terminal = findTerminal();
    if (terminal.empty()) {
        std::cerr << "No terminal emulator found on this system.\n";
        std::cerr << "Please install gnome-terminal / konsole / xfce4-terminal / mate-terminal / xterm.\n";
        return 4;
    }

    std::string launchCmd;
    if (terminal == "gnome-terminal") {
        launchCmd = "gnome-terminal -- bash " + shellQuote(scriptPath);
    } else if (terminal == "konsole") {
        launchCmd = "konsole -e bash " + shellQuote(scriptPath);
    } else if (terminal == "xfce4-terminal") {
        launchCmd = "xfce4-terminal --command=" + shellQuote(std::string("bash ") + scriptPath);
    } else if (terminal == "mate-terminal") {
        launchCmd = "mate-terminal -e bash " + shellQuote(scriptPath);
    } else {
        launchCmd = "xterm -hold -e bash " + shellQuote(scriptPath);
    }
    std::cout << "Launching command in new terminal...\n";
    std::cout << "command: " << command << "\n";
    std::cout << "log: " << logFile << "\n";
    std::cout << "terminal: " << launchCmd << "\n";

    const int rc = std::system(launchCmd.c_str());
    if (rc != 0) {
        std::cerr << "Terminal launch failed, code=" << rc << "\n";
        return 5;
    }

    return 0;
}
