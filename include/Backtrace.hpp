#pragma once
#include <string>

namespace LocalizationToolkit::program
{
    /**
     *
     * @param skip
     * @return
     */
    std::string makeBacktrace(int skip);

    /**
     *
     * @param signal
     */
    [[maybe_unused]] void printBacktraceAndExitHandler(int signal);
}
