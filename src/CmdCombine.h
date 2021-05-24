#ifndef CMD_COMBINE_H
#define CMD_COMBINE_H

#include <stdlib.h>
#include <string>
#include <iostream>
#include <vector>

#if (defined _WIN32 && defined RF62X_LIBRARY)
#define API_EXPORT __declspec(dllexport)
#else
#define API_EXPORT
#endif

namespace DJIR_SDK {

API_EXPORT class CmdCombine
{
public:
    CmdCombine();
    ~CmdCombine();

    std::vector<uint8_t> combine(
            uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id,
            std::vector<uint8_t> data);

private:
    std::vector<uint8_t> seq_num();

};

}

#endif //CMD_COMBINE_H
