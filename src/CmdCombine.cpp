#include "CmdCombine.h"
#include "custom_crc16.h"
#include "custom_crc32.h"

DJIR_SDK::CmdCombine::CmdCombine()
{

}

DJIR_SDK::CmdCombine::~CmdCombine()
{

}

std::vector<uint8_t> DJIR_SDK::CmdCombine::combine(uint8_t cmd_type, uint8_t cmd_set, uint8_t cmd_id, std::vector<uint8_t> data)
{
/* DJI R SDK Protocol Description
 *
 * 2.1 Data Format
 * +----------------------------------------------+------+------+------+
 * |                     PREFIX                   | CRC  | DATA | CRC  |
 * |------+----------+-------+------+------+------+------+------+------|
 * |SOF   |Ver/Length|CmdType|ENC   |RES   |SEQ   |CRC-16|DATA  |CRC-32|
 * |------|----------|-------|------|------|------|------|------|------|
 * |1-byte|2-byte    |1-byte |1-byte|3-byte|2-byte|2-byte|n-byte|4-byte|
 * +------+----------+-------+------+------+------+------+------+------+
 *
 * 2.2 Data Segment (field DATA in 2.1 Data Format)
 * +---------------------+
 * |           DATA      |
 * |------+------+-------|
 * |CmdSet|CmdID |CmdData|
 * |------|------|-------|
 * |1-byte|1-byte|n-byte |
 * +------+------+-------+
 */


    int prefix_length = 10;
    int crc16_length = 2;
    int data_length = 2 + data.size();
    int crc32_length = 4;

    uint16_t cmd_length = prefix_length + crc16_length + data_length + crc32_length;
    auto seqnum = seq_num();


    int i = 0;
    std::vector<uint8_t> cmd = std::vector<uint8_t>(cmd_length);
    cmd[i] = 0xAA; i++;
    cmd[i] = ((uint8_t*)&cmd_length)[0]; i++;
    cmd[i] = ((uint8_t*)&cmd_length)[1]; i++;
    cmd[i] = cmd_type; i++;
    cmd[i] = 0x00; i++; // enc
    cmd[i] = 0x00; i++; // res1
    cmd[i] = 0x00; i++; // res2
    cmd[i] = 0x00; i++; // res3
    cmd[i] = seqnum[0]; i++; // seqnum [1:]
    cmd[i] = seqnum[1]; i++; // seqnum [0:1]

    crc16_t crc16;
    crc16 = crc16_init();
    crc16 = crc16_update(crc16, cmd.data(), i);
    crc16 = crc16_finalize(crc16);

    cmd[i] = ((uint8_t*)&crc16)[0]; i++; // crc16 [1:]
    cmd[i] = ((uint8_t*)&crc16)[1]; i++; // crc16 [0:1]

    cmd[i] = cmd_set; i++; // crc16 [0:1]
    cmd[i] = cmd_id; i++; // crc16 [0:1]
    for (size_t j = 0; j < data.size(); j++)
    {
        cmd[i] = data[j]; i++;
    }
    crc32_t crc32;
    crc32 = crc32_init();
    crc32 = crc32_update(crc32, cmd.data(), i);
    crc32 = crc32_finalize(crc32);

    cmd[i] = ((uint8_t*)&crc32)[0]; i++; // crc32 [3:]
    cmd[i] = ((uint8_t*)&crc32)[1]; i++; // crc32 [2:3]
    cmd[i] = ((uint8_t*)&crc32)[2]; i++; // crc32 [1:2]
    cmd[i] = ((uint8_t*)&crc32)[3]; i++; // crc32 [0:1]

    return cmd;
}

std::vector<uint8_t> DJIR_SDK::CmdCombine::seq_num()
{
    static uint16_t Seq_Init_Data = 0x2210;
    if (Seq_Init_Data >= 0xFFFD)
        Seq_Init_Data = 0x0002;
    Seq_Init_Data += 1;

    std::vector<uint8_t> ret = std::vector<uint8_t>();
    uint8_t* seq = (uint8_t*)&Seq_Init_Data;
    ret.push_back(seq[1]);
    ret.push_back(seq[0]);
    return ret;
}
