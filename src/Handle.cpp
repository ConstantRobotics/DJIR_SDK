#include "Handle.h"

#include "USBCAN_SDK.h"
#include "custom_crc16.h"
#include "custom_crc32.h"

DJIR_SDK::DataHandle::DataHandle(void *can_connection)
{
    _stopped = false;
    _input_position_ready_flag = false;
    _dev = can_connection;
    _cmd_list = std::vector<std::vector<uint8_t>>();
    _rsps = std::vector<std::string>();
}

void DJIR_SDK::DataHandle::start()
{
    _thread = std::thread(&DataHandle::run, this);
}

void DJIR_SDK::DataHandle::stop()
{
    _stopped = true;
    _thread.join();
}

void DJIR_SDK::DataHandle::add_cmd(std::vector<uint8_t> cmd)
{
    _rdcontent_lock.lock();
    _cmd_list.push_back(cmd);
    if (_cmd_list.size() > 10)
        _cmd_list.erase(_cmd_list.begin());
    _rdcontent_lock.unlock();
}

bool DJIR_SDK::DataHandle::get_position(int16_t &yaw, int16_t &roll, int16_t &pitch, uint16_t timeout_ms)
{
    // Wait data.
    std::unique_lock<std::mutex> lk(_input_position_mutex);
    while (!_input_position_ready_flag)
    {
        if (_input_position_cond_var.wait_for(
                    lk, std::chrono::milliseconds(timeout_ms)) == std::cv_status::timeout)
        {
            // Reset data ready flag.
            _input_position_ready_flag = false;
            // Unlock mutex.
            lk.unlock();

            return false;
        }
    }
    lk.unlock();

    // Reset data ready flag.
    _input_position_ready_flag = false;
    yaw = _yaw;
    roll = _roll;
    pitch = _pitch;
    return true;
}

void DJIR_SDK::DataHandle::run()
{
    std::vector<uint8_t> v1_pack_list = std::vector<uint8_t>();
    size_t pack_len = 0;
    int step = 0;
    std::string canid_raw_str = "";
    std::string canid_str = "";
    USBCAN_SDK::CANConnection* dev = (USBCAN_SDK::CANConnection*)_dev;
    while (!_stopped)
    {
        auto frame = dev->get_tunnel()->pop_data_from_recv_queue();
        for (size_t i = 0; i < frame.size(); i++)
        {
            if (step == 0)
            {
                if((uint8_t)frame[i] == 0xAA)
                {
                    v1_pack_list.push_back(frame[i]);
                    step = 1;
                }

            }else if(step == 1)
            {
                pack_len = int(frame[i]);
                v1_pack_list.push_back(frame[i]);
                step = 2;
            }else if(step == 2)
            {
                pack_len |= ((int(frame[i]) & 0x3) << 8);
                v1_pack_list.push_back(frame[i]);
                step = 3;
            }else if(step == 3)
            {
                v1_pack_list.push_back(frame[i]);
                if (v1_pack_list.size() == 12)
                {
                    if (_check_head_crc(v1_pack_list))
                        step = 4;
                    else
                    {
                        step = 0;
                        v1_pack_list.clear();
                    }

                }

            }else if(step == 4)
            {
                v1_pack_list.push_back(frame[i]);
                if (v1_pack_list.size() == pack_len)
                {
                    step = 0;
                    if (_check_pack_crc(v1_pack_list))
                        _process_cmd(v1_pack_list);
                    v1_pack_list.clear();
                }
            }else
            {
                step = 0;
                v1_pack_list.clear();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void DJIR_SDK::DataHandle::_process_cmd(std::vector<uint8_t> data)
{
    uint8_t cmd_type = (uint8_t)data[3];
    bool is_ok = false;
    uint8_t cmd_key[2] = {0, 0};

    // If it is a response frame, need to check the corresponding send command
    if (cmd_type == 0x20)
    {
        _rdcontent_lock.lock();
        for(size_t i = 0; i < _cmd_list.size(); i++)
        {
            std::vector<uint8_t> cmd = _cmd_list[i];
            if (cmd.size() >= 10)
            {
                uint16_t last_cmd_crc = *((uint16_t*)&cmd.data()[8]);
                uint16_t data_crc = *((uint16_t*)&data.data()[8]);
                if (last_cmd_crc == data_crc)
                {
                    cmd_key[0] = (uint8_t)cmd[12];
                    cmd_key[1] = (uint8_t)cmd[13];
                    _cmd_list.erase(_cmd_list.begin() + i);
                    is_ok = true;
                    break;
                }
            }
        }
        _rdcontent_lock.unlock();
    }else
    {
        cmd_key[0] = (uint8_t)data[12];
        cmd_key[1] = (uint8_t)data[13];
        is_ok = true;
    }

    if (is_ok)
    {
        switch (*(uint16_t*)&cmd_key[0]) {
        case 0x000e:
        {
            printf("get posControl request\n");
            break;
        }
        case 0x020e:
        {
//            printf("get getGimbalInfo request\n");
//            if (data[13] == 0x00)
//                std::cout << "Data is not ready\n" << std::endl;
//            if (data[13] == 0x01)
//                std::cout << "The current angle is attitude angle\n"<<std::endl;
//            if (data[13] == 0x02)
//                std::cout << "The current angle is joint angle\n" << std::endl;

            _yaw = *(int16_t*)&data.data()[14];
            _roll = *(int16_t*)&data.data()[16];
            _pitch = *(int16_t*)&data.data()[18];

//            std::cout << "yaw = " << _yaw << " roll = " << _roll << " pitch = " << _pitch << std::endl;

            _input_position_ready_flag = true;
            _input_position_cond_var.notify_one();


            break;
        }
        default:
        {
            printf("get unknown request\n");
            break;
        }
        }
    }

}

bool DJIR_SDK::DataHandle::_check_head_crc(std::vector<uint8_t> data)
{
    crc16_t crc16;
    crc16 = crc16_init();
    crc16 = crc16_update(crc16, data.data(), 10);
    crc16 = crc16_finalize(crc16);

    uint16_t recv_crc = (*(uint16_t*)&data.data()[data.size() - 2]);

    if (crc16 == recv_crc)
        return true;
    return false;

}

bool DJIR_SDK::DataHandle::_check_pack_crc(std::vector<uint8_t> data)
{
    crc32_t crc32;
    crc32 = crc32_init();
    crc32 = crc32_update(crc32, data.data(), data.size() - 4);
    crc32 = crc32_finalize(crc32);

    uint32_t recv_crc =  (*(uint32_t*)&data.data()[data.size() - 4]);

    if (crc32 == recv_crc)
        return true;
    return false;
}
