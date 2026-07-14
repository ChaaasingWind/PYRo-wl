#ifndef __PYRO_COM_CANRX_H__
#define __PYRO_COM_CANRX_H__

#include "pyro_bsp_can.h"
#include <array>
#include <cstdint>
#include <vector>

namespace pyro
{

class can_rx_drv_t
{
public:
    can_rx_drv_t(const can_rx_drv_t &) = delete;
    can_rx_drv_t &operator=(const can_rx_drv_t &) = delete;

    static can_rx_drv_t *instance();

    static status_t subscribe(bsp_can::which_can which, uint32_t id);
    static bool get_data(bsp_can::which_can which, uint32_t id, std::array<uint8_t, 8> &data);
    static bool is_fresh(bsp_can::which_can which, uint32_t id);

private:
    can_rx_drv_t() = default;
    ~can_rx_drv_t();

    struct rx_node_t
    {
        bsp_can::which_can which;
        uint32_t id;
        can_msg_buffer_t *msg_buffer;
    };

    static rx_node_t *get_node(bsp_can::which_can which, uint32_t id);

    std::vector<rx_node_t> _rx_nodes{};
};

} // namespace pyro

#endif // __PYRO_COM_CANRX_H__
