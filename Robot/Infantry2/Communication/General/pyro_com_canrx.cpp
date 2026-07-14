#include "pyro_com_canrx.h"

namespace pyro
{

can_rx_drv_t::~can_rx_drv_t()
{
    for (auto &node : _rx_nodes)
    {
        delete node.msg_buffer;
        node.msg_buffer = nullptr;
    }
}

can_rx_drv_t *can_rx_drv_t::instance()
{
    static can_rx_drv_t instance;
    return &instance;
}

can_rx_drv_t::rx_node_t *can_rx_drv_t::get_node(const bsp_can::which_can which,
                                                const uint32_t id)
{
    for (auto &node : instance()->_rx_nodes)
    {
        if (node.id == id && node.which == which)
            return &node;
    }
    return nullptr;
}

status_t can_rx_drv_t::subscribe(const bsp_can::which_can which, const uint32_t id)
{
    if (get_node(which, id) != nullptr)
        return PYRO_ERROR;

    can_drv_t *driver = bsp_can::get_can(which);
    if (driver == nullptr)
        return PYRO_ERROR;

    auto *new_buffer = new can_msg_buffer_t(id);
    if (new_buffer == nullptr)
        return PYRO_NO_MEMORY;

    const status_t reg_status = driver->register_rx_msg(new_buffer);
    if (reg_status != PYRO_OK)
    {
        delete new_buffer;
        return reg_status;
    }

    rx_node_t new_node{};
    new_node.which      = which;
    new_node.id         = id;
    new_node.msg_buffer = new_buffer;
    instance()->_rx_nodes.push_back(new_node);

    return PYRO_OK;
}

bool can_rx_drv_t::get_data(const bsp_can::which_can which, const uint32_t id,
                            std::array<uint8_t, 8> &data)
{
    const rx_node_t *node = get_node(which, id);
    if (!node || !node->msg_buffer)
        return false;

    if (!node->msg_buffer->is_fresh())
        return false;

    if (node->msg_buffer->get_data(data))
    {
        node->msg_buffer->mark_read();
        return true;
    }

    return false;
}

bool can_rx_drv_t::is_fresh(const bsp_can::which_can which, const uint32_t id)
{
    const rx_node_t *node = get_node(which, id);
    return node && node->msg_buffer && node->msg_buffer->is_fresh();
}

} // namespace pyro
