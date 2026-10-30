// =============================================================================
//  common.cpp —— 功能码 / 异常码 / Result 的实现
// =============================================================================
#include <modbusmech/common.hpp>

namespace modbusmech {

bool is_known_function(Function f) noexcept
{
    switch (f) {
    case Function::read_coils:
    case Function::read_discrete_inputs:
    case Function::read_holding_registers:
    case Function::read_input_registers:
    case Function::write_single_coil:
    case Function::write_single_register:
    case Function::write_multiple_coils:
    case Function::write_multiple_registers:
        return true;
    }
    return false;
}

const char* exception_name(Exception e) noexcept
{
    switch (e) {
    case Exception::none:                     return "none";
    case Exception::illegal_function:         return "illegal function";
    case Exception::illegal_data_address:     return "illegal data address";
    case Exception::illegal_data_value:       return "illegal data value";
    case Exception::server_device_failure:    return "server device failure";
    case Exception::acknowledge:              return "acknowledge";
    case Exception::server_device_busy:       return "server device busy";
    case Exception::memory_parity_error:      return "memory parity error";
    case Exception::gateway_path_unavailable: return "gateway path unavailable";
    case Exception::gateway_target_failed:    return "gateway target failed";
    }
    return "unknown exception";
}

bool Result::retryable() const noexcept
{
    if (ec) {
        return true;   // 传输失败：可以重试
    }
    // 协议异常一般不重试 —— 从站明确拒绝了，再问一遍还是同样答案。
    // 唯一的例外是"从站忙"，那是临时的。
    return exception == Exception::server_device_busy
        || exception == Exception::acknowledge;
}

std::string Result::describe() const
{
    if (ok()) {
        return "ok";
    }
    std::string s;
    if (ec) {
        s += "transport: ";
        s += ec.message();
    }
    if (exception != Exception::none) {
        if (!s.empty()) {
            s += "; ";
        }
        s += "exception: ";
        s += exception_name(exception);
    }
    return s;
}

namespace {

class ModbusCategory final : public std::error_category {
public:
    [[nodiscard]] const char* name() const noexcept override { return "modbusmech"; }

    [[nodiscard]] std::string message(int code) const override
    {
        switch (static_cast<ModbusErrc>(code)) {
        case ModbusErrc::ok:                 return "ok";
        case ModbusErrc::invalid_argument:   return "invalid argument";
        case ModbusErrc::frame_too_short:    return "frame too short";
        case ModbusErrc::frame_too_long:     return "frame too long";
        case ModbusErrc::crc_mismatch:       return "CRC mismatch";
        case ModbusErrc::bad_protocol_id:    return "bad protocol id";
        case ModbusErrc::bad_transaction:    return "bad transaction id";
        case ModbusErrc::bad_unit_id:        return "bad unit id";
        case ModbusErrc::bad_length:         return "bad length field";
        case ModbusErrc::unexpected_response:return "unexpected response";
        case ModbusErrc::timeout:            return "timeout";
        case ModbusErrc::not_connected:      return "not connected";
        case ModbusErrc::io_error:           return "I/O error";
        }
        return "unknown modbusmech error";
    }
};

} // namespace

const std::error_category& modbus_category() noexcept
{
    static const ModbusCategory kCat;
    return kCat;
}

std::error_code make_error_code(ModbusErrc e) noexcept
{
    return {static_cast<int>(e), modbus_category()};
}

} // namespace modbusmech
