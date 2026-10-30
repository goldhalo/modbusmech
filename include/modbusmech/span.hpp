// =============================================================================
//  modbusmech - 轻量 span
// =============================================================================
//  与其他几个库保持一致：只读的最小实现，不做生命周期检查。
// =============================================================================
#pragma once

#include <cstddef>
#include <type_traits>

namespace modbusmech {

template <typename T>
class span {
public:
    using element_type = T;
    using value_type   = std::remove_cv_t<T>;
    using size_type    = std::size_t;

    constexpr span() noexcept = default;
    constexpr span(T* data, size_type len) noexcept : m_data(data), m_size(len) {}

    template <typename U, std::size_t N>
    constexpr span(T (&arr)[N]) noexcept : m_data(arr), m_size(N) {}

    [[nodiscard]] constexpr T*        data() const noexcept { return m_data; }
    [[nodiscard]] constexpr size_type size() const noexcept { return m_size; }
    [[nodiscard]] constexpr bool      empty() const noexcept { return m_size == 0; }
    [[nodiscard]] constexpr T&        operator[](size_type i) const noexcept { return m_data[i]; }
    [[nodiscard]] constexpr T*        begin() const noexcept { return m_data; }
    [[nodiscard]] constexpr T*        end() const noexcept { return m_data + m_size; }

    [[nodiscard]] constexpr span subspan(size_type off, size_type len) const noexcept
    {
        return span(m_data + off, len);
    }

private:
    T*        m_data = nullptr;
    size_type m_size = 0;
};

} // namespace modbusmech
