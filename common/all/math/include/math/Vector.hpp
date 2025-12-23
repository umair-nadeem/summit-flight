#pragma once

#include <array>
#include <cmath>

#include "error/error_handler.hpp"

namespace math
{

template <typename T, std::size_t N>
struct Vector
{
   Vector()                               = default;
   Vector(const Vector& other)            = default;
   Vector& operator=(const Vector& other) = default;

   explicit Vector(const std::array<T, N>& data)
       : m_data{data}
   {
   }

   explicit Vector(const T scalar)
       : m_data{scalar}
   {
      for (std::size_t i = 0; i < N; i++)
      {
         m_data[i] = scalar;
      }
   }

   Vector& operator=(const std::array<T, N>& other) noexcept
   {
      for (std::size_t i = 0; i < N; i++)
      {
         m_data[i] = other[i];
      }
      return *this;
   }

   constexpr const T& operator[](const std::size_t i) const
   {
      error::verify(i < N);
      return m_data[i];
   }

   T& operator[](const std::size_t i)
   {
      error::verify(i < N);
      return m_data[i];
   }

   constexpr bool operator==(const Vector& other) const noexcept
   {
      for (std::size_t i = 0; i < N; i++)
      {
         if constexpr (std::is_floating_point_v<T>)
         {
            if (std::abs(m_data[i] - other.m_data[i]) > static_cast<T>(0.0001))
            {
               return false;
            }
         }
         else
         {
            if (m_data[i] != other.m_data[i])
            {
               return false;
            }
         }
      }
      return true;
   }

   constexpr bool operator!=(const Vector& other) const noexcept
   {
      return !(*this == other);
   }

   constexpr Vector operator+(const Vector& other) const noexcept
   {
      Vector result{};

      for (std::size_t i = 0; i < N; i++)
      {
         result[i] = m_data[i] + other[i];
      }
      return result;
   }

   constexpr void operator+=(const Vector& other) noexcept
   {
      for (std::size_t i = 0; i < N; i++)
      {
         m_data[i] += other[i];
      }
   }

   constexpr Vector operator+(const T scalar) const noexcept
   {
      Vector result{};

      for (std::size_t i = 0; i < N; i++)
      {
         result[i] = m_data[i] + scalar;
      }
      return result;
   }

   constexpr Vector operator-(const Vector& other) const noexcept
   {
      Vector result{};

      for (std::size_t i = 0; i < N; i++)
      {
         result[i] = m_data[i] - other[i];
      }
      return result;
   }

   constexpr void operator-=(const Vector& other) noexcept
   {
      for (std::size_t i = 0; i < N; i++)
      {
         m_data[i] -= other[i];
      }
   }

   constexpr Vector operator-(const T scalar) const noexcept
   {
      Vector result{};

      for (std::size_t i = 0; i < N; i++)
      {
         result[i] = m_data[i] - scalar;
      }
      return result;
   }

   constexpr T dot(const Vector& other) const noexcept
   {
      T result{};
      for (std::size_t i = 0; i < N; i++)
      {
         result += m_data[i] * other[i];
      }

      return result;
   }

   constexpr Vector emul(const Vector& other) const noexcept
   {
      Vector result{};

      for (std::size_t i = 0; i < N; i++)
      {
         result[i] = m_data[i] * other[i];
      }
      return result;
   }

   constexpr Vector operator*(const T scalar) const noexcept
   {
      Vector result{};

      for (std::size_t i = 0; i < N; i++)
      {
         result[i] = m_data[i] * scalar;
      }
      return result;
   }

   void operator*=(const T scalar)
   {
      for (std::size_t i = 0; i < N; i++)
      {
         m_data[i] *= scalar;
      }
   }

   constexpr Vector ediv(const Vector& other) const noexcept
   {
      Vector result{};

      for (std::size_t i = 0; i < N; i++)
      {
         result[i] = m_data[i] / other[i];
      }
      return result;
   }

   constexpr Vector operator/(const T scalar) const noexcept
   {
      Vector result{};

      for (std::size_t i = 0; i < N; i++)
      {
         result[i] = m_data[i] / scalar;
      }
      return result;
   }

   void operator/=(const T scalar)
   {
      auto& a = *(this);
      a *= (static_cast<T>(1) / scalar);
   }

   constexpr Vector normalized() const
   {
      const T n = norm();
      if (n > static_cast<T>(1e-6f))
      {
         return {(*this) / n};
      }
      return Vector{};
   }

   constexpr T norm() const
   {
      const auto& a = *(this);
      return static_cast<T>(std::sqrt(a.dot(a)));
   }

   constexpr T norm_squared() const noexcept
   {
      const auto& a = *(this);
      return a.dot(a);
   }

   void zero() noexcept
   {
      m_data.fill(static_cast<T>(0));
   }

   void normalize() noexcept
   {
      const T n = norm();
      if (n > static_cast<T>(1e-6f))
      {
         auto& a = *(this);
         a /= n;
      }
   }

protected:
   std::array<T, N> m_data{};
};

}   // namespace math
