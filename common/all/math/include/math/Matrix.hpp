#pragma once

#include <array>
#include <cstdint>

#include "error/error_handler.hpp"

namespace math
{

template <typename T, std::size_t Rows, std::size_t Cols>
struct Matrix
{
   std::array<T, Rows * Cols> data{};

   static constexpr std::size_t max_rows    = 10u;
   static constexpr std::size_t max_columns = 10u;

   static_assert(0 < Rows);
   static_assert(0 < Cols);
   static_assert(Rows <= max_rows);
   static_assert(Cols <= max_columns);

   constexpr const Matrix& get() const noexcept
   {
      return *this;
   }

   constexpr T& at(const std::size_t row, const std::size_t col)
   {
      error::verify((row < Rows) && (col < Cols));
      const std::size_t index = (row * Cols) + col;
      return data[index];
   }

   constexpr T at(const std::size_t row, const std::size_t col) const
   {
      error::verify((row < Rows) && (col < Cols));
      const std::size_t index = (row * Cols) + col;
      return data[index];
   }

   // matrix addition
   constexpr Matrix operator+(const Matrix& rhs) const noexcept
   {
      Matrix result{};
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         result.data[i] = data[i] + rhs.data[i];
      }
      return result;
   }

   // matrix addition
   constexpr const Matrix& operator+=(const Matrix& rhs) noexcept
   {
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         this->data[i] += rhs.data[i];
      }
      return *this;
   }

   // scalar addition
   constexpr const Matrix& operator+=(const T& scalar) noexcept
   {
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         this->data[i] += scalar;
      }
      return *this;
   }

   // matrix subtraction
   constexpr Matrix operator-(const Matrix& rhs) const noexcept
   {
      Matrix result{};
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         result.data[i] = data[i] - rhs.data[i];
      }
      return result;
   }

   // matrix subtraction
   constexpr const Matrix& operator-=(const Matrix& rhs) noexcept
   {
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         this->data[i] -= rhs.data[i];
      }
      return *this;
   }

   // scalar subtraction
   constexpr const Matrix& operator-=(const T& scalar) noexcept
   {
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         this->data[i] -= scalar;
      }
      return *this;
   }

   // matrix multiplication
   template <std::size_t NewCols>
   constexpr Matrix<T, Rows, NewCols> operator*(const Matrix<T, Cols, NewCols>& rhs) const noexcept
   {
      Matrix<T, Rows, NewCols> result{};
      for (std::size_t i = 0; i < Rows; ++i)
      {
         for (std::size_t j = 0; j < NewCols; ++j)
         {
            T sum{};
            for (std::size_t k = 0; k < Cols; ++k)
            {
               sum += at(i, k) * rhs.at(k, j);
            }
            result.at(i, j) = sum;
         }
      }
      return result;
   }

   // scalar multiplication
   constexpr Matrix operator*(const T& scalar) const noexcept
   {
      Matrix result{};
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         result.data[i] = data[i] * scalar;
      }
      return result;
   }

   // scalar multiplication
   constexpr const Matrix& operator*=(const T& scalar) noexcept
   {
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         this->data[i] *= scalar;
      }
      return *this;
   }

   // scalar division
   constexpr const Matrix& operator/=(const T& scalar) noexcept
   {
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         this->data[i] /= scalar;
      }
      return *this;
   }

   constexpr Matrix<T, Cols, Rows> transpose() const noexcept
   {
      Matrix<T, Cols, Rows> result{};
      for (std::size_t i = 0; i < Rows; ++i)
      {
         for (std::size_t j = 0; j < Cols; ++j)
         {
            result.at(j, i) = at(i, j);
         }
      }
      return result;
   }

   constexpr void reset() noexcept
   {
      for (std::size_t i = 0; i < Rows * Cols; i++)
      {
         data[i] = static_cast<T>(0);
      }
   }

   static constexpr Matrix Identity()
      requires(Rows == Cols)
   {
      Matrix result{};
      for (std::size_t i = 0; i < Rows; ++i)
      {
         result.at(i, i) = static_cast<T>(1u);
      }

      return result;
   }

   constexpr void set_identity()
   {
      error::verify(Rows == Cols);

      reset();
      for (std::size_t i = 0; i < Rows; i++)
      {
         at(i, i) = static_cast<T>(1u);
      }
   }
};

}   // namespace math
