#pragma once
#include <cmath>
#include <optional>

namespace Algebra
{
    class Vector2D
    {
        float X = 0.0f;
        float Y = 0.0f;
    public:
        Vector2D() = default;
        constexpr Vector2D(float InX, float InY) : X(InX), Y(InY) {}
        
        float GetX() const { return X; }
        float GetY() const { return Y; }
        void SetX(float InX) { X = InX; }
        void SetY(float InY) { Y = InY; }
        void Set(float InX, float InY) { X = InX; Y = InY; }
        Vector2D& Normalize()
        {
            float Length = this->LengthSq();
            if (Length > 0.0f)
            {
                float InvLength = 1.0f / sqrtf(Length);
                X *= InvLength;
                Y *= InvLength;
            }
            return *this;
        }
        
        Vector2D& operator+=(const Vector2D& InPoint)
        {
            X += InPoint.X;
            Y += InPoint.Y;
            return *this;
        }

        Vector2D& operator-=(const Vector2D& InPoint)
        {
            X -= InPoint.X;
            Y -= InPoint.Y;
            return *this;
        }

        Vector2D& operator*=(float InScalar)
        {
            X *= InScalar;
            Y *= InScalar;
            return *this;
        }

        Vector2D operator-() const
        {
            return {-X, -Y};
        }

        Vector2D operator*(float InScalar) const
        {
            return {X * InScalar, Y * InScalar};
        }

        bool operator==(const Vector2D& Rhs) const
        {
            return X == Rhs.X && Y == Rhs.Y;
        }

        friend Vector2D operator*(float InScalar, const Vector2D& InPoint)
        {
            return {InPoint.X * InScalar, InPoint.Y * InScalar};
        }

        friend Vector2D operator+(const Vector2D& Lhs, const Vector2D& Rhs)
        {
            return {Lhs.X + Rhs.X, Lhs.Y + Rhs.Y};
        }

        friend Vector2D operator-(const Vector2D& Lhs, const Vector2D& Rhs)
        {
            return {Lhs.X - Rhs.X, Lhs.Y - Rhs.Y};
        }

        float Dot(const Vector2D& Rhs) const
        {
            return X * Rhs.X + Y * Rhs.Y;
        }

        Vector2D Cross(const Vector2D& Rhs) const
        {
            return {X * Rhs.Y - Y * Rhs.X, Y * Rhs.X - X * Rhs.Y};
        }
        
        float LengthSq() const
        {
            return this->Dot(*this);
        }
    };

    class Matrix2D
    {
        float Elements[2][2] =
            {
            {1.0f, 0.0f},
            {0.0f, 1.0f}
            };

    public:
        Matrix2D() = default;
        ~Matrix2D() = default;
        constexpr Matrix2D(float m00, float m01, float m10, float m11)
        {
            Elements[0][0] = m00; Elements[0][1] = m01;
            Elements[1][0] = m10; Elements[1][1] = m11;
        }
        Matrix2D(const Vector2D& v0, const Vector2D& v1)
        {
            Elements[0][0] = v0.GetX(); Elements[0][1] = v0.GetY();
            Elements[1][0] = v1.GetX(); Elements[1][1] = v1.GetY();
        }
        Matrix2D(const Matrix2D& Rhs)
        {
            Elements[0][0] = Rhs.Elements[0][0];
            Elements[0][1] = Rhs.Elements[0][1];
            Elements[1][0] = Rhs.Elements[1][0];
            Elements[1][1] = Rhs.Elements[1][1];
        }
        
        float& operator()(int row, int col) 
        { 
            return Elements[row][col]; 
        }
        float operator()(int row, int col) const 
        { 
            return Elements[row][col]; 
        }

        float Determinant() const
        {
            return Elements[0][0] * Elements[1][1] - Elements[0][1] * Elements[1][0];
        }

        std::optional<Matrix2D> Inverse() const
        {
            float det = Determinant();
            if (std::abs(det) < 1e-6f)
            {
                return std::nullopt;
            }

            float invDet = 1.0f / det;
            return Matrix2D(
                Elements[1][1] * invDet, -Elements[0][1] * invDet,
                -Elements[1][0] * invDet, Elements[0][0] * invDet
            );
        }
        
        Matrix2D operator*(const Matrix2D& rhs) const
        {
            return {
                Elements[0][0] * rhs.Elements[0][0] + Elements[0][1] * rhs.Elements[1][0],
                Elements[0][0] * rhs.Elements[0][1] + Elements[0][1] * rhs.Elements[1][1],
                Elements[1][0] * rhs.Elements[0][0] + Elements[1][1] * rhs.Elements[1][0],
                Elements[1][0] * rhs.Elements[0][1] + Elements[1][1] * rhs.Elements[1][1]
            };
        }

        Vector2D operator*(const Vector2D& vec) const
        {
            return {
                Elements[0][0] * vec.GetX() + Elements[0][1] * vec.GetY(),
                Elements[1][0] * vec.GetX() + Elements[1][1] * vec.GetY()
            };
        }

        Matrix2D operator*(float scalar) const
        {
            return {
                Elements[0][0] * scalar, Elements[0][1] * scalar,
                Elements[1][0] * scalar, Elements[1][1] * scalar
            };
        }

        Matrix2D operator+(const Matrix2D& rhs) const
        {
            return {
                Elements[0][0] + rhs.Elements[0][0], Elements[0][1] + rhs.Elements[0][1],
                Elements[1][0] + rhs.Elements[1][0], Elements[1][1] + rhs.Elements[1][1]
            };
        }

        Matrix2D operator-(const Matrix2D& rhs) const
        {
            return {
                Elements[0][0] - rhs.Elements[0][0], Elements[0][1] - rhs.Elements[0][1],
                Elements[1][0] - rhs.Elements[1][0], Elements[1][1] - rhs.Elements[1][1]
            };
        }

        static Matrix2D Identity()
        {
            return {};
        }

        static Matrix2D Rotation(float Angle)
        {
            float c = std::cos(Angle);
            float s = std::sin(Angle);
            return {c, -s, s, c};
        }

        static Matrix2D Scale(float Sx, float Sy)
        {
            return {Sx, 0.0f, 0.0f, Sy};
        }
        
        friend Matrix2D operator*(float scalar, const Matrix2D& mat)
        {
            return mat * scalar;
        }
    };

}

// overloaded 
inline float fabsf(const Algebra::Vector2D& Vector)
{
    return sqrtf(Vector.LengthSq());
}
