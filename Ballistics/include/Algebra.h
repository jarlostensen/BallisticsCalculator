#pragma once

namespace Algebra
{
    class Vector2D
    {
        float X = 0.0f;
        float Y = 0.0f;
    public:
        Vector2D() = default;
        constexpr Vector2D(float InX, float InY) : X(InX), Y(InY) {}
        Vector2D(const Vector2D& InPoint) = default;

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
}

// overloaded 
inline float fabsf(const Algebra::Vector2D& Vector)
{
    return sqrtf(Vector.LengthSq());
}
