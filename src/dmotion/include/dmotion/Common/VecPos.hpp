#pragma once

#include "dcommon/dcommon.hpp"
#include "fast_math.hpp"
#include <ostream>

enum CoordSystemT
{
    CARTESIAN,
    POLAR
};

class VecPos
{
  public:
    ddouble_t m_x;
    ddouble_t m_y;

    VecPos(ddouble_t x, ddouble_t y, CoordSystemT t);
    VecPos(ddouble_t x, ddouble_t y);
    VecPos();
    VecPos(const VecPos& v);

    inline void operator=(const VecPos& p);
    inline void operator+=(const VecPos& p);
    inline void operator-=(const VecPos& p);

    inline VecPos operator+(const VecPos& p) const;
    inline VecPos operator-(const VecPos& p) const;
    inline VecPos operator/(ddouble_t n) const;
    inline VecPos operator*(ddouble_t n) const;

    inline bool operator==(const VecPos& p) const;
    inline bool operator!=(const VecPos& p) const;

    ddouble_t getMagnitude() const;
    ddouble_t getAngle() const;
    /* get the delta vecpos's angle, have direction */
    ddouble_t getAngle(const VecPos& p) const;

    ddouble_t getDistance(const VecPos& p) const;
    inline void rotate(ddouble_t angle);

    std::string toString();
    friend std::ostream& operator<<(std::ostream&, const VecPos&);

    // rapidjson serialize
    template<typename Writer>
    void Serialize(Writer& writer) const
    {
        writer.StartObject();

        writer.String("x");
        writer.Double(m_x);

        writer.String("y");
        writer.Double(m_y);

        writer.EndObject();
    }
};

/**
 * operator overload
 **/

inline void
VecPos::operator=(const VecPos& p)
{
    m_x = p.m_x;
    m_y = p.m_y;
}

inline void
VecPos::operator+=(const VecPos& p)
{
    m_x += p.m_x;
    m_y += p.m_y;
}

inline void
VecPos::operator-=(const VecPos& p)
{
    m_x -= p.m_x;
    m_y -= p.m_y;
}

inline VecPos
VecPos::operator+(const VecPos& p) const
{
    VecPos temp(0, 0);
    temp.m_x = m_x + p.m_x;
    temp.m_y = m_y + p.m_y;
    return temp;
}

inline VecPos
VecPos::operator-(const VecPos& p) const
{
    VecPos temp(0, 0);
    temp.m_x = m_x - p.m_x;
    temp.m_y = m_y - p.m_y;
    return temp;
}

inline VecPos
VecPos::operator/(ddouble_t n) const
{
    VecPos temp(0, 0);
    temp.m_x = m_x / n;
    temp.m_y = m_y / n;
    return temp;
}

inline VecPos VecPos::operator*(ddouble_t n) const
{
    VecPos temp(0, 0);
    temp.m_x = m_x * n;
    temp.m_y = m_y * n;
    return temp;
}

inline bool
VecPos::operator==(const VecPos& p) const
{
    if (m_x == p.m_x && m_y == p.m_y) {
        return true;
    } else {
        return false;
    }
}

inline bool
VecPos::operator!=(const VecPos& p) const
{
    if (m_x != p.m_x || m_y != p.m_y) {
        return true;
    } else {
        return false;
    }
}

inline void
VecPos::rotate(ddouble_t angle)
{
    ddouble_t x = m_x;
    ddouble_t y = m_y;
    ddouble_t sinAngle = std::sin(angle * M_PI / 180.0);
    ddouble_t cosAngle = std::cos(angle * M_PI / 180.0);
    m_x = x * cosAngle - y * sinAngle;
    m_y = x * sinAngle + y * cosAngle;
}
