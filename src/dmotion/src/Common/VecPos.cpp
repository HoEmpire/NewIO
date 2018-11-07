#include "dmotion/Common/VecPos.hpp"
#include <cmath>

VecPos::VecPos(void) //默认构造函数
{
    m_x = 0;
    m_y = 0;
}

VecPos::VecPos(ddouble_t x, ddouble_t y) //无坐标系类型构造函数：按照直角坐标构造
{
    m_x = x;
    m_y = y;
}

VecPos::VecPos(ddouble_t x, ddouble_t y, CoordSystemT t) //带坐标系类型参数的构造函数(极坐标下y为角度)
{
    ddouble_t r = 0;
    switch (t) {
        case POLAR:
            r = std::abs(x);
            m_x = r * std::cos(y * M_PI / 180.0);
            m_y = r * std::sin(y * M_PI / 180.0);
            break;
        case CARTESIAN:
            m_x = x;
            m_y = y;
            break;
        default:
            m_x = x;
            m_y = y;
    }
}

VecPos::VecPos(const VecPos& v)
{
    m_x = v.m_x;
    m_y = v.m_y;
}

std::ostream&
operator<<(std::ostream& out, const VecPos& pos)
{
    out << "(" << pos.m_x << "," << pos.m_y << ")";
    return out;
}

std::string
VecPos::toString()
{
    return "VecPos(" + std::to_string(m_x) + ", " + std::to_string(m_y) + ")";
}
////////////////////////////////////////////////////////////////
ddouble_t
VecPos::getMagnitude(void) const
{
    ddouble_t m;
    m = std::sqrt(m_x * m_x + m_y * m_y);
    return m;
}

ddouble_t
VecPos::getDistance(const VecPos& p) const
{
    ddouble_t d;
    d = std::sqrt((m_x - p.m_x) * (m_x - p.m_x) + (m_y - p.m_y) * (m_y - p.m_y));
    return d;
}

ddouble_t
VecPos::getAngle() const
{
    ddouble_t angle;
    if (m_x == 0) {
        angle = (m_y > 0) ? 90 : -90;
        return angle;
    }
    angle = std::atan(m_y / m_x) * 180 / M_PI;
    if (m_x <= 0 && m_y >= 0)
        angle += 180;
    else if (m_x <= 0 && m_y < 0)
        angle -= 180;
    return angle;
}

ddouble_t
VecPos::getAngle(const VecPos& p) const
{
    ddouble_t angle;
    VecPos temp;
    temp.m_x = p.m_x - m_x;
    temp.m_y = p.m_y - m_y;
    angle = temp.getAngle();
    return angle;
}
