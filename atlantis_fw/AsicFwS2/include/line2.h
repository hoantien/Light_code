#ifndef __LINE2_H__
#define __LINE2_H__


#ifdef __cplusplus

#include <limits>
#include <cassert>
#include "matrix.h"


namespace ltaf {

/// Restrict the input to be floating type.
template<typename T>
class Line2
{
    static_assert( std::is_floating_point<T>::value, "only floating-point line types supported!" );
public:
    Line2(): _line( T{1}, 0, 0 ) {}

    /// The line is represented by ax + by + c = 0;
    Line2( T a, T b, T c ): _line( a, b, c )
    {
        assert( isValidLine() ); //, "Not a valid line!" );
        normalize();
    }

    Line2( const Vec3<T> & line ) : _line( line )
    {
        assert( isValidLine() );//, "Not a valid line!" );
        normalize();
    }

    /// Project a point to the line.
    /// we are looking for a point pl such that pl = point + t * normal
    /// and it needs to satisfy line equation a * x + b * y + c = 0
    /// solve for t
    /// a * (x + t * a) + b * (y + t * b) + c = 0
    /// t * (a*a + b*b) = -c -by -ax
    /// also, assume that this line is normalized -> 1 == a * a + b * b
    Vec2<T> closest( const Vec2<T> & pt ) const
    {
        Vec2<T> dir( _line.x(), _line.y() );
        T t = _line.z() + dot( dir, pt );
        return pt - t * dir;
    }

    /// Find the x coordinate given y coordinate.
    T xGivenY( T y ) const
    {
        //LT_ASSERT( abs( _line.x() ) > kEps10x, "Can't solve x coordinate for a horiontal line" );
        assert(std::abs( _line.x() ) > kEps10x);
        return -( _line.y() * y + _line.z() ) / _line.x();
    }

    /// Find the y coordinate given x coordinate.
    T yGivenX( T x ) const
    {
        //LT_ASSERT( abs( _line.y() ) > kEps10x, "Can't solve y coordinate for a vertical line" );
        assert(std::abs( _line.y() ) > kEps10x);
        return -( _line.x() * x + _line.z() ) / _line.y();
    }


    /// Distance. Assume (a,b) is normalized.
    T dist( const Vec2<T> & p )       const { return std::abs( dot( _line, Vec3<T>( p.x(), p.y(), T{1} ) ) ); }

    T x() const {return _line.x();}
    T y() const {return _line.y();}
    T z() const {return _line.z();}

    static constexpr T kEps10x = 10 * std::numeric_limits<T>::epsilon();

private:

    bool isValidLine() const { return std::abs( _line.x() ) > kEps10x || std::abs( _line.y() ) > kEps10x; }

    void normalize() { _line *= T{1} / length( Vec2<T>( _line.x(), _line.y() ) );}

    Vec3<T> _line;
};


using Line2f = Line2<float>;
using Line2d = Line2<double>;


bool IntersectBox( const Line2f & line, float start_x, float start_y, int width, int height, 
                    Vec2f & p1, Vec2f & p2 );


} // end namespace ltaf

#endif

#endif // LINE2_H
