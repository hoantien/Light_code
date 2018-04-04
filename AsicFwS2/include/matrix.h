// we want to have support for the following:
// matrix3x3f, matrix4x4f, vec2f, vec3f, vec4f
// plus, minus, multiplication, inverse, transpose

#ifndef __MATRIX_H__
#define __MATRIX_H__

#ifdef __cplusplus

#include <cstdlib>
#include <algorithm> // has compile problem...
#include <cmath>

namespace ltaf { // lt auto focus

// -----------------------------------------------------------
// 			Vec2 class
// -----------------------------------------------------------
template<typename T>
class Vec2
{
public:
    Vec2() {}
    Vec2( T t ) : _x( t ), _y( t ) {}
    Vec2( T x, T y ) : _x( x ), _y( y ) {}
    Vec2( const Vec2 & t ) : _x( t._x ), _y( t._y ) {}

    template<typename U>
    explicit Vec2( const Vec2<U> & t )
        : _x( static_cast<T>( t.x() ) )
        , _y( static_cast<T>( t.y() ) ) {}

    Vec2 & operator=( const Vec2 & t )
    { _x = t._x; _y = t._y; return *this; }

    Vec2 & operator/=( const Vec2 & t )
    { _x /= t._x; _y /= t._y; return *this; }

    Vec2 & operator%=( const Vec2 & t )
    { _x %= t._x; _y %= t._y; return *this; }

    Vec2 & operator*=( const Vec2 & t )
    { _x *= t._x; _y *= t._y; return *this; }

    Vec2 & operator+=( const Vec2 & t )
    { _x += t._x; _y += t._y; return *this; }

    Vec2 & operator-=( const Vec2 & t )
    { _x -= t._x; _y -= t._y; return *this; }

    Vec2 & operator&=( const Vec2 & t )
    { _x &= t._x; _y &= t._y; return *this; }

    Vec2 & operator|=( const Vec2 & t )
    { _x |= t._x; _y |= t._y; return *this; }

    Vec2 & operator>>=( int t )
    { _x >>= t; _y >>= t; return *this; }

    Vec2 & operator<<=( int t )
    { _x <<= t; _y <<= t; return *this; }

    Vec2 operator-() const { return { -_x, -_y}; }

    template<typename U> Vec2<typename std::common_type<T, U>::type>
    operator+( const Vec2<U> & t ) const { return { _x + t.x(), _y + t.y() }; }

    template<typename U> Vec2<typename std::common_type<T, U>::type>
    operator-( const Vec2<U> & t ) const { return { _x - t.x(), _y - t.y() }; }

    template<typename U> Vec2<typename std::common_type<T, U>::type>
    operator*( const Vec2<U> & t ) const { return { _x * t.x(), _y * t.y() }; }

    template<typename U> Vec2<typename std::common_type<T, U>::type>
    operator/( const Vec2<U> & t ) const { return { _x / t.x(), _y / t.y() }; }

    template < typename U,
               typename = typename std::enable_if < std::is_integral<T>::value && std::is_integral<U>::value >::type >
    Vec2<typename std::common_type<T, U>::type>
    operator%( const Vec2<U> & t ) const { return { _x % t.x(), _y % t.y() }; }

    template < typename U,
               typename = typename std::enable_if < std::is_arithmetic<U>::value >::type >
    Vec2<typename std::common_type<T, U>::type>
    operator+( U t ) const { return { _x + t, _y + t }; }

    template < typename U,
               typename = typename std::enable_if < std::is_arithmetic<U>::value >::type >
    Vec2<typename std::common_type<T, U>::type>
    operator-( U t ) const { return { _x - t, _y - t }; }

    template < typename U,
               typename = typename std::enable_if < std::is_arithmetic<U>::value >::type >
    Vec2<typename std::common_type<T, U>::type>
    operator*( U t ) const { return { _x * t, _y * t }; }

    template < typename U,
               typename = typename std::enable_if < std::is_arithmetic<U>::value >::type >
    Vec2<typename std::common_type<T, U>::type>
    operator/( U t ) const { return { _x / t, _y / t }; }

    template < typename U,
               typename = typename std::enable_if < std::is_integral<T>::value && std::is_integral<U>::value >::type >
    Vec2<typename std::common_type<T, U>::type>
    operator%( U t ) const { return { _x % t, _y % t }; }

    T min() const { return std::min( _x, _y ); }
    T max() const { return std::max( _x, _y ); }
    T sum() const { return _x + _y; }
    T dot( const Vec2<T> & t ) const { return _x * t.x() + _y * t.y(); }

    Vec2 abs() const { return { std::abs( _x ), std::abs( _y ) }; }
    
    const T & x() const { return _x; }
    const T & y() const { return _y; }

    void setX( T x ) { _x = x; }
    void setY( T y ) { _y = y; }

private:
	T _x, _y;
};


// basic add/sub/mul/div operators
template < typename T, typename U,
           typename = typename std::enable_if < std::is_arithmetic<T>::value >::type >
Vec2<typename std::common_type<T, U>::type>
operator+( T a, const Vec2<U> & b ) { return b + a; }

template < typename T, typename U,
           typename = typename std::enable_if < std::is_arithmetic<T>::value >::type >
Vec2<typename std::common_type<T, U>::type>
operator*( T a, const Vec2<U> & b ) { return b * a; }

template < typename T, typename U,
           typename = typename std::enable_if < std::is_arithmetic<T>::value >::type >
Vec2<typename std::common_type<T, U>::type>
operator-( T a, const Vec2<U> & b ) { return Vec2<T>( a ) - b; }

template < typename T, typename U,
           typename = typename std::enable_if < std::is_arithmetic<T>::value >::type >
Vec2<typename std::common_type<T, U>::type>
operator/( T a, const Vec2<U> & b ) { return Vec2<T>( a ) / b; }

// basic math functions
template<typename T>
T min( const Vec2<T> & t ) { return t.min(); }

template<typename T>
T max( const Vec2<T> & t ) { return t.max(); }

template<typename T>
T sum( const Vec2<T> & t ) { return t.sum(); }

template<typename T>
T dot( const Vec2<T> & a, const Vec2<T> & b ) { return a.dot( b ); }

template<typename T>
Vec2<T> min( const Vec2<T> & a, const Vec2<T> & b )
{ return { std::min( a.x(), b.x() ), std::min( a.y(), b.y() ) }; }

template<typename T>
Vec2<T> abs( const Vec2<T> & t ) { return t.abs(); }

// template<typename T>
// Vec2<T> sign( const Vec2<T> & t ) { return t.sign(); }

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type
length( const Vec2<T> & a )
{ return std::sqrt( dot( a, a ) ); }

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, Vec2<T> >::type
sqrt( const Vec2<T> & a )
{ return { std::sqrt( a.x() ), std::sqrt( a.y() ) }; }

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type
squaredSum( const Vec2<T> & a )
{ return dot( a, a ); }

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, Vec2<T> >::type
normalize( const Vec2<T> & a )
{
    return a / length( a );
}

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec2<T> >::type
// pow( const Vec2<T> & a, const Vec2<T> & b )
// { return { std::pow( a.x(), b.x() ), std::pow( a.y(), b.y() ) }; }

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec2<T> >::type
// pow( const Vec2<T> & a, T  b )
// { return { std::pow( a.x(), b ), std::pow( a.y(), b ) }; }

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec2<T> >::type
// pow( T a, const Vec2<T> & b )
// { return { std::pow( a, b.x() ), std::pow( a, b.y() ) }; }

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec2<T> >::type
// floor( const Vec2<T> & a )
// { return { std::floor( a.x() ), std::floor( a.y() ) }; }

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec2<T> >::type
// ceil( const Vec2<T> & a )
// { return { std::ceil( a.x() ), std::ceil( a.y() ) }; }





// -----------------------------------------------------------
// 			Vec3 class
// -----------------------------------------------------------
template<typename T>
class Vec3
{
public:
    Vec3() {}
    Vec3( T t ) : _x( t ), _y( t ), _z( t ) {}
    Vec3( T x, T y, T z ) : _x( x ), _y( y ), _z( z ) {}
    Vec3( const Vec3 & t ) : _x( t._x ), _y( t._y ), _z( t._z ) {}

    template<typename U>
    explicit Vec3( const Vec3<U> & t )
        : _x( static_cast<T>( t.x() ) )
        , _y( static_cast<T>( t.y() ) )
        , _z( static_cast<T>( t.z() ) ) {}

    Vec3 & operator=( const Vec3 & t )
    { _x = t._x; _y = t._y; _z = t._z; return *this; }

    Vec3 & operator/=( const Vec3 & t )
    { _x /= t._x; _y /= t._y; _z /= t._z; return *this; }

    Vec3 & operator%=( const Vec3 & t )
    { _x %= t._x; _y %= t._y; _z %= t._z; return *this; }

    Vec3 & operator*=( const Vec3 & t )
    { _x *= t._x; _y *= t._y; _z *= t._z; return *this; }

    Vec3 & operator+=( const Vec3 & t )
    { _x += t._x; _y += t._y; _z += t._z; return *this; }

    Vec3 & operator-=( const Vec3 & t )
    { _x -= t._x; _y -= t._y; _z -= t._z; return *this; }

    Vec3 & operator&=( const Vec3 & t )
    { _x &= t._x; _y &= t._y; _z &= t._z; return *this; }

    Vec3 & operator|=( const Vec3 & t )
    { _x |= t._x; _y |= t._y; _z |= t._z; return *this; }

    Vec3 & operator>>=( int t )
    { _x >>= t; _y >>= t; _z >>= t; return *this; }

    Vec3 & operator<<=( int t )
    { _x <<= t; _y <<= t; _z <<= t; return *this; }

    Vec3 operator-() const { return { -_x, -_y, -_z }; }

    template<typename U > 
    Vec3<typename std::common_type<T, U>::type>
    operator+( const Vec3<U> & t ) const { return { _x + t.x(), _y + t.y(), _z + t.z() }; }

    template<typename U> Vec3<typename std::common_type<T, U>::type>
    operator-( const Vec3<U> & t ) const { return { _x - t.x(), _y - t.y(), _z - t.z() }; }

    template<typename U> Vec3<typename std::common_type<T, U>::type>
    operator*( const Vec3<U> & t ) const { return { _x * t.x(), _y * t.y(), _z * t.z() }; }

    template<typename U> Vec3<typename std::common_type<T, U>::type>
    operator/( const Vec3<U> & t ) const { return { _x / t.x(), _y / t.y(), _z / t.z() }; }

    template < typename U,
               typename = typename std::enable_if < std::is_integral<T>::value && std::is_integral<U>::value >::type >
    Vec3<typename std::common_type<T, U>::type>
    operator%( const Vec3<U> & t ) const { return { _x % t.x(), _y % t.y(), _z % t.z() }; }

    template < typename U,
               typename = typename std::enable_if < std::is_arithmetic<U>::value >::type >
    Vec3<typename std::common_type<T, U>::type>
    operator+( U t ) const { return { _x + t, _y + t, _z + t }; }

    template < typename U,
               typename = typename std::enable_if < std::is_arithmetic<U>::value >::type >
    Vec3<typename std::common_type<T, U>::type>
    operator-( U t ) const { return { _x - t, _y - t, _z - t }; }

    template < typename U,
               typename = typename std::enable_if < std::is_arithmetic<U>::value >::type >
    Vec3<typename std::common_type<T, U>::type>
    operator*( U t ) const { return { _x * t, _y * t, _z * t }; }

    template < typename U,
               typename = typename std::enable_if < std::is_arithmetic<U>::value >::type >
    Vec3<typename std::common_type<T, U>::type>
    operator/( U t ) const { return { _x / t, _y / t, _z / t }; }

    template < typename U,
               typename = typename std::enable_if < std::is_integral<T>::value && std::is_integral<U>::value >::type >
    Vec3<typename std::common_type<T, U>::type>
    operator%( U t ) const { return { _x % t, _y % t, _z % t }; }
    Vec3<bool> operator==( const Vec3 & t ) const { return { _x == t._x, _y == t._y, _z == t._z }; }
    Vec3<bool> operator!=( const Vec3 & t ) const { return { _x != t._x, _y != t._y, _z != t._z }; }
    Vec3<bool> operator>=( const Vec3 & t ) const { return { _x >= t._x, _y >= t._y, _z >= t._z }; }
    Vec3<bool> operator>( const Vec3 & t )  const { return { _x >  t._x, _y >  t._y, _z >  t._z }; }
    Vec3<bool> operator<=( const Vec3 & t ) const { return { _x <= t._x, _y <= t._y, _z <= t._z }; }
    Vec3<bool> operator<( const Vec3 & t )  const { return { _x <  t._x, _y <  t._y, _z <  t._z }; }

    T min() const { return std::min( std::min( _x, _y ), _z ); }
    T max() const { return std::max( std::max( _x, _y ), _z ); }
    T sum() const { return _x + _y + _z; }
    T dot( const Vec3<T> & t ) const { return _x * t.x() + _y * t.y() + _z * t.z(); }

    // cross product in right-hand side coordinate (i.e. if this=(1,0,0), t=(0,1,0), out=(0,0,1)).
    Vec3<T> cross( const Vec3<T> & t ) const
    { return Vec3<T> {_y * t.z() - _z * t.y(), _z * t.x() - _x * t.z(), _x * t.y() - _y * t.x()};}

    Vec3 abs() const { return { std::abs( _x ), std::abs( _y ), std::abs( _z ) }; }

    const T & x() const { return _x; }
    const T & y() const { return _y; }
    const T & z() const { return _z; }

    void setX( T x ) { _x = x; }
    void setY( T y ) { _y = y; }
    void setZ( T z ) { _z = z; }

    Vec3 & normalize()
    {
        T len = length(*this);
        _x /= len;
        _y /= len;
        _z /= len;
        return *this;
    }

private:
    T _x, _y, _z;
};

// basic math functions
template<typename T>
T min( const Vec3<T> & t ) { return t.min(); }

template<typename T>
T max( const Vec3<T> & t ) { return t.max(); }

template<typename T>
T sum( const Vec3<T> & t ) { return t.sum(); }

template<typename T>
T dot( const Vec3<T> & a, const Vec3<T> & b ) { return a.dot( b ); }

template<typename T>
Vec3<T> cross( const Vec3<T> & a, const Vec3<T> & b ) { return a.cross( b ); }

template<typename T>
Vec3<T> min( const Vec3<T> & a, const Vec3<T> & b )
{ return { std::min( a.x(), b.x() ), std::min( a.y(), b.y() ), std::min( a.z(), b.z() ) }; }

// template<typename T, typename U>
// typename std::enable_if<std::is_arithmetic<U>::value, Vec3<T> >::type
// min( U a, const Vec3<T> & b )
// { return min( Vec3<T>( a ), b ); }

// template<typename T, typename U>
// typename std::enable_if<std::is_arithmetic<U>::value, Vec3<T> >::type
// min( const Vec3<T> & a, U b )
// { return min( a, Vec3<T>( b ) ); }

// template<typename T>
// Vec3<T> max( const Vec3<T> & a, const Vec3<T> & b )
// { return { max( a.x(), b.x() ), max( a.y(), b.y() ), max( a.z(), b.z() ) }; }

// template<typename T, typename U>
// typename std::enable_if<std::is_arithmetic<U>::value, Vec3<T> >::type
// max( U a, const Vec3<T> & b )
// { return max( Vec3<T>( a ), b ); }

// template<typename T, typename U>
// typename std::enable_if<std::is_arithmetic<U>::value, Vec3<T> >::type
// max( const Vec3<T> & a, U b )
// { return max( a, Vec3<T>( b ) ); }

// template<typename T>
// Vec3<T> clamp( const Vec3<T> & t, const Vec3<T> & min_t, const Vec3<T> & max_t )
// { return min( max( t, min_t ), max_t ); }

// template<typename T, typename U>
// typename std::enable_if<std::is_arithmetic<U>::value, Vec3<T> >::type
// clamp( const Vec3<T> & t, U min_t, const Vec3<T> & max_t )
// { return clamp( t, Vec3<T>( min_t ), max_t ); }

// template<typename T, typename U>
// typename std::enable_if<std::is_arithmetic<U>::value, Vec3<T> >::type
// clamp( const Vec3<T> & t, const Vec3<T> & min_t, U max_t )
// { return clamp( t, min_t, Vec3<T>( max_t ) ); }

// template<typename T, typename U, typename V>
// typename std::enable_if < std::is_arithmetic<U>::value && std::is_arithmetic<V>::value, Vec3<T> >::type
// clamp( const Vec3<T> & t, U min_t, V max_t )
// { return clamp( t, Vec3<T>( min_t ), Vec3<T>( max_t ) ); }

template<typename T>
Vec3<T> abs( const Vec3<T> & t ) { return t.abs(); }

// template<typename T>
// Vec3<T> sign( const Vec3<T> & t ) { return t.sign(); }

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type
length( const Vec3<T> & a )
{ return std::sqrt( dot( a, a ) ); }

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, Vec3<T> >::type
sqrt( const Vec3<T> & a )
{ return { std::sqrt( a.x() ), std::sqrt( a.y() ), std::sqrt( a.z() ) }; }

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, T>::type
squaredSum( const Vec3<T> & a )
{ return dot( a, a ); }

template<typename T>
typename std::enable_if<std::is_floating_point<T>::value, Vec3<T> >::type
normalize( const Vec3<T> & a )
{
    return a / length( a );
}

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec3<T> >::type
// pow( const Vec3<T> & a, const Vec3<T> & b )
// { return { std::pow( a.x(), b.x() ), std::pow( a.y(), b.y() ), std::pow( a.z(), b.z() ) }; }

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec3<T> >::type
// pow( const Vec3<T> & a, T  b )
// { return { std::pow( a.x(), b ), std::pow( a.y(), b ), std::pow( a.z(), b ) }; }

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec3<T> >::type
// pow( T a, const Vec3<T> & b )
// { return { std::pow( a, b.x() ), std::pow( a, b.y() ), std::pow( a, b.z() ) }; }

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec3<T> >::type
// floor( const Vec3<T> & a )
// { return { std::floor( a.x() ), std::floor( a.y() ), std::floor( a.z() ) }; }

// template<typename T>
// typename std::enable_if<std::is_floating_point<T>::value, Vec3<T> >::type
// ceil( const Vec3<T> & a )
// { return { std::ceil( a.x() ), std::ceil( a.y() ), std::ceil( a.z() ) }; }


// -----------------------------------------------------------
// 			Vec4 class
// -----------------------------------------------------------
template<typename T>
class Vec4
{
public:
    Vec4() {}
    Vec4( T t ) : _x( t ), _y( t ), _z( t ), _a( t ) {}
    Vec4( T x, T y, T z, T a ) : _x( x ), _y( y ), _z( z ), _a ( a ){}
    Vec4( const Vec4 & t ) : _x( t._x ), _y( t._y ), _z( t._z ), _a( t._a ) {}

    template<typename U>
    explicit Vec4( const Vec4<U> & t )
        : _x( static_cast<T>( t.x() ) )
        , _y( static_cast<T>( t.y() ) )
        , _z( static_cast<T>( t.z() ) )
        , _a( static_cast<T>( t.a() ) ) {}

    Vec4 & operator=( const Vec4 & t )
    { _x = t._x; _y = t._y; _z = t._z; _a = t._a; return *this; }

    Vec4 & operator/=( const Vec4 & t )
    { _x /= t._x; _y /= t._y; _z /= t._z; _a /= t._a; return *this; }

    Vec4 & operator%=( const Vec4 & t )
    { _x %= t._x; _y %= t._y; _z %= t._z; _a %= t._a; return *this; }

    Vec4 & operator*=( const Vec4 & t )
    { _x *= t._x; _y *= t._y; _z *= t._z; _a *= t._a; return *this; }

    Vec4 & operator+=( const Vec4 & t )
    { _x += t._x; _y += t._y; _z += t._z; _a += t._a; return *this; }

    Vec4 & operator-=( const Vec4 & t )
    { _x -= t._x; _y -= t._y; _z -= t._z; _a -= t._a; return *this; }

    Vec4 operator-() const { return { -_x, -_y, _z, _a}; }

    Vec4<bool> operator==( const Vec4 & t ) const { return { _x == t._x, _y == t._y, _z == t._z, _a == t._a }; }
    Vec4<bool> operator!=( const Vec4 & t ) const { return { _x != t._x, _y != t._y, _z != t._z, _a != t._a }; }
    Vec4<bool> operator>=( const Vec4 & t ) const { return { _x >= t._x, _y >= t._y, _z >= t._z, _a >= t._a }; }
    Vec4<bool> operator>( const  Vec4 & t )  const { return { _x >  t._x, _y >  t._y, _z > t._z, _a > t._a }; }
    Vec4<bool> operator<=( const Vec4 & t ) const { return { _x <= t._x, _y <= t._y, _z <= t._z, _a <= t._a }; }
    Vec4<bool> operator<( const  Vec4 & t )  const { return { _x <  t._x, _y <  t._y, _z <  t._z, _a < t._a }; }

    T min() const { return std::min( std::min( std::min( _x, _y ), _z ), _a); }
    T max() const { return std::max( std::max( std::max( _x, _y ), _z ), _a); }
    T sum() const { return _x + _y + _z + _a; }
    T dot( const Vec4<T> & t ) const { return _x * t.x() + _y * t.y() + _z * t.z() + _a * t.a(); }

    Vec4 abs() const { return { std::abs( _x ), std::abs( _y ), std::abs( _z ), std::abs( _a ) }; }

    const T & x() const { return _x; }
    const T & y() const { return _y; }
    const T & z() const { return _z; }
    const T & a() const { return _a; }

    void setX( T x ) { _x = x; }
    void setY( T y ) { _y = y; }
    void setZ( T z ) { _z = z; }
	void setA( T a ) { _a = a; }
private:
    T _x, _y, _z, _a;
};


using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;
using Vec3u16 = Vec3<std::uint16_t>;
using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;
using Vec4f = Vec4<float>;
using Vec4d = Vec4<double>;


// -----------------------------------------------------------
// 			Matrix class
// -----------------------------------------------------------
namespace Internal
{

template<typename T, int R, int C>
class MatrixStorage
{
public:
    MatrixStorage() {}
protected:
    T _data[R * C];
};

#if 0
/// Some Matrix types require aligned storage due to SIMD acceleration
template<>
class MatrixStorage<float, 4, 4>
{
public:
    MatrixStorage() {}
protected:
    alignas( 16 ) float _data[16];
};
#endif
} // Internal namespace

template<typename T, int R, int C, bool RowMajor = true>
class Matrix : public Internal::MatrixStorage<T, R, C>
{
    //static_assert( std::is_floating_point<T>::value, "Matrix template class requires floating point type!" );
    static constexpr int kSize = R * C;

    static inline constexpr int Idx( int row, int col )
    {
        return RowMajor ? row * C + col : col * R + row;
    }

public:
    using ScalarType = T;

    static constexpr int kRows = R;
    static constexpr int kCols = C;

    Matrix() { }

    Matrix( const Matrix<T, R, C, true> & src ) { *this = src; }

    Matrix( const Matrix<T, R, C, false> & src ) { *this = src; }

    /// Input in row-major order.
    Matrix( std::initializer_list<T> list ) { *this = list; }

    /// Input in row-major order.
    explicit Matrix( const std::vector<T> & data ) { *this = data; }

    /// Input in row-major order.
    explicit Matrix( const T * ptr )
    {
        for( int i = 0; i < R; ++i )
        {
            for( int j = 0; j < C; ++j )
            { this->_data[Idx( i, j )] = ptr[i * C + j]; }
        }
    }

    // generic size/type converting constructor
    // performs matrix truncation/expansion if necessary
    template<typename U, int M, int N, bool D>
    explicit Matrix( const Matrix<U, M, N, D> & src )
    {
        constexpr int kMinRow = M < R ? M : R;
        constexpr int kMinCol = N < C ? N : C;
        for( int i = 0; i < kMinRow; ++i )
        {
            for( int j = 0; j < kMinCol; ++j )
            { this->_data[Idx( i, j )] = static_cast<T>( src( i, j ) ); }
            for( int j = kMinCol; j < C; ++j )
            { this->_data[Idx( i, j )] = i == j ? T{1} : T{0}; }
        }
        for( int i = kMinRow; i < R; ++i )
        {
            for( int j = 0; j < C; ++j )
            { this->_data[Idx( i, j )] = i == j ? T{1} : T{0}; }
        }
    }

    template<int M, int N, bool D>
    Matrix & operator=( const Matrix<T, M, N, D> & src )
    {
        static_assert( R == M && C == N, "matrix size mismatch!" );
        if( RowMajor == D )
        {
            // same element ordering, just copy the elements
            const T * src_ptr = src.data();
            for( int i = 0; i < kSize; ++i ) { this->_data[i] = src_ptr[i]; }
        }
        else
        {
            // different element ordering requires a transpose
            for( int i = 0; i < R; ++i )
            {
                for( int j = 0; j < C; ++j )
                { this->_data[Idx( i, j )] = src( i, j ); }
            }
        }
        return *this;
    }

    /// Input in row-major order.
    Matrix & operator=( const std::vector<T> & src )
    {
        //if( src.size() != kSize )
        //{ LT_THROW( "source data and matrix size mismatch!" ); }

        auto it = src.begin();
        for( int i = 0; i < R; ++i )
        {
            for( int j = 0; j < C; ++j, ++it )
            { this->_data[Idx( i, j )] = *it; }
        }

        return *this;
    }

    /// Input in row-major order.
    Matrix & operator=( const std::initializer_list<T> src )
    {
        //if( src.size() != kSize )
        //{ LT_THROW( "source data and matrix size mismatch!" ); }

        auto it = src.begin();
        for( int i = 0; i < R; ++i )
        {
            for( int j = 0; j < C; ++j, ++it )
            { this->_data[Idx( i, j )] = *it; }
        }

        return *this;
    }

    Matrix & operator=( T scalar )
    {
        for( int i = 0; i < kSize; ++i ) { this->_data[i] = scalar; }
        return *this;
    }

    template<typename U, int M, int N, bool D>
    bool operator==( const Matrix<U, M, N, D> & mat ) const
    {
        static_assert( R == M && C == N, "matrix size mismatch!" );
        for( int i = 0; i < R; ++i )
        {
            for( int j = 0; j < C; ++j )
            { if( this->_data[Idx( i, j )] != mat( i, j ) ) { return false; } }
        }
        return true;
    }

    template<typename U, int M, int N, bool D>
    bool operator!=( const Matrix<U, M, N, D> & mat ) const
    { return !( *this == mat ); }

    const T * data() const { return this->_data; }
    T * data() { return this->_data; }

    T & operator()( int r, int c ) { return this->_data[Idx( r, c )]; }
    const T & operator()( int r, int c ) const { return this->_data[Idx( r, c )]; }

    template<int M, int N, bool D>
    Matrix<T, R, N> operator*( const Matrix<T, M, N, D> & mat ) const
    {
        static_assert( C == M, "invalid matrix product!" );
        Matrix<T, R, N> rval;

        // XXX: is this code actually more SIMD-friendly than a naive implementation?
        for( int i = 0; i < R; ++i )
        {
            // set 1st row
            for( int j = 0; j < N; ++j )
            { rval( i, j ) = this->_data[Idx( i, 0 )] * mat( 0, j ); }
            // accum n-th row
            for( int k = 1; k < M; ++k )
            {
                for( int j = 0; j < N; ++j )
                { rval( i, j ) += this->_data[Idx( i, k )] * mat( k, j ); }
            }
        }
        return rval;
    }

    template<int M, int N, bool D>
    Matrix & operator *=( const Matrix<T, M, N, D> & mat )
    {
        *this = *this * mat;
        return *this;
    }

    template<int M, int N, bool D>
    Matrix & operator +=( const Matrix<T, M, N, D> & mat )
    {
        static_assert( R == M && C == N, "matrix size mismatch!" );
        for( int i = 0; i < R; ++i )
        {
            for( int j = 0; j < C; ++j )
            { this->_data[Idx( i, j )] += mat( i, j ); }
        }
        return *this;
    }

    template<int M, int N, bool D>
    Matrix & operator -=( const Matrix<T, M, N, D> & mat )
    {
        static_assert( R == M && C == N, "matrix size mismatch!" );
        for( int i = 0; i < R; ++i )
        {
            for( int j = 0; j < C; ++j )
            { this->_data[Idx( i, j )] -= mat( i, j ); }
        }
        return *this;
    }

    Matrix & operator *=( T scalar )
    {
        for( int i = 0; i < kSize; ++i ) { this->_data[i] *= scalar; }
        return *this;
    }

    Matrix & operator +=( T scalar )
    {
        for( int i = 0; i < kSize; ++i ) { this->_data[i] += scalar; }
        return *this;
    }

    Matrix & operator /=( T scalar )
    {
        scalar = 1.0f / scalar;
        for( int i = 0; i < kSize; ++i ) { this->_data[i] *= scalar; }
        return *this;
    }

    Matrix & operator -=( T scalar )
    {
        for( int i = 0; i < kSize; ++i ) { this->_data[i] -= scalar; }
        return *this;
    }

    template<int M, int N, bool D>
    Matrix operator+( const Matrix<T, M, N, D> & mat ) const
    { Matrix rval{*this}; rval += mat; return rval; }

    template<int M, int N, bool D>
    Matrix operator-( const Matrix<T, M, N, D> & mat ) const
    { Matrix rval{*this}; rval -= mat; return rval; }

    Matrix operator*( T scalar ) const
    { Matrix rval{*this}; rval *= scalar; return rval; }

    Matrix operator+( T scalar ) const
    { Matrix rval{*this}; rval += scalar; return rval; }

    Matrix operator-( T scalar ) const
    { Matrix rval{*this}; rval -= scalar; return rval; }

    Matrix operator/( T scalar ) const
    { Matrix rval{*this}; rval /= scalar; return rval; }

    Matrix<T, 1, C> row( int i ) const
    {
        Matrix<T, 1, C> rval;
        for( int j = 0; j < C; ++j ) { rval( 0, j ) = this->_data[Idx( i, j )]; }
        return rval;
    }

    Matrix<T, 1, C> column( int i ) const
    {
        Matrix<T, R, 1> rval;
        for( int j = 0; j < R; ++j ) { rval( j, 0 ) = this->_data[Idx( j, i )]; }
        return rval;
    }

    Matrix & swapRow( int i, int j )
    {
        using std::swap;
        for( int k = 0; k < C; ++k )
        { swap( this->_data[Idx( i, k )], this->_data[Idx( j, k )] ); }
        return *this;
    }

    Matrix & swapColumn( int i, int j )
    {
        using std::swap;
        for( int k = 0; k < R; ++k )
        { swap( this->_data[Idx( k, i )], this->_data[Idx( k, j )] ); }
        return *this;
    }

    T sum() const
    {
        T sum = this->_data[0];
        for( int j = 1; j < kSize; ++j ) { sum += this->_data[j]; }
        return sum;
    }

    Matrix & setIdentity()
    {
        for( int i = 0; i < R; ++i )
        {
            for( int j = 0; j < C; ++j )
            { this->_data[Idx( i, j )] = i == j ? T{1} : T{0}; }
        }
        return *this;
    }

    const Matrix < T, C, R, !RowMajor > & transpose() const
    {
        return *reinterpret_cast < const Matrix < T, C, R, !RowMajor > * >( this );
    }

    // friend std::ostream & operator<<( std::ostream & os, const Matrix & m )
    // {
    //     std::stringstream sb;
    //     sb << std::endl << "[ ";
    //     for( int i = 0; i < R; ++i )
    //     {
    //         for( int j = 0; j < C; ++j ) { sb << m( i, j ) << " "; }
    //         if( i != R - 1 ) { sb << std::endl; }
    //     }
    //     sb << "]" << std::endl;
    //     return os << sb.rdbuf();
    // }

    static constexpr bool IsRowMajor() { return RowMajor; }

    static Matrix Identity()
    {
        Matrix rval;
        return rval.setIdentity();
    }
};

template<typename T, int R, int C, bool D>
Matrix < T, C, R, !D > transpose( const Matrix<T, R, C, D> & mat )
{
    // simply copy the data and flip the matrix ordering
    Matrix < T, C, R, !D > rval;
    T * dst = rval.data();
    const T * src = mat.data();
    for( int i = 0; i < R * C; ++i ) { dst[i] = src[i]; }
    return rval;
}

template<typename T, int N, bool D>
Matrix<T, N, N, D> inverse( const Matrix<T, N, N, D> & mat );

template<typename T, int N, bool D>
T determinant( const Matrix<T, N, N, D> & mat );

// scalar-matrix operators
template<typename T, typename U, int R, int C, bool D>
typename std::enable_if<std::is_arithmetic<T>::value, Matrix<U, R, C, D> >::type
operator+( T a, const Matrix<U, R, C, D> & b ) { return b + a; }

template<typename T, typename U, int R, int C, bool D>
typename std::enable_if<std::is_arithmetic<T>::value, Matrix<U, R, C, D> >::type
operator-( T a, const Matrix<U, R, C, D> & b )
{
    Matrix<U, R, C, D> rval;
    for( int i = 0; i < R; ++i )
    { for( int j = 0; j < C; ++j ) { rval( i, j ) = U( a ) - b( i, j ); } }
    return rval;
}

template<typename T, typename U, int R, int C, bool D>
typename std::enable_if<std::is_arithmetic<T>::value, Matrix<U, R, C, D> >::type
operator*( T a, const Matrix<U, R, C, D> & b ) { return b * a; }

using Matrix2x2f = Matrix<float, 2, 2>;
using Matrix3x3f = Matrix<float, 3, 3>;
using Matrix4x4f = Matrix<float, 4, 4>;
using Matrix2x2d = Matrix<double, 2, 2>;
using Matrix3x3d = Matrix<double, 3, 3>;
using Matrix4x4d = Matrix<double, 4, 4>;

// optimized implementation of matrix-vector operations
// mind the order of color channels in memory is BGRA, i.e.,
// blue channel corresponds to the first matrix dimension!

template<bool D>
Vec4f operator*( const Matrix<float, 4, 4, D> & mat, const Vec4f & vec )
{
    return
    {
        mat( 0, 0 ) * vec.x() + mat( 0, 1 ) * vec.y() + mat( 0, 2 ) * vec.z() + mat( 0, 3 ) * vec.a(),
        mat( 1, 0 ) * vec.x() + mat( 1, 1 ) * vec.y() + mat( 1, 2 ) * vec.z() + mat( 1, 3 ) * vec.a(),
        mat( 2, 0 ) * vec.x() + mat( 2, 1 ) * vec.y() + mat( 2, 2 ) * vec.z() + mat( 2, 3 ) * vec.a(),
        mat( 3, 0 ) * vec.x() + mat( 3, 1 ) * vec.y() + mat( 3, 2 ) * vec.z() + mat( 3, 3 ) * vec.a()
    };
}

template<bool D>
Vec4f operator*( const Vec4f & vec, const Matrix<float, 4, 4, D> & mat )
{
    return
    {
        mat( 0, 0 ) * vec.x() + mat( 1, 0 ) * vec.y() + mat( 2, 0 ) * vec.z() + mat( 3, 0 ) * vec.a(),
        mat( 0, 1 ) * vec.x() + mat( 1, 1 ) * vec.y() + mat( 2, 1 ) * vec.z() + mat( 3, 1 ) * vec.a(),
        mat( 0, 2 ) * vec.x() + mat( 1, 2 ) * vec.y() + mat( 2, 2 ) * vec.z() + mat( 3, 2 ) * vec.a(),
        mat( 0, 3 ) * vec.x() + mat( 1, 3 ) * vec.y() + mat( 2, 3 ) * vec.z() + mat( 3, 3 ) * vec.a()
    };
}

// Vec3 left- and right-hand side multiplication operator
template<typename T, typename U, bool D>
Vec3<typename std::common_type<T, U>::type> operator*( const Matrix<T, 3, 3, D> & mat, const Vec3<U> & vec )
{
    return
    {
        mat( 0, 0 ) * vec.x() + mat( 0, 1 ) * vec.y() + mat( 0, 2 ) * vec.z(),
        mat( 1, 0 ) * vec.x() + mat( 1, 1 ) * vec.y() + mat( 1, 2 ) * vec.z(),
        mat( 2, 0 ) * vec.x() + mat( 2, 1 ) * vec.y() + mat( 2, 2 ) * vec.z()
    };
}

template<typename T, typename U, bool D>
Vec3<typename std::common_type<T, U>::type> operator*( const Vec3<T> & vec, const Matrix<U, 3, 3, D> & mat )
{
    return
    {
        mat( 0, 0 ) * vec.x() + mat( 1, 0 ) * vec.y() + mat( 2, 0 ) * vec.z(),
        mat( 0, 1 ) * vec.x() + mat( 1, 1 ) * vec.y() + mat( 2, 1 ) * vec.z(),
        mat( 0, 2 ) * vec.x() + mat( 1, 2 ) * vec.y() + mat( 2, 2 ) * vec.z()
    };
}


// mat3x3 and vec2 special multiplication operation used in roitransfer
/// Return H * p for 2D p, p is made homogeneous, division in the end.
template<typename T, typename U, bool D>
Vec2<typename std::common_type<T, U>::type> operator*( const Matrix<T, 3, 3, D> & h, const Vec2<U> & p )
{
    T w_inv = T{1} / ( h(2, 0) * p.x() + h(2, 1) * p.y() + h(2, 2) );
    return  { 
                ( h(0, 0) * p.x() + h(0, 1) * p.y() + h(0, 2) ) * w_inv,
                ( h(1, 0) * p.x() + h(1, 1) * p.y() + h(1, 2) ) * w_inv
            };
}



// Vec2 multiplication operator
//right hand
template<typename T, typename U, bool D>
Vec2<typename std::common_type<T, U>::type> operator*( const Matrix<T, 2, 2, D> & mat, const Vec2<U> & vec )
{
    return
    {
        mat( 0, 0 ) * vec.x() + mat( 0, 1 ) * vec.y(),
        mat( 1, 0 ) * vec.x() + mat( 1, 1 ) * vec.y()
    };
}
//left hand
template<typename T, typename U, bool D>
Vec2<typename std::common_type<T, U>::type> operator*( const Vec2<U> & vec, const Matrix<T, 2, 2, D> & mat )
{
    return
    {
        mat( 0, 0 ) * vec.x() + mat( 1, 0 ) * vec.y(),
        mat( 0, 1 ) * vec.x() + mat( 1, 1 ) * vec.y()
    };
}

// Create a diagonal matrix from a vector
inline Matrix4x4f diagonal( const Vec4f & vec )
{
    return
    {
        vec.x(), 0.0f, 0.0f, 0.0f,
        0.0f, vec.y(), 0.0f, 0.0f,
        0.0f, 0.0f, vec.z(), 0.0f,
        0.0f, 0.0f, 0.0f, vec.a()
    };
}

template<typename T>
Matrix<T, 3, 3> diagonal( const Vec3<T> & vec )
{
    return
    {
        vec.x(), T{0}, T{0},
        T{0}, vec.y(), T{0},
        T{0}, T{0}, vec.z()
    };
}

template<typename T>
Matrix<T, 2, 2> diagonal( const Vec2<T> & vec )
{
    return
    {
        vec.x(), T{0},
        T{0}, vec.y(),
    };
}

// ------------------------------------
// determinant, inverse, etc
// ------------------------------------

template<typename T, int N>
struct MatrixDeterminant { };

template<typename T>
struct MatrixDeterminant<T, 2>
{
    T operator()( const T * mat ) const
    {
        return mat[0] * mat[3] - mat[1] * mat[2];
    }
};

template<typename T>
struct MatrixDeterminant<T, 3>
{
    T operator()( const T * mat ) const
    {
        return + mat[0] * ( mat[4] * mat[8] - mat[7] * mat[5] )
               - mat[1] * ( mat[3] * mat[8] - mat[5] * mat[6] )
               + mat[2] * ( mat[3] * mat[7] - mat[4] * mat[6] );
    }
};

template<typename T>
struct MatrixDeterminant<T, 4>
{
    T operator()( const T * mat ) const
    {
        T dst[4];
        T tmp[12];

        // calculate pairs for first 8 elements (cofactors)
        tmp[0] = mat[10] * mat[15];
        tmp[1] = mat[11] * mat[14];
        tmp[2] = mat[9] * mat[15];
        tmp[3] = mat[11] * mat[13];
        tmp[4] = mat[9] * mat[14];
        tmp[5] = mat[10] * mat[13];
        tmp[6] = mat[8] * mat[15];
        tmp[7] = mat[11] * mat[12];
        tmp[8] = mat[8] * mat[14];
        tmp[9] = mat[10] * mat[12];
        tmp[10] = mat[8] * mat[13];
        tmp[11] = mat[9] * mat[12];

        // calculate first 8 elements (cofactors)
        dst[0]  = tmp[0] * mat[5] + tmp[3] * mat[6] + tmp[4]  * mat[7];
        dst[0] -= tmp[1] * mat[5] + tmp[2] * mat[6] + tmp[5]  * mat[7];
        dst[1]  = tmp[1] * mat[4] + tmp[6] * mat[6] + tmp[9]  * mat[7];
        dst[1] -= tmp[0] * mat[4] + tmp[7] * mat[6] + tmp[8]  * mat[7];
        dst[2]  = tmp[2] * mat[4] + tmp[7] * mat[5] + tmp[10] * mat[7];
        dst[2] -= tmp[3] * mat[4] + tmp[6] * mat[5] + tmp[11] * mat[7];
        dst[3]  = tmp[5] * mat[4] + tmp[8] * mat[5] + tmp[11] * mat[6];
        dst[3] -= tmp[4] * mat[4] + tmp[9] * mat[5] + tmp[10] * mat[6];

        // calculate determinant
        return mat[0] * dst[0] + mat[1] * dst[1] + mat[2] * dst[2] + mat[3] * dst[3];
    }
};


template<typename T, int N>
struct SquareMatrixInverter { };

template<typename T>
struct SquareMatrixInverter<T, 2>
{
    void run( T * mat ) const
    {
        const T det = mat[0] * mat[3] - mat[1] * mat[2];

        const T invdet = T{1} / det;

        T tmp[4];
        tmp[0] = mat[3];
        tmp[1] = -mat[1];
        tmp[2] = -mat[2];
        tmp[3] = mat[0];

        for( int i = 0; i < 4; ++i ) { mat[i] = tmp[i] * invdet; }
    }
};

template<typename T>
struct SquareMatrixInverter<T, 3>
{
    void run( T * mat ) const
    {
        const T det = + mat[0] * ( mat[4] * mat[8] - mat[7] * mat[5] )
                      - mat[1] * ( mat[3] * mat[8] - mat[5] * mat[6] )
                      + mat[2] * ( mat[3] * mat[7] - mat[4] * mat[6] );

        const T invdet = T{1} / det;

        T tmp[9];
        tmp[0] =  mat[4] * mat[8] - mat[7] * mat[5];
        tmp[1] = -( mat[1] * mat[8] - mat[2] * mat[7] );
        tmp[2] =  mat[1] * mat[5] - mat[2] * mat[4];
        tmp[3] = -( mat[3] * mat[8] - mat[5] * mat[6] );
        tmp[4] =  mat[0] * mat[8] - mat[2] * mat[6];
        tmp[5] = -( mat[0] * mat[5] - mat[3] * mat[2] );
        tmp[6] =  mat[3] * mat[7] - mat[6] * mat[ 4];
        tmp[7] = -( mat[0] * mat[7] - mat[6] * mat[1] );
        tmp[8] =  mat[0] * mat[4] - mat[3] * mat[1];

        for( int i = 0; i < 9; ++i ) { mat[i] = tmp[i] * invdet; }
    }
};

template<typename T>
struct SquareMatrixInverter<T, 4>
{
    void run( T * mat ) const
    {
        T dst[16];
        T tmp[12];

        // calculate pairs for first 8 elements (cofactors)
        tmp[0] = mat[10] * mat[15];
        tmp[1] = mat[11] * mat[14];
        tmp[2] = mat[9] * mat[15];
        tmp[3] = mat[11] * mat[13];
        tmp[4] = mat[9] * mat[14];
        tmp[5] = mat[10] * mat[13];
        tmp[6] = mat[8] * mat[15];
        tmp[7] = mat[11] * mat[12];
        tmp[8] = mat[8] * mat[14];
        tmp[9] = mat[10] * mat[12];
        tmp[10] = mat[8] * mat[13];
        tmp[11] = mat[9] * mat[12];

        // calculate first 8 elements (cofactors)
        dst[0]   = tmp[0] * mat[5] + tmp[3] * mat[6] + tmp[4]  * mat[7];
        dst[0]  -= tmp[1] * mat[5] + tmp[2] * mat[6] + tmp[5]  * mat[7];
        dst[4]   = tmp[1] * mat[4] + tmp[6] * mat[6] + tmp[9]  * mat[7];
        dst[4]  -= tmp[0] * mat[4] + tmp[7] * mat[6] + tmp[8]  * mat[7];
        dst[8]   = tmp[2] * mat[4] + tmp[7] * mat[5] + tmp[10] * mat[7];
        dst[8]  -= tmp[3] * mat[4] + tmp[6] * mat[5] + tmp[11] * mat[7];
        dst[12]  = tmp[5] * mat[4] + tmp[8] * mat[5] + tmp[11] * mat[6];
        dst[12] -= tmp[4] * mat[4] + tmp[9] * mat[5] + tmp[10] * mat[6];

        // calculate determinant
        const T det = mat[0] * dst[0] + mat[1] * dst[4] + mat[2] * dst[8] + mat[3] * dst[12];

        const T inv_det = T{1} / det;

        dst[1]   = tmp[1] * mat[1] + tmp[2] * mat[2] + tmp[5]  * mat[3];
        dst[1]  -= tmp[0] * mat[1] + tmp[3] * mat[2] + tmp[4]  * mat[3];
        dst[5]   = tmp[0] * mat[0] + tmp[7] * mat[2] + tmp[8]  * mat[3];
        dst[5]  -= tmp[1] * mat[0] + tmp[6] * mat[2] + tmp[9]  * mat[3];
        dst[9]   = tmp[3] * mat[0] + tmp[6] * mat[1] + tmp[11] * mat[3];
        dst[9]  -= tmp[2] * mat[0] + tmp[7] * mat[1] + tmp[10] * mat[3];
        dst[13]  = tmp[4] * mat[0] + tmp[9] * mat[1] + tmp[10] * mat[2];
        dst[13] -= tmp[5] * mat[0] + tmp[8] * mat[1] + tmp[11] * mat[2];

        // calculate pairs for second 8 elements (cofactors)
        tmp[0] = mat[2] * mat[7];
        tmp[1] = mat[3] * mat[6];
        tmp[2] = mat[1] * mat[7];
        tmp[3] = mat[3] * mat[5];
        tmp[4] = mat[1] * mat[6];
        tmp[5] = mat[2] * mat[5];
        tmp[6] = mat[0] * mat[7];
        tmp[7] = mat[3] * mat[4];
        tmp[8] = mat[0] * mat[6];
        tmp[9] = mat[2] * mat[4];
        tmp[10] = mat[0] * mat[5];
        tmp[11] = mat[1] * mat[4];

        // calculate second 8 elements (cofactors)
        dst[2]   = tmp[0]  * mat[13] + tmp[3]  * mat[14] + tmp[4]  * mat[15];
        dst[2]  -= tmp[1]  * mat[13] + tmp[2]  * mat[14] + tmp[5]  * mat[15];
        dst[3]   = tmp[2]  * mat[10] + tmp[5]  * mat[11] + tmp[1]  * mat[9];
        dst[3]  -= tmp[4]  * mat[11] + tmp[0]  * mat[9]  + tmp[3]  * mat[10];
        dst[6]   = tmp[1]  * mat[12] + tmp[6]  * mat[14] + tmp[9]  * mat[15];
        dst[6]  -= tmp[0]  * mat[12] + tmp[7]  * mat[14] + tmp[8]  * mat[15];
        dst[7]   = tmp[8]  * mat[11] + tmp[0]  * mat[8]  + tmp[7]  * mat[10];
        dst[7]  -= tmp[6]  * mat[10] + tmp[9]  * mat[11] + tmp[1]  * mat[8];
        dst[10]  = tmp[2]  * mat[12] + tmp[7]  * mat[13] + tmp[10] * mat[15];
        dst[10] -= tmp[3]  * mat[12] + tmp[6]  * mat[13] + tmp[11] * mat[15];
        dst[11]  = tmp[6]  * mat[9]  + tmp[11] * mat[11] + tmp[3]  * mat[8];
        dst[11] -= tmp[10] * mat[11] + tmp[2]  * mat[8]  + tmp[7]  * mat[9];
        dst[14]  = tmp[5]  * mat[12] + tmp[8]  * mat[13] + tmp[11] * mat[14];
        dst[14] -= tmp[4]  * mat[12] + tmp[9]  * mat[13] + tmp[10] * mat[14];
        dst[15]  = tmp[10] * mat[10] + tmp[4]  * mat[8]  + tmp[9]  * mat[9];
        dst[15] -= tmp[8]  * mat[9]  + tmp[11] * mat[10] + tmp[5]  * mat[8];

        // calculate matrix inverse
        for( int i = 0; i < 16; i++ ) { mat[i] = dst[i] * inv_det; }
    }
};

template<typename T, int N, bool D>
Matrix<T, N, N, D> inverse( const Matrix<T, N, N, D> & mat )
{
    Matrix<T, N, N, D> rval( mat );
    SquareMatrixInverter<T, N> inverter;
    inverter.run( rval.data() );
    return rval;
}

template<typename T, int N, bool D>
T determinant( const Matrix<T, N, N, D> & mat )
{
    MatrixDeterminant<T, N> det;
    return det( mat.data() );
}

template Matrix<float, 2, 2, true>   inverse( const Matrix<float, 2, 2, true> & );
template Matrix<float, 3, 3, true>   inverse( const Matrix<float, 3, 3, true> & );
template Matrix<float, 4, 4, true>   inverse( const Matrix<float, 4, 4, true> & );

template Matrix<float, 2, 2, false>  inverse( const Matrix<float, 2, 2, false> & );
template Matrix<float, 3, 3, false>  inverse( const Matrix<float, 3, 3, false> & );
template Matrix<float, 4, 4, false>  inverse( const Matrix<float, 4, 4, false> & );

template Matrix<double, 2, 2, true>  inverse( const Matrix<double, 2, 2, true> & );
template Matrix<double, 3, 3, true>  inverse( const Matrix<double, 3, 3, true> & );
template Matrix<double, 4, 4, true>  inverse( const Matrix<double, 4, 4, true> & );

template Matrix<double, 2, 2, false> inverse( const Matrix<double, 2, 2, false> & );
template Matrix<double, 3, 3, false> inverse( const Matrix<double, 3, 3, false> & );
template Matrix<double, 4, 4, false> inverse( const Matrix<double, 4, 4, false> & );

template float  determinant( const Matrix<float, 2, 2, true> & );
template float  determinant( const Matrix<float, 3, 3, true> & );
template float  determinant( const Matrix<float, 4, 4, true> & );

template float  determinant( const Matrix<float, 2, 2, false> & );
template float  determinant( const Matrix<float, 3, 3, false> & );
template float  determinant( const Matrix<float, 4, 4, false> & );

template double determinant( const Matrix<double, 2, 2, true> & );
template double determinant( const Matrix<double, 3, 3, true> & );
template double determinant( const Matrix<double, 4, 4, true> & );

template double determinant( const Matrix<double, 2, 2, false> & );
template double determinant( const Matrix<double, 3, 3, false> & );
template double determinant( const Matrix<double, 4, 4, false> & );

} // end namespace ltaf

#endif // #ifdef c++

#endif //#define __MATRIX_H__
