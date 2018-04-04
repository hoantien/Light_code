#include "../include/line2.h"

namespace ltaf{

/// Calculate two points on the box boundary where the line intersects the box.
/// If the line doesn't intersect, return false.
bool IntersectBox( const Line2f & line, float start_x, float start_y, int width, int height, 
                    Vec2f & p1, Vec2f & p2 )
{
    float end_x = start_x + width;
    float end_y = start_y + height;

    auto inSpan = []( int coord, float start, float end ) { return coord >= start && coord < end; };

    if( std::abs( line.y() ) < Line2f::kEps10x )
    {
        // vertical
        float top_x = line.xGivenY( start_y );
        if( !inSpan( top_x, start_x, end_x ) ) { return false; }
        p1 = { top_x, start_y };
        p2 = { top_x, end_y   };
        return true;
    }
    if( std::abs( line.x() ) < Line2f::kEps10x )
    {
        // horizontal
        float left_y = line.yGivenX( start_x );
        if( !inSpan( left_y, start_y, end_y ) ) { return false; }
        p1 = { start_x, left_y };
        p2 = { end_x,   left_y };
        return true;
    }

    // find the four intersections
    bool first_done = false;

    // left
    float left_y  = line.yGivenX( start_x );
    if( inSpan( left_y, start_y, end_y ) )
    {
        p1 = { start_x, left_y }; first_done = true;
    }

    // right
    float right_y = line.yGivenX( end_x );
    if( inSpan( right_y, start_y, end_y ) )
    {
        if( first_done )
        {
            p2 = { end_x, right_y };
            return true;
        }
        p1 = { end_x, right_y };
        first_done = 1;
    }

    // top
    float top_x   = line.xGivenY( start_y );
    if( inSpan( top_x, start_x, end_x ) )
    {
        if( first_done )
        {
            p2 = { top_x, start_y };
            return true;
        }
        p1 = { top_x, start_y };
        first_done = 1;
    }

    // bottom
    float bot_x   = line.xGivenY( end_y );
    if( inSpan( bot_x, start_x, end_x ) )
    {
        if( first_done )
        {
            p2 = { bot_x, end_y };
            return true;
        }
    }
    return false;
}

}