#pragma once
#include <stdint.h>

namespace ZFeedforwardResearch {
struct Point {int32_t x,y,z;};
inline uint32_t distance(int32_t a,int32_t b) {
    return a<=b?uint32_t(b)-uint32_t(a):uint32_t(a)-uint32_t(b);
}
struct Path {
    Point first,last;
    uint32_t primary;
    bool valid() const {
        return primary && primary<=INT32_MAX && distance(first.x,last.x)<=primary &&
            distance(first.y,last.y)<=primary && distance(first.z,last.z)<=primary;
    }
    int32_t axis(int32_t a,int32_t b,uint32_t i) const {
        const uint32_t d=distance(a,b),bias=(primary-1)/2;
        if(!d)return a;
        if(i==primary)return b;
        const uint32_t offset=primary<=65535U?(d*i+bias)/primary:
            uint32_t((uint64_t(d)*i+bias)/primary);
        return a<=b?a+int32_t(offset):a-int32_t(offset);
    }
    // Last sample inside an inclusive coordinate band, for this monotone
    // path. Invert the exact integer Bresenham expression, not a float t.
    // Precondition: current point is inside the band.
    uint32_t lastInside(int32_t a,int32_t b,int32_t low,int32_t high) const {
        if(b>=low&&b<=high)return primary;
        const uint32_t d=distance(a,b);
        const uint32_t allowed=b>a?distance(a,high):distance(a,low);
        const uint32_t bias=(primary-1)/2;
        if(primary<=65535U)return ((allowed+1)*primary-1-bias)/d;
        return uint32_t((uint64_t(allowed+1)*primary-1-bias)/d);
    }
    Point at(uint32_t i) const {
        return {axis(first.x,last.x,i),axis(first.y,last.y,i),axis(first.z,last.z,i)};
    }
};
enum class Zone:uint8_t {Negative,Zero,Full,Fade,Flat};
struct Region {
    Zone zone;
    uint8_t xLow,xHigh,yLow,yHigh;
    uint32_t widthX,widthY,fadeSpan,slopeX,slopeY,cross;
    Point min,max;
    bool same(const Region& r) const {
        return zone==r.zone&&xLow==r.xLow&&xHigh==r.xHigh&&yLow==r.yLow&&yHigh==r.yHigh;
    }
};
struct Prepared {
    uint32_t first;
    uint16_t length;
    int32_t motorStart,motorEnd;
    bool zeroTransition;
};
// One bounded foreground cursor. Reader owns immutable matrix/fade context.
// This prepares chord endpoints and conservative *continuous curvature*,
// not a bound for staged firmware rounding, actual pulse acceleration,
// foreground availability or admission. Reader rejects invalid matrices.
class Preparer {
    Path path_{};
    uint32_t index_=0;
    bool valid_=false;
    static uint32_t q8(uint32_t delta,uint32_t width) {
        if(!width)return 0;
        if(delta>width)return 257;
        if(width<=0x00ff00ffUL)return (delta*256+width-1)/width;
        return uint32_t((uint64_t(delta)*256+width-1)/width);
    }
    static bool curvatureFits(const Region& r,Point a,Point b) {
        const uint32_t u=q8(distance(a.x,b.x),r.widthX),v=q8(distance(a.y,b.y),r.widthY),
            w=q8(distance(a.z,b.z),r.fadeSpan);
        if(u>256||v>256||w>256)return false;
        // B along a line is quadratic; C=(B-O)*fade+O. For one
        // cell: |C''| <= 2|cross*u*v| + 2|B'|*|fade'|.
        // |B'| <= slopeX*u+slopeY*v. Q8 outward rounding.
        // Bound chord error by max|C''|/8 <= 1 step.
        const uint64_t bound=uint64_t(r.cross)*u*v+
            (uint64_t(r.slopeX)*u+uint64_t(r.slopeY)*v)*w;
        return bound<=4UL*65536;
    }
    static bool nearZero(Zone z){return z==Zone::Negative||z==Zone::Zero;}
public:
    bool start(Path path){path_=path;index_=0;valid_=path.valid();return valid_;}
    uint32_t index()const{return index_;}
    template<class Reader>
    bool next(Reader& reader,Prepared& out) {
        if(!valid_||index_==path_.primary)return false;
        const uint32_t i=index_;
        const Point a=path_.at(i);
        const Region ra=reader.region(a);
        if(a.x<ra.min.x||a.x>ra.max.x||a.y<ra.min.y||a.y>ra.max.y||a.z<ra.min.z||a.z>ra.max.z){valid_=false;return false;}
        uint32_t j=i+((path_.primary-i)>32767U?32767U:path_.primary-i);
        const auto limit=[&](uint32_t v){if(v<j)j=v;};
        limit(path_.lastInside(path_.first.x,path_.last.x,ra.min.x,ra.max.x));
        limit(path_.lastInside(path_.first.y,path_.last.y,ra.min.y,ra.max.y));
        limit(path_.lastInside(path_.first.z,path_.last.z,ra.min.z,ra.max.z));
        if(j<i){valid_=false;return false;}
        if(j==i)++j; // exactly one boundary transition
        Point b=path_.at(j);Region rb=reader.region(b);
        if(j-i>1&&!ra.same(rb)){valid_=false;return false;}
        while(j-i>1&&!curvatureFits(ra,a,b)) {
            j=i+(j-i)/2;b=path_.at(j);rb=reader.region(b);
        }
        const int64_t p0=int64_t(a.z)+reader.correction(a),p1=int64_t(b.z)+reader.correction(b);
        if(p0<INT32_MIN||p0>INT32_MAX||p1<INT32_MIN||p1>INT32_MAX){valid_=false;return false;}
        out={i,uint16_t(j-i),int32_t(p0),int32_t(p1),ra.zone!=rb.zone&&(nearZero(ra.zone)||nearZero(rb.zone))};
        index_=j;return true;
    }
};
}
