#pragma once
#include "z-chord-preparer.h"

namespace ZFeedforwardResearch {
// Counterfactual: same chord boundary decisions and endpoints as Preparer,
// but do not rediscover a known interior region during curvature bisection.
// Reuse the previous endpoint, which is immutable within this context.
class FastPreparer {
    Path path_{};
    uint32_t index_=0;
    Point previous_{};
    int32_t motor_=0;
    bool valid_=false,haveEndpoint_=false;
    static uint32_t q8(uint32_t delta,uint32_t width) {
        if(!width||!delta)return 0;
        if(delta>width)return 257;
        if(delta==width)return 256;
        if(width<=0x00ff00ffUL)return (delta*256+width-1)/width;
        return uint32_t((uint64_t(delta)*256+width-1)/width);
    }
    // Decide c*x*y <= budget without 64-bit multiplication. x,y <= 256,
    // coefficient bound follows from differences of four int16 matrix cells.
    // If both factors exceed these thresholds, product > 2^18.
    static bool charge(uint32_t& budget,uint32_t c,uint32_t x,uint32_t y){
        if(!c||!x||!y)return true;
        if(c>131070UL||x>256||y>256)return false;
        const uint32_t scale=x*y;
        if(c>1024&&scale>256)return false;
        const uint32_t cost=c*scale; // <= max(131070*256,1024*65536)
        if(cost>budget)return false;
        budget-=cost;
        return true;
    }
    static bool fits(const Region& r,Point a,Point b){
        if(!r.cross&&!r.fadeSpan)return true;
        const uint32_t u=q8(distance(a.x,b.x),r.widthX),v=q8(distance(a.y,b.y),r.widthY),
            w=q8(distance(a.z,b.z),r.fadeSpan);
        if(u>256||v>256||w>256)return false;
        uint32_t budget=4UL*65536;
        return charge(budget,r.cross,u,v)&&charge(budget,r.slopeX,u,w)&&charge(budget,r.slopeY,v,w);
    }
    static bool nearZero(Zone z){return z==Zone::Negative||z==Zone::Zero;}
    static bool inside(Point p,const Region& r){
        return p.x>=r.min.x&&p.x<=r.max.x&&p.y>=r.min.y&&p.y<=r.max.y&&p.z>=r.min.z&&p.z<=r.max.z;
    }
    static bool motorPosition(Point p,int32_t correction,int32_t& result){
        if((correction>0&&p.z>INT32_MAX-correction)||
           (correction<0&&p.z<INT32_MIN-correction))return false;
        result=p.z+correction;return true;
    }
    int32_t axis(int32_t a,int32_t b,uint32_t i)const{
        if(!i||a==b)return a;
        if(i==path_.primary)return b;
        const uint32_t d=distance(a,b);
        if(d==path_.primary)return a<=b?a+int32_t(i):a-int32_t(i);
        return path_.axis(a,b,i);
    }
    Point at(uint32_t i)const{
        return {axis(path_.first.x,path_.last.x,i),axis(path_.first.y,path_.last.y,i),axis(path_.first.z,path_.last.z,i)};
    }
public:
    bool start(Path path){
        path_=path;index_=0;previous_=path.first;haveEndpoint_=false;
        valid_=path.valid();return valid_;
    }
    uint32_t index()const{return index_;}
    template<class Reader>
    bool next(Reader& reader,Prepared& out){
        if(!valid_||index_==path_.primary)return false;
        const uint32_t i=index_;
        const Point a=previous_;
        const Region ra=reader.region(a);
        if(!inside(a,ra)){valid_=false;return false;}
        uint32_t j=i+((path_.primary-i)>32767U?32767U:path_.primary-i);
        const auto limit=[&](uint32_t v){if(v<j)j=v;};
        limit(path_.lastInside(path_.first.x,path_.last.x,ra.min.x,ra.max.x));
        limit(path_.lastInside(path_.first.y,path_.last.y,ra.min.y,ra.max.y));
        limit(path_.lastInside(path_.first.z,path_.last.z,ra.min.z,ra.max.z));
        if(j<i){valid_=false;return false;}
        if(j==i)++j;
        Point b=at(j);
        if(j-i>1&&!inside(b,ra)){valid_=false;return false;}
        while(j-i>1&&!fits(ra,a,b)){
            j=i+(j-i)/2;b=at(j);
        }
        // Only a one-sample boundary bridge may leave the current band.
        bool zeroTransition=false;
        if(!inside(b,ra)){
            const Region rb=reader.region(b);
            zeroTransition=ra.zone!=rb.zone&&(nearZero(ra.zone)||nearZero(rb.zone));
        }
        int32_t p0=motor_,p1;
        if(!haveEndpoint_&&!motorPosition(a,reader.correction(a),p0)){valid_=false;return false;}
        if(!motorPosition(b,reader.correction(b),p1)){valid_=false;return false;}
        out={i,uint16_t(j-i),p0,p1,zeroTransition};
        index_=j;previous_=b;motor_=p1;haveEndpoint_=true;
        return true;
    }
};
}
