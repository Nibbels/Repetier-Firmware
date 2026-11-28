#include "z-chord-handoff.h"
#include <cassert>
#include <cstdio>
using ZFeedforwardResearch::Handoff;
using ZFeedforwardResearch::Tag;
using Take=Handoff::Take;
int main(){
    Handoff h;
    uint64_t advances=0,probes=0,stale=0;
    for(uint32_t move=1;move<=10000;++move){
        Tag t{move,0,uint16_t(move)};
        assert(h.offer(t,-27,18,53,[&]{
            assert(h.take(t,-27)==Take::Missing);
            ++probes;
        }));
        assert(h.take(t,-27)==Take::Accepted);
        Tag next{move,53,uint16_t(move)};
        assert(h.offer(next,18,-27,53,[&]{
            assert(h.take(next,18)==Take::Busy);
            assert(h.advance());++advances;++probes;
        }));
        assert(!h.offer(next,0,0,1));
        while(h.remaining()){assert(h.advance());++advances;}
        assert(h.value()==18);
        assert(!h.advance());
        assert(h.take(next,18)==Take::Accepted);
        while(h.remaining()){assert(h.advance());++advances;}
        assert(h.value()==-27);
        h.invalidate();assert(!h.advance());
        Tag old{move,106,uint16_t(move)},changed{move+1,0,uint16_t(move+1)};
        assert(h.offer(old,-27,40,5,[&]{
            h.invalidate();
            assert(h.take(changed,-27)==Take::Missing);
            ++probes;
        }));
        assert(h.take(changed,-27)==Take::Stale);++stale;
        assert(!h.valid()&&!h.advance());
        assert(h.offer(changed,-26,40,5));
        assert(h.take(changed,-27)==Take::Discontinuous);
        assert(!h.offer(changed,0,0,0));
        assert(h.take(changed,-27)==Take::Missing);
    }
    printf("Z chord handoff loops=10000 advances=%llu injection_probes=%llu stale_rejected=%llu PASS\n",
        (unsigned long long)advances,(unsigned long long)probes,(unsigned long long)stale);
}
