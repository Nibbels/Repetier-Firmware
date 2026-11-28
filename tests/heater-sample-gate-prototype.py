#!/usr/bin/env python3
"""Actual C++ sample reservation, abstract independent counter and wrap flag."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
code=r"""
#include "HeatSampleGate.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
#include <type_traits>
struct Clock {
    uint16_t count=0;
    bool wrap=false;
    uint16_t now()const{return count;}
    bool wrapped()const{return wrap;}
    void clearWrap(){wrap=false;}
    void advance(uint32_t n){if(n>=65536u-count)wrap=true;count=uint16_t(count+n);}
};
template<uint8_t N> void run(){
    using Gate=HeatSampleGate<Clock,N,6250>;
    using Ticket=typename Gate::Ticket;
    static_assert(!std::is_copy_constructible<Ticket>::value,"no alias");
    static_assert(!std::is_move_constructible<Ticket>::value,"no alias");
    unsigned long cases=0;
    for(uint32_t start=0;start<65536;++start){
        for(uint8_t c=0;c<N;++c){
            for(uint32_t age:{0u,1u,6249u,6250u,6251u,65536u,131073u}){
                Clock t;t.count=uint16_t(start);Gate gate(t);
                gate.publish(c,1234);
                {
                    Ticket p(gate,c);assert(p.captured()&&p.raw()==1234);
                    Ticket nested(gate,c);assert(!nested.captured()&&!nested.consume());
                    t.advance(age);
                    bool valid=age<=6250&&start+age<65536;
                    assert(p.consume()==valid);
                    assert(!p.consume());
                }
                {Ticket p(gate,c);assert(!p.captured());}
                gate.publish(c,1234); // identical reading, distinct publication
                {Ticket p(gate,c);assert(p.captured()&&p.consume());}
                ++cases;
            }
        }
    }
    // Publication and invalidation preserve reservation ownership.
    Clock t;Gate gate(t);
    for(unsigned n:{1u,2u,256u,65536u,131073u}){
        gate.publish(0,42);
        {
            Ticket old(gate,0);assert(old.captured());
            for(unsigned i=0;i<n;++i)gate.publish(0,42);
            assert(!old.consume());
            Ticket nested(gate,0);assert(!nested.captured());
        }
        {Ticket newer(gate,0);assert(newer.captured()&&newer.consume());}
    }
    gate.publish(0,43);
    {
        Ticket p(gate,0);gate.invalidateAll();gate.publish(0,44);
        assert(p.captured()&&!p.consume());
        Ticket nested(gate,0);assert(!nested.captured());
    }
    {Ticket p(gate,0);assert(p.captured()&&p.raw()==44&&p.consume());}
    // An unrelated channel may publish/consume independently.
    gate.publish(0,1);gate.publish(1,2);
    {Ticket a(gate,0);Ticket b(gate,1);gate.publish(1,3);
        assert(a.consume()&&!b.consume());}
    {Ticket b(gate,1);assert(b.raw()==3&&b.consume());}
    // After multiple wraps, even equal timestamp must not renew old data.
    gate.publish(0,7);
    {Ticket a(gate,0);t.advance(131072);gate.publish(1,8);
        assert(!a.consume());Ticket b(gate,1);assert(b.consume());}
    {Ticket invalid(gate,N);assert(!invalid.captured()&&!invalid.consume());}
    std::printf("channels=%u sample age/phase cases=%lu\n",N,cases);
}
int main(){run<2>();run<3>();run<4>();}
"""
with tempfile.TemporaryDirectory() as temp:
    cpp=Path(temp)/"sample.cpp"
    exe=Path(temp)/"sample"
    cpp.write_text(code)
    subprocess.run(["g++","-std=c++11","-Wall","-Wextra","-Werror","-O2",
                    "-I",str(root/"tools/task0034"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
