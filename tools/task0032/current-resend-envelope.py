#!/usr/bin/env python3
"""Identify replayable wire envelopes without the previous host oracle.

Candidate header consumed by the next receiver stage. It never infers host
history from an unnumbered/unchecked frame, a hardware-loss flag or a run of
zeros. Binary integrity is supplied only by the actual Fletcher parser after
shape capture, before its text mutation can destroy evidence. No shipping
source modifications or protocol release claim.
"""
from pathlib import Path
import subprocess
import tempfile

HEADER=r'''
#ifndef RF_RESEND_ENVELOPE_H
#define RF_RESEND_ENVELOPE_H
#include <stdint.h>

struct ResendEnvelope {
    enum class Mode : uint8_t { None, Ascii, Binary };
    uint32_t line=0;
    Mode mode=Mode::None;
    bool verified=false;

    bool exact(uint32_t expected) const {
        return mode==Mode::Ascii ? line==expected :
            mode==Mode::Binary && uint16_t(line)==uint16_t(expected);
    }
    // A verified, forward sequence gap can be recovered in an established
    // checked stream. A damaged frame can only identify the expected line.
    // Neither rule asserts that the host actually retains the history.
    bool mayRequest(Mode established,uint32_t expected) const {
        if(mode==Mode::None || mode!=established)return false;
        if(exact(expected))return !verified;
        if(!verified)return false;
        if(mode==Mode::Ascii)return line>expected;
        const uint16_t distance=uint16_t(line-expected);
        return distance!=0 && distance<0x8000;
    }
    bool mayAdmit(Mode established,uint32_t expected) const {
        return verified && mode==established && exact(expected);
    }

    static bool space(uint8_t c){return c==' ' || c=='\t';}
    static bool digit(uint8_t c){return c>='0' && c<='9';}
    static bool command(uint8_t c){
        c|=0x20;
        return c=='g'||c=='m'||c=='t'||c=='f';
    }

    // Capture before the permissive ASCII parser can narrow a checksum or
    // overwrite '*' in borrowed panel text. Length excludes the delimiter.
    static ResendEnvelope ascii(const uint8_t* bytes,uint8_t length) {
        ResendEnvelope out;
        uint8_t p=0;
        while(p<length && space(bytes[p]))++p;
        if(p==length || (bytes[p]!='N' && bytes[p]!='n'))return out;
        ++p;
        while(p<length && space(bytes[p]))++p;
        if(p==length || !digit(bytes[p]))return out;
        uint32_t line=0;
        do {
            const uint8_t d=bytes[p++]-'0';
            if(line>(uint32_t(INT32_MAX)-d)/10)return out;
            line=line*10+d;
        }while(p<length && digit(bytes[p]));
        while(p<length && space(bytes[p]))++p;
        // A second N before the command, or bare N without a command,
        // is not an identifiable envelope.
        if(p==length || !command(bytes[p]))return out;
        uint8_t sum=0;
        uint8_t star=0;
        for(;star<length && bytes[star]!='*';++star){
            const uint8_t c=bytes[star];
            if(c==0 || c=='\r'||c=='\n'||c>=0x80)return out;
            sum^=c;
        }
        if(star<=p || star==length)return out;
        p=star+1;
        while(p<length && space(bytes[p]))++p;
        if(p==length || !digit(bytes[p]))return out;
        uint16_t given=0;
        do {
            given=uint16_t(given*10+bytes[p++]-'0');
            if(given>255)return out;
        }while(p<length && digit(bytes[p]));
        while(p<length && space(bytes[p]))++p;
        if(p!=length)return out;
        out.line=line;out.mode=Mode::Ascii;out.verified=given==sum;
        return out;
    }

    // Validate supported frame shape and its raw N location. No unaligned
    // reads, no uint8 size wrap, no trust in the 31-zero host convention.
    // Do NOT recompute Fletcher on the healthy path: the real parser
    // already performs it. Call binaryParsed() with that actual result.
    static ResendEnvelope binary(const uint8_t* bytes,uint8_t length) {
        ResendEnvelope out;
        if(length<6 || length>96)return out;
        const uint16_t flags=uint16_t(bytes[0])|(uint16_t(bytes[1])<<8);
        if((flags&0x81)!=0x81 || (flags&0x6000) || !(flags&0x0306))return out;
        const bool v2=(flags&0x1000)!=0;
        const bool text=(flags&0x8000)!=0;
        uint8_t n=2;
        uint16_t size=6; // header, N, Fletcher
        if(v2){
            if(length<8)return out;
            const uint16_t ext=uint16_t(bytes[2])|(uint16_t(bytes[3])<<8);
            if(ext&0xf800)return out;
            n=4;size+=2;
            for(uint16_t bit=1;bit<=0x400;bit<<=1)
                if(ext&bit)size+=4;
            if(text){
                if(length<9 || bytes[4]>=80)return out;
                size+=uint16_t(1)+bytes[4];++n;
            }
        }else if(text)size+=16;
        if(flags&2)size+=v2?2:1;
        if(flags&4)size+=v2?2:1;
        if(flags&0x0008)size+=4;
        if(flags&0x0010)size+=4;
        if(flags&0x0020)size+=4;
        if(flags&0x0040)size+=4;
        if(flags&0x0100)size+=4;
        if(flags&0x0200)size+=1;
        if(flags&0x0400)size+=4;
        if(flags&0x0800)size+=4;
        if(size!=length || n+2>length-2)return out;
        out.mode=Mode::Binary;
        out.line=uint16_t(bytes[n])|(uint16_t(bytes[n+1])<<8);
        return out;
    }
    void binaryParsed(bool accepted) {
        if(mode==Mode::Binary)verified=accepted;
    }
};
#endif
'''

TEST=r'''
#include "ResendEnvelope.h"
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>
using E=ResendEnvelope;
using M=E::Mode;
static std::string checked(const std::string& line) {
    uint8_t checksum=0;for(unsigned char c:line)checksum^=c;
    return line+"*"+std::to_string(checksum);
}
static E ascii(const std::string& data) {
    assert(data.size()<96);
    return E::ascii(reinterpret_cast<const uint8_t*>(data.data()),uint8_t(data.size()));
}
static E binary(const std::vector<uint8_t>& data){
    return E::binary(data.data(),uint8_t(data.size()));
}
int main(){
    for(uint32_t n:{0u,1u,65535u,65536u,2147483647u}){
        auto raw=checked("N"+std::to_string(n)+" G1 X1");
        auto good=ascii(raw);
        assert(good.verified && good.mode==M::Ascii && good.line==n);
        assert(good.mayAdmit(M::Ascii,n) && !good.mayRequest(M::Ascii,n));
        assert(!good.mayAdmit(M::Binary,n) && !good.mayAdmit(M::Ascii,n+65536u));
        raw[raw.find("X1")+1]='2'; // preserve N, shape and checksum spelling
        auto bad=ascii(raw);
        assert(!bad.verified && bad.mode==M::Ascii && bad.line==n);
        assert(bad.mayRequest(M::Ascii,n) && !bad.mayAdmit(M::Ascii,n));
        assert(!bad.mayRequest(M::None,n) && !bad.mayRequest(M::Ascii,n+1));
    }
    for(const auto& raw:{"G1 X1","N1 G1 X1","N1 N2 G1*0","N1*0",
        "N2147483648 G1*0","N-1 G1*0","N1 G1*256","N1 G1*-1",
        "N1 G1*1 M112","N1 G1*1*2","N1x G1*0","N1 M105*9999999999"}){
        auto e=ascii(raw);assert(e.mode==M::None && !e.mayRequest(M::Ascii,1));
    }
    assert(ascii(checked("n 2 m117 retained N99 text")).mayAdmit(M::Ascii,2));
    assert(ascii(checked("N3 G1")).mayRequest(M::Ascii,2));
    assert(!ascii(checked("N1 G1")).mayRequest(M::Ascii,2));
    // V1, V2, V2 text (length byte precedes N), binary modulo wrap.
    auto e=binary({0x85,0,0xff,0xff,1,0,0});
    assert(e.mode==M::Binary && e.line==65535 && !e.verified);
    assert(e.mayRequest(M::Binary,65535));e.binaryParsed(true);
    assert(e.mayAdmit(M::Binary,65535) && !e.mayRequest(M::Binary,65535));
    e=binary({0x85,0x10,0,0,0,0,1,0,0,0});e.binaryParsed(true);
    assert(e.mayAdmit(M::Binary,65536) && e.mayRequest(M::Binary,65535));
    e=binary({0x83,0x90,0,0,1,2,0,117,0,'X',0,0});
    assert(e.mode==M::Binary && e.line==2 && !e.verified);
    e.binaryParsed(false);assert(e.mayRequest(M::Binary,2));
    for(auto raw:std::vector<std::vector<uint8_t>>{
        {0x84,0,1,0,0},{0x85,0,1,0,0,0},{0x85,0,1,0,1,0,0,0},
        {0x85,0x30,0,0,1,0,1,0,0,0},{0x85,0x10,0,8,1,0,1,0,0,0},
        {0x83,0x90,0,0,80,2,0,117,0,0,0},{0x81,0,1,0,0,0}}){
        auto invalid=binary(raw);assert(invalid.mode==M::None);
        invalid.binaryParsed(true);assert(!invalid.mayAdmit(M::Binary,1));
    }
    std::puts("Wire evidence: exact N, malformed/unchecked rejection, damaged expected frame, forward gap, binary shape and wrap PASS");
}
'''

def main():
    with tempfile.TemporaryDirectory(prefix="task0032-wire-evidence-") as directory:
        root=Path(directory)
        (root/"ResendEnvelope.h").write_text(HEADER)
        (root/"test.cpp").write_text(TEST)
        for flags in (["-O2"],["-O1","-fsanitize=undefined","-fno-sanitize-recover=all"]):
            subprocess.run(["c++","-std=c++11","-Wall","-Wextra","-Werror",*flags,
                str(root/"test.cpp"),"-o",str(root/"test")],check=True)
            subprocess.run([str(root/"test")],check=True,timeout=10)

if __name__=="__main__":
    main()
