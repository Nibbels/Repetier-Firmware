#include "compare-phase-clock.h"
#include <cassert>
#include <cstdint>
#include <iostream>

namespace {
struct Timer {
    explicit Timer(uint16_t initialTop) : top(initialTop) {}
    ComparePhaseClock clock;
    uint64_t now = 0, lastEdge = 0;
    uint16_t phase = 0, top;
    bool pending = false, hadEdge = false;
    unsigned edges = 0, denied = 0, coalesced = 0, ambiguous = 0;

    // CTC counts 0..TOP. An unbuffered TOP write below TCNT can miss
    // the match and run through 0xffff first. The model's flag represents
    // the sticky match/reset event; a separate directed case samples the
    // counter on either side of that boundary.
    void advance(uint32_t ticks) {
        now += ticks;
        const uint32_t period = uint32_t(top) + 1;
        const uint32_t untilReset = phase <= top
            ? period - phase : 65536UL - phase + period;
        if (ticks < untilReset) {
            phase = uint16_t(uint32_t(phase) + ticks);
            return;
        }
        ticks -= untilReset;
        if (pending || ticks >= period)
            ++coalesced;
        pending = true;
        phase = uint16_t(ticks % period);
    }

    void enterVector() {
        assert(pending);
        pending = false;  // hardware clears the serviced compare flag
        clock.onCompare(top);
        check();
    }

    void writeTop(uint16_t value, uint32_t observationDelay = 0) {
        top = value;
        advance(observationDelay);
        clock.afterTopWrite(pending);
        if (pending)
            ++ambiguous;
        check();
    }

    void sampleEdge(uint32_t readDelay, uint32_t flagDelay) {
        lastEdge = now;
        hadEdge = true;
        ++edges;
        advance(readDelay);
        const uint16_t sampledPhase = phase;
        advance(flagDelay);
        clock.afterPhysicalEdge(sampledPhase, pending);
        if (pending)
            ++ambiguous;
        check();
    }

    void tryPulse(uint32_t minimum, uint32_t flagDelay,
                  uint32_t edgeDelay, uint32_t readDelay, uint32_t sampleFlagDelay) {
        const uint16_t sampledPhase = phase;
        advance(flagDelay);
        const bool admitted = clock.admits(sampledPhase, pending, minimum);
        if (admitted) {
            assert(!pending);
            advance(edgeDelay);
            assert(!hadEdge || now - lastEdge >= minimum);
            sampleEdge(readDelay, sampleFlagDelay);
        } else {
            ++denied;
        }
        check();
    }

    void check() const {
        const uint32_t age = clock.elapsedLowerBound(phase, pending);
        if (pending) {
            assert(age == 0);
            assert(!clock.admits(phase, true, 0));
        } else if (hadEdge) {
            assert(age <= now - lastEdge);
        }
    }

    uint32_t age() const { return clock.elapsedLowerBound(phase, pending); }
};

uint32_t random32(uint32_t& state) {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}
}

int main() {
    // Exact, no uncertainty: in-vector progress, one compare, TOP
    // changes before the match, and long-delay compare chunks.
    {
        Timer t(1023);
        t.advance(500);
        t.sampleEdge(0, 0);
        t.advance(300);
        assert(t.age() == 300);
        t.advance(224);
        t.enterVector();
        assert(t.age() == 524);
        t.writeTop(32768);
        t.advance(32769);
        t.enterVector();
        assert(t.age() == 33293);
        t.writeTop(99);
        t.advance(100);
        t.enterVector();
        assert(t.age() == 33393);
    }
    // A pending short compare must not be credited as a newly written,
    // much longer TOP. Reject admission while the flag remains pending.
    {
        Timer t(1023);
        t.advance(900);
        t.sampleEdge(0, 0);
        t.advance(124);
        assert(t.pending);
        t.writeTop(50000);
        t.tryPulse(512, 0, 0, 0, 0);
        assert(t.edges == 1);
        t.enterVector();
        assert(t.age() == 0);
        t.advance(1412);
        t.tryPulse(512, 0, 0, 0, 0);
        assert(t.edges == 2);
    }
    // Compare between the edge and the phase/flag sample: that pending
    // compare must not credit time prior to the newly emitted edge.
    {
        Timer t(1023);
        t.advance(1022);
        t.tryPulse(512, 0, 1, 0, 2);
        assert(t.pending && t.edges == 1);
        t.enterVector();
        assert(t.age() == 0);
        t.tryPulse(512, 0, 0, 0, 0);
        assert(t.edges == 1);
    }
    // Timer wrap without compare after an unbuffered TOP write below
    // TCNT. Conservative undercount is permitted; false elapsed is not.
    {
        Timer t(61000);
        t.advance(60000);
        t.sampleEdge(0, 0);
        t.writeTop(511);
        t.advance(5536);
        assert(t.phase == 0 && !t.pending);
        t.check();
        t.advance(512);
        t.enterVector();
        t.check();
    }
    // Saturation and full-width TOP must not wrap a 16-bit intermediate.
    {
        Timer t(65535);
        t.sampleEdge(0, 0);
        for (unsigned i = 0; i < 300; ++i) {
            t.advance(65536);
            t.enterVector();
        }
        assert(t.age() == uint32_t(ComparePhaseClock::maxAge));
    }

    uint64_t edges = 0, denied = 0, coalesced = 0, ambiguous = 0;
    constexpr unsigned seeds = 32, operations = 100000;
    constexpr uint16_t tops[] = {0, 1, 99, 511, 1388, 2999, 32768, 61000, 65535};
    for (unsigned seed = 1; seed <= seeds; ++seed) {
        uint32_t rng = 0x9e3779b9U * seed;
        auto r = [&]() { return random32(rng); };
        Timer t(2999);
        for (unsigned i = 0; i < operations; ++i) {
            switch (r() % 6) {
            case 0:
                t.advance(r() % 100000);
                break;
            case 1:
                if (t.pending) t.enterVector();
                else t.advance(r() % 5000);
                break;
            case 2:
                t.writeTop(tops[r() % (sizeof(tops)/sizeof(tops[0]))], r() % 8);
                break;
            case 3:
                t.writeTop(uint16_t(r()), r() % 8);
                break;
            default:
                // Variable physical interval and edge phase; deliberately
                // cover setup/hold sized intervals and slow Z correction.
                const uint32_t minimum = (r() & 1) ? 4 + r() % 3000 : 512 + r() % 60000;
                const uint32_t flagDelay = r() % 4;
                const uint32_t edgeDelay = r() % 20;
                const uint32_t readDelay = r() % 20;
                const uint32_t sampleFlagDelay = r() % 4;
                t.tryPulse(minimum, flagDelay, edgeDelay, readDelay, sampleFlagDelay);
                break;
            }
            t.check();
        }
        edges += t.edges;
        denied += t.denied;
        coalesced += t.coalesced;
        ambiguous += t.ambiguous;
    }
    assert(edges && denied && coalesced && ambiguous);
    std::cout << "compare-phase clock: " << uint64_t(seeds) * operations
              << " operations, " << edges << " admitted edges, " << denied
              << " deferred, " << coalesced << " coalescence cases, "
              << ambiguous << " ambiguous samples; no premature admission\n";
}
