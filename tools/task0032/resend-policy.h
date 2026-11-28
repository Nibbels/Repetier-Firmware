#ifndef TASK0032_RESEND_POLICY_H
#define TASK0032_RESEND_POLICY_H

#include <stdint.h>

// Task-private executable candidate, not linked into shipping firmware.
// Foreground only; frame validation, committed sequence ownership, ISR
// quarantine and the terminal actuator transition remain outside this class.
class ResendPolicy {
public:
    enum class Phase : uint8_t { Healthy, Pending, Waiting, Fatal };
    enum class Cause : uint8_t { None, Unreplayable, TxStall, Exhausted };
    static constexpr uint8_t maxRequests = 3;
    static constexpr uint32_t responseMs = 2000;
    static constexpr uint32_t txStallMs = 2000;

    Phase phase() const { return phase_; }
    Cause cause() const { return cause_; }
    uint8_t requests() const { return requests_; }
    uint32_t expected() const { return expected_; }

    // Only the foreground can start an episode. A subsequent loss cannot
    // restart its budget; the caller must separately invalidate frame/sync
    // evidence and quarantine the ring. expected is last COMMITTED + 1.
    void fault(uint32_t expected, bool replayable, uint32_t now) {
        if (phase_ == Phase::Fatal) return;
        if (!replayable) { fatal(Cause::Unreplayable); return; }
        if (phase_ != Phase::Healthy) return;
        expected_ = expected;
        requests_ = 0;
        phase_ = Phase::Pending;
        restartClock(now);
    }

    // ready: storage available (queue/borrowed text do not block receipt).
    // ackOrdered: older committed ACK has already been published.
    // publish is all-or-nothing TX-span publication. No nested service loop.
    // Time accrues according to the last observed readiness. Call on
    // readiness transitions and at bounded service intervals. Parse a
    // buffered verified candidate before testing its response deadline.
    template<class Publish>
    void service(uint32_t now, bool ready, bool ackOrdered, Publish publish) {
        if (phase_ == Phase::Healthy || phase_ == Phase::Fatal) return;
        const uint32_t delta = now - last_;
        last_ = now;
        if (wasReady_) {
            // Saturate: even a long service gap must not wrap elapsed time.
            const uint32_t room = UINT32_MAX - elapsed_;
            elapsed_ += delta > room ? room : delta;
        }
        wasReady_ = ready;
        if (phase_ == Phase::Waiting) {
            if (elapsed_ < responseMs) return;
            if (requests_ == maxRequests) {
                fatal(Cause::Exhausted);
                return;
            }
            phase_ = Phase::Pending;
            elapsed_ = 0;
        }
        if (elapsed_ >= txStallMs) {
            fatal(Cause::TxStall);
            return;
        }
        if (!ready || !ackOrdered || !publish(expected_)) return;
        ++requests_;
        phase_ = Phase::Waiting;
        elapsed_ = 0;
    }

    // Invoke ONLY after actual admission of the requested frame under the
    // existing final loss fence. Caller must prove exact sequence, checksum,
    // supported shape, replayable stream, fresh sync and no special bypass.
    // No zero-run or parsed-but-uncommitted frame can reset the budget.
    void committedExpected() {
        if (phase_ != Phase::Waiting) return;
        phase_ = Phase::Healthy;
        requests_ = 0;
        elapsed_ = 0;
        wasReady_ = false;
    }

private:
    void restartClock(uint32_t now) {
        last_ = now;
        elapsed_ = 0;
        wasReady_ = false;
    }
    void fatal(Cause cause) {
        phase_ = Phase::Fatal;
        cause_ = cause;
    }
    uint32_t expected_ = 0;
    uint32_t last_ = 0;
    uint32_t elapsed_ = 0;
    Phase phase_ = Phase::Healthy;
    Cause cause_ = Cause::None;
    uint8_t requests_ = 0;
    bool wasReady_ = false;
};

// Exact legacy wire reply (including the leading CRLF and control ok).
// The temporary span is never visible to the ISR until publication succeeds.
// A common formatter can replace this cold-path implementation at integration.
template<class Serial>
bool publishResend(uint32_t line) {
    char reply[25]; // CRLF + Resend: + 10 digits + CRLF + ok + CRLF
    char* out = reply + sizeof(reply);
    *--out = '\n'; *--out = '\r'; *--out = 'k'; *--out = 'o';
    *--out = '\n'; *--out = '\r';
    do {
        const uint32_t q = line / 10;
        *--out = char('0' + line - 10*q);
        line = q;
    } while (line);
    *--out = ':'; *--out = 'd'; *--out = 'n'; *--out = 'e';
    *--out = 's'; *--out = 'e'; *--out = 'R';
    *--out = '\n'; *--out = '\r';
    return Serial::tryWriteSpan(out, uint8_t(reply + sizeof(reply) - out));
}
#endif
