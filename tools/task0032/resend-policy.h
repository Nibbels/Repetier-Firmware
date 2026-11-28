#ifndef TASK0032_RESEND_POLICY_H
#define TASK0032_RESEND_POLICY_H

#include <stdint.h>

// Task-private executable candidate, not linked into shipping firmware.
// Foreground only; frame validation, committed sequence ownership, ISR
// quarantine and the terminal actuator transition remain outside this class.
class ResendPolicy {
public:
    enum class Phase : uint8_t { Healthy, Pending, Sending, Waiting, Fatal };
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
    // deferredHandoff: the TX marker is still pending when publish succeeds.
    // The historical immediate-publish fixture uses the default false;
    // a real UART must select true and observe the marker before parsing.
    template<class Publish>
    void service(uint32_t now, bool ready, bool ackOrdered, Publish publish,
                 bool deferredHandoff=false) {
        if (phase_ == Phase::Healthy || phase_ == Phase::Fatal) return;
        if (phase_ == Phase::Sending) {
            // Absolute publication-to-handoff deadline independent of
            // receiver readiness or trickling TX progress.
            if (uint32_t(now-last_) >= txStallMs) fatal(Cause::TxStall);
            return;
        }
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
        phase_ = deferredHandoff ? Phase::Sending : Phase::Waiting;
        elapsed_ = 0;
    }

    // Poll in foreground before parsing a possibly already buffered reply.
    // The marker is an ISR UDR handoff lower bound, not a TXC/host receipt.
    // Start the response window at observation (at most one bounded visit
    // later than actual handoff); do not charge publication delay to host.
    void observeHandoff(uint32_t now, bool ready) {
        if (phase_ != Phase::Sending) return;
        phase_ = Phase::Waiting;
        restartClock(now);
        wasReady_ = ready;
    }

    // TX intent is independently bounded even when receive storage is
    // blocked, or when no RX fault has occurred yet. Preserve first cause.
    void failTx() {
        if (phase_ != Phase::Fatal) fatal(Cause::TxStall);
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

// Bound an unpublished ACK independently of the resend/receiver-ready
// clock. A later fault or a full command queue must not reset its age.
// This is an intent publication deadline, NOT a TX-wire-progress proof.
// The caller must stage at the actual admission instant, attempt publication
// before polling, and keep the first terminal cause. No new buffer.
class AckPublicationDeadline {
public:
    void staged(uint32_t now) { since_ = now; }
    bool expired(bool pending, uint32_t now) const {
        return pending && uint32_t(now - since_) >= ResendPolicy::txStallMs;
    }
private:
    uint32_t since_ = 0;
};

// Actual software-ring retirement progress, not TXC or receipt by host.
// The UART producer provides a 16-bit monotonically incrementing byte
// count minus current occupancy. Foreground-only sampling. At fixed
// 250000 baud 8N1, 65536 retirements require >2.62 s; the integration must
// establish a service interval below that, independently of this timer.
// No ISR counter or extra queue. An unchanged occupancy after refill
// does NOT imply no drain.
class TxRetirementDeadline {
public:
    bool poll(uint16_t retired, bool nonempty, uint32_t now) {
        if (!nonempty) { active_ = false; return false; }
        if (!active_ || retired != last_) {
            active_ = true;
            last_ = retired;
            since_ = now;
            return false;
        }
        return uint32_t(now - since_) >= ResendPolicy::txStallMs;
    }
private:
    uint32_t since_ = 0;
    uint16_t last_ = 0;
    bool active_ = false;
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
