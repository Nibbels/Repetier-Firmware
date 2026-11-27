#pragma once
#include <stdint.h>

// Historical counterfactual, NOT a shipping driver. Only normal DDRAM text.
// One foreground owner; the caller owns the desired row and its confirmed
// shadow. Zero is reserved for unknown shadow cells, not visible text.
// Boot/CGRAM/clear/re-init retain their exclusive synchronous transport until
// separately qualified; invalidate all shadow cells after such operations.
namespace Task0032Lcd {
enum class Result : uint8_t { Idle, Waiting, Address, Data };

class ByteService {
    uint32_t completedAt_ = 0;
    uint8_t nextAddress_ = 0xff;
    bool waiting_ = false;
public:
    // Preserve >=320 us under the existing 4 us micros quantization.
    // Requires a qualified monotonic timebase; no hardware claim here.
    static constexpr uint32_t minimumClockDeltaUs = 324;
    // Called ONLY after exclusive operations have completed their own
    // required execution delay. Do not call to escape an ordinary wait.
    void afterExclusiveOperation() {
        nextAddress_ = 0xff;
        waiting_ = false;
    }

    template<uint8_t Columns>
    static void invalidate(char (&shadow)[Columns]) {
        for(uint8_t i=0;i<Columns;++i) shadow[i]=0;
    }

    // sink.write(value, rs) emits one complete byte, without a busy delay.
    // sink.now() samples the clock AFTER the final enable edge.
    // Use the same monotonic modulo-2^32 microsecond clock for both.
    // At most one byte per visit, including cursor commands. A changed
    // desired row between visits supersedes unsent data.
    template<uint8_t Columns, class Sink>
    Result service(uint8_t base, const char (&desired)[Columns],
                   char (&shadow)[Columns], uint32_t now, Sink& sink) {
        static_assert(Columns==16 || Columns==20, "supported panel widths");
        uint8_t col=0;
        while(col<Columns && desired[col]==shadow[col]) ++col;
        if(col==Columns) return Result::Idle;
        // The normal caller must space-pad and reject embedded NUL;
        // boot CGRAM slot zero is deliberately outside this interface.
        if(waiting_ && uint32_t(now-completedAt_)<minimumClockDeltaUs) return Result::Waiting;
        const uint8_t address=base+col;
        Result result;
        if(nextAddress_!=address) {
            sink.write(uint8_t(0x80|address), false);
            nextAddress_=address;
            result=Result::Address;
        } else {
            sink.write(uint8_t(desired[col]), true);
            shadow[col]=desired[col];
            nextAddress_=col+1==Columns?0xff:uint8_t(address+1);
            result=Result::Data;
        }
        completedAt_=sink.now();
        waiting_=true;
        return result;
    }
};
}
