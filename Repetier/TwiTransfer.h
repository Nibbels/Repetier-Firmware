#ifndef TWI_TRANSFER_H
#define TWI_TRANSFER_H

#include <stdint.h>

// One small hardware I2C transaction. The foreground owns the bus and advances
// each completed byte. Callers may do other work while a byte is in flight.
class TwiTransfer {
public:
    TwiTransfer() = default;
    TwiTransfer(const TwiTransfer&) = delete;
    TwiTransfer& operator=(const TwiTransfer&) = delete;

    static void beginBus();
    static void finishActive();

    bool write(uint8_t address, const uint8_t* bytes, uint8_t length, uint32_t now);
    bool read(uint8_t address, uint8_t length, uint32_t now);
    void service(uint32_t now);
    bool wait();

    bool busy() const { return active_ == this; }
    bool succeeded() const { return state_ == State::Done; }
    uint8_t byteAt(uint8_t index) const { return bytes_[index]; }

private:
    enum class State : uint8_t { Error, Moving, Stop, StopAfterError, Done };
    static TwiTransfer* active_;

    bool start(uint8_t addressByte, uint8_t length, uint32_t now);
    void stop(bool success);
    void finish(bool success);
    void receiveByte();

    uint32_t lastEventAt_ = 0;
    uint8_t bytes_[3]{};
    uint8_t addressByte_ = 0;
    uint8_t length_ = 0;
    uint8_t position_ = 0;
    State state_ = State::Error;
};

#endif
