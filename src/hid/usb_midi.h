#pragma once
#ifndef __DSY_MIDIUSBTRANSPORT_H__
#define __DSY_MIDIUSBTRANSPORT_H__

#include "hid/usb.h"
#include "sys/system.h"
#include "util/ringbuffer.h"

namespace daisy
{
/** @brief USB Transport for MIDI
 *  @ingroup midi
 */
class MidiUsbTransport
{
  public:
    typedef void (*MidiRxParseCallback)(uint8_t* data,
                                        size_t   size,
                                        void*    context);

    struct Config
    {
        enum Periph
        {
            INTERNAL = 0,
            EXTERNAL,
            HOST
        };

        Periph periph;

        /**
         * When sending MIDI messages immediately back-to-back in user code,
         * sometimes the USB CDC driver is still "busy".
         *
         * This option configures the number of times to retry a Tx after
         * delaying for 100 microseconds (default = 3 retries).
         *
         * If you set this to zero, Tx will not retry so the attempt will block
         * for slightly less time, but transmit can fail if the Tx state is busy.
         */
        uint8_t tx_retry_count;

        Config() : periph(INTERNAL), tx_retry_count(3) {}
    };

    void Init(Config config);

    void StartRx(MidiRxParseCallback callback, void* context);
    bool RxActive();
    void FlushRx();
    void Tx(uint8_t* buffer, size_t size);
    bool IsTxBusy();

    /** Flow control. queue_free(context) returns how many events the MIDI
     *  handler's queue can still take. After each packet the transport leaves
     *  the OUT endpoint un-armed when fewer than a packet's worth of events
     *  fit: the host sees NAK and waits, and ResumeRx() re-arms once there is
     *  room. Without it the queue overflows silently. Ignored in HOST mode.
     *  MidiHandler calls this from StartReceive(). */
    void SetRxFlowControl(size_t (*queue_free)(void*), void* context);

    /** Re-arms reception if it was held for lack of queue room and there is
     *  room now. Returns true when reception was resumed. */
    bool ResumeRx();

    /** True while the OUT endpoint is held (the host is being NAKed). */
    bool RxHeld();

    class Impl;

    MidiUsbTransport() : pimpl_(nullptr) {}
    ~MidiUsbTransport() {}
    MidiUsbTransport(const MidiUsbTransport& other) = default;
    MidiUsbTransport& operator=(const MidiUsbTransport& other) = default;

  private:
    Impl* pimpl_;
};

} // namespace daisy

#endif // __DSY_MIDIUSBTRANSPORT_H__
