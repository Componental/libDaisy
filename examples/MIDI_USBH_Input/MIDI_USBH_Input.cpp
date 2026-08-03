/** Example of setting reading MIDI Input via USB Host
 *  
 * 
 *  This requires a USB-A connector
 * 
 *  This example will also log incoming messages to the serial port for general MIDI troubleshooting
 */
#include "daisy_seed.h"
#include "usbh_midi.h"
#include "src/dev/mcp23x17.h"
#include "src/per/i2c.h"

/** This prevents us from having to type "daisy::" in front of a lot of things. */
using namespace daisy;

/** Global Hardware access */
DaisySeed       hw;
MidiUsbHandler midi;
USBHostHandle usbHost;
daisy::Mcp23017 expander;
daisy::I2CHandle i2c;

/** FIFO to hold messages as we're ready to print them */
FIFO<MidiEvent, 128> event_log;

void USBH_Connect(void* data)
{
    hw.PrintLine("device connected");
}

void USBH_Disconnect(void* data)
{
    hw.PrintLine("device disconnected");
}

void USBH_ClassActive(void* data)
{
    if(usbHost.IsActiveClass(USBH_MIDI_CLASS))
    {
        hw.PrintLine("MIDI device class active");
        MidiUsbHandler::Config midi_config;
        midi_config.transport_config.periph = MidiUsbTransport::Config::Periph::HOST;
        midi.Init(midi_config);
        midi.StartReceive();
    }
}

void USBH_Error(void* data)
{
    hw.PrintLine("USB device error");
}

int main(void)
{
    /** Initialize our hardware */
    hw.Init();

    hw.StartLog(false);

    daisy::I2CHandle::Config i2c_config;
    i2c_config.speed = daisy::I2CHandle::Config::Speed::I2C_400KHZ;
    i2c_config.mode = daisy::I2CHandle::Config::Mode::I2C_MASTER;
    i2c_config.periph = daisy::I2CHandle::Config::Peripheral::I2C_1;
    i2c_config.pin_config.scl = {daisy::GPIOPort::PORTB, 8};
    i2c_config.pin_config.sda = {daisy::GPIOPort::PORTB, 9};

    if (i2c.Init(i2c_config) != daisy::I2CHandle::Result::OK)
    {
        hw.PrintLine("I2C Init failed");
    }

    // i2c Expander Init
    daisy::Mcp23017::Config expander_cfg;
    expander_cfg.transport_config.i2c_address = 0x20;
    expander_cfg.transport_config.i2c_config = i2c.GetConfig();
    expander_cfg.transport_config.i2c_config.speed = daisy::I2CHandle::Config::Speed::I2C_400KHZ;
    expander.Init(expander_cfg);

    expander.PinMode((uint8_t)2, daisy::MCPMode::OUTPUT, false);

    expander.WritePin(2, 1);


    daisy::System::Delay(10);
    expander.PortMode(daisy::MCPPort::A, 0x00, 0xFF, 0x00);
    expander.PortMode(daisy::MCPPort::B, 0xFF, 0xFF, 0x00);

    hw.PrintLine("MIDI USB Host start");

    /** Configure USB host */
    USBHostHandle::Config usbhConfig;
    usbhConfig.connect_callback = USBH_Connect,
    usbhConfig.disconnect_callback = USBH_Disconnect,
    usbhConfig.class_active_callback = USBH_ClassActive,
    usbhConfig.error_callback = USBH_Error,
    usbHost.Init(usbhConfig);

    usbHost.RegisterClass(USBH_MIDI_CLASS);

    uint32_t now      = System::GetNow();
    uint32_t log_time = System::GetNow();
    uint32_t blink_time = 0;
    bool ledState = false;

    hw.PrintLine("MIDI USB Host initialized");

    /** Infinite Loop */
    while(1)
    {
        now = System::GetNow();

        if (now > blink_time)
        {
            hw.SetLed(ledState);
            ledState = !ledState;
            if (usbHost.GetPresent())
                blink_time = now + 400;
            else
                blink_time = now + 80;
        }
        /** Run USB host process */
        usbHost.Process();

        if(usbHost.IsActiveClass(USBH_MIDI_CLASS) && midi.RxActive())
        {
            /** Process MIDI in the background */
            midi.Listen();

            /** Loop through any MIDI Events */
            while(midi.HasEvents())
            {
                MidiEvent msg = midi.PopEvent();

                /** Handle messages as they come in 
                 *  See DaisyExamples for some examples of this
                 */
                switch(msg.type)
                {
                    case NoteOn:
                        // Do something on Note On events
                        {
                            uint8_t bytes[3] = {0x90, 0x00, 0x00};
                            bytes[1] = msg.data[0];
                            bytes[2] = msg.data[1];
                            midi.SendMessage(bytes, 3);
                        }
                        break;
                    default: break;
                }

                /** Regardless of message, let's add the message data to our queue to output */
                event_log.PushBack(msg);
            }

            /** Now separately, every 5ms we'll print the top message in our queue if there is one */
            if(now - log_time > 5)
            {
                log_time = now;
                if(!event_log.IsEmpty())
                {
                    auto msg = event_log.PopFront();
                    char outstr[128];
                    const char* type_str = MidiEvent::GetTypeAsString(msg);
                    sprintf(outstr,
                            "time:\t%ld\ttype: %s\tChannel:  %d\tData MSB: "
                            "%d\tData LSB: %d\n",
                            now,
                            type_str,
                            msg.channel,
                            msg.data[0],
                            msg.data[1]);
                    hw.PrintLine(outstr);
                }
            }
        }
    }
}
