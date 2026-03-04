#include <string.h>
#include <cmath>
#include<vector>
#include "moteus_multiplex.h"
#include "moteus_protocol.h"

namespace mjbots 
{
    namespace power_distributor
    {
        enum DRegister : uint16_t
        {
            kState = 0x000,
            kFault = 0x001,
            kSwitchStatus = 0x002,
            kLockTime = 0x003,
            kBootTime = 0x004,
            kOutputVoltage = 0x010,
            kOutputCurrent = 0x011,
            kTemperature = 0x12,
            kEnergy = 0x013,

        };

        enum State
        {
            kPowerOff = 0,
            kPreCharging = 1,
            kPowerOn = 2,
            kFaultState = 3,
        };
        struct Query
        {
            struct Result 
            {
                State state = kPowerOff;
                u_int8_t fault = 0;
                bool switch_status = 0;
                double lock_time = 0;
                double boot_time = 0;
                double output_voltage = 0;
                double output_current = 0;
                double temperature = 0;
                double energy =0;
            };

            struct Format
            {
                moteus::Resolution state = moteus::Resolution::kInt8;
                moteus::Resolution fault = moteus::Resolution::kInt8;
                moteus::Resolution switch_status = moteus::Resolution::kIgnore;
                moteus::Resolution lock_time = moteus::Resolution::kIgnore;
                moteus::Resolution boot_time = moteus::Resolution::kIgnore;
                moteus::Resolution output_voltage = moteus::Resolution::kFloat;
                moteus::Resolution output_current = moteus::Resolution::kFloat;
                moteus::Resolution temperature = moteus::Resolution::kIgnore;
                moteus::Resolution energy = moteus::Resolution::kIgnore;

            };

            static uint8_t Make(moteus::WriteCanData* frame, const Format& format)
            {
                uint8_t reply_size = 0;
                {
                    const moteus::Resolution kResolutions[] = {
                        format.state,
                        format.fault,
                        format.switch_status,
                        format.lock_time,
                        format.boot_time
                    };
                    const uint16_t kResolutionSize = sizeof(kResolutions)/sizeof(*kResolutions);
                    moteus::WriteCombiner combiner( 
                        frame,0x10,DRegister::kState,
                        kResolutions,kResolutionSize
                    );

                    for(uint16_t i = 0; i< kResolutionSize; i++)
                        combiner.MaybeWrite();
                    reply_size += combiner.reply_size();
                }
                
                {
                     const moteus::Resolution kResolutions[] = {
                        format.output_voltage,
                        format.output_current,
                        format.temperature,
                        format.energy
                    };
                    const uint16_t kResolutionSize = sizeof(kResolutions)/sizeof(*kResolutions);
                    moteus::WriteCombiner combiner( 
                        frame,0x10,DRegister::kOutputVoltage,
                        kResolutions,kResolutionSize
                    );

                    for(uint16_t i = 0; i< kResolutionSize; i++)
                        combiner.MaybeWrite();
                    reply_size += combiner.reply_size();
                }
                return reply_size;
            };
            static Result Parse(const uint8_t* data, uint8_t size) 
            {
                moteus::MultiplexParser parser(data, size);

                return Parse(&parser);
            }
            static Result Parse(const moteus::CanData* frame) 
            {
                moteus::MultiplexParser parser(frame);

                return Parse(&parser);
            }
        
            static Result Parse(moteus::MultiplexParser* parser) 
            {
                Result result;

                while (true) 
                {
                    const auto current = parser->next();
                    if (current.done) { return result; }

                    const auto res = current.resolution;
                    switch (static_cast<DRegister>(current.value))
                    {
                        case DRegister::kState:
                        {
                            result.state = static_cast<State>(parser->ReadInt(res));
                            break;
                        }
                        case  DRegister::kFault:
                        {
                            result.fault = parser->ReadInt(res);
                            break;
                        }
                        case DRegister::kSwitchStatus:
                        {
                            result.switch_status = static_cast<bool>(parser->ReadInt(res));
                            break;
                        }
                        case DRegister::kLockTime:
                        {
                            result.lock_time = parser->ReadInt(res);
                            break;
                        }
                        case DRegister::kBootTime:
                        {
                            result.boot_time = parser->ReadInt(res);
                            break;
                        }
                        case DRegister::kOutputVoltage:
                        {
                            result.output_voltage = parser->ReadVoltage(res);
                            break;
                        }
                        case DRegister::kOutputCurrent:
                        {
                            result.output_current = parser->ReadCurrent(res);
                            break;
                        }
                        case DRegister::kTemperature:
                        {
                            result.temperature = parser->ReadTemperature(res);
                            break;
                        }
                        case DRegister::kEnergy:
                        {
                            result.energy = parser->ReadEnergy(res);
                            break;
                        }

                    }
                }

            }
        };
        struct StateCommand
        {
            struct Command
            {
                int state = 2;
            };
            struct Format
            {
                mjbots::moteus::Resolution state = mjbots::moteus::Resolution::kInt8;
            };
            static uint8_t Make(moteus::WriteCanData* frame,
                                const Command& command,
                                const Format& format
                            )
            {
                frame->Write<int8_t>(moteus::Multiplex::kWriteInt8 | 0x01);
                frame->Write<int8_t>(power_distributor::DRegister::kState);
                frame->Write<int8_t>(command.state == 0 ? power_distributor::State::kPowerOff : power_distributor::State::kPowerOn );
                return 0;
            };
        };
    }
}