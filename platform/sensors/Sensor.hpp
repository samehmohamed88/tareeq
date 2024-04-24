#pragma once

#include "platform/io/Status.hpp"

#include <any>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

namespace platform::sensors {

class Sensor
{
private:
    struct SensorConcept
    {
        virtual ~SensorConcept() = default;

        virtual void do_publishSensorData() const = 0;

        virtual std::unique_ptr<SensorConcept> clone() const = 0;
    };

    template<typename SensorT, typename ReadStrategy, typename DataPublisher>
    struct ExtendedSensorModel : public SensorConcept
    {
    public:
        explicit ExtendedSensorModel(SensorT sensor, ReadStrategy readStrategy, DataPublisher publisher)
            : sensor_{std::move(sensor)}
            , reader_{std::move(readStrategy)}
            , publisher_{std::move(publisher)}
        {}

        void do_publishSensorData() const override
        {
            auto [status, data] = reader_();
            if (status.isSuccess()) {
                publisher_();
            }
        }

        std::unique_ptr<SensorConcept> clone() const override { return std::make_unique<ExtendedSensorModel>(*this); }

    private:
        SensorT sensor_;
        ReadStrategy reader_;
        DataPublisher publisher_;
    };

    friend void publishSensorData(Sensor const& sensor) { sensor.pimpl_->do_publishSensorData(); }

    std::unique_ptr<SensorConcept> pimpl_;

public:
    template<typename SensorT, typename DataReader, typename DataPublisher>
    Sensor(SensorT sensor, DataReader reader, DataPublisher publisher)
        : pimpl_{std::make_unique<ExtendedSensorModel<SensorT, DataReader, DataPublisher>>(std::move(sensor),
                                                                                           std::move(reader),
                                                                                           std::move(publisher))}
    {}

    Sensor(Sensor const& other)
        : pimpl_(other.pimpl_->clone())
    {}

    Sensor& operator=(Sensor const other)
    {
        // Copy and swap idiom
        other.pimpl_->clone().swap(pimpl_);
        return *this;
    }

    Sensor& operator=(Sensor&& other) noexcept
    {
        pimpl_.swap(other.pimpl_);
        return *this;
    }
};

} // namespace platform::sensors
