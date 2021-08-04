#ifndef CHARGING_STATION
#define CHARGING_STATION

#define ESC_PIN 23
#define CHARGING_PIN 22

const uint8_t POWER_SUPPLY_STATUS_CHARGING = 1;
const uint8_t POWER_SUPPLY_STATUS_NOT_CHARGING = 3;
const uint8_t POWER_SUPPLY_TECHNOLOGY_NICD = 5;

class ChargingStation {
    public:
        ChargingStation();
        void setEsc(bool state);
        bool isEscEnabled();
        uint8_t areBatteriesCharging();

    private:
        bool ESC;
        bool batteriesCharging;
};

#endif // CHARGING_STATION