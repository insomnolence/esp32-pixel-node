#ifndef GENERIC_PACKET_H_
#define GENERIC_PACKET_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Generic packet container - constrained by BLE MTU limits
class GenericPacket {
public:
    // BLE MTU constraint - maximum usable payload
    static const size_t MAX_PACKET_SIZE = 244;
    
    GenericPacket();
    GenericPacket(const uint8_t* data, size_t length);
    GenericPacket(const GenericPacket& other);
    GenericPacket& operator=(const GenericPacket& other);
    ~GenericPacket();
    
    // Data access
    const uint8_t* getData() const;
    uint8_t* getMutableData();
    size_t getLength() const;
    
    // Data manipulation
    bool setData(const uint8_t* data, size_t length);
    void clear();
    bool isValid() const;
    
    // Convenience methods for typed packets
    template<typename T>
    bool setPacket(const T& packet) {
        static_assert(sizeof(T) <= MAX_PACKET_SIZE, "Packet type too large for BLE transmission");
        return setData(reinterpret_cast<const uint8_t*>(&packet), sizeof(T));
    }
    
    template<typename T>
    bool getPacket(T& packet) const {
        if (data_length != sizeof(T)) {
            return false;
        }
        memcpy(&packet, packet_data, sizeof(T));
        return true;
    }
    
    // Check if packet could be a specific type
    template<typename T>
    bool couldBePacketType() const {
        return data_length == sizeof(T);
    }

private:
    uint8_t packet_data[MAX_PACKET_SIZE];
    size_t data_length;
};

// Abstract packet processor interface
class PacketProcessor {
public:
    virtual ~PacketProcessor() = default;
    
    // Process any packet - implementation decides how to decode
    virtual bool processPacket(const GenericPacket& packet) = 0;
    
    // Optional: Get packet format version/type info
    virtual uint32_t getSupportedPacketVersion() const { return 1; }
    virtual const char* getProcessorName() const = 0;
    
    // Optional: Validate packet before processing
    virtual bool canProcessPacket(const GenericPacket& packet) const { return true; }
};

#endif // GENERIC_PACKET_H_