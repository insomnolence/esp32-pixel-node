#include "generic_packet.h"
#include "esp_log.h"

static const char* TAG = "GenericPacket";

GenericPacket::GenericPacket() : data_length(0) {
    memset(packet_data, 0, MAX_PACKET_SIZE);
}

GenericPacket::GenericPacket(const uint8_t* data, size_t length) : data_length(0) {
    memset(packet_data, 0, MAX_PACKET_SIZE);
    setData(data, length);
}

GenericPacket::GenericPacket(const GenericPacket& other) : data_length(other.data_length) {
    memcpy(packet_data, other.packet_data, data_length);
    // Zero out the rest
    if (data_length < MAX_PACKET_SIZE) {
        memset(packet_data + data_length, 0, MAX_PACKET_SIZE - data_length);
    }
}

GenericPacket& GenericPacket::operator=(const GenericPacket& other) {
    if (this != &other) {
        data_length = other.data_length;
        memcpy(packet_data, other.packet_data, data_length);
        // Zero out the rest
        if (data_length < MAX_PACKET_SIZE) {
            memset(packet_data + data_length, 0, MAX_PACKET_SIZE - data_length);
        }
    }
    return *this;
}

GenericPacket::~GenericPacket() {
    // Nothing to clean up - using stack buffer
}

const uint8_t* GenericPacket::getData() const {
    return packet_data;
}

uint8_t* GenericPacket::getMutableData() {
    return packet_data;
}

size_t GenericPacket::getLength() const {
    return data_length;
}

bool GenericPacket::setData(const uint8_t* data, size_t length) {
    if (length > MAX_PACKET_SIZE) {
        ESP_LOGE(TAG, "Packet length %zu exceeds maximum %zu", length, MAX_PACKET_SIZE);
        return false;
    }
    
    if (data == nullptr && length > 0) {
        ESP_LOGE(TAG, "Null data pointer with non-zero length");
        return false;
    }
    
    data_length = length;
    if (length > 0) {
        memcpy(packet_data, data, length);
    }
    
    // Zero out unused portion
    if (length < MAX_PACKET_SIZE) {
        memset(packet_data + length, 0, MAX_PACKET_SIZE - length);
    }
    
    return true;
}

void GenericPacket::clear() {
    data_length = 0;
    memset(packet_data, 0, MAX_PACKET_SIZE);
}

bool GenericPacket::isValid() const {
    return data_length > 0 && data_length <= MAX_PACKET_SIZE;
}