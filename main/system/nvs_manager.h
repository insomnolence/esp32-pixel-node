#pragma once

#include "esp_err.h"

class NvsManager {
public:
    NvsManager();
    ~NvsManager();

    esp_err_t init();

private:
    static const char* TAG;
};

extern NvsManager nvsManager;
