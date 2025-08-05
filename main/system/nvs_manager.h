#ifndef NVS_MANAGER_H_
#define NVS_MANAGER_H_

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

#endif // NVS_MANAGER_H_
