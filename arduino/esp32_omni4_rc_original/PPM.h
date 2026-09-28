#pragma once

#include <Arduino.h>

namespace ppm {

constexpr uint8_t MAX_CHANNELS = 6;

volatile uint16_t ppmChannels[MAX_CHANNELS];
volatile uint8_t currentChannel = 0;
volatile uint8_t receivedChannels = 0;
volatile uint32_t lastEdgeTime = 0;
volatile bool newPPMFrame = false;

portMUX_TYPE ppmMux = portMUX_INITIALIZER_UNLOCKED;


// ============================================================
// Interrupción PPM
// ============================================================
void IRAM_ATTR ppmISR()
{
    uint32_t now = micros();
    uint32_t interval = now - lastEdgeTime;
    lastEdgeTime = now;

    // Gap largo = inicio de un nuevo frame PPM
    if (interval > 3000)
    {
        if (currentChannel > 0)
        {
            receivedChannels = currentChannel;
            newPPMFrame = true;
        }

        currentChannel = 0;
        return;
    }

    // Intervalo normal de un canal
    if (interval >= 800 && interval <= 2200)
    {
        if (currentChannel < MAX_CHANNELS)
        {
            ppmChannels[currentChannel] = interval;
            currentChannel++;
        }
    }
}


// ============================================================
// Inicialización
// ============================================================
inline void comenzar(uint8_t pin)
{
    // Inicializar variables
    for (uint8_t i = 0; i < MAX_CHANNELS; i++)
    {
        ppmChannels[i] = 1500;
    }

    currentChannel = 0;
    receivedChannels = 0;
    newPPMFrame = false;
    lastEdgeTime = micros();

    pinMode(pin, INPUT);

    attachInterrupt(
        digitalPinToInterrupt(pin),
        ppmISR,
        RISING
    );
}


// ============================================================
// Leer todos los canales
//
// Devuelve true si hay un frame nuevo.
// Los valores se copian a "salida".
// ============================================================
inline bool leer(uint16_t salida[MAX_CHANNELS], uint8_t &cantidad)
{
    bool hayFrame;

    portENTER_CRITICAL(&ppmMux);

    hayFrame = newPPMFrame;
    cantidad = receivedChannels;

    if (cantidad > MAX_CHANNELS)
        cantidad = MAX_CHANNELS;

    if (hayFrame)
    {
        for (uint8_t i = 0; i < cantidad; i++)
        {
            salida[i] = ppmChannels[i];
        }

        newPPMFrame = false;
    }

    portEXIT_CRITICAL(&ppmMux);

    return hayFrame;
}


// ============================================================
// Validar señal PPM
// ============================================================
inline bool valido(uint8_t cantidad)
{
    return cantidad >= 3;
}

} // namespace ppm
