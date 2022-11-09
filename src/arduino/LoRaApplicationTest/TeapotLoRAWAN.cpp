#include "TeapotLoRaWAN.h"

namespace teapot {
namespace connection {

  Lorawan::Lorawan( const uint8_t* device_eui, uint8_t device_eui_size,
                    const uint8_t* app_eui, uint8_t app_eui_size,
                    const uint8_t* app_key, uint8_t app_key_size,
                    RAK_LORA_BAND band,
                    uint8_t rejoin_retry,
                    uint8_t send_request_retry ){
    if( device_eui == nullptr || device_eui_size != sizeof(this->device_eui) ||
        app_eui    == nullptr || app_eui_size    != sizeof(this->app_eui)    ||
        app_key    == nullptr || app_key_size    != sizeof(this->app_key) ){
      this->initialized = false;
      return;
    }

    memcpy(this->device_eui, device_eui, sizeof(this->device_eui));
    memcpy(this->app_eui, app_eui, sizeof(this->app_eui));
    memcpy(this->app_key, app_key, sizeof(this->app_key));

    this->rejoin_retry = rejoin_retry;
    this->send_request_retry = send_request_retry;
    this->band = band;
    this->initialized = true;
  }
  bool Lorawan::IsInitialized(){
    return this->initialized;
  }
  ReturnCode Lorawan::Connect()
  {
    if (!api.lorawan.appeui.set(this->app_eui, 8)) {
      return ReturnCode::kInvalidAppEui;
    }
    if (!api.lorawan.appkey.set(this->app_key, 16)) {
      return ReturnCode::kInvalidAppKey;
    }
    if (!api.lorawan.deui.set(this->device_eui, 8)) {
      return ReturnCode::kInvalidDeui;
    }
  
    if (!api.lorawan.band.set(this->band)) {
      return ReturnCode::kInvalidBand;
    }

    api.lorawan.deviceClass.set(RAK_LORA_CLASS_A);
    api.lorawan.njm.set(RAK_LORA_OTAA);
    
    if (!api.lorawan.join())  // Join to Gateway
    {
      return ReturnCode::kFailJoinNetwork;
    }
  
    /** Wait for Join success */
    while (api.lorawan.njs.get() == 0) {
      Serial.println("Wait for LoRaWAN join...");
      api.lorawan.join();
      delay(10000);
    }

    // enable ADR
    api.lorawan.adr.set(true);
    
    // enable send retry
    api.lorawan.rety.set(2);

    // enable send confirmation
    api.lorawan.cfm.set(1);
    
    api.lorawan.registerRecvCallback(ReceiveCb);
    api.lorawan.registerJoinCallback(JoinCb);
    api.lorawan.registerSendCallback(SendCb);
    
    return ReturnCode::kOk;
  }

  ReturnCode Lorawan::Send(CayenneLPP& payload)
  {
    return Send((uint8_t *) payload.getBuffer(), payload.getSize());
  }

  ReturnCode Lorawan::Send(uint8_t* payload, size_t size)
  {
    int retry = this->rejoin_retry;
    
    // If not part of a network, do rejoin
    while (api.lorawan.njs.get() == 0) {
      Serial.println("wait for LoRaWAN join...");
      api.lorawan.join();
      api.system.sleep.all(10000);
      if (retry-- <= 0) {
        break;
      }    
    }
  
    if(retry > 0 ){
      /** Send the data package */
      uint8_t try_send_request = this->send_request_retry;
      while( !api.lorawan.send(size, payload, 1) && try_send_request--) {
       Serial.println("retry sending payload");
       delay(1000);
      }
      
      if (!try_send_request){  
        return ReturnCode::kFailSendPayload;
      } else { 
        return ReturnCode::kOk;
      }
    }
    
    return ReturnCode::kFailJoinNetwork;
  }
  
  void Lorawan::ReceiveCb(SERVICE_LORA_RECEIVE_T * data)
  {
    if (data->BufferSize > 0) {
      Serial.println("something received!");
      for (int i = 0; i < data->BufferSize; i++) {
        Serial.printf("%x", data->Buffer[i]);
      }
      Serial.print("\r\n");
    }
  }
  
  void Lorawan::JoinCb(int32_t status)
  {
    Serial.printf("join status: %d\r\n", status);
  }
  
  void Lorawan::SendCb(int32_t status)
  {
    if (status == 0) {
      Serial.println("successfully sent");
    } else {
      Serial.println("sending failed");
    }
  }

} // namespace connection
} // namespace teapot
