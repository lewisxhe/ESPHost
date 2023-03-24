#ifndef _ARDUINO_ESP_CONTROL_REQUEST_CLASS_H
#define _ARDUINO_ESP_CONTROL_REQUEST_CLASS_H

#include "esp_hosted_config.pb-c.h"

#include "CMsg.h"
#define SUCCESS                              0
#define FAILURE                              -1

#define SSID_LENGTH                          32
#define MAX_MAC_STR_LEN                      18
#define BSSID_LENGTH                         MAX_MAC_STR_LEN
#define PASSWORD_LENGTH                      64
#define STATUS_LENGTH                        14
#define VENDOR_OUI_BUF                       3

#define CALLBACK_SET_SUCCESS                 0
#define CALLBACK_AVAILABLE                   0
#define CALLBACK_NOT_REGISTERED              -1
#define MSG_ID_OUT_OF_ORDER                  -2

/* If request is already being served and
 * another request is pending, time period for
 * which new request will wait in seconds
 * */
#define WAIT_TIME_B2B_CTRL_REQ               5
#define DEFAULT_CTRL_RESP_TIMEOUT            30
#define DEFAULT_CTRL_RESP_AP_SCAN_TIMEOUT    (60*3)


#define SUCCESS_STR                          "success"
#define FAILURE_STR                          "failure"
#define NOT_CONNECTED_STR                    "not_connected"

/*---- Control structures ----*/

enum {
   CTRL_ERR_NOT_CONNECTED = 1,
   CTRL_ERR_NO_AP_FOUND,
   CTRL_ERR_INVALID_PASSWORD,
   CTRL_ERR_INVALID_ARGUMENT,
   CTRL_ERR_OUT_OF_RANGE,
   CTRL_ERR_MEMORY_FAILURE,
   CTRL_ERR_UNSUPPORTED_MSG,
   CTRL_ERR_INCORRECT_ARG,
   CTRL_ERR_PROTOBUF_ENCODE,
   CTRL_ERR_PROTOBUF_DECODE,
   CTRL_ERR_SET_ASYNC_CB,
   CTRL_ERR_TRANSPORT_SEND,
   CTRL_ERR_REQUEST_TIMEOUT,
   CTRL_ERR_REQ_IN_PROG,
   OUT_OF_RANGE
};


typedef enum {

   CTRL_MSGTYPE_INVALID = CTRL_MSG_TYPE__MsgType_Invalid,
   CTRL_REQ = CTRL_MSG_TYPE__Req,
   CTRL_RESP = CTRL_MSG_TYPE__Resp,
   CTRL_EVENT = CTRL_MSG_TYPE__Event,
   CTRL_MSGTYPE_MAX = CTRL_MSG_TYPE__MsgType_Max,

} AppMsgType_e;

typedef enum {

   CTRL_MSGID_INVALID = CTRL_MSG_ID__MsgId_Invalid,
   /*
    ** Request Msgs *
    */
   CTRL_REQ_BASE                      = CTRL_MSG_ID__Req_Base,
   CTRL_REQ_GET_MAC_ADDR              = CTRL_MSG_ID__Req_GetMACAddress, //0x65
   CTRL_REQ_SET_MAC_ADDR              = CTRL_MSG_ID__Req_SetMacAddress, //0x66
   CTRL_REQ_GET_WIFI_MODE             = CTRL_MSG_ID__Req_GetWifiMode,   //0x67
   CTRL_REQ_SET_WIFI_MODE             = CTRL_MSG_ID__Req_SetWifiMode,   //0x68

   CTRL_REQ_GET_AP_SCAN_LIST          = CTRL_MSG_ID__Req_GetAPScanList, //0x69
   CTRL_REQ_GET_AP_CONFIG             = CTRL_MSG_ID__Req_GetAPConfig,   //0x6a
   CTRL_REQ_CONNECT_AP                = CTRL_MSG_ID__Req_ConnectAP,     //0x6b
   CTRL_REQ_DISCONNECT_AP             = CTRL_MSG_ID__Req_DisconnectAP,  //0x6c

   CTRL_REQ_GET_SOFTAP_CONFIG         = CTRL_MSG_ID__Req_GetSoftAPConfig,            //0x6d
   CTRL_REQ_SET_SOFTAP_VND_IE         = CTRL_MSG_ID__Req_SetSoftAPVendorSpecificIE,  //0x6e
   CTRL_REQ_START_SOFTAP              = CTRL_MSG_ID__Req_StartSoftAP,                //0x6f
   CTRL_REQ_GET_SOFTAP_CONN_STA_LIST  = CTRL_MSG_ID__Req_GetSoftAPConnectedSTAList,  //0x70
   CTRL_REQ_STOP_SOFTAP               = CTRL_MSG_ID__Req_StopSoftAP,                 //0x71

   CTRL_REQ_SET_PS_MODE               = CTRL_MSG_ID__Req_SetPowerSaveMode,   //0x72
   CTRL_REQ_GET_PS_MODE               = CTRL_MSG_ID__Req_GetPowerSaveMode,   //0x73

   CTRL_REQ_OTA_BEGIN                 = CTRL_MSG_ID__Req_OTABegin,           //0x74
   CTRL_REQ_OTA_WRITE                 = CTRL_MSG_ID__Req_OTAWrite,           //0x75
   CTRL_REQ_OTA_END                   = CTRL_MSG_ID__Req_OTAEnd,             //0x76

   CTRL_REQ_SET_WIFI_MAX_TX_POWER     = CTRL_MSG_ID__Req_SetWifiMaxTxPower,  //0x77
   CTRL_REQ_GET_WIFI_CURR_TX_POWER    = CTRL_MSG_ID__Req_GetWifiCurrTxPower, //0x78

   CTRL_REQ_CONFIG_HEARTBEAT          = CTRL_MSG_ID__Req_ConfigHeartbeat,    //0x79
   /*
    * Add new control path command response before Req_Max
    * and update Req_Max
    */
   CTRL_REQ_MAX = CTRL_MSG_ID__Req_Max,
   /*
    ** Response Msgs *
    */
   CTRL_RESP_BASE                     = CTRL_MSG_ID__Resp_Base,
   CTRL_RESP_GET_MAC_ADDR             = CTRL_MSG_ID__Resp_GetMACAddress,  //0x65 -> 0xc9
   CTRL_RESP_SET_MAC_ADDRESS          = CTRL_MSG_ID__Resp_SetMacAddress,  //0x66 -> 0xca
   CTRL_RESP_GET_WIFI_MODE            = CTRL_MSG_ID__Resp_GetWifiMode,    //0x67 -> 0xcb
   CTRL_RESP_SET_WIFI_MODE            = CTRL_MSG_ID__Resp_SetWifiMode,    //0x68 -> 0xcc

   CTRL_RESP_GET_AP_SCAN_LIST         = CTRL_MSG_ID__Resp_GetAPScanList,  //0x69 -> 0xcd
   CTRL_RESP_GET_AP_CONFIG            = CTRL_MSG_ID__Resp_GetAPConfig,    //0x6a -> 0xce
   CTRL_RESP_CONNECT_AP               = CTRL_MSG_ID__Resp_ConnectAP,      //0x6b -> 0xdf
   CTRL_RESP_DISCONNECT_AP            = CTRL_MSG_ID__Resp_DisconnectAP,   //0x6c -> 0xd0

   CTRL_RESP_GET_SOFTAP_CONFIG        = CTRL_MSG_ID__Resp_GetSoftAPConfig,           //0x6d -> 0xd1
   CTRL_RESP_SET_SOFTAP_VND_IE        = CTRL_MSG_ID__Resp_SetSoftAPVendorSpecificIE, //0x6e -> 0xd2
   CTRL_RESP_START_SOFTAP             = CTRL_MSG_ID__Resp_StartSoftAP,               //0x6f -> 0xd3
   CTRL_RESP_GET_SOFTAP_CONN_STA_LIST = CTRL_MSG_ID__Resp_GetSoftAPConnectedSTAList, //0x70 -> 0xd4
   CTRL_RESP_STOP_SOFTAP              = CTRL_MSG_ID__Resp_StopSoftAP,                //0x71 -> 0xd5

   CTRL_RESP_SET_PS_MODE              = CTRL_MSG_ID__Resp_SetPowerSaveMode, //0x72 -> 0xd6
   CTRL_RESP_GET_PS_MODE              = CTRL_MSG_ID__Resp_GetPowerSaveMode, //0x73 -> 0xd7

   CTRL_RESP_OTA_BEGIN                = CTRL_MSG_ID__Resp_OTABegin,         //0x74 -> 0xd8
   CTRL_RESP_OTA_WRITE                = CTRL_MSG_ID__Resp_OTAWrite,         //0x75 -> 0xd9
   CTRL_RESP_OTA_END                  = CTRL_MSG_ID__Resp_OTAEnd,           //0x76 -> 0xda

   CTRL_RESP_SET_WIFI_MAX_TX_POWER     = CTRL_MSG_ID__Resp_SetWifiMaxTxPower,  //0x77 -> 0xdb
   CTRL_RESP_GET_WIFI_CURR_TX_POWER    = CTRL_MSG_ID__Resp_GetWifiCurrTxPower, //0x78 -> 0xdc

   CTRL_RESP_CONFIG_HEARTBEAT          = CTRL_MSG_ID__Resp_ConfigHeartbeat,    //0x79 -> 0xdd
   /*
    * Add new control path comm       and response before Resp_Max
    * and update Resp_Max
    */
   CTRL_RESP_MAX = CTRL_MSG_ID__Resp_Max,
   /*
    ** Events
    */
   CTRL_EVENT_BASE            = CTRL_MSG_ID__Event_Base,
   CTRL_EVENT_ESP_INIT        = CTRL_MSG_ID__Event_ESPInit,
   CTRL_EVENT_HEARTBEAT       = CTRL_MSG_ID__Event_Heartbeat,
   CTRL_EVENT_STATION_DISCONNECT_FROM_AP =
      CTRL_MSG_ID__Event_StationDisconnectFromAP,
   CTRL_EVENT_STATION_DISCONNECT_FROM_ESP_SOFTAP =
      CTRL_MSG_ID__Event_StationDisconnectFromESPSoftAP,
   /*
    * Add new control path command notification before Event_Max
    * and update Event_Max
    */
   CTRL_EVENT_MAX = CTRL_MSG_ID__Event_Max,
} AppMsgId_e;


typedef struct {
   /* Should be set to WIFI_VENDOR_IE_ELEMENT_ID (0xDD) */
   uint8_t element_id;
   /* Len of all bytes in the element data
    * following this field. Minimum 4 */
   uint8_t length;
   /* Vendor identifier (OUI) */
   uint8_t vendor_oui[VENDOR_OUI_BUF];
   /* Vendor-specific OUI type */
   uint8_t vendor_oui_type;
   /*length of payload field*/
   uint16_t payload_len;
   /* Payload. Length is equal to value in 'length' field, minus 4 */
   uint8_t* payload;

} vendor_ie_data_t;

typedef struct {
   uint8_t bssid[BSSID_LENGTH];
   int rssi;
} wifi_connected_stations_list_t;

typedef struct {
   int count;
   /* dynamic list*/
   wifi_connected_stations_list_t *out_list;
} wifi_softap_conn_sta_list_t;

typedef struct {
   int ps_mode;
} wifi_power_save_t;

typedef enum {
   WIFI_VND_IE_TYPE_BEACON      = CTRL__VENDOR_IETYPE__Beacon,
   WIFI_VND_IE_TYPE_PROBE_REQ   = CTRL__VENDOR_IETYPE__Probe_req,
   WIFI_VND_IE_TYPE_PROBE_RESP  = CTRL__VENDOR_IETYPE__Probe_resp,
   WIFI_VND_IE_TYPE_ASSOC_REQ   = CTRL__VENDOR_IETYPE__Assoc_req,
   WIFI_VND_IE_TYPE_ASSOC_RESP  = CTRL__VENDOR_IETYPE__Assoc_resp,
} wifi_vendor_ie_type_e;

typedef enum {
   WIFI_VND_IE_ID_0 = CTRL__VENDOR_IEID__ID_0,
   WIFI_VND_IE_ID_1 = CTRL__VENDOR_IEID__ID_1,
} wifi_vendor_ie_id_e;

typedef struct {
   bool enable;
   wifi_vendor_ie_type_e type;
   wifi_vendor_ie_id_e idx;
   vendor_ie_data_t vnd_ie;
} wifi_softap_vendor_ie_t;

typedef struct {
   uint8_t *ota_data;
   uint32_t ota_data_len;
} ota_write_t;

typedef struct {
   int power;
} wifi_tx_power_t;

typedef struct {
   /* event */
   uint32_t hb_num;
   /* Req */
   uint8_t enable;
   uint32_t duration;
} event_heartbeat_t;

typedef struct {
   int32_t reason;
   char mac[MAX_MAC_STR_LEN];
} event_station_disconn_t;




typedef enum {
   WIFI_AUTH_OPEN = CTRL__WIFI_SEC_PROT__Open,
   WIFI_AUTH_WEP = CTRL__WIFI_SEC_PROT__WEP,
   WIFI_AUTH_WPA_PSK = CTRL__WIFI_SEC_PROT__WPA_PSK,
   WIFI_AUTH_WPA2_PSK = CTRL__WIFI_SEC_PROT__WPA2_PSK,
   WIFI_AUTH_WPA_WPA2_PSK = CTRL__WIFI_SEC_PROT__WPA_WPA2_PSK,
   WIFI_AUTH_WPA2_ENTERPRISE = CTRL__WIFI_SEC_PROT__WPA2_ENTERPRISE,
   WIFI_AUTH_WPA3_PSK = CTRL__WIFI_SEC_PROT__WPA3_PSK,
   WIFI_AUTH_WPA2_WPA3_PSK = CTRL__WIFI_SEC_PROT__WPA2_WPA3_PSK,
   WIFI_AUTH_MAX,
} wifi_auth_mode_e;

typedef enum {
   WIFI_BW_HT20 = CTRL__WIFI_BW__HT20,
   WIFI_BW_HT40 = CTRL__WIFI_BW__HT40,
} wifi_bandwidth_e;

typedef enum {
   WIFI_PS_MIN_MODEM = CTRL__WIFI_POWER_SAVE__MIN_MODEM,
   WIFI_PS_MAX_MODEM = CTRL__WIFI_POWER_SAVE__MAX_MODEM,
   WIFI_PS_INVALID,
} wifi_ps_type_e;





typedef struct {
   uint8_t ssid[SSID_LENGTH];
   uint8_t bssid[BSSID_LENGTH];
   int rssi;
   int channel;
   int encryption_mode;
} wifi_scanlist_t;



typedef struct {
   int mode;
   char mac[MAX_MAC_STR_LEN];
} wifi_mac_t;

typedef struct {
   int mode;
} wifi_mode_t;

typedef struct {
   uint8_t ssid[SSID_LENGTH];
   uint8_t pwd[PASSWORD_LENGTH];
   uint8_t bssid[BSSID_LENGTH];
   bool is_wpa3_supported;
   int rssi;
   int channel;
   int encryption_mode;
   uint16_t listen_interval;
   char status[STATUS_LENGTH];
   char out_mac[MAX_MAC_STR_LEN];
} wifi_ap_config_t;

typedef struct {
   uint8_t ssid[SSID_LENGTH];
   uint8_t pwd[PASSWORD_LENGTH];
   int channel;
   int encryption_mode;
   int max_connections;
   bool ssid_hidden;
   wifi_bandwidth_e bandwidth;
   char out_mac[MAX_MAC_STR_LEN];
} softap_config_t;

typedef struct {
   int count;
   /* dynamic size */
   wifi_scanlist_t *out_list;
} wifi_ap_scan_list_t;


typedef struct Ctrl_cmd_t {
   /* msg type could be 1. req 2. resp 3. notification */
   uint8_t msg_type;

   /* control path protobuf msg number */
   uint16_t msg_id;

   /* statusof response or notification */
   uint8_t resp_event_status;

   union {
      wifi_mac_t                  wifi_mac;
      wifi_mode_t                 wifi_mode;

      wifi_ap_scan_list_t         wifi_ap_scan;
      wifi_ap_config_t            wifi_ap_config;

      softap_config_t             wifi_softap_config;
      wifi_softap_vendor_ie_t     wifi_softap_vendor_ie;
      wifi_softap_conn_sta_list_t wifi_softap_con_sta;

      wifi_power_save_t           wifi_ps;

      ota_write_t                 ota_write;

      wifi_tx_power_t             wifi_tx_power;

      event_heartbeat_t           e_heartbeat;

      event_station_disconn_t     e_sta_disconnected;
   }u;

   /* By default this callback is set to NULL.
    * When this callback is set by app while triggering request,
    * it will be automatically called asynchronously
    * by hosted control lib on receiving control response
    * in this case app will not be waiting for response.
    *
    * Whereas, when this is not set i.e. is NULL, it is understood
    * as synchronous response, and app after sending request,
    * will wait till getting a response
    */
   int (*ctrl_resp_cb)(struct Ctrl_cmd_t *data);

   /* Wait for timeout duration, if response not received,
    * it will send timeout response.
    * Default value for this time out is DEFAULT_CTRL_RESP_TIMEOUT */
   int cmd_timeout_sec;

   /* assign the data pointer to free by lower layer.
    * Ignored if assigned as NULL */
   void *free_buffer_handle;

   /* free handle to be registered
    * Ignored if assigned as NULL */
   void (*free_buffer_func)(void *free_buffer_handle);
} ctrl_cmd_t;


#define PRIO_Q_SERIAL                             0
#define PRIO_Q_BT                                 1
#define PRIO_Q_OTHERS                             2
#define MAX_PRIORITY_QUEUES                       3

/* ESP Payload Header Flags */
#define MORE_FRAGMENT                             (1 << 0)

/* Serial interface */
#define SERIAL_IF_FILE                            "/dev/esps0"





typedef enum {
   ESP_OPEN_DATA_PATH,
   ESP_CLOSE_DATA_PATH,
   ESP_RESET,
   ESP_MAX_HOST_INTERRUPT,
} ESP_HOST_INTERRUPT;


typedef enum {
   ESP_WLAN_SDIO_SUPPORT = (1 << 0),
   ESP_BT_UART_SUPPORT = (1 << 1),
   ESP_BT_SDIO_SUPPORT = (1 << 2),
   ESP_BLE_ONLY_SUPPORT = (1 << 3),
   ESP_BR_EDR_ONLY_SUPPORT = (1 << 4),
   ESP_WLAN_SPI_SUPPORT = (1 << 5),
   ESP_BT_SPI_SUPPORT = (1 << 6),
   ESP_CHECKSUM_ENABLED = (1 << 7),
} ESP_CAPABILITIES;

typedef enum {
   ESP_TEST_RAW_TP = (1 << 0),
   ESP_TEST_RAW_TP__ESP_TO_HOST = (1 << 1)
} ESP_RAW_TP_MEASUREMENT;

typedef enum {
   ESP_PACKET_TYPE_EVENT,
} ESP_PRIV_PACKET_TYPE;

typedef enum {
   ESP_PRIV_EVENT_INIT,
} ESP_PRIV_EVENT_TYPE;

typedef enum {
   ESP_PRIV_CAPABILITY,
   ESP_PRIV_SPI_CLK_MHZ,
   ESP_PRIV_FIRMWARE_CHIP_ID,
   ESP_PRIV_TEST_RAW_TP
} ESP_PRIV_TAG_TYPE;

struct esp_priv_event {
   uint8_t     event_type;
   uint8_t     event_len;
   uint8_t     event_data[0];
}__attribute__((packed));


typedef enum {
   WIFI_MODE_NONE = CTRL__WIFI_MODE__NONE,
   WIFI_MODE_STA = CTRL__WIFI_MODE__STA,
   WIFI_MODE_AP = CTRL__WIFI_MODE__AP,
   WIFI_MODE_APSTA = CTRL__WIFI_MODE__APSTA,
   WIFI_MODE_MAX
} WifiMode_t;


class CCtrlTranslate {
public:
   /* this function extracts a CtrlMsg from a received CMsg 
      the CtrlMsg obtained from this function has to be deleted! */
   static CtrlMsg *CMsg2CtrlMsg(CMsg& msg) {
      static CMsg msg_accumulator;
      CtrlMsg *rv = nullptr;
      /* need to save the more message here because msg is deleted in add_msg function
         and only the payload is copied so that the flags are lost */
      bool more_msg = msg.get_flags() & MORE_FRAGMENT;
      msg_accumulator.add_msg(msg);
      if( !more_msg) {
         /* there a not more fragment to wait for -> process the message */
         if(msg_accumulator.verify_tlv_header()) {
            /* header is correct */
            rv = ctrl_msg__unpack(NULL, msg_accumulator.get_protobuf_dim(), msg_accumulator.get_protobuf_ptr());
         }
         msg_accumulator.clear();
      }
      return rv;
   }

   static bool extractMacAddress(CtrlMsg *ans, char *mac_out, int mac_out_dim) {
      bool rv = false;
      if(ans != nullptr) {
         if(ans->msg_id == CTRL_RESP_GET_MAC_ADDR) {
            if(ans->resp_get_mac_address != nullptr) {
               if(ans->resp_get_mac_address->mac.data != nullptr) {
                  if(ans->resp_get_mac_address->resp == 0) {
                     memset(mac_out,0x00,mac_out_dim);
                     uint8_t len_l = (ans->resp_get_mac_address->mac.len < mac_out_dim-1) ? ans->resp_get_mac_address->mac.len : mac_out_dim-1;
                     strncpy(mac_out,(char *)ans->resp_get_mac_address->mac.data, len_l);
                     rv = true;
                  }
               }
            }
         }
      }
      return rv;
   }
};


template<typename T>
class CCtrlMsgWrapper {
private:
   T *payload;
   CtrlMsg request;
   
   CMsg get_msg() {
      int protobuf_len = ctrl_msg__get_packed_size(&request);
      CMsg msg(protobuf_len);
      if(msg.is_valid()){
         /* pack request into the message */
         ctrl_msg__pack(&request, msg.get_protobuf_ptr());
         msg.set_tlv_header(CTRL_EP_NAME_RESP);
         msg.set_payload_header(ESP_SERIAL_IF, 0);
         return msg;
      }
      return CMsg(0);
   }

public:
   /* ----------------------------------------------------------------------- */
   CCtrlMsgWrapper(int request_id, void (*init)(T *)) {
      /* init the protobuf request*/
      memset(&request,0x00,sizeof(request));
      ctrl_msg__init(&request);
      /* set the id of the message and the payload case */
      request.msg_id = (CtrlMsgId)request_id;
      request.payload_case = (CtrlMsg__PayloadCase)request_id;
      /* allocate the payload */
      payload = new T;
      /* call the init function for the payload */
      if(payload != nullptr) {
         if(init != nullptr) {
            init(payload);
         }
      }
   }
   
   /* ----------------------------------------------------------------------- */

   CMsg get_wifi_mac_address_msg(WifiMode_t mode) {
      if(payload != nullptr && mode < WIFI_MODE_MAX && mode > WIFI_MODE_NONE) {
         request.req_get_mac_address = payload;
         payload->mode = (CtrlWifiMode)mode;
      }
      return get_msg();
   }

   ~CCtrlMsgWrapper() {
      if(payload != nullptr) {
         delete payload;
      }
   }

};



#endif