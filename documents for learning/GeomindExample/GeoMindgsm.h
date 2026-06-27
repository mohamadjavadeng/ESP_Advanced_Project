#ifndef GEOMIND_H
#define GEOMIND_H

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SSLClient.h>


#define EARTH_RADIUS 6378137.0
class Files;
class Vector;
class Api;
class Attachment;
class Table;

const String TokenPath = "/v1/auth/token/";
const String FilePath = "/v1/files/";
const String VectorPath = "/v1/vectorLayers/";
const String AttachmentPath = "/v1/attachments/";
const String TablePath = "/v1/tables/";

class GeoMind{
    public:
        GeoMind(const String& baseUrl = "https://app.geo-mind.ai",
                SSLClient* sslClient=nullptr);
        // Authentication
        // You can either provide username/password (login) or set token directly.
        // bool login(const String& username, const String& password, String& outError, int timeoutMs = 10000);
        void setToken(const String& token); // set bearer token manually
        bool getToken(const String& username, const String& password);
        // void setRefreshToken(const String& refToken);
        // void refreshToken();
        // Simple wrappers
        // GET request to path (e.g. "/v1/some/endpoint?x=1"). Returns true if HTTP 200..299
        bool get(const String& path, JsonDocument& outJson, String& outError, int timeoutMs = 10000);

        // POST JSON. requestJson will be serialized. outJson will be parsed from response body.
        bool post(const String& path, const JsonDocument& requestJson, JsonDocument& outJson, String& outError, int timeoutMs = 10000);

        // Convenience: raw POST with string body
        bool postRaw(const String& path, const String& body, JsonDocument& outJson, String& outError, int timeoutMs = 10000);
        // Last HTTP status
        int lastHttpStatus();
        String getBaseUrl();
        friend class Files;
        friend class Vector;
        friend class Api;
        friend class Attachment;
        friend class Table;
    
    private:
        String _baseUrl;
        String _token;
        String _refreshToken;
        uint32_t _tokenExpireTime = 0;
        SSLClient* _sslClient;
        HttpClient* _http;
        int _lastStatus = -1;

        String _urlFor(const String& path);
        bool _sendRequest(const String& method, 
                            const String& path, 
                            const String& body, 
                            JsonDocument* outJson, 
                            String& outError, 
                            int timeoutMs);
    
};

/*
------------------------------------------
Class for Working with Vectors in Geomind
get vectors list
get vectors by ID
update vector info
get unique value of a field
get Vector UUID
get feature list
update features
------------------------------------------
*/
class Vector {
  public:
    Vector(GeoMind* gm);
    void begin(int vectorID);
    bool getVectors(void);
    bool getVectorByID(int vectorId);
    bool UpdateVector(String& vectorUUID, String vectorName, 
                      String displayName, String description);
    void getUniqueValuesField(int fieldID);
    void getVectorUUID(String& outUUID);
    void getfeature(int featureID);
    // void UpdateFeature(int featureID, double x, double y, String type, JsonVariant JsonFeature);
    bool createFeature(int &fID, double x, double y, String type, String Name);
    friend class Attachment;


  private:
    void _getFields(void);
    void _getVectorUUID(String& outUUID);
    int _getFieldsUniqueValuesCount(int fieldID);
    void _mercatorToWGS84(double x, double y, double &lon, double &lat);
    void _wgs84ToMercator(double lon, double lat, double &x, double &y);
    GeoMind* _gm;
    int _vectorId;
    String _vectorName;
    String _vectorUUID;
    String _displayName;
    String _description;
    String _fields[10];
    String _fieldTypes[10];
    int _fieldIDs[10];
    int _numFields;
    double _xCoord;
    double _yCoord;
    double _lon;
    double _lat;
    String _featureName;
    String _featureType;
};
#endif // GEOMIND_H
