#include "GeoMindgsm.h"


// #define PI 3.14159265358979323846
GeoMind::GeoMind(const String& baseUrl, SSLClient* sslClient){
    _baseUrl = baseUrl;
    _sslClient = sslClient;
    _sslClient->setInsecure(); // For simplicity, we skip SSL certificate verification. Not recommended for production.
    // _sslClient->setBufferSizes(4096, 4096); // Adjust buffer sizes as needed
    _http = new HttpClient(*_sslClient, _baseUrl.c_str(), 443);
}


void GeoMind::setToken(const String& token){
  _token = token;
};

bool GeoMind::getToken(const String& username, const String& password){
  String contentType = "application/x-www-form-urlencoded";
  String body = "grant_type=password";
  body += "&username=" + username;
  body += "&password=" + password;
  _http->beginRequest();
  int err = _http->post(TokenPath, contentType, body);
  if (err != 0) {
    Serial.println("Connection failed: " + String(err));
    return false;
  }
  _http->endRequest();
  int status = _http->responseStatusCode();
  String response = _http->responseBody();
  while (_sslClient->available()) {
    _sslClient->read();
  }
  Serial.print("HTTP Status: ");
  Serial.println(status);

  Serial.println("Response:");
  Serial.println(response);

  if (status != 200) {
    return false;
  }
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    Serial.println("Failed to parse JSON: " + String(error.c_str()));
    return false;
  }
  _token = doc["access_token"].as<String>();
  // _refreshToken = doc["refresh_token"].as<String>();
  // _tokenExpireTime = doc["expires_in"].as<uint32_t>(); // Note
  Serial.println("Access Token: " + _token);
  return true;
}
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

Vector::Vector(GeoMind* gm) : _gm(gm) {};

void Vector::begin(int vectorID){
  _vectorId = vectorID;
  getVectorByID(vectorID);
}

bool Vector::getVectors(void){
  Serial.println("Getting vectors...");
  String urlVector = _gm->_baseUrl + VectorPath;
  Serial.println("Vector URL: " + urlVector);
  _gm ->_http->beginRequest();
  _gm ->_http->get(VectorPath.c_str());
  _gm->_http->sendHeader("Authorization", "Bearer " + _gm->_token);
  _gm->_http->endRequest();
  int status = _gm->_http->responseStatusCode();
  String response = _gm->_http->responseBody();
  while (_gm->_sslClient->available()) {
    _gm->_sslClient->read();
  }
  Serial.print("HTTP Status: ");
  Serial.println(status);
  Serial.println("Response:");
  Serial.println(response);
  if (status != 200) {
    Serial.println("Failed to get vectors");
    return false;
  }
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    Serial.println("Failed to parse JSON: " + String(error.c_str()));
    return false;
  }
  // Process the JSON document as needed
  Serial.println("Vectors retrieved successfully");
  // For example, if the response is an array of vectors:
  for (JsonObject vector : doc.as<JsonArray>()) {
    int id = vector["id"];
    String name = vector["name"].as<String>();
    Serial.println("Vector ID: " + String(id) + ", Name: " + name);
  }
  return true;
}

bool Vector::getVectorByID(int vectorId){
  Serial.println("Getting vector by ID...");
  String urlVector = VectorPath + "get-layers/" + "?ids="
                    + String(vectorId) + "&include_settings=true";
  Serial.println("Vector URL: " + urlVector);
  _gm ->_http->beginRequest();
  _gm ->_http->get(urlVector.c_str());
  _gm->_http->sendHeader("Authorization", "Bearer " + _gm->_token);
  _gm->_http->endRequest();
  int status = _gm->_http->responseStatusCode();
  String response = _gm->_http->responseBody();
  while (_gm->_sslClient->available()) {
    _gm->_sslClient->read();
  }
  Serial.print("HTTP Status: ");
  Serial.println(status);
  Serial.println("Response:");
  Serial.println(response);
  // Process the JSON document as needed
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    Serial.println("Failed to parse JSON: " + String(error.c_str()));
    return false;
  }
  Serial.println("Vectors retrieved successfully");
  JsonArray vector = doc.as<JsonArray>();
  _vectorId = vector[0]["id"].as<int>();
  _vectorName = vector[0]["name"].as<String>();
  _vectorUUID = vector[0]["uuid"].as<String>();
  Serial.println("Vector ID: " + String(_vectorId));
  Serial.println("Vector Name: " + _vectorName);
  Serial.println("Vector UUID: " + _vectorUUID);
  return true;
}

// bool Vector::UpdateVector(String& vectorUUID, String vectorName, 
//                       String displayName, String description){
//   Serial.println("Updating vector...");
//   String urlVector = VectorPath + vectorUUID + "/";
//   Serial.println("Vector URL: " + urlVector);
//   JsonDocument doc;
//   doc["name"] = vectorName;
//   doc["display_name"] = displayName;
//   doc["description"] = description;
//   String body;
//   serializeJson(doc, body);
//   Serial.println("Request Body:");
//   Serial.println(body);
//   _gm ->_http->beginRequest();
//   _gm ->_http->put(urlVector, "application/json", body);
//   _gm->_http->sendHeader("accept", "application/json");
//   _gm->_http->sendHeader("Authorization", "Bearer " + _gm->_token);
//   _gm->_http->endRequest();
//   int status = _gm->_http->responseStatusCode();
//   String response = _gm->_http->responseBody();
//   while (_gm->_sslClient->available()) {
//     _gm->_sslClient->read();
//   }
//   Serial.print("HTTP Status: ");
//   Serial.println(status);
//   Serial.println("Response:");
//   Serial.println(response);
//   if (status != 200) {
//     Serial.println("Failed to update vector");
//     return false;
//   }
//   return true;
// }

bool Vector::createFeature(int &fID, double x, double y, String type, String Name){
  String urlFeatureCreat = VectorPath + String(_vectorUUID) + "/features/";
  Serial.println("Feature Create URL: " + urlFeatureCreat);
  JsonDocument doc;
  doc["geometry"]["type"] = type;
  doc["geometry"]["coordinates"][0] = x;
  doc["geometry"]["coordinates"][1] = y;
  doc["properties"]["name"] = Name;
  String body;
  serializeJson(doc, body);
  Serial.println("Request Body:");
  Serial.println(body);
  _gm ->_http->beginRequest();
  _gm ->_http->post(urlFeatureCreat, "application/json", body);
  _gm->_http->sendHeader("accept", "application/json");
  _gm->_http->sendHeader("Authorization", "Bearer " + _gm->_token);
  _gm->_http->endRequest();
  int status = _gm->_http->responseStatusCode();
  String response = _gm->_http->responseBody();
  while (_gm->_sslClient->available()) {
    _gm->_sslClient->read();
  }
  Serial.print("HTTP Status: ");
  Serial.println(status);
  Serial.println("Response:");
  Serial.println(response);
  if (status != 201) {
    Serial.println("Failed to create feature");
    return false;
  }
  JsonDocument resDoc;
  DeserializationError error = deserializeJson(resDoc, response);
  if (error) {
    Serial.println("Failed to parse JSON: " + String(error.c_str()));
    return false;
  }
  fID = resDoc["id"].as<int>();
  Serial.println("Feature created successfully with ID: " + String(fID));
  return true;
}
