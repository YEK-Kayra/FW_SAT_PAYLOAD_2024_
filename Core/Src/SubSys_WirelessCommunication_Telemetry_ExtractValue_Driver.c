
#include "SubSys_WirelessCommunication_Telemetry_ExtractValue_Driver.h"

void extractValues(const char* input, char* value1, char* value2) {
    const char* start = strchr(input, '<');
    if (start != NULL) {
        start++;  // '<' karakterinin sonrasına geç
        const char* end = strchr(start, '>');
        if (end != NULL) {
            size_t length = end - start;  // İlk '<' ve '>' arasındaki uzunluk
            strncpy(value1, start, length);  // İlk değeri kopyala
            value1[length] = '\0';  // Null karakter ekle

            start = strchr(end + 1, '<');
            if (start != NULL) {
                start++;  // İkinci '<' karakterinin sonrasına geç
                end = strchr(start, '>');
                if (end != NULL) {
                    length = end - start;  // İkinci '<' ve '>' arasındaki uzunluk
                    strncpy(value2, start, length);  // İkinci değeri kopyala
                    value2[length] = '\0';  // Null karakter ekle
                } else {
                    value2[0] = '\0';  // İkinci değer yoksa boş string döndür
                }
            } else {
                value2[0] = '\0';  // İkinci '<' bulunamazsa boş string döndür
            }
        } else {
            value1[0] = '\0';  // İlk '>' bulunamazsa boş string döndür
            value2[0] = '\0';  // İkinci '<' ve '>' arasında veri yoksa boş string döndür
        }
    } else {
        value1[0] = '\0';  // İlk '<' bulunamazsa boş string döndür
        value2[0] = '\0';  // İkinci '<' ve '>' arasında veri yoksa boş string döndür
    }

}
