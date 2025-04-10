#include "string_utils.h"

int stringLength(char *str) {
    int len = 0;
    while (str[len] != '\0') len++;
    return len;
}

void stringConcat(char *dest, char *src) {
    while (*dest) dest++;
    while (*src) *dest++ = *src++;
    *dest = '\0';
}

int stringCompare(char *str1, char *str2) {
    while (*str1 && (*str1 == *str2)) {
        str1++;
        str2++;
    }
    return *(unsigned char *)str1 - *(unsigned char *)str2;
}
