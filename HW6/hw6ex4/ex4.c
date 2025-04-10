#include "simpletools.h"
#include "string_utils.h"

int main() {
    char str1[100], str2[100];
    char result[200];

    print("Enter first string:\n");
    scan("%s", str1);

    print("Enter second string:\n");
    scan("%s", str2);

    int len1 = stringLength(str1);
    int len2 = stringLength(str2);

    print("Length of first string: %d\n", len1);
    print("Length of second string: %d\n", len2);

    stringConcat(result, str1);
    stringConcat(result, str2);
    print("Concatenated string: %s\n", result);

    int cmp = stringCompare(str1, str2);
    if (cmp == 0) {
        print("Strings are equal.\n");
    } else {
        print("Strings are not equal. Comparison result: %d\n", cmp);
    }

    return 0;
}
