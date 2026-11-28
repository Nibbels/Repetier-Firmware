// Isolated candidate: exact AVR signed-long value semantics for radix 10.
// No pointer consumption, errno observation or stricter G-code grammar.
// The caller retains its original outer character scan.
#include <stdint.h>
static inline int32_t parseDecimal32(const char* p) {
    uint8_t c=uint8_t(*p);
    while(c==' ' || (c>=9 && c<=13)) c=uint8_t(*++p);
    const bool negative=c=='-';
    if(c=='+' || c=='-') c=uint8_t(*++p);
    uint32_t value=0;
    for(;;) {
        const uint8_t digit=uint8_t(c-'0');
        if(digit>9) break;
        // Test the threshold before multiplying. For negative values the
        // final allowed digit is eight, otherwise seven. Once saturated,
        // consuming the remaining digits cannot change the returned value.
        if(value>214748364UL || (value==214748364UL && digit>(negative?8:7)))
            return negative?INT32_MIN:INT32_MAX;
        value=value*10u+digit;
        c=uint8_t(*++p);
    }
    if(!negative) return int32_t(value);
    return value==2147483648UL?INT32_MIN:-int32_t(value);
}
