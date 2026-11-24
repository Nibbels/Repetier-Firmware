#ifndef R_MATH_H
#define R_MATH_H

#undef min
#undef max

class RMath {
public:
    static inline float min(float a, float b) {
        if (a < b)
            return a;
        return b;
    } // min

    static inline float max(float a, float b) {
        if (a < b)
            return b;
        return a;
    } // max

    static inline long min(long a, long b) {
        if (a < b)
            return a;
        return b;
    } // min

    static inline long max(long a, long b) {
        if (a < b)
            return b;
        return a;
    } // max

    static inline int min(int a, int b) {
        if (a < b)
            return a;
        return b;
    } // min

    static inline int max(int a, int b) {
        if (a < b)
            return b;
        return a;
    } // max

    static inline unsigned int min(unsigned int a, unsigned int b) {
        if (a < b)
            return a;
        return b;
    } // min

    static inline unsigned int max(unsigned int a, unsigned int b) {
        if (a < b)
            return b;
        return a;
    } // max

    static inline long sqr(long a) {
        return a * a;
    } // sqr

    static inline float sqr(float a) {
        return a * a;
    } // sqr
};

#endif // R_MATH_H
