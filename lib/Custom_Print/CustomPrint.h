#ifndef CUSTOM_PRINT_H
#define CUSTOM_PRINT_H

#include <Arduino.h>

#ifdef DEBUG_MODE
// Wrapper functions to centralize Serial.print / Serial.println usage
inline void print(const char *s) { Serial.print(s); }
inline void println(const char *s) { Serial.println(s); }

inline void print(const String &s) { Serial.print(s); }
inline void println(const String &s) { Serial.println(s); }

inline void print(const __FlashStringHelper *s) { Serial.print(s); }
inline void println(const __FlashStringHelper *s) { Serial.println(s); }

inline void print(char c) { Serial.print(c); }
inline void println(char c) { Serial.println(c); }

inline void print(bool b) { Serial.print(b); }
inline void println(bool b) { Serial.println(b); }

inline void print(int v) { Serial.print(v); }
inline void println(int v) { Serial.println(v); }

inline void print(unsigned int v) { Serial.print(v); }
inline void println(unsigned int v) { Serial.println(v); }

inline void print(long v) { Serial.print(v); }
inline void println(long v) { Serial.println(v); }

inline void print(unsigned long v) { Serial.print(v); }
inline void println(unsigned long v) { Serial.println(v); }

inline void print(int16_t v) { Serial.print(v); }
inline void println(int16_t v) { Serial.println(v); }

inline void print(uint16_t v) { Serial.print(v); }
inline void println(uint16_t v) { Serial.println(v); }

// inline void print(int32_t v) { Serial.print(v); }
// inline void println(int32_t v) { Serial.println(v); }

// inline void print(uint32_t v) { Serial.print(v); }
// inline void println(uint32_t v) { Serial.println(v); }

inline void print(float v) { Serial.print(v); }
inline void println(float v) { Serial.println(v); }

inline void print(float v, int digits) { Serial.print(v, digits); }
inline void println(float v, int digits) { Serial.println(v, digits); }

// Generic numeric/base overloads (for integer base or float precision)
template<typename T>
inline void print(T v, int base) { Serial.print(v, base); }

template<typename T>
inline void println(T v, int base) { Serial.println(v, base); }
#else
// Wrapper functions to not print in production mode
inline void print(const char *s) { }
inline void println(const char *s) { }

inline void print(const String &s) { }
inline void println(const String &s) { }

inline void print(const __FlashStringHelper *s) { }
inline void println(const __FlashStringHelper *s) { }

inline void print(char c) { }
inline void println(char c) { }

inline void print(bool b) { }
inline void println(bool b) { }

inline void print(int v) { }
inline void println(int v) { }

inline void print(unsigned int v) { }
inline void println(unsigned int v) { }

inline void print(long v) { }
inline void println(long v) { }

inline void print(unsigned long v) { }
inline void println(unsigned long v) { }

inline void print(int16_t v) { }
inline void println(int16_t v) { }

inline void print(uint16_t v) { }
inline void println(uint16_t v) { }

// inline void print(int32_t v) { }
// inline void println(int32_t v) { }

// inline void print(uint32_t v) { }
// inline void println(uint32_t v) { }

inline void print(float v) { }
inline void println(float v) { }

inline void print(float v, int digits) { }
inline void println(float v, int digits) { }

// Generic numeric/base overloads (for integer base or float precision)
template<typename T>
inline void print(T v, int base) { }

template<typename T>
inline void println(T v, int base) { }
#endif

#endif // CUSTOM_PRINT_H
