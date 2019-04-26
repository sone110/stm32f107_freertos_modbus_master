#ifndef APP_UTIL_H__
#define APP_UTIL_H__

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"


#ifdef __cplusplus
extern "C" {
#endif

/** The upper 8 bits of a 32 bit value */
#define HIGH_8BITS_OF_32(a) (((a) & 0xFF000000) >> 24)
/** The lower 8 bits (of a 32 bit value) */
#define LOW_8BITS_OF_32(a) ((a) & 0x000000FF)

/** The upper 8 bits of a 16 bit value */
#define HIGH_8BITS_OF_16(a) (((a) & 0xFF00) >> 8)
/** The lower 8 bits (of a 16 bit value) */
#define LOW_8BITS_OF_16(a) ((a) & 0x00FF)

/** The upper 16 bits of a 32 bit value */
#define HIGH_16BITS_OF_32(a) (((a) & 0xFFFF0000) >> 16)
/** The lower 16 bits (of a 32 bit value) */
#define LOW_16BITS_OF_32(a) ((a) & 0x0000FFFF)  
  
#define CONVERT_TO_16BITS(a)   ((uint16_t)(a))
#define CONVERT_TO_32BITS(a)   ((uint32_t)(a))  
  
/** Leaves the minimum of the two 32-bit arguments */
#define MIN(a, b) ((a) < (b) ? (a) : (b))
/** Leaves the maximum of the two 32-bit arguments */
#define MAX(a, b) ((a) < (b) ? (b) : (a))


#define CONCAT_2(p1, p2)      CONCAT_2_(p1, p2)
#define CONCAT_2_(p1, p2)     p1##p2

#define CONCAT_3(p1, p2, p3)  CONCAT_3_(p1, p2, p3)
#define CONCAT_3_(p1, p2, p3) p1##p2##p3

#define NUM_TO_STR_INTERNAL(val) #val
#define NUM_TO_STR(val) NUM_TO_STR_INTERNAL(val)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

      

/**@brief Set a bit in the uint32 word.
 *
 * @param[in] W  Word whose bit is being set.
 * @param[in] B  Bit number in the word to be set.
*/
#ifndef SET_BIT
#define SET_BIT(W,B)  ((W) |= (uint32_t)(1U << (B)))
#endif

/**@brief Clears a bit in the uint32 word.
 *
 * @param[in] W   Word whose bit is to be cleared.
 * @param[in] B   Bit number in the word to be cleared.
 */
#ifndef CLR_BIT
#define CLR_BIT(W, B) ((W) &= (~((uint32_t)1 << (B))))
#endif

/**@brief Checks if a bit is set.
 *
 * @param[in] W   Word whose bit is to be checked.
 * @param[in] B   Bit number in the word to be checked.
 *
 * @retval 1 if bit is set.
 * @retval 0 if bit is not set.
 */
#ifndef IS_SET
#define IS_SET(W,B) (((W) >> (B)) & 1)
#endif
      
   
#define UNUSED_VARIABLE(X)  ((void)(X))
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)
#define UNUSED_RETURN_VALUE(X) UNUSED_VARIABLE(X)
      
#if   defined ( __CC_ARM )
  #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler          */
  #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler       */
  #define __STATIC_INLINE  static __inline

#elif defined ( __GNUC__ )
  #define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
  #define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */
  #define __STATIC_INLINE  static inline

#elif defined ( __ICCARM__ )
  #define __ASM            __asm                                      /*!< asm keyword for IAR Compiler          */
  #define __INLINE         inline                                     /*!< inline keyword for IAR Compiler. Only available in High optimization mode! */
  #define __STATIC_INLINE  static inline
#endif

/**@brief Macro for performing rounded integer division (as opposed to truncating the result).
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Rounded (integer) result of dividing A by B.
 */
#define ROUNDED_DIV(A, B) (((A) + ((B) / 2)) / (B))


/**@brief Macro for checking if an integer is a power of two.
 *
 * @param[in]   A   Number to be tested.
 *
 * @return      true if value is power of two.
 * @return      false if value not power of two.
 */
#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )





/**@brief Macro for performing integer division, making sure the result is rounded up.
 *
 * @details One typical use for this is to compute the number of objects with size B is needed to
 *          hold A number of bytes.
 *
 * @param[in]   A   Numerator.
 * @param[in]   B   Denominator.
 *
 * @return      Integer result of dividing A by B, rounded up.
 */
#define CEIL_DIV(A, B)      \
    (((A) + (B) - 1) / (B))



/**
 * @brief Size of a field in declared structure
 *
 * Macro that returns the size of the structure field.
 * @param struct_type Variable type to get the field size from
 * @param field Field name to analyze. It can be even field inside field (field.somethingelse.and_another).
 *
 * @return Size of the field
 */
#define FIELD_SIZE(struct_type, field) sizeof(((struct struct_type*)NULL)->field)

/**
 * @brief Number of elements in field array in declared structure
 *
 * Macro that returns number of elementy in structure field.
 * @param struct_type Variable type to get the field size from
 * @param field Field name to analyze.
 *
 * @return Number of elements in field array
 *
 * @sa FIELD_SIZE
 */
#define FIELD_ARRAY_SIZE(struct_type, field) (FIELD_SIZE(struct_type, field) / FIELD_SIZE(struct_type, field[0]))



/**@brief Function for encoding a uint16 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}

/**@brief Function for encoding a three-byte value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint24_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    return 3;
}

/**@brief Function for encoding a uint32 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}

/**@brief Function for encoding a uint48 value.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data is to be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint48_encode(uint64_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x0000000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x00000000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x000000FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0x0000FF000000) >> 24);
    p_encoded_data[4] = (uint8_t) ((value & 0x00FF00000000) >> 32);
    p_encoded_data[5] = (uint8_t) ((value & 0xFF0000000000) >> 40);
    return 6;
}

/**@brief Function for decoding a uint16 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}

/**@brief Function for decoding a uint16 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint16_t uint16_big_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0]) << 8 ) |
                 (((uint16_t)((uint8_t *)p_encoded_data)[1])) );
}

/**@brief Function for decoding a three-byte value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value (uint32_t).
 */
static __INLINE uint32_t uint24_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16));
}

/**@brief Function for decoding a uint32 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint32_t uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}

static __INLINE uint32_t uint32_decode_uint16(const uint16_t * p_encoded_data)
{
    return ( (((uint32_t)((uint16_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint16_t *)p_encoded_data)[1]) << 16 ));
}
/**@brief Function for decoding a uint32 value in big-endian format.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value.
 */
static __INLINE uint32_t uint32_big_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 24) |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 0) );
}
static __INLINE uint32_t uint32_big_decode_uint16(const uint16_t * p_encoded_data)
{
    return ( (((uint32_t)((uint16_t *)p_encoded_data)[0]) << 16) |
             (((uint32_t)((uint16_t *)p_encoded_data)[1]) << 0) );
}
/**
 * @brief Function for encoding an uint16 value in big-endian format.
 *
 * @param[in]   value            Value to be encoded.
 * @param[out]  p_encoded_data   Buffer where the encoded data will be written.
 *
 * @return      Number of bytes written.
 */
static __INLINE uint8_t uint16_big_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) (value >> 8);
    p_encoded_data[1] = (uint8_t) (value & 0xFF);

    return sizeof(uint16_t);
}



/**@brief Function for decoding a uint48 value.
 *
 * @param[in]   p_encoded_data   Buffer where the encoded data is stored.
 *
 * @return      Decoded value. (uint64_t)
 */
static __INLINE uint64_t uint48_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint64_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint64_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint64_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint64_t)((uint8_t *)p_encoded_data)[3]) << 24) |
             (((uint64_t)((uint8_t *)p_encoded_data)[4]) << 32) |
             (((uint64_t)((uint8_t *)p_encoded_data)[5]) << 40 ));
}

/** @brief Function for converting the input voltage (in milli volts) into percentage of 3.0 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.0V returns 100% battery level. The limit for power failure is 2.1V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for CR2032 is non-linear. In this model it is split into
 *           4 linear sections:
 *           - Section 1: 3.0V - 2.9V = 100% - 42% (58% drop on 100 mV)
 *           - Section 2: 2.9V - 2.74V = 42% - 18% (24% drop on 160 mV)
 *           - Section 3: 2.74V - 2.44V = 18% - 6% (12% drop on 300 mV)
 *           - Section 4: 2.44V - 2.1V = 6% - 0% (6% drop on 340 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
*/
static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}

/**@brief Function for checking if a pointer value is aligned to a 4 byte boundary.
 *
 * @param[in]   p   Pointer value to be checked.
 *
 * @return      TRUE if pointer is aligned to a 4 byte boundary, FALSE otherwise.
 */
static __INLINE bool is_word_aligned(void const* p)
{
    return (((uintptr_t)p & 0x03) == 0);
}




#ifdef __cplusplus
}
#endif

#endif // APP_UTIL_H__

/** @} */
