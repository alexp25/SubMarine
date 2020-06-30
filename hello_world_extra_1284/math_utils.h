#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define GRAD 57.29577951308232087679
#define RAD 0.0174532925199433
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))
    
#ifdef __cplusplus
}
#endif

#endif