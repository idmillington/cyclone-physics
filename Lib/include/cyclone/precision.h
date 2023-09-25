/*
 * Interface file for code that changes when the core's precision is
 * altered.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

/**
 * @file
 *
 * Because Cyclone is designed to work at either single or double
 * precision, mathematical functions such as sqrt cannot be used
 * in the source code or headers. This file provides defines for
 * the real number type and mathematical formulae that work on it.
 *
 * @note All the contents of this file need to be changed to compile
 * Cyclone at a different precision.
 */
#ifndef CYCLONE_PRECISION_H
#define CYCLONE_PRECISION_H

#include <float.h>

namespace cyclone {

#if 0
    /**
     * Defines we're in single precision mode, for any code
     * that needs to be conditionally compiled.
     */
    #define SINGLE_PRECISION

    /**
     * Defines a real number precision. Cyclone can be compiled in
     * single or double precision versions. By default single precision is
     * provided.
     */
    typedef float real;

    /** Defines the highest value for the real number. */
    #define REAL_MAX FLT_MAX

    /** Defines the precision of the square root operator. */
    #define real_sqrt sqrtf
    /** Defines the precision of the absolute magnitude operator. */
    #define real_abs fabsf
    /** Defines the precision of the sine operator. */
    #define real_sin sinf

    /** Defines the precision of the cosine operator. */
    #define real_cos cosf

    /** Defines the precision of the exponent operator. */
    #define real_exp expf
    /** Defines the precision of the power operator. */
    #define real_pow powf

    /** Defines the precision of the floating point modulo operator. */
    #define real_fmod fmodf
    
    /** Defines the number e on which 1+e == 1 **/
    #define real_epsilon FLT_EPSILON

    #define R_PI 3.14159f
#else
    #define DOUBLE_PRECISION
    typedef double real;
    #define REAL_MAX DBL_MAX
    #define real_sqrt sqrt
    #define real_abs fabs
    #define real_sin sin
    #define real_cos cos
    #define real_exp exp
    #define real_pow pow
    #define real_fmod fmod
    #define real_epsilon DBL_EPSILON
    #define R_PI 3.14159265358979
#endif
}

#endif // CYCLONE_PRECISION_H
