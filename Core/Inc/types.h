//------------------------------------------------------------------------------
// UM30 NEO Типы и определения
//
// Copyright (c) 2011 by Eugeny Solodov
//
/**
* @page Types
* типы и определения
* @author Евгений Солодов <e.solodov@allmonitoring.ru>
* @author Александр Черненко <a.chernenko@allmonitoring.ru>
*/
// 
//------------------------------------------------------------------------------
#ifndef TYPES_H
#define TYPES_H
//--------------------------------------------------------------------------------

//#include "RTL.h"
//--------------------------------------------------------------------------------
/// проверка определения NULL
#ifndef NULL
    /// определение NULL
    #define NULL ((void*)(0))
#endif
//--------------------------------------------------------------------------------

#define U64_MAX_VALUE   (0xFFFFFFFFFFFFFFFF)
#define U32_MAX_VALUE   (0xFFFFFFFF)
#define U16_MAX_VALUE   (0xFFFF)
#define U8_MAX_VALUE    (0xFF)

#define S64_MAX_VALUE   (U64_MAX_VALUE>>1)
#define S32_MAX_VALUE   (U32_MAX_VALUE>>1)
#define S16_MAX_VALUE   (U16_MAX_VALUE>>1)
#define S8_MAX_VALUE    (U8_MAX_VALUE>>1)

#define INT_MAX_VALUE   0x7FFFFFFF
#define INT_MIN_VALUE   0x80000000

///Макрос, вытаскивающий размер поля из структуры
#define sizeof_member(type, member) sizeof( ((type*)0)->member )

///Макрос, вычисляющие смещение поля стурктуры относительно ее начала
#define offsetof_member(type ,member) ((size_t) &(((type *)0)->member))

typedef unsigned char       u8;
typedef unsigned short      u16;
typedef unsigned long int   u32;
typedef unsigned long long  u64;
typedef signed   long long  s64;
typedef u8                  bool8;
typedef u16                 bool16;
typedef u32                 bool32;

typedef unsigned char uchar;
typedef unsigned char byte;
typedef unsigned long ulong;
typedef unsigned int  uint;

typedef signed char s8;
typedef signed int s16;
typedef signed long int s32;
//--------------------------------------------------------------------------------

typedef long          portBASE_TYPE;
typedef unsigned long portTickType;

#endif
