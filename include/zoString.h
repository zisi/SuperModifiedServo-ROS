#ifndef ZO_STRING_H
# define ZO_STRING_H

#ifndef INLINE
# define INLINE extern inline
#endif

#include "zoTypes.h"

INLINE u16 strToU16(u08* str)
{
	U16 *num;
	num = (U16*)str;
	return num->all;
}

INLINE s16 strToS16(u08* str)
{
	S16* num;
	num = (S16*)str;
	return num->all;
}

INLINE u32 strToU32(u08* str)
{
	U32 *num;
	num = (U32*)str;
	return num->all;
}

INLINE s32 strToS32(u08* str)
{
	S32 *num;
	num = (S32*)str;
	return num->all;
}

INLINE u64 strToU64(u08* str)
{
	U64 *num;
	num = (U64*)str;
	return num->all;
}

INLINE s64 strToS64(u08* str)
{
	S64 *num;
	num = (S64*)str;
	return num->all;
}

INLINE void u16ToStr(u16 data, u08* str)
{
	U16* num;
	num = (U16*)str;
	num->all = data;
}

INLINE void s16ToStr(s16 data, u08* str)
{
	S16* num;
	num = (S16*)str;
	num->all = data;
}

INLINE void u32ToStr(u32 data, u08* str)
{
	U32* num;
	num = (U32*)str;
	num->all = data;
}

INLINE void s32ToStr(s32 data, u08* str)
{
	S32* num;
	num = (S32*)str;
	num->all = data;
}

INLINE void u64ToStr(u64 data, u08* str)
{
	U64* num;
	num = (U64*)str;
	num->all = data;
}

INLINE void s64ToStr(s64 data, u08* str)
{
	S64* num;
	num = (S64*)str;
	num->all = data;
}

#endif
