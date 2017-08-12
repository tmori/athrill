/*
 *  mruby => TECS brige
 *  
 *   Copyright (C) 2008-2012 by TOPPERS Project
 *--
 *   上記著作権者は，以下の(1)(4)の条件を満たす場合に限り，本ソフトウェ
 *   ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *   変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *   (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *       権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *       スコード中に含まれていること．
 *   (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *       用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *       者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *       の無保証規定を掲載すること．
 *   (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *       用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *       と．
 *     (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *         作権表示，この利用条件および下記の無保証規定を掲載すること．
 *     (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *         報告すること．
 *   (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *       害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *       また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *       由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *       免責すること．
 *  
 *   本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *   よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *   に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *   アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *   の責任を負わない．
 *  
 *   $Id: TECSPointer.h 2637 2017-05-08 10:30:59Z okuma-top $ 
 */

#ifndef TECSPointer_h__
#define TECSPointer_h__

#include <string.h>
#include <stdint.h>
#include <limits.h>

#ifndef CHAR_T_DEFINED
typedef	int		bool_t;
typedef	char	char_t;
typedef	double	double64_t;
typedef	float	float32_t;
#endif /* CHAR_T_DEFINED */

////  define Pointer classes ////
#define POINTER_BODY( Type, TYPE )										\
																		\
	struct Type ## PointerBody {										\
		uint32_t	length;												\
		TYPE 		buf[ /* length */ ];								\
	};


// struct TypePointerBody
POINTER_BODY( Bool,  bool_t )		/* struct BoolPointerBody */
POINTER_BODY( Double64, double64_t )/* struct Double64PointerBody */
POINTER_BODY( Float32, float32_t )	/* struct Float32PointerBody */

POINTER_BODY( Int8,  int8_t )		/* struct Int8PointerBody */
POINTER_BODY( Int16, int16_t )		/* struct Int16PointerBody */
POINTER_BODY( Int32, int32_t )		/* struct Int32PointerBody */
POINTER_BODY( Int64, int64_t )		/* struct Int64PointerBody */
POINTER_BODY( Char,  char_t )		/* struct CharPointerBody */
POINTER_BODY( Int,   int )			/* struct IntPointerBody */
POINTER_BODY( Short, short )		/* struct ShortPointerBody */
POINTER_BODY( Long,  long )			/* struct LongPointerBody */

POINTER_BODY( UInt8,  uint8_t )		/* struct UInt8PointerBody */
POINTER_BODY( UInt16, uint16_t )	/* struct UInt16PointerBody */
POINTER_BODY( UInt32, uint32_t )	/* struct UInt32PointerBody */
POINTER_BODY( UInt64, uint64_t )	/* struct UInt64PointerBody */
POINTER_BODY( UChar,  uint8_t )		/* struct UCharPointerBody */
POINTER_BODY( UInt,   unsigned int )	/* struct UIntPointerBody */
POINTER_BODY( UShort, unsigned short )	/* struct UShortPointerBody */
POINTER_BODY( ULong,  unsigned long )	/* struct ULongPointerBody */

POINTER_BODY( SChar,  char_t )		/* struct SCharPointerBody */

// ■ TECS_NO_VAL_CHECK を define すると、値チェックを省略する
#ifndef TECS_NO_VAL_CHECK
#define	VALCHECK_INT(Type, TYPE, type )									\
	/*  check before assigning mrb_int val to intN_t */					\
	static inline void													\
	VALCHECK_ ## Type( mrb_state *mrb, mrb_int val )					\
	{																	\
		if( sizeof( mrb_int ) > sizeof( type ) ){						\
			if( val > TYPE ## _MAX || val < TYPE ## _MIN ) 				\
				mrb_raise(mrb, E_ARGUMENT_ERROR, "too large or too small for "  #type "_t"); \
		}																\
	}
#define	VALCHECK_MRB_INT(Type, TYPE, type )								\
	/*  check before assigning val (intN_t) to mrb_int */				\
	static inline void													\
	VALCHECK_MRB_ ## Type( mrb_state *mrb, type val )					\
	{																	\
		if( sizeof( type ) > sizeof( mrb_int ) ){						\
			if( val >= (((type)1) << (sizeof(mrb_int)*8-1))				\
				|| val < -(((type)1) << (sizeof(mrb_int)*8-1)) )		\
				/* '=' unecessary for negative value	*/				\
				/* ignore warning on int32_t */							\
				mrb_raise(mrb, E_ARGUMENT_ERROR, "too large or too small for mrb_int"); \
		}																\
	}

#define	VALCHECK_UINT(Type, TYPE, type )								\
	/*  check before assigning mrb_int val to intN_t */					\
	static inline void													\
	VALCHECK_ ## Type( mrb_state *mrb, mrb_int  val )					\
	{																	\
		if( sizeof( mrb_int ) > sizeof( type ) ){						\
			if( val > TYPE ## _MAX || val < 0 ) 						\
				mrb_raise(mrb, E_ARGUMENT_ERROR, "too large or too small for "  #type "_t"); \
		}																\
	}
#define	VALCHECK_MRB_UINT(Type, TYPE, type )							\
	/*  check before assigning val (intN_t) to mrb_int */				\
	static inline void													\
	VALCHECK_MRB_ ## Type( mrb_state *mrb, type val )					\
	{																	\
		if( sizeof( type ) > sizeof( mrb_int ) ){						\
			if( val >= (((type)1) << (sizeof(mrb_int)*8)))				\
				/* '=' unecessary for negative value	*/				\
				/* ignore warning on int32_t */							\
				mrb_raise(mrb, E_ARGUMENT_ERROR, "too large or too small for mrb_int"); \
		}																\
	}
#else // TECS_NO_VAL_CHECK

#define	VALCHECK_INT(Type, TYPE, type )
#define	VALCHECK_UINT(Type, TYPE, type )

#endif // TECS_NO_VAL_CHECK

// define VALCHECK function
#define VALCHECK_Bool( mrb, val )
#define	VALCHECK_MRB_Bool( mrb, val )

VALCHECK_INT( Int8, INT8, int8_t )
VALCHECK_MRB_INT( Int8, INT8, int8_t )
VALCHECK_INT( Int16, INT16, int16_t )
VALCHECK_MRB_INT( Int16, INT16, int16_t )
VALCHECK_UINT( UInt8, UINT8, uint8_t )
#define	VALCHECK_MRB_UInt8(mrb,val)
VALCHECK_UINT( UInt16, UINT16, uint16_t )
#define	VALCHECK_MRB_UInt16(mrb,val)
VALCHECK_INT( Int32, INT32, int32_t )
#define VALCHECK_MRB_Int32(mrb,val)
VALCHECK_UINT( UInt32, UINT32, uint32_t )
#define VALCHECK_MRB_UInt32(mrb,val)

VALCHECK_INT( Int64, INT64, int64_t )
VALCHECK_UINT( UInt64, UINT64, uint64_t )
#ifndef mrb_int_IS_int64
	 // ■ int 32 bit を仮定している. 64 bit であれば mrb_int_IS_64 を define する
VALCHECK_MRB_INT( Int64, INT64, int64_t )
VALCHECK_MRB_UINT( UInt64, UINT64, uint64_t )
#else
#define VALCHECK_MRB_Int64(mrb,val)      // 範囲チェックが無意味であるため、警告が出るのを回避する
#define VALCHECK_MRB_UInt64(mrb,val)      // 範囲チェックが無意味であるため、警告が出るのを回避する
#endif

#ifndef TECS_NO_VAL_CHECK
	 // ■ char_t は unsigned とみなす
static inline void
VALCHECK_Char( mrb_state *mrb, mrb_int  val )
{
	if( val > 0xff || val < 0 )	
		mrb_raise(mrb, E_ARGUMENT_ERROR, "too large or too small for char_t");
}
#define	VALCHECK_MRB_Char( mrb, val )

VALCHECK_INT( Short, SHRT, short )
#define	VALCHECK_MRB_Short( mrb, val )

VALCHECK_INT( Int, INT, int )
#define	VALCHECK_MRB_Int( mrb, val )

VALCHECK_INT( Long, LONG, long )
#define	VALCHECK_MRB_Long( mrb, val )

VALCHECK_INT( SChar, SCHAR, signed char )
#define	VALCHECK_MRB_SChar( mrb, val )

VALCHECK_UINT( UChar, UCHAR, unsigned char )
#define	VALCHECK_MRB_UChar( mrb, val )

VALCHECK_UINT( UInt, UINT, unsigned int )
#define	VALCHECK_MRB_UInt( mrb, val )

VALCHECK_UINT( UShort, USHRT, unsigned short )
#define	VALCHECK_MRB_UShort( mrb, val )

VALCHECK_UINT( ULong, ULONG, unsigned long )
#define	VALCHECK_MRB_ULong( mrb, val )

#else // TECS_NO_VAL_CHECK

#define VALCHECK_Char( mrb, val )
#define	VALCHECK_MRB_Char( mrb, val )
#define VALCHECK_Int( mrb, val )
#define	VALCHECK_MRB_Int( mrb, val )
#define	VALCHECK_Short( mrb, val )
#define	VALCHECK_MRB_Short( mrb, val )
#define	VALCHECK_Long( mrb, val )
#define	VALCHECK_MRB_Long( mrb, val )
#define	VALCHECK_SChar( mrb, val )
#define	VALCHECK_MRB_SChar( mrb, val )
#define	VALCHECK_UChar( mrb, val )
#define	VALCHECK_MRB_UChar( mrb, val )
#define	VALCHECK_UInt( mrb, val )
#define	VALCHECK_MRB_UInt( mrb, val )
#define	VALCHECK_UShort( mrb, val )
#define	VALCHECK_MRB_UShort( mrb, val )
#define	VALCHECK_ULong( mrb, val )
#define	VALCHECK_MRB_ULong( mrb, val )

#endif // TECS_NO_VAL_CHECK

#define CHECK_AND_GET_POINTER( Type, type )								\
	type *CheckAndGet ## Type ## Pointer( mrb_state *mrb, mrb_value value, int32_t len ) \
	{																	\
		if( mrb_type(value) != MRB_TT_DATA								\
			|| DATA_TYPE( value ) == 0									\
			|| strcmp( DATA_TYPE( value )->struct_name,  "TECS::" #Type "Pointer" ) ) \
			mrb_raise(mrb, E_TYPE_ERROR, "not PointerType or unexpected PointerType"); \
		if( len != -1 && ((struct Type ## PointerBody *)DATA_PTR( value ))->length < len ) \
			mrb_raise(mrb, E_ARGUMENT_ERROR, "size_is too short");		\
		return ((struct Type##PointerBody*)(DATA_PTR(value)))->buf;		\
	}																	\
																		\
	type *CheckAndGet ## Type ## PointerNullable( mrb_state *mrb, mrb_value value, int32_t len ) \
	{																	\
		if( mrb_nil_p(value) )											\
			return NULL;												\
		return CheckAndGet ## Type ## Pointer( mrb, value, len );		\
	}


/*
 * CHECK_AND_GET_CHAR_POINTER & CHECK_AND_GET_CHAR_POINTER_MOD are used 
 * in case that the macro TECS_Use_MrbString_for_CharPointer is defined.
 * This does not work as str_modify is static funcion.
 */
#define CHECK_AND_GET_CHAR_POINTER( Type, type )								\
	type *CheckAndGet ## Type ## Pointer( mrb_state *mrb, mrb_value value, int32_t len ) \
	{																	\
		if( mrb_type(value) != MRB_TT_STRING )							\
			mrb_raise(mrb, E_TYPE_ERROR, "not String Type");			\
		if( len != -1 && RSTRING_LEN(value) < len )						\
			mrb_raise(mrb, E_ARGUMENT_ERROR, "size_is too short");		\
		return RSTRING_PTR(value);										\
	}																	\
																		\
	type *CheckAndGet ## Type ## PointerNullable( mrb_state *mrb, mrb_value value, int32_t len ) \
	{																	\
		if( mrb_nil_p(value) )											\
			return NULL;												\
		return CheckAndGet ## Type ## Pointer( mrb, value, len );		\
	}

#define CHECK_AND_GET_CHAR_POINTER_MOD( Type, type )					\
	type *CheckAndGet ## Type ## PointerMod( mrb_state *mrb, mrb_value value, int32_t len ) \
	{																	\
		char *ptr;														\
		if( mrb_type(value) != MRB_TT_STRING )							\
			mrb_raise(mrb, E_TYPE_ERROR, "not String Type");			\
		str_modify(mrb,RSTRING(value));									\
		if( len == -1 || RSTRING_LEN(value) < len )						\
			mrb_raise(mrb, E_ARGUMENT_ERROR, "size_is too short");		\
		ptr = RSTRING_PTR(value);										\
		ptr[len] = '\0';												\
		return (type *)ptr;												\
	}																	\
																		\
	type *CheckAndGet ## Type ## PointerModNullable( mrb_state *mrb, mrb_value value, int32_t len ) \
	{																	\
		if( mrb_nil_p(value) )											\
			return NULL;												\
		return CheckAndGet ## Type ## Pointer( mrb, value, len );		\
	}

#define CHECK_AND_GET_POINTER_PROTO( Type, type )						\
	type *CheckAndGet ## Type ## Pointer( mrb_state *mrb, mrb_value value, int32_t len ); \
	type *CheckAndGet ## Type ## PointerNullable( mrb_state *mrb, mrb_value value, int32_t len )
#define CHECK_AND_GET_POINTER_MOD_PROTO( Type, type )					\
	type *CheckAndGet ## Type ## PointerMod( mrb_state *mrb, mrb_value value, int32_t len ); \
	type *CheckAndGet ## Type ## PointerModNullable( mrb_state *mrb, mrb_value value, int32_t len )

CHECK_AND_GET_POINTER_PROTO( Bool,  bool_t );		/* Bool pointer */
CHECK_AND_GET_POINTER_PROTO( Double64, double64_t );/* Double64 pointer */
CHECK_AND_GET_POINTER_PROTO( Float32, float32_t );	/* Float32 pointer */

CHECK_AND_GET_POINTER_PROTO( Int8,  int8_t );		/* Int8 pointer */
CHECK_AND_GET_POINTER_PROTO( Int16, int16_t );		/* Int16 pointer */
CHECK_AND_GET_POINTER_PROTO( Int32, int32_t );		/* Int32 pointer */
CHECK_AND_GET_POINTER_PROTO( Int64, int64_t );		/* Int64 pointer */
CHECK_AND_GET_POINTER_PROTO( Int,   int );			/* Int pointer */
CHECK_AND_GET_POINTER_PROTO( Short, short );		/* Short pointer */
CHECK_AND_GET_POINTER_PROTO( Long,  long );			/* Long pointer */

CHECK_AND_GET_POINTER_PROTO( UInt8,  uint8_t );		/* UInt8 pointer */
CHECK_AND_GET_POINTER_PROTO( UInt16, uint16_t );	/* UInt16 pointer */
CHECK_AND_GET_POINTER_PROTO( UInt32, uint32_t );	/* UInt32 pointer */
CHECK_AND_GET_POINTER_PROTO( UInt64, uint64_t );	/* UInt64 pointer */
CHECK_AND_GET_POINTER_PROTO( UInt,   unsigned int );/* UInt pointer */
CHECK_AND_GET_POINTER_PROTO( UShort, unsigned short );	/* UShort pointer */
CHECK_AND_GET_POINTER_PROTO( ULong,  unsigned long );	/* ULong pointer */

#ifndef TECS_Use_MrbString_for_CharPointer
CHECK_AND_GET_POINTER_PROTO( Char,  char_t );		/* Char pointer */
CHECK_AND_GET_POINTER_PROTO( SChar,  char_t );		/* SChar pointer */
//CHECK_AND_GET_POINTER_PROTO( SChar,  schar_t );		/* SChar pointer */
CHECK_AND_GET_POINTER_PROTO( UChar,  uint8_t );		/* UChar pointer */
//CHECK_AND_GET_POINTER_PROTO( UChar,  uchar_t );		/* UChar pointer */
#define CheckAndGetCharPointerMod	CheckAndGetCharPointer
#define CheckAndGetSCharPointerMod	CheckAndGetSCharPointer
#define CheckAndGetUCharPointerMod	CheckAndGetUCharPointer
#define CheckAndGetCharPointerModNullable	CheckAndGetCharPointerNullable
#define CheckAndGetSCharPointerModNullable	CheckAndGetSCharPointerNullable
#define CheckAndGetUCharPointerModNullable	CheckAndGetUCharPointerNullable
#else
CHECK_AND_GET_POINTER_PROTO( Char,  char_t );		/* Char pointer */
CHECK_AND_GET_POINTER_PROTO( SChar,  char_t );		/* SChar pointer */
CHECK_AND_GET_POINTER_PROTO( UChar,  uint8_t );		/* UChar pointer */
CHECK_AND_GET_POINTER_MOD_PROTO( Char,  char_t );	/* Char pointer */
CHECK_AND_GET_POINTER_MOD_PROTO( SChar,  char_t );	/* SChar pointer */
CHECK_AND_GET_POINTER_MOD_PROTO( UChar,  uint8_t );	/* UChar pointer */
#endif /* TECS_Use_MrbString_for_CharPointer */

#define GET_SET_BOOL( Type, TYPE )										\
	static mrb_value													\
	Type ## Pointer_aget(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self );		\
		mrb_int	subscript;												\
		TYPE	val;													\
																		\
		mrb_get_args(mrb, "i", &subscript );							\
		if( subscript < 0 || subscript >= pointerBody->length )			\
			mrb_raise(mrb, E_INDEX_ERROR, "index is out of array (get)"); \
		val = pointerBody->buf[ subscript ];							\
		VALCHECK_MRB_ ## Type( mrb, val );								\
		return val ? mrb_true_value() : mrb_false_value();				\
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_aset(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self ); 	\
		mrb_int	subscript;												\
		mrb_value	val;													\
																		\
		mrb_get_args(mrb, "io", &subscript, &val );						\
																		\
		if( subscript < 0 || subscript >= pointerBody->length )			\
			mrb_raise(mrb, E_INDEX_ERROR, "index is out of array (set)"); \
		pointerBody->buf[ subscript ] = mrb_test( val );				\
																		\
		return self;													\
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_get_val(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self );		\
		TYPE	val;													\
																		\
		val = *pointerBody->buf;										\
		VALCHECK_MRB_ ## Type( mrb, val );								\
		return val ? mrb_true_value() : mrb_false_value();				\
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_set_val(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self ); 	\
		mrb_value	val;												\
																		\
		mrb_get_args(mrb, "o", &val );									\
																		\
		*pointerBody->buf = mrb_test( val );							\
		return self;													\
	}

#define GET_SET_CHAR( Type, TYPE )		GET_SET_INT( Type, TYPE )
#define GET_SET_INT( Type, TYPE )										\
	static mrb_value													\
	Type ## Pointer_aget(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self );		\
		mrb_int	subscript;												\
		TYPE	val;													\
																		\
		mrb_get_args(mrb, "i", &subscript );							\
		if( subscript < 0 || subscript >= pointerBody->length )			\
			mrb_raise(mrb, E_INDEX_ERROR, "index is out of array (get)"); \
		val = pointerBody->buf[ subscript ];							\
		VALCHECK_MRB_ ## Type( mrb, val );								\
		return mrb_fixnum_value( val );									\
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_aset(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self ); 	\
		mrb_int	subscript;												\
		mrb_int	val;													\
																		\
		mrb_get_args(mrb, "ii", &subscript, &val );						\
																		\
		if( subscript < 0 || subscript >= pointerBody->length )			\
			mrb_raise(mrb, E_INDEX_ERROR, "index is out of array (set)"); \
		VALCHECK_ ## Type ( mrb, val );									\
		pointerBody->buf[ subscript ] = val;							\
																		\
		return self;													\
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_get_val(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self );		\
		TYPE	val;													\
																		\
		val = *pointerBody->buf;										\
		VALCHECK_MRB_ ## Type( mrb, val );								\
		return mrb_fixnum_value( val );									\
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_set_val(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self ); 	\
		mrb_int	val;													\
																		\
		mrb_get_args(mrb, "i", &val );									\
																		\
		VALCHECK_ ## Type ( mrb, val );									\
		*pointerBody->buf = val;										\
																		\
		return self;													\
	}

#define GET_SET_FLOAT( Type, TYPE )										\
	static mrb_value													\
	Type ## Pointer_aget(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self );		\
		mrb_int	subscript;												\
																		\
		mrb_get_args(mrb, "i", &subscript );							\
		if( subscript < 0 || subscript >= pointerBody->length )			\
			mrb_raise(mrb, E_INDEX_ERROR, "index is out of array (get)"); \
		return mrb_float_value( mrb, pointerBody->buf[ subscript ] );   \
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_aset(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self ); 	\
		mrb_int	subscript;												\
		mrb_float	val;												\
																		\
		mrb_get_args(mrb, "if", &subscript, &val );						\
																		\
		if( subscript < 0 || subscript >= pointerBody->length )			\
			mrb_raise(mrb, E_INDEX_ERROR, "index is out of array (set)"); \
		pointerBody->buf[ subscript ] = val;							\
																		\
		return self;													\
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_get_val(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self );		\
																		\
		return mrb_float_value( mrb, *pointerBody->buf );               \
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_set_val(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self ); 	\
		mrb_float	val;												\
																		\
		mrb_get_args(mrb, "f", &val );									\
																		\
		*pointerBody->buf = val;										\
																		\
		return self;													\
	}

////  define Pointer classes ////
#define POINTER_CLASS( Type, TYPE )										\
																		\
	static void															\
	Type ## PointerBody_free( mrb_state *mrb, void *p )					\
	{																	\
		(void)mrb_free( mrb, p );										\
	}																	\
																		\
	struct mrb_data_type Type ## PointerBody_mrb_data_type =			\
		{																\
			"TECS::" #Type "Pointer",									\
			Type ## PointerBody_free									\
		};																\
																		\
	static mrb_value													\
	Type ## Pointer_initialize( mrb_state *mrb, mrb_value self)			\
	{																	\
		mrb_int	length;													\
		struct Type ## PointerBody *pointerBody;						\
																		\
		mrb_get_args(mrb, "i", &length );								\
		if( length < 0 )												\
			mrb_raise(mrb, E_INDEX_ERROR, "negative length");			\
		DATA_TYPE( self ) = &Type ## PointerBody_mrb_data_type;			\
		pointerBody = (struct Type ## PointerBody *)mrb_malloc(mrb, sizeof(struct Type ## PointerBody) + length * sizeof( TYPE ) ); \
		pointerBody->length = length;									\
		DATA_PTR( self ) = (void *)pointerBody;							\
						 												\
		return self;													\
	}																	\
																		\
	static mrb_value													\
	Type ## Pointer_size(mrb_state *mrb, mrb_value self)				\
	{																	\
		struct Type ## PointerBody *pointerBody = DATA_PTR( self );		\
																		\
		return mrb_fixnum_value( pointerBody->length );					\
	}																	\
																		\
	struct RClass *														\
	tecs_init_## Type ## Pointer(mrb_state *mrb, struct RClass *TECS)	\
	{																	\
		struct RClass *a;												\
																		\
		a = mrb_define_class_under(mrb, TECS, #Type "Pointer", mrb->object_class); \
		MRB_SET_INSTANCE_TT(a, MRB_TT_DATA);							\
																		\
		mrb_define_method(mrb, a, "initialize",      Type ## Pointer_initialize,   MRB_ARGS_REQ(1)); \
		mrb_define_method(mrb, a, "[]",              Type ## Pointer_aget,         MRB_ARGS_REQ(1)); \
		mrb_define_method(mrb, a, "value",           Type ## Pointer_get_val,      MRB_ARGS_NONE()); \
		mrb_define_method(mrb, a, "[]=",             Type ## Pointer_aset,         MRB_ARGS_REQ(2)); \
		mrb_define_method(mrb, a, "value=",          Type ## Pointer_set_val,      MRB_ARGS_REQ(1)); \
		mrb_define_method(mrb, a, "size",            Type ## Pointer_size,         MRB_ARGS_NONE()); \
		mrb_define_method(mrb, a, "length",          Type ## Pointer_size,         MRB_ARGS_NONE()); \
		return	a;														\
	}

static inline mrb_value
CharPointer_to_s( mrb_state *mrb, mrb_value self )
{
	struct CharPointerBody *pointerBody = DATA_PTR( self );
	return mrb_str_new( mrb, pointerBody->buf, strnlen( pointerBody->buf, pointerBody->length ) );
}

static inline mrb_value
CharPointer_from_s( mrb_state *mrb, mrb_value self )
{
	mrb_value str;
	struct CharPointerBody *pointerBody = DATA_PTR( self );

	mrb_get_args(mrb, "o", &str );
	if( mrb_type( str ) != MRB_TT_STRING ){
		mrb_raise(mrb, E_ARGUMENT_ERROR, "Not String");
	}

	strncpy( pointerBody->buf, RSTRING_PTR(str), pointerBody->length );
	return self;
}

/* Initialize TECSPointer classes */
void	init_TECSPointer( mrb_state *mrb, struct RClass *TECS );

#endif /* TECSPointer_h__ */

