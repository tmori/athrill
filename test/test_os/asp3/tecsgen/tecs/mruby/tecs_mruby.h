/*
 *  Copyright (C) 2008-2017 by TOPPERS Project
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  @(#) $Id: tecs_mruby.h 2640 2017-06-03 11:27:12Z okuma-top $
 */

#ifndef tecs_mruby_h__
#define tecs_mruby_h__

#ifndef TECSGEN

// tecsgen doesn't include actual mruby.h
#include "mruby.h"
#include "mruby/class.h"
#include "mruby/data.h"
#include "mruby/string.h"
#include "mruby/irep.h"
#include "mruby/dump.h"

#include "TECSPointer.h"
#include "TECSStruct.h"

#if  ! defined( MRUBY_RELEASE_MAJOR ) || MRUBY_RELEASE_MAJOR == 1 && MRUBY_RELEASE_MINOR < 2
#ifndef MRB_ARGS_REQ
#define MRB_ARGS_REQ(n)     ARGS_REQ(n)
#define MRB_ARGS_OPT(n)     ARGS_OPT(n)
#define MRB_ARGS_ARG(n1,n2) ARGS_ARG(n1,n2)
#define MRB_ARGS_REST()     ARGS_REST()
#define MRB_ARGS_POST(n)    ARGS_POST(n)
#define MRB_ARGS_KEY(n1,n2) ARGS_KEY(n1,n2)
#define MRB_ARGS_BLOCK()    ARGS_BLOCK()
#define MRB_ARGS_ANY()      ARGS_ANY()
#define MRB_ARGS_NONE()     ARGS_NONE()
#endif /* MRB_ARGS_REQ */
#endif

#if  ! defined( MRUBY_RELEASE_MAJOR )
#define mrb_float_value( mrb, val )  mrb_float_value( val )
#endif

#else

/*
 * fake tecsgen because tecsgen cannot accept actual mruby.h in case of below.
 *   types:   long long, long long int
 *   special keyword __attribute__(x), __extension__
 */
typedef int mrb_state;
typedef int mrb_irep;
typedef int mrb_context;
struct  RClass {int dummy;};
struct  RProc  {int dummy;};

typedef int CELLCB;

#endif /* TECSGEN */

#endif /* tecs_mruby_h__ */
