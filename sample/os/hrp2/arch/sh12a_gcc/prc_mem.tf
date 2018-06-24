$ 
$ 		パス4のプロセッサ依存テンプレート（SH12A用）
$ 

$ 
$  スタック領域の先頭番地を求める関数
$ 　　アーキテクチャ依存部でタスク初期化ブロックのデータ構造を
$ 　　独自に定義しているので、非依存部からこの関数が呼び出される。
$ 
$ 　　引数：タスク初期化ブロック（TINIB）の先頭番地
$ 
$FUNCTION GET_STK_TSKINICTXB$
	$p_tinib = ARGV[1]$
	$stksz = PEEK(p_tinib + offsetof_TINIB_TSKINICTXB_stksz, sizeof_SIZE)$
	$stk_bottom = PEEK(p_tinib + offsetof_TINIB_TSKINICTXB_stk_bottom, sizeof_void_ptr)$
	$RESULT = stk_bottom - stksz$
$END$


$ 
$  標準テンプレートファイルのインクルード
$ 
$INCLUDE "kernel/kernel_mem.tf"$
