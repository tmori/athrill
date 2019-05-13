#ifndef _STDLIBS_H_
#define _STDLIBS_H_

#define clr()				printf("\033[2J") //画面クリア
#define clr_right()			printf("\033[0K") //カーソル位置からその行の右端までをクリア
#define clr_left()			printf("\033[1K") //カーソル位置からその行の左端までをクリア
#define clr_line()			printf("\033[2K") //カーソル位置の行をクリア
#define location(x, y)		printf("\033[%d;%dH" ,x,y) //カーソル位置を移動
#define right(x)			printf("\033[%dC" ,x) //カーソルを指定数だけ右に移動
#define left(x)				printf("\033[%dD" ,x) //カーソルを指定数だけ左に移動
#define down(x)				printf("\033[%dB" ,x) //カーソルを指定数だけ下に移動
#define up(x)				printf("\033[%dA" ,x) //カーソルを指定数だけ上に移動

#endif /* _STDLIBS_H_ */