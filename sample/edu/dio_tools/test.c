#include <stdio.h>
#include <unistd.h>
#include "tool_config.h"

#define clr()				printf("\033[2J") //画面クリア
#define clr_right()			printf("\033[0K") //カーソル位置からその行の右端までをクリア
#define clr_left()			printf("\033[1K") //カーソル位置からその行の左端までをクリア
#define clr_line()			printf("\033[2K") //カーソル位置の行をクリア
#define location(x, y)		printf("\033[%d;%dH" ,x,y) //カーソル位置を移動
#define right(x)			printf("\033[%dC" ,x) //カーソルを指定数だけ右に移動
#define left(x)				printf("\033[%dD" ,x) //カーソルを指定数だけ左に移動
#define down(x)				printf("\033[%dB" ,x) //カーソルを指定数だけ下に移動
#define up(x)				printf("\033[%dA" ,x) //カーソルを指定数だけ上に移動

int main(void)
{
#if 0
	int i;

	clr();//画面クリア
	for(i=0;i<100;i++){
		location(10,20);//カーソル位置をheight 10 width 20に移動
		printf("%d",i);
		fflush(stdout);//バッファをフラッシュ
		sleep(1);
	}
#endif
	ToolConfigType config;

	ToolReturnType err = tool_config_load("digital.config",  &config);
	printf("err = %d\n", err);

	printf("filepath=%s\n", config.filepath);
	printf("digital_num=%d\n", config.digital_num);
	uint32 i;
	for (i = 0; i < config.digital_num; i++) {
		printf("file=%s\n", config.digital_configs[i].param[DIO_CONFIG_PARAM_INDEX_FPATH]);
		printf("name=%s\n", config.digital_configs[i].param[DIO_CONFIG_PARAM_INDEX_NAME]);
		printf("foff=%s\n", config.digital_configs[i].param[DIO_CONFIG_PARAM_INDEX_FOFF]);
		printf("foff=%d\n", config.digital_configs[i].file_offset);
		printf("boff=%s\n", config.digital_configs[i].param[DIO_CONFIG_PARAM_INDEX_BOFF]);
		printf("boff=%d\n", config.digital_configs[i].bit_offset);
	}
	return (0);
}
