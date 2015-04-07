#include <stdio.h>

#define FIELDX 16 
#define FIELDY 16 
int maze[FIELDY][FIELDX];//迷路の真の壁情報

void ReadMaze(char *filename){//ファイルから迷路の真の壁情報を記録する
	FILE *fp;
	signed int i,t;
	fp=fopen(filename,"r");
	if(fp==NULL){
		puts("cantopenfile");
	}else{
		for(i = 0;i <FIELDY ;i++){
			for(t = 0;t < FIELDX;t++){
				fscanf(fp,"%x",&maze[i][t]);
				maze[i][t]|=0xf0;
			}
			fscanf(fp,"\n");
		}
		fclose(fp);
		puts("load done");
	}
}

void printmaze(int mousex,int mousey,int moused,int nowmaze[FIELDY][FIELDX]){
	char str[4][3]={"北","東","南","西"};
	char str2[5]="^>v<";
	int i,t;
	printf("mouseの位置:x%d y%d mouseの向き:%s\n",mousex,mousey,str[moused]);
	for(i = 0;i <FIELDY ;i++){
		//壁の上側表示
		for(t = 0;t < FIELDX;t++){
			if((nowmaze[i][t]&0x10)==0)
				printf("*?");
			else
				if((nowmaze[i][t]&0x01)==0)
					printf("* ");
				else
					printf("*-");
		}
		printf("*   ");
		for(t = 0;t < FIELDX;t++){
			if((maze[i][t]&0x10)==0)
				printf("*?");
			else
				if((maze[i][t]&0x01)==0)
					printf("* ");
				else
					printf("*-");
		}
		puts("*");

		for(t = 0;t < FIELDX;t++){
			if((nowmaze[i][t]&0x80)==0)
				printf("?",nowmaze[i][t]);
			else
				if((nowmaze[i][t]&0x08)==0)
					printf(" ");
				else
					printf("|");
			if(i==mousey && t==mousex)
				printf("%c",str2[moused]);
			else
				printf(" ");
		}
		if((nowmaze[i][FIELDX-1]&0x20)==0)
			printf("?",nowmaze[i][FIELDX-1]);
		else
			if((nowmaze[i][FIELDX-1]&0x02)==0)
				printf(" ");
			else
				printf("|");
		printf("   ");
		for(t = 0;t < FIELDX;t++){
			if((maze[i][t]&0x08)==0)printf(" ");
			else printf("|");
			if(i==mousey && t==mousex)printf("%c",str2[moused]);
			else printf(" ");
		}
		if((maze[i][FIELDX-1]&0x02)==0)printf(" \n");
		else printf("|\n");
	}
	for(t = 0;t < FIELDX;t++){
		if((nowmaze[FIELDY-1][t]&0x40)==0)
			printf("*?");
		else
			if((nowmaze[FIELDY-1][t]&0x04)==0)
				printf("* ");
			else
				printf("*-");
	}
	printf("*   ");
	for(t = 0;t < FIELDX;t++){
		if((maze[FIELDY-1][t]&0x40)==0)
			printf("*?");
		else
			if((maze[FIELDY-1][t]&0x04)==0)
				printf("* ");
			else
				printf("*-");
	}
	puts("*");
}
int senscheckfront(int mousex,int mousey,int moused){
	int mask;
	mask=1<<moused;
	if((maze[mousey][mousex]&mask)==mask)return 1;
	return 0;
}
int senscheckright(int mousex,int mousey,int moused){
	int mask;
	mask=1<<((moused+1)%4);
	if((maze[mousey][mousex]&mask)==mask)return 1;
	return 0;
}
int senscheckleft(int mousex,int mousey,int moused){
	int mask;
	mask=1<<((moused+3)%4);
	if((maze[mousey][mousex]&mask)==mask)return 1;
	return 0;
}
void set(int mousex,int mousey,int d,int nowmaze[FIELDY][FIELDX]){
	switch(d){
		case 0:nowmaze[mousey][mousex]|=0x11;
           if(mousey>0) nowmaze[mousey-1][mousex]|=0x44; break;
		case 1:nowmaze[mousey][mousex]|=0x22;
           if(mousex<FIELDX-1)nowmaze[mousey][mousex+1]|=0x88; break;
		case 2:nowmaze[mousey][mousex]|=0x44;
           if(mousey<FIELDY-1)nowmaze[mousey+1][mousex]|=0x11; break;
		case 3:nowmaze[mousey][mousex]|=0x88;
           if(mousex>0)nowmaze[mousey][mousex-1]|=0x22;break;
	}
}
void unset(int mousex,int mousey,int d,int nowmaze[FIELDY][FIELDX]){
	switch(d){
		case 0:nowmaze[mousey][mousex]|=0x10;
           nowmaze[mousey][mousex]&=0xfe;
           if(mousey>0){
             nowmaze[mousey-1][mousex]|=0x40;
             nowmaze[mousey-1][mousex]&=0xfb;
           }break;
		case 1:nowmaze[mousey][mousex]|=0x20;
          nowmaze[mousey][mousex]&=0xfd;
          if(mousex<FIELDX-1){
            nowmaze[mousey][mousex+1]|=0x80;
            nowmaze[mousey][mousex+1]&=0xf7;
          }break;
		case 2:nowmaze[mousey][mousex]|=0x40;
           nowmaze[mousey][mousex]&=0xfb;
           if(mousey<FIELDY-1){
            nowmaze[mousey+1][mousex]|=0x10;
            nowmaze[mousey+1][mousex]&=0xfe;
           }break;
		case 3:nowmaze[mousey][mousex]|=0x80;
            nowmaze[mousey][mousex]&=0xf7;
            if(mousex>0       ){
            nowmaze[mousey][mousex-1]|=0x20;
            nowmaze[mousey][mousex-1]&=0xfd;
            }break;
	}
}
void readsensor(int mousex,int mousey,int moused,int nowmaze[FIELDY][FIELDX]){
	int c=0,l=0,r=0;
	switch(moused){
		case 0:c=0;r=1;l=3;break;
		case 1:c=1;r=2;l=0;break;
		case 2:c=2;r=3;l=1;break;
		case 3:c=3;r=0;l=2;break;
	}
 	if(senscheckfront(mousex,mousey,moused)){
    set(mousex,mousey,c,nowmaze);
  }else {
    unset(mousex,mousey,c,nowmaze);
  }//if(senscheckfront())部分を,if(前センサーの値>壁があるかないか判断するための基準値）と書き換えればこの関数がそのまま実機実装に流用できる
	if(senscheckright(mousex,mousey,moused)){
    set(mousex,mousey,r,nowmaze);
  }else {
    unset(mousex,mousey,r,nowmaze);
  }
	if(senscheckleft(mousex,mousey,moused) ){
    set(mousex,mousey,l,nowmaze);
  }else {
    unset(mousex,mousey,l,nowmaze);
  }
}

//足立法で使うポテンシャル場を作成する
int makepotential(int startx, int starty, int goalx, int goaly, int potential[FIELDY][FIELDX],int nowmaze[FIELDY][FIELDX]){
	int i,t,j;
	int foundflag;
	//初期化
	for(i = 0;i < FIELDY;i++)
		for(t = 0;t < FIELDX;t++)
			potential[i][t] = FIELDX*FIELDY + 1;//∞を初期コストとして与えておく．
	potential[goaly][goalx] = 0;
	//ポテンシャルマップ作成
	for(j = 0;j < FIELDX*FIELDY; j++){
		foundflag=1;
		for(i = 0;i < FIELDY; i++){//ポテンシャルがjであるマス（探索点）を探す
			for(t = 0;t < FIELDX; t++){
				if(potential[i][t] == j){
					foundflag=0;
					//東西南北を探す
					if(i > 0)//北側
						if((nowmaze[i][t]&0x01) == 0 || (nowmaze[i][t]&0x10) == 0)//北に移動可能(壁がないか，未探索であれば移動可能)
							if(potential[i][t] + 1 < potential[i-1][t])
								potential[i-1][t] = potential[i][t] + 1;
					if(t < FIELDX-1)
						if((nowmaze[i][t]&0x02) == 0 || (nowmaze[i][t]&0x20) == 0)//東に移動可能(壁がないか，未探索であれば移動可能)
							if(potential[i][t] + 1 < potential[i][t+1])
								potential[i][t+1] = potential[i][t] + 1;
					if(i < FIELDY-1)
						if((nowmaze[i][t]&0x04) == 0 || (nowmaze[i][t]&0x40) == 0)//南に移動可能(壁がないか，未探索であれば移動可能)
							if(potential[i][t] + 1 < potential[i+1][t])
								potential[i+1][t] = potential[i][t] + 1;
					if(t > 0)
						if((nowmaze[i][t]&0x08) == 0 || (nowmaze[i][t]&0x80) == 0)//西に移動可能(壁がないか，未探索であれば移動可能)
							if(potential[i][t] + 1 < potential[i][t-1])
								potential[i][t-1] = potential[i][t] + 1;
					if(i == starty && t == startx)//スタートが見つかった
						return 1;
				}
			}
		}
		if(foundflag)break;//スタートが見つからなかった
	}
	return 0;
}

//四方のマスの中で一番値が小さいものを探す(0:北,1:東,2:南,3:西)
int decidetodirec(int mousex,int mousey,int potential[FIELDY][FIELDX],int nowmaze[FIELDY][FIELDX]){
	int neighborhood[4];//例えば，neighboorhood[0]には，もし北向きに移動できる場合は北隣のポテンシャル値を代入する．移動できない場合には十分大きな値を代入する．
	int i;//for文で使う汎用変数
	int minnum,minindex;
	for(i = 0;i < 4;i++)
		neighborhood[i]=FIELDY*FIELDX+1;//十分大きな値を先に代入しておく
	if(mousey>0)//各種チェックの前に，枠外チェックを独立して行うとバグをおこしにくい
		if((nowmaze[mousey][mousex]&0x01)!=0x01)//北隣に壁があるかチェック
			neighborhood[0]=potential[mousey-1][mousex];
	if(mousex<FIELDX-1)//各種チェックの前に，枠外チェックを独立して行うとバグをおこしにくい
		if((nowmaze[mousey][mousex]&0x02)!=0x02)//東隣に壁があるかチェック
			neighborhood[1]=potential[mousey][mousex+1];
	if(mousey<FIELDY-1)//各種チェックの前に，枠外チェックを独立して行うとバグをおこしにくい
		if((nowmaze[mousey][mousex]&0x04)!=0x04)//南隣に壁があるかチェック
			neighborhood[2]=potential[mousey+1][mousex];
	if(mousex>0)//各種チェックの前に，枠外チェックを独立して行うとバグをおこしにくい
		if((nowmaze[mousey][mousex]&0x08)!=0x08)//西隣に壁があるかチェック
			neighborhood[3]=potential[mousey][mousex-1];
	//neighborhood[0]-neighborhood[3]の中で最小のものを探す
	minnum=neighborhood[0];
	minindex=0;
	for(i = 1;i < 4;i++){
		if(neighborhood[i]<minnum){
			minnum=neighborhood[i];
			minindex=i;
		}
	}
	return minindex;
}
//指定したマスに動くためのコマンドを決定(指定されたマスに進むには，どの向きに回転すればいいか？）
int decidecommand(int todirec,int moused){
	if( moused     ==todirec)return 0;//回転しなくても指定されたマスの向きになっている
	if((moused+3)%4==todirec)return 1;//左９０度ターンすれば指定されたマスの向きになる
	if((moused+1)%4==todirec)return 2;//右９０度ターンすれば指定されたマスの向きになる
	return 3;                         //１８０度ターンすれば指定されたマスの向きになる
}
main(){
	int potential[FIELDY][FIELDX];//ポテンシャル値は最悪の場合（うずまき迷路が最悪），FIELDY*FIELDXの値を保持する必要がある．2byteの変数をつかったほうがよい．
	int nowmaze[FIELDY][FIELDX];//マウスが覚えている迷路情報
	int i,t;//for文などで使う汎用変数
	int command;//0:１マス進む 1:左９０度ターンしてから１マス進む,2:右90度ターンしてから１マス進む,3:１８０度ターンしてから1マス進む
	int todirec;//0:北のマスへ進む,1:東のマスへ進む,2:南のマスに進む,3:西のマスに進む
	int mousex,mousey;//マウスの現在位置，左上のマスをmousex=0,mousey=0とする
	int moused;//マウスの現在の方向，北向きを0,東向きを1,南向きを2,西向きを3とする．

	ReadMaze("maze.dat");//迷路の真の情報を保持する
	//初期化:ロボットは最初北向きで左下にいる
	mousex=0;
	mousey=FIELDX-1;
	moused=0;
	//初期化:最初ロボットは迷路情報を何も知らない
	for(i = 0;i < FIELDY;i++)
		for(t = 0;t < FIELDX;t++)
			nowmaze[i][t]=0x00;//16進法表示で0を意味する
	while(!(mousex==7 && mousey==FIELDY-8)){//足立法ループ：マウスがゴールに到着するまで無限ループを行う
		printmaze(mousex,mousey,moused,nowmaze);//現在状態をターミナル（コマンドプロンプト）に表示
		getchar();//キーボードから入力があるまで待つ
		readsensor(mousex,mousey,moused,nowmaze);//センサー値を読んで迷路情報を更新する関数（実機に実装する場合はこの関数を書き換えてください
		makepotential(mousex, mousey, 7, FIELDY-8,potential,nowmaze);//更新された迷路情報を用いてポテンシャル値を計算
		todirec=decidetodirec(mousex,mousey,potential,nowmaze);//四方のマスの中で一番値が小さいものを探す(0:北,1:東,2:南,3:西)
		command=decidecommand(todirec,moused);//指定したマスに動くためのコマンドを決定(指定されたマスに進むには，どの向きに回転すればいいか？）
		switch(command){//指定したコマンドの通り動く
			case 0:break;//ターンしない
			case 1:moused+=3;moused%=4;/*マウスをその場左９０度ターンする関数*/break;
			case 2:moused+=1;moused%=4;/*マウスをその場右９０度ターンする関数*/break;
			case 3:moused+=2;moused%=4;/*マウスをその場１８０度ターンする関数*/break;
		}
		switch(moused){//１マス進む
			case 0:mousey--;/*マウスをその場から１マス進ませる関数*/break;//マウスが現在北向きの場合
			case 1:mousex++;/*マウスをその場から１マス進ませる関数*/break;//マウスが現在東向きの場合
			case 2:mousey++;/*マウスをその場から１マス進ませる関数*/break;//マウスが現在南向きの場合
			case 3:mousex--;/*マウスをその場から１マス進ませる関数*/break;//マウスが現在西向きの場合
		}
	}
	puts("マウスがゴールにつきました．");
	return 0;
}
