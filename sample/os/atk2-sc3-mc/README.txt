
		TOPPERS/ATK2-SC3-MC（Release 1.4.0）
        ＜ATK2-SC3-MC Readmeファイル＞

本ファイルでは，ATK2-SC3-MC を使用する上で必要な情報を紹介します．

----------------------------------------------------------------------
TOPPERS ATK2
    Toyohashi Open Platform for Embedded Real-Time Systems
    Automotive Kernel Version 2

Copyright (C) 2011-2015 by Center for Embedded Computing Systems
            Graduate School of Information Science, Nagoya Univ., JAPAN
Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
Copyright (C) 2011-2013 by Spansion LLC, USA
Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
Copyright (C) 2011-2015 by Witz Corporation
Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
Copyright (C) 2015 by SUZUKI MOTOR CORPORATION

上記著作権者は，以下の (1)〜(3)の条件を満たす場合に限り，本ドキュメ
ント（本ドキュメントを改変したものを含む．以下同じ）を使用・複製・改
変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
(1) 本ドキュメントを利用する場合には，上記の著作権表示，この利用条件
    および下記の無保証規定が，そのままの形でドキュメント中に含まれて
    いること．
(2) 本ドキュメントを改変する場合には，ドキュメントを改変した旨の記述
    を，改変後のドキュメント中に含めること．ただし，改変後のドキュメ
    ントが，TOPPERSプロジェクト指定の開発成果物である場合には，この限
    りではない．
(3) 本ドキュメントの利用により直接的または間接的に生じるいかなる損害
    からも，上記著作権者およびTOPPERSプロジェクトを免責すること．また，
    本ドキュメントのユーザまたはエンドユーザからのいかなる理由に基づ
    く請求からも，上記著作権者およびTOPPERSプロジェクトを免責すること．

本ドキュメントは，AUTOSAR（AUTomotive Open System ARchitecture）仕様
に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するものではな
い．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利用する
者に対して，AUTOSARパートナーになることを求めている．

本ドキュメントは，無保証で提供されているものである．上記著作権者およ
びTOPPERSプロジェクトは，本ドキュメントに関して，特定の使用目的に対す
る適合性も含めて，いかなる保証も行わない．また，本ドキュメントの利用
により直接的または間接的に生じたいかなる損害に関しても，その責任を負
わない．

$Id: README.txt 503 2015-12-23 07:50:20Z ertl-ishikawa $
----------------------------------------------------------------------

ATK2-SC3-MCは，「AUTOSAR R4.0 Rev 3」仕様に準拠した，スケーラビリティ
クラス3の機能を実装したマルチコア用リアルタイムカーネルです．また，ス
ケーラビリティクラス3については独自に定義した機能レベルにおける，レベ
ル2を採用しています（機能レベルについては「次世代車載システム向けRTOS
外部仕様書」を参照ください)．

【ジェネレータのバージョンに関する注意】

ATK2-SC3-MC Release 1.4.0以降では，TOPPERS新世代カーネル用コンフィギュ
レータ（cfg）(ジェネレータと呼ぶ)の最新版（1.9.5）の機能を用いている．
1.9.4以前のバージョンのジェネレータでは動作しないので注意すること．

【最初に読むべきドキュメント】

ATK2-SC3-MCのユーザーズマニュアルが，doc/user.txtにあります．ATK2-SC3
-MCを使用する場合には，まずはこのドキュメントからお読み下さい．

【ファイルの閲覧にあたって】

ATK2-SC3-MCのドキュメント（プレーンテキストファイル）およびソースファ
イルを読む際には，TABを4に設定してください．

【利用条件】

ATK2-SC3-MCの利用条件は，各ファイルの先頭に表示されているTOPPERSライ
センスです．TOPPERSライセンスに関するFAQが，以下のページにあります．

    http://www.toppers.jp/faq/faq_ct12.html

【質問・バグレポート・意見等の送付先】

ATK2-SC3-MCをより良いものにするためのご意見等を歓迎します．
ATK2-SC3-MCに関する質問やバグレポート，ご意見等は，TOPPERSプロジェクト
の会員はTOPPERS開発者メーリングリスト（dev@toppers.jp）宛に，その他の
方はTOPPERSユーザーズメーリングリスト（users@toppers.jp）宛にお願いし
ます．

TOPPERSユーザーズメーリングリストへの登録方法については，以下のページに
説明があります．

    http://www.toppers.jp/community.html

【ポーティングにあたって】

ATK2-SC3-MCを，TOPPERSプロジェクトから公開することを前提に，未サポート
のターゲットにポーティングされる場合には，あらかじめご相談くださると幸
いです．

以上
