
		TOPPERS/ATK2-SC1（Release 1.4.2）
        ＜ATK2-SC1 Readmeファイル＞

本ファイルでは，ATK2-SC1 を使用する上で必要な情報を紹介します．

----------------------------------------------------------------------
TOPPERS ATK2
    Toyohashi Open Platform for Embedded Real-Time Systems
    Automotive Kernel Version 2

Copyright (C) 2011-2017 by Center for Embedded Computing Systems
            Graduate School of Information Science, Nagoya Univ., JAPAN
Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
Copyright (C) 2011-2013 by Spansion LLC, USA
Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
Copyright (C) 2011-2017 by Witz Corporation
Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION

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

$Id: README.txt 761 2017-03-01 07:28:00Z witz-itoyo $
----------------------------------------------------------------------

ATK2-SC1は，「AUTOSAR R4.0 Rev 3」仕様に準拠した，スケーラビリティク
ラス1の機能を実装したリアルタイムカーネルです．

【ジェネレータのバージョンに関する注意】

ATK2-SC1 Release 1.4.0以降では，TOPPERS新世代カーネル用コンフィギュレー
タ（cfg）(ジェネレータと呼ぶ)の最新版（1.9.6）の機能を用いている．
1.9.4以前のバージョンでは動作しないので注意すること．

【最初に読むべきドキュメント】

ATK2-SC1のユーザーズマニュアルが，doc/user.txtにあります．ATK2-SC1を
使用する場合には，まずはこのドキュメントからお読み下さい．

【ファイルの閲覧にあたって】

ATK2-SC1のドキュメント（プレーンテキストファイル）およびソースファイル
を読む際には，TABを4に設定してください．

【利用条件】

ATK2-SC1の利用条件は，各ファイルの先頭に表示されているTOPPERSライセン
スです．TOPPERSライセンスに関するFAQが，以下のページにあります．

    http://www.toppers.jp/faq/faq_ct12.html

【質問・バグレポート・意見等の送付先】

ATK2-SC1をより良いものにするためのご意見等を歓迎します．ATK2-SC1に関
する質問やバグレポート，ご意見等は，TOPPERSプロジェクトの会員はTOPPERS
開発者メーリングリスト（dev@toppers.jp）宛に，その他の方はTOPPERSユー
ザーズメーリングリスト（users@toppers.jp）宛にお願いします．

TOPPERSユーザーズメーリングリストへの登録方法については，以下のページ
に説明があります．

    http://www.toppers.jp/community.html

【ポーティングにあたって】

ATK2-SC1を，TOPPERSプロジェクトから公開することを前提に，未サポートの
ターゲットにポーティングされる場合には，あらかじめご相談くださると幸い
です．

以上
