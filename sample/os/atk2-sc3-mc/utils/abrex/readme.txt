
	ABREX - AUTOSAR BSW and RTE XML Generator -

本ドキュメントは，ABREXを使用するために必要な事項を説明するものである．

----------------------------------------------------------------------
ABREX
    AUTOSAR BSW and RTE XML Generator

Copyright (C) 2013-2015 by Center for Embedded Computing Systems
            Graduate School of Information Science, Nagoya Univ., JAPAN
Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
Copyright (C) 2013-2015 by FUJI SOFT INCORPORATED, JAPAN
Copyright (C) 2014-2015 by NEC Communication Systems, Ltd., JAPAN
Copyright (C) 2013-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
Copyright (C) 2013-2014 by Renesas Electronics Corporation, JAPAN
Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
Copyright (C) 2013-2015 by Sunny Giken Inc., JAPAN
Copyright (C) 2013-2015 by TOSHIBA CORPORATION, JAPAN
Copyright (C) 2013-2015 by Witz Corporation
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

$Id: readme.txt 571 2015-12-21 05:01:59Z t_ishikawa $
----------------------------------------------------------------------

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
(1) ABREX概要
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ABREXは，OS，COMを始めとするBSWと，RTEのコンフィギュレーションファイル
(XML)を生成するツールである．YAMLフォーマットで記述したコンフィギュレー
ション情報を入力として，対応するXMLを生成する．

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
(2) 使い方
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
———————————————————————————————————
(2.1) 動作環境
———————————————————————————————————

ABREXはRubyによって記述されているため，Rubyの実行環境が必要である．
Cygwinに含まれる以下のバージョンのRubyで動作確認済みである．

ruby 1.9.3p327 (2012-11-10 revision 37606) [i386-cygwin]

———————————————————————————————————
(2.2) コンテナ情報の生成
———————————————————————————————————
※本手順は既にparam_info.yamlが存在する場合，不要である

-pオプションに，AUTOSARから公開されているECUコンフィギュレーションパラ
メータコンテナ情報ファイル(*)を引数として与え，実行する．
(*)http://www.autosar.org
   Methodology and Templates -> Templates -> Standard Specifications
    -> AUTOSAR_MOD_ECUConfigurationParameters.zip
   解凍すると"AUTOSAR_MOD_ECUConfigurationParameters.arxml"となる

$ ruby abrex.rb -p AUTOSAR_MOD_ECUConfigurationParameters.arxml

各パラメータコンテナのデータ型，外部参照型の参照先情報などが，
param_info.yamlに出力される．

———————————————————————————————————
(2.3) YAMLファイルの作成
———————————————————————————————————
生成したいXMLファイルの情報をYAMLフォーマットで記述する．
・最上位のレイヤがパッケージ名，その次がECUモジュール名となる．
・ECUモジュール名の下に該当モジュールに含まれるパラメータ名をキーとして，
  パラメータコンテナと値をハッシュ形式で記述する．
・サブコンテナは，ハッシュをネストすることで表現する．
・コンテナへの参照は，YAMLファイルに定義したパッケージ名に従ってパスを
  記述する．

■OSの例

Os:
  Os:
    MainApp:
      DefinitionRef: OsAppMode
    OsOS:
      OsStackMonitoring: 1
      OsStatus: EXTENDED
      OsUseGetServiceId: 1
      OsUseParameterAccess: 1
      OsScalabilityClass: SC1
      OsHooks:
        OsErrorHook: 0
        OsPostTaskHook: 0
        OsPreTaskHook: 0
        OsProtectionHook: 0
        OsShutdownHook: 0
        OsStartupHook: 0
      OsHookStack:
        OsHookStackSize: 1024
      OsOsStack:
        OsOsStackSize: 1024
    TASK1:
      DefinitionRef: OsTask
      OsTaskActivation: 1
      OsTaskPriority: 10
      OsTaskSchedule: FULL
      OsTaskStackSize: 1024
      OsTaskEventRef: /Os/Os/EVENT1
      OsTaskAutostart:
        OsTaskAppModeRef: /Os/Os/MainApp
    EVENT1:
      DefinitionRef: OsEvent

———————————————————————————————————
(2.4) XMLファイルの生成
———————————————————————————————————
作成したYAMLファイルを引数として，abrex.rbを実行する．

$ ruby abrex.rb ./sample.yaml

入力したYAMLファイルの情報に対応するXMLファイルが，入力ファイル名の拡張
子をarxmlに変更したファイルに出力される．
※上記例の場合，sample.arxml

・(2.1)で生成したparam_info.yamlに含まれないキー名が登場した場合，エラ
  ー終了する．
・サブコンテナ名はparam_info.yamlに含まれず，YAMLファイルに記述した名称
  が正しいものとして，サブコンテナを生成する．
・設定値の妥当性等のチェックは一切行わない．

引数のYAMLファイルは複数指定することができ，すべてのYAMLファイルの情報
をマージしたXMLファイルを生成する．同じパスのコンテナが異なるファイルに
存在する場合，1つのコンテナにマージする．例えば，以下のa.yamlとb.yamlを
入力した場合，MAIN_HW_COUNTERには両方のファイルに指定されたすべてのパラ
メータが設定されたXMLファイルとなる．

＜a.yaml＞
Os:
  Os:
    MAIN_HW_COUNTER:
      DefinitionRef: OsCounter
      OsCounterMaxAllowedValue: 0x7FFFFFFF
      OsCounterTicksPerBase: 10
      OsCounterMinCycle: 4000
      OsCounterType: HARDWARE
      OsSecondsPerTick: 1.666666e-08
      OsCounterIsrRef: /Os/Os/ISR1

＜b.yaml＞
Os:
  Os:
    MAIN_HW_COUNTER:
      OsCounterAccessingApplication: OSAP1

※同じパスのパラメータが複数のファイルに含まれる場合は，多重度が*と判断
  して，複数のコンテナを生成する．(パラメータ毎の多重度情報は保持しない)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
(3) その他の機能
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
———————————————————————————————————
(3.1) XMLからYAMLを作成する(逆変換)
———————————————————————————————————
ABREXでは，作成済みのXMLファイルから，YAMLファイルを作成することが出来
る．-iオプションに，AUTOSAR準拠のXMLを引数として与え，実行する．

$ ruby abrex.rb -i sample.arxml

入力したXMLファイルの情報に対応するYAMLファイルが，入力ファイル名の拡張
子をyamlに変更したファイルに出力される．
※上記例の場合，sample.yaml

———————————————————————————————————
(3.2) ジェネレータ用csvファイルの生成
———————————————————————————————————
TOPPERS/ATK2で使用するジェネレータ(cfg.exe)はXMLコンテナ情報をCSVファイ
ルで与える必要があるが，ECUコンフィギュレーションパラメータコンテナ情報
ファイルから生成することが可能である．-cオプションに，ECUコンフィギュレ
ーションパラメータコンテナ情報ファイルを引数として与え，-bオプションに，
生成する対象のモジュール名(*)を引数として与え，実行する．

(*)http://www.autosar.org
   Software Architecture -> General -> AUTOSAR_TR_BSWModuleList.pdf
   上記ファイルに規定されているModule abbreviationで指定する．

$ ruby abrex.rb -b Com -c AUTOSAR_MOD_ECUConfigurationParameters.arxml

指定した対象のモジュール名のCSVファイルに出力される．
※上記例の場合，Com.csv

＜注意事項＞
生成するCSVファイルのコンテナの短縮名は，コンテナ名をそのまま使用する．
従って，同じ名称のコンテナが異なるコンテナのサブコンテナとして存在する
場合は，ジェネレータ実行時に区別できなくなる．この場合，手動で区別でき
るように修正する必要がある．

(例)ComのComTxModeFalseとComTxModeTrueに含まれるComTxModeコンテナ
/AUTOSAR/EcucDefs/Com/ComConfig/ComIPdu/ComTxIPdu/ComTxModeFalse/ComTxMode,ComTxMode,,1
/AUTOSAR/EcucDefs/Com/ComConfig/ComIPdu/ComTxIPdu/ComTxModeTrue/ComTxMode,ComTxMode,,1

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
(4) 注意事項
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
———————————————————————————————————
(4.1) ECUコンフィギュレーションパラメータコンテナ情報ファイルの誤記修正
———————————————————————————————————
コンテナ情報の生成(本ドキュメントの2.2章を参照)において使用している，
ECUコンフィギュレーションパラメータコンテナ情報ファイルである
AUTOSAR_MOD_ECUConfigurationParameters.arxmlの誤記を修正する必要がある．

現在判明している誤記を以下に示す．

・(誤)WdgMInternallCheckpointFinalRef
  (正)WdgMInternalCheckpointFinalRef


以上
