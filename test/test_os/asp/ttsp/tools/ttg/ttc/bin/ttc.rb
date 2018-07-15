#!ruby -Ke
#
#  TTG
#      TOPPERS Test Generator
#
#  Copyright (C) 2009-2012 by Center for Embedded Computing Systems
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#  Copyright (C) 2010-2011 by Graduate School of Information Science,
#                             Aichi Prefectural Univ., JAPAN
#
#  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
#  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
#  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
#  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
#      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
#      スコード中に含まれていること．
#  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
#      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
#      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
#      の無保証規定を掲載すること．
#  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
#      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
#      と．
#    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
#        作権表示，この利用条件および下記の無保証規定を掲載すること．
#    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
#        報告すること．
#  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
#      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
#      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
#      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
#      免責すること．
#
#  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#  の責任を負わない．
#
#  $Id: ttc.rb 6 2012-09-03 11:06:01Z nces-shigihara $
#
if ($0 == __FILE__)
  TOOL_ROOT = File.expand_path(File.dirname(__FILE__) + "/../../")
  $LOAD_PATH.unshift(TOOL_ROOT)
end
require "optparse"
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/test_scenario/TestScenario.rb"
require "ttc/bin/class/TTCCommon.rb"
require "ttc/bin/class/MultipleChecker.rb"

#=====================================================================
# TTCModule
#=====================================================================
module TTCModule
  #===================================================================
  # クラス名: TTC
  # 概　  要: TTCメインクラス
  #===================================================================
  class TTC
    include CommonModule
    include TTCModule
    attr_reader :hErrors, :hExclude

    #=================================================================
    # 概　要: コンストラクタ
    #=================================================================
    def initialize()
      @cConf   = Config.new()  # configure
      @hErrors = Hash.new{|hash, key|  # エラー情報
        hash[key] = []
      }
      @hExclude = Hash.new{|hash, key|  # バリエーション除外
        hash[key] = {
          :sMessage   => "",
          :aFileNames => []
        }
      }
      @aFileNames     = []
      @nTestCaseCount = 0
    end

    #=================================================================
    # 概　要: 前処理を実行しTestScenarioインスタンスの配列を返す
    #=================================================================
    def pre_process(aArgs)
      check_class(Array, aArgs)      # 実行時引数

      @aFileNames = parse_option(aArgs)  # オプションパース
      @aFileNames = @aFileNames.uniq().sort()

      # configureロード
      sConfFile = @cConf.get_configure_file()
      if (sConfFile.nil?())
        sConfFile = DEFAULT_CONFIG_FILE
      end
      @cConf.load_config(sConfFile)
      @cConf.environment_check()

      # ファイルが開けるかチェック
      pre_file_check()

      # プリプロセッシング実行
      aTestScenario = exec_pre_process()

      # エラー表示
      print_error()

      # 有効なシナリオがない
      if (aTestScenario.empty?())
        raise(TTCError.new(ERR_NO_AVAILABLE_SCENARIO))
      end

      return aTestScenario  # [Array]TestScenarioインスタンスの配列
    rescue TTCError
      $stderr.puts ERR_HEADER
      $stderr.puts $!.message()
      print_exclude()
      exit(1)
    rescue TTCMultiError
      $stderr.puts(ERR_HEADER)
      $!.aErrors.each_with_index{|ex, index|
        $stderr.puts("  #{index + 1}. #{ex.message}")
      }
      print_exclude()
      exit(1)
    end

    #=================================================================
    # 概　要: 入力テストシナリオをチェックし，TestScenarioインスタンス
    #       : の配列を返す
    #=================================================================
    def exec_pre_process()
      nCnt = 0
      nTotal = @aFileNames.size()

      aTestScenario = []
      cMultipleChecker = MultipleChecker.new()
      @aFileNames.each{|sFileName|
        nCnt += 1

        begin
          bExist = false  # シナリオが存在するか
          # ファイル内容読み込み
          hYaml = load_yaml_file(sFileName)
          ### T1_001: 入力YAMLがHashではない
          unless (hYaml.is_a?(Hash))
            sErr = sprintf("T1_001: " + ERR_INVALID_TYPE_TESTCASE, Hash, hYaml.class())
            raise(YamlError.new(sErr))
          end

          # YAML内のテストシナリオ分析
          hYaml.each{|sTestID, hScenario|
            if (sTestID == TSR_LBL_VERSION)
              next
            end
            @nTestCaseCount += 1

            # プログレスバー
            if (@cConf.is_no_progress_bar_mode?())
              $stderr.puts "[TTC][#{"%5.1f" % (100 * nCnt.to_f / nTotal.to_f)}\% (#{"%4d" % nCnt}/#{"%4d" % nTotal})] #{sTestID}"
            else
              print_progress("TTC", sTestID, nCnt, nTotal)
            end

            # TestScenarioインスタンス生成
            bExist  = true
            hBefore = {sTestID => safe_dup(hScenario)}

            begin
              cTS = make_test_scenario(sTestID, hScenario)
              cMultipleChecker.store(cTS, sFileName)

              # デバッグ出力
              if (@cConf.is_debug_mode?())
                dump(hBefore, TTC_DEBUG_FILE_BEFORE)
                dump(cTS.to_yaml(), TTC_DEBUG_FILE_AFTER)
              end

              aTestScenario.push(cTS)
            rescue TTCMultiExcludeError
              $!.aErrors.each{|cErr|
                @hExclude[cErr.lErrCode][:sMessage] = cErr.message()
                @hExclude[cErr.lErrCode][:aFileNames].push(sFileName + " (" + sTestID + ")")
              }
              next
            end
          }

          ### T1_002: 入力YAML内にテストシナリオが1つもない
          if (bExist == false)
            raise(YamlError.new("T1_002: " + ERR_SCENARIO_NOT_EXIST))
          end
        rescue TTCMultiError
          @hErrors[sFileName].concat($!.aErrors)
        rescue TTCError
          @hErrors[sFileName].push($!)
        end
      }

      # プログレスバー用表示のリセット
      unless (@cConf.is_no_progress_bar_mode?())
        finish_progress("TTC", nTotal)
      end

      # シナリオ間チェック
      cMultipleChecker.multiple_check()

      # テストIDでソート
      aTestScenario = aTestScenario.sort{|cTS1, cTS2|
        cTS1.sTestID.downcase() <=> cTS2.sTestID.downcase()
      }

      return aTestScenario  # [Array]TestScenarioインスタンスの配列
    end
    private :exec_pre_process

    #=================================================================
    # 概　要: 1件分のTestScenarioインスタンスを生成し返す
    #=================================================================
    def make_test_scenario(sTestID, hScenario)
      check_class(String, sTestID)          # テストID
      check_class(Object, hScenario, true)  # テストシナリオ

      # TestScenarioインスタンス作成＋basic_check
      cTS = TestScenario.new(sTestID, hScenario)
      cTS.set_nil_attribute()

      # 補完
      cTS.complement()
      # マクロチェック
      cTS.scenario_check_macro()

      # global対応
      unless (@cConf.is_timer_local?())
        cTS.convert_global()
        # デバッグ出力
        if (@cConf.is_debug_mode?())
          dump(cTS.to_yaml(), TTC_DEBUG_FILE_GLOBAL)
        end
      end

      # マクロ置換
      cTS.convert_macro()

      # チェック
      cTS.attribute_check()
      cTS.object_check()
      cTS.condition_check()
      cTS.scenario_check()

      # バリエーションチェック
      cTS.variation_check()

      # エイリアス
      cTS.alias()
      cTS.init_conditions()

      return cTS  # [TestScenario]1件分のTestScenarioインスタンス
    end
    private :make_test_scenario

    #=================================================================
    # 概　要: 出力するファイルのチェックをする
    #=================================================================
    def pre_file_check()
      # 出力ファイルが開けない
      if ($0 != __FILE__)
        # チェックするファイル
        aFileNames = [TTC_EXCLUSION_FILE, TTJ_PRINT_FILE]
        aExt = [".c", ".h", ".cfg", ".html"]
        aExt.each{|sExt|
          aFileNames.push(@cConf.get_out_file() + sExt)
        }
        # 実際に開いてみて確認
        aFileNames.each{|sFileName|
          if (File.exist?(sFileName))
            begin
              File.open(sFileName, "w"){|cIO|
              }
            rescue SystemCallError
              sErr = sprintf(ERR_CANNOT_OPEN_FILE, sFileName)
              raise(TTCError.new(sErr))
            end
          end
        }
      end

      # デバッグ出力用ファイル削除
      if (@cConf.is_debug_mode?())
        aFiles = [TTC_DEBUG_FILE_BEFORE, TTC_DEBUG_FILE_AFTER, TTC_DEBUG_FILE_GLOBAL]
        aFiles.each{|sFileName|
          if (File.exist?(sFileName))
            begin
              File.delete(sFileName)
            rescue SystemCallError
              sErr = sprintf(ERR_CANNOT_OPEN_FILE, sFileName)
              raise(TTCError.new(sErr))
            end
          end
        }
      end
    end

    #=================================================================
    # 概　要: オプションをパースしオプション以外の引数を返す
    #=================================================================
    def parse_option(aArgs)
      check_class(Array, aArgs)  # コマンドライン引数

      cOpt = OptionParser.new(banner="Usage: ttg.rb [-a | -f] [options]... [yaml files]...")
      # バージョン情報設定
      cOpt.version = $Version

      # プロファイル設定
      cOpt.on("-a", "asp mode"){
        @cConf.set(CFG_PROFILE, "asp")
      }
      cOpt.on("-f", "fmp mode"){
        if (@cConf.is_profile_set?())
          puts cOpt.help()
          exit(1)
        end
        @cConf.set(CFG_PROFILE, "fmp")
      }

      # メインオプション
      cOpt.on("-c CONFFILE", "set configure file"){|val|
        @cConf.set(CFG_FILE, val)
      }
      cOpt.on("-d", "debug mode on"){|val|
        @cConf.set(CFG_DEBUG_MODE, val)
      }
      cOpt.on("-j", "ttj mode on"){|val|
        @cConf.set(CFG_ENABLE_TTJ, val)
      }
      cOpt.on("-p", "no progress bar mode on"){|val|
        @cConf.set(CFG_NO_PROGRESS_BAR, val)
      }
      cOpt.on("-t", "test program flow mode on"){|val|
        @cConf.set(CFG_TEST_FLOW, val)
      }

      # ターゲットのハードウェアアーキテクチャに関する設定
      cOpt.on("--#{CFG_PRC_NUM} NUM", "set processor number"){|val|
        @cConf.set(CFG_PRC_NUM, val)
      }
      cOpt.on("--#{CFG_MAIN_PRCID} NUM", "set main task's processor ID"){|val|
        @cConf.set(CFG_MAIN_PRCID, val)
      }
      cOpt.on("--#{CFG_DEFAULT_CLASS} CLASS", "set main task's class name"){|val|
        @cConf.set(CFG_DEFAULT_CLASS, val)
      }
      cOpt.on("--#{CFG_TIMER_ARCH} TYPE", "set timer architecture [#{TSR_PRM_TIMER_LOCAL} | #{TSR_PRM_TIMER_GLOBAL}]"){|val|
        @cConf.set(CFG_TIMER_ARCH, val)
      }
      cOpt.on("--#{CFG_TIME_MANAGE_PRCID} NUM", "set time manage processor ID"){|val|
        @cConf.set(CFG_TIME_MANAGE_PRCID, val)
      }
      cOpt.on("--#{CFG_TIME_MANAGE_CLASS} CLASS", "set time manage class name"){|val|
        @cConf.set(CFG_TIME_MANAGE_CLASS, val)
      }
      cOpt.on("--#{CFG_TIMER_INT_PRI} INTPRI", "set timer interrupt priority"){|val|
        @cConf.set(CFG_TIMER_INT_PRI, val)
      }
      cOpt.on("--#{CFG_SPINLOCK_NUM} NUM", "set spinlock number"){|val|
        @cConf.set(CFG_SPINLOCK_NUM, val)
      }
      cOpt.on("--#{CFG_IRC_ARCH} TYPE",
              "set irc architecture [#{TSR_PRM_IRC_LOCAL} | #{TSR_PRM_IRC_GLOBAL} | #{TSR_PRM_IRC_COMBINATION}]"){|val|
        @cConf.set(CFG_IRC_ARCH, val)
      }
      cOpt.on("--#{CFG_SUPPORT_GET_UTM} BOOL", "support api \"#{API_GET_UTM}\""){|val|
        @cConf.set(CFG_SUPPORT_GET_UTM, val)
      }
      cOpt.on("--#{CFG_SUPPORT_ENA_INT} BOOL", "support api \"#{API_ENA_INT}\""){|val|
        @cConf.set(CFG_SUPPORT_ENA_INT, val)
      }
      cOpt.on("--#{CFG_SUPPORT_DIS_INT} BOOL", "support api \"#{API_DIS_INT}\""){|val|
        @cConf.set(CFG_SUPPORT_DIS_INT, val)
      }
      cOpt.on("--#{CFG_OWN_IPI_RAISE} BOOL", "support own ipi raise"){|val|
        @cConf.set(CFG_OWN_IPI_RAISE, val)
      }
      cOpt.on("--#{CFG_ENA_EXC_LOCK} BOOL", "support cpu exception in cpu lock"){|val|
        @cConf.set(CFG_ENA_EXC_LOCK, val)
      }
      cOpt.on("--#{CFG_ENA_CHGIPM} BOOL", "support chg_ipm in non task"){|val|
        @cConf.set(CFG_ENA_CHGIPM, val)
      }

      # ターゲット依存部毎に用意するテストライブラリ
      cOpt.on("--#{CFG_FUNC_TIME} BOOL", "time function is available"){|val|
        @cConf.set(CFG_FUNC_TIME, val)
      }
      cOpt.on("--#{CFG_FUNC_INTERRUPT} BOOL", "interrupt function is available"){|val|
        @cConf.set(CFG_FUNC_INTERRUPT, val)
      }
      cOpt.on("--#{CFG_FUNC_EXCEPTION} BOOL", "exception function is available"){|val|
        @cConf.set(CFG_FUNC_EXCEPTION, val)
      }

      # 必要に応じて変更可能なパラメータ
      cOpt.on("--#{CFG_ALL_GAIN_TIME} BOOL", "set all_gain_time parameter"){|val|
        @cConf.set(CFG_ALL_GAIN_TIME, val)
      }
      cOpt.on("--#{CFG_STACK_SHARE} BOOL", "set stack share mode"){|val|
        @cConf.set(CFG_STACK_SHARE, val)
      }
      cOpt.on("--#{CFG_OUT_FILE} FILE", "set output program file name's prefix"){|val|
        @cConf.set(CFG_OUT_FILE, val)
      }
      cOpt.on("--#{CFG_WAIT_SPIN_LOOP} NUM", "set wait spinlock loop"){|val|
        @cConf.set(CFG_WAIT_SPIN_LOOP, val)
      }
      cOpt.on("--#{CFG_EXCEPT_ARG_NAME} NAME", "set exception arg name"){|val|
        @cConf.set(CFG_EXCEPT_ARG_NAME, val)
      }
      cOpt.on("--#{CFG_YAML_LIBRARY} TYPE", "set yaml library [#{CFG_LIB_YAML} | #{CFG_LIB_KWALIFY}]"){|val|
        @cConf.set(CFG_YAML_LIBRARY, val)
      }
      cOpt.on("--#{CFG_ENABLE_GCOV} BOOL", "enable gcov output"){|val|
        @cConf.set(CFG_ENABLE_GCOV, val)
      }
      cOpt.on("--#{CFG_ENABLE_LOG} BOOL", "enable log output"){|val|
        @cConf.set(CFG_ENABLE_LOG, val)
      }

      # 標準オプション
      cOpt.on("-v", "--version", "show version information"){
        puts cOpt.ver()
        exit(1)
      }
      cOpt.on("-h", "--help", "show help (this)"){
        puts cOpt.help()
        exit(1)
      }

      # 解析実行
      begin
        aPath = cOpt.parse(aArgs)
      rescue OptionParser::ParseError
        $stderr.puts ERR_HEADER
        $stderr.puts ERR_OPTION_INVALID
        $stderr.puts ERR_LINE
      end

      # プロファイル・ファイル指定がされていない場合
      if (aPath.nil?() || aPath.empty?() || !@cConf.is_profile_set?())
        puts cOpt.help()
        exit(1)
      end

      return aPath  # [Array]オプション以外の引数
    end
    private :parse_option

    #=================================================================
    # 概　要: エラーを出力する
    #=================================================================
    def print_error()
      unless (@hErrors.empty?())
        nNum = 0
        $stderr.puts(ERR_HEADER)
        @aFileNames.each{|sFileName|
          if (@hErrors.has_key?(sFileName) && !@hErrors[sFileName].empty?())
            nNum = nNum + 1
            $stderr.puts("* #{sFileName}")
            @hErrors[sFileName].each_with_index{|ex, index|
              if (ex.is_a?(YamlError))
                $stderr.puts("  #{index + 1}. [#{ex.path}]")
                $stderr.puts("  => #{ex.message}")
              else
                $stderr.puts("  #{index + 1}. #{ex.message}")
              end
            }
          end
        }
        $stderr.puts(ERR_LINE)
        $stderr.puts(sprintf(ERR_RESULT, nNum))

        exit(1)
      end
    end
    private :print_error

    #=================================================================
    # 概　要: 除外内容を出力する
    #=================================================================
    def print_exclude()
      unless (@hExclude.empty?())
        # 文字列の最大長・含まれるテストケース数取得
        nMaxLength = 0
        aTestCases = []
        @hExclude.each_value{|hItem|
          nSize = hItem[:sMessage].size()
          if (nSize > nMaxLength)
            nMaxLength = nSize
          end
          aTestCases.concat(hItem[:aFileNames])
        }
        #aTestCases = aTestCases.uniq()
        # 統計情報
        nExclude = aTestCases.size()
        nPercent = (@nTestCaseCount.to_f() - nExclude) / @nTestCaseCount * 100
        nPercent = nPercent.truncate()
        sTotal   = sprintf(ERR_EXCLUDE_RESULT, nExclude, nPercent, @nTestCaseCount - nExclude, @nTestCaseCount)

        # 出力
        $stderr.puts ERR_EXCLUDE_HEADER
        @hExclude.each_value{|hItem|
          $stderr.print(sprintf("* %-#{nMaxLength}s", hItem[:sMessage]))
          $stderr.puts("  #{hItem[:aFileNames].size()} test cases")
        }
        $stderr.puts(ERR_LINE)
        $stderr.puts(sTotal)
        # ファイルに出力
        File.open(TTC_EXCLUSION_FILE, "w"){|cIO|
          cIO.puts(ERR_EXCLUDE_HEADER)
          @hExclude.each_value{|hItem|
            cIO.puts("\n#{hItem[:sMessage]} (#{hItem[:aFileNames].size()} test cases)")
            hItem[:aFileNames].each{|sFileName|
              cIO.puts("* #{sFileName}")
            }
            cIO.puts(ERR_LINE)
          }
          cIO.puts(sTotal)
        }
      end
    end

    #=================================================================
    # 概　要: デバッグ用YAMLファイルを生成する
    #=================================================================
    def dump(hYaml, sFileName)
      check_class(Hash, hYaml)        # 出力YAML
      check_class(String, sFileName)  # 出力先ファイル名

      cIO = File.open(sFileName, "a")
      cIO.print("---")
      dump_yaml(cIO, hYaml)
      cIO.puts()
      cIO.close()
    end

    #=================================================================
    # 概　要: YAMLオブジェクトをソートし出力する
    #=================================================================
    def dump_yaml(cIO, data, nNest = 1)
      check_class(IO, cIO)             # 出力先IO
      check_class(Object, data, true)  # 出力データ
      check_class(Integer, nNest)      # ネスト数

      sSpace = "  " * nNest
      if (data.is_a?(Hash))
        cIO.puts()
        aKeys = data.keys()
        bAllInt = aKeys.all?(){|key|
          parse_value(key, @cConf.get_macro()).is_a?(Integer)
        }
        hKeys = {}
        if (bAllInt == true)
          aKeys.each{|key|
            nVal = parse_value(key, @cConf.get_macro())
            hKeys[nVal] = key
          }
        else
          aKeys.each{|key|
            hKeys[key.to_s()] = key
          }
        end
        hKeys.sort().each{|atr, val|
          sOriginal = hKeys[atr]
          cIO.print("#{sSpace}#{sOriginal}:")
          dump_yaml(cIO, data[sOriginal], nNest + 1)
        }
      elsif (data.is_a?(Array))
        cIO.puts()
        data.each{|val|
          cIO.print("#{sSpace}-")
          dump_yaml(cIO, val, nNest + 1)
        }
      elsif (data.nil?())
        cIO.puts()
      else
        cIO.puts(" #{data}")
      end
    end
  end
end


#=====================================================================
# main
#=====================================================================
if ($0 == __FILE__)
  module TTCModule
    include CommonModule

    cTTC = TTC.new()
    aTestScenario = cTTC.pre_process(ARGV)
    cTTC.print_exclude()
  end
end
