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
#  $Id: MultipleChecker.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "common/bin/Config.rb"
require "common/bin/test_scenario/TestScenario.rb"
require "ttc/bin/class/TTCCommon.rb"

#=====================================================================
# TTCModule
#=====================================================================
module TTCModule
  #===================================================================
  # クラス名: MultipleChecker
  # 概　  要: シナリオ間のエラーチェック
  #===================================================================
  class MultipleChecker
    include CommonModule
    include TTCModule

    #=================================================================
    # 概　要: コンストラクタ
    #=================================================================
    def initialize()
      @cConf = Config.new()
      # テストID重複チェック
      @hMultipleTestID = Hash.new{|hash, key|
        hash[key] = []
      }
      # スピンロック数カウント
      @aSpinlockID     = []
      @aExistSpinFiles = []
      # 各種統一パラメータ
      @hUnifiedAtrs = {
        :hInthdr    => {},
        :hIsr       => {},
        :hIsr2      => {},
        :hException => {}
      }

=begin
      # 割込み番号数カウント
      @aIntno            = []
      @aExistIntnoTestID = []
=end
    end

    #=================================================================
    # 概　要: エラーチェック用のデータを格納する
    #=================================================================
    def store(cTS, sFileName)
      check_class(TestScenario, cTS)  # テストシナリオ
      check_class(String, sFileName)  # ファイル名

      # スピンロックチェック
      if (@cConf.is_fmp?())
        aSpinNames = cTS.get_spinlock_names()
        @aSpinlockID.concat(aSpinNames)
        if (aSpinNames.size() > 0)
          @aExistSpinFiles.push(sFileName)
        end
      end

=begin
      # 割込み番号チェック
      aIntno = cTS.get_all_intno()
      @aIntno.concat(aIntno)
      if (aIntno.size() > 0)
        @aExistIntnoTestID.push(cTS.sTestID)
      end
=end

      # テストID重複チェック用
      @hMultipleTestID[cTS.sTestID].push(sFileName)
      # 各種統一パラメータ
      @hUnifiedAtrs[:hInthdr][sFileName]    = cTS.get_inthdr_attributes_by_intno()
      @hUnifiedAtrs[:hIsr][sFileName]       = cTS.get_isr_attributes_by_intno()
      @hUnifiedAtrs[:hIsr2][sFileName]      = cTS.get_isr_attributes_by_intno_and_isrpri()
      @hUnifiedAtrs[:hException][sFileName] = cTS.get_exception_attributes_by_excno()

    end

    #=================================================================
    # 概　要: シナリオ間エラーチェックを実行する
    #=================================================================
    def multiple_check()
      aErrors = []

      # 同じテストIDを複数のYAMLが使用している場合
      @hMultipleTestID.each{|sTestID, aFileNames|
        ### T7_001: テストIDが重複している
        if (aFileNames.size() > 1)
          sErr = sprintf("T7_001: " + ERR_TESTID_MULTIPLE, sTestID)
          aFileNames.each{|sFileName|
            sErr.concat("\n * #{sFileName}")
          }
          aErrors.push(TTCError.new(sErr))
        end
      }

      # 全テストケースで統一された属性が必要なオブジェクトのチェック
      ### T7_002: 同一割込み番号に対する割込みハンドラにおいてinhno，intpri，classが異なる
      ### T7_003: 同一割込み番号に対する割込みサービスルーチンにおいてintpriが異なる
      ### T7_004: 同一割込み番号に対するisrpriが同一のサービスルーチンにおいてexinf，classが異なる
      ### T7_006: 同一CPU例外ハンドラ番号に対するCPU例外ハンドラにおいてclassが異なる
      hChech = {
        :hInthdr    => "T7_002: " + ERR_UNIFIED_ATR_INTHDR,
        :hIsr       => "T7_003: " + ERR_UNIFIED_ATR_ISR,
        :hIsr2      => "T7_004: " + ERR_UNIFIED_ATR_ISR,
        :hException => "T7_006: " + ERR_UNIFIED_ATR_EXCEPTION
      }
      hChech.each{|lKey, sErr|
        begin
          multiple_check_unified_attributes(@hUnifiedAtrs[lKey], sErr)
        rescue TTCError
          aErrors.push($!)
        end
      }


      # 割り込み番号の衝突をチェック
      hCheck = Hash.new{|hash, key|
        hash[key] = Hash.new{|hash2, key2|
          hash2[key2] = []
        }
      }
      [:hInthdr, :hIsr].each{|lKey|
        @hUnifiedAtrs[lKey].each{|sTestID, hData|
          hData.each_key{|hGroupKeyAtrs|
            hCheck[hGroupKeyAtrs[TSR_PRM_INTNO]][lKey].push(sTestID)
          }
        }
      }
      # チェック
      # (ハッシュキーが:hInthdr，:hIsrとなっているが中身は配列)
      hCheck.each{|snIntNo, hGroup|
        ### T7_005: 同一割込み番号に対して，割込みハンドラと割込みサービスルーチンが定義されている
        if (hGroup[:hInthdr].size() > 0 && hGroup[:hIsr].size() > 0)
          sErr = sprintf("T7_005: " + ERR_CONFLICT_INTNO, snIntNo) + "\n"
          sErr.concat(" #{TSR_OBJ_INTHDR}\n")
          hGroup[:hInthdr].sort().each{|sFileName|
            sErr.concat(" * #{sFileName}\n")
          }
          sErr.concat(" #{TSR_OBJ_ISR}\n")
          hGroup[:hIsr].sort().each{|sFileName|
            sErr.concat(" * #{sFileName}\n")
          }
          aErrors.push(TTCError.new(sErr))
        end
      }

=begin
      ### T7_007: 割込み番号の使用数が設定値より超えている
      @aIntno = @aIntno.uniq().sort()
      if (@aIntno.size() > @cConf.get_intno_num())
        sErr = "T7_007: " + ERR_OVER_INTNO_SUM + "\n"
        @aExistIntnoTestID.each{|sTestID|
          sErr += " * #{sTestID}\n"
        }
        aErrors.push(TTCError.new(sErr))
      end
=end

      ### T7_F001: スピンロックの数が設定値より超えている
      if (@cConf.is_fmp?() && @cConf.get_spinlock_num() != 0)
        @aSpinlockID = @aSpinlockID.uniq().sort()
        if (@aSpinlockID.size() > @cConf.get_spinlock_num())
          sErr = "T7_F001: " + ERR_OVER_SPINLOCK_SUM + "\n"
          @aExistSpinFiles.each{|sFileName|
            sErr += " * #{sFileName}\n"
          }
          aErrors.push(TTCError.new(sErr))
        end
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概　要: パラメータがテストケース間で統一されているかチェックする
    #=================================================================
    def multiple_check_unified_attributes(hUnifiedAtrs, sErr)
      check_class(Hash, hUnifiedAtrs)  # チェックするデータ
      check_class(String, sErr)        # エラーメッセージ

      # パラメータの組み合わせごとにテストIDをまとめる
      hGroups = Hash.new{|hash, key|
        hash[key] = Hash.new{|hash2, key2|
          hash2[key2] = []
        }
      }
      hUnifiedAtrs.each{|sTestID, hData|
        hData.each{|hGroupKeyAtrs, aAtrs|
          aAtrs.each{|hAtrs|
            hGroups[hGroupKeyAtrs][hAtrs].push(sTestID)
          }
        }
      }

      # パラメータの組み合わせが異なるテストケースがあるかチェック
      hGroups.each{|hGroupKeyAtrs, hAtrGroups|
        if (hAtrGroups.size() > 1)
          nNum = 1
          # グループ
          aTmp = []
          hGroupKeyAtrs.each{|atr, val|
            aTmp.push("#{atr} = #{val}")
          }
          sErr.concat("\ntarget: " + aTmp.join(", ") + "\n")
          # 組み合わせごとにテストID一覧出力
          hAtrGroups.each{|hAtrs, aTestID|
            # パラメータの組み合わせ
            sErr.concat("-" * 30 + "\n")
            sErr.concat("case#{nNum}: ")
            nNum += 1
            aTmp = []
            hAtrs.each{|atr, val|
              unless (val.nil?())
                aTmp.push("#{atr} = #{val}")
              end
            }
            sErr.concat(aTmp.join(", ") + "\n")
            # テストID
            aTestID.each{|sTestID|
              sErr.concat(" * #{sTestID}\n")
            }
          }

          raise(TTCError.new(sErr))
        end
      }
    end
  end
end
