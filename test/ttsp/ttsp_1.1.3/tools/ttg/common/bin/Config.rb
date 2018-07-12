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
#  $Id: Config.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "common/bin/CommonModule.rb"
require "ttc/bin/class/TTCCommon.rb"

module CommonModule
  #===================================================================
  # クラス名: Config
  # 概　  要: 設定を管理
  #===================================================================
  class Config
    include CommonModule
    include TTCModule

    @@hConf = {}  # configure設定

    #=================================================================
    # 概　要: configureファイルを読み込む
    #=================================================================
    def load_config(sFileName)
      check_class(String, sFileName)  # configureファイル

      hConf = load_yaml_file(sFileName)
      # Hashならば中身をsetしていく
      if (hConf.is_a?(Hash))
        hConf.each{|sKey, val|
          # 既に設定されている項目は飛ばす
          if (@@hConf[sKey].nil?())
            set(sKey, val)
          end
        }
      ### T0_008: configureがHashでない
      else
        sErr = sprintf("T0_008: " + ERR_INVALID_TYPE, "configure", Hash, hConf.class())
        raise(TTCError.new(sErr))
      end

      # マクロの値を計算する
      calc_macro()
    end

    #=================================================================
    # 概　要: 値を設定する
    #=================================================================
    def set(sKey, val)
      check_class(String, sKey)       # キー
      check_class(Object, val, true)  # 値

      # 16進数処理
      if (val =~ TTC_REX_HEX)
        val = val.hex()
      end

      case sKey
      # 文字列
      when CFG_FILE, CFG_DEFAULT_CLASS, CFG_OUT_FILE, CFG_TIME_MANAGE_CLASS, CFG_EXCEPT_ARG_NAME
        ### T0_002: configureの設定値が指定された型と異なる
        unless (val.is_a?(String))
          sErr = sprintf("T0_002: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        end

      # 真偽値
      when CFG_TEST_FLOW, CFG_DEBUG_MODE, CFG_STACK_SHARE, CFG_ALL_GAIN_TIME,
           CFG_OWN_IPI_RAISE, CFG_ENA_EXC_LOCK, CFG_ENA_CHGIPM,
           CFG_FUNC_TIME, CFG_FUNC_INTERRUPT, CFG_FUNC_EXCEPTION,
           CFG_NO_PROGRESS_BAR, CFG_ENABLE_GCOV, CFG_ENABLE_LOG, CFG_ENABLE_TTJ,
           CFG_SUPPORT_GET_UTM, CFG_SUPPORT_ENA_INT, CFG_SUPPORT_DIS_INT
        val = to_bool(val)
        ### T0_002: configureの設定値が指定された型と異なる
        unless (Bool.include?(val.class()))
          sErr = sprintf("T0_002: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        end

      # 0より大きい整数
      when CFG_MAIN_PRCID, CFG_PRC_NUM, CFG_TIME_MANAGE_PRCID
        # オプション指定時は文字列になるため変換
        if (val =~ TTC_REX_NUM)
          val = val.to_i()
        end
        ### T0_002: configureの設定値が指定された型と異なる
        if (!val.is_a?(Integer))
          sErr = sprintf("T0_002: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        ### T0_003: configureの設定値が有効値でない
        elsif (val <= 0)
          sErr = sprintf("T0_003: " + ERR_CFG_BE_INTEGER_GT, sKey, 0, val)
          raise(TTCError.new(sErr))
        end

      # 0以上の整数
      when CFG_SPINLOCK_NUM, CFG_WAIT_SPIN_LOOP
        # オプション指定時は文字列になるため変換
        if (val =~ TTC_REX_NUM)
          val = val.to_i()
        end
        ### T0_002: configureの設定値が指定された型と異なる
        if (!val.is_a?(Integer))
          sErr = sprintf("T0_002: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        ### T0_003: configureの設定値が有効値でない
        elsif (val < 0)
          sErr = sprintf("T0_003: " + ERR_CFG_BE_INTEGER_GE, sKey, 0, val)
          raise(TTCError.new(sErr))
        end

      # 文字列か0より小さい整数
      when CFG_TIMER_INT_PRI
        ### T0_002: configureの設定値が指定された型と異なる
        if (!val.is_a?(String) && !val.is_a?(Integer))
          sErr = sprintf("T0_002: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        ### T0_003: configureの設定値が有効値でない
        elsif (val.is_a?(Integer) && (val >= 0))
          sErr = sprintf("T0_003: " + ERR_CFG_BE_INTEGER_LT, sKey, 0, val)
          raise(TTCError.new(sErr))
        end

      # プロファイル
      when CFG_PROFILE
        # オプション解析で有効値をチェックする

      # タイマアーキテクチャ
      when CFG_TIMER_ARCH
        ### T0_002: configureの設定値が指定された型と異なる
        if (!val.is_a?(String))
          sErr = sprintf("T0_002: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        ### T0_003: configureの設定値が有効値でない
        elsif (val != TSR_PRM_TIMER_GLOBAL && val != TSR_PRM_TIMER_LOCAL)
          sErr = sprintf("T0_003: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        end

      # IRCアーキテクチャ
      when CFG_IRC_ARCH
        ### T0_002: configureの設定値が指定された型と異なる
        if (!val.is_a?(String))
          sErr = sprintf("T0_002: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        ### T0_003: configureの設定値が有効値でない
        elsif (val != TSR_PRM_IRC_GLOBAL && val != TSR_PRM_IRC_LOCAL && val != TSR_PRM_IRC_COMBINATION)
          sErr = sprintf("T0_003: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        end

      # YAMLライブラリ
      when CFG_YAML_LIBRARY
        ### T0_002: configureの設定値が指定された型と異なる
        if (!val.is_a?(String))
          sErr = sprintf("T0_002: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        ### T0_003: configureの設定値が有効値でない
        elsif (val != CFG_LIB_YAML && val != CFG_LIB_KWALIFY)
          sErr = sprintf("T0_003: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        end

      # マクロ
      when CFG_MACRO
        ### T0_009: マクロ定義部がHashでない
        unless (val.nil?() || val.is_a?(Hash))
          sErr = sprintf("T0_009: " + ERR_CFG_INVALID_VALUE, sKey, val)
          raise(TTCError.new(sErr))
        end

      # 定義されていないキー
      else
        ### T0_010: configureに規定されていない項目が記述されている
        sErr = sprintf("T0_010: " + ERR_CFG_UNDEFINED_KEY, sKey, "configure")
        raise(TTCError.new(sErr))
      end

      # チェックが通れば代入
      @@hConf[sKey] = val
    end

    #=================================================================
    # 概　要: マクロの値を処理
    #=================================================================
    def calc_macro()
      if (@@hConf[CFG_MACRO].is_a?(Hash))
        hMacro = @@hConf.dup()
        hMacro.delete(CFG_MACRO)
        @@hConf[CFG_MACRO].each{|sMacro, val|
          @@hConf[CFG_MACRO][sMacro] = parse_value(val, hMacro)
        }
      end
    end

    #=================================================================
    # 概　要: 真偽値を表す文字列を変換する
    #=================================================================
    def to_bool(sStr)
      check_class(Object, sStr, true)  # 変換する文字列

      case sStr
      when "true"
        return true # [Bool]変換結果
      when "false"
        return false # [Bool]変換結果
      else
        return safe_dup(sStr) # [Bool]変換結果
      end
    end

    #=================================================================
    # 概　要: configureのチェックを行う
    #=================================================================
    def environment_check()
      @aErrors = []

      # 設定のチェック
      GRP_CFG_NECESSARY_KEYS.each{|sKey|
        ### T0_001: configureの設定で記述されていない項目がある
        unless (@@hConf.has_key?(sKey))
          sErr = sprintf("T0_001: " + ERR_CFG_REQUIRED_KEY, sKey)
          @aErrors.push(TTCError.new(sErr))
        end
      }

      # マクロのチェック
      hMacro = get_macro()
      hBitPattern = Hash.new{|hash, key|
        hash[key] = {}
      }

      ### T0_007: 必要なマクロが定義されていない
      GRP_CFG_NECESSARY_MACRO.each{|sMacro|
        if (hMacro.nil?() || !hMacro.has_key?(sMacro))
          sErr = sprintf("T0_007: " + ERR_CFG_REQUIRED_MACRO, sMacro)
          @aErrors.push(TTCError.new(sErr))
        end
      }

      if (hMacro.is_a?(Hash))
        ### T0_004: マクロの設定値が指定された型と異なる
        ### T0_005: マクロの設定値が有効値でない
        hMacro.each{|sMacro, val|
          case sMacro
          # [0より大きい整数]
          when /^PRC_/, "MAIN_PRCID", "ANY_INI_BLKCNT", "ANY_NOW_BLKCNT", "ANY_BLKSZ", "ANY_MAX_SEMCNT", "ANY_NOW_SEMCNT"
            if (!val.is_a?(Integer))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, Integer, val.class())
              @aErrors.push(TTCError.new(sErr))
            elsif (val <= 0)
              sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_GT, sMacro, 0, val)
              @aErrors.push(TTCError.new(sErr))
            end

          # [文字列]
          when /^CLS_/
            unless (val.is_a?(String))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, String, val.class())
              @aErrors.push(TTCError.new(sErr))
            end

          # [1から4の範囲の整数]
          when "TSK_PRI_LE_4", "TSK_PRI_LE_LE_4"
            if (!val.is_a?(Integer))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, Integer, val.class())
              @aErrors.push(TTCError.new(sErr))
            elsif (val < 1 || val > 4)
              sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_RANGE, sMacro, 1, 4, val)
              @aErrors.push(TTCError.new(sErr))
            end

          # [13から16の範囲の整数]
          when "TSK_PRI_GE_13"
            if (!val.is_a?(Integer))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, Integer, val.class())
              @aErrors.push(TTCError.new(sErr))
            elsif (val < 13 || val > 16)
              sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_RANGE, sMacro, 13, 16, val)
              @aErrors.push(TTCError.new(sErr))
            end

          # [0以上の整数]
          when /^BIT_PATTERN_/, /^TEXPTN_/
            nVal = parse_value(val)
            unless (nVal.is_a?(Integer))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, Integer, val.class())
              @aErrors.push(TTCError.new(sErr))
            end
            # 0である場合
            if (nVal == 0 && sMacro =~ /^(BIT_PATTERN|TEXPTN)_[^0]/)
              sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_NE, sMacro, 0, val)
              @aErrors.push(TTCError.new(sErr))
            end
            # ビットユニーク判定用
            if (sMacro =~ /^BIT_PATTERN_/)
              hBitPattern[:hBitPtn][sMacro] = nVal
            elsif (sMacro =~ /^TEXPTN_/)
              hBitPattern[:hTexPtn][sMacro] = nVal
            end

          # [TWF_ANDW，TWF_ORW]
          when /^WAIT_FLG_MODE_/
            unless (val == KER_TWF_ANDW || val == KER_TWF_ORW)
              sErr = sprintf("T0_005: " + ERR_MCR_INVALID_VALUE, sMacro, val)
              @aErrors.push(TTCError.new(sErr))
            end

          # [0以上の整数]
          when "ANY_INI_SEMCNT", /^RELATIVE_/
            if (!val.is_a?(Integer))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, Integer, val.class())
              @aErrors.push(TTCError.new(sErr))
            elsif (val < 0)
              sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_GE, sMacro, 0, val)
              @aErrors.push(TTCError.new(sErr))
            end

          # [1から16の整数]
          when /^TSK_PRI_/, /^DATA_PRI_/, /^MSG_PRI_/, /^ISR_PRI_/
            if (!val.is_a?(Integer))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, Integer, val.class())
              @aErrors.push(TTCError.new(sErr))
            elsif (val < 1 || val > 16)
              sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_RANGE, sMacro, 1, 16, val)
              @aErrors.push(TTCError.new(sErr))
            end

          # [0より大きい整数]
          when /_TIME$/
            if (!val.is_a?(Integer))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, Integer, val.class())
              @aErrors.push(TTCError.new(sErr))
            elsif (val <= 0)
              sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_GE, sMacro, 0, val)
              @aErrors.push(TTCError.new(sErr))
            end

          # [文字列か0より小さい整数]
          when /^ANY_IPM/, /^INT_PRI_/
            unless (val.is_a?(String))
              if (!val.is_a?(Integer))
                sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, "Integer or String", val.class())
                @aErrors.push(TTCError.new(sErr))
              elsif (val >= 0)
                sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_LT, sMacro, 0, val)
                @aErrors.push(TTCError.new(sErr))
              end
            end

          # [文字列か0以上の整数]
          when /^INTNO_/, /^INHNO_/, /^EXCNO_/, /^ANY_/
            unless (val.is_a?(String))
              if (!val.is_a?(Integer))
                sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, "Integer or String", val.class())
                @aErrors.push(TTCError.new(sErr))
              elsif (val < 0)
                sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_GE, sMacro, 0, val)
                @aErrors.push(TTCError.new(sErr))
              end
            end

          # [文字列か整数]
          when /^DATA_/, /^EXINF_/
            if (!val.is_a?(Integer) && !val.is_a?(String))
              sErr = sprintf("T0_004: " + ERR_MCR_INVALID_TYPE, sMacro, "Integer or String", val.class())
              @aErrors.push(TTCError.new(sErr))
            end
          end
        }
        check_error(@aErrors)

        # 依存関係のチェック
        check_priority("TSK_PRI_LOW", "TSK_PRI_MID", "TSK_PRI_HIGH")
        check_le("TSK_PRI_LE_LE_4", "TSK_PRI_LE_4")
        check_le("TSK_PRI_LE_GE_13", "TSK_PRI_GE_13")
        check_priority("DATA_PRI_LOW", "DATA_PRI_MID", "DATA_PRI_HIGH", "DATA_PRI_MAX")
        check_priority("MSG_PRI_LOW", "MSG_PRI_MID", "MSG_PRI_HIGH", "MSG_PRI_MAX")
        check_priority("ISR_PRI_LOW", "ISR_PRI_MID", "ISR_PRI_HIGH")
        check_le("ANY_NOW_BLKCNT", "ANY_INI_BLKCNT")
        check_le("ANY_INI_SEMCNT", "ANY_MAX_SEMCNT")
        check_lt("ANY_NOW_SEMCNT", "ANY_MAX_SEMCNT")
        check_bit_unique(hBitPattern[:hBitPtn])
        check_bit_unique(hBitPattern[:hTexPtn])
      end

      ### T0_006: 時間操作関数がないのに全テストケース時間停止指定である
      if (!is_all_gain_time_mode?() && get_func_time() == false)
        @aErrors.push(TTCError.new("T0_006: " + ERR_VARIATION_TIME_GAIN))
      end

      check_error(@aErrors)
    end

    #=================================================================
    # 概　要: 優先度の依存関係チェックを行う
    #=================================================================
    def check_priority(sLow, sMid, sHigh, sMax = nil)
      check_class(String, sLow)        # LOW
      check_class(String, sMid)        # MID
      check_class(String, sHigh)       # HIGH
      check_class(String, sMax, true)  # MAX

      hMacro = get_macro()
      # 最大値
      unless (hMacro[sMax].nil?())
        check_le(sLow, sMax)
        check_le(sMid, sMax)
        check_le(sHigh, sMax)
      end

      # 大小関係
      check_lt(sMid, sLow)
      check_lt(sHigh, sMid)
      check_lt(sHigh, sLow)
    end

    #=================================================================
    # 概　要: 大小関係チェックを行う
    #=================================================================
    def check_le(sKey1, sKey2)
      check_class(String, sKey1, true)  # 元となる値のキー
      check_class(String, sKey2, true)  # 比較対象のキー

      hMacro = get_macro()
      if (hMacro[sKey1] > hMacro[sKey2])
        sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_LE, sKey1, sKey2, hMacro[sKey2])
        @aErrors.push(TTCError.new(sErr))
      end
    end

    #=================================================================
    # 概　要: 大小関係チェックを行う
    #=================================================================
    def check_lt(sKey1, sKey2)
      check_class(String, sKey1, true)  # 元となる値のキー
      check_class(String, sKey2, true)  # 比較対象のキー

      hMacro = get_macro()
      if (hMacro[sKey1] >= hMacro[sKey2])
        sErr = sprintf("T0_005: " + ERR_MCR_BE_INTEGER_LT, sKey1, sKey2, hMacro[sKey2])
        @aErrors.push(TTCError.new(sErr))
      end
    end

    #=================================================================
    # 概　要: ビットユニークチェックを行う
    #=================================================================
    def check_bit_unique(hBitPattern)
      check_class(Hash, hBitPattern)  # ビットパターンが入っている配列

      nTmp = 0
      hBitPattern.each{|sMacro, nVal|
        if (nTmp & nVal != 0)
          sErr = sprintf("T0_005: " + ERR_MCR_NOT_BIT_UNIQUE, sMacro)
          @aErrors.push(TTCError.new(sErr))
        end
        nTmp = nTmp | nVal
      }
    end

    #=================================================================
    # 概　要: 読み込むconfigureファイル名を取得
    #=================================================================
    def get_configure_file()
      return @@hConf[CFG_FILE] # [String]configureファイル名
    end

    #=================================================================
    # 概　要: 出力ファイル名を取得
    #=================================================================
    def get_out_file()
      return @@hConf[CFG_OUT_FILE] # [String]出力ファイル名
    end

    #=================================================================
    # 概　要: デフォルトクラス名を取得
    #=================================================================
    def get_default_class()
      return @@hConf[CFG_DEFAULT_CLASS] # [String]デフォルトクラス名
    end

    #=================================================================
    # 概　要: メインプロセッサIDを取得
    #=================================================================
    def get_main_prcid()
      return @@hConf[CFG_MAIN_PRCID] # [Integer]メインプロセッサID
    end

    #=================================================================
    # 概　要: プロセッサ数を取得
    #=================================================================
    def get_prc_num()
      return @@hConf[CFG_PRC_NUM] # [Integer]プロセッサ数
    end

    #=================================================================
    # 概　要: スピンロック数を取得
    #=================================================================
    def get_spinlock_num()
      return @@hConf[CFG_SPINLOCK_NUM] # [Integer]スピンロック数
    end

    #=================================================================
    # 概　要: スピンロック取得待ちを確認するためのループ回数を取得
    #=================================================================
    def get_wait_spin_loop()
      return @@hConf[CFG_WAIT_SPIN_LOOP] # [Integer]スピンロック取得待ちを確認するためのループ回数
    end

    #=================================================================
    # 概　要: タイマアーキテクチャを取得
    #=================================================================
    def get_timer_arch()
      return @@hConf[CFG_TIMER_ARCH] # [String]タイマアーキテクチャ
    end

    #=================================================================
    # 概　要: IRCアーキテクチャを取得
    #=================================================================
    def get_irc_arch()
      return @@hConf[CFG_IRC_ARCH] # [String]IRCアーキテクチャ
    end

    #=================================================================
    # 概　要: グローバルタイマ用クラスを取得
    #=================================================================
    def get_time_manage_class()
      return @@hConf[CFG_TIME_MANAGE_CLASS] # [String]グローバルタイマ用クラス
    end

    #=================================================================
    # 概　要: システム時刻管理プロセッサIDを取得
    #=================================================================
    def get_time_manage_prcid()
      return @@hConf[CFG_TIME_MANAGE_PRCID] # [Integer]システム時刻管理プロセッサID
    end

    #=================================================================
    # 概　要: ユーザ定義マクロを取得
    #=================================================================
    def get_macro()
      return @@hConf[CFG_MACRO] # [Hash]マクロのハッシュ
    end

    #=================================================================
    # 概　要: 時間操作関数が使用可能かを返す
    #=================================================================
    def get_func_time()
      return @@hConf[CFG_FUNC_TIME] # [Bool]時間操作関数が使用可能か
    end

    #=================================================================
    # 概　要: 割込み発生関数が使用可能かを返す
    #=================================================================
    def get_func_interrupt()
      return @@hConf[CFG_FUNC_INTERRUPT] # [Bool]割込み発生関数が使用可能か
    end

    #=================================================================
    # 概　要: CPU例外発生関数が使用可能かを返す
    #=================================================================
    def get_func_exception()
      return @@hConf[CFG_FUNC_EXCEPTION] # [Bool]CPU例外発生関数が使用可能か
    end

    #=================================================================
    # 概　要: 自プロセッサへのプロセッサ間割込みが可能かを返す
    #=================================================================
    def get_own_ipi_raise()
      return @@hConf[CFG_OWN_IPI_RAISE] # [Bool]自プロセッサへのプロセッサ間割込みが可能か
    end

    #=================================================================
    # 概　要: CPUロック中のCPU例外発生をサポートしているかを返す
    #=================================================================
    def get_enable_exc_in_cpulock()
      return @@hConf[CFG_ENA_EXC_LOCK] # [Bool]CPUロック中のCPU例外発生をサポートしているか
    end

    #=================================================================
    # 概　要: 非タスクコンテキストからの割込み優先度マスク変更をサポー
    #       : トしているかを返す
    #=================================================================
    def get_enable_chg_ipm_in_non_task()
      return @@hConf[CFG_ENA_CHGIPM] # [Bool]サポートしているか
    end

    #=================================================================
    # 概　要: CPU例外ハンドラの引数とする変数名を取得
    #=================================================================
    def get_exception_arg_name()
      return @@hConf[CFG_EXCEPT_ARG_NAME] # [String]引数とする変数名
    end

    #=================================================================
    # 概　要: APIのget_utmをサポートしているかを返す
    #=================================================================
    def get_api_support_get_utm()
      return @@hConf[CFG_SUPPORT_GET_UTM] # [Bool]サポートしているか
    end

    #=================================================================
    # 概　要: APIのena_intをサポートしているかを返す
    #=================================================================
    def get_api_support_ena_int()
      return @@hConf[CFG_SUPPORT_ENA_INT] # [Bool]サポートしているか
    end

    #=================================================================
    # 概　要: APIのdis_intをサポートしているかを返す
    #=================================================================
    def get_api_support_dis_int()
      return @@hConf[CFG_SUPPORT_DIS_INT] # [Bool]サポートしているか
    end

    #=================================================================
    # 概　要: タイマ割込みの割込み優先度を返す(現状不使用)
    #=================================================================
    def get_timer_int_pri()
      return @@hConf[CFG_TIMER_INT_PRI] # [Bool]タイマ割込みの割込み優先度
    end

    #=================================================================
    # 概　要: テストフローモードが有効かを返す
    #=================================================================
    def is_testflow_mode?()
      return @@hConf[CFG_TEST_FLOW] # [Bool]テストフローモードが有効か
    end

    #=================================================================
    # 概　要: デバッグモードが有効かを返す
    #=================================================================
    def is_debug_mode?()
      return @@hConf[CFG_DEBUG_MODE] # [Bool]デバッグモードが有効か
    end

    #=================================================================
    # 概　要: スタック共有モードが有効かを返す
    #=================================================================
    def is_stack_share_mode?()
      return @@hConf[CFG_STACK_SHARE] # [Bool]スタック共有モードが有効か
    end

    #=================================================================
    # 概　要: 全テストシナリオで時間を進めるモードが有効かを返す
    #=================================================================
    def is_all_gain_time_mode?()
      return @@hConf[CFG_ALL_GAIN_TIME] # [Bool]全テストシナリオで時間を進めるモードが有効か
    end

    #=================================================================
    # 概　要: プログレスバー非表示モードが有効かを返す
    #=================================================================
    def is_no_progress_bar_mode?()
      return @@hConf[CFG_NO_PROGRESS_BAR] # [Bool]プログレスバー非表示モードが有効か
    end

    #=================================================================
    # 概　要: TTJ出力モードが有効かを返す
    #=================================================================
    def is_enable_ttj_mode?()
      return @@hConf[CFG_ENABLE_TTJ] # [Bool]TTJ出力モードが有効か
    end

    #=================================================================
    # 概　要: プロファイルがASPかチェック
    #=================================================================
    def is_asp?()
      return (@@hConf[CFG_PROFILE] == CFG_PROFILE_ASP) # [Bool]プロファイルがASPか
    end

    #=================================================================
    # 概　要: プロファイルがFMPかチェック
    #=================================================================
    def is_fmp?()
      return (@@hConf[CFG_PROFILE] == CFG_PROFILE_FMP) # [Bool]プロファイルがFMPか
    end

    #=================================================================
    # 概　要: タイマアーキテクチャがローカルかチェック
    #=================================================================
    def is_timer_local?()
      return (@@hConf[CFG_TIMER_ARCH] == TSR_PRM_TIMER_LOCAL) # [Bool]タイマアーキテクチャがローカルか
    end

    #=================================================================
    # 概　要: プロファイルがセットされているかチェック
    #=================================================================
    def is_profile_set?()
      return (!@@hConf[CFG_PROFILE].nil?()) # [Bool]プロファイルがセットされているか
    end

    #=================================================================
    # 概　要: IRCアーキテクチャがグローバルかチェック(現状不使用)
    #=================================================================
    def is_irc_global?()
      return (@@hConf[CFG_IRC_ARCH] == TSR_PRM_IRC_GLOBAL) #[Bool]IRCアーキテクチャがグローバルか
    end

    #=================================================================
    # 概　要: IRCアーキテクチャがローカルかチェック(現状不使用)
    #=================================================================
    def is_irc_local?()
      return (@@hConf[CFG_IRC_ARCH] == TSR_PRM_IRC_LOCAL) #[Bool]IRCアーキテクチャがローカルか
    end

    #=================================================================
    # 概　要: IRCアーキテクチャを両方サポートするかチェック(現状不使用)
    #=================================================================
    def is_irc_combination?()
      return (@@hConf[CFG_IRC_ARCH] == TSR_PRM_IRC_COMBINATION) #[Bool]IRCアーキテクチャを両方サポートするか
    end

    #=================================================================
    # 概　要: YAMLライブラリを使うかを返す
    #=================================================================
    def use_yaml_library?()
      return (@@hConf[CFG_YAML_LIBRARY] == CFG_LIB_YAML) #[Bool]YAMLライブラリを使うか
    end

    #=================================================================
    # 概　要: GCOVを取得するかを返す
    #=================================================================
    def enable_gcov?()
      return (@@hConf[CFG_ENABLE_GCOV] == true) #[Bool]GCOVを取得するか
    end

    #=================================================================
    # 概　要: ログを取得するかを返す(現状不使用)
    #=================================================================
    def enable_log?()
      return (@@hConf[CFG_ENABLE_LOG] == true) #[Bool]ログを取得するか
    end

    #=================================================================
    # 概　要: 値を取得（将来的に廃止）
    #=================================================================
    def get(sKey)
      check_class(String, sKey)  # キー

      return @@hConf[sKey] # [String, Bool]取得値
    end

    #=================================================================
    # 概　要: 保持している情報をリセットする
    #       : （カバレッジテストプログラム用）
    #=================================================================
    def reset()
      @@hConf = {}
    end
  end
end
