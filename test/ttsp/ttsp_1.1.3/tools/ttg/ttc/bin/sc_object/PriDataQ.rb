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
#  $Id: PriDataQ.rb 9 2012-09-11 01:47:48Z nces-shigihara $
#
require "ttc/bin/sc_object/SCObject.rb"

#=====================================================================
# CommonModule
#=====================================================================
module CommonModule
  #===================================================================
  # クラス名: PriDataQ
  # 概    要: 優先度データキューの情報を処理するクラス
  #===================================================================
  class PriDataQ < SCObject
    #=================================================================
    # 概  要: 属性チェック
    #=================================================================
    def attribute_check()
      aErrors = []
      begin
        super()
      rescue TTCMultiError
        aErrors = $!.aErrors
      end

      begin
        # stsklist
        sAtr = TSR_PRM_STSKLIST
        if (is_specified?(sAtr))
          cProc = Proc.new(){|hData, aPath|
            # 要素のチェック
            if (hData.is_a?(Hash))
              # 必要な要素が指定されているかチェック
              unless (hData.has_key?(TSR_VAR_DATA) && hData.has_key?(TSR_VAR_DATAPRI))
                sErr = sprintf(ERR_REQUIRED_KEY, "#{TSR_VAR_DATA} and #{TSR_VAR_DATAPRI}")
                raise(YamlError.new(sErr, aPath))
              else
                aProcErrors = []
                hData.each{|atr, val|
                  begin
                    case atr
                    when TSR_VAR_DATA
                      check_attribute_unsigned(atr, val, aPath)
                    when TSR_VAR_DATAPRI
                      check_attribute_range(atr, val, TTC_MAX_PRI, TTC_MIN_PRI, aPath)
                    else
                      sErr = sprintf(ERR_UNDEFINED_KEY, atr)
                      raise(YamlError.new(sErr, aPath))
                    end
                  rescue YamlError
                    aProcErrors.push($!)
                  end
                }
                check_error(aProcErrors)
              end
            else
              sErr = sprintf(ERR_LIST_ITEM_INVALID_TYPE, sAtr, Hash, hData.class())
              raise(YamlError.new(sErr, aPath))
            end
          }
          attribute_check_task_list(sAtr, @hState[sAtr], cProc)
        end
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      begin
        # rtsklist
        sAtr = TSR_PRM_RTSKLIST
        if (is_specified?(sAtr))
          cProc = Proc.new(){|hData, aPath|
            if (hData.is_a?(Hash))
              aProcErrors = []
              hData.each{|atr, val|
                begin
                  case atr
                  when TSR_VAR_VARDATA, TSR_VAR_VARPRI
                    check_attribute_variable(atr, val, aPath)
                  else
                    sErr = sprintf(ERR_UNDEFINED_KEY, atr)
                    raise(YamlError.new(sErr, aPath))
                  end
                rescue YamlError
                  aProcErrors.push($!)
                end
              }
              check_error(aProcErrors)
            elsif (!hData.nil?())
              sErr = sprintf(ERR_LIST_ITEM_INVALID_TYPE_NIL, sAtr, Hash, hData.class())
              raise(YamlError.new(sErr, aPath))
            end
          }
          attribute_check_task_list(sAtr, @hState[sAtr], cProc)
        end
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      begin
        # datalist
        sAtr = TSR_PRM_DATALIST
        if (is_specified?(sAtr))
          check_attribute_type(sAtr, @hState[sAtr], Array, false, @aPath)
          aPath = @aPath + [sAtr]
          aTmpErrors = []
          @hState[sAtr].each_with_index{|hData, nIndex|
            # リストの要素がHashか
            unless (hData.is_a?(Hash))
              sErr = sprintf(ERR_LIST_INVALID_TYPE, sAtr, Hash, hData.class())
              raise(YamlError.new(sErr, aPath + [nIndex]))
            end
            # 必要な要素が指定されているかチェック
            unless (hData.has_key?(TSR_VAR_DATA) && hData.has_key?(TSR_VAR_DATAPRI))
              sErr = sprintf(ERR_REQUIRED_KEY, "#{TSR_VAR_DATA}\" and \"#{TSR_VAR_DATAPRI}")
              raise(YamlError.new(sErr, aPath + [nIndex]))
            end
            # 要素の内容チェック
            hData.each{|atr, val|
              begin
                case atr
                when TSR_VAR_DATA
                  check_attribute_unsigned(atr, val, aPath + [nIndex])
                when TSR_VAR_DATAPRI
                  check_attribute_range(atr, val, TTC_MAX_PRI, TTC_MIN_PRI, aPath + [nIndex])
                else
                  sErr = sprintf(ERR_UNDEFINED_KEY, atr)
                  raise(YamlError.new(sErr, aPath + [nIndex]))
                end
              rescue YamlError
                aTmpErrors.push($!)
              end
            }
          }
          check_error(aTmpErrors)
        end
      rescue YamlError
        aErrors.push($!)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: オブジェクトチェック
    #=================================================================
    def object_check(bIsPre)
      check_class(Bool, bIsPre)  # pre_conditionか

      aErrors = []
      begin
        super(bIsPre)
      rescue TTCMultiError
        aErrors.concat($!.aErrors)
      end

      # datalistのサイズ
      if (@hState[TSR_PRM_DATALIST].nil?())
        nDataCount = 0
      else
        nDataCount = @hState[TSR_PRM_DATALIST].size()
      end
      ### T3_PDQ001: 管理領域に空きがあるのに送信待ちタスクがいる
      if (@hState[TSR_PRM_DATACNT] > nDataCount && !@hState[TSR_PRM_STSKLIST].nil?() && !@hState[TSR_PRM_STSKLIST].empty?())
        aErrors.push(YamlError.new("T3_PDQ001: " + ERR_SEND_WAITING_HAVE_SPACE, @aPath))
      end
      ### T3_PDQ002: 管理領域にデータを持っているのに受信待ちタスクがいる
      if (nDataCount > 0 && !@hState[TSR_PRM_RTSKLIST].nil?() && !@hState[TSR_PRM_RTSKLIST].empty?())
        aErrors.push(YamlError.new("T3_PDQ002: " +ERR_RECV_WAITING_HAVE_DATA, @aPath))
      end
      ### T3_PDQ003: 管理領域以上のデータを持っている
      if (nDataCount > @hState[TSR_PRM_DATACNT])
        aErrors.push(YamlError.new("T3_PDQ003: " +ERR_DATALIST_HAVE_OVER_DATA, @aPath))
      end

      check_error(aErrors)
    end

    #=================================================================
    # 概  要: 初期値を補完する
    #=================================================================
    def complement_init_object_info()
      super()

      # pdqatr
      unless (is_specified?(TSR_PRM_PDQATR))
        @hState[TSR_PRM_ATR] = "ANY_ATT_PDQ"
      end
      # maxdpri
      unless (is_specified?(TSR_PRM_MAXDPRI))
        @hState[TSR_PRM_MAXDPRI] = "DATA_PRI_MAX"
      end
      # pdqcnt
      unless (is_specified?(TSR_PRM_PDQCNT))
        @hState[TSR_PRM_DATACNT] = "ANY_DATA_CNT"
      end
    end

    #=================================================================
    # 概  要: 受信待ちタスクリスト内の変数と型の組み合わせ一覧を返す
    #=================================================================
    def get_rtsklist_variable()
      hVars = {}
      unless (@hState[TSR_PRM_RTSKLIST].nil?())
        @hState[TSR_PRM_RTSKLIST].each{|hTask|
          hTask.each{|sTask, hData|
            unless (hData.nil?())
              hVars[sTask] = {}
              hData.each{|sAtr, sVarName|
                case sAtr
                when TSR_VAR_VARDATA
                  hVars[sTask][sVarName] = [TYP_INTPTR_T]
                when TSR_VAR_VARPRI
                  hVars[sTask][sVarName] = [TYP_PRI]
                end
              }
            end
          }
        }
      end

      return hVars  # [Hash]受信待ちタスクリスト内の変数と型の組み合わせ一覧
    end
  end
end
