# -*- coding: utf-8 -*-
#
#  TECS Generator
#      Generator for TOPPERS Embedded Component System
#  
#   Copyright (C) 2008-2014 by TOPPERS Project
#--
#   上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
#   ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
#   変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
#   (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
#       権表示，この利用条件および下記の無保証規定が，そのままの形でソー
#       スコード中に含まれていること．
#   (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
#       用できる形で再配布する場合には，再配布に伴うドキュメント（利用
#       者マニュアルなど）に，上記の著作権表示，この利用条件および下記
#       の無保証規定を掲載すること．
#   (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
#       用できない形で再配布する場合には，次のいずれかの条件を満たすこ
#       と．
#     (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
#         作権表示，この利用条件および下記の無保証規定を掲載すること．
#     (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
#         報告すること．
#   (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
#       害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
#       また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
#       由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
#       免責すること．
#  
#   本ソフトウェアは，無保証で提供されているものである．上記著作権者お
#   よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
#   に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
#   アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
#   の責任を負わない．
#  
#   $Id: messages.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

#== TECS の生成する各国語化必要な文字列
# 現状、エラーメッセージは英語のみ
# 生成ファイルのコメントとして出力される文字列
class TECSMsg

  #=== TECSMsg#生成するヘッダやテンプレートなどに含めるコメントの取得
  # CDL の文字コードに合わせて、文字コード変換を行う
  def self.get( msg )
    str = @@comment[ msg ]
#2.0    if $KCONV_TECSGEN == $KCONV_CDL
    if $KCONV_TECSGEN == $KCONV_CDL || $KCONV_CDL == Kconv::BINARY then
      return str
    else
      return str.kconv( $KCONV_CDL, $KCONV_TECSGEN )
    end
  end

 #=== TECSMsg#ローカライズされたエラーメッセージを得る
 #body::String   : "S0001 error message body"  の形式
 # S0001 の部分が使用される
 # Generator.error2 から呼び出される
 def self.get_error_message( body )
   body =~ /^[A-Z0-9]+/    # エラー番号を取り出す
   num = $&
   if num then
     msg = @@error_message[ num.to_sym ]
   else
     msg = nil
   end
   if msg == nil then
     m = body
   else
     m = num + " " + msg
   end
   return m
 end

 #=== TECSMsg#ローカライズされたウォーニングメッセージを得る
 # Generator.warning2 から呼び出される
 def self.get_warning_message( body )
   body =~ /^[A-Z0-9]+/    # ウォーニング番号を取り出す
   num = $&
   msg = @@warning_message[ num.to_sym ]
   if msg == nil then
     m = body
   else
     m = num + " " + msg
   end
   return m
 end

 #=== TECSMsg#ローカライズされた情報メッセージを得る
 # Generator.info2 から呼び出される
 def self.get_info_message( body )
   body =~ /^[A-Z0-9]+/    # 情報番号を取り出す
   num = $&
   msg = @@info_message[ num.to_sym ]
   if msg == nil then
     m = body
   else
     m = num + " " + msg
   end
   return m
 end

end
