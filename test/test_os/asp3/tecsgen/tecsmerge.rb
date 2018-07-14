#! /usr/bin/env ruby
# -*- coding: utf-8 -*-

#
#  TECS merger
#      Merger for TECS generated templates
#  
#   Copyright (C) 2008-2017 by TOPPERS Project
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
#   $Id: tecsmerge.rb 2633 2017-04-02 06:02:05Z okuma-top $
#++

#= tecsgen  : TECS のマージャ
#
#Authors::   大山 博司
#Version::   
#Copyright:: Copyright (C) TOPPERS Project, 2008-2017. All rights reserved.
#License::   TOPPERS ライセンスに準拠

=begin
tecsmerge はテンプレートファイルを元に作成されたセルタイプコード（含インライン関数を記述したセルタイプインラインヘッダ）を保守するものである。
セルタイプに以下の改変が加えられた場合に対応する。
・受け口関数が増えた
・受け口の名前が変更された
・受け口関数の名前が変更された

% tecsmerge source_dir   dest_dir
% tecsmerge source_files dest_dir

% tecsmerge -p port_name -f old_name:new_name source_file dest_dir
% tecsmerge -p old_name:new_name source_file dest_dir

source_file の名前は _teml.c, _templ.h で終わっていなくてはならない
dest_file は source_file の _templ を取り除いた名前でなくてはならない
dest_file が存在しない場合、_templ を取り除いた名前でコピーするだけである

以下のキーワードを 含む行を探す

0) 先頭
1) /* #[<PREAMBLE>]#,
2)  * #[</PREAMBLE>]#
3) /* #[<ENTRY_PORT>]#
4)  * #[</ENTRY_PORT>]#
5) /* #[<ENTRY_FUNC>]#
6) {                       # new mode (function header is changed)
    * #[/<ENTRY_FUNC>]#    # when --old-mode
    * #[/ENTRY_FUNC>]#     # for Bug compatibility
7) /* #[<POSTAMBLE>]#
8)  * #[</POSTAMBLE>]#
9) 終わり

上記のうち 1), 3), 5), 7) を開始キーワードと呼ぶ。
なお、特に断りのない場合、9) も開始キーワードに含む。

名前の変更

a) 受け口名の変更

  -p old_port_name:new_port_name

b) 受け口関数名の変更

  -f old_port_name:new_port_name

 old_port_name, new_port_name は、次の形式でなくてはならない。

  受け口名 + '_' + 関数名


i) ヘッダ１

  0) と開始キーワードの間

ii) ヘッダ２

  1) と開始キーワードの間

通常、これが複数存在することはないが、複数存在した場合、すべてファイル
の先頭に出力される。

iii) 受け口ヘッダ

  3) と開始キーワードの間

受け口ヘッダは、後続の受け口関数の受け口を表すものと仮定される。
受け口ヘッダは、重要ではない。

iv) 受け口関数

  5) と開始キーワードの間

=end

require 'optparse'
$n_err   = 0
$b_show  = false
$b_exist = false            # コピー先にファイルがある場合のみマージ
$old_mode = false           # old_mode (関数本体として /<ENTRY_FUNC> の代わりに '{' を使う

#2.0
def write_array( file, array )
  array.each{ |line|
    file.write line
  }
end

class PortRenamer
  @@port_renamer_list = {}
  @@port_renamer_current = nil

  def initialize( old_port_name, new_port_name )
    old_port_name = old_port_name.to_sym
    if new_port_name then
      new_port_name = new_port_name.to_sym
    end
    # p "port_renamer: #{old_port_name}"
    @@port_renamer_list[old_port_name] = self
    @@port_renamer_current = self
    @old_port_name = old_port_name
    @new_port_name = new_port_name
    @func_renamer_list = {}
  end

  def self.add_func( old_name, new_name )
    # p "add_func: #{old_name}"
    if @@port_renamer_current == nil then
      STDERR.puts( "error: #{old_name}: specify -p before -f" )
      n_err += 1
    else
      @@port_renamer_current.add_func( old_name, new_name )
    end
  end

  def add_func( old_name, new_name )
    old_name = old_name.to_sym
    new_name = new_name.to_sym
    if @func_renamer_list[old_name] then
      STDERR.puts( "error: #{old_name}: duplicate in port #{@old_port_name}" )
      $n_err += 1
    elsif old_name == new_name then
      STDERR.puts( "error: #{old_name}: old and new are the same name" )
      $n_err += 1
    end
    @func_renamer_list[old_name] = new_name
  end

  def self.show
    if @@port_renamer_list then
      @@port_renamer_list.each { |pr,pc|
        pc.show
      }
    end
  end

  def self.get_list
    @@port_renamer_list
  end

  attr_accessor  :old_port_name, :new_port_name, :func_renamer_list

  def show
    print "port: #{@old_port_name}"
    if @new_port_name then
      print " => #{@new_port_name}"
    end
    puts ""

    @func_renamer_list.each { |old,new|
      print "  func: #{old} => #{new}\n"
    }
  end
end


class CDLContents
  #@head::             [string]
  #@preamble_comment:: [string]
  #@preamble_body::    [string]
  #@entry_port::       {ep_name=>CDLEntryPort}
  #@entry_port_array:: [CDLEntryPort]
  #@postamble_comment::[string]
  #@postamble_body::   [string]

  @@DELIMITERS = {
    # state             => [ regexp_original,                 [previous states]                   , Regexp ]
    :HEAD               => [ "#[<BeginingOfFile>]#",          []                       ],
    :PREAMBLE_COMMENT   => [ "/* #[<PREAMBLE>]#",             [:HEAD]                  ],
    :PREAMBLE_BODY      => [ " * #[</PREAMBLE>]#",            [:PREAMBLE_COMMENT]      ],
    :ENTRY_COMMENT      => [ "/* #[<ENTRY_PORT>]# PORT_NAME", [:PREAMBLE_BODY, :ENTRY_FUNC_BODY, :ENTRY_FUNC_BODY2] ],
    :ENTRY_BODY         => [ " * #[</ENTRY_PORT>]#",          [:ENTRY_COMMENT]         ],
    :ENTRY_FUNC_COMMENT => [ "/* #[<ENTRY_FUNC>]# FUNC_NAME", [:ENTRY_BODY, :ENTRY_FUNC_BODY, :ENTRY_FUNC_BODY2]    ],
    :POSTAMBLE_COMMENT  => [ "/* #[<POSTAMBLE>]#",            [:ENTRY_FUNC_BODY, :ENTRY_FUNC_BODY2, :PREAMBLE_BODY] ],
    :POSTAMBLE_BODY     => [ " * #[</POSTAMBLE>]#",           [:POSTAMBLE_COMMENT]     ],
    :EOF                => [ "#[</EndOfFile>]#",              [:POSTAMBLE_BODY,:ENTRY_FUNC_BODY, :ENTRY_FUNC_BODY2, :PREAMBLE_BODY]  ],
  }

  @@DELIMITERS_FUNC_BY_COMMENT = {
    :ENTRY_FUNC_BODY    => [ " * #[</ENTRY_FUNC>]#",          [:ENTRY_FUNC_COMMENT]    ],
    :ENTRY_FUNC_BODY2   => [ " * #[/ENTRY_FUNC>]#",           [:ENTRY_FUNC_COMMENT]    ],  # for Bug compatibility
  }

  @@DELIMITERS_FUNC_BY_BRACKET = {
    :ENTRY_FUNC_BODY    => [ "\\\{",                          [:ENTRY_FUNC_COMMENT]    ],
  }

  def self.rewrite_DELIMITERS  delimiters
    delimiters.each{ |stat, stat_info|
      s = stat_info[0].gsub( /([\*\[\]])/, "\\\\\\1" )   # *, [, ] の前に \\ を挿入
      s.gsub!( /\s*\w*_NAME/, "\\s*(\\w*)" )             # ..._NAME を (w*) に変更
      s = "^" + s                                        # ^ を先頭に挿入
      stat_info[2] = Regexp.new( s )
      # p stat_info[2]
    }
  end

  rewrite_DELIMITERS @@DELIMITERS
  rewrite_DELIMITERS @@DELIMITERS_FUNC_BY_COMMENT
  rewrite_DELIMITERS @@DELIMITERS_FUNC_BY_BRACKET

  #=== モードに従い DELIMITERS に DELIMITERS_FUNC_BY_* をマージする
  # mode :OLD_FUNC_BODY, :NEW_FUNC_BODY
  def self.merge_DELIMITERS mode
    case mode
    when :OLD_FUNC_BODY
      @@DELIMITERS.merge! @@DELIMITERS_FUNC_BY_COMMENT
    when :NEW_FUNC_BODY
      @@DELIMITERS.merge! @@DELIMITERS_FUNC_BY_BRACKET
    else
      raise "unknown mode"
    end
  end

  #all_contents:: [string,...]
  def initialize all_contents
    part = []
    line_no = 0
    stat = :HEAD
    arg = nil
    port_name = nil
    func_name = nil

    @entry_port = {}
    @entry_port_array = []

    @head = []
    @preamble_comment = []
    @preamble_body = []
    @postamble_comment = []
    @postamble_body = []

    (all_contents+[nil]).each{ |line|  # nil: EOF
      line_no += 1
      # p "L: #{line}"
      b_delim = false        # デリミタキーワードの行
      @@DELIMITERS.each { |next_stat,stat_info|

        next if next_stat == :HEAD || ( next_stat == :EOF && line != nil )
        # #1002 tecsmerge の非受け口関数 (POSTAMBLE部) の行頭に '{' があるとエラーになる 	
        if (! $old_mode) && ( /^\{/ =~ line ) && ( stat == :PREAMBLE_BODY || stat == :POSTAMBLE_BODY )
          p line + "  next_stat=" + next_stat.to_s + "stat=" + stat.to_s
          next
        end

        # p "R: #{stat_info[0]}"
        if stat_info[2] =~ line || ( line == nil && next_stat == :EOF ) then
          # p "D: #{line}: #{stat}"
          b_delim = true

          found = false
          stat_info[1].each{ |prev_stat|
            if stat == prev_stat then
              found = true
            end
          }
          if ! found then
            STDERR.puts "error #{line_no}: unsuitable previous keyword"
            STDERR.puts "error #{line_no}:   previous:  \"#{@@DELIMITERS[stat][0]}\""
            STDERR.puts "error #{line_no}:   current:   \"#{@@DELIMITERS[next_stat][0]}\""
            expect = ""; delim = ""
            stat_info[1].each{ |prev_stat| expect = "#{expect+delim}\"#{@@DELIMITERS[prev_stat][0]}\""; delim = ", " }
            STDERR.puts "error #{line_no}:   suitable previous: #{expect}"
            $n_err += 1
          end

          case stat
          when :PREAMBLE_COMMENT, :ENTRY_COMMENT, :ENTRY_FUNC_COMMENT, :POSTAMBLE_COMMENT
            part << line
          end

          case stat   # 前の状態
          when :HEAD
            @head = part
          when :PREAMBLE_COMMENT
            @preamble_comment = part
          when :PREAMBLE_BODY
            @preamble_body = part
          when :ENTRY_COMMENT
            port_name = arg.to_sym
            @entry_port[ port_name ] = CDLEntryPort.new
            @entry_port_array << port_name
            @entry_port[ port_name ].entry_comment = part
          when :ENTRY_BODY
            @entry_port[ port_name ].entry_body = part
          when :ENTRY_FUNC_COMMENT
            func_name = arg.to_sym
            if @entry_port[ port_name ] then   # nil なら既にエラー
              @entry_port[ port_name ].entry_func_comment[ func_name ] = part
              @entry_port[ port_name ].entry_func_array << func_name
            end
          when :ENTRY_FUNC_BODY, :ENTRY_FUNC_BODY2
            if @entry_port[ port_name ] then   # nil なら既にエラー
              @entry_port[ port_name ].entry_func_body[ func_name ] = part
            end
          when :POSTAMBLE_COMMENT
            @postamble_comment = part
          when :POSTAMBLE_BODY
            @postamble_body = part
          else
            raise "Unknown state #{stat}"
          end

          case next_stat
          when :PREAMBLE_COMMENT, :ENTRY_COMMENT, :ENTRY_FUNC_COMMENT, :POSTAMBLE_COMMENT
            part = [ line ]
          else
            part = []
          end

          stat = next_stat
          arg = $1     # arg に取っておく
          # p stat, arg
          break
        end
      }

      if ! b_delim then
        part << line
      end
    }
      
  end

  def check template
    # template にないものをチェック
    @entry_port.each{ |port_name, entry_port|
      temp_entry_port = template.entry_port[port_name]
      if temp_entry_port == nil then
        STDERR.puts "info: #{port_name} is deleted port"
        next
      end
      # temp_entry_port.entry_func_body.each{ |f,b|  p f }
      entry_port.entry_func_body.each{ |func_name, func_body|
        if temp_entry_port.entry_func_body[func_name] == nil then
          STDERR.puts "info: #{func_name} is deleted function"
        end
      }
    }
  end

  def rename

    renamed_entry_port = {}
    PortRenamer.get_list.each{ |pon,pr|

      # 対象受け口を捜す
      ep = @entry_port[pon]
      if ep == nil then
        STDERR.puts "warning: #{pon}: renaming port not found"
        next
      end

      # ポートの rename
      pnn = pr.new_port_name    # 置換後の名前
      if pnn then
        # 置換する名前があれば、登録しなおす
        renamed_entry_port[pnn] = @entry_port[pon]
        @entry_port.delete pon
      end

      # 指定された関数の置換
      renamed_func_comment = {}
      renamed_func_body = {}
      pr.func_renamer_list.each{ |old,new|
        ofn = "#{pon}_#{old}".to_sym
        nfn = "#{pon}_#{new}".to_sym
        # p "fnn: #{ofn} #{nfn} #{pon}"
        if ep.entry_func_comment[ofn] == nil then
          STDERR.puts "warning: #{old}: renaming function not found"
          next
        end
        ep.entry_func_array.map! { |fn|
          # p fn, nfn, ofn, fn==ofn ? nfn : fn
          fn==ofn ? nfn : fn
        }
        renamed_func_comment[nfn] = ep.entry_func_comment[ofn]
        renamed_func_body[nfn]    = ep.entry_func_body[ofn]
        ep.entry_func_comment.delete ofn
        ep.entry_func_body.delete    ofn
      }
      ep.entry_func_comment.merge! renamed_func_comment
      ep.entry_func_body.merge!    renamed_func_body

      # ポート名の変更による関数名の置換
      renamed_func_comment = {}
      renamed_func_body = {}
      if pnn then
        ep.entry_func_array.map! { |ofn|
          nfn = ofn.to_s.sub( /#{pon.to_s}/, pnn.to_s ).to_sym
          # p "pnn: #{ofn} #{nfn} #{pon}  #{pnn}"
          if nfn != ofn then
            renamed_func_comment[nfn] = ep.entry_func_comment[ofn]
            renamed_func_body[nfn]    = ep.entry_func_body[ofn]
            ep.entry_func_comment.delete ofn
            ep.entry_func_body.delete    ofn
            nfn
          else
            ofn
          end
        }
        ep.entry_func_comment.merge! renamed_func_comment
        ep.entry_func_body.merge!    renamed_func_body

        # ep.entry_func_comment.each { |f,e| p "FF: #{f}" }
        # ep.entry_func_array.each { |f,e| p "FA: #{f}" }
      end
    }
    # p renamed_entry_port
    @entry_port.merge! renamed_entry_port
  end

  def merge src
    @head = src.head
    @preamble_body = src.preamble_body
    @postamble_body = src.postamble_body

    @entry_port_array.each{ |port_name|
      # p "merging #{port_name}"
      entry_port = @entry_port[ port_name ]
      src_entry_port = src.entry_port[port_name]
      if src_entry_port == nil then
        print "port merged:   #{port_name}\n"
        next
      end
      entry_port.entry_body = src_entry_port.entry_body
      entry_port.entry_func_array.each{ |func_name|
        # p "merging #{func_name}"
        func_body = entry_port.entry_func_body[ func_name ]
        if src_entry_port.entry_func_body[func_name] == nil then
          print "func merged:   #{func_name}\n"
          next
        end
        print "func remained: #{func_name}\n"
        # entry_port.entry_func_comment[func_name] = src_entry_port.entry_func_comment[func_name]
        entry_port.entry_func_body[func_name]    = src_entry_port.entry_func_body[func_name]
      }
    }
  end

  def write file
#2.0    file.write @head
    write_array( file, @head )
#2.0    file.write @preamble_comment
    write_array( file, @preamble_comment )
#2.0    file.write @preamble_body
    write_array( file, @preamble_body )
    @entry_port_array.each{ |port_name| @entry_port[port_name].write file }
#2.0    file.write @postamble_comment
    write_array( file, @postamble_comment )
#2.0    file.write @postamble_body
    write_array( file, @postamble_body )
  end

  attr_accessor :head, :preamble_comment, :preamble_body, :entry_port, :postamble_body
end

class CDLEntryPort
  #@entry_comment::      [string]
  #@entry_body::         [string]
  #@entry_func_comment:: {ep_func_name=>string}
  #@entry_func_body::    {ep_func_name=>string}
  #@entry_func_array::   [string]

  def initialize
    @entry_comment = []
    @entry_body = []
    @entry_func_comment = {}
    @entry_func_body = {}
    @entry_func_array = []
  end

  def write file
#2.0    file.write @entry_comment
    write_array( file, @entry_comment )
#2.0    file.write @entry_body
    write_array( file, @entry_body )
    @entry_func_array.each{ |fnm|
      # p @entry_func_comment[fnm][0]

#2.0      file.write @entry_func_comment[fnm]
      write_array( file, @entry_func_comment[fnm] )
#2.0      file.write @entry_func_body[fnm]
      write_array( file, @entry_func_body[fnm] )
    }
  end

  attr_accessor :entry_comment, :entry_body, :entry_func_comment, :entry_func_body, :entry_func_array
end

def merge( src_file, dst_dir )
  unless src_file =~ /(.*)_templ(.[ch])$/ then
    STDERR.puts( "error: #{src_file}: not end with _templ.c/h" )
    exit 1
  end

  fname = "#{$1}#{$2}"
  dst_file = "#{dst_dir}/#{File.basename fname}"
  if FileTest.file?( dst_file ) then
    print( "merging #{src_file} to #{dst_file}\n" )
    # dst_file の読込み
    begin
      dst = open( dst_file )
#2.0
      set_encoding dst
      old_contents = dst.readlines
    rescue
      STDERR.puts "error: cannot open #{dst_file}"
      $n_err += 1
      exit 1
    ensure
      dst.close
    end
    old = CDLContents.new( old_contents )

    # template の読込み
    begin
      src = open( src_file )
#2.0
      set_encoding src
      new_contents = src.readlines
    rescue
      STDERR.puts "error: cannot open #{src_file}"
      $n_err += 1
      exit 1
    ensure
      src.close
    end
    templ = CDLContents.new( new_contents )

    old.rename
    error_check dst_file, 1

    old.check templ
    error_check dst_file, 2

    templ.merge old
    error_check dst_file, 3

    bkup_file = rename_dst( dst_file )

    begin
      dst = open( dst_file, "w" )
#2.0
      set_encoding dst
      templ.write dst
    ensure
      dst.close
    end

  elsif $b_exist == false then
    # src_file を dst_file へコピー
    begin
      src = File.open( src_file )
#2.0
      set_encoding src
      contents = src.readlines
    rescue
      print "#{src_file}: fail to read\n"
    ensure
      src.close
    end

    begin
      dst = File.open( dst_file, "w" )
      set_encoding dst
#2.0      dst.write( contents )
      contents.each{ |line|
        dst.print line
      }
    rescue
      print "#{dst_file}: fail to write\n"
    ensure
      dst.close
    end
  else
    print "info: #{dst_file} skipped\n"
  end
end

def error_check dst_file, level
  if $n_err > 0 then
    STDERR.puts "=== #{dst_file} not generated because of error ==="
    exit level
  end
end


#=== ファイルのエンコーディングを ASCII-8BIT に変更
# Ruby 1.9 以上の場合に変更
def set_encoding file
  if RUBY_VERSION >= "1.9.0" then
    file.set_encoding "ASCII-8BIT"
  end
end

#=== rename_dst
# dst_file のバックアップファイル名を決定し、リネームする
# 成功すれば、リネーム後のファイル名を返す
# dst_file が存在しなければ（リネームは行われず）nil を返す。
def rename_dst( dst_file )
  begin
    File.stat dst_file
  rescue
    STDERR.puts( "info: backup not generated for #{dst_file}" )
    # なければ終わり
    return nil
  end

  i = 0
  found = true
  while found == true
    begin
      i = i + 1
      bkup_file = "#{dst_file}_#{i}"
      File.stat bkup_file
    rescue
      found = false
    end
  end

  begin
    File.rename( dst_file, bkup_file )
    STDERR.puts( "info: backup generated for #{dst_file} to #{bkup_file}" )
    return bkup_file
  rescue
    STDERR.puts( "error: fail to rename backup file: #{bkup_file}" )
    exit 1
  end

  return nil
end

ARGV.options {|parser|
  parser.banner =<<EOT
Usage: tecsmerge [options] files_templ.c   src_dir
       tecsmerge [options] gen_dir         src_dir
EOT
  parser.on('-e', '--exist-only',  "merge if exist in destination directory") { |fs|
    $b_exist = true
  }
  parser.on('-p', '--port=old_name:new_name',  "specify entry port name change") { |ps|
    p = ps.split( ':' )
    PortRenamer.new( p[0], p[1] )
  }
  parser.on('-f', '--func=old_name:new_name',  "specify entry function name change") { |fs|
    f = fs.split( ':' )
    PortRenamer.add_func( f[0], f[1] )
  }
  parser.on('-s', '--show_change_set',  "show change set. all substitute") {
    $b_show = true
  }
  parser.on('-o', '--old-mode',  "old mode (function head not substituted)") { |fs|
    $old_mode = true
  }
  parser.version = #{$version}
  parser.release = nil
  parser.parse!
#  if ARGV.empty?
  if ARGV.length < 2 then
    puts parser.help
    exit 1
  end
}

if $b_show then
  PortRenamer.show
end

if $old_mode then
  CDLContents.merge_DELIMITERS :OLD_FUNC_BODY
else
  CDLContents.merge_DELIMITERS :NEW_FUNC_BODY
end

if $n_err > 0 then
  STDERR.puts( "#{$n_err} errors" )
  exit 1
end

src = ARGV[0,ARGV.length-1]
dst = ARGV[-1]

begin
  stat = File.stat dst
rescue
  STDERR.puts( "error: #{dst}: not found" )
  exit 1
ensure
  if stat && ! stat.directory? then
    STDERR.puts( "error: #{dst}: not directory" )
    exit 1
  end
end

src.each { |s|
  begin
    stat = File.stat s
  rescue
    STDERR.puts( "error: #{s}: not found or cannot access" )
    exit 1
  end

  if stat.directory? then
    Dir.foreach(s) {|file|
      if file =~ /_templ.[ch]$/ then
        merge( "#{s}/#{file}", dst )
      end
    }
  elsif stat.file? then
    merge( s, dst )
  end
}
