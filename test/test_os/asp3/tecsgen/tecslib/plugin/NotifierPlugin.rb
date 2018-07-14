# -*- coding: utf-8 -*-
#
#  Copyright (C) 2015 by Ushio Laboratory
#              Graduate School of Engineering Science, Osaka Univ., JAPAN
#  Copyright (C) 2015-2016 by Embedded and Real-Time Systems Laboratory
#              Graduate School of Information Science, Nagoya Univ., JAPAN
#

NotifierPluginArgProc = {
	"factory" => Proc.new { |obj, rhs| obj.set_factory(rhs) },
	"output_file" => Proc.new { |obj, rhs| obj.set_factory_output_file(rhs) }
}

class NotifierPlugin < CelltypePlugin

	# ---------- アダプタ関数の生成 -------------
	#
	# siHandlerBodyの受け口関数は，タイムイベント通知の通知先として直接指定する
	# ことはできない．シグネチャが一致していないことが理由である．このため，アダ
	# プタとして動作する関数を生成し，カーネルからの呼出しをTECSの呼出しに変換で
	# きるようにする必要がある．
	#
	# 基本的には，受け口毎にアダプタ関数を生成すれば十分である．しかし，これでは
	# メモリ消費量が不必要に増加してしまう．そこで，通知先関数にintptr_t型の引数
	# を渡せることに着目し，関数の"一般化"を図る．すなわち受け口のある属性(ここ
	# では，結合先のセル，添字などを指す)を，アダプタ関数の引数として受け取れる
	# ようにし，1個のアダプタ関数を2個以上の結合に対し用いることができるように
	# する．
	#
	# アダプタ関数の属性について整理すると，
	#  - 受け口関数 - 一般化を行うと，実行時コストが大きく増大してしまうことが
	#    確認されている．このため，一般化は行わない．EntryPropertyにも含めない．
	#  - セルインデックス - CELLIDX型で，型の規定はないが，"ポインタ値であったり
	#    整数値であったりする。" (TECS 5.3.6) より，インデックスかポインタある
	#    ことが分かる．インデックスだとすると，この値はセルCBのアドレッシングに
	#    使用されるので，intptr_tに収まる筈である．ポインタの場合，当然intptr_t
	#    に収まる．
	#  - 受け口配列の添字 - int_t．同様にintptr_tに収まる筈である．
	# これらのうち，セルインデックスと受け口配列の添字はパラメータに含めることが
	# できそうであるが，両方は無理である．
	# 両方を格納した配列を生成し，その配列の要素へのポインタを渡すようにするとい
	# う選択肢も可能であるが，実行速度を優先するために，この方法はとらなかった．
	#
	# このため，一般化は以下のパターンに分類して行う．
	#
	# 1. セルインデックスのみ一般化．セルインデックスのパターン数が受け口配列の
	#    添字のパターン数より多いか，あるいは受け口が配列でない場合に行われる．
	# 2. 受け口配列の添字のみ一般化．この場合，セルごとに異なる関数を用いる．
	#
	# これより，各受け口関数について，アダプタ関数の生成個数は，
	#         O(min{セルインデックスのパターン数, 添字のパターン数})
	# となる．
	#
	# プラグインでの処理をワンパスで行うために，tecsgen.cfgではアダプタ関数を直
	# 接指定するのではなく，代わりにアダプタ関数を表すマクロを使用する．この
	# マクロはアダプタ関数ハンドルと呼ぶことにする．
	# アダプタ関数ハンドルは，次の2個の要素から成る．
	#  - アダプタ関数へののポインタ
	#  - アダプタ関数の引数
	#
	#   $Id: NotifierPlugin.rb 2640 2017-06-03 11:27:12Z okuma-top $

	# @private
	class AdapterGenerator

		# 結合先に関する属性を含む．セル，受け口配列の添字から成る．
		# 同一のEntryPropertyとなる結合は，全く同じ方法でその受け口関数を呼び
		# 出せる．
		# @private
		class EntryProperty
			# @return [Cell] 受け口側のセル．
			attr_reader :cell

			# @return [Integer, nil] 受け口配列の添字．配列でない場合はnil．
			attr_reader :subscript

			def initialize(cell, subscript)
				@cell = cell
				@subscript = subscript
			end

			def self.from_join(join)
				EntryProperty.new(join.get_rhs_cell, join.get_rhs_subscript)
			end

			# 同値性の定義．Hashのキーとして使用するのに必要．
			def eql?(o) @cell == o.cell && @subscript == o.subscript end
			def hash() @cell.hash ^ @subscript.hash end
		end

		# @private
		class EntryPort
			# @param [Port] port 結合先のセルのセルタイプの受け口．
			def initialize(port, prefix)
				@port = port
				@global_name = "#{prefix}_#{@port.get_celltype.get_global_name}_#{@port.get_name}"

				# 受け口関数名．siHandlerBodyを想定しているので，関数名はmainで固定である．
				@entry_fn_name = "#{@port.get_celltype.get_global_name}_#{@port.get_name}_main"

				@props = [] 	# Array<EntryProperty>
				@prop_map = {}	# Hash<EntryProperty, Integer>
			end

			# @return [String] グローバルに一意(なものとして扱えるよう)な識別子．
			attr_reader :global_name

			# @return [Port]
			attr_reader :port

			# 指定したEntryPropertyに対応するアダプタ関数ハンドルを取得する．
			# @param [EntryProperty] ep
			# @return [String] アダプタ関数ハンドル．
			# @private
			def adapter_handle_for_entry_property(ep)
				index = @prop_map.fetch(ep)
				return [
					"#{@global_name}_#{index}_fp",
					"#{@global_name}_#{index}_arg"
				]
			end

			# 結合先の情報に応じたアダプタ関数をソース・ヘッダーに出力する．
			# 一般化指定は，`cell`または`subscript`の一方のみ行うことができる．
			#
			# @param [AdapterGenerator] context
			# @param [String] fn_name 関数名．
			# @param [Cell, Symbol] cell セル．セルについて一般化する場合は `:generic`
			# @param [Integer, Symbol, nil] subscript 添字．添字について一般化する場合は `:generic`
			# @private
			def generate_inner(context, fn_name, cell, subscript)
				source_file = context.source_file
				header_file = context.header_file

				source_file.print "void #{fn_name}(intptr_t extinf) {\n"

				params = []
				ct = @port.get_celltype

				# シングルトンセルタイプ以外では，CELLIDXの指定が必要．
				unless ct.is_singleton?
					if cell == :generic
						params << "(CELLIDX)extinf"
					else
						# セルのCELLIDXを得る
						if ct.has_INIB? || ct.has_CB?
							params << ct.get_name_array(cell)[7]
						else
							params << "0"
						end
					end
				end

				# 受け口配列の添字．
				if @port.get_array_size
					if subscript == :generic
						params << "(int_t)extinf"
					else
						params << "#{subscript}"
					end
				end

				params_str = params.join(", ")

				source_file.print "\t#{@entry_fn_name}(#{params_str});\n"
				source_file.print "}\n\n"

				header_file.print "extern void #{fn_name}(intptr_t extinf);\n\n"

			end

			# 指定したJoinに対応するアダプタ関数ハンドルを取得する．
			# @return [Array] アダプタ関数ハンドル．
			def make_adapter_handle(join)
				prop = EntryProperty.from_join(join)
				unless @prop_map.has_key?(prop)
					@prop_map[prop] = @props.length
					@props << prop
				end
				return adapter_handle_for_entry_property(prop)
			end

			# ソース・ヘッダーの記述を生成する．
			# @param [AdapterGenerator] context
			def generate(context)
				header_file = context.header_file
				return if @props.empty?

				ct = @port.get_celltype

				header_file.print "/*\n * #{@global_name}\n"

				cells = @props.group_by { |prop| prop.cell }
				subscripts = @props.group_by { |prop| prop.subscript }
				no_cellidx = false
				if !(ct.has_INIB? || ct.has_CB?)
					# CB, INIB最適化により，CB, INIBが両方不要になったケース．
					# CELLIDXが不要であるので，セルについて一般化しても意味
					# はないので，添字による一般化を選択する．
					generalize_by_cell_idx = false
					no_cellidx = true

					# 全てのセルを同一視する．
					cells = { @props[0].cell => @props }

					header_file.print " * No INIB & CB: generalized by subscript\n"
				elsif @port.get_array_size
					# 一般化パターンの分類を行うために，受け口側セルや添字の
					# パターン数を分析して，最適な方を選択する．
					generalize_by_cell_idx = cells.length >= subscripts.length
					if generalize_by_cell_idx
						header_file.print " * more cells than subscripts: generalized by cell\n"
					else
						header_file.print " * more subscripts than cells: generalized by subscript\n"
					end
				else
					# 常にCELLIDXで一般化
					generalize_by_cell_idx = true
					header_file.print " * non-array entry port: generalized by cell\n"
				end

				header_file.print " */\n\n"

				if generalize_by_cell_idx
					# CELLIDXについて一般化
					subscripts.each { |subscript, props|
						if subscript
							fn_name = "#{@global_name}_adap_#{subscript}"
						else
							# 受け口配列でない場合
							fn_name = "#{@global_name}_adap"
						end

						generate_inner context, fn_name,
							:generic, subscript

						props.each { |prop|
							handle = adapter_handle_for_entry_property(prop)

							# セルのCELLIDXを得る
							if ct.has_INIB? || ct.has_CB?
								idx = ct.get_name_array(prop.cell)[7]
							else
								idx = "0"
							end
							header_file.print "\#define #{handle[0]} &#{fn_name}\n"
							header_file.print "\#define #{handle[1]} #{idx}\n\n"
						}
					}
				else
					# 添字について一般化
					cells.each { |cell, props|
						if no_cellidx
							# CB/INIB なし
							fn_name = "#{@global_name}_adap"
						else
							fn_name = "#{@global_name}_adap_#{cell.get_global_name}"
						end

						generate_inner context, fn_name,
							cell, :generic

						props.each { |prop|
							handle = adapter_handle_for_entry_property(prop)

							header_file.print "\#define #{handle[0]} &#{fn_name}\n"
							header_file.print "\#define #{handle[1]} #{prop.subscript || 0}\n\n"
						}
					}
				end
			end
		end

		# @private
		attr :source_file

		# @private
		attr :header_file

		# @param [String] celltype_name ハンドラ関数のセルタイプ．
		# @param [String] prefix 名前衝突を防ぐためのプレフィックス．
		def initialize(celltype_name, prefix)
			@celltype_name = celltype_name
			@prefix = prefix

			# Hash<Port, EntryPort>
			@entry_ports = {}
		end

		# ===AdapterGenerator#make_adapter_handle===
		# 指定した結合の呼出しを行うためのアダプタ関数ハンドルを生成する．
		# @return [Array] アダプタ関数ハンドル．
		def make_adapter_handle(join)
			entry_port = @entry_ports[join.get_rhs_port]
			unless entry_port
				entry_port = EntryPort.new(join.get_rhs_port,
					"#{@celltype_name}_#{@prefix}")
				@entry_ports[join.get_rhs_port] = entry_port
			end
			return entry_port.make_adapter_handle(join)
		end

		# ===AdapterGenerator#finish===
		# 各受け口に対し，アダプタ関数を生成する．
		def finish
			@source_file = AppFile.open( "#{$gen}/#{@celltype_name}.c" )
	        @source_file.print "\n/* Generated by #{self.class.name} */\n\n"
	        @source_file.print "\#include \"#{@celltype_name}_aux.h\"\n\n"
	        @source_file.print "\#include \"#{@celltype_name}_tecsgen.h\"\n\n"

			@header_file = AppFile.open( "#{$gen}/#{@celltype_name}.h" )
	        @header_file.print "\n/* Generated by #{self.class.name} */\n\n"

			# NotifierPluginを使用するセルタイプが複数ある場合，それぞれに
			# 対しAdapterGenerator#finishが呼び出される．tTimeEventHandler.hに
			# 続けて書き込んでしまうと，ヘッダーガードの関係で2回目以降の記述
			# が読み込まれなくなってしまう．このため，ファイル名 + セルタイプ名
			# という少し特殊なヘッダーガードを用いる．
			header_guard = "#{@celltype_name}_H_#{@prefix}"

	        @header_file.print "\#ifndef #{header_guard}\n"
	        @header_file.print "\#define #{header_guard}\n\n"

	        # カーネルコンフィギュレータを実行する際，ハンドラ受け口のセルタイプ
	        # のセルCBの定義が必要な場合がある．
	        @header_file.print "\#include \"#{@celltype_name}_aux.h\"\n\n"

			# 結合先のセルタイプの定義は，自分のセルのtecsgen.hよりも先に
			# 読み込まなければならないが，このプラグインが複数実行されると，
			# 順序が崩れてしまう．そこで，結合先のセルタイプの定義はもう一つの
			# ヘッダーファイル(tCelltypeName_aux.h)から読み込むようにする．
			aux_header_file = AppFile.open( "#{$gen}/#{@celltype_name}_aux.h" )
	        aux_header_file.print "\n/* Generated by #{self.class.name} */\n\n"

			aux_header_guard = "#{@celltype_name}_AUX_H_#{@prefix}"

	        aux_header_file.print "\#ifndef #{aux_header_guard}\n"
	        aux_header_file.print "\#define #{aux_header_guard}\n\n"

			cb_type_only_guard = "#{@celltype_name}_AUX_H_#{@prefix}_CB_TYPE_ONLY"

			# 結合先のセルタイプの定義を読み込む
			aux_header_file.print "#ifndef TOPPERS_CB_TYPE_ONLY\n"
		    aux_header_file.print "#define TOPPERS_CB_TYPE_ONLY\n"
		    aux_header_file.print "#define #{cb_type_only_guard}\n"
			aux_header_file.print "#endif\n"
			@entry_ports.values.map { |ep|
				ep.port.get_celltype
			}.uniq.each { |ct|
				hname = "#{ct.get_global_name}_tecsgen.#{$h_suffix}"
				aux_header_file.print "\#include \"#{hname}\"\n"
			}
			aux_header_file.print "#ifdef #{cb_type_only_guard}\n"
		    aux_header_file.print "#undef #{cb_type_only_guard}\n"
		    aux_header_file.print "#undef TOPPERS_CB_TYPE_ONLY\n"
			aux_header_file.print "#endif\n\n"

			aux_header_file.print "\#endif\n"
			aux_header_file.close

			@entry_ports.each { |port, entry_port|
				entry_port.generate self
			}

	        @header_file.print "\#endif\n"

			@source_file.close
			@header_file.close
		end

	end

	# ------ 通知のハンドラの種類の定義 -------

	class Handler
		def initialize(call_port_name)
			@call_port_name = call_port_name
		end

		attr :call_port_name
	end

	# 通常のハンドラ
	EVENT_HANDLER = Handler::new("ciNotificationHandler")

	# エラーハンドラ (通常のハンドラが失敗した場合に呼び出される)
	ERROR_HANDLER = Handler::new("ciErrorNotificationHandler")

	HANDLERS = [
		EVENT_HANDLER,
		ERROR_HANDLER
	]

	class HandlerAttribute
		def initialize(name, error_name = nil)
			@name = name
			@error_name = error_name || (name + 'ForError')
		end

		def name_for_handler(handler)
			case handler
			when EVENT_HANDLER then return @name
			when ERROR_HANDLER then return @error_name
			else raise "unknown handler #{handler}"
			end
		end
	end

	# ------ 通知の属性の定義 -------
	#
	# ハンドラタイプに合致しない属性が指定された場合に
	# エラーを出力できるよう、全ての属性をここで列挙する。

	SETVAR_ADDR_ATTR =   HandlerAttribute::new("setVariableAddress")
	SETVAR_VALUE_ATTR =  HandlerAttribute::new("setVariableValue")
	INCVAR_ADDR_ATTR =   HandlerAttribute::new("incrementedVariableAddress")
	SNDDTQ_VALUE_ATTR =  HandlerAttribute::new("dataqueueSentValue")
	SETFLG_FLAG_ATTR =   HandlerAttribute::new("flagPattern")

	ATTRS = [
		SETVAR_ADDR_ATTR,
		SETVAR_VALUE_ATTR,
		INCVAR_ADDR_ATTR,
		SNDDTQ_VALUE_ATTR,
		SETFLG_FLAG_ATTR
	]

	# ------ ハンドラタイプの定義 -------

	class BaseHandlerType

		def initialize()
			super

			# HandlerAttribute[]
			@required_attributes = []
		end

		attr :required_attributes

	    #=== NotifierPlugin#BaseHandlerType#validate_join
	    # 指定したセルの結合先が、このハンドラタイプに該当するかを検証
	    # handler:: Handler : ハンドラ
	    # cell:: Cell : セル
	    # join:: Join : 結合 (declarationがPortであるもの)
		def validate_join(handler, cell, join)
        	return !generate_attr_map(handler, cell).nil?
		end

	    #=== NotifierPlugin#BaseHandlerType#generate_attr_map
	    # 指定したセルの属性と、既知のHandlerAttributeのマッピングを
	    # 生成し、Hash<HandlerAttribute, Join> (各属性とそれに対応する
	    # Join(declarationがDeclのもの)を表すHash)、あるいは、
		# マッピングが行えない場合(属性の不足、過剰)はnilを返す。
	    #
	    # handler:: Handler : ハンドラ
	    # cell:: Cell : セル
		def generate_attr_map(handler, cell)
			map = {}

			join_list = cell.get_join_list

			ATTRS.each { |known_attr|
				attr_name = known_attr.name_for_handler(handler)
				join = join_list.get_item(attr_name.to_sym)

				# このセルタイプにおいて必須の属性か?
				is_required = @required_attributes.include?(known_attr)

				# 属性の指定が不足している? or 過剰?
				# 注: ハンドラタイプの判別には、セルで値が指定されているか
				#     が考慮される。セルタイプで初期値が指定されていても、
				#     それはハンドラタイプの決定に影響しない。
				return nil if join.nil? != !is_required

				# 必要のない属性であり、指定もされていないので飛ばす
				next if join.nil?

				# TODO: attrの結合であることを検証

				map[known_attr] = join
			}

			return map
		end

	    #=== NotifierPlugin#BaseHandlerType#gen_cfg_handler_type
	    # タイムイベントの通知の種類を表すコンフィギュレータの記述を生成し、Stringまたはnilを返す
	    # handler:: Handler : ハンドラ
		def gen_cfg_handler_type(handler)
        	raise "called abstract method gen_cfg_handler_type"
		end

	    #=== NotifierPlugin#BaseHandlerType#gen_cfg_handler_parameters
	    # タイムイベントの通知の引数を表すコンフィギュレータの記述を生成し、String[]を返す
	    # handler:: Handler : ハンドラ
	    # join:: Join : 結合 (declarationがPortであるもの)
	    # attrMap:: Hash<HandlerAttribute, Join> :
	    #     各属性とそれに対応するJoin (declarationがDeclのもの)
	    # cell:: Cell : セル
		# adpt_gen:: AdapterGenerator : アダプタ関数を生成するオブジェクト
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
        	return nil
		end

	    #=== NotifierPlugin#BaseHandlerType#might_fail
	    # 通知の際、エラーが発生し、その結果エラー通知を呼ぶ必要が生じる
	    # かどうかを返す。
		def might_fail
			return false
		end

	end
	class BaseTaskHandlerType < BaseHandlerType
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join && join.get_rhs_cell.get_celltype.get_name == :tTask
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			taskCell = join.get_cell
			id_attr_join = taskCell.get_join_list.get_item(:id)
			id_attr = join.get_rhs_cell.get_celltype.find(:id)
			if id_attr_join
				# セル生成時に初期化する場合
				id = id_attr_join.get_rhs.to_s
			else
				# セルタイプの初期化値を使う場合
				id = id_attr.get_initializer.to_s
			end

			# $id$等の置換
			name_array = taskCell.get_celltype.get_name_array(taskCell)
			id = taskCell.get_celltype.subst_name(id, name_array)

        	return [id]
		end
		def might_fail
			return true
		end
	end
	class ActivateTaskHandlerType < BaseTaskHandlerType
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join.get_port_name == :eiActivateNotificationHandler
		end
		def gen_cfg_handler_type(handler)
			case handler
				when EVENT_HANDLER then return "TNFY_ACTTSK"
				when ERROR_HANDLER then return "TENFY_ACTTSK"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class WakeUpTaskHandlerType < BaseTaskHandlerType
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join.get_port_name == :eiWakeUpNotificationHandler
		end
		def gen_cfg_handler_type(handler)
			case handler
				when EVENT_HANDLER then return "TNFY_WUPTSK"
				when ERROR_HANDLER then return "TENFY_WUPTSK"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class SetVariableHandlerType < BaseHandlerType
		def initialize()
			super
			@required_attributes = [
				SETVAR_ADDR_ATTR,
				SETVAR_VALUE_ATTR
			] # .to_set
		end
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join.nil? &&
				handler == EVENT_HANDLER
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			var_addr = attrMap[SETVAR_ADDR_ATTR].get_rhs.to_s
			var_value = attrMap[SETVAR_VALUE_ATTR].get_rhs.to_s

			# $id$等の置換
			name_array = cell.get_celltype.get_name_array(cell)
			var_addr = cell.get_celltype.subst_name(var_addr, name_array)
			var_value = cell.get_celltype.subst_name(var_value, name_array)

        	return [var_addr, var_value]
		end
		def gen_cfg_handler_type(handler)
			case handler
				when EVENT_HANDLER then return "TNFY_SETVAR"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class SetVariableToErrorCodeHandlerType < BaseHandlerType
		def initialize()
			super
			@required_attributes = [
				SETVAR_ADDR_ATTR
			] # .to_set
		end
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join.nil? &&
				handler == ERROR_HANDLER
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			var_addr = attrMap[SETVAR_ADDR_ATTR].get_rhs.to_s

			# $id$等の置換
			name_array = cell.get_celltype.get_name_array(cell)
			var_addr = cell.get_celltype.subst_name(var_addr, name_array)

        	return [var_addr]
		end
		def gen_cfg_handler_type(handler)
			case handler
				when ERROR_HANDLER then return "TENFY_SETVAR"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class IncrementVariableHandlerType < BaseHandlerType
		def initialize()
			super
			@required_attributes = [
				INCVAR_ADDR_ATTR
			] # .to_set
		end
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join.nil?
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			var_addr = attrMap[INCVAR_ADDR_ATTR].get_rhs.to_s

			# $id$等の置換
			name_array = cell.get_celltype.get_name_array(cell)
			var_addr = cell.get_celltype.subst_name(var_addr, name_array)

        	return [var_addr]
		end
		def gen_cfg_handler_type(handler)
			case handler
				when EVENT_HANDLER then return "TNFY_INCVAR"
				when ERROR_HANDLER then return "TENFY_INCVAR"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class SignalSemaphoreHandlerType < BaseHandlerType
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join && join.get_rhs_cell.get_celltype.get_name == :tSemaphore
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			semaphoreCell = join.get_cell
			id_attr_join = semaphoreCell.get_join_list.get_item(:id)
			id_attr = join.get_rhs_cell.get_celltype.find(:id)
			if id_attr_join
				# セル生成時に初期化する場合
				id = id_attr_join.get_rhs.to_s
			else
				# セルタイプの初期化値を使う場合
				id = id_attr.get_initializer.to_s
			end

			# $id$等の置換
			name_array = semaphoreCell.get_celltype.get_name_array(semaphoreCell)
			id = semaphoreCell.get_celltype.subst_name(id, name_array)

        	return [id]
		end
		def might_fail
			return true
		end
		def gen_cfg_handler_type(handler)
			case handler
				when EVENT_HANDLER then return "TNFY_SIGSEM"
				when ERROR_HANDLER then return "TENFY_SIGSEM"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class SetEventflagHandlerType < BaseHandlerType
		def initialize()
			super
			@required_attributes = [
				SETFLG_FLAG_ATTR
			] # .to_set
		end
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join && join.get_rhs_cell.get_celltype.get_name == :tEventflag
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			eventflagCell = join.get_cell
			id_attr_join = eventflagCell.get_join_list.get_item(:id)
			id_attr = join.get_rhs_cell.get_celltype.find(:id)
			if id_attr_join
				# セル生成時に初期化する場合
				id = id_attr_join.get_rhs.to_s
			else
				# セルタイプの初期化値を使う場合
				id = id_attr.get_initializer.to_s
			end
			flg_pattern = attrMap[SETFLG_FLAG_ATTR].get_rhs.to_s

			# $id$等の置換
			name_array = eventflagCell.get_celltype.get_name_array(eventflagCell)
			id = eventflagCell.get_celltype.subst_name(id, name_array)

			name_array = cell.get_celltype.get_name_array(cell)
			flg_pattern = cell.get_celltype.subst_name(flg_pattern, name_array)

        	return [id, flg_pattern]
		end
		def might_fail
			return true
		end
		def gen_cfg_handler_type(handler)
			case handler
				when EVENT_HANDLER then return "TNFY_SETFLG"
				when ERROR_HANDLER then return "TENFY_SETFLG"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class DataqueueHandlerType < BaseHandlerType
		def initialize()
			super
		end
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join && join.get_rhs_cell.get_celltype.get_name == :tDataqueue
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			dataqueueCell = join.get_cell
			id_attr_join = dataqueueCell.get_join_list.get_item(:id)
			id_attr = join.get_rhs_cell.get_celltype.find(:id)
			if id_attr_join
				# セル生成時に初期化する場合
				id = id_attr_join.get_rhs.to_s
			else
				# セルタイプの初期化値を使う場合
				id = id_attr.get_initializer.to_s
			end

			# $id$等の置換
			name_array = dataqueueCell.get_celltype.get_name_array(dataqueueCell)
			id = dataqueueCell.get_celltype.subst_name(id, name_array)

			name_array = cell.get_celltype.get_name_array(cell)

        	return [id]
		end
		def might_fail
			return true
		end
	end
	class SendToDataqueueHandlerType < DataqueueHandlerType
		def initialize()
			super
			@required_attributes = [
				SNDDTQ_VALUE_ATTR
			] # .to_set
		end
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) && handler == EVENT_HANDLER
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			params = super(handler, join, attrMap, cell, adpt_gen)

			sent_value = attrMap[SNDDTQ_VALUE_ATTR].get_rhs.to_s

			# $id$等の置換
			name_array = cell.get_celltype.get_name_array(cell)
			sent_value = cell.get_celltype.subst_name(sent_value, name_array)

			params << sent_value

        	return params
		end
		def gen_cfg_handler_type(handler)
			case handler
				when EVENT_HANDLER then return "TNFY_SNDDTQ"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class SendErrorCodeToDataqueueHandlerType < DataqueueHandlerType
		def initialize()
			super
		end
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) && handler == ERROR_HANDLER
		end
		def gen_cfg_handler_type(handler)
			case handler
				when ERROR_HANDLER then return "TENFY_SNDDTQ"
				else raise "unknown handler #{handler}"
			end
		end
	end
	class UserHandlerType < BaseHandlerType
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				handler != ERROR_HANDLER && # invalid for error handler
				join && join.get_rhs_cell.get_celltype.get_name == :tTimeEventHandler
		end
		def gen_cfg_handler_type(handler)
			case handler
				when EVENT_HANDLER then return "TNFY_HANDLER"
				else raise "unknown handler #{handler}"
			end
		end
		def gen_cfg_handler_parameters(handler, join, attrMap, cell, adpt_gen)
			# tTimeEventHandlerの結合先を取得
			handler_cell = join.get_rhs_cell
	    	call_join = handler_cell.get_join_list.get_item(:ciHandlerBody)

			# 結合されていない場合はtecsgenがエラーを出すはずなのでここでは
			# エラーにせず無視する．
			return [] unless call_join

			# アダプタ関数ハンドルを取得
			adapter_handle = adpt_gen.make_adapter_handle(call_join)
        	return [adapter_handle[1], adapter_handle[0]]
		end
	end
	class NullHandlerType < BaseHandlerType
		def validate_join(handler, cell, join, *args)
			return super(handler, cell, join, *args) &&
				join.nil? &&
				handler != EVENT_HANDLER # handler is mandatory for normal handler!
		end
		def gen_cfg_handler_type(handler)
			case handler
				when ERROR_HANDLER then return nil
				else raise "unknown handler #{handler}"
			end
		end
	end

	HANDLER_TYPES = [
		ActivateTaskHandlerType.new,
		WakeUpTaskHandlerType.new,
		SetVariableHandlerType.new,
		SetVariableToErrorCodeHandlerType.new,
		IncrementVariableHandlerType.new,
		SignalSemaphoreHandlerType.new,
		SetEventflagHandlerType.new,
		SendToDataqueueHandlerType.new,
		SendErrorCodeToDataqueueHandlerType.new,
		UserHandlerType.new,
		NullHandlerType.new
	]

    #@celltype:: Celltype
    #@option:: String     :オプション文字列
    def initialize( celltype, option )
    	super
    	@plugin_arg_check_proc_tab = NotifierPluginArgProc
    	@plugin_arg_str = option
    	@plugin_arg_str = option.gsub( /\A"(.*)/, '\1' )    # 前後の "" を取り除く
    	@plugin_arg_str.sub!( /(.*)"\z/, '\1' )
    	@factory = nil
    	@output_file = nil
    	parse_plugin_arg
    	unless @factory
    		cdl_error("ASP1003 celltype $1: option factory is not specified",
    			celltype.get_name)
    	end
    	unless @output_file
    		cdl_error("ASP1003 celltype $1: option output_file is not specified",
    			celltype.get_name)
    	end
    end

    def set_factory(template_string)
    	unless @factory.nil?
    		cdl_error("ASP1003 celltype $1: option factory was specified more than once",
    			celltype.get_name)
    	end
    	@factory = template_string
    end

    def set_factory_output_file(output_file)
    	unless @output_file.nil?
    		cdl_error("ASP1003 celltype $1: option output_file was specified more than once",
    			celltype.get_name)
    	end
    	@output_file = output_file
    end

    def gen_factory file
        puts "===== begin #{@celltype.get_name.to_s} plugin ====="

        kernelCfg = AppFile.open( "#{$gen}/#{@output_file}" )
        kernelCfg.print "\n/* Generated by #{self.class.name} */\n"
        kernelCfg.print "\#include \"tTimeEventHandler.h\"\n"

		# アダプタ関数を生成する準備
		@adpt_gen = AdapterGenerator.new("tTimeEventHandler", @celltype.get_global_name)

    	# 属性置換が行えることを検証する。
    	# ここで行うのは、factoryで指定された属性名が
    	# 存在することを確認し、しなければエラーを出力することのみである。
    	# セルごとの処理の最中にエラーを出力することも可能ではあるが、
    	# そうするとセルタイプ側の問題であるのにもかかわらず、セルごとに
    	# エラーが表示されてしまう。
    	# {{attribute_name}} -> attribute_value
    	@factory.scan(/\{\{([a-zA-Z0-9_]*?)\}\}/) { |match|
    		name = $1.to_sym

    		# {{_handler_params_}} はハンドラに関する指定。プラグイン内で値が生成される
    		next if name == :_handler_params_

			subst_attr = @celltype.find(name)
			unless subst_attr
	    		cdl_error( "ASP1007 celltype $1: additional_param: attribute $2 does not exist.",
	    			@celltype.get_name, name)
			end
		}

        @celltype.get_cell_list.each { |cell|
        	gen_factory_for_cell kernelCfg, cell
        }

		# アダプタ関数の生成を完了させる
		@adpt_gen.finish

        kernelCfg.close
        puts "===== end #{@celltype.get_name.to_s} plugin ====="
    end

    def gen_factory_for_cell(kernelCfg, cell)
    	handler_flags = []
    	handler_args = []

    	event_handler_might_fail = true
    	handler_flag = nil

		# ignoreErrorsを取得
		ignoreErrors_attr_join = cell.get_join_list.get_item(:ignoreErrors)
		ignoreErrors_attr = cell.get_celltype.find(:ignoreErrors)
		if ignoreErrors_attr_join
			# セル生成時に初期化する場合
			ignoreErrors = ignoreErrors_attr_join.get_rhs.to_s
		else
			# セルタイプの初期化値を使う場合
			ignoreErrors = ignoreErrors_attr.get_initializer.to_s
		end
		case ignoreErrors
			when 'true' then ignoreErrors = true
			when 'false' then ignoreErrors = false
			else
				cdl_warning( "ASP1005 cell $1: unrecognized value '$2' specified for ignoreErrors",
					cell.get_name, ignoreErrors )
				ignoreErrors = false
		end

    	[EVENT_HANDLER, ERROR_HANDLER].each { |handler|
    		# 呼び口の結合を取得
    		call_join = cell.get_join_list.get_item(handler.call_port_name.to_sym)

	    	# ハンドラタイプを判別する
	    	matches = HANDLER_TYPES.select { |handler_type|
	    		handler_type.validate_join(handler, cell, call_join)
	    	}

	    	if matches.length == 0
	    		cdl_error( "ASP1001 cell $1: no matching handler type found for $2", cell.get_name, handler.call_port_name )
	    		next
	    	end

			# 最初に見つかった有効なハンドラタイプを使用
	    	ht = matches[0]

			# 通知ハンドラで「エラーが発生するはずがない」のに「エラーハンドラが指定されている」
			# もしくはその逆のパターンを検出する。
			# (handler_flagがnilである場合、ハンドラタイプが不明であり、エラーが発生するか不明
			#  なため、検出は行わない。)
    		if handler == ERROR_HANDLER && !ht.is_a?(NullHandlerType) && !event_handler_might_fail
	    		cdl_error( "ASP1004 cell $1: handler type $2 which never raises an error was inferred for the normal notification handler, but an error notification handler was specified.",
	    			cell.get_name, handler_flag)
    		end
    		if handler == ERROR_HANDLER && ht.is_a?(NullHandlerType) && event_handler_might_fail && !ignoreErrors
	    		cdl_warning( "ASP1006 cell $1: handler type $2 which might raise an error was inferred for the normal notificaton handler, but an error notification handler was not specified.",
	    			cell.get_name, handler_flag)
    		end

			# assertion
	    	unless ht.validate_join(handler, cell, call_join)
	    		raise "!validate_join"
	    	end

			# 通知方法の静的API記述を生成する
	    	handler_flag = ht.gen_cfg_handler_type(handler)
	    	handler_flags << handler_flag if handler_flag

	    	attr_map = ht.generate_attr_map(handler, cell)

	    	handler_arg = ht.gen_cfg_handler_parameters(handler, call_join, attr_map, cell, @adpt_gen)
	    	handler_args += handler_arg if handler_arg

	    	if handler == EVENT_HANDLER
	    		event_handler_might_fail = ht.might_fail
	    	end
    	}

		# $id$等の置換
		name_array = cell.get_celltype.get_name_array(cell)
		handler_args.collect! { |e|
			if e == "$cbp$"
				cell.get_celltype.subst_name(e, name_array)
			else
				e
			end
		}

    	# tecsgen.cfgの記述を生成する。
    	# factoryに対し、パラメータ置換を行う。
    	# {{attribute_name}} -> attribute_value
    	text = @factory.gsub(/\{\{([a-zA-Z0-9_]*?)\}\}/) { |match|
    		name = $1.to_sym
			subst_attr = cell.get_celltype.find(name)

			# {{_handler_params_}} はハンドラの指定に置換する。
			if name == :_handler_params_
		    	args_joined = handler_flags.join(' | ')
		    	if handler_args.length > 0
			    	args_joined << ", "
			    	args_joined << handler_args.join(', ')
			    end
			    next args_joined
			end

			unless subst_attr
				# 属性が見つからないというエラーはすでに報告されているので
				# ここではダミー値を返しておくだけである。
				next ""
			end

			subst_attr_join = cell.get_join_list.get_item(name)
			if subst_attr_join
				# セル生成時に初期化する場合
				subst = subst_attr_join.get_rhs.to_s
			else
				# セルタイプの初期化値を使う場合
				subst = subst_attr.get_initializer.to_s
			end

			# $id$等の置換
			cell.get_celltype.subst_name(subst, name_array)
    	}

    	# 出力
    	kernelCfg.puts text

    end
    private :gen_factory_for_cell

end
