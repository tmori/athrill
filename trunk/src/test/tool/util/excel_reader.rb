class ExcelReader
	attr_accessor :workbook
	attr_accessor :sheetName

	def initialize(workbook, sheetName)
		self.workbook = workbook
		self.sheetName = sheetName
	end

	def exist?()
		ret = false
		self.workbook.WorkSheets.each do |elm|
			if elm.name == self.sheetName
				ret = true
			end
		end
		return ret
	end
  
	
	def ref(row, column)
		value = self.workbook.WorkSheets(self.sheetName).rows[row].columns[column].Value;
		if value.instance_of?(Integer)
			return value.to_s
		end
		if value.instance_of?(Float)
			return value.to_i.to_s
		end
		return value
	end
end
