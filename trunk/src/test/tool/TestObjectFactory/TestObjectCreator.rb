#! ruby -Ks
require 'win32ole'
require 'excel_reader.rb'
require 'TestItemCreator.rb'
require 'TestTargetFuncCreator.rb'
require 'TestSpec.rb'
require 'TestTargetFunc.rb'

class TestObjectCreator
  attr_accessor :path
  @@myInstance = nil
  def initialize(path)
    self.path = path
  end
  
  def loadTestTarget(workbook)
    spec_reader = ExcelReader.new(workbook, "TEST_SPEC")
    
    #meta
    ins_name = spec_reader.ref(4,3)
    inp_num = spec_reader.ref(4, 4).to_i
    out_num = spec_reader.ref(4, 5).to_i
    func_name = spec_reader.ref(4,6)
    ins_form = spec_reader.ref(4,7)
    
    spec = TestSpec.new(func_name)
    spec.setForm(ins_form)
    target = TestTargetFunc.new(ins_name)

    #input
    row_inp = 8
    column_sheet = 4
    column_row = 5
    column_type = 6
    column_reg = 7
    
    sheet_name = spec_reader.ref(row_inp, column_sheet)
    row = spec_reader.ref(row_inp, column_row).to_i
    c_type = spec_reader.ref(row_inp, column_type).to_i
    c_reg = spec_reader.ref(row_inp, column_reg).to_i

    #read
    input_reader = ExcelReader.new(workbook, sheet_name)
    creator = TestTargetFuncCreator.new()
    
    while (target.inp.length < inp_num)
      row = row + target.inp.length
      input = creator.loadInput(input_reader, row, c_type, c_reg)
      target.addInput(input)
    end
    

    #output    
    row_out = 12
    column_sheet = 4
    column_row = 5
    column_type = 6
    column_test = 7
    column_out = 8
    column_exp = 9
    
    sheet_name = spec_reader.ref(row_out, column_sheet)
    row = spec_reader.ref(row_out, column_row).to_i
    c_type = spec_reader.ref(row_out, column_type).to_i
    c_test_reg = spec_reader.ref(row_out, column_test).to_i
    c_out_reg = spec_reader.ref(row_out, column_out).to_i
    c_exp_reg = spec_reader.ref(row_out, column_exp).to_i
    
    #read
    output_reader = ExcelReader.new(workbook, sheet_name)
    creator = TestTargetFuncCreator.new()
    
    while (target.out.length < out_num)
      row = row + target.out.length
      output = creator.loadOutput(output_reader, row, c_type, c_test_reg, c_out_reg, c_exp_reg)
      target.addOutput(output)
    end

    #set
    spec.setTarget(target)
    
    return spec
  end
  
  def loadTestItem(workbook, spec)
    spec_reader = ExcelReader.new(workbook, "TEST_SPEC")
    item_row = 16
    column_sheet = 3
    column_row = 4
    column_num = 5
    column_c = 6
    
    #meta
    sheet_name = spec_reader.ref(item_row, column_sheet)
    row = spec_reader.ref(item_row, column_row).to_i
    item_num = spec_reader.ref(item_row, column_num).to_i
    
    #input columns
    max = spec.target.inp.length
    columns_inp = Array.new()
    while (columns_inp.length < max)
      column_c = column_c + columns_inp.length
      columns_inp[columns_inp.length] = spec_reader.ref(item_row, column_c).to_i
    end
    column_c = column_c + 1

    #output columns
    max = spec.target.out.length
    columns_exp = Array.new()
    while (columns_exp.length < max)
      column_c = column_c + columns_exp.length
      columns_exp[columns_exp.length] = spec_reader.ref(item_row, column_c).to_i
    end

    #test item        
    item_reader = ExcelReader.new(workbook, sheet_name)
    while (spec.items.length < item_num)
      creator = TestItemCreator.new(spec.name + "_" + spec.items.length.to_s, item_reader)
      item = creator.load(row + spec.items.length, columns_inp, columns_exp)
      spec.addItem(item)
    end
    
  end
  
  def load()
    excel = nil
    workbook = nil
    begin
      fso = WIN32OLE.new('Scripting.FileSystemObject')
      absPath = fso.GetAbsolutePathName(self.path)
      excel = WIN32OLE.new('Excel.Application')
      excel.visible = false
      workbook = excel.Workbooks.Open absPath
      
      spec = self.loadTestTarget(workbook)
      self.loadTestItem(workbook, spec)
      
      workbook.close(:SaveChanges => false)
      excel.quit
      @@myInstance = spec
      return spec
    rescue => e
      if workbook != nil
        workbook.close(:SaveChanges => false)
      end
      if excel != nil
        excel.quit
        excel = nil
      end
      raise e
    end
  end
  
  def self.ref()
    return @@myInstance
  end
  
end