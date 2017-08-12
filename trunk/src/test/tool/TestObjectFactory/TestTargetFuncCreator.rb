require 'excel_reader.rb'
require 'GeneralRegister.rb'
require 'SystemRegister.rb'
require 'ImmediateValue.rb'
require 'TestTargetFuncInput.rb'
require 'TestTargetFuncOutput.rb'

class TestTargetFuncCreator
  def initialize()
  end
  
  def loadInput(reader, row, column_type, column_reg)
    type = reader.ref(row, column_type)
    reg  = reader.ref(row, column_reg)
    
    cls = eval type
    return TestTargetFuncInput.new(cls.new(reg))
  end

  def loadOutput(reader, row, column_type, column_test, column_out, column_exp)
    type = reader.ref(row, column_type)
    test_reg  = reader.ref(row, column_test)
    out_reg  = reader.ref(row, column_out)
    exp_reg  = reader.ref(row, column_exp)
    
    cls = eval type
    return TestTargetFuncOutput.new(cls.new(test_reg), cls.new(out_reg), cls.new(exp_reg))
  end


end