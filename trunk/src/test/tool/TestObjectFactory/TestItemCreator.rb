require 'excel_reader.rb'
require 'TestItem.rb'

class TestItemCreator
  attr_accessor :name
  attr_accessor :reader
  
  def initialize(name, reader)
    self.name = name
    self.reader = reader
  end
  
  def load(row, column_inp, column_exp)
    item = TestItem.new(self.name)
    column_inp.each { |c|
      data = reader.ref(row, c)
      item.addInput(TestTargetFuncInputValue.new(data))
    }
    column_exp.each { |e|
      data = reader.ref(row, e)
      item.addOutput(TestTargetFuncExpectedValue.new(data))
    }
    return item
  end
end