require 'TestTargetFuncInputValue.rb'
require 'TestTargetFuncExpectedValue.rb'

class TestItem
  attr_accessor :name
  attr_accessor :inp
  attr_accessor :out
  
  def initialize(name)
    self.name = name
    self.inp = Array.new()
    self.out = Array.new()
  end
  
  def addInput(inp)
    self.inp[self.inp.length] = inp
  end

  def addOutput(out)
    self.out[self.out.length] = out
  end
  
end
