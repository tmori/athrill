require 'TestTargetFuncElement.rb'

class ImmediateValue < TestTargetFuncElement
  attr_accessor :bitsize
  
  def initialize(name)
    super(name)
    self.bitsize = name.split("imm")[1].to_i
  end
end
