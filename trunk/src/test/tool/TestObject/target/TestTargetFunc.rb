require 'GeneralRegister.rb'
require 'SystemRegister.rb'
require 'ImmediateValue.rb'

class TestTargetFunc
  attr_accessor :name
  attr_accessor :inp
  attr_accessor :out
  
  def initialize(name)
    self.name = name
    self.inp = Array.new()
    self.out = Array.new()
  end
  
  def addInput(input)
    self.inp[self.inp.length] = input
  end
  
  def addOutput(output)
    self.out[self.out.length] = output
  end
  
  def getGeneralRegisters()
    regs = Array.new()
    self.inp.each { |e|
      if e.elm.instance_of?(GeneralRegister)
        regs[regs.length] = e.elm.name
      end
    }
    self.out.each { |e|
      elm = e.test_output
      if elm.instance_of?(GeneralRegister)
        regs[regs.length] = elm.name
      end
      
      regs[regs.length] = e.reg_output.name
      regs[regs.length] = e.reg_expect.name
    }
    return regs.sort.uniq
  end
  
end