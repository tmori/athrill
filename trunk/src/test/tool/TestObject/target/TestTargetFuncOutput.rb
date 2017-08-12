
class TestTargetFuncOutput
  attr_accessor :test_output
  attr_accessor :reg_output
  attr_accessor :reg_expect
    
  def initialize (test_output, reg_output, reg_expect)
    self.test_output = test_output
    self.reg_output = reg_output
    self.reg_expect = reg_expect
  end
end