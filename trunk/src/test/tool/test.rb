require 'GeneralRegister.rb'
require 'SystemRegister.rb'
require 'ImmediateValue.rb'
require 'TestTargetFunc.rb'
require 'TestTargetFuncInput.rb'
require 'TestTargetFuncOutput.rb'
require 'TestSpec.rb'
require 'TestItem.rb'
require 'TestTargetFuncInputValue.rb'
require 'TestTargetFuncExpectedValue.rb'

target = TestTargetFunc.new("add")

cls = eval "#{ImmediateValue}"

p cls.new("imm5")


