
class TestSpec
  attr_accessor :name
  attr_accessor :form
  attr_accessor :target
  attr_accessor :items

  def initialize(name)
    self.name = name
    self.items = Array.new()
  end
  
  def setForm(form)
    self.form = form
  end
  
  def setTarget(target)
    self.target = target
  end
  
  def addItem(item)
    self.items[self.items.length] = item
  end
  
  
end