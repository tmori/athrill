require File.dirname(__FILE__) + '/PrimitiveType'

class DataElementSingleType
    @@objs = Array.new()
    
    attr_accessor :name
    attr_accessor :ptype

    def initialize(name, ptyperef)
        self.name = name
        type = ptyperef.split("/")[1]
        self.ptype = PrimitiveType.get(type)

        DataElementSingleType.register(self)

    end

    def self.add(name, ptyperef)
        DataElementSingleType.new(name, ptyperef)
    end

    def self.register(obj)
        @@objs.push(obj)
    end

    def self.get(name)
        for elm in @@objs do
            if elm.name == name then
                return elm
            end
        end
        return nil
    end
end
