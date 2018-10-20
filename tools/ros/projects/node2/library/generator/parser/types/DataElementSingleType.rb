require File.dirname(__FILE__) + '/PrimitiveType'

class DataElementSingleType
    @@objs = Array.new()
    
    attr_accessor :name
    attr_accessor :ptype
    attr_accessor :array_size

    def initialize(name, ptyperef)
        self.name = name
        type = ptyperef.split("/")[1]
        self.ptype = PrimitiveType.get(type)
        self.array_size = 1

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
        p "ERROR: DataElementSingleType(" + name + ") is not found!"
        return nil
    end
end
