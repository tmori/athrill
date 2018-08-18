require File.dirname(__FILE__) + '/PrimitiveType'

class DataElementArrayType
    @@objs = Array.new()

    attr_accessor :name
    attr_accessor :ptype
    attr_accessor :size
    def initialize(name, ptyperef, size)
        self.name = name
        self.size = size

        type = ptyperef.split("/")[1]
        self.ptype = PrimitiveType.get(type)

        DataElementArrayType.register(self)
    end

    def self.add(name, ptyperef, size)
        DataElementArrayType.new(name, ptyperef, size)
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
