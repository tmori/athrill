class PrimitiveType
    @@objs = Array.new()
    attr_accessor :name
    attr_accessor :size

    def self.add(name, size)
        PrimitiveType.new(name, size)
    end
    
    def initialize(name, size)
        self.name = name
        self.size = size.to_i
        PrimitiveType.register(self)
    end

    def self.register(obj)
        @@objs.push(obj)
    end

    def self.get(type)
        for ptype in @@objs do
            if ptype.name == type then
                return ptype
            end
        end
        return nil
    end

    def self.parse(obj)
        case obj.name
        when "uint8" then
            return true
        when "uint16" then
            return true
        when "uint32" then
            return true
        else
            return false
        end
    end
end
