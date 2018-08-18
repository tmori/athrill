
class BusElement
    attr_accessor :name
    attr_accessor :busElmType

    def initialize(name, busElmTypeRef)
        self.name = name

        elmName = busElmTypeRef.split("/")[0]
        typeName = busElmTypeRef.split("/")[1]
        if elmName == "BusElementQueueType"
            self.busElmType = BusElementQueueType.get(typeName)
        else
            self.busElmType = BusElementDataType.get(typeName)
        end
    end
end

class BusType
    @@objs = Array.new()
    attr_accessor :name
    attr_accessor :elements

    def initialize(name)
        self.name = name
        self.elements = Array.new()

        BusType.register(self)
    end

    def add(name, busElmTypeRef)
        self.elements.push(BusElement.new(name, busElmTypeRef))
    end

    def self.add(name)
        return BusType.new(name)
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