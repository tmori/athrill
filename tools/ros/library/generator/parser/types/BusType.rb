
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
            #p "BusElement:" + typeName
            self.busElmType = BusElementDataType.get(typeName)
            #p self.busElmType
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
        p "ERROR: BusType(" + name + ") is not found!"
        return nil
    end

    def self.getAll()
        return @@objs
    end

end